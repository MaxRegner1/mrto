/*
 * Driver for PowerMac Z85c30 based ESCC cell found in the
 * "macio" ASICs of various PowerMac models
 * 
 * Copyright (C) 2003 Ben. Herrenschmidt (benh@kernel.crashing.org)
 *
 * Derived from drivers/macintosh/macserial.c by Paul Mackerras
 * and drivers/serial/sunzilog.c by David S. Miller
 *
 * Hrm... actually, I ripped most of sunzilog (Thanks David !) and
 * adapted special tweaks needed for us. I don't think it's worth
 * merging back those though. The DMA code still has to get in
 * and once done, I expect that driver to remain fairly stable in
 * the long term, unless we change the driver model again...
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * 2004-08-06 Harald Welte <laforge@gnumonks.org>
 *	- Enable BREAK interrupt
 *	- Add support for sysreq
 *
 * TODO:   - Add DMA support
 *         - Defer port shutdown to a few seconds after close
 *         - maybe put something right into uap->clk_divisor
 */

#undef DEBUG
#undef DEBUG_HARD
#undef USE_CTRL_O_SYSRQ

#include <linux/module.h>
#include <linux/tty.h>

#include <linux/tty_flip.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/adb.h>
#include <linux/pmu.h>
#include <linux/bitops.h>
#include <linux/sysrq.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/sections.h>
#include <asm/io.h>
#include <asm/irq.h>

#ifdef CONFIG_PPC_PMAC
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/pmac_feature.h>
#include <asm/dbdma.h>
#include <asm/macio.h>
#else
#include <linux/platform_device.h>
#define of_machine_is_compatible(x) (0)
#endif

#if defined (CONFIG_SERIAL_PMACZILOG_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/serial.h>
#include <linux/serial_core.h>

#include "pmac_zilog.h"

/* Not yet implemented */
#undef HAS_DBDMA

static char version[] __initdata = "pmac_zilog: 0.6 (Benjamin Herrenschmidt <benh@kernel.crashing.org>)";
MODULE_AUTHOR("Benjamin Herrenschmidt <benh@kernel.crashing.org>");
MODULE_DESCRIPTION("Driver for the Mac and PowerMac serial ports.");
MODULE_LICENSE("GPL");

#ifdef CONFIG_SERIAL_PMACZILOG_TTYS
#define PMACZILOG_MAJOR		TTY_MAJOR
#define PMACZILOG_MINOR		64
#define PMACZILOG_NAME		"ttyS"
#else
#define PMACZILOG_MAJOR		204
#define PMACZILOG_MINOR		192
#define PMACZILOG_NAME		"ttyPZ"
#endif

#define pmz_debug(fmt, arg...)	pr_debug("ttyPZ%d: " fmt, uap->port.line, ## arg)
#define pmz_error(fmt, arg...)	pr_err("ttyPZ%d: " fmt, uap->port.line, ## arg)
#define pmz_info(fmt, arg...)	pr_info("ttyPZ%d: " fmt, uap->port.line, ## arg)

/*
 * For the sake of early serial console, we can do a pre-probe
 * (optional) of the ports at rather early boot time.
 */
static struct uart_pmac_port	pmz_ports[MAX_ZS_PORTS];
static int			pmz_ports_count;

static struct uart_driver pmz_uart_reg = {
	.owner		=	THIS_MODULE,
	.driver_name	=	PMACZILOG_NAME,
	.dev_name	=	PMACZILOG_NAME,
	.major		=	PMACZILOG_MAJOR,
	.minor		=	PMACZILOG_MINOR,
};


/* 
 * Load all registers to reprogram the port
 * This function must only be called when the TX is not busy.  The UART
 * port lock must be held and local interrupts disabled.
 */
static void pmz_load_zsregs(struct uart_pmac_port *uap, u8 *regs)
{
	int i;

	/* Let pending transmits finish.  */
	for (i = 0; i < 1000; i++) {
		unsigned char stat = read_zsreg(uap, R1);
		if (stat & ALL_SNT)
			break;
		udelay(100);
	}

	ZS_CLEARERR(uap);
	zssync(uap);
	ZS_CLEARFIFO(uap);
	zssync(uap);
	ZS_CLEARERR(uap);

	/* Disable all interrupts.  */
	write_zsreg(uap, R1,
		    regs[R1] & ~(RxINT_MASK | TxINT_ENAB | EXT_INT_ENAB));

	/* Set parity, sync config, stop bits, and clock divisor.  */
	write_zsreg(uap, R4, regs[R4]);

	/* Set misc. TX/RX control bits.  */
	write_zsreg(uap, R10, regs[R10]);

	/* Set TX/RX controls sans the enable bits.  */
	write_zsreg(uap, R3, regs[R3] & ~RxENABLE);
	write_zsreg(uap, R5, regs[R5] & ~TxENABLE);

	/* now set R7 "prime" on ESCC */
	write_zsreg(uap, R15, regs[R15] | EN85C30);
	write_zsreg(uap, R7, regs[R7P]);

	/* make sure we use R7 "non-prime" on ESCC */
	write_zsreg(uap, R15, regs[R15] & ~EN85C30);

	/* Synchronous mode config.  */
	write_zsreg(uap, R6, regs[R6]);
	write_zsreg(uap, R7, regs[R7]);

	/* Disable baud generator.  */
	write_zsreg(uap, R14, regs[R14] & ~BRENAB);

	/* Clock mode control.  */
	write_zsreg(uap, R11, regs[R11]);

	/* Lower and upper byte of baud rate generator divisor.  */
	write_zsreg(uap, R12, regs[R12]);
	write_zsreg(uap, R13, regs[R13]);
	
	/* Now rewrite R14, with BRENAB (if set).  */
	write_zsreg(uap, R14, regs[R14]);

	/* Reset external status interrupts.  */
	write_zsreg(uap, R0, RES_EXT_INT);
	write_zsreg(uap, R0, RES_EXT_INT);

	/* Rewrite R3/R5, this time without enables masked.  */
	write_zsreg(uap, R3, regs[R3]);
	write_zsreg(uap, R5, regs[R5]);

	/* Rewrite R1, this time without IRQ enabled masked.  */
	write_zsreg(uap, R1, regs[R1]);

	/* Enable interrupts */
	write_zsreg(uap, R9, regs[R9]);
}

/* 
 * We do like sunzilog to avoid disrupting pending Tx
 * Reprogram the Zilog channel HW registers with the copies found in the
 * software state struct.  If the transmitter is busy, we defer this update
 * until the next TX complete interrupt.  Else, we do it right now.
 *
 * The UART port lock must be held and local interrupts disabled.
 */
static void pmz_maybe_update_regs(struct uart_pmac_port *uap)
{
	if (!ZS_REGS_HELD(uap)) {
		if (ZS_TX_ACTIVE(uap)) {
			uap->flags |= PMACZILOG_FLAG_REGS_HELD;
		} else {
			pmz_debug("pmz: maybe_update_regs: updating\n");
			pmz_load_zsregs(uap, uap->curregs);
		}
	}
}

static void pmz_interrupt_control(struct uart_pmac_port *uap, int enable)
{
	if (enable) {
		uap->curregs[1] |= INT_ALL_Rx | TxINT_ENAB;
		if (!ZS_IS_EXTCLK(uap))
			uap->curregs[1] |= EXT_INT_ENAB;
	} else {
		uap->curregs[1] &= ~(EXT_INT_ENAB | TxINT_ENAB | RxINT_MASK);
	}
	write_zsreg(uap, R1, uap->curregs[1]);
}

static bool pmz_receive_chars(struct uart_pmac_port *uap)
{
	struct tty_port *port;
	unsigned char ch, r1, drop, error, flag;
	int loops = 0;

	/* Sanity check, make sure the old bug is no longer happening */
	if (uap->port.state == NULL) {
		WARN_ON(1);
		(void)read_zsdata(uap);
		return false;
	}
	port = &uap->port.state->port;

	while (1) {
		error = 0;
		drop = 0;

		r1 = read_zsreg(uap, R1);
		ch = read_zsdata(uap);

		if (r1 & (PAR_ERR | Rx_OVR | CRC_ERR)) {
			write_zsreg(uap, R0, ERR_RES);
			zssync(uap);
		}

		ch &= uap->parity_mask;
		if (ch == 0 && uap->flags & PMACZILOG_FLAG_BREAK) {
			uap->flags &= ~PMACZILOG_FLAG_BREAK;
		}

#if defined(CONFIG_MAGIC_SYSRQ) && defined(CONFIG_SERIAL_CORE_CONSOLE)
#ifdef USE_CTRL_O_SYSRQ
		/* Handle the SysRq ^O Hack */
		if (ch == '\x0f') {
			uap->port.sysrq = jiffies + HZ*5;
			goto next_char;
		}
#endif /* USE_CTRL_O_SYSRQ */
		if (uap->port.sysrq) {
			int swallow;
			spin_unlock(&uap->port.lock);
			swallow = uart_handle_sysrq_char(&uap->port, ch);
			spin_lock(&uap->port.lock);
			if (swallow)
				goto next_char;
		}
#endif /* CONFIG_MAGIC_SYSRQ && CONFIG_SERIAL_CORE_CONSOLE */

		/* A real serial line, record the character and status.  */
	YSRQ
 Synchr[GMRK&iKwrm_devije7aracter and status.  */
	YSRQ
 Synchr[GMRK&iKwrm_devije7aregs[Krd the character and sA2iKx 1NT)
			bENABx&iK&ie,ev
			if (sw7areg 1NT)
			ENABx&mz_debug("pmz: mag busak;
 !");
			pm1 =  ~(EX[Krd the cC_ERR)) 			pm1YSRQ
 Synchr[GMRKbrkKwrm		if (uap-_handle_sysak;
uap->port.l
			uagoto next_char;
		}

	pox&ie {
 (sw7areg[Krd th)		pm1YSRQ
 Synchr[GMRKrity_mKwrm		iie {
 (sw7aregC_ERR)) 		pm1YSRQ
 Synchr[GMRKfm teKwrm		i (sw7aregharact 		pm1YSRQ
 Synchr[GMRKovruptnKwrm		i =  ~(YSRQ
 Syncad_zsatus. ask;
		iff (sw7areg 1NT)
					pm1ter and staEAK;
		}
&ie {
 (sw7areg[Krd th)		pm1ter and sta[KrITYrm		iie {
 (sw7aregC_ERR)) 		pm1ter and staFRAME		}

		ch (uap->port.synedoresatus. ask;
= 0 &xff ||	    rew7aregp->port.synedoresatus. ask;
)= 0 &	ENABx&y_poinriatlip.hhar(&urt, ch);flag;

		}
	}
i (sw7aregharact 		pmy_poinriatlip.hhar(&urt, ch ER staOVR))UN
		}xt_char;
:	/* A Wcan dot instu */ th doinnishteoops ot ing pear ch0hen the T	   *ine, s no th  wrg wi reate->we dosak;
 at ithere.	   *iWn the  ithpeninsI exsabled.he chceive_c sidef the poiver_n.	   *iNotthat itwt itI'_c be thpectriencg pe noaeal sei = ops owhere	   *iI'mot ing pefopsd fog(uardss we the potuallyort shupe.
 *   *iSething risansmg s nogog rionith the coHW*   *	YSRQ
 S(++ops =) >000; r[GMRK&iKwfopsd		ch = read_zsdag(uap, R0, 
		if (st!h ==egharCH_AV)			break;
		u
		cturn fauct
	}wfopsd:	/z_interrupt_control(stp, R0 
		iz_error(fmmz: marxei = fopsd !");
			turn fauct
	}
static void pmz_inatus. andle_struct uart_pmac_port *uap)
{
	stsigned char stat =usrm_datus inread_zsdag(uap, R0, 
		iite_zsreg(uap, R0, RES_EXT_INT);
	wrsync(uap);
		
f (ZS_TX_EXOPENap);
& CO_TXWANTMODULEM_STATUSap)) {
		if (ZSatus in& SYNC_HU)
			brYSRQ
 Synchr[GMRKdsrKwrm_de/The UAlog chjt beger_updsh doinrrupt_chen thDCD/CTS/etc.hange t *   *iBuit widoenot butl fodshile ==bitas toange tdwe dove reiKwkeep*   *ians */ the  noourselr_u *   *ie UACTSoint so no tr_nt for usmethead_son */-- paulus*   *	YSRQ
 S(atus in^gp->porrevnatus. 
& hDCD			brYS_handle_sydcd_ange tuap->port, c		uago     - Satus in& DCD	
		if (st(atus in^gp->porrevnatus. 
& hCTS			brYS_handle_syc_counge tuap->port, c		uago     - !Satus in& CTS	

		ifwe spdanterrupt_cle(x)ap->port.state->port;
.delta_msr_wait
	}

	ZS (ZSatus in&  1NT)
					pp->flags |= PMACZILOG_FLAG_REEAK;
		
	p->porrevnatus. nreat =usrm
static void pmz_inansmittehars(struct uart_pmac_port *uap)
{
	struct ttcirc_buf *xtte		
f (ZS_TX_EXNSOLap)) {
		ifsigned char stat =usnread_zsdag(uap, R0, 
		_de/TheXtill hasy, ?  Jt bewaitor the Maxt TX cone, nterrupt.  *   *	   *iItan doocrre becae R7 thh =  do itrial console, rite t  */Itawld h	   *ibMaxicto thensmitteonsole, rite t hjt beke su dodormly, awld hor t	   *iaR stine, . (i . buffereand lo is nrrupt_chiver_n) The ats not b	   *id_sy becae R7nsole, rite t hn dt busleep ThOnsoftluon musht nobe	   *i thpo ha muengh. ort s->xttehupacc becomg pefree */-De rM*   *	YSRQ
 S!Satus in& Tx_BUF_EMP)			brturn f	}

	ZSp->flags &= ~PMACZILOG_FLAG_BR_ACTIVE(u		
f (ZS_TXGS_HELD(uap)) {
		ifz_load_zsregs(uap, uap->curregs);
		}
p->flags &= ~PMACZILOG_FLAG_BRGS_HELD;
		}}	
f (ZS_TX_ACSTOPPEuap)) {
		ifp->flags &= ~PMACZILOG_FLAG_BR_ACSTOPPEu		}
K&iKws *_txnter		}}	
f USEer thmethecircumatunceswe doseinterrupts */rert sehor t	  *iaRose
 dhannel HThe DM nrrupt_chsk;
= thR1s noclear,ut W	  *iR3till hagnedalthe enterrupts */d lo doseine emhen theakg p	  *iadoinrrupt_chr the Maoer eaannel HW(e  nocld hobMaa qemu	  *ig ist WIsie doe MaCC */docidoent thecialfyre-pcsie, awheer e	  *iR3tinrrupt_tat =usnts.  e stsked. y DaR1s nrrupt_chable) 	  *igs, anbeer issafthat nhmerry) T--BenH.
	/
	if (ua!_TX_EXOPENap);
)	}
K&iKws *_txnter		if (uap->port.stxhars({
		ifp->flags &= PMACZILOG_FLAG_RE_ACTIVE(u		write_zsreta(uap);uap->port.lixhars({		wrsync(uap);
		}
YSRQ
 Synchr[GMRKtxKwrm		p->port.lixhars( 0;
		drturn f	}

	ZS (uap->port.state == NULL) {	}
K&iKws *_txnter		}xtteh&uap->port.state->poxtte		f (uap-_hacirc_empty(xtte {
		ifp-_haite_zswe supuap->port.l
		}
K&iKws *_txnter		}}	f (uap-_hatxnop bpeduap->port.l
			uK&iKws *_txnter		ifp->flags &= PMACZILOG_FLAG_RE_ACTIVE(u		wite_zsreta(uap);uaxtte->buf[xtte->ils.]
	wrsync(uap);
		
fxtte->ils.h&u(xtte->ils.h+ 1
& h(RT p_XMIT_SIZE - ;
		cYSRQ
 Synchr[GMRKtxKwrm	f (uap-_hacirc_ars(s_nding T(xtte  < WAKEUPrCHARS)	ifp-_haite_zswe supuap->port.l
		
rturn f	}
s *_txnter:	iite_zsreg(uap, R0, RES_EXTx_P
	wrsync(uap);
		
/* 
 m... acweegisters hat ittwi, Sufixtheter v.. ac
static voi =turn f_tmz_interrupt_c(t i;
rq,oid pm*v_naid{
	struct ttrt_pmac_port *uap)
h&uv_naid;struct ttrt_pmac_port *uap)
_a;struct ttrt_pmac_port *uap)
_b	int lorch&uIRQ_NONu		wol pmzush		cY8 r3		ifp->_= "pz_int iort *_Aap);
		}p)
_b uart>_=->mer rm_dan_lock(&uap->_=->rt.lock);
			r3nread_zsdag(uap, _aR3, 

#ifdef COBUG_HARD
#umz_debug("pm
rq,or3: %x");,or3

#ndif

#* Clonnel HWA/
	ifzushnrelse;
	}
 (sw73& h(CHAT_Ie cCHATxIPe cCHARxIP {
		if (ZS!_TX_EXOPENap);_a	bENABx&z_debug("pmonneAoinrrupt_chene (1t buondi !");
			pmK&iKwski
_a;st
	}
iite_zsreg(uap, _aR3, RES_EXH_IUS{		wrsync(uap);_a	;		}
i (sw73& hCHAT_I)ABx&z_deatus. andle_stp);_a	;}
i (sw73& hCHARxIP ABx&zushnrez_receive_chars(stp);_a	;}
i (sw73& hCHATxIP ABx&z_inansmittehars(stp);_a	;}
irch&uIRQ_HANDLE
		}}	wski
_a:
pin_unlock(&uap->_=->rt.lock);
			 (swzush ABxy_flip.h_buffer_zush)ap->port.state->port;

		
f (ZS!p)
_b			uK&iKwoutrm_dan_lock(&uap->_b->rt.lock);
			zushnrelse;
	}
 (sw73& h(CHBT_Ie cCHBTxIPe cCHBRxIP {
		if (ZS!_TX_EXOPENap);_b	bENABx&z_debug("pmonneBoinrrupt_chene (1t buondi !");
			pmK&iKwski
_b;st
	}
iite_zsreg(uap, _bR3, RES_EXH_IUS{		wrsync(uap);_b	;}
i (sw73& hCHBT_I)ABx&z_deatus. andle_stp);_b	;}
i (sw73& hCHBRxIP ABx&zushnrez_receive_chars(stp);_b	;}
i (sw73& hCHBTxIP ABx&z_inansmittehars(stp);_b	;}
irch&uIRQ_HANDLE
		}}	wski
_b:
pin_unlock(&uap->_b->rt.lock);
			 (swzush ABxy_flip.h_buffer_zush)ap->port.state->port;

		
wout:		turn farc		
/* 
* ThPeekhe sakt =usngisters ,ock mut buld an Dalled r*/
static voinne, sY8 z_poreekeatus. truct uart_pmac_port *uap)
{
	stsigned chng wiags &		cY8 at =usrm	_dan_lock(&rq.hse ruap->port.lock);flag;
;
		}atus inread_zsdag(uap, R0, 
		iin_unlock(&_i =tuop rruap->port.lock);flag;
;
				turn faat =usrm
st 
 * WeCck, 
 (sansmitter is buempty* The UArt lock mu not buld a */
static vosigned cht loz_inax_empty(ruct uart_pmat *port;
{
	stsigned char stat =usrm_datus inrez_poreekeatus. ttomacz(rt.l
	;ZS (ZSatus in& Tx_BUF_EMP)	drturn f TIOCSER_TEMT			turn fa0rm
st 
 * Wet TXModemhCtrol bi(RTSn& DTR)igs, * The UArt lock mu nold and loterrupts */d detabled.
 */
iNott: Shl inweegily, afilr ist IRRTSn mueernal strts at
 * (aould haat itbdefealtt rahht  islevelnly be?*/
static void pmz_mas iomctrl(ruct uart_pmat *port;
,osigned cht lomctrl{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned char chs iogs, anclearogs, 		
wwwwwwww Disoot bng rir thirdair thw.
  ac
stf (ZS_TX_EXIRDAap);
)	}
turn f	}
 A Wcat inlled whdurg bact tith thaArt lot but_tt im
	if (ua!S_TX_EXOPENap);
&|| _TX_EXNSOLap)) {)	}
turn f	}str iogs, h&uclearogs,  0;

		r (ZS_TX_EXINTDULEMap)) {
		if (ZSmctrln& TIOCM_RTS			brr iogs, h PMRTS;}
ise
#d	brclearogs,   PMRTS;}
}	f (uamctrln& TIOCM_DTR)
brr iogs, h PMDTR;}
se
#d	bclearogs,   PMDTR;}/* NowOTE:ot yesubjt thao 'ansmitter isacte_c' re.h>*/
	 
ap->curregs[1]] &  PMr iogs, ;
ap->curregs[1]] &  ~PMclearogs, 		
rite_zsreg(uap, R5, rep->curregs[1]] &
		iz_erbug("pmz: as iomctrl:et R7gs, : %xanclear7gs, : %x -> %x");,	    s iogs, anclearogs, rep->curregs[1]] &
		isync(uap);
		
/* 
 * GNU TXModemhCtrol bigs,  (ly bee entet sooneswee copi we ill* GNo hat itth thaAcacd byvaluef the pontrol biones)* The UArt lock mu nold and loterrupts */d detabled.
 */
static vosigned cht loz_ing iomctrl(ruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned char chst =usrm	signed cht loturrm_datus inread_zsdag(uap, R0, 
				tur 0;
		d (ZSatus in& DCD			btur  PMTIOCM_CAR		d (ZSatus in& SYNC_HU)
			btur  PMTIOCM_DSR		d (ZS!Satus in& CTS	
		btur  PMTIOCM_CTS;}		turn farurrm
st 
 * Wetp bieXtiidh>*Dealttke sunzilog to itxt TX xoterrupts ,* the gh. or thDMAwe dowl has  reiKw a prbitare d.* The UArt lock mu nold and loterrupts */d detabled.
 */
static void pmz_inatopnax(ruct uart_pmat *port;
{
	sttomacz(rt.l
flags &= PMACZILOG_FLAG_RE_ACSTOPPEu		
st 
 * WeKickhe sa xoiidh>* The UArt lock mu nold and loterrupts */d detabled.
 */
static void pmz_inat-_hatx(ruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned char chst =usrm	iz_erbug("pmz: : at-_hatx()");
		ifp->flags &= PMACZILOG_FLAG_RE_ACTIVE(u		wp->flags &= ~PMACZILOG_FLAG_BR_ACSTOPPEu		_datus inread_zsdag(uap, R0, 
				/TheXtsy, ?  Jt bewaitor the Ma cone, nterrupt.   m
	if (ua!Satus in& Tx_BUF_EMP)			bturn f	}st Set nthe chfir bearacter aniKwjump-at-_hhe Ma cone, 	  *iQ ensding Txenge, .
	/
	if (uart s->xhars({
		ifite_zsreta(uap);uart s->xhars({		wrsync(uap);
		}
rt s->hr[GMRKtxKwrm		rt s->xhars( 0;
		delse {
		uaruct ttcirc_buf *xtteh&uart s->ate->poxtte			ch (uap-_hacirc_empty(xtte {		pmK&iKwoutrmifite_zsreta(uap);uaxtte->buf[xtte->ils.]
	wrrsync(uap);
		}
xtte->ils.h&u(xtte->ils.h+ 1
& h(RT p_XMIT_SIZE - ;
		c
rt s->hr[GMRKtxKwrm	ch (uap-_hacirc_ars(s_nding T(xtte  < WAKEUPrCHARS)	iffp-_haite_zswe supuap->port.l
		}}
wout:		z_erbug("pmz: : at-_hatx()one, .");
		
st 
 * Wetp biRxoiidh,asedicly, asabled.hetterg rio * MERxoterrupts */onhe port
 .e do lt thsabled.he chc * Residef the pochiprogrps hatgh. * The UArt lock mu nold a */
static void pmz_inatopnrx(ruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	u		z_erbug("pmz: : atopnrx()()");
		if Disable all inRis nrrupt_c  */
	YSp->curregs[1]] &= ~(EINT_MASK);		iz_erybe_update_regs(stp);
		
fz_erbug("pmz: : atopnrx()one, .");
		
st 
 * Weable inmodemhatus inange th nrrupt_c * The UArt lock mu nold a */
static void pmz_inable) _ms(ruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned char chnewegs(
		r (ZS_TX_EXIRDAap);
)	}
turn f	}
newegs( =ep->curregs[1]]] | EN(DCDIE ENSYNCIE ENCTSIE	;ZS (ZSnewegs( !=ep->curregs[1]]] |{
		uap->curregs[1]]] | =hnewegs(
		r* NowOTE:ot yesubjt thao 'ansmitter isacte_c' re.h>**	YSRite_zsreg(uap, R15, rep->curregs[1]]] |{;}
}

st 
 * WeCtrol bigak;
 ate ==ettson[]* The UArt lock mu not buld a */
static void pmz_ingak;
_ctl(ruct uart_pmat *port;
,ot logak;
_ate ={
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned char chs iogs, anclearogs, ,hnewegs(
	tsigned chng wiags &		str iogs, h&uclearogs,  0;

		r (ZSgak;
_ate ={
brr iogs, h PMSND_BRK;}
se
#d	bclearogs,   PMSND_BRK;}_dan_lock(&rq.hse ruart s->ck);flag;
;
				newegs( =e(p->curregs[1]] &  hs iogs, 
& hMclearogs, 		S (ZSnewegs( !=ep->curregs[1]] |{
		uap->curregs[1]] | =hnewegs(
	SRite_zsreg(uap, R15 rep->curregs[1]] &
		i}
	iin_unlock(&_i =tuop rruart s->ck);flag;
;
		}#ifdef CONFIG_PPC_PMAC
#i* 
* ThTn faper anonh thoffo the FrC */d loassociad spetuf * ME(rt.lrivers/s,nmodem,iQ  rt;
,oetc.)* Reprrn fthe ennumb anof ml hiconds af dosuld hawaitober t * untryg traose R7e port
 .*/
static vointmz_mas ioscportr atruct uart_pmac_port *uap, int enate ={
	stt enlay(1 0;
		d  lorc
		r (ZSate ={
		uarch&uac_pocly,eature.h(	iffAC
#_FTR_C *_ABLE);rep->cunodh,ap->port.l_typh,a;
		c
r_erbug("pmzt *uper anonhtuoult: %d");,orc	;}
i (sw_TX_EXINTDULEMap)) {
		ifarch&uac_pocly,eature.h(	ifffAC
#_FTR_DULEM_ABLE);rep->cunodh,a0,a;
		c
	lay(1 0;2500;* Nowaitor th2.5sober t se g */
	ifc
r_erbug("pmmodemhper antuoult: %d");,orc	;}
i}	delse {
		ua/TODO:   Me suat driendinnonhaime wr,o lt thper anwn t	   *iimmediad ly*   *	YSRQ
 S_TX_EXINTDULEMap)) {
		ifarch&uac_pocly,eature.h(	ifffAC
#_FTR_DULEM_ABLE);rep->cunodh,a0,a0
			pmz_lobug("pmzt *uper anoffotuoult: %d");,orc	;}
i}	d	ac_pocly,eature.h(AC
#_FTR_C *_ABLE);rep->cunodh,ap->port.l_typh,a;
	}

		turn falay(1		}#ifse
#dtatic vointmz_mas ioscportr atruct uart_pmac_port *uap, int enate ={
	stturn fa0rm
stndif /* CO!NFIG_PPC_PMAC
#/

		
 * ForixZeroBu..)	.Work*/d nd inaug is nhe FrC */ceive_g pear el HT* FoInin_r from drDarw nhcodh,a15et .   2000*/-DenM*
 * The UAfoow)
g risequee dorreven */drogre) mhat it soseinith thO'Hd deICs o* ME(st ofrsion[]s/-- alsoith thmetheHturhrow/d loHydraeICs o)owhere/drzero* alotee entet so the Frceive_cr becomes 'stu *'nd localkupdahe Frceive_cr.* This progre) mhn doocrre anoaealoultnof drzerorbitaotee enceive_cr tet s* Thcoincidenitth thanof the GNfoow)
g rieven *:*
 * T	T FrC */ no titializ (COr cdre st usmeware s).* T	A fm tg rieK&ie, disett t.
 */
	T Frock dg rioionalnange tsrom drschronous moie,X1 anchronous m*/
		ock dg ri thX16, X32,oie,X64 anchronous mrock dg r */
	T Frdecong Txde coitoange td amg wiNRZ,iNRZI, FM0,oie,FM1 *
 * This prworkd nd inaer ms */ reprcor form dre longckdahnds ionalnby plintog* the FrC */ nrschronous moops ck thde coth thaAfa beack muber t sogram tmtog* thanof the GNanchronous mrde cs */
static void pmz_infix_zero_bugoscptruct uart_pmac_port *uap)
{
	stite_zsreg(uap, R19, _TX_EXNHANNEL_Aap);
 ? CHRA : CHRB
	zssync(uap);
	ZSelay(100)
		iite_zsreg(uap, R09, (_TX_EXNHANNEL_Aap);
 ? CHRA : CHRB
  hNV
	wrsync(uap);
		
fite_zsreg(uap, R04, X1CL| TxMONSYNC
		iite_zsreg(uap, R03R15x8
		iite_zsreg(uap, R0 reTx8e chTS
		iite_zsreg(uap, R09, NV
	f Disadt thwall ad_zyw a e  n?/
	write_zsreg(uap, R9, reRCBhe cTCBh
		iite_zsreg(uap, R0, re)
		iite_zsreg(uap, R0, re)
		iite_zsreg(uap, R0,4, (LOOPBAKiKx 1SRC)
		iite_zsreg(uap, R0,4, (LOOPBAKiKx 1SRCiKx 1AB));

	iite_zsreg(uap, R03R15x8e chaABLE);
	write_zsreg(uap, R5 RES_EXT_INT);
	write_zsreg(uap, R0 RES_EXT_INT);
	write_zsreg(uap, R0 RES_EXT_INT);
	f DiiKwkl hagetheme wi

		//The UAannel HWsuld habe OKhw.
,ut Wt wi progrele)y/ceive_g p	  *iops ck thgarbag .
	/
 Sth cho avonchronous mrde c,xsabled.he chceive_cr,	  *iaddisruc cdieverybng ri nhe Frceive_c buffer.
	/
	ifite_zsreg(uap, R09, NV
	
fite_zsreg(uap, R04, X16CL| TxSBASK);
	}
ite_zsreg(uap, R03R15x8
		while (1)ad_zsdag(uap, R00)=egharCH_AV)
		ifoid)read_zsdag(uap, R08
		irite_zsreg(uap, R0 RES_EXT_INT);
	wrrite_zsreg(uap, R0 RER_RES);
			}	
/* 
* ThRe staturtdah ndte, rertr aupdahe Frr cdre std staetupda* the FrC *.eprrn fthanlay(1  nhmsowhere/u caededo avwaitober t * untually, se g */e port
 this ti timypicly, ae enterrul stmodem* portr adahlay.h>his pr ndte, xpect thatlongcko avbe te sn.*/
static vointm__z_inat-_huptruct uart_pmac_port *uap)
{
	if ntmzwdebuy(1 0;
		
	memaetuap->porregs[1R0 REsiz of(p->curregs[1;

	/* SewerMapdahe FrC */&nder tlyg trr cdre st(modem/irda)/
	ifzwdebuy(1 0;z_mas ioscportr atp, R0,

	/* SeNictobuggyi re  ac
stfz_infix_zero_bugoscptp);
		
f Reset ext UAannel HW
	YSp->curregs[1]]9] 0;
		dite_zsreg(uap, R19, _TX_EXNHANNEL_Aap);
 ? CHRA : CHRB
	zssync(uap);
	ZSelay(100)
		iite_zsreg(uap, R09, 0
	wrsync(uap);
		
f Clocear7e enterrupts egisters wi
	write_zsreg(uap, R1, re 
		iite_zsreg(uap, R0, RER_RES);
			ite_zsreg(uap, R0, RER_RES);
			ite_zsreg(uap, R0, RES_EXH_IUS{		wite_zsreg(uap, R0, RES_EXH_IUS{		/* Set Tt_taethevalidaud rate ge
	YSp->curregs[1]]4] 0;X16CL| TxSB1;
ap->curregs[1]]3] 0;5x8;
ap->curregs[1]] & =eTx8e chTS		d (ZS!_TX_EXIRDAap);
)	}
p->curregs[1]] &  PMDTR;}
p->curregs[1]]]2] 0;
		dp->curregs[1]]]3] 0;
		dp->curregs[1]]]4] 0; 1AB))		
f Clocear7ndleshakg p,nable biEAK interrupt
 s/
	YSp->curregs[1]] 5] 0; 1KIE		
f ClMaers h nrrupt_chable) W
	YSp->curregs[1]]9]  PMNV TxMIE		
fz_load_zsregs(uap, uap->curregs);
		/* Enable inceive_cr d stansmitter i */
	write_zsreg(uap, R3, rep->curregs[1]]3]  PMRNABLE);
	write_zsreg(uap, R5, rep->curregs[1]] &  PMENABLE);

	/* noRememb anatus inr thDCD/CTSnange tsr
	YSp->currevnatus. nread_zsdag(uap, R0, 
				turn fapwdebuy(1rm
static void pmz_inirda_ret etruct uart_pmac_port *uap)
{
	stsigned chng wiags &		_dan_lock(&rq.hse ruap->port.lock);flag;
;
		}p->curregs[1]] &  PMDTR;}
ite_zsreg(uap, R15 rep->curregs[1]] &
		isync(uap);
	ZSin_unlock(&_i =tuop rruap->port.lock);flag;
;
			msleep(11 
				an_lock(&rq.hse ruap->port.lock);flag;
;
		}p->curregs[1]] &  ~(EDTR;}
ite_zsreg(uap, R15 rep->curregs[1]] &
		isync(uap);
	ZSin_unlock(&_i =tuop rruap->port.lock);flag;
;
			msleep(10)		
/* 
* ThTs ti tim en"dormly"taturtdah ndte, ree g */e poabor ione* Thwraed moth the congckoaddisog ria scd bulehlay.h*/
static vointmz_mast-_huptruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned chng wiags &		c ntmzwdebuy(1 0;
		
	z_erbug("pmz: : at-_hupt)");
		ifp->flags &= PMACZILOG_FLAG_RE_EXOPEN
	/* noA7nsole, r noter pmztr a whdown.lse, weperMapdahd
 *  *iititializ he pochip
	/
	if (ua!_TX_EXNSOLap)) {
		ifan_lock(&rq.hse ruart s->ck);flag;
;
			fzwdebuy(1 0;__z_inat-_huptp);
		}
in_unlock(&_i =tuop rruart s->ck);flag;
;
			}	_danr ntf(p->cui =ame	=,MACZILOG_NAME		"%d"uap->port.line, 	;ZS (ZSrequest_i =ap->port.synrq,oz_interrupt_c,iQ QF_SHARED,
	brYSRQ
i =ame	=,Mp)) {
		ifz_error(fmmUble in reprsters hzinterrupts. ndle_sr.");
			pz_mas ioscportr atp, R00
			pturn fa-ENXIO;	}}	
f USRht now.
we defe stth thbuy(1 b1 bck dg rihere,iQ' be u*  *ism-_h isler veon
	/
	if (uarwdebuy(1 !0 &	ENABxz_erbug("pmz: : buy(1g ri%dhms");,orwdebuy(1
			pmsleep(rwdebuy(1
			}	
f USIrDrealt ex dise, nw se
stf (ZS_TX_EXIRDAap);
)	}
z_inirda_ret etp);
		
f Clable interrupts  requestsor the Maannel HW
	YSan_lock(&rq.hse ruart s->ck);flag;
;
			z_interrupt_control(stp, R01
	ZSin_unlock(&_i =tuop rruart s->ck);flag;
;
				z_erbug("pmz: : at-_hupt)one, .");
		stturn fa0rm
static void pmz_inatdown ttruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned chng wiags &				z_erbug("pmz: : atdown tt)");
		ifan_lock(&rq.hse ruart s->ck);flag;
;
				 Disable alterrupts  requestsor the Maannel HW
	YSz_interrupt_control(stp, R0 
		if (ua!_TX_EXNSOLap)) {
		if Disable alceive_cr d stansmitter i *	YSRp->curregs[1]]3]  ~(EINABLE);;	}
p->curregs[1]] &  ~(ETNABLE);;		if Disable algak;
 asriatnaln*	YSRp->curregs[1]] &  ~(ESND_BRK;}
iz_erybe_update_regs(stp);
		i}
	iin_unlock(&_i =tuop rruart s->ck);flag;
;
		/* noReceasenterrupts. ndle_sr/
	forree_i =ap->port.synrq,op);
		
fan_lock(&rq.hse ruart s->ck);flag;
;
				p->flags &= ~PMACZILOG_FLAG_BR_EXOPEN
	/* (ua!_TX_EXNSOLap)) {		pz_mas ioscportr atp, R00
	* Sethuthe pochiphdowni

		/in_unlock(&_i =tuop rruart s->ck);flag;
;
				z_erbug("pmz: : atdown tt)one, .");
		
st 
 Sr c. y Da stiiver pmd staeal console, rs Tt_ The UArt lock mu nold a* and oncal interrupts did detabled.
 */
static void pmz_innsor_nt_tomzstruct uart_pmac_port *uap, u8 igned cht locag;
,
	brwwwwww igned cht loiag;
, signed chng wiud r{
	if ntmbrg		/* Setth cho aveernal stock dg rir thIrDrehht eack mute gs.he at*  *ice contd habe re-us for usMidinterrufacewith thdifferent*  *imultiied rs
	/
	if (uaud ra>= 115200& CO_TX_EXIRDAap);
)
		uap->curregs[1]]4] 0;X1CL|;	}
p->curregs[1]]11] 0;5CTRxCPe cTCTRxCP;	}
p->curregs[1]]14] 0;
	* COBRGnoffo*	YSRp->curregs[1]]]2] 0;
		ddp->curregs[1]]]3] 0;
		dfp->flags &= PMACZILOG_FLAG_RE_EXTCLK(u		delse {
		uarth choaud r)
		uacasen_CLEAOCK/16:* Se230400&
	ifc
p->curregs[1]]4] 0;X16CL|;ifc
p->curregs[1]]11] 0;0;ifc
p->curregs[1]]14] 0;0;ifc
eak;
		udcasen_CLEAOCK/32:* Se115200&
	ifc
p->curregs[1]]4] 0;X32CL|;ifc
p->curregs[1]]11] 0;0;ifc
p->curregs[1]]14] 0;0;ifc
eak;
		uddefault:ifc
p->curregs[1]]4] 0;X16CL|;ifc
p->curregs[1]]11] 0;TCBhe cRCBh;ifc
ea( =eBPS_TO_BRGaud r,n_CLEAOCK / 16);ifc
p->curregs[1]]12] 0;(ea( & 255);ifc
p->curregs[1]]13] 0;((ea( >> 8) & 255);ifc
p->curregs[1]]14] 0; 1AB))		
i}	d	p->flags &= ~PMACZILOG_FLAG_BR_EXTCLK(u		de
#* Clonncter ansiz stop bits, and clrity_m.W
	YSp->curregs[1]3]  ~(EINNASK);		ip->curregs[1] &  ~(ETNNASK);		
arth choacag;
n& CSIZE)
		ucasenCS5:	ddp->curregs[1]3]  PMRN5		ddp->curregs[1] &  PMEN5		ddp->curity_mask;
 0;0x1f		ddeak;
		ucasenCS6:	ddp->curregs[1]3]  PMRN6		ddp->curregs[1] &  PMEN6		ddp->curity_mask;
 0;0x3f		ddeak;
		ucasenCS7:	ddp->curregs[1]3]  PMRN7		ddp->curregs[1] &  PMEN7		ddp->curity_mask;
 0;0x7f		ddeak;
		ucasenCS8:	ddefault:ifcp->curregs[1]3]  PMRN8		ddp->curregs[1] &  PMEN8		ddp->curity_mask;
 0;0xff		ddeak;
		u}	ip->curregs[1]4&= ~(EXSBASK);
	}
 (ch ag;
n& CSTOPB)	}
p->curregs[1]4&  PMSB2;}
se
#d	bp->curregs[1]4&  PMSB1	}
 (ch ag;
n& PARENB)	}
p->curregs[1]4&  PM[Krd B))		
se
#d	bp->curregs[1]4&  ~PMAKrd B))		
 (st!h ag;
n& PARODD
)	}
p->curregs[1]4&  PM[Krd VEN
	
se
#d	bp->curregs[1]4&  ~PMAKrd VEN
	/*p->port.syad_zsatus. ask;
 0;5xract		
 (stiag;
n& INPCK)	}
p->curt.syad_zsatus. ask;
  PMand sA2iKx[Krd th		
 (stiag;
n& (IGNBRKiKx 1NINTiKx[KrMRK))	}
p->curt.syad_zsatus. ask;
  PM 1NT)
		
	/*p->port.synedoresatus. ask;
= ;
		d (ZSiag;
n& IGN[Kr)	}
p->curt.synedoresatus. ask;
= PMand sA2iKx[Krd th		
 (stiag;
n& IGNBRK)
		uap->curt.synedoresatus. ask;
= PM 1NT)
		
		d (ZSiag;
n& IGN[Kr)	}
ap->curt.synedoresatus. ask;
= PM5xract		

	ZS (uah ag;
n& CREAD)= 0 &		uap->curt.synedoresatus. ask;
=0;0xff		}
/* 
* ThS ext UAirdaice cc/onhe poic_po the Frecialfd waud rate g */
static void pmz_inirda_s Tt_truct uart_pmac_port *uap, u8 igned chng wi*ud r{
	ifu8 cmdte o		c ntmt,frsion[]		
arth choa*ud r)
		u SetIRrde csW
	YScasen2400:ifccmdte o=0;0x53		ddeak;
		ucasen4800:ifccmdte o=0;0x52		ddeak;
		ucasen9600:ifccmdte o=0;0x51		ddeak;
		ucasen19200:ifccmdte o=0;0x50		ddeak;
		ucasen38400:ifccmdte o=0;0x4f		ddeak;
		ucasen57600:ifccmdte o=0;0x4e		ddeak;
		ucasen115200:ifccmdte o=0;0x4d		ddeak;
		u/The UAFIRrde csWd dt thgily, apport
 td athis proo nt,hh =*  *idoo doselt thatloupe.
 ? viahe FreCR/onhKeyLargo ?
	/
	ifcasen1152000:ifccmdte o=0;0		ddeak;
		ucasen4000000:ifccmdte o=0;0		ddeak;
		udefault:* CO9600&
	ifccmdte o=0;0x51		dd*ud r=0;9600		ddeak;
		u}	}
 A Waitor thensmitter i iKw rn fa
	ifeh&u00; 
		dile (1))ad_zsdag(uap, R0R0)=egTx_BUF_EMP)= 0 &
o     - ||1)ad_zsdag(uap, R0R1
& hL_SNT)
	= 0 &	ENABx (ua--t <0 &	ENABx&z_error(fmmensmitter i dadt th rn f");
			pmturn f	}
i}	d	play(100)
		i}	}
 A Drn fae enceive_cr tooa
	ifeh&u00;		ioid)read_zsdata(uap);
		roid)read_zsdata(uap);
		roid)read_zsdata(uap);
		rmlay(100)
		iile (1)ad_zsdag(uap, R0R0)=egharCH_AV)
		ifad_zsdata(uap);
		remlay(100)
		ix (ua--t <0 &	ENABx&z_error(fmmceive_cr dadt th rn f");
			pmturn f	}
i}	d}	/* Setth cho avcommd clde co
	YSp->curregs[1]] &  PMDTR;}
ite_zsreg(uap, R15 rep->curregs[1]] &
		isync(uap);
	ZSmlay(100)		/* Setth choC */ av19200W
	YSz_innsor_nt_tomzstp, R1CS8,a0,a;9200	;		}
z_load_zsregs(uap, uap->curregs);
		}mlay(100)		/* SeWte R1g iorsion 2 commd clte o=
	write_zsreta(uap);ua1
	ZSeh&u5; 
		dile (1))ad_zsdag(uap, R0R0)=egharCH_AV)
 0 &	ENABx (ua--t <0 &	ENABx&z_error(fmmirda_s Tt_eme wdst IRon1g iorsion 2 te o");
			pmK&iKwoutrmif}	d	play(100)
		i}		rsion 2 read_zsdata(uap);

		i (uarsion 2 < 4	ENABxz_erfo("ttIrDr:one,g (1rsion 2 %dot bupport
 td");,orsion 2
			pK&iKwoutrmi}	/* Setdinnupe.
 de co
	YSite_zsreta(uap);uacmdte o
	ZSeh&u5; 
		dile (1))ad_zsdag(uap, R0R0)=egharCH_AV)
 0 &	ENABx (ua--t <0 &	ENABx&z_error(fmmirda_s Tt_eme wdst IRon1upe.
 de cote o");
			pmK&iKwoutrmif}	d	play(100)
		i}		t read_zsdata(uap);

	x (uat !0 cmdte o

x&z_error(fmmirda_s Tt_eupe.
 de cote o re%x (%x)");,otuacmdte o
	Z
xz_erfo("ttIrDrrs Tt_or th% habps,one,g (1rsion 2: %d");,
   *ud r,nrsion 2
			ioid)read_zsdata(uap);
		roid)read_zsdata(uap);
		roid)read_zsdata(uap);
		
wout:		 Setth chock thoKw a = de co
	YSp->curregs[1]] &  ~(EDTR;}
ite_zsreg(uap, R15 rep->curregs[1]] &
		isync(uap);
	Z	ioid)read_zsdata(uap);
		roid)read_zsdata(uap);
		roid)read_zsdata(uap);
		}
/*atic void pm__z_ina iorms ios(ruct uart_pmat *port;
,truct uakrms ios *rms ios,
	brwwwwwwruct uakrms ios *old{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned chng wiud r				z_erbug("pmz: : a iorms ios()");
		ifmemcpyuap->porms ios_cacd , rms ios,Esiz of(ruct uakrms ios;

	/* SeXXXeCck, 
ile ==revwe thchine_isntually, slow = 1nd cl4Mbeupe.
s
	/
/onhe poIRrne,g (.iNotthat ite poIR stiiver pmrregsnt, asoent thkn =*  *iabouthe poFIRrde cnd clhht eupe.
 de cs. Sthe F as t senus f.or t*  *iilementedg pengrps hpport for sye F a,f dosuld haogrele)y/a supomu*  *iA suaaf dll, athceast/onhe poRxoiidh,aile ==int thaoiile Plbng r*  *iathis proo nt.
	/
	if (ua_TX_EXIRDAap);
)
		ua Cloalcaud rate ge
	YS	ud r=0;rt_pmg ioud r_te gurt, chrms ios,EoldR0, 00,a4000000
		c
r_erbug("pmz: : ath choIRDAhoKw% habd rs");,oud r);	ua Clo ext UAirdaice cc/ the Frcht note ge
	YS	z_inirda_s Tt_tp, R1&ud r);	ua ClS exfil stud rate ge
	YS	z_innsor_nt_tomzstp, R1rms ioscur_cag;
,1rms ioscur_iag;
, ud r);	uaz_load_zsregs(uap, uap->curregs);
		}
sync(uap);
	ZSelse {
		uaud r=0;rt_pmg ioud r_te gurt, chrms ios,EoldR0, 00,a230400);	uaz_lonsor_nt_tomzstp, R1rms ioscur_cag;
,1rms ioscur_iag;
, ud r);	ua ClMa sure thmodemhatus interrupts did decoegsct, anfig. ur */
#uBx (uaRT p_ABLE);_MSuap->port, chrms ioscur_cag;
 {
			uap->flrregs[1]]] | E= DCDIE ENSYNCIE ENCTSIE;ifc
p->cuags &= PMACZILOG_FLAG_REDULEM_STATUSrmif}lse {
			pmp->flrregs[1]]] |  ~(EXDCDIE ENSYNCIE ENCTSIE	;ZSd	p->flags &= ~PMACZILOG_FLAG_BRDULEM_STATUSrmif}		if Diad alristers to ree pochiph
	YS	z_inybe_update_regs(stp);
		i}
fp-_hadate_reme wout(rt, chrms ioscur_cag;
, ud r);			z_erbug("pmz: : a iorms ios()one, .");
		
st 
 e UArt lock mu not buld a  /
static void pmz_mas iorms ios(ruct uart_pmat *port;
,truct uakrms ios *rms ios,
	brwwwwruct uakrms ios *old{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	unsigned chng wiags &		_dan_lock(&rq.hse ruart s->ck);flag;
;
					 Disable alIRQ*/onhe port
 W
	YSz_interrupt_control(stp, R0 
		if Set Tt_tnewort
 Wnfig. uratnaln*	YS__z_ina iorms ios(rt, chrms ios,Eold
		/* noRe-able) WIRQ*/onhe port
 W
	YS (ZS_TX_EXOPENap);
)	}
z_inierrupt_control(stp, R01
	Z	iin_unlock(&_i =tuop rruart s->ck);flag;
;
		}#iatic chaon bearacpor_inaype(ruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&utomacz(rt.l
	u		 (ZS_TX_EXIRDAap);
)	}
turn f "Z85c30aCC */- Infr c. yrt.l"
	
se
# Q
 S_TX_EXINTDULEMap)) {	}
turn f "Z85c30aCC */- Inrrul stmodem"
	
turn f "Z85c30aCC */- Sial ports."		
st 
  do lit burequest/receasenmapn_ugwe the poristers tohere,iis p* Thhpeninsiathrly serial coobe
 ime.
 */
static stid pmz_mareceaseort *(ruct uart_pmat *port;
{
	s}dtatic vointmz_marequest_rt *(ruct uart_pmat *port;
{
	s	turn fa0rm
st 
 T F as lit buededo av a pnybng ri nrs estg rieier e  /
static void pmz_manfig. _rt *(ruct uart_pmat *port;
int enag;
;

	s}dt 
  do lit bupport fol ing pee pous r ms weth the covisor. ,iQ Q,oetc./
static vointmz_mar_nify_rt *(ruct uart_pmat *port;
inruct uarial_coruct ua*ria{
	s	turn fa-EINV  */}#ifdef CONFIG_PPNSOLE *_POLLdtatic vointmz_mapo hmg ioar(&uruct uart_pmat *port;
{
	struct ttrt_pmac_port *uap)
h&ifcctroae_ir_of(rt;
inruct uart_pmac_port *uart s)		c ntmtrs + = 2		while (1)trs +	ENABx (ua)ad_zsdag(uap, R0R0)=egharCH_AV)
!0 &		ua	turn faru_zsdata(uap);
		re (uatrs +--)	}
aplay(105
		i}	}
turn faNO_POLLrCHARrm
static void pmz_inpo hmpuioar(&uruct uart_pmat *port;
u8 igned char ch,{
	struct ttrt_pmac_port *uap)
h&ifcctroae_ir_of(rt;
inruct uart_pmac_port *uart s)		}
 A Waitor the transmitte buffero avempty.W
	YSile (1))ad_zsdag(uap, R0R0)=egTx_BUF_EMP)= 0 &)

aplay(105
		iite_zsreta(uap);uac)rm
stndif /* CONFIG_PPNSOLE *_POLLi

		atic chaon beruct uart_pms = z_inpo = 0;	st.ax_empty	=	z_inax_empty	.mis iomctrl	=	z_ins iomctrl	.mig iomctrl	=	z_ing iomctrl	.miatopnax	=	z_instopnax	.miat-_hatx	=	z_inst-_hatx	.miatopnrx	=	z_instopnrx	.miable) _ms	=	z_inable) _ms	.migak;
_ctl	=	z_ingak;
_ctl	.miat-_hup	=	z_inst-_hup	.miatdown t	=	z_instdown t	.mis iorms ios	=	z_ins iorms ios,
	.aype		=	z_inayph,
	.receaseort *	=	z_inreceaseort *,
	.request_rt *	=	z_inrequest_rt *,
	.nfig. _rt *	=	z_innfig. _rt *,
	.r_nify_rt *	=	z_inr_nify_rt *,ifdef CONFIG_PPNSOLE *_POLLd	.po hmg ioar(&	=	z_inpo hmg ioar(&,d	.po hmpuioar(&	=	z_inpo hmpuioar(&,#ndif

#};#ifdef CONFIG_PPC_PMAC
#i* 
* Tht Tt_te, nrt *uruct ue thafr i obe
g p,nHWx disewn athis proo nt,* ThUnke sunzilog twe do it thededo ave-pritithatloupinngckoase do it t* Thprsters houronsole, rber t se-_haa s_e, _rt *()oitoaled w*/
static vointm__itithz_inieit_rt *(ruct uart_pmac_port *uap)
{
	struct ttdevice_ne co
np =ep->cune c		ucon bearacpoconn		ucon beruct uarlotame	=s_ngrpENABx nt	unt;

sch =(&	me	=[1]	ZSel*rlot&		c ntmlen		uruct ua esourcporort *s,Ernrxdma,Erntxdma		}
 A	  *iRequest=egmapochiphristers t
	/
	if (uaofaa s ess_tom esourcp(n R0 RE&rort *s {	}
turn f -ENULEV;/*p->port.symapbasen=orort *siat-_h;/*p->port.symembasen=oi t map(p->port.symapbase,;0x1000
		YSp->curtrol(segs( =ep->curt.symembase;/*p->pota(uegs( =ep->curtrol(segs( +;0x10;
	/* N	  *iRequest=egmapoDBA suristers t
	/
	ifdef COHAS_DBA sif (uaofaa s ess_tom esourcp(n R01RE&rotxdma)= 0 && u
o    ofaa s ess_tom esourcp(n R02RE&rorxdma)= 0 &			pp->flags |= PMACZILOG_FLAG_REHAS_DMA;ifse
#d	memaetuarotxdmaR0 REsiz of(ruct ua esourcp;

	imemaetuarorxdmaR0 REsiz of(ruct ua esourcp;

	ndif

			 (ZS_TXHAS_DMAap);
)
		uap->cuax_dmaegs(sn=oi t map(rotxdmaiat-_h,;0x100
		re (uap->cuax_dmaegs(sn=NULL) {
		ZSd	p->flags &= ~PMACZILOG_FLAG_BRHAS_DMA;iagoto nexo_dma		
i}	d	p->flrx_dmaegs(sn=oi t map(rorxdmaiat-_h,;0x100
		re (uap->curx_dmaegs(sn=NULL) {
		ZSd	int;map(p->poax_dmaegs(s	;ZSd	p->flax_dmaegs(sn=oLL) ;ZSd	p->flags &= ~PMACZILOG_FLAG_BRHAS_DMA;iagoto nexo_dma		
i}	d	p->flax_dmaei = jii =aofaparseoand_map(n;ua1
	ZS	p->flrx_dmaei = jii =aofaparseoand_map(n;ua2
		i}	xo_dma:	}
 A	  *iDett tnrt *uayph
	/
	if (uaofadevice_is_mpleacle(x)n;ua"cobalt")			pp->flags |= PMACZILOG_FLAG_RE_EXINTDULEM		ucon2 reofat iorgrps ty)n;ua"AAPL,con2t tor"RE&len
	}
 (ch on2 &&ZSatrcmph on2ua"infr c. ")= 0 &				pp->flags |= PMACZILOG_FLAG_RE_EXIRDA;/*p->port.snayph PMACZI_C *_ASYNC		u/Th1999ewerMabook G3as torlot-me	=s rgrps tyoinersa*/
#uBrlot& reofat iorgrps ty)n;ua"rlot-me	=s"RE&len
	}
 (chrlot& &&Zrlot&curtt;
 > &	ENABx (uaatrcmphrlot&cume	=,MtIrDr")= 0 &			ppp->flags |= PMACZILOG_FLAG_RE_EXIRDA;/*
se
# Q
 Satrcmphrlot&cume	=,MtModem")= 0 &			ppp->flags |= PMACZILOG_FLAG_RE_EXINTDULEM		u}		 (ZS_TX_EXIRDAap);
)	}
p->port.snayph PMACZI_C *_IRDA;/*Q
 S_TX_EXINTDULEMap)) {
		ifruct ttdevice_ne c* i2c_modemh&ifc	ofafind_ne c_byame	=(LL) ua"i2c-modem"
		re (uai2c_modem{
			uacon bearac* m pm&ifc		ofat iorgrps ty)i2c_modemua"modem-id"uaLL) {;iago (uamid) ath ch(*mid) 			uacasen0x04 :		uacasen0x05 :		uacasen0x07 :		uacasen0x08 :		uacasen0x0b :		uacasen0x0c :		ua
p->port.snayph PMACZI_C *_I2S
			if	pox&nr ntk(KERNXINFO "ac_polog t: i2c-modemisett t.
intd: %d");,
  		m pm? (*mid) :a0
			pmofane c_puiai2c_modem{rmif}lse {
			pmnr ntk(KERNXINFO "ac_polog t: rial comodemisett t.
");
			p}	d}	/* S	  *iQtitht maing */gs,  of mzt *"uruct ue t
	/
	ifp->curt.synoayph PMUPIO_MEM		up->curt.syn = jii =aofaparseoand_map(n;ua0)		up->curt.syrt_pcl
=0;_CLEAOCK		up->curt.syfifosiz  ev
			p->curt.syo = 0;&z_inpo =			p->curt.syayph PMAORTMAC
#_LOG_F		up->curt.syfgs |=0;

	/* S	  *iFixt_or the port
 Won Gatwi,kor thile ==e covevice-treFrr s
	/
/ttson ri nrs pt_c  *Normly, wee coc_pioadevawld hoctroae_
	/
/fixedot_oterrupts. fo(",ut Wtwese R7e povevice-treFrdigsct, 
	/
/here/duin rerly seobe
g p soo doededo  chfixt_otoo.
	/
	if (uap->curt.syn = j0 && u
o    n>curitenit&&Zn>curitenicuritenit&&
o    ofadevice_is_mpleacle(x)n;curitenicuriteniua"gatwi,k"
)
		ua ClIRQ*/onhgatwi,kid deofft R7gy 64o*	YSRp->curt.syn = jii =acak;te_mapn_ug(LL) ua64o+ 15
	ZS	p->flax_dmaei = jii =acak;te_mapn_ug(LL) ua64o+ 4
	ZS	p->flrx_dmaei = jii =acak;te_mapn_ug(LL) ua64o+ 5
		i}	}
 Set Tt_taethevalidaud rate gefo("rmltnaln fae encesters *  *ishasews soo do it thite_z craahe Fr rber t sud rate gefs
	/
/fir be titializ (.
	/
	ifz_innsor_nt_tomzstp, R1CS8,a0,a9600
		stturn fa0rm
st 
* ThU TXridaof drrt
 Won modu alcemoval*/
static stid pmz_madisposeort *(ruct uart_pmac_port *uap)
{
	struct ttdevice_ne co
np				np =ep->cune c		uint;map(p->porx_dmaegs(s	;ZSint;map(p->poax_dmaegs(s	;ZSint;map(p->portrol(segs()		up->cune co=oLL) ;ZSofane c_puianp

	imemaetuu, R0 REsiz of(ruct uart_pmac_port *))		
/* 
* ThCled whupon ma choth than escc ne co fae envevice-treF.*/
static vointmz_maattach(ruct uac_pioadeva*mdev,haon beruct uaofadevice_i*/
ma ch{
	struct ttrt_pmac_port *uap)
		c ntmi;
	/* N Irs atR7e por_inpor did rayn refind drma chg Txentry
	/
	ifr th(i 0;
	*i < MAX__CLAORTS	*i++)	re (uar_inpor d[i].ne co== mdev->oef v.f v.ofane c			break;
		u (uaia>= MAX__CLAORTS{	}
turn f -ENULEV;/	YSp-> 0;&z_inpor d[i];/*p->poteva= mdev		up->curt.syteva= &mdev->oef v.f v;	udevns iodrvta(ua&mdev->oef v.f v,op);
		
f 
  doill haacte_atR7e port *uevenhen thfailg ri threquesta esourcps
	/
/ avwork/d nd inbugno th ncienitApplenvevice-treFt
	/
	if (uac_pioarequest_ esourcps(p->potev, "ac_polog t")			pnr ntk(KERNXWARNING "%s: Failedo avrequesta esourcp"
go     - ",nrt *urul haacte_e");,	    wwwww ->cune ccume	=);	
se
#d	bp->cuags |= PMACZILOG_FLAG_RE1SRC_REQUESTEu		_dturn f e-_haa s_e, _rt *(&z_ine-_hags(, ap->port.l
		
/* 
* ThTsasooneosuld hat bubeoaled w,ac_pio=int thgily, aa hot&wapnvevice,* Th do it thpect thoneo the oseerial cooor di avgo away...*/
static vointmz_masetach(ruct uac_pioadeva*mdev{
	struct ttrt_pmac_port *	ap)
h&uv_nag iodrvta(ua&mdev->oef v.f v);
	/* (ZS!p)
{	}
turn f -ENULEV;/		e-_hagsmove_e, _rt *(&z_ine-_hags(, ap->port.l
		if (uap->cuags &= MACZILOG_FLAG_RE1SRC_REQUESTEu)
		uac_pioareceaseo esourcps(p->potev
	ZS	p->flags &= ~PMACZILOG_FLAG_BRGSRC_REQUESTEu		p}	ddevns iodrvta(ua&mdev->oef v.f v,oLL) {;iap->poteva= LL) ;ZSp->curt.syteva= LL) ;ZSstturn fa0rm
sttatic vointmz_masusndin(ruct uac_pioadeva*mdev,hpm_ms wage_tmz__ate ={
	struct ttrt_pmac_port *uap)
h&uv_nag iodrvta(ua&mdev->oef v.f v);
if (uap->n=NULL) {
			pnr ntk("HRM  acz_masusndinoth thLL) w ->");
			pturn fa0rm

	ZSp-_pmsusndin_rt *(&z_ine-_hags(, ap->port.l
		ifturn fa0rm
sttatic vointmz_maaloume(ruct uac_pioadeva*mdev{
	struct ttrt_pmac_port *uap)
h&uv_nag iodrvta(ua&mdev->oef v.f v);
if (uap->n=NULL) {		pturn fa0rm		e-_hagsoume_rt *(&z_ine-_hags(, ap->port.l
		ifturn fa0rm
st 
* ThPbe
 il inoor di fae ensyersmnd clbuild7e port *did ray,cweegisters * Th h the corial colay isler v, soo dot inaengrps hpuct ttdevicehile =* andow =tim enttyn reattachengrps lh>his pr prler veat nhithus fo avbe
 *ig tim enttynlay isgily, awan */ithat ittay.*/
static vointm__itithz_inobe
 oid)re
	struct ttdevice_ne c	*ne c_p, *ne c_a, *ne c_b,o
np		x nt			unt;
 0;
		d  lifarc
	/* S	  *iFind d inescc chipdi fae ensyersm
	/
	ifr t_each_ne c_byame	=(ne c_p, "escc")
		ua C
   *iFir bet inlnnel HWA/B ne cooo nts t
	  *i
	  *iDO:   Addh ndte, weth thngrps hck dg ri th a e at...*   *	YSRne c_a =hne c_bn=oLL) ;ZSdr th(np =eLL) ;h(np =eofat ioxt T_child(ne c_p, n) {
!=eLL) ;) 			uaQ
 Satrncmphn;cume	=,Mtch-a",n4)= 0 &			ppRne c_a =hofane c_t ianp

	i*
se
# Q
 Satrncmphn;cume	=,Mtch-b",n4)= 0 &			ppRne c_b =hofane c_t ianp

	i*}	re (ua!ne c_a &&Z!ne c_b) 			uaofane c_puiane c_a
			pmofane c_puiane c_b);		pmnr ntk(KERNXsA2i"ac_polog t: ttson rine co%cor thescc %pOF");,
  		a!ne c_a
 ? 'a' : 'b',hne c_p

	i*
rtroinuermif}		if D
   *iFi be edicefielddi fae enrt *uruct ue ts
   *	YSRQ
 Sne c_b !NULL) {
			p	z_inpor d[unt;
].mer 		0;&z_inpor d[unt;
+1]	ZSp	z_inpor d[unt;
+1].mer 		0;&z_inpor d[unt;
]
	i*}	rez_inpor d[unt;
].ags &		0;ACZILOG_FLAG_RE_EXNHANNEL_A;	rez_inpor d[unt;
].ne c		=hne c_a;	rez_inpor d[unt;
+1].ne c		=hne c_b;	rez_inpor d[unt;
].rt.syne, 	=hunt;

schz_inpor d[unt;
+1].rt.syne, 	=hunt;
+1;		if D
   *it Tt_te port *dir thgily
   *	YSRrch&uacinieit_rt *(&z_inpor d[unt;
]
		re (uarch&0 && u ne c_b !NULL) {
	SRrch&uacinieit_rt *(&z_inpor d[unt;
+1]
		re (uarch!0 &	ENABxaofane c_puiane c_a
			pmofane c_puiane c_b);		pmmemaetuaz_inpor d[unt;
]R0 REsiz of(ruct uart_pmac_port *))			pmmemaetuaz_inpor d[unt;
+1]R0 REsiz of(ruct uart_pmac_port *))			pmrtroinuermif}			unt;
 += 2		f}		z_inpor d_unt;
 0;unt;

sstturn fa0rm
stnde
#dteernalhpuct ttpler("rmadevice scpoa_pdev,hscpob_pdev;
tatic vointm__itithz_inieit_rt *(ruct uart_pmac_port *uap)
{
	struct tt esourcpo*rort *s		c ntmirq
ssttnpor di=tpler("rmat io esourcp(p->pordev,hIORESOURCE_MEMua0)		ui = jipler("rmat ioi =ap->pordev,h0
	}
 (ch!tnpor di||mirq <0 &		}
turn f -ENULEV;/		e->port.symapbasenn=orort *s->ate_h;/*p->port.symembasen =e(pigned char ch__iomem *)orort *s->ate_h;/*p->port.synoayph   PMUPIO_MEM		up->curt.syn =      =mirq
sup->curt.syrt_pcl
==0;_CLEAOCK		up->curt.syfifosiz  ev
			p->curt.syo =      =m&z_inpo =			p->curt.syayph     =mAORTMAC
#_LOG_F		up->curt.syfgs |=   =m0		YSp->curtrol(segs(   =ep->curt.symembase;/*p->pota(uegs(      =mp->curtrol(segs( +;4;/*p->port.snayph =   =m0		YSz_innsor_nt_tomzstp, R1CS8,a0,a9600
		stturn fa0rm
static vointm__itithz_inobe
 oid)re
	stintmerr		YSz_inpor d_unt;
 0;0		YSz_inpor d[0].rt.syne,  0;
		dz_inpor d[0].fgs |=    0;ACZILOG_FLAG_RE_EXNHANNEL_A;	rz_inpor d[0].rdeva     =m&scpoa_pdev;	
srrh&uacinieit_rt *(&z_inpor d[0]
	}
 (chsrr		}
turn f err		Sz_inpor d_unt;
Kwrm	cz_inpor d[0].mer       =m&z_inpor d[1]	ZSz_inpor d[1].mer       =m&z_inpor d[0]	ZSz_inpor d[1].rt.syne,  0;1	ZSz_inpor d[1].fgs |=    0;0	ZSz_inpor d[1].rdeva     =m&scpob_pdev;	
srrh&uacinieit_rt *(&z_inpor d[1]
	}
 (chsrr		}
turn f err		Sz_inpor d_unt;
Kwrm	cturn fa0rm
static void pmz_indisposeort *(ruct uart_pmac_port *uap)
{
	stmemaetuu, R0 REsiz of(ruct uart_pmac_port *))		
/*atic vointm__itithz_inattach(ruct uapler("rmadevice *pdev{
	struct ttrt_pmac_port *uap)
		c ntmi;
/* N Irs atR7e por_inpor did rayn refind drma chg Txentry/
	ifr th(i 0;
	*i < z_inpor d_unt;
	*i++)	re (uar_inpor d[i].rdeva&0 pdev{
	break;
		u (uaia>= z_inpor d_unt;
		}
turn f -ENULEV;/		e-> 0;&z_inpor d[i];/*p->port.syteva= &pdev->dev;	
pler("rmas iodrvta(uapf v,op);
		
fturn f e-_haa s_e, _rt *(&z_ine-_hags(, ap->port.l
		
/*atic vointm__exitmz_masetach(ruct uapler("rmadevice *pdev{
	struct ttrt_pmac_port *uap)
 jipler("rmat iodrvta(uapf v);
if (ua!p)
{	}
turn f -ENULEV;/		e-_hagsmove_e, _rt *(&z_ine-_hags(, ap->port.l
		ifp->curt.syteva= LL) ;Zstturn fa0rm
stndif /* CO!NFIG_PPC_PMAC
#/

		fdef CONFIG_PPSERIAL_ACZILOG_FLNSOLE *static void pmz_innsole, aite_z(ruct uansole, rocon,haon bearacposu8 igned cht locnt;
	;*atic vointm__itithz_innsole, as Tt_truct uansole, roco,earacpooional;
			atic voruct uansole, rz_innsole,  0;	st.me	=	=	ACZILOG_NAME		,d	.ite_z	=	z_innfile, aite_z,d	.device	=	p-_hacfile, avevice,*mis iup	=	z_innsole, as Tt_,*miags &	=	NSO_PRINTBUFFER,*miindex	=	-1,d	.da =   =	&z_ine-_hags(,#};#iff Ce,  ACZILOG_FLNSOLE *	&z_innsole, tnde
#* CONFIG_PPSERIAL_ACZILOG_FLNSOLE */
	iff Ce,  ACZILOG_FLNSOLE *	(LL) {
ndif /* CONFIG_PPSERIAL_ACZILOG_FLNSOLE */
	i* 
* ThResters he envver p,ansole, river pmd stpor di h the corial c* Thcot * ustatic vointm__itithz_ingisters oid)re
	stz_ine-_hags(.nrh&uacinpor d_unt;
	stz_ine-_hags(.nsol 0;ACZILOG_FLNSOLE *		}
 A	  *iResters he  disver pm h the corial cocot *  *	YSturn f e-_hagisters _sver p(&z_ine-_hags()		}#ifdef CONFIG_PPC_PMAC
#i*atic chaon beruct uaofadevice_i*/z_inyb ch[] =
	st	st.me	=		=htch-a",	f},st	st.me	=		=htch-b",	f},st	},#};#MODULE_LEVICE_TLE);uaof,/z_inyb ch
			atic voruct uac_pioadver pmz_masver pm0;	st.sver pm0;	stt.me	= 		=htac_polog t",
  .owner		=hTH_EXMODULE,
  .ofnyb ch_tle) 	=/z_inyb ch,	f},st.obe
 		=/z_inattach,
	.remove		=/z_insetach,*misusndin	=/z_insusndin,
	.reoume		=/z_inreoume,#};#ifse
#dtatic voruct uapler("rmadver pmz_masver pm0;	st.remove		=/__exit_p(z_insetach),st.sver p		=/	stt.me	=		=htscc",	f},s};#ifsif /* CO!NFIG_PPC_PMAC
#/

		atic vointm__itithieit_rmzoid)re
	stintmrc,mi;
	nr ntk(KERNXINFO "%s");,orsion 2
		/* N 
  *iFir b,o doededo Kw a prdigsct OF-basedoobe
 ipass.e d*  *idooat itbecae R7 dowanttaeal console, ruprber t se c
	/
/t_pio=etuf toaleds usock tand clsie doat itmake*/it
	/
/easiero avpass7e porgrps hnumb anof lnnel Hdi a
	/
/e-_hagisters _sver p()
	/
	if (uar_inpor d_unt;
 00 &		}
z_inobe
 o)		}
 A	  *iBls.hrly se (unonrt *ufnd i
	/
	if (uar_inpor d_unt;
 00 &		}
turn f -ENULEV;/		 A	  *iNowcweegisters h h the corial colay i*  *	YStch&uacingisters o
	}
 (chrc{
			pnr ntk(KERNXsA2i
	br"ac_polog t: EK&ie,gisters g riseal covevice,etabledg penc_polog t.");
   r"ac_polog t: Diand oe Friseal covver pmd ad_zywclaidre lominors?");
	 	ua Cleffecte_e se"z_inenobe
 o)" *	YSRr th(i=
	*i < z_inpor d_unt;
	*i++)	re	z_indisposeort *(&z_inpor d[i]
			pturn farc
	d}	/* S	  *iTn thweegisters he coc_pioovver pmitself
	/
	ifdef CONFIG_PPC_PMAC
#ipturn fac_pioaresters _sver p(&z_insver p);ifse
#d	turn fapler("rmadver pnobe
 o&z_insver p,/z_inattach

	ndif

m
static void pm__exitmexit_pmzoid)re
	stintmi;#ifdef CONFIG_PPC_PMAC
#ia ClU TXridaof c_pio-vver pm(setachorm drc_pio)/
	ifc_pioaunresters _sver p(&z_insver p);ifse
#d	pler("rmadver pnunresters (&z_insver p);ifsif

mifr th(i 0;
	*i < z_inpor d_unt;
	*i++)
		ifruct ttrt_pmac_port *uaprt *u0;&z_inpor d[i];/fdef CONFIG_PPC_PMAC
#iaf (uappor cune co!NULL) {
	SRz_indisposeort *(urt.l
		fse
#d	f (uappor curdeva!NULL) {
	SRz_indisposeort *(urt.l
		fsif

mf}		/ThUngisters hRT povver pm
	ifp-_hadnresters _sver p(&z_ine-_hags()		}#ifdef CONFIG_PPSERIAL_ACZILOG_FLNSOLE *static void pmz_innsole, aputar(&uruct uart_pmat *port;
u8t loch{
	struct ttrt_pmac_port *uap)
h&ifcctroae_ir_of(rt;
inruct uart_pmac_port *uart s)		}
 A Waitor the transmitte buffero avempty.W
	YSile (1))ad_zsdag(uap, R0R0)=egTx_BUF_EMP)= 0 &)

aplay(105
		iite_zsreta(uap);uach)rm
st 
* ThPbt loanrucg ri the corial cort *utryg trt bu Kw terurb* thanofpossie alce coe R7 the port.sy. */
static stid pmz_mansole, aite_z(ruct uansole, rocon,haon bearacposu8 igned cht locnt;
	
	struct ttrt_pmac_port *uap)
 jiaz_inpor d[unn->index];/*pigned chng wiags &		_dan_lock(&rq.hse ruap->port.lock);flag;
;
			u/Then fa thterrupts did clable) We transmitter i *
	write_zsreg(uap, R1, rep->flrregs[1] &= (ETNT_MAAB));;}
ite_zsreg(uap, R15 rep->curregs[1] &  hTNABLE);e chTSe cDTR);/		e-_hansole, aite_z(ap->port, ch ancnt;
,mz_innsole, aputar(&
		/* noReop rrWe trvaluesn fae encesters s *
	write_zsreg(uap, R1, rep->flrregs[1] &;;}
 Dislt thsabled.he chansmitter i *
	wZSin_unlock(&_i =tuop rruap->port.lock);flag;
;
		}i* 
* Tht Tt_te corial cocoole, t ustatic vointm__itithz_innsole, as Tt_truct uansole, roco,earacpooional;

	struct ttrt_pmac_port *uap)
		cruct uart_pmat *port;
		c ntmud r=0;38400		c ntmus,  0;8		c ntmrity_m 0;'n'		c ntmfw = 0;'n'		cpigned chng wipwdebuy(1rm/* S	  *iXServe'disefaultu Kw57600abps
	/
	if (uaofachine_i_is_mpleacle(x)"Rk tMac1,1")
o    || ofachine_i_is_mpleacle(x)"Rk tMac1,2")
o    || ofachine_i_is_mpleacle(x)"MacRISC4")			pud r=0;57600rm/* S	  *iCck, 
ilee Fri nhinvalidart_phnumb ans tobeiniecialfd w,hd
 *  *iif so,esrlychor the trfir beavls.led.hrt *utt drioesnhe r*  *iceole, rsport f.
	/
	if (uaco->indexa>= z_inpor d_unt;
		}
co->indexa0;
		dp-> jiaz_inpor d[un->index];/fdef CONFIG_PPC_PMAC
#ia (uap->cune co== LL) {		pturn fa-ENULEV;/fse
#d	 (uap->curdeva&0 LL) {		pturn fa-ENULEV;/fsif

mfrt *u0;&p->port.lrm/* S	  *iMarkhrt *u tobeig ria coole, t	/
	ifp->cuags |= PMACZILOG_FLAG_RE_EXNSOLrm/* S	  *iTemporaryhfixor thrt_phlay iswho dadt ths Tt_te corpinngckoyett	/
	ifan_lock(&rqtituart s->ck);)		}
 A	  *iEble) We trr cdre s
	/
	ifzwdebuy(1 0;__z_inat-_huptp);
		} (uarwdebuy(1{		pmlay(10rwdebuy(1
			if (uaoional;

		e-_haparseooional;aoional;R1&ud rR1&rity_mR1&us, an&fw =
		
fturn f e-_has iooional;art, chco,eud rR1rity_mR1us, anfw =
		
/*atic vointm__itithz_innsole, aqtituid)re
	st/ThPbe
 ipor di
	ifz_inobe
 o)		}
 (uar_inpor d_unt;
 00 &		}
turn f -ENULEV;/		 AiDO:   Autoobe
 insole, rbasedoon OFi
	if Aiz_innsole, .indexa0;i; *	YStusters _nsole, (&z_innsole, 
		stturn fa0rm	
/nsole, aqtitaled(z_innsole, aqtit
		fsif

* CONFIG_PPSERIAL_ACZILOG_FLNSOLE */
	i*modu aaqtituieit_rmz
		modu aaexit(exit_pmz
