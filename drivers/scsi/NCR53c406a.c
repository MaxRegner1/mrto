/* 
 *  NCR53c406.c
 *  Low-level SCSI driver for NCR53c406a chip.
 *  Copyright (C) 1994, 1995, 1996 Normunds Saumanis (normunds@fi.ibm.com)
 * 
 *  LILO command line usage: ncr53c406a=<PORTBASE>[,<IRQ>[,<FASTPIO>]]
 *  Specify IRQ = 0 for non-interrupt driven mode.
 *  FASTPIO = 1 for fast pio mode, 0 for slow mode.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2, or (at your option) any
 *  later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 */

#define NCR53C406A_DEBUG 0
#define VERBOSE_NCR53C406A_DEBUG 0

/* Set this to 1 for PIO mode (recommended) or to 0 for DMA mode */
#define USE_PIO 1

#define USE_BIOS 0
				/* #define BIOS_ADDR 0xD8000 *//* define this if autoprobe fails */
				/* #define PORT_BASE 0x330 *//* define this if autoprobe fails */
				/* #define IRQ_LEV   0	*//* define this if autoprobe fails */
#define DMA_CHAN  5		/* this is ignored if DMA is disabled */

/* Set this to 0 if you encounter kernel lockups while transferring 
 * data in PIO mode */
#define USE_FAST_PIO 1

/* ============= End of user configurable parameters ============= */

#include <linux/module.h>

#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/irq.h>

#include <linux/blkdev.h>
#include <linux/spinlock.h>
#include "scsi.h"
#include <scsi/scsi_host.h>

/* ============================================================= */

#define WATCHDOG 5000000

#define SYNC_MODE 0		/* Synchronous transfer mode */

#ifdef DEBUG
#undef NCR53C406A_DEBUG
#define NCR53C406A_DEBUG 1
#endif

#if USE_PIO
#define USE_DMA 0
#else
#define USE_DMA 1
#endif

/* Default configuration */
#define C1_IMG   0x07		/* ID=7 */
#define C2_IMG   0x48		/* FE SCSI2 */
#if USE_DMA
#define C3_IMG   0x21		/* CDB TE */
#else
#define C3_IMG   0x20		/* CDB */
#endif
#define C4_IMG   0x04		/* ANE */
#define C5_IMG   0xb6		/* AA PI SIE POL */

#define REG0 (outb(C4_IMG, CONFIG4))
#define REG1 (outb(C5_IMG, CONFIG5))

#if NCR53C406A_DEBUG
#define DEB(x) x
#else
#define DEB(x)
#endif

#if VERBOSE_NCR53C406A_DEBUG
#define VDEB(x) x
#else
#define VDEB(x)
#endif

#define LOAD_DMA_COUNT(count) \
  outb(count & 0xff, TC_LSB); \
  outb((count >> 8) & 0xff, TC_MSB); \
  outb((count >> 16) & 0xff, TC_HIGH);

/* Chip commands */
#define DMA_OP               0x80

#define SCSI_NOP             0x00
#define FLUSH_FIFO           0x01
#define CHIP_RESET           0x02
#define SCSI_RESET           0x03
#define RESELECT             0x40
#define SELECT_NO_ATN        0x41
#define SELECT_ATN           0x42
#define SELECT_ATN_STOP      0x43
#define ENABLE_SEL           0x44
#define DISABLE_SEL          0x45
#define SELECT_ATN3          0x46
#define RESELECT3            0x47
#define TRANSFER_INFO        0x10
#define INIT_CMD_COMPLETE    0x11
#define MSG_ACCEPT           0x12
#define TRANSFER_PAD         0x18
#define SET_ATN              0x1a
#define RESET_ATN            0x1b
#define SEND_MSG             0x20
#define SEND_STATUS          0x21
#define SEND_DATA            0x22
#define DISCONN_SEQ          0x23
#define TERMINATE_SEQ        0x24
#define TARG_CMD_COMPLETE    0x25
#define DISCONN              0x27
#define RECV_MSG             0x28
#define RECV_CMD             0x29
#define RECV_DATA            0x2a
#define RECV_CMD_SEQ         0x2b
#define TARGET_ABORT_DMA     0x04

/*----------------------------------------------------------------*/
/* the following will set the monitor border color (useful to find
   where something crashed or gets stuck at */
/* 1 = blue
   2 = green
   3 = cyan
   4 = red
   5 = magenta
   6 = yellow
   7 = white
*/

#if NCR53C406A_DEBUG
#define rtrc(i) {inb(0x3da);outb(0x31,0x3c0);outb((i),0x3c0);}
#else
#define rtrc(i) {}
#endif
/*----------------------------------------------------------------*/

enum Phase {
	idle,
	data_out,
	data_in,
	command_ph,
	status_ph,
	message_out,
	message_in
};

/* Static function prototypes */
static void NCR53c406a_intr(void *);
static irqreturn_t do_NCR53c406a_intr(int, void *);
static void chip_init(void);
static void calc_port_addr(void);
#ifndef IRQ_LEV
static int irq_probe(void);
#endif

/* ================================================================= */

#if USE_BIOS
static void *bios_base;
#endif

#ifdef PORT_BASE
static int port_base = PORT_BASE;
#else
static int port_base;
#endif

#ifdef IRQ_LEV
static int irq_level = IRQ_LEV;
#else
static int irq_level = -1;	/* 0 is 'no irq', so use -1 for 'uninitialized' */
#endif

#if USE_DMA
static int dma_chan;
#endif

#if USE_PIO
static int fast_pio = USE_FAST_PIO;
#endif

static Scsi_Cmnd *current_SC;
static char info_msg[256];

/* ================================================================= */

/* possible BIOS locations */
#if USE_BIOS
static void *addresses[] = {
	(void *) 0xd8000,
	(void *) 0xc8000
};
#define ADDRESS_COUNT ARRAY_SIZE(addresses)
#endif				/* USE_BIOS */

/* possible i/o port addresses */
static unsigned short ports[] = { 0x230, 0x330, 0x280, 0x290, 0x330, 0x340, 0x300, 0x310, 0x348, 0x350 };
#define PORT_COUNT ARRAY_SIZE(ports)

#ifndef MODULE
/* possible interrupt channels */
static unsigned short intrs[] = { 10, 11, 12, 15 };
#define INTR_COUNT ARRAY_SIZE(intrs)
#endif /* !MODULE */

/* signatures for NCR 53c406a based controllers */
#if USE_BIOS
struct signature {
	char *signature;
	int sig_offset;
	int sig_length;
} signatures[] __initdata = {
	/*          1         2         3         4         5         6 */
	/* 123456789012345678901234567890123456789012345678901234567890 */
	{
"Copyright (C) Acculogic, Inc.\r\n2.8M Diskette Extension Bios ver 4.04.03 03/01/1993", 61, 82},};

#define SIGNATURE_COUNT ARRAY_SIZE(signatures)
#endif				/* USE_BIOS */

/* ============================================================ */

/* Control Register Set 0 */
static int TC_LSB;		/* transfer counter lsb         */
static int TC_MSB;		/* transfer counter msb */
static int SCSI_FIFO;		/* scsi fifo register   */
static int CMD_REG;		/* command register             */
static int STAT_REG;		/* status register              */
static int DEST_ID;		/* selection/reselection bus id */
static int INT_REG;		/* interrupt status register    */
static int SRTIMOUT;		/* select/reselect timeout reg  */
static int SEQ_REG;		/* sequence step register       */
static int SYNCPRD;		/* synchronous transfer period  */
static int FIFO_FLAGS;		/* indicates # of bytes in fifo */
static int SYNCOFF;		/* synchronous offset register  */
static int CONFIG1;		/* configuration register       */
static int CLKCONV;		/* clock conversion reg */
				/*static int TESTREG;*//* test mode register           */
static int CONFIG2;		/* Configuration 2 Register     */
static int CONFIG3;		/* Configuration 3 Register     */
static int CONFIG4;		/* Configuration 4 Register     */
static int TC_HIGH;		/* Transfer Counter High */
				/*static int FIFO_BOTTOM;*//* Reserve FIFO byte register   */

/* Control Register Set 1 */
				/*static int JUMPER_SENSE;*//* Jumper sense port reg (r/w) */
				/*static int SRAM_PTR;*//* SRAM address pointer reg (r/w) */
				/*static int SRAM_DATA;*//* SRAM data register (r/w) */
static int PIO_FIFO;		/* PIO FIFO registers (r/w) */
				/*static int PIO_FIFO1;*//*  */
				/*static int PIO_FIFO2;*//*  */
				/*static int PIO_FIFO3;*//*  */
static int PIO_STATUS;		/* PIO status (r/w) */
				/*static int ATA_CMD;*//* ATA command/status reg (r/w) */
				/*static int ATA_ERR;*//* ATA features/error register (r/w) */
static int PIO_FLAG;		/* PIO flag interrupt enable (r/w) */
static int CONFIG5;		/* Configuration 5 register (r/w) */
				/*static int SIGNATURE;*//* Signature Register (r) */
				/*static int CONFIG6;*//* Configuration 6 register (r) */

/* ============================================================== */

#if USE_DMA
static __inline__ int NCR53c406a_dma_setup(unsigned char *ptr, unsigned int count, unsigned char mode)
{
	unsigned limit;
	unsigned long flags = 0;

	VDEB(printk("dma: before count=%d   ", count));
	if (dma_chan <= 3) {
		if (count > 65536)
			count = 65536;
		limit = 65536 - (((unsigned) ptr) & 0xFFFF);
	} else {
		if (count > (65536 << 1))
			count = (65536 << 1);
		limit = (65536 << 1) - (((unsigned) ptr) & 0x1FFFF);
	}

	if (count > limit)
		count = limit;

	VDEB(printk("after count=%d\n", count));
	if ((count & 1) || (((unsigned) ptr) & 1))
		panic("NCR53c406a: attempted unaligned DMA transfer\n");

	flags = claim_dma_lock();
	disable_dma(dma_chan);
	clear_dma_ff(dma_chan);
	set_dma_addr(dma_chan, (long) ptr);
	set_dma_count(dma_chan, count);
	set_dma_mode(dma_chan, mode);
	enable_dma(dma_chan);
	release_dma_lock(flags);

	return count;
}

static __inline__ int NCR53c406a_dma_write(unsigned char *src, unsigned int count)
{
	return NCR53c406a_dma_setup(src, count, DMA_MODE_WRITE);
}

static __inline__ int NCR53c406a_dma_read(unsigned char *src, unsigned int count)
{
	return NCR53c406a_dma_setup(src, count, DMA_MODE_READ);
}

static __inline__ int NCR53c406a_dma_residual(void)
{
	register int tmp;
	unsigned long flags;

	flags = claim_dma_lock();
	clear_dma_ff(dma_chan);
	tmp = get_dma_residue(dma_chan);
	release_dma_lock(flags);

	return tmp;
}
#endif				/* USE_DMA */

#if USE_PIO
static __inline__ int NCR53c406a_pio_read(unsigned char *request, unsigned int reqlen)
{
	int i;
	int len;		/* current scsi fifo size */

	REG1;
	while (reqlen) {
		i = inb(PIO_STATUS);
		/*    VDEB(printk("pio_status=%x\n", i)); */
		if (i & 0x80)
			return 0;

		switch (i & 0x1e) {
		default:
		case 0x10:
			len = 0;
			break;
		case 0x0:
			len = 1;
			break;
		case 0x8:
			len = 42;
			break;
		case 0xc:
			len = 84;
			break;
		case 0xe:
			len = 128;
			break;
		}

		if ((i & 0x40) && len == 0) {	/* fifo empty and interrupt occurred */
			return 0;
		}

		if (len) {
			if (len > reqlen)
				len = reqlen;

			if (fast_pio && len > 3) {
				insl(PIO_FIFO, request, len >> 2);
				request += len & 0xfc;
				reqlen -= len & 0xfc;
			} else {
				while (len--) {
					*request++ = inb(PIO_FIFO);
					reqlen--;
				}
			}
		}
	}
	return 0;
}

static __inline__ int NCR53c406a_pio_write(unsigned char *request, unsigned int reqlen)
{
	int i = 0;
	int len;		/* current scsi fifo size */

	REG1;
	while (reqlen && !(i & 0x40)) {
		i = inb(PIO_STATUS);
		/*    VDEB(printk("pio_status=%x\n", i)); */
		if (i & 0x80)	/* error */
			return 0;

		switch (i & 0x1e) {
		case 0x10:
			len = 128;
			break;
		case 0x0:
			len = 84;
			break;
		case 0x8:
			len = 42;
			break;
		case 0xc:
			len = 1;
			break;
		default:
		case 0xe:
			len = 0;
			break;
		}

		if (len) {
			if (len > reqlen)
				len = reqlen;

			if (fast_pio && len > 3) {
				outsl(PIO_FIFO, request, len >> 2);
				request += len & 0xfc;
				reqlen -= len & 0xfc;
			} else {
				while (len--) {
					outb(*request++, PIO_FIFO);
					reqlen--;
				}
			}
		}
	}
	return 0;
}
#endif				/* USE_PIO */

static int __init NCR53c406a_detect(struct scsi_host_template * tpnt)
{
	int present = 0;
	struct Scsi_Host *shpnt = NULL;
#ifndef PORT_BASE
	int i;
#endif

#if USE_BIOS
	int ii, jj;
	bios_base = 0;
	/* look for a valid signature */
	for (ii = 0; ii < ADDRESS_COUNT && !bios_base; ii++)
		for (jj = 0; (jj < SIGNATURE_COUNT) && !bios_base; jj++)
			if (!memcmp((void *) addresses[ii] + signatures[jj].sig_offset, (void *) signatures[jj].signature, (int) signatures[jj].sig_length))
				bios_base = addresses[ii];

	if (!bios_base) {
		printk("NCR53c406a: BIOS signature not foundet, (vxURE;*//* Signature Register (r) */
				/*static int CONFIG6;*//* Configuration 6 r (void *) signatures[jg_length))
				bios_baAnt ii, jj;
	*_legned) ptr) & 1))
		panic("NCR53c406a: attempted unaligned DMA transfer\n");

	flagature */
	for c^co==========Bile (dBxCBU4ignature Register (r) */
	) signatures[jg_length]===================================================== */

#if USE_DMA
static __inldBPG
CBP!MA
st
		if (len) {
			if (len > reqlen)
				len = reqlen;

			if (fast_pio && len > 3) biosCBxIet, t, unsigned char mode)
{
	unsigned limit;
	unsigned long flags = 0;

	VDEB(printk(3uLCBxCBxCBPuLCBxCBxd   ", count));
	if (dma_chan <= 3) {
		if (count > 65536)
			count = 65536;
		lim3u	================A^cng flags = 0;

	VD/
staCBxCBxCBUixre Register (r) */
				/*static int CONFIG6;*//* Configuration 6 r (void *) signat
	REG1int CONF __infndefCgnat
t PIO_FIm sjj;
BASE
	int Register (r) */
				===]tic int CONFIG6;*//* Cos_base= reqlen;

			if (fast:;*//* ) binteruseosCBx				===])); (jj < SIGNATURE_V= reqlen;

			if (fast:;*//* ) binavaillongosCBx				===])); (jj	ine CHIP_RESE				===]forif	d)essessetusTA command/sst, un);
				===]forif	gs ^ n);
				===]forif	gs)en)
7gnatun);
				===]forif	gs ^ n);
				===]forif	gs)en)
7gnatun);
				===]forif	gs jj = 8)en)
0x58ios_base;guration 6 r				===]d *) adV= reqlen;

			if (fast:;Sig< 1))
			config(len)d *) adV= reqlen;

	guration =) biosCBx^cng flags)d *) ad = NULL;
+ signareak;
		c(r) */
				===]tic in); (jj ures[jj]gned limit;
	/ioport.h>&& leint R^cng flags = 0;
nohan <= & len > 6553len;

			if (fast:;nohavaillonghan <= & len(len > reqlen)
				len = reqlen;

			if (fast_======eg(len)d aset dma_chan;
#e	case#endif

#)d aif USE_PIO
statie {
		RAY_SIZE(<
{
	int i
	VD/
staCBxCB {
>=/
stat/ssRAY_SIZE(adpio = USE_n > r {
		RAY_SIZE(<
{
	int iTrouonghmmand/3len;

			if (fast:;thou= USlem,ARRAY_SIZE=%d, giv*/
sup0xfc;
RAY_SIZE); (jjgoton--)_eak;
		;ures[jj]gned len = reqlen;

			if (fast:rus*/
sguration 6) biosCBx^cng flags)d 
	if (!bios_ tpnesse->fine nam 6 rCONFIG6;*//*d 
	3c406a: 
				 1))
			(esseticreturn N!3c406DMA
static __inldBPG
CBP!MU"dma: tndicates # bios, giv*/
sup.(len > rgoton--)_eak;
		;ur __inlin	RAY_SIZE(>
{
	i> r {
	egister 	RAn	RAY_SIZE, is 'no irq', so ustict CONFIG6;*//*, 3c406Dios_basqlen;

			if (fast:ru"dma: tndal15 };inclu nt coun
RAY_SIZE); (jjgoton--)_al P_
			;ures[jnesse->can_istuee * tpnt= reqlen;

			if (fast:ral15 };idnclu nt coun
RAY_SIZE)ma_chan, conlin	RAY_SIZE(n)
{
	i[jnesse->can_istuee *0tpnt= reqlen;

			if (fast:rNo;		/* currs_======eg(len)d asqlen;

			if (fast Free Sonoh;

	 ATAupan <= poll*/
s		/* face(len > rqlen;

	Pk;
		;
	ail  SCSI-
			@v	 A.x/blkd.org(len >                        }

	if (countasqlen;

			if (fast:rNo;		/* currs_& len >nd/* =======ic intd. Giv*/
sup.(len >	case 0xc:
			len = 84;
jjgoton--)_al P_
			;ur < SIGNATUR= reqlen;

			if (fast:;Shouldn't		ca prot!(len)d asgoton--)_al P_
			;ur 	}

	if (count	isable_dm=ux/bitopseturn Negister flags;

	flat CONFIG6;*//*  !)
{
	i[jnqlen;

			if (fast:ru"dma: tndal15 };in = 8(C) Acc nt counn = 1;
			brsgoton--)_al P_	RA			len = reqlen;

	Al15 };idn = 8(C) Acc nt counn = 1;
		n >	case 0xc:
			len = 84;

	3c406->	RA(adpio _SIZE;
	3c406->	oma_ch6 r				address	3c406->n_	oma_ch6 rc in;}

	if (count	3c406->n =6 rhort ports[] = { 0x230, 0xount	3qlen;f(ZE(portst CONFIG6;*// > 3) bi,nclu nt,n = 8(C) Acc nt.CBx^cng flagun
RAY_SIZEunn = 1;
			be ADDR	3qlen;f(ZE(portst CONFIG6;*// > 3) bi,nclu nt,n%sude "scsi.CBx^cng flagun
RAY_SIZEunos_base =? "os_b" : "AR Pen >	case = 42;
			b(if (!bi)d aif 0, 0xount      --)_al P_	RA:_inlin	RAY_SIZE
	*_ll P_	RAn	RAY_SIZE, 3c406D >	case =      --)_al P_
			:_i
				un 1))
			(3c406D >      --)_eak;
		:
reak;
		c(r) */
				/*static in		breaase 0xc:
			len = ;
		default:
		eak;
		() {
		printk("NCR53c"NCs[ii];lin3c"NC->	RA
	*_ll P_	RAn3c"NC->	RA, BIOS);}

	if (count	;lin3c"NC->n = 1;
	Acc !)
{xff
	*_ll P_flag3c"NC->n = 1;
	AccD >	case =];lin3c"NC->	oma_ch6nat3c"NC->n_	oma_ch)
areak;
		c(r) */
3c"NC->	oma_ch,t3c"NC->n_	oma_ch)d 
	3				un 1))
			(3c"NCs	breaase 0xc:
		234567890123456789et ma.hfrom if

/	ain.ce count=%d   ",int) signatures[jjEG1;
	n)
{
	itrs[ii]unt=%d _pio_t EG1;
_idxos_base)pio_t upt occuoccs[4====== reqlen;

			if (fast:;SG1;
9et ma.(len > {
	unsi=];lin3G1;
_idxo>= __infndefCg-a_dmi[jnqlen;

			if (fast:rSG1;
9et ma. tno manyiguras.  Badi
	VD/=====s?(len > reqlen)
				le		cas 1

#ds() {, 4,uoccsreturn Noccs[0](<
1ma_soccs[0]( look forqlen;

			if (fast:rMalt
	mS */
ted in the(len > rqlen;

	nldBPG
CBP!MUope that it will be useful, but
 *  WITHOUT ANY(len > reqlen)
				le	t
	REG1int CONF __infndefCgnat
t PIO_FIm sjj;
> r {
					===]f==soccs[1]ios_basquration 6 roccs[1]; (jj= reqlen;

			if (fast:;SNTY; be fguration 6) biosCBx^cng flags; (jj{
	unures[jint R^cng flags => rqlen;

	nldBPG
CBP!MInonfigue useful ) binsNTY; be  coun
ccs[1]i > reqlen)
				len rn Noccs[0](>a_dmi[jnrn Noccs[2](n)
{
	i[jnssRAY_SIZE(ad0; (jj= reqlen;

			if (fast:;SNTY; be f	RA(nt coun
RAY_SIZE); (jj{
	unures  ADDR	nat
	REG1int CONF 93", 61, 82nat	RAY_SIZE(<
{ sjj;
> rjnrn Noccr===]f==soccs[2]ios_base;sRAY_SIZE(adpccs[2]d *) ad= reqlen;

			if (fast:;SNTY; be f	RA(nt coun^cng flags; (jjjj{
	unureres[jn {
		RAY_SIZE(<
{
and/3len;

			if (fast:;tnonfiguclu ntnsNTY; be  coun
ccs[2])			len rn Noccs[0](>a2
	*_l0, 0x330, occs[3====== reqlen;

			if (fast:;guration =) biun
RA=%d, l0, 0x33=nt coun^cng flagun
RAY_SIZEunos_base );)breaase 01c:
		_jEG1;
	"at it will oungnatures[jjEG1;
)d aiTURE_COUNT ARRAY_SIZEUNT ARRA#dst 0;
			'no irq', so fo() {
		printk("NCR5SCc"NCs[ii]= reqlen;

			if (fastso fo9et ma.(lens	breaase 0(ZE(ports)c:
		234 0ARRAY_SIZE(inwaite
static i) ptr) & 1))
		

		G1injif bes +ine C5_IM====COUNT) gura_, uns((r) if bes)IO_FIFOnb( */
stats jj =eest, :
		wait;*//* Cpseudoimplied war4;
jjcpu	eakax_n > rbarrie#e	caslen rn Ngura_n");

_eq((r) if bes)
	int iTimS *tion4;
jj=====n); (jfine PORT_->f (fine=uxID_TIME__BAS_loc6; (jfine PORT_->SCp.phon 6 rodle; (jfine PORT_->3				done(fine PORT_i > reqlen)caslen 'no irq', so useBIOS);}} 0x310, 0x348, 0;
		default:
		istue_lck(x350 };
#de SCp-1 for 'u(*done) (x350 };
#de)nd interrupt
dV= reqlen;

			if (fast_istueeet ma.(lens	b        = reqlen;

	cmd=%02iuncmd=====%02iuntar	ca=%02iunlu==%02iunbuff====%t counSCp-1->c;
#[0]unSCp-1->c;d====unSCp-1->device->tar	caBxIe8)SCp-1->device->lu=, 
				buff===(SCp-1)))d aif 00
dV= ret
	REG1int CONF SCp-1->c;d==== sjj;
>      qlen;

	cmd[%d]=%02ilaim_iunSCp-1->c;
#[=])); (V= reqlen;

	(lens	b	case = 4fine PORT_1inSCp-1;
jfine PORT_->3				done6 rhone;
jfine PORT_->SCp.phon 6 rort_base =;
jfine PORT_->SCp.Sa_dma_ad0; (fine PORT_->SCp.Mort_baed unalit iWe ende0x8:S *proto3) {
		i for P m 'ulaye{
				0x00; (ine Csc;d=id(SCp-1), */
stat)essessTA dterin x
#else
(ine Cine MSG_AC,o */
sta)essesse	message(fass84;

	t
	REG1int CONF SCp-1->c;d==== sjj;
os_baine CSCp-1->c;
#[=],ic int FIF	casleaine CSne SEND_MSG ,o */
sta)e= 42====1		breaase 0xc:
			len = DEF_c intQ */(		if (fast_istue) 0x348, 0;
		default:
		bios_se	me(x350 };
#de SCp-1s[ii]= reqlen;

			if (fastsse	meset ma.(lens	bR	3qin 0x8:_	RAnSCp-1->device->c"NC->bios_0x8:s	bR	define FLUSH_FIFO    essesS ConfisetusTA 0lse
(ine Cine TRANSF,o */
sta)e=aine CSCe INIT,o */
sta)essessequisi f, unsise	messe
(ine Cine SET_AT,o */
sta)e=ae#endif

#)d a42====2s	bR	3qin un0x8:_	RAnSCp-1->device->c"NC->bios_0x8:s	bR	eaase 0SUCCESSc:
			len = ;
		default:
		d ch===mlength))
				device *disk,>                               ength))b0x8:_device *dev, (jj{
	u   eonfor_esetpacityun
cc *ZE(poils ynd interr)pio===== reqlen;

			if (fast	d ch===mset ma.(lens	bR	3pio =setpacitypt oc(poils y[0](= 64essesh{
	linux/oc(poils y[1](= 32essessTnforlinux/oc(poils y[2](nt_pio N3  ressescy thderlinux/of0(ZE(poils y[2](3  024
	int ibig<asmkstat/ssc(poils y[0](= 255;t/ssc(poils y[1](= 63;t/ssc(poils y[2](nt_pio / (255de 63	casleaeaase 0xc:
			len = ;= -1;	/* 0 is 'no irq', so use -1 unused for 'undev_ i) ptr) & 1))
		

		switch e) {
		printk("NCR5dev6 rhev_ i	bR	3qin 0x8:_	RAsave(hev->bios_0x8:, 			len = 'no irq', so usedev_ i);R	3qin un0x8:_	RAse	fore(hev->bios_0x8:, 			len = eaase 0IO
sHANDLEDc:
			len = = IRQ_LEV;
#else
static intdev_ i) ptr= re			len = 0;
		(fas_)pio==j{
	unur    = re			len = 0;
		seqc(r)==j{
	unur			len = 0;
		sa_dmaun
ccc(r)==		break;
		cr			len = 0;
		len -= lenh e) {
		psc_inlrliNCR53g	b        t, (vxURE;*//* dV= reqlen;

			if (fast_
staset ma.(lens	bR		break;
		crn > 3) len -= len>> 2);
				request +	case =]0x00; (-= len>> 2);
 */
stats;tr= reseqc(r)>> 2);
 tatic 	returccc(r)>> 2);
*statics;tr= re(fas_)pio>> 2);
static intount;
}fs	bR		brid calc_port_addr/3len;

	-= len &02iunseqc(r) &02iunrccc(r) &02iun(fas_)pio &02i*, 3a_dmaunseqc(r)unrccc(r)un(fas_)pio);}

	i(f (coun)r/3len;

	(len >	cADDR	3len;

	, x33=n02i coun^en -= lenn >	case 0xc:
			len = 84;
	case 0xc:
		 NCR53C406A_DEBUG&& leint rccc(r)>	while 	int i
 *  se	mes
stas4;
jj=====3	cas== reqlen;

			if (fast:;se	mes
stasreceiveg(len)d asfine PORT_->SCp.phon 6 rodle; (jfine PORT_->f (fine=uxID_ET_ATN_loc6; (jfine PORT_->3				done(fine PORT_i > reqlen)casle		break;
		cr {
		en -= len>	while 	i> rqlen;

	nldBP3C406: Warning:ude "--) {!(len > rfine PORT_->SCp.phon 6 rodle; (jfine PORT_->f (fine=uxID_ERRORN_loc6; (jfine PORT_->3				done(fine PORT_i > reqlen)casle	void *) signatures[jj].s];lin3= len>	whi2e 	int iParityn--) {
					qlen;

			if (fast:;Warning:uparityn--) {!(len > rfine PORT_->SCp.phon 6 rodle; (jfine PORT_->f (fine=uxID_PAREV  _loc6; (jfine PORT_->3				done(fine PORT_i > reqlen)casles];lin3= len>	whi4e 	int iGrossn--) {
					qlen;

			if (fast:;Warning:ugrossn--) {!(len > rfine PORT_->SCp.phon 6 rodle; (jfine PORT_->f (fine=uxID_ERRORN_loc6; (jfine PORT_->3				done(fine PORT_i > reqlen)casleleint rccc(r)>	whi2e 	int iDisconn========== reqlen;

			if (fast:;disconn====
stasreceiveg(len)d asnlineine PORT_->SCp.phon 6!= port_base; 	int iUnexp===eg;disconn====mmand/fine PORT_->f (fine=uxID_D_Ma
  ESET_loc6; (j < SIGNA
	if (ted inortpletere *ase 03= len>d inport_ba=mmand/fine PORT_->f (fine=uneine PORT_->SCp.S= len>	whiff
	*_j{
	u_setfine PORT_->SCp.Mort_bae	whiff
T_lo8)u_sexID_OKT_loc6);ures[
jj=====n); (jfine PORT_->SCp.phon 6 rodle; (jfine PORT_->3				done(fine PORT_i > reqlen)caslen +, PIO_F3= len>	whi07 	int i		if phon 6mman
#endif	0: signtic -_BAS else {
		ccc(r)>	whi1{
	int iTar	cae */
	fo*/
s		foConfigurat					ou====5s; (jjfine PORT_->SCp.phon 6 rORT_BASE; (jjV= reqlen;

			if (fast:;DRT_-Out phon (len)d *) ine Cine MSG_AC,o */
sta)e *) LECT_ATN_STOP  
				buff===(fine PORT_i)essesMaxConfigurat_pio && 

	if (counxc:
		 o s/gTAupan < 1

/* ==					o(dma_chan);
	release_
				sgliNC(fine PORT_i,>                                             ecsd		buff===(fine PORT_i)e
>	case 0xc:
			len = 84;
jj ine Cine RECV_MSG  |efine T,o */
sta)e 		break;
		c                        ecsi_1

_each	sg(fine PORT_unsg, 
				sgR53c406fine PORT_i, ====>                                default:
		case 0xe:
sgRvirt
sg)unsg->======= >                        signa0x00; 	void *) signatures[jj].res[jn = NULLan
#endif	1: signtic -INS else {
		ccc(r)>	whi1{
	int iTar	cae */
	fo*/
s		foConfigurat					ou====6s; (jjfine PORT_->SCp.phon 6 rORT_Bin; (jjV= reqlen;

			if (fast:;DRT_-In phon (len)d *) ine Cine MSG_AC,o */
sta)e *) LECT_ATN_STOP  
				buff===(fine PORT_i)essesMaxConfigurat_pio && 

	if (counxc:
		 o s/gTAupan < 1

/* ==					o(dma_chan);
	re
		if
				sgliNC(fine PORT_i,>                                            ecsd		buff===(fine PORT_i)e
	case 0xc:
			len = 84;
jj ine Cine RECV_MSG  |efine T,o */
sta)e 		break;
		c                        ecsi_1

_each	sg(fine PORT_unsg, 
				sgR53c406fine PORT_i, ====>                                default:
		case
		if
gRvirt
sg)unsg->======= >                        signa0x00; 	void *) signatures[jj].res[jn = NULLan
#endif	2: signCOMMANDr4;
jjcine PORT_->SCp.phon 6 rort_base =;
j	qlen;

			if (fast:;Warning:uUnknown;		/* current scsi fin*/
ted inphon !(len > r = NULLan
#endif	3: signfollowi4;
jj=====7); (jfine PORT_->SCp.phon 6 rRT_BASE;
; (jV= reqlen;

			if (fast:;S= len>phon (len)d *)ine Cine MSG_AC,o */
sta)e *)ine Cine RECV_CMD     ,o */
sta)e *) = NULLan
#endif	4: sign/* PIO d6mman
#endif	5: sign/* PIO d6mman	qlen;

			if (fast:;WARNING:n/* PIO d6phon !!!(len > r = NULLan
#endif	6: signMESSAGE-_BAS else= reqlen;

			if (fast:;Mort_ba-Out phon (len)d *)fine PORT_->SCp.phon 6 rse
static ie *)ine C-------,o */
sta)essesRejonfig P mort_ba=mmandine Cine RECV_D,o */
sta)e *) = NULLan
#endif	7: signMESSAGE-INS else=====4); (jV= reqlen;

			if (fast:;Mort_ba-In phon (len)d *)fine PORT_->SCp.phon 6 rse
statiiii, jjfine PORT_->SCp.Sa_dma_ad2);
  int FIF	cas(fine PORT_->SCp.Mort_baed 2);
  int FIF	ca (jV= reqlen;

	
 *   int )pio &  coun
c;
static intount;
}fs	cas== reqlen;

	Sa_dma_ad%02ilaMort_baed n02i counfine PORT_->SCp.Sa_dmaunfine PORT_->SCp.Mort_bans	bR	snlineine PORT_->SCp.Mort_baed= SAVureOINTERSma_seine PORT_->SCp.Mort_baed= enta
  ESEios_basine C-------,o */
sta)essesRejonfiport_ba=mmand/= reqlen;

	Discard*/
sSAVureOINTERSmport_ba(len)d *)}andine Cine RECV_D,o */
sta)e *) = NULL)}a} aif USE_PIO
static int fast_pio = USE_FAST_d interrurqsun
RA;tr) & 1))
		

		Gsi=];);
*statics; JumpereaEBUG 0		/* curreicates # nux/orqs6 r	 USE_	RA_on#)d a4
#denonfigu/
ted in_ph,
ca_BIOan;		/* curre				0x00; (ine C   0x4 */
sta)e= 4t iWait;*//*UG 0		/* curretoent sc nux/o1injif bes +ine C5_IM===COUNT) gura_, uns((r) if bes)IO_FIFOnb( */
stats jj =80 Confbarrie#e	casrn Ngura_n");

_eq((r) if bes)
	int iTimS *tio, musts */hardmendetrouonghmmand	 USE_	RA_off		RAsi > reqlen)
-1casleleiRA(ad	 USE_	RA_off		RAsi > 4t iKick*UG 0e#enlse
(ine Cine TRANSF,o */
sta)e=aine CSCe INIT,o */
sta)e=ae#endif

#)d a42qlen)

RA;tle	void *) signclude <lSIZEUNT ARRd' */
#endif

#if USd inn > 3)

	if (count	ine C      				request +	c36;
		lim3atures[jj].rine C   1  				request +	case =]ine C      				c ins	bR	define FLUSH_FIFO    esses0x00;lse
(ine Ci3FLUSH_FIFO  3)e=aine COUNT(cH_FIFO  2)e=aine CO1NT(cH_FIFO  1s	bR	definif	5,ENSE;*// esseser sense port regfanforjj].rine C  9_un
static  essesS Conf reggurationj].rine C   5,Estatic  essesSFIG4;		/* Configuration 4 Rse
(ine CiSIE POL ,tic int )essessFIG4;		/* CG, CONFI
			len = = IRQint) sint dma_chan;
#endif
d inreselect timeout reg  */
stat	REG;		e=unguration 6orif	n); (CPRD;	e=unguration 6orif	1		br  int FIFe=unguration 6orif	2)e=a */
stae=unguration 6orif	3)e=a */
state=unguration 6orif	4s;tr= 
state=unguration 6orif	4s;tr*statice=unguration 6orif	5s; (
static e=unguration 6orif	5s; (
tatic e=unguration 6orif	6s; (static e=unguration 6orif	6s; (static inte=unguration 6orif	7s; (statnt e=unguration 6orif	7s; (FIFO  1e=unguration 6orif	8s; (FSE;*//e=unguration 6orif	9s; (t iTt SRAM          =unguration +if	A		} elsFIFO  2e=unguration 6orif	Bs; (FIFO  3e=unguration 6orif	Cs; (FIFO  4e=unguration 6orif	D); (CPR*//*e=unguration 6orif	Es; (t iint PIO_STA      =unguration +if	F		} elinreselect timeout reg  */1r msb */w) */
				/*     =unguration +if	0		} els CONFIG*/
s         =unguration +if	1		} els CONFIG*(i),0x3c0);}=unguration +if	2		} elsmcmp((voe=unguration 6orif	4s;trags = /
				0x3c0);}=unguration +if	5		} els COonfigurat0x3c0);}=unguration +if	6		} els COonfigura30x3c0);}=unguration +if	7		} elsmcmpfollowi=unguration 6orif	8s; (ount,  {inb(0x3da);o=unguration +if	9		} els COgned in          =unguration +if	A		} els				c ine=unguration 6orif	Bs; (FIFO  5e=unguration 6orif	D); ( CON	panic("        =unguration +if	E		} els COlse {
	          =unguration +if	F		} el
		 ARRAY_LIC		/*("GPLunsig
		 OTE: psc_inlr-ga DMA Aupan < only worksnclude "scsi.ck.hUn 6ne NONE
#includG, COisk("dma:d!
lSIZEUNT ARRength))
				bios_base = adFree S_base = ad=
=>     .fine nam 666666666	 rCONFIG6;*//*	lim3fine nam 6*/,66666666>     .nam 66666666666666	 rCONFIG6;*//*	lim3nam 6*/,6666666666666>     .=======66666666666	 rgnatures[jj].sig_lim3=========,66666666666>     .eak;
		=66666666666	 rgnatures[jjeak;
		,>     .		foC6666666666666	 r		if (fastso foTC_HIGHigh *,6666666666666>     .istue/
ted in66666	 r		if (fastsistueC_HIistue/
ted in *,66666>     .eh	bios_se	me_hd il	/*stat=	default:
		bios_se	messesse	mes *,666666666666>     .d cha=====66666666	 r		if (fastsd ch===mnt ibich===ms *,666666666>     .can_istuee66666666	 r1		lim3can_istuee*/,66666666>     .<asm_id66666666666	 r7		lim3
 *  atene VERBe#enlse,>     .sg_tdma:_pio 66666	 r32		limne RLL*/ imne NONE*/,6>     .uIG4e8:S _is);
	r6	 r1		lim3uIG4e8:S _is);
	r6se,>     ._BI_clut re*/
s666	 rine DISCLUSTERING,
#endDMA
#define C3_DOG 500c"ig
	ck.hOstaCBxC
#endiEmacs USEtoprowe		ca a3uIit
	m tdmb*/
sstyli.ck.hEmacs _ph,
notice <asm/stuff > 3VERBe======sage(fle>d inu	=