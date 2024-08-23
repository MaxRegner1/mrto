/*
 * Driver core for Samsung SoC onboard UARTs.
 *
 * Ben Dooks, Copyright (c) 2003-2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* Hote on 2410 error handling
 *
 * The s3c2410 manual has a love/hate affair with the contents of the
 * UERSTAT register in the UART blocks, and keeps marking some of the
 * error bits as reserved. Having checked with the s3c2410x01,
 * it copes with BREAKs properly, so I am happy to ignore the RESERVED
 * feature from the latter versions of the manual.
 *
 * If it becomes aparrent that latter versions of the 2410 remove these
 * bits, then action will have to be taken to differentiate the versions
 * and change the policy on BREAK
 *
 * BJD, 04-Nov-2004
*/

#if defined(CONFIG_SERIAL_SAMSUNG_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_s3c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/of.h>

#include <asm/irq.h>

#include "samsung.h"

#if	defined(CONFIG_SERIAL_SAMSUNG_DEBUG) &&	\
	!defined(MODULE)

extern void printascii(const char *);

__printf(1, 2)
static void dbg(const char *fmt, ...)
{
	va_list va;
	char buff[256];

	va_start(va, fmt);
	vscnprintf(buff, sizeof(buff), fmt, va);
	va_end(va);

	printascii(buff);
}

#else
#define dbg(fmt, ...) do { if (0) no_printk(fmt, ##__VA_ARGS__); } while (0)
#endif

/* UART name and device definitions */

#define S3C24XX_SERIAL_NAME	"ttySAC"
#define S3C24XX_SERIAL_MAJOR	204
#define S3C24XX_SERIAL_MINOR	64

#define S3C24XX_TX_PIO			1
#define S3C24XX_TX_DMA			2
#define S3C24XX_RX_PIO			1
#define S3C24XX_RX_DMA			2
/* macros to change one thing to another */

#define tx_enabled(port) ((port)->unused[0])
#define rx_enabled(port) ((port)->unused[1])

/* flag to ignore all characters coming in */
#define RXSTAT_DUMMY_READ (0x10000000)

static inline struct s3c24xx_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct s3c24xx_uart_port, port);
}

/* translate a port to the device name */

static inline const char *s3c24xx_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static int s3c24xx_serial_txempty_nofifo(struct uart_port *port)
{
	return rd_regl(port, S3C2410_UTRSTAT) & S3C2410_UTRSTAT_TXE;
}

/*
 * s3c64xx and later SoC's include the interrupt mask and status registers in
 * the controller itself, unlike the s3c24xx SoC's which have these registers
 * in the interrupt controller. Check if the port type is s3c64xx or higher.
 */
static int s3c24xx_serial_has_interrupt_mask(struct uart_port *port)
{
	return to_ourport(port)->info->type == PORT_S3C6400;
}

static void s3c24xx_serial_rx_enable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon, ufcon;
	int count = 10000;

	spin_lock_irqsave(&port->lock, flags);

	while (--count && !s3c24xx_serial_txempty_nofifo(port))
		udelay(100);

	ufcon = rd_regl(port, S3C2410_UFCON);
	ufcon |= S3C2410_UFCON_RESETRX;
	wr_regl(port, S3C2410_UFCON, ufcon);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon |= S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	rx_enabled(port) = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c24xx_serial_rx_disable(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	rx_enabled(port) = 0;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c24xx_serial_stop_tx(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);
	struct s3c24xx_uart_dma *dma = ourport->dma;
	struct circ_buf *xmit = &port->state->xmit;
	struct dma_tx_state state;
	int count;

	if (!tx_enabled(port))
		return;

	if (s3c24xx_serial_has_interrupt_mask(port))
		s3c24xx_set_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);
	else
		disable_irq_nosync(ourport->tx_irq);

	if (dma && dma->tx_chan && ourport->tx_in_progress == S3C24XX_TX_DMA) {
		dmaengine_pause(dma->tx_chan);
		dmaengine_tx_status(dma->tx_chan, dma->tx_cookie, &state);
		dmaengine_terminate_all(dma->tx_chan);
		dma_sync_single_for_cpu(ourport->port.dev,
			dma->tx_transfer_addr, dma->tx_size, DMA_TO_DEVICE);
		async_tx_ack(dma->tx_desc);
		count = dma->tx_bytes_requested - state.residue;
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		port->icount.tx += count;
	}

	tx_enabled(port) = 0;
	ourport->tx_in_progress = 0;

	if (port->flags & UPF_CONS_FLOW)
		s3c24xx_serial_rx_enable(port);

	ourport->tx_mode = 0;
}

static void s3c24xx_serial_start_next_tx(struct s3c24xx_uart_port *ourport);

static void s3c24xx_serial_tx_dma_complete(void *args)
{
	struct s3c24xx_uart_port *ourport = args;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct s3c24xx_uart_dma *dma = ourport->dma;
	struct dma_tx_state state;
	unsigned long flags;
	int count;


	dmaengine_tx_status(dma->tx_chan, dma->tx_cookie, &state);
	count = dma->tx_bytes_requested - state.residue;
	async_tx_ack(dma->tx_desc);

	dma_sync_single_for_cpu(ourport->port.dev, dma->tx_transfer_addr,
				dma->tx_size, DMA_TO_DEVICE);

	spin_lock_irqsave(&port->lock, flags);

	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	port->icount.tx += count;
	ourport->tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	s3c24xx_serial_start_next_tx(ourport);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void enable_tx_dma(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	u32 ucon;

	/* Mask Tx interrupt */
	if (s3c24xx_serial_has_interrupt_mask(port))
		s3c24xx_set_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);
	else
		disable_irq_nosync(ourport->tx_irq);

	/* Enable tx dma mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TXBURST_MASK | S3C64XX_UCON_TXMODE_MASK);
	ucon |= (dma_get_cache_alignment() >= 16) ?
		S3C64XX_UCON_TXBURST_16 : S3C64XX_UCON_TXBURST_1;
	ucon |= S3C64XX_UCON_TXMODE_DMA;
	wr_regl(port,  S3C2410_UCON, ucon);

	ourport->tx_mode = S3C24XX_TX_DMA;
}

static void enable_tx_pio(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	u32 ucon, ufcon;

	/* Set ufcon txtrig */
	ourport->tx_in_progress = S3C24XX_TX_PIO;
	ufcon = rd_regl(port, S3C2410_UFCON);
	wr_regl(port,  S3C2410_UFCON, ufcon);

	/* Enable tx pio mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TXMODE_MASK);
	ucon |= S3C64XX_UCON_TXMODE_CPU;
	wr_regl(port,  S3C2410_UCON, ucon);

	/* Unmask Tx interrupt */
	if (s3c24xx_serial_has_interrupt_mask(port))
		s3c24xx_clear_bit(port, S3C64XX_UINTM_TXD,
				  S3C64XX_UINTM);
	else
		enable_irq(ourport->tx_irq);

	ourport->tx_mode = S3C24XX_TX_PIO;
}

static void s3c24xx_serial_start_tx_pio(struct s3c24xx_uart_port *ourport)
{
	if (ourport->tx_mode != S3C24XX_TX_PIO)
		enable_tx_pio(ourport);
}

static int s3c24xx_serial_start_tx_dma(struct s3c24xx_uart_port *ourport,
				      unsigned int count)
{
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct s3c24xx_uart_dma *dma = ourport->dma;


	if (ourport->tx_mode != S3C24XX_TX_DMA)
		enable_tx_dma(ourport);

	dma->tx_size = count & ~(dma_get_cache_alignment() - 1);
	dma->tx_transfer_addr = dma->tx_addr + xmit->tail;

	dma_sync_single_for_device(ourport->port.dev, dma->tx_transfer_addr,
				dma->tx_size, DMA_TO_DEVICE);

	dma->tx_desc = dmaengine_prep_slave_single(dma->tx_chan,
				dma->tx_transfer_addr, dma->tx_size,
				DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!dma->tx_desc) {
		dev_err(ourport->port.dev, "Unable to get desc for Tx\n");
		return -EIO;
	}

	dma->tx_desc->callback = s3c24xx_serial_tx_dma_complete;
	dma->tx_desc->callback_param = ourport;
	dma->tx_bytes_requested = dma->tx_size;

	ourport->tx_in_progress = S3C24XX_TX_DMA;
	dma->tx_cookie = dmaengine_submit(dma->tx_desc);
	dma_async_issue_pending(dma->tx_chan);
	return 0;
}

static void s3c24xx_serial_start_next_tx(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long count;

	/* Get data size up to the end of buffer */
	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (!count) {
		s3c24xx_serial_stop_tx(port);
		return;
	}

	if (!ourport->dma || !ourport->dma->tx_chan ||
	    count < ourport->min_dma_size ||
	    xmit->tail & (dma_get_cache_alignment() - 1))
		s3c24xx_serial_start_tx_pio(ourport);
	else
		s3c24xx_serial_start_tx_dma(ourport, count);
}

static void s3c24xx_serial_start_tx(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);
	struct circ_buf *xmit = &port->state->xmit;

	if (!tx_enabled(port)) {
		if (port->flags & UPF_CONS_FLOW)
			s3c24xx_serial_rx_disable(port);

		tx_enabled(port) = 1;
		if (!ourport->dma || !ourport->dma->tx_chan)
			s3c24xx_serial_start_tx_pio(ourport);
	}

	if (ourport->dma && ourport->dma->tx_chan) {
		if (!uart_circ_empty(xmit) && !ourport->tx_in_progress)
			s3c24xx_serial_start_next_tx(ourport);
	}
}

static void s3c24xx_uart_copy_rx_to_tty(struct s3c24xx_uart_port *ourport,
		struct tty_port *tty, int count)
{
	struct s3c24xx_uart_dma *dma = ourport->dma;
	int copied;

	if (!count)
		return;

	dma_sync_single_for_cpu(ourport->port.dev, dma->rx_addr,
				dma->rx_size, DMA_FROM_DEVICE);

	ourport->port.icount.rx += count;
	if (!tty) {
		dev_err(ourport->port.dev, "No tty port\n");
		return;
	}
	copied = tty_insert_flip_string(tty,
			((unsigned char *)(ourport->dma->rx_buf)), count);
	if (copied != count) {
		WARN_ON(1);
		dev_err(ourport->port.dev, "RxData copy to tty layer failed\n");
	}
}

static void s3c24xx_serial_stop_rx(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport = to_ourport(port);
	struct s3c24xx_uart_dma *dma = ourport->dma;
	struct tty_port *t = &port->state->port;
	struct dma_tx_state state;
	enum dma_status dma_status;
	unsigned int received;

	if (rx_enabled(port)) {
		dbg("s3c24xx_serial_stop_rx: port=%p\n", port);
		if (s3c24xx_serial_has_interrupt_mask(port))
			s3c24xx_set_bit(port, S3C64XX_UINTM_RXD,
					S3C64XX_UINTM);
		else
			disable_irq_nosync(ourport->rx_irq);
		rx_enabled(port) = 0;
	}
	if (dma && dma->rx_chan) {
		dmaengine_pause(dma->tx_chan);
		dma_status = dmaengine_tx_status(dma->rx_chan,
				dma->rx_cookie, &state);
		if (dma_status == DMA_IN_PROGRESS ||
			dma_status == DMA_PAUSED) {
			received = dma->rx_bytes_requested - state.residue;
			dmaengine_terminate_all(dma->rx_chan);
			s3c24xx_uart_copy_rx_to_tty(ourport, t, received);
		}
	}
}

static inline struct s3c24xx_uart_info
	*s3c24xx_port_to_info(struct uart_port *port)
{
	return to_ourport(port)->info;
}

static inline struct s3c2410_uartcfg
	*s3c24xx_port_to_cfg(struct uart_port *port)
{
	struct s3c24xx_uart_port *ourport;

	if (port->dev == NULL)
		return NULL;

	ourport = container_of(port, struct s3c24xx_uart_port, port);
	return ourport->cfg;
}

static int s3c24xx_serial_rx_fifocnt(struct s3c24xx_uart_port *ourport,
				     unsigned long ufstat)
{
	struct s3c24xx_uart_info *info = ourport->info;

	if (ufstat & info->rx_fifofull)
		return ourport->port.fifosize;

	return (ufstat & info->rx_fifomask) >> info->rx_fifoshift;
}

static void s3c64xx_start_rx_dma(struct s3c24xx_uart_port *ourport);
static void s3c24xx_serial_rx_dma_complete(void *args)
{
	struct s3c24xx_uart_port *ourport = args;
	struct uart_port *port = &ourport->port;

	struct s3c24xx_uart_dma *dma = ourport->dma;
	struct tty_port *t = &port->state->port;
	struct tty_struct *tty = tty_port_tty_get(&ourport->port.state->port);

	struct dma_tx_state state;
	unsigned long flags;
	int received;

	dmaengine_tx_status(dma->rx_chan,  dma->rx_cookie, &state);
	received  = dma->rx_bytes_requested - state.residue;
	async_tx_ack(dma->rx_desc);

	spin_lock_irqsave(&port->lock, flags);

	if (received)
		s3c24xx_uart_copy_rx_to_tty(ourport, t, received);

	if (tty) {
		tty_flip_buffer_push(t);
		tty_kref_put(tty);
	}

	s3c64xx_start_rx_dma(ourport);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c64xx_start_rx_dma(struct s3c24xx_uart_port *ourport)
{
	struct s3c24xx_uart_dma *dma = ourport->dma;

	dma_sync_single_for_device(ourport->port.dev, dma->rx_addr,
				dma->rx_size, DMA_FROM_DEVICE);

	dma->rx_desc = dmaengine_prep_slave_single(dma->rx_chan,
				dma->rx_addr, dma->rx_size, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT);
	if (!dma->rx_desc) {
		dev_err(ourport->port.dev, "Unable to get desc for Rx\n");
		return;
	}

	dma->rx_desc->callback = s3c24xx_serial_rx_dma_complete;
	dma->rx_desc->callback_param = ourport;
	dma->rx_bytes_requested = dma->rx_size;

	dma->rx_cookie = dmaengine_submit(dma->rx_desc);
	dma_async_issue_pending(dma->rx_chan);
}

/* ? - where has parity gone?? */
#define S3C2410_UERSTAT_PARITY (0x1000)

static void enable_rx_dma(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ucon;

	/* set Rx mode to DMA mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_RXBURST_MASK |
			S3C64XX_UCON_TIMEOUT_MASK |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_DMASUS_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_MASK);
	ucon |= S3C64XX_UCON_RXBURST_16 |
			0xf << S3C64XX_UCON_TIMEOUT_SHIFT |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_DMA;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_mode = S3C24XX_RX_DMA;
}

static void enable_rx_pio(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ucon;

	/* set Rx mode to DMA mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TIMEOUT_MASK |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_DMASUS_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_MASK);
	ucon |= 0xf << S3C64XX_UCON_TIMEOUT_SHIFT |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_CPU;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_mode = S3C24XX_RX_PIO;
}

static void s3c24xx_serial_rx_drain_fifo(struct s3c24xx_uart_port *ourport);

static irqreturn_t s3c24xx_serial_rx_chars_dma(void *dev_id)
{
	unsigned int utrstat, ufstat, received;
	struct s3c24xx_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct s3c24xx_uart_dma *dma = ourport->dma;
	struct tty_struct *tty = tty_port_tty_get(&ourport->port.state->port);
	struct tty_port *t = &port->state->port;
	unsigned long flags;
	struct dma_tx_state state;

	utrstat = rd_regl(port, S3C2410_UTRSTAT);
	ufstat = rd_regl(port, S3C2410_UFSTAT);

	spin_lock_irqsave(&port->lock, flags);

	if (!(utrstat & S3C2410_UTRSTAT_TIMEOUT)) {
		s3c64xx_start_rx_dma(ourport);
		if (ourport->rx_mode == S3C24XX_RX_PIO)
			enable_rx_dma(ourport);
		goto finish;
	}

	if (ourport->rx_mode == S3C24XX_RX_DMA) {
		dmaengine_pause(dma->rx_chan);
		dmaengine_tx_status(dma->rx_chan, dma->rx_cookie, &state);
		dmaengine_terminate_all(dma->rx_chan);
		received = dma->rx_bytes_requested - state.residue;
		s3c24xx_uart_copy_rx_to_tty(ourport, t, received);

		enable_rx_pio(ourport);
	}

	s3c24xx_serial_rx_drain_fifo(ourport);

	if (tty) {
		tty_flip_buffer_push(t);
		tty_kref_put(tty);
	}

	wr_regl(port, S3C2410_UTRSTAT, S3C2410_UTRSTAT_TIMEOUT);

finish:
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static void s3c24xx_serial_rx_drain_fifo(struct s3c24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ufcon, ch, flag, ufstat, uerstat;
	unsigned int fifocnt = 0;
	int max_count = port->fifosize;

	while (max_count-- > 0) {
		/*
		 * Receive all characters known to be in FIFO
		 * before reading FIFO level again
		 */
		if (fifocnt == 0) {
			ufstat = rd_regl(port, S3C2410_UFSTAT);
			fifocnt = s3c24xx_serial_rx_fifocnt(ourport, ufstat);
			if (fifocnt == 0)
				break;
		}
		fifocnt--;

		uerstat = rd_regl(port, S3C2410_UERSTAT);
		ch = rd_regb(port, S3C2410_URXH);

		if (port->flags & UPF_CONS_FLOW) {
			int txe = s3c24xx_serial_txempty_nofifo(port);

			if (rx_enabled(port)) {
				if (!txe) {
					rx_enabled(port) = 0;
					continue;
				}
			} else {
				if (txe) {
					ufcon = rd_regl(port, S3C2410_UFCON);
					ufcon |= S3C2410_UFCON_RESETRX;
					wr_regl(port, S3C2410_UFCON, ufcon);
					rx_enabled(port) = 1;
					return;
				}
				continue;
			}
		}

		/* insert the character into the buffer */

		/CGx		rx_enabl.TGx0bufWB	}
		X_cpu(ourport->port.dev,
			dma->tx_transfer_addr, dma->tx_size, DMA_TO_DEVICE);
		async_tx_ack(dman_lock_irqsaveCGyode = S3C24XX_RX_DM dma-aGyTGyTGyTGyport)) {
				if (!txe) _z_mSz_b>tail + count) & (Rx\n");
		retuvUfer */

		/CGx		rx_enabl.Tack = s3c24xx_vyTGyXaGyTGb
#define SUPPORT_SYSRQGyiASK);
	ucon |AtortW:dt_nq_pa->rx__m	}
			

/* ?UERSTAT);
	ess = 0;sn");
		redSUPPoshift;
}

static void s3c64xx_start_rx0lON_D_enariiegl(port,
static inPart_rx0lON_D_enariiegl(port,
AT_DUM=x16 : S3
	esframe 0;sn")ic inPart_rx0lON_D_enariiegl(pOV>calNAT_DUM=x16 : S3
	esov->rxnXX_RX_Dint txe =&IFO leveial_sted - rq);
;
static inPart_rx0lON_D_enariiegl(pAtortWT_DUM_ack(dman_lAtort;sn") S3C2ic inPart_rx0lON_D_enariiegl(p;
	unsWT_DUM_ack(dman_l;
	uns;sn") S3C2ic inPart_rx0l(ON_D_enariiegl(port,
3c24xxCGx		rON_D_enariiegl(pOV>calNAWT_DUM_ack(dman_lort,
;ransfer_
		redSUPPoshiftnclud uartacter ichAWT_DU3c64xx_start_rx0lON_D_enariiegl(p
ic voidv_err(ouartacter inPart_rl_txempty_nofifo(pOV>calNreturn 		 * Rec	return in_unlock_irqrestore(t = rd_regl(port, uart_port *port *ourport = dev_id;
	struct uart_p}

	*port = &ourport-rport->dma;
	struct tty_struct *tty = tty_port_tty_get(&ourport->port.state->port);
	st2410_UFSTAT);

	spin_		if (ourport->rx_mode == S3C24XX_RX_PIO)S3C2410_UTRSTAT, S3C2410_UTRSTAT_TIMEOUTc24xx_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
port *port *ourport = dev_id;
	struct uart_(			crt , *port = &ourport-rport->dma;
	struct tty_struct *tty = tty_poal_start_next_tx(ourport);
	}
}

stary(xmit) &&port = = dev_id;
	struct uart_port = &ourpg ufstat)
= dev_id;
	struct uart_p}

	= &ourpg t_port *port *ourport = dev_id;
	strutt uart_(			crt , *port urport-rport->dma;
	struct tty_struct *tty y_port_tty_get(&ourport->port.state->port);
	struct ttNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

e.residue;
	async;
		sading FIFfcon = rd_regl(port, S3C2410_UFCON);
	ufcon |=n;
	}

	if (!ourport->dma || !ourport->dma->tx_chan ||
	    count < ourrt_next_tx(ourport);
	}
}

static voiurpourport);
	e>=se
		s3c24xx_serial_std s3c24
	e voidt);
	};
}

static void s3c24xx, "Rxa(ourport, count);
}

static void s3c24xx_serXX_RX_DMocnt = voidt>=se
		s3c24xx_serial_std s3c2			sading FIFocnt = void;T_DU3cng FIF void;T_D}->rx_cooki	s3c24t uartd s3c2he chax_enabled(port)) TXH, 	s3c24t uartdqsaveCGyode = S3Ctx 0;sn"	s3c24t uartFCON_RESrx_choue(port);addr to_orrt)-n'oundyto ignm the oturn t(ou,ype VICEet(&t)->STA
s3c2				peporan);
		 VICEet(&tt coexit
	l(p
ipin_unlock_irqx_to_tty(str||get(&oueste		pepl(port, S3C2a_size ||
	    xmit->tail & (dmarx_choue(port);addtry_MINOR2410EVICE);
		a... l(p
ipin_t);
	e>FO level again
, S3C2ding FIFO level again
		2			sading FIF0return I
		if uart_copy_rx_to_tty(structt);
	e>F0d s3c24xx_		}
		fifocnt--;

		uerstat = & >> info->rx_ftatic3c64xx_startlags & U3c2he chax_enabled(port)) TXH, a(ourp);
[a(ourport,] (dma(uart_circ_chars_pending(xm1t->flags & UPF_CONS_FLOW)
		s3c24xx_serial_ 0;sn"ocnt ==a->rx_cooki!UFCON_RES		sading , S3C2a_size ||
	    xmourport = to_ourport		sading ,(dmarx_choue(port);pin_unlock_irqrestore(&port->lock, flags);
}

sta S3C2a4xx_uart_p3C2410_UFCON,(dma void enable_tx_dma(struc	 = rd_regl3C2410_UFCON,(dmrt);pin_unlock_irqx_to_tty(strial_start_tx(struct it->tail & (d
oues3c24xx_uart_port *ourport)
{
	struct uart_port*port = &ourport->port;
addr,M);
	elsPoshifrGyXaGSoC's which have these .3C6400;
}

t *ourport = d6irq_nosync(oushiftt_po			crt , *port urport-rport->dma;
	struct tty_struct *tty y_port_tty_get(&ourport->port.state->port);
	stknown to be ie(&penabled(port) = 1;
	engine_paPack =t *ourport *ouena&ourport->por_cooki	(&pe&dma && dma->rx_cha_MStW:dt_n*ouena= dev_id;
	struct uart_(	t , urpg uthe character intoengine_paP(dma && dma->rx_cha_MStW;a->rx_coo	(&pe&dma && dma->rx_Tha_MStW:dt_n*ouena= dev_id;
	strutt uart_(	t , urpg uthe character intoengine_paP(dma && dma->rx_Tha_MStW;a->rx and staeid *args)
{
	known to be i= dev_id;
	strutt x_to_tner_of(port, struct s3c24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc	ort->port.fifosize;
k;
		}
		fifocnt--;

		uerstat = rdort->port.fifosirx_enabled(port) = 1;
					return;
	oshift;
N |
	inue;
			}
		}
ifocn_fid s3c24xx_t);
static void t3c24xx_seria!IF0->rx_crport);
static void t3c24xxxx_sttartlrport->por &&port = 1(dmrt);fstat)
= dev_id;
	strulse {
				if (txe) {
	t;
addnwr_regmort *por rt->sS3C6400;
}
known to be i= dev_id;
	stru
}

mctrld_regl(port, S3C2410_UCON);
	ucon &=  0) {mze;
k;
		}
		x_enabled(port)) Mport);
		hift;mrt_rx0lON_D_enarMport_CTS) &&port = TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;struct s3port = TIOCM_CAR | TIOCM_DSR;
	struct circ_buf *xmit = &port-}

mctrld_regl(port, S3C2410_UC,
known to be imctrlON);
	ucon &=  0) {mrx_enabled(port) = 1;
					reMurn;
	oshiftmctrlx0lTIOCM_RTS) &&{mrx_entinue;
			}MurM_RTS_LOW;struct s3{mrx_eock, flags);
MurM_RTS_LOW;sn IRQ_HANDLED;
}

static Mffer *m

		/C	struct circ_buf *xmit = &portags &_ctld_regl(port, S3C2410_UC,
 0) ags &_py_rx_N);
	ucon &= ~S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	rx_enabled(port) = 0;
	spin_unlock_irqrestore(&poshiftags &_py_rx_N	rt->lock, flags);
}

sSAtort;snruct s3{t->lock, flags);
}

sSAtort;sruct s3c24xx_uart_port *ourport);

statruct s3c24xx_uart_dma *dma = ourport->dma;

	dma_sync_nfo *info = ourport-erial_ to DMA mode */
	ucon = rd_reglp>rx_size, DMA_FROM_DEVICE);
	->statep DMA_FROM_DEaeid );addDefaulDMA
		dort figurRSTAT issuee410_Ul(po		s3c24xx_nf.direif de		=n");
		return;
;po		s3c24xx_nf.sirqto g_width	=n");
SLAVE_BUSWIDTH_1_BYTE;po		s3c24xx_nf.sirqto g		=npat = rdmapbase(xmd(port)) {
	;po		s3c24xx_nf.sirqmaxburst		}
		24xx_serial_nf.direif de		=n");
turn -EIO;;24xx_serial_nf.dl_ to g_width	=n");
SLAVE_BUSWIDTH_1_BYTE;po		s3crial_nf.dl_ to g		=npat = rdmapbase(xmd(port)) T
	;po		s3crial_nf.dl_ maxburst		}
		24xx_sery(xmitt);
	};-erial_ xmit(p->rx_desc->carx"(&poshiftIS_ERR);

		enable_r) &&port = PTR_ERR);

		enable_rsync_tx_ack(dm{
		dert figstate.residue;
&		s3c24xx_nfV, DMA_PREP_Ixmitt);
	};-erial_ xmit(p->rx_desc->catx"(&pshiftIS_ERR);

		ESS ||
	W:dt_n*ouenaPTR_ERR);

		ESS ||
			dmrx_ch);
;-eleaseuct;equested _ack(dm{
		dert figstate.tesidue;
&		s3ct4xx_nfV, DMaddRXrt);
		retur has parity enaPAGE_CONS		24xx_sery(_END= kmalurp( has parity , GFP_KERNELack = s3c24xx_ser	}
}:dt_n*ouena-ENOn;
;pomrx_ch);
;-eleaseutt;equested able to gt);
	};mapev_err(op
				dma->rx_addr, d	}
rx_size, DMA_DEV_TO_MEM,
				DMA_PREP_ue;
			dminux/pl_e RESop
				dma->rx_addr, dto g)}:dt_n*ouena-Es_reqmrx_ch);
;s
 *uct;equestaddTXrt);
		retur has te to gt);
	};mapev_err(op
				dma->rxpat = rd_regl(pty(s.	}
rx_sizags & UPF_CONS>tail + count) & (Ue;
			dminux/pl_e RESop
				dma->rx_addrt dto g)}:dt_n*ouena-Es_reqmrx_ch);
;unmapect;equestrport->por );
;unmapect:tatic unmapev_err(op
				dma->rx_addr, dto get desc for Rx\	/CGxMEM,
				DMA_PREP_);
;s
 *uct:taks
 *(24xx_ser	}
}P_);
;-eleaseutt:tatic -eleaseuidueneasync_tx_ack(dma-);
;-eleaseuct:tatic -eleaseuidueneasync_tenable_rx_p and staeid *args)
{
	ed int ufcon, ch, flaeleaseuto DMA mode */
	ucon = rd_reglp>rx_size, DMA_FROM_DEVICE);
	->statep DMA_FR(Ue;
			de);
		if (dma_status == D, t, received);

		enable_rx_pitic unmapev_err(op
				dma->rx_addr, dto gex_size, DMA_DEV_TO_MEM,
				DMA_PREP_uaks
 *(24xx_ser	}
}P_	atic -eleaseuidueneasync_tenable_rx_p4xx_sery(xmitt);_rx_fimrt);pin_ync_tx_ack(dmdma_status == D, t, received);

		ESS ||
			dma_stunmapev_err(op
				dma->rx_addrt dto gex_sizags & UPF_CONS>tail + count) & (Uatic -eleaseuidueneasync_tx_ack(dma-	MA_PREP_Ixmitt);_rx_fimrt	struct circ_buf *xmit = &port-hutdownt->dma;
	struct tty_port *t = &port->state->port;
	struct dma_tx_state state;
	enu < ourrt_next_tP_Ixlaimedd s3c24xx_uble_irq_nosync(ourport->rx_irq);
		rx_enables
 *ut_port *ourport)
{
, status(dma->

	ourport->tx_mode = 0	rt_next_tP_Ixlaimedode = 0	rt_next_tP_I_dma_compl>rx_cookie, &state);
xlaimedd s3c24xx_uble_irq_nosync(ourport->rx_irq);
		rx_enables
 *ut_port *ourport)
{
, status(dma->e, &state);
xlaimedode = 0	(dma->rx_chan,
				dma->rstaddCX_TX e(&portdr,M);
	elshich erruptlldr,M);
	elshe
		enable_irq(ourport->tx_irq);

	ourport->tx_m s3c2s
 *ut_pos3c24xx{
, status(dma uthe character intoengine_paP(d0xfpg uthe character intoengine_paM(d0xfpg urial_start_next_tx(oart_port *ourport);
eleaseuto Dstatus(dma urial_start_next_tx(ourport);
t_port *port = &ourport->port;
	stupt->dma;
	struct tty_port *t = &port->state->port;
	struct dma_tx_state state;
	enumM_DEaeid );S3C64XX_UINTM_RXD,
				stup64XX_UINT (%08llx,%p)/

		/rporcter i(	ucon &= ~S3C2~S3C)	s3c24xapbase, 	s3c24membase)art_port *port)
{
	unsigne
n*ouena-erial_ t_port *ourport)
{
, = dev_id;
	struct uart_(d0		/CGx	ial_txempty_nofifo(struc
{
	u, status(dma ut(tty)ta!IF0rx_dma_completnext_txc->cacuenotram =
{
 %dM);
	 dmaengine_tx_status(and staeid uriale, &state);
xlaimedode1d );S3C64-erial_ ignox=
{
...
			

);

	ourport->tx_modegne
n*ouena-erial_ t_port *ourpott)
{
, = dev_id;
	strutt uart_(d0		/CGx	ial_txempty_nofifo(struc
{
	u, status(dma ut(tty)trx_dma_completnext_txc->cacuenotram =
{
 %dM);
	 dmaengint_tx_statusrx_ch);
d uriale, &statet;
xlaimedode1d );S3C64XX_UINTM_RXD,
				stup ok
			

);/*to_ourportfeat
	asma_shouldx_seridlag todortrreif
s3c2,
 * it cat
up fpe VICErportrt *por24XX_Ts(and staeid -);
:t s3c24xx_uart_porhutdownte;
	enum and staeid *args)
{
	rt = &o6rport->port;
	stupt->dma;
	struct tty_port *t = &port->state->port;
	struct dma_tx_state state;
	enum	ucon &= ~S3C2410_UCON_RXIRQMODE;
	wrt))
		udelaeid );S3C64XX_6rport->port;
	stup64XX_UINT (%08llx,%p)/

		/rporcter i(	ucon &= ~S3C2~S3C)	s3c24xapbase, 	s3c24membase)art_he character intoengine_paM(d0xfpg u_start_next_tx(oa:dt_n*ouena= dev_id;
	strucerial_ to D== S3C24XX_RX_DM*oue< == 0)
		_comwarntnext_txc->eturn "	S3Ccerial_ourport>tailfined(notrbe 24xx
			

/* xc-m_ks
 *(next_txc->crt_next_tx(oa

/* rt_next_tx(ou);_rx_fimD}->rx_c*ouena-erial_ t_pos3c24xx{
, = d6irq_nosync(oushiftt_p,a&ouF_SHARED		/CGx	ial_txempty_nofifo(struc
{
	u, status(dmaut(tty)trx_dma_completnext_txc->cacuenotram =
{
 %dM);
	s3c24xx{
tatus(and staeid urial/*tFotec.mp)
{biltruct becs
static ic'she
		x_enabl.TGx0bufWB	}
		Xe, &state);
xlaimedode1d ;

	ourport->tx_mode = 0;
}

static xlaimedode1d );port, S3C2410_UCON, ucon);

	rx_enabled(poregl(port, S3C2410_UCON);
	ucon |= S3C2410_UCON_RXIRQMODE;
	wr_regl(RST_5PV2QMODE;
	wrXTRIG8ck = s3ctty) {
cludeFCON, ufcon410_UCON_RXIRQMODE;
	wr_regT(port, S3C2410_UCON, ucon);

	rx_enabled(pout(tty);
	}

	wr_regl(poTc24xx_uart_port *ourport)
{
	struct uart_port *N_TXMODE_CRx ITM);
	else
		 = S3C24XX_TX_PIO;
}

static void s3c2R0_UCON);
	ucon &= ~();S3C64XX_6rport->port;
	stup ok
			

m and staeid *arN_Tpowt cpowt cmanage s3cort *por ame;
}

starc_buf *xmit = &portpmd_regl(port, S3C2410_UC,
known to be it(our		/CGx		r3c24xx_uart_dmaolrport-rport->dma;
	struct tty_struct *tty _state state;
	enumM_DEtimeou00);

	ufcon tate->portm_t(ourp=it(ourpoTc2t bch (t(ourrx_dmcase(3: uthS3C2410_timeou00RESETRX;
	wr_regl(port, S3C2410_UFCON, ufcoon);

	ucon = rd = s3cIS_ERR)tate->porbaudclkAWT_DU3lkurport);_uaesc)are)tate->porbaudclkA= rd 3lkurport);_uaesc)are)tate->porclkA= tlags & U3ccase(0:rd 3lkuesc)arestatic vtate->porclkA= rd = s3cIS_ERR)tate->porbaudclkAWT_DU3lkuesc)arestatic vtate->porbaudclkA= rd ags & UPdefaulD:dma_completnext_txc->caTRX;
	wr_regl(:c24FSTAT)pm %dM);
	t(ourrfimrt	stN_Tbaud rt->lcalculRSTATeps markings freUPPOVICEN, ucon/N, uc40the FAL_SOVICitecAKs pits, ta number marofNG_CONSOLE statces,registe ignoICErregphThe scAKs  ("pclk")hich a the 6];

	e scAKs  ("uclk").rkingN, uc40talso adds todortrescAKs  ("fclk")the t beca versionmODE_C6];raNG_visor.eps markingfollowthe lsma_goes torougaving cAKs  statces,rAGIC_alculRSes to04-Novaud cAKs pi(AGICing *ouulDa
	e cte
 *baud rt->s)hich 
*/

tries tote afis  ing cAKsal_olag t cone* it * an.eps racters comiMAX_CLK4XX_T_LENGTH 15t;

	if (port->dbe i= dev_id;
	stru
}
statcetner_of(port, struct s3c24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc	ort->portE;
	wr_regl(= s3 void num4XXk3c24x1) &&port = 0ed(port) = 0;
	spin_unlock_irqrestore(&port->lock void XXk3elrq);
;
&port = rt->ll_rx_dma_XXk3elre(void *args)
{
	struct sxmit = &port-}
statcetner_of(port, struct s3		/CG24xx_uart_dma lku3elc24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc	ort->portE;
	wr_regl(= s3 void num4XXk3c24x1) &&port =ed(port) = 0;
	spin_unlock_irqrestore(&po4xx_t)N |
	i void XXk3elrq);
)ll_rx_dma_XXk3elre(voic24x lku3elc2&&port =ed(port) ock, void XXk3elrq);
;
&t->lock, lku3elcon)x_dma_XXk3elre(void uct s3c24xx_uart_port *ourport);

st*args)
{
	known to be i= dev_id;
	stru
}
XXkif (ufstat & info->rx_fifofull)
		returhas_interrupt_mq_baudt *ourporXXk **bal_ xlkreturhas_interrupt* lkunum
	return (ufstat & info->rx_fifomask) >> info->rx_fif	*ourporXXk *XXknum	ucon &= ~S3C2r->rx_bytes_requ_dma nc;
baudt quor iclku3el;
bal_ quorode = 0uartFclkstru[MAX_CLK4XX_T_LENGTH]numM_DE_alce(dmaRSTAT, (dmaRSTAT_cha1con)304xx_s U3cclku3elc=tart_next_tcfga_XXku3elc ? rt_next_tcfga_XXku3el :
/* rt_next_tx_dma_rs _XXku3elnumfpe (FO
		 * a nc n)x_dma_num4XXk3 a nc++d s3c24xx_u(XXku3el &ha1con) nc)AWT_DU3c64xx_st
c	 = k(fmt,clkstru>cacXkunfo->baud%d;
	cg ,(dmaXXk k, lkuate;e;
	dma->rx_desc->cclkstru4XX_RX_DMIS_ERR)clkAWT_DU3c64xx_st
c	 rt->lk, lkuate_rt->(clkA= tl4xx_ur_rx_N	rU3c64xx_st
c	 _start_next_tx_dma_>tx_G_vslot= 0)
			ucon &= ~S3C2G_v = 0t->l/t_mq_baud = s3c24xkingsDIVSLO01,
 * it cPPOVICEnewt cs freUalurws 24 tot	 s3c2am =aNG_visor adjust s3coofN1/16becPPOVICEvaud cAKs .t	 s3ct	 s3c2Wridla'tly, ses witDIVSLO01valu241s wi16bes wet	 s3c2_alculRSeth thnotrmulDiply ignoICEvaud  th, ucas it
	 s3c2is easy enougavio);
	alculRSe.t	 s3c(p
ic	quorodeG_v /h, 

/* vaud = 0t->l/tG_vfimD}, S3C2410_Uquorode(0t->l+ (83c2,
q_baud))l/t(163c2,
q_baud)

/* vaud = 0t->l/t(quoro*h, ufimD}->Uquor {
			i_alce(dmaRSTATena-er_baudxx_baud =_RX_DMoalce(dmaRSTATe<port->f_alce(dmaRSTATena-_alce(dmaRSTATt
c	 _staoalce(dmaRSTATe<p(dmaRSTAT= 0)
		*bal_ xlklk, lk

/* val_ quorodequor

/* * lkunumlk, nt

/* xc-aRSTATena_alce(dmaRSTATt
mD}->rx_c*out = val_ quord *arN_TuG_vslot_tODE_[]lic License t;
		 VL_Ss todofr#if de
 *valu24cy on BbaudxG_visor a cogivesthe portre th s3deconet_ ignfpe VICEsDIVSLO01,
 * it T_S3C6400;
}
u163uG_vslot_tODE_[16]ena0)
[0]ena0x
	ufret[1]ena0x
	8fret[2]ena0x
808ret[3]ena0x
888ret[4]ena0x2222ret[5]ena0x4924ret[6]ena0x4A52ret[7]ena0x54AAret[8]ena0x5555ret[9]ena0xD555ret[10]ena0xD5D5ret[11]ena0xDDD5ret[12]ena0xDDDDret[13]ena0xDFDDret[14]ena0xDFDFret[15]ena0xFFDFre};struct circ_buf *xmit = &port-}

, t, ostner_of(port, struct s3		/CGGx		r3c *ourpork, t, os *, t, os		/CGGx		r3c *ourpork, t, os *olrport-rport->dma;
		return N *Xfgk) *s3c24xx_port_tontaia(struc	t)) {
		if (port->flags & UPF_CONS_FLOW)
			s3c24xx_serial_rx_dXk *XXkS_FERR_PTR(-EINVALenum	ucon &= ~S3C2410_UCON_RXIRQMODE;
	baudt quor iclku3elode = 0ort->portE;
	wlt))
			ucon &=  0) {mrx_
			ucon &=  0) {G_vslotFIFfcon fsta3c2Wridla'tlsup_CONS_regmort *por rt->s.
stat);, t, osa_Xontack(			S3HUPCL | CMSPARenum, t, osa_Xontack(ck,CLOCALcon fsta3c2Ask todortres[1])alculRSeice(po_visor fpe us.
stat)
 vaud = t->flate_vaud_rt->(xx_uar, t, os	aolr(d0	 115200*8enumquorode= dev_id;
	stru
}
XXkio_ourportbaudt &xlkr &xlku3elc&po4xx_vaud == 384000RESxe) {
					rx_enablSPDort, S == nablSPDoCUST)
_Uquorodenext_tcustom_o_visor&pshiftIS_ERR)clkAWT_Dport =ed(p24xx_vyTG[1]seC2ic wCEnef (r[1])

/* fcAKs  statce l(p
ipin_tate->porbaudclkserialkA S3C2dlkuesc)arestatic vclkA= rd t sxmit = &port-}
statceturportxlku3elc&prd = s3cIS_ERR)tate->porbaudclkAW 0)
		3lkurport);_uaesc)are)tate->porbaudclkA= /* rt_next_tbaudclks_FERR_PTR(-EINVALenumnsfer_rt_next_tbaudclks_F lk

/*rt_next_tbaudclk_rt->lk, lk ?  lkuate_rt->(clkA :ompl>rx_cookie, &statex_dma_>tx_G_vslot= 0)
		ucon &=  0) G_v = rt_next_tbaudclk_rt->l/ baud = s3_staofga_>tx_fr#ival= 0)
			G_vslotFIF(G_v & 15	

/* x3C64fr#ivalFIF%04*/

	 {G_vslot)fimD}, S3C2410_U	G_vslotFIFuG_vslot_tODE_[G_v & 15]

/* x3C64	G_vslotFIF%04*F(G_v %d)/

	 {G_vslot, G_v & 15	

/*}->rx_c2t bch (, t, osa_Xontack(	 C  coux_dmcase(CS5:dma_3C64rt fig: 5v-20/uart
			

/*wlt)) ON_RXIRQMOL;
	wCS5= tlags & Umcase(CS6:dma_3C64rt fig: 6v-20/uart
			

/*wlt)) ON_RXIRQMOL;
	wCS6= tlags & Umcase(CS7:dma_3C64rt fig: 7v-20/uart
			

/*wlt)) ON_RXIRQMOL;
	wCS7= tlags & Umcase(CS8:UPdefaulD:dma_3C64rt fig: 8v-20/uart
			

/*wlt)) ON_RXIRQMOL;
	wCS8= tlags & Umrial/*tpfeatrv24crigie
 *lt)) IRonet_ igshe
		wlt)) DMA;ofga_wlt)) &N_RXIRQMOL;
	wIRMdma ut(tt, t, osa_Xontack(	 C TOPB)
/*wlt)) |ON_RXIRQMOL;
	w TOPBma ut(tt, t, osa_Xontack(	 PARENBd s3c24xx_, t, osa_Xontack(	 PARODDufcoonlt)) |ON_RXIRQMOL;
	wPODD= tla->tx_chnlt)) |ON_RXIRQMOL;
	wPEVEN Umr, S3C2410_nlt)) |ON_RXIRQMOL;
	wPNONE Umrialport, S3C2410_UCON, ucon);

	rx_enabled(pS3C64Xet_ ignnlt)) [1]er *rtbrdG_v [1]ed	 {G_vslot er */

		/	r3c2l		/*
quor i{G_vslot)fi uct s3c24xx_uart_port *oLurportl

		/CGct s3c24xx_uart_port *oBRDIV*
quor)fi u ucon)
			s3c			SUP(!txe)UTOCTS;s
	{mrx_enabled(port) = 1;
					reMurn;
	ut(tt, t, osa_Xontack(	 CRTSCTS)2410_nmrx_entinue;
			}MurM_AFC= tladdDn);
		 RTS*ourndRXrifocn s3c24xs 63 c24xxgl(por{mrx_eock, flags2reMurn_AFC_8= tl ucon)
			s3c= na(!txe)UTOCTS;smr, S3C2410_nmrx_eock, flags);
MurM_AFC= t}n IRQ_HANDLED;
}

static Mffer *m

		/C_cookie, &statex_dma_>tx_G_vslot= uthe character into2443_DIVSLO0 i{G_vslot)fi ux3C64	->f: wlt)) ONUfer *ort);
 ONUfer *ortf);
 ONUfer */

		/	r3cbled(port) = 1;
					reLurp)		/	r3cbled(port) = 1;
					reurp)		/	r3cbled(port) = 1;
					ren |= )con fsta3c2UpdRSeice(pper-_CONStimeou0.
stat);t->flupdRSe_timeou0(xx_uar, t, osa_Xontackrtbaud)con fsta3c2Wc24xxD_enariieg
			s3c				rxare wCEirq);al_rx_in?
stat);&state)al_sted - rq);
 ON_RXIRQMOnofifo(pOV>calN
	ut(tt, t, osa_Xoitack(	 INPCtWT_D&state)al_sted - rq);
 |ON_RXIRQMOnofifo(port,
3c24xxON_D_enariiegl(p;
	uns;n fsta3c2Wc24xxD_enariieg
			s3c				rxshouldxwCEix0lON?
stat);&stateix0lONsted - rq);
 ON0
	ut(tt, t, osa_Xoitack(	 IGNPAReT_D&stateix0lONsted - rq);
 |ON_RXIRQMOnofifo(pOV>calN
	ut(tt, t, osa_Xoitack(	 IGNBRK0RES, t, osa_Xoitack(	 IGNPAReT_D&stateix0lONsted - rq);
 |ON_RXIRQMOnofifo(port,
;rn fsta3c2rx0lON_port, S3C2410_Ut(tCREADt)->STtonet.
stat);4xx_t, t, osa_Xontack(	 CREAD)f (port->&stateix0lONsted - rq);
 |ONRXifo(pDUMMY_READstatruct s3c24xx_uart_dma *dma = ourport->dma;

	dma_sync_{
clt to ttyTRX;
	wr_regl(poypetner_of(port, struct s3c24xx_t bch (

statiypeux_dmcase(, ufcon;IRQM: &&port = "on;IRQM";dmcase(, ufcon;IR4M: &&port = "on;IR4M";dmcase(, ufcon;IR12: &&port = "on;IRQ2";dmcase(, ufcon;
	in: &&port = "on;
	in/QM";dmdefaulD:dma_serial_rx_fimrt	sters comiMAP_CONS_gned i)args)
{
	ed int ufcon, ch, flaeleaseu		s3cstruct uart_port *port)
{
	stleaseumemed(piontnext_txapbase, MAP_CONS;

	dma_sync_nfo *info = ourport-erial_ 		s3cstruct uart_port *port)
{
	{
clt to ttystruode= dev_id;
	struifo(struc
{
	u

m and staerial_ memed(piontnext_txapbase, MAP_CONS, stru4 ? 0 :o-EBUSYd *args)
{
	struct sxmit = &portrt fig 		s3cstruct uart_port *port,
 0) ->dma;24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc);4xx_				rx_engs & |=FIG_TYPEurpourpor*info = ourport-erial_ 		s3c>tx_mod(port->&stateiype ck void iyped *arN_the vurpfyOVICEnew t->port;
art_p( ourpIOCSSERIALeT_S3C6400;
}
 0)
*info = ourportvurpfy 		s3cstruct uart_port *port,
_uart_po->port;
art_p*o->;24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc);4xx_o->teiype eri, ufcUNKNOWN0RESo->teiype eri void iype) &&port = -EINVALrt *port = );
t_p
#ifrs   |=FIG_SERIAL_SAMSUNGled(pOLEargs)
{
	ial_rx_d
cludect sxmit = &portrt lude;dma_sync_nfo _o_cio *info = ourportrt ludeo_cio	*por)
{
	st * it  {
cludeF&*info = ourportrt ludeu

m and st);
t_rt ludeo_cioma_able_irq(ourport-rt ludeo_cioruc)t *port = &ouXX_SERIAL_ed(pOLE &*info = ourportrt lude
#a->txt *port = &ouXX_SERIAL_ed(pOLE _rx_
#andifp
#if  *portd( |=FIG_SERIAL_SAMSUNGled(pOLEtruct *portd( |=FIG_ed(pOLEwPO3c24a_sync_nfo *info = ourportate_pollouartastruct uart_port *port)*port = &ourport->port;

	strpue_pollouartastruct uart_port *port		/CGxata copy to tt

st#andifp
gs)
{
	ial_rx_uart_opsort->port;

	stropsona0)
.pm		=nf *xmit = &portpm		/.tt x_to_	=nf *xmit = &porttt x_to_		/.
}

mctrl	=nf *xmit = &port
}

mctrl		/.s}

mctrl	=nf *xmit = &ports}

mctrl		/.s it->t	=nf *xmit = &ports it->t		/.s ourpor	=nf *xmit = &ports ourpor		/.s it-rt	=nf *xmit = &ports it-rr		/.ags &_ctl	=nf *xmit = &portags &_ctl		/.s ourup	=nf *xmit = &ports ourup		/.shutdown	=nf *xmit = &portshutdown		/.s}

, t, os	=nf *xmit = &ports}

, t, os		/.iype		=nf *xmit = &portiype		/.aeleaseu		s3	=nf *xmit = &portaeleaseu		s3		/.aerial_ 		s3	=nf *xmit = &portaerial_ 		s3		/.rt fig 		s3	=nf *xmit = &portrt fig 		s3		/.vurpfy 		s3	=nf *xmit = &portvurpfy 		s3,
#if  *portd( |=FIG_SERIAL_SAMSUNGled(pOLEtruct *portd( |=FIG_ed(pOLEwPO3c24
.pollo
}

sartFCO*info = ourportate_pollouart,4
.pollopue_sartFCO*info = ourportpue_pollouart,t#andifp};struct ciial_rx_uart_driverMA_FROM_DEVICE)rvona0)
.owneg		=nTHIS_MODULE,4
.driver_stru	=n"dma;
		rourpor
		/.ng		=n |=FIG_SERIAL_SAMSUNGlngs S		/.rt s		=n= &ouXX_SERIAL_ed(pOLE,4
.dev_stru	=n= &ouXX_SERIAL_XX_T,4
.major		=n= &ouXX_SERIAL_MAJOR,4
.minor		=n= &ouXX_SERIAL_MINOR,4}uc)t *port __, ufcLOCKcUNLOCKED(i) \
	__SPINcLOCKcUNLOCKED(= dev_id;
	struifo(s[i].rx_deFCON,truct ciial_rx_	if (port->flags 
*info = ourportifo(s[ |=FIG_SERIAL_SAMSUNGlngs S]ena0)
[0]ena410_._CONS_F410_UeFCON		=n__, ufcLOCKcUNLOCKED(0)		/CG.ioiype		=nUPIurn;
	}

	.eturnlk	=n0		/CGxx_uart_p		}
6		/CGxops		=n&*info = ourportops		/CGxx			r		=nUPF_BOOxe)UTOC|=F		/CGxrt->		=n0		/C}->rret[1]ena410_._CONS_F410_UeFCON		=n__, ufcLOCKcUNLOCKED(1)		/CG.ioiype		=nUPIurn;
	}

	.eturnlk	=n0		/CGxx_uart_p		}
6		/CGxops		=n&*info = ourportops		/CGxx			r		=nUPF_BOOxe)UTOC|=F		/CGxrt->		=n1		/C}->rre#if  |=FIG_SERIAL_SAMSUNGlngs S > 2
et[2]ena410_._CONS_F410_UeFCON		=n__, ufcLOCKcUNLOCKED(2)		/CG.ioiype		=nUPIurn;
	}

	.eturnlk	=n0		/CGxx_uart_p		}
6		/CGxops		=n&*info = ourportops		/CGxx			r		=nUPF_BOOxe)UTOC|=F		/CGxrt->		=n2		/C}->rre#andifp#if  |=FIG_SERIAL_SAMSUNGlngs S > 3et[3]ena410_._CONS_F410_UeFCON		=n__, ufcLOCKcUNLOCKED(3)		/CG.ioiype		=nUPIurn;
	}

	.eturnlk	=n0		/CGxx_uart_p		}
6		/CGxops		=n&*info = ourportops		/CGxx			r		=nUPF_BOOxe)UTOC|=F		/CGxrt->		=n3		/C}->rt#andifp};s#unrs  __, ufcLOCKcUNLOCKEDarN_Tf *xmit = &portaes}
ags 
ic Licfeat
	todof_uar a cooo_or	todonet_ igs.
ame;
}

starc_buf *xmit = &portaes}
ags tner_of(port, struct s3		/CGGx		rport->dma;
		return N *Xfgc24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc	ort->port.fifosrt) = 0;
	spin_unlock_irqrestore(&porrt->portE;
	wr_rrq);
;
stwr_rrq);
 ck void XXk3elrq);
;
&= s3 void iype c=(, ufcon;IR4M_N	rt->lrq);
 |ON_RXIR4estore0_DIVrt, ed(port) ockwr_rrq);
;
uct s3c24xx_uart_port *ourporosrt) | ofga_w

		/C_c/icfeat
	boo_of_uar at);t, S3C2410_UCON, ucon);

	rx_ofga_w410_UCN_RXIRQMODE;
	wr_regBOTH	/CGct s3c24xx_uart_port *o

	rx_ofga_w410_	/C_c/icsome );

	t)->aeriirortafiiegf_uacfeat
	at);t);

	uc);
t_p
#ifrs   |=FIG_ARM_= &ouXX_CPUFREQdma_sync_nfo *info = ourportcpuf-er_urn tif detner_of(notifier_bAKs  *nbn) {
		 ourport->port.fifoval, *port =ataport-rport->dma;
	struct tty_sttty_;ort_tty_get(&ourportutty_;o);&sta ct s3c24xx_uartnbn *ourport,
				     unsignef-er_urn tif de(&por&sta ct&	dma->rx_ded(p24xx_vyTG[1]seC2ic &sta is eurport l(p
ipin_	dma->rm_t(ourp!(port->port = 0ed(paddtry_MINOworkhouedr to_otbaudrt->lis )

/* ig,xwCEhe Fdeteif
s3c2a])

/* fin 0t->, buedwridl>STto_serisup_CONS ourdeteif ig
s3c2a]d* iurbanc fin ing cAKs -rt->lov->->tx_siz/* .
stat)
 hiftIS_ERR)e->porbaudclkAW
usrx_ch)x) = 1;
		inext_tbaudclk_rt->lkk, lkuate_rt->(e->porbaudclkAW
usrx_ch)x) = 1;
		ivalFI=n PUFREQ>rx_CHANGE{
			ufsdwrishouldxgs lly shut VICErportdown hS3Cst VIC
 s3c2faeriancy])

/* fisfin _tx(ourp. l(p
ir, S3C2
		ivalFI=n PUFREQ>rOSTCHANGE{
			u*ourpork, t, os *, t, os;

	dma_sync_sigs;
	struct_RX_DM dma ucon)
			e int s3c24xxsrx_ch)x) = 1;	uct dma ucon)
			e->rx_deuct_RX_DM dmuct dnt s3c24xxsrx_ch)x) = 1;	u t, os ct&uctd i t, os;
3c24xx_, t, os dnt s3c2 0)
		_comwarnta ucon)xc->ca%s:dnwr, t, os?/

	 __func__	

/* rx_ch)x) = mnsfer_f *xmit = &port-}

, t, ostuxx_uar, t, os	a s3c2 Umria)x) : *port = );
t_p

	if (port->dbe 
*info = ourportcpuf-er_st * it DMA mode */
	ucon = rd_reglport)
{
	e) {
		-er_urn tif de.notifier_ma_aFCO*info = ourportcpuf-er_urn tif dert *port = cpuf-er_st * it _notifier*dma = of-er_urn tif den) {
		  PUFREQ>TRANSITI
	wNOTIFIER);
t_p

	if (port->d*por
*info = ourportcpuf-er_dest * it DMA mode */
	ucon = rd_reglport)
{
	cpuf-er_unst * it _notifier*dma = of-er_urn tif den) {
	ourp PUFREQ>TRANSITI
	wNOTIFIER);
t_p#a->tx

	if (port->dbe 
*info = ourportcpuf-er_st * it DMA mode */
	ucon = rd_reglport)
{
	port = );
t_p

	if (port->d*por
*info = ourportcpuf-er_dest * it DMA mode */
	ucon = rd_reglport)
{
rt#andifprN_Tf *xmit = &port_cio_ags 
ic Lic_ciopori3C2a v_err(po->porErportts, tVICErlat oum (dma->ogiven
 ame;
}

stanfo *info = ourport_cio_ags if (ufstat & info->rx_fifofull)
		return ourf (ufstrlat oume(dma->glplat(dm
	int max_count = port->fifosize;

	while (max_rport->dma;
		return N *Xfgk) {
	struct s3c2_rport->ourtatce lour;		udelaeid );S3C64XX_fo = ourport_cio_ags 64XX_UINT, plat(dmINTM);
		els, plat(dmruc);4xx_plat(dm int s3c24xx_serial-ENOIO;;21;
		inext_tmapbase(!(port->port = -EINVALrt *N_EMPTupk voiS our port-t);&state(dm	=n&plat(dmte(dmrt *N_ES
	stup serianc fisfG_CONSOLE yXaGSoC's which highe these he
		enable_irq(ourport->tx_irq);

	ourport->tx_mer_f *xmit = &portops.s ourupFCO*in6rport->port;
	stupfi u ucon)eturnlkode1d );_staofga_unt =				rx_enabled(port) = 0;
	S3C64XX_fo = ourport_cio_ags 64eurpo ignflowort *por
			

/*ma = of			rx|=enabled(port)  Umrial/*tsporttattVICErhysima_hich virte
 *to gurpes yXaGeachengs 4XX_Ts(as ctrlat oumeate_restatcetulat(dm, IOr_rOURCErn;
	 0dmaut(tty)s dnt s3c2 0)
	_completnext_txc->caurport _chan,d memory_ourtatce fpe unt dma->rx_bytes_ -EINVALrtqueste3C64-ertatce %pR)/

	 -er)fi u ucon)membase y = tm_ioremaptnext_txc->c-ern)
		r(t);
rtatceDEV_Tty)s)ack = s3c ucon)membase2 0)
	_completnext_txc->caurport _chremaport *porifrGto gurpdma->rx_bytes_ -EBUSYd questnext_tmapbase(=c-ern)
		r(;_c*ouenarlat oumeate_t_poslat(dm, 0dmaut(tty)te<port->&stateirq ON0
	u S3C2410_&stateirq ONaeid uXe, &state);
irq ONaeid uXe, &statet;
irq ONaei(xm1;->rx_c*ouenarlat oumeate_t_poslat(dm, 1dmaut(tty)te>port->e, &statet;
irq ONaei;n fsta3c2ailfis )urSOLEly sup_CONrt only 0_UDTarlat oums,ref2ailf_txperties
s3c2ar(popecified.
stat);4xx_plat(dmte(dm.of_nodeurporf_an,d__txperty_plat(dmte(dm.of_noden) {
			 ourp"drpo"	a s3c22 0)
	rt_next_tx(ou);xc-m_kzalurp(next_txc->) {
		 ourEV_Tart*rt_next_tx(o)>) {
		 ourGFP_KERNELack  = s3crt_next_tx(oa:dt_nn*ouena-ENOn;
;pomsrx_ch);
d u*}->rx_ctate->porclk	k, lkuate;&plat(dmte(dm>caunt "(&pshiftIS_ERR)tate->porclkAa:dt_nprmpleta%s:dCt *porifrGcAKs  STtofound/

		/CG	dev_stru;&plat(dmte(dm)->rx_byt naPTR_ERR)tate->porclkA= tlrx_ch);
d urialbyt na3lkuesc)arestatic vtate->porclkA= ut(tty)trx_dmaprmpleta	->f: cAKs  urport _chesc)are+tatic : %dM);
	y)tr= tl3lkueut)tate->porclkA= tlrx_ch);
d urial/*tK, setlldr,M);
	elshrporortaGIC_X_TXrt l(p	enable_irq(ourport->tx_irq);

	ourport->tx_m s3c2he character intoengine_paM(d0xfpg uthe character intoengine_paP(d0xfpg uthe character intoengine_paSP(d0xfpg uriale3C64ags 64mapINTa, memINT, irq=%d (%d,%d), cAKs =%u/

		/	r3c&next_txapbase,  ucon)membase
	s3c24xx{
		/	r3crt *ourport)
{
, status(pott)
{
,  ucon)eturnlk	/C_c/icfeat
	todof_uar (t conerupFVICEet(&)se
		 = S3C24= &portaes}
ags tcter iXfgcrt *port = );
-);
:t next_tmapbase(=c0;
m and staeid *arN_TDdma->gdriverMA->porErport_txbe ame;
}

sta{
clt rport->of_(dma->__buf *xmit EVICE)ourptch[];;
}

stanfo _txbe_irdex;_p

	if (port->df (ufstat & inf= &portdrv_=atatyTRX;
	wrate_driver_=ata(	/CGf (ufstrlat oume(dma->glp(dm
	in#ifrs   |=FIG_OF);4xx_p(dmte(dm.of_node, S3C2diclt rport->of_(dma->__bu*rptchg utrptchk) {furptch_node(f *xmit EVICE)ourptch,  (dmte(dm.of_node,>rx_bytes_ if (ufstat & inf= &portdrv_=ataty)rptchte(ata;->rt#andifp_bytes_ if (ufstat & inf= &portdrv_=ataty)	/CGrlat oumeate_(dma->__b_p(dm)te(river_=ata

	dma_sync_nfo *info = ourport_txbe(f (ufstrlat oume(dma->glp(dm
	inlock_irqsdma->_nodeu*npFCO (dmte(dm.of_node;c	t)) {
		if (port->flags & UPF_CON;		udelirdexFCO txbe_irdex;_	udelaeid );4xx_npa:dt_n*ouenaof_oritx_ate_td_np>caTurpor
4XX_RX_DM*oue>(port->firdexFCOaeid urialS3C64XX_fo = ourport_txbe(%pa:%dM);
	s(dm>cirdexruc);4xx_irdexF>(pARRAY_CONS(= dev_id;
	struifo(s)2 0)
	_complet&p(dmte(dm>ca;
	str%d ouedof r
/* M);
	irdexrucx_bytes_ -EINVALrtquectate->p =n&*info = ourportifo(s[irdex]con tate->pordrv_=atatCO*info = ate_driver_=ata(p(dmruc = s3crt_next_txrv_=ata2 0)
	_complet&p(dmte(dm>cacouldxSTtofiINOR2iverM=atadma->rx_bytes_ -ENOIO;;2>rx_ctate->porbaudclks_FERR_PTR(-EINVALenume, &statex_dmk) {
	structxrv_=ata->rx_fif	{
	struct s3FIF(Gcomate_plat(ata(&p(dmte(dm)2 ?)
		_comate_plat(ata(&p(dmte(dm) :
/* rt_next_txrv_=ata->rs _Xs3c2);4xx_npa)
	rf__txperty_)al_su32_np>
/* "samsung,t->f-x_uart_p;
	

	utrstat = rdl again
,nt < ourrt_next_txrv_=ata->l again
[irdex]rt->e, &state = rdl again
k) {
	structxrv_=ata->l again
[irdex]
	u S3C2ookie, &statex_dma_l again
,t->e, &state = rdl again
k) {
	structx_dma_l again
;rn fsta3c2ailfurn tf10_Umustrbe  voidorta it(ast V1])acICErt->df Rx\	/3c2	chan,d m_cimorEurn tf10df Rx suit;
		 fpe 	S3C64XX
stat);e
		s3c24xx_serial_stk) (fifto			, status(po = rdl again
return ournt);
}

static void s3c24)fi ux3C64%s:d_ciopori3 igne->p %p...
			 __func__, status(dma u*ouena= dev_id;
	stru_cio_ags io_ourportp(dmruc = s3y)te<port->(and staeid -24xx_uble_irq_EVICE)rvd_regla:dt_n*ouenaEVICEst * it _R2iverF&*info = EVICE)rv4XX_RX_DM*oue< == 0)
		prmpletaFrport _chre * it cSamsungengs 4R2iver
			

/* (and staeid u*}->rx_cx3C64%s:dto  igne->p/

	 __func__	

/EVICEto _oneu		s3c&*info = EVICE)rv
	

	utrstat = r	

/rlat oumeste_drv=ata(p(dm
	

	utrstat = r	

n fsta3c2ae#if vRSeice(pcAKs  eurport i)
= dev_id;
	stru_cio_ags  _orr\	/3c2	ch* anca vots3cporEre-eurpor s3cotorougaving pm-_allbas  ov->laps
s3c2a,d y, ssice(pcAKs  eurport i)
tnse case.
stat);3lkurport);_uaesc)are)tate->porclkA=  u*ouena= dev_id;
	strucpuf-er_st * it Dstatus(dmaut(tty)te<port->_complet&p(dmte(dm>caurport _chto  cpuf-er(notifier
			

); txbe_irdexXX_RX_port = );
t_p

	if (poo *info = ourport-emov-(f (ufstrlat oume(dma->gl(dm
	int max_count = port->fifosi*info = _comtou		s3c&(dmte(dm);21;
		inext, S3C2a_size ||
	    cpuf-er_dest * it DLOW)
			s3c24xx_	

/*wVICEstmov-_oneu		s3c&*info = EVICE)rv
	tus(dmaurx_ct->flunst * it _R2iverF&*info = EVICE)rv4XXX_port = );
t_pN_Tngs 4powt cmanage s3cortde ame#ifrs   |=FIG_PM_SLEEPport *port = &ourport->port;use(&p(ock_irqsdma->gl(dm
	int max_count = port->fifosi*info = _comtou		s3c(dm);21;
		inext,
/*wVICE;use(&pu		s3c&*info = EVICE)rv
	tus(dmaX_port = );
t_p

	if (poo *info = ourport-esume(ock_irqsdma->gl(dm
	int max_count = port->fifosi*info = _comtou		s3c(dm);2 = &port->state->port;
	struct dma_tx_state state;
	enu < ournext, S3C23lkuesc)arestatic vtate->porclkA= u = s3cIS_ERR)tate->porbaudclkAWT_DU3lkuesc)arestatic vtate->porbaudclkA= t_port *ourport);
es}
ags tcter i*s3c24xx_port_tontaia(strA= u = s3cIS_ERR)tate->porbaudclkAWT_DU3lkurport);_uaesc)are)tate->porbaudclkA= /*3lkurport);_uaesc)are)tate->porclkA=  u*wVICEstsumeu		s3c&*info = EVICE)rv
	tus(dmaurx_cport = );
t_p

	if (poo *info = ourport-esume_not_poock_irqsdma->gl(dm
	int max_count = port->fifosi*info = _comtou		s3c(dm);2 = &port->state->port;
	struct dma_tx_state state;
	enu < ournext, S3C2/icfeadma a&ou errupat);
			ile_irq(ourport->tx_irq);

	ourport->tx_m s3c2orrt->portE;
	wE;
m ONUff

/* CON, u= rd_regl(port,eturnwE;
m ock, fl&& dma->rx_Tha_MSt

/* CON,n = rd_regl(port,eturnwE;
m ock, fl&& dma->rx_Rha_MSt

/* 3lkuesc)arestatic vtate->porclkA= u  = s3cIS_ERR)tate->porbaudclkAWT_DUU3lkuesc)arestatic vtate->porbaudclkA= t_2he character intoengine_paM(dwE;
mA= u  = s3cIS_ERR)tate->porbaudclkAWT_DUU3lkurport);_uaesc)are)tate->porbaudclkA= /* 3lkurport);_uaesc)are)tate->porclkA= tl}aurx_cport = );
t_p

	if (diclt rport->_compm_opsort->port;

	strpm_opsona0)
.;use(&pena= dev_id;
	stru;use(&p		/.aesumeena= dev_id;
	struaesume		/.aesume_not_pena= dev_id;
	struaesume_not_p,4}uct *port =ERIAL_SAMSUNGlPM_OPS	F&*info = ourportpm_ops)_p#a->tart_! |=FIG_PM_SLEEP racters comi=ERIAL_SAMSUNGlPM_OPS	_rx_
#andifart_ |=FIG_PM_SLEEP ractrt_ 
cludecrtde ame
#ifrs   |=FIG_SERIAL_SAMSUNGled(pOLEargs)
{
	ial_rx_>port;
	strdicl->por;dma_sync_nfo
*info = ourportct ludeotxrd_tner_of(port, struct s3,
known to be iw410_	24xx_uart_port, port);
	rx_fifomask) *s3c24xx_port_to_cfg(a(struc	ort->port.fifosize;
,
ktart_rnu < our;
N |
	inue;
			}
		}
ifocn_fid s3c2rt_f_uac_dma_-xx_vyTGamFCON_ofNGatati)
f_uacfe * it s... l(p
i	size;
k;
		}
		fifocnt--;

		uerstat = rd_bytes_ i);
static void t3c24xxxx_st ? 0 :o1;->rx_caddr,(non-f_uac_dma,xwCEgo2a,d useice(ptxrt);
		rx_to_ l(p
iktart_rk;
		}
		fifocnt--;

		uerTRtat = rdbytes_ i)tart_rx0lON_D_enarTRtat _ThEt ? 1 :omplt_p

	if (bool
*s3c24xx_portrt figureglrrt->portE;
	wr_r	24xx24xxt lidor	todone>porErportrt figuregdr to_ottx/rxc_dma_at
	at);bytes_ i)N |
	i0xfp(!(po;
t_p#ifrs   |=FIG_ed(pOLEwPO3crN_the  
cludecpoll ignroutt->sSfpe  ena igna,d )al_ ignfs, tVICEet(&thS3C2 Lic_cna,dr,M);
	elsourdebugort *ext.
 ame;
}

stanfo *info = ourportate_pollouartastruct uart_port *port) *t = &port->state->port;
	struct dma_tx_state state;
	enum	ucon &= be iw4rt_rnu <size;
k;
		}
		fifocnt--;

		uerstat = rd			ile_irq(ourport-r3c24xxcn io_ourportsize;
mod(port->_serial_OwPO3c
}

smaX_port = 		}
		x_enabled(port)) RXH)d *args)
{
	struct sxmit = &portpue_pollouartastruct uart_port *port		/Cata copy to tt

N);
	ucon &=  0) {regl(port, S3C2410_UCON);
	ucon |= S3C2rt->portE;
	wr_r = 0;
	spin_unlock_irqrestore(&pxx24xSTtopossi
		 Vo a(ou 0_Uunct figuregdport */-24xx_uble_irq__portrt figureglrr_r	WT_Dport =ed(pI
		if u*info = ourportct ludeotxrd_turportsir_r	WT_Dcputaelax(	/CGct s3cx_enabled(port)) TXH, c);
t_p#andifart_ |=FIG_ed(pOLEwPO3c ame;
}

starc_b
*info = ourportct ludeopueuartastruct uart_port *port	u_dma h
N);
	ucon &=  0) {regl(port, S3C2410_UCON);
	ucon |= S3(pI
		if u*info = ourportct ludeotxrd_turportsir_r	WT_Dcputaelax(	/CGct s3cx_enabled(port)) TXH, ch)d *args)
{
	stru
*info = ourportct ludeo enabastruct d
cludecrdi,_{
clt to ttyT		/CGx		r3ytes_requ_dma ing ,N);
	ucon &=  0) {egl(port, S3C2dicl->porock_irqrestore(&pxx24xSTtopossi
		 Vo a(ou 0_Uunct figuregdport */-24xx_uble_irq__portrt figureglrr_r	WT_Dport =ed(ptty) {
cludeo enabadicl->porocs,a ing , = dev_id;
	struct ludeopueuart)d *args)
{
	struc_o_cio
*info = ourportate_opf destner_of(port, struct s3	rrupt*baudt	/CGx		rupt*)arity	rrupt*bita;24xx_uart_pXXk *XXknum	ucon &= E;
	wlt))
			ucon &=  0) {t))
			ucon &=  0) {brG_vfim	ucon &= ~S3C2r->rx_bytes_requ_dma Xku3elnumuartFclk_stru[MAX_CLK4XX_T_LENGTH]nu		wlt)) (port, S3C2410_UCON);
	ucoLore(&port->l (port, S3C2410_UCON);
	ucoore(&porbrG_v(port, S3C2410_UCON);
	ucoBRDIVled(pS3C64Xinfo = ourportate_opf des64XX_UINT\n"	/	r3c"fe * it s: wlt))=er *ort);
=er *ortbR2iv=er */

		/	r3curportsl		/*
u		/*
ubrG_vruc);4xx_ole_irq__portrt figureglrr_r	W S3C2at bch (wlt)) &N_RXIRQMOL;
	wCSrt, S S3C23ase(_RXIRQMOL;
	wCS5:
/* *bita(po5= tllags & Um23ase(_RXIRQMOL;
	wCS6:
/* *bita(po6= tllags & Um23ase(_RXIRQMOL;
	wCS7:
/* *bita(po7= tllags & Um23ase(_RXIRQMOL;
	wCS8:dma_cfaulD:dma *bita(po8= tllags & Um2sfer_ft bch (wlt)) &N_RXIRQMOL;
	wPrt, S S3C23ase(_RXIRQMOL;
	wPEVEN:dma *)arity(po'e'= tllags & U3C23ase(_RXIRQMOL;
	wPODD:dma *)arity(po'o'= tllags & U3C23ase(_RXIRQMOL;
	wPNONE:dma_cfaulD:dma *)arity(po'n' Um2sfer_24xSTw])alculRSeice(pbaud rt->ll(p
i	clku3elodeXinfo = ourportatestatceturpoA= t_p k(fmt,clk_stru>cacXkunfo->baud%d;
	clku3elc&prd XXk k, lkuate;structxc->cclk_stru4XX_RX_DM!IS_ERR)clkAWT_DUrt->lk, lkuate_rt->(clkA= tla->tx_chrt->lk,1d );	*vaud = 0t->l/t(163c2(rbrG_v(+ 1rA= u _3C64ralculRSeth aud %dM);
	*baud)

/sfe}dma_sync_nfo _o_cio
*info = ourportct ludeonerupastruct d
cludecrdi,_{o ttyopf des
	int max_count = port->fif;_	udelvaud = 9600;_	udelvita(po8= tnfo _arity(po'n' Um 0) ->Tw]po'n' U(pS3C64Xinfo = ourportct ludeonerup: d
INT (%d), %s/

		/	r3cdi,_{
ctx_de*oropf des
&pxx24xse tnse aovaligdport */-);_stao
ctx_de*od(p-1 ||_{
ctx_de*F>(p |=FIG_SERIAL_SAMSUNGlngs SWT_Dc
ctx_de*od 0ed(p dma_tx&*info = ourportifo(s[c
ctx_de*].rx_d&pxx24xse tneErportrt figureg? l(p
ipin_	dma->mapbase(=na0x
24xx_serial-ENOIO;;21;dicl->porodenext U(pS3C64Xinfo = ourportct ludeonerup: XX_UINT (%d)M);
		els, c
ctx_de*	

n fsta3c2C_vyTGwheo_or	a,dr,valigd>poronumber has beenpopecified,rAGIta3c2pinsi,_search fpe VICEfiart avail;
		 rport* ancdoes _serta3c2d
cludectup_CON.
stat);4xx_opf des
	u*wVICE_arse_opf destopf des, &baudt &)arity	r&vita	r&->Tw);snruct s3*info = ourportate_opf dest	els, &baudt &)arity	r&vitaled(pS3C64Xinfo = ourportct ludeonerup:  aud %dM);
	baud)con port = rVICE;te_opf dest	els, di,_baudt )arity	rvita	r->Tw);s}args)
{
	ial_rx_d
cludect sxmit = &portrt ludeona0)
.stru		=n= &ouXX_SERIAL_XX_T,4
.sdma->		=ntty) {
cludeosdma->,4
.x			r		=n;
	wPRINTBUFFER,4
.x_de*		=n-1		/. enab		=nf *xmit = &port{
cludeo enab		/.s}
up		=nf *xmit = &port{
cludeos}
up,4
.sata		=n&*info = EVICE)rv
4}uctandifart_ |=FIG_SERIAL_SAMSUNGled(pOLE ame
#ifrs   |=FIG_CPUcon;IRQMtruct ciial_rx_	if (por= &portdrv_=atatdma;
		rourpor_drv_=atatCO{4
.x_ask) &if (ufstat & info->rxx_asS S3C2.stru		=n"Samsungeon;IRQMTngs 
		/C.iype		=n, ufcon;IRQM		/C.x_uart_p		}
6		/C.r3c24xxrpor	=--;

		uerstat _RXrt, 		/C.r3c24xxe(voi	=--;

		uerstat _RXSHIFT		/C.r3c24xxxx_s	=--;

		uerstat _RXFULL		/C.i3c24xxxx_s	=--;

		uerstat _TXFULL		/C.i3c24xxrpor	=--;

		uerstat _TXrt, 		/C.t3c24xxe(voi	=--;

		uerstat _TXSHIFT		/C.rs _XXku3el	=--;

		uer;
	wCLKSELM		/C.num4XXk3	=n2		/C.XXk3elrq);
	=--;

		uer;
	wCLKrt, 		/C.XXk3elre(voi	=--;

		uer;
	wCLKSHIFT		/},4
.sd _Xs3k) &if (ufstat & 		return NS S3C2.rr_r		=--;

		uer;
	wDEFAULT		/C.sir_r		=--;

		uerF;
	wDEFAULT		/},4}uct *port =;

		ueSERIAL_DRV_DATA ((kernelru~S3C_t)&dma;
		rourpor_drv_=ata)
#a->txt *port = &ou	ueSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
#andifp
#ifrs   |=FIG_CPUcon;IRQ2truct ciial_rx_	if (por= &portdrv_=atatdma;
	2rourpor_drv_=atatCO{4
.x_ask) &if (ufstat & info->rxx_asS S3C2.stru		=n"Samsungeon;IRQ2Tngs 
		/C.iype		=n, ufcon;IRQ2		/C.x_uart_p		}64		/C.>tx_G_vslot	=n1		/C.r3c24xxrpor	=--;

	4uerstat _RXrt, 		/C.r3c24xxe(voi	=--;

	4uerstat _RXSHIFT		/C.r3c24xxxx_s	=--;

	4uerstat _RXFULL		/C.i3c24xxxx_s	=--;

	4uerstat _TXFULL		/C.i3c24xxrpor	=--;

	4uerstat _TXrt, 		/C.t3c24xxe(voi	=--;

	4uerstat _TXSHIFT		/C.rs _XXku3el	=--;

		uer;
	wCLKSEL2		/C.num4XXk3	=n4		/C.XXk3elrq);
	=--;

		2er;
	wCLKrt, 		/C.XXk3elre(voi	=--;

		2er;
	wCLKSHIFT		/},4
.sd _Xs3k) &if (ufstat & 		return NS S3C2.rr_r		=--;

		uer;
	wDEFAULT		/C.sir_r		=--;

		uerF;
	wDEFAULT		/},4}uct *port =;

		2eSERIAL_DRV_DATA ((kernelru~S3C_t)&dma;
	2rourpor_drv_=ata)
#a->txt *port = &ou	2eSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
#andifp
#ift *portd( |=FIG_ePUcon;IR40) ||_ *portd( |=FIG_ePUcon;IR, uc||_\dmdefortd( |=FIG_ePUcon;IR43) ||_ *portd( |=FIG_ePUcon;IR42,truct ciial_rx_	if (por= &portdrv_=atatdma;
4	rourpor_drv_=atatCO{4
.x_ask) &if (ufstat & info->rxx_asS S3C2.stru		=n"Samsungeon;IR4MTngs 
		/C.iype		=n, ufcon;IR4M		/C.x_uart_p		}64		/C.>tx_G_vslot	=n1		/C.r3c24xxrpor	=--;

	4uerstat _RXrt, 		/C.r3c24xxe(voi	=--;

	4uerstat _RXSHIFT		/C.r3c24xxxx_s	=--;

	4uerstat _RXFULL		/C.i3c24xxxx_s	=--;

	4uerstat _TXFULL		/C.i3c24xxrpor	=--;

	4uerstat _TXrt, 		/C.t3c24xxe(voi	=--;

	4uerstat _TXSHIFT		/C.rs _XXku3el	=--;

		uer;
	wCLKSEL2		/C.num4XXk3	=n4		/C.XXk3elrq);
	=--;

		2er;
	wCLKrt, 		/C.XXk3elre(voi	=--;

		2er;
	wCLKSHIFT		/},4
.sd _Xs3k) &if (ufstat & 		return NS S3C2.rr_r		=--;

		uer;
	wDEFAULT		/C.sir_r		=--;

		uerF;
	wDEFAULT		/},4}uct *port =;

	4ueSERIAL_DRV_DATA ((kernelru~S3C_t)&dma;
4	rourpor_drv_=ata)
#a->txt *port = &ou4ueSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
#andifp
#ift *portd( |=FIG_ePUcon;
	in) ||_ *portd( |=FIG_ePUcon;6		u,truct ciial_rx_	if (por= &portdrv_=atatdma
	inrourpor_drv_=atatCO{4
.x_ask) &if (ufstat & info->rxx_asS S3C2.stru		=n"Samsungeon;
	inTngs 
		/C.iype		=n, ufcon;
	in		/C.x_uart_p		}64		/C.>tx_G_vslot	=n1		/C.r3c24xxrpor	=--;

	4uerstat _RXrt, 		/C.r3c24xxe(voi	=--;

	4uerstat _RXSHIFT		/C.r3c24xxxx_s	=--;

	4uerstat _RXFULL		/C.i3c24xxxx_s	=--;

	4uerstat _TXFULL		/C.i3c24xxrpor	=--;

	4uerstat _TXrt, 		/C.t3c24xxe(voi	=--;

	4uerstat _TXSHIFT		/C.rs _XXku3el	=--;

		uer;
	wCLKSEL2		/C.num4XXk3	=n4		/C.XXk3elrq);
	=--;

	inrr;
	wCLKrt, 		/C.XXk3elre(voi	=--;

	inrr;
	wCLKSHIFT		/},4
.sd _Xs3k) &if (ufstat & 		return NS S3C2.rr_r		=--;

		uer;
	wDEFAULT		/C.sir_r		=--;

		uerF;
	wDEFAULT		/},4}uct *port =;

	inrSERIAL_DRV_DATA ((kernelru~S3C_t)&dma
	inrourpor_drv_=ata)
#a->txt *port = &
	inrSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
#andifp
#ifrs   |=FIG_CPUco5PV2QMtruct ciial_rx_	if (por= &portdrv_=atatd5pv2		rourpor_drv_=atatCO{4
.x_ask) &if (ufstat & info->rxx_asS S3C2.stru		=n"Samsungeo5PV2QMTngs 
		/C.iype		=n, ufcon;
	in		/C.>tx_G_vslot	=n1		/C.r3c24xxrpor	=--5PV2QMODEtat _RXrt, 		/C.r3c24xxe(voi	=--5PV2QMODEtat _RXSHIFT		/C.r3c24xxxx_s	=--5PV2QMODEtat _RXFULL		/C.i3c24xxxx_s	=--5PV2QMODEtat _TXFULL		/C.i3c24xxrpor	=--5PV2QMODEtat _TXrt, 		/C.t3c24xxe(voi	=--5PV2QMODEtat _TXSHIFT		/C.rs _XXku3el	=--;

		uer;
	wCLKSELM		/C.num4XXk3	=n2		/C.XXk3elrq);
	=--5PV2QMOD;
	wCLKrt, 		/C.XXk3elre(voi	=--5PV2QMOD;
	wCLKSHIFT		/},4
.sd _Xs3k) &if (ufstat & 		return NS S3C2.rr_r		=--5PV2QMOD;
	wDEFAULT		/C.sir_r		=--5PV2QMODE;
	wDEFAULT		/},4	dl again
k) { 256,}64	}
6	}
6 },4}uct *port =5PV2QMOSERIAL_DRV_DATA ((kernelru~S3C_t)&d5pv2		rourpor_drv_=ata)
#a->txt *port =5PV2QMOSERIAL_DRV_DATA	(kernelru~S3C_t)_rx_
#andifp
#ift *portd( |=FIG_ARCH_EXYNOS)xt *port EXYNOS_COMM
	w ERIAL_DRV_DATA				\4
.x_ask) &if (ufstat & info->rxx_asS S			\4
2.stru		=n"SamsungeExynosTngs 
		\4
2.iype		=n, ufcon;
	in				\4
2.>tx_G_vslot	=n1					\4
2.r3c24xxrpor	=--5PV2QMODEtat _RXrt, 		\4
2.r3c24xxe(voi	=--5PV2QMODEtat _RXSHIFT		\4
2.r3c24xxxx_s	=--5PV2QMODEtat _RXFULL		\4
2.i3c24xxxx_s	=--5PV2QMODEtat _TXFULL		\4
2.i3c24xxrpor	=--5PV2QMODEtat _TXrt, 		\4
2.i3c24xxe(voi	=--5PV2QMODEtat _TXSHIFT		\4
2.rs _XXku3el	=--;

		uer;
	wCLKSELM			\4
2.sum4XXk3	=n1					\4
2.XXk3elrq);
	=-0					\4
2.XXk3elre(voi	=-0					\4
}, {
				\4
.sd _Xs3k) &if (ufstat & 		return NS S			\4
2.rr_r		=--5PV2QMOD;
	wDEFAULT			\4
2.rir_r		=--5PV2QMODE;
	wDEFAULT		\4
2.>tx_fr#ival	=n1					\4
} {
				\4truct ciial_rx_	if (por= &portdrv_=atatexynos42		rourpor_drv_=atatCO{4
EXYNOS_COMM
	w ERIAL_DRV_DATA,4	dl again
k) { 256,}64	}
6	}
6 },4}uctruct ciial_rx_	if (por= &portdrv_=atatexynos5433rourpor_drv_=atatCO{4
EXYNOS_COMM
	w ERIAL_DRV_DATA,4	dl again
k) { 64	}256,}
6	}256 },4}uctt *port EXYNOS42QMOSERIAL_DRV_DATA ((kernelru~S3C_t)&exynos42		rourpor_drv_=ata)xt *port EXYNOS5433rSERIAL_DRV_DATA ((kernelru~S3C_t)&exynos5433rourpor_drv_=ata)
#a->txt *port EXYNOS42QMOSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
# *port EXYNOS5433rSERIAL_DRV_DATA (kernelru~S3C_t)_rx_
#andifp


	if (diclt rport->rlat oume(dma->_ruct sxmit = &portdriver_ids[]ena410S3C2.stru		=n"at & 		-unt "		/C.rriver_=ata	=--;

		ue ERIAL_DRV_DATA,4	}, S3C2.stru		=n"at & 	2-unt "		/C.rriver_=ata	=--;

		2eSERIAL_DRV_DATA,4	}, S3C2.stru		=n"at & 4	-unt "		/C.rriver_=ata	=--;

	4ueSERIAL_DRV_DATA,4	}, S3C2.stru		=n"at 
	in-unt "		/C.rriver_=ata	=--;

	inrSERIAL_DRV_DATA,4	}, S3C2.stru		=n"a5pv2		-unt "		/C.rriver_=ata	=--5PV2QMOSERIAL_DRV_DATA,4	}, S3C2.stru		=n"exynos42		-unt "		/C.rriver_=ata	=-EXYNOS42QMOSERIAL_DRV_DATA,4	}, S3C2.stru		=n"exynos5433-unt "		/C.rriver_=ata	=-EXYNOS5433rSERIAL_DRV_DATA		/},4	{ },4}ucMODULEwDEVICE_TABLE(rlat oum, = dev_id;
	strudriver_idsruc)tifrs   |=FIG_OF)
}

sta{
clt rport->of_(dma->__buf *xmit EVICE)ourptch[]ena410S .comp

sbdeona"samsung,at & 		-unt "		/C.ratatCO(*port )= &ou	ueSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,at & 	2-unt "		/C.ratatCO(*port )= &ou	2eSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,at & 4	-unt "		/C.ratatCO(*port )= &ou4ueSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,at 
	in-unt "		/C.ratatCO(*port )= &
	inrSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,a5pv2		-unt "		/C.ratatCO(*port )=5PV2QMOSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,exynos42		-unt "		/C.ratatCO(*port )EXYNOS42QMOSERIAL_DRV_DATA },4	{ .comp

sbdeona"samsung,exynos5433-unt "		/C.ratatCO(*port )EXYNOS5433rSERIAL_DRV_DATA },4	{},4}ucMODULEwDEVICE_TABLE(of,uf *xmit EVICE)ourptch
st#andifp
gs)
{
	ial_rx_rlat oume(riverMAamsungd;
	strudrivertCO{4
. txbe		=nf *xmit = &portptxbe		/.aemov-		=nf *xmit = &portaemov-,4
.xd_tODE_	=nf *xmit = &portdriver_ids,4
.driver		=nS3C2.stru	na"samsung-unt "		/C.pm	=--ERIAL_SAMSUNGlPM_OPS		/C.{furptch_tODE_	=n{furptch_ptr(f *xmit EVICE)ourptch)		/},4}uc
modudeoplat oume(river(Aamsungd;
	strudriverruc)tifrs   |=FIG_SERIAL_SAMSUNGled(pOLEaN_the Early_d
clude.
 ame;
}l_rx_	amsungdearly {
cludeosatat);
	32 txxx_srq);
;
};struct circ_bufamsungdearly busyEVICastruct uart_port *port) *t I
		if u()al_l_	dma->membase +--;

		uerTRtat =x0lON_D_enarTRtat _ThFEAWT_Dd *args)
{
	structamsungdearly busyEVICc24xxastruct uart_port *port) *t = &port-amsungdearly {
cludeosatat*ratatCOtrstat r vRSe_=ata

t I
		if )al_l_	dma->membase +--;

		uerFtat =x0l=ata->txxx_srq);
WT_Dd *args)
{
	structamsungdearly pueuastruct uart_port *port	u_dma ) *t t(tty)l_l_	dma->membase +--;

		uerFore(
	inue;
			}
		}
ifocn_fid s3*amsungdearly busyEVICc24xxaurpoA= truct s3*amsungdearly busyEVICae;
	enu < enabb(c,  ucon)membase +--;

		uerTXH)d *args)
{
	structamsungdearly  enabastruct d
cludecrdin,_{
clt to ttyT	3ytes_requ_	24xx_uart_pearlyr_rrsdma->gl(dm ct s3te(ata;-(ptty) {
cludeo enaba&(dmtecter i*, n,_tamsungdearly pueu;

	dma_sync_nfo _o_cio *amsungdearly {
cludeonerupastruct earlyr_rrsdma->gl(dma->,4

			 ourp_{
clt to ttyopt) *t t(tt!(dma->po = rdmembase24xx_serial-ENOIO;;21;(dma->po s3te enabodeXamsungdearly  enab;X_port = );
t_pN_Ton;IRQMT3C6400;
}
= &port-amsungdearly {
cludeosatatat & 		rearly {
cludeosatatCO{4
.txxx_srq);
 =--;

		uerstat _TXFULL		};struct cinfo _o_cio *info		rearly {
cludeonerupastruct earlyr_rrsdma->gl(dma->,4

			 ourp_{
clt to ttyopt) *t (dma->po = rd r vRSe_=ata_tx&*info		rearly {
cludeosata;X_port = *amsungdearly {
cludeonerupa(dma->,ropf;

	dOF_EARLY;
	wDECLARS(= dev10,a"samsung,at & 		-unt "		/C	*info		rearly {
cludeonerupenu N_Ton;IRQ2 into2440 intoenxxT3C6400;
}
= &port-amsungdearly {
cludeosatatat & 4	rearly {
cludeosatatCO{4
.txxx_srq);
 =--;

	4uerstat _TXFULL		};struct cinfo _o_cio *info4	rearly {
cludeonerupastruct earlyr_rrsdma->gl(dma->,4

			 ourp_{
clt to ttyopt) *t (dma->po = rd r vRSe_=ata_tx&*info4	rearly {
cludeosata;X_port = *amsungdearly {
cludeonerupa(dma->,ropf;

	dOF_EARLY;
	wDECLARS(= dev12,a"samsung,at & 	2-unt "		/C	*info4	rearly {
cludeonerup;

OF_EARLY;
	wDECLARS(= dev40,a"samsung,at & 4	-unt "		/C	*info4	rearly {
cludeonerup;

OF_EARLY;
	wDECLARS(= d
	in	a"samsung,at 
	in-unt "		/C	*info4	rearly {
cludeonerup;

 N_To5PV2QM,-EXYNOST3C6400;
}
= &port-amsungdearly {
cludeosatata5pv2		rearly {
cludeosatatCO{4
.txxx_srq);
 =--5PV2QMODEtat _TXFULL		};struct cinfo _o_cio *5pv2		rearly {
cludeonerupastruct earlyr_rrsdma->gl(dma->,4

			 ourp_{
clt to ttyopt) *t (dma->po = rd r vRSe_=ata_tx&*5pv2		rearly {
cludeosata;X_port = *amsungdearly {
cludeonerupa(dma->,ropf;

	dOF_EARLY;
	wDECLARS(=5pv2		,a"samsung,a5pv2		-unt "		/C	*5pv2		rearly {
cludeonerup;

OF_EARLY;
	wDECLARS(exynos42		,a"samsung,exynos42		-unt "		/C	*5pv2		rearly {
cludeonerup;

#andifp
MODULEwALIAS("plat oum:samsung-unt ")ucMODULEw