/*
 * UART driver for PNX8XXX SoCs
 *
 * Author: Per Hallsmark per.hallsmark@mvista.com
 * Ported to 2.6 kernel by EmbeddedAlley
 * Reworked by Vitaly Wool <vitalywool@gmail.com>
 *
 * Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 * Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of
 * any kind, whether express or implied.
 *
 */

#if defined(CONFIG_SERIAL_PNX8XXX_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_pnx8xxx.h>

#include <asm/io.h>
#include <asm/irq.h>

/* We'll be using StrongARM sa1100 serial port major/minor */
#define SERIAL_PNX8XXX_MAJOR	204
#define MINOR_START		5

#define NR_PORTS		2

#define PNX8XXX_ISR_PASS_LIMIT	256

/*
 * Convert from ignore_status_mask or read_status_mask to FIFO
 * and interrupt status bits
 */
#define SM_TO_FIFO(x)	((x) >> 10)
#define SM_TO_ISTAT(x)	((x) & 0x000001ff)
#define FIFO_TO_SM(x)	((x) << 10)
#define ISTAT_TO_SM(x)	((x) & 0x000001ff)

/*
 * This is the size of our serial port register set.
 */
#define UART_PORT_SIZE	0x1000

/*
 * This determines how often we check the modem status signals
 * for any change.  They generally aren't connected to an IRQ
 * so we have to poll them.  We also check immediately before
 * filling the TX fifo incase CTS has been dropped.
 */
#define MCTRL_TIMEOUT	(250*HZ/1000)

extern struct pnx8xxx_port pnx8xxx_ports[];

static inline int serial_in(struct pnx8xxx_port *sport, int offset)
{
	return (__raw_readl(sport->port.membase + offset));
}

static inline void serial_out(struct pnx8xxx_port *sport, int offset, int value)
{
	__raw_writel(value, sport->port.membase + offset);
}

/*
 * Handle any change of modem status signal since we were last called.
 */
static void pnx8xxx_mctrl_check(struct pnx8xxx_port *sport)
{
	unsigned int status, changed;

	status = sport->port.ops->get_mctrl(&sport->port);
	changed = status ^ sport->old_status;

	if (changed == 0)
		return;

	sport->old_status = status;

	if (changed & TIOCM_RI)
		sport->port.icount.rng++;
	if (changed & TIOCM_DSR)
		sport->port.icount.dsr++;
	if (changed & TIOCM_CAR)
		uart_handle_dcd_change(&sport->port, status & TIOCM_CAR);
	if (changed & TIOCM_CTS & TIOCM_CAR);
	eMGk K