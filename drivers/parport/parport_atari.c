/* Low-level parallel port routines for the Atari builtin port
 *
 * Author: Andreas Schwab <schwab@issan.informatik.uni-dortmund.de>
 *
 * Based on parport_amiga.c.
 *
 * The built-in Atari parallel port provides one port at a fixed address
 * with 8 output data lines (D0 - D7), 1 output control line (STROBE)
 * and 1 input status line (BUSY) able to cause an interrupt.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/parport.h>
#include <linux/interrupt.h>
#include <asm/setup.h>
#include <asm/atarihw.h>
#include <asm/irq.h>
#include <asm/atariints.h>

static struct parport *this_port;

static unsigned char
parport_atari_read_data(struct parport *p)
{
	unsigned long flags;
	unsigned char data;

	local_irq_save(flags);
	sound_ym.rd_data_reg_sel = 15;
	data = sound_ym.rd_data_reg_sel;
	local_irq_restore(flags);
	return data;
}

static void
parport_atari_write_data(struct parport *p, unsigned char data)
{
	unsigned long flags;

	local_irq_save(flags);
	sound_ym.rd_data_reg_sel = 15;
	sound_ym.wd_data = data;
	local_irq_restore(flags);
}

static unsigned char
parport_atari_read_control(struct parport *p)
{
	unsigned long flags;
	unsigned char control = 0;

	local_irq_save(flags);
	sound_ym.rd_data_reg_sel = 14;
	if (!(sound_ym.rd_data_reg_sel & (1 << 5)))
		control = PARPORT_CONTROL_STROBE;
	local_irq_restore(flags);
	return control;
}

static void
parport_atari_write_control(struct parport *p, unsigned char control)
{
	unsigned long flags;

	local_irq_save(flags);
	sound_ym.rd_data_reg_sel = 14;
	if (control & PARPORT_CONTROL_STROBE)
		sound_ym.wd_data = sound_ym.rd_data_reg_sel & ~(1 << 5);
	else
		sound_ym.wd_data = sound_ym.rd_data_reg_sel | (1 << 5);
	local_irq_restore(flags);
}

static unsigned char
parport_atari_frob_control(struct parport *p, unsigned char mask,
			   unsigned char val)
{
	unsigned char old = parport_atari_read_control(p);
	parport_atari_write_control(p, (old & ~mask) ^ val);
	return old;
}

static unsigned char
parport_atari_read_status(struct parport *p)
{
	return ((st_mfp.par_dt_reg & 1 ? 0 : PARPORT_STATUS_BUSY) |
		PARPORT_STATUS_SELECT | PARPORT_STATUS_ERROR);
}

static void
parport_atari_init_state(struct pardevice *d, struct parport_state *s)
{
}

static void
parport_atari_save_state(struct parport *p, struct parport_state *s)
{
}

static void
parport_atari_restore_state(struct parport *p, struct parport_state *s)
{
}

static void
parport_atari_enable_irq(struct parport *p)
{
	enable_irq(IRQ_MFP_BUSY);
}

static void
parport_atari_disable_irq(struct parport *p)
{
	disable_irq({tic void
parport_atari_init_statect parport *p)
{
	UkD7C_state(struct pardevice *d, struct k~e_irq(struct parport *p)
{
	disable_irq({tic void
parport^k~7&-