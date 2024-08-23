/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1997, 1998, 2001, 03, 05, 06 by Ralf Baechle
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <linux/rtc/ds1286.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/notifier.h>
#include <linux/pm.h>
#include <linux/timer.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/reboot.h>
#include <asm/sgialib.h>
#include <asm/sgi/ioc.h>
#include <asm/sgi/hpc3.h>
#include <asm/sgi/mc.h>
#include <asm/sgi/ip22.h>

/*
 * Just powerdown if init hasn't done after POWERDOWN_TIMEOUT seconds.
 * I'm not sure if this feature is a good idea, for now it's here just to
 * make the power button make behave just like under IRIX.
 */
#define POWERDOWN_TIMEOUT	120

/*
 * Blink frequency during reboot grace period and when panicked.
 */
#define POWERDOWN_FREQ		(HZ / 4)
#define PANIC_FREQ		(HZ / 8)

static struct timer_list power_timer, blink_timer, debounce_timer;

#define MACHINE_PANICED		1
#define MACHINE_SHUTTING_DOWN	2

static int machine_state;

static void __noreturn sgi_machine_power_off(void)
{
	unsigned int tmp;

	local_irq_disable();

	/* Disable watchdog */
	tmp = hpc3c0->rtcregs[RTC_CMD] & 0xff;
	hpc3c0->rtcregs[RTC_CMD] = tmp | RTC_WAM;
	hpc3c0->rtcregs[RTC_WSEC] = 0;
	hpc3c0->rtcregs[RTC_WHSEC] = 0;

	while (1) {
		sgioc->panel = ~SGIOC_PANEL_oc~A4ot.c	f,bye cruoc-world ..Lictchc~A4otIf we'it'still runnncy, we prob	/*yre  I'eignan alarmc~A   <linux/int  Reade.  SelagsubjclearforLictch	dog */
	tmp = hpc3c0->rtcreHOURS_ALARM]oc~}
}_state;

static void __noreturn sgi_mretatrt(charf*commperff(vot s(ic int machinTC_define MACHINE_SHUTTI)e (1) urn sgi_machine_powisa(1) m
		cpticrl0 |= nelMregCTRL0_SYSINIT;] = 0;

	wh;
}_state;

static void __noreturn sgi_mhaltwer_off(vot s(ic int machinTC_define MACHINE_SHUTTI)e (1) urn sgi_machine_powisa(ArcElinuIlinuactis Mod_disa}_state;

statir_list powout(id)
{
	unlocy dataff(vo1) urn sgi_machine_powisa}_state;

statitimer, bliout(id)
{
	unlocy dataff(vo4otXXXSeexsure ifidefullhouseictch	1) u {
mretet ^= (nel = RESET_LC0OFF|nel = RESET_LC1OFFisa(1) {
		retet = 1) u {
mretetEC] modebounc(&timer, blink_jifux/s + datafsa}_state;

statitimer, d(id)
{
	unlocy dataff(vodelebounc(&timer, debouncisa(t s(1) nt->itate1TC_nelNT_ISTAT1_PWRhile (4otIlinux/in'still bency 'eig.ctch	dtimer, debounc.exp mas = jifux/s + _FREQ	20); 4ot0.05s	tch	daddebounc(&timer, debouncisae (1) {
		sgioc->pnel = ~SGIOC_PANEL_] =nel = ~SGIOC_PANElNTR |e (A       nel = ~SGIOCVOLDNlNTR | nel = ~SGIOCVOLDNHOLD |e (A       nel = ~SGIOCVOLUPlNTR | nel = ~SGIOCVOLUPHOLDsae (id __noc~}
vot s(ic int machinTC_define MACHINE_)e (1) m
		cpticrl0 |= nelMregCTRL0_SYSINIT;]
	en	/* 
	lo(nel ~SGIOCIRQfsa}_state;

inl	1
#statir_list powerwer_off(vot s(ic int machinTC_define MACHINE_)e (id __nocvot s((ic int machinTC_define MACHINE_SHUTTI) ||e (Akill_cad_pid(SIelNTC) )hile (4otNodown iprocesndihe power pmass	untwic
Lictch	d1) urn sgi_machine_powisa(}
voic int machinT|= define MACHINE_SHUTTIsa(timer, blin.data->pdefine POWERDOsa(timer, bliout(define POWERDOisae own ebounc(&r_list poweisa(r_list powe.fr, d co->pr_list powoutsa(r_list powe.exp mas = jifux/s +  after POWERDOWN_T* HZsa(addebounc(&r_list poweisa}_state;

irqid __n_t sgioc_ nt(signirq,#stati*dev__off(void)
{
	unsign powerssae  powers = 1) {
		sgiocsa(1) {
		sgioc->pnel = ~SGIOC_PANEL_] =nel = ~SGIOC_PANElNTRocvot s(1) nt->itate1TC_nelNT_ISTAT1_PWRhile (4otWan iuntil <linux/inre /s awayctch	dt

	/* 
	locnosync(nel ~SGIOCIRQfsa	 own ebounc(&timer, debouncisa(dtimer, debounc.fr, d co->ptimer, dsa(dtimer, debounc.exp mas = jifux/s + 5;h	daddebounc(&timer, debouncisa(}
vo4otPe the power was pmass	u
	T* sm/sps pux/ 22: "T  SPgioc-Regitathee ip;
l	unPe theControl 05,Full
	T* House. On*yrle time2 bit of usts	u. Gunt sndts	ndtp grifiur bit 
	T* fidevolumerms trol".