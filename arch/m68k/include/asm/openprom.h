/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SPARC_OPENPROM_H
#define __SPARC_OPENPROM_H

/* openprom.h:  Prom structures and defines for access to the OPENBOOT
 *              prom routines and data areas.
 *
 * Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 */


/* Empirical constants... */
#ifdef CONFIG_SUN3
#define KADB_DEBUGGER_BEGVM     0x0fee0000    /* There is no kadb yet but...*/
#define LINUX_OPPROM_BEGVM      0x0fef0000
#define LINUX_OPPROM_ENDVM      0x0ff10000    /* I think this is right - tm */
#else
#define KADB_DEBUGGER_BEGVM     0xffc00000    /* Where kern debugger is in virt-mem */
#define	LINUX_OPPROM_BEGVM	0xffd00000
#define	LINUX_OPPROM_ENDVM	0xfff00000
#define	LINUX_OPPROM_MAGIC      0x10010407
#endif

#ifndef __ASSEMBLY__
/* V0 prom device operations. */
struct linux_dev_v0_funcs {
	int (*v0_devopen)(char *device_str);
	int (*v0_devclose)(int dev_desc);
	int (*v0_rdblkdev)(int dev_desc, int num_blks, int blk_st, char *buf);
	int (*v0_wrblkdev)(int dev_desc, int num_blks, int blk_st, char *buf);
	int (*v0_wrnetdev)(int dev_desc, int num_bytes, char *buf);
	int (*v0_rdnetdev)(int dev_desc, int num_bytes, char *buf);
	int (*v0_rdchardev)(int dev_desc, int num_bytes, int dummy, char *buf);
	int (*v0_wrchardev)(int dev_desc, int num_bytes, int dummy, char *buf);
	int (*v0_seekdev)(int dev_desc, long logical_offst, int from);
};

/* V2 and later prom device operations. */
struct linux_dev_v2_funcs {
	int (*v2_inst2pkg)(int d);	/* Convert ihandle to phandle */
	char * (*v2_dumb_mem_alloc)(char *va, unsigned sz);
	void (*v2_dumb_mem_free)(char *va, unsigned sz);

	/* To map devices into virtual I/O space. */
	char * (*v2_dumb_mmap)(char *virta, int which_io, unsigned paddr, unsigned sz);
	void (*v2_dumb_munmap)(char *virta, unsigned size);

	int (*v2_dev_open)(char *devpath);
	void (*v2_dev_close)(int d);
	int (*v2_dev_read)(int d, char *buf, int nbytes);
	int (*v2_dev_write)(int d, char *buf, int nbytes);
	int (*v2_dev_seek)(int d, int hi, int lo);

	/* Never issued (multistage load support) */
	void (*v2_wheee2)(void);
	void (*v2_wheee3)(void);
};

struct linux_mlist_v0 {
	struct linux_mlist_v0 *theres_more;
	char *start_adr;
	unsigned num_bytes;
};

struct linux_mem_v0 {
	struct linux_mlist_v0 **v0_totphys;
	struct linux_mlist_v0 **v0_prommap;
	struct linux_mlist_v0 **v0_available; /* What we can use */
};

/* Arguments sent to the kernel from the boot prompt. */
struct linux_arguments_v0 {
	char *argv[8];
	char args[100];
	char boot_dev[2];
	int boot_dev_ctrl;
	int boot_dev_unit;
	int dev_partition;
	char *kernel_file_name;
	void *aieee1;           /* XXX */
};

/* V2 and up boot things. */
struct linux_bootargs_v2 {
	char **bootpath;
	char **bootargs;
	int *fd_stdin;
	int *fd_stdout;
};

#if defined(CONFIG_SUN3) || defined(CONFIG_SUN3X)
struct linux_romvec {
	char		*pv_initsp;
	int		(*pv_startmon)(void);

	int		*diagberr;

	struct linux_arguments_v0 **pv_v0bootargs;
	unsigned	*pv_sun3mem;

	unsigned char	(*pv_getchar)(void);
	int		(*pv_putchar)(int ch);
	int		(*pv_nbgetchar)(void);
	int		(*pv_nbputchar)(int ch);
	unsigned char	*pv_echo;
	unsigned char	*pv_insource;
	unsigned char	*pv_outsink;

	int		(*pv_getkey)(void);
	int		(*pv_initgetkey)(void);
	unsigned int	*pv_translation;
	unsigned char	*pv_keybid;
	int		*pv_screen_x;
	int		*pv_screen_y;
	struct keybuf	*pv_keybuf;

	char		*pv_monid;

	/*
	 * Frame buffer output and terminal emulation
	 */

	int		(*pv_fbwritechar)(char);
	int		*pv_fbaddr;
	char		**pv_font;
	int		(*pv_fbwritestr)(char);

	void		(*pv_reboot)(char *bootstr);

	/*
	 * Line input and parsing
	 */

	unsigned char	*pv_linebuf;
	unsigned char	**pv_lineptr;
	int		*pv_linesize;
	int		(*pv_getline)(void);
	unsigned char	(*pv_getnextchar)(void);
	unsigned char	(*pv_peeknextchar)(void);
	int		*pv_fbthere;
	int		(*pv_getnum)(void);

	void		(*pv_printf)(const char *fmt, ...);
	int		(*pv_printhex)(void);

	unsigned char	*pv_leds;
	int		(*pv_setleds)(void);

	/*
	 * Non-maskable interrupt  (nmi) information
	 */

	int		(*pv_nmiaddr)(void);
	int		(*pv_abortentry)(void);
	int		*pv_nmiclock;

	int		*pv_fbtype;

	/*
	 * Assorted other things
	 */

	unsigned	pv_romvers;
	struct globram  *pv_globram;
	char		*pv_kbdzscc;

	int		*pv_keyrinit;
	unsigned char	*pv_keyrtick;
	unsigned	*pv_memoryavail;
	long		*pv_resetaddr;
	long		*pv_resetmap;

	void		(*pv_halt)(void);
	unsigned char	*pv_memorybitmap;

#ifdef CONFIG_SUN3
	void		(*pv_setctxt)(int ctxt, char *va, int pmeg);
	void		(*pv_vector_cmd)(void);
	int		dummy1z;
	int		dummy2z;
	int		dummy3z;
	int		dummy4z;
#endif
};
#else
/* The top level PROM vector. */
struct linux_romvec {
	/* Version numbers. */
	unsigned int pv_magic_cookie;
	unsigned int pv_romvers;
	unsigned int pv_plugin_revision;
	unsigned int pv_printrev;

	/* Version 0 memory descriptors. */
	struct linux_mem_v0 pv_v0mem;

	/* Node operations. */
	struct linux_nodeops *pv_nodeops;

	char **pv_bootstr;
	struct linux_dev_v0_funcs pv_v0devops;

	char *pv_stdin;
	char *pv_stdout;
#define	PROMDEV_KBD	0	din;rt) fVv_seek)(int d, int hi, int lo);

	/* Never issued SR2tLion;
	unsigned char	*pv_keybid;
	int		*pv_screen_xatxt)(in	"_wheee3)(void);
};

struct linux_mlist_v0 {
	strucSV(5V(5V(5RI(5Pmlist_v0 {
	stru5Nngz;
	int	"adr;
	unsigned num_bytes;
};

struct linux_mem_v0 SV(5Ruu7oOM vect	"bputchar)(int ch);
	unsigned char	*pv_echo;
	unsigbputsigned 	"pt. */
struct linux_arguments_v0 {
	char *argv[8];(5V(5MnedSRnLfbaddr;
	char		**pv_font;
	int		(*pv_fbwritestr)(c/5V(5N4nsigntin_5V(5V(5V(5Taddrnhar 	pm(5VnL8];
	char args[100];
	char boot_dev[2];
	int boot_SV(5Rrp	{	"signed char	**pv_lineptr;
	int		*pv_linesize;
	int/5V(5V(5V(5OuVm;

	unsigned char	(*pv_getchar)(void);
	int		(*pv-oL_peeknextchar)(void);
	int		*pv_fbthere;
	int		(*p/5V(5N)(voeinsfVlinux_romvec {
	char		*pv_initsp;
	int		(*pv_start(5Pn);

	/* NeSV(5NI(5Sn_dv_fx_arguments_oid);
}atruct l	"t and terminal emulation
	 */

	int		(*pv_fbwritecnar)(voidint	"ad	" char	*pv_echo;
	unsigned char	*pv_insource;
	unsipsid)m/5P0/5V(L(-l_dintwrints_	asr *buf);
	int (* t*pvb )0hoX	)ft (* t-l_dintwrints_	asr 0	asr 0*pv();
	int nitYegsion
	Get eap;

/* Tesbootstr;
	struct linrgumeenl;
	lod num_bytes;
enl;
	l t-l_dintwrints_	gs;
	unsigne);
	2);
	int		(*ar *virta, int which_io, u);
	2{
	stru5Nnices his i[15]ead)(intV0 ponVM	0x)(v4c/)(v4ponlpeeknextchar)(void int pv_romvers;
	unsigned int pv_plugid)(in  /* v, int h3 M_v0 d	"
#der	*0
#defin.ntV0 psteptM	0xcrazy.*pv_key joke. Call_SV(ap;/
}whigbtBEGVM	0xonlpponVMcpuad	"b
	uy*pv_kcrasp;/x_boomacifde, havnmap))(vo* V0 . :-) CONFIG_(inv3_cpuhar	*() whis har	*x_boocpua'nt (*cpu'nt (mmu-sfVtid)*pv_k' V0 sfVtid)' exec#defg    /* Tesb 'd	"gnsfugned' CONFIem_v0 {
3_cpuhar	*)(;
	struct linnt (*cpu int pers;bl_	*p,twr	
	int p V0 sfVtid)
	unsignd	"gnsfugnedugid)(in
3_cpuhaop() whis ca*/
}cpua'nt (*cpu'no;
	ode exec#defg*pv_kugnilt		Tes 0	}cpuacallM	0xmade. CONFIem_v0 {
3_cpuhaop)(;
	struct linnt (*cpuugid)(in
3_cpuidle() whis idle}cpua'nt (*cpu'nugnilt			ode or*pv_kTes 0	}cpuacallM	0xmade. CONFIem_v0 {
3_cpuidle)(;
	struct linnt (*cpuugid)(in
3_cpuTes 0	() whis Tes 0	}d	"
#der	*'nt (*cpu'nexec#defg*pv_khar	*efg wiv_fwh	int li'pc'_echo'npc'_wEGVMleft    _bo*pv_klavo*'idle' r	*'	ode'acall. CONFIem_v0 {
3_cpuTes 0	)(;
	struct linnt (*cpuugi}x_mem_v0 ;

	R
#definear	*ritv, inSV(ap;/
	char * (*v_seetruct linux_nodeop_wheee3)nsigned pno_oid)_whev_romv_whev;signed pno_cifldv_romv_whev;signed pno_
	cpoid)_romv_whe
	unsign	intv;signed pno_lin
	cp)_romv_whe
	unsign	int
	unsignedlv;signed pno_sin
	cp)_romv_whe
	unsign	int
	unsignedlhar)(void);
_write)(inno_oid)
	cp)_romv_whe
	unsign	into virtual MoGVM_iotions.There is noar	*r * (*vd	"bnSV. {
	char *ationsREG_MAX;

	i16	char *ationsVADDR_MAX;

16	char *ationsInsi_MAX;

	15char args[100];
	chid)gisned3)nsignednt (*v2_;oid);

	int	is* V0 prn OBIint (*v?_dev_write)	int_/* Th);

	int	/* N	intGER_B/* Tesb of* V0 pd)gisnedONFIem_v0d)g_pv_seoid);

	int	Howxmany rgv[8 does* V0 pd)gisnedOtake up?iagberr;ar args[100];
	chiirq3)nsigned(5N;__ASSEMBRQd(5NorityONFIem_v0 */
	s; (intV0 pinoar"b
chawh	i does*ii do?iagberr;

	Ele0	as of* Ve "ranges"0 */
	sruct linux_nodeop
	chidange