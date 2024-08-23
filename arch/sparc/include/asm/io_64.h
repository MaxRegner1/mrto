/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SPARC64_IO_H
#define __SPARC64_IO_H

#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/types.h>

#include <asm/page.h>      /* IO address mapping routines need this */
#include <asm/asi.h>
#include <asm-generic/pci_iomap.h>

/* BIO layer definitions. */
extern unsigned long kern_base, kern_size;

/* __raw_{read,write}{b,w,l,q} uses direct access.
 * Access the memory as big endian bypassing the cache
 * by using ASI_PHYS_BYPASS_EC_E
 */
#define __raw_readb __raw_readb
static inline u8 __raw_readb(const volatile void __iomem *addr)
{
	u8 ret;

	__asm__ __volatile__("lduba\t[%1] %2, %0\t/* pci_raw_readb */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));

	return ret;
}

#define __raw_readw __raw_readw
static inline u16 __raw_readw(const volatile void __iomem *addr)
{
	u16 ret;

	__asm__ __volatile__("lduha\t[%1] %2, %0\t/* pci_raw_readw */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));

	return ret;
}

#define __raw_readl __raw_readl
static inline u32 __raw_readl(const volatile void __iomem *addr)
{
	u32 ret;

	__asm__ __volatile__("lduwa\t[%1] %2, %0\t/* pci_raw_readl */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));

	return ret;
}

#define __raw_readq __raw_readq
static inline u64 __raw_readq(const volatile void __iomem *addr)
{
	u64 ret;

	__asm__ __volatile__("ldxa\t[%1] %2, %0\t/* pci_raw_readq */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));

	return ret;
}

#define __raw_writeb __raw_writeb
static inline void __raw_writeb(u8 b, const volatile void __iomem *addr)
{
	__asm__ __volatile__("stba\t%r0, [%1] %2\t/* pci_raw_writeb */"
			     : /* no outputs */
			     : "Jr" (b), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));
}

#define __raw_writew __raw_writew
static inline void __raw_writew(u16 w, const volatile void __iomem *addr)
{
	__asm__ __volatile__("stha\t%r0, [%1] %2\t/* pci_raw_writew */"
			     : /* no outputs */
			     : "Jr" (w), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));
}

#define __raw_writel __raw_writel
static inline void __raw_writel(u32 l, const volatile void __iomem *addr)
{
	__asm__ __volatile__("stwa\t%r0, [%1] %2\t/* pci_raw_writel */"
			     : /* no outputs */
			     : "Jr" (l), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));
}

#define __raw_writeq __raw_writeq
static inline void __raw_writeq(u64 q, const volatile void __iomem *addr)
{
	__asm__ __volatile__("stxa\t%r0, [%1] %2\t/* pci_raw_writeq */"
			     : /* no outputs */
			     : "Jr" (q), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E));
}

/* Memory functions, same as I/O accesses on Ultra.
 * Access memory as little endian bypassing
 * the cache by using ASI_PHYS_BYPASS_EC_E_L
 */
#define readb readb
#define readb_relaxed readb
static inline u8 readb(const volatile void __iomem *addr)
{	u8 ret;

	__asm__ __volatile__("lduba\t[%1] %2, %0\t/* pci_readb */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");
	return ret;
}

#define readw readw
#define readw_relaxed readw
static inline u16 readw(const volatile void __iomem *addr)
{	u16 ret;

	__asm__ __volatile__("lduha\t[%1] %2, %0\t/* pci_readw */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");

	return ret;
}

#define readl readl
#define readl_relaxed readl
static inline u32 readl(const volatile void __iomem *addr)
{	u32 ret;

	__asm__ __volatile__("lduwa\t[%1] %2, %0\t/* pci_readl */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");

	return ret;
}

#define readq readq
#define readq_relaxed readq
static inline u64 readq(const volatile void __iomem *addr)
{	u64 ret;

	__asm__ __volatile__("ldxa\t[%1] %2, %0\t/* pci_readq */"
			     : "=r" (ret)
			     : "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");

	return ret;
}

#define writeb writeb
#define writeb_relaxed writeb
static inline void writeb(u8 b, volatile void __iomem *addr)
{
	__asm__ __volatile__("stba\t%r0, [%1] %2\t/* pci_writeb */"
			     : /* no outputs */
			     : "Jr" (b), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");
}

#define writew writew
#define writew_relaxed writew
static inline void writew(u16 w, volatile void __iomem *addr)
{
	__asm__ __volatile__("stha\t%r0, [%1] %2\t/* pci_writew */"
			     : /* no outputs */
			     : "Jr" (w), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");
}

#define writel writel
#define writel_relaxed writel
static inline void writel(u32 l, volatile void __iomem *addr)
{
	__asm__ __volatile__("stwa\t%r0, [%1] %2\t/* pci_writel */"
			     : /* no outputs */
			     : "Jr" (l), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");
}

#define writeq writeq
#define writeq_relaxed writeq
static inline void writeq(u64 q, volatile void __iomem *addr)
{
	__asm__ __volatile__("stxa\t%r0, [%1] %2\t/* pci_writeq */"
			     : /* no outputs */
			     : "Jr" (q), "r" (addr), "i" (ASI_PHYS_BYPASS_EC_E_L)
			     : "memory");
}

#define inb inb
static inline u8 inb(unsigned long addr)
{
	return readb((volatile void __iomem *)addr);
}

#define inw inw
static inline u16 inw(unsigned long addr)
{
	return readw((volatile void __iomem *)addr);
}

#define inl inl
static inline u32 inl(unsigned long addr)
{
	return readl((volatile void __iomem *)addr);
}

#define outb outb
static inline void outb(u8 b, unsigned long addr)
{
	writeb(b, (volatile void __iomem *)addr);
}

#define outw outw
static inline void outw(u16 w, unsigned long addr)
{
	writew(w, (volatile void __iomem *)addr);
}

#define outl outl
static inline void outl(u32 l, unsigned long addr)
{
	writel(l, (volatile void __iomem *)addr);
}


#define inb_p(__addr) 		inb(__addr)
#define outb_p(__b, __addr)	outb(__b, __addr)
#define inw_p(__addr)		inw(__addr)
#define outw_p(__w, __addr)	outw(__w, __addr)
#define inl_p(__addr)		inl(__addr)
#define outl_p(__l, __addr)	outl(__l, __addr)

void outsb(unsigned long, const void *, unsigned long);
void outsw(unsigned long, const void *, unsigned long);
void outsl(unsigned long, const void *, unsigned long);
void insb(unsigned long, void *, unsigned long);
void insw(unsigned long, void *, unsigned long);
void insl(unsigned long, void *, unsigned long);

static inline void ioread8_rep(void __iomem *port, void *buf, unsigned long count)
{
	insb((unsigned long __force)port, buf, count);
}
static inline void ioread16_rep(void __iomem *port, void *buf, unsigned long count)
{
	insw((unsigned long __force)port, buf, count);
}

static inline void ioread32_rep(void __iomem *port, void *buf, unsigned long count)
{
	insl((unsigned long __force)port, buf, count);
}

static inline void iowrite8_rep(void __iomem *port, const void *buf, unsigned long count)
{
	outsb((unsigned long __force)port, buf, count);
}

static inline void iowrite16_rep(void __iomem *port, const void *buf, unsigned long count)
{
	outsw((unsigned long __force)port, buf, count);
}

static inline void iowrite32_rep(void __iomem *port, const void *buf, unsigned long count)
{
	outsl((unsigned long __force)port, buf, count);
}

/* Valid I/O Space regions are anyw+t);
}

static inline void iowrite32_rep(void __iomem *port, const void *buf, unsigned long count)
{
	outsl((unsignKU
Ev
Eginuf, unsigned long count)
{
	outsl((unliavSpace regions are anyw+t);
}

static inline void iowrite32_rep(void __iomem *port, const void *buf, unsigned long KU
Ev
Ev
EAeRlxaxedwva)nt p(g KU
E inline void writeq(u64 q, volatile void __iomem :sicalng ASI_PHrange.et;
}

#definIOkernCunsIMIT 0xffffffffffffffffUL, conNo, vSBUS variants, only diff
Evn unfrom)nt pi : "at we doe vonot
			" (addr-, "i" (tics.et;
} void __iomem *)as(g void __iomem *addr)
{
	u16 ret;

	__asm__ __vfine outile void __nst void *volatile void __ios(g void  __iomem *addr)
{
	u32 ret;

	__asm__ __fine outile void w_nst void *volatile void _l_ps(g void  __iomem *addr)
{
	u64 ret;

	__asm__ __fine outile void l_nst void *volatile void _vols(g void tile void __iomem *addr)
{
	__asm__ __vofine outile void q_nst void *volatile void  *adds(g void writew(u16 w, volatile void __iomem *addr)
onst volati(u1nst void *volatile void  *adds(g void wwritel(u32 l, volatile void __iomem *addr)
const volatwu1nst void *volatile void  *adds(g void wwriteq(u64 q, volatile void __iomem *addr)
const volatlu1nst void *volatile void  *adds(g void wb((volatile void __iomem *)addr);
}

#definittle endiaqu1nst void *volatile void  *adds(g vaddset *)d outsb(unsigned long, cdsoidint csl(ue}{b,wuba\t_t nl, __ahb(u(n--)  __	s(g void wricsldsoory"	dso++ry"}id *volatile void  *addaddset *)d outsb(unsigned long, cdsoidint csl(ue}{b,wuba\t_t nl, __ outsb(unsigned long, cd =ldso		  ahb(u (n--)  __	oid wricsldory"	d++ry"}id *volatile void  *adds(g vaddcpy_from*)d ovSpadsoidle void __iomem *addr)
{
	__asrc,gnedd longl(ue}{b,wuba\t_t nl, __char cd =ldso		  ahb(u (n--)  __	char tmp =ls(g void __srcory"	*d++ =ltmpry"	src++ry"}id **volatile void  *addaddcpy_from*)d ovSpadsoidle void __iomem *addr)
{
	__asrc,gnedd (ue}{b,wuba\t_t nl, __char cd =ldso		  ahb(u (n--)  __	char tmp =loid __srcory"	*d++ =ltmpry"	src++ry"}id *volatile void  *adds(g vaddcpy_to*)d outsb(unsigned long, cdsoidl((unliavSpasrc,gnedd lon(ue}{b,wuba\t_t nl, __c((unlchar cs =lsrc;__ outsb(unsigned long, cd =ldso		  ahb(u (n--)  __	char tmp =l*s++ry"	s(g void writmpsldory"	d++ry"}id *volatile void  *addaddcpy_to*)d outsb(unsigned long, cdsoidl((unliavSpasrc,gned  longl(ue}{b,wuba\t_t nl, __c((unlchar cs =lsrc;__ outsb(unsigned long, cd =ldso		  ahb(u (n--)  __	char tmp =l*s++ry"	oid writmpsldory"	d++ry"}id *

#definmm*)wb()dianfrn unsKERNEL__, conOn sparcvolwe hateqiomewholmem :sicalnsing ASI_PHsf, un */"
	ibl	retu	     m :sically g ASI_Poid iad londdstite0\t/oefine do
 */ofinng.et;
} void __iomem signed long, critemapnline void iowroffsetregions are anywba\t __vofine oud long count)
{)offsetned long addr)itemap_noC_E_L(X,Yinswitemapn(X),(Y))long addr)itemap_wc(X,Yinsswitemapn(X),(Y))long addr)itemap_wt(X,Yinsswitemapn(X),(Y))lutsl((unsignKU
Ev
Eginunmapnle void __iomem *)addr);
}

#ded long addr)ited _nssw readw
#definep(void ssw rewdw
#definep(void be		line u32 __rw
#definep(voi32ssw relrw
#definep(voi32be		line u32 _lrw
#definepid iow"	oid wrrw
#definepid iod ssitel writel
#deepid iod be		line uitel writel
#deepid io32ssiteq writeq
#deepid io32be		line uitel l, conC32 tun  virtualnYS_BYPAScookiunfor64 qsin inlHranget;
}signed long, cri inl_mapnline void iowrp(voidline voidint nrigned loni inl_unmapnleong count)
{);, conC32 tun  virtualnYS_BYPAScookiunfor64)nt pBAR (     : or6IO)t;
} vructong adevgned lo_readbunmapn vructong adev cdev,nsigned long, coline void ioread3int s(g vid _dma_64volnleon __vofine ou1 inline void iowriint s(g vid _burst64nleon __vofine ou1 inlinructodevicegned los(g vset s(g 64ninru