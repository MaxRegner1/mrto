/*************************************************************************/ /*!
@File           rgx_mips.h
@Title
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Platform       RGX
@Description    RGX MIPS definitions, kernel/user space
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(RGX_MIPS_H)
#define RGX_MIPS_H

/*
 * Utility defines for memory management
 */
#define RGXMIPSFW_LOG2_PAGE_SIZE_4K              (12)
#define RGXMIPSFW_PAGE_SIZE_4K                   (0x1 << RGXMIPSFW_LOG2_PAGE_SIZE_4K)
#define RGXMIPSFW_PAGE_MASK_4K                   (RGXMIPSFW_PAGE_SIZE_4K - 1)
#define RGXMIPSFW_LOG2_PAGE_SIZE_64K             (16)
#define RGXMIPSFW_PAGE_SIZE_64K                  (0x1 << RGXMIPSFW_LOG2_PAGE_SIZE_64K)
#define RGXMIPSFW_PAGE_MASK_64K                  (RGXMIPSFW_PAGE_SIZE_64K - 1)
#define RGXMIPSFW_LOG2_PAGE_SIZE_256K            (18)
#define RGXMIPSFW_PAGE_SIZE_256K                 (0x1 << RGXMIPSFW_LOG2_PAGE_SIZE_256K)
#define RGXMIPSFW_PAGE_MASK_256K                 (RGXMIPSFW_PAGE_SIZE_256K - 1)
#define RGXMIPSFW_LOG2_PAGE_SIZE_1MB             (20)
#define RGXMIPSFW_PAGE_SIZE_1MB                  (0x1 << RGXMIPSFW_LOG2_PAGE_SIZE_1MB)
#define RGXMIPSFW_PAGE_MASK_1MB                  (RGXMIPSFW_PAGE_SIZE_1MB - 1)
#define RGXMIPSFW_LOG2_PAGE_SIZE_4MB             (22)
#define RGXMIPSFW_PAGE_SIZE_4MB                  (0x1 << RGXMIPSFW_LOG2_PAGE_SIZE_4MB)
#define RGXMIPSFW_PAGE_MASK_4MB                  (RGXMIPSFW_PAGE_SIZE_4MB - 1)
#define RGXMIPSFW_LOG2_PTE_ENTRY_SIZE            (2)
/* log2 page table sizes dependent on FW heap size and page size (for each OS) */
#define RGXMIPSFW_LOG2_PAGETABLE_SIZE_4K         (RGX_FIRMWARE_HEAP_SHIFT - RGXMIPSFW_LOG2_PAGE_SIZE_4K + RGXMIPSFW_LOG2_PTE_ENTRY_SIZE)
#define RGXMIPSFW_LOG2_PAGETABLE_SIZE_64K        (RGX_FIRMWARE_HEAP_SHIFT - RGXMIPSFW_LOG2_PAGE_SIZE_64K + RGXMIPSFW_LOG2_PTE_ENTRY_SIZE)
/* Maximum log2 page table size (across OSes) */
#define RGXMIPSFW_MAX_LOG2_PAGETABLE_SIZE        (18) /* 256KB page table */
/* Maximum number of page table pages (both Host and MIPS pages) */
#define RGXMIPSFW_MAX_NUM_PAGETABLE_PAGES        (4)
/* Total number of TLB entries */
#define RGXMIPSFW_NUMBER_OF_TLB_ENTRIES          (16)
/* "Uncached" caching policy */
#define RGXMIPSFW_UNCACHED_CACHE_POLICY          (0X00000002)
/* "Write-back write-allocate" caching policy */
#define RGXMIPSFW_WRITEBACK_CACHE_POLICY         (0X00000003)
/* "Write-through no write-allocate" caching policy */
#define RGXMIPSFW_WRITETHROUGH_CACHE_POLICY      (0X00000001)
/* Cached policy used by MIPS in case of physical bus on 32 bit */
#define RGXMIPSFW_CACHED_POLICY                  (RGXMIPSFW_WRITEBACK_CACHE_POLICY)
/* Cached policy used by MIPS in case of physical bus on more than 32 bit */
#define RGXMIPSFW_CACHED_POLICY_ABOVE_32BIT      (RGXMIPSFW_WRITETHROUGH_CACHE_POLICY)
/* Total number of Remap entries */
#define RGXMIPSFW_NUMBER_OF_REMAP_ENTRIES        (2 * RGXMIPSFW_NUMBER_OF_TLB_ENTRIES)


/*
 * MIPS EntryLo/PTE format
 */

#define RGXMIPSFW_ENTRYLO_READ_INHIBIT_SHIFT     (31U)
#define RGXMIPSFW_ENTRYLO_READ_INHIBIT_CLRMSK    (0X7FFFFFFF)
#define RGXMIPSFW_ENTRYLO_READ_INHIBIT_EN        (0X80000000)

#define RGXMIPSFW_ENTRYLO_EXEC_INHIBIT_SHIFT     (30U)
#define RGXMIPSFW_ENTRYLO_EXEC_INHIBIT_CLRMSK    (0XBFFFFFFF)
#define RGXMIPSFW_ENTRYLO_EXEC_INHIBIT_EN        (0X40000000)

/* Page Frame Number */
#define RGXMIPSFW_ENTRYLO_PFN_SHIFT              (6)
#define RGXMIPSFW_ENTRYLO_PFN_ALIGNSHIFT         (12)
/* Mask used for the MIPS Page Table in case of physical bus on 32 bit */
#define RGXMIPSFW_ENTRYLO_PFN_MASK               (0x03FFFFC0)
#define RGXMIPSFW_ENTRYLO_PFN_SIZE               (20)
/* Mask used for the MIPS Page Table in case of physical bus on more than 32 bit */
#define RGXMIPSFW_ENTRYLO_PFN_MASK_ABOVE_32BIT   (0x3FFFFFC0)
#define RGXMIPSFW_ENTRYLO_PFN_SIZE_ABOVE_32BIT   (24)
#define RGXMIPSFW_ADDR_TO_ENTRYLO_PFN_RSHIFT     (RGXMIPSFW_ENTRYLO_PFN_ALIGNSHIFT - \
                                                  RGXMIPSFW_ENTRYLO_PFN_SHIFT)

#define RGXMIPSFW_ENTRYLO_CACHE_POLICY_SHIFT     (3U)
#define RGXMIPSFW_ENTRYLO_CACHE_POLICY_CLRMSK    (0XFFFFFFC7)

#define RGXMIPSFW_ENTRYLO_DIRTY_SHIFT            (2U)
#define RGXMIPSFW_ENTRYLO_DIRTY_CLRMSK           (0XFFFFFFFB)
#define RGXMIPSFW_ENTRYLO_DIRTY_EN               (0X00000004)

#define RGXMIPSFW_ENTRYLO_VALID_SHIFT            (1U)
#define RGXMIPSFW_ENTRYLO_VALID_CLRMSK           (0XFFFFFFFD)
#define RGXMIPSFW_ENTRYLO_VALID_EN               (0X00000002)

#define RGXMIPSFW_ENTRYLO_GLOBAL_SHIFT           (0U)
#define RGXMIPSFW_ENTRYLO_GLOBAL_CLRMSK          (0XFFFFFFFE)
#define RGXMIPSFW_ENTRYLO_GLOBAL_EN              (0X00000001)

#define RGXMIPSFW_ENTRYLO_DVG                    (RGXMIPSFW_ENTRYLO_DIRTY_EN | \
                                                  RGXMIPSFW_ENTRYLO_VALID_EN | \
                                                  RGXMIPSFW_ENTRYLO_GLOBAL_EN)
#define RGXMIPSFW_ENTRYLO_UNCACHED               (RGXMIPSFW_UNCACHED_CACHE_POLICY << \
                                                  RGXMIPSFW_ENTRYLO_CACHE_POLICY_SHIFT)
#define RGXMIPSFW_ENTRYLO_DVG_UNCACHED           (RGXMIPSFW_ENTRYLO_DVG | RGXMIPSFW_ENTRYLO_UNCACHED)


/* Remap Range Config Addr Out */
/* These defines refer to the upper half of the Remap Range Config register */
#define RGXMIPSFW_REMAP_RANGE_ADDR_OUT_MASK      (0x0FFFFFF0)
#define RGXMIPSFW_REMAP_RANGE_ADDR_OUT_SHIFT     (4)  /* wrt upper half of the register */
#define RGXMIPSFW_REMAP_RANGE_ADDR_OUT_ALIGNSHIFT (12)
#define RGXMIPSFW_ADDR_TO_RR_ADDR_OUT_RSHIFT     (RGXMIPSFW_REMAP_RANGE_ADDR_OUT_ALIGNSHIFT - \
                                                  RGXMIPSFW_REMAP_RANGE_ADDR_OUT_SHIFT)

#if defined(SECURE_FW_CODE_OSID) && (SECURE_FW_CODE_OSID + 1 > 2)
#define MIPS_FW_CODE_OSID                        (SECURE_FW_CODE_OSID)
#elif defined(SECURE_FW_CODE_OSID)
#define MIPS_FW_CODE_OSID                        (1U)
#endif


/*
 * Pages to trampoline problematic physical addresses:
 *   - RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN : 0x1FC0_0000
 *   - RGXMIPSFW_DATA_REMAP_PHYS_ADDR_IN : 0x1FC0_1000
 *   - RGXMIPSFW_CODE_REMAP_PHYS_ADDR_IN : 0x1FC0_2000
 *   - (benign trampoline)               : 0x1FC0_3000
 * that would otherwise be erroneously remapped by the MIPS wrapper
 * (see "Firmware virtual layout and remap configuration" section below)
 */

#define RGXMIPSFW_TRAMPOLINE_LOG2_NUMPAGES       (2)
#define RGXMIPSFW_TRAMPOLINE_NUMPAGES            (1 << RGXMIPSFW_TRAMPOLINE_LOG2_NUMPAGES)
#define RGXMIPSFW_TRAMPOLINE_SIZE                (RGXMIPSFW_TRAMPOLINE_NUMPAGES << RGXMIPSFW_LOG2_PAGE_SIZE_4K)
#define RGXMIPSFW_TRAMPOLINE_LOG2_SEGMENT_SIZE   (RGXMIPSFW_TRAMPOLINE_LOG2_NUMPAGES + RGXMIPSFW_LOG2_PAGE_SIZE_4K)

#define RGXMIPSFW_TRAMPOLINE_TARGET_PHYS_ADDR    (RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN)
#define RGXMIPSFW_TRAMPOLINE_OFFSET(a)           (a - RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN)

#define RGXMIPSFW_SENSITIVE_ADDR(a)              (RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN == (~((1<<RGXMIPSFW_TRAMPOLINE_LOG2_SEGMENT_SIZE)-1) & a))

/*
 * Firmware virtual layout and remap configuration
 */
/*
 * For each remap region we define:
 * - the virtual base used by the Firmware to access code/data through that region
 * - the microAptivAP physical address correspondent to the virtual base address,
 *   used as input address and remapped to the actual physical address
 * - log2 of size of the region remapped by the MIPS wrapper, i.e. number of bits from
 *   the bottom of the base input address that survive onto the output address
 *   (this defines both the alignment and the maximum size of the remapped region)
 * - one or more code/data segments within the remapped region
 */

/* Boot remap setup */
#define RGXMIPSFW_BOOT_REMAP_VIRTUAL_BASE        (0xBFC00000)
#define RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN        (0x1FC00000)
#define RGXMIPSFW_BOOT_REMAP_LOG2_SEGMENT_SIZE   (12)
#define RGXMIPSFW_BOOT_NMI_CODE_VIRTUAL_BASE     (RGXMIPSFW_BOOT_REMAP_VIRTUAL_BASE)

/* Data remap setup */
#define RGXMIPSFW_DATA_REMAP_VIRTUAL_BASE        (0xBFC01000)
#define RGXMIPSFW_DATA_CACHED_REMAP_VIRTUAL_BASE (0x9FC01000)
#define RGXMIPSFW_DATA_REMAP_PHYS_ADDR_IN        (0x1FC01000)
#define RGXMIPSFW_DATA_REMAP_LOG2_SEGMENT_SIZE   (12)
#define RGXMIPSFW_BOOT_NMI_DATA_VIRTUAL_BASE     (RGXMIPSFW_DATA_REMAP_VIRTUAL_BASE)

/* Code remap setup */
#define RGXMIPSFW_CODE_REMAP_VIRTUAL_BASE        (0x9FC02000)
#define RGXMIPSFW_CODE_REMAP_PHYS_ADDR_IN        (0x1FC02000)
#define RGXMIPSFW_CODE_REMAP_LOG2_SEGMENT_SIZE   (12)
#define RGXMIPSFW_EXCEPTIONS_VIRTUAL_BASE        (RGXMIPSFW_CODE_REMAP_VIRTUAL_BASE)

/* Permanent mappings setup */
#define RGXMIPSFW_PT_VIRTUAL_BASE                (0xCF000000)
#define RGXMIPSFW_REGISTERS_VIRTUAL_BASE         (0xCF800000)
#define RGXMIPSFW_STACK_VIRTUAL_BASE             (0xCF600000)


/*
 * Bootloader configuration data
 */
/* Bootloader configuration offset (where RGXMIPSFW_BOOT_DATA lives)
 * within the bootloader/NMI data page */
#define RGXMIPSFW_BOOTLDR_CONF_OFFSET                         (0x0)


/*
 * NMI shared data
 */
/* Base address of the shared data within the bootloader/NMI data page */
#define RGXMIPSFW_NMI_SHARED_DATA_BASE                        (0x100)
/* Size used by Debug dump data */
#define RGXMIPSFW_NMI_SHARED_SIZE                             (0x2B0)
/* Offsets in the NMI shared area in 32-bit words */
#define RGXMIPSFW_NMI_SYNC_FLAG_OFFSET                        (0x0)
#define RGXMIPSFW_NMI_STATE_OFFSET                            (0x1)
#define RGXMIPSFW_NMI_ERROR_STATE_SET                         (0x1)

/*
 * MIPS boot stage
 */
#define RGXMIPSFW_BOOT_STAGE_OFFSET                           (0x400)

/*
 * MIPS private data in the bootloader data page.
 * Memory below this offset is used by the FW only, no interface data allowed.
 */
#define RGXMIPSFW_PRIVATE_DATA_OFFSET                         (0x800)


/* The things that follow are excluded when compiling assembly sources */
#if !defined(RGXMIPSFW_ASSEMBLY_CODE)
#include "img_types.h"
#include "km/rgxdefs_km.h"

typedef struct
{
	IMG_UINT64 ui64StackPhyAddr;
	IMG_UINT64 ui64RegBase;
	IMG_UINT64 aui64PTPhyAddr[RGXMIPSFW_MAX_NUM_PAGETABLE_PAGES];
	IMG_UINT32 ui32PTLog2PageSize;
	IMG_UINT32 ui32PTNumPages;
	IMG_UINT32 ui32Reserved1;
	IMG_UINT32 ui32Reserved2;
} RGXMIPSFW_BOOT_DATA;

#define RGXMIPSFW_GET_OFFSET_IN_DWORDS(offset)                (offset / sizeof(IMG_UINT32))
#define RGXMIPSFW_GET_OFFSET_IN_QWORDS(offset)                (offset / sizeof(IMG_UINT64))

/* Used for compatibility checks */
#define RGXMIPSFW_ARCHTYPE_VER_CLRMSK                         (0xFFFFE3FFU)
#define RGXMIPSFW_ARCHTYPE_VER_SHIFT                          (10U)
#define RGXMIPSFW_CORE_ID_VALUE                               (0x001U)
#define RGXFW_PROCESSOR_MIPS                                  "MIPS"

/* microAptivAP cache line size */
#define RGXMIPSFW_MICROAPTIVEAP_CACHELINE_SIZE                (16U)

/* The SOCIF transactions are identified with the top 16 bits of the physical address emitted by the MIPS */
#define RGXMIPSFW_WRAPPER_CONFIG_REGBANK_ADDR_ALIGN           (16U)

/* Values to put in the MIPS selectors for performance counters */
#define RGXMIPSFW_PERF_COUNT_CTRL_ICACHE_ACCESSES_C0          (9U)   /* Icache accesses in COUNTER0 */
#define RGXMIPSFW_PERF_COUNT_CTRL_ICACHE_MISSES_C1            (9U)   /* Icache misses in COUNTER1 */

#define RGXMIPSFW_PERF_COUNT_CTRL_DCACHE_ACCESSES_C0          (10U)  /* Dcache accesses in COUNTER0 */
#define RGXMIPSFW_PERF_COUNT_CTRL_DCACHE_MISSES_C1            (11U) /* Dcache misses in COUNTER1 */

#define RGXMIPSFW_PERF_COUNT_CTRL_ITLB_INSTR_ACCESSES_C0      (5U)  /* ITLB instruction accesses in COUNTER0 */
#define RGXMIPSFW_PERF_COUNT_CTRL_JTLB_INSTR_GXMIPSFW_PERF_COUW&6PW_PERF_COUyOS TW&WG#define RGXMIPSFW_REGISTERS_VIRTUAL_BASE         (0xCF8000EGB63oW&WG00000)
#define RGXMIPSFW_BOOT_REMAP_PHYS_ADDR_IN        (064F   (11U) /* Dcache misses in COUNTER1 Jy_LOu.defs_km.h"

typeF5&WGine RGXMIPSFW_BOOT_NMI_CODE_VIRTUAL_BASE     (RGXMIPSFW_BO63TORT O|8oW&4FFC7)

a5&WG Data remap setup */
#define RGXMIPSFW_DATA_REMAP_VIRTUAL_6WG/* Bootloader configuration offset (where RGXMIPSFW_BOOT_Dache missesiEGMENT_SIZE   (12)
#define RGXMIPSFW_BOOT_NMI_DATA_VIRTUALG    (0x1FC01000)
#define RGXMIPSFW_DATA_REMAP_LOG2_SEGMENT6VoeU) /* Dcache misses in COUNTERAP_VIRTUAL_BASE        (0x9FC02000)
#define RGXMIPSFW_CODEDATAW&WG00)
#define RGXMIPSFW_CODE_REMAP_PHYS_ADDR_IN        (0x1FGBASE        (0x9FC02000)
#define RGXMIPSFW_CODE_REMAP_PHYS6/L(reRAP_PHYS6/L8G1FC02000)
#define RGXMIPSFW_CODE_REMAP_LOG2_SEGMENT_SIZE  6WGFW_CODE_REMAP_VIRTUAL_BASE)

/* Permanent mappings setup *G            (0xCF000000)
#define RGXMIPSFW_REGISTERS_VIRTUG/
#define RGXMIPSFW_PT_VIRTUAL_BASE                (0xCF006.    6WG           (0x001U)
#define RGXFW_PROCESSOR_MIPS          A
#define RGXMIP
e RGXFxCF   W&0XMIPSF(0xCF006. Za5&.XMIne aZaMIP
e RXMIefin  W&0XMIPSF(0xCF006fine (IPSFW_REu1)
#dC1RGXMIPSFu          (0x001U)
#0u4EaUWIu10U1A_OFOCset (whhEIPS_DAT
fOuOAHu4EaUWIu10U1A_OFOCset (whhEIPS_r0OhCeENT_SIZE  6WGFW_COD OCset (whhEIPS_r0OhCeENTproATA_(0xUnus#defi   end/or u          (0x001U)
#0u4EaUWIuUNUSine RUNITDAT
fOuOS_DAT
fOuOIu108uOAHu43U1A_OFOCset (whhEIPS_r0OhCeENPEN****_W&0XMDAT
fOuOS_DAT
fOuOAHu43fcXMIOIu11W_BOOT_DATA lives)
 * r0OhCeENFDCIPEN****(0xCF006. Za_SIZ2if !defined(RGXMIPSFW r0OhCeENIVW&0XMIPSF(0xCF006. Za_SIZ23U1A_OFOCset (whhEIPS_r0OhCeENICW&0XMIPSF(0xCF006. Za_SIZ25U1A_OFOCset (whhEIPS_r0OhCeENPCIPEN****(0xCF006.  Za_SIZ2ne RGXMIPSFW_ADDR_TO__r0OhCeENTIPEN****(0xCF006.   Za_SIZ3W_BOOT_DATA lives)
 * r0OhCeENBRANCH_DELAYCF006.   Za_SIZ3 "km/rxCF006fine (IPSFW_REuPS pri1RGXMIPSFu          (0x001U)
#0u4DEBUGu10U1A_OFDEBUGuOS_DDEBUGuOIu11W_OAHu41f_BOOT_DATA lives)
 * r0DEBUGuDSR_IN)

#define RGXMW_BOOT_DATA lives)
 * r0DEBUGuDBP_IN)

#define RGXMif !defined(RGXMIPSFW r0DEBUGuDDBLIN)

#define RGXM0U1A_OFOCset (whhEIPS_r0DEBUGuDDBSIN)

#define RGXM3U1A_OFOCset (whhEIPS_r0DEBUGuDIB_IN)

#define RGXM(0X00000004)

#define_r0DEBUGuDINTIN)

#define RGXM5U1A_OFOCset (whhEIPS_r0DEBUGuDIBIMPR

#define RGXMne RGXMIPSFW_ADDR_TO__r0DEBUGuDDBLIMPR

#defie RGXMi RGXMIPSFW_PAGE_SIZE__r0DEBUGuDDBSIMPR

#defie RGXMi9RGXMIPSFW_PAGE_SIZE__r0DEBUGuIEXIIN)

#define RGXM0W_BOOT_DATA lives)
 * r0DEBUGuDBCeEP)

#define RGXM0if !defined(RGXMIPSFW r0DEBUGueF5&WEP)

#defie RGXM00U1A_OFOCset (whhEIPS_r0DEBUGuM5&WCKP)

#defie RGXM03U1A_OFOCset (whhEIPS_r0DEBUGuIBCeEP)

#define RGXM0(0X00000004)

#define_r0DEBUGuDM_IN)

#defin Za_SIZ3W_BOOT_DATA lives)
 * r0DEBUGuDBDIN)

#defin Za_SIZ3 "km/rxCF006fine (IPSFW_it */
#define RGXMIPSFW_CACHED_PO#defROAP    (G2_PTE_EN (whhEIPS_DG2_PTE_EN (Iu113_OAHuRGXMIOR_MIPS          A
#d#defROAP used by (G2_PTE_EN (w(S_DG2_PTE_EN (|MENT_FF)3000 (Iu111DE_VIe table paEMAKBne RGXMIPSFW_CACHED_PO#defROAPG2_PTE_EN( used by  (w(S_DG2_PT by  (GXMi1DEss OOAH~u4EFF)3_VIe table paEMAKBne RGXMIPSFW_CACHED_PO#defROAPVPN2(ack wrHI         ((ack wrHI  Iu113_RGXMIPSFW_CACHED_PO#defROAPCOHERENCY(ack wrLO)EIPS_Dack wrLO)EIu13uOAHu4EOR_MIPS          A
#d#defROAP FN(ack wrLO)EIPwhhEIPS_Dack wrLO)EIu16_OAHuRGXMIIOR__VIROAPG2e;
	NT_ non-isiodard  FN   sk  (0x3     MPOLINE_NUne RGXMIPSFW_CACHED_PO#defROAPG2(ack wrLO)EIPwhhEIPPS_Dtransaction(ack wrLO)E&ne RGXMIPSFW_ENTRYLO_DIRTY_CLRMSK      )RGXMne RGXMIPSFW_ADDR_TO_#defROAPFW_ENTR(ack wrLO)EIPwhS_Dack wrLO)EIu13W_OAHu4LOBAL_SHIFT           #defROAPDGV(ack wrLO)EIPwhhEIPS_ack wrLO)E&nu4EOR_MIPS          A
#d#defR0x0FF                    TRYLO_VALID_EN | \
    #def Rema                     TRYRGXMif !defined(RGXMIPSFW#defi* Re                     TRYRGXM     (a - RGXMIPSFW_BOdefXIIN)

#defin             TRYRGXM3W_BOOT_DATA lives)
 *OdefRIIN)

#defin             TRYRGXM3CACHED           (RGXM (0x0)ROAPHAREODIRTY_(HAREODIRTY_FW_1A_***) Za_SIZ((HAREODIRTY_FW_1A_***3000 (GXMif)NT64))

/* Used  {                 OdeefinLICY                  OdeHi                  OdeLo0                  OdeLo1 (0x001      ENTRYLO_Y;NT64))

/* Used  {                 SID)     In;     _VIalways 4kDATA_R   e R                SID)     Out;    _VIalways 4kDATA_R   e R                SID) R RGXM      0x001      0)

/* PagY;NT64))

/* Used  {                 ErrorState; efineiIRTustL_BAE, ArsTER1 */

 Used ure e R                Er