/*************************************************************************/ /*!
@File
@Title          Rogue firmware utility routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Rogue firmware utility routines
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

#if defined(LINUX)
#include <linux/stddef.h>
#else
#include <stddef.h>
#endif

#include "img_defs.h"

#include "rgxdefs_km.h"
#include "rgx_fwif_km.h"
#include "pdump_km.h"
#include "osfunc.h"
#include "oskm_apphint.h"
#include "cache_km.h"
#include "allocmem.h"
#include "physheap.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "devicemem_server.h"

#include "pvr_debug.h"
#include "pvr_notifier.h"
#include "rgxfwutils.h"
#include "rgx_options.h"
#include "rgx_fwif_alignchecks.h"
#include "rgx_fwif_resetframework.h"
#include "rgx_pdump_panics.h"
#include "fwtrace_string.h"
#include "rgxheapconfig.h"
#include "pvrsrv.h"
#include "rgxdebug.h"
#include "rgxhwperf.h"
#include "rgxccb.h"
#include "rgxcompute.h"
#include "rgxtransfer.h"
#include "rgxpower.h"
#include "rgxtdmtransfer.h"
#if defined(SUPPORT_DISPLAY_CLASS)
#include "dc_server.h"
#endif
#include "rgxmem.h"
#include "rgxta3d.h"
#include "rgxkicksync.h"
#include "rgxutils.h"
#include "rgxtimecorr.h"
#include "sync_internal.h"
#include "sync.h"
#include "sync_checkpoint.h"
#include "sync_checkpoint_external.h"
#include "tlstream.h"
#include "devicemem_server_utils.h"
#include "htbuffer.h"
#include "rgx_bvnc_defs_km.h"
#include "info_page.h"

#include "physmem_lma.h"
#include "physmem_osmem.h"

#ifdef __linux__
#include <linux/kernel.h>	/* sprintf */
#include <linux/string.h>	/* strncpy, strlen */
#include "rogue_trace_events.h"
#else
#include <stdio.h>
#include <string.h>
#endif
#if defined(PVRSRV_ENABLE_PROCESS_STATS)
#include "process_stats.h"
#endif

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
#include "rgxworkest.h"
#endif

#if defined(SUPPORT_PDVFS)
#include "rgxpdvfs.h"
#endif

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
#if defined(SUPPORT_BUFFER_SYNC)
#include "pvr_buffer_sync.h"
#endif
#endif

#include "vz_support.h"
#include "vz_physheap.h"
#include "rgx_heaps.h"

#if defined(MTK_GPU_BM_SUPPORT)
#include <gpu_bm.h>
#endif

/*!
 ******************************************************************************
 * HWPERF
 *****************************************************************************/
/* Size of the Firmware L1 HWPERF buffer in bytes (2MB). Accessed by the
 * Firmware and host driver. */
#define RGXFW_HWPERF_L1_SIZE_MIN        (16U)
#define RGXFW_HWPERF_L1_SIZE_DEFAULT    PVRSRV_APPHINT_HWPERFFWBUFSIZEINKB
#define RGXFW_HWPERF_L1_SIZE_MAX        (12288U)

/* Kernel CCB length */
/* Reducing the size of the KCCB in an attempt to avoid flooding and overflowing the FW kick queue
 * in the case of multiple OSes */
#define RGXFWIF_KCCB_NUMCMDS_LOG2_GPUVIRT_WITHOUT_FEATURE  (6)
#define RGXFWIF_KCCB_NUMCMDS_LOG2_DEFAULT                  (7)


/* Firmware CCB length */
#if defined(SUPPORT_PDVFS)
#define RGXFWIF_FWCCB_NUMCMDS_LOG2   (8)
#else
#define RGXFWIF_FWCCB_NUMCMDS_LOG2   (5)
#endif

#if defined(RGX_FW_IRQ_OS_COUNTERS)
const IMG_UINT32 gaui32FwOsIrqCntRegAddr[RGXFW_MAX_NUM_OS] = {IRQ_COUNTER_STORAGE_REGS};
#endif

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
/* Checkpoint CCB length */
#define RGXFWIF_CHECKPOINTCCB_NUMCMDS_LOG2 (10)
#endif

/* Workload Estimation Firmware CCB length */
#define RGXFWIF_WORKEST_FWCCB_NUMCMDS_LOG2   (7)

/* Size of memory buffer for firmware gcov data
 * The actual data size is several hundred kilobytes. The buffer is an order of magnitude larger. */
#define RGXFWIF_FIRMWARE_GCOV_BUFFER_SIZE (4*1024*1024)

typedef struct
{
	RGXFWIF_KCCB_CMD        sKCCBcmd;
	DLLIST_NODE             sListNode;
	PDUMP_FLAGS_T           uiPdumpFlags;
	PVRSRV_RGXDEV_INFO      *psDevInfo;
	RGXFWIF_DM              eDM;
} RGX_DEFERRED_KCCB_CMD;

#if defined(PDUMP)
/* ensure PIDs are 32-bit because a 32-bit PDump load is generated for the
 * PID filter example entries
 */
static_assert(sizeof(IMG_PID) == sizeof(IMG_UINT32),
		"FW PID filtering assumes the IMG_PID type is 32-bits wide as it "
		"generates WRW commands for loading the PID values");
#endif

static PVRSRV_ERROR _AllocateSLC3Fence(PVRSRV_RGXDEV_INFO* psDevInfo, RGXFWIF_INIT* psRGXFWInit)
{
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC** ppsSLC3FenceMemDesc = &psDevInfo->psSLC3FenceMemDesc;
	IMG_UINT32	ui32CacheLineSize = GET_ROGUE_CACHE_LINE_SIZE(
			RGX_GET_FEATURE_VALUE(psDevInfo, SLC_CACHE_LINE_SIZE_BITS));

	PVR_DPF_ENTERED;

	eError = DevmemAllocate(psDevInfo->psFirmwareMainHeap,
			1,
			ui32CacheLineSize,
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_FW_LOCAL,
			"FwSLC3FenceWA",
			ppsSLC3FenceMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF_RETURN_RC(eError);
	}

	/* We need to map it so the heap for this allocation is set */
	eError = DevmemMapToDevice(*ppsSLC3FenceMemDesc,
			psDevInfo->psFirmwareMainHeap,
			&psRGXFWInit->sSLC3FenceDevVAddr);
	if (eError != PVRSRV_OK)
	{
		DevmemFwFree(psDevInfo, *ppsSLC3FenceMemDesc);
		*ppsSLC3FenceMemDesc = NULL;
	}

	PVR_DPF_RETURN_RC1(eError, *ppsSLC3FenceMemDesc);
}

static void _FreeSLC3Fence(PVRSRV_RGXDEV_INFO* psDevInfo)
{
	DEVMEM_MEMDESC* psSLC3FenceMemDesc = psDevInfo->psSLC3FenceMemDesc;

	if (psSLC3FenceMemDesc)
	{
		DevmemReleaseDevVirtAddr(psSLC3FenceMemDesc);
		DevmemFree(psSLC3FenceMemDesc);
	}
}

static void __MTSScheduleWrite(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT32 ui32Value)
{
	/* ensure memory is flushed before kicking MTS */
	OSWriteMemoryBarrier();

	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_MTS_SCHEDULE, ui32Value);

	/* ensure the MTS kick goes through before continuing */
	OSMemoryBarrier();
}


/*!
 *******************************************************************************
 @Function		RGXFWSetupSignatureChecks
 @Description
 @Input			psDevInfo

 @Return		PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXFWSetupSignatureChecks(PVRSRV_RGXDEV_INFO* psDevInfo,
		DEVMEM_MEMDESC**    ppsSigChecksMemDesc,
		IMG_UINT32          ui32SigChecksBufSize,
		RGXFWIF_SIGBUF_CTL* psSigBufCtl,
		const IMG_CHAR*     pszBufferName)
{
	PVRSRV_ERROR	eError;
	DEVMEM_FLAGS_T	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for %s signature checks", pszBufferName);
	eError = DevmemFwAllocate(psDevInfo,
			ui32SigChecksBufSize,
			uiMemAllocFlags,
			"FwSignatureChecks",
			ppsSigChecksMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for signature checks (%u)",
				ui32SigChecksBufSize,
				eError));
		return eError;
	}

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(&psSigBufCtl->sBuffer,
			*ppsSigChecksMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	DevmemPDumpLoadMem(	*ppsSigChecksMemDesc,
			0,
			ui32SigChecksBufSize,
			PDUMP_FLAGS_CONTINUOUS);

	psSigBufCtl->ui32LeftSizeInRegs = ui32SigChecksBufSize / sizeof(IMG_UINT32);

	return PVRSRV_OK;
}

#if defined(SUPPORT_FIRMWARE_GCOV)
/*!
 *******************************************************************************
 @Function		RGXFWSetupFirmwareGcovBuffer
 @Description
 @Input			psDevInfo

 @Return		PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXFWSetupFirmwareGcovBuffer(PVRSRV_RGXDEV_INFO*			psDevInfo,
		DEVMEM_MEMDESC**			ppsBufferMemDesc,
		IMG_UINT32					ui32FirmwareGcovBufferSize,
		RGXFWIF_FIRMWARE_GCOV_CTL*	psFirmwareGcovCtl,
		const IMG_CHAR*				pszBufferName)
{
	PVRSRV_ERROR	eError;
	DEVMEM_FLAGS_T	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Allocate memory for gcov */
	PDUMPCOMMENT("Allocate memory for %s", pszBufferName);
	eError = DevmemFwAllocate(psDevInfo,
			ui32FirmwareGcovBufferSize,
			uiMemAllocFlags,
			pszBufferName,
			ppsBufferMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for firmware gcov buffer (%u)",
				ui32FirmwareGcovBufferSize,
				eError));
		return eError;
	}

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(&psFirmwareGcovCtl->sBuffer,
			*ppsBufferMemDesc,
			0,
			RFW_FWADDR_NOREF_FLAG);

	psFirmwareGcovCtl->ui32Size = ui32FirmwareGcovBufferSize;

	return PVRSRV_OK;
}
#endif

#if defined(SUPPORT_POWER_SAMPLING_VIA_DEBUGFS)
/*!
 *******************************************************************************
 @Function		RGXFWSetupCounterBuffer
 @Description
 @Input			psDevInfo

 @Return		PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXFWSetupCounterBuffer(PVRSRV_RGXDEV_INFO*			psDevInfo,
		DEVMEM_MEMDESC**			ppsBufferMemDesc,
		IMG_UINT32					ui32CounterDataBufferSize,
		RGXFWIF_COUNTER_DUMP_CTL*	psCounterDumpCtl,
		const IMG_CHAR*				pszBufferName)
{
	PVRSRV_ERROR	eError;
	DEVMEM_FLAGS_T	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for %s power counter buffer", pszBufferName);
	eError = DevmemFwAllocate(psDevInfo,
			ui32CounterDataBufferSize,
			uiMemAllocFlags,
			"FwCounterBuffer",
			ppsBufferMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for counter buffer (%u)",
				ui32CounterDataBufferSize,
				eError));
		return eError;
	}

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(&psCounterDumpCtl->sBuffer,
			*ppsBufferMemDesc,
			0,
			RFW_FWADDR_NOREF_FLAG);

	psCounterDumpCtl->ui32SizeInDwords = ui32CounterDataBufferSize >> 2;

	return PVRSRV_OK;
}
#endif

#if defined(RGXFW_ALIGNCHECKS)
/*!
 *******************************************************************************
 @Function		RGXFWSetupAlignChecks
 @Description   This functions allocates and fills memory needed for the
                aligns checks of the UM and KM structures shared with the
                firmware. The format of the data in the memory is as follows:
                    <number of elements in the KM array>
                    <array of KM structures' sizes and members' offsets>
                    <number of elements in the UM array>
                    <array of UM structures' sizes and members' offsets>
                The UM array is passed from the user side. Now the firmware is
                is responsible for filling this part of the memory. If that
                happens the check of the UM structures will be performed
                by the host driver on client's connect.
                If the macro is not defined the client driver fills the memory
                and the firmware checks for the alignment of all structures.
 @Input			psDevInfo

 @Return		PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXFWSetupAlignChecks(PVRSRV_RGXDEV_INFO* psDevInfo,
		RGXFWIF_DEV_VIRTADDR	*psAlignChecksDevFW,
		IMG_UINT32				*pui32RGXFWAlignChecks,
		IMG_UINT32				ui32RGXFWAlignChecksArrLength)
{
	IMG_UINT32		aui32RGXFWAlignChecksKM[] = { RGXFW_ALIGN_CHECKS_INIT_KM };
	IMG_UINT32		ui32RGXFWAlingChecksTotal;
	IMG_UINT32*		paui32AlignChecks;
	PVRSRV_ERROR	eError;

	/* In this case we don't know the number of elements in UM array.
	 * We have to assume something so we assume RGXFW_ALIGN_CHECKS_UM_MAX.
	 */
	PVR_ASSERT(ui32RGXFWAlignChecksArrLength == 0);
	ui32RGXFWAlingChecksTotal = sizeof(aui32RGXFWAlignChecksKM)
	                            + RGXFW_ALIGN_CHECKS_UM_MAX * sizeof(IMG_UINT32)
	                            + 2 * sizeof(IMG_UINT32);

	/* Allocate memory for the checks */
	PDUMPCOMMENT("Allocate memory for alignment checks");
	eError = DevmemFwAllocate(psDevInfo,
			ui32RGXFWAlingChecksTotal,
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | PVRSRV_MEMALLOCFLAG_UNCACHED,
			"FwAlignmentChecks",
			&psDevInfo->psRGXFWAlignChecksMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to allocate %d bytes for alignment checks (%u)",
				ui32RGXFWAlingChecksTotal,
				eError));
		goto failAlloc;
	}

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc,
			(void **)&paui32AlignChecks);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire kernel addr for alignment checks (%u)",
				eError));
		goto failAqCpuAddr;
	}

	/* Copy the values */
	*paui32AlignChecks++ = ARRAY_SIZE(aui32RGXFWAlignChecksKM);
	OSDeviceMemCopy(paui32AlignChecks, &aui32RGXFWAlignChecksKM[0], sizeof(aui32RGXFWAlignChecksKM));
	paui32AlignChecks += ARRAY_SIZE(aui32RGXFWAlignChecksKM);

	*paui32AlignChecks = 0;

	DevmemPDumpLoadMem(	psDevInfo->psRGXFWAlignChecksMemDesc,
			0,
			ui32RGXFWAlingChecksTotal,
			PDUMP_FLAGS_CONTINUOUS);

	/* Prepare the pointer for the fw to access that memory */
	RGXSetFirmwareAddress(psAlignChecksDevFW,
			psDevInfo->psRGXFWAlignChecksMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	return PVRSRV_OK;


	failAqCpuAddr:
	DevmemFwFree(psDevInfo, psDevInfo->psRGXFWAlignChecksMemDesc);
	psDevInfo->psRGXFWAlignChecksMemDesc = NULL;
	failAlloc:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static void RGXFWFreeAlignChecks(PVRSRV_RGXDEV_INFO* psDevInfo)
{
	if (psDevInfo->psRGXFWAlignChecksMemDesc != NULL)
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWAlignChecksMemDesc);
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWAlignChecksMemDesc);
		psDevInfo->psRGXFWAlignChecksMemDesc = NULL;
	}
}
#endif

static void
RGXVzDevMemFreeGuestFwHeap(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT32 ui32OSID)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_VZ_RETN_IF_NOT_MODE(DRIVER_MODE_HOST);

	if (!ui32OSID || ui32OSID >= RGXFW_NUM_OS)
	{
		/* Guest OSID(s) range [1 up to (RGXFW_NUM_OS-1)] */
		PVR_DPF((PVR_DBG_ERROR,
				"Deallocating guest fw heap with invalid OSID:%u, MAX:%u",
				ui32OSID, RGXFW_NUM_OS - 1));
		return;
	}

	if (psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID])
	{
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;
		DevmemReleaseDevVirtAddr(psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID]);
		DevmemFree(psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID]);
		psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID] = NULL;
	}

	if (psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID])
	{
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;
		DevmemReleaseDevVirtAddr(psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID]);
		DevmemFree(psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID]);
		psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID] = NULL;
	}

	if (psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID])
	{
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;
		DevmemReleaseDevVirtAddr(psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID]);
		DevmemFree(psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID]);
		psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID] = NULL;
	}
}

static PVRSRV_ERROR
RGXVzDevMemAllocateGuestFwHeap(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT32 ui32OSID)
{
	PVRSRV_ERROR eError;
	IMG_CHAR szHeapName[32];
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	PVRSRV_DEVICE_PHYS_HEAP_ORIGIN eHeapOrigin;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	IMG_UINT32 ui32CacheLineSize =
		GET_ROGUE_CACHE_LINE_SIZE(RGX_GET_FEATURE_VALUE(psDevInfo, SLC_CACHE_LINE_SIZE_BITS));
	IMG_UINT32 ui32FwHeapAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
									  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
									  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
									  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
									  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
									  PVRSRV_MEMALLOCFLAG_UNCACHED |
									  PVRSRV_MEMALLOCFLAG_FW_LOCAL |
									  PVRSRV_MEMALLOCFLAG_FW_GUEST;

	/*
	 * This is called by the host driver only, it pre-allocates and maps
	 * into the firmware kernel memory context all guest firmware physheaps
	 * so we fail the call if an invalid OSID (i.e. either host OSID or
	 * OSID outside range) is supplied (i.e. as this would have been due
	 * to an internal error).
	 */
	PVRSRV_VZ_RET_IF_NOT_MODE(DRIVER_MODE_HOST, PVRSRV_ERROR_INTERNAL_ERROR);
	if (!ui32OSID || ui32OSID >= RGXFW_NUM_OS)
	{
		/* Guest OSID(s) range [1 up to (RGXFW_NUM_OS-1)] */
		PVR_DPF((PVR_DBG_ERROR,
				"Allocating guest fw heap with invalid OSID:%u, MAX:%u",
				ui32OSID, RGXFW_NUM_OS - 1));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto fail;
	}

	PDUMPCOMMENT("Mapping firmware physheaps for OSID: [%d]", ui32OSID);

	SysVzGetPhysHeapOrigin(psDeviceNode->psDevConfig,
						   PVRSRV_DEVICE_PHYS_HEAP_FW_LOCAL,
						   &eHeapOrigin);

	if (eHeapOrigin == PVRSRV_DEVICE_PHYS_HEAP_ORIGIN_HOST)
	{
		/* Target OSID physheap for allocation */
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;

		OSSNPrintf(szHeapName, sizeof(szHeapName), "GuestFirmwareConfig%d", ui32OSID);
		/* This allocates the memory for guest Fw Config heap */
		eError = DevmemAllocate(psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
								RGX_FIRMWARE_CONFIG_HEAP_SIZE,
								ui32CacheLineSize,
								ui32FwHeapAllocFlags | PVRSRV_MEMALLOCFLAG_FW_CONFIG,
								szHeapName,
								&psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID]);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemAllocate() failed for Firmware Config heap (%u)", eError));
			goto fail;
		}

		/* If allocation is successful, permanently map this into device */
		eError = DevmemMapToDevice(psDevInfo->psGuestFirmwareConfigMemDesc[ui32OSID],
								   psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
								   &sTmpDevVAddr);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemMapToDevice() failed for Firmware Config heap (%u)", eError));
			goto fail;
		}

		/* Target OSID physheap for allocation */
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;

		OSSNPrintf(szHeapName, sizeof(szHeapName), "GuestFirmwareMain%d", ui32OSID);
		/* This allocates the memory for guest Fw Main heap */
		eError = DevmemAllocate(psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
								RGXGetFwMainHeapSize(psDevInfo),
								ui32CacheLineSize,
								ui32FwHeapAllocFlags,
								szHeapName,
								&psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID]);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemAllocate() failed for Firmware Main heap (%u)", eError));
			goto fail;
		}

		/* If allocation is successful, permanently map this into device */
		eError = DevmemMapToDevice(psDevInfo->psGuestFirmwareMainMemDesc[ui32OSID],
								   psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
								   &sTmpDevVAddr);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemMapToDevice() failed for Firmware Main heap (%u)", eError));
			goto fail;
		}
	}
	else
	{
		/* Target OSID physheap for allocation */
		psDeviceNode->uiKernelFwRAIdx = ui32OSID;

		OSSNPrintf(szHeapName, sizeof(szHeapName), "GuestFirmwareRaw%d", ui32OSID);
		/* This allocates the memory for guest Fw Raw heap */
		eError = DevmemAllocate(psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
								RGX_FIRMWARE_RAW_HEAP_SIZE,
								ui32CacheLineSize,
								ui32FwHeapAllocFlags,
								szHeapName,
								&psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID]);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemAllocate() failed for Firmware Raw heap (%u)", eError));
			goto fail;
		}

		/* If allocation is successful, permanently map this into device */
		eError = DevmemMapToDevice(psDevInfo->psGuestFirmwareRawMemDesc[ui32OSID],
					   psDevInfo->psGuestFirmwareRawHeap[ui32OSID],
					   &sTmpDevVAddr);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,	"DevmemMapToDevice() failed for Firmware Raw heap (%u)", eError));
			goto fail;
		}
	}

	return eError;

fail:
	RGXVzDevMemFreeGuestFwHeap(psDeviceNode, ui32OSID);

	return eError;
}

static PVRSRV_ERROR RGXVzSetupFirmware(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_ERROR eError;
	PVRSRV_DEVICE_PHYS_HEAP_ORIGIN eHeapOrigin;
	PVRSRV_DEVICE_PHYS_HEAP eHeapType = PVRSRV_DEVICE_PHYS_HEAP_FW_LOCAL;
	PVRSRV_VZ_RET_IF_NOT_MODE(DRIVER_MODE_HOST, PVRSRV_OK);

	eError = SysVzGetPhysHeapOrigin(psDeviceNode->psDevConfig, eHeapType, &eHeapOrigin);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

#if (RGXFW_GUEST_OSID_START < RGXFW_NUM_OS)
	if (eHeapOrigin == PVRSRV_DEVICE_PHYS_HEAP_ORIGIN_HOST)
	{
		IMG_UINT32 ui32OSID;

		/* Guest OSID(s) in range [RGXFW_GUEST_OSID_START up to (RGXFW_NUM_OS-1)] */
		for (ui32OSID = RGXFW_GUEST_OSID_START; ui32OSID < RGXFW_NUM_OS; ui32OSID++)
		{
			eError = RGXVzDevMemAllocateGuestFwHeap(psDeviceNode, ui32OSID);
			PVR_ASSERT(eError == PVRSRV_OK);
		}
	}
#endif

	return eError;
}

static void
RGXVzFreeFirmware(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_ERROR eError;
	PVRSRV_DEVICE_PHYS_HEAP_ORIGIN eHeapOrigin;
	PVRSRV_DEVICE_PHYS_HEAP eHeapType = PVRSRV_DEVICE_PHYS_HEAP_FW_LOCAL;
	PVRSRV_VZ_RETN_IF_NOT_MODE(DRIVER_MODE_HOST);

	eError = SysVzGetPhysHeapOrigin(psDeviceNode->psDevConfig, eHeapType, &eHeapOrigin);
	if (eError != PVRSRV_OK)
	{
		return;
	}

#if (RGXFW_GUEST_OSID_START < RGXFW_NUM_OS)
	if (eHeapOrigin == PVRSRV_DEVICE_PHYS_HEAP_ORIGIN_HOST)
	{
		IMG_UINT32 ui32OSID;

		/* Guest OSID(s) in range [RGXFW_GUEST_OSID_START up to (RGXFW_NUM_OS-1)] */
		for (ui32OSID = RGXFW_GUEST_OSID_START; ui32OSID < RGXFW_NUM_OS; ui32OSID++)
		{
			RGXVzDevMemFreeGuestFwHeap(psDeviceNode, ui32OSID);
		}
	}
#endif
}

void RGXSetFirmwareAddress(RGXFWIF_DEV_VIRTADDR	*ppDest,
		DEVMEM_MEMDESC		*psSrc,
		IMG_UINT32			uiExtraOffset,
		IMG_UINT32			ui32Flags)
{
	PVRSRV_ERROR		eError;
	IMG_DEV_VIRTADDR	psDevVirtAddr;
	PVRSRV_DEVICE_NODE	*psDeviceNode;
	PVRSRV_RGXDEV_INFO	*psDevInfo;

	psDeviceNode = (PVRSRV_DEVICE_NODE *) DevmemGetConnection(psSrc);
	psDevInfo = (PVRSRV_RGXDEV_INFO *)psDeviceNode->pvDevice;

	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META))
	{
		IMG_UINT32	    ui32Offset;
		IMG_BOOL            bCachedInMETA;
		DEVMEM_FLAGS_T      uiDevFlags;
		IMG_UINT32          uiGPUCacheMode;

		eError = DevmemAcquireDevVirtAddr(psSrc, &psDevVirtAddr);
		PVR_ASSERT(eError == PVRSRV_OK);

		/* Convert to an address in META memmap */
		ui32Offset = psDevVirtAddr.uiAddr + uiExtraOffset - RGX_FIRMWARE_RAW_HEAP_BASE;

		/* Check in the devmem flags whether this memory is cached/uncached */
		DevmemGetFlags(psSrc, &uiDevFlags);

		/* Honour the META cache flags */
		bCachedInMETA = (PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) & uiDevFlags) != 0;

		/* Honour the SLC cache flags */
		eError = DevmemDeviceCacheMode(psDeviceNode, uiDevFlags, &uiGPUCacheMode);
		PVR_ASSERT(eError == PVRSRV_OK);

		ui32Offset += RGXFW_SEGMMU_DATA_BASE_ADDRESS;

		if (bCachedInMETA)
		{
			ui32Offset |= RGXFW_SEGMMU_DATA_META_CACHED;
		}
		else
		{
			ui32Offset |= RGXFW_SEGMMU_DATA_META_UNCACHED;
		}

		if (PVRSRV_CHECK_GPU_CACHED(uiGPUCacheMode))
		{
			ui32Offset |= RGXFW_SEGMMU_DATA_VIVT_SLC_CACHED;
		}
		else
		{
			ui32Offset |= RGXFW_SEGMMU_DATA_VIVT_SLC_UNCACHED;
		}
		ppDest->ui32Addr = ui32Offset;
	}else
	{
		eError = DevmemAcquireDevVirtAddr(psSrc, &psDevVirtAddr);
		PVR_ASSERT(eError == PVRSRV_OK);
		ppDest->ui32Addr = (IMG_UINT32)((psDevVirtAddr.uiAddr + uiExtraOffset) & 0xFFFFFFFF);
	}

	if (ui32Flags & RFW_FWADDR_NOREF_FLAG)
	{
		DevmemReleaseDevVirtAddr(psSrc);
	}
}

void RGXSetMetaDMAAddress(RGXFWIF_DMA_ADDR		*psDest,
		DEVMEM_MEMDESC		*psSrcMemDesc,
		RGXFWIF_DEV_VIRTADDR	*psSrcFWDevVAddr,
		IMG_UINT32			uiOffset)
{
	PVRSRV_ERROR		eError;
	IMG_DEV_VIRTADDR	sDevVirtAddr;

	eError = DevmemAcquireDevVirtAddr(psSrcMemDesc, &sDevVirtAddr);
	PVR_ASSERT(eError == PVRSRV_OK);

	psDest->psDevVirtAddr.uiAddr = sDevVirtAddr.uiAddr;
	psDest->psDevVirtAddr.uiAddr += uiOffset;
	psDest->pbyFWAddr.ui32Addr = psSrcFWDevVAddr->ui32Addr;

	DevmemReleaseDevVirtAddr(psSrcMemDesc);
}


void RGXUnsetFirmwareAddress(DEVMEM_MEMDESC *psSrc)
{
	DevmemReleaseDevVirtAddr(psSrc);
}

struct _RGX_SERVER_COMMON_CONTEXT_ {
	PVRSRV_RGXDEV_INFO *psDevInfo;
	DEVMEM_MEMDESC *psFWCommonContextMemDesc;
	PRGXFWIF_FWCOMMONCONTEXT sFWCommonContextFWAddr;
	DEVMEM_MEMDESC *psFWMemContextMemDesc;
	DEVMEM_MEMDESC *psFWFrameworkMemDesc;
	DEVMEM_MEMDESC *psContextStateMemDesc;
	RGX_CLIENT_CCB *psClientCCB;
	DEVMEM_MEMDESC *psClientCCBMemDesc;
	DEVMEM_MEMDESC *psClientCCBCtrlMemDesc;
	IMG_BOOL bCommonContextMemProvided;
	IMG_UINT32 ui32ContextID;
	DLLIST_NODE sListNode;
	RGXFWIF_CONTEXT_RESET_REASON eLastResetReason;
	IMG_UINT32 ui32LastResetJobRef;
};

PVRSRV_ERROR FWCommonContextAllocate(CONNECTION_DATA *psConnection,
		PVRSRV_DEVICE_NODE *psDeviceNode,
		RGX_CCB_REQUESTOR_TYPE eRGXCCBRequestor,
		RGXFWIF_DM eDM,
		DEVMEM_MEMDESC *psAllocatedMemDesc,
		IMG_UINT32 ui32AllocatedOffset,
		DEVMEM_MEMDESC *psFWMemContextMemDesc,
		DEVMEM_MEMDESC *psContextStateMemDesc,
		IMG_UINT32 ui32CCBAllocSizeLog2,
		IMG_UINT32 ui32CCBMaxAllocSizeLog2,
		IMG_UINT32 ui32Priority,
		RGX_COMMON_CONTEXT_INFO *psInfo,
		RGX_SERVER_COMMON_CONTEXT **ppsServerCommonContext)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGX_SERVER_COMMON_CONTEXT *psServerCommonContext;
	RGXFWIF_FWCOMMONCONTEXT *psFWCommonContext;
	IMG_UINT32 ui32FWCommonContextOffset;
	IMG_UINT8 *pui8Ptr;
	PVRSRV_ERROR eError;

	/*
	 * Allocate all the resources that are required
	 */
	psServerCommonContext = OSAllocMem(sizeof(*psServerCommonContext));
	if (psServerCommonContext == NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	psServerCommonContext->psDevInfo = psDevInfo;

	if (psAllocatedMemDesc)
	{
		PDUMPCOMMENT("Using existing MemDesc for Rogue firmware %s context (offset = %d)",
				aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT],
				ui32AllocatedOffset);
		ui32FWCommonContextOffset = ui32AllocatedOffset;
		psServerCommonContext->psFWCommonContextMemDesc = psAllocatedMemDesc;
		psServerCommonContext->bCommonContextMemProvided = IMG_TRUE;
	}
	else
	{
		/* Allocate device memory for the firmware context */
		PDUMPCOMMENT("Allocate Rogue firmware %s context", aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT]);
		eError = DevmemFwAllocate(psDevInfo,
				sizeof(*psFWCommonContext),
				RGX_FWCOMCTX_ALLOCFLAGS,
				"FwContext",
				&psServerCommonContext->psFWCommonContextMemDesc);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate firmware %s context (%s)",
					__func__,
					aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT],
					PVRSRVGetErrorString(eError)));
			goto fail_contextalloc;
		}
		ui32FWCommonContextOffset = 0;
		psServerCommonContext->bCommonContextMemProvided = IMG_FALSE;
	}

	/* Record this context so we can refer to it if the FW needs to tell us it was reset. */
	psServerCommonContext->eLastResetReason    = RGXFWIF_CONTEXT_RESET_REASON_NONE;
	psServerCommonContext->ui32LastResetJobRef = 0;
	psServerCommonContext->ui32ContextID       = psDevInfo->ui32CommonCtxtCurrentID++;

	/*
	 * Temporarily map the firmware context to the kernel and init it
	 */
	eError = DevmemAcquireCpuVirtAddr(psServerCommonContext->psFWCommonContextMemDesc,
			(void **)&pui8Ptr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to map firmware %s context (%s)to CPU",
				__func__,
				aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT],
				PVRSRVGetErrorString(eError)));
		goto fail_cpuvirtacquire;
	}

	/* Allocate the client CCB */
	eError = RGXCreateCCB(psDevInfo,
			ui32CCBAllocSizeLog2,
			ui32CCBMaxAllocSizeLog2,
			psConnection,
			eRGXCCBRequestor,
			psServerCommonContext,
			&psServerCommonContext->psClientCCB,
			&psServerCommonContext->psClientCCBMemDesc,
			&psServerCommonContext->psClientCCBCtrlMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: failed to create CCB for %s context(%s)",
				__func__,
				aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT],
				PVRSRVGetErrorString(eError)));
		goto fail_allocateccb;
	}

	psFWCommonContext = (RGXFWIF_FWCOMMONCONTEXT *) (pui8Ptr + ui32FWCommonContextOffset);
	psFWCommonContext->eDM = eDM;

	/* Set the firmware CCB device addresses in the firmware common context */
	RGXSetFirmwareAddress(&psFWCommonContext->psCCB,
			psServerCommonContext->psClientCCBMemDesc,
			0, RFW_FWADDR_FLAG_NONE);
	RGXSetFirmwareAddress(&psFWCommonContext->psCCBCtl,
			psServerCommonContext->psClientCCBCtrlMemDesc,
			0, RFW_FWADDR_FLAG_NONE);

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, META_DMA))
	{
		RGXSetMetaDMAAddress(&psFWCommonContext->sCCBMetaDMAAddr,
				psServerCommonContext->psClientCCBMemDesc,
				&psFWCommonContext->psCCB,
				0);
	}

	/* Set the memory context device address */
	psServerCommonContext->psFWMemContextMemDesc = psFWMemContextMemDesc;
	RGXSetFirmwareAddress(&psFWCommonContext->psFWMemContext,
			psFWMemContextMemDesc,
			0, RFW_FWADDR_FLAG_NONE);

	/* Set the framework register updates address */
	psServerCommonContext->psFWFrameworkMemDesc = psInfo->psFWFrameworkMemDesc;
	if (psInfo->psFWFrameworkMemDesc != NULL)
	{
		RGXSetFirmwareAddress(&psFWCommonContext->psRFCmd,
				psInfo->psFWFrameworkMemDesc,
				0, RFW_FWADDR_FLAG_NONE);
	}
	else
	{
		/* This should never be touched in this contexts without a framework
		 * memdesc, but ensure it is zero so we see crashes if it is.
		 */
		psFWCommonContext->psRFCmd.ui32Addr = 0;
	}

	psFWCommonContext->ui32Priority = ui32Priority;
	psFWCommonContext->ui32PrioritySeqNum = 0;

	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, CDM_CONTROL_STREAM_FORMAT) &&
			(RGX_GET_FEATURE_VALUE(psDevInfo, CDM_CONTROL_STREAM_FORMAT) == 2) && \
			(RGX_IS_FEATURE_SUPPORTED(psDevInfo, SIGNAL_SNOOPING)))
	{
		if (eDM == RGXFWIF_DM_CDM)
		{
			if (psInfo->psResumeSignalAddr != NULL)
			{
				psFWCommonContext->ui64ResumeSignalAddr = psInfo->psResumeSignalAddr->uiAddr;
			}
		}
	}

	/* Store a references to Server Common Context and PID for notifications back from the FW. */
	psFWCommonContext->ui32ServerCommonContextID = psServerCommonContext->ui32ContextID;
	psFWCommonContext->ui32PID                   = OSGetCurrentClientProcessIDKM();

	/* Set the firmware GPU context state buffer */
	psServerCommonContext->psContextStateMemDesc = psContextStateMemDesc;
	if (psContextStateMemDesc)
	{
		RGXSetFirmwareAddress(&psFWCommonContext->psContextState,
				psContextStateMemDesc,
				0,
				RFW_FWADDR_FLAG_NONE);
	}

	/*
	 * Dump the created context
	 */
	PDUMPCOMMENT("Dump %s context", aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT]);
	DevmemPDumpLoadMem(psServerCommonContext->psFWCommonContextMemDesc,
			ui32FWCommonContextOffset,
			sizeof(*psFWCommonContext),
			PDUMP_FLAGS_CONTINUOUS);

	/* We've finished the setup so release the CPU mapping */
	DevmemReleaseCpuVirtAddr(psServerCommonContext->psFWCommonContextMemDesc);

	/* Map this allocation into the FW */
	RGXSetFirmwareAddress(&psServerCommonContext->sFWCommonContextFWAddr,
			psServerCommonContext->psFWCommonContextMemDesc,
			ui32FWCommonContextOffset,
			RFW_FWADDR_FLAG_NONE);

#if defined(LINUX)
	{
		IMG_UINT32 ui32FWAddr;
		switch (eDM) {
		case RGXFWIF_DM_TA:
			ui32FWAddr = (IMG_UINT32) ((uintptr_t) IMG_CONTAINER_OF((void *) ((uintptr_t)
					psServerCommonContext->sFWCommonContextFWAddr.ui32Addr), RGXFWIF_FWRENDERCONTEXT, sTAContext));
			break;
		case RGXFWIF_DM_3D:
			ui32FWAddr = (IMG_UINT32) ((uintptr_t) IMG_CONTAINER_OF((void *) ((uintptr_t)
					psServerCommonContext->sFWCommonContextFWAddr.ui32Addr), RGXFWIF_FWRENDERCONTEXT, s3DContext));
			break;
		default:
			ui32FWAddr = psServerCommonContext->sFWCommonContextFWAddr.ui32Addr;
			break;
		}

		trace_rogue_create_fw_context(OSGetCurrentClientProcessNameKM(),
				aszCCBRequestors[eRGXCCBRequestor][REQ_PDUMP_COMMENT],
				ui32FWAddr);
	}
#endif
	/*Add the node to the list when finalised */
	OSWRLockAcquireWrite(psDevInfo->hCommonCtxtListLock);
	dllist_add_to_tail(&(psDevInfo->sCommonCtxtListHead), &(psServerCommonContext->sListNode));
	OSWRLockReleaseWrite(psDevInfo->hCommonCtxtListLock);

	*ppsServerCommonContext = psServerCommonContext;
	return PVRSRV_OK;

	fail_allocateccb:
	DevmemReleaseCpuVirtAddr(psServerCommonContext->psFWCommonContextMemDesc);
	fail_cpuvirtacquire:
	if (!psServerCommonContext->bCommonContextMemProvided)
	{
		DevmemFwFree(psDevInfo, psServerCommonContext->psFWCommonContextMemDesc);
		psServerCommonContext->psFWCommonContextMemDesc = NULL;
	}
	fail_contextalloc:
	OSFreeMem(psServerCommonContext);
	fail_alloc:
	return eError;
}

void FWCommonContextFree(RGX_SERVER_COMMON_CONTEXT *psServerCommonContext)
{

	OSWRLockAcquireWrite(psServerCommonContext->psDevInfo->hCommonCtxtListLock);
	/* Remove the context from the list of all contexts. */
	dllist_remove_node(&psServerCommonContext->sListNode);
	OSWRLockReleaseWrite(psServerCommonContext->psDevInfo->hCommonCtxtListLock);

	/*
		Unmap the context itself and then all its resources
	 */

	/* Unmap the FW common context */
	RGXUnsetFirmwareAddress(psServerCommonContext->psFWCommonContextMemDesc);
	/* Umap context state buffer (if there was one) */
	if (psServerCommonContext->psContextStateMemDesc)
	{
		RGXUnsetFirmwareAddress(psServerCommonContext->psContextStateMemDesc);
	}
	/* Unmap the framework buffer */
	if (psServerCommonContext->psFWFrameworkMemDesc)
	{
		RGXUnsetFirmwareAddress(psServerCommonContext->psFWFrameworkMemDesc);
	}
	/* Unmap client CCB and CCB control */
	RGXUnsetFirmwareAddress(psServerCommonContext->psClientCCBCtrlMemDesc);
	RGXUnsetFirmwareAddress(psServerCommonContext->psClientCCBMemDesc);
	/* Unmap the memory context */
	RGXUnsetFirmwareAddress(psServerCommonContext->psFWMemContextMemDesc);

	/* Destroy the client CCB */
	RGXDestroyCCB(psServerCommonContext->psDevInfo, psServerCommonContext->psClientCCB);


	/* Free the FW common context (if there was one) */
	if (!psServerCommonContext->bCommonContextMemProvided)
	{
		DevmemFwFree(psServerCommonContext->psDevInfo,
				psServerCommonContext->psFWCommonContextMemDesc);
		psServerCommonContext->psFWCommonContextMemDesc = NULL;
	}
	/* Free the hosts representation of the common context */
	OSFreeMem(psServerCommonContext);
}

PRGXFWIF_FWCOMMONCONTEXT FWCommonContextGetFWAddress(RGX_SERVER_COMMON_CONTEXT *psServerCommonContext)
{
	return psServerCommonContext->sFWCommonContextFWAddr;
}

RGX_CLIENT_CCB *FWCommonContextGetClientCCB(RGX_SERVER_COMMON_CONTEXT *psServerCommonContext)
{
	return psServerCommonContext->psClientCCB;
}

RGXFWIF_CONTEXT_RESET_REASON FWCommonContextGetLastResetReason(RGX_SERVER_COMMON_CONTEXT *psServerCommonContext,
		IMG_UINT32 *pui32LastResetJobRef)
{
	RGXFWIF_CONTEXT_RESET_REASON eLastResetReason;

	PVR_ASSERT(psServerCommonContext != NULL);
	PVR_ASSERT(pui32LastResetJobRef != NULL);

	/* Take the most recent reason & job ref and reset for next time... */
	eLastResetReason      = psServerCommonContext->eLastResetReason;
	*pui32LastResetJobRef = psServerCommonContext->ui32LastResetJobRef;
	psServerCommonContext->eLastResetReason = RGXFWIF_CONTEXT_RESET_REASON_NONE;
	psServerCommonContext->ui32LastResetJobRef = 0;

	if (eLastResetReason == RGXFWIF_CONTEXT_RESET_REASON_HARD_CONTEXT_SWITCH)
	{
		PVR_DPF((PVR_DBG_WARNING,"A Hard Context Switch was triggered on the GPU to ensure Quality of Service."));
	}

	return eLastResetReason;
}

PVRSRV_RGXDEV_INFO* FWCommonContextGetRGXDevInfo(RGX_SERVER_COMMON_CONTEXT *psServerCommonContext)
{
	return psServerCommonContext->psDevInfo;
}

/*!
 *******************************************************************************
 @Function		RGXFreeCCB
 @Description	Free the kernel or firmware CCB
 @Input			psDevInfo
 @Input			ppsCCBCtl
 @Input			ppsCCBCtlMemDesc
 @Input			ppsCCBMemDesc
 @Input			psCCBCtlFWAddr
 ******************************************************************************/
static void RGXFreeCCB(PVRSRV_RGXDEV_INFO	*psDevInfo,
					   RGXFWIF_CCB_CTL		**ppsCCBCtl,
					   DEVMEM_MEMDESC		**ppsCCBCtlMemDesc,
					   IMG_UINT8			**ppui8CCB,
					   DEVMEM_MEMDESC		**ppsCCBMemDesc)
{
	if (*ppsCCBMemDesc != NULL)
	{
		if (*ppui8CCB != NULL)
		{
			DevmemReleaseCpuVirtAddr(*ppsCCBMemDesc);
			*ppui8CCB = NULL;
		}
		DevmemFwFree(psDevInfo, *ppsCCBMemDesc);
		*ppsCCBMemDesc = NULL;
	}
	if (*ppsCCBCtlMemDesc != NULL)
	{
		if (*ppsCCBCtl != NULL)
		{
			DevmemReleaseCpuVirtAddr(*ppsCCBCtlMemDesc);
			*ppsCCBCtl = NULL;
		}
		DevmemFwFree(psDevInfo, *ppsCCBCtlMemDesc);
		*ppsCCBCtlMemDesc = NULL;
	}
}

/*!
 *******************************************************************************
 @Function		RGXSetupCCB
 @Description	Allocate and initialise the kernel CCB
 @Input			psDevInfo
 @Input			ppsCCBCtl
 @Input			ppsCCBCtlMemDesc
 @Input			ppui8CCB
 @Input			ppsCCBMemDesc
 @Input			psCCBCtlFWAddr
 @Input			ui32NumCmdsLog2
 @Input			ui32CmdSize
 @Input			pszName

 @Return		PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXSetupCCB(PVRSRV_RGXDEV_INFO	*psDevInfo,
								RGXFWIF_CCB_CTL		**ppsCCBCtl,
								DEVMEM_MEMDESC		**ppsCCBCtlMemDesc,
								IMG_UINT8			**ppui8CCB,
								DEVMEM_MEMDESC		**ppsCCBMemDesc,
								PRGXFWIF_CCB_CTL	*psCCBCtlFWAddr,
								PRGXFWIF_CCB		*psCCBFWAddr,
								IMG_UINT32			ui32NumCmdsLog2,
								IMG_UINT32			ui32CmdSize,
								DEVMEM_FLAGS_T		uiCCBMemAllocFlags,
								const IMG_CHAR		*pszName)
{
	const IMG_UINT32	ui32MaxInputStrSize	= 13;
	const IMG_UINT32	ui32AppendStrSize	= 7;
	const IMG_UINT32	ui32MaxTotalStrSize	= ui32MaxInputStrSize + ui32AppendStrSize + 1;
	const IMG_CHAR		sAppend[] = "Control";
	PVRSRV_ERROR		eError;
	RGXFWIF_CCB_CTL		*psCCBCtl;
	DEVMEM_FLAGS_T		uiCCBCtlMemAllocFlags;
	IMG_UINT32		ui32CCBSize = (1U << ui32NumCmdsLog2);
	IMG_CHAR		sCCBCtlName[ui32MaxTotalStrSize];

	PVR_ASSERT(strlen(sAppend) == ui32AppendStrSize);
	PVR_ASSERT(strlen(pszName) <= ui32MaxInputStrSize);

	uiCCBCtlMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Append "Control" to the name for the control struct. */
	strcpy(sCCBCtlName, pszName);
	strncat(sCCBCtlName, sAppend, ui32AppendStrSize);

	/* Allocate memory for the CCB control.*/
	PDUMPCOMMENT("Allocate memory for %s", sCCBCtlName);
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_CCB_CTL),
			uiCCBCtlMemAllocFlags,
			sCCBCtlName,
			ppsCCBCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate %s (%u)", __func__, sCCBCtlName, eError));
		goto fail;
	}

	/*
	 * Allocate memory for the CCB.
	 * (this will reference further command data in non-shared CCBs)
	 */
	PDUMPCOMMENT("Allocate memory for %s", pszName);
	eError = DevmemFwAllocate(psDevInfo,
			ui32CCBSize * ui32CmdSize,
			uiCCBMemAllocFlags,
			pszName,
			ppsCCBMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate %s (%u)", __func__,  pszName, eError));
		goto fail;
	}

	/*
		Map the CCB control to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsCCBCtlMemDesc,
			(void **)ppsCCBCtl);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire cpu %s (%u)", __func__,  sCCBCtlName, eError));
		goto fail;
	}

	/*
	 * Map the CCB to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsCCBMemDesc,
			(void **)ppui8CCB);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire cpu %s (%u)", __func__,  pszName, eError));
		goto fail;
	}

	/*
	 * Initialise the CCB control.
	 */
	psCCBCtl = *ppsCCBCtl;
	psCCBCtl->ui32WriteOffset = 0;
	psCCBCtl->ui32ReadOffset = 0;
	psCCBCtl->ui32WrapMask = ui32CCBSize - 1;
	psCCBCtl->ui32CmdSize = ui32CmdSize;

	/* Set-up RGXFWIfCtl pointers to access the kCCB */
	RGXSetFirmwareAddress(psCCBCtlFWAddr,
			*ppsCCBCtlMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	RGXSetFirmwareAddress(psCCBFWAddr,
			*ppsCCBMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	/* Pdump the CCB control */
	PDUMPCOMMENT("Initialise %s", sCCBCtlName);
	DevmemPDumpLoadMem(*ppsCCBCtlMemDesc,
			0,
			sizeof(RGXFWIF_CCB_CTL),
			0);

	return PVRSRV_OK;

	fail:
	RGXFreeCCB(psDevInfo,
			ppsCCBCtl,
			ppsCCBCtlMemDesc,
			ppui8CCB,
			ppsCCBMemDesc);

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static void RGXSetupFaultReadRegisterRollback(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PMR *psPMR;

	if (psDevInfo->psRGXFaultAddressMemDesc)
	{
		if (DevmemServerGetImportHandle(psDevInfo->psRGXFaultAddressMemDesc,(void **)&psPMR) == PVRSRV_OK)
		{
			PMRUnlockSysPhysAddresses(psPMR);
		}
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFaultAddressMemDesc);
		psDevInfo->psRGXFaultAddressMemDesc = NULL;
	}
}

static PVRSRV_ERROR RGXSetupFaultReadRegister(PVRSRV_DEVICE_NODE	*psDeviceNode, RGXFWIF_INIT *psRGXFWInit)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	IMG_UINT32			*pui32MemoryVirtAddr;
	IMG_UINT32			i;
	size_t				ui32PageSize = OSGetPageSize();
	DEVMEM_FLAGS_T		uiMemAllocFlags;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	PMR					*psPMR;

	/* Allocate page of memory to use for page faults on non-blocking memory transactions */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED;

	psDevInfo->psRGXFaultAddressMemDesc = NULL;
	eError = DevmemFwAllocateExportable(psDeviceNode,
			ui32PageSize,
			ui32PageSize,
			uiMemAllocFlags,
			"FwExFaultAddress",
			&psDevInfo->psRGXFaultAddressMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Failed to allocate mem for fault address (%u)",
				eError));
		goto failFaultAddressDescAlloc;
	}


	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFaultAddressMemDesc,
			(void **)&pui32MemoryVirtAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed to acquire mem for fault address (%u)",
				eError));
		goto failFaultAddressDescAqCpuVirt;
	}

	for (i = 0; i < ui32PageSize/sizeof(IMG_UINT32); i++)
	{
		*(pui32MemoryVirtAddr + i) = 0xDEADBEEF;
	}

	eError = DevmemServerGetImportHandle(psDevInfo->psRGXFaultAddressMemDesc,(void **)&psPMR);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error getting PMR for fault address (%u)",
				eError));

		goto failFaultAddressDescGetPMR;
	}
	else
	{
		IMG_BOOL bValid;
		IMG_UINT32 ui32Log2PageSize = OSGetPageShift();

		eError = PMRLockSysPhysAddresses(psPMR);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error locking physical address for fault address MemDesc (%u)",
					eError));

			goto failFaultAddressDescLockPhys;
		}

		eError = PMR_DevPhysAddr(psPMR,ui32Log2PageSize,1,0,&(psRGXFWInit->sFaultPhysAddr),&bValid);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Error getting physical address for fault address MemDesc (%u)",
					eError));

			goto failFaultAddressDescGetPhys;
		}

		if (!bValid)
		{
			psRGXFWInit->sFaultPhysAddr.uiAddr = 0;
			PVR_DPF((PVR_DBG_ERROR,"RGXSetupFirmware: Failed getting physical address for fault address MemDesc - invalid page (0x%" IMG_UINT64_FMTSPECX ")",
					psRGXFWInit->sFaultPhysAddr.uiAddr));

			goto failFaultAddressDescGetPhys;
		}
	}

	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFaultAddressMemDesc);

	return PVRSRV_OK;

	failFaultAddressDescGetPhys:
	PMRUnlockSysPhysAddresses(psPMR);

	failFaultAddressDescLockPhys:

	failFaultAddressDescGetPMR:
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFaultAddressMemDesc);

	failFaultAddressDescAqCpuVirt:
	DevmemFwFree(psDevInfo, psDevInfo->psRGXFaultAddressMemDesc);
	psDevInfo->psRGXFaultAddressMemDesc = NULL;

	failFaultAddressDescAlloc:

	return eError;
}

#if defined(PDUMP)
/* Replace the DevPhy address with the one Pdump allocates at pdump_player run time */
static PVRSRV_ERROR RGXPDumpFaultReadRegister(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	PVRSRV_ERROR eError;
	PMR *psFWInitPMR, *psFaultAddrPMR;
	IMG_UINT32 ui32Dstoffset;

	psFWInitPMR = (PMR *)(psDevInfo->psRGXFWIfInitMemDesc->psImport->hPMR);
	ui32Dstoffset = psDevInfo->psRGXFWIfInitMemDesc->uiOffset + offsetof(RGXFWIF_INIT, sFaultPhysAddr.uiAddr);

	psFaultAddrPMR = (PMR *)(psDevInfo->psRGXFaultAddressMemDesc->psImport->hPMR);

	eError = PDumpMemLabelToMem64(psFaultAddrPMR,
			psFWInitPMR,
			0,
			ui32Dstoffset,
			PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXPDumpFaultReadRegister: Dump of Fault Page Phys address failed(%u)", eError));
	}
	return eError;
}
#endif

/*************************************************************************/ /*!
@Function       RGXTBIBufferIsInitRequired

@Description    Returns true if the firmware tbi buffer is not allocated and
		might be required by the firmware soon. TBI buffer allocated
		on-demand to reduce RAM footprint on systems not needing
		tbi.

@Input          psDevInfo	 RGX device info

@Return		IMG_BOOL	Whether on-demand allocation(s) is/are needed
				or not
 */ /**************************************************************************/
INLINE IMG_BOOL RGXTBIBufferIsInitRequired(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_TRACEBUF*  psTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	/* The firmware expects a tbi buffer only when:
	 *	- Logtype is "tbi"
	 */
	if ((psDevInfo->psRGXFWIfTBIBufferMemDesc == NULL)
			&& (psTraceBufCtl->ui32LogType & ~RGXFWIF_LOG_TYPE_TRACE)
			&& (psTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_GROUP_MASK))
	{
		return IMG_TRUE;
	}

	return IMG_FALSE;
}

/*************************************************************************/ /*!
@Function       RGXTBIBufferDeinit

@Description    Deinitialises all the allocations and references that are made
		for the FW tbi buffer

@Input          ppsDevInfo	 RGX device info
@Return		void
 */ /**************************************************************************/
static void RGXTBIBufferDeinit(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfTBIBufferMemDesc);
	psDevInfo->psRGXFWIfTBIBufferMemDesc = NULL;
	psDevInfo->ui32RGXFWIfHWPerfBufSize = 0;
}

/*************************************************************************/ /*!
@Function       RGXTBIBufferInitOnDemandResources

@Description    Allocates the firmware TBI buffer required for reading SFs
		strings and initialize it with SFs.

@Input          psDevInfo	 RGX device info

@Return		PVRSRV_OK	If all went good, PVRSRV_ERROR otherwise.
 */ /**************************************************************************/
PVRSRV_ERROR RGXTBIBufferInitOnDemandResources(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	DEVMEM_FLAGS_T     uiMemAllocFlags;
	PVRSRV_ERROR       eError = PVRSRV_OK;
	IMG_UINT32         i, ui32Len;
	const IMG_UINT32   ui32NumFWTBIEntries = sizeof(SFs) / sizeof(SFs[0]);
	const IMG_UINT32   ui32FWTBIBufsize = ui32NumFWTBIEntries * sizeof(RGXFW_STID_FMT);
	RGXFW_STID_FMT     *psFW_SFs = NULL;

	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw tbi buffer");
	eError = DevmemFwAllocate(psDevInfo,
			ui32FWTBIBufsize,
			uiMemAllocFlags,
			"FwTBIBuffer",
			&psDevInfo->psRGXFWIfTBIBufferMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate %u bytes for fw TBI buffer (Error code:%u)",
				__func__,
				ui32FWTBIBufsize,
				eError));
		goto fail;
	}

	/* Firmware address should not be already set */
	if (psDevInfo->sRGXFWIfTBIBuffer.ui32Addr)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: FW address for FWTBI is already set. Resetting it with newly allocated one", __func__));
	}

	/* for the FW to use this address when reading strings from tbi buffer */
	RGXSetFirmwareAddress(&psDevInfo->sRGXFWIfTBIBuffer,
			psDevInfo->psRGXFWIfTBIBufferMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	/* Set an address for the host to be able to write SFs strings in buffer */
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfTBIBufferMemDesc,
			(void **)&psFW_SFs);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire kernel tbibuf ctl (Error code: %u)",
				__func__, eError));
		goto fail;
	}

	/* Copy SFs entries to FW buffer */
	for ( i = 0; i < ui32NumFWTBIEntries; i++)
	{
		OSDeviceMemCopy(&psFW_SFs[i].ui32Id, &SFs[i].ui32Id, sizeof(SFs[i].ui32Id));
		ui32Len = OSStringLength(SFs[i].psName);
		OSDeviceMemCopy(psFW_SFs[i].sName, SFs[i].psName, MIN(ui32Len, IMG_SF_STRING_MAX_SIZE - 1));
	}

	/* Set size of TBI buffer */
	psDevInfo->ui32FWIfTBIBufferSize = ui32FWTBIBufsize;

	/* release CPU mapping */
	DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfTBIBufferMemDesc);

	return PVRSRV_OK;
	fail:
	RGXTBIBufferDeinit(psDevInfo);
	return eError;
}

/*************************************************************************/ /*!
@Function       RGXTraceBufferIsInitRequired

@Description    Returns true if the firmware trace buffer is not allocated and
		might be required by the firmware soon. Trace buffer allocated
		on-demand to reduce RAM footprint on systems not needing
		firmware trace.

@Input          psDevInfo	 RGX device info

@Return		IMG_BOOL	Whether on-demand allocation(s) is/are needed
				or not
 */ /**************************************************************************/
INLINE IMG_BOOL RGXTraceBufferIsInitRequired(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_TRACEBUF*  psTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	/* The firmware expects a trace buffer only when:
	 *	- Logtype is "trace" AND
	 *	- at least one LogGroup is configured
	 *	- the Driver Mode is not Guest
	 */
	if ((psDevInfo->psRGXFWIfTraceBufferMemDesc[0] == NULL)
			&& (psTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_TRACE)
			&& (psTraceBufCtl->ui32LogType & RGXFWIF_LOG_TYPE_GROUP_MASK)
			&& !PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST))
	{
		return IMG_TRUE;
	}

	return IMG_FALSE;
}

/*************************************************************************/ /*!
@Function       RGXTraceBufferDeinit

@Description    Deinitialises all the allocations and references that are made
		for the FW trace buffer(s)

@Input          ppsDevInfo	 RGX device info
@Return		void
 */ /**************************************************************************/
static void RGXTraceBufferDeinit(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_TRACEBUF*  psTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
	IMG_UINT32 i;

	for (i = 0; i < RGXFW_THREAD_NUM; i++)
	{
		if (psDevInfo->psRGXFWIfTraceBufferMemDesc[i])
		{
			if (psTraceBufCtl->sTraceBuf[i].pui32TraceBuffer != NULL)
			{
				DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufferMemDesc[i]);
				psTraceBufCtl->sTraceBuf[i].pui32TraceBuffer = NULL;
			}

			DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfTraceBufferMemDesc[i]);
			psDevInfo->psRGXFWIfTraceBufferMemDesc[i] = NULL;
		}
	}
}

/*************************************************************************/ /*!
@Function       RGXTraceBufferInitOnDemandResources

@Description    Allocates the firmware trace buffer required for dumping trace
		info from the firmware.

@Input          psDevInfo	 RGX device info

@Return		PVRSRV_OK	If all went good, PVRSRV_ERROR otherwise.
 */ /**************************************************************************/
PVRSRV_ERROR RGXTraceBufferInitOnDemandResources(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_TRACEBUF*  psTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;
	DEVMEM_FLAGS_T     uiMemAllocFlags;
	PVRSRV_ERROR       eError = PVRSRV_OK;
	IMG_UINT32         ui32FwThreadNum;
	IMG_UINT32         ui32DefaultTraceBufSize;
	IMG_DEVMEM_SIZE_T  uiTraceBufSizeInBytes;
	void               *pvAppHintState = NULL;

	/* Check AppHint value for module-param FWTraceBufSizeInDWords */
	OSCreateKMAppHintState(&pvAppHintState);
	ui32DefaultTraceBufSize = RGXFW_TRACE_BUF_DEFAULT_SIZE_IN_DWORDS;
	OSGetKMAppHintUINT32(pvAppHintState,
	                     FWTraceBufSizeInDWords,
						 &ui32DefaultTraceBufSize,
						 &psTraceBufCtl->ui32TraceBufSizeInDWords);
	OSFreeKMAppHintState(pvAppHintState);
	pvAppHintState = NULL;

	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	uiTraceBufSizeInBytes = psTraceBufCtl->ui32TraceBufSizeInDWords * sizeof(IMG_UINT32);

	for (ui32FwThreadNum = 0; ui32FwThreadNum < RGXFW_THREAD_NUM; ui32FwThreadNum++)
	{
		/* Ensure allocation API is only called when not already allocated */
		PVR_ASSERT(psDevInfo->psRGXFWIfTraceBufferMemDesc[ui32FwThreadNum] == NULL);

		PDUMPCOMMENT("Allocate rgxfw trace buffer(%u)", ui32FwThreadNum);
		eError = DevmemFwAllocate(psDevInfo,
		                          uiTraceBufSizeInBytes,
								  uiMemAllocFlags,
								  "FwTraceBuffer",
								  &psDevInfo->psRGXFWIfTraceBufferMemDesc[ui32FwThreadNum]);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate " IMG_DEVMEM_SIZE_FMTSPEC " bytes for fw trace buffer %u (Error code:%u)",
					__func__,
					uiTraceBufSizeInBytes,
					ui32FwThreadNum,
					eError));
			goto fail;
		}

		/* Firmware address should not be already set */
		PVR_ASSERT(psTraceBufCtl->sTraceBuf[ui32FwThreadNum].pui32RGXFWIfTraceBuffer.ui32Addr == 0x0);

		/* for the FW to use this address when dumping in log (trace) buffer */
		RGXSetFirmwareAddress(&psTraceBufCtl->sTraceBuf[ui32FwThreadNum].pui32RGXFWIfTraceBuffer,
				psDevInfo->psRGXFWIfTraceBufferMemDesc[ui32FwThreadNum],
				0, RFW_FWADDR_NOREF_FLAG);
		/* Set an address for the host to be able to read fw trace buffer */
		eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufferMemDesc[ui32FwThreadNum],
				(void **)&psTraceBufCtl->sTraceBuf[ui32FwThreadNum].pui32TraceBuffer);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire kernel tracebuf (%u) ctl (Error code: %u)",
					__func__, ui32FwThreadNum, eError));
			goto fail;
		}
	}

	return PVRSRV_OK;
	fail:
	RGXTraceBufferDeinit(psDevInfo);
	return eError;
}

static PVRSRV_ERROR RGXSetupOSConfig(PVRSRV_RGXDEV_INFO  *psDevInfo,
		RGXFWIF_INIT        *psRGXFWInit,
		IMG_UINT32           ui32ConfigFlags,
		IMG_UINT32           ui32ConfigFlagsExt,
		RGXFWIF_DEV_VIRTADDR sTracebufCtl,
		PRGXFWIF_HWRINFOBUF  sRGXFWIfHWRInfoBufCtl)
{
	PVRSRV_ERROR       eError = PVRSRV_OK;
	DEVMEM_FLAGS_T     uiMemAllocFlags;
	RGXFWIF_OS_CONFIG *psOSConfig;

	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;


	PDUMPCOMMENT("Allocate RGXFW_OS_CONFIG structure");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_OS_CONFIG),
			uiMemAllocFlags,
			"FwOSConfigStructure",
			&psDevInfo->psRGXFWIfOSConfigDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate %u bytes for OS Config (%u)",
				__func__,
				(IMG_UINT32)sizeof(RGXFWIF_OS_CONFIG),
				eError));
		goto fail1;
	}

	RGXSetFirmwareAddress(&psRGXFWInit->sOSConfig,
			psDevInfo->psRGXFWIfOSConfigDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfOSConfigDesc,
			(void **)&psOSConfig);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire OS Config (%u)",
				__func__,
				eError));
		goto fail2;
	}

	psOSConfig->ui32ConfigFlags    = ui32ConfigFlags & RGXFWIF_INICFG_ALL;
	psOSConfig->ui32ConfigFlagsExt = ui32ConfigFlagsExt;

	eError = SyncPrimGetFirmwareAddr(psDevInfo->psPowSyncPrim, &psOSConfig->sPowerSync.ui32Addr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to get Sync Prim FW address with error (%u)",
				__func__, eError));
		goto fail2;
	}

	psDevInfo->psFWIfOSConfig = psOSConfig;

	/* Set the Tracebuf and HWRInfoBufCtl offsets */
	psOSConfig->sTraceBufCtl               = sTracebufCtl;
	psOSConfig->sRGXFWIfHWRInfoBufCtl      = sRGXFWIfHWRInfoBufCtl;

#if defined(PDUMP)
	PDUMPCOMMENT("Dump initial state of RGXFW_OS_CONFIG structure");
	DevmemPDumpLoadMem(psDevInfo->psRGXFWIfOSConfigDesc,
			0,
			sizeof(RGXFWIF_OS_CONFIG),
			PDUMP_FLAGS_CONTINUOUS);
#endif

	return PVRSRV_OK;

	fail2:
	DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfOSConfigDesc);
	fail1:
	return eError;
}

#if defined(MTK_GPU_BM_SUPPORT)
static PVRSRV_ERROR MTKJobStatusResources(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	DEVMEM_FLAGS_T     uiMemAllocFlags;
	PVRSRV_ERROR       eError = PVRSRV_OK;
	int                size = 4096;
	IMG_BOOL           valid;

	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT)
				| PVRSRV_MEMALLOCFLAG_GPU_READABLE
				| PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE
				| PVRSRV_MEMALLOCFLAG_CPU_READABLE
				| PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE
				| PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE
				| PVRSRV_MEMALLOCFLAG_UNCACHED;

	eError = DevmemFwAllocate(psDevInfo,
							size,
							uiMemAllocFlags,
							"FwJobStatusBuf",
							&psDevInfo->psJobStatusQOSBufMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate %d bytes for jobstatus (%u)",
				__func__, size, eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psDevInfo->psRGXFWIfRuntimeCfg->sJobStatsQOS,
						psDevInfo->psJobStatusQOSBufMemDesc,
						0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psJobStatusQOSBufMemDesc,
									  (void **)&psDevInfo->psJobStatusQOSBuf);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire cpu_addr (%u)",
				__func__, eError));
		goto fail;
	}

	eError = PMR_CpuPhysAddr(
			psDevInfo->psJobStatusQOSBufMemDesc->psImport->hPMR,
			1, 1, 0,
			&psDevInfo->ui32JobStatusQOSPhyAddr,
			&valid);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire phy_addr (%u)",
				__func__, eError));
		goto fail;
	}

	MTKGPUQoS_setup(
		(uint32_t *)psDevInfo->psJobStatusQOSBuf,
		psDevInfo->ui32JobStatusQOSPhyAddr.uiAddr, size);

fail:
	return eError;
}
#endif

/*!
 *******************************************************************************

 @Function	RGXSetupFirmware

 @Description

 Setups all the firmware related data

 @Input psDevInfo

 @Return PVRSRV_ERROR

 ******************************************************************************/
PVRSRV_ERROR RGXSetupFirmware(PVRSRV_DEVICE_NODE       *psDeviceNode,
		IMG_BOOL                 bEnableSignatureChecks,
		IMG_UINT32               ui32SignatureChecksBufSize,
		IMG_UINT32               ui32HWPerfFWBufSizeKB,
		IMG_UINT64               ui64HWPerfFilter,
		IMG_UINT32               ui32RGXFWAlignChecksArrLength,
		IMG_UINT32               *pui32RGXFWAlignChecks,
		IMG_UINT32               ui32ConfigFlags,
		IMG_UINT32               ui32ConfigFlagsExt,
		IMG_UINT32               ui32LogType,
		RGXFWIF_BIFTILINGMODE    eBifTilingMode,
		IMG_UINT32               ui32NumTilingCfgs,
		IMG_UINT32               *pui32BIFTilingXStrides,
		IMG_UINT32               ui32FilterFlags,
		IMG_UINT32               ui32JonesDisableMask,
		IMG_UINT32               ui32HWRDebugDumpLimit,
		IMG_UINT32               ui32HWPerfCountersDataSize,
		IMG_UINT32               *pui32TPUTrilinearFracMask,
		RGX_RD_POWER_ISLAND_CONF eRGXRDPowerIslandConf,
		FW_PERF_CONF             eFirmwarePerf)

{
	PVRSRV_ERROR		eError;
	DEVMEM_FLAGS_T		uiMemAllocFlags;
	RGXFWIF_INIT		*psRGXFWInitScratch = NULL;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	IMG_UINT32		dm;
	IMG_UINT32		ui32kCCBSize = (!PVRSRV_VZ_MODE_IS(DRIVER_MODE_NATIVE) &&
			!(psDevInfo->sDevFeatureCfg.ui64Features & RGX_FEATURE_GPU_VIRTUALISATION_BIT_MASK)) ?\
					(RGXFWIF_KCCB_NUMCMDS_LOG2_GPUVIRT_WITHOUT_FEATURE) : (RGXFWIF_KCCB_NUMCMDS_LOG2_DEFAULT);
#if defined(SUPPORT_PDVFS)
	RGXFWIF_PDVFS_OPP   *psPDVFSOPPInfo;
	IMG_DVFS_DEVICE_CFG *psDVFSDeviceCfg;
#endif

	/* Fw init data */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
			PVRSRV_MEMALLOCFLAG_FW_LOCAL |
			PVRSRV_MEMALLOCFLAG_FW_CONFIG;
	/* FIXME: Change to Cached */


	PDUMPCOMMENT("Allocate RGXFWIF_INIT structure");

	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_INIT),
			uiMemAllocFlags,
			"FwInitStructure",
			&psDevInfo->psRGXFWIfInitMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %zu bytes for fw if ctl (%u)",
				__func__,
				sizeof(RGXFWIF_INIT),
				eError));
		goto fail;
	}

	psRGXFWInitScratch = OSAllocZMem(sizeof(*psRGXFWInitScratch));

	if (psRGXFWInitScratch == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate RGXFWInit scratch structure",
				__func__));
		goto fail;
	}

	/* Setup FW coremem data */
	if (psDevInfo->psRGXFWIfCorememDataStoreMemDesc)
	{
		IMG_BOOL bMetaDMA = RGX_IS_FEATURE_SUPPORTED(psDevInfo, META_DMA);

		psRGXFWInitScratch->sCorememDataStore.pbyFWAddr = psDevInfo->sFWCorememDataStoreFWAddr;

		if (bMetaDMA)
		{
			RGXSetMetaDMAAddress(&psRGXFWInitScratch->sCorememDataStore,
					psDevInfo->psRGXFWIfCorememDataStoreMemDesc,
					&psRGXFWInitScratch->sCorememDataStore.pbyFWAddr,
					0);
		}
	}

	/* init HW frame info */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw HW info buffer");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_HWRINFOBUF),
			uiMemAllocFlags,
			"FwHWInfoBuffer",
			&psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %zu bytes for HW info (%u)",
				__func__,
				sizeof(RGXFWIF_HWRINFOBUF),
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sRGXFWIfHWRInfoBufCtl,
			psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc,
			(void **)&psDevInfo->psRGXFWIfHWRInfoBuf);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire kernel tracebuf ctl (%u)",
				__func__,
				eError));
		goto fail;
	}

	/* Might be uncached. Be conservative and use a DeviceMemSet */
	OSDeviceMemSet(psDevInfo->psRGXFWIfHWRInfoBuf, 0, sizeof(RGXFWIF_HWRINFOBUF));

	/* Allocate a sync for power management */
	eError = SyncPrimContextCreate(psDevInfo->psDeviceNode,
			&psDevInfo->hSyncPrimContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate sync primitive context with error (%u)",
				__func__,
				eError));
		goto fail;
	}

	eError = SyncPrimAlloc(psDevInfo->hSyncPrimContext, &psDevInfo->psPowSyncPrim, "fw power ack");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate sync primitive with error (%u)",
				__func__,
				eError));

		goto fail;
	}

	/* Setup Fault read register */
	eError = RGXSetupFaultReadRegister(psDeviceNode, psRGXFWInitScratch);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to setup fault read register",
				__func__));
		goto fail;
	}

	/* Allocation flags for the kernel CCB */
	uiMemAllocFlags  =   PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(FIRMWARE_CACHED) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Set up kernel CCB */
	eError = RGXSetupCCB(psDevInfo,
			&psDevInfo->psKernelCCBCtl,
			&psDevInfo->psKernelCCBCtlMemDesc,
			&psDevInfo->psKernelCCB,
			&psDevInfo->psKernelCCBMemDesc,
			&psRGXFWInitScratch->psKernelCCBCtl,
			&psRGXFWInitScratch->psKernelCCB,
			ui32kCCBSize,
			sizeof(RGXFWIF_KCCB_CMD),
			uiMemAllocFlags,
			"FwKernelCCB");

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate Kernel CCB", __func__));
		goto fail;
	}

	/* Allocation flags for the firmware and checkpoint CCB */
	uiMemAllocFlags  =   PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Set up firmware CCB */
	eError = RGXSetupCCB(psDevInfo,
			&psDevInfo->psFirmwareCCBCtl,
			&psDevInfo->psFirmwareCCBCtlMemDesc,
			&psDevInfo->psFirmwareCCB,
			&psDevInfo->psFirmwareCCBMemDesc,
			&psRGXFWInitScratch->psFirmwareCCBCtl,
			&psRGXFWInitScratch->psFirmwareCCB,
			RGXFWIF_FWCCB_NUMCMDS_LOG2,
			sizeof(RGXFWIF_FWCCB_CMD),
			uiMemAllocFlags,
			"FwCCB");

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate Firmware CCB", __func__));
		goto fail;
	}

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
	/* Set up checkpoint CCB */
	eError = RGXSetupCCB(psDevInfo,
			&psDevInfo->psCheckpointCCBCtl,
			&psDevInfo->psCheckpointCCBCtlMemDesc,
			&psDevInfo->psCheckpointCCB,
			&psDevInfo->psCheckpointCCBMemDesc,
			&psRGXFWInitScratch->psCheckpointCCBCtl,
			&psRGXFWInitScratch->psCheckpointCCB,
			RGXFWIF_CHECKPOINTCCB_NUMCMDS_LOG2,
			sizeof(PRGXFWIF_UFO_ADDR),
			uiMemAllocFlags,
			"FwChptCCB");

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate Checkpoint CCB", __func__));
		goto fail;
	}
#endif /* defined(PVRSRV_SYNC_CHECKPOINT_CCB) */

	/* RD Power Island */
	{
		RGX_DATA *psRGXData = (RGX_DATA*) psDeviceNode->psDevConfig->hDevData;
		IMG_BOOL bSysEnableRDPowIsland = psRGXData->psRGXTimingInfo->bEnableRDPowIsland;
		IMG_BOOL bEnableRDPowIsland = ((eRGXRDPowerIslandConf == RGX_RD_POWER_ISLAND_DEFAULT) && bSysEnableRDPowIsland) ||
				(eRGXRDPowerIslandConf == RGX_RD_POWER_ISLAND_FORCE_ON);

		ui32ConfigFlags |= bEnableRDPowIsland? RGXFWIF_INICFG_POW_RASCALDUST : 0;
	}

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
	ui32ConfigFlags |= RGXFWIF_INICFG_WORKEST_V2;
#if defined(SUPPORT_PDVFS)
	/* Pro-active DVFS depends on Workload Estimation */
	psPDVFSOPPInfo = &psRGXFWInitScratch->sPDVFSOPPInfo;
	psDVFSDeviceCfg = &psDeviceNode->psDevConfig->sDVFS.sDVFSDeviceCfg;
	PVR_LOG_IF_FALSE(psDVFSDeviceCfg->pasOPPTable, "RGXSetupFirmware: Missing OPP Table");

	if (psDVFSDeviceCfg->pasOPPTable != NULL)
	{
		if (psDVFSDeviceCfg->ui32OPPTableSize > ARRAY_SIZE(psPDVFSOPPInfo->asOPPValues))
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: OPP Table too large: Size = %u, Maximum size = %lu",
					__func__,
					psDVFSDeviceCfg->ui32OPPTableSize,
					(unsigned long)(ARRAY_SIZE(psPDVFSOPPInfo->asOPPValues))));
			eError = PVRSRV_ERROR_INVALID_PARAMS;
			goto fail;
		}

		OSDeviceMemCopy(psPDVFSOPPInfo->asOPPValues,
				psDVFSDeviceCfg->pasOPPTable,
				sizeof(psPDVFSOPPInfo->asOPPValues));

		psPDVFSOPPInfo->ui32MaxOPPPoint = psDVFSDeviceCfg->ui32OPPTableSize - 1;

		ui32ConfigFlags |= RGXFWIF_INICFG_PDVFS_V2;

		/* Tell the FW that the Host might use the reactive timer.
		 * The FW might clear this flag if the timer can be run on the FW instead. */
		ui32ConfigFlagsExt |= RGXFWIF_INICFG_EXT_PDVFS_HOST_REACTIVE_TIMER;
	}
#endif
#endif

	/* FW trace control structure */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw trace control structure");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_TRACEBUF),
			uiMemAllocFlags,
			"FwTraceCtlStruct",
			&psDevInfo->psRGXFWIfTraceBufCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to allocate %zu bytes for fw trace (%u)",
				__func__,
				sizeof(RGXFWIF_TRACEBUF),
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sTraceBufCtl,
			psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			(void **)&psDevInfo->psRGXFWIfTraceBuf);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire kernel tracebuf ctl (%u)",
				__func__,
				eError));
		goto fail;
	}

	/* Set initial firmware log type/group(s) */
	if (ui32LogType & ~RGXFWIF_LOG_TYPE_MASK)
	{
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		PVR_DPF((PVR_DBG_ERROR,"%s: Invalid initial log type (0x%X)",
				__func__, ui32LogType));
		goto fail;
	}
	psDevInfo->psRGXFWIfTraceBuf->ui32LogType = ui32LogType;

#if !defined(PDUMP)
	/* When PDUMP is enabled, ALWAYS allocate on-demand trace buffer resource
	 * (irrespective of loggroup(s) enabled), given that logtype/loggroups can
	 * be set during PDump playback in logconfig, at any point of time,
	 * Otherwise, allocate only if required. */
	if (RGXTraceBufferIsInitRequired(psDevInfo))
#endif
	{
		eError = RGXTraceBufferInitOnDemandResources(psDevInfo);
	}

	PVR_LOGG_IF_ERROR(eError, "RGXTraceBufferInitOnDemandResources", fail);

	eError = RGXSetupOSConfig(psDevInfo, psRGXFWInitScratch, ui32ConfigFlags, ui32ConfigFlagsExt,
	                          psRGXFWInitScratch->sTraceBufCtl, psRGXFWInitScratch->sRGXFWIfHWRInfoBufCtl);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to set up the per OS configuration",
				__func__));
		goto fail;
	}

	psRGXFWInitScratch->eGPIOValidationMode = RGXFWIF_GPIO_VAL_OFF;
#if defined(SUPPORT_VALIDATION)
	{
		IMG_INT32 ui32AppHintDefault;
		IMG_INT32 ui32GPIOValidationMode;
		void      *pvAppHintState = NULL;

		/* Check AppHint for GPIO validation mode */
		OSCreateKMAppHintState(&pvAppHintState);
		ui32AppHintDefault = PVRSRV_APPHINT_GPIOVALIDATIONMODE;
		OSGetKMAppHintUINT32(pvAppHintState,
				GPIOValidationMode,
				&ui32AppHintDefault,
				&ui32GPIOValidationMode);
		OSFreeKMAppHintState(pvAppHintState);
		pvAppHintState = NULL;

		if (ui32GPIOValidationMode >= RGXFWIF_GPIO_VAL_LAST)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: Invalid GPIO validation mode: %d, only valid if smaller than %d. Disabling GPIO validation.",
					__func__,
					ui32GPIOValidationMode,
					RGXFWIF_GPIO_VAL_LAST));
		}
		else
		{
			psRGXFWInitScratch->eGPIOValidationMode = (RGXFWIF_GPIO_VAL_MODE) ui32GPIOValidationMode;
		}

		psRGXFWInitScratch->eGPIOValidationMode = ui32GPIOValidationMode;
	}
#endif

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
	uiMemAllocFlags = PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/* Set up Workload Estimation firmware CCB */
	eError = RGXSetupCCB(psDevInfo,
			&psDevInfo->psWorkEstFirmwareCCBCtl,
			&psDevInfo->psWorkEstFirmwareCCBCtlMemDesc,
			&psDevInfo->psWorkEstFirmwareCCB,
			&psDevInfo->psWorkEstFirmwareCCBMemDesc,
			&psRGXFWInitScratch->psWorkEstFirmwareCCBCtl,
			&psRGXFWInitScratch->psWorkEstFirmwareCCB,
			RGXFWIF_WORKEST_FWCCB_NUMCMDS_LOG2,
			sizeof(RGXFWIF_WORKEST_FWCCB_CMD),
			uiMemAllocFlags,
			"FwWEstCCB");

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate Workload Estimation Firmware CCB",
				__func__));
		goto fail;
	}
#endif

#if defined(SUPPORT_POWER_SAMPLING_VIA_DEBUGFS)

	eError = RGXFWSetupCounterBuffer(psDevInfo,
			&psDevInfo->psCounterBufferMemDesc,
			PAGE_SIZE,
			&psRGXFWInitScratch->sCounterDumpCtl,
			"CounterBuffer");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate counter buffer",
				__func__));
		goto fail;
	}

#endif

#if defined(SUPPORT_FIRMWARE_GCOV)

	eError = RGXFWSetupFirmwareGcovBuffer(psDevInfo,
			&psDevInfo->psFirmwareGcovBufferMemDesc,
			RGXFWIF_FIRMWARE_GCOV_BUFFER_SIZE,
			&psRGXFWInitScratch->sFirmwareGcovCtl,
			"FirmwareGcovBuffer");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate firmware gcov buffer",
				__func__));
		goto fail;
	}

	psDevInfo->ui32FirmwareGcovSize = RGXFWIF_FIRMWARE_GCOV_BUFFER_SIZE;

#endif

	/* Require a minimum amount of memory for the signature buffers */
	if (ui32SignatureChecksBufSize < RGXFW_SIG_BUFFER_SIZE_MIN)
	{
		ui32SignatureChecksBufSize = RGXFW_SIG_BUFFER_SIZE_MIN;
	}

	/* Setup Signature and Checksum Buffers for TA and 3D */
	eError = RGXFWSetupSignatureChecks(psDevInfo,
			&psDevInfo->psRGXFWSigTAChecksMemDesc,
			ui32SignatureChecksBufSize,
			&psRGXFWInitScratch->asSigBufCtl[RGXFWIF_DM_TA],
			"TA");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to setup TA signature checks",
				__func__));
		goto fail;
	}
	psDevInfo->ui32SigTAChecksSize = ui32SignatureChecksBufSize;

	eError = RGXFWSetupSignatureChecks(psDevInfo,
			&psDevInfo->psRGXFWSig3DChecksMemDesc,
			ui32SignatureChecksBufSize,
			&psRGXFWInitScratch->asSigBufCtl[RGXFWIF_DM_3D],
			"3D");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to setup 3D signature checks",
				__func__));
		goto fail;
	}
	psDevInfo->ui32Sig3DChecksSize = ui32SignatureChecksBufSize;

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, TDM_PDS_CHECKSUM))
	{
		/* Buffer allocated only when feature present because, all known TDM
		 * signature registers are dependent on this feature being present */
		eError = RGXFWSetupSignatureChecks(psDevInfo,
				&psDevInfo->psRGXFWSigTDM2DChecksMemDesc,
				ui32SignatureChecksBufSize,
				&psRGXFWInitScratch->asSigBufCtl[RGXFWIF_DM_TDM],
				"TDM");
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: Failed to setup TDM signature checks",
					__func__));
			goto fail;
		}
		psDevInfo->ui32SigTDM2DChecksSize = ui32SignatureChecksBufSize;
	}
	else
	{
		psDevInfo->psRGXFWSigTDM2DChecksMemDesc = NULL;
		psDevInfo->ui32SigTDM2DChecksSize = 0;
	}

#if defined(RGXFW_ALIGNCHECKS)
	eError = RGXFWSetupAlignChecks(psDevInfo,
			&psRGXFWInitScratch->sAlignChecks,
			pui32RGXFWAlignChecks,
			ui32RGXFWAlignChecksArrLength);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to setup alignment checks",
				__func__));
		goto fail;
	}
#endif

	psRGXFWInitScratch->ui32FilterFlags = ui32FilterFlags;


	if (RGX_IS_BRN_SUPPORTED(psDevInfo, 65273))
	{
		/* Fill the remaining bits of fw the init data */
		psRGXFWInitScratch->sPDSExecBase.uiAddr = RGX_PDSCODEDATA_BRN_65273_HEAP_BASE;
		psRGXFWInitScratch->sUSCExecBase.uiAddr = RGX_USCCODE_BRN_65273_HEAP_BASE;
	}
	else
	{
		/* Fill the remaining bits of fw the init data */
		psRGXFWInitScratch->sPDSExecBase.uiAddr = RGX_PDSCODEDATA_HEAP_BASE;
		psRGXFWInitScratch->sUSCExecBase.uiAddr = RGX_USCCODE_HEAP_BASE;
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, S7_TOP_INFRASTRUCTURE))
	{
		psRGXFWInitScratch->ui32JonesDisableMask = ui32JonesDisableMask;
	}
	psDevInfo->bPDPEnabled = (ui32ConfigFlags & RGXFWIF_INICFG_DISABLE_PDP_EN)
									? IMG_FALSE : IMG_TRUE;
	psRGXFWInitScratch->ui32HWRDebugDumpLimit = ui32HWRDebugDumpLimit;

	psRGXFWInitScratch->eFirmwarePerf = eFirmwarePerf;

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, SLC_VIVT))
	{
		eError = _AllocateSLC3Fence(psDevInfo, psRGXFWInitScratch);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: Failed to allocate memory for SLC3Fence",
					__func__));
			goto fail;
		}
	}


	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META) &&
			((ui32ConfigFlags & RGXFWIF_INICFG_METAT1_ENABLED) != 0))
	{
		/* Allocate a page for T1 stack */
		eError = DevmemFwAllocate(psDevInfo,
				RGX_META_STACK_SIZE,
				RGX_FWCOMCTX_ALLOCFLAGS,
				"FwMETAT1Stack",
				& psDevInfo->psMETAT1StackMemDesc);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: Failed to allocate T1 Stack",
					__func__));
			goto fail;
		}

		RGXSetFirmwareAddress(&psRGXFWInitScratch->sT1Stack,
				psDevInfo->psMETAT1StackMemDesc,
				0, RFW_FWADDR_NOREF_FLAG);

		PVR_DPF((PVR_DBG_MESSAGE,
				"%s: T1 Stack Frame allocated at %x",
				__func__,
				psRGXFWInitScratch->sT1Stack.ui32Addr));
	}

#if defined(SUPPORT_PDVFS)
	/* Core clock rate */
	uiMemAllocFlags =
			PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	eError = DevmemFwAllocate(psDevInfo,
			sizeof(IMG_UINT32),
			uiMemAllocFlags,
			"FwCoreClkRate",
			&psDevInfo->psRGXFWIFCoreClkRateMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate PDVFS core clock rate",
				__func__));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sCoreClockRate,
			psDevInfo->psRGXFWIFCoreClkRateMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	PVR_DPF((PVR_DBG_MESSAGE, "%s: PDVFS core clock rate allocated at %x",
			__func__,
			psRGXFWInitScratch->sCoreClockRate.ui32Addr));

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIFCoreClkRateMemDesc,
			(void **)&psDevInfo->pui32RGXFWIFCoreClkRate);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire core clk rate (%u)",
				__func__,
				eError));
		goto fail;
	}
#endif


#if !defined(PDUMP)
	/* allocate only if required */
	if (RGXTBIBufferIsInitRequired(psDevInfo))
#endif
	{
		/* When PDUMP is enabled, ALWAYS allocate on-demand TBI buffer resource
		 * (irrespective of loggroup(s) enabled), given that logtype/loggroups
		 * can be set during PDump playback in logconfig, at any point of time
		 */
		eError = RGXTBIBufferInitOnDemandResources(psDevInfo);
	}

	PVR_LOGG_IF_ERROR(eError, "RGXTBIBufferInitOnDemandResources", fail);
	psRGXFWInitScratch->sTBIBuf = psDevInfo->sRGXFWIfTBIBuffer;

	/* Allocate shared buffer for GPU utilisation */
	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate shared buffer for GPU utilisation");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_GPU_UTIL_FWCB),
			uiMemAllocFlags,
			"FwGPUUtilisationBuffer",
			&psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %zu bytes for GPU utilisation buffer ctl (%u)",
				__func__,
				sizeof(RGXFWIF_GPU_UTIL_FWCB),
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sGpuUtilFWCbCtl,
			psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc,
			(void **)&psDevInfo->psRGXFWIfGpuUtilFWCb);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire kernel GPU utilisation buffer ctl (%u)",
				__func__,
				eError));
		goto fail;
	}

	/* Initialise GPU utilisation buffer */
	psDevInfo->psRGXFWIfGpuUtilFWCb->ui64LastWord =
			RGXFWIF_GPU_UTIL_MAKE_WORD(OSClockns64(),RGXFWIF_GPU_UTIL_STATE_IDLE);


	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw FW runtime configuration (FW)");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_RUNTIME_CFG),
			uiMemAllocFlags,
			"FwRuntimeCfg",
			&psDevInfo->psRGXFWIfRuntimeCfgMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %zu bytes for FW runtime configuration (%u)",
				__func__,
				sizeof(RGXFWIF_RUNTIME_CFG),
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sRuntimeCfg,
			psDevInfo->psRGXFWIfRuntimeCfgMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);

	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfRuntimeCfgMemDesc,
			(void **)&psDevInfo->psRGXFWIfRuntimeCfg);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire kernel FW runtime configuration (%u)",
				__func__,
				eError));
		goto fail;
	}
	/* HWPerf: Determine the size of the FW buffer */
	if (ui32HWPerfFWBufSizeKB == 0 ||
			ui32HWPerfFWBufSizeKB == RGXFW_HWPERF_L1_SIZE_DEFAULT)
	{
		/* Under pvrsrvctl 0 size implies AppHint not set or is set to zero,
		 * use default size from driver constant. Set it to the default
		 * size, no logging.
		 */
		psDevInfo->ui32RGXFWIfHWPerfBufSize = RGXFW_HWPERF_L1_SIZE_DEFAULT<<10;
	}
	else if (ui32HWPerfFWBufSizeKB > (RGXFW_HWPERF_L1_SIZE_MAX))
	{
		/* Size specified as a AppHint but it is too big */
		PVR_DPF((PVR_DBG_WARNING,
				"%s: HWPerfFWBufSizeInKB value (%u) too big, using maximum (%u)",
				__func__,
				ui32HWPerfFWBufSizeKB, RGXFW_HWPERF_L1_SIZE_MAX));
		psDevInfo->ui32RGXFWIfHWPerfBufSize = RGXFW_HWPERF_L1_SIZE_MAX<<10;
	}
	else if (ui32HWPerfFWBufSizeKB > (RGXFW_HWPERF_L1_SIZE_MIN))
	{
		/* Size specified as in AppHint HWPerfFWBufSizeInKB */
		PVR_DPF((PVR_DBG_WARNING,
				"%s: Using HWPerf FW buffer size of %u KB",
				__func__,
				ui32HWPerfFWBufSizeKB));
		psDevInfo->ui32RGXFWIfHWPerfBufSize = ui32HWPerfFWBufSizeKB<<10;
	}
	else
	{
		/* Size specified as a AppHint but it is too small */
		PVR_DPF((PVR_DBG_WARNING,
				"%s: HWPerfFWBufSizeInKB value (%u) too small, using minimum (%u)",
				__func__,
				ui32HWPerfFWBufSizeKB, RGXFW_HWPERF_L1_SIZE_MIN));
		psDevInfo->ui32RGXFWIfHWPerfBufSize = RGXFW_HWPERF_L1_SIZE_MIN<<10;
	}

	/* init HWPERF data */
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfRIdx = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfWIdx = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfWrapCount = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfSize = psDevInfo->ui32RGXFWIfHWPerfBufSize;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfUt = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32HWPerfDropCount = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32FirstDropOrdinal = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32LastDropOrdinal = 0;
	psDevInfo->psRGXFWIfTraceBuf->ui32PowMonEstimate = 0;

	/* Second stage initialisation or HWPerf, hHWPerfLock created in first
	 * stage. See RGXRegisterDevice() call to RGXHWPerfInit(). */
	if (psDevInfo->ui64HWPerfFilter == 0)
	{
		psDevInfo->ui64HWPerfFilter = ui64HWPerfFilter;
		psRGXFWInitScratch->ui64HWPerfFilter = ui64HWPerfFilter;
	}
	else
	{
		/* The filter has already been modified. This can happen if
		 * pvr/apphint/EnableFTraceGPU was enabled. */
		psRGXFWInitScratch->ui64HWPerfFilter = psDevInfo->ui64HWPerfFilter;
	}

	/*Send through the BVNC Feature Flags*/
	eError = RGXServerFeatureFlagsToHWPerfFlags(psDevInfo, &psRGXFWInitScratch->sBvncKmFeatureFlags);
	PVR_LOGG_IF_ERROR(eError, "RGXServerFeatureFlagsToHWPerfFlags", fail);

#if !defined(PDUMP)
	/* Allocate if HWPerf filter has already been set. This is possible either
	 * by setting a proper AppHint or enabling GPU ftrace events. */
	if (psDevInfo->ui64HWPerfFilter != 0)
#endif
	{
		/* When PDUMP is enabled, ALWAYS allocate on-demand HWPerf resources
		 * (irrespective of HWPerf enabled or not), given that HWPerf can be
		 * enabled during PDump playback via RTCONF at any point of time. */
		eError = RGXHWPerfInitOnDemandResources(psDevInfo);
		PVR_LOGG_IF_ERROR(eError, "RGXHWPerfInitOnDemandResources", fail);
	}

#if defined(MTK_GPU_BM_SUPPORT)
	MTKJobStatusResources(psDevInfo);
#endif

	RGXHWPerfInitAppHintCallbacks(psDeviceNode);

#if defined(SUPPORT_USER_REGISTER_CONFIGURATION)
	PDUMPCOMMENT("Allocate rgxfw register configuration structure");
	eError = DevmemFwAllocate(psDevInfo,
			sizeof(RGXFWIF_REG_CFG),
			uiMemAllocFlags | PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE,
			"FwRegisterConfigStructure",
			&psDevInfo->psRGXFWIfRegCfgMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %zu bytes for fw register configurations (%u)",
				__func__,
				sizeof(RGXFWIF_REG_CFG),
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sRegCfg,
			psDevInfo->psRGXFWIfRegCfgMemDesc,
			0, RFW_FWADDR_NOREF_FLAG);
#endif

	uiMemAllocFlags =	PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
			PVRSRV_MEMALLOCFLAG_GPU_READABLE |
			PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
			PVRSRV_MEMALLOCFLAG_CPU_READABLE |
			PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE |
			PVRSRV_MEMALLOCFLAG_UNCACHED |
			PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	PDUMPCOMMENT("Allocate rgxfw hwperfctl structure");
	eError = DevmemFwAllocate(psDevInfo,
			ui32HWPerfCountersDataSize,
			uiMemAllocFlags,
			"FwHWPerfControlStructure",
			&psDevInfo->psRGXFWIfHWPerfCountersMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to allocate %u bytes for fw hwperf control (%u)",
				__func__,
				ui32HWPerfCountersDataSize,
				eError));
		goto fail;
	}

	RGXSetFirmwareAddress(&psRGXFWInitScratch->sHWPerfCtl,
			psDevInfo->psRGXFWIfHWPerfCountersMemDesc,
			0, 0);

	/* Required info by FW to calculate the ActivePM idle timer latency */
	{
		RGX_DATA *psRGXData = (RGX_DATA*) psDeviceNode->psDevConfig->hDevData;
		RGXFWIF_RUNTIME_CFG *psRuntimeCfg = psDevInfo->psRGXFWIfRuntimeCfg;

		psRGXFWInitScratch->ui32InitialCoreClockSpeed = psRGXData->psRGXTimingInfo->ui32CoreClockSpeed;
		psRGXFWInitScratch->ui32ActivePMLatencyms = psRGXData->psRGXTimingInfo->ui32ActivePMLatencyms;

		/* Initialise variable runtime configuration to the system defaults */
		psRuntimeCfg->ui32CoreClockSpeed = psRGXFWInitScratch->ui32InitialCoreClockSpeed;
		psRuntimeCfg->ui32ActivePMLatencyms = psRGXFWInitScratch->ui32ActivePMLatencyms;
		psRuntimeCfg->bActivePMLatencyPersistant = IMG_TRUE;

		/* Initialize the DefaultDustsNumInit Field to Max Dusts */
		psRuntimeCfg->ui32DefaultDustsNumInit = psDevInfo->sDevFeatureCfg.ui32MAXDustCount;
	}
#if defined(PDUMP)
	PDUMPCOMMENT("Dump initial state of FW runtime configuration");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfRuntimeCfgMemDesc,
			0,
			sizeof(RGXFWIF_RUNTIME_CFG),
			PDUMP_FLAGS_CONTINUOUS);
#endif

	/* Initialize FW started flag */
	psRGXFWInitScratch->bFirmwareStarted = IMG_FALSE;
	psRGXFWInitScratch->ui32MarkerVal = 1;

	/* Initialise the compatibility check data */
	RGXFWIF_COMPCHECKS_BVNC_INIT(psRGXFWInitScratch->sRGXCompChecks.sFWBVNC);
	RGXFWIF_COMPCHECKS_BVNC_INIT(psRGXFWInitScratch->sRGXCompChecks.sHWBVNC);

	PDUMPCOMMENT("Dump RGXFW Init data");
	if (!bEnableSignatureChecks)
	{
#if defined(PDUMP)
		PDUMPCOMMENT("(to enable rgxfw signatures place the following line after the RTCONF line)");
		DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfInitMemDesc,
				offsetof(RGXFWIF_INIT, asSigBufCtl),
				sizeof(RGXFWIF_SIGBUF_CTL)*(RGXFWIF_DM_MAX),
				PDUMP_FLAGS_CONTINUOUS);
#endif
		psRGXFWInitScratch->asSigBufCtl[RGXFWIF_DM_3D].sBuffer.ui32Addr = 0x0;
		psRGXFWInitScratch->asSigBufCtl[RGXFWIF_DM_TA].sBuffer.ui32Addr = 0x0;
	}

	for (dm = 0; dm < (RGXFWIF_DM_MAX); dm++)
	{
		psDevInfo->psRGXFWIfTraceBuf->aui32HwrDmLockedUpCount[dm] = 0;
		psDevInfo->psRGXFWIfTraceBuf->aui32HwrDmOverranCount[dm] = 0;
		psDevInfo->psRGXFWIfTraceBuf->aui32HwrDmRecoveredCount[dm] = 0;
		psDevInfo->psRGXFWIfTraceBuf->aui32HwrDmFalseDetectCount[dm] = 0;
	}

	/*
	 * BIF Tiling configuration
	 */

	psRGXFWInitScratch->eBifTilingMode = eBifTilingMode;

	psRGXFWInitScratch->sBifTilingCfg[0].uiBase = RGX_BIF_TILING_HEAP_1_BASE;
	psRGXFWInitScratch->sBifTilingCfg[0].uiLen = RGX_BIF_TILING_HEAP_SIZE;
	psRGXFWInitScratch->sBifTilingCfg[0].uiXStride = pui32BIFTilingXStrides[0];
	psRGXFWInitScratch->sBifTilingCfg[1].uiBase = RGX_BIF_TILING_HEAP_2_BASE;
	psRGXFWInitScratch->sBifTilingCfg[1].uiLen = RGX_BIF_TILING_HEAP_SIZE;
	psRGXFWInitScratch->sBifTilingCfg[1].uiXStride = pui32BIFTilingXStrides[1];
	psRGXFWInitScratch->sBifTilingCfg[2].uiBase = RGX_BIF_TILING_HEAP_3_BASE;
	psRGXFWInitScratch->sBifTilingCfg[2].uiLen = RGX_BIF_TILING_HEAP_SIZE;
	psRGXFWInitScratch->sBifTilingCfg[2].uiXStride = pui32BIFTilingXStrides[2];
	psRGXFWInitScratch->sBifTilingCfg[3].uiBase = RGX_BIF_TILING_HEAP_4_BASE;
	psRGXFWInitScratch->sBifTilingCfg[3].uiLen = RGX_BIF_TILING_HEAP_SIZE;
	psRGXFWInitScratch->sBifTilingCfg[3].uiXStride = pui32BIFTilingXStrides[3];

#if defined(SUPPORT_VALIDATION)
	/*
	 * TPU trilinear rounding mask override
	 */
	for (dm = 0; dm < RGXFWIF_TPU_DM_LAST; dm++)
	{
		psRGXFWInitScratch->aui32TPUTrilinearFracMask[dm] = pui32TPUTrilinearFracMask[dm];
	}
#endif

	/* update the FW structure proper */
	eError = DevmemAcquireCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc,
			(void **) &psDevInfo->psRGXFWIfInit);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to acquire kernel fw if ctl (%u)",
				__func__,
				eError));
		goto fail;
	}

	OSDeviceMemCopy(psDevInfo->psRGXFWIfInit, psRGXFWInitScratch,
	                sizeof(*psDevInfo->psRGXFWIfInit));

#if defined(PDUMP)
	PDUMPCOMMENT("Dump rgxfw hwperfctl structure");
	DevmemPDumpLoadZeroMem (psDevInfo->psRGXFWIfHWPerfCountersMemDesc,
			0,
			ui32HWPerfCountersDataSize,
			PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("Dump rgxfw trace control structure");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			0,
			sizeof(RGXFWIF_TRACEBUF),
			PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("Dump rgx TBI buffer");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfTBIBufferMemDesc,
			0,
			psDevInfo->ui32FWIfTBIBufferSize,
			PDUMP_FLAGS_CONTINUOUS);

#if defined(SUPPORT_USER_REGISTER_CONFIGURATION)
	PDUMPCOMMENT("Dump rgxfw register configuration buffer");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfRegCfgMemDesc,
			0,
			sizeof(RGXFWIF_REG_CFG),
			PDUMP_FLAGS_CONTINUOUS);
#endif
	PDUMPCOMMENT("Dump rgxfw init structure");
	DevmemPDumpLoadMem(	psDevInfo->psRGXFWIfInitMemDesc,
			0,
			sizeof(RGXFWIF_INIT),
			PDUMP_FLAGS_CONTINUOUS);

	/* RGXFW Init structure needs to be loaded before we overwrite FaultPhysAddr, else this address patching won't have any effect */
	PDUMPCOMMENT("Overwrite FaultPhysAddr of FWInit in pdump with actual physical address");
	RGXPDumpFaultReadRegister(psDevInfo);

	PDUMPCOMMENT("RTCONF: run-time configuration");


	/* Dump the config options so they can be edited.
	 *
	 * FIXME: Need new DevmemPDumpWRW API which writes a WRW to load ui32ConfigFlags
	 */
	PDUMPCOMMENT("(Set the FW config options here)");
	PDUMPCOMMENT("( Ctx Switch TA Enable: 0x%08x)", RGXFWIF_INICFG_CTXSWITCH_TA_EN);
	PDUMPCOMMENT("( Ctx Switch 3D Enable: 0x%08x)", RGXFWIF_INICFG_CTXSWITCH_3D_EN);
	PDUMPCOMMENT("( Ctx Switch CDM Enable: 0x%08x)", RGXFWIF_INICFG_CTXSWITCH_CDM_EN);
	PDUMPCOMMENT("( Ctx Switch Rand mode: 0x%08x)", RGXFWIF_INICFG_CTXSWITCH_MODE_RAND);
	PDUMPCOMMENT("( Ctx Switch Soft Reset Enable: 0x%08x)", RGXFWIF_INICFG_CTXSWITCH_SRESET_EN);
	PDUMPCOMMENT("( Rascal+Dust Power Island: 0x%08x)", RGXFWIF_INICFG_POW_RASCALDUST);
	PDUMPCOMMENT("( Enable HWPerf: 0x%08x)", RGXFWIF_INICFG_HWPERF_EN);
	PDUMPCOMMENT("( Enable HWR: 0x%08x)", RGXFWIF_INICFG_HWR_EN);
	PDUMPCOMMENT("( Check MList: 0x%08x)", RGXFWIF_INICFG_CHECK_MLIST_EN);
	PDUMPCOMMENT("( Disable Auto Clock Gating: 0x%08x)", RGXFWIF_INICFG_DISABLE_CLKGATING_EN);
	PDUMPCOMMENT("( Enable HWPerf Polling Perf Counter: 0x%08x)", RGXFWIF_INICFG_POLL_COUNTERS_EN);

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, VDM_OBJECT_LEVEL_LLS))
	{
		PDUMPCOMMENT("( Ctx Switch Object mode Index: 0x%08x)", RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INDEX);
		PDUMPCOMMENT("( Ctx Switch Object mode Instance: 0x%08x)", RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INSTANCE);
		PDUMPCOMMENT("( Ctx Switch Object mode List: 0x%08x)", RGXFWIF_INICFG_VDM_CTX_STORE_MODE_LIST);
	}

	PDUMPCOMMENT("( Enable register configuration: 0x%08x)", RGXFWIF_INICFG_REGCONFIG_EN);
	PDUMPCOMMENT("( Assert on TA Out-of-Memory: 0x%08x)", RGXFWIF_INICFG_ASSERT_ON_OUTOFMEMORY);
	PDUMPCOMMENT("( Disable HWPerf custom counter filter: 0x%08x)", RGXFWIF_INICFG_HWP_DISABLE_FILTER);
	PDUMPCOMMENT("( Enable HWPerf custom performance timer: 0x%08x)", RGXFWIF_INICFG_CUSTOM_PERF_TIMER_EN);
	PDUMPCOMMENT("( Enable CDM Killing Rand mode: 0x%08x)", RGXFWIF_INICFG_CDM_KILL_MODE_RAND_EN);
	PDUMPCOMMENT("( Enable Ctx Switch profile mode: 0x%08x (none=b'000, fast=b'001, medium=b'010, slow=b'011, nodelay=b'100))", RGXFWIF_INICFG_CTXSWITCH_PROFILE_MASK);
	PDUMPCOMMENT("( Disable DM overlap (except TA during SPM): 0x%08x)", RGXFWIF_INICFG_DISABLE_DM_OVERLAP);
	PDUMPCOMMENT("( Enable Meta T1 running main code: 0x%08x)", RGXFWIF_INICFG_METAT1_MAIN);
	PDUMPCOMMENT("( Enable Meta T1 running dummy code: 0x%08x)", RGXFWIF_INICFG_METAT1_DUMMY);
	PDUMPCOMMENT("( Assert on HWR trigger (page fault, lockup, overrun or poll failure): 0x%08x)", RGXFWIF_INICFG_ASSERT_ON_HWR_TRIGGER);

	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfOSConfigDesc,
			offsetof(RGXFWIF_OS_CONFIG, ui32ConfigFlags),
			ui32ConfigFlags,
			PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("( Extended FW config options start here )");
	PDUMPCOMMENT("( Lower Priority Ctx Switch  2D Enable: 0x%08x)", RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TDM);
	PDUMPCOMMENT("( Lower Priority Ctx Switch  TA Enable: 0x%08x)", RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TA);
	PDUMPCOMMENT("( Lower Priority Ctx Switch  3D Enable: 0x%08x)", RGXFWIF_INICFG_EXT_LOW_PRIO_CS_3D);
	PDUMPCOMMENT("( Lower Priority Ctx Switch CDM Enable: 0x%08x)", RGXFWIF_INICFG_EXT_LOW_PRIO_CS_CDM);

	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfOSConfigDesc,
			offsetof(RGXFWIF_OS_CONFIG, ui32ConfigFlagsExt),
			ui32ConfigFlagsExt,
			PDUMP_FLAGS_CONTINUOUS);

	/* default: no filter */
	psRGXFWInitScratch->sPIDFilter.eMode = RGXFW_PID_FILTER_INCLUDE_ALL_EXCEPT;
	psRGXFWInitScratch->sPIDFilter.asItems[0].uiPID = 0;

	PDUMPCOMMENT("( PID filter type: %X=INCLUDE_ALL_EXCEPT, %X=EXCLUDE_ALL_EXCEPT)",
			RGXFW_PID_FILTER_INCLUDE_ALL_EXCEPT,
			RGXFW_PID_FILTER_EXCLUDE_ALL_EXCEPT);

	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfInitMemDesc,
			offsetof(RGXFWIF_INIT, sPIDFilter.eMode),
			psRGXFWInitScratch->sPIDFilter.eMode,
			PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("( PID filter PID/OSID list (Up to %u entries. Terminate with a zero PID))",
			RGXFWIF_PID_FILTER_MAX_NUM_PIDS);
	{
		IMG_UINT32 i;

		/* generate a few WRWs in the pdump stream as an example */
		for (i = 0; i < MIN(RGXFWIF_PID_FILTER_MAX_NUM_PIDS, 8); i++)
		{
			/*
			 * Some compilers cannot cope with the uses of offsetof() below - the specific problem being the use of
			 * a non-const variable in the expression, which it needs to be const. Typical compiler output is
			 * "expression must have a constant value".
			 */
			const IMG_DEVMEM_OFFSET_T uiPIDOff
			= (IMG_DEVMEM_OFFSET_T)(uintptr_t)&(((RGXFWIF_INIT *)0)->sPIDFilter.asItems[i].uiPID);

			const IMG_DEVMEM_OFFSET_T uiOSIDOff
			= (IMG_DEVMEM_OFFSET_T)(uintptr_t)&(((RGXFWIF_INIT *)0)->sPIDFilter.asItems[i].ui32OSID);

			PDUMPCOMMENT("(PID and OSID pair %u)", i);

			PDUMPCOMMENT("(PID)");
			DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfInitMemDesc,
					uiPIDOff,
					0,
					PDUMP_FLAGS_CONTINUOUS);

			PDUMPCOMMENT("(OSID)");
			DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfInitMemDesc,
					uiOSIDOff,
					0,
					PDUMP_FLAGS_CONTINUOUS);
		}
	}

	/*
	 * Dump the log config so it can be edited.
	 */
	PDUMPCOMMENT("(Set the log config here)");
	PDUMPCOMMENT("( Log Type: set bit 0 for TRACE, reset for TBI)");
	PDUMPCOMMENT("( MAIN Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_MAIN);
	PDUMPCOMMENT("( MTS Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_MTS);
	PDUMPCOMMENT("( CLEANUP Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_CLEANUP);
	PDUMPCOMMENT("( CSW Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_CSW);
	PDUMPCOMMENT("( BIF Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_BIF);
	PDUMPCOMMENT("( PM Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_PM);
	PDUMPCOMMENT("( RTD Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_RTD);
	PDUMPCOMMENT("( SPM Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_SPM);
	PDUMPCOMMENT("( POW Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_POW);
	PDUMPCOMMENT("( HWR Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_HWR);
	PDUMPCOMMENT("( HWP Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_HWP);

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, META_DMA))
	{
		PDUMPCOMMENT("( DMA Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_DMA);
	}

	PDUMPCOMMENT("( MISC Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_MISC);
	PDUMPCOMMENT("( DEBUG Group Enable: 0x%08x)", RGXFWIF_LOG_TYPE_GROUP_DEBUG);
	DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfTraceBufCtlMemDesc,
			offsetof(RGXFWIF_TRACEBUF, ui32LogType),
			psDevInfo->psRGXFWIfTraceBuf->ui32LogType,
			PDUMP_FLAGS_CONTINUOUS);

	PDUMPCOMMENT("Set the HWPerf Filter config here");
	DevmemPDumpLoadMemValue64(psDevInfo->psRGXFWIfInitMemDesc,
			offsetof(RGXFWIF_INIT, ui64HWPerfFilter),
			psRGXFWInitScratch->ui64HWPerfFilter,
			PDUMP_FLAGS_CONTINUOUS);

#if defined(SUPPORT_USER_REGISTER_CONFIGURATION)
	PDUMPCOMMENT("(Number of registers configurations for types(byte index): pow on(%d), dust change(%d), ta(%d), 3d(%d), cdm(%d), tla(%d), TDM(%d))",\
			RGXFWIF_REG_CFG_TYPE_PWR_ON,\
			RGXFWIF_REG_CFG_TYPE_DUST_CHANGE,\
			RGXFWIF_REG_CFG_TYPE_TA,\
			RGXFWIF_REG_CFG_TYPE_3D,\
			RGXFWIF_REG_CFG_TYPE_CDM,\
			RGXFWIF_REG_CFG_TYPE_TLA,\
			RGXFWIF_REG_CFG_TYPE_TDM);

	{
		IMG_UINT32 i;

		/**
		 * Write 32 bits in each iteration as required by PDUMP WRW command.
		 */
		for (i = 0; i < RGXFWIF_REG_CFG_TYPE_ALL; i += sizeof(IMG_UINT32))
		{
			DevmemPDumpLoadMemValue32(psDevInfo->psRGXFWIfRegCfgMemDesc,
					offsetof(RGXFWIF_REG_CFG, aui8NumRegsType[i]),
					0,
					PDUMP_FLAGS_CONTINUOUS);
		}
	}

	PDUMPCOMMENT("(Set registers here: address, mask, value)");
	DevmemPDumpLoadMemValue64(psDevInfo->psRGXFWIfRegCfgMemDesc,
			offsetof(RGXFWIF_REG_CFG, asRegConfigs[0].ui64Addr),
			0,
			PDUMP_FLAGS_CONTINUOUS);
	DevmemPDumpLoadMemValue64(psDevInfo->psRGXFWIfRegCfgMemDesc,
			offsetof(RGXFWIF_REG_CFG, asRegConfigs[0].ui64Mask),
			0,
			PDUMP_FLAGS_CONTINUOUS);
	DevmemPDumpLoadMemValue64(psDevInfo->psRGXFWIfRegCfgMemDesc,
			offsetof(RGXFWIF_REG_CFG, asRegConfigs[0].ui64Value),
			0,
			PDUMP_FLAGS_CONTINUOUS);
#endif /* SUPPORT_USER_REGISTER_CONFIGURATION */
#endif /* PDUMP */

	/* Perform additional virtualisation initialisation */
	eError = RGXVzSetupFirmware(psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed RGXVzSetupFirmware", __func__));
		goto fail;
	}

	OSFreeMem(psRGXFWInitScratch);

	psDevInfo->bFirmwareInitialised = IMG_TRUE;

	return PVRSRV_OK;

	fail:
	if (psRGXFWInitScratch)
	{
		OSFreeMem(psRGXFWInitScratch);
	}

	RGXFreeFirmware(psDevInfo);

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*!
 *******************************************************************************

 @Function	RGXFreeFirmware

 @Description

 Frees all the firmware-related allocations

 @Input psDevInfo

 @Return PVRSRV_ERROR

 ******************************************************************************/
void RGXFreeFirmware(PVRSRV_RGXDEV_INFO	*psDevInfo)
{
	psDevInfo->bFirmwareInitialised = IMG_FALSE;

	RGXVzFreeFirmware(psDevInfo->psDeviceNode);

	RGXFreeCCB(psDevInfo,
	           &psDevInfo->psKernelCCBCtl,
	           &psDevInfo->psKernelCCBCtlMemDesc,
	           &psDevInfo->psKernelCCB,
	           &psDevInfo->psKernelCCBMemDesc);

	RGXFreeCCB(psDevInfo,
	           &psDevInfo->psFirmwareCCBCtl,
	           &psDevInfo->psFirmwareCCBCtlMemDesc,
	           &psDevInfo->psFirmwareCCB,
	           &psDevInfo->psFirmwareCCBMemDesc);

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
	RGXFreeCCB(psDevInfo,
	           &psDevInfo->psCheckpointCCBCtl,
	           &psDevInfo->psCheckpointCCBCtlMemDesc,
	           &psDevInfo->psCheckpointCCB,
	           &psDevInfo->psCheckpointCCBMemDesc);
#endif

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
	RGXFreeCCB(psDevInfo,
	           &psDevInfo->psWorkEstFirmwareCCBCtl,
	           &psDevInfo->psWorkEstFirmwareCCBCtlMemDesc,
	           &psDevInfo->psWorkEstFirmwareCCB,
	           &psDevInfo->psWorkEstFirmwareCCBMemDesc);
#endif

#if defined(RGXFW_ALIGNCHECKS)
	if (psDevInfo->psRGXFWAlignChecksMemDesc)
	{
		RGXFWFreeAlignChecks(psDevInfo);
	}
#endif

	if (psDevInfo->psRGXFWIfOSConfigDesc)
	{
		if (psDevInfo->psFWIfOSConfig)
		{
			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfOSConfigDesc);
			psDevInfo->psFWIfOSConfig = NULL;
		}

		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfOSConfigDesc);
		psDevInfo->psRGXFWIfOSConfigDesc = NULL;
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, TDM_PDS_CHECKSUM) &&
	    psDevInfo->psRGXFWSigTDM2DChecksMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWSigTDM2DChecksMemDesc);
		psDevInfo->psRGXFWSigTDM2DChecksMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWSigTAChecksMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWSigTAChecksMemDesc);
		psDevInfo->psRGXFWSigTAChecksMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWSig3DChecksMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWSig3DChecksMemDesc);
		psDevInfo->psRGXFWSig3DChecksMemDesc = NULL;
	}

#if defined(SUPPORT_POWER_SAMPLING_VIA_DEBUGFS)
	if (psDevInfo->psCounterBufferMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psCounterBufferMemDesc);
		psDevInfo->psCounterBufferMemDesc = NULL;
	}
#endif

#if defined(SUPPORT_FIRMWARE_GCOV)
	if (psDevInfo->psFirmwareGcovBufferMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psFirmwareGcovBufferMemDesc);
		psDevInfo->psFirmwareGcovBufferMemDesc = NULL;
	}
#endif

	RGXSetupFaultReadRegisterRollback(psDevInfo);

	if (psDevInfo->psPowSyncPrim != NULL)
	{
		SyncPrimFree(psDevInfo->psPowSyncPrim);
		psDevInfo->psPowSyncPrim = NULL;
	}

	if (psDevInfo->hSyncPrimContext != (IMG_HANDLE) NULL)
	{
		SyncPrimContextDestroy(psDevInfo->hSyncPrimContext);
		psDevInfo->hSyncPrimContext = (IMG_HANDLE) NULL;
	}

	if (psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc)
	{
		if (psDevInfo->psRGXFWIfGpuUtilFWCb != NULL)
		{
			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc);
			psDevInfo->psRGXFWIfGpuUtilFWCb = NULL;
		}
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc);
		psDevInfo->psRGXFWIfGpuUtilFWCbCtlMemDesc = NULL;
	}

	RGXHWPerfDeinit(psDevInfo);

	if (psDevInfo->psRGXFWIfRuntimeCfgMemDesc)
	{
		if (psDevInfo->psRGXFWIfRuntimeCfg != NULL)
		{
			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfRuntimeCfgMemDesc);
			psDevInfo->psRGXFWIfRuntimeCfg = NULL;
		}
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfRuntimeCfgMemDesc);
		psDevInfo->psRGXFWIfRuntimeCfgMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc)
	{
		if (psDevInfo->psRGXFWIfHWRInfoBuf != NULL)
		{
			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc);
			psDevInfo->psRGXFWIfHWRInfoBuf = NULL;
		}
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc);
		psDevInfo->psRGXFWIfHWRInfoBufCtlMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWIfCorememDataStoreMemDesc)
	{
		psDevInfo->psRGXFWIfCorememDataStoreMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWIfTraceBufCtlMemDesc)
	{
		if (psDevInfo->psRGXFWIfTraceBuf != NULL)
		{
			/* first deinit/free the tracebuffer allocation */
			RGXTraceBufferDeinit(psDevInfo);

			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
			psDevInfo->psRGXFWIfTraceBuf = NULL;
		}
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfTraceBufCtlMemDesc);
		psDevInfo->psRGXFWIfTraceBufCtlMemDesc = NULL;
	}

	if (psDevInfo->psRGXFWIfTBIBufferMemDesc)
	{
		RGXTBIBufferDeinit(psDevInfo);
	}

#if defined(SUPPORT_USER_REGISTER_CONFIGURATION)
	if (psDevInfo->psRGXFWIfRegCfgMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfRegCfgMemDesc);
		psDevInfo->psRGXFWIfRegCfgMemDesc = NULL;
	}
#endif
	if (psDevInfo->psRGXFWIfHWPerfCountersMemDesc)
	{
		RGXUnsetFirmwareAddress(psDevInfo->psRGXFWIfHWPerfCountersMemDesc);
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfHWPerfCountersMemDesc);
		psDevInfo->psRGXFWIfHWPerfCountersMemDesc = NULL;
	}
	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, SLC_VIVT))
	{
		_FreeSLC3Fence(psDevInfo);
	}

	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META) && (psDevInfo->psMETAT1StackMemDesc))
	{
		DevmemFwFree(psDevInfo, psDevInfo->psMETAT1StackMemDesc);
		psDevInfo->psMETAT1StackMemDesc = NULL;
	}

#if defined(SUPPORT_PDVFS)
	if (psDevInfo->psRGXFWIFCoreClkRateMemDesc)
	{
		if (psDevInfo->pui32RGXFWIFCoreClkRate != NULL)
		{
			DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIFCoreClkRateMemDesc);
			psDevInfo->pui32RGXFWIFCoreClkRate = NULL;
		}

		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIFCoreClkRateMemDesc);
		psDevInfo->psRGXFWIFCoreClkRateMemDesc = NULL;
	}
#endif


	if (psDevInfo->psRGXFWIfInit)
	{
		DevmemReleaseCpuVirtAddr(psDevInfo->psRGXFWIfInitMemDesc);
		psDevInfo->psRGXFWIfInit = NULL;
	}


	if (psDevInfo->psRGXFWIfInitMemDesc)
	{
		DevmemFwFree(psDevInfo, psDevInfo->psRGXFWIfInitMemDesc);
		psDevInfo->psRGXFWIfInitMemDesc = NULL;
	}
}


/******************************************************************************
 FUNCTION	: RGXAcquireKernelCCBSlot

 PURPOSE	: Attempts to obtain a slot in the Kernel CCB

 PARAMETERS	: psCCB - the CCB
			: Address of space if available, NULL otherwise

 RETURNS	: PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXAcquireKernelCCBSlot(DEVMEM_MEMDESC *psKCCBCtrlMemDesc,
		RGXFWIF_CCB_CTL	*psKCCBCtl,
		IMG_UINT32			*pui32Offset)
{
	IMG_UINT32	ui32OldWriteOffset, ui32NextWriteOffset;

	ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
	ui32NextWriteOffset = (ui32OldWriteOffset + 1) & psKCCBCtl->ui32WrapMask;

	/*
	 * Note: The MTS can queue up to 255 kicks (254 pending kicks and 1
	 * executing kick), hence the kernel CCB should not queue more than
	 * 254 commands.
	 */
	PVR_ASSERT(psKCCBCtl->ui32WrapMask < 255);

#if defined(PDUMP)
	/* Wait for sufficient CCB space to become available */
	PDUMPCOMMENTWITHFLAGS(0, "Wait for kCCB woff=%u", ui32NextWriteOffset);
	DevmemPDumpCBP(psKCCBCtrlMemDesc,
			offsetof(RGXFWIF_CCB_CTL, ui32ReadOffset),
			ui32NextWriteOffset,
			1,
			(psKCCBCtl->ui32WrapMask + 1));
#endif

	if (ui32NextWriteOffset == psKCCBCtl->ui32ReadOffset)
	{
		return PVRSRV_ERROR_KERNEL_CCB_FULL;
	}
	*pui32Offset = ui32NextWriteOffset;
	return PVRSRV_OK;
}

/******************************************************************************
 FUNCTION	: RGXPollKernelCCBSlot

 PURPOSE	: Poll for space in Kernel CCB

 PARAMETERS	: psCCB - the CCB
			: Address of space if available, NULL otherwise

 RETURNS	: PVRSRV_ERROR
 ******************************************************************************/
static PVRSRV_ERROR RGXPollKernelCCBSlot(DEVMEM_MEMDESC *psKCCBCtrlMemDesc,
		RGXFWIF_CCB_CTL	*psKCCBCtl)
{
	IMG_UINT32	ui32OldWriteOffset, ui32NextWriteOffset;

	ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
	ui32NextWriteOffset = (ui32OldWriteOffset + 1) & psKCCBCtl->ui32WrapMask;

	/*
	 * Note: The MTS can queue up to 255 kicks (254 pending kicks and 1
	 * executing kick), hence the kernel CCB should not queue more than
	 * 254 commands.
	 */
	PVR_ASSERT(psKCCBCtl->ui32WrapMask < 255);

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{

		if (ui32NextWriteOffset != psKCCBCtl->ui32ReadOffset)
		{
			return PVRSRV_OK;
		}
		{
			/*
			 * The following sanity check doesn't impact performance,
			 * since the CPU has to wait for the GPU anyway (full kernel CCB).
			 */
			if (PVRSRVGetPVRSRVData()->eServicesState != PVRSRV_SERVICES_STATE_OK)
			{
				return PVRSRV_ERROR_KERNEL_CCB_FULL;
			}
		}

		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	return PVRSRV_ERROR_KERNEL_CCB_FULL;
}

/******************************************************************************
 FUNCTION	: RGXGetCmdMemCopySize

 PURPOSE	: Calculates actual size of KCCB command getting used

 PARAMETERS	: eCmdType     Type of KCCB command

 RETURNS	: Returns actual size of KCCB command on success else zero
 ******************************************************************************/
static IMG_UINT32 RGXGetCmdMemCopySize(RGXFWIF_KCCB_CMD_TYPE eCmdType)
{
	/* First get offset of uCmdData inside the struct RGXFWIF_KCCB_CMD
	 * This will account alignment requirement of uCmdData union
	 *
	 * Then add command-data size depending on command type to calculate actual
	 * command size required to do mem copy
	 *
	 * NOTE: Make sure that uCmdData is the last member of RGXFWIF_KCCB_CMD struct.
	 */
	switch (eCmdType)
	{
		case RGXFWIF_KCCB_CMD_KICK:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_KCCB_CMD_KICK_DATA);
		}
		case RGXFWIF_KCCB_CMD_MMUCACHE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_MMUCACHEDATA);
		}
		case RGXFWIF_KCCB_CMD_BP:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_BPDATA);
		}
		case RGXFWIF_KCCB_CMD_SYNC:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_KCCB_CMD_SYNC_DATA);
		}
		case RGXFWIF_KCCB_CMD_SLCFLUSHINVAL:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_SLCFLUSHINVALDATA);
		}
		case RGXFWIF_KCCB_CMD_CLEANUP:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_CLEANUP_REQUEST);
		}
		case RGXFWIF_KCCB_CMD_POW:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_POWER_REQUEST);
		}
		case RGXFWIF_KCCB_CMD_ZSBUFFER_BACKING_UPDATE:
		case RGXFWIF_KCCB_CMD_ZSBUFFER_UNBACKING_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_ZSBUFFER_BACKING_DATA);
		}
		case RGXFWIF_KCCB_CMD_FREELIST_GROW_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_FREELIST_GS_DATA);
		}
		case RGXFWIF_KCCB_CMD_FREELISTS_RECONSTRUCTION_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_FREELISTS_RECONSTRUCTION_DATA);
		}
		case RGXFWIF_KCCB_CMD_NOTIFY_SIGNAL_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_SIGNAL_UPDATE_DATA);
		}
		case RGXFWIF_KCCB_CMD_NOTIFY_WRITE_OFFSET_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_WRITE_OFFSET_UPDATE_DATA);
		}
		case RGXFWIF_KCCB_CMD_FORCE_UPDATE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_KCCB_CMD_FORCE_UPDATE_DATA);
		}
#if defined(SUPPORT_USER_REGISTER_CONFIGURATION)
		case RGXFWIF_KCCB_CMD_REGCONFIG:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_REGCONFIG_DATA);
		}
#endif
		case RGXFWIF_KCCB_CMD_HWPERF_SELECT_CUSTOM_CNTRS:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_HWPERF_SELECT_CUSTOM_CNTRS);
		}
#if defined(SUPPORT_PDVFS)
		case RGXFWIF_KCCB_CMD_PDVFS_LIMIT_MAX_FREQ:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_PDVFS_MAX_FREQ_DATA);
		}
#endif
		case RGXFWIF_KCCB_CMD_OSID_PRIORITY_CHANGE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_OSID_PRIORITY_DATA);
		}
		case RGXFWIF_KCCB_CMD_HCS_SET_DEADLINE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_HCS_CTL);
		}
		case RGXFWIF_KCCB_CMD_OS_ISOLATION_GROUP_CHANGE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_OSID_ISOLATION_GROUP_DATA);
		}
		case RGXFWIF_KCCB_CMD_OS_ONLINE_STATE_CONFIGURE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_OS_STATE_CHANGE_DATA);
		}
		case RGXFWIF_KCCB_CMD_COUNTER_DUMP:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_COUNTER_DUMP_DATA);
		}
		case RGXFWIF_KCCB_CMD_SLCBPCTL:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_SLCBPCTLDATA);
		}
		case RGXFWIF_KCCB_CMD_HWPERF_UPDATE_CONFIG:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_HWPERF_CTRL);
		}
		case RGXFWIF_KCCB_CMD_HWPERF_CONFIG_ENABLE_BLKS:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_HWPERF_CONFIG_ENABLE_BLKS);
		}
		case RGXFWIF_KCCB_CMD_HWPERF_CTRL_BLKS:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_HWPERF_CTRL_BLKS);
		}
		case RGXFWIF_KCCB_CMD_CORECLKSPEEDCHANGE:
		{
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData) + sizeof(RGXFWIF_CORECLKSPEEDCHANGE_DATA);
		}
		case RGXFWIF_KCCB_CMD_HEALTH_CHECK:
		case RGXFWIF_KCCB_CMD_HWPERF_CONFIG_ENABLE_BLKS_DIRECT:
		case RGXFWIF_KCCB_CMD_LOGTYPE_UPDATE:
#if defined(SUPPORT_PDVFS)
		case RGXFWIF_KCCB_CMD_PDVFS_REQUEST_REACTIVE_UPDATE:
#endif
		case RGXFWIF_KCCB_CMD_STATEFLAGS_CTRL:
		{
			/* No command specific data */
			return offsetof(RGXFWIF_KCCB_CMD, uCmdData);
		}
		default:
		{
			/* Invalid (OR) Unused (OR) Newly added command type */
			return 0; /* Error */
		}
	}
}

static PVRSRV_ERROR RGXSendCommandRaw(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eKCCBType,
		RGXFWIF_KCCB_CMD	*psKCCBCmd,
		IMG_UINT32             uiPdumpFlags)
{
	PVRSRV_ERROR		eError;
	PVRSRV_DEVICE_NODE	*psDeviceNode = psDevInfo->psDeviceNode;
	RGXFWIF_CCB_CTL		*psKCCBCtl = psDevInfo->psKernelCCBCtl;
	IMG_UINT8			*pui8KCCB = psDevInfo->psKernelCCB;
	IMG_UINT32			ui32NewWriteOffset;
	IMG_UINT32			ui32OldWriteOffset = psKCCBCtl->ui32WriteOffset;
	IMG_UINT32			ui32CmdMemCopySize;

#if !defined(PDUMP)
	PVR_UNREFERENCED_PARAMETER(uiPdumpFlags);
#else
	IMG_BOOL bPdumpEnabled = IMG_FALSE;
	IMG_BOOL bPDumpPowTrans = PDUMPPOWCMDINTRANS();
	IMG_BOOL bContCaptureOn = PDumpIsContCaptureOn(); /* client connected or in pdump init phase */

	if (bContCaptureOn)
	{
		IMG_BOOL bIsInCaptureRange;

		PDumpIsCaptureFrameKM(&bIsInCaptureRange);
		bPdumpEnabled = (bIsInCaptureRange || PDUMP_IS_CONTINUOUS(uiPdumpFlags)) && !bPDumpPowTrans;

		/* in capture range */
		if (bPdumpEnabled)
		{
			if (!psDevInfo->bDumpedKCCBCtlAlready)
			{
				/* entering capture range */
				psDevInfo->bDumpedKCCBCtlAlready = IMG_TRUE;

				/* Wait for the live FW to catch up */
				PVR_DPF((PVR_DBG_MESSAGE, "%s: waiting on fw to catch-up, roff: %d, woff: %d",
						__func__,
						psKCCBCtl->ui32ReadOffset, ui32OldWriteOffset));
				PVRSRVPollForValueKM((IMG_UINT32 __iomem *)&psKCCBCtl->ui32ReadOffset, ui32OldWriteOffset, 0xFFFFFFFF);

				/* Dump Init state of Kernel CCB control (read and write offset) */
				PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Initial state of kernel CCB Control, roff: %d, woff: %d",
						psKCCBCtl->ui32ReadOffset, psKCCBCtl->ui32WriteOffset);

				DevmemPDumpLoadMem(psDevInfo->psKernelCCBCtlMemDesc,
						0,
						sizeof(RGXFWIF_CCB_CTL),
						PDUMP_FLAGS_CONTINUOUS);
			}
		}
	}
#endif

	psKCCBCmd->eDM = eKCCBType;

	PVR_ASSERT(sizeof(RGXFWIF_KCCB_CMD) == psKCCBCtl->ui32CmdSize);
	if (!OSLockIsLocked(psDeviceNode->hPowerLock))
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s called without power lock held!",
				__func__));
		PVR_ASSERT(OSLockIsLocked(psDeviceNode->hPowerLock));
	}

	/* Acquire a slot in the CCB */
	eError = RGXAcquireKernelCCBSlot(psDevInfo->psKernelCCBCtlMemDesc, psKCCBCtl, &ui32NewWriteOffset);
	if (eError != PVRSRV_OK)
	{
		goto _RGXSendCommandRaw_Exit;
	}

	/* Calculate actual size of command to optimize device mem copy */
	ui32CmdMemCopySize = RGXGetCmdMemCopySize(psKCCBCmd->eCmdType);
	PVR_LOGR_IF_FALSE(ui32CmdMemCopySize !=0, "RGXGetCmdMemCopySize failed", PVRSRV_ERROR_INVALID_CCB_COMMAND);

	/* Copy the command into the CCB */
	OSDeviceMemCopy(&pui8KCCB[ui32OldWriteOffset * psKCCBCtl->ui32CmdSize],
			psKCCBCmd, ui32CmdMemCopySize);

	/* ensure kCCB data is written before the offsets */
	OSWriteMemoryBarrier();

	/* Move past the current command */
	psKCCBCtl->ui32WriteOffset = ui32NewWriteOffset;
	/* Force a read-back to memory to avoid posted writes on certain buses */
	(void) psKCCBCtl->ui32WriteOffset;


#if defined(PDUMP)
	if (bContCaptureOn)
	{
		/* in capture range */
		if (bPdumpEnabled)
		{
			/* Dump new Kernel CCB content */
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Dump kCCB cmd for DM %d, woff = %d",
					eKCCBType,
					ui32OldWriteOffset);
			DevmemPDumpLoadMem(psDevInfo->psKernelCCBMemDesc,
					ui32OldWriteOffset * psKCCBCtl->ui32CmdSize,
					ui32CmdMemCopySize,
					PDUMP_FLAGS_CONTINUOUS);

			/* Dump new kernel CCB write offset */
			PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "Dump kCCBCtl woff (added new cmd for DM %d): %d",
					eKCCBType,
					ui32NewWriteOffset);
			DevmemPDumpLoadMem(psDevInfo->psKernelCCBCtlMemDesc,
					offsetof(RGXFWIF_CCB_CTL, ui32WriteOffset),
					sizeof(IMG_UINT32),
					uiPdumpFlags);

			/* mimic the read-back of the write from above */
			DevmemPDumpDevmemPol32(psDevInfo->psKernelCCBCtlMemDesc,
					offsetof(RGXFWIF_CCB_CTL, ui32WriteOffset),
					ui32NewWriteOffset,
					0xFFFFFFFF,
					PDUMP_POLL_OPERATOR_EQUAL,
					uiPdumpFlags);

		}
		/* out of capture range */
		else
		{
			eError = RGXPdumpDrainKCCB(psDevInfo, ui32OldWriteOffset);
			PVR_LOGG_IF_ERROR(eError, "RGXPdumpDrainKCCB", _RGXSendCommandRaw_Exit);
		}
	}
#endif


	PDUMPCOMMENTWITHFLAGS(uiPdumpFlags, "MTS kick for kernel CCB");
	/*
	 * Kick the MTS to schedule the firmware.
	 */
	{
		IMG_UINT32 ui32MTSRegVal;

		if (!PVRSRV_VZ_MODE_IS(DRIVER_MODE_NATIVE) &&
				!(RGX_IS_FEATURE_SUPPORTED(psDevInfo, GPU_VIRTUALISATION)))
		{
#if defined(SUPPORT_STRIP_RENDERING)
			ui32MTSRegVal = ((RGXFWIF_DM_GP + PVRSRV_VZ_DRIVER_OSID) & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK);
#else
			ui32MTSRegVal = ((RGXFWIF_DM_GP + PVRSRV_VZ_DRIVER_OSID) & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_COUNTED;
#endif
		}
		else
		{
#if defined(SUPPORT_STRIP_RENDERING)
			ui32MTSRegVal = (RGXFWIF_DM_GP & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK);
#else
			ui32MTSRegVal = (RGXFWIF_DM_GP & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_COUNTED;
#endif
		}


		__MTSScheduleWrite(psDevInfo, ui32MTSRegVal);

		PDUMPREG32(RGX_PDUMPREG_NAME, RGX_CR_MTS_SCHEDULE, ui32MTSRegVal, uiPdumpFlags);
	}

#if defined(NO_HARDWARE)
	/* keep the roff updated because fw isn't there to update it */
	psKCCBCtl->ui32ReadOffset = psKCCBCtl->ui32WriteOffset;
#endif

	_RGXSendCommandRaw_Exit:
	return eError;
}

/******************************************************************************
 FUNCTION	: _AllocDeferredCommand

 PURPOSE	: Allocate a KCCB command and add it to KCCB deferred list

 PARAMETERS	: psDevInfo	RGX device info
			: eKCCBType		Firmware Command type
			: psKCCBCmd		Firmware Command
			: uiPdumpFlags	Pdump flags

 RETURNS	: PVRSRV_OK	If all went good, PVRSRV_ERROR_RETRY otherwise.
 ******************************************************************************/
static PVRSRV_ERROR _AllocDeferredCommand(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eKCCBType,
		RGXFWIF_KCCB_CMD	*psKCCBCmd,
		IMG_UINT32		uiPdumpFlags)
{
	RGX_DEFERRED_KCCB_CMD *psDeferredCommand;

	psDeferredCommand = OSAllocMem(sizeof(*psDeferredCommand));

	if (!psDeferredCommand)
	{
		PVR_DPF((PVR_DBG_ERROR,"Deferring a KCCB command failed: allocation failure: requesting retry"));
		return PVRSRV_ERROR_RETRY;
	}

	psDeferredCommand->sKCCBcmd = *psKCCBCmd;
	psDeferredCommand->eDM = eKCCBType;
	psDeferredCommand->uiPdumpFlags = uiPdumpFlags;
	psDeferredCommand->psDevInfo = psDevInfo;

	OSLockAcquire(psDevInfo->hLockKCCBDeferredCommandsList);
	dllist_add_to_tail(&(psDevInfo->sKCCBDeferredCommandsListHead), &(psDeferredCommand->sListNode));
	psDevInfo->ui32KCCBDeferredCommandsCount++;
	OSLockRelease(psDevInfo->hLockKCCBDeferredCommandsList);

	return PVRSRV_OK;
}

/******************************************************************************
 FUNCTION	: _FreeDeferredCommand

 PURPOSE	: Remove from the deferred list the sent deferred KCCB command

 PARAMETERS	: psNode			Node in deferred list
			: psDeferredKCCBCmd	KCCB Command to free

 RETURNS	: None
 ******************************************************************************/
static void _FreeDeferredCommand(DLLIST_NODE *psNode, RGX_DEFERRED_KCCB_CMD *psDeferredKCCBCmd)
{
	dllist_remove_node(psNode);
	psDeferredKCCBCmd->psDevInfo->ui32KCCBDeferredCommandsCount--;
	OSFreeMem(psDeferredKCCBCmd);
}

/******************************************************************************
 FUNCTION	: RGXSendCommandsFromDeferredList

 PURPOSE	: Try send KCCB commands in deferred list to KCCB
 		  Should be called by holding PowerLock

 PARAMETERS	: psDevInfo	RGX device info
		: bPoll		Poll for space in KCCB

 RETURNS	: PVRSRV_OK	If all commands in deferred list are sent to KCCB,
			  PVRSRV_ERROR_KERNEL_CCB_FULL otherwise.
 ******************************************************************************/
PVRSRV_ERROR RGXSendCommandsFromDeferredList(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_BOOL bPoll)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	DLLIST_NODE *psNode, *psNext;
	RGX_DEFERRED_KCCB_CMD *psTempDeferredKCCBCmd;

	OSLockAcquire(psDevInfo->hLockKCCBDeferredCommandsList);

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		if (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHead))
		{
			OSLockRelease(psDevInfo->hLockKCCBDeferredCommandsList);
			return PVRSRV_OK;
		}

		/* For every deferred KCCB command, try to send it*/
		dllist_foreach_node(&psDevInfo->sKCCBDeferredCommandsListHead, psNode, psNext)
		{
			psTempDeferredKCCBCmd = IMG_CONTAINER_OF(psNode, RGX_DEFERRED_KCCB_CMD, sListNode);
			eError = RGXSendCommandRaw(psTempDeferredKCCBCmd->psDevInfo,
					psTempDeferredKCCBCmd->eDM,
					&(psTempDeferredKCCBCmd->sKCCBcmd),
					psTempDeferredKCCBCmd->uiPdumpFlags);
			if (eError == PVRSRV_OK)
			{
				_FreeDeferredCommand(psNode, psTempDeferredKCCBCmd);
			}
			else
			{
				if (bPoll)
				{
					break;
				}
				else
				{
					OSLockRelease(psDevInfo->hLockKCCBDeferredCommandsList);
					return PVRSRV_ERROR_KERNEL_CCB_FULL;
				}
			}
		}

		if (bPoll)
		{
			eError = RGXPollKernelCCBSlot(psDevInfo->psKernelCCBCtlMemDesc, psDevInfo->psKernelCCBCtl);
			if (eError == PVRSRV_ERROR_KERNEL_CCB_FULL)
			{
				OSLockRelease(psDevInfo->hLockKCCBDeferredCommandsList);
				return PVRSRV_ERROR_KERNEL_CCB_FULL;
			}
		}

	} END_LOOP_UNTIL_TIMEOUT();

	OSLockRelease(psDevInfo->hLockKCCBDeferredCommandsList);
	return PVRSRV_OK;
}

PVRSRV_ERROR RGXSendCommand(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eKCCBType,
		RGXFWIF_KCCB_CMD	*psKCCBCmd,
		IMG_UINT32		uiPdumpFlags)
{

	PVRSRV_ERROR eError;
	IMG_BOOL	bPoll = IMG_FALSE;

	if (eKCCBType == RGXFWIF_DM_GP)
	{
		/* Do not defer GP cmds as server will poll for its completion anyway */
		bPoll = IMG_TRUE;
	}

	/* First try to Flush all the cmds in deferred list */
	eError = RGXSendCommandsFromDeferredList(psDevInfo, bPoll);
	if (eError == PVRSRV_OK)
	{
		eError = RGXSendCommandRaw(psDevInfo,
				eKCCBType,
				psKCCBCmd,
				uiPdumpFlags);
	}
	/*
	 * If we don't manage to enqueue one of the deferred commands or the command
	 * passed as argument because the KCCB is full, insert the latter into the deferred commands list.
	 * The deferred commands will also be flushed eventually by:
	 *  - one more KCCB command sent for any DM
	 *  - RGX_MISRHandler_CheckFWActivePowerState
	 */
	if (eError == PVRSRV_ERROR_KERNEL_CCB_FULL)
	{
		eError = _AllocDeferredCommand(psDevInfo, eKCCBType, psKCCBCmd, uiPdumpFlags);
	}
	return eError;
}

PVRSRV_ERROR RGXSendCommandWithPowLock(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eKCCBType,
		RGXFWIF_KCCB_CMD	*psKCCBCmd,
		IMG_UINT32			ui32PDumpFlags)
{
	PVRSRV_ERROR		eError;
	PVRSRV_DEVICE_NODE *psDeviceNode = psDevInfo->psDeviceNode;

	/* Ensure Rogue is powered up before kicking MTS */
	eError = PVRSRVPowerLock(psDeviceNode);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING,
				"%s: failed to acquire powerlock (%s)",
				__func__,
				PVRSRVGetErrorString(eError)));

		goto _PVRSRVPowerLock_Exit;
	}

	PDUMPPOWCMDSTART();
	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode,
			PVRSRV_DEV_POWER_STATE_ON,
			IMG_FALSE);
	PDUMPPOWCMDEND();

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: failed to transition Rogue to ON (%s)",
				__func__,
				PVRSRVGetErrorString(eError)));

		goto _PVRSRVSetDevicePowerStateKM_Exit;
	}

	eError = RGXSendCommand(psDevInfo, eKCCBType,  psKCCBCmd, ui32PDumpFlags);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: failed to schedule command (%s)",
				__func__,
				PVRSRVGetErrorString(eError)));
#if defined(DEBUG)
		/* PVRSRVDebugRequest must be called without powerlock */
		PVRSRVPowerUnlock(psDeviceNode);
		PVRSRVDebugRequest(psDeviceNode, DEBUG_REQUEST_VERBOSITY_MAX, NULL, NULL);
		goto _PVRSRVPowerLock_Exit;
#endif
	}

	_PVRSRVSetDevicePowerStateKM_Exit:
	PVRSRVPowerUnlock(psDeviceNode);

	_PVRSRVPowerLock_Exit:
	return eError;
}

void RGXScheduleProcessQueuesKM(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = (PVRSRV_DEVICE_NODE*) hCmdCompHandle;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	OSScheduleMISR(psDevInfo->hProcessQueuesMISR);
}

/*!
 ******************************************************************************

 @Function	RGX_MISRHandler_ScheduleProcessQueues

 @Description - Sends uncounted kick to all the DMs (the FW will process all
				the queue for all the DMs)
 ******************************************************************************/
static void RGX_MISRHandler_ScheduleProcessQueues(void *pvData)
{
	PVRSRV_DEVICE_NODE     *psDeviceNode = pvData;
	PVRSRV_RGXDEV_INFO     *psDevInfo = psDeviceNode->pvDevice;
	PVRSRV_ERROR           eError;
	PVRSRV_DEV_POWER_STATE ePowerState;

	/* We don't need to acquire the BridgeLock as this power transition won't
	   send a command to the FW */
	eError = PVRSRVPowerLock(psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "RGXScheduleProcessQueuesKM: failed to acquire powerlock (%s)",
				PVRSRVGetErrorString(eError)));
		return;
	}

	/* Check whether it's worth waking up the GPU */
	eError = PVRSRVGetDevicePowerState(psDeviceNode, &ePowerState);

	if (!PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST) &&
			(eError == PVRSRV_OK) && (ePowerState == PVRSRV_DEV_POWER_STATE_OFF))
	{
		/* For now, guest drivers will always wake-up the GPU */
		RGXFWIF_GPU_UTIL_FWCB  *psUtilFWCb = psDevInfo->psRGXFWIfGpuUtilFWCb;
		IMG_BOOL               bGPUHasWorkWaiting;

		bGPUHasWorkWaiting =
				(RGXFWIF_GPU_UTIL_GET_STATE(psUtilFWCb->ui64LastWord) == RGXFWIF_GPU_UTIL_STATE_BLOCKED);

		if (!bGPUHasWorkWaiting)
		{
			/* all queues are empty, don't wake up the GPU */
			PVRSRVPowerUnlock(psDeviceNode);
			return;
		}
	}

	PDUMPPOWCMDSTART();
	/* wake up the GPU */
	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode,
			PVRSRV_DEV_POWER_STATE_ON,
			IMG_FALSE);
	PDUMPPOWCMDEND();

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "RGXScheduleProcessQueuesKM: failed to transition Rogue to ON (%s)",
				PVRSRVGetErrorString(eError)));

		PVRSRVPowerUnlock(psDeviceNode);
		return;
	}

	/* uncounted kick to the FW */
	{
		IMG_UINT32 ui32MTSRegVal;

		if (!PVRSRV_VZ_MODE_IS(DRIVER_MODE_NATIVE) &&
				!(RGX_IS_FEATURE_SUPPORTED(psDevInfo, GPU_VIRTUALISATION)))
		{
			ui32MTSRegVal = ((RGXFWIF_DM_GP + PVRSRV_VZ_DRIVER_OSID) & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_NON_COUNTED;
		}
		else
		{
			ui32MTSRegVal = (RGXFWIF_DM_GP & ~RGX_CR_MTS_SCHEDULE_DM_CLRMSK) | RGX_CR_MTS_SCHEDULE_TASK_NON_COUNTED;
		}

		HTBLOGK(HTB_SF_MAIN_KICK_UNCOUNTED);
		__MTSScheduleWrite(psDevInfo, ui32MTSRegVal);
	}

	PVRSRVPowerUnlock(psDeviceNode);
}

PVRSRV_ERROR RGXInstallProcessQueuesMISR(IMG_HANDLE *phMISR, PVRSRV_DEVICE_NODE *psDeviceNode)
{
	return OSInstallMISR(phMISR,
			RGX_MISRHandler_ScheduleProcessQueues,
			psDeviceNode,
			"RGX_ScheduleProcessQueues");
}

/*!
 ******************************************************************************

 @Function	RGXScheduleCommand

 @Description - Submits a CCB command and kicks the firmware but first schedules
                any commands which have to happen before handle

 @Input psDevInfo		 - pointer to device info
 @Input eKCCBType		 - see RGXFWIF_CMD_*
 @Input psKCCBCmd		 - kernel CCB command
 @Input ui32CacheOpFence - CPU dcache operation fence
 @Input ui32PDumpFlags - PDUMP_FLAGS_CONTINUOUS bit set if the pdump flags should be continuous


 @Return PVRSRV_ERROR

 ******************************************************************************/
PVRSRV_ERROR RGXScheduleCommand(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eKCCBType,
		RGXFWIF_KCCB_CMD	*psKCCBCmd,
		IMG_UINT32			ui32CacheOpFence,
		IMG_UINT32			ui32PDumpFlags)
{
	PVRSRV_ERROR eError;
	IMG_UINT16 uiMMUSyncUpdate;

	eError = CacheOpFence(eKCCBType, ui32CacheOpFence);
	if (unlikely(eError != PVRSRV_OK)) goto RGXScheduleCommand_exit;

#if defined(SUPPORT_VALIDATION)
	/* For validation, force the core to different dust count states with each kick */
	if ((eKCCBType == RGXFWIF_DM_TA) || (eKCCBType == RGXFWIF_DM_CDM))
	{
		if (psDevInfo->ui32DeviceFlags & RGXKM_DEVICE_STATE_DUST_REQUEST_INJECT_EN)
		{
			IMG_UINT32 ui32NumDusts = RGXGetNextDustCount(&psDevInfo->sDustReqState, psDevInfo->sDevFeatureCfg.ui32MAXDustCount);
			PVRSRVDeviceDustCountChange(psDevInfo->psDeviceNode, ui32NumDusts);
		}
	}
#endif

	/* PVRSRVPowerLock guarantees atomicity between commands. This is helpful
	   in a scenario with several applications allocating resources. */
	eError = PVRSRVPowerLock(psDevInfo->psDeviceNode);
	if (unlikely(eError != PVRSRV_OK))
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: failed to acquire powerlock (%s)",
				__func__, PVRSRVGetErrorString(eError)));

		/* If system is found powered OFF, Retry scheduling the command */
		if(likely(eError == PVRSRV_ERROR_SYSTEM_STATE_POWERED_OFF))
		{
			eError = PVRSRV_ERROR_RETRY;
		}

		goto RGXScheduleCommand_exit;
	}

	/* Ensure device is powered up before sending any commands */
	PDUMPPOWCMDSTART();
	eError = PVRSRVSetDevicePowerStateKM(psDevInfo->psDeviceNode,
			PVRSRV_DEV_POWER_STATE_ON,
			IMG_FALSE);
	PDUMPPOWCMDEND();
	if (unlikely(eError != PVRSRV_OK))
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: failed to transition RGX to ON (%s)",
				__func__, PVRSRVGetErrorString(eError)));
		goto _PVRSRVSetDevicePowerStateKM_Exit;
	}

	eError = RGXPreKickCacheCommand(psDevInfo, eKCCBType, &uiMMUSyncUpdate, IMG_FALSE);
	if (unlikely(eError != PVRSRV_OK)) goto _PVRSRVSetDevicePowerStateKM_Exit;

	eError = RGXSendCommand(psDevInfo, eKCCBType, psKCCBCmd, ui32PDumpFlags);
	if (unlikely(eError != PVRSRV_OK)) goto _PVRSRVSetDevicePowerStateKM_Exit;

	_PVRSRVSetDevicePowerStateKM_Exit:
	PVRSRVPowerUnlock(psDevInfo->psDeviceNode);

	RGXScheduleCommand_exit:
	return eError;
}

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
/*
 * RGXCheckCheckpointCCB
 */
void RGXCheckCheckpointCCB(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	IMG_BOOL bSignal = IMG_FALSE;

	PRGXFWIF_UFO_ADDR *psFwUFOAddr;
	RGXFWIF_CCB_CTL *psChptCCBCtl = psDevInfo->psCheckpointCCBCtl;
	IMG_UINT8 *psChptCCB = psDevInfo->psCheckpointCCB;
	IMG_UINT32 ui32WriteOffset, ui32ReadOffset, ui32WrapMask = psChptCCBCtl->ui32WrapMask;
	IMG_UINT32 uiFwAddr;
	PVRSRV_SYNC_CHECKPOINT_STATE uiChptState;

	/*
	 * Check if the firmware has signalled a full sync state check.
	 */
	if (psDevInfo->psRGXFWIfTraceBuf->ui32FWSyncCheckMark != psDevInfo->psRGXFWIfTraceBuf->ui32HostSyncCheckMark)
	{
		/*
		 * Update the offsets first so that if the firmware tries to write
		 * another checkpoint it is not missed by the check state.
		 */
		psDevInfo->psRGXFWIfTraceBuf->ui32HostSyncCheckMark = psDevInfo->psRGXFWIfTraceBuf->ui32FWSyncCheckMark;
		psChptCCBCtl->ui32ReadOffset = psChptCCBCtl->ui32WriteOffset;

		PVR_DPF((PVR_DBG_MESSAGE, "%s: Checkpoint CCB full, performing full sync checkpoint state check", __func__));

		SyncCheckpointCheckState();
		bSignal = IMG_TRUE;

#if defined(SUPPORT_BUFFER_SYNC)
		pvr_buffer_sync_check_state();
#endif

		goto exit_signal;
	}

	/*
	 * Take a snapshot of the current CCB ctl pointers at the start of
	 * processing.
	 */
	ui32WriteOffset = psChptCCBCtl->ui32WriteOffset;
	ui32ReadOffset = psChptCCBCtl->ui32ReadOffset;
	ui32WrapMask = psChptCCBCtl->ui32WrapMask;

	while (ui32ReadOffset != ui32WriteOffset)
	{
		/* Point to the next checkpoint address */
		psFwUFOAddr = ((PRGXFWIF_UFO_ADDR *)psChptCCB) + ui32ReadOffset;

		/*
		 * State is encoded in bit 1 of ufo address
		 * 1 = signalled, 0 = errored
		 */
		uiChptState = PVRSRV_SYNC_CHECKPOINT_ERRORED;
		uiFwAddr = psFwUFOAddr->ui32Addr;

		if (uiFwAddr & 0x1U)
		{
			uiChptState = PVRSRV_SYNC_CHECKPOINT_SIGNALLED;
		}
		uiFwAddr |= 0x1U;

		if (SyncCheckpointUFOHasSignalled(psDeviceNode, uiFwAddr, uiChptState))
		{
			bSignal = IMG_TRUE;
		}
		else
#if defined(SUPPORT_BUFFER_SYNC)
		if (pvr_buffer_sync_checkpoint_ufo_has_signalled(uiFwAddr, uiChptState))
		{
			/* Buffer sync does not need a signal call. */
		}
		else
#endif
		{
			PVR_DPF((PVR_DBG_MESSAGE, "%s: Firmware signalled checkpoint (%#08X) with no host backing", __func__, uiFwAddr));
		}

		/* Update read offset */
		ui32ReadOffset = (ui32ReadOffset + 1) & ui32WrapMask;
	}

	psChptCCBCtl->ui32ReadOffset = ui32ReadOffset;

exit_signal:
	if (bSignal)
	{
		SyncCheckpointSignalWaiters();
	}
}
#endif /* defined(PVRSRV_SYNC_CHECKPOINT_CCB) */

/*
 * RGXCheckFirmwareCCB
 */
void RGXCheckFirmwareCCB(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_FWCCB_CMD *psFwCCBCmd;

	RGXFWIF_CCB_CTL *psFWCCBCtl = psDevInfo->psFirmwareCCBCtl;
	IMG_UINT8 *psFWCCB = psDevInfo->psFirmwareCCB;

	while (psFWCCBCtl->ui32ReadOffset != psFWCCBCtl->ui32WriteOffset)
	{
		/* Point to the next command */
		psFwCCBCmd = ((RGXFWIF_FWCCB_CMD *)psFWCCB) + psFWCCBCtl->ui32ReadOffset;

		HTBLOGK(HTB_SF_MAIN_FWCCB_CMD, psFwCCBCmd->eCmdType);
		switch (psFwCCBCmd->eCmdType)
		{
		case RGXFWIF_FWCCB_CMD_ZSBUFFER_BACKING:
		{
			if (psDevInfo->bPDPEnabled)
			{
				PDUMP_PANIC(ZSBUFFER_BACKING, "Request to add backing to ZSBuffer");
			}
			RGXProcessRequestZSBufferBacking(psDevInfo,
					psFwCCBCmd->uCmdData.sCmdZSBufferBacking.ui32ZSBufferID);
			break;
		}

		case RGXFWIF_FWCCB_CMD_ZSBUFFER_UNBACKING:
		{
			if (psDevInfo->bPDPEnabled)
			{
				PDUMP_PANIC(ZSBUFFER_UNBACKING, "Request to remove backing from ZSBuffer");
			}
			RGXProcessRequestZSBufferUnbacking(psDevInfo,
					psFwCCBCmd->uCmdData.sCmdZSBufferBacking.ui32ZSBufferID);
			break;
		}

		case RGXFWIF_FWCCB_CMD_FREELIST_GROW:
		{
			if (psDevInfo->bPDPEnabled)
			{
				PDUMP_PANIC(FREELIST_GROW, "Request to grow the free list");
			}
			RGXProcessRequestGrow(psDevInfo,
					psFwCCBCmd->uCmdData.sCmdFreeListGS.ui32FreelistID);
			break;
		}

		case RGXFWIF_FWCCB_CMD_FREELISTS_RECONSTRUCTION:
		{
			if (psDevInfo->bPDPEnabled)
			{
				PDUMP_PANIC(FREELISTS_RECONSTRUCTION, "Request to reconstruct free lists");
			}

			if (PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST))
			{
				PVR_DPF((PVR_DBG_MESSAGE, "%s: Freelist reconstruction request (%d) for %d freelists",
						__func__,
						psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.ui32HwrCounter+1,
						psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.ui32FreelistsCount));
			}
			else
			{
				PVR_ASSERT(psDevInfo->psRGXFWIfTraceBuf);
				PVR_DPF((PVR_DBG_MESSAGE, "%s: Freelist reconstruction request (%d/%d) for %d freelists",
						__func__,
						psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.ui32HwrCounter+1,
						psDevInfo->psRGXFWIfTraceBuf->ui32HwrCounter+1,
						psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.ui32FreelistsCount));
			}

			RGXProcessRequestFreelistsReconstruction(psDevInfo,
					psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.ui32FreelistsCount,
					psFwCCBCmd->uCmdData.sCmdFreeListsReconstruction.aui32FreelistIDs);
			break;
		}

		case RGXFWIF_FWCCB_CMD_CONTEXT_RESET_NOTIFICATION:
		{
			DLLIST_NODE *psNode, *psNext;
			RGXFWIF_FWCCB_CMD_CONTEXT_RESET_DATA *psCmdContextResetNotification =
					&psFwCCBCmd->uCmdData.sCmdContextResetNotification;
			IMG_UINT32 ui32ServerCommonContextID =
					psCmdContextResetNotification->ui32ServerCommonContextID;
			RGX_SERVER_COMMON_CONTEXT *psServerCommonContext = NULL;

			OSWRLockAcquireRead(psDevInfo->hCommonCtxtListLock);
			dllist_foreach_node(&psDevInfo->sCommonCtxtListHead, psNode, psNext)
			{
				RGX_SERVER_COMMON_CONTEXT *psThisContext =
						IMG_CONTAINER_OF(psNode, RGX_SERVER_COMMON_CONTEXT, sListNode);

				if (psThisContext->ui32ContextID == ui32ServerCommonContextID)
				{
					psServerCommonContext = psThisContext;
					break;
				}
			}

			PVR_DPF((PVR_DBG_MESSAGE, "%s: Context 0x%p reset (ID=0x%08x, Reason=%d, JobRef=0x%08x)",
					__func__,
					psServerCommonContext,
					psCmdContextResetNotification->ui32ServerCommonContextID,
					(IMG_UINT32)(psCmdContextResetNotification->eResetReason),
					psCmdContextResetNotification->ui32ResetJobRef));

			if (psServerCommonContext != NULL)
			{
				psServerCommonContext->eLastResetReason    = psCmdContextResetNotification->eResetReason;
				psServerCommonContext->ui32LastResetJobRef = psCmdContextResetNotification->ui32ResetJobRef;
			}
			OSWRLockReleaseRead(psDevInfo->hCommonCtxtListLock);

			if (psCmdContextResetNotification->bPageFault)
			{
				DevmemIntPFNotify(psDevInfo->psDeviceNode,
						psCmdContextResetNotification->ui64PCAddress,
						psCmdContextResetNotification->sFaultAddress);
			}
			break;
		}

		case RGXFWIF_FWCCB_CMD_DEBUG_DUMP:
		{
			RGXDumpDebugInfo(NULL,NULL,psDevInfo);
			/* Notify the OS of an issue that triggered a debug dump */
			OSWarnOn(IMG_TRUE);
			break;
		}

		case RGXFWIF_FWCCB_CMD_UPDATE_STATS:
		{
#if defined(PVRSRV_ENABLE_PROCESS_STATS)
			IMG_PID pidTmp = psFwCCBCmd->uCmdData.sCmdUpdateStatsData.pidOwner;
			IMG_INT32 i32AdjustmentValue = psFwCCBCmd->uCmdData.sCmdUpdateStatsData.i32AdjustmentValue;

			switch (psFwCCBCmd->uCmdData.sCmdUpdateStatsData.eElementToUpdate)
			{
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_PARTIAL_RENDERS:
			{
				PVRSRVStatsUpdateRenderContextStats(i32AdjustmentValue,0,0,0,0,0,pidTmp);
				break;
			}
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_OUT_OF_MEMORY:
			{
				PVRSRVStatsUpdateRenderContextStats(0,i32AdjustmentValue,0,0,0,0,pidTmp);
				break;
			}
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_TA_STORES:
			{
				PVRSRVStatsUpdateRenderContextStats(0,0,i32AdjustmentValue,0,0,0,pidTmp);
				break;
			}
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_3D_STORES:
			{
				PVRSRVStatsUpdateRenderContextStats(0,0,0,i32AdjustmentValue,0,0,pidTmp);
				break;
			}
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_SH_STORES:
			{
				PVRSRVStatsUpdateRenderContextStats(0,0,0,0,i32AdjustmentValue,0,pidTmp);
				break;
			}
			case RGXFWIF_FWCCB_CMD_UPDATE_NUM_CDM_STORES:
			{
				PVRSRVStatsUpdateRenderContextStats(0,0,0,0,0,i32AdjustmentValue,pidTmp);
				break;
			}
			}
#endif
			break;
		}
		case RGXFWIF_FWCCB_CMD_CORE_CLK_RATE_CHANGE:
		{
#if defined(SUPPORT_PDVFS)
			PDVFS_PROCESS_CORE_CLK_RATE_CHANGE(psDevInfo,
					psFwCCBCmd->uCmdData.sCmdCoreClkRateChange.ui32CoreClkRate);
#endif
			break;
		}

		case RGXFWIF_FWCCB_CMD_REQUEST_GPU_RESTART:
		{
			if (psDevInfo->psRGXFWIfTraceBuf != NULL  &&
					psDevInfo->psRGXFWIfTraceBuf->ePowState != RGXFWIF_POW_OFF)
			{
				PVRSRV_ERROR  eError;

				/* Power down... */
				eError = PVRSRVSetDeviceSystemPowerState(psDevInfo->psDeviceNode,
						PVRSRV_SYS_POWER_STATE_OFF);
				if (eError == PVRSRV_OK)
				{
					/* Clear the FW faulted flags... */
					psDevInfo->psRGXFWIfTraceBuf->ui32HWRStateFlags &= ~(RGXFWIF_HWR_FW_FAULT|RGXFWIF_HWR_RESTART_REQUESTED);

					/* Power back up again... */
					eError = PVRSRVSetDeviceSystemPowerState(psDevInfo->psDeviceNode,
							PVRSRV_SYS_POWER_STATE_ON);

					/* Send a dummy KCCB command to ensure the FW wakes up and checks the queues... */
					if (eError == PVRSRV_OK)
					{
						LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
						{
							eError = RGXFWHealthCheckCmd(psDevInfo);
							if (eError != PVRSRV_ERROR_RETRY)
							{
								break;
							}
							OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
						} END_LOOP_UNTIL_TIMEOUT();
					}
				}

				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR, "%s: Failed firmware restart (%s)",
							__func__, PVRSRVGetErrorString(eError)));
				}
			}
			break;
		}

		default:
		{
			/* unknown command */
			PVR_DPF((PVR_DBG_WARNING, "%s: Unknown Command (eCmdType=0x%08x)",
					__func__, psFwCCBCmd->eCmdType));
			/* Assert on magic value corruption */
			PVR_ASSERT((((IMG_UINT32)psFwCCBCmd->eCmdType & RGX_CMD_MAGIC_DWORD_MASK) >> RGX_CMD_MAGIC_DWORD_SHIFT) == RGX_CMD_MAGIC_DWORD);
		}
		}

		/* Update read offset */
		psFWCCBCtl->ui32ReadOffset = (psFWCCBCtl->ui32ReadOffset + 1) & psFWCCBCtl->ui32WrapMask;
	}
}

/*
 * PVRSRVRGXFrameworkCopyCommand
 */
PVRSRV_ERROR PVRSRVRGXFrameworkCopyCommand(DEVMEM_MEMDESC	*psFWFrameworkMemDesc,
		IMG_PBYTE		pbyGPUFRegisterList,
		IMG_UINT32		ui32FrameworkRegisterSize)
{
	PVRSRV_ERROR	eError;
	RGXFWIF_RF_REGISTERS	*psRFReg;

	eError = DevmemAcquireCpuVirtAddr(psFWFrameworkMemDesc,
			(void **)&psRFReg);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXFrameworkCopyCommand: Failed to map firmware render context state (%u)",
				eError));
		return eError;
	}

	OSDeviceMemCopy(psRFReg, pbyGPUFRegisterList, ui32FrameworkRegisterSize);

	/* Release the CPU mapping */
	DevmemReleaseCpuVirtAddr(psFWFrameworkMemDesc);

	/*
	 * Dump the FW framework buffer
	 */
#if defined(PDUMP)
	PDUMPCOMMENT("Dump FWFramework buffer");
	DevmemPDumpLoadMem(psFWFrameworkMemDesc, 0, ui32FrameworkRegisterSize, PDUMP_FLAGS_CONTINUOUS);
#endif

	return PVRSRV_OK;
}

/*
 * PVRSRVRGXFrameworkCreateKM
 */
PVRSRV_ERROR PVRSRVRGXFrameworkCreateKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
		DEVMEM_MEMDESC		**ppsFWFrameworkMemDesc,
		IMG_UINT32			ui32FrameworkCommandSize)
{
	PVRSRV_ERROR			eError;
	PVRSRV_RGXDEV_INFO		*psDevInfo = psDeviceNode->pvDevice;

	/*
		Allocate device memory for the firmware GPU framework state.
		Sufficient info to kick one or more DMs should be contained in this buffer
	 */
	PDUMPCOMMENT("Allocate Rogue firmware framework state");

	eError = DevmemFwAllocate(psDevInfo,
			ui32FrameworkCommandSize,
			RGX_FWCOMCTX_ALLOCFLAGS,
			"FwGPUFrameworkState",
			ppsFWFrameworkMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXFrameworkContextKM: Failed to allocate firmware framework state (%u)",
				eError));
		return eError;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXWaitForFWOp(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM eDM,
		PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
		IMG_UINT32 ui32PDumpFlags)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	PVRSRV_DEVICE_NODE *psDeviceNode = psDevInfo->psDeviceNode;
	RGXFWIF_KCCB_CMD	sCmdSyncPrim;

	/* Setup sync primitive */
	eError = SyncPrimSet(psSyncPrim, 0);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to set SyncPrim (%u)",
				__func__, eError));
		goto _Error_Exit;
	}

	/* prepare a sync command */
	eError = SyncPrimGetFirmwareAddr(psSyncPrim,
			&sCmdSyncPrim.uCmdData.sSyncData.sSyncObjDevVAddr.ui32Addr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to get SyncPrim FW address(%u)",
				__func__, eError));
		goto _Error_Exit;
	}
	sCmdSyncPrim.eCmdType = RGXFWIF_KCCB_CMD_SYNC;
	sCmdSyncPrim.uCmdData.sSyncData.uiUpdateVal = 1;

	PDUMPCOMMENT("RGXWaitForFWOp: Submit Kernel SyncPrim [0x%08x] to DM %d",
			sCmdSyncPrim.uCmdData.sSyncData.sSyncObjDevVAddr.ui32Addr, eDM);

	eError = RGXScheduleCommand(psDevInfo,
			eDM,
			&sCmdSyncPrim,
			0,
			ui32PDumpFlags);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to schedule Kernel SyncPrim with error (%u)",
				__func__,
				eError));
		goto _Error_Exit;
	}

	/* Wait for sync primitive to be updated */
#if defined(PDUMP)
	PDUMPCOMMENT("RGXScheduleCommandAndWait: Poll for Kernel SyncPrim [0x%08x] on DM %d",
			sCmdSyncPrim.uCmdData.sSyncData.sSyncObjDevVAddr.ui32Addr, eDM);

	SyncPrimPDumpPol(psSyncPrim,
			1,
			0xffffffff,
			PDUMP_POLL_OPERATOR_EQUAL,
			ui32PDumpFlags);
#endif

	{
		RGXFWIF_CCB_CTL *psKCCBCtl = psDevInfo->psKernelCCBCtl;
		IMG_UINT32 ui32CurrentQueueLength =
				(psKCCBCtl->ui32WrapMask+1 +
						psKCCBCtl->ui32WriteOffset -
						psKCCBCtl->ui32ReadOffset) & psKCCBCtl->ui32WrapMask;
		IMG_UINT32 ui32MaxRetries;

		ui32CurrentQueueLength += psDevInfo->ui32KCCBDeferredCommandsCount;
		for (ui32MaxRetries = (ui32CurrentQueueLength + 1) * 3;
				ui32MaxRetries > 0;
				ui32MaxRetries--)
		{
			eError = PVRSRVWaitForValueKMAndHoldBridgeLockKM(psSyncPrim->pui32LinAddr, 1, 0xffffffff);

			if (eError != PVRSRV_ERROR_TIMEOUT)
			{
				break;
			}

			/*
			 * In case the KCCB was full we must ensure we flush any deferred
			 * commands because they may be preventing the RGXFWIF_KCCB_CMD_SYNC
			 * from being sent. No need to check the error, if the KCCB is
			 * still full then we wait anyway.
			 */

			if (PVRSRVPowerLock(psDeviceNode) != PVRSRV_OK)
			{
				/* RGXSendCommandsFromDeferredList should be called while holding PowerLock */
				PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire PowerLock (device: %p)",
					__func__, psDeviceNode));
				continue;
			}

			RGXSendCommandsFromDeferredList(psDevInfo, IMG_FALSE);

			PVRSRVPowerUnlock(psDeviceNode);
		}

		if (eError == PVRSRV_ERROR_TIMEOUT)
		{
			PVR_DPF((PVR_DBG_ERROR,"%s: PVRSRVWaitForValueKMAndHoldBridgeLock timed out. Dump debug information.",
					__func__));
			PVRSRVDebugRequest(psDeviceNode, DEBUG_REQUEST_VERBOSITY_MAX, NULL, NULL);
			PVR_ASSERT(eError != PVRSRV_ERROR_TIMEOUT);
			goto _Error_Exit;
		}
	}

	_Error_Exit:
	return eError;
}

PVRSRV_ERROR IMG_CALLCONV RGXPollForGPCommandCompletion(PVRSRV_DEVICE_NODE  *psDevNode,
												volatile IMG_UINT32	__iomem *pui32LinMemAddr,
												IMG_UINT32			ui32Value,
												IMG_UINT32			ui32Mask)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_CCB_CTL *psKCCBCtl;
	IMG_UINT32 ui32CurrentQueueLength, ui32MaxRetries;
	PVRSRV_RGXDEV_INFO	*psDevInfo = psDevNode->pvDevice;

	psKCCBCtl = psDevInfo->psKernelCCBCtl;
	ui32CurrentQueueLength = (psKCCBCtl->ui32WrapMask+1 +
					psKCCBCtl->ui32WriteOffset -
					psKCCBCtl->ui32ReadOffset) & psKCCBCtl->ui32WrapMask;
	ui32CurrentQueueLength += psDevInfo->ui32KCCBDeferredCommandsCount;

	for (ui32MaxRetries = ui32CurrentQueueLength + 1;
				ui32MaxRetries > 0;
				ui32MaxRetries--)
	{

		eError = PVRSRVPollForValueKM(pui32LinMemAddr, ui32Value, ui32Mask);
		if (eError != PVRSRV_ERROR_TIMEOUT)
		{
			break;
		}

		RGXSendCommandsFromDeferredList(psDevInfo, IMG_FALSE);
	}

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed! Error(%s) CPU linear address(%p) Expected value(%u)",
		                        __func__, PVRSRVGetErrorString(eError),
								pui32LinMemAddr, ui32Value));
		PVRSRVDebugRequest(psDevNode, DEBUG_REQUEST_VERBOSITY_MAX, NULL, NULL);
	}

	return eError;
}

PVRSRV_ERROR RGXStateFlagCtrl(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32Config,
		IMG_UINT32 *pui32ConfigState,
		IMG_BOOL bSetNotClear)
{
	PVRSRV_ERROR eError;
	PVRSRV_DEV_POWER_STATE ePowerState;
	RGXFWIF_KCCB_CMD sStateFlagCmd;
	PVRSRV_DEVICE_NODE *psDeviceNode;
	RGXFWIF_OS_CONFIG *psOSConfig;

	if (!psDevInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	psDeviceNode = psDevInfo->psDeviceNode;
	psOSConfig = psDevInfo->psFWIfOSConfig;

	if (NULL == psOSConfig)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: OS Config is not mapped into CPU space", __func__));
		return PVRSRV_ERROR_INVALID_CPU_ADDR;
	}

	/* apply change and ensure the new data is written to memory
	 * before requesting the FW to read it
	 */
	ui32Config = ui32Config & RGXFWIF_INICFG_ALL;
	if (bSetNotClear)
	{
		psOSConfig->ui32ConfigFlags |= ui32Config;
	}
	else
	{
		psOSConfig->ui32ConfigFlags &= ~ui32Config;
	}

	/* return current/new value to caller */
	if (pui32ConfigState)
	{
		*pui32ConfigState = psOSConfig->ui32ConfigFlags;
	}

	OSMemoryBarrier();

	eError = PVRSRVPowerLock(psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire power lock (%u)", __func__, eError));
		goto error_lock;
	}

	/* notify FW to update setting */
	eError = PVRSRVGetDevicePowerState(psDeviceNode, &ePowerState);

	if ((eError == PVRSRV_OK) && (ePowerState != PVRSRV_DEV_POWER_STATE_OFF))
	{
		/* Ask the FW to update its cached version of the value */
		sStateFlagCmd.eCmdType = RGXFWIF_KCCB_CMD_STATEFLAGS_CTRL;

		eError = RGXSendCommand(psDevInfo,
				RGXFWIF_DM_GP,
				&sStateFlagCmd,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: RGXSendCommand failed. Error:%u", __func__, eError));
			goto error_cmd;
		}
		else
		{
			/* Give up the power lock as its acquired in RGXWaitForFWOp */
			PVRSRVPowerUnlock(psDeviceNode);

			/* Wait for the value to be updated as the FW validates
			 * the parameters and modifies the ui32ConfigFlags
			 * accordingly
			 * (for completeness as registered callbacks should also
			 *  not permit invalid transitions)
			 */
			eError = RGXWaitForFWOp(psDevInfo, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, PDUMP_FLAGS_CONTINUOUS);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"%s: Waiting for value aborted with error (%u)", __func__, eError));
			}
			goto error_lock;
		}
	}

	error_cmd:
	PVRSRVPowerUnlock(psDeviceNode);
	error_lock:
	return eError;
}

static
PVRSRV_ERROR RGXScheduleCleanupCommand(PVRSRV_RGXDEV_INFO	*psDevInfo,
		RGXFWIF_DM			eDM,
		RGXFWIF_KCCB_CMD		*psKCCBCmd,
		RGXFWIF_CLEANUP_TYPE	eCleanupType,
		PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
		IMG_UINT32				ui32PDumpFlags)
{
	PVRSRV_ERROR eError;

	psKCCBCmd->eCmdType = RGXFWIF_KCCB_CMD_CLEANUP;

	psKCCBCmd->uCmdData.sCleanupData.eCleanupType = eCleanupType;
	eError = SyncPrimGetFirmwareAddr(psSyncPrim, &psKCCBCmd->uCmdData.sCleanupData.sSyncObjDevVAddr.ui32Addr);
	if (eError != PVRSRV_OK)
	{
		goto fail_command;
	}

	eError = SyncPrimSet(psSyncPrim, 0);
	if (eError != PVRSRV_OK)
	{
		goto fail_command;
	}

	/*
		Send the cleanup request to the firmware. If the resource is still busy
		the firmware will tell us and we'll drop out with a retry.
	 */
	eError = RGXScheduleCommand(psDevInfo,
			eDM,
			psKCCBCmd,
			0,
			ui32PDumpFlags);
	if (eError != PVRSRV_OK)
	{
		goto fail_command;
	}

	/* Wait for sync primitive to be updated */
#if defined(PDUMP)
	PDUMPCOMMENT("Wait for the firmware to reply to the cleanup command");
	SyncPrimPDumpPol(psSyncPrim,
			RGXFWIF_CLEANUP_RUN,
			RGXFWIF_CLEANUP_RUN,
			PDUMP_POLL_OPERATOR_EQUAL,
			ui32PDumpFlags);

	/*
	 * The cleanup request to the firmware will tell us if a given resource is busy or not.
	 * If the RGXFWIF_CLEANUP_BUSY flag is set, this means that the resource is still in use.
	 * In this case we return a PVRSRV_ERROR_RETRY error to the client drivers and they will
	 * re-issue the cleanup request until it succeed.
	 *
	 * Since this retry mechanism doesn't work for pdumps, client drivers should ensure
	 * that cleanup requests are only submitted if the resource is unused.
	 * If this is not the case, the following poll will block infinitely, making sure
	 * the issue doesn't go unnoticed.
	 */
	PDUMPCOMMENT("Cleanup: If this poll fails, the following resource is still in use (DM=%u, type=%u, address=0x%08x), which is incorrect in pdumps",
			eDM,
			psKCCBCmd->uCmdData.sCleanupData.eCleanupType,
			psKCCBCmd->uCmdData.sCleanupData.uCleanupData.psContext.ui32Addr);
	SyncPrimPDumpPol(psSyncPrim,
			0,
			RGXFWIF_CLEANUP_BUSY,
			PDUMP_POLL_OPERATOR_EQUAL,
			ui32PDumpFlags);
#endif

	{
		RGXFWIF_CCB_CTL  *psKCCBCtl = psDevInfo->psKernelCCBCtl;
		IMG_UINT32       ui32CurrentQueueLength = (psKCCBCtl->ui32WrapMask+1 +
				psKCCBCtl->ui32WriteOffset -
				psKCCBCtl->ui32ReadOffset) & psKCCBCtl->ui32WrapMask;
		IMG_UINT32       ui32MaxRetries;

		ui32CurrentQueueLength += psDevInfo->ui32KCCBDeferredCommandsCount;
		for (ui32MaxRetries = ui32CurrentQueueLength + 1;
				ui32MaxRetries > 0;
				ui32MaxRetries--)
		{
			eError = PVRSRVWaitForValueKMAndHoldBridgeLockKM(psSyncPrim->pui32LinAddr, RGXFWIF_CLEANUP_RUN, RGXFWIF_CLEANUP_RUN);

			if (eError != PVRSRV_ERROR_TIMEOUT)
			{
				break;
			}

			if (PVRSRVPowerLock(psDevInfo->psDeviceNode) != PVRSRV_OK)
			{
				/* RGXSendCommandsFromDeferredList should be called while holding PowerLock */
				PVR_DPF((PVR_DBG_ERROR,"%s: Failed to acquire PowerLock (device: %p)",
					__func__, psDevInfo->psDeviceNode));
				continue;
			}

			RGXSendCommandsFromDeferredList(psDevInfo, IMG_FALSE);

			PVRSRVPowerUnlock(psDevInfo->psDeviceNode);
		}

		/*
			If the firmware hasn't got back to us in a timely manner
			then bail and let the caller retry the command.
		 */
		if (eError == PVRSRV_ERROR_TIMEOUT)
		{
			PVR_DPF((PVR_DBG_WARNING,"RGXScheduleCleanupCommand: PVRSRVWaitForValueKMAndHoldBridgeLock timed out. Dump debug information."));

			eError = PVRSRV_ERROR_RETRY;
#if defined(DEBUG)
			PVRSRVDebugRequest(psDevInfo->psDeviceNode,
					DEBUG_REQUEST_VERBOSITY_MAX, NULL, NULL);
#endif
			goto fail_poll;
		}
		else if (eError != PVRSRV_OK)
		{
			goto fail_poll;
		}
	}

	/*
		If the command has was run but a resource was busy, then the request
		will need to be retried.
	 */
	if (OSReadDeviceMem32(psSyncPrim->pui32LinAddr) & RGXFWIF_CLEANUP_BUSY)
	{
		eError = PVRSRV_ERROR_RETRY;
		goto fail_requestbusy;
	}

	return PVRSRV_OK;

	fail_requestbusy:
	fail_poll:
	fail_command:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

/*
	RGXRequestCommonContextCleanUp
 */
PVRSRV_ERROR RGXFWRequestCommonContextCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
		RGX_SERVER_COMMON_CONTEXT *psServerCommonContext,
		PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
		RGXFWIF_DM eDM,
		IMG_UINT32 ui32PDumpFlags)
{
	RGXFWIF_KCCB_CMD			sRCCleanUpCmd = {0};
	PVRSRV_ERROR				eError;
	PRGXFWIF_FWCOMMONCONTEXT	psFWCommonContextFWAddr;
	PVRSRV_RGXDEV_INFO			*psDevInfo = (PVRSRV_RGXDEV_INFO*)psDeviceNode->pvDevice;

	/* Force retry if this context's CCB is currently being dumped
	 * as part of the stalled CCB debug */
	if (psDevInfo->pvEarliestStalledClientCCB == (void*)psServerCommonContext->psClientCCB)
	{
		return PVRSRV_ERROR_RETRY;
	}

	psFWCommonContextFWAddr = FWCommonContextGetFWAddress(psServerCommonContext);
#if defined(PDUMP)
	PDUMPCOMMENT("Common ctx cleanup Request DM%d [context = 0x%08x]",
			eDM, psFWCommonContextFWAddr.ui32Addr);
	PDUMPCOMMENT("Wait for CCB to be empty before common ctx cleanup");

	RGXCCBPDumpDrainCCB(FWCommonContextGetClientCCB(psServerCommonContext), ui32PDumpFlags);
#endif

	/* Setup our command data, the cleanup call will fill in the rest */
	sRCCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psContext = psFWCommonContextFWAddr;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
			eDM,
			&sRCCleanUpCmd,
			RGXFWIF_CLEANUP_FWCOMMONCONTEXT,
			psSyncPrim,
			ui32PDumpFlags);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXRequestCommonContextCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

/*
 * RGXFWRequestHWRTDataCleanUp
 */

PVRSRV_ERROR RGXFWRequestHWRTDataCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
		PRGXFWIF_HWRTDATA psHWRTData,
		PVRSRV_CLIENT_SYNC_PRIM *psSync,
		RGXFWIF_DM eDM)
{
	RGXFWIF_KCCB_CMD			sHWRTDataCleanUpCmd = {0};
	PVRSRV_ERROR				eError;

	PDUMPCOMMENT("HW RTData cleanup Request DM%d [HWRTData = 0x%08x]", eDM, psHWRTData.ui32Addr);

	sHWRTDataCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psHWRTData = psHWRTData;

	eError = RGXScheduleCleanupCommand(psDeviceNode->pvDevice,
			eDM,
			&sHWRTDataCleanUpCmd,
			RGXFWIF_CLEANUP_HWRTDATA,
			psSync,
			PDUMP_FLAGS_NONE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXFWRequestHWRTDataCleanUp: Failed to schedule a HWRTData cleanup with error (%u)", eError));
	}

	return eError;
}

/*
	RGXFWRequestFreeListCleanUp
 */
PVRSRV_ERROR RGXFWRequestFreeListCleanUp(PVRSRV_RGXDEV_INFO *psDevInfo,
		PRGXFWIF_FREELIST psFWFreeList,
		PVRSRV_CLIENT_SYNC_PRIM *psSync)
{
	RGXFWIF_KCCB_CMD			sFLCleanUpCmd = {0};
	PVRSRV_ERROR				eError;

	PDUMPCOMMENT("Free list cleanup Request [FreeList = 0x%08x]", psFWFreeList.ui32Addr);

	/* Setup our command data, the cleanup call will fill in the rest */
	sFLCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psFreelist = psFWFreeList;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDevInfo,
			RGXFWIF_DM_GP,
			&sFLCleanUpCmd,
			RGXFWIF_CLEANUP_FREELIST,
			psSync,
			PDUMP_FLAGS_NONE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXFWRequestFreeListCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

/*
	RGXFWRequestZSBufferCleanUp
 */
PVRSRV_ERROR RGXFWRequestZSBufferCleanUp(PVRSRV_RGXDEV_INFO *psDevInfo,
		PRGXFWIF_ZSBUFFER psFWZSBuffer,
		PVRSRV_CLIENT_SYNC_PRIM *psSync)
{
	RGXFWIF_KCCB_CMD			sZSBufferCleanUpCmd = {0};
	PVRSRV_ERROR				eError;

	PDUMPCOMMENT("ZS Buffer cleanup Request [ZS Buffer = 0x%08x]", psFWZSBuffer.ui32Addr);

	/* Setup our command data, the cleanup call will fill in the rest */
	sZSBufferCleanUpCmd.uCmdData.sCleanupData.uCleanupData.psZSBuffer = psFWZSBuffer;

	/* Request cleanup of the firmware resource */
	eError = RGXScheduleCleanupCommand(psDevInfo,
			RGXFWIF_DM_3D,
			&sZSBufferCleanUpCmd,
			RGXFWIF_CLEANUP_ZSBUFFER,
			psSync,
			PDUMP_FLAGS_NONE);

	if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXFWRequestZSBufferCleanUp: Failed to schedule a memory context cleanup with error (%u)", eError));
	}

	return eError;
}

PVRSRV_ERROR RGXFWSetHCSDeadline(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32HCSDeadlineMs)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CMD	sSetHCSDeadline;

	sSetHCSDeadline.eCmdType                            = RGXFWIF_KCCB_CMD_HCS_SET_DEADLINE;
	sSetHCSDeadline.eDM                                 = RGXFWIF_DM_GP;
	sSetHCSDeadline.uCmdData.sHCSCtrl.ui32HCSDeadlineMS = ui32HCSDeadlineMs;

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		eError = RGXScheduleCommand(psDevInfo,
				RGXFWIF_DM_GP,
				&sSetHCSDeadline,
				0,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	return eError;
}

PVRSRV_ERROR RGXFWHealthCheckCmd(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_KCCB_CMD	sCmpKCCBCmd;

	sCmpKCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_HEALTH_CHECK;

	return	RGXScheduleCommand(psDevInfo,
							   RGXFWIF_DM_GP,
							   &sCmpKCCBCmd,
							   0,
							   PDUMP_FLAGS_CONTINUOUS);
}

PVRSRV_ERROR RGXFWSetOSIsolationThreshold(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32IsolationPriorityThreshold)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CMD	sOSidIsoConfCmd;

	sOSidIsoConfCmd.eCmdType = RGXFWIF_KCCB_CMD_OS_ISOLATION_GROUP_CHANGE;
	sOSidIsoConfCmd.uCmdData.sCmdOSidIsolationData.ui32IsolationPriorityThreshold = ui32IsolationPriorityThreshold;

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		eError = RGXScheduleCommand(psDevInfo,
				RGXFWIF_DM_GP,
				&sOSidIsoConfCmd,
				0,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	return eError;
}

PVRSRV_ERROR RGXFWSetFwOsState(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT32 ui32OSid,
							   RGXFWIF_OS_STATE_CHANGE eOSOnlineState)
{
	PVRSRV_ERROR         eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD     sOSOnlineStateCmd;
	RGXFWIF_TRACEBUF    *psRGXFWIfTraceBuf = psDevInfo->psRGXFWIfTraceBuf;
	PVRSRV_VZ_RET_IF_MODE(DRIVER_MODE_GUEST, PVRSRV_OK);

	sOSOnlineStateCmd.eCmdType = RGXFWIF_KCCB_CMD_OS_ONLINE_STATE_CONFIGURE;
	sOSOnlineStateCmd.uCmdData.sCmdOSOnlineStateData.ui32OSid = ui32OSid;
	sOSOnlineStateCmd.uCmdData.sCmdOSOnlineStateData.eNewOSState = eOSOnlineState;

	if (eOSOnlineState == RGXFWIF_OS_ONLINE)
	{
		LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
		{
			eError = RGXScheduleCommand(psDevInfo,
					RGXFWIF_DM_GP,
					&sOSOnlineStateCmd,
					0,
					PDUMP_FLAGS_CONTINUOUS);
			if (eError != PVRSRV_ERROR_RETRY) break;

			OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
		} END_LOOP_UNTIL_TIMEOUT();
	}
	else if (psRGXFWIfTraceBuf)
	{
		volatile RGXFWIF_PER_OS_STATES *psPerOsState;

		psPerOsState = (volatile RGXFWIF_PER_OS_STATES*) &psRGXFWIfTraceBuf->sPerOsStateMirror[ui32OSid];
		/* Attempt several times until the FW manages to offload the OS */
		LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
		{
			/* Send request */
			eError = RGXScheduleCommand(psDevInfo,
					RGXFWIF_DM_GP,
					&sOSOnlineStateCmd,
					0,
					PDUMP_FLAGS_CONTINUOUS);
			if (unlikely(eError == PVRSRV_ERROR_RETRY)) continue;
			PVR_LOGG_IF_ERROR(eError, "RGXScheduleCommand", return_);

			/* Wait for FW to process the cmd */
			eError = RGXWaitForFWOp(psDevInfo,
					RGXFWIF_DM_GP,
					psDevInfo->psDeviceNode->psSyncPrim,
					PDUMP_FLAGS_CONTINUOUS);
			PVR_LOGG_IF_ERROR(eError, "RGXWaitForFWOp", return_);

			/* read the OS state */
			OSMemoryBarrier();
			/* check if FW finished offloading the OSID and is stopped */
			if (psPerOsState->bfOsState == RGXFW_OS_STATE_STOPPED)
			{
				eError = PVRSRV_OK;
				break;
			}
			else
			{
				eError = PVRSRV_ERROR_TIMEOUT;
			}

			OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
		} END_LOOP_UNTIL_TIMEOUT();
	}
	else
	{
		eError = PVRSRV_ERROR_NOT_INITIALISED;
	}

	return_ :
	return eError;
}

PVRSRV_ERROR RGXFWChangeOSidPriority(PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32OSid,
		IMG_UINT32 ui32Priority)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CMD	sOSidPriorityCmd;

	sOSidPriorityCmd.eCmdType = RGXFWIF_KCCB_CMD_OSID_PRIORITY_CHANGE;
	sOSidPriorityCmd.uCmdData.sCmdOSidPriorityData.ui32OSidNum = ui32OSid;
	sOSidPriorityCmd.uCmdData.sCmdOSidPriorityData.ui32Priority = ui32Priority;

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		eError = RGXScheduleCommand(psDevInfo,
				RGXFWIF_DM_GP,
				&sOSidPriorityCmd,
				0,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	return eError;
}

PVRSRV_ERROR ContextSetPriority(RGX_SERVER_COMMON_CONTEXT *psContext,
		CONNECTION_DATA *psConnection,
		PVRSRV_RGXDEV_INFO *psDevInfo,
		IMG_UINT32 ui32Priority,
		RGXFWIF_DM eDM)
{
	IMG_UINT32				ui32CmdSize;
	IMG_UINT8				*pui8CmdPtr;
	RGXFWIF_KCCB_CMD		sPriorityCmd;
	RGXFWIF_CCB_CMD_HEADER	*psCmdHeader;
	RGXFWIF_CMD_PRIORITY	*psCmd;
	PVRSRV_ERROR			eError;

	/*
		Get space for command
	 */
	ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_CCB_CMD_HEADER) + sizeof(RGXFWIF_CMD_PRIORITY));

	eError = RGXAcquireCCB(FWCommonContextGetClientCCB(psContext),
			ui32CmdSize,
			(void **) &pui8CmdPtr,
			PDUMP_FLAGS_CONTINUOUS);
	if (eError != PVRSRV_OK)
	{
		if (eError != PVRSRV_ERROR_RETRY)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to acquire space for client CCB", __func__));
		}
		goto fail_ccbacquire;
	}

	/*
		Write the command header and command
	 */
	psCmdHeader = (RGXFWIF_CCB_CMD_HEADER *) pui8CmdPtr;
	psCmdHeader->eCmdType = RGXFWIF_CCB_CMD_TYPE_PRIORITY;
	psCmdHeader->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_CMD_PRIORITY));
	pui8CmdPtr += sizeof(*psCmdHeader);

	psCmd = (RGXFWIF_CMD_PRIORITY *) pui8CmdPtr;
	psCmd->ui32Priority = ui32Priority;
	pui8CmdPtr += sizeof(*psCmd);

	/*
		We should reserved space in the kernel CCB here and fill in the command
		directly.
		This is so if there isn't space in the kernel CCB we can return with
		retry back to services client before we take any operations
	 */

	/*
		Submit the command
	 */
	RGXReleaseCCB(FWCommonContextGetClientCCB(psContext),
			ui32CmdSize,
			PDUMP_FLAGS_CONTINUOUS);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to release space in client CCB", __func__));
		return eError;
	}

	/* Construct the priority command. */
	sPriorityCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	sPriorityCmd.uCmdData.sCmdKickData.psContext = FWCommonContextGetFWAddress(psContext);
	sPriorityCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = RGXGetHostWriteOffsetCCB(FWCommonContextGetClientCCB(psContext));
	sPriorityCmd.uCmdData.sCmdKickData.ui32NumCleanupCtl = 0;
	sPriorityCmd.uCmdData.sCmdKickData.ui32WorkEstCmdHeaderOffset = 0;

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		eError = RGXScheduleCommand(psDevInfo,
				eDM,
				&sPriorityCmd,
				0,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"%s: Failed to submit set priority command with error (%u)",
				__func__,
				eError));
	}

	return PVRSRV_OK;

	fail_ccbacquire:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*
	RGXReadMETAAddr
 */
PVRSRV_ERROR RGXReadMETAAddr(PVRSRV_RGXDEV_INFO	*psDevInfo, IMG_UINT32 ui32METAAddr, IMG_UINT32 *pui32Value)
{
	IMG_UINT8 __iomem  *pui8RegBase = psDevInfo->pvRegsBaseKM;
	IMG_UINT32 ui32Value;

	/* Wait for Slave Port to be Ready */
	if (PVRSRVPollForValueKM(
			(IMG_UINT32 __iomem *) (pui8RegBase + RGX_CR_META_SP_MSLVCTRL1),
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN) != PVRSRV_OK)
	{
		return PVRSRV_ERROR_TIMEOUT;
	}

	/* Issue the Read */
	OSWriteHWReg32(
			psDevInfo->pvRegsBaseKM,
			RGX_CR_META_SP_MSLVCTRL0,
			ui32METAAddr | RGX_CR_META_SP_MSLVCTRL0_RD_EN);

	/* Wait for Slave Port to be Ready: read complete */
	if (PVRSRVPollForValueKM(
			(IMG_UINT32 __iomem *) (pui8RegBase + RGX_CR_META_SP_MSLVCTRL1),
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN) != PVRSRV_OK)
	{
		return PVRSRV_ERROR_TIMEOUT;
	}

	/* Read the value */
	ui32Value = OSReadHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVDATAX);

	*pui32Value = ui32Value;

	return PVRSRV_OK;
}

/*
	RGXWriteMETAAddr
 */
PVRSRV_ERROR RGXWriteMETAAddr(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT32 ui32METAAddr, IMG_UINT32 ui32Value)
{
	IMG_UINT8 __iomem *pui8RegBase = psDevInfo->pvRegsBaseKM;

	/* Wait for Slave Port to be Ready */
	if (PVRSRVPollForValueKM((IMG_UINT32 __iomem *)
			(pui8RegBase + RGX_CR_META_SP_MSLVCTRL1),
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN,
			RGX_CR_META_SP_MSLVCTRL1_READY_EN|RGX_CR_META_SP_MSLVCTRL1_GBLPORT_IDLE_EN) != PVRSRV_OK)
	{
		return PVRSRV_ERROR_TIMEOUT;
	}

	/* Issue the Write */
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVCTRL0, ui32METAAddr);
	OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_META_SP_MSLVDATAT, ui32Value);

	return PVRSRV_OK;
}

void RGXCheckForStalledClientContexts(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_BOOL bIgnorePrevious)
{
	/* Attempt to detect and deal with any stalled client contexts.
	 * bIgnorePrevious may be set by the caller if they know a context to be
	 * stalled, as otherwise this function will only identify stalled
	 * contexts which have not been previously reported.
	 */

	IMG_UINT32 ui32StalledClientMask = 0;

	if (!(OSTryLockAcquire(psDevInfo->hCCBStallCheckLock)))
	{
		PVR_LOG(("RGXCheckForStalledClientContexts: Failed to acquire hCCBStallCheckLock, returning..."));
		return;
	}

	ui32StalledClientMask |= CheckForStalledClientTransferCtxt(psDevInfo);

	ui32StalledClientMask |= CheckForStalledClientRenderCtxt(psDevInfo);

	ui32StalledClientMask |= CheckForStalledClientKickSyncCtxt(psDevInfo);

	if (psDevInfo->sDevFeatureCfg.ui64Features & RGX_FEATURE_COMPUTE_BIT_MASK)
	{
		ui32StalledClientMask |= CheckForStalledClientComputeCtxt(psDevInfo);
	}

	/* If at least one DM stalled bit is different than before */
	if (bIgnorePrevious || (psDevInfo->ui32StalledClientMask != ui32StalledClientMask))//(psDevInfo->ui32StalledClientMask ^ ui32StalledClientMask))
	{
		if (ui32StalledClientMask > 0)
		{
			static __maybe_unused const char *pszStalledAction =
#if defined(PVRSRV_STALLED_CCB_ACTION)
					"force";
#else
					"warn";
#endif
			/* Print all the stalled DMs */
			PVR_LOG(("Possible stalled client RGX contexts detected: %s%s%s%s%s%s%s%s%s",
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_GP),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_TDM_2D),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_TA),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_3D),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_CDM),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_RTU),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_SHG),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_TQ2D),
					 RGX_STRINGIFY_KICK_TYPE_DM_IF_SET(ui32StalledClientMask, RGX_KICK_TYPE_DM_TQ3D)));

			PVR_LOG(("Trying to identify stalled context...(%s) [%d]",
			         pszStalledAction, bIgnorePrevious));

			DumpStalledContextInfo(psDevInfo);
		}
		else
		{
			if (psDevInfo->ui32StalledClientMask> 0)
			{
				/* Indicate there are no stalled DMs */
				PVR_LOG(("No further stalled client contexts exist"));
			}
		}
		psDevInfo->ui32StalledClientMask = ui32StalledClientMask;
	}
	OSLockRelease(psDevInfo->hCCBStallCheckLock);
}

/*
	RGXUpdateHealthStatus
 */
PVRSRV_ERROR RGXUpdateHealthStatus(PVRSRV_DEVICE_NODE* psDevNode,
		IMG_BOOL bCheckAfterTimePassed)
{
	PVRSRV_DATA*                 psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_DEVICE_HEALTH_STATUS  eNewStatus   = PVRSRV_DEVICE_HEALTH_STATUS_OK;
	PVRSRV_DEVICE_HEALTH_REASON  eNewReason   = PVRSRV_DEVICE_HEALTH_REASON_NONE;
	PVRSRV_RGXDEV_INFO*  psDevInfo;
	RGXFWIF_TRACEBUF*  psRGXFWIfTraceBufCtl;
	RGXFWIF_CCB_CTL *psKCCBCtl;
	IMG_UINT32  ui32ThreadCount;
	IMG_BOOL  bKCCBCmdsWaiting;
	PVRSRV_VZ_RET_IF_MODE(DRIVER_MODE_GUEST, PVRSRV_OK);

	PVR_ASSERT(psDevNode != NULL);
	psDevInfo = psDevNode->pvDevice;
	psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	/* If the firmware is not initialised, there is not much point continuing! */
	if (!psDevInfo->bFirmwareInitialised || psDevInfo->pvRegsBaseKM == NULL ||
			psDevInfo->psDeviceNode == NULL)
	{
		return PVRSRV_OK;
	}

	/* If this is a quick update, then include the last current value... */
	if (!bCheckAfterTimePassed)
	{
		eNewStatus = OSAtomicRead(&psDevNode->eHealthStatus);
		eNewReason = OSAtomicRead(&psDevNode->eHealthReason);
	}

	/* If Rogue is not powered on, just skip ahead and check for stalled client CCBs */
	if (PVRSRVIsDevicePowered(psDevNode))
	{
		if (psRGXFWIfTraceBufCtl != NULL)
		{
			/*
			   Firmware thread checks...
			 */
			for (ui32ThreadCount = 0;  ui32ThreadCount < RGXFW_THREAD_NUM;  ui32ThreadCount++)
			{
				IMG_CHAR* pszTraceAssertInfo = psRGXFWIfTraceBufCtl->sTraceBuf[ui32ThreadCount].sAssertBuf.szInfo;

				/*
				Check if the FW has hit an assert...
				 */
				if (*pszTraceAssertInfo != '\0')
				{
					PVR_DPF((PVR_DBG_WARNING, "%s: Firmware thread %d has asserted: %s (%s:%d)",
							__func__, ui32ThreadCount, pszTraceAssertInfo,
							psRGXFWIfTraceBufCtl->sTraceBuf[ui32ThreadCount].sAssertBuf.szPath,
							psRGXFWIfTraceBufCtl->sTraceBuf[ui32ThreadCount].sAssertBuf.ui32LineNum));
					eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_DEAD;
					eNewReason = PVRSRV_DEVICE_HEALTH_REASON_ASSERTED;
					{
						/* MTK: dump log */
						static int dump = 0;

						if (dump == 0)
						{
							MTKPP_TriggerAEE(0);
							dump = 1;
						}
					}
					goto _RGXUpdateHealthStatus_Exit;
				}

				/*
				   Check the threads to see if they are in the same poll locations as last time...
				 */
				if (bCheckAfterTimePassed)
				{
					if (psRGXFWIfTraceBufCtl->aui32CrPollAddr[ui32ThreadCount] != 0  &&
							psRGXFWIfTraceBufCtl->aui32CrPollAddr[ui32ThreadCount] == psDevInfo->aui32CrLastPollAddr[ui32ThreadCount])
					{
						PVR_DPF((PVR_DBG_WARNING, "%s: Firmware stuck on CR poll: T%u polling %s (reg:0x%08X mask:0x%08X)",
								__func__, ui32ThreadCount,
								((psRGXFWIfTraceBufCtl->aui32CrPollAddr[ui32ThreadCount] & RGXFW_POLL_TYPE_SET)?("set"):("unset")),
								psRGXFWIfTraceBufCtl->aui32CrPollAddr[ui32ThreadCount] & ~RGXFW_POLL_TYPE_SET,
								psRGXFWIfTraceBufCtl->aui32CrPollMask[ui32ThreadCount]));
						eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_NOT_RESPONDING;
						eNewReason = PVRSRV_DEVICE_HEALTH_REASON_POLL_FAILING;
						goto _RGXUpdateHealthStatus_Exit;
					}
					psDevInfo->aui32CrLastPollAddr[ui32ThreadCount] = psRGXFWIfTraceBufCtl->aui32CrPollAddr[ui32ThreadCount];
				}
			}

			/*
			Check if the FW has faulted...
			 */
			if (psRGXFWIfTraceBufCtl->ui32HWRStateFlags & RGXFWIF_HWR_FW_FAULT)
			{
				PVR_DPF((PVR_DBG_WARNING,
						"%s: Firmware has faulted and needs to restart",
						__func__));
				eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_FAULT;
				if (psRGXFWIfTraceBufCtl->ui32HWRStateFlags & RGXFWIF_HWR_RESTART_REQUESTED)
				{
					eNewReason = PVRSRV_DEVICE_HEALTH_REASON_RESTARTING;
				}
				else
				{
					eNewReason = PVRSRV_DEVICE_HEALTH_REASON_IDLING;
				}
				goto _RGXUpdateHealthStatus_Exit;
			}
		}

		/*
		   Event Object Timeouts check...
		 */
		if (!bCheckAfterTimePassed)
		{
			if (psDevInfo->ui32GEOTimeoutsLastTime > 1 && psPVRSRVData->ui32GEOConsecutiveTimeouts > psDevInfo->ui32GEOTimeoutsLastTime)
			{
				PVR_DPF((PVR_DBG_WARNING, "%s: Global Event Object Timeouts have risen (from %d to %d)",
						__func__,
						psDevInfo->ui32GEOTimeoutsLastTime, psPVRSRVData->ui32GEOConsecutiveTimeouts));
				eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_NOT_RESPONDING;
				eNewReason = PVRSRV_DEVICE_HEALTH_REASON_TIMEOUTS;
			}
			psDevInfo->ui32GEOTimeoutsLastTime = psPVRSRVData->ui32GEOConsecutiveTimeouts;
		}

		/*
		   Check the Kernel CCB pointer is valid. If any commands were waiting last time, then check
		   that some have executed since then.
		 */
		bKCCBCmdsWaiting = IMG_FALSE;
		psKCCBCtl = psDevInfo->psKernelCCBCtl;

		if (psKCCBCtl != NULL)
		{
			if (psKCCBCtl->ui32ReadOffset > psKCCBCtl->ui32WrapMask  ||
					psKCCBCtl->ui32WriteOffset > psKCCBCtl->ui32WrapMask)
			{
				PVR_DPF((PVR_DBG_WARNING, "%s: KCCB has invalid offset (ROFF=%d WOFF=%d)",
						__func__, psKCCBCtl->ui32ReadOffset, psKCCBCtl->ui32WriteOffset));
				eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_DEAD;
				eNewReason = PVRSRV_DEVICE_HEALTH_REASON_QUEUE_CORRUPT;
			}

			if (psKCCBCtl->ui32ReadOffset != psKCCBCtl->ui32WriteOffset)
			{
				bKCCBCmdsWaiting = IMG_TRUE;
			}
		}

		if (bCheckAfterTimePassed && psDevInfo->psRGXFWIfTraceBuf != NULL)
		{
			IMG_UINT32 ui32KCCBCmdsExecuted = psDevInfo->psRGXFWIfTraceBuf->ui32KCCBCmdsExecuted;

			if (psDevInfo->ui32KCCBCmdsExecutedLastTime == ui32KCCBCmdsExecuted)
			{
				/*
				   If something was waiting last time then the Firmware has stopped processing commands.
				 */
				if (psDevInfo->bKCCBCmdsWaitingLastTime)
				{
					PVR_DPF((PVR_DBG_WARNING, "%s: No KCCB commands executed since check!",
							__func__));
					eNewStatus = PVRSRV_DEVICE_HEALTH_STATUS_NOT_RESPONDING;
					eNewReason = PVRSRV_DEVICE_HEALTH_REASON_QUEUE_STALLED;
				}

				/*
				   If no commands are currently pending and nothing happened since the last poll, then
				   schedule a dummy command to ping the firmware so we know it is alive and processing.
				 */
				if (!bKCCBCmdsWaiting)
				{
					/* Protect the PDumpLoadMem. RGXScheduleCommand() cannot take the
					 * PMR lock itself, because some bridge functions will take the PMR lock
					 * before calling RGXScheduleCommand
					 */
					PVRSRV_ERROR eError = RGXFWHealthCheckCmd(psDevNode->pvDevice);

					if (eError != PVRSRV_OK)
					{
						PVR_DPF((PVR_DBG_WARNING, "%s: Cannot schedule Health Check command! (0x%x)",
								__func__, eError));
					}
					else
					{
						bKCCBCmdsWaiting = IMG_TRUE;
					}
				}
			}

			psDevInfo->bKCCBCmdsWaitingLastTime     = bKCCBCmdsWaiting;
			psDevInfo->ui32KCCBCmdsExecutedLastTime = ui32KCCBCmdsExecuted;
		}
	}

	if (bCheckAfterTimePassed && (PVRSRV_DEVICE_HEALTH_STATUS_OK==eNewStatus))
	{
		RGXCheckForStalledClientContexts(psDevInfo, IMG_FALSE);
	}

	/*
	   Finished, save the new status...
	 */
	_RGXUpdateHealthStatus_Exit:
	OSAtomicWrite(&psDevNode->eHealthStatus, eNewStatus);
	OSAtomicWrite(&psDevNode->eHealthReason, eNewReason);
	RGXSRV_HWPERF_DEVICE_INFO(psDevInfo, RGX_HWPERF_DEV_INFO_EV_HEALTH, eNewStatus, eNewReason);

	/*
	 * Attempt to service the HWPerf buffer to regularly transport idle/periodic
	 * packets to host buffer.
	 */
	if (psDevNode->pfnServiceHWPerf != NULL)
	{
		PVRSRV_ERROR eError = psDevNode->pfnServiceHWPerf(psDevNode);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_WARNING, "%s: "
					"Error occurred when servicing HWPerf buffer (%d)",
					__func__, eError));
		}
	}

	/* Attempt to refresh timer correlation data */
	RGXTimeCorrRestartPeriodic(psDevNode);

	return PVRSRV_OK;
} /* RGXUpdateHealthStatus */

PVRSRV_ERROR CheckStalledClientCommonContext(RGX_SERVER_COMMON_CONTEXT *psCurrentServerCommonContext, RGX_KICK_TYPE_DM eKickTypeDM)
{
	if (psCurrentServerCommonContext == NULL)
	{
		/* the context has already been freed so there is nothing to do here */
		return PVRSRV_OK;
	}

	return CheckForStalledCCB(psCurrentServerCommonContext->psDevInfo->psDeviceNode,
	                          psCurrentServerCommonContext->psClientCCB,
	                          eKickTypeDM);
}

void DumpFWCommonContextInfo(RGX_SERVER_COMMON_CONTEXT *psCurrentServerCommonContext,
                             DUMPDEBUG_PRINTF_FUNC *pfnDumpDebugPrintf,
                             void *pvDumpDebugFile,
                             IMG_UINT32 ui32VerbLevel)
{
	if (psCurrentServerCommonContext == NULL)
	{
		/* the context has already been freed so there is nothing to do here */
		return;
	}

	if (DD_VERB_LVL_ENABLED(ui32VerbLevel, DEBUG_REQUEST_VERBOSITY_HIGH))
	{
		/* If high verbosity requested, dump whole CCB */
		DumpCCB(psCurrentServerCommonContext->psDevInfo,
		        psCurrentServerCommonContext->sFWCommonContextFWAddr,
		        psCurrentServerCommonContext->psClientCCB,
		        pfnDumpDebugPrintf,
		        pvDumpDebugFile);
	}
	else
	{
		/* Otherwise, only dump first stalled command in the CCB */
		DumpStalledCCBCommand(psCurrentServerCommonContext->sFWCommonContextFWAddr,
		                      psCurrentServerCommonContext->psClientCCB,
		                      pfnDumpDebugPrintf,
		                      pvDumpDebugFile);
	}
}

void AttachKickResourcesCleanupCtls(PRGXFWIF_CLEANUP_CTL *apsCleanupCtl,
		IMG_UINT32 *pui32NumCleanupCtl,
		RGXFWIF_DM eDM,
		IMG_BOOL bKick,
		RGX_RTDATA_CLEANUP_DATA        *psRTDataCleanup,
		RGX_ZSBUFFER_DATA              *psZBuffer,
		RGX_ZSBUFFER_DATA              *psSBuffer,
		RGX_ZSBUFFER_DATA              *psMSAAScratchBuffer)
{
	PRGXFWIF_CLEANUP_CTL *psCleanupCtlWrite = apsCleanupCtl;

	PVR_ASSERT((eDM == RGXFWIF_DM_TA) || (eDM == RGXFWIF_DM_3D));

	if (bKick)
	{
		if (eDM == RGXFWIF_DM_TA)
		{
			if (psRTDataCleanup)
			{
				PRGXFWIF_CLEANUP_CTL psCleanupCtl;

				RGXSetFirmwareAddress(&psCleanupCtl, psRTDataCleanup->psFWHWRTDataMemDesc,
						offsetof(RGXFWIF_HWRTDATA, sTACleanupState),
						RFW_FWADDR_NOREF_FLAG);

				*(psCleanupCtlWrite++) = psCleanupCtl;
			}
		}
		else
		{
			RGXFWIF_PRBUFFER_TYPE eBufferType;
			RGX_ZSBUFFER_DATA *psBuffer = NULL;

			if (psRTDataCleanup)
			{
				PRGXFWIF_CLEANUP_CTL psCleanupCtl;

				RGXSetFirmwareAddress(&psCleanupCtl, psRTDataCleanup->psFWHWRTDataMemDesc,
						offsetof(RGXFWIF_HWRTDATA, s3DCleanupState),
						RFW_FWADDR_NOREF_FLAG);

				*(psCleanupCtlWrite++) = psCleanupCtl;
			}

			for (eBufferType = RGXFWIF_PRBUFFER_START; eBufferType < RGXFWIF_PRBUFFER_MAXSUPPORTED; eBufferType++)
			{
				switch (eBufferType)
				{
				case RGXFWIF_PRBUFFER_ZBUFFER:
					psBuffer = psZBuffer;
					break;
				case RGXFWIF_PRBUFFER_SBUFFER:
					psBuffer = psSBuffer;
					break;
				case RGXFWIF_PRBUFFER_MSAABUFFER:
					psBuffer = psMSAAScratchBuffer;
					break;
				case RGXFWIF_PRBUFFER_MAXSUPPORTED:
					psBuffer = NULL;
					break;
				}
				if (psBuffer)
				{
					(psCleanupCtlWrite++)->ui32Addr = psBuffer->sZSBufferFWDevVAddr.ui32Addr +
							offsetof(RGXFWIF_PRBUFFER, sCleanupState);
					psBuffer = NULL;
				}
			}
		}
	}

	*pui32NumCleanupCtl = psCleanupCtlWrite - apsCleanupCtl;

	PVR_ASSERT(*pui32NumCleanupCtl <= RGXFWIF_KCCB_CMD_KICK_DATA_MAX_NUM_CLEANUP_CTLS);
}

PVRSRV_ERROR RGXResetHWRLogs(PVRSRV_DEVICE_NODE *psDevNode)
{
	PVRSRV_RGXDEV_INFO	*psDevInfo;
	RGXFWIF_HWRINFOBUF	*psHWRInfoBuf;
	RGXFWIF_TRACEBUF	*psRGXFWIfTraceBufCtl;
	IMG_UINT32			i;

	if (psDevNode->pvDevice == NULL)
	{
		return PVRSRV_ERROR_INVALID_DEVINFO;
	}
	psDevInfo = psDevNode->pvDevice;

	psHWRInfoBuf = psDevInfo->psRGXFWIfHWRInfoBuf;
	psRGXFWIfTraceBufCtl = psDevInfo->psRGXFWIfTraceBuf;

	for (i = 0 ; i < RGXFWIF_DM_MAX ; i++)
	{
		/* Reset the HWR numbers */
		psRGXFWIfTraceBufCtl->aui32HwrDmLockedUpCount[i] = 0;
		psRGXFWIfTraceBufCtl->aui32HwrDmFalseDetectCount[i] = 0;
		psRGXFWIfTraceBufCtl->aui32HwrDmRecoveredCount[i] = 0;
		psRGXFWIfTraceBufCtl->aui32HwrDmOverranCount[i] = 0;
	}

	for (i = 0 ; i < RGXFWIF_HWINFO_MAX ; i++)
	{
		psHWRInfoBuf->sHWRInfo[i].ui32HWRNumber = 0;
	}

	for (i = 0 ; i < RGXFW_THREAD_NUM ; i++)
	{
		psHWRInfoBuf->ui32FirstCrPollAddr[i] = 0;
		psHWRInfoBuf->ui32FirstCrPollMask[i] = 0;
		psHWRInfoBuf->ui32FirstCrPollLastValue[i] = 0;
	}

	psHWRInfoBuf->ui32WriteIndex = 0;
	psHWRInfoBuf->ui32DDReqCount = 0;

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXGetPhyAddr(PMR *psPMR,
		IMG_DEV_PHYADDR *psPhyAddr,
		IMG_UINT32 ui32LogicalOffset,
		IMG_UINT32 ui32Log2PageSize,
		IMG_UINT32 ui32NumOfPages,
		IMG_BOOL *bValid)
{

	PVRSRV_ERROR eError;

	eError = PMRLockSysPhysAddresses(psPMR);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: PMRLockSysPhysAddresses failed (%u)",
				__func__,
				eError));
		return eError;
	}

	eError = PMR_DevPhysAddr(psPMR,
			ui32Log2PageSize,
			ui32NumOfPages,
			ui32LogicalOffset,
			psPhyAddr,
			bValid);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: PMR_DevPhysAddr failed (%u)",
				__func__,
				eError));
		return eError;
	}


	eError = PMRUnlockSysPhysAddresses(psPMR);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: PMRUnLockSysPhysAddresses failed (%u)",
				__func__,
				eError));
		return eError;
	}

	return eError;
}

#if defined(PDUMP)
PVRSRV_ERROR RGXPdumpDrainKCCB(PVRSRV_RGXDEV_INFO *psDevInfo, IMG_UINT32 ui32WriteOffset)
{
	RGXFWIF_CCB_CTL	*psKCCBCtl = psDevInfo->psKernelCCBCtl;
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (psDevInfo->bDumpedKCCBCtlAlready)
	{
		/* exiting capture range or pdump block */
		psDevInfo->bDumpedKCCBCtlAlready = IMG_FALSE;

		/* make sure previous cmd is drained in pdump in case we will 'jump' over some future cmds */
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWER,
				"kCCB(%p): Draining rgxfw_roff (0x%x) == woff (0x%x)",
				psKCCBCtl,
				ui32WriteOffset,
				ui32WriteOffset);
		eError = DevmemPDumpDevmemPol32(psDevInfo->psKernelCCBCtlMemDesc,
				offsetof(RGXFWIF_CCB_CTL, ui32ReadOffset),
				ui32WriteOffset,
				0xffffffff,
				PDUMP_POLL_OPERATOR_EQUAL,
				PDUMP_FLAGS_CONTINUOUS | PDUMP_FLAGS_POWER);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXPdumpDrainKCCB: problem pdumping POL for kCCBCtl (%d)", eError));
		}
	}

	return eError;

}
#endif

/*!
 *******************************************************************************

 @Function	RGXClientConnectCompatCheck_ClientAgainstFW

 @Description

 Check compatibility of client and firmware (build options)
 at the connection time.

 @Input psDeviceNode - device node
 @Input ui32ClientBuildOptions - build options for the client

 @Return   PVRSRV_ERROR - depending on mismatch found

 ******************************************************************************/
PVRSRV_ERROR IMG_CALLCONV RGXClientConnectCompatCheck_ClientAgainstFW(PVRSRV_DEVICE_NODE * psDeviceNode, IMG_UINT32 ui32ClientBuildOptions)
{
#if !defined(NO_HARDWARE) || defined(PDUMP)
#if !defined(NO_HARDWARE)
	IMG_UINT32		ui32BuildOptionsMismatch;
	IMG_UINT32		ui32BuildOptionsFW;
#endif
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
#endif
	PVRSRV_VZ_RET_IF_MODE(DRIVER_MODE_GUEST, PVRSRV_OK);

#if !defined(NO_HARDWARE)
	if (psDevInfo == NULL || psDevInfo->psRGXFWIfInitMemDesc == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Cannot acquire kernel fw compatibility check info, RGXFWIF_INIT structure not allocated.",
				__func__));
		return PVRSRV_ERROR_NOT_INITIALISED;
	}

	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		if (*((volatile IMG_BOOL *) &psDevInfo->psRGXFWIfInit->sRGXCompChecks.bUpdated))
		{
			/* No need to wait if the FW has already updated the values */
			break;
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} END_LOOP_UNTIL_TIMEOUT();
#endif

#if defined(PDUMP)
	{
		PVRSRV_ERROR eError;

		PDUMPCOMMENT("Compatibility check: client and FW build options");
		eError = DevmemPDumpDevmemPol32(psDevInfo->psRGXFWIfInitMemDesc,
				offsetof(RGXFWIF_INIT, sRGXCompChecks) +
				offsetof(RGXFWIF_COMPCHECKS, ui32BuildOptions),
				ui32ClientBuildOptions,
				0xffffffff,
				PDUMP_POLL_OPERATOR_EQUAL,
				PDUMP_FLAGS_CONTINUOUS);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					"%s: problem pdumping POL for psRGXFWIfInitMemDesc (%d)",
					__func__,
					eError));
			return eError;
		}
	}
#endif

#if !defined(NO_HARDWARE)
	ui32BuildOptionsFW = psDevInfo->psRGXFWIfInit->sRGXCompChecks.ui32BuildOptions;
	ui32BuildOptionsMismatch = ui32ClientBuildOptions ^ ui32BuildOptionsFW;

	if (ui32BuildOptionsMismatch != 0)
	{
		if ( (ui32ClientBuildOptions & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware and client build options; "
					"extra options present in client: (0x%x). Please check rgx_options.h",
					ui32ClientBuildOptions & ui32BuildOptionsMismatch ));
		}

		if ( (ui32BuildOptionsFW & ui32BuildOptionsMismatch) != 0)
		{
			PVR_LOG(("(FAIL) RGXDevInitCompatCheck: Mismatch in Firmware and client build options; "
					"extra options present in Firmware: (0x%x). Please check rgx_options.h",
					ui32BuildOptionsFW & ui32BuildOptionsMismatch ));
		}

		return PVRSRV_ERROR_BUILD_OPTIONS_MISMATCH;
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "RGXDevInitCompatCheck: Firmware and client build options match. [ OK ]"));
	}
#endif

	return PVRSRV_OK;
}

/*!
 *******************************************************************************

 @Function	RGXVzRegisterFirmwarePhysHeap

 @Description Register firmware heap for the specified guest OSID

 @Input psDeviceNode - device node
 @Input ui32OSID     - Guest OSID
 @Input sDevPAddr    - Heap address
 @Input ui64DevPSize - Heap size

 @Return   PVRSRV_ERROR - PVRSRV_OK if heap setup was successful.

 ******************************************************************************/
PVRSRV_ERROR RGXVzRegisterFirmwarePhysHeap(PVRSRV_DEVICE_NODE *psDeviceNode,
		IMG_UINT32 ui32OSID,
		IMG_DEV_PHYADDR sDevPAddr,
		IMG_UINT64 ui64DevPSize)
{
	PVRSRV_ERROR eError;
	PVRSRV_VZ_RET_IF_NOT_MODE(DRIVER_MODE_HOST, PVRSRV_OK);

	if (!ui32OSID ||
		!ui64DevPSize ||
		!sDevPAddr.uiAddr ||
		ui32OSID >= RGXFW_NUM_OS ||
		ui64DevPSize != RGX_FIRMWARE_RAW_HEAP_SIZE)
	{
		/* Guest OSID(s) range [1 up to (RGXFW_NUM_OS-1)] */
		PVR_DPF((PVR_DBG_ERROR, "Invalid guest %d fw physheap spec", ui32OSID));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Registration creates internal RA to managed the guest(s) firmware heap */
	eError = PVRSRVVzRegisterFirmwarePhysHeap (psDeviceNode, sDevPAddr, ui64DevPSize, ui32OSID);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Registering guest %d fw physheap failed", ui32OSID));
		return eError;
	}

	/* Map guest DMA fw physheap into the fw kernel memory context */
	eError = RGXVzDevMemAllocateGuestFwHeap(psDeviceNode, ui32OSID);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Mapping guest %d fw physheap failed", ui32OSID));
		return eError;
	}

	return eError;
}

/*!
 *******************************************************************************

 @Function	RGXVzUnregisterFirmwarePhysHeap

 @Description Unregister firmware heap for the specified guest OSID

 @Input psDeviceNode - device node
 @Input ui32OSID     - Guest OSID

 @Return   PVRSRV_ERROR - PVRSRV_OK if heap setup was successful.

 ******************************************************************************/
PVRSRV_ERROR RGXVzUnregisterFirmwarePhysHeap(PVRSRV_DEVICE_NODE *psDeviceNode,
		IMG_UINT32 ui32OSID)
{
	PVRSRV_ERROR eError;
	PVRSRV_VZ_RET_IF_NOT_MODE(DRIVER_MODE_HOST, PVRSRV_OK);

	if (!ui32OSID || ui32OSID >= RGXFW_NUM_OS)
	{
		/* Guest OSID(s) range [1 up to (RGXFW_NUM_OS-1)] */
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Free guest fw physheap from fw kernel memory context */
	RGXVzDevMemFreeGuestFwHeap(psDeviceNode, ui32OSID);

	/* Unregistration deletes state required to maintain heap */
	eError = PVRSRVVzUnregisterFirmwarePhysHeap (psDeviceNode, ui32OSID);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Registering guest %d fw physheap failed", ui32OSID));
		return eError;
	}

	return eError;
}

/*!
 *******************************************************************************

 @Function	RGXVzCreateFWKernelMemoryContext

 @Description Setup additional firmware state specific to VZ

 @Input psDeviceNode - device node

 @Return   PVRSRV_ERROR - PVRSRV_OK if successful.

 ******************************************************************************/
PVRSRV_ERROR RGXVzCreateFWKernelMemoryContext(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_VZ_RET_IF_MODE(DRIVER_MODE_NATIVE, PVRSRV_OK);

	if (PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST))
	{
		eError = SysVzRegisterFwPhysHeap(psDeviceNode->psDevConfig);
	}
#if (RGXFW_GUEST_OSID_START < RGXFW_NUM_OS)
	else
	{
		PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
		IMG_CHAR szHeapName[32];
		IMG_UINT32 ui32OSID;

		/* Initialise each guest OSID firmware physheap heaps, note that the guest
		   OSID(s) range is [RGXFW_GUEST_OSID_START up to (RGXFW_NUM_OS-1)] */
		for (ui32OSID = RGXFW_GUEST_OSID_START; ui32OSID < RGXFW_NUM_OS; ui32OSID++)
		{
			OSSNPrintf(szHeapName, sizeof(szHeapName), RGX_FIRMWARE_GUEST_RAW_HEAP_IDENT, ui32OSID);

			eError = DevmemFindHeapByName(psDevInfo->psKernelDevmemCtx, szHeapName,
					&psDevInfo->psGuestFirmwareRawHeap[ui32OSID]);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "DevmemFindHeapByName() for guest %d failed", ui32OSID));
			}
		}
	}
#endif

	return eError;
}

/*!
 *******************************************************************************

 @Function	RGXVzDestroyFWKernelMemoryContext

 @Description Destroy additional firmware state specific to VZ

 @Input psDeviceNode - device node

 @Return   PVRSRV_ERROR - PVRSRV_OK if successful.

 ******************************************************************************/
PVRSRV_ERROR RGXVzDestroyFWKernelMemoryContext(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_VZ_RET_IF_MODE(DRIVER_MODE_NATIVE, PVRSRV_OK);

	if (PVRSRV_VZ_MODE_IS(DRIVER_MODE_GUEST))
	{
		return SysVzUnregisterFwPhysHeap(psDeviceNode->psDevConfig);
	}
	return PVRSRV_OK;
}


IMG_UINT32 RGXGetFwMainHeapSize(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	if (psDevInfo == NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid device info", __func__));
		return 0;
	}

	if (RGX_IS_FEATURE_SUPPORTED(psDevInfo, MIPS))
	{
		return RGX_FIRMWARE_MIPS_MAIN_HEAP_SIZE;
	}
	else
	{
		return RGX_FIRMWARE_META_MAIN_HEAP_SIZE;
	}
}

/******************************************************************************
 End of file (rgxfwutils.c)
 ******************************************************************************/
