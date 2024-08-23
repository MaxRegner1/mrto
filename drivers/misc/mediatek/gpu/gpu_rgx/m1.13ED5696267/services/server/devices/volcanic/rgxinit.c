/*************************************************************************/ /*!
@File
@Title          Device specific initialisation routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Device specific functions
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
#include "pvr_notifier.h"
#include "pvrsrv.h"
#include "pvrsrv_bridge_init.h"
#include "syscommon.h"
#include "rgx_heaps.h"
#include "rgxheapconfig.h"
#include "rgxdefs_km.h"
#include "rgxpower.h"
#include "tlstream.h"
#include "pvrsrv_tlstreams.h"

#include "rgxinit.h"
#include "rgxbvnc.h"

#include "pdump_km.h"
#include "handle.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "rgxmem.h"
#include "sync_internal.h"
#include "pvrsrv_apphint.h"
#include "oskm_apphint.h"
#include "rgxfwdbg.h"
#include "info_page.h"

#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgx_fwif_km.h"

#include "rgxmmuinit.h"
#include "devicemem_utils.h"
#include "devicemem_server.h"
#include "physmem_osmem.h"
#include "physmem_lma.h"

#include "rgxdebug.h"
#include "rgxhwperf.h"
#include "htbserver.h"

#include "rgx_options.h"
#include "pvrversion.h"

#include "rgx_compat_bvnc.h"

#include "rgx_heaps.h"

#include "rgxta3d.h"
#include "rgxtimecorr.h"
#include "rgxshader.h"

#if defined(PDUMP)
#include "rgxstartstop.h"
#endif

#include "rgx_fwif_alignchecks.h"
#include "vmm_pvz_client.h"

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
#include "rgxworkest.h"
#endif
#if defined(SUPPORT_PDVFS)
#include "rgxpdvfs.h"
#endif

#if defined(SUPPORT_VALIDATION) && defined(SUPPORT_SOC_TIMER) && defined(PDUMP) && defined(NO_HARDWARE)
#include "validation_soc.h"
#endif

#if defined(PDUMP) && defined(SUPPORT_SECURITY_VALIDATION)
#include "pdump_physmem.h"
#endif

static PVRSRV_ERROR RGXDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode);
static PVRSRV_ERROR RGXDevVersionString(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_CHAR **ppszVersionString);
static PVRSRV_ERROR RGXDevClockSpeed(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_PUINT32 pui32RGXClockSpeed);
static PVRSRV_ERROR RGXSoftReset(PVRSRV_DEVICE_NODE *psDeviceNode, IMG_UINT64 ui64ResetValue, IMG_UINT64 ui64SPUResetValue);
static PVRSRV_ERROR RGXPhysMemDeviceHeapsInit(PVRSRV_DEVICE_NODE *psDeviceNode);

#if (RGX_NUM_OS_SUPPORTED > 1)
static PVRSRV_ERROR RGXInitFwRawHeap(DEVMEM_HEAP_BLUEPRINT *psDevMemHeap, IMG_UINT32 ui32OSid);
static void RGXDeInitFwRawHeap(DEVMEM_HEAP_BLUEPRINT *psDevMemHeap);
#endif

#if defined(SUPPORT_AUTOVZ)
#define RGX_FW_MMU_RESERVED_MEM_SETUP(devnode)	(MMU_PX_SETUP) {							\
													LMA_PhyContigPagesAlloc,				\
													LMA_PhyContigPagesFree,					\
													LMA_PhyContigPagesMap,					\
													LMA_PhyContigPagesUnmap,				\
													LMA_PhyContigPagesClean,				\
													OSGetPageShift(),						\
													(devnode)->psFwMMUReservedMemArena		\
												}
#endif


#define RGX_MMU_LOG2_PAGE_SIZE_4KB   (12)
#define RGX_MMU_LOG2_PAGE_SIZE_16KB  (14)
#define RGX_MMU_LOG2_PAGE_SIZE_64KB  (16)
#define RGX_MMU_LOG2_PAGE_SIZE_256KB (18)
#define RGX_MMU_LOG2_PAGE_SIZE_1MB   (20)
#define RGX_MMU_LOG2_PAGE_SIZE_2MB   (21)

#define RGX_MMU_PAGE_SIZE_4KB   (   4 * 1024)
#define RGX_MMU_PAGE_SIZE_16KB  (  16 * 1024)
#define RGX_MMU_PAGE_SIZE_64KB  (  64 * 1024)
#define RGX_MMU_PAGE_SIZE_256KB ( 256 * 1024)
#define RGX_MMU_PAGE_SIZE_1MB   (1024 * 1024)
#define RGX_MMU_PAGE_SIZE_2MB   (2048 * 1024)
#define RGX_MMU_PAGE_SIZE_MIN RGX_MMU_PAGE_SIZE_4KB
#define RGX_MMU_PAGE_SIZE_MAX RGX_MMU_PAGE_SIZE_2MB

#define VAR(x) #x

#define MAX_BVNC_LEN (12)
#define RGXBVNC_BUFFER_SIZE (((PVRSRV_MAX_DEVICES)*(MAX_BVNC_LEN))+1)

static void RGXDeInitHeaps(DEVICE_MEMORY_INFO *psDevMemoryInfo);

#if defined(PVRSRV_DEBUG_LISR_EXECUTION)

/* bits used by the LISR to provide a trace of its last execution */
#define RGX_LISR_DEVICE_NOT_POWERED	(1 << 0)
#define RGX_LISR_FWIF_POW_OFF		(1 << 1)
#define RGX_LISR_EVENT_EN		(1 << 2)
#define RGX_LISR_COUNTS_EQUAL		(1 << 3)
#define RGX_LISR_PROCESSED		(1 << 4)

typedef struct _LISR_EXECUTION_INFO_
{
	/* bit mask showing execution flow of last LISR invocation */
	IMG_UINT32 ui32State;
	/* snapshot from the last LISR invocation, regardless of
	 * whether an interrupt was handled
	 */
	IMG_UINT32 aui32InterruptCountSnapshot[RGXFW_THREAD_NUM];
	/* time of the last LISR invocation */
	IMG_UINT64 ui64Clockns;
} LISR_EXECUTION_INFO;

/* information about the last execution of the LISR */
static LISR_EXECUTION_INFO g_sLISRExecutionInfo;

#endif

#if !defined(NO_HARDWARE)
/*************************************************************************/ /*!
@Function       SampleIRQCount
@Description    Utility function taking snapshots of RGX FW interrupt count.
@Input          paui32Input  A pointer to RGX FW IRQ count array.
                             Size of the array should be equal to RGX FW thread
                             count.
@Input          paui32Output A pointer to array containing sampled RGX FW
                             IRQ counts
@Return         IMG_BOOL     Returns IMG_TRUE, if RGX FW IRQ is not equal to
                             sampled RGX FW IRQ count for any RGX FW thread.
*/ /**************************************************************************/
static INLINE IMG_BOOL SampleIRQCount(volatile IMG_UINT32 *paui32Input,
									  volatile IMG_UINT32 *paui32Output)
{
	IMG_UINT32 ui32TID;
	IMG_BOOL bReturnVal = IMG_FALSE;

	for (ui32TID = 0; ui32TID < RGXFW_THREAD_NUM; ui32TID++)
	{
		if (paui32Output[ui32TID] != paui32Input[ui32TID])
		{
			/**
			 * we are handling any unhandled interrupts here so align the host
			 * count with the FW count
			 */

			/* Sample the current count from the FW _after_ we've cleared the interrupt. */
			paui32Output[ui32TID] = paui32Input[ui32TID];
			bReturnVal = IMG_TRUE;
		}
	}

	return bReturnVal;
}

static IMG_BOOL _WaitForInterruptsTimeoutCheck(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	RGXFWIF_SYSDATA *psFwSysData = psDevInfo->psRGXFWIfFwSysData;
	IMG_BOOL bScheduleMISR = IMG_FALSE;
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
	IMG_UINT32 ui32TID;
#endif

	RGXDEBUG_PRINT_IRQ_COUNT(psDevInfo);

#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
	PVR_DPF((PVR_DBG_ERROR,
	        "Last RGX_LISRHandler State: 0x%08X Clock: %llu",
	        g_sLISRExecutionInfo.ui32State,
	        g_sLISRExecutionInfo.ui64Clockns));

	for (ui32TID = 0; ui32TID < RGXFW_THREAD_NUM; ui32TID++)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"RGX FW thread %u: InterruptCountSnapshot: 0x%X",
				ui32TID, g_sLISRExecutionInfo.aui32InterruptCountSnapshot[ui32TID]));
	}
#else
	PVR_DPF((PVR_DBG_ERROR, "No further information available. Please enable PVRSRV_DEBUG_LISR_EXECUTION"));
#endif


	if (psFwSysData->ePowState != RGXFWIF_POW_OFF)
	{
		PVR_DPF((PVR_DBG_ERROR, "_WaitForInterruptsTimeout: FW pow state is not OFF (is %u)",
				(unsigned int) psFwSysData->ePowState));
	}

	bScheduleMISR = SampleIRQCount(psFwSysData->aui32InterruptCount,
								   psDevInfo->aui32SampleIRQCount);
	return bScheduleMISR;
}

void RGX_WaitForInterruptsTimeout(PVRSRV_RGXDEV_INFO *psDevInfo)
{
	IMG_BOOL bScheduleMISR;

	if (PVRSRV_VZ_MODE_IS(GUEST))
	{
		bScheduleMISR = IMG_TRUE;
	}
	else
	{
		bScheduleMISR = _WaitForInterruptsTimeoutCheck(psDevInfo);
	}

	if (bScheduleMISR)
	{
		OSScheduleMISR(psDevInfo->pvMISRData);

		if (psDevInfo->pvAPMISRData != NULL)
		{
			OSScheduleMISR(psDevInfo->pvAPMISRData);
		}
	}
}

/*
	RGX LISR Handler
*/
static IMG_BOOL RGX_LISRHandler (void *pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = pvData;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	IMG_BOOL bInterruptProcessed;
	RGXFWIF_SYSDATA *psFwSysData;
	IMG_UINT32 ui32IRQStatus;
	IMG_UINT32 ui32IRQThreadMask;

	if (PVRSRV_VZ_MODE_IS(GUEST))
	{
		if (! psDevInfo->bRGXPowered)
		{
			return IMG_FALSE;
		}

		/* guest OS should also clear interrupt register */
		ui32IRQStatus = OSReadHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_IRQ_OS0_EVENT_STATUS);
		ui32IRQThreadMask = (ui32IRQStatus & ~RGX_CR_IRQ_OS0_EVENT_STATUS_SOURCE_CLRMSK);
		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_IRQ_OS0_EVENT_CLEAR, ui32IRQThreadMask);
		OSScheduleMISR(psDevInfo->pvMISRData);
		return IMG_TRUE;
	}
	else
	{
		bInterruptProcessed = IMG_FALSE;
		psFwSysData = psDevInfo->psRGXFWIfFwSysData;
	}

#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
	{
		IMG_UINT32 ui32TID;

		for (ui32TID = 0; ui32TID < RGXFW_THREAD_NUM; ui32TID++)
		{
			g_sLISRExecutionInfo.aui32InterruptCountSnapshot[ui32TID] = psFwSysData->aui32InterruptCount[ui32TID];
		}

		g_sLISRExecutionInfo.ui32State = 0;
		g_sLISRExecutionInfo.ui64Clockns = OSClockns64();
	}
#endif

	if (psDevInfo->bRGXPowered == IMG_FALSE)
	{
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
		g_sLISRExecutionInfo.ui32State |= RGX_LISR_DEVICE_NOT_POWERED;
#endif
		if (psFwSysData->ePowState == RGXFWIF_POW_OFF)
		{
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
			g_sLISRExecutionInfo.ui32State |= RGX_LISR_FWIF_POW_OFF;
#endif
			return bInterruptProcessed;
		}
	}

	/*
	 * Due to the remappings done by the hypervisor we
	 * will always "see" our registers in OS0s regbank
	 */
	ui32IRQStatus = OSReadHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_IRQ_OS0_EVENT_STATUS);
	ui32IRQThreadMask = (ui32IRQStatus & ~RGX_CR_IRQ_OS0_EVENT_STATUS_SOURCE_CLRMSK);

	/* Check for an interrupt from _any_ FW thread */
	if (ui32IRQThreadMask != 0)
	{
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
		g_sLISRExecutionInfo.ui32State |= RGX_LISR_EVENT_EN;
		psDeviceNode->ui64nLISR++;
#endif

		OSWriteHWReg32(psDevInfo->pvRegsBaseKM, RGX_CR_IRQ_OS0_EVENT_CLEAR, ui32IRQThreadMask);

		bInterruptProcessed = SampleIRQCount(psFwSysData->aui32InterruptCount,
											 psDevInfo->aui32SampleIRQCount);

		if (!bInterruptProcessed)
		{
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
			g_sLISRExecutionInfo.ui32State |= RGX_LISR_COUNTS_EQUAL;
#endif
			return bInterruptProcessed;
		}

		bInterruptProcessed = IMG_TRUE;
#if defined(PVRSRV_DEBUG_LISR_EXECUTION)
		g_sLISRExecutionInfo.ui32State |= RGX_LISR_PROCESSED;
		psDeviceNode->ui64nMISR++;
#endif

		OSScheduleMISR(psDevInfo->pvMISRData);

		if (psDevInfo->pvAPMISRData != NULL)
		{
			OSScheduleMISR(psDevInfo->pvAPMISRData);
		}
	}

	return bInterruptProcessed;
}

static void RGX_MISR_ProcessKCCBDeferredList(PVRSRV_DEVICE_NODE	*psDeviceNode)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	OS_SPINLOCK_FLAGS uiFlags;

	/* First check whether there are pending commands in Deferred KCCB List */
	OSSpinLockAcquire(psDevInfo->hLockKCCBDeferredCommandsList, uiFlags);
	if (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHead))
	{
		OSSpinLockRelease(psDevInfo->hLockKCCBDeferredCommandsList, uiFlags);
		return;
	}
	OSSpinLockRelease(psDevInfo->hLockKCCBDeferredCommandsList, uiFlags);

	/* Powerlock to avoid further Power transition requests
	   while KCCB deferred list is being processed */
	eError = PVRSRVPowerLock(psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "%s: Failed to acquire PowerLock (device: %p, error: %s)",
				 __func__, psDeviceNode, PVRSRVGetErrorString(eError)));
		goto _RGX_MISR_ProcessKCCBDeferredList_PowerLock_failed;
	}

	/* Try to send deferred KCCB commands Do not Poll from here*/
	eError = RGXSendCommandsFromDeferredList(psDevInfo, IMG_FALSE);

	PVRSRVPowerUnlock(psDeviceNode);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_MESSAGE,
				 "%s could not flush Deferred KCCB list, KCCB is full.",
				 __func__));
	}

_RGX_MISR_ProcessKCCBDeferredList_PowerLock_failed:

	return;
}

static void RGX_MISRHandler_CheckFWActivePowerState(void *psDevice)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode = psDevice;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_SYSDATA *psFwSysData = psDevInfo->psRGXFWIfFwSysData;
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (psFwSysData->ePowState == RGXFWIF_POW_ON || psFwSysData->ePowState == RGXFWIF_POW_IDLE)
	{
		RGX_MISR_ProcessKCCBDeferredList(psDeviceNode);
	}

	if (psFwSysData->ePowState == RGXFWIF_POW_IDLE)
	{
		/* The FW is IDLE and therefore could be shut down */
		eError = RGXActivePowerRequest(psDeviceNode);

		if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_DEVICE_POWER_CHANGE_DENIED))
		{
			if (eError != PVRSRV_ERROR_RETRY)
			{
				PVR_DPF((PVR_DBG_WARNING,
					"%s: Failed RGXActivePowerRequest call (device: %p) with %s",
					__func__, psDeviceNode, PVRSRVGetErrorString(eError)));
				PVRSRVDebugRequest(psDeviceNode, DEBUG_REQUEST_VERBOSITY_MAX, NULL, NULL);
			}
			else
			{
				/* Re-schedule the power down request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvAPMISRData);
			}
		}
	}

}

/* Shorter defines to keep the code a bit shorter */
#define GPU_IDLE       RGXFWIF_GPU_UTIL_STATE_IDLE
#define GPU_ACTIVE     RGXFWIF_GPU_UTIL_STATE_ACTIVE
#define GPU_BLOCKED    RGXFWIF_GPU_UTIL_STATE_BLOCKED
#define MAX_ITERATIONS 64

static PVRSRV_ERROR RGXGetGpuUtilStats(PVRSRV_DEVICE_NODE *psDeviceNode,
                                       IMG_HANDLE hGpuUtilUser,
                                       RGXFWIF_GPU_UTIL_STATS *psReturnStats)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	volatile RGXFWIF_GPU_UTIL_FWCB *psUtilFWCb = psDevInfo->psRGXFWIfGpuUtilFWCb;
	RGXFWIF_GPU_UTIL_STATS *psAggregateStats;
	IMG_UINT64 ui64TimeNow;
	IMG_UINT32 ui32Attempts;
	IMG_UINT32 ui32Remainder;


	/***** (1) Initialise return stats *****/

	psReturnStats->bValid = IMG_FALSE;
	psReturnStats->ui64GpuStatIdle       = 0;
	psReturnStats->ui64GpuStatActive     = 0;
	psReturnStats->ui64GpuStatBlocked    = 0;
	psReturnStats->ui64GpuStatCumulative = 0;

	if (hGpuUtilUser == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	psAggregateStats = hGpuUtilUser;


	/* Try to acquire GPU utilisation counters and repeat if the FW is in the middle of an update */
	for (ui32Attempts = 0; ui32Attempts < 4; ui32Attempts++)
	{
		IMG_UINT64 aui64TmpCounters[RGXFWIF_GPU_UTIL_STATE_NUM] = {0};
		IMG_UINT64 ui64LastPeriod = 0, ui64LastWord = 0, ui64LastState = 0, ui64LastTime = 0;
		IMG_UINT32 i = 0;


		/***** (2) Get latest data from shared area *****/

		OSLockAcquire(psDevInfo->hGPUUtilLock);

		/*
		 * First attempt at detecting if the FW is in the middle of an update.
		 * This should also help if the FW is in the middle of a 64 bit variable update.
		 */
		while (((ui64LastWord != psUtilFWCb->ui64LastWord) ||
				(aui64TmpCounters[ui64LastState] !=
				 psUtilFWCb->aui64StatsCounters[ui64LastState])) &&
			   (i < MAX_ITERATIONS))
		{
			ui64LastWord  = psUtilFWCb->ui64LastWord;
			ui64LastState = RGXFWIF_GPU_UTIL_GET_STATE(ui64LastWord);
			aui64TmpCounters[GPU_IDLE]    = psUtilFWCb->aui64StatsCounters[GPU_IDLE];
			aui64TmpCounters[GPU_ACTIVE]  = psUtilFWCb->aui64StatsCounters[GPU_ACTIVE];
			aui64TmpCounters[GPU_BLOCKED] = psUtilFWCb->aui64StatsCounters[GPU_BLOCKED];
			i++;
		}

		OSLockRelease(psDevInfo->hGPUUtilLock);

		if (i == MAX_ITERATIONS)
		{
			PVR_DPF((PVR_DBG_WARNING,
			         "RGXGetGpuUtilStats could not get reliable data after trying %u times", i));
			return PVRSRV_ERROR_TIMEOUT;
		}


		/***** (3) Compute return stats *****/

		/* Update temp counters to account for the time since the last update to the shared ones */
		OSMemoryBarrier(); /* Ensure the current time is read after the loop above */
		ui64TimeNow    = RGXFWIF_GPU_UTIL_GET_TIME(RGXTimeCorrGetClockns64());
		ui64LastTime   = RGXFWIF_GPU_UTIL_GET_TIME(ui64LastWord);
		ui64LastPeriod = RGXFWIF_GPU_UTIL_GET_PERIOD(ui64TimeNow, ui64LastTime);
		aui64TmpCounters[ui64LastState] += ui64LastPeriod;

		/* Get statistics for a user since its last request */
		psReturnStats->ui64GpuStatIdle = RGXFWIF_GPU_UTIL_GET_PERIOD(aui64TmpCounters[GPU_IDLE],
		                                                             psAggregateStats->ui64GpuStatIdle);
		psReturnStats->ui64GpuStatActive = RGXFWIF_GPU_UTIL_GET_PERIOD(aui64TmpCounters[GPU_ACTIVE],
		                                                               psAggregateStats->ui64GpuStatActive);
		psReturnStats->ui64GpuStatBlocked = RGXFWIF_GPU_UTIL_GET_PERIOD(aui64TmpCounters[GPU_BLOCKED],
		                                                                psAggregateStats->ui64GpuStatBlocked);
		psReturnStats->ui64GpuStatCumulative = psReturnStats->ui64GpuStatIdle +
		                                       psReturnStats->ui64GpuStatActive +
		                                       psReturnStats->ui64GpuStatBlocked;

		if (psAggregateStats->ui64TimeStamp != 0)
		{
			IMG_UINT64 ui64TimeSinceLastCall = ui64TimeNow - psAggregateStats->ui64TimeStamp;
			/* We expect to return at least 75% of the time since the last call in GPU stats */
			IMG_UINT64 ui64MinReturnedStats = ui64TimeSinceLastCall - (ui64TimeSinceLastCall / 4);

			/*
			 * If the returned stats are substantially lower than the time since
			 * the last call, then the Host might have read a partial update from the FW.
			 * If this happens, try sampling the shared counters again.
			 */
			if (psReturnStats->ui64GpuStatCumulative < ui64MinReturnedStats)
			{
				PVR_DPF((PVR_DBG_MESSAGE,
				         "%s: Return stats (%" IMG_UINT64_FMTSPEC ") too low "
				         "(call period %" IMG_UINT64_FMTSPEC ")",
				         __func__, psReturnStats->ui64GpuStatCumulative, ui64TimeSinceLastCall));
				PVR_DPF((PVR_DBG_MESSAGE, "%s: Attempt #%u has failed, trying again",
				         __func__, ui32Attempts));
				continue;
			}
		}

		break;
	}


	/***** (4) Update aggregate stats for the current user *****/

	psAggregateStats->ui64GpuStatIdle    += psReturnStats->ui64GpuStatIdle;
	psAggregateStats->ui64GpuStatActive  += psReturnStats->ui64GpuStatActive;
	psAggregateStats->ui64GpuStatBlocked += psReturnStats->ui64GpuStatBlocked;
	psAggregateStats->ui64TimeStamp       = ui64TimeNow;


	/***** (5) Convert return stats to microseconds *****/

	psReturnStats->ui64GpuStatIdle       = OSDivide64(psReturnStats->ui64GpuStatIdle, 1000, &ui32Remainder);
	psReturnStats->ui64GpuStatActive     = OSDivide64(psReturnStats->ui64GpuStatActive, 1000, &ui32Remainder);
	psReturnStats->ui64GpuStatBlocked    = OSDivide64(psReturnStats->ui64GpuStatBlocked, 1000, &ui32Remainder);
	psReturnStats->ui64GpuStatCumulative = OSDivide64(psReturnStats->ui64GpuStatCumulative, 1000, &ui32Remainder);

	/* Check that the return stats make sense */
	if (psReturnStats->ui64GpuStatCumulative == 0)
	{
		/* We can enter here only if all the RGXFWIF_GPU_UTIL_GET_PERIOD
		 * returned 0. This could happen if the GPU frequency value
		 * is not well calibrated and the FW is updating the GPU state
		 * while the Host is reading it.
		 * When such an event happens frequently, timers or the aggregate
		 * stats might not be accurate...
		 */
		PVR_DPF((PVR_DBG_WARNING, "RGXGetGpuUtilStats could not get reliable data."));
		return PVRSRV_ERROR_RESOURCE_UNAVAILABLE;
	}

	psReturnStats->bValid = IMG_TRUE;

	return PVRSRV_OK;
}

PVRSRV_ERROR SORgxGpuUtilStatsRegister(IMG_HANDLE *phGpuUtilUser)
{
	RGXFWIF_GPU_UTIL_STATS *psAggregateStats;

	/* NoStats used since this may be called outside of the register/de-register
	 * process calls which track memory use. */
	psAggregateStats = OSAllocMemNoStats(sizeof(RGXFWIF_GPU_UTIL_STATS));
	if (psAggregateStats == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psAggregateStats->ui64GpuStatIdle    = 0;
	psAggregateStats->ui64GpuStatActive  = 0;
	psAggregateStats->ui64GpuStatBlocked = 0;
	psAggregateStats->ui64TimeStamp      = 0;

	/* Not used */
	psAggregateStats->bValid = IMG_FALSE;
	psAggregateStats->ui64GpuStatCumulative = 0;

	*phGpuUtilUser = psAggregateStats;

	return PVRSRV_OK;
}

PVRSRV_ERROR SORgxGpuUtilStatsUnregister(IMG_HANDLE hGpuUtilUser)
{
	RGXFWIF_GPU_UTIL_STATS *psAggregateStats;

	if (hGpuUtilUser == NULL)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psAggregateStats = hGpuUtilUser;
	OSFreeMemNoStats(psAggregateStats);

	return PVRSRV_OK;
}

/*
	RGX MISR Handler
*/
static void RGX_MISRHandler_Main (void *pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = pvData;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/* Give the HWPerf service a chance to transfer some data from the FW
	 * buffer to the host driver transport layer buffer.
	 */
	RGXHWPerfDataStoreCB(psDeviceNode);

#if defined(PVRSRV_SYNC_CHECKPOINT_CCB)
	/* Process the signalled checkpoints in the checkpoint CCB, before
	 * handling all other notifiers. */
	RGXCheckCheckpointCCB(psDeviceNode);
#endif /* defined(PVRSRV_SYNC_CHECKPOINT_CCB) */

	/* Inform other services devices that we have finished an operation */
	PVRSRVCheckStatus(psDeviceNode);

#if defined(SUPPORT_PDVFS) && defined(RGXFW_META_SUPPORT_2ND_THREAD)
	/*
	 * Firmware CCB only exists for primary FW thread. Only requirement for
	 * non primary FW thread(s) to communicate with host driver is in the case
	 * of PDVFS running on non primary FW thread.
	 * This requirement is directly handled by the below
	 */
	RGXPDVFSCheckCoreClkRateChange(psDeviceNode->pvDevice);
#endif

	/* Process the Firmware CCB for pending commands */
	RGXCheckFirmwareCCB(psDeviceNode->pvDevice);

	/* Calibrate the GPU frequency and recorrelate Host and GPU timers (done every few seconds) */
	RGXTimeCorrRestartPeriodic(psDeviceNode);

#if defined(SUPPORT_WORKLOAD_ESTIMATION)
	/* Process Workload Estimation Specific commands from the FW */
	WorkEstCheckFirmwareCCB(psDeviceNode->pvDevice);
#endif

	if (psDevInfo->pvAPMISRData == NULL)
	{
		RGX_MISR_ProcessKCCBDeferredList(psDeviceNode);
	}
}
#endif /* !defined(NO_HARDWARE) */

#if defined(SUPPORT_GPUVIRT_VALIDATION)

PVRSRV_ERROR PVRSRVGPUVIRTPopulateLMASubArenasKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
                                                 IMG_UINT32          aui32OSidMin[GPUVIRT_VALIDATION_NUM_REGIONS][GPUVIRT_VALIDATION_NUM_OS],
                                                 IMG_UINT32          aui32OSidMax[GPUVIRT_VALIDATION_NUM_REGIONS][GPUVIRT_VALIDATION_NUM_OS],
                                                 IMG_BOOL            bEnableTrustedDeviceAceConfig)
{
	IMG_UINT32 ui32OS, ui32Region;

	for (ui32OS = 0; ui32OS < GPUVIRT_VALIDATION_NUM_OS; ui32OS++)
	{
		for (ui32Region = 0; ui32Region < GPUVIRT_VALIDATION_NUM_REGIONS; ui32Region++)
		{
			PVR_DPF((PVR_DBG_MESSAGE,
			         "OS=%u, Region=%u, Min=0x%x, Max=0x%x",
			         ui32OS,
			         ui32Region,
			         aui32OSidMin[ui32Region][ui32OS],
			         aui32OSidMax[ui32Region][ui32OS]));
		}
	}

	PopulateLMASubArenas(psDeviceNode, aui32OSidMin, aui32OSidMax);

	PVR_UNREFERENCED_PARAMETER(bEnableTrustedDeviceAceConfig);

	return PVRSRV_OK;
}
#endif /* defined(SUPPORT_GPUVIRT_VALIDATION) */

static PVRSRV_ERROR RGXSetPowerParams(PVRSRV_RGXDEV_INFO   *psDevInfo,
                                      PVRSRV_DEVICE_CONFIG *psDevConfig)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Save information used on power transitions for later
	 * (when RGXStart and RGXStop are executed)
	 */
	psDevInfo->sLayerParams.psDevInfo = psDevInfo;
	psDevInfo->sLayerParams.psDevConfig = psDevConfig;
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdumpFlags = PDUMP_FLAGS_CONTINUOUS;
#endif
	if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META) ||
	    RGX_IS_FEATURE_SUPPORTED(psDevInfo, RISCV_FW_PROCESSOR))
	{
		IMG_DEV_PHYADDR sKernelMMUCtxPCAddr;

		eError = MMU_AcquireBaseAddr(psDevInfo->psKernelMMUCtx,
		                             &sKernelMMUCtxPCAddr);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXSetPowerParams: Failed to acquire Kernel MMU Ctx page catalog"));
			return eError;
		}

		psDevInfo->sLayerParams.sPCAddr = sKernelMMUCtxPCAddr;
	}

#if defined(SUPPORT_TRUSTED_DEVICE) && !defined(NO_HARDWARE) && !defined(SUPPORT_SECURITY_VALIDATION)
	/* Send information used on power transitions to the trusted device as
	 * in this setup the driver cannot start/stop the GPU and perform resets
	 */
	if (psDevConfig->pfnTDSetPowerParams)
	{
		PVRSRV_TD_POWER_PARAMS sTDPowerParams;

		if (RGX_IS_FEATURE_VALUE_SUPPORTED(psDevInfo, META) ||
		    RGX_IS_FEATURE_SUPPORTED(psDevInfo, RISCV_FW_PROCESSOR))
		{
			sTDPowerParams.sPCAddr = psDevInfo->sLayerParams.sPCAddr;
		}

		eError = psDevConfig->pfnTDSetPowerParams(psDevConfig->hSysData,
												  &sTDPowerParams);
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXSetPowerParams: TDSetPowerParams not implemented!"));
		eError = PVRSRV_ERROR_NOT_IMPLEMENTED;
	}
#endif

	return eError;
}

/*
	RGXSystemGetFabricCoherency
*/
PVRSRV_ERROR RGXSystemGetFabricCoherency(IMG_CPU_PHYADDR sRegsCpuPBase,
										 IMG_UINT32 ui32RegsSize,
										 PVRSRV_DEVICE_FABRIC_TYPE *peDevFabricType,
										 PVRSRV_DEVICE_SNOOP_MODE *peCacheSnoopingMode)
{
	IMG_CHAR *aszLabels[] = {"none", "acelite", "fullace", "unknown"};
	PVRSRV_DEVICE_SNOOP_MODE eAppHintCacheSnoopingMode;
	PVRSRV_DEVICE_SNOOP_MODE eDeviceCacheSnoopingMode;
	IMG_UINT32 ui32AppHintFabricCoherency;
	IMG_UINT32 ui32DeviceFabricCoherency;
	void *pvAppHintState = NULL;
	IMG_UINT32 ui32AppHintDefault;
#if !defined(NO_HARDWARE)
	void *pvRegsBaseKM;
#endif

	if (!sRegsCpuPBase.uiAddr || !ui32RegsSize)
	{
		PVR_DPF((PVR_DBG_ERROR,
		         "RGXSystemGetFabricCoherency: Invalid RGX register base/size parameters"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

#if !defined(NO_HARDWARE)
	pvRegsBaseKM = OSMapPhysToLin(sRegsCpuPBase, ui32RegsSize, PVRSRV_MEMALLOCFLAG_CPU_UNCACHED);
	if (! pvRegsBaseKM)
	{
		PVR_DPF((PVR_DBG_ERROR,
		         "RGXSystemGetFabricCoherency: Failed to create RGX register mapping"));
		return PVRSRV_ERROR_BAD_MAPPING;
	}

	/* AXI support within the SoC, bitfield COHERENCY_SUPPORT [1 .. 0]
		value NO_COHERENCY        0x0 {SoC does not support any form of Coherency}
		value ACE_LITE_COHERENCY  0x1 {SoC supports ACE-Lite or I/O Coherency}
		value FULL_ACE_COHERENCY  0x2 {SoC supports full ACE or 2-Way Coherency} */
	ui32DeviceFabricCoherency = OSReadHWReg32(pvRegsBaseKM, RGX_CR_SOC_AXI);
	PVR_LOG(("AXI fabric coherency (RGX_CR_SOC_AXI): 0x%x", ui32DeviceFabricCoherency));
#if defined(DEBUG)
	if (ui32DeviceFabricCoherency & ~((IMG_UINT32)RGX_CR_SOC_AXI_MASKFULL))
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid RGX_CR_SOC_AXI value.", __func__));
		return PVRSRV_ERROR_INVALID_DEVICE;
	}
#endif
	ui32DeviceFabricCoherency &= ~((IMG_UINT32)RGX_CR_SOC_AXI_COHERENCY_SUPPORT_CLRMSK);
	ui32DeviceFabricCoherency >>= RGX_CR_SOC_AXI_COHERENCY_SUPPORT_SHIFT;

	/* UnMap Regs */
	OSUnMapPhysToLin(pvRegsBaseKM, ui32RegsSize, PVRSRV_MEMALLOCFLAG_CPU_UNCACHED);

	switch (ui32DeviceFabricCoherency)
	{
	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_FULL_ACE_COHERENCY:
		eDeviceCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CROSS;
		*peDevFabricType = PVRSRV_DEVICE_FABRIC_FULLACE;
		break;

	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_ACE_LITE_COHERENCY:
		eDeviceCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CPU_ONLY;
		*peDevFabricType = PVRSRV_DEVICE_FABRIC_ACELITE;
		break;

	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_NO_COHERENCY:
	default:
		eDeviceCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_NONE;
		*peDevFabricType = PVRSRV_DEVICE_FABRIC_NONE;
		break;
	}
#else
#if defined(RGX_FEATURE_GPU_CPU_COHERENCY)
	*peDevFabricType = PVRSRV_DEVICE_FABRIC_FULLACE;
	eDeviceCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CROSS;
	ui32DeviceFabricCoherency = RGX_CR_SOC_AXI_COHERENCY_SUPPORT_FULL_ACE_COHERENCY;
#else
	*peDevFabricType = PVRSRV_DEVICE_FABRIC_ACELITE;
	eDeviceCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CPU_ONLY;
	ui32DeviceFabricCoherency = RGX_CR_SOC_AXI_COHERENCY_SUPPORT_ACE_LITE_COHERENCY;
#endif
#endif

	OSCreateKMAppHintState(&pvAppHintState);
	ui32AppHintDefault = RGX_CR_SOC_AXI_COHERENCY_SUPPORT_FULL_ACE_COHERENCY;
	OSGetKMAppHintUINT32(pvAppHintState, FabricCoherencyOverride,
						 &ui32AppHintDefault, &ui32AppHintFabricCoherency);
	OSFreeKMAppHintState(pvAppHintState);

#if defined(SUPPORT_SECURITY_VALIDATION)
	/* Temporarily disable coherency */
	ui32AppHintFabricCoherency = RGX_CR_SOC_AXI_COHERENCY_SUPPORT_NO_COHERENCY;
#endif

	/* Suppress invalid AppHint value */
	switch (ui32AppHintFabricCoherency)
	{
	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_NO_COHERENCY:
		eAppHintCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_NONE;
		break;

	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_ACE_LITE_COHERENCY:
		eAppHintCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CPU_ONLY;
		break;

	case RGX_CR_SOC_AXI_COHERENCY_SUPPORT_FULL_ACE_COHERENCY:
		eAppHintCacheSnoopingMode = PVRSRV_DEVICE_SNOOP_CROSS;
		break;

	default:
		PVR_DPF((PVR_DBG_ERROR,
				"Invalid FabricCoherencyOverride AppHint %d, ignoringiLnoringiLnoringiLnoringiLnoringiLnoringiLnoringiLnoringiLO
OqeRV_DEVICE_SNOOP_CROSS;
		break;

	default:
		PVi:yP:Vay6yP:Vx);

	PVR_UNREFER6w K6wSO,K6h;
		break;

	case ult:
		PVi:yP:Vay6yP:Veqe-;T_ACE_LIT6yITE_COHERENCY:
		e6yP:Veqe-;T_ACE_LIT6yI;

	PVode = PVRSRV_DEVICE_SNOOP_CPU_ONLY;
		*peDevFabricType = on non pe>6yP:yP:yStieDevFabricTy6yP:yP:ySNOOP_CPU_ONLY;
		*peDevFabricType = PVRSRV_DEV:yP:bn suckCoreClkRateChange(psDeviceNode->pvDevice);
#endif

	/* PedoDevFabricType = PVRSRVL PVRSRVL PVRSRVL PVRSRVL PVRSRVL PVRSRVL PVRSRVL PVRSRVLO>6w-LnoringiLnVL PVRSRVL PVRSRVL PVROqe>6yP:wLay6yP:Vx);

	PVR_UNREFER6w K6wSO,K6h;
		break;

	case ult:Meqe>6vKi:yP:hKi:byP:wLay6yP:Vx);

sVICE_vgoherei:g*****************************************L**************************L************************* LeK6h*/
	WorkEstChee>6yP:yP:yKi:yP:VeturnSte>6V>bValide>6worin*******L************************* Lon obtaining a copy
of this softOqe>6yP:yP:yP:yP:yP:v{FabricTydCY  0x2 {SoC s}G_UINT32 ui32State;
	/* snapshot fruptProcesseProcess ther cannot stbER6w K6wSO,K6h;
	
	psDevInfo->sLayerParams.ui32Pdrin*******L*******_UNREFlkRas******g a copyPoweoftOqe>6yP:yP:yP:yP:yP:v{FabricTydCY  0x2 {SoC s}G_UINT32e |= RGX_LIS use. */
	poringiLnVLr cMMUAt       s>6yP:wLay6yd)
	
	caseATTRIBS *nVLr cMMUAt       sown request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvAPMISRDDDDDDDDrruptsTimeevInfo,puUtiCtxrParad)
	
	caseATTRIBS *psd)
efeAt  sV_DEVICE_NFO *pevInfo,puUtiCtxs deve copRate*peDroguethis sver tranRegion = 0; ui32Region < evInfo,puUtiCtxr, KCCB is sseProcess thfnvDevi

	/* PCE_urite", "fpsd)
efeAt  sV_DcesseProcess thed)
efeAt  s;2e |= RGX_LIShed)
efeAt  s;2vRegsBaseKM, RGXe "rgxstartstop.h"
#endif

#include "rgx_fwif_ngiLnVLXI_SecuriPDump,puspace>6yP:wLay6y
				/* Re-scheduleSecuriPDump,puspaceown request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvAPMISRDDDDDDDDX_CR_SOC_AXI_MASKFUS_TS uiFlag/
				OSScheduleMISR(psDevInfo->pvAPMISRDDDDDDDDrrupTDSetPpsz,puspaceNam*/
				OSScheduleMISR(psDevInfo->pvAPMISRDDDDDDDDrrup***********,puspaceNam*Len[ui32Region][ui32OS]));
		}
	}

	PopulatranRegion = 0; ui32Region <RV_OK)
	{
		PVR_DPF((
	}

	Pthe Hf

#inEned(Ct w( uiFlagseError = 
	}

	Pthe Hf

#inEned(RSRV( uiFlagse dList(PVRSNP	PVtf(psz,puspaceNam*/
DevInfo;
	psD****,puspaceNam*Len/
DevInfo;
	psDPMROC_ASPCR_Sime sin/
DevInfo;
	psD"TDFWC_A")break;

	defaul
	}

	Pthe Hf

#inEnE_16KB( uiFlagsdList(PVRSNP	PVtf(psz,puspaceNam*/
DevInfo;
	psD****,puspaceNam*Len/
DevInfo;
	psDPMROC_ASPCR_Sime sin/
DevInfo;
	psD"TD

#E_1C_A")break;

	dvoid *pvRegsBaseKM;
#endif

	if (!sRegsCpu****ter
aer iInfodif

ecutionf
    0x%"DX_CR_SOC_AXI_MASKFUS_ime sin/
DevInfo;
	p4GpuStatCumuiFlagsdS_FEATURE_VALUE_SUPPORTED(2-Way Co = RGXA;2e |= RGX_LISTURE_SUPPO_UINT32 ngi *onString_OKPart2
 R))
		{
			sTDPowerring_OKPart2 UPPORT_WORKLOAD_ESTIMATION)
	/* ProU and perrrup******OOP_NON

	/* PFlag/
U and perrrup******OOP_NON;
	OSFad(sBufT32)KB/
U and perrrup******OOP_NON;
	OSFad(sFilt
/* U and perE_SNers[uiPM     SOC_CBDefeManno/
U and perrrup******OOP_NONAck: %lluPowUingseadMaui32Region][ui32	FEATURE_tilStatsUnregister(IM		
#define GPU_ACTIVE     RGXFWIF_GPU_UTPPORT_WORK		}

		of anorin RGX_s Do not PCE_LITE_COHER		}

		of anrenGXSystemGetFabric      	*ATION) */

static PProcess ther cannot E_FABRIC_TYPEOOP_NONAllPowUingseadMFF)
+1)

ation used o
	deCE_uriCfg*psDeMAXPowUingad */
 - 1	ui32DeviceFabrTIM>6wseErY        0x0 {SoCOS/* NVL PAc>pvDTo	OSF
		     En(are CCB for Xe "rCOM, ME(
	IMGHANDLE hG      Part 2"thread. HANDLE hGpuc PPro ,
    iceAceConfig);
_NON

	/* PFlagce this{
		PV

	/* PFlagARDWARE) && _NON

	/* PFlag/cy value
thread. Aif

ecpucd(PVTCE_LI(needACE_Lbfodif

echaveV_INFOcommu24)
#ve, 1 != PVCoreponices deiANDLE hGpaveVcapRavInfo-> deaY  RT_2ce);
betwe 0)
		m) iceAceConfig);
psGpucd(PTCE_LIle data."Z,purn PVRSR*(ceConfig);
psGpucd(PTCE_L)ILABLE;
	}
Config);
psGpucd(PTCE_LIlFirmwareCCB(ppvRegsBaseKM;
#endif

	if (!sRegsCpuystemGwerring_OKPart2KM:ain.
		INUOUif

ecpugpu de " tCE_LIstorage     "RGXSystemGetFabricCohRV_OK;
}

PVRSRV_ERRd. HANDLE hGpu;
	OSFad(sats = hGcquire Keser;
	OSFad(stic PRDWARE) && _NON;
	OSFad(sBufT32)KB)IlFiROR,
				 "%s: FE;
	}
Config);
_NON;
	OSFad(sFilt
/ide64(pssDevInser;
	OSFad(s	PVEe, 1Filt
/PRDWARE) && _NON;
	OSFad(sFilt
/) to the t/AggreAggrega   g_sdOUif

ecpu *pss souobtaiurn stattCCB(ht nots = hGcquirFE;
	}
Config);
_NON;
	OSFad(sFilt
/i,
		         e Keser;
	OSFad(stic OnDeClkRR souobtata->ePowStaefined(PDUMP)
	psePowState == RGXFWIF_POW_IDLE)
	 ";
	OSFad(sats = h    deClkRed stats are s"eiANDLE hG      in.
		FWIF_GPU			{
				P;

	dvoid *pvRegsBaseKM;
#en_IDLE)
	 ";
	OSFad(sats = h iANDLE hG      in.
		FWIF_GPESSOR))
	{
		IMG_DEV_PHuirement is directly handHANDLE hGpu.h"
 e */
	RGXPD chec list is beinERATIOVICE_F
}

static vohNode->p psUtil*pvReASSERT;
#if delFiROR,
				 lt = RGX_CR_SOHANDLE hGpul checof ZSBs = hsc list is beinERATIOVICE_F
}

static vohATIOZSBs = htil*pvReASSERT;
#if delFiROR,
				 lt	rruptProic P}

static voiZSBs = hferre;eAceConfig);
_NONZSBs = hCurr~RGX_1hread. HANDLE hGpul checof growCE_LIe = l chec list is beinERATIOVICE_F
}

static vohATIOe =  Caltil*pvReASSERT;
#if delFiROR,
				 lt	rruptProic P}

static voie =  Calferre;eAceConfig);
_NONe = l chCurr~RGX_1hreat is beinERATIOVICE_F
}

static vohre coFRGX_tic  psUtilFWPF((PVR_DBG_ERROR,
				 "%s: FTED(psDevInfo, MORT_FULLuletic _HEAre coFFlagKM(gxs	eErrorGX_CR_SO(21)
FAULTAR, ui3ENis cD "%s:_FEATURE_VALERATIOVICE_F
}

static voh, RISCRY;
	 psUtilFWCb->u
#if defined(PDUMP)
	psDevInTED(psDevInfo, METAV_ERRd. S		{
	 (1) Initialise tats mtCall =G_WAbahec list is beinERATIOVICE_F
}

static vohWord  = psUtil*pvReASSERT;
#if delFiROR,
				 lt cCoherency;
	IMG_UINT32 ui3
Config);
pfn
		/* We can ent=chedule the power dlt = RGX_CRrin RGX_s Do not PCE_LITE_COHER		}

		of anrenGXS3
Config);
C_CBDefeMannoOqeRVCBDefeMannohread. k memequiremeS(1)vMemoa     NDLE hzhGpuUnumb h  feS(1ACE_L      {
	quire Ke(_NONAck: %lluPowUingseadMxs	_NONAllPowUingseadM)ide64(psRetupvRegsBaseKM;
#endif

	if (!sRegsCpu***OHERENCYS(1)vMemo(AllN_NUuestse FpRatN_NUu). At-ui64Gpith S(1)vustCE_Lbfopata;
	}upVRSRVGvInfo;
	p4GpuStatCRVGvInfo;
	p_NONAllPowUingseadMCRVGvInfo;
	p_NONAck: %lluPowUingseadMaSoC supports full ACE or 2-Way CoS(1HERENeStats->uConfig);
_NONAck: %lluPowUingseadMx=p_NONAck: %lluPowUingseadMxs	_NONAllPowUingseadM;2 ui32State;
	/* snapshot fruSR tet-{
			PVTIL_STAre penMgmt=G_WAbahec lisCB(psDevRSRV_OK)regiPVR_DP(sDevRSRV*)atic PProcess ther cannot vohrexGpuUtilPowState != ysGPUVIRAPMui64TmegiPVRL_STATETimingXFWIfFwGPUVIRACBDefeMtilPowState !=GPUVIRAPMui6((C_CBDefeMannoOq=rE_SNers[uiPM DEFAULTgxsta= ysGPUVIRAPM64LastStastats (C_CBDefeMannoOq=rE_SNers[uiPM FOurnetlyilFWCb->u=GPUVIRAPMu;
	P!Pata != NULL)
		{
NAs[ui))yP:yKi:yP:VeturnSte>6V>bValide>6wohe sharIL_STAre penManHEAvicesT_ACE_L     virVRSRizalise messRENCY  0x2 {SoC s	=GPUVIRAPMui6fa
	d to the	break;

	case RIDATIOfined(NO_Hta;
	Pse RIDATIOfined(NO_H > 1gxstartstop.h"
#endifAUTOVZ)
de = psDeauto-vzVR_DPF((   g_sLa virVRSRiialise watchdogegsBaorepalig_sLype =APMuats->ui64ASSERT;=GPUVIRAPMui=sDeviceNode);	 psDevInfb->u=GPUVIRAPMyP:yKi:yPt is beinERIntatllf (eE}

static vos */
	RGXChe/
U andto send deferred KCCB commands Do not P/
U andATION)
	/* ProU and"E_SNOKCCB commands Do "SoC s	b->u
#if defined(PDUMP)
	psePowStaTED(psDevInfo, ME the thandleve, 10		PVR PPro CoreClwokettemveV_INFOInfo-> de= NUize gCE_LdoE_DENIEDrin RGX_s Do not PCE_LITE_COHER		}

		of anre
		gMETAV_E CCB for XITE_CDeviceNGPU statdeferres_SUPPORAPPH */
	t iPUVIRAPMRGXFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSSSE_SQueryAPMnot P/
UFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSSSE_SS		APMnot P/
UFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSSSATION)
	/* ProUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSSSviceNod case
	 * of HANDV_DEVICE_WAbaheuffer.
	 */
	RGXHW      GPU stat0		PVR PPro NE IMG_BO      manHEArat leN    l/H= IMG D_DPF(***_     "RG      manHEAvicet leG; /* D_DPF(***  whiles (%" Ily6yP:yP:yP      manHEAvicet >hLockKCCBDeferredCGPU stats Do r.
	 *(ATION)
	/* ProU andastats &tionrds Do not P/ &tiond(ss Do not P/
U andastats vice as
	 * in tnrds Do not P/ vice as
	 * in tnd(ss Do not P/
U andastats &tionrds couSpeedus(psD/ &tiond(ss couSpeedus(psD/
U andastats &tioF obtdIMG_uld be / &tioCf (hlF obtdIMG_uld be /
U andastats &tionowUingsnot PeadMus(psD/
U andastats sAggregateS)ATION)
	/* ProU andastats LITE_COHER		}

		of anre
	roU andastats rin RGX_s Do not P
	if (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSuystemGwerring_OKPart2KM:ain.
		INUOBG_ERROR,R PPro NE IM      manHEAr     "RGXSysteevInfo, MORT_t is being prosetup the driver cE) && ui32OSidMax[	if (dllist_is_empty(&psDev GXSysteevInfo, ****************L*****************fo->ze,
										 PVRSRV_DEVICE_
32IRQThreadMask =evFabricType,
		t(PVRVICE_FABRIC_FULLACE;
	eDeviceCacheSnoopiingMode = PVRSRV_DEVICpty(&psAPPH */
TESTSL. */E_CDCE_FC_AXI_COHERENCY_SUPPORT_FULL_ACE_COHERTbe SL.     EREpsDevInfo;
	psDevInfo->sLPVRSRV_DEVICE_FABRIC_A>uConfig);
_NONTbe SL.     EREoopiiNOOP_CROSS_AXI_COHERENCY_SUPPORTbe SL.     EREoask =evFabricType,
=PVRS>uConfig);
_NONTbe SL.     ERE=PVRSRVGvInfo;
	VRSRV_DEVICE_FABRIC_>uConfig);
_NONTbe SL.     EREoif (! pe = PVRSRV_DEVICE_SNOOP_CPU_ONLY;
	u	->uConfig);
_NONTbe SL.
		  ormation used _NONTbe SL.     ERE	u	->uConfig);
_NONSL.Skip coinform0) ||
		    RGROR RGXSetPowerParaROR RGXSetPo* snapshot fruXe "rCOM, MEWITHSKFUS(            DEINITohe
			iurn statB) *otic voi iMG_"thread. KipuStaatB) o (h,    _AXI_GXActf deneedACE_Ln updaoa   tetStaatiMG_G_ERRORer tr    REGPORE_SNr    REG_NAMUVIRT_VALE_SNOOPMTOfi	PVRULUVIRT_VALE_S    =DM_GPed(PVRSRV_DMTOfi	PVRULU=DM_("AXI VIRT_VAL                       |             DEINIT)hreat is beinConmemPDumpConmemPol32(psDevInfo->pESSAGE,
				 "%s MemDescroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSofftetRSRV_ERROR_ PVRSRV, ",
				ui3roUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSVICE_NODE	*psDevroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSS0xFFFFFFFFUroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSS      POLL_OP;
		or 	psDeroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSS                       |             DEINIT)hrf (dllist_is_empty(&psDev GXSysteevInfo,  = RGX_CR_SORunRAMETER(bNE IMG_BOry FWdaoPDumpnf
    E_Lfeed->ui64Tim-fE eDLn   NDats = h er tr    COM, MEWITHSKFUS(            DEINITohe	IMGn   NDLE hG      oreClkRa"threas(PVRSRV_RGXDEV_INFO   *psDevInfo,
    |=             DEINIT |             NOHWilFWPF((! Pata != NULL)
		{
			OSSchedulet is being prER(P}

static voiDEV_INFO   evConfig;
#if defined(PDUMP)
 GXSysteevInfo, tats->uConfig);
XDEV_INFO   *psDevInfo,
    adHWR            DEINIT |             NOHWe);	 psDevIui32State;
	/* snapshot frut is being pIntatllde->pvDQue bef (eE}

static vohde->pvDQue bef (ete == RGXFWIF_
	if (dllist_is_empty(&psDevInfo->tProcessed)
		{
#if defined(PVRSRV_DEBUG_LISgrega) (ui3intatllf (eE	g_sLISRExecutionInfo.ui32StateRGXSysteevInfo, MORT_PVRSRU stat0		PV|= RGX_LISheferresc list is beinERIntatllf (eE}

static vos 
	RGXChe/
U a*phGpuUtilUser = psA/
U aATION)
	/* ProU a"E_SN psA"
	if (dllist_is_empty(&psDevInfo->tProcessed)
		{
#if defined(PVRSRV_DEBUG_LISgrega) (ui3intatllf (eE	g_sLISRExecutionInfo.ui32StateRgrega) (ui3intatllf (eE	g_sLISRExehde->pvDQue bef (e   "RGXSysteevInfo, MORT_t is bein		 IntatllION)
	L (eE	g_sLannot start/stop the GPU as vice as
	 * i;
	ui32 the GPU as X_CR_SOCODNAMUVIRT_and perE_SNLuUtilUser  the GPU as vice )
	/* ProU andast  }

static vos LnInfo.ui32S (dllist_is_empty(&psDevInfo->tProcessed)
		{
#if defined(PVRSRV_DEBUG_LISgrega) (ui3intatllf (eE	g_sLISRExecutionInfo.ui32StateRgrega) (ui3intatllf (eE	g_sLISRExehde->pvDQue bef (e   "Rgrega) (ui3intatllf (eE	g_sLISRExecu
			return IMG_FALSEevInfo, MORT		    RGROR RGXSetPowerPararnStatne		INUOwrapMain (voidiurn S7_
		PV_HI;
	RCHY CoreClyP:yP:yL    ht ni *o	break;

	case RGX_CR_SOS7_
		PV_HI;
	RCHY)...		    ,_DEV &ui32RICE) && !defined(NO_Hi *omacrUOBGfbricTesed RGXvMemoX, NULLde_DPFdeqe-;ToryBarrieODE eDeviNING, "if dehileed chi *o   archiupdaus svING, "d while	 * b
of tfCE_uri.
 R))	break;

	case RGX_CR_SOS7_
		PV_HI;
	RCHY)FWPF((!dr = sKernelMMUCined(NO_HARDWARE) && S7_
		PV_HI;
	RCHY)Sch_UINT32efo->tPro!Pata !_UNREFVL PVRSROfCPUPVRSRE	g_sLannot d area *	!Pata !_UNREFVL PVRSROfRSRVL PVRSRE	g_sLannot dyP:yKi:yP:    COM, MEWITHSKFUS(                      ohe_UNREFl agaNO _ActatiL PVRSR"i32StateRf ((eEr     e KePata !_UNREFVL PVRSROfCPUPVRSRE	g_sLannot d
	psePowStat    COM, MEWITHSKFUS(                      ohe_UNREFl agaCPU _ActatiL PVRSR"i32St;

		ie KePata !_UNREFVL PVRSROfRSRVL PVRSRE	g_sLannot dyP:yePowStat    COM, MEWITHSKFUS(                      ohe_UNREFl agatFabri _ActatiL PVRSR"i32St;

		TAV_E CCB for ckKCCBDeferredCTQLoadShade_uffer.
	 */
	RGXHf (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid))
	{
		OSSpinLelow
TQ* Wede_uRENCY  0x2 {SoC supportsevInfo, tats->uConfig);
bConfiit2Dith ncy value
		 * is not well calibratAX, NULL NUse RGW_FILENAMUCinFFIX ".vz"tAX, NULLse RGW_FILENAMUCstWoSIZRVGe(rn PVRSRV_ERGW_FILENAMU)+ \
U a*phGBVNC_STRoSIZRCstW+n PVRSR NUse RGW_FILENAMUCinFFIX)Schative = 0;

	_ddr WF	OSNam*own request as it was deferred. */
		rrupTDSetPpsz WF	OSnaf thr/
		rrupTDSetPpsz WpF	OSnaf thrFWIF_GPU_UTIL_STATE_IDLE
#define GPU_ACTIVE     RGXFWIF_GPU_UTcontaDrrupTDSetP contaDpsz WF	OSnaf ts =ix =i:yP:Vea != NULL)
		{
NAs[ui) ? "" :L NUse RGW_FILENAMUCinFFIX;RV_DESNP	PVtf(psz WF	OSnaf thr/Lse RGW_FILENAMUCstWoSIZRroU a"%s." *phGBVNC_STRoime since
				"Inse RGW_FILENAMU/
U aATION used o
	deCE_uriCfg*psDeB,
ation used o
	deCE_uriCfg*psDeV/
U aATION used o
	deCE_uriCfg*psDeN,
ation used o
	deCE_uriCfg*psDeC/
U aATz WF	OSnaf ts =ix);RV_DESNP	PVtf(psz WpF	OSnaf thr/Lse RGW_FILENAMUCstWoSIZRroU a"%s." *phGBVNC_STRPoime since
				"Inse RGW_FILENAMU/
U aATION used o
	deCE_uriCfg*psDeB,
ation used o
	deCE_uriCfg*psDeV/
U aATION used o
	deCE_uriCfg*psDeN,
ation used o
	deCE_uriCfg*psDeC/
U aATz WF	OSnaf ts =ix);RratcontaDze,
		 *phLoadAndddr Wretuown request as it was deferred. */
		OSRGW_IMAGwas*ppESSAGErParams: TDSetasz WF	OSnaf thr[se RGW_FILENAMUCstWoSIZR];rams: TDSetasz WpF	OSnaf thr[se RGW_FILENAMUCstWoSIZR];rams: TDSetPpszLoadedFwthr;

	/* Frepanfo->hGimEATUf	OSnaf   E_LpRave paramfollowND_THRdORer t_ddr WF	OSNam*oVIRT_VALIDATIONsz WF	OSnaf thr/Lasz WpF	OSnaf thrthread. te tV_INFOR_INV(RGXFW_MEimEATUiceAcezLoadedFwthrPU_Nsz WF	OSnaf thr;PVRSpESSAGEeinERATad(RGXFW_MoVIRT_VALIDATIOcezLoadedFwthr, OSRGW_VERIFY_FUNC*****XHf (dlRSpESSAGEeiFirmwareCCB(pcezLoadedFwthrPU_Nsz WpF	OSnaf thr;PVVRSpESSAGEeinERATad(RGXFW_MoVIRT_VALIDATIOcezLoadedFwthr, OSRGW_VERIFY_FUNC*****XHff (dlRSpESSAGEeiFirmwareCePowStcezLoadedFwthrPU_se RGW_FILENAMU32St;RSpESSAGEeinERATad(RGXFW_MoVIRT_VALIDATIOcezLoadedFwthr, OSRGW_VERIFY_FUNC*****XHfff (dlRSpESSAGEeiFirmwareCeePowState == RGXFWIF_POWFATALid)A de	IMG(RGXFW_MEimEATUelowgain.
		iurn '%s'SysData-	Nsz WF	OSnaf thri64MinRe is notEVICE_F
			{
				PViNOOP_CROSS	IMG(RGXFW_MEimEATU'%s'Uelowed"IOcezLoadedFwthr)NS; ui32RegiOS(RGXFW_Mretuo*ppESSAGEr;2vRegsBaseKM, RGXe "rg)
		{
			sTDPowerring;
	OSF
		     	break;

	case ult:Meqe>6vKi:yP:hKi:bIdle    =evic_CMD sKccbCmd;32Region][ui32OS]));
hread. Fiui64GpG_BOryeClkR strudaus  NE IMG_BO DE eDevictne		ateChansKccbCmd.eCmd:
		eDedle    =evic_CMD_;
	ERFc      3ENis c_BLKS_DIRECT;RT_t is being prondCyeClkRWE IPow psU

	/* Process the FirmwroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSS&sKccbCmdroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSS                      NS; ui32RegiTURE_SUPPO_UINT32 
		{
			sTDPowerringVICE_FFWevInfo,puUtiCo   xt	break;

	case ult:Meqe>6vKi:yP:hKi:bISR tettemvfwtGpuUtilco   xtsver trani32OS,
			         ui32RegioPU_ACTIVE     RGXFWIF_GPU_UTPPORT_WORK		         aui32OSidMaxstatic PProcess ther cannot E_FRegion][ui32OS]));
hre***************L****AUTOVZ)
dd)
	PWoSETUP sin RGX_sxS		{
	tatic PProcess tic Pd)
sxS		{
VR_DPF((
	}

	P NULL)
		{
HOOSScheduleOHERENCY_SUPPORswapMain (RGXPx NUizodstats se RGX_ LMAOBG_E   of	 (1)physheapMao
urn sUif
wMG_BO DATUtCE_Lecof  *psGpuUtil
		 have finisFwif
	if co   xtCE_Lbfopor =d
urn s4GpaY  di
echavGpuUtilcarveout.i32Attempts++)
f
wMG_BOfRGXFW_ME
		    sMao
urn spO,K6(s) == MAaead(s)kf
	if crashetFaR_DPF((led t.ice a 	tic PProcess tic Pd)
sxS		{
PU_se RGW_d)
	RESERVED
}

oSETUPibrate the GPU freq = RGX_CR_SOCe10		PVR PPro ERENCY:
		e6yP:VeeV_INFOGEeco   xtC_DPF(driver tt is being prarams.sPCAddr = psDevInfvice as
	 * isPCAddr;
		}

		eError = pvice as
	 * i;
	uTDSetPowerParams(psDe&tic PProcess t			  &sTDPowerParams);
	}
	&vice as
	 * iERROR, "RGXSetPower	if (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{
		OSSpg prarams.sPCAddr = psDevIn (%u)RSRVGvInfo;
	p4GpuStatCRVGvInfo;
	plist_ii64MingoE_Lf		OSS_to__DPF((_ctx, MORT_PVRSRU stat0G_WAbaheaiurn _DPF(drivofVR PPro GpuUtilco   xtsver ts sseProcess thfnSRU stat,puUtiCo   xteing pSRU stat,puUtiCo   xt; ts sseProcess thfnRY;
	}

	p,puUtiCo   xteing pRY;
	}

	p,puUtiCo   xthread. ODPF((PCountpuUtilco   xtiurn statfRGXFW_M.ver tt is beinConmemODPF((Co   xt	VIRT_VALIDATIOORK}

oHEAPCFG(PVRSroU andast }

static vosDevInfoConmemOtxr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{
		OSSpConmemODPF((Co   xt (%u)RSRVGvInfo;
	p4GpuStatCRVGvInfo;
	plist_ii64MingoE_Lf		OSS_to__DPF((_ctx, MORT_t is beinConmemFindHeapByNam*oVIRT_tic vosDevInfoConmemOtx/Lse RGIRMhot CstINoHEAP_IDENTroU and  }

static voss(RGXFW_M psAHeapr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{
		OSSpConmemFindHeapByNam* (%u)RSRVGvInfo;
	p4GpuStatCRVGvInfo;
	plist_ii64MingoE_Lf		OSS_to_***d_heap, MORT_t is beinConmemFindHeapByNam*oVIRT_tic vosDevInfoConmemOtx/Lse RGIRMhot C      3HEAP_IDENTroU and  }

static voss(RGXFW_Mas
	 *Heapr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{
		OSSpConmemFindHeapByNam* (%u)RSRVGvInfo;
	p4GpuStatCRVGvInfo;
	plist_ii64MingoE_Lf		OSS_to_***d_heap, MORT	break;

	case RIDATIOfined(NO_Hta;
	Pse RIDATIOfined(NO_H > 1g_DPF((
	}

	P NULL)
		{
HOOSSchedule                 IA) |  IMG_UINT32~RGX_se RGIRST_RAW3HEAP_  IA)       IA < se RIDATIOfined(NO_H)       IAMG_UINT32  ms: TDSetszHeapNam*[X_CR_SOCtWoRA_NAMU_LENGTH]      DESNP	PVtf(szHeapNam*, n PVRSRszHeapNam*)/Lse RGIRMhot C			OS_RAW3HEAP_IDENTr       IA64MinRt is beinConmemFindHeapByNam*oVIRT_tic vosDevInfoConmemOtx/LszHeapNam*,arams);
	}
	 }

static vossG; /*(RGXFW_MRawHeap[      IA]64MinRNOOP_CR_GOTO_  =andsLllist_iid)ConmemFindHeapByNam*"IOf		OSS_to_***d_heap;

	for (u		    RGROR RGXSetPose R NUof aIC_CARVERV_OFW3HEAPSg_DPF((
	}

	P NULL)
		{
HOOSSchedule    er
	 * (when viceHeapBa	d to                  IA) ulet is beinviceHeap32OS, GPV

	Pnfo->sLayeProcess tap viceHeap[PPORT_WORK		   * S3HEAP_FW3LOCAL], ivid viceHeapBa	doopiiNOOP_CR_GOTO_  =andsLllist_iid)viceHeap32OS, GPV

	Pnfo-"IOf		OSS_to_***d_heap;

 |  IMG_UINT32~RGX_se RGIRST_RAW3HEAP_  IA)       IA < se RIDATIOfined(NO_H)       IAMG_UINT32  ms: er
	 * (when RawFwHeapBa	dnot  viceHeapBa	dbels[] =+G_UINT32~RG*Lse RGIRMhot CRAW3HEAP_SIZR)}      t is being pFwRawHeapata.")ap(ATION)
	/* ProU andasta	        IAroU andasta	   RawFwHeapBa	droU andasta	  se RGIRMhot CRAW3HEAP_SIZR)oC s	b->u
#if defined(PDUMP)
	psePowSta IMG_)       IA >_se RGIRST_RAW3HEAP_  IA)       IA--
	pseePowSta	g pFwRawHeapUn
		e = 	VIRT_VALIDATIO      IA64MinR;

		iiNOOP_CR_GOTO_  =andsLllist_iid)g pFwRawHeapata.")ap"IOf		OSS_to_***d_heap;

	f
			{
		***************L****AUTOVZ)
dad. ledtis soe RGX_ Px 
		{
	DENIEtic PProcess tic Pd)
sxS		{
PU_sin RGX_sxS		{
;G_UINT32e |COHERENPF((
	}

	P NULL)
		{
			OSSchedulet is beinPvzClient)ap

	PiceHeapocesseProcess ther cannot oopiiNOOP_CR_GOTO_  =andsLllist_iid)vvzClient)ap

	PiceHeap"IOf		OSS_to_***d_heap;

	fConmemHeaprosere
		yer buffer.
tic voss(RGXFW_M psAHeap/cy value
thr	fConmemHeaprosere
		yer buffer.
tic voss(RGXFW_Mas
	 *Heap/cy value
thr			PVR_DPF((PVR_DBG_Mse R NUof aIC_CARVERV_OFW3HEAPSgice a GXSysteevInfo, *f		OSS_to_***d_heap:CB)
	/* PClearPCountpueco   xtC_DPF(e0G_WAbaheaieV_INFOdedtroy->ui64Gp	IMGfRGXFW_M	/* Pco   xtCE_Laze,
	a spurious0G_WAbahe.DeviceAceConProcess thfnSRU stat,puUtiCo   xteinEVICE_Fs sseProcess thfnRY;
	}

	p,puUtiCo   xteinEVICE_FConmemDedtroyCo   xt	VIRT_tic vosDevInfoConmemOtxr, fVIRT_tic vosDevInfoConmemOtxeinEVICE_f		OSS_to__DPF((_ctx:GX_IS_FEATURE_SUPPOR0;

	*phDering_OdtroyFWevInfo,puUtiCo   xt	break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTIL_STATE_IDLE
#define GPU_ACTIVE     RGXFWIF_GPU_UTRegion][ui32Ofo;
	plist_i;e***************L****AUTOVZ)
dd)
	PWoSETUP sin RGX_sxS		{
	tatic PProcess tic Pd)
sxS		{
VR		    RGROR RGXSetPose R NUof aIC_CARVERV_OFW3HEAPSg_DPF((
	}

	P NULL)
		{
HOOSSchedule                 IA) u***************L****AUTOVZ)
datic PProcess tic Pd)
sxS		{
PU_se RGW_d)
	RESERVED
}

oSETUPibrate the GPU f	 psDevInf IMG_UINT32~RGX_se RGIRST_RAW3HEAP_  IA)       IA < se RIDATIOfined(NO_H)       IAMG_UINT32  g pFwRawHeapUn
		e = 	VIRT_VALIDATIO      IA64Min}2e |COHERENPF((
	}

	P NULL)
		{
			OSSchedulegrega) vvzClientUn
		

	PiceHeapocesseProcess ther cannot oopo->tProcessed)
		{
s(RGXFW_M psAHeaprUINT32  ConmemHeaprosere
		yer buffer.
tic voss(RGXFW_M psAHeap/cy vaceNode);f
			{tProcessed)
		{
s(RGXFW_Mas
	 *HeaprUINT32  ConmemHeaprosere
		yer buffer.
tic voss(RGXFW_Mas
	 *Heap/cy vaceNode);f
			eq = RGX_CR_S	/* PClearPCountpueco   xtC_DPF(e0G_WAbaheaieV_INFOdedtroy->ui64Gp	IMGfRGXFW_M	/* Pco   xtCE_Laze,
	a spurious0G_WAbahe.DeviceAceConProcess thfnSRU stat,puUtiCo   xteinEVICE_Fs sseProcess thfnRY;
	}

	p,puUtiCo   xteinEVICE_BLE;
	}
Config);
psevInfoConmemOtxrhedulet is beinConmemDedtroyCo   xt	VIRT_tic vosDevInfoConmemOtxr, f*pvReASSERT;
#if delFiROR,
				 lt	
		***************L****AUTOVZ)
dPF((
	}

	P NULL)
		{
HOOSScheduletic PProcess tic Pd)
sxS		{
PU_sin RGX_sxS		{
;G	eq = RGX_}gion=%u, Min=0x%x, Max=0xAlignviceOKCCB	break;

	case ult:Meqe>6vd. */
				OSScheduleMISR(psDevInfo->pvAPMISRIRQThreadMask =elignOKCCBetPower				OSScheduleMISR(psDevInfo->pvAPMISRIRQThreadMaask =elignOKCCBe[]Ki:bI:wLay6yP:VxhreadMaask =elignOKCCBeKMs not dle  _ALIGNPthe HS_INIT_KMR_NOT_IMPLEL_STATE_IDLE
#define GPU_ACTIV   RGXFWIF_GPU_UTP:VxhreadMai,E
#ask =FWelignOKCCBeE_FRegion][ui32OS]));
		}
	}

	PopulateLMAkipMain alignvice (voidiif			PVR_DPF((is g; /*oUFWIthe aggrfo-> denoOfRGXFW_MEPVR_voidiagaintaver trani32O NUsET_  =LL)

			OS,plist_iiE_BLE;
	}
Config);
psdle  elignOKCCBeMemDescIlFirmwareCCB(ppvRegsBaseKM;
#endif

	if (!sRegsCpu)
	{
W Alignvice OKCCB Mem Descriptis> dermwaRSRVGvInfo;
	p4GpuStataSoC supports full ACE or ALIGN, ME_ARRAY(psDe0. This could hat is beinConmem */
	psCpuVirtnfo->sLayerParams.dle  elignOKCCBeMemDescroUFWIF_SYSSSSSSSSSSSSSSSSSSSSSSSSSSSgregate*) &#ask =FWelignOKCCBer, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{
		OSSpNUOUS;
#endkf
	if afo-_COH*peDelignvice (voids (%u)RSRVGvInfo;
	p4GpuStatCRVGvInfo;
	plist_ii64MinupportsevInfo, tats->ask =FWelignOKCCBe += ARRAY(SIZR(ask =elignOKCCBeKM)=+G1XHf (dlRSask =FWelignOKCCBe++s_emsk =elignOKCCBetPowvInfo->sKCCBDeferredCommandsListHSSSSSSSSSu)
	{Mismatchs4Gpnumb h  festrudaus sEPVR_voidVRSRVGvInfo;
	p4GpuStatsdS_FEATURE_VALUE_SUPPORTED(2-Way CoALIGN, ME4MingoE_Lupport_, tats- IMG_iN_NUM_iCacheSnelignOKCCBetPowM_i+IONS][GP (dlask =elignOKCCBe[i]s_emSask =FWelignOKCCBe[i]
	psDevInfo->sLayerParams.ui32Pdrin*******L*******Cvoidiurn strudaus dDelignvice in.
		FWyP:v{FabricTydCY  0x2 {SoC sEATURE_VALUE_SUPPORTED(2-Way CoALIGN, ME4MinngoE_Lupport_, t
				PVupport_:
_FConmemReui64sCpuVirtnfo->sLayerParams.dle  elignOKCCBeMemDescNS; ui32RegiTURE_SUPPOion=%u, 
		{
			sTDPowerAif

ecpFW,puUti32OS, own request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvAPMISRDms: er
}

oSIZRCTcheSntPower				OSScheduleMISR(psDevInfo->pvAPMISRRIRQThreadMask,puAif

iFlag/
				OSScheduleMISR(psDevInfo->pvAPMISRDPeError;
	ed(PV         e        				OSScheduleMISR(psDevInfo->pvAPMISRDcontaDrrupPTDSetpszT xt  				OSScheduleMISR(psDevInfo->pvAPMISRDer
}

o}

DESCas*ppEMemDescPhrFWIF_GPU_UTI[ui32OS]));
		}
	}

	Popul	ms: er
}

oLOG2ALIGNPTcheLog=eligneinERGoseageShift(
	ui32DeviceFabricCohereDEDICAR sKed(PV ORY)FWPanRegion = 0; ui32Region <e      or (uie "rCOM, ME(
Aif

ecpu  di
echav
W %sntpuUti"IOcezT xt)hreat is beinConmemAif

ecpD di
echaFW,pu(ATION)
	/* ProU aheSntPowerU aheLog=elignerU ahe,puAif

iFlag/
wStcezT xt  wStcpEMemDescPhrFO_COHER F((PVR_DBG_MESSAGE,
DEDICAR sKed(PV ORY)U timers (done every few YADDR sKernelMM(uie "rCOM, ME(
ImyP:yPr iInfo
W %sntpuUti"IOcezT xt)hreat is beinConmemImyP:yTDFWCpu(ATION)
	/* ProUduleMISR(psDevInfo->pvAPMISRDheSntPowerUduleMISR(psDevInfo->pvAPMISRDheLog=elignerUduleMISR(psDevInfo->pvAPMISRDhe,puAif

iFlag/
wduleMISR(psDevInfo->pvAPMISRDe        wduleMISR(psDevInfo->pvAPMISRDcpEMemDescPhrFO_COHER
ahe,puAif

iFlag |=  _CR_SOC_AXI_MASKFULL))
WRIDATIOMBINE | wduleMISR(psDevInfo- _CR_SOC_AXI_MASKFULZERO_ON_XI_MAulatranRegion = 0; ui32Region <heLog=eligntil*pvReegion = 0; ui32Region <e      or (uie "rCOM, ME(
Aif

ecpu
W %sntpuUti"IOcezT xt)hreat is beinConmemFwAif

ecp

	/* Process the FirmwroUFWIF_SYSSSSSSSSSSSSSSSSSSSheSntPowerUduleMISR(psDevInfo->pvAPMIhe,puAif

iFlag/
wduleMISR(psDevInfo->pvAPMIcezT xt  weMISR(psDevInfo->pvAPMISRDcpEMemDescPhrFO_CO    RGRVR_DPF((PVR_DBG_MESSAGE,
DEDICAR sKed(PV ORY)U timX_IS_FEATURE_SUPPORTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_KMBuildOpelMMUKedAgaintaD_DPF(

 @DescriptlMM

 k memequireme
W build opelMMUiagaintavKMVR_DPF((build opelMMUi(KMVbuild opelMMUive c)

 FollowND_THvoidiisLupdundant,veVcapRavn xtC_voidi(voids remesam* bits.
 Rpdundancy ociInsveVcapRavDPFclient-serPF((W_MEbuild-orepalig_sLats client-fRGXFW_MEW_M	Ebuild-orepalig_sLt_UNRserPF(-fRGXFW_MEW_MEbuild-orepalig_sLas well.

 32AttHvoidiisLleftiurn claritys4Gpeis bem_COagesvDPFSMaphe repaligilitysociIns.

 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_KMBuildOpelMMUKedAgaintaD_DPF(RV_ERROR_OSINIT *psFwOsringKi:byP:wState;
	/* snapshot fruABRIC_TYPEOOP_NONBuildOpelMMUIO    BuildOpelMMUFWKMPartIO    BuildOpelMMUMismatchE_BLE;
	}
FwOsring iFirmwareCenoopingMode;
	IMG_UINT32 ui32AppHint
P_NONBuildOpelMMU_DP(sDevBUILD_OP****S_KM)nt
P_NONBuildOpelMMUFWKMPartPU_ACFwOsring isPGXVrepOKCCBe._NONBuildOpelMMU_& sDevBUILD_OP****S_EREN_noop
******** BuildOpelMMU__emsk =BuildOpelMMUFWKMPartONS][GP    BuildOpelMMUMismatch emsk =BuildOpelMMU ^msk =BuildOpelMMUFWKMPartlt cCoherency;
Mode;
	STRICTTIOMPATPthe H)
dad.eadMx		PVR bugnf
   opelMM ouaDas we"d wyP:yP:yP rebinaelMMUivfVR bugnvsLupui64ss4Gpum_& kmChange(psBuildOpelMMUMismatch adHWOP****S_R, ui3ERENeS_UINT32e********** BuildOpelMMU_&O    BuildOpelMMUMismatch)i,
		         NOOP_CROSS(FAIL)chedr cringVrepalOKCCB	{Mismatchs4Gp(RGXFW_MEats KMVR_DPF((build opelMMU; ed sta" xtra opelMMUiXI_Cices GpG_BOKMVR_DPF(: (CY_S). Pui64ssHvoidirgx_opelMMU.hE_SNOOP_NONBuildOpelMMU_&O    BuildOpelMMUMismatch _SUPPORTED(psDMode;
	IMG_UIBUILD_OP****S_EISMATCH, t
		2e********** BuildOpelMMUFWKMPartP&O    BuildOpelMMUMismatch)i,
		         NOOP_CROSS(FAIL)chedr cringVrepalOKCCB	{Mismatchs4Gp(RGXFW_M-ht noats KMVR_DPF((build opelMMU; ed sta" xtra opelMMUiXI_Cices Gp(RGXFW_M: (CY_S). Pui64ssHvoidirgx_opelMMU.hE_SNOOP_NONBuildOpelMMUFWKMPartP&O    BuildOpelMMUMismatch _SUPPORTED(psDMode;
	IMG_UIBUILD_OP****S_EISMATCH, t
		yP:VeturnSte>6V>bValide>6wohehedr cringVrepalOKCCB	{(RGXFW_MEats KMVR_DPF((build opelMMU NT3= hGWIF_GPES;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{(RGXFW_MEats KMVR_DPF((build opelMMU match. [ OK ]WIF_GPES		    RGX_IS_FEA well calibratTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_DDKSO,K6h;KedAgaintaD_DPF(

 @DescriptlMM

 k memequi
W DDK vO,K6h;iagaintavR_DPF((DDK vO,K6h;

 @InpuaDpsefine GP-VR PPro ie G
 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_DDKSO,K6h;KedAgaintaD_DPF(
Mode;
	L_STATE_IDLE
#define GroU andasta										V_ERROR_OSINIT *psFwOsringKi:byP:wseKM, RGXe "rg||(State;
	/* snapshot ffruABRIC_TYPEOOP_NONDDKSO,K6h;;32Region][ui32	FS]));
hrea_NONDDKSO,K6h;		}
	}VERS***32ACK(
	}VERS***3MAJ,}
	}VERS***3MIN)VR		    RGROR RGXSetPoXe "rg)uie "rCOM, ME(
CrepaligilityscKCCB	{KMVR_DPF((ats 
W DDK vO,K6h;"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& _NON
DKSO,K6h;)roUUUUUUUUUUUU_NON
DKSO,K6h;roUUUUUUUUUUUU0xffffffffroUUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUUU                      NS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinupportsevInfo, tat	 psDevIui32State;
	/* snapshot fruE;
	}
FwOsring iFirmwareCenoopingMode;
	IMG_UINT32 ui32AppHint
PE;
	}
FwOsring isPGXVrepOKCCBe._NONDDKSO,K6h;	_emsk =
DKSO,K6h;)VICE_SNOOP_CROSS(FAIL)chedr cringVrepalOKCCB	{Inorepalig_sLR_DPF((DDK vO,K6h; (%u.%u) /{(RGXFW_MEDDK r PPK6h; (%u.%u)FWyP:v{	
	}VERS***3MAJ,}
	}VERS***3MINyP:v{	
	}VERS***3UN2ACK3MAJ	}
FwOsring isPGXVrepOKCCBe._NONDDKSO,K6h;)yP:v{	
	}VERS***3UN2ACK3MIN	}
FwOsring isPGXVrepOKCCBe._NONDDKSO,K6h;)sdS_FEATURE_VALUE_SUPPORTED(DDK_VERS***3MISMATCH, t
rredCommBREAK4MinupportsevInfo, tat;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{R_DPF((DDK vO,K6h; (%u.%u) ats 
RGXFW_MEDDK r PPK6h; (%u.%u) match. [ OK ]WyP:v{	
	}VERS***3MAJ,}
	}VERS***3MINyP:v{	
	}VERS***3MAJ,}
	}VERS***3MIN)F_GPES		    RGX_IS_FEA well calibratTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_DDKBuildKedAgaintaD_DPF(

 @DescriptlMM

 k memequi
W DDK build againtavR_DPF((DDK build

 @InpuaDpsefine GP-VR PPro ie G
 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_DDKBuildKedAgaintaD_DPF(
Mode;
	L_STATE_IDLE
#define GroU andasta										V_ERROR_OSINIT *psFwOsringKi:b2Region][ui32	FS]));
= well calibyP:wseKM, RGXe "rg||(State;
	/* snapshot ffruABRIC_TYPEOOP_NONDDKBuildhrea_NONDDKBuild		}
	}VERS***3BUILDVR		    RGROR RGXSetPoXe "rgxstartstop.hMode;
	STRICTTIOMPATPthe H)
die "rCOM, ME(
CrepaligilityscKCCB	{KMVR_DPF((ats 
W DDK build"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& _NON
DKBuild)roUUUUUUUUUUUU_NON
DKBuildroUUUUUUUUUUUU0xffffffffroUUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUUU                      NS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinupportsevInfo, tat	 psDevIui32State;
	/* snapshot fruE;
	}
FwOsring iFirmwareCenoopingMode;
	IMG_UINT32 ui32AppHint
PE;
	}
FwOsring isPGXVrepOKCCBe._NONDDKBuild	_emsk =
DKBuild)VICE_SNOOP_CROSS(lide)chedr cringVrepalOKCCB	{DT3= hicesT_DPF((DDK build vO,K6h; (%d) /{(RGXFW_MEDDK build vO,K6h; (%d).E_SNOOP_NON
DKBuildr_ACFwOsring isPGXVrepOKCCBe._NON
DKBuild))ibyP:wseKM, RGXode;
	STRICTTIOMPATPthe H)
daATURE_VALUE_SUPPORTED(DDK_BUILD_MISMATCH, t
rredCommBREAK4MinupportsevInfo, _UINT32e |;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{R_DPF((DDK build vO,K6h; (%d) ats 
RGXFW_MEDDK build vO,K6h; (%d) match. [ OK ]WyP:v{	_NON
DKBuildr_ACFwOsring isPGXVrepOKCCBe._NON
DKBuild))ibs}G_UINT32e_IS_FEATURE_SUPPORTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_BVNC_edAgaintaD_DPF(

 @DescriptlMM

 k memequi
W BVNC againtavR_DPF((BVNC

 @InpuaDpsefine GP-VR PPro ie G
 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_BVNC_edAgaintaD_DPF(
Mode;
	L_STATE_IDLE
#define GroU andasta										V_ERROR_OSINIT *psFwOsringKi:byP:wState;
	/* snapshot fruABRIate !=Crepalig_sAll,!=Crepalig_sSO,K6h;r!=Crepalig_sBVNC, _UINT32yP:wseKM, RGXe "rg||(State;
	/* snapshot ffruV_ERROR_IOMPthe HS_BVNC_DECLot CAND_INIT(sBVNC);32Region][ui32	F	FS]));
hreasBVNC._N64BVNC =irgx_bvnc_pasU

	/*  used o
	deCE_uriCfg*psDeB,oU andation used o
	deCE_uriCfg*psDeV/
U a aATION used o
	deCE_uriCfg*psDeN,
U a aATION used o
	deCE_uriCfg*psDeC)VR		    RGROR RGXSetPoXe "rg)uie "rCOM, ME(
CrepaligilityscKCCB	{KMVR_DPF((ats 
W BVNC (struda vO,K6h;)"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sFWBVNC) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _NONLayoutSO,K6h;)roUUUUUUUUUUUsBVNC._NONLayoutSO,K6h;roUUUUUUUUUUU0xffffffffroUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUU                      NS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MiPOR)uie "rCOM, ME(
CrepaligilityscKCCB	{KMVR_DPF((ats 
W BVNC (BNC partP- l     32 bits)"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sFWBVNC) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _N64BVNC)roUUUUUUUUUUU(ABRIC_TYPE)sBVNC._N64BVNCroUUUUUUUUUUU0xffffffffroUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUU                      NS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MiPORuie "rCOM, ME(
CrepaligilityscKCCB	{KMVR_DPF((ats 
W BVNC (BNC partP- High   32 bits)"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sFWBVNC) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _N64BVNC) +oUUUUUUUUUUUn PVRSRABRIC_TYPE)roUUUUUUUUUUU(ABRIC_TYPE)(sBVNC._N64BVNC >> PE)roUUUUUUUUUUU0xffffffffroUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUU                      NS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MiPO	 psDevIui32State;
	/* snapshot fruE;
	}
FwOsring iFirmwareCenoopingMode;
	IMG_UINT32 ui32AppHint
P*phGBVNC_	psDe(sBVNCr_ACFwOsring isPGXVrepOKCCBe.sFWBVNC,!=Crepalig_sAll,!=Crepalig_sSO,K6h;r!=Crepalig_sBVNC)VR_DPF((
	}

	P NULL)
		{
			OSSchedule=Crepalig_sAll ncy value
		MORT_FULL!=Crepalig_sAllONS][GP (dl!=Crepalig_sSO,K6h;         NOOP_CROSS(FAIL)c****Inorepalig_sL repaligilitysstruda vO,K6h;ivfVR_DPF(((%u) ats fRGXFW_ME(%u)FWyP:v{		4GpuStatCRVG	UUsBVNC._NONLayoutSO,K6h;roUUUUUACFwOsring isPGXVrepOKCCBe.sFWBVNC._NONLayoutSO,K6h;{SoC sEATURE_VALUE_SUPPORTED(BVNC_MISMATCH, t
nTED(psDevInfo, META[GP (dl!=Crepalig_sBVNC)        NOOP_CROSS(FAIL)chedr cringVrepalOKCCB	{Mismatchs4GpKMVR_DPF((BVNC (%u.%u.%u.%u) ats 
RGXFW_MEBVNC (%u.%u.%u.%u)WyP:v{		*phGBVNC_2ACKED_EXTD(B(sBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(V(sBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(N(sBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(C(sBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(B	}
FwOsring isPGXVrepOKCCBe.sFWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(V(}
FwOsring isPGXVrepOKCCBe.sFWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(N	}
FwOsring isPGXVrepOKCCBe.sFWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(C	}
FwOsring isPGXVrepOKCCBe.sFWBVNC._N64BVNC){SoC sEATURE_VALUE_SUPPORTED(BVNC_MISMATCH, t
nTED(psDevInfo, METAPES;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{(RGXFW_MEBVNC ats KMVR_DPF((BNVC match. [ OK ]WIF_GPES		    RX_IS_FEA well calibratTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_BVNC_HdAgaintaD_DPF(

 @DescriptlMM

 k memequiHW BVNC againtavR_DPF((BVNC

 @InpuaDpsefine GP-VR PPro ie G
 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_BVNC_HdAgaintaD_DPF(
Mode;
	L_STATE_IDLE
#define GroU andasta								V_ERROR_OSINIT *psFwOsringKi:byP:wseKM, RGXe "rg ||wState;
	/* snapshot fruABRIC_TY64 _N64eadMBVNC =i*phGBVNC_2ACK_EREN_B astStasta	*phGBVNC_2ACK_EREN_V astStasta	*phGBVNC_2ACK_EREN_N astStasta	*phGBVNC_2ACK_EREN_Aulatranion][ui32	F	FS]));
hruV_ERROR_IOMPthe HS_BVNC_DECLot CAND_INIT(sSWBVNC)VR	 psDevIui32State;
	/* snapshot fruV_ERROR_IOMPthe HS_BVNC_DECLot CAND_INIT(sHWBVNC)VRuABRIate !=Crepalig_sAll,!=Crepalig_sSO,K6h;r!=Crepalig_sBVNC, _UINT32GROR RGXSetPoXe "rg)uie "r       TcheSnPDumpiFlag =S                       f	 psDevInE;
	}
Config);
bIgnoreHWReyP:yL BVNC)  CE_SNOOP_CROSSBVNC crepaligilityscKCCBsveVtweenVR_DPF((ats HW W_MET_ACE_L  (OP_CPU_ oPF(ride)"aSoC supports full AOKlt	
		************Xe "rg ||wState;
	/* snapshot fr************IOMPATPBVNC_MREN_B)ea_N64eadMBNC adHW*phGBVNC_2ACK_EREN_B, _UINT32yP:wseKM, RGIOMPATPBVNC_MREN_V)ea_N64eadMBVNC adHW*phGBVNC_2ACK_EREN_V, _UINT32yP:wseKM, RGIOMPATPBVNC_MREN_N)ea_N64eadMBVNC adHW*phGBVNC_2ACK_EREN_N, _UINT32yP:wseKM, RGIOMPATPBVNC_MREN_C)ea_N64eadMBVNC adHW*phGBVNC_2ACK_EREN_C, _UINT32G	sSWBVNC._N64BVNC =irgx_bvnc_pasU

	/*  used o
	deCE_uriCfg*psDeB,oU andUUUUACion used o
	deCE_uriCfg*psDeV/
U a aUUUUACion used o
	deCE_uriCfg*psDeN/
U a aUUUUACion used o
	deCE_uriCfg*psDeC)VR_p
*******64eadMBVNC !DP(sDevBVNC_2ACK_EREN_B ai*phGBVNC_2ACK_EREN_V ai*phGBVNC_2ACK_EREN_N ai*phGBVNC_2ACK_EREN_C))  CE_SNOOP_CROSSCrepaligilityscKCCB***IgnorND_Tfield***'%s%s%s%s'ivfVHW BVNCFWyP:v{	((!***64eadMBVNC &i*phGBVNC_2ACK_EREN_B))?SSB"):(""aSyP:v{	((!***64eadMBVNC &i*phGBVNC_2ACK_EREN_V))?SSV"):(""aSyP:v{	((!***64eadMBVNC &i*phGBVNC_2ACK_EREN_N))?SSN"):(""aSyP:v{	((!***64eadMBVNC &i*phGBVNC_2ACK_EREN_C))?SSC"):(""aSi64MiPO	 psDevIui32RGXSetPoXe "rg)uie "rCOM, MEWITHSKFUS(heSnPDumpiFlagoheCrepaligilityscKCCB	{Layout vO,K6h;ivfVcrepcKCCBsvstruda"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sHWBVNC) +oUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _NONLayoutSO,K6h;)roUUUUUUUUUUUsSWBVNC._NONLayoutSO,K6h;roUUUUUUUUUUU0xffffffffroUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUUheSnPDumpiFlagNS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinupportsevInfo, tats-ie "rCOM(heSnPDumpiFlagoheBVNC crepaligilityscKCCBG_ER:yL "
	if (dl**64eadMBVNC &i(sDevBVNC_2ACK_EREN_B ai*phGBVNC_2ACK_EREN_N ai*phGBVNC_2ACK_EREN_C))  CE_SNe "rIF("DISis c_HWBNC_the H", heSnPDumpiFlagNS;fSNe "rELSE("DISis c_HWBNC_the H", heSnPDumpiFlagNS;fSNe "rCOM, MEWITHSKFUS(heSnPDumpiFlagoheCrepaligilityscKCCB	{HW BNC ats FW BNC (L     32 bits)"i32SSt is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sHWBVNC) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _N64BVNC)roUUUUUUUUUUUU(ABRIC_TYPE)sSWBVNC._N64BVNC roUUUUUUUUUUUU(ABRIC_TYPE)l**64eadMBVNC &iW*phGBVNC_2ACK_EREN_V)roUUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUUUheSnPDumpiFlagNS;fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32Pdd)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinnTED(psDevInfo, META[GPNe "rCOM, MEWITHSKFUS(heSnPDumpiFlagoheCrepaligilityscKCCB	{HW BNC ats FW BNC (High   32 bits)"i32SSt is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUUUUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS& sHWBVNC) +oUUUUUUUUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _N64BVNC) +oUUUUUUUUUUUUn PVRSRABRIC_TYPE)roUUUUUUUUUUUU(ABRIC_TYPE)(sSWBVNC._N64BVNC >> PE)roUUUUUUUUUUUU(ABRIC_TYPE)(l**64eadMBVNC &iW*phGBVNC_2ACK_EREN_V) >> PE)roUUUUUUUUUUUU      POLL_OP;
		or 	psDeroUUUUUUUUUUUUheSnPDumpiFlagNS;fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32Pdd)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinnTED(psDevInfo, META[GPNe "rFI("DISis c_HWBNC_the H", heSnPDumpiFlagNS;f}if (dl**64eadMBVNC &i*phGBVNC_2ACK_EREN_V)  CE_SNe "rIF("DISis c_HWV_the H", heSnPDumpiFlagNS;fSNe "rELSE("DISis c_HWV_the H", heSnPDumpiFlagNS;[GPNe "rCOM, MEWITHSKFUS(heSnPDumpiFlagoheCrepaligilityscKCCB	{HW V ats FW V"i32SSt is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUofftetRSRV_ERROR_IOMPthe HS& sHWBVNC) +oUUUUUofftetRSRV_ERROR_IOMPthe HS_BVNC& _N64BVNC) +oUUUUU((sDevBVNC_2ACK_SHIFT_V >= PE) ? n PVRSRABRIC_TYPE) : 0)roUUUUU(ABRIC_TYPE)(sSWBVNC._N64BVNC >> ((sDevBVNC_2ACK_SHIFT_V >= PE) ? 32 : 0))yP:v{		*phGBVNC_2ACK_EREN_V >> ((sDevBVNC_2ACK_SHIFT_V >= PE) ? 32 : 0)yP:v{		      POLL_OP;
		or 	psDeroUUUUUheSnPDumpiFlagNS;fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32Pdd)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinnTED(psDevInfo, METAGPNe "rFI("DISis c_HWV_the H", heSnPDumpiFlagNS;f}s-ie "rCOM(heSnPDumpiFlagoheBVNC crepaligilityscKCCBGXSeishL "
	i	 psDevIui32State;
	/* snapshot fruE;
	}
FwOsring iFirmwareC{eCenoopingMode;
	IMG_UINT32 ui32AppHintETA[GsHWBVNCPU_ACFwOsring isPGXVrepOKCCBe.sHWBVNC;A[GsHWBVNC._N64BVNC &= _N64eadMBVNC;G	sSWBVNC._N64BVNC &= _N64eadMBVNC;G
P*phGBVNC_	psDe(sSWBVNC,!sHWBVNC,!=Crepalig_sAll,!=Crepalig_sSO,K6h;r!=Crepalig_sBVNC)VR_DPF((
	}

	P NULL)
		{
			OSSchedule=Crepalig_sAll ncy value
		MORT_FULL!=Crepalig_sAllONS][GP (dl!=Crepalig_sSO,K6h;         NOOP_CROSS(FAIL)c****Inorepalig_sL repaligilitysstruda vO,K6h;ivfVHW (%d) ats 
W (%d).E_SNOOP	4GpuStatCRVG	UUsHWBVNC._NONLayoutSO,K6h;roUUUUUsSWBVNC._NONLayoutSO,K6h;{SoC sEATURE_VALUE_SUPPORTED(BVNC_MISMATCH, t
nTED(psDevInfo, META[GP (dl!=Crepalig_sBVNC)        NOOP_CROSS(FAIL)chedr cringVrepalOKCCB	{Inorepalig_sLHW BVNC (%d.%d.%d.%d)(ats 
W BVNC (%d.%d.%d.%d).WyP:v{		*phGBVNC_2ACKED_EXTD(B(sHWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(V(sHWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(N(sHWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(C(sHWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(B(sSWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(V(sSWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(N(sSWBVNC._N64BVNC)yP:v{		*phGBVNC_2ACKED_EXTD(C(sSWBVNC._N64BVNC){SoC sEATURE_VALUE_SUPPORTED(BVNC_MISMATCH, t
nTED(psDevInfo, METAPES;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{HW BVNC (%d.%d.%d.%d)(ats 
W BVNC (%d.%d.%d.%d) match. [ OK ]WyP:v{	*phGBVNC_2ACKED_EXTD(B(sHWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(V(sHWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(N(sHWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(C(sHWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(B(sSWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(V(sSWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(N(sSWBVNC._N64BVNC)yP:v{	*phGBVNC_2ACKED_EXTD(C(sSWBVNC._N64BVNC){SoC ES		    RGX_IS_FEA well calibratTE!
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB_PVRSCorsSO,K6h;_AgaintaD_DPF(

 @DescriptlMM

 k memequiHW PVRS vO,K6h;iagaintavR_DPF((PVRS vO,K6h;

 @InpuaDpsefine GP-VR PPro ie G
 @InpuaDpsFwOsring -e
W   NDameqa

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB_FWde->pvDorSO,K6h;_AgaintaD_DPF(
Mode;
	L_STATE_IDLE
#define GroU andastaV_ERROR_OSINIT *psFwOsringKi:byP:wseKM, RGXe "rg||(State;
	/* snapshot ffruRegion][ui32	FS]));
hr_UINT32GROR RGXSetPoXe "rg)uie "r       TcheSnPDumpiFlag =S                       f	 psDevInABRIC_TYPEOsk =FWCorsIDk mueN_NUMrams: TDSetPpcV_ERR_PROCESS-scinEVICE_F (dlR = sKernelMMUC32 UUCined(NO_HARDWARE) && PVRSScheduleswitchslR = GETernelMMUC32 UUARDWARE) && PVRSScheedulec64ssMTP218: sk =FWCorsIDk mueN_NR = CR_PVRS_MTP218   MUCui332 UU; break;ulec64ssMTP219: sk =FWCorsIDk mueN_NR = CR_PVRS_MTP219   MUCui332 UU; break;ulec64ssLTP218: sk =FWCorsIDk mueN_NR = CR_PVRS_LTP218   MUCui332 UU; break;ulec64ssLTP217: sk =FWCorsIDk mueN_NR = CR_PVRS_LTP217   MUCui332 UU; break;uleoe RGX_:evInfo->sLayerParams.ui32Pdd)****UnRGXSetPe
W   MUCui332 UURENCY  0x2 {SoC s*pvReASSERT;0e);f
			{pcV_ERR_PROCESS-scinV_ERR_PROCESS-s_PVRS, tat;

	d  (dlR = sKernelMMUCined(NO_HARDWARE) && RISCV_RR_PROCESS-sSchedulesk =FWCorsIDk mueN_NR =RISCV
W   MUCui332 UU;		{pcV_ERR_PROCESS-scinV_ERR_PROCESS-s_RISCV, tat;

	dvoid *pvRegsBaseKM;
#enui32Pdd)****UnRGXSetPe
W   MUCui332 UURENCY  0x2 {SoC spvReASSERT;0e);fvRegsBaseKM, RGXe "rg)SNe "rIF("DISis c_HWPVRS_the H", heSnPDumpiFlagNS;fNe "rELSE("DISis c_HWPVRS_the H", heSnPDumpiFlagNS;fNe "rCOM, MEWITHSKFUS(heSnPDumpiFlagoheCrepaligilityscKCCB	{KMVR_DPF((ats HW 
W de->pvDor vO,K6h;"i32St is beinConmemPDumpConmemOKCCB32(psDevInfo->pESSAGE,
OsringMemDescroUUUUUofftetRSRV_ERROR_OSINIT, sPGXVrepOKCCBe) +oUUUUUofftetRSRV_ERROR_IOMPthe HS& sk =FWde->pvDorSO,K6h;)roUUUUUheSnFWCorsIDk mueroUUUUU0xffffffffroUUUUU      POLL_OP;
		or 	psDeroUUUUUheSnPDumpiFlagNS;f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLid)hedr cringVrepalOKCCB	{prog_sm pInfoND_TPOLiurn pESSAGE,
OsringMemDesc (%d)",plist_ii64MinupportsevInfo, tatPNe "rFI("DISis c_HWPVRS_the H", heSnPDumpiFlagNS;	 psDevIui32State;
	/* snapshot fruE;
	}
FwOsring iFirmwareCenoopingMode;
	IMG_UINT32 ui32AppHint
PE;
	}
FwOsring isPGXVrepOKCCBe._NONFWde->pvDorSO,K6h;	_emsk =FWCorsIDk mue)  CE_SNOOP_CROSShedr cringVrepalOKCCB	{Inorepalig_sLR_DPF((%s vO,K6h; (%d) /{HW(%s vO,K6h; (%d).WyP:v{	 pcV_ERR_PROCESS-syP:v{	 heSnFWCorsIDk mueroUUUU pcV_ERR_PROCESS-syP:v{	 }
FwOsring isPGXVrepOKCCBe._NONFWde->pvDorSO,K6h;sdS_FEATURE_VALUE_SUPPORTED(FWdROCESS-s_PISMATCH, t
rredCommBREAK4MinupportsevInfo, tat;

	dvoid *pvRegsBaseKM;
#enMESSAGEohehedr cringVrepalOKCCB	{Crepalig_sLR_DPF((%s vO,K6h; (%d) /{HW(%s vO,K6h; (%d) [OK].WyP:v{	 pcV_ERR_PROCESS-syP:v{	 heSnFWCorsIDk mueroUUUU pcV_ERR_PROCESS-syP:v{	 }
FwOsring isPGXVrepOKCCBe._NONFWde->pvDorSO,K6h;sdS_FES		    RX_IS_FEA well calibratTE!
*******************************************************************************
*******************************************************************************

 @F 0x(driLnVLr cringVrepalOKCCB

 @DescriptlMM

 Cvoidi repaligilitysof hotavR_DPF((ats fRGXFW_ME(DDK ats build opelMMU)
iurn nVLVR PPros atRserPPros/R PPro ieiti mesat6h;

 @InpuaDpsefiProcessP-VR PPro ness

 @RIS_FEA  Min=0x%x, Max-Y  RT_2ND_Tse mismatchsfound

******************************************************************************yP:wLay6y
				/* Re-schedr cringVrepalOKCCB	break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTI[ui32	FS]));
hrOT_IMPLEL_STATE_IDL	
#define GPU_ACTIVE     RGXFWIF_GPU_UyP:wState;
	/* snapshot fruABRIC_TYPEOOP_NONRegk mue;ruABRIC_TY8OOP_N8FwOs
		  nt
PLOOP_U   L_TIMERV_(CtWoHW_TIME_USONS][GP (dl*((volLay_sLABRIate !*)&psDevInfo->pESSAGE,
Osring isPGXVrepOKCCBe.bUpmequdScheeduleteLMNotne		EPVRwang if			PV
W has already upmequd			PVv muesver t		break;ule			{OSWangus(CtWoHW_TIME_US/WAIw YAY_IOU  dS_FE END_LOOP_U   L_TIMERV_()nt
P_NONRegk mueN_NUMr
PE;
	(!
	}

	P NULL)
		{
			OSSc &&
{	*phGsKernelMMUC32 UUCined(NO_HARDWARE) && PVRSSchedulet is being pReadPVRSnfo->sLayerPar& PVRS CR_T0ENis c_OFFSET, &_NONRegk mueoopo->tPro
#if defined(PDUMP)
	psDevInfo->_CROSS****ReadND_TnVLVPVRS ;
	}

	p in.
		F Is reme (1)correctly p    ud	up? (%u)RSRVGOP	4GpuStatCplist_ii64MinngoE_Lchk_exit, META[GP (dl!(_NONRegk mueN& PVRS CR_TXENis c_ENis c_BITScheeduletATURE_VALUE_SUPPORTED(PVRS THREAD0(psDeENis cDoC s*pvResLayerParams.ui32Pdrin*{	 S****RVLVPVRS  denot runnND_F Is reme (1)correctly p    ud	up? %d (%u)RSRVGOP	4GpuStatCppsDevInfo->pESSAGE,
Osring isPGXVrepOKCCBe.bUpmequdCplist_ii64MinngoE_Lchk_exit, METAMORT_FULL!*((volLay_sLABRIate !*)&psDevInfo->pESSAGE,
Osring isPGXVrepOKCCBe.bUpmequdSche{_FEATURE_VALUE_SUPPORTED(TIMERV_;d *pvRegsBaseKM;
#enui32Pdd)**** (1)(RGXFW_MEnot respo_2ND_: in.
		EPVRyP:ylyi repaligilitysie G (%u)RSRVGOP4GpuStatCplist_ii64MinPF((
	}

	P NULL)
		{
			OSSchesDevInfo->sLayerParams.ui32Pdd)****Potenti m capRa***fRGXFW_MEnot ieiti mes		Ern statcurhicesG; /*vR_DPF('s ed staaaaaa"Os
	
	 * ieiti mesat6h;ameqaRwadenot accepqud	by statfRGXFW_MRENCY  0x2 {SoC s			{goE_Lchk_exit, M}t
P_N8FwOs
		  PU_ACTIVInfo->pESSAGE,
Osring isPGXVrepOKCCBe.sringOpelMMU._N8Os
		  SP:yP:yS;f (dl(
	}

	P NULL)
		{
NATIVEta;
	P_N8FwOs
		  P> 1gg ||		{(
	}

	P NULL)
		{
HOOSSa;
	P_N8FwOs
		  P!= se RIDATIOfined(NO_H))vInfo->sKCCBDeferredCommlide>6wohe)
	{MismatchseVtweenVstatnumb h  feOperat6D_TraramsswyP:yP:yud	by KMVR_DPF(((%d) ats 
W (%d)RSRVGOP4GpuStatCp(
	}

	P NULL)
		{
NATIVEt) ? (1) : ase RIDATIOfined(NO_Ht, he8FwOs
		  sdS_FES		    F((PVR_DBG_M* snapshot fU timXt is being pr cringVrepalOKCCB_KMBuildOpelMMUKedAgaintaD_DPF(RACTIVInfo->pESSAGE,
OsringNS;f (dllist_is_empty(&psDevInfo->goE_Lchk_exit, M}t
Pt is being pr cringVrepalOKCCB_DDKSO,K6h;KedAgaintaD_DPF(
sLayerPar& ACTIVInfo->pESSAGE,
OsringNS;f (dllist_is_empty(&psDevInfo->goE_Lchk_exit, M}t
Pt is being pr cringVrepalOKCCB_DDKBuildKedAgaintaD_DPF(
sLayerPar& ACTIVInfo->pESSAGE,
OsringNS;f (dllist_is_empty(&psDevInfo->goE_Lchk_exit, M}t
PFULL!
	}

	P NULL)
		{
			OSSchedulet is being pr cringVrepalOKCCB_BVNC_edAgaintaD_DPF(
sLayerPar& ACTIVInfo->pESSAGE,
OsringNS;f>tPro
#if defined(PDUMP)
	psDevIngoE_Lchk_exit, META[GPt is being pr cringVrepalOKCCB_BVNC_HdAgaintaD_DPF(
sLayerPar& ACTIVInfo->pESSAGE,
OsringNS;f>tPro
#if defined(PDUMP)
	psDevIngoE_Lchk_exit, METAM}t
Pt is being pr cringVrepalOKCCB_FWde->pvDorSO,K6h;_AgaintaD_DPF(
sLayerPar& ACTIVInfo->pESSAGE,
OsringNS;f (dllist_is_empty(&psDevInfo->goE_Lchk_exit, M}t
Pt is bein well calibchk_exit:; ui32RegiTURE_SUPPOion=%u, regatEL_SSoftResetToggle
Mode;
	L_STATE_IDLE
#define GroeMISR(psDevInfo->pvAPMISRDDDDDDDABRIC_TY64  _N64Resetk mueroeMISR(psDevInfo->pvAPMISRDDDDDDDABRIC_TY64  _N64SPUResetk mueKi:bIOSWriteHWReg64RACTIVInfo->pvRegsBa	dKM,NR = CR_SOFT	RESET, _N64Resetk mueNS;f (dlR = GETernelMMUC32 UUARDWARE) && POWER		{LAND_VERS***) iFi1vInfo->OSWriteHWReg64RACTIVInfo->pvRegsBa	dKM,NR = CR_SOFT	RESET_SPU, _N64SPUResetk mueK, M}t
P((PRead soft-I_CitCE_Lfee agpr PPous0writes4GpordF((E_LclearPCounSOCIF pipelDBGver tgrega) OSReadHWReg64RACTIVInfo->pvRegsBa	dKM,NR = CR_SOFT	RESETNS;f (dlR = GETernelMMUC32 UUARDWARE) && POWER		{LAND_VERS***) iFi1vInfo->grega) OSReadHWReg64RACTIVInfo->pvRegsBa	dKM,NR = CR_SOFT	RESET_SPUK, M}tratTE*************************************************************************y TE!
@F 0x(driDDDDDDDL_SSoftReset
@DescriptlMMDDDDLesetswyome moduleUivfV64Gp	IMGR PPro
@InpuaDo->pvAPMIceTIVE     R  ConPro ness
@InpuaDo->pvAPMI_N64Resetk mue  A madMxurn whichseachseitRset)correspo_2soeMISR(psDevInfo->pvAPMISRDDDDDDDNUOU module E_Lupset)(viaPCounSOFT	RESEToeMISR(psDevInfo->pvAPMISRDDDDDDD;
	}

	p).
@InpuaDo->pvAPMI_N64SPUResetk mue A madMxurn whichseachseitRset)correspo_2soeMISR(psDevInfo->pvAPMISRDDDDDDDNUOU module E_Lupset)(viaPCounSOFT	RESET_SPUoeMISR(psDevInfo->pvAPMISRDDDDDDD;
	}

	p).
@RIS_FEA        UE_SUPPORTED
*y TEE*************************************************************************yP:wLay6y
				/* Re-schedSoftResetown request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvAABRIC_TY64  _N64Resetk mueroeMISR(psDevInfo->pvAPMISRDDDDDDDDABRIC_TY64  _N64SPUResetk mueKi:bIMode;
	L_STATE_IDLEEEEEEEE
#define GulatranRASSERT;psefiProcessP!Firmwar;atranRASSERT;psefiProcessGXFWIF_GPUP!Firmwar;atrani32O NUsET_  =LL)

			OS,ppty(&psDevMr
PE;
	((_N64Resetk mue &NR = CR_SOFT	RESET_ERENFmwar	_emsk64Resetk mueNo->||dl**64SPUResetk mue &NR = CR_SOFT	RESET_SPU_ERENFmwar	_emsk64SPUResetk mueKiC{eCenoopingMode;
	IMG_UINT32 ui32AppHintETA[G((PT	PVR PPro ie GviceAceConne GPU_ACTIVE     RGXFWIF_GPU_UateLMAees Gpsoft-I_CitCiceAEL_SSoftResetToggle
RDWARE) && _N64Resetk muer _N64SPUResetk mueK, [G((PTakuirememoduleUivuaDvfVI_Cit...CiceAEL_SSoftResetToggle
RDWARE) && 0& 0NS; ui32Regi well calibrat
				/* Re-schedringAif

iWImgCpu(wn request as it wa as deferred. */
				OSScheduleMISR(psDevInfo->pvAPms: er
}

oSIZRCTc   _NFWCodeLe   				OSScheduleMISR(psDevInfo->pvAPms: er
}

oSIZRCTc   _NFWDeqaLe   				OSScheduleMISR(psDevInfo->pvAPms: er
}

oSIZRCTc   _NFWCorsmemOodeLe   				OSScheduleMISR(psDevInfo->pvAPms: er
}

oSIZRCTc   _NFWCorsmemDeqaLe Ki:bIer
}

o      T ahe,puAif

iFlaghrOT_IMPLEL_STATE_IDL	
#define GPU_ACTIVE     RGXFWIF_GPU_UTRegion][ui32Ofo;
	piTURE_SUPCR_S	/* PAeesup Aif

eclMMDurn 
W cessPsex(driDeviceAhe,puAif

iFlag =  _CR_SOC_AXI_MASKFULest as SKFU(PMPVRS PROTECT) | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	READis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	WRIDAis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULCP
	READis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULCP
	WRIDAis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	CACH
		NCOH = 0T | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULest as SKFU(FIRMhot 	CACH
D) | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULKERNELLCP
	MAPPis cou
Pt is being pAif

ecpFW,puUti32OS, oATION)
	/* ProUduleMISR(psDevInfo->pvAPMISRDDDDDDD_NFWCodeLe   UduleMISR(psDevInfo->pvAPMISRDDDDDDD_N,puAif

iFlag/
wduleMISR(psDevInfo->pvAPMISRDevInfo _CR_SOest as SW   )
	      /
wduleMISR(psDevInfo->pvAPMISRDevInfo"FwCode32OS, "/
wduleMISR(psDevInfo->pvAPMISRDevInfo&psDevInfo->pESSAGECodeMemDescNS; u (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu
		OSSpNUOUif

ecpufw cessPmem (%u)RSRVGvInfo;
	plist_ii64MingoE_Lin.
GECodeMemDescAif

, M}t
Pt is beinDonmem */
	psDonVirtnfo->sLayerParams.dle  CodeMemDesc/
wduleMISR(psDevInfo->pvAPMISRDevInf&psDevInfo->s  CodeDonVnfo-Ba	dr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu
		OSSpNUOUS;
#enddonVnfo-Durn fw cessPmem (%u)RSRVGvInfo;
	plist_ii64MingoE_Lin.
GECodeMemDescAqDonVirtntETA[G((
	(PT	PV
W cessPmust be statfRGst aif

eclMMD GpG_BOfRGXFW_MEheap, ogrfowi	dvo*pG_BObootloadF((willenot work (		PV
W willenot be able E_LfindpG_BObootloadF().
	er tranRASSERT;psefiInfo->s  CodeDonVnfo-Ba	d.uinfo-D== se RFIRMhot 	HOOS	MAIN_HEAP_BASE)UPCR_S	/* PAeesup Aif

eclMMDurn 
W meqaRsex(driDeviceAhe,puAif

iFlag =  _CR_SOC_AXI_MASKFULest as SKFU(PMPVRS PROTECT) | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	READis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	WRIDAis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULCP
	READis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULCP
	WRIDAis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULest as SKFU(FIRMhot 	CACH
D) | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULGP
	CACH
		NCOH = 0T | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULKERNELLCP
	MAPPis c | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULCP
	WRIDATIOMBINE | wduleMISR(psDevInfo _CR_SOC_AXI_MASKFULZERO_ON_XI_MAulatt is being pAif

ecpFW,puUti32OS, oATION)
	/* ProUduleMISR(psDevInfo->pvAPMISRDDDDDDD_NFWDeqaLe   UduleMISR(psDevInfo->pvAPMISRDDDDDDD_N,puAif

iFlag/
wduleMISR(psDevInfo->pvAPMISRDevInfo _CR_SOest as SW PRIVADATDARS       /
wduleMISR(psDevInfo->pvAPMISRDevInfo"FwDeqa32OS, "/
wduleMISR(psDevInfo->pvAPMISRDevInfo&psDevInfo->pESSAGEDeqaMemDescNS; u (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu
		OSSpNUOUif

ecpufw meqaRmem (%u)RSRVGvInfo;
	plist_ii64MingoE_Lin.
GEDeqaMemDescAif

, M}t
Pt is beinDonmem */
	psDonVirtnfo->sLayerParams.dle  DeqaMemDesc/
wduleMISR(psDevInfo->pvAPMISRDevInf&psDevInfo->s  DeqaDonVnfo-Ba	dr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu
		OSSpNUOUS;
#enddonVnfo-Durn fw meqaRmem (%u)RSRVGvInfo;
	plist_ii64MingoE_Lin.
GEDeqaMemDescAqDonVirtntETA[G (dl**FWCorsmemOodeLe i,
		   fo->_S	//* PAeesup Aif

eclMMDurn 
W cersmem cessPsex(driDeeviceAAhe,puAif

iFlag =  _CR_SOC_AXI_MASKFULest as SKFU(PMPVRS PROTECT) | w		 _CR_SOC_AXI_MASKFULest as SKFU(FIRMhot 	CACH
D) | w		 _CR_SOC_AXI_MASKFULGP
	READis c | w		 _CR_SOC_AXI_MASKFULCP
	READis c | w		 _CR_SOC_AXI_MASKFULCP
	WRIDAis c | w		 _CR_SOC_AXI_MASKFULGP
	CACH
		NCOH = 0T | w		 _CR_SOC_AXI_MASKFULKERNELLCP
	MAPPis c | w		 _CR_SOC_AXI_MASKFULCP
	WRIDATIOMBINE | w		 _CR_SOC_AXI_MASKFULZERO_ON_XI_MAulattt is being pAif

ecpFW,puUti32OS, oATION)
	/* ProUUduleMISR(psDevInfo->pvAPMISRDDDDDDD_NFWCorsmemOodeLe   UUduleMISR(psDevInfo->pvAPMISRDDDDDDD_N,puAif

iFlag/
wSduleMISR(psDevInfo->pvAPMISRDDDDDDD _CR_SOest as SW   RE}

o  )
	      /
wwduleMISR(psDevInfo->pvAPMISRDevInfo"FwCorsmemOode32OS, "/
wwduleMISR(psDevInfo->pvAPMISRDevInfo&psDevInfo->pESSAGECorsmemOodeMemDescNS; uSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"F		OSSpNUOUif

ecpufw cersmem cessPmem, n PV: %"APms: _TY64_FMTSPECd ",nf
  s: %"A _CR_SOC_AXI_MASKFUS_FMTSPEC " (%u)RSRVGOSRDDDDDDD_NFWCorsmemOodeLe  D_N,puAif

iFlag/plist_ii64MinngoE_Lin.
GECorsmemMemDescAif

, META[GPt is beinDonmem */
	psDonVirtnfo->sLayerParams.dle  CorsmemOodeMemDesc/
wwduleMISR(psDevInfo->pvAPMISRDevInf&psDevInfo->s  CorsmemOodeDonVnfo-Ba	dr, fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"F		OSSpNUOUS;
#enddonVnfo-Durn fw cersmem mem cessP(%u)RSRVGOSRDDDDDDDlist_ii64MinngoE_Lin.
GECorsmemCodeMemDescAqDonVirtntEETA[GPt is being pAee(RGXFW_Mnfo-ess(&psDevInfo->s  CorsmemOodeFWnfo-/
wwduleMISR(psDevInfo->pvsLayerParams.dle  CorsmemOodeMemDesc/
wwduleMISR(psDevInfo->pv0, RSW FWnDDR_N REF SKFUSoC spvReLOULGOTO_  =ui32Pu
#if dohehedAee(RGXFW_Mnfo-ess:1",nfn.
GECorsmemCodeMemDescFwnfo-F_GPES;

	dvoid *psDevInfo->s  CorsmemOodeDonVnfo-Ba	d.uinfo-D=NUMra*psDevInfo->s  CorsmemOodeFWnfo-._NONnfo-D=NUMraTA[G (dl**FWCorsmemDeqaLe i,
		   fo->_S	//* PAeesup Aif

eclMMDurn 
W cersmem meqaRsex(driDeeviceAAhe,puAif

iFlag =  _CR_SOC_AXI_MASKFULest as SKFU(PMPVRS PROTECT) | w			 _CR_SOC_AXI_MASKFULest as SKFU(FIRMhot 	CACH
D) | w			 _CR_SOC_AXI_MASKFULGP
	READis c  | w			 _CR_SOC_AXI_MASKFULGP
	WRIDAis c | w			 _CR_SOC_AXI_MASKFULCP
	READis c  | w			 _CR_SOC_AXI_MASKFULCP
	WRIDAis c | w			 _CR_SOC_AXI_MASKFULZERO_ON_XI_MA | w			 _CR_SOC_AXI_MASKFULGP
	CACH
		NCOH = 0Tulattt is being pAif

ecpFW,puUti32OS, oATION)
	/* ProUUAAheFWCorsmemDeqaLe roUUAAhe,puAif

iFlag/
wS		 _CR_SOest as SW   RE}

oDARS       /
waaa"FwCorsmemDeqa32OS, "/
wwww&psDevInfo->pESSAGE,
CorsmemDeqaStorsMemDescNS; uSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"F		OSSpNUOUif

ecpufw cersmem meqaRmem, ed stSRDevInfo"n PV: %"APms: _TY64_FMTSPECd ",nf
  s: %"A _CR_SOC_AXI_MASKFUS_FMTSPEC " (%u)RSRVGOSRDDDDDDD_NFWCorsmemDeqaLe roUUASRDDDDDDD_N,puAif

iFlag/
wS	SRDDDDDDDlist_ii64MinngoE_Lin.
GECorsmemDeqaMemDescAif

, META[GPt is beinDonmem */
	psDonVirtnfo->sLayerParams.dle  ,
CorsmemDeqaStorsMemDesc/
wwww&psDevInfo->sGECorsmemDeqaStorsDonVnfo-Ba	dr, fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"F		OSSpNUOUS;
#enddonVnfo-Durn fw cersmem mem meqaR(%u)RSRVGOSRDDDDDDDlist_ii64MinngoE_Lin.
GECorsmemDeqaMemDescAqDonVirtntEETA[GPt is being pAee(RGXFW_Mnfo-ess(&psDevInfo->s  CorsmemDeqaStorsFWnfo-/
wwa*psDevInfo->s.dle  ,
CorsmemDeqaStorsMemDesc/
wwww0, RSW FWnDDR_N REF SKFUSoC spvReLOULGOTO_  =ui32Pu
#if dohehedAee(RGXFW_Mnfo-ess:2",nfn.
GECorsmemDeqaMemDescFwnfo-F_GPES;

	dvoid *psDevInfo->s  CorsmemDeqaStorsDonVnfo-Ba	d.uinfo-D=NUMra*psDevInfo->s  CorsmemDeqaStorsFWnfo-._NONnfo-D=NUMraTA[Gi32Regi well calib
fn.
GECorsmemDeqaMemDescFwnfo-:
fn.
GECorsmemDeqaMemDescAqDonVirt:[G (dl**FWCorsmemDeqaLe i,
		   fo->DonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGE,
CorsmemDeqaStorsMemDescNS;a*psDevInfo->s.dle  ,
CorsmemDeqaStorsMemDesccinEVICE_F}
fn.
GECorsmemDeqaMemDescAif

:
fn.
GECorsmemCodeMemDescFwnfo-:
fn.
GECorsmemCodeMemDescAqDonVirt:[G (dl**FWCorsmemOodeLe i,
		   fo->DonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGECorsmemOodeMemDescNS;a*psDevInfo->s.dle  CorsmemOodeMemDesccinEVICE_F}
fn.
GECorsmemMemDescAif

:
fn.
GEDeqaMemDescAqDonVirt:[GDonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGEDeqaMemDescNS;*psDevInfo->s.dle  DeqaMemDesccinEVICE_in.
GEDeqaMemDescAif

:
fn.
GECodeMemDescAqDonVirt:[GDonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGECodeMemDescNS;aACTIVInfo->pESSAGECodeMemDesccinEVICE_in.
GECodeMemDescAif

:2e_IS_FEATURE_SUPPORTE
	OP_CPU_ parame
	p PU_erface
*yP:wLay6t
				/* Re-schedFWTraceQueryFil_er(contavwn request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvA  contavregats dPrivecp/
				OSScheduleMISR(psDevInfo->pvA  ABRIC_TYPEts psDeV mueKi:bIMode;
	 Re-sce32sul nt
Pe32sul  =  _CR_Sdle  DebugQueryFWLogKM(EVIC,_ACTIVE     R,_ApsDeV mueKS;as psDeV mue &= V_ERROR_LOULTYPE_GROUP_EREN;2e_IS_FEAT32sul ntPOion=%u,t
				/* Re-schedFWTraceQueryLogType(contavwn request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvA  contavregats dPrivecp/
				OSScheduleMISR(psDevInfo->pvA  ABRIC_TYPEts psDeV mueKi:bIMode;
	 Re-sce32sul nt
Pe32sul  =  _CR_Sdle  DebugQueryFWLogKM(EVIC,_ACTIVE     R,_ApsDeV mueKS;aPF((
	}

	POKD== e32sul ONS][GP (dl* psDeV mue & V_ERROR_LOULTYPE_TRACE
	psDevIn* psDeV mue =NUMF((PTraceviceAA			{

	dvosDevIn* psDeV mue =N1MF((PTBIviceAA			}2e_IS_FEAT32sul ntPOion=%u,t
				/* Re-schedFWTraceAee(Rl_er(contavwn request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvA contavregats dPrivecp/
				OSScheduleMISR(psDevInfo->pvA ABRIC_TYPEtpsDeV mueKi:bIMode;
	 Re-sce32sul nt	ABRIC_TYPEtpsDehedFWLogTypent
Pe32sul  = hedFWTraceQueryLogType(ACTIVE     R,_EVIC,_&psDehedFWLogTypeKS;aPF((
	}

	POKD== e32sul ONS][GP (dl0D== psDehedFWLogTypeKvosDevInBITEREN_SET(psDeV mue, V_ERROR_LOULTYPE_TRACE
oC s			{e32sul  =  _CR_Sdle  DebugAee(WLogKM(EVIC,_ACTIVE     R,_psDeV mueKS;a}2e_IS_FEAT32sul ntPOion=%u,t
				/* Re-schedFWTraceAeeLogType(contavwn request as it was deferred. */
				OSScheduleMISR(psDevInfo->pvA   contavregats dPrivecp/
				OSScheduleMISR(psDevInfo->pvA   ABRIC_TYPEtpsDeV mueKi:bIMode;
	 Re-sce32sul nt	ABRIC_TYPEtpsDehedFWLogType =tpsDeV muent
Pe32sul  = hedFWTraceQuery(Rl_er(ACTIVE     R,_EVIC,_&psDehedFWLogTypeKS;aPF((
	}

	POKD!= e32sul ONS][GP_IS_FEAT32sul ntETA[G((P0 - trace, 1 - tbiviceA (dl0D== psDek mueKiC{eCeBITEREN_SET(psDehedFWLogType, V_ERROR_LOULTYPE_TRACE
oC }egsBaseKM, RGined(NO_TBI _TYERFACE
	p

	d  (dl1D== psDek mueKiC{eCeBITEREN_UNSET(psDehedFWLogType, V_ERROR_LOULTYPE_TRACE
oC }eg	    RX

	dvoid *pvRegsBaseKM;
#enui32PdstHSSSSSSSSSu****Inv mem parame
	p %u specifi		EPVRyet 
W log type OP_CPU_."/
wwduleMISR(4GpuStatCppsDek mueKSoC supports full AIMG_UINT32 ui32AppHintETA[Ge32sul  =  _CR_Sdle  DebugAee(WLogKM(EVIC,_ACTIVE     R,_psDehedFWLogTypeKS;a_IS_FEAT32sul ntPOion=%u,t
				/* Re-schedQueryFWPoisonOnFree
contavwn request as it was deferred. */
 staaaaaacontavregats dPrivecp/
 staaaaaaABRIate !*pbk mueKi:bIMode;
	L_STATE_IDLE
#define GPU_
Mode;
	L_STATE_IDLE
)_ACTIVE     RGXFWIF_GPU_Uat*pbk muePU_
Mode;
	C_AXI_MASKFULPOISON_ON_FREED== ACTIVInfo->_NONFWdoisonOnFreeiFlaKvos?cy value
vos:cy vaFALSE; ui32Regi well calibraton=%u,t
				/* Re-schedAee(WPoisonOnFree
contavwn request as it was deferred. */
 staaaaaacontavregats dPrivecp/
 staaaaaaABRIate !bk mueKi:bIMode;
	L_STATE_IDLE
#define GPU_
Mode;
	L_STATE_IDLE
)_ACTIVE     RGXFWIF_GPU_U	ACTIVInfo->_NONFWdoisonOnFreeiFlaPU_bk mue
 st?  _CR_SOC_AXI_MASKFULPOISON_ON_FREE
 st: 0ULS; ui32Regi well calibratTE
 *chedring(RGXFW_M
viceUE_SUPPORTED
hedring(RGXFW_M(wn request as it wa aaaaas deferred. */
				OSScheduleMIABRIate !!!!!!!!!!!!!!!!!bEnableSignE_uriCKCCBe/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONSignE_uriCKCCBeBufSiz*/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONHWPerfFWBufSiz*KB/
				OSScheduleMIABRIC_TY64  ttttttttttttt_N64HWPerfFRl_er/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONhedFWAlignCKCCBeArrLe gth/
				OSScheduleMIABRIC_TYPEttttttttttttttt* psDehedFWAlignCKCCBe/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NON
	
	 *iFlag/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONLogType,
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONFRl_eriFlag/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONJonesD_ACE_LeadM/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONHWRDebugDumpLimit/
				OSScheduleMIABRIC_TYPE:v{	 heSnKillingCtl/
				OSScheduleMIABRIC_TYPE:v{	 * psDeTPUTrilDBGarFraceadM/
				OSScheduleMIABRIC_TYPE:v{	 * psDeUSRMNum32OS, g/
				OSScheduleMIABRIC_TY64:v{	 * ps64UVBRMNum32OS, g/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONHWPerf
		  ersDeqaSiz*/
				OSScheduleMIse RRi32OWER		{LAND_CONFAT3GXRDP    Isla_2ND_
	
	/
				OSScheduleMISW PERF_CONFAAAAAAAAAAAAAe(RGXFW_MPerf/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NON
	
	 *iFlagExt/
				OSScheduleMIABRIC_TYPEttttttttttttttt_NONAvailCE_LP  UingseadM/
aaaaABRIC_TYPEttttttttttttttt_NONFwOs
f*iFlagKi:bIMode;
	 Re-sce]));
hrOT_IMPLEL_STATE_IDLE
#define GPU_
Mode;
	L_STATE_IDLE
)ACTIVE     RGXFWIF_GPU_U	ABRIate !bEnableFWdoisonOnFree ncy vaFALSE; 
Pt is being pAeeup(RGXFW_M(ATION)
	/* ProUduleMISR(psDevInfo->pvAPMIbEnableSignE_uriCKCCBe/
UduleMISR(psDevInfo->pvAPMI_NONSignE_uriCKCCBeBufSiz*/
UduleMISR(psDevInfo->pvAPMI_NONHWPerfFWBufSiz*KB/
UduleMISR(psDevInfo->pvAPMI_N64HWPerfFRl_er/
UduleMISR(psDevInfo->pvAPMI_NONhedFWAlignCKCCBeArrLe gth/
UduleMISR(psDevInfo->pvAPMI psDehedFWAlignCKCCBe/
UduleMISR(psDevInfo->pvAPMI_NON
	
	 *iFlag/
UduleMISR(psDevInfo->pvAPMI_NON
	
	 *iFlagExt/
UduleMISR(psDevInfo->pvAPMI_NONFwOs
f*iFlag/
UduleMISR(psDevInfo->pvAPMI_NONLogType,
UduleMISR(psDevInfo->pvAPMI_NONFRl_eriFlag/
UduleMISR(psDevInfo->pvAPMI_NONJonesD_ACE_LeadM/
UduleMISR(psDevInfo->pvAPMI_NONHWRDebugDumpLimit/
UduleMISR(psDevInfo->pvAPMI_NONHWPerf
		  ersDeqaSiz*/
UduleMISR(psDevInfo->pvAPMI_NONKillingCtl/
UduleMISR(psDevInfo->pvAPMI psDeTPUTrilDBGarFraceadM/
UduleMISR(psDevInfo->pvAPMI psDeUSRMNum32OS, g/
UduleMISR(psDevInfo->pvAPMI ps64UVBRMNum32OS, g/
UduleMISR(psDevInfo->pvAPMIT3GXRDP    Isla_2ND_
	
	/
UduleMISR(psDevInfo->pvAPMIT(RGXFW_MPerf/
UduleMISR(psDevInfo->pvAPMI_NONAvailCE_LP  UingseadMr, f (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsListHSSSSSSSSSu _CR_Sdlering(RGXFW_MKM:ng pAeeup(RGXFW_M in.
		E(%u)RSRVGvInfo;
	plist_ii64MingoE_Lin.
ed_ieit_fRGXFW_M, M}t
PFULL!
	}

	P NULL)
		{
			OSSchedule
	}

	OP_CPU_R
	}

	pHa_2lersC_TYPE(APPH_TYCui3EnableLogGroup/
wwduleMISR(psDevInfo->pvAPMISRDevInfo hedFWTraceQuery(Rl_er/
wwduleMISR(psDevInfo->pvAPMISRDevInfo hedFWTraceAee(Rl_er/
wwduleMISR(psDevInfo->pvAPMISRDevInfo ATION)
	/* ProUUduleMISR(psDevInfo->pvAPMISRDDDDDDDirmwar;ate
	}

	OP_CPU_R
	}

	pHa_2lersC_TYPE(APPH_TYCui3(RGXFW_MLogType,
UwduleMISR(psDevInfo->pvAPMISRDevInfo hedFWTraceQueryLogType,
UwduleMISR(psDevInfo->pvAPMISRDevInfo hedFWTraceAeeLogType/
wwduleMISR(psDevInfo->pvAPMISRDevInfo ATION)
	/* ProUUduleMISR(psDevInfo->pvAPMISRDDDDDDDirmwar;at}t
PbEnableFWdoisonOnFree nc
	}

	PAPPH_TYCENis cFWdOISONONFREEulatranionOP_CPU_R
	}

	pHa_2lersate (APPH_TYCui3EnableFWdoisonOnFree/
wduleMISR(psDevInfo->pvAPMISRDevInfohedQueryFWPoisonOnFree/
wduleMISR(psDevInfo->pvAPMISRDevInfohedAee(WPoisonOnFree/
wduleMISR(psDevInfo->pvAPMISRDevInfoATION)
	/* ProUduleMISR(psDevInfo->pvAPMISRDDDDDDDrmwar;aU	ACTIVInfo->_NONFWdoisonOnFreeiFlaPU_bEnableFWdoisonOnFree
 st?  _CR_SOC_AXI_MASKFULPOISON_ON_FREE
 st: 0ULS; ui32Regi well calib
in.
ed_ieit_fRGXFW_M: tranRASSERT;list_is_empty(&psDev;2e_IS_FEATURE_SUPPORTE SePVR PPro.hDurn f 0x(driDdeclareclMMD*yP:wLay6y
				/* Re-schedAif

UFOBf

B	break;

	case ult:Meqe>6vKi:yP:h/
 staaaaaa er
}

o}

DESC *eqeMemDescroUUUUUUUUU ABRIC_TYPEts psSyncPrimVnfo-/
wwa*UUUUU ABRIC_TYPEts psSyncPrimBf

BSiz*Ki:bIMode;
	L_STATE_IDLE
#define G_UTRegion][ui32OS]));
hruV_ERROR_TATEVIRTnDDR p(RGXFW_Mnfo-_U	ABRIer
}

oSIZRCTcuiUFOBf

BSiz*PU_n PVRSRABRIC_TYPE)_U	ABRIer
}

oALIGN TcheSnUFOBf

BAlignPU_n PVRSRABRIC_TYPE)_U	ABRIC_TYPEtpsDeCoherencyiFlaPU_0;aU	ACTIVInfoPU_ACTIVE     RGXFWIF_GPU_UateLMAiz*Pats alignPW_M 'expatsed' becapRa we req; /*van ExyP:yalignPWif

eclMMDiceAt is beinDonmemExyP:yalignAdjustAiz*AndAlign(DonmemGetHeapLog2PageAiz*>sLayerParams.(RGXFW_MMainHeap)roUUUUUUUUUU&uiUFOBf

BSiz*roUUUUUUUUUU&uiSnUFOBf

BAlignNS; u (dllist_is_empty(&psDevInfo->goE_LeUMraTA[G (dlpty(&praramsSnoooND_OfIF_GPUCache;psefiProcessGXFsefi
	
	 *c &&
{	pty(&praramsSnoooND_OfCPUCache;psefiProcessGXFsefi
	
	 *cchedulesk =CoherencyiFlaPU_ _CR_SOC_AXI_MASKFULCACH
	COH = 0TulPES;

	dvoid *sk =CoherencyiFlaPU_ _CR_SOC_AXI_MASKFULUNCACH
D, M}t
Pt is beinDonmemFwAif

ecpExyP:yablM(ATION)
	/* ProUUUUUUUUUUuiUFOBf

BSiz*roUUUUUUUUUUuiSnUFOBf

BAlignroUUUUUUUUUU _CR_SOC_AXI_MASKFULest as SKFU(PMPVRS PROTECT) | w			UUUUUU _CR_SOC_AXI_MASKFULKERNELLCP
	MAPPis c | w			UUUUUU _CR_SOC_AXI_MASKFULZERO_ON_XI_MA | w			UUUUUU _CR_SOC_AXI_MASKFULGP
	READis c | w			UUUUUU _CR_SOC_AXI_MASKFULGP
	WRIDAis c | w			UUUUUU _CR_SOC_AXI_MASKFULCP
	READis c | w			UUUUUU _CR_SOC_AXI_MASKFULCP
	WRIDAis c | w			UUUUUUsk =CoherencyiFlaroUUUUUUUUUU"FwExUFOBf

B"roUUUUUUUUUUqeMemDescNS;f (dllist_is_empty(&psDevInfo->goE_LeUMraTA[Gt is being pAee(RGXFW_Mnfo-ess(&p(RGXFW_Mnfo-, eqeMemDescrv0, RSW FWnDDR_SKFULNONEr;atranRGOTO_  =ui32Pu
#if dohe1)_Uat*ppsSyncPrimVnfo-PU_A(RGXFW_Mnfo-._NONnfo-;at*ppsSyncPrimBf

BSiz*PU_TRUNCATE_64BITS_TO_32BITS(uiUFOBf

BSiz*NS; ui32Regi well calib
e1:[GDonmemFwUnmapAndFree
sLayerPar& *qeMemDescNS;e0:2e_IS_FEATURE_SUPPORTE SePVR PPro.hDurn f 0x(driDdeclareclMMD*yP:wLay6yregatV_ERreeUFOBf

B	break;

	case ult:Meqe>6vKi:yP:h/
 staaaaer
}

o}

DESC *qeMemDescNi:bIMode;
	L_STATE_IDLE
#define GPU_ACTIVE     RGXFWIF_GPU_UateL
 sIfV64Gpsarams has snoooND_ivfV64GpR PPro cacheV64GGpG_BOUFO bf

B
 smight be  GpG_BOcacheVso we ne		EPVRflush itivuaDbeurnM ireeND_
 sremempuUtiA[GPW4GGpG_BOR PPro isDbeND_ishutdown/d /*roy		Ewe don'tOcar*PatyuUte.[GPSeveral ne>pvDary meqaRstrudau-esEPVRissu*PaRflush    u d /*roy		[GPalready.
	er t (dlpty(&praramsSnoooND_OfIF_GPUCache;psefiProcessGXFsefi
	
	 *c &&
{	psefiProcessGXsDonStecpu_empty(&ps
	case STADATDEINITvInfo->V_ERROR_KCCB_CMD sFlushInv mCmd;ate
	}

	][ui32OS]));
hru	ABRIC_TYPEtpsDekCCBCommatsSlo nt
PteLMAchedule EounSLCRflush commats ...CicegsBaseKM, RGXe "rg)SfNe "rCOM, MEWITHSKFUS(                      , "SubmitiSLCRflush ats inv memecp"
	i	 psDev		sFlushInv mCmd.eCmdType =tV_ERROR_KCCB_CMD_SLCFLUSHNT32 ;v		sFlushInv mCmd.uCmdDeqa.sSLCFlushInv mDeqa._N64Siz*PU_0;v		sFlushInv mCmd.uCmdDeqa.sSLCFlushInv mDeqa._N64nfo-essPU_0;v		sFlushInv mCmd.uCmdDeqa.sSLCFlushInv mDeqa.bInv m ncy value
		M	sFlushInv mCmd.uCmdDeqa.sSLCFlushInv mDeqa.bDM
	
text ncy vaFALSE; M	sFlushInv mCmd.uCmdDeqa.sSLCFlushInv mDeqa.ps
	
text._NONnfo-D=NUMr[GPt is being pAendCommatsWithPowL

BAndGetKCCBSlo 
sLayerPar&oUUUUUUUUUUUUUU &sFlushInv mCmd&oUUUUUUUUUUUUUU                       ,oUUUUUUUUUUUUUU &psDekCCBCommatsSlo r, fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"****F		OSSpNUOschedule SLCRflush commats with eif de(%u)RSRVGOSRDDDDDDD4GpuStatCRVG	SRDDDDDDDlist_ii64Min			{

	dvosDevIneLMWangDurn EounSLCRflush E_LcompleteviceAAPt is being pWangForKCCBSlo Upmequ
RDWARE) && _NDekCCBCommatsSlo ,                       64Minnb->u
#if defined(PDUMP)
	pssDevIn*pvResLayerParams.ui32Pdrin*{SRDevInfo"****SLCRflush ats inv memecp abP:yud	with eif de(%u)RSRVGOOSRDDDDDDD4GpuStatCRVG	OSRDDDDDDDlist_ii64Minn}eAAPt
	d  (dlunlikely>sLayerParams_NONKernelCCBRtnSlo s[_NDekCCBCommatsSlo ] &oUUUUUUU tV_ERROR_KCCB_RTN_SLOT POLL_FAILUt ffrussDevIn*pvResLayerParams.lide>6wohe)
	{FW pollTse a{HW(operat6MMDu		OSSRENCY  0x2 {SoC s*}
METAM}t
PV_EUnsee(RGXFW_Mnfo-ess(qeMemDescNS;fDonmemFwUnmapAndFree
sLayerPar& ACMemDescNS;PORTE
	ayeDeringV_E
iceUE_SUPPORTED ayeDeringV_E	break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTIL_STATE_IDL		
#define GPU_
Mode;
	L_STATE_IDL
)ACTIVE     RGXFWIF_GPU_U	UE_SUPPORTED		FS]));
hrO
	case }

ORYE_IDL		
#defi,puUtine G_UTABRIC_TYPE:vpsDeTemp=UMr
PE;
	!#define G   fo->_S Can happGGpE;
D cring	IMGin.
		EiceAApvResLayerParams.ui32Pd "ayeDeringV_E: NullTefine G"KSoC supports full AalibaTA[G (dlACTIVInfo->pESSAGE,
OsringN  fo->KM_SETTIOf   NECTION(OFFLINE& ACTIVInfo)MraTA[Gt is beinIF_GPUDepBridgeDering;psefiInfo->sefiFeE_uriCfg._N64FeE_urisr;atranRLOUL  =ui32Pu
#if doheIF_GPUDepBridgeDering"
	i	if	seKM, RGXe "rg)SDonmemIntFreeDefBackND_Page(ATION)
	/* ProUUUUUUUU&ACTIVE     RGXsDummyPageroUUUUUUUUe "MY_PAGENS;fDonmemIntFreeDefBackND_Page(ATION)
	/* ProUUUUUUUU&ACTIVE     RGXsDevZeroPageroUUUUUUUUeATEZERO_PAGENS;_UINT32GROR RGXSetPoXfull AFORse UNLOADL  =BADLSTADA) t (dlpty(&pGetpty(&pDeqa()GXsSerPProsStecpu_empty(&psSERcaseS STADATDevInfo->OSAtomicWrite(&ACTIVE     RGXsDummyPage.atRef
		  er& 0NS;AApvReUNREF = 0CEi32AppHETER(psDeTempF_GPES;

	dv#

	dvoid */*DeletevEounDummy page relequd	ie GviceAvpsDeTempPU_
ABRIC_TYPE)OSAtomicRead(&ACTIVE     RGXsDummyPage.atRef
		  err, fSb->u0	_emsk =TempF	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"****Dummy page referencsL r	  er  denon zeroe(%u)RSRVGOSRDDDDDDD4GpuStatCRVG	SRDDDDDDDsk =TempFSoC s*pvReASSERT;0e);f
			ES		    RGX/*DeletevEounDummy page relequd	ie GviceApsDeTempPU_
ABRIC_TYPE)OSAtomicRead(&ACTIVE     RGXsDevZeroPage.atRef
		  err, fb->u0	_emsk =TempF	pfo->sKCCBDeferredCommandsListH	vInfo"****Zero page referencsL r	  er  denon zeroe(%u)RSRVGOSRDDD4GpuStatCRVG	SRDDDsk =TempFSoC vRegsBaseKM, RGXe "rg)Sb->urmwa	_emACTIVE     RGXsDummyPage.hPInfoPgF	pfo->se "rCOM, ME(" is bedummy page ha_2le  destilleax(dvp"
	iaTA[G (dlrmwa	_emACTIVE     RGXsDevZeroPage.hPInfoPgF	pfo->se "rCOM, ME(" is beZero page ha_2le  destilleax(dvp"
	iaTA		    RGX/*T	PVf

B type ne		EPVRbe dispatchstype here becapRa itOcan be aS;
#end irom MISR (Z-buffer) path iceAOSL

BD /*roy(ACTIVE     RGXsDummyPage.psPgL

BK, [G((PD /*royvEounzero page f

B iceAOSL

BD /*roy(ACTIVE     RGXsDevZeroPage.psPgL

BK, [G((PUn;
	}

	p debug req; /*vnotifi	rstfRGst as remyi ruldY  RT_2Tse anythND_F  timXg pr bugDeiing;psefiInfoK, [
>_S Cancelvnotifi
eclMMsEPVRthNsOR PPro er tran(&pUn;
	}

	pCmdCompleteNotify(ACTIVE     RGXhCmdCompNotifyNS;aACTIVE     RGXhCmdCompNotifycinEVICE_CR_S	/* PDe-ieiti mes	  Gprevers	 ordF(,Vso stage 2   NDaNsOundonatfRGst.DeviceA (dlACTIVInfo->bD cring2Dona)void *psDevInfo->bD cring2Dona ncy vaFALSE; 
PPt is bein well TQUnloadShadF(s(ACTIVE     Rr, fSb->u
#if defined(PDUMP)
	psDevInTED(psDevInfo, META[yP:wState;
	/* snapshot fru>grega) SysUnintaallTIVE  LISRRACTIVInfo->pvLISRDeqar, fSgrega) OSUnintaallMISRRACTIVInfo->pvMISRDeqar, fSgrega) OSUnintaallMISRRACTIVInfo->hde->pvDQue; /MISRr, fSb->uACTIVInfo->pvAPMISRDeqaP!Firmwar	psDevIngrega) OSUnintaallMISRRACTIVInfo->pvAPMISRDeqae);f
				    F((P!* snapshot   timXP((PRemovepG_BOR PPro irom G_BOp     matagerviceAvt is bein well RemoveP    TIVE  (ACTIVE     Rr, fSb->u
#if defined(PDUMP)
	psDevInTED(psDevInfo, META[a*psDevInfo->sfnGetGpuUtilStecscinEVICE_FAOSL

BD /*roy(ACTIVInfo->hGPUUtilL

BK, [GP((PFree DVFS Table iceAv (dlACTIVInfo->pEGpuDVFSTable !Firmwar	psDevInOSFree,pulACTIVInfo->pEGpuDVFSTableSoC s*ACTIVInfo->pEGpuDVFSTable inEVICE_FA} [GP((PDe-ieitPFreemests/ZBuffers...CiceAAOSL

BD /*roy(ACTIVInfo->hL

BFreeLest)E_FAOSL

BD /*roy(ACTIVInfo->hL

BZSBufferK, [gsBaseKM, RGined(NO_WORKLOADLESTIMATION)[GP((PDe-ieitPwork  /*imat6MMDf

B iceAAOSL

BD /*roy(ACTIVInfo->hWorkEstL

BK, 		    RGXG((PUn;
	}

	p MMU relequd	stuffviceAvt is being pMMUring_Un;
	}

	p(ACTIVE     Rr, fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"ayeDeringV_E: F		OSSpg pMMUring_Un;
	}

	p>u0x%x)RSRVGOSRDDDDDDDlist_ii64MinnTED(psDevInfo, METAETA[G((PUnMap RegsviceA (dlACTIVInfo->pvRegsBa	dKM !Firmwar	p{[yP:wState;
	/* snapshot fru>OSUnMapPhysToLin((regatE_forro e) ACTIVInfo->pvRegsBa	dKMroUUUUUU ACTIVInfo->_NONRegSiz*roUUUUUUo _CR_SOC_AXI_MASKFULCP
	UNCACH
DK, 		    F((P!* snapshot   tis*ACTIVInfo->pvRegsBa	dKM inEVICE_F}
[yP:w0F((Pnot re;
#end atRthNsO*imeviceA (dlACTIVInfo->hTimerchedulet is beinOSRemoveTimerlACTIVInfo->hTimerc, fSb->u
#if defined(PDUMP)
	psDevInfo->sLayerParams.ui32PdevInSRDevInfo"ayeDeringV_E: F		OSSpE_LupmovepGimer"i64MinnTED(psDevInfo, METAE	ACTIVInfo->hTimer inEVICE_F}
		    RGX#defi,puUtine G in&ACTIVE     RGXsDev,puUtine G_UmXg pr ringHeaps(ACTIV,puUtine GNS; u (dlACTIVInfo->pESSAGECodeMemDescN  fo->_S Free fw cessPiceAApe "rCOM, ME("FreeND_V
W cessPmpuUti"
	iafDonmemRelea	dDonVirtnfo->sLayerParams.dle  CodeMemDesc
	iafDonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGECodeMemDescNS;aaACTIVInfo->pESSAGECodeMemDesccinEVICE_ETAE (dlACTIVInfo->pESSAGEDeqaMemDescN  fo->_S Free fw meqaRiceAApe "rCOM, ME("FreeND_V
W meqaRmemUti"
	iafDonmemRelea	dDonVirtnfo->sLayerParams.dle  DeqaMemDescNS;*GDonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGEDeqaMemDescNS;**psDevInfo->s.dle  DeqaMemDesccinEVICE_ETAE (dlACTIVInfo->pESSAGECorsmemOodeMemDescN  fo->_S Free fw cere mem cessPiceAApe "rCOM, ME("FreeND_V
W cersmem cessPmemUti"
	iafDonmemRelea	dDonVirtnfo->sLayerParams.dle  CorsmemOodeMemDescNS;a*DonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGECorsmemOodeMemDescNS;a*psDevInfo->s.dle  CorsmemOodeMemDesccinEVICE_F}
[G (dlACTIVInfo->pESSAGE,
CorsmemDeqaStorsMemDescN  fo->_S Free fw cere mem meqaRiceAApe "rCOM, ME("FreeND_V
W cersmem meqaRstere memUti"
	iafDonmemRelea	dDonVirtnfo->sLayerParams.dle  ,
CorsmemDeqaStorsMemDescNS;a*DonmemFwUnmapAndFree
sLayerPar& ACTIVInfo->pESSAGE,
CorsmemDeqaStorsMemDescNS;a*psDevInfo->s.dle  ,
CorsmemDeqaStorsMemDesccinEVICE_F}
CR_S	/*  Free G_BOfRGXFW_MEWif

eclMMs.DeviceAV_ERree(RGXFW_M(ATIONInfo)Mrag pr ringD /*royFWKernel,puUti
	
text(ACTIVE     Rr, [gsBaseKM, RGined(NO_32 uiATION)[Gg pP    Tomainr ringStecp(&psDevInfo->sP    TomainStecpK, 		    RGX/ PDe-ieiti mes	 non-R PPro specific (TL) pRarUivfVRIMGR PPro memUtiviceAV_EHWPerfHostDering;psefiInfo)Mrat is beinHTBDering;r;atranRLOUL  =ui32Pu
#if doheHTBDering"K, [G((Pd /*royvEountaallSSpCCBDf

Bs iceAOSL

BD /*roy(ACTIVInfo->hCCBRecoveryLo
BK, AOSL

BD /*roy(ACTIVInfo->hCCBSaallCKCCBL

BK, [G((Pd /*royvEounc	
text mestDf

Bs iceAOSL

BD /*roy(ACTIVInfo->sReg
	
gfig.hLo
BK, AOSL

BD /*roy(ACTIVInfo->hBPLo
BK, AOSL

BD /*roy(ACTIVInfo->hdle  ,
BufringLo
BK, AOSWRL

BD /*roy(ACTIVInfo->hd	  erCtxLestLo
BK, AOSWRL

BD /*roy(ACTIVInfo->hComputeCtxLestLo
BK, AOSWRL

BD /*roy(ACTIVInfo->hTransferCtxLestLo
BK, AOSWRL

BD /*roy(ACTIVInfo->hTDMCtxLestLo
BK, AOSWRL

BD /*roy(ACTIVInfo->hKickSyncCtxLestLo
BK, AOSWRL

BD /*roy(ACTIVInfo->h,puUti
txLestLo
BK, AOSSpinL

BD /*roy(ACTIVInfo->hL

BKCCBDeferendCommatssLest)E_FOSWRL

BD /*roy(ACTIVInfo->hCommon
txtLestLo
BK, eA (dlACTIVInfo->hr bugFaultInfoLo
B !Firmwar	p{[AAOSL

BD /*roy(ACTIVInfo->hr bugFaultInfoLo
B
	iaTA[G (dlGetInfoPager bugFFlagKM() & DEBUGernelMMUCPAGE_FAULT_DEBUGeENis cDONS][GP (dlACTIVInfo->h,MU
txUn;
	Lo
B !Firmwar	psDevInOSL

BD /*roy(ACTIVInfo->h,MU
txUn;
	Lo
Be);f
			ES
>_S Free R PPro BVNCRstrND_ViceA (dlrmwa	_emACTIVInfo->sefiFeE_uriCfg.pszBVNCStrND_r	p{[AAOSFree,pulACTIVInfo->sefiFeE_uriCfg.pszBVNCStrND_rE_F}
CR_SPDeAif

ecp R PPe GviceAOSFree,pulACTIVInfor;aU	ACTIVProcessGXFWIF_GPUPinEVICE_CRi32Regi well calibratgsBaseKM, RGXe "rg)on=%u,t
				/* Re-schedResetPDump	break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTIL_STATE_IDLE
#define GPU_
Mode;
	L_STATE_IDLE
);psefiProcessGXFWIF_GPUr;aU	ACTIVInfo->bDumpedKCCBCtlAlready ncy vaFALSE; 
Pi32Regi well calibra		    F((PXe "r  tim:wLay6yINLINE er
}

oHEAP_BLUEPRINT _blueprNDt_ieit
ABRICnap *nam*roUUABRIC_TY64 heap_ba	droUUABRIer
}

oSIZRCTcheap_le gth/
UUABRIer
}

oSIZRCTcheap_I_Cirved_r2OS, _le gth/
UUABRIC_TYPEtlog2_imyP:y_alignmentKi:bIregats vOP_CPU_StecpuinEVICE_FABRIC_TYPEtpsDeOP_CPU_Defaul  =  _CR_SPAPPH_TYCGENERALNON4KHEAPPAGESIZRE_FABRIC_TYPEtpsDeGeneralNon4KHeapPageAiz*_UTABRIC_TYPE:psDeOSL
g2PageAhifteinOSGetpageAhift()_UTABRIC_TYPE:psDeOSPageAiz*_UbIer
}

oHEAP_BLUEPRINT bein{[AA.pszNam*einnam*roUU.sHeapBa	dnfo-._Nnfo-D=Nheap_ba	droUU._NHeapLe gthD=Nheap_le gth/
UU._NR_Cirved32OS, Le gthD=Nheap_I_Cirved_r2OS, _le gth/
UU._NL
g2DeqaPageAiz*eing pHeapDe_DPFPageAiz*>psDeOSL
g2PageAhift)/
UU._NL
g2ImyP:yAlignmenteinlog2_imyP:y_alignment/
U}nt
P_NONOSPageAiz*PU_
1 << psDeOSL
g2PageAhift), [G((PAnyNheap le gth shruldYatDfea	t matchsOS page siz*Patirememieimum or	/* PaRmul iple  feOS page siz*PiceA (dl(b._NHeapLe gthD=
		  ||dlb._NHeapLe gthD
	P_NONOSPageAiz*P- 1ggF	pfo->sKCCBDeferredCommandsListH	vInfo"****Inv mem Heap \"**\"MAiz*: %llu>u0x%llx)RSRVGOSRDDD4GpuStatCRVG	SRDDDb.pszNam*, b._NHeapLe gth, b._NHeapLe gth)NS;AApvReBDeferredCommandsListH	vInfo"Heap Aiz*PshruldYalways be a non-zero v muePats a ed stSRDev"mul iple  feOS PageMAiz*:%uu0x%x)RSRVGOSRDDD_NONOSPageAiz*,D_NONOSPageAiz*)NS;AApvReASSERT;b._NHeapLe gthD>=D_NONOSPageAiz*)E_F}
C
ApvReASSERT;b._NR_Cirved32OS, Le gthD%Ise RHEAP_RESERVEDoSIZRCGRAEVIARITYD=
		 Mr
PE;
	!OSStrND_NVrepa_M(nam*rIse RGENERAL_NON4KRHEAP_ID ME,_n PVRSRse RGENERAL_NON4KRHEAP_ID MEggF	pfo->OSCreateKMOP_CPU_Stecp(&pvOP_CPU_Stecp)E_FAOSGetKMOP_CPU_C_TYPE(pvOP_CPU_Stecp, GeneralNon4KHeapPageAiz*/
wwww&psDeOP_CPU_Defaul ,_&psDeGeneralNon4KHeapPageAiz*)E_FAswitchs(psDeGeneralNon4KHeapPageAiz*)	psDevInca	d_
1<<se RHEAP_4KBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_4KBCPAGE_SHIFT;
wwwwbreak;evInca	d_
1<<se RHEAP_16KBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_16KBCPAGE_SHIFT;
wwwwbreak;evInca	d_
1<<se RHEAP_64KBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_64KBCPAGE_SHIFT;
wwwwbreak;evInca	d_
1<<se RHEAP_256KBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_256KBCPAGE_SHIFT;
wwwwbreak;evInca	d_
1<<se RHEAP_1MBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_1MBCPAGE_SHIFT;
wwwwbreak;evInca	d_
1<<se RHEAP_2MBCPAGE_SHIFT):
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_2MBCPAGE_SHIFT;
wwwwbreak;evIndefaul :
wwwwb._NL
g2DeqaPageAiz*eing pRHEAP_16KBCPAGE_SHIFT;

vIn*pvResLayerParams.ui32Pdrin*{GOS"Inv mem OP_CPU_ GeneralAltHeapPageAiz* [%d] v mue,D_sND_V16KB"roUUUUUUtpsDeOP_CPU_Defaul i64Minnwbreak;evI}[AAOSFreeKMOP_CPU_Stecp(pvOP_CPU_Stecp)E_FTA[Gi32RegibibratgseKM,  INITRHEAP(NAME) \
do { \
	eqe>6vKi:,puUtiHeapCurs bein_blueprNDt_ieit
 \
	->V_E_ ## NAME ## RHEAP_ID ME,_\
	->V_E_ ## NAME ## RHEAP_BASE,_\
	->V_E_ ## NAME ## RHEAP_SIZR,_\
	->V_E_ ## NAME ## RHEAP_RESERVEDoSIZR,_\
	->	 M_\
	qe>6vKi:,puUtiHeapCurs b++M_\
} while (0)atgseKM,  INITRFW	MAIN_HEAP(LL)
, FW  RE) \
RDDDdo { \
	eqe>6vKi:,puUtiHeapCurs bein_blueprNDt_ieit
 \
	->VGOSRDDDRe RFIRMhot 	MAIN_HEAP_ID ME,_\
	->VGOSRDDDRe RFIRMhot 	 ## LL)
 ## RMAIN_HEAP_BASE,_\
	->VGOSRDDDRe RFIRMhot 	 ## FW  RE ## RMAIN_HEAP_SIZR,_\
	->VGOSRDDD0,F((PN_Lupserved sparo ie anyV
W heaps ic_\
	->VGO	 	 M_\
	->VGOSRDDDqe>6vKi:,puUtiHeapCurs b++M_\
RDDD} while (0)atgseKM,  INITRFW	CONFIG_HEAP(LL)
) \
RDDDdo { \
	eqe>6vKi:,puUtiHeapCurs bein_blueprNDt_ieit
 \
	->VGOSRDDDRe RFIRMhot 	CONFIG_HEAP_ID ME,_\
	->VGOSRDDDRe RFIRMhot 	 ## LL)
 ## RCONFIG_HEAP_BASE,_\
	->VGOSRDDDRe RFIRMhot 	CONFIG_HEAP_SIZR,_\
	->VGOSRDDD0,F((PN_Lupserved sparo ie anyV
W heaps ic_\
	->VGOSRDDD0 M_\
	->VGOSRDDDqe>6vKi:,puUtiHeapCurs b++M_\
RDDD} while (0)atgseKM,  INITRHEAP_NAME(STR, NAME) \
do { \
	eqe>6vKi:,puUtiHeapCurs bein_blueprNDt_ieit
 \
	->V_E_ ## STR ## RHEAP_ID ME,_\
	->V_E_ ## NAME ## RHEAP_BASE,_\
	->V_E_ ## NAME ## RHEAP_SIZR,_\
	->V_E_ ## STR ## RHEAP_RESERVEDoSIZR,_\
	->	 M_\
	qe>6vKi:,puUtiHeapCurs b++M_\
} while (0)atP:wLay6y
				/* Re-schedringHeaps(GPU_UTIL_STATE_IDLE
#define GroUUUUUUUU 
	case }

ORYE_IDLE
#dNew,puUtine GroUUUUUUUU ABRIC_TYPEts psDeL
g2DummyPgSiz*Ki:bIer
}

oHEAP_BLUEPRINT eqe>6vKi:,puUtiHeapCurs b;bIregats vOP_CPU_StecpuinEVICE_FABRIC_TYPEtpsDeOP_CPU_Defaul  =  _CR_SPAPPH_TYCGENERALNON4KHEAPPAGESIZRE_FABRIC_TYPEtpsDeGeneralNon4KHeapPageAiz*_U
	qeNew,puUtine GGXFsefiKi:,puUtiHeapeinOSAif

,puln PVRSRer
}

oHEAP_BLUEPRINT) *chedRMAXRHEAP_IDr, fb->uqeNew,puUtine GGXFsefiKi:,puUtiHeapeiFirmwar	p{[AAsKCCBDeferredCommandsListHSSSSSSSSSuhedRe	}

	pIF_GPUP: F		OSSpE_LWif

 memUtivurn er
}

oHEAP_BLUEPRINT"i64MingoE_LeUMraTA[G((PGet G_BOpage siz*Purn Eoundummy page irom G_BONON4KNheap apphPU_ iceAOSCreateKMOP_CPU_Stecp(&pvOP_CPU_Stecp)E_FOSGetKMOP_CPU_C_TYPE(pvOP_CPU_Stecp, GeneralNon4KHeapPageAiz*/
www&psDeOP_CPU_Defaul ,_&psDeGeneralNon4KHeapPageAiz*)E_Fs psDeL
g2DummyPgSiz*einExaceLog2(psDeGeneralNon4KHeapPageAiz*);
AOSFreeKMOP_CPU_Stecp(pvOP_CPU_Stecp)E_[G((PIeiti mes	 G_BOheaps ic
	qe>6vKi:,puUtiHeapCurs bPU_ACNew,puUtine GGXFsefiKi:,puUtiHeapE_[GINITRHEAP(GENERAL_SVM)_UTANITRHEAP(GENERAL)_UTANITRHEAP(GENERAL_NON4K)_UTANITRHEAP(PDS  )
DARS)_UTANITRHEAP(USC  )
)_UTANITRHEAP(TQ3D2AppHETERS)_UTANITRHEAP(SIGNALS)_UTANITRHEAP(COMPON ME_CTRL)_UTANITRHEAP(FBCDC)_UTANITRHEAP(FBCDC_IARGE)_UTANITRHEAP(PDSE_IDIRECTLSTADA)_UTANITRHEAP(TEXlMMUCSTADA)_UTANITRHEAP(TDM_TPU_YUV	COEFFS)_UTANITRHEAP(VISIBILITY_T	OSSE_[G((Pvulkan cap_uriLupplay bufferNheap ic
	INITRHEAP_NAME(VK_CAPT_REPLAY_BUF, VK_CAPT_REPLAY_BUF Mr
PE;
	
	}

	P NULL)
		{
			OSScheduleINITRFW	CONFIG_HEAP(			OSS;uleINITRFW	MAIN_HEAP(			OS, PVRSF_GPES;

	dveduleINITRFW	MAIN_HEAP(HOOS, PVRSF_GPeINITRFW	CONFIG_HEAP(HOOSrE_F}
CR_SPset G_BOheap  r	   ic
	qeNew,puUtine GGX_NONHeapCr	   U_
ABRIC_TYPE)(qe>6vKi:,puUtiHeapCurs bP-_ACNew,puUtine GGXFsefiKi:,puUtiHeap);
 tranRASSERT;psNew,puUtine GGX_NONHeapCr	   <=chedRMAXRHEAP_IDr, CR_S	/*  IGpG_BOnewOheap seeup,Ewe ieiti mes	 2nc	
figurat6MMs:
ww1 - O,  willeb*Purn EounfRGXFW_MEonly (index 1 ie arrayfrussa. ThNsOprNmarily has remOfRGXFW_MEheap ie it.Dewwb. It aiso has addit6MMal g; /*vOSIDOfRGXFW_MEheap(sfruss	- O,ly ifV64GpnumberNof supyP:yOfRGXFW_MEOSIDO> 1rus2 - Ogrfos shalleb*Purn clientsEonly (index 0 ie arrayfrussa. ThNsOhas alle64Gpogrfo clientOheaps ie it.Deic
	qeNew,puUtine GGX_NNumHeapCr
figscin2;
	qeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArrayeinOSAif

,puln PVRSRer
}

oHEAP_CONFIG) *cqeNew,puUtine GGX_NNumHeapCr
figsr, fb->uqeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArrayeiFirmwar	p{[AAsKCCBDeferredCommandsListHSSSSSSSSSuhedRe	}

	pIF_GPUP: F		OSSpE_LWif

 memUtivurn er
}

oHEAP_CONFIG"i64MingoE_Le1E_F}
CRqeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[0].pszNam*ein"Defaul  HeapeC	
figurat6MM";
	qeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[0]._NNumHeapsPU_ACNew,puUtine GGX_NONHeapCr	   -DRe RFIRMhot 	NUMBER_OFRFW	HEAPS;
	qeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[0].psHeapBlueprNDtArrayeinACNew,puUtine GGXFsefiKi:,puUtiHeapE_[GqeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[1].pszNam*ein"(RGXFW_M HeapeC	
figurat6MM";
	qeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[1]._NNumHeapsPU_Re RFIRMhot 	NUMBER_OFRFW	HEAPS;
	qeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[1].psHeapBlueprNDtArrayeinAC>6vKi:,puUtiHeapCurs bP-_Re RFIRMhot 	NUMBER_OFRFW	HEAPS;
 fb->use RGETernelMMUCVALUE
sLayerPar& MMU_VERSION)D>=D4cheduleIBRIC_TYPEti, [GP((
//* PZero alleMMU PageMAiz* RangeMCr
fig ;
	}

	psVso remyiwon'tOparay6ipecpuiiDeevi add-essPdecess.MAiz* fieldD=NU aiso means 4KNpage siz*Pwhich NsOR faul Deevi urn alleadd-essesenot withNDe64GprangesaseKM, R byvEouse ;
	}

	ps.
eeviceAAurn (i =NUMFi < ARRAY_SIZRlACTIVInfo->a_N64MMUPageAiz*RangeV mueKS ++i)	psDevInACTIVInfo->a_N64MMUPageAiz*RangeV mue[i]PU_0;v		} [GP((
//* PAeesup remOfRG/*vrangeEonly E_Lupflect G_BONon4K generalOheap.
eevi IGpfu_uri,Ewe cruldYallowRmul iple simul aneoudenon4KNpage siz*s.
eeviceAA (dl**DeGeneralNon4KHeapPageAiz*	_em4*1024)	psDevInnACTIVInfo->a_N64MMUPageAiz*RangeV mue[0]eing pMMUring_GetCr
figRangeV muel**DeGeneralNon4KHeapPageAiz*,oUUUUUUUUUUUUUUUUUUUUUU Ise RGENERAL_NON4KRHEAP_BASE,oUUUUUUUUUUUUUUUUUUUUUU Ise RGENERAL_NON4KRHEAP_SIZRe);f
			ES
#b->use RNUMTIOfined(NOEDO> 1)
PE;
	
	}

	P NULL)
		{
HOOSrcheduleIBRIC_TYPEt_NONOSid, [GP((PCreate addit6MMal rawOfRGXFW_MEheapsviceAAurn (_NONOSidPU_Re RFIRST_RAW	HEAP_OSID;t_NONOSid < se RNUMTIOfined(NOED;t_NONOSid++)	psDevInb->use ring(wRawHeap(qe>6vKi:,puUtiHeapCurs b,t_NONOSid)efined(PDUMP)
	pssDevIn*((Pb->anyVWif

eclMMDin.
s, ireeOpr6vKoudlyVWif

ec R heapsvats abandon ieiti meseclMMDiceA	AAurn (;t_NONOSid >_Re RFIRST_RAW	HEAP_OSID;t_NONOSid--fruss	DevIn*ag pr ring(wRawHeap(qe>6vKi:,puUtiHeapCurs b64Minnw	qe>6vKi:,puUtiHeapCurs b--4Minnw}eAAPngoE_Le1E_F		} [GPG((PApRT_2Taddit6MMal fRGXFW_MEheapsvE_Lho/*vd_DPFr fRGXFW_MEc	
text heap  r
figurat6MMDiceA	AqeNew,puUtine GGXFsefiKi:,puUtiHeapCr
figArray[1]._NNumHeapsP+=N1M [GPG((PadvanceEPVRthBOnext heap iceA	Aqe>6vKi:,puUtiHeapCurs b++M
f
			ES		    F((Puse RNUMTIOfined(NOEDO> 1)  timXi32Regi well calibe1:[GOSFree,pulACNew,puUtine GGXFsefiKi:,puUtiHeap);
e0:2e_IS_FEA full AIMG_UIOUT_OFR}

ORYibratgundef INITRHEAPtgundef INITRHEAP_NAMEatP:wLay6yregatV_Er ringHeaps(
	case }

ORYE_IDLE
#dTIV,puUtine GN
{
#b->use RNUMTIOfined(NOEDO> 1)
PE;
	
	}

	P NULL)
		{
HOOSrcheduleIBRIC_TYPEt_NONOSid, 	Ier
}

oHEAP_BLUEPRINT eqe>6vKi:,puUtiHeapCurs beinAC>6v,puUtine GGXFsefiKi:,puUtiHeapE_[GR_SPDeletevalleg; /*vfRGXFW_MEheapsviceAAurn (_NONOSidPU_Re RFIRST_RAW	HEAP_OSID;t_NONOSid < se RNUMTIOfined(NOED;t_NONOSid++)	psDevIng pr ring(wRawHeap(qe>6vKi:,puUtiHeapCurs b64Minnqe>6vKi:,puUtiHeapCurs b++M
f
			ES		    F((Puse RNUMTIOfined(NOEDO> 1)  timXOSFree,pulACTIV,puUtine GGXFsefiKi:,puUtiHeapCr
figArray);
AOSFree,pulACTIV,puUtine GGXFsefiKi:,puUtiHeap)ibraton=%u,y
				/* Re-schedPhysMemDeiKi:Heapsring;break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTI[ui32OS]));
 =s full AalibaPHYS	HEAPts dPhysHeapE_aPHYS	HEAPLTYPE :HeapTypentUABRIC_TY64 uPhysheapAiz*_UTABRIC_TYPE _NONReg6MMCr	  _UTABRICP
	PHYnDDR sCpuPnfo-_U	ABRIer
	PHYnDDR s>6vPnfo-_U	break;

	case CONFIGMeqe>6vCr
fig U_ACTIVE     RGXFe>6vCr
figE_[G((PIeiti mes	 G_BOobjects pRaSpE_Lmatage G_BOphysical fRGXFW_MEheap ic
	qePhysHeap U_ACTIVE     RGXaqePhysHeap[break;

	case PHYS	HEAPLFW	LOCAL]MratHeapType =s hysHeapGetType(AC hysHeap Mr
PE;
	tHeapType ==s HYS	HEAPLTYPE_UMAr	p{[AAsKCCBDeferredCommMESSAGEohe)
	{FRGXFW_MEphysical heap useseOS rarams memUtiv(UMArRENCY  0x2 {SoC sACTIVE     RGXFfnCreateRamBackedPMR[break;

	case PHYS	HEAPLFW	LOCAL] =s hysmemNewOSRamBackedPMR_GPES;

	dveduleIBRIC_TY64 uRawHeapBa	doC sRA_BASE TchFwCfgSubHeapBa	d,chFwMainSubHeapBa	doC scontavIBRIC_TY64 uN64ExpectedHeapAiz*PU_Re RFIRMhot 	RAW	HEAP_SIZRE_FscontavRA_LENGTH TchFwCfgSubHeapAiz*PPU_Re RFIRMhot 	CONFIG_HEAP_SIZRE_FscontavRA_LENGTH TchFwMainSubHeapAiz*PU_DRe RFIRMhot 	MVRS MAIN_HEAP_SIZRE_[GRsKCCBDeferredCommMESSAGEohe)
	{FRGXFW_MEphysical heap usesef

el memUtivmatageR byvEouvd_DPFr (LMArRENCY  0x2 {SoC s_NONReg6MMCr	   =s hysHeapNumberOf32OS, g(AC hysHeap MrAA (dl**DeReg6MMCr	   > 1)
PsDevInfo->sLayerParams.lide>6wohe)
	{FRGXFW_MEheap currentlyVsupyP:yO1 r2OS, Eonly. ed st		"break;

	case PHYS	HEAPLCP
	LOCALEc	
tains %u r2OS, s. O,ly Eouv1st willeb*PpRaS.RENCY  0x2 , _NONReg6MMCr	  i64Min		
Avt is bein hysHeapReg6MMGet>6vPnfo-(AC hysHeaprv0, &s>6vPnfo-NS;AApvReLOULGOTO_  =ui32Pu
#if dohe hysHeapReg6MMGet>6vPnfo-REN#if dDeiing);	
Avt is bein hysHeapReg6MMGetCpuPnfo-(AC hysHeaprv0, &sCpuPnfo-NS;AApvReLOULGOTO_  =ui32Pu
#if dohe hysHeapReg6MMGetCpuPnfo-REN#if dDeiing);	
Avt is bein hysHeapReg6MMGetAiz*>sL hysHeaprv0, &uPhysheapAiz*NS;AApvReLOULGOTO_  =ui32Pu
#if dohe hysHeapReg6MMGetAiz*REN#if dDeiing);	AApvReLOULGOTO_  =FALSE(uPhysheapAiz*D>=D_N64ExpectedHeapAiz*,oUUUUUUU S"Inv mem fRGXFW_MEphysical heap siz*.REN#if dDeiing);	
Av((PN_wEwe crnstrudavRAspE_Lmatage G_BO
W heaps icC s_RawHeapBa	dPU_n>6vPnfo-._Nnfo-;	
AvE;
	
	}

	P NULL)
		{
			OSSchesDevIneLMG; /*vsubheap layout:MCr
fig + Main iceA	AhFwCfgSubHeapBa	d =tpRawHeapBa	doC s	hFwMainSubHeapBa	d =tpFwCfgSubHeapBa	d +chFwCfgSubHeapAiz*;evI}[AA

	dvosDevIneLMNa(dvp/Hostvsubheap layout:MMain + (opt6MMal MIPSLupserved range) + Cr
fig iceA	AhFwMainSubHeapBa	d =tpRawHeapBa	doC s	hFwCfgSubHeapBa	d =tpRawHeapBa	d + Re RFIRMhot 	RAW	HEAP_SIZR -chFwCfgSubHeapAiz*;evI}[
Avt is bein well CreateReg6MMRA(Fe>6vCr
fig/
wwa*UUUUU n&ACTIVE     RGXpsKernelFwMainMemArena/
wwa*UUUUU nACTIVE     RGXszKernelFwMainRANam*roUUa*UUUUU nsCpuPnfo-._Nnfo-D+ (hFwMainSubHeapBa	d -tpRawHeapBa	d)roUUa*UUUUU nhFwMainSubHeapBa	droUUa*UUUUU nhFwMainSubHeapAiz*,oUUUUUUUUUDD0,oUUUUUUUUUDD"FwMMain subheap"NS;AApvReLOULGOTO_  =ui32Pu
#if dohe well CreateReg6MMRA(FwMain)REN#if dDeiing);	
Avt is bein well CreateReg6MMRA(Fe>6vCr
fig/
wwa*UUUUU n&ACTIVE     RGXpsKernelFwCr
figMemArena/
wwa*UUUUU nACTIVE     RGXszKernelFwCr
figRANam*roUUa*UUUUU nsCpuPnfo-._Nnfo-D+ (hFwCfgSubHeapBa	d -tpRawHeapBa	d)roUUa*UUUUU nhFwCfgSubHeapBa	d,oUUa*UUUUU nhFwCfgSubHeapAiz*,oUUUUUUUUUDD0,oUUUUUUUUUDD"FwMCfg subheap"NS;AApvReLOULGOTO_  =ui32Pu
#if dohe well CreateReg6MMRA(FwCfg)REN#if dDeiing);	
AvACTIVE     RGXFfnCreateRamBackedPMR[break;

	case PHYS	HEAPLFW	LOCAL] =s hysmemNewL

elRamBackedPMR_G[gsBaseKM, RGined(NO_AUTOVZ)
AvE;
	
	}

	P NULL)
		{
HOOSrchesDevIneLM1 MbOcan holdirememaximum amr	   ofNpage yablMsPurn EounmemUtivshaend betweGGpG_BOfRGXFW_MEWts all KMvd_DPFrs:
wwevi  MAX(RAW	HEAP_SIZR) =sPE Mb; MAX(NUMBER_OS) =s8; Total shaend memUtiv= 256 Mb;
wwevi  MMUOobjects re;
#end: 65536 PTEs; 16 PDEs; 1 PCE; iceA	ARA_LENGTH TchMaxFwMmuPageTableSiz*PU_1vi 1024vi 1024M [GPG((PByOR faul pG_BOfRGXFW_MEMMU'sNpage yablMsPW_MEWif

ecnd irom Eountam*ecarvevuaDmemUtivas remOfRGXFW_MEheap.
wwevi If a    ferent ba	d add-essP despecifi		Eurn EoisLupserved range,D_s	 G_BOoPFrri2ND_aseKM,  intaead.CicegsBaseKM, RGXanRAUTOVZ_OVui3I)
	FW	MMU_CARVEOUT_BASE nDDR)eA	ARA_BASE TchFwMmuR_CirvedMemStar  =  _CRAUTOVZ_OVui3I)
	FW	MMU_CARVEOUT_BASE nDDR;v#

	dvo	ARA_BASE TchFwMmuR_CirvedMemStar  = pRawHeapBa	d + (Re RFIRMhot 	RAW	HEAP_SIZR * se RNUMTIOfined(NOEDK, 		    RGXGvt is bein well CreateReg6MMRA(Fe>6vCr
fig/
wwa*UUUUUU n&ACTIVE     RGXpsFwMMUR_CirvedMemArena/
wwa*UUUUU	DDrmwa/
wwa*UUUUU	DD0/
wwa*UUUUU	DDhFwMmuR_CirvedMemStar /
wwa*UUUUU	DDhMaxFwMmuPageTableSiz*/
wwa*UUUUU	DD0/
wwa*UUUUU	DD"FwMMMUOMem "SoC s*pvReLOULGOTO_  =ui32Pu
#if dohe well CreateReg6MMRA(FwMMU)REN#if dDeiing);	f
				    _FTA[Gi32RegievInfo, 
#if dDeiing: tranRASSERT;y vaFALSE)_U	break;PhysMemHeapsDeiing;psefiE     Rr, [e_IS_FEATURE_SUPPORTEeAV_ERe	}

	pIF_GPU
iceUE_SUPPORTED hedRe	}

	pIF_GPUP;break;

	case ult:Meqe>6vKi:yP:hKi:bIGPU_UTI[ui32OS]));
;rO
	case }

ORYE_IDLE
#dTIV,puUtine G;bIGPU_UTIL_STATE_IDL	
#define G_U
Ape "rCOM, ME("IF_GPUPNam*: %s",_ACTIVE     RGXFe>6vCr
figGXFezNam*K, eA (dlACTIVE     RGXFe>6vCr
figGXFezVFrs6MMF	pfo->se "rCOM, ME("IF_GPUPVFrs6MM: %s",_ACTIVE     RGXFe>6vCr
figGXFezVFrs6MMFE_F}
CRse "rCOM, ME("RIMGIeiti meseclMMD(Par  1)"K, [G((((((((((((((((((((((	/* PDe_GPUPnessPseeup (	/*                     /ateLMAeeup on=%u,ymeqaRWts 
ellba
Bs oGpG_BOR PPro agnostu,yme_GPUPnessPicegsBaseKM, RGXe "rg)SACTIVE     RGXsDevId.pszPDumpRe	Nam*	U_Re RXe "rREG_NAME;)SACTIVE     RGXsDevId.pszPDumpDevNam*	U_ hysHeapPDump,pusparoNam*(ACTIVE     RGXaqePhysHeap[break;

	case PHYS	HEAPLGP
	LOCAL]NS;aACTIVE     RGXFfnPDumpringD _GPUPin&hedResetPDumpS;aACTIVE     RGX_N64FBCClearColourPU_Re RFBC_CC

	FAULT_G[g	    F((PXe "r  tim>OSAtomicWrite(&ACTIVE     RGXtHealthStecus,mpty(&ps
	case HEALTH STADUSsDev;2eOSAtomicWrite(&ACTIVE     RGXtHealthReason,mpty(&ps
	case HEALTH REASON_NONEr;a;aACTIVE     RGXFfnTIVSLCFlushRangeMing pALCFlushRangeS;aACTIVE     RGXFfnInv mFBSCTable inse riv memecpFBSCTable;a;aACTIVE     RGXFfnV memecpOrTweakPhysnfo-scinEVICE_;aACTIVE     RGXFfn,MU
acheriv memecpeing pMMU
acheriv memecpE_;aACTIVE     RGXFfn,MU
acheriv memecpKickeing pMMU
acheriv memecpKickE_[G((PRe	}

	pVRIMGE_LupceDPFvnotifi	s w4GGpogrfo me_GPUsLcompletevsom*ework er tran(&pRe	}

	pCmdCompleteNotify(&ACTIVE     RGXhCmdCompNotify,n&hedAchedulede->pvDQue; /KM,_ACTIVE     Rr;a;aACTIVE     RGXFfnringD _GPUVrepatCKCCB	in&hedD cringVrepatCKCCBE_[G((PRe	}

	pV
ellba
Bs urn creatS, EofGR PPro memUtivc	
texts ic
	qe>6vKi:   RGXFfnRe	}

	p,puUti
	
texteing pRe	}

	p,puUti
	
textS;aACTIVE     RGXFfnUn;
	}

	p,puUti
	
texteing pUn;
	}

	p,puUti
	
textE_[G((PRe	}

	pV
ellba
Bs urn Unifi		EFencsLObjects ic
	qe>6vKi:   RGXFfnAif

UFOBf

Being pAif

UFOBf

BS;aACTIVE     RGXFfnRreeUFOBf

B =tV_ERreeUFOBf

BE_[G((PRe	}

	pV
ellba
B urn cKCCBND_aG_BOR PPro'sNhealth ic
	qe>6vKi:   RGXFfnUpmequHealthStecuseing pUpmequHealthStecusE_[G((PRe	}

	pVmethoSpNUOserPPro G_BO
W HWPerf bufferNic
	qe>6vKi:   RGXFfnSerPProHWPerf ing pHWPerfDeqaStorsCBE_[G((PRe	}

	pV
ellba
B urn gettND_aG_BOR PPro vFrs6MM	ie Grmat6MMDstrND_ViceAACTIVE     RGXFfnTIVE  VFrs6MMStrND_ ing pTIVVFrs6MMStrND_E_[G((PRe	}

	pV
ellba
B urn gettND_aG_BOR PPro cf

B spe		EiceAACTIVE     RGXFfnTIVE  Cf

BSpe		Eing pTIVCf

BSpe		E_[G((PRe	}

	pV
ellba
B urn softLupsettND_asom*eR PPro modules ic
	qe>6vKi:   RGXFfnSoftResetMing pAoftResetE_[G((PRe	}

	pV
ellba
B urn upsettND_aG_BOHWRnlogs ic
	qe>6vKi:   RGXFfnResetHWRL
gseing pResetHWRL
gsE_[G((PRe	}

	pV
ellba
B urn upsettND_aG_BOHWRnlogs ic
	qe>6vKi:   RGXFfnVFrifyBVNCRing pVFrifyBVNCE_[G((PRe	}

	pV
ellba
B urn cKCCBND_aalignmentEofGUMRstrudau-esEic
	qe>6vKi:   RGXFfnAiignmentCKCCBeing pAiignmentCKCCBE_[G((Re	}

	pV
ellba
B urn cKCCBND_aG_BOsupyP:y		EueE_urisRWts gettND_aG_B	/* Pcorrispo_2ND_ v muesEic
	qe>6vKi:   RGXFfnCKCCB>6vKi:FeE_urieing pBvncCKCCBFeE_uriSupyP:y		S;aACTIVE     RGXFfnGet>6vKi:FeE_uriV mueeing pBvncGetAupyP:y		FeE_uriV mue;[
>_S Callba
B urn cKCCBND_a  Fsarams lay	pVsupyP:ys FBC 3.1Eic
	qe>6vKi:   RGXFfnHasFBCDCVFrs6MM31cinEVICE_CR_S Callba
B urn gettND_aG_BOMMUOR PPro attrNbutesEic
	qe>6vKi:   RGXFfnGetMMU>6vKi:AttrNbutesEing pTIVMMUAttrNbutes;atgsBaseKM, RGXe "rg &&aseKM, RGined(NO_SECURITY_32 uiATION)[G_S Callba
B urn gettND_aaOsecuriePDumpnmemUtivsparo nam*Eic
	qe>6vKi:   RGXFfnGetSecuriPDump,pusparoEing pGetSecuriPDump,pusparo, 		    RGX/ PRe	}

	pV
ellba
B urn ieiti mesND_asevKi:-specific physical memUtivheaps ic
	qe>6vKi:   RGXFfnPhysMemDeiKi:HeapsringEing pPhysMemDeiKi:Heapsring_UateLMAeesup re;
#end supyP:yOf bedummy page iceAOSAtomicWrite(&(ACTIVE     RGXsDummyPage.atRef
		  err& 0NS;AOSAtomicWrite(&(ACTIVE     RGXsDevZeroPage.atRef
		  err,		 Mr
PeLMAeesG_BOord	pVNUO0 ic
	qe>6vKi:   RGXsDummyPage.sPageHa_2le.uiOrd	pVU_0;v	ACTIVE     RGXsDevZeroPage.sPageHa_2le.uiOrd	pVU_0;v
PeLMAeesG_BOsiz*PvfV64GpDummy page NUOzero ic
	qe>6vKi:   RGXsDummyPage.psDeL
g2PgSiz*ein0;v
PeLMAeesG_BOsiz*PvfV64GpZero page NUOzero ic
	qe>6vKi:   RGXsDevZeroPage.psDeL
g2PgSiz*ein0;v
PeLMAeesG_BODummy page phys add- ic
	qe>6vKi:   RGXsDummyPage.ps64PgPhysnfo-einMMU_BADLPHYS	nDDR;v
PeLMAeesG_BOZero page phys add- ic
	qe>6vKi:   RGXsDevZeroPage.ps64PgPhysnfo-einMMU_BADLPHYS	nDDR;v
PeLMT	PVf

B can be aS;
#end irom MISR (Z-buffer) path iceAt is beinOSLo
BCreate(&ACTIVE     RGXsDummyPage.psPgL

BK, vE;
	
	}

	POKefinlist_ii	p{[AAsKCCBDeferredCommandsLio"****F		OSSpNUOcreate dummy page f

B"rNCY  0x2 {SoC sTED(psDevInfo, M}
CR_SPCreate t	PVf

B f bezero page iceAt is beinOSLo
BCreate(&ACTIVE     RGXsDevZeroPage.psPgL

BK, vE;
	
	}

	POKefinlist_ii	p{[AAsKCCBDeferredCommandsLio"****F		OSSpNUOcreate Zero page f

B"rNCY  0x2 {SoC sgoE_Liree_dummy_page, M}
gsBaseKM, RGXe "rg)SACTIVE     RGXsDummyPage.hPInfoPguinEVICE_FACTIVE     RGXsDevZeroPage.hPInfoPguinEVICE_		    RGX#defiKi:   RGXFfnHasFBCDCVFrs6MM31cing pAaramsHasFBCDCVFrs6MM31;v
PeLMT	PVR PPro shaend-virtual-memUtivheap add-ess-sparoEsiz*P destor R her*Purn fa

	p	/*  look-up withvuaDhavND_aGo walkaG_BOR PPro heap  r
figurat6MMDstrudau-esEdurND_
 *  clientOR PPro connex(driD (i.e. EoisLsiz*P dereleqDPFvE_LWezero-ba	ddPvffset) ic
	qe>6vKi:   RGXps64GeneralSVMHeapTopVAPU_Re RGENERAL_SVMRHEAP_BASE + Re RGENERAL_SVMRHEAP_SIZRE_[G((((((((((((((((((((((	/* PDe_GPUPPe Gvseeup (	/*                     /ateLMAif

ecp R PPro control bf

B ic
	qe>6vne GPU_OSAif

Z,puln PVRSR
#define G)r, fb->uqe>6vne GPUFirmwar	p{[AAsKCCBDeferredCommandsListHSSSSSSSSSuD cring	IMPar 1P: F		OSSpE_LWif

 memUtivurn efine G"KSoC supports full AIMG_UIOUT_OFR}

ORYibM}
CR_SPcreate f

Bs urn Eounc	
text mestdestor R NDe64Gp>6vne GPstrudau-e.DevivEouse mestdeW_MEmodifi		EriDc	
text create/d /*royRWts read byvEouDevivwatchdo_aG_readDeviceeAt is beinOSWRL

BCreate(&(ACTIVInfo->hd	  erCtxLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate r	  ernc	
text mestDf

B"rNCY  0x2 {SoC sgoE_LeUMraTA[Gt is beinOSWRL

BCreate(&(ACTIVInfo->hComputeCtxLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate computenc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le1MraTA[Gt is beinOSWRL

BCreate(&(ACTIVInfo->hTransferCtxLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate transfernc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le2MraTA[Gt is beinOSWRL

BCreate(&(ACTIVInfo->hTDMCtxLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate TDMnc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le3MraTA[Gt is beinOSWRL

BCreate(&(ACTIVInfo->hKickSyncCtxLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate kickesyncnc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le4MraTA[Gt is beinOSWRL

BCreate(&(ACTIVInfo->h,puUti
txLestLo
BKr, fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate memUtivc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le5MraTA[Gt is beinOSSpinL

BCreate(&ACTIVInfo->hL

BKCCBDeferendCommatssLest)E_Fb->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOKCCBaseKerend commatss mestDf

B"rNCY  0x2 {SoC sgoE_Le6_GPES;dlmest_ieit
&(ACTIVInfo->sKCCBDeferendCommatssLestHead) Mr
Pdlmest_ieit
&(ACTIVInfo->sd	  erCtxtLestHead) MrPdlmest_ieit
&(ACTIVInfo->sComputeCtxtLestHead) MrPdlmest_ieit
&(ACTIVInfo->sTDMCtxtLestHead) MrPdlmest_ieit
&(ACTIVInfo->sKickSyncCtxtLestHead) Mr
Pdlmest_ieit
&(ACTIVInfo->sCommon
txtLestHead) MrPACTIVInfo->sk =Common
txtCurrentID =N1M [[Gt is beinOSWRL

BCreate(&ACTIVInfo->hCommon
txtLestLo
BK, eA (dllist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate commriDc	
text mestDf

B"rNCY  0x2 {SoC sgoE_Le7MraTA[Gt is beinOSL

BCreate(&ACTIVInfo->sReg
	
gfig.hLo
BK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate r		}

	pV
r
figurat6MMDf

B"rNCY  0x2 {SoC sgoE_Le8MraTA[Gt is beinOSL

BCreate(&ACTIVInfo->hBPLo
BK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate f

B f bebreak points"rNCY  0x2 {SoC sgoE_Le9MraTA[Gt is beinOSL

BCreate(&ACTIVInfo->hdle  ,
BufringLo
BK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate f

B f betraro buffers"rNCY  0x2 {SoC sgoE_Le1UMraTA[Gt is beinOSL

BCreate(&ACTIVInfo->hCCBSaallCKCCBL

BK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate taallSSpCCBDcKCCBND_af

B"rNCY  0x2 {SoC sgoE_Le11_GPES;
 is beinOSL

BCreate(&ACTIVInfo->hCCBRecoveryLo
BK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOcreate taallSSpCCBDrecoveryaf

B"rNCY  0x2 {SoC sgoE_Le12MraTA[Gdlmest_ieit
&ACTIVInfo->s,puUti
	
textLest)E_CR_SPieiti mes	 sk =SLRHoldvff
		  erPiceA (dlRe RINITIAL_SLR_HOLDOFF_PERIOD_MS > 
	caseS_WATCHDOG_POWER_ON_SLEEP_TIMEOUT)void *psDevInfo->sk =SLRHoldvff
		  erPU_Re RINITIAL_SLR_HOLDOFF_PERIOD_MS / 
	caseS_WATCHDOG_POWER_ON_SLEEP_TIMEOUT_GPES;

	dvedulepsDevInfo->sk =SLRHoldvff
		  erPU_UMraTA[G((PAeeup on=%u,ymeqaRWts 
ellba
Bs oGpG_BOR PPro specific de_GPUPPe Gvic
	qe>6vne GGXFsefiKi:   R		U_ACTIVE     R;a;aACTIV,puUtine G in&ACTIVE     RGXsDev,puUtine G_U*ACTIVInfo->pvDfiKi:,puUtiHeapeinAC>6v,puUtine GGXFsefiKi:,puUtiHeapE_[G_S	/* PMap RGXPRe	}

	psDeviceApsDevInfo->sk =RegSiz* U_ACTIVE     RGXFe>6vCr
fig->sk =RegsAiz*;evACTIVInfo->sRegePhysBa	d =tACTIVE     RGXFe>6vCr
fig->sRegeCpuPBa	doC[yP:wState;
	/* snapshot fruACTIVInfo->pvRegsBa	dKM in(regatE_iomem *) OSMapPhysToLin(ACTIVE     RGXFe>6vCr
fig->sRegeCpuPBa	d,oUUUUUUUUUUUUACTIVE     RGXFe>6vCr
fig->sk =RegsAiz*/
wwa*UUUUU	DDDDD _CR_SOC_AXI_MASKFULCP
	UNCACH
DK,  fb->uqe>6vne G->pvRegsBa	dKM iFirmwar	p{[AAsKCCBDeferredCommandsListHSSSSSSSSSuran(&pRe ring>6vPar 2KM**F		OSSpNUOcreate RGXPr		}

	pVmapping"KSoC sS]));
 =s full AIMG_UIBADLMAPPINGoC sgoE_Le13MraTA#

	dvoACTIVInfo->pvRegsBa	dKM inEVICE_		    F((P!* snapshot   timXACTIVE     RGXFWIF_GPUPin#define G_U

vt is being pBvncIeiti mes	C	
figurat6MM(ACTIVE     Rr, fE;
	
	}

	POKefinlist_ii	p{[AAsKCCBDeferredCommandsListHSSSSSSSSSu****UnsupyP:y		EHW de_GPUPdetected byvd_DPFr"istHSSSSSSSSSCY  0x2 {SoC sgoE_Le14MraTA[G((PpInfoPPe GvabvuaDEounc	ro er tre "rCOM, ME("RIMGVFrs6MM Ie Grmat6MMD(KM): %d.%d.%d.%d"ist             ACTIVInfo->sDfiFeE_uriCfg._N32Bist             ACTIVInfo->sDfiFeE_uriCfg._N32Vist             ACTIVInfo->sDfiFeE_uriCfg._N32Nist             ACTIVInfo->sDfiFeE_uriCfg._N32C)E_CR_SPC	
figurBOMMUOspecific stuffviceAg pMMUring_Re	}

	p;psefiE     Rr, [et is being pringHeaps(ACTIVrPar& ACTIV,puUtine GroUUU&ACTIVE     RGXsDummyPage.psDeL
g2PgSiz*K, Ab->ulist_is_empty(&psDevInfo->goE_Le14MraTA[G((AeesG_BOzero page siz*Pas ne				Eurn Eoo heap with bigg /*vpage siz*PiceAqe>6vKi:   RGXsDevZeroPage.psDeL
g2PgSiz*einACTIVE     RGXsDummyPage.psDeL
g2PgSiz*, [et is being pHWPerfring;psefiInfo)MrapvReLOULGOTO_  =ui32Pu
#if doheg pHWPerfring"rNe14), [gsBaseKM, RGined(NO_32 uiATION)[Gt is being pP    TomainringStecp(&psDevInfo->sP    TomainStecp/
wwa*UUUUU	ACTIVInfo->sDfiFeE_uriCfg._N32MApP  UingVr	  i, Ab->ulist_is_empty(&psDevInfo->goE_Le15, M}
gsBaseKM, RGined(NO_SOC_TIMERg &&aseKM, RGXe "rg &&aseKM, RG* snapshot fru{uleIBRIBOOLtpsDeOP_CPU_Defaul  = y vaFALSE; leIBRIBOOLtbringSocTimer; leregats vOP_CPU_StecpuinEVICE_
->OSCreateKMOP_CPU_Stecp(&pvOP_CPU_Stecp)E_FAOSGetKMOP_CPU_BOOL(pvOP_CPU_Stecp, V memecpSOCUSCTimer,_&psDeOP_CPU_Defaul ,_&bringSocTimer)E_FAOSFreeKMOP_CPU_Stecp(pvOP_CPU_Stecp)E_[GAb->ubringSocTimer)
PsDevInfo-ak;PdumpringSOCUSCTimer(e);f
			ES		    R		    RGX/ PRe	}

	pV
ellba
B urn dumpND_asebug Pe Gvic
	t is being pr bugring;psefiInfo)MrapvReLOULGOTO_  =ui32Pu
#if doheg pr bugring"rNe16), [gsBaseKM, RGXe "rg)St is beinDonmemIntAif

DefBackND_Page;psefiE     R/
wwa*UUUUUU n&ACTIVE     RGXsDummyPage/
wwa*UUUUUU nrredCUMMYCPAGE_INITRVALUE/
wwa*UUUUUU nCUMMYCPAGEist                                      IBRITRUEK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOaif

ecp Rummy page."rNCY  0x2 {SoC sgoE_Le17_GPES;
 is beinDonmemIntAif

DefBackND_Page;psefiE     R/
wwa*UUUUUU n&ACTIVE     RGXsDevZeroPage/
wwa*UUUUUU nrredZEROCPAGE_INITRVALUE/
wwa*UUUUUU nCEVdZEROCPAGEist                                      IBRITRUEK, Ab->ulist_is_empty(&psDevInfo->sKCCBDeferredCommandsLio"****F		OSSpNUOaif

ecp Zero page."rNCY  0x2 {SoC sgoE_Le18;_F}
		    RG[G((PIeiti mes	 G_BOde_GPUPdeRT_2ent bridgesEic
	
 is beinDonGPUDepBridgering;psefiInfo->sDfiFeE_uriCfg._N64FeE_uris)MrapvReLOUL  =ui32Pu
#if doheDonGPUDepBridgering"r, [e_IS_FEA full Aalib[gsBaseKM, RGXe "rg)e18:
	DonmemIntFreeDefBackND_Page;psefiE     R/
wwa*UUUU&ACTIVE     RGXsDummyPage/
wwa*UUUUCUMMYCPAGE);
e17:rag pr bugDeiing;psefiInfo)Mr		    Re16:[gsBaseKM, RGined(NO_32 uiATION)[Gg pP    Tomainr ringStecp(&psDevInfo->sP    TomainStecpK, e15:			    _Fg pHWPerfDeiing;psefiInfo)Mre14:[yP:wState;
	/* snapshot fruOSUnMapPhysToLin((regatE_forro e) ACTIVInfo->pvRegsBa	dKMroUUUUUUU ACTIVInfo->_NONRegSiz*roUUUUUUUo _CR_SOC_AXI_MASKFULCP
	UNCACH
DK, re13: 		    F((P!* snapshot   tisOSL

BD /*roy(ACTIVInfo->hCCBRecoveryLo
BK, e12: AOSL

BD /*roy(ACTIVInfo->hCCBSaallCKCCBL

BK, e11: AOSL

BD /*roy(ACTIVInfo->hdle  ,
BufringLo
BK, e10: AOSL

BD /*roy(ACTIVInfo->hBPLo
BK, e9: AOSL

BD /*roy(ACTIVInfo->sReg
	
gfig.hLo
BK, e8:_FOSWRL

BD /*roy(ACTIVInfo->hCommon
txtLestLo
BK, e7: AOSSpinL

BD /*roy(ACTIVInfo->hL

BKCCBDeferendCommatssLest)E_e6:_FOSWRL

BD /*roy(ACTIVInfo->h,puUti
txLestLo
BK, e5:_FOSWRL

BD /*roy(ACTIVInfo->hKickSyncCtxLestLo
BK, e4:_FOSWRL

BD /*roy(ACTIVInfo->hTDMCtxLestLo
BK, e3:_FOSWRL

BD /*roy(ACTIVInfo->hTransferCtxLestLo
BK, e2:_FOSWRL

BD /*roy(ACTIVInfo->hComputeCtxLestLo
BK, e1:[GOSWRL

BD /*roy(ACTIVInfo->hd	  erCtxLestLo
BK, e0:eAOSFree,pulACTIVInfor;aU	_SPDe/*royvEounzero page f

BOcreates abovep tisOSL

BD /*roy(ACTIVE     RGXsDevZeroPage.psPgL

BK, 
iree_dummy_page:U	_SPDe/*royvEoundummy page f

BOcreates abovep tisOSL

BD /*roy(ACTIVE     RGXsDummyPage.psPgL

BK,  tranRASSERT;list_is_empty(&psDev;[e_IS_FEATURE_SUPPORIBRIPCnap g pTIVBVNCStrND_(GPU_UTIL_STATE_IDLE
#define GKi:bIIBRIPCnap psz emACTIVInfo->sefiFeE_uriCfg.pszBVNCStrND_, Ab->urmwa	=emACzfru{uleIBRICnap pszBVNCInfo[g pRHWPERFRMAXRBVNC_LEN]Mra	siz*_ttpsBVNCStrND_Aiz*;evIsiz*_ttpsStrND_Le gthE_[GApsStrND_Le gtheinOSSNPrNDtf(pszBVNCInforIse RHWPERFRMAXRBVNC_LENio"*d.%d.%d.%d"istUU	ACTIVInfo->sDfiFeE_uriCfg._N32BistUU	ACTIVInfo->sDfiFeE_uriCfg._N32VistUU	ACTIVInfo->sDfiFeE_uriCfg._N32NistUU	ACTIVInfo->sDfiFeE_uriCfg._N32CNS;AApvReASSERT;psStrND_Le gthe<Ise RHWPERFRMAXRBVNC_LEN)E_[GApsBVNCStrND_Aiz* in(psStrND_Le gthe+ 1)  _n PVRSRIBRICnapSoC sACzeinOSAif

,pulpsBVNCStrND_Aiz* MrAA (dlrmwa	_emACzr	psDevInOS
achedMemCopy(ACz, pszBVNCInforIpsBVNCStrND_Aiz* MrAA	ACTIVInfo->sDfiFeE_uriCfg.pszBVNCStrND_ emACz);f
			A

	dvosDevInsKCCBDeferredCommMESSAGEooUUUUU"****Aif

ecND_ memUtivurn BVNCRne GPstrND_ f		OSS"roUUUUUCY  0x2 {SoC s}_FTA[Gi32RegiACz);PORTEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/F((!
@F 0x(driD      g pTIVVFrs6MMStrND_
@Descrip(driD   Gets remOvFrs6MM	strND_ frn Eoo givenyme_GPUPnessPWts re2Regs
                a point	pVNUOit NDepFezVFrs6MMStrND_. It is remGpG_B
                rispo_sibilityPvfV64Gp
ell	pVNUOfree G_is memUti.
@Input          ACTIVE     R            De_GPUPnessPirom which NUOobtainpG_B
                                        vFrs6MM	strND_
@Outputt        pFezVFrs6MMStrND_	C	
tains remOvFrs6MM	strND_ upo_ re2Reg
@R32Regiiiiiiiii full AIMG_U
E/F((EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/ton=%u,y
				/* Re-schedTIVVFrs6MMStrND_;break;

	case ult:Meqe>6vKi:yP:hroUUUUUABRICnap **pFezVFrs6MMStrND_N
{
#b->tate;
	/COMPATRBVNC_MASK_B  ||dtate;
	/COMPATRBVNC_MASK_V  ||dtate;
	/COMPATRBVNC_MASK_N  ||dtate;
	/COMPATRBVNC_MASK_C  ||dtate;
	/* snapshot f ||dtate;
	/EMULATOR)eAcontavIBRICnap szFGrmatStrND_[] =s"GPU variant BVNC: %s (SW)";v#

	dvocontavIBRICnap szFGrmatStrND_[] =s"GPU variant BVNC: %s (HW)";v#
    _FGPU_UTIL_STATE_IDLE
#define G;bIIBRIPCnap pszBVNCE_Isiz*_ttpsStrND_Le gthE_[G (dlACTIVE     R iFirmwa ||dpFezVFrs6MMStrND_ iFirmwar	p{[AAupports full AIMG_UIIN32 ui_2AppHS;_F}
CRqeDfine GPU_
Mode;
	L_STATE_IDLE
)ACTIVE     RGXFWIF_GPU;evACzBVNCRing pTIVBVNCStrND_(ACTIVInfor;aU	b->urmwa	=emACzBVNCr	p{[AAupports full AIMG_UIOUT_OFR}

ORYibM}
CRpsStrND_Le gtheinOSStrND_Le gth(pszBVNCSoC psStrND_Le gthe+U_
n PVRSRszFGrmatStrND_) - 2);F((Pn PVRS NDcludesV64Gpnull, -2 frn "**"p tis*pFezVFrs6MMStrND_einOSAif

,pulpsStrND_Le gthe _n PVRSRIBRICnapSK, Ab->u*pFezVFrs6MMStrND_eiFirmwar	p{[AAupports full AIMG_UIOUT_OFR}

ORYibM}
CROSSNPrNDtf(*pFezVFrs6MMStrND_,tpsStrND_Le gth, szFGrmatStrND_,C sACzBVNCSoCCRi32Regi well calibrat((EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/F((!
@F 0x(driD      g pTIVCf

BSpe		
@Descrip(driD   Gets remOcf

B spe		Efrn Eoo givenyme_GPUPnessPWts re2Regs
                it NDep_NONRGXCf

BSpe		.
@Input          ACTIVE     R		De_GPUPness
@Output         A_NONRGXCf

BSpe		  Variable urn storND_aG_BOcf

B spe		
@R32Regiiiiiiiii full AIMG_U
E/F((EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/ton=%u,y
				/* Re-schedTIVCf

BSpe		;break;

	case ult:Meqe>6vKi:yP:hroUUUUUABRIPC_TYPE  A_NONRGXCf

BSpe		Ki:bIse RDARSMeqehedTeqaR=dlRe RDARSe) ACTIVE     RGXFe>6vCr
fig->h>6vTeqa;aU	_SPget cf

B spe		EiceA*A_NONRGXCf

BSpe		 emAChedTeqaGXFehedTimND_Info->sk =CoreCf

BSpe		E_[Gi32Regi well calibratgsBause RNUMTIOfined(NOEDO> 1)
((!
*                                                                               

 @F 0x(driIng pring(wRawHeap

 @Descrip(dri	CallSSpNUOper GrmTaddit6MMal ieiti meseclMM
*                                                                              /ton=%u,y
				/* Re-schedring(wRawHeap(er
}

oHEAP_BLUEPRINT eqe>6vMemHeap, IBRIC_TYPEt_NONOSidKi:bIIBRIC_TYPEt_NStrND_Le gthE_IIBRIC_TYPEt_NStrND_Le gthMax =sPE;
CRpsStrND_Le gtheinMIN
n PVRSRRe RFIRMhot 				OS	RAW	HEAP_ID MEg,tpsStrND_Le gthMax + 1);v
PeLMAtar  byVWif

eclD_ memUtivurn G_is OSIDOheap i2entificat6MMDstrND_ViceAACTIVMemHeapGXFezNam*einOSAif

,pulpsStrND_Le gthe _n PVRSRIBRICnapSK, Ab->uACTIVMemHeapGXFezNam*eiFirmwar	p{[AAupports full AIMG_UIOUT_OFR}

ORYibM}
CR((PApRT_2TG_BOOSIDOnumberNPVRthBORe RFIRMhot 				OS	RAW	HEAP_ID MEDstrND_ViceAOSSNPrNDtf((ABRICnap *)ACTIVMemHeapGXFezNam*,tpsStrND_Le gth, Re RFIRMhot 				OS	RAW	HEAP_ID ME,t_NONOSid);v
PeLMUs	 G_BOcommriDblueprNDt template tupyP:yOf 0x(driDNUOieiti mes	 G_BOheapEiceA*ACTIVMemHeap in_blueprNDt_ieit
(ABRICnap *)ACTIVMemHeapGXFezNam*,vo	ARe RFIRMhot 	RAW	HEAP_BASE + (_NONOSidP* Re RFIRMhot 	RAW	HEAP_SIZR),vo	ARe RFIRMhot 	RAW	HEAP_SIZR,vo	A0/
wwa0SoCCRi32Regi well calibrat((!
*                                                                               

 @F 0x(driIng pr ring(wRawHeap

 @Descrip(dri	CallSSpNUOper GrmTaddit6MMal deieiti meseclMM
*                                                                              /ton=%u,yregatV_Er ring(wRawHeap(er
}

oHEAP_BLUEPRINT eqe>6vMemHeapKi:bIIBRIC_TY64 uNBa	d =tRe RFIRMhot 	RAW	HEAP_BASE + Re RFIRMhot 	RAW	HEAP_SIZRE_FIBRIC_TY64 uNSpan = piBa	d + (use RNUMTIOfined(NOEDO- 1)  _Re RFIRMhot 	RAW	HEAP_SIZR);v
PeLMAafFvE_Ldovas remOg; /*vfRGXFW_MEheapsvW_MElast NDe64GpmestDiceA (dlACTIVMemHeapGXsHeapBa	dnfo-._Nnfo-D>= piBa	d &&st    ACTIVMemHeapGXsHeapBa	dnfo-._Nnfo-D< uNSpanr	p{[AAregats ezNam*ein(rega*)ACTIVMemHeapGXFezNam*E_FAOSFree,pulACzNam*K, M}
ES		    F((Puse RNUMTIOfined(NOEDO> 1)  tim((EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
 EndPvfvfRle (rgxieit.c)
(EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE/
