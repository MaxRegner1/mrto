/*************************************************************************/ /*!
@File
@Title          RGX firmware interface structures used by pvrsrvkm
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware interface structures used by pvrsrvkm
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

#if !defined(RGX_FWIF_KM_H)
#define RGX_FWIF_KM_H

#include "img_types.h"
#include "rgx_fwif_shared.h"
#include "rgxdefs_km.h"
#include "dllist.h"
#include "rgx_hwperf.h"


/*************************************************************************/ /*!
 Logging type
*/ /**************************************************************************/
#define RGXFWIF_LOG_TYPE_NONE			0x00000000U
#define RGXFWIF_LOG_TYPE_TRACE			0x00000001U
#define RGXFWIF_LOG_TYPE_GROUP_MAIN		0x00000002U
#define RGXFWIF_LOG_TYPE_GROUP_MTS		0x00000004U
#define RGXFWIF_LOG_TYPE_GROUP_CLEANUP	0x00000008U
#define RGXFWIF_LOG_TYPE_GROUP_CSW		0x00000010U
#define RGXFWIF_LOG_TYPE_GROUP_BIF		0x00000020U
#define RGXFWIF_LOG_TYPE_GROUP_PM		0x00000040U
#define RGXFWIF_LOG_TYPE_GROUP_RTD		0x00000080U
#define RGXFWIF_LOG_TYPE_GROUP_SPM		0x00000100U
#define RGXFWIF_LOG_TYPE_GROUP_POW		0x00000200U
#define RGXFWIF_LOG_TYPE_GROUP_HWR		0x00000400U
#define RGXFWIF_LOG_TYPE_GROUP_HWP		0x00000800U
#define RGXFWIF_LOG_TYPE_GROUP_RPM		0x00001000U
#define RGXFWIF_LOG_TYPE_GROUP_DMA		0x00002000U
#define RGXFWIF_LOG_TYPE_GROUP_MISC		0x00004000U
#define RGXFWIF_LOG_TYPE_GROUP_DEBUG	0x80000000U
#define RGXFWIF_LOG_TYPE_GROUP_MASK		0x80007FFEU
#define RGXFWIF_LOG_TYPE_MASK			0x80007FFFU

/* String used in pvrdebug -h output */
#define RGXFWIF_LOG_GROUPS_STRING_LIST   "main,mts,cleanup,csw,bif,pm,rtd,spm,pow,hwr,hwp,rpm,dma,misc,debug"

/* Table entry to map log group strings to log type value */
typedef struct {
	const IMG_CHAR* pszLogGroupName;
	IMG_UINT32      ui32LogGroupType;
} RGXFWIF_LOG_GROUP_MAP_ENTRY;

/*
  Macro for use with the RGXFWIF_LOG_GROUP_MAP_ENTRY type to create a lookup
  table where needed. Keep log group names short, no more than 20 chars.
*/
#define RGXFWIF_LOG_GROUP_NAME_VALUE_MAP { "none",    RGXFWIF_LOG_TYPE_NONE }, \
                                         { "main",    RGXFWIF_LOG_TYPE_GROUP_MAIN }, \
                                         { "mts",     RGXFWIF_LOG_TYPE_GROUP_MTS }, \
                                         { "cleanup", RGXFWIF_LOG_TYPE_GROUP_CLEANUP }, \
                                         { "csw",     RGXFWIF_LOG_TYPE_GROUP_CSW }, \
                                         { "bif",     RGXFWIF_LOG_TYPE_GROUP_BIF }, \
                                         { "pm",      RGXFWIF_LOG_TYPE_GROUP_PM }, \
                                         { "rtd",     RGXFWIF_LOG_TYPE_GROUP_RTD }, \
                                         { "spm",     RGXFWIF_LOG_TYPE_GROUP_SPM }, \
                                         { "pow",     RGXFWIF_LOG_TYPE_GROUP_POW }, \
                                         { "hwr",     RGXFWIF_LOG_TYPE_GROUP_HWR }, \
                                         { "hwp",     RGXFWIF_LOG_TYPE_GROUP_HWP }, \
                                         { "rpm",     RGXFWIF_LOG_TYPE_GROUP_RPM }, \
                                         { "dma",     RGXFWIF_LOG_TYPE_GROUP_DMA }, \
                                         { "misc",    RGXFWIF_LOG_TYPE_GROUP_MISC }, \
                                         { "debug",   RGXFWIF_LOG_TYPE_GROUP_DEBUG }


/* Used in print statements to display log group state, one %s per group defined */
#define RGXFWIF_LOG_ENABLED_GROUPS_LIST_PFSPEC  "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"

/* Used in a print statement to display log group state, one per group */
#define RGXFWIF_LOG_ENABLED_GROUPS_LIST(types)  (((types) & RGXFWIF_LOG_TYPE_GROUP_MAIN)	?("main ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_MTS)		?("mts ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_CLEANUP)	?("cleanup ")	:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_CSW)		?("csw ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_BIF)		?("bif ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_PM)		?("pm ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_RTD)		?("rtd ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_SPM)		?("spm ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_POW)		?("pow ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_HWR)		?("hwr ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_HWP)		?("hwp ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_RPM)		?("rpm ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_DMA)		?("dma ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_MISC)	?("misc ")		:("")),		\
                                                (((types) & RGXFWIF_LOG_TYPE_GROUP_DEBUG)	?("debug ")		:(""))


/************************************************************************
* RGX FW signature checks
************************************************************************/
#define RGXFW_SIG_BUFFER_SIZE_MIN       (8192)

/*!
 ******************************************************************************
 * Trace Buffer
 *****************************************************************************/

/*! Default size of RGXFWIF_TRACEBUF_SPACE in DWords */
#define RGXFW_TRACE_BUF_DEFAULT_SIZE_IN_DWORDS 12000U
#define RGXFW_TRACE_BUFFER_ASSERT_SIZE 200U
#if defined(RGXFW_META_SUPPORT_2ND_THREAD)
#define RGXFW_THREAD_NUM 2U
#else
#define RGXFW_THREAD_NUM 1U
#endif

#define RGXFW_POLL_TYPE_SET 0x80000000U

typedef struct
{
	IMG_CHAR	szPath[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
	IMG_CHAR	szInfo[RGXFW_TRACE_BUFFER_ASSERT_SIZE];
	IMG_UINT32	ui32LineNum;
} UNCACHED_ALIGN RGXFWIF_FILE_INFO_BUF;

typedef struct
{
	IMG_UINT32			ui32TracePointer;

#if defined(RGX_FIRMWARE)
	IMG_UINT32 *pui32RGXFWIfTraceBuffer;		/* To be used by firmware for writing into trace buffer */
#else
	RGXFWIF_DEV_VIRTADDR pui32RGXFWIfTraceBuffer;
#endif
	IMG_PUINT32             pui32TraceBuffer;	/* To be used by host when reading from trace buffer */

	RGXFWIF_FILE_INFO_BUF	sAssertBuf;
} UNCACHED_ALIGN RGXFWIF_TRACEBUF_SPACE;


#define RGXFWIF_FWFAULTINFO_MAX 	(8U)			/* Total number of FW fault logs stored */

typedef struct
{
	IMG_UINT64 RGXFW_ALIGN	ui64CRTimer;
	IMG_UINT64 RGXFW_ALIGN	ui64OSTimer;
	IMG_UINT32 RGXFW_ALIGN	ui32Data;
	IMG_UINT32 ui32Reserved;
	RGXFWIF_FILE_INFO_BUF	sFaultBuf;
} UNCACHED_ALIGN RGX_FWFAULTINFO;


#define RGXFWIF_POW_STATES \
  X(RGXFWIF_POW_OFF)			/* idle and handshaked with the host (ready to full power down) */ \
  X(RGXFWIF_POW_ON)				/* running HW commands */ \
  X(RGXFWIF_POW_FORCED_IDLE)	/* forced idle */ \
  X(RGXFWIF_POW_IDLE)			/* idle waiting for host handshake */

typedef enum
{
#define X(NAME) NAME,
	RGXFWIF_POW_STATES
#undef X
} RGXFWIF_POW_STATE;

/* Firmware HWR states */
#define RGXFWIF_HWR_HARDWARE_OK			(0x1U << 0U)	/*!< The HW state is ok or locked up */
#define RGXFWIF_HWR_ANALYSIS_DONE		(0x1U << 2U)	/*!< The analysis of a GPU lockup has been performed */
#define RGXFWIF_HWR_GENERAL_LOCKUP		(0x1U << 3U)	/*!< A DM unrelated lockup has been detected */
#define RGXFWIF_HWR_DM_RUNNING_OK		(0x1U << 4U)	/*!< At least one DM is running without being close to a lockup */
#define RGXFWIF_HWR_DM_STALLING			(0x1U << 5U)	/*!< At least one DM is close to lockup */
#define RGXFWIF_HWR_FW_FAULT			(0x1U << 6U)	/*!< The FW has faulted and needs to restart */
#define RGXFWIF_HWR_RESTART_REQUESTED	(0x1U << 7U)	/*!< The FW has requested the host to restart it */
typedef IMG_UINT32 RGXFWIF_HWR_STATEFLAGS;

/* Firmware per-DM HWR states */
#define RGXFWIF_DM_STATE_WORKING 					(0x00U)		/*!< DM is working if all flags are cleared */
#define RGXFWIF_DM_STATE_READY_FOR_HWR 				(IMG_UINT32_C(0x1) << 0)	/*!< DM is idle and ready for HWR */
#define RGXFWIF_DM_STATE_NEEDS_SKIP					(IMG_UINT32_C(0x1) << 2)	/*!< DM need to skip to next cmd before resuming processing */
#define RGXFWIF_DM_STATE_NEEDS_PR_CLEANUP			(IMG_UINT32_C(0x1) << 3)	/*!< DM need partial render cleanup before resuming processing */
#define RGXFWIF_DM_STATE_NEEDS_TRACE_CLEAR			(IMG_UINT32_C(0x1) << 4)	/*!< DM need to increment Recovery Count once fully recovered */
#define RGXFWIF_DM_STATE_GUILTY_LOCKUP				(IMG_UINT32_C(0x1) << 5)	/*!< DM was identified as locking up and causing HWR */
#define RGXFWIF_DM_STATE_INNOCENT_LOCKUP			(IMG_UINT32_C(0x1) << 6)	/*!< DM was innocently affected by another lockup which caused HWR */
#define RGXFWIF_DM_STATE_GUILTY_OVERRUNING			(IMG_UINT32_C(0x1) << 7)	/*!< DM was identified as over-running and causing HWR */
#define RGXFWIF_DM_STATE_INNOCENT_OVERRUNING		(IMG_UINT32_C(0x1) << 8)	/*!< DM was innocently affected by another DM over-running which caused HWR */
#define RGXFWIF_DM_STATE_HARD_CONTEXT_SWITCH		(IMG_UINT32_C(0x1) << 9)	/*!< DM was forced into HWR as it delayed more important workloads */

/* Per-OS Fw States */
typedef enum
{
	RGXFW_OS_STATE_STOPPED,		/*!< OS in this state is ignored by the FW */
	RGXFW_OS_STATE_READY,		/*!< OS is allowed to run by the Host/Hypervisor but is uninitialised */
	RGXFW_OS_STATE_ACTIVE,		/*!< OS is fully up and running */
	RGXFW_OS_STATE_OFFLOADING	/*!< OS is in a transitory state, finishing its tasks before stopping */
} RGXFWIF_OS_STATE;

typedef struct
{
	IMG_UINT			bfOsState		: 3;
	IMG_UINT			bfFLOk			: 1;
	IMG_UINT			bfFLGrowPending	: 1;
	IMG_UINT			bfIsolatedOS	: 1;
	IMG_UINT			bfReserved		: 26;
} RGXFWIF_PER_OS_STATES;

typedef IMG_UINT32 RGXFWIF_HWR_RECOVERYFLAGS;

typedef struct
{
	IMG_UINT32				ui32LogType;
	volatile RGXFWIF_POW_STATE		ePowState;
	RGXFWIF_TRACEBUF_SPACE	sTraceBuf[RGXFW_THREAD_NUM];
	IMG_UINT32              ui32TraceBufSizeInDWords; /* Member initialised only when sTraceBuf is actually allocated
	                                                   * (in RGXTraceBufferInitOnDemandResources) */

	IMG_UINT32				aui32HwrDmLockedUpCount[RGXFWIF_DM_DEFAULT_MAX];
	IMG_UINT32				aui32HwrDmOverranCount[RGXFWIF_DM_DEFAULT_MAX];
	IMG_UINT32				aui32HwrDmRecoveredCount[RGXFWIF_DM_DEFAULT_MAX];
	IMG_UINT32				aui32HwrDmFalseDetectCount[RGXFWIF_DM_DEFAULT_MAX];
	IMG_UINT32				ui32HwrCounter;

	IMG_UINT32				aui32CrPollAddr[RGXFW_THREAD_NUM];
	IMG_UINT32				aui32CrPollMask[RGXFW_THREAD_NUM];

	RGXFWIF_HWR_STATEFLAGS		ui32HWRStateFlags;
	RGXFWIF_HWR_RECOVERYFLAGS	aui32HWRRecoveryFlags[RGXFWIF_DM_DEFAULT_MAX];

	volatile IMG_UINT32		ui32HWPerfRIdx;
	volatile IMG_UINT32		ui32HWPerfWIdx;
	volatile IMG_UINT32		ui32HWPerfWrapCount;
	IMG_UINT32				ui32HWPerfSize;       /* Constant after setup, needed in FW */
	IMG_UINT32				ui32HWPerfDropCount;  /* The number of times the FW drops a packet due to buffer full */

	/* These next three items are only valid at runtime when the FW is built
	 * with RGX_HWPERF_UTILIZATION & RGX_HWPERF_DROP_TRACKING defined
	 * in rgxfw_hwperf.c */
	IMG_UINT32				ui32HWPerfUt;         /* Buffer utilisation, high watermark of bytes in use */
	IMG_UINT32				ui32FirstDropOrdinal; /* The ordinal of the first packet the FW dropped */
	IMG_UINT32				ui32LastDropOrdinal;  /* The ordinal of the last packet the FW dropped */
#if !defined(RGX_FW_IRQ_OS_COUNTERS)
	volatile IMG_UINT32			aui32InterruptCount[RGXFW_THREAD_NUM]; /*!< Interrupt count from Threads > */
#endif
	IMG_UINT32				ui32KCCBCmdsExecuted;
	IMG_UINT64 RGXFW_ALIGN			ui64StartIdleTime;
	IMG_UINT32				ui32PowMonEstimate;	/* Non-volatile power monitoring results:
													   static power (by default)
													   energy count (PVR_POWER_MONITOR_DYNAMIC_ENERGY) */
#define RGXFWIF_MAX_PCX 16U
	IMG_UINT32				ui32T1PCX[RGXFWIF_MAX_PCX];
	IMG_UINT32				ui32T1PCXWOff;

	RGXFWIF_PER_OS_STATES	sPerOsStateMirror[RGXFW_NUM_OS];	/*!< State flags for each Operating System mirrored from Fw coremem> */

	IMG_UINT32				ui32MMUFlushCounter;

	RGX_FWFAULTINFO			sFaultInfo[RGXFWIF_FWFAULTINFO_MAX];
	IMG_UINT32				ui32FWFaults;

	/* Markers to signal that the host should perform a full sync check. */
	IMG_UINT32				ui32FWSyncCheckMark;
	IMG_UINT32				ui32HostSyncCheckMark;

#if defined(SUPPORT_RGXFW_STATS_FRAMEWORK)
#define RGXFWIF_STATS_FRAMEWORK_LINESIZE	(8)
#define RGXFWIF_STATS_FRAMEWORK_MAX			(2048*RGXFWIF_STATS_FRAMEWORK_LINESIZE)
	IMG_UINT32 RGXFW_ALIGN	aui32FWStatsBuf[RGXFWIF_STATS_FRAMEWORK_MAX];
#endif

	IMG_UINT32              ui32TracebufFlags; /*!< Compatibility and other flags */
} UNCACHED_ALIGN RGXFWIF_TRACEBUF;


/*!
 ******************************************************************************
 * HWR Data
 *****************************************************************************/
typedef enum
{
	RGX_HWRTYPE_UNKNOWNFAILURE = 0,
	RGX_HWRTYPE_OVERRUN        = 1,
	RGX_HWRTYPE_POLLFAILURE    = 2,
	RGX_HWRTYPE_BIF0FAULT      = 3,
	RGX_HWRTYPE_BIF1FAULT      = 4,
	RGX_HWRTYPE_TEXASBIF0FAULT = 5,
	RGX_HWRTYPE_MMUFAULT       = 6,
	RGX_HWRTYPE_MMUMETAFAULT   = 7,
} RGX_HWRTYPE;

#define RGXFWIF_HWRTYPE_BIF_BANK_GET(eHWRType) ((eHWRType == RGX_HWRTYPE_BIF0FAULT) ? 0 : 1)

#define RGXFWIF_HWRTYPE_PAGE_FAULT_GET(eHWRType) ((eHWRType == RGX_HWRTYPE_BIF0FAULT      ||       \
                                                   eHWRType == RGX_HWRTYPE_BIF1FAULT      ||       \
                                                   eHWRType == RGX_HWRTYPE_TEXASBIF0FAULT ||       \
                                                   eHWRType == RGX_HWRTYPE_MMUFAULT       ||       \
                                                   eHWRType == RGX_HWRTYPE_MMUMETAFAULT) ? 1 : 0)

typedef struct
{
	IMG_UINT64	RGXFW_ALIGN		ui64BIFReqStatus;
	IMG_UINT64	RGXFW_ALIGN		ui64BIFMMUStatus;
	IMG_UINT64	RGXFW_ALIGN		ui64PCAddress; /*!< phys address of the page catalogue */
	IMG_UINT64	RGXFW_ALIGN		ui64Reserved;
} RGX_BIFINFO;

typedef struct
{
	IMG_UINT64	RGXFW_ALIGN		ui64MMUStatus;
	IMG_UINT64	RGXFW_ALIGN		ui64PCAddress; /*!< phys address of the page catalogue */
	IMG_UINT64	RGXFW_ALIGN		ui64Reserved;
} RGX_MMUINFO;

typedef struct
{
	IMG_UINT32	ui32ThreadNum;
	IMG_UINT32 	ui32CrPollAddr;
	IMG_UINT32 	ui32CrPollMask;
	IMG_UINT32 	ui32CrPollLastValue;
	IMG_UINT64 	RGXFW_ALIGN ui64Reserved;
} UNCACHED_ALIGN RGX_POLLINFO;

typedef struct
{
	union
	{
		RGX_BIFINFO  sBIFInfo;
		RGX_MMUINFO  sMMUInfo;
		RGX_POLLINFO sPollInfo;
	} uHWRData;

	IMG_UINT64 RGXFW_ALIGN ui64CRTimer;
	IMG_UINT64 RGXFW_ALIGN ui64OSTimer;
	IMG_UINT32             ui32FrameNum;
	IMG_UINT32             ui32PID;
	IMG_UINT32             ui32ActiveHWRTData;
	IMG_UINT32             ui32HWRNumber;
	IMG_UINT32             ui32EventStatus;
	IMG_UINT32             ui32HWRRecoveryFlags;
	RGX_HWRTYPE            eHWRType;
	RGXFWIF_DM             eDM;
	IMG_UINT64 RGXFW_ALIGN ui64CRTimeOfKick;
	IMG_UINT64 RGXFW_ALIGN ui64CRTimeHWResetStart;
	IMG_UINT64 RGXFW_ALIGN ui64CRTimeHWResetFinish;
	IMG_UINT64 RGXFW_ALIGN ui64CRTimeFreelistReady;
	IMG_UINT64 RGXFW_ALIGN ui64Reserved[2];
} UNCACHED_ALIGN RGX_HWRINFO;

#define RGXFWIF_HWINFO_MAX_FIRST 8U							/* Number of first HWR logs recorded (never overwritten by newer logs) */
#define RGXFWIF_HWINFO_MAX_LAST 8U							/* Number of latest HWR logs (older logs are overwritten by newer logs) */
#define RGXFWIF_HWINFO_MAX (RGXFWIF_HWINFO_MAX_FIRST + RGXFWIF_HWINFO_MAX_LAST)	/* Total number of HWR logs stored in a buffer */
#define RGXFWIF_HWINFO_LAST_INDEX (RGXFWIF_HWINFO_MAX - 1U)	/* Index of the last log in the HWR log buffer */
typedef struct
{
	RGX_HWRINFO sHWRInfo[RGXFWIF_HWINFO_MAX];

	IMG_UINT32	ui32FirstCrPollAddr[RGXFW_THREAD_NUM];
	IMG_UINT32	ui32FirstCrPollMask[RGXFW_THREAD_NUM];
	IMG_UINT32	ui32FirstCrPollLastValue[RGXFW_THREAD_NUM];
	IMG_UINT32	ui32WriteIndex;
	IMG_UINT32	ui32DDReqCount;
	IMG_UINT32	ui32HWRInfoBufFlags; /* Compatibility and other flags */
} UNCACHED_ALIGN RGXFWIF_HWRINFOBUF;

typedef enum
{
	RGX_ACTIVEPM_FORCE_OFF = 0,
	RGX_ACTIVEPM_FORCE_ON = 1,
	RGX_ACTIVEPM_DEFAULT = 2
} RGX_ACTIVEPM_CONF;

typedef enum
{
	RGX_RD_POWER_ISLAND_FORCE_OFF = 0,
	RGX_RD_POWER_ISLAND_FORCE_ON = 1,
	RGX_RD_POWER_ISLAND_DEFAULT = 2
} RGX_RD_POWER_ISLAND_CONF;

typedef enum
{
	RGX_META_T1_OFF   = 0x0,           /*!< No thread 1 running (unless 2nd thread is used for HWPerf) */
	RGX_META_T1_MAIN  = 0x1,           /*!< Run the main thread 0 code on thread 1 (and vice versa if 2nd thread is used for HWPerf) */
	RGX_META_T1_DUMMY = 0x2            /*!< Run dummy test code on thread 1 */
} RGX_META_T1_CONF;

/*!
 ******************************************************************************
 * Querying DM state
 *****************************************************************************/

typedef enum
{
	RGXFWIF_DM_STATE_NORMAL			= 0,
	RGXFWIF_DM_STATE_LOCKEDUP		= 1
} RGXFWIF_DM_STATE;

typedef struct
{
	IMG_UINT16 ui16RegNum;				/*!< Register number */
	IMG_UINT16 ui16IndirectRegNum;		/*!< Indirect register number (or 0 if not used) */
	IMG_UINT16 ui16IndirectStartVal;	/*!< Start value for indirect register */
	IMG_UINT16 ui16IndirectEndVal;		/*!< End value for indirect register */
} RGXFW_REGISTER_LIST;


#define RGXFWIF_CTXSWITCH_PROFILE_FAST_EN		(1U)
#define RGXFWIF_CTXSWITCH_PROFILE_MEDIUM_EN		(2U)
#define RGXFWIF_CTXSWITCH_PROFILE_SLOW_EN		(3U)
#define RGXFWIF_CTXSWITCH_PROFILE_NODELAY_EN	(4U)

/*!
 ******************************************************************************
 * RGX firmware Init Config Data
 *****************************************************************************/
#define RGXFWIF_INICFG_CTXSWITCH_TA_EN				(IMG_UINT32_C(0x1) << 0)
#define RGXFWIF_INICFG_CTXSWITCH_3D_EN				(IMG_UINT32_C(0x1) << 1)
#define RGXFWIF_INICFG_CTXSWITCH_CDM_EN				(IMG_UINT32_C(0x1) << 2)
#define RGXFWIF_INICFG_CTXSWITCH_MODE_RAND			(IMG_UINT32_C(0x1) << 3)
#define RGXFWIF_INICFG_CTXSWITCH_SRESET_EN			(IMG_UINT32_C(0x1) << 4)
#define RGXFWIF_INICFG_POW_RASCALDUST				(IMG_UINT32_C(0x1) << 5)
#define RGXFWIF_INICFG_HWPERF_EN					(IMG_UINT32_C(0x1) << 6)
#define RGXFWIF_INICFG_HWR_EN						(IMG_UINT32_C(0x1) << 7)
#define RGXFWIF_INICFG_CHECK_MLIST_EN				(IMG_UINT32_C(0x1) << 8)
#define RGXFWIF_INICFG_DISABLE_CLKGATING_EN 		(IMG_UINT32_C(0x1) << 9)
#define RGXFWIF_INICFG_POLL_COUNTERS_EN				(IMG_UINT32_C(0x1) << 10)
#define RGXFWIF_INICFG_VDM_CTX_STORE_MODE_SHIFT		(11)
#define RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INDEX		(RGX_CR_VDM_CONTEXT_STORE_MODE_MODE_INDEX << RGXFWIF_INICFG_VDM_CTX_STORE_MODE_SHIFT)
#define RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INSTANCE	(RGX_CR_VDM_CONTEXT_STORE_MODE_MODE_INSTANCE << RGXFWIF_INICFG_VDM_CTX_STORE_MODE_SHIFT)
#define RGXFWIF_INICFG_VDM_CTX_STORE_MODE_LIST		(RGX_CR_VDM_CONTEXT_STORE_MODE_MODE_LIST << RGXFWIF_INICFG_VDM_CTX_STORE_MODE_SHIFT)
#define RGXFWIF_INICFG_VDM_CTX_STORE_MODE_MASK		(RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INDEX |\
                                                     RGXFWIF_INICFG_VDM_CTX_STORE_MODE_INSTANCE |\
                                                     RGXFWIF_INICFG_VDM_CTX_STORE_MODE_LIST)
#define RGXFWIF_INICFG_REGCONFIG_EN					(IMG_UINT32_C(0x1) << 13)
#define RGXFWIF_INICFG_ASSERT_ON_OUTOFMEMORY		(IMG_UINT32_C(0x1) << 14)
#define RGXFWIF_INICFG_HWP_DISABLE_FILTER			(IMG_UINT32_C(0x1) << 15)
#define RGXFWIF_INICFG_CUSTOM_PERF_TIMER_EN			(IMG_UINT32_C(0x1) << 16)
#define RGXFWIF_INICFG_CDM_KILL_MODE_RAND_EN		(IMG_UINT32_C(0x1) << 17)
#define RGXFWIF_INICFG_DISABLE_DM_OVERLAP			(IMG_UINT32_C(0x1) << 18)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT		(19)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_FAST		(RGXFWIF_CTXSWITCH_PROFILE_FAST_EN << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_MEDIUM		(RGXFWIF_CTXSWITCH_PROFILE_MEDIUM_EN << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_SLOW		(RGXFWIF_CTXSWITCH_PROFILE_SLOW_EN << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_NODELAY	(RGXFWIF_CTXSWITCH_PROFILE_NODELAY_EN << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT)
#define RGXFWIF_INICFG_CTXSWITCH_PROFILE_MASK		(IMG_UINT32_C(0x7) << RGXFWIF_INICFG_CTXSWITCH_PROFILE_SHIFT)
#define RGXFWIF_INICFG_METAT1_SHIFT					(22)
#define RGXFWIF_INICFG_METAT1_MAIN					((IMG_UINT32)RGX_META_T1_MAIN  << RGXFWIF_INICFG_METAT1_SHIFT)
#define RGXFWIF_INICFG_METAT1_DUMMY					((IMG_UINT32)RGX_META_T1_DUMMY << RGXFWIF_INICFG_METAT1_SHIFT)
#define RGXFWIF_INICFG_METAT1_ENABLED				(RGXFWIF_INICFG_METAT1_MAIN | RGXFWIF_INICFG_METAT1_DUMMY)
#define RGXFWIF_INICFG_METAT1_MASK					(RGXFWIF_INICFG_METAT1_ENABLED >> RGXFWIF_INICFG_METAT1_SHIFT)
#define RGXFWIF_INICFG_ASSERT_ON_HWR_TRIGGER		(IMG_UINT32_C(0x1) << 24)
#define RGXFWIF_INICFG_WORKEST_V1					(IMG_UINT32_C(0x1) << 25)
#define RGXFWIF_INICFG_WORKEST_V2					(IMG_UINT32_C(0x1) << 26)
#define RGXFWIF_INICFG_PDVFS_V1						(IMG_UINT32_C(0x1) << 27)
#define RGXFWIF_INICFG_PDVFS_V2						(IMG_UINT32_C(0x1) << 28)
#define RGXFWIF_INICFG_DISABLE_PDP_EN				(IMG_UINT32_C(0x1) << 29)
#define RGXFWIF_INICFG_ALL							(0x3FFFFFFFU)

#define RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TDM          (0x1U <<  0)
#define RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TA           (0x1U <<  1)
#define RGXFWIF_INICFG_EXT_LOW_PRIO_CS_3D           (0x1U <<  2)
#define RGXFWIF_INICFG_EXT_LOW_PRIO_CS_CDM          (0x1U <<  3)
#define RGXFWIF_INICFG_EXT_VALIDATE_IRQ             (0x1U <<  4)
#define RGXFWIF_INICFG_EXT_LOW_PRIO_CS_MASK         (RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TDM |\
                                                     RGXFWIF_INICFG_EXT_LOW_PRIO_CS_TA  |\
                                                     RGXFWIF_INICFG_EXT_LOW_PRIO_CS_3D  |\
                                                     RGXFWIF_INICFG_EXT_LOW_PRIO_CS_CDM |\
                                                     RGXFWIF_INICFG_EXT_VALIDATE_IRQ)
#define RGXFWIF_INICFG_EXT_FBCDC_V3_1_EN			(0x1U <<  5)
#define RGXFWIF_INICFG_EXT_PDVFS_HOST_REACTIVE_TIMER (0x1U << 6)

#define RGXFWIF_FILTCFG_TRUNCATE_HALF		(0x1U << 3)
#define RGXFWIF_FILTCFG_TRUNCATE_INT		(0x1U << 2)
#define RGXFWIF_FILTCFG_NEW_FILTER_MODE		(0x1U << 1)

#define RGXFWIF_INICFG_CTXSWITCH_DM_ALL		(RGXFWIF_INICFG_CTXSWITCH_TA_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_3D_EN | \
											 RGXFWIF_INICFG_CTXSWITCH_CDM_EN)

#define RGXFWIF_INICFG_CTXSWITCH_CLRMSK		~(RGXFWIF_INICFG_CTXSWITCH_DM_ALL | \
											 RGXFWIF_INICFG_CTXSWITCH_MODE_RAND | \
											 RGXFWIF_INICFG_CTXSWITCH_SRESET_EN)

#if defined(RGX_FW_IRQ_OS_COUNTERS)
/* Unused registers re-purposed for storing counters of the Firmware's
 * interrupts for each OS
 */
#define IRQ_COUNTER_STORAGE_REGS                        \
		0x2028U, /* RGX_CR_PM_TA_MMU_FSTACK         */  \
		0x2050U, /* RGX_CR_PM_3D_MMU_FSTACK         */  \
		0x2030U, /* RGX_CR_PM_START_OF_MMU_TACONTEXT*/  \
		0x2058U, /* RGX_CR_PM_START_OF_MMU_3DCONTEXT*/  \
		0x2058U, /* RGX_CR_PM_START_OF_MMU_3DCONTEXT*/  \
		0x2058U, /* RGX_CR_PM_START_OF_MMU_3DCONTEXT*/  \
		0x2058U, /* RGX_CR_PM_START_OF_MMU_3DCONTEXT*/  \
		0x2058U, /* RGX_CR_PM_START_OF_MMU_3DCONTEXT*/
#endif

#if defined(RGX_FIRMWARE)
typedef DLLIST_NODE							RGXFWIF_DLLIST_NODE;
#else
typedef struct {RGXFWIF_DEV_VIRTADDR p;
                RGXFWIF_DEV_VIRTADDR n;}	RGXFWIF_DLLIST_NODE;
#endif

typedef RGXFWIF_DEV_VIRTADDR  P                        RGXFWIF_INICFG_EXT_LOW_PRIe RGXFWIF_INICFG_CTXSWITCH_CLRMSK		~(