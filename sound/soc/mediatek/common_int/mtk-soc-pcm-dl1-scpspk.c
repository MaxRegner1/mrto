/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mt_soc_pcm_scpspk.c
 *
 * Project:
 * --------
 *    Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio dl1 scp spk
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 *
 *
 ******************************************************************************
 */

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/

/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "audio_dma_buf_control.h"
#include "audio_ipi_client_spkprotect.h"
#include "audio_spkprotect_msg_id.h"
#include "mtk-auddrv-afe.h"
#include "mtk-auddrv-ana.h"
#include "mtk-auddrv-clk.h"
#include "mtk-auddrv-common.h"
#include "mtk-auddrv-def.h"
#include "mtk-auddrv-kernel.h"
#include "mtk-auddrv-scp-spkprotect-common.h"
#include "mtk-soc-afe-control.h"
#include "mtk-soc-pcm-common.h"
#include "mtk-soc-pcm-platform.h"

#include <linux/dma-mapping.h>

#ifdef CONFIG_SND_SOC_MTK_SCP_SMARTPA
#include "scp_helper.h"
#include <audio_dma_buf_control.h>
#include <audio_ipi_client_spkprotect.h>
#include <audio_task_manager.h>
#endif

#define use_wake_lock
#ifdef use_wake_lock
static DEFINE_SPINLOCK(scp_spk_lock);
struct wakeup_source scp_spk_suspend_lock;
#endif

#define MAX_PARLOAD_SIZE (10)
#define DEFAULT_PAYLOAD_SIZE (40)

static struct afe_mem_control_t *pdl1spkMemControl;

struct SPK_PROTECT_SERVICE {
	bool ipiwait;
	bool ipiresult;
};

#define SPKPROTECT_IPIMSG_TIMEOUT 50
#define SPKPROTECT_WAITCHECK_INTERVAL_MS 1

static struct snd_dma_buffer Dl1Spk_Playback_dma_buf; /* pre_allocate dram */
static struct snd_dma_buffer Dl1Spk_feedback_dma_buf; /* pre_allocate dram*/
static struct snd_dma_buffer
	Dl1Spk_runtime_feedback_dma_buf; /* real time for IV feedback buffer*/

static const int Dl1Spk_feedback_buf_offset =
	(SOC_NORMAL_USE_BUFFERSIZE_MAX * 2);
static unsigned int Dl1Spk_feedback_user;
static unsigned int mspkPlaybackDramState;
static unsigned int mspkPlaybackFeedbackDramState;
static int mspkiv_meminterface_type;
static bool vcore_dvfs_enable;

static struct audio_resv_dram_t *p_resv_dram;
static struct SPK_PROTECT_SERVICE spk_protect_service;
#ifdef CONFIG_MTK_TINYSYS_SCP_SUPPORT
static struct audio_resv_dram_t *p_resv_dram_normal;
static struct scp_reserve_mblock ScpReserveBuffer;
static struct snd_dma_buffer ScpDramBuffer;

static const int platformBufferOffset;
static struct snd_dma_buffer PlatformBuffer;
static const int SpkDL1BufferOffset = SOC_NORMAL_USE_BUFFERSIZE_MAX;
static struct snd_dma_buffer SpkDL1Buffer;

static int SpkIrq_mode = Soc_Aud_IRQ_MCU_MODE_IRQ7_MCU_MODE;
static uint32_t ipi_payload_buf[MAX_PARLOAD_SIZE];
#endif

/*
 *    function implementation
 */
static int mtk_dl1spk_probe(struct platform_device *pdev);
static int mtk_pcm_dl1spk_close(struct snd_pcm_substream *substream);
static int mtk_afe_dl1spk_probe(struct snd_soc_platform *platform);
static void set_dl1_spkbuffer(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params);
static void stop_spki2s2adc2_hardware(struct snd_pcm_substream *substream);
static void start_spki2s2adc2_hardware(struct snd_pcm_substream *substream);
static int audio_spk_pcm_dump_set(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol);
static int audio_spk_pcm_dump_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol);

static int mdl1spk_hdoutput_control;
static bool mdl1spkPrepareDone;

static const void *spk_irq_user_id;
static unsigned int spk_irq_cnt;
static struct device *mDev;
static const char *const dl1_scpspk_HD_output[] = {"Off", "On"};
static const char *const dl1_scpspk_pcmdump[] = {"Off", "normal_dump",
						 "split_dump"};
static bool scpspk_pcmdump;

static const struct soc_enum Audio_dl1spk_Enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dl1_scpspk_HD_output),
			    dl1_scpspk_HD_output),
};

static const struct soc_enum audio_dl1spk_pcmdump_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(dl1_scpspk_pcmdump), dl1_scpspk_pcmdump),
};

static int audio_dl1spk_hdoutput_get(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("Audio_AmpR_Get = %d\n", mdl1spk_hdoutput_control);
	ucontrol->value.integer.value[0] = mdl1spk_hdoutput_control;
	return 0;
}

static int audio_dl1spk_hdoutput_set(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s()\n", __func__);
	if (ucontrol->value.enumerated.item[0] >
	    ARRAY_SIZE(dl1_scpspk_HD_output)) {
		pr_warn("return -EINVAL\n");
		return -EINVAL;
	}

	mdl1spk_hdoutput_control = ucontrol->value.integer.value[0];

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_HDMI) == true) {
		pr_warn("return HDMI enabled\n");
		return 0;
	}
	return 0;
}

#ifdef use_wake_lock
static void scp_spk_int_wakelock(bool enable)
{
	spin_lock(&scp_spk_lock);
	if (enable)
		aud_wake_lock(&scp_spk_suspend_lock);
	else
		aud_wake_unlock(&scp_spk_suspend_lock);
	spin_unlock(&scp_spk_lock);
}
#endif

static int audio_irqcnt7_spk_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	AudDrv_Clk_On();
	ucontrol->value.integer.value[0] = Afe_Get_Reg(AFE_IRQ_MCU_CNT1);
	AudDrv_Clk_Off();
	return 0;
}

static int audio_irqcnt7_set(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s(), spk_irq_user_id = %p, spk_irq_cnt = %d, value = %ld\n",
		 __func__, spk_irq_user_id, spk_irq_cnt,
		 ucontrol->value.integer.value[0]);
	if (spk_irq_cnt == ucontrol->value.integer.value[0])
		return 0;

	spk_irq_cnt = ucontrol->value.integer.value[0];

	AudDrv_Clk_On();
	if (spk_irq_user_id && spk_irq_cnt)
		irq_update_user(spk_irq_user_id, SpkIrq_mode, 0, spk_irq_cnt);
	else
		pr_debug(
			"warn, cannot update irq counter, user_id = %p, spk_irq_cnt = %d\n",
			spk_irq_user_id, spk_irq_cnt);

	AudDrv_Clk_Off();
	return 0;
}

static const struct snd_kcontrol_new Audio_snd_dl1spk_controls[] = {
	SOC_ENUM_EXT("Audio_dl1spk_hd_Switch", Audio_dl1spk_Enum[0],
		     audio_dl1spk_hdoutput_get, audio_dl1spk_hdoutput_set),
	SOC_SINGLE_EXT("Audio spk IRQ7 CNT", SND_SOC_NOPM, 0, IRQ_MAX_RATE, 0,
		       audio_irqcnt7_spk_get, audio_irqcnt7_set),
	SOC_ENUM_EXT("Audio_spk_pcm_dump", audio_dl1spk_pcmdump_enum[0],
		     audio_spk_pcm_dump_get, audio_spk_pcm_dump_set),
};

static struct snd_pcm_hardware mtk_dl1spk_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_ADV_MT_FMTS,
	.rates = SOC_H