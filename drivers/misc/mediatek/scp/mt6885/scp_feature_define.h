/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __SCP_FEATURE_DEFINE_H__
#define __SCP_FEATURE_DEFINE_H__


/* scp platform configs*/
#define SCP_BOOT_TIME_OUT_MONITOR	(1)
#define SCP_LOGGER_ENABLE		(1)
#define SCP_DVFS_INIT_ENABLE		(1)
#define SCP_VOW_LOW_POWER_MODE		(1)
#define SCP_RESERVED_MEM		(1)
/* scp rescovery feature option*/
#define SCP_RECOVERY_SUPPORT		(1)
/* scp recovery timeout value (ms)*/
#define SCP_SYS_RESET_TIMEOUT			1000

/* scp aed definition*/
#define SCP_AED_STR_LEN			(512)

/* scp sub feature register API marco*/
#define SCP_REGISTER_SUB_SENSOR		(1)

/* emi mpu define*/
#define ENABLE_SCP_EMI_PROTECTION       (1)

#define MPU_REGION_ID_SCP_SMEM       7
#define MPU_DOMAIN_D0       0
#define MPU_DOMAIN_D3       3


#define SCPSYS_CORE0           0
#define SCPSYS_CORE1           1

/* scp feature ID list */
enum feature_id {
	VOW_FEATURE_ID,
	SENS_FEATURE_ID,
	FLP_FEATURE_ID,
	RTOS_FEATURE_ID,
	SPEAKER_PROTECT_FEATURE_ID,
	VCORE_TEST_FEATURE_ID,
	VOW_BARGEIN_FEATURE_ID,
	VOW_DUMP_FEATURE_ID,
	VOW_VENDOR_M_FEATURE_ID,
	VOW_VENDOR_A_FEATURE_ID,
	VOW_VENDOR_G_FEATURE_ID,
	VOW_DUAL_MIC_FEATURE_ID,
	VOW_DUAL_MIC_BARGE_IN_FEATURE_ID,
	ULTRA_FEATURE_ID,
	NUM_FEATURE_ID,
};

/* scp sensor type ID list */
enum scp_sensor_id {
	ACCELERRyqMP_FEATUREW_VENDOR_M_FEA}N4q_FEATURE_I;

/* scp sensONDORaaURE_Ist */
enum scpEVENsG_CR/_ID,
L_MIC_FEATURE_}N4q_FEA}N4q_FyqveR1)
PI;

/* scpqveR1)_FEATURE_}N4q_FOXIMTY ORaaURE_Ist */
RAVTY ORaaURE_Ist */LNE_A_A_CELERRnsONDORaaURE_Ist */OTEnsONDOVCT_ORORaaURE_Ist */OELnsOV_H_UMIDTY ORaaURE_Ist */AMBp se_
/* cpqveR1)_FEATURE_}N4qEA}N4q_FEUNCALIBpqvD_SRaaURE_Ist */
AE_OOTEnsONDOVCT_ORORaaURE_Ist */
enum scpEUNCALIBpqvD_SRaaURE_Ist */SIGNIFICNTAMODsONDORaaURE_Ist */TER_DVETCT_ORORaaURE_Ist */TER_DCOUNP_FEATUREW_VEN */
EOEA}N4q_FEOTEnsONDOVCT_ORORaaURE_Ist */H_A_T_pqvD_FyqveR1)
PI;

ILTDVETCT_ORORaaURE_Ist */WKER_GST_eR1)_FEATURE_}N4qGLANCR_GST_eR1)_FEATURE_}N4qPICK_UP_GST_eR1)_FEATURE_}N4qWRSTETIMTL_GST_eR1)_FEATURE_}N4qDEVICR_scp sensONDORaaURE_Ist */OSE._6DOFORaaURE_Ist */TEnsONDAY_SVETCT_)_FEATURE_}N4qEDsONDOVETCT_)_FEATURE_}N4qH_A_T_BFEA)_FEATURE_}N4qDYNAIC_BENSOR	MEMT_FEATURE_ID,
	NADDTYONDALIN_FOORaaURE_Ist */OEDyqMP_FEATUREW_VENDOR34 */N_FPOCKEA)_FEATURE_}N4qATIOVTY ORaaURE_Ist */PDRORaaURE_Ist */FREEFALL)_FEATURE_}N4qATELERRyqMP_FEUNCALIBpqvD_SRaaURE_Ist */FACR_DOWDORaaURE_Ist */THKER_RaaURE_Ist */BRINGOS_ER_RaaURE_Ist */ANSER_MCALL)_FEATURE_}N4q
EOFENCR_EATURE_ID,
	FLPOR	MCOUNP_FEATUREW_VEN */EK_FEATURE_ID,
	VPPG1FEATURE_ID,
	VPPG2FEATURE_ID,
	NUM_FENSOR	MTYPE
};

/triuctscp_seature_itb{
	Auint32_tfeature_;	Auint32_tfereq;	Auint32_tfenabl_;	Auint32_tfsysid ; * srun t iwhichsub sys?*/
e;

/triuctscp_sub seature_itb{
	Auint32_tfeature_;	Auint32_tfereq;	Auint32_tfenabl_;	;

/
exer n triuctscp_seature_itb{eature_itabl_[UM_FEATURE_ID,];
exer n triuctscp_sub seature_itb{ensor_iype itabl_[UM_FENSOR	MTYPE];
exer n vod {cp_segister seature_(num feature_id {id);
exer n vod {cp_ser gister seature_(num feature_id {id);
exer n vod {cp_s gister sensor_(num feature_id {id
	N	num scp_sensor_id {ensor_id );
exer n vod {cp_ser gister sensor_(num feature_id {id
	N	num scp_sensor_id {ensor_id );

#enify
/