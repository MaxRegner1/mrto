/* accelhub motion sensor driver
 *
 * Copyright (C) 2016 MediaTek Inc.
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

#define pr_fmt(fmt) "[Gsensor] " fmt

#include "accelhub.h"
#include "SCP_power_monitor.h"
#include <SCP_sensorHub.h>
#include <accel.h>
#include <hwmsensor.h>

#define DEBUG 1
#define SW_CALIBRATION
#define ACCELHUB_AXIS_X 0
#define ACCELHUB_AXIS_Y 1
#define ACCELHUB_AXIS_Z 2
#define ACCELHUB_AXES_NUM 3
#define ACCELHUB_DATA_LEN 6
#define ACCELHUB_DEV_NAME                                                      \
	"accel_hub_pl" /* name must different with accel accelhub */
/* dadadadada */
enum ACCELHUB_TRC {
	ACCELHUB_TRC_FILTER = 0x01,
	ACCELHUB_TRC_RAWDATA = 0x02,
	ACCELHUB_TRC_IOCTL = 0x04,
	ACCELHUB_TRC_CALI = 0X08,
	ACCELHUB_TRC_INFO = 0X10,
};
struct accelhub_ipi_data {
	/*misc */
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest_status;
	int32_t static_cali[ACCELHUB_AXES_NUM];
	uint8_t static_cali_status;
	int32_t dynamic_cali[ACCELHUB_AXES_NUM];
	int direc