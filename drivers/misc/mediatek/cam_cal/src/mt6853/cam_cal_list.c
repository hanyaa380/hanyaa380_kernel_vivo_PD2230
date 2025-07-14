/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_16K 0x4000
#define MAX_EEPROM_SIZE_32K 0x8000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
#ifdef CONFIG_MTK_CAM_PD2215F_EX
/*vivo zhangyiteng add by eeprom i2c addr start 2022.04.24*/
	{S5KGW3SP13PD2215F_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_32K},
	{S5KJN1SQ03PD2215F_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846PD2215F_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1PD2215F_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_16K},
/*vivo zhangyiteng add by eeprom i2c addr end 2022.04.24*/
#endif
#ifdef CONFIG_MTK_CAM_PD2257F_EX
/*vivo cxc add by eeprom i2c addr start 2022.10.22*/
	{S5KGW3SP13PD2257F_SENSOR_ID, 0xA4, Common_read_region, MAX_EEPROM_SIZE_32K},
	{S5K3P9SP04PD2257F_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{GC02M1PD2257F_SENSOR_ID, 0x20, Common_read_region_gc02m1pd2257f},
	{GC02M1BPD2257F_SENSOR_ID, 0x6e, Common_read_region_gc02m1bpd2257f},
	{GC02M1BZPD2257F_SENSOR_ID, 0x6e, Common_read_region_gc02m1bzpd2257f},
/*vivo cxc add by eeprom i2c addr end 2022.10.22*/
#endif
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K,
		BL24SA64_write_region},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{OV16A10_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP_SENSOR_ID, 0xA8, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{S5KJD1_SENSOR_ID, 0xB0, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		DW9763_write_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{IMX481_SENSOR_ID, 0xA4, Common_read_region, DEFAULT_MAX_EEPROM_SIZE_8K,
		BL24SA64_write_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


