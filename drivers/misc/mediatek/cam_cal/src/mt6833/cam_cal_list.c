/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#define MAX_EEPROM_SIZE_16K 0x4000

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
#if defined(CONFIG_MTK_CAM_PD2279F)
	{S5KJN1SQPD2279F_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KJNSPD2279F_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3P9SP04PD2279F_SENSOR_ID, 0xA2, Common_read_region},
	{IMX355PD2279F_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1BPD2279F_SENSOR_ID, 0x20, Common_read_region_gc02m1bpd2279f},
	{GC02M1BPD2279HF_SENSOR_ID, 0x20, Common_read_region_gc02m1bpd2279hf},
#elif defined(CONFIG_MTK_CAM_PD2279)
	{S5KJNSPD2279_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KJNSV1PD2279_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX355PD2279_SENSOR_ID, 0xA2, Common_read_region},
#elif defined(CONFIG_MTK_CAM_PD2250)
	{OV13B10SUNPD2250_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10QTEPD2250_SENSOR_ID, 0xA4, Common_read_region},
	{S5K5E9PD2250_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1PD2250_SENSOR_ID, 0x20, Common_read_region_gc02m1pd2250},
#elif defined(CONFIG_MTK_CAM_PD2250F)
	{S5KJN1SQPD2250F_SENSOR_ID, 0xA0, Common_read_region},
	{HI1634QPD2250F_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1PD2250F_SENSOR_ID, 0x20, Common_read_region_gc02m1pd2250f},
#elif defined (CONFIG_MTK_CAM_PD2230)
	{OV13B10PD2230_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V1PD2230_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V2PD2230_SENSOR_ID, 0xA4, Common_read_region},
	{S5K5E9PD2230_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1PD2230_SENSOR_ID, 0x20, Common_read_region_gc02m1},
	{OV02B10PD2230_SENSOR_ID, 0xA4, Common_read_region_ov02b10pd2230},
#elif defined(CONFIG_MTK_CAM_PD2230F)
	{S5KJN1SQPD2230F_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI1634QPD2230F_SENSOR_ID, 0xA2, Common_read_region},
	{GC02M1BPD2230F_SENSOR_ID, 0x20, Common_read_region_gc02m1bpd2230f},
	{IMX355PD2230F_SENSOR_ID, 0xA2, Common_read_region},
/*vivo zhangyiteng add by eeprom i2c addr for PD2224(PD2166G) start 2022.11.30*/
#elif defined( CONFIG_MTK_CAM_PD2166G)
    {S5KJN1SQ03PD2166G_SENSOR_ID, 0xA0, Common_read_region},
	{OV13B10PD2224_SENSOR_ID, 0xA4, Common_read_region},
	{OV13B10V1PD2224_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1PD2166G_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1V1PD2166G_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1V2PD2166G_SENSOR_ID, 0xA4, Common_read_region},
	{S5K4H7PD2166G_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7V1PD2166G_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7V2PD2166G_SENSOR_ID, 0xA2, Common_read_region},
	{S5K4H7V3PD2166G_SENSOR_ID, 0xA2, Common_read_region},
/*vivo zhangyiteng add by eeprom i2c addr for PD2224(PD2166G) end 2022.11.30*/
#else
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX576_SENSOR_ID, 0xA2, Common_read_region},
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{IMX319_SENSOR_ID, 0xA2, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K3M5SX_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{IMX686_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{HI846_SENSOR_ID, 0xA0, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5KGD1SP_SENSOR_ID, 0xA8, Common_read_region, MAX_EEPROM_SIZE_16K},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
#endif
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}


