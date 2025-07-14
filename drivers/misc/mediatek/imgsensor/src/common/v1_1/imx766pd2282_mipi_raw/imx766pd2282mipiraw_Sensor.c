/*
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX766PD2282mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx766pd2282_Sensor_setting.h"
#include "imx766pd2282mipiraw_Sensor.h"
#include "imx766pd2282_eeprom.h"
/*chenhan add for ois*/
#include "../imgsensor_sensor.h"
#include "ois_core.h"
/*add end*/

#undef VENDOR_EDIT

#define USE_BURST_MODE 1

/***************Modify Following Strings for Debug**********************/
#define PFX "IMX766PD2282_camera_sensor"
/****************************   Modify end	**************************/
#define LOG_INF(format, args...) pr_err(PFX "[%s] " format, __func__, ##args)

static kal_uint8 qsc_flag;
BYTE imx766pd2282_LRC_data[352] = {0};
//static gain_shutter_sync shutter_compensat;

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static struct mutex ois_lock;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX766PD2282_SENSOR_ID,

	.checksum_value = 0x8ac2d94a,

	.pre = {	/* reg_B 4096x3072@30fps, w/ PDAF(HVBin)*/
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.cap = {	/* reg_B 4096x3072@30fps, w/ PDAF(HVBin)*/
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.normal_video = {	/*2 QBIN_VBin_4096x2304_30FPS */  //reg-MTK-12
		.pclk = 2922400000,
		.linelength = 15616,
		.framelength = 6238,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1574860000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.hs_video = { /*3 QBIN_VBin_1920x1080_120FPS */  //reg-D3
		.pclk = 1554800000,
		.linelength = 8816,
		.framelength = 1468,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 760690000,
		.max_framerate = 1200,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.slim_video = {	/*4 QBIN_VBin_1920x1080_240FPS */  //reg-D4
		.pclk = 3504800000,
		.linelength = 8816,
		.framelength = 1656,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1901710000,
		.max_framerate = 2400,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom1 = { /*2 QBIN_VBin_4096x2304_30FPS */  //reg-MTK-12
		.pclk = 2922400000,
		.linelength = 15616,
		.framelength = 6238,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1574860000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom2 = { /*6 QBIN_VBin_3840x2160_60FPS */  //reg-E2
		.pclk = 2849600000,
		.linelength = 15616,
		.framelength = 3040,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1206400000,
		.max_framerate = 600,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom3 = { /*7 QBIN_VBin_3840x2160_60FPS */  //reg-E2
		.pclk = 2849600000,
		.linelength = 15616,
		.framelength = 3040,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1206400000,
		.max_framerate = 600,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom4 = { /*reg B-4-HV 4096x3072@60fps, w/ PDAF(HVBin) SuperEis 2*/
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom5 = { /* reg A-1 8192x6144@15fps, PDAF*/
		.pclk = 1284400000,
		.linelength = 11552,
		.framelength = 7412,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8192,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1366860000,
		.max_framerate = 150,
		.cit_min_value = 9, /*shutter min value*/
		.cit_step_value = 1, /*shutter step value*/
	},
	.custom6 = { /*10 QBIN_VBin_4096x3072_30FPS */  //reg-MTK-21
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom7 = { /*11 QBIN_VBin_4096x3072_30FPS */  //reg-MTK-21
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom8 = { /*12 QBIN_VBin_4096x3072_30FPS */  //reg-MTK-19B
		.pclk = 1788800000,
		.linelength = 11552,
		.framelength = 5160,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom9 = { /*13 QBIN_VBin_4096x3072_30FPS */  //reg-21
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom10 = {/* reg_B-3 SHDR 4096x2304@30fps, F2-DOL, F-DOL 1ST*/
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 959770000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.custom11 = {/* MTK-12-S7 4096x2304@30fps vbin*/
		.pclk = 1788800000,
		.linelength = 15616,
		.framelength = 3818,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4096,
		.grabwindow_height = 2304,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 1854170000,
		.max_framerate = 300,
		.cit_min_value = 8, /*shutter min value*/
		.cit_step_value = 2, /*shutter step value*/
	},
	.min_gain = 64, /*1x gain*/
	.max_gain = 4096, /*16x gain*/
	.min_gain_iso = 50,
	.margin = 48,		/* sensor framelength & shutter margin */
	.min_shutter = 8,	/* min shutter */
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 0, /* 1, support; 0,not support */
	.sensor_mode_num = 16,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom5_delay_frame = 2,
	.custom6_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom7_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom8_delay_frame = 2,
	.custom9_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom10_delay_frame = 2,
	.custom11_delay_frame = 2,
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_2MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_CPHY, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_B,
	.mclk = 26, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	/*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
	.mipi_lane_num = SENSOR_MIPI_3_LANE,
	.i2c_addr_table = {0x34, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};
static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = 0x100,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
	.current_ae_effective_frame = 2,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[16] = {

	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* Preview VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* capture VBin*/
	{8192, 6144, 0,    768,  8192, 4608, 4096, 2304, 0,   0,   4096, 2304,  0,  0, 4096, 2304}, /* normal video HVBin*/
	{8192, 6144, 0,    896,  8192, 4352, 2048, 1088, 64,  4,   1920, 1080,  0,  0, 1920, 1080}, /* hs_video VBin*/
	{8192, 6144, 0,    896,  8192, 4352, 2048, 1088, 64,  4,   1920, 1080,  0,  0, 1920, 1080}, /* slim video VBin*/
	{8192, 6144, 0,    768,  8192, 4608, 4096, 2304, 0,   0,   4096, 2304,  0,  0, 4096, 2304}, //custom1 VBin
	{8192, 6144, 0,    896,  8192, 4352, 4096, 2176, 128, 8,   3840, 2160,  0,  0, 3840, 2160}, /* custom2 VBin*/
	{8192, 6144, 0,    896,  8192, 4352, 4096, 2176, 128, 8,   3840, 2160,  0,  0, 3840, 2160}, /* custom3 VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom4 VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 8192, 6144, 0,   0,   8192, 6144,  0,  0, 8192, 6144}, /* custom5 VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom6 VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom7 VBin*/
	{8192, 6144, 2048, 1536, 4096, 3072, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom8 roi*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom9 VBin*/
	{8192, 6144, 0,    0,    8192, 6144, 4096, 3072, 0,   0,   4096, 3072,  0,  0, 4096, 3072}, /* custom10 VBin*/
	{8192, 6144, 0,    768,  8192, 4608, 4096, 2304, 0,   0,   4096, 2304,  0,  0, 4096, 2304}, /* custom11 HVBin*/
};

static struct  SENSOR_RAWINFO_STRUCT imgsensor_raw_info = {
	 4096,//raw_weight
 	 3072,//raw_height
	 2,//raw_dataBit
	 BAYER_BGGR,//raw_colorFilterValue
	 64,//raw_blackLevel
	 85,//raw_viewAngle
	 10,//raw_bitWidth
	 64//raw_maxSensorGain
};
 /*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X36), unit : 10bit */

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
    .i4OffsetX = 0,
    .i4OffsetY = 0,
    .i4PitchX = 0,
    .i4PitchY = 0,
    .i4PairNum = 0,
    .i4SubBlkW = 0,
    .i4SubBlkH = 0,
    .i4PosL = {{0, 0} },
    .i4PosR = {{0, 0} },
    .i4BlockNumX = 0,
    .i4BlockNumY = 0,
    .i4LeFirst = 0,
    .i4Crop = {
        {0, 0}, {0, 0}, {0, 384}, {64, 228}, {64, 228},
        {0, 384}, {128, 456}, {128, 456}, {0, 0}, {0, 0}, {0, 0}, {0, 0} ,{2048, 1536}, {0, 0}, {0, 0}, {0, 384}, //bining-size/2,re:(2048-1120)/2=464 (1536-840)/2=348
    },  //{0, 1632}
    .iMirrorFlip = 3,
};
//the index order of VC_STAGGER_NE/ME/SE in array identify the order they are read out in MIPI transfer
static struct SENSOR_VC_INFO2_STRUCT SENSOR_VC_INFO2[16] = {
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//preview hvbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},  //4096 * 768
		},
		1
	},
	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00,//capture vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//video vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x240},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//hs video vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x0780, 0x0438},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x960, 0x10E},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//slim video hvbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x0780, 0x0438},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x960, 0x10E},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//custom1 vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x240},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//custom2 Vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x0F00, 0x0870},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x12C0, 0x21C},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//custom3 VBin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x0F00, 0x0870},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x12C0, 0x21C},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//custom4 Vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},  //4096 * 768
		},
		1
	},
 	{
		0x01, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom5 QRMSC 8192x6144_15FPS
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x2000, 0x1800},
			//{VC_PDAF_STATS, 0x00, 0x30, 0x1400, 0x0C00},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom6 VBin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},  //4096 * 768
		},
		1
	},
 	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom7 HVBin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x300},  //4096 * 768
		},
		1
	},
 	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00, //custom8 HVBin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0xE80, 0x0828},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0xA00, 0x600},  // 2048 * 1536
		},
		1
	},
	{
		0x04, 0x0a, 0x00, 0x08, 0x40, 0x00,//VBin custom9 4096x3072 2Exp
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x0300},
		},
		1
	},
	{
		0x04, 0x0a, 0x00, 0x08, 0x40, 0x00,//VBin custom10 4096x2304 2Exp
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0C00},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x0300},
		},
		1
	},
	{
		0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,//video vbin
		{
			{VC_STAGGER_NE, 0x00, 0x2b, 0x1000, 0x0900},
			{VC_PDAF_STATS_NE_PIX_1, 0x00, 0x30, 0x1400, 0x240},
		},
		1
	},
};

static int vivo_otp_read_when_power_on;
extern int MAIN_imx766pd2282_otp_read(void);
//extern u32 ois_type;


extern otp_error_code_t IMX766PD2282_OTP_ERROR_CODE;
MUINT32  moduleid_inf_main_imx766pd2282[2];
MUINT32  sn_inf_main_imx766pd2282[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main_imx766pd2282[4];
MUINT32  af_calib_inf_main_imx766pd2282[6];
extern u32 sensor_temperature[10];


static u8 imx766pd2282_QSC_setting[3072];

#if 0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF)};

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00) | ((get_byte>>8)&0x00ff);
}
#endif

static int is_sensor_write8_sequential(struct i2c_client *client,
 	u16 addr, u8 *val, u16 num)
  {
	int ret = 0;
	struct i2c_msg msg[1];
	int i = 0;
	u8 *wbuf;

	if (val == NULL) {
		pr_err("val array is null\n");
		ret = -ENODEV;
		goto p_err;
	}

	if (!client->adapter) {
		pr_err("Could not find adapter!\n");
		ret = -ENODEV;
		goto p_err;
	}

	wbuf = kzalloc((2 + (num * 2)), GFP_KERNEL);
	if (!wbuf) {
		pr_err("failed to alloc buffer for burst i2c\n");
		ret = -ENODEV;
		goto p_err;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2 + (num * 1);
	msg->buf = wbuf;
	wbuf[0] = (addr & 0xFF00) >> 8;
	wbuf[1] = (addr & 0xFF);
	for (i = 0; i < num; i++) {
		wbuf[(i * 1) + 2] = val[i];
	}

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		pr_err("i2c treansfer fail(%d)", ret);
		goto p_err_free;
	}

	LOG_INF("I2CW08(%d) [0x%04x] : 0x%04x\n", client->addr, addr, *val);
	kfree(wbuf);
	return 0;

	p_err_free:
	kfree(wbuf);
	p_err:
	return ret;
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF),
			(char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static kal_uint32 get_cur_exp_cnt(){
	kal_uint32 exp_cnt = 1;

	if (0x1 == (read_cmos_sensor_8(0x33D0) & 0x1)) { // DOL_EN
		if (0x1 == (read_cmos_sensor_8(0x33D1) & 0x3)) { // DOL_MODE
			exp_cnt = 3;
		} else {
			exp_cnt = 2;
		}
	}

	return exp_cnt;
}

#if USE_BURST_MODE
#define I2C_BUFFER_LEN 255 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx766pd2282_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
		/* Write when remain buffer size is less than 3 bytes
		 * or reach end of data
		 */
		if ((I2C_BUFFER_LEN - tosend) < 3
			|| IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
						tosend,
						imgsensor.i2c_write_id,
						3,
						imgsensor_info.i2c_speed);
			tosend = 0;
		}
	}
	return 0;
}
#else
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
static kal_uint16 imx766pd2282_table_write_cmos_sensor(kal_uint16 *para,
						 kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;


	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;
	}

	return 0;
}
#endif

static void imx766pd2282_get_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	int i, idx;

	for (i = 0; i < regNum; i++) {
		idx = 2 * i;
		regDa[idx + 1] = read_cmos_sensor_8(regDa[idx]);
		/*LOG_INF("%x %x", regDa[idx], regDa[idx+1]);*/
	}
}
static void imx766pd2282_set_pdaf_reg_setting(MUINT32 regNum, kal_uint16 *regDa)
{
	imx766pd2282_table_write_cmos_sensor(regDa, regNum*2);
}

static void set_dummy(void)
{
	kal_uint32 exp_cnt = get_cur_exp_cnt();

	LOG_INF("dummyline = %d, dummypixels = %d, imgsensor.frame_length = %d, exp_cnt = %d\n",
		imgsensor.dummy_line, imgsensor.dummy_pixel, imgsensor.frame_length, exp_cnt);

	/* return;*/ /* for test */
	write_cmos_sensor_8(0x0104, 0x01);

	write_cmos_sensor_8(0x0340, imgsensor.frame_length / exp_cnt >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length / exp_cnt & 0xFF);

	write_cmos_sensor_8(0x0104, 0x00);

}	/*	set_dummy  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(0x0101, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x03);
	break;
	}
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

#define MAX_CIT_LSHIFT 7
static void write_shutter(kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;

	int longexposure_times = 0;
	static int long_exposure_status;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10
				/ imgsensor.frame_length;
		LOG_INF("autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	}

	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		pr_err("enter long exposure mode, time is %d\n",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + imgsensor_info.margin;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3128, longexposure_times & 0x07);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3128, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		pr_err("exit long exposure mode");
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	LOG_INF("shutter =%d, framelength =%d\n",
		shutter, imgsensor.frame_length);
}	/*	write_shutter  */

static kal_uint32 shutter_set_odd_when_smaller_than_16(kal_uint32 shutter)
{
	return shutter >= 16?shutter:((shutter - 1) | 0x1);
}
static kal_uint32 align_shutter(kal_uint8 sensor_mode, kal_uint32 shutter)
{
	kal_uint32 output_shutter = 0;
	switch (sensor_mode) {
	case IMGSENSOR_MODE_PREVIEW:
	case IMGSENSOR_MODE_CAPTURE:
	case IMGSENSOR_MODE_VIDEO:
	case IMGSENSOR_MODE_HIGH_SPEED_VIDEO:
	case IMGSENSOR_MODE_SLIM_VIDEO:
	case IMGSENSOR_MODE_CUSTOM1:
	case IMGSENSOR_MODE_CUSTOM2:
	case IMGSENSOR_MODE_CUSTOM3:
	case IMGSENSOR_MODE_CUSTOM4:
	case IMGSENSOR_MODE_CUSTOM6:
	case IMGSENSOR_MODE_CUSTOM7:
	case IMGSENSOR_MODE_CUSTOM9:
	case IMGSENSOR_MODE_CUSTOM10:
	case IMGSENSOR_MODE_CUSTOM11:
		output_shutter = round_down(shutter - imgsensor_info.pre.cit_min_value, imgsensor_info.pre.cit_step_value) + imgsensor_info.pre.cit_min_value;
		break;
	case IMGSENSOR_MODE_CUSTOM5:
	case IMGSENSOR_MODE_CUSTOM8:
		output_shutter = shutter_set_odd_when_smaller_than_16(shutter);
		break;
	default:
		LOG_INF("Error sensormode");
	}
	output_shutter = (kal_uint32)max((output_shutter), imgsensor_info.pre.cit_min_value);
	LOG_INF("current sensorMode = %d shutter = %d output_shutter = %d",sensor_mode, shutter, output_shutter);
	return output_shutter;
}

static void compensated_again_to_reg(kal_uint32 shutter)
{
	u32 dgain_reg = 0;

	dgain_reg = (((100*imgsensor.shutter) / shutter - 100) * 256)/100;

	LOG_INF("dgain_reg = %d",dgain_reg);

	write_cmos_sensor_8(0x020E, 0x1 & 0xFF);
	write_cmos_sensor_8(0x020F, dgain_reg & 0xFF);

}
/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;

	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	shutter = align_shutter(imgsensor.sensor_mode, shutter);

	write_shutter(shutter);

	if((imgsensor.shutter < 50) && imgsensor.shutter > shutter ) {
		compensated_again_to_reg(shutter);
	}
} /* set_shutter */

static void set_shutter_frame_length(
				kal_uint32 shutter, kal_uint16 frame_length,
				kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	int longexposure_times = 0;
	static int long_exposure_status;

	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	shutter = align_shutter(imgsensor.sensor_mode, shutter);
	/* LOG_INF("shutter =%d, frame_time =%d\n", shutter, frame_time); */

	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger
	 * than frame exposure
	 */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure
	 * lines must be updated here.
	 */

	/* OV Recommend Solution */
	/* if shutter bigger than frame_length,
	 * should extend frame length first
	 */
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	//if (shutter > imgsensor.frame_length - imgsensor_info.margin)
	//	imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	while (shutter >= 65535) {
		shutter = shutter / 2;
		longexposure_times += 1;
	}

	if (longexposure_times > 0) {
		pr_err("enter long exposure mode, time is %d\n",
			longexposure_times);
		long_exposure_status = 1;
		imgsensor.frame_length = shutter + imgsensor_info.margin;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3128, longexposure_times & 0x07);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	} else if (long_exposure_status == 1) {
		long_exposure_status = 0;
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x3128, 0x00);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);

		pr_err("exit long exposure mode");
	}

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		    write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor_8(0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	if((imgsensor.shutter < 50) && imgsensor.shutter > shutter ) {
		compensated_again_to_reg(shutter);
	}

	LOG_INF(
	    "Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d, autoflicker_en=%d\n",
	    shutter, imgsensor.frame_length,
	    frame_length, dummy_line,
	    read_cmos_sensor_8(0x0350), imgsensor.autoflicker_en);
}	/* set_shutter_frame_length */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	// 766 max gain: 64x
	kal_uint16 reg_gain = 0x0;
	kal_uint16 gain_value = gain;


	if (gain_value < imgsensor_info.min_gain || gain_value > imgsensor_info.max_gain) {
		LOG_INF("Error: gain value out of range");

		if (gain_value < imgsensor_info.min_gain)
			gain_value = imgsensor_info.min_gain;
		else if (gain_value > imgsensor_info.max_gain)
			gain_value = imgsensor_info.max_gain;
	}

	reg_gain = 16384 - (16384*64)/gain_value;
	return (kal_uint16) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;
	kal_uint16 t_gain;
	kal_uint16 i = 0;

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_INF("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}
	t_gain = gain;
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5) {
		LOG_INF("full size gain is over 16x");
		if (gain > 1024)
			gain = 1024;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);

#if 1
	if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5 || imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM8) {
		for (i = 0; i < ARRAY_SIZE(gains); i++) {
			if (gains[i].permile >= gain) {
				write_cmos_sensor_8(0x38C4, gains[i].code1);
				write_cmos_sensor_8(0x38C5, gains[i].code2);
				//write_cmos_sensor_8(0x38E0, gains[i].code3);
				//write_cmos_sensor_8(0x38E1, gains[i].code4);
				write_cmos_sensor_8(0x3890, gains[i].code3);
				write_cmos_sensor_8(0x3891, gains[i].code4);
				//write_cmos_sensor_8(0x3894, gains[i].code7);
				//write_cmos_sensor_8(0x3895, gains[i].code8);
				LOG_INF("gain = %d, reg_gain = 0x%x code(0x%x,0x%x,0x%x,0x%x)\n",
						gain, reg_gain, gains[i].code1, gains[i].code2, gains[i].code3, gains[i].code4);
				break;
			}
		}
	} else {
		write_cmos_sensor_8(0x3890, 0x00);
		write_cmos_sensor_8(0x3891, 0x00);
	}
#endif
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable) {
		write_cmos_sensor_8(0x460D, 0x07);
		write_cmos_sensor_8(0x3960, 0x00);
		write_cmos_sensor_8(0x44d0, 0x00);
		write_cmos_sensor_8(0x44d1, 0x16);
		write_cmos_sensor_8(0x4102, 0xff);
		write_cmos_sensor_8(0x410e, 0x00);
		write_cmos_sensor_8(0x410f, 0x00);
		write_cmos_sensor_8(0x401a, 0x00);
		write_cmos_sensor_8(0x0100, 0x01);
		}
	else
		write_cmos_sensor_8(0x0100, 0x00);
	return ERROR_NONE;
}

#define FMC_GPH_START		do { \
					write_cmos_sensor_8(0x0104, 0x01); \
					write_cmos_sensor_8(0x3010, 0x02); \
				} while (0)

#define FMC_GPH_END		do { \
					write_cmos_sensor_8(0x0104, 0x00); \
					write_cmos_sensor_8(0x3010, 0x00); \
				} while (0)

static kal_uint32 seamless_switch(enum MSDK_SCENARIO_ID_ENUM scenario_id,
	kal_uint32 shutter, kal_uint32 gain,
	kal_uint32 shutter_2ndframe, kal_uint32 gain_2ndframe)
{

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		{
			spin_lock(&imgsensor_drv_lock);
			imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
			imgsensor.autoflicker_en = KAL_FALSE;
			imgsensor.pclk = imgsensor_info.pre.pclk;
			imgsensor.line_length = imgsensor_info.pre.linelength;
			imgsensor.frame_length = imgsensor_info.pre.framelength;
			imgsensor.min_frame_length = imgsensor_info.pre.framelength;
			spin_unlock(&imgsensor_drv_lock);

			FMC_GPH_START;
			imx766pd2282_table_write_cmos_sensor(imx766pd2282_preview_setting,
					sizeof(imx766pd2282_preview_setting) / sizeof(kal_uint16));
			set_shutter(shutter);
			set_gain(gain);
			FMC_GPH_END;
		}
		break;
		case MSDK_SCENARIO_ID_CUSTOM8:
		{
			spin_lock(&imgsensor_drv_lock);
			imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
			imgsensor.autoflicker_en = KAL_FALSE;
			imgsensor.pclk = imgsensor_info.custom8.pclk;
			imgsensor.line_length = imgsensor_info.custom8.linelength;
			imgsensor.frame_length = imgsensor_info.custom8.framelength;
			imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
			spin_unlock(&imgsensor_drv_lock);

			FMC_GPH_START;
			imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom8_setting,
					sizeof(imx766pd2282_custom8_setting) / sizeof(kal_uint16));
			set_shutter(shutter);
			set_gain(gain);
			FMC_GPH_END;
		}
		break;
		default:
		{
			LOG_INF("%s error! wrong setting in set_seamless_switch = %d", __func__, scenario_id);
			write_cmos_sensor_8(0x3010, 0x00);
			return 0xff;
		}
	}

	LOG_INF("%s success, scenario is switched to %d", __func__, scenario_id);

	if (shutter_2ndframe != 0)
		set_shutter(shutter_2ndframe);
	if (gain_2ndframe != 0)
		set_gain(gain_2ndframe);


	return 0;
}

static void sensor_init(void)
{
	LOG_INF("E\n");
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_init_setting,
		sizeof(imx766pd2282_init_setting)/sizeof(kal_uint16));

	/*enable temperature sensor, TEMP_SEN_CTL:*/
	write_cmos_sensor_8(0x0138, 0x01);

	set_mirror_flip(imgsensor.mirror);

	LOG_INF("X");
}	/*	  sensor_init  */

static void preview_setting(void)
{
	LOG_INF("E Preview 4096x3072 30fps HVBin\n");

	imx766pd2282_table_write_cmos_sensor(imx766pd2282_preview_setting,
		sizeof(imx766pd2282_preview_setting)/sizeof(kal_uint16));

	LOG_INF("X");
} /* preview_setting */

/*full size 30fps*/
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s Capture 4096x3072 30fps HVBin\n", __func__);
	/*************MIPI output setting************/

	imx766pd2282_table_write_cmos_sensor(imx766pd2282_capture_30_setting,
		sizeof(imx766pd2282_capture_30_setting)/sizeof(kal_uint16));
	LOG_INF("%s(PD 012515) 30 fpsX\n", __func__);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s normal_video 4096x2304 30fps HVBin\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_normal_video_setting,
	sizeof(imx766pd2282_normal_video_setting)/sizeof(kal_uint16));
	LOG_INF("X\n");
}

static void slim_video_setting(void)
{
	LOG_INF("%s 1920x1080 240fps HVBin E\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_slim_video_setting,
		sizeof(imx766pd2282_slim_video_setting)/sizeof(kal_uint16));
}

static void hs_video_setting(void)
{
	LOG_INF("%s 1920x1080 120fps HVBin E\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_hs_video_setting,
		sizeof(imx766pd2282_hs_video_setting)/sizeof(kal_uint16));
}

static void custom1_setting(void)
{
	LOG_INF("%s Preview Mode 4096x3072 30fps HVBin\n", __func__);
	/*************MIPI output setting************/

	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom1_setting,
		sizeof(imx766pd2282_custom1_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}


static void custom2_setting(void)
{
	LOG_INF("%s 3840x2160 60fps Video@60fps VBin\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom2_setting,
		sizeof(imx766pd2282_custom2_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom3_setting(void)
{
	LOG_INF("%s SuperEIS 3840x2160 60fps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom3_setting,
		sizeof(imx766pd2282_custom3_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}
static void custom4_setting(void)
{
	LOG_INF("%s 4095x3072 60fps HVBin PDAF\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom4_setting,
		sizeof(imx766pd2282_custom4_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

/*full size 16M@24fps*/
static void custom5_setting(void)
{
	LOG_INF("%s Full-Remosaic 10fps E! currefps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom5_setting,
		sizeof(imx766pd2282_custom5_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom6_setting(void)
{
	LOG_INF("%s Dual Slave 2160x1620 30fps HVBin\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom6_setting,
		sizeof(imx766pd2282_custom6_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom7_setting(void)
{
	LOG_INF("%s Video Mode 4096x2304 60fps HVBin\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom7_setting,
		sizeof(imx766pd2282_custom7_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom8_setting(void)
{
	LOG_INF("%s Preview Mode 4096x3072 30fps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom8_setting,
		sizeof(imx766pd2282_custom8_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom9_setting(void)
{
	LOG_INF("%s Stagger HDR 4:3(4096x3072) 30fps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom9_setting,
		sizeof(imx766pd2282_custom9_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom10_setting(void)
{
	LOG_INF("%s Stagger HDR 16:9(4096x2304) 30fps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom10_setting,
		sizeof(imx766pd2282_custom10_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void custom11_setting(void)
{
	LOG_INF("%s 16:9 (4096x2304) 30fps\n", __func__);
	imx766pd2282_table_write_cmos_sensor(imx766pd2282_custom11_setting,
		sizeof(imx766pd2282_custom11_setting)/sizeof(kal_uint16));

	LOG_INF("X");
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
	kal_uint16 realtime_fps = 0;
	kal_uint16 exposure_cnt = 0;

	if (le) {
		exposure_cnt++;
	}

	if (me) {
		exposure_cnt++;
	}

	if (se) {
		exposure_cnt++;
	}

	le = (kal_uint16)max(imgsensor_info.min_shutter, (kal_uint32)le);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = max((kal_uint32)(le + me + se + imgsensor_info.margin),
		imgsensor.min_frame_length);
	imgsensor.frame_length = min(imgsensor.frame_length, imgsensor_info.max_frame_length);
	spin_unlock(&imgsensor_drv_lock);

	if (le) {
		le = round_down(le, 4);
	}

	if (me) {
		me = round_down(me, 4);
	}

	if (se) {
		se = round_down(se, 4);
	}

	LOG_INF("E! le:0x%x, me:0x%x, se:0x%x autoflicker_en %d frame_length %d\n",
		le, me, se, imgsensor.autoflicker_en, imgsensor.frame_length);

	write_cmos_sensor_8(0x0104, 0x01);
	if (imgsensor.autoflicker_en) {
		realtime_fps =
			imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8); /*FRM_LENGTH_LINES[15:8]*/
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);/*FRM_LENGTH_LINES[7:0]*/
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, (imgsensor.frame_length >> 8));
		write_cmos_sensor_8(0x0341, (imgsensor.frame_length) & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}
	write_cmos_sensor_8(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, le & 0xFF);
	/* Muddle exposure */
	if (me != 0) {
		/*MID_COARSE_INTEG_TIME[15:8]*/
		write_cmos_sensor_8(0x313A, (me >> 8) & 0xFF);
		/*MID_COARSE_INTEG_TIME[7:0]*/
		write_cmos_sensor_8(0x313B, me & 0xFF);
	}
	/* Short exposure */
	write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor_8(0x0225, se & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
}

static void hdr_write_tri_gain(kal_uint16 lgain, kal_uint16 mg, kal_uint16 sg)
{
	kal_uint16 reg_lg, reg_mg, reg_sg;

	reg_lg = gain2reg(lgain);
	reg_mg = gain2reg(mg);
	reg_sg = gain2reg(sg);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_lg;
	spin_unlock(&imgsensor_drv_lock);
	write_cmos_sensor_8(0x0104, 0x01);
	/* Long Gian */
	write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_lg & 0xFF);
	/* Middle Gian */
	if (mg != 0) {
		write_cmos_sensor_8(0x313C, (reg_mg>>8) & 0xFF);
		write_cmos_sensor_8(0x313D, reg_mg & 0xFF);
	}
	/* Short Gian */
	write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
	write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	LOG_INF(
		"lgain:0x%x, reg_lg:0x%x, sg:0x%x, reg_mg:0x%x, mg:0x%x, reg_sg:0x%x\n",
		lgain, reg_lg, mg, reg_mg, sg, reg_sg);
}

static u8 read_cmos_eeprom_8(kal_uint16 addr)
{
	u8 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, 0xA0);
	return get_byte;
}

static void read_sensor_Cali(void)
{
	kal_uint16 idx = 0, addr_qsc = 0x435C;
	for (idx = 0; idx < 3072; idx++) {
		addr_qsc = 0x435C + idx;
		imx766pd2282_QSC_setting[idx] = read_cmos_eeprom_8(addr_qsc);
	}
}

static void write_sensor_QSC(void)
{
	struct IMGSENSOR_I2C_CFG *pi2c_cfg = NULL;

	pi2c_cfg = imgsensor_i2c_get_device();
	write_cmos_sensor_8(0x86A9, 0x4E);

	is_sensor_write8_sequential(pi2c_cfg->pinst->pi2c_client, 0xC800, imx766pd2282_QSC_setting, 3072);

	write_cmos_sensor_8(0x32D2, 0x01);
}

/*************************************************************************
 * FUNCTION
 *	ois_driver_destroy
 *
 * DESCRIPTION
 *	chenhan add this function to destroy ois instance
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 *************************************************************************/
/*static kal_uint32 ois_driver_destroy(void)
{
	LOG_INF("E");
	mutex_lock(&ois_lock);
	ois_interface_destroy();
	mutex_unlock(&ois_lock);
	LOG_INF("X");

	return ERROR_NONE;
}
*/
#if 0
/*************************************************************************
 * FUNCTION
 *	ois_driver_create
 *
 * DESCRIPTION
 *	chenhan add this function to create & init ois for multi-gyro solution
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 *************************************************************************/
static kal_uint32 ois_driver_create(int sat_mode)
{
	struct IMGSENSOR_I2C_CFG *pi2c_cfg = NULL;

	pi2c_cfg = imgsensor_i2c_get_device();
	if (pi2c_cfg && pi2c_cfg->pinst) {
		LOG_INF("E (sensor=0x%x i2c=%p)", imgsensor_info.sensor_id, pi2c_cfg->pinst->pi2c_client);
		mutex_lock(&ois_lock);
		ois_interface_create(pi2c_cfg->pinst->pi2c_client, NULL, OISDRV_RUMBAS4SW, 0, sat_mode);
		mutex_unlock(&ois_lock);
		LOG_INF("X success");
	}  else {
		LOG_INF("X fail(cfg=%p, inst=%p)", pi2c_cfg, pi2c_cfg->pinst);
	}

	return ERROR_NONE;
}
#endif

#if 1
/*************************************************************************
 * FUNCTION
 *	ois_fw_check
 *
 * DESCRIPTION
 *	chenhan add this function do ois fw check and update when start camera provider
 *
 * PARAMETERS
 *	*params : sensor device handle need to pass to ois
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 ois_fw_check(UINT8 *params)
{
	struct IMGSENSOR_I2C_CFG *pi2c_cfg = NULL;

	if (!params) {
		pr_debug("ois_: sensor device handle error");
		return ERROR_NONE;
	}

	pi2c_cfg = imgsensor_i2c_get_device();
	if (pi2c_cfg && pi2c_cfg->pinst) {
		pr_info("ois_: fw check start(sensor=0x%x i2c=%p)", imgsensor_info.sensor_id, pi2c_cfg->pinst->pi2c_client);
		mutex_lock(&ois_lock);
		ois_interface_create(pi2c_cfg->pinst->pi2c_client, (struct device *)params, OISDRV_RUMBAS4SW, 1, 0);
		ois_interface_dispatcher(AFIOC_X_OIS_FWUPDATE, NULL);
		ois_interface_destroy();
		mutex_unlock(&ois_lock);
		pr_info("ois_: fw check end");
	}  else {
		pr_debug("ois_: i2c instance fail(cfg=%p, inst=%p)", pi2c_cfg, pi2c_cfg->pinst);
	}

	return ERROR_NONE;
}
#endif

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			if (*sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, *sensor_id);
				vivo_otp_read_when_power_on = MAIN_imx766pd2282_otp_read();
				read_sensor_Cali();
				//ois_type = read_cmos_eeprom_8(0x0015);
				return ERROR_NONE;
			}

			LOG_INF("Read sensor id fail, id: 0x%x,sensor_id = 0x%x,imgsensor_info.sensor_id = 0x%x\n",
				imgsensor.i2c_write_id,*sensor_id,imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;
	mutex_init(&ois_lock);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = ((read_cmos_sensor_8(0x0016) << 8)
					| read_cmos_sensor_8(0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n",
				imgsensor.i2c_write_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	if (!qsc_flag) {
		LOG_INF("write_sensor_QSC Start\n");
		write_sensor_QSC();
		LOG_INF("write_sensor_QSC End\n");
		qsc_flag = 1;
	}

	/* initail sequence write in  */
	sensor_init();

	sensor_temperature[0] = 0;
	LOG_INF("sensor_temperature[0] = %d\n", sensor_temperature[0]);



	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*chenhan add for ois destroy*/
	//ois_driver_destroy();
	/*add end*/

	/*No Need to implement this function*/
	LOG_INF("sensor_temperature[0] = %d\n", sensor_temperature[0]);
	write_cmos_sensor_8(0x0100, 0x00);
	qsc_flag = 0;
	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			imgsensor.current_fps,
			imgsensor_info.cap.max_framerate / 10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;

	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	/* set_mirror_flip(imgsensor.mirror); */

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);

	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
				MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/*imgsensor.video_mode = KAL_TRUE;*/
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/*imgsensor.current_fps = 300;*/
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	/*set_mirror_flip(imgsensor.mirror);*/

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom1_setting();

	return ERROR_NONE;
}	/* custom1 */

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();

	return ERROR_NONE;
}	/* custom2 */

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();

	return ERROR_NONE;
}	/* custom3 */
static kal_uint32 custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	imgsensor.pclk = imgsensor_info.custom4.pclk;
	imgsensor.line_length = imgsensor_info.custom4.linelength;
	imgsensor.frame_length = imgsensor_info.custom4.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom4_setting();

	return ERROR_NONE;
}	/* custom4 */

static kal_uint32 custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom5_setting();

	return ERROR_NONE;
}	/* custom5 */

static kal_uint32 custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
	imgsensor.pclk = imgsensor_info.custom6.pclk;
	imgsensor.line_length = imgsensor_info.custom6.linelength;
	imgsensor.frame_length = imgsensor_info.custom6.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom6_setting();

	return ERROR_NONE;
}	/* custom6 */

static kal_uint32 custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
	imgsensor.pclk = imgsensor_info.custom7.pclk;
	imgsensor.line_length = imgsensor_info.custom7.linelength;
	imgsensor.frame_length = imgsensor_info.custom7.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom7_setting();

	return ERROR_NONE;
}	/* custom7 */

static kal_uint32 custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
	imgsensor.pclk = imgsensor_info.custom8.pclk;
	imgsensor.line_length = imgsensor_info.custom8.linelength;
	imgsensor.frame_length = imgsensor_info.custom8.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom8_setting();

	return ERROR_NONE;
}	/* custom8 */

static kal_uint32 custom9(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM9;
	imgsensor.pclk = imgsensor_info.custom9.pclk;
	imgsensor.line_length = imgsensor_info.custom9.linelength;
	imgsensor.frame_length = imgsensor_info.custom9.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom9.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom9_setting();

	return ERROR_NONE;
}	/* custom9 */

static kal_uint32 custom10(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM10;
	imgsensor.pclk = imgsensor_info.custom10.pclk;
	imgsensor.line_length = imgsensor_info.custom10.linelength;
	imgsensor.frame_length = imgsensor_info.custom10.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom10.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom10_setting();

	return ERROR_NONE;
}	/* custom10 */

static kal_uint32 custom11(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM11;
	imgsensor.pclk = imgsensor_info.custom11.pclk;
	imgsensor.line_length = imgsensor_info.custom11.linelength;
	imgsensor.frame_length = imgsensor_info.custom11.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom11.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom11_setting();

	return ERROR_NONE;
}	/* custom11 */

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width =
		imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height =
		imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width =
		imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height =
		imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width =
		imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =
		imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width =
		imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =
		imgsensor_info.custom5.grabwindow_height;

	sensor_resolution->SensorCustom6Width =
		imgsensor_info.custom6.grabwindow_width;
	sensor_resolution->SensorCustom6Height =
		imgsensor_info.custom6.grabwindow_height;

	sensor_resolution->SensorCustom7Width =
		imgsensor_info.custom7.grabwindow_width;
	sensor_resolution->SensorCustom7Height =
		imgsensor_info.custom7.grabwindow_height;

	sensor_resolution->SensorCustom8Width =
		imgsensor_info.custom8.grabwindow_width;
	sensor_resolution->SensorCustom8Height =
		imgsensor_info.custom8.grabwindow_height;

	sensor_resolution->SensorCustom9Width =
		imgsensor_info.custom9.grabwindow_width;
	sensor_resolution->SensorCustom9Height =
		imgsensor_info.custom9.grabwindow_height;

	sensor_resolution->SensorCustom10Width =
		imgsensor_info.custom10.grabwindow_width;
	sensor_resolution->SensorCustom10Height =
		imgsensor_info.custom10.grabwindow_height;

	sensor_resolution->SensorCustom11Width =
		imgsensor_info.custom11.grabwindow_width;
	sensor_resolution->SensorCustom11Height =
		imgsensor_info.custom11.grabwindow_height;
	return ERROR_NONE;
} /* get_resolution */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
	sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->Custom6DelayFrame = imgsensor_info.custom6_delay_frame;
	sensor_info->Custom7DelayFrame = imgsensor_info.custom7_delay_frame;
	sensor_info->Custom8DelayFrame = imgsensor_info.custom8_delay_frame;
	sensor_info->Custom9DelayFrame = imgsensor_info.custom9_delay_frame;
	sensor_info->Custom10DelayFrame = imgsensor_info.custom10_delay_frame;
	sensor_info->Custom11DelayFrame = imgsensor_info.custom11_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV_QPD;

	sensor_info->HDR_Support = 5;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame;
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom7.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom8.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		sensor_info->SensorGrabStartX = imgsensor_info.custom9.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom9.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom9.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		sensor_info->SensorGrabStartX = imgsensor_info.custom10.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom10.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom10.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM11:
		sensor_info->SensorGrabStartX = imgsensor_info.custom11.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom11.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom11.mipi_data_lp2hs_settle_dc;
		break;
	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}

	return ERROR_NONE;
}	/*	get_info  */

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		custom4(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		custom5(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		custom6(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		custom7(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		custom8(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		custom9(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		custom10(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM11:
		custom11(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /*enable auto flicker*/
		imgsensor.autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF(
				"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
				, framerate
				, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10
					/ imgsensor_info.cap.linelength;

		if (frame_length > imgsensor_info.max_frame_length) {
			LOG_INF(
				"Warning: frame_length %d > max_frame_length %d!\n"
				, frame_length
				, imgsensor_info.max_frame_length);
			break;
		}

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.cap.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
				+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
				/ imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom1.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
				/ imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
				/ imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom3.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10
				/ imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom4.framelength)
			? (frame_length - imgsensor_info.custom4.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom4.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10
				/ imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom5.framelength)
			? (frame_length - imgsensor_info.custom5.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom5.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		frame_length = imgsensor_info.custom6.pclk / framerate * 10
				/ imgsensor_info.custom6.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom6.framelength)
			? (frame_length - imgsensor_info.custom6.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom6.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		frame_length = imgsensor_info.custom7.pclk / framerate * 10
				/ imgsensor_info.custom7.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom7.framelength)
			? (frame_length - imgsensor_info.custom7.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom7.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		frame_length = imgsensor_info.custom8.pclk / framerate * 10
				/ imgsensor_info.custom8.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom8.framelength)
			? (frame_length - imgsensor_info.custom8.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom8.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		frame_length = imgsensor_info.custom9.pclk / framerate * 10
				/ imgsensor_info.custom9.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom9.framelength)
			? (frame_length - imgsensor_info.custom9.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom9.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		frame_length = imgsensor_info.custom10.pclk / framerate * 10
				/ imgsensor_info.custom10.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom10.framelength)
			? (frame_length - imgsensor_info.custom10.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom10.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM11:
		frame_length = imgsensor_info.custom11.pclk / framerate * 10
				/ imgsensor_info.custom11.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.custom11.framelength)
			? (frame_length - imgsensor_info.custom11.framelength)
			: 0;
		imgsensor.frame_length =
			imgsensor_info.custom11.framelength
			+ imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		*framerate = imgsensor_info.custom4.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		*framerate = imgsensor_info.custom5.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		*framerate = imgsensor_info.custom6.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		*framerate = imgsensor_info.custom7.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		*framerate = imgsensor_info.custom8.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM9:
		*framerate = imgsensor_info.custom9.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM10:
		*framerate = imgsensor_info.custom10.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM11:
		*framerate = imgsensor_info.custom11.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}
static kal_uint32 imx766pd2282_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	pr_debug("%s\n", __func__);

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	pr_debug(
		"[%s] ABS_GAIN_GR:%d, grgain_32:%d\n, ABS_GAIN_R:%d, rgain_32:%d\n, ABS_GAIN_B:%d, bgain_32:%d,ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR, grgain_32,
		pSetSensorAWB->ABS_GAIN_R, rgain_32,
		pSetSensorAWB->ABS_GAIN_B, bgain_32,
		pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

	write_cmos_sensor_8(0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor_8(0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}
static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable)
		write_cmos_sensor_8(0x0601, 0x01); /*100% Color bar*/
	else
		write_cmos_sensor_8(0x0601, 0x00); /*No pattern*/

	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 imx766pd2282_ana_gain_table[] = {
	100000,
	100098,
	100196,
	100294,
	100392,
	100491,
	100589,
	100688,
	100787,
	100887,
	100986,
	101086,
	101186,
	101286,
	101386,
	101487,
	101587,
	101688,
	101789,
	101891,
	101992,
	102094,
	102196,
	102298,
	102400,
	102503,
	102605,
	102708,
	102811,
	102915,
	103018,
	103122,
	103226,
	103330,
	103434,
	103539,
	103644,
	103749,
	103854,
	103959,
	104065,
	104171,
	104277,
	104383,
	104490,
	104597,
	104703,
	104811,
	104918,
	105026,
	105133,
	105242,
	105350,
	105458,
	105567,
	105676,
	105785,
	105895,
	106004,
	106114,
	106224,
	106334,
	106445,
	106556,
	106667,
	106778,
	106889,
	107001,
	107113,
	107225,
	107338,
	107450,
	107563,
	107676,
	107789,
	107903,
	108017,
	108131,
	108245,
	108360,
	108475,
	108590,
	108705,
	108820,
	108936,
	109052,
	109168,
	109285,
	109402,
	109519,
	109636,
	109753,
	109871,
	109989,
	110108,
	110226,
	110345,
	110464,
	110583,
	110703,
	110823,
	110943,
	111063,
	111183,
	111304,
	111425,
	111547,
	111668,
	111790,
	111913,
	112035,
	112158,
	112281,
	112404,
	112527,
	112651,
	112775,
	112900,
	113024,
	113149,
	113274,
	113400,
	113525,
	113651,
	113778,
	113904,
	114031,
	114158,
	114286,
	114413,
	114541,
	114670,
	114798,
	114927,
	115056,
	115186,
	115315,
	115445,
	115576,
	115706,
	115837,
	115968,
	116100,
	116232,
	116364,
	116496,
	116629,
	116762,
	116895,
	117029,
	117162,
	117297,
	117431,
	117566,
	117701,
	117837,
	117972,
	118108,
	118245,
	118382,
	118519,
	118656,
	118794,
	118931,
	119070,
	119208,
	119347,
	119487,
	119626,
	119766,
	119906,
	120047,
	120188,
	120329,
	120471,
	120612,
	120755,
	120897,
	121040,
	121183,
	121327,
	121471,
	121615,
	121760,
	121905,
	122050,
	122196,
	122342,
	122488,
	122635,
	122782,
	122929,
	123077,
	123225,
	123373,
	123522,
	123671,
	123821,
	123971,
	124121,
	124272,
	124423,
	124574,
	124726,
	124878,
	125031,
	125183,
	125337,
	125490,
	125644,
	125799,
	125953,
	126108,
	126264,
	126420,
	126576,
	126733,
	126890,
	127047,
	127205,
	127363,
	127522,
	127681,
	127840,
	128000,
	128160,
	128321,
	128482,
	128643,
	128805,
	128967,
	129130,
	129293,
	129456,
	129620,
	129785,
	129949,
	130114,
	130280,
	130446,
	130612,
	130779,
	130946,
	131114,
	131282,
	131451,
	131620,
	131789,
	131959,
	132129,
	132300,
	132471,
	132642,
	132815,
	132987,
	133160,
	133333,
	133507,
	133681,
	133856,
	134031,
	134207,
	134383,
	134560,
	134737,
	134914,
	135092,
	135271,
	135450,
	135629,
	135809,
	135989,
	136170,
	136352,
	136533,
	136716,
	136898,
	137082,
	137265,
	137450,
	137634,
	137820,
	138005,
	138192,
	138378,
	138566,
	138753,
	138942,
	139130,
	139320,
	139510,
	139700,
	139891,
	140082,
	140274,
	140466,
	140659,
	140853,
	141047,
	141241,
	141436,
	141632,
	141828,
	142025,
	142222,
	142420,
	142618,
	142817,
	143017,
	143217,
	143417,
	143619,
	143820,
	144023,
	144225,
	144429,
	144633,
	144837,
	145042,
	145248,
	145455,
	145661,
	145869,
	146077,
	146286,
	146495,
	146705,
	146915,
	147126,
	147338,
	147550,
	147763,
	147977,
	148191,
	148406,
	148621,
	148837,
	149054,
	149271,
	149489,
	149708,
	149927,
	150147,
	150367,
	150588,
	150810,
	151032,
	151256,
	151479,
	151704,
	151929,
	152155,
	152381,
	152608,
	152836,
	153064,
	153293,
	153523,
	153754,
	153985,
	154217,
	154449,
	154683,
	154917,
	155152,
	155387,
	155623,
	155860,
	156098,
	156336,
	156575,
	156815,
	157055,
	157296,
	157538,
	157781,
	158025,
	158269,
	158514,
	158760,
	159006,
	159253,
	159502,
	159750,
	160000,
	160250,
	160502,
	160754,
	161006,
	161260,
	161514,
	161769,
	162025,
	162282,
	162540,
	162798,
	163057,
	163317,
	163578,
	163840,
	164103,
	164366,
	164630,
	164895,
	165161,
	165428,
	165696,
	165964,
	166234,
	166504,
	166775,
	167047,
	167320,
	167594,
	167869,
	168144,
	168421,
	168699,
	168977,
	169256,
	169536,
	169818,
	170100,
	170383,
	170667,
	170952,
	171237,
	171524,
	171812,
	172101,
	172391,
	172681,
	172973,
	173266,
	173559,
	173854,
	174150,
	174446,
	174744,
	175043,
	175342,
	175643,
	175945,
	176248,
	176552,
	176857,
	177163,
	177470,
	177778,
	178087,
	178397,
	178709,
	179021,
	179335,
	179649,
	179965,
	180282,
	180600,
	180919,
	181239,
	181560,
	181883,
	182206,
	182531,
	182857,
	183184,
	183513,
	183842,
	184173,
	184505,
	184838,
	185172,
	185507,
	185844,
	186182,
	186521,
	186861,
	187203,
	187546,
	187890,
	188235,
	188582,
	188930,
	189279,
	189630,
	189981,
	190335,
	190689,
	191045,
	191402,
	191760,
	192120,
	192481,
	192844,
	193208,
	193573,
	193939,
	194307,
	194677,
	195048,
	195420,
	195793,
	196169,
	196545,
	196923,
	197303,
	197683,
	198066,
	198450,
	198835,
	199222,
	199610,
	200000,
	200391,
	200784,
	201179,
	201575,
	201972,
	202372,
	202772,
	203175,
	203579,
	203984,
	204391,
	204800,
	205210,
	205622,
	206036,
	206452,
	206869,
	207287,
	207708,
	208130,
	213354,
	208980,
	209407,
	209836,
	210267,
	210700,
	211134,
	211570,
	212008,
	212448,
	212890,
	213333,
	213779,
	214226,
	214675,
	215126,
	215579,
	216034,
	216490,
	216949,
	217410,
	217872,
	218337,
	218803,
	219272,
	219742,
	220215,
	220690,
	221166,
	221645,
	222126,
	222609,
	223094,
	223581,
	224070,
	224561,
	225055,
	225551,
	226049,
	226549,
	227051,
	227556,
	228062,
	228571,
	229083,
	229596,
	230112,
	230631,
	231151,
	231674,
	232200,
	232727,
	233257,
	233790,
	234325,
	234862,
	235402,
	235945,
	236490,
	237037,
	237587,
	238140,
	238695,
	239252,
	239813,
	240376,
	240941,
	241509,
	242080,
	242654,
	243230,
	243810,
	244391,
	244976,
	245564,
	246154,
	246747,
	247343,
	247942,
	248544,
	249148,
	249756,
	250367,
	250980,
	251597,
	252217,
	252840,
	253465,
	254094,
	254726,
	255362,
	256000,
	256642,
	257286,
	257935,
	258586,
	259241,
	259898,
	260560,
	261224,
	261893,
	262564,
	263239,
	263918,
	264599,
	265285,
	265974,
	266667,
	267363,
	268063,
	268766,
	269474,
	270185,
	270899,
	271618,
	272340,
	273067,
	273797,
	274531,
	275269,
	276011,
	276757,
	277507,
	278261,
	279019,
	279781,
	280548,
	281319,
	282094,
	282873,
	283657,
	284444,
	285237,
	286034,
	286835,
	287640,
	288451,
	289266,
	290085,
	290909,
	291738,
	292571,
	293410,
	294253,
	295101,
	295954,
	296812,
	297674,
	298542,
	299415,
	300293,
	301176,
	302065,
	302959,
	303858,
	304762,
	305672,
	306587,
	307508,
	308434,
	309366,
	310303,
	311246,
	312195,
	313150,
	314110,
	315077,
	316049,
	317028,
	318012,
	319003,
	320000,
	321003,
	322013,
	323028,
	324051,
	325079,
	326115,
	327157,
	328205,
	329260,
	330323,
	331392,
	332468,
	333550,
	334641,
	335738,
	336842,
	337954,
	339073,
	340199,
	341333,
	342475,
	343624,
	344781,
	345946,
	347119,
	348299,
	349488,
	350685,
	351890,
	353103,
	354325,
	355556,
	356794,
	358042,
	359298,
	360563,
	361837,
	363121,
	364413,
	365714,
	367025,
	368345,
	369675,
	371014,
	372364,
	373723,
	375092,
	376471,
	377860,
	379259,
	380669,
	382090,
	383521,
	384962,
	386415,
	387879,
	389354,
	390840,
	392337,
	393846,
	395367,
	396899,
	398444,
	400000,
	401569,
	403150,
	404743,
	406349,
	407968,
	409600,
	411245,
	412903,
	414575,
	416260,
	417959,
	419672,
	421399,
	423140,
	424896,
	426667,
	428452,
	430252,
	432068,
	433898,
	435745,
	437607,
	439485,
	441379,
	443290,
	445217,
	447162,
	449123,
	451101,
	453097,
	455111,
	457143,
	459193,
	461261,
	463348,
	465455,
	467580,
	469725,
	471889,
	474074,
	476279,
	478505,
	480751,
	483019,
	485308,
	487619,
	489952,
	492308,
	494686,
	497087,
	499512,
	501961,
	504433,
	506931,
	509453,
	512000,
	514573,
	517172,
	519797,
	522449,
	525128,
	527835,
	530570,
	533333,
	536126,
	538947,
	541799,
	544681,
	547594,
	550538,
	553514,
	556522,
	559563,
	562637,
	565746,
	568889,
	572067,
	575281,
	578531,
	581818,
	585143,
	588506,
	591908,
	595349,
	598830,
	602353,
	605917,
	609524,
	613174,
	616867,
	620606,
	624390,
	628221,
	632099,
	636025,
	640000,
	644025,
	648101,
	652229,
	656410,
	660645,
	664935,
	669281,
	673684,
	678146,
	682667,
	687248,
	691892,
	696599,
	701370,
	706207,
	711111,
	716084,
	721127,
	726241,
	731429,
	736691,
	742029,
	747445,
	752941,
	758519,
	764179,
	769925,
	775758,
	781679,
	787692,
	793798,
	800000,
	806299,
	812698,
	819200,
	825806,
	832520,
	839344,
	846281,
	853333,
	860504,
	867797,
	875214,
	882759,
	890435,
	898246,
	906195,
	914286,
	922523,
	930909,
	939450,
	948148,
	957009,
	966038,
	975238,
	984615,
	994175,
	1003922,
	1013861,
	1024000,
	1034343,
	1044898,
	1055670,
	1066667,
	1077895,
	1089362,
	1101075,
	1113043,
	1125275,
	1137778,
	1150562,
	1163636,
	1177011,
	1190698,
	1204706,
	1219048,
	1233735,
	1248780,
	1264198,
	1280000,
	1296203,
	1312821,
	1329870,
	1347368,
	1365333,
	1383784,
	1402740,
	1422222,
	1442254,
	1462857,
	1484058,
	1505882,
	1528358,
	1551515,
	1575385,
	1600000,
};

static kal_int32 get_sensor_temperature(void)
{
	UINT8 temperature = 0;
	INT32 temperature_convert = 0;

	if (qsc_flag)
	temperature = read_cmos_sensor_8(0x013a);

	if (temperature >= 0x0 && temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (INT8)temperature | 0xFFFFFF0;

	temperature_convert = (temperature_convert > 100) ? 100: temperature_convert;

	sensor_temperature[0] = temperature_convert;

	LOG_INF("get_sensor_temperature temp_c(%d), read_reg(%d)\n", temperature_convert, temperature);


	return temperature_convert;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	uint32_t *pAeCtrls;
	uint32_t *pScenarios;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_RAWINFO_STRUCT *rawinfo;
	//struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	 struct SET_SENSOR_AWB_GAIN *pSetSensorAWB
	   = (struct SET_SENSOR_AWB_GAIN *)feature_para;


	struct SENSOR_VC_INFO2_STRUCT *pvcinfo2;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(imx766pd2282_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)imx766pd2282_ana_gain_table,
			sizeof(imx766pd2282_ana_gain_table));
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CUSTOM5:
			case MSDK_SCENARIO_ID_CUSTOM8:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = 1024;
				break;
			default:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = imgsensor_info.max_gain;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CUSTOM5:
			case MSDK_SCENARIO_ID_CUSTOM8:
				*(feature_data + 1) = 9;  /////  imgsensor_info.min_shutter
				*(feature_data + 2) = 1;
				break;
			default:
				*(feature_data + 1) = 8;  /////  imgsensor_info.min_shutter
				*(feature_data + 2) = 2;
				break;
		}
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 2867000;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom5.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom6.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom7.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom8.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom9.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom10.pclk;
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom11.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom4.framelength << 16)
				+ imgsensor_info.custom4.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom5.framelength << 16)
				+ imgsensor_info.custom5.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom6.framelength << 16)
				+ imgsensor_info.custom6.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom7.framelength << 16)
				+ imgsensor_info.custom7.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom8.framelength << 16)
				+ imgsensor_info.custom8.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom9.framelength << 16)
				+ imgsensor_info.custom9.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom10.framelength << 16)
				+ imgsensor_info.custom10.linelength;
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom11.framelength << 16)
				+ imgsensor_info.custom11.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case MSDK_SCENARIO_ID_CUSTOM5:
			*feature_return_para_32 = 1; /*BINNING_NONE*/
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM9:
		case MSDK_SCENARIO_ID_CUSTOM10:
		case MSDK_SCENARIO_ID_CUSTOM11:
		default:
			*feature_return_para_32 = 1; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;

		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		#if 0
		read_3P8_eeprom((kal_uint16)(*feature_data),
				(char *)(uintptr_t)(*(feature_data+1)),
				(kal_uint32)(*(feature_data+2)));
		#endif
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", (UINT32)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_RAW_INFO:
		pr_debug("SENSOR_FEATURE_GET_RAW_INFO scenarioId:%d\n",
			(UINT32) *feature_data);
		rawinfo = (struct SENSOR_RAWINFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		memcpy((void *)rawinfo,
				(void *)&imgsensor_raw_info,
				sizeof(struct SENSOR_RAWINFO_STRUCT));
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[1],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[8],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[9],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[10],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[11],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[12],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[13],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[14],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[15],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
//		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		//case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CUSTOM8:
		case MSDK_SCENARIO_ID_CUSTOM9:
		case MSDK_SCENARIO_ID_CUSTOM10:
			//#if 0
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			//#endif
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM11:
			//#if 0
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			//#endif
			break;


		default:
			break;
		}
		break;
	case SENSOR_FEATURE_GET_AWB_REQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		LOG_INF(
		"SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
			(UINT16) *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			/* video & capture use same setting */
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
			break;
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
 /*
	case SENSOR_FEATURE_SET_SEAMLESS_EXTEND_FRAME_LENGTH:
		pr_debug("extend_frame_len %d\n", *feature_data);
		extend_frame_length((MUINT32) *feature_data);
		pr_debug("extend_frame_len done %d\n", *feature_data);
		break;
 */
	case SENSOR_FEATURE_SEAMLESS_SWITCH:
	{
		LOG_INF("SENSOR_FEATURE_SEAMLESS_SWITCH");
		if ((feature_data + 1) != NULL)
			pAeCtrls = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else
			pr_debug("warning! no ae_ctrl input");

		if (feature_data == NULL) {
			pr_info("error! input scenario is null!");
			return ERROR_INVALID_SCENARIO_ID;
		}
		LOG_INF("call seamless_switch");
		if (pAeCtrls != NULL) {
			LOG_INF("call seamless_switch_arctrl shutter[%d, %d], gain[%d, %d]",
					*pAeCtrls, *(pAeCtrls + 1), *(pAeCtrls + 4), *(pAeCtrls + 5));
			seamless_switch((*feature_data),
					*pAeCtrls, *(pAeCtrls + 1),
					*(pAeCtrls + 4), *(pAeCtrls + 5));
		} else {
			LOG_INF("call seamless_switch_null");
			seamless_switch((*feature_data),
					0, 0, 0, 0);
		}
	}
		break;
	case SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS:
		if ((feature_data + 1) != NULL)
			pScenarios = (MUINT32 *)((uintptr_t)(*(feature_data + 1)));
		else {
			pr_info("input pScenarios vector is NULL!\n");
			return ERROR_INVALID_SCENARIO_ID;
		}

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*pScenarios = MSDK_SCENARIO_ID_CUSTOM8;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*pScenarios = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
			break;
		default:
			*pScenarios = 0xff;
			break;
		}

		pr_debug("SENSOR_FEATURE_GET_SEAMLESS_SCENARIOS %d %d\n", *feature_data, *pScenarios);
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		/*
			HDR_NONE = 0,
			HDR_RAW = 1,
			HDR_CAMSV = 2,
			HDR_RAW_ZHDR = 9,
			HDR_MultiCAMSV = 10,
			HDR_RAW_STAGGER_2EXP = 0xB,
			HDR_RAW_STAGGER_MIN = HDR_RAW_STAGGER_2EXP,
			HDR_RAW_STAGGER_3EXP = 0xC,
			HDR_RAW_STAGGER_MAX = HDR_RAW_STAGGER_3EXP,
		 */
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM9:
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0xB;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		LOG_INF(
			"SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n",
			*feature_data, *(MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
		/*END OF HDR CMD */
		case SENSOR_FEATURE_GET_VC_INFO2: {
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO2 %d\n",
							(UINT16) (*feature_data));
		pvcinfo2 = (struct SENSOR_VC_INFO2_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[1],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[2],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[3],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[4],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[5],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[6],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[7],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[8],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[9],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[10],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[11],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[12],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[13],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[14],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[15],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo2, (void *)&SENSOR_VC_INFO2[0],
				sizeof(struct SENSOR_VC_INFO2_STRUCT));
			break;
		}
	}
		break;
	case SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO:
		printk("zxw SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO %d",(UINT16) (*feature_data));
		if (*feature_data == MSDK_SCENARIO_ID_CAMERA_PREVIEW) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM9;
				break;
			case HDR_RAW_STAGGER_3EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM3;
				break;
			default:
				break;
			}
		} else if (*feature_data == MSDK_SCENARIO_ID_VIDEO_PREVIEW) {
			switch (*(feature_data + 1)) {
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM9;
				break;
			case HDR_RAW_STAGGER_3EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM3;
				break;
			default:
				break;
			}
		} else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM9) {
			switch (*(feature_data + 1)) {
			case HDR_NONE:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
				break;
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM9;
				break;
			default:
				break;
			}
		} else if (*feature_data == MSDK_SCENARIO_ID_CUSTOM3) {
			switch (*(feature_data + 1)) {
			case HDR_NONE:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
				break;
			case HDR_RAW_STAGGER_2EXP:
				*(feature_data + 2) = MSDK_SCENARIO_ID_CUSTOM9;
				break;
			default:
				break;
			}
		}
		LOG_INF("SENSOR_FEATURE_GET_STAGGER_TARGET_SCENARIO %d %d %d\n",
							(UINT16) (*feature_data),
				(UINT16) *(feature_data + 1),
				(UINT16) *(feature_data + 2));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 1; //always 1
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_GET_STAGGER_MAX_EXP_TIME:
		if (*feature_data == MSDK_SCENARIO_ID_CUSTOM1
			|| *feature_data == MSDK_SCENARIO_ID_CUSTOM2
			|| *feature_data == MSDK_SCENARIO_ID_CUSTOM3) {
			switch (*(feature_data + 1)) {
			case VC_STAGGER_NE:
					*(feature_data + 2) = 32757;
					break;
			case VC_STAGGER_ME:
					*(feature_data + 2) = 32757;
					break;
			case VC_STAGGER_SE:
					*(feature_data + 2) = 32757;
					break;
			default:
					*(feature_data + 2) = 32757;
					break;
			}
		} else {
			*(feature_data + 2) = 0;
		}
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER: //for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
				(UINT16) *feature_data, (UINT16) *(feature_data + 1));
		// implement write shutter for NE/SE
		hdr_write_tri_shutter((UINT16)*feature_data,
					0,
					(UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_DUAL_GAIN: //for 2EXP
		LOG_INF("SENSOR_FEATURE_SET_DUAL_GAIN LE=%d, SE=%d\n",
				(UINT16) *feature_data, (UINT16) *(feature_data + 1));
		// implement write gain for NE/SE
		hdr_write_tri_gain((UINT16)*feature_data,
				   0,
				   (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER://for 3EXP
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, ME=%d, SE=%d\n",
			(UINT16) *feature_data,(UINT16) *(feature_data + 1),(UINT16) *(feature_data + 2));
		// implement write shutter for NE/ME/SE
		hdr_write_tri_shutter((UINT16)*feature_data,
					(UINT16)*(feature_data+1),
					(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
			(UINT16) *feature_data, (UINT16) *(feature_data + 1), (UINT16) *(feature_data + 2));
		// implement write gain for NE/ME/SE
		hdr_write_tri_gain((UINT16)*feature_data,
				   (UINT16)*(feature_data+1),
				   (UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PDAF_REG_SETTING:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_REG_SETTING %d",
			(*feature_para_len));
		imx766pd2282_get_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF_REG_SETTING:
		/*LOG_INF("SENSOR_FEATURE_SET_PDAF_REG_SETTING %d",
		 *	(*feature_para_len));
		 */
		imx766pd2282_set_pdaf_reg_setting((*feature_para_len) / sizeof(UINT32)
					   , feature_data_16);
		break;
	case SENSOR_FEATURE_SET_PDAF:
		LOG_INF("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(
			(UINT32) (*feature_data),
			(UINT16) (*(feature_data + 1)),
			(BOOL) (*(feature_data + 2)));
		break;
	#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN_IMX766_OTP_ERROR_CODE:%d \n", *feature_data, IMX766PD2282_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			LOG_INF("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = IMX766PD2282_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_main_imx766pd2282, sizeof(MUINT32)*13);
				memcpy( feature_data+10, material_inf_main_imx766pd2282, sizeof(MUINT32)*4);
				memcpy( feature_data+12, af_calib_inf_main_imx766pd2282, sizeof(MUINT32)*6);
				memcpy( feature_data+15, moduleid_inf_main_imx766pd2282, sizeof(MUINT32)*2);
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_main_imx766pd2282[%d]= 0x%x\n", i, sn_inf_main_imx766pd2282[i]);
						}
				#endif
			}
				break;
			}
			break;
	#endif
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx766pd2282_awb_gain(pSetSensorAWB);
		break;
#if 0
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		#if 0
		ihdr_write_shutter((UINT16)*feature_data,
				   (UINT16)*(feature_data+1));
		#endif
		break;
#endif
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = imgsensor.current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom5.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom6.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom7.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom8.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM9:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom9.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM10:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom10.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM11:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom11.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	}
	/*chenhan add*/
	case SENSOR_FEATURE_OIS_FW_UPDATE:
	{
		ois_fw_check(feature_para);
		break;
	}
	/*add end*/
	/*chenwenjie add*/
	#if 0
	case SENSOR_FEATURE_SET_SAT_MODE:
	{
		LOG_INF("SENSOR_FEATURE_SET_SAT_MODE %d\n",
							(UINT16) (*feature_data));
		if (qsc_flag)
			ois_driver_create((UINT16) (*feature_data));

		break;
	}
	#endif
	/*chenwenjie add end*/
	default:
		break;
	}

	return ERROR_NONE;
} /* feature_control() */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX766PD2282_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
} /* IMX766PD2282_MIPI_RAW_SensorInit */
