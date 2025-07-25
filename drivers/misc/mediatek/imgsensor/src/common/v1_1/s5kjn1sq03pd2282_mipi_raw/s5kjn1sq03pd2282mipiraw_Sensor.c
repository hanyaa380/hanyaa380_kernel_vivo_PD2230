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
 *	 s5kjn1sq03mipiraw_Sensor.c
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

/*****************************************************************************
*long exposure write shutter update for shutter >= 20s  by samsung Carl
 ****************************************************************************/

#define PFX "MAIN[38E1]_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


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
#include "s5kjn1sq03pd2282mipiraw_Sensor.h"
#include "../imgsensor_common.h"

//#include "s5kjn1sq03mipiraw_freq.h"
#include "../imgsensor_sensor.h"
/*chenhan add for ois*/
//#include "ois_core.h"
/*add end*/

/*
 * #define PK_DBG(format, args...) pr_debug(
 * PFX "[%s] " format, __func__, ##args)
 */
static DEFINE_SPINLOCK(imgsensor_drv_lock);
static bool bIsLongExposure = KAL_FALSE;

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KJN1SQ03PD2282_SENSOR_ID,
	.checksum_value = 0x8ac2d94a,

	.pre = {
		.pclk =563000000,
		.linelength = 5392,
		.framelength = 3472,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3060,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 535600000, //1.339G
		.max_framerate = 300,//30.06
	},
	.cap = {
		.pclk =563000000,
		.linelength = 5392,
		.framelength = 3472,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3060,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 535600000, //1.339G
		.max_framerate = 300,//30.06
	},
	.normal_video = {
		.pclk =563000000,
		.linelength = 5910,
		.framelength = 3168,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 499200000,
		.max_framerate = 300,//30.09
	},
	.hs_video = { /*hs_video 120fps*/
		.pclk =598000000,
        .linelength = 2116,
        .framelength = 2352,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 644800000, //1848
		.max_framerate = 1200,//120.16
		},
	.slim_video = { /* hs_video 240fps*/
		.pclk =598000000,
		.linelength = 2064,
		.framelength = 1208,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 644800000, //1612.0
		.max_framerate = 2400,//240.16
		},
	.custom1 = {
		.pclk =563000000,
		.linelength = 5392,
		.framelength = 3472,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3060,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 535600000, //1.339G
		.max_framerate = 300,//30.06
	},
	.custom2 = { /* 3840*2160@60fps */
		.pclk =563000000,
		.linelength = 4096,
		.framelength = 2292,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 702000000,/*511200000*/
		.max_framerate = 600,//60.01
		},
	.custom3 = { /* 3840*2160@30fps */
		.pclk =563000000,
		.linelength = 4096,
		.framelength = 4584,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 702000000,/*511200000*/
		.max_framerate = 300,//60.01
		},
	.custom4 = { /* 2032*1536@15fps */
		.pclk =598000000,
		.linelength = 4800,
		.framelength = 4152,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2032,
		.grabwindow_height = 1536,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 322400000,/*511200000*/
		.max_framerate = 300,//30.01
	},
    .custom5 = { /*sw remosiac */
		.pclk =563000000,
		.linelength = 8688,
		.framelength = 6432,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 556400000,
		.max_framerate = 100,//10.08
	},
	.custom6 = { // 3264*2448
		.pclk =563000000,
		.linelength = 4584,
		.framelength = 4088,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 660400000,
		.max_framerate = 300,//30.06
	},
	.custom7 = { // 3264*1836
		.pclk =563000000,
		.linelength = 4584,
		.framelength = 4088,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 660400000,
		.max_framerate = 300,//30.06
	},
	.custom8 = { // 2032*1136
		.pclk =598000000,
		.linelength = 4800,
		.framelength = 4152,
		.startx =0,
		.starty = 0,
		.grabwindow_width = 2032,
		.grabwindow_height = 1136,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 322400000,
		.max_framerate = 300,//30.01
	},
		.margin = 10,		/* sensor framelength & shutter margin */
		.min_shutter = 6,	/* min shutter */
		.min_gain = 64,
		.max_gain = 4096,//mode  gain different (64x64)
		.min_gain_iso = 50,
		.gain_step = 2,
		.gain_type = 2,
		.max_frame_length = 0xffff,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.frame_time_delay_frame = 2,	/*sony sensor must be 3,non-sony sensor must be 2 , The delay frame of setting frame length  */
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/
		.sensor_mode_num = 13,	  /*support sensor mode num*/

		.cap_delay_frame = 3,	/* enter capture delay frame num */
		.pre_delay_frame = 3,	/* enter preview delay frame num */
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
		.custom1_delay_frame = 3,
		.custom2_delay_frame = 3,
		.custom3_delay_frame = 3,
		.custom4_delay_frame = 3,
		.custom5_delay_frame = 3,
		.custom6_delay_frame = 3,
		.custom7_delay_frame = 3,
		.custom8_delay_frame = 3,
		.isp_driving_current = ISP_DRIVING_2MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
		//.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_Gr,
		.mclk = 26,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 1000,
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, /*IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video*/
	.shutter = 0x3D0,					/*current shutter*/
	.gain = 0x100,						/*current gain*/
	.dummy_pixel = 0,					/*current dummypixel*/
	.dummy_line = 0,					/*current dummyline*/
	.current_fps = 0,  /*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.autoflicker_en = KAL_FALSE,  /*auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker*/
	.test_pattern = KAL_FALSE,		/*test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x20,
	.current_ae_effective_frame = 1,
	.freq_setting = 0,
  	.present_freq_setting = 0,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[13] = {
	{8160, 6144,	0,    12, 8160, 6120, 4080, 3060, 0, 0, 4080, 3060, 0, 0, 4080, 3060}, /*Preview*/
	{8160, 6144,    0,  12, 8160, 6120, 4080, 3060, 0, 0, 4080, 3060, 0, 0, 4080, 3060}, /* capture */
	{8160, 6144,    0,  776, 8160, 4592, 4080, 2296, 0, 0, 4080, 2296, 0, 0, 4080, 2296}, /* normal video */
	{8160, 6144,  240,  912, 7680, 4320, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080}, /* hs_video */
	{8160, 6144,  240,  912, 7680, 4320, 1920, 1080, 0, 0, 1920, 1080, 0, 0, 1920, 1080}, /* slim video */
	{8160, 6144,    0,    12, 8160, 6120, 4080, 3060, 0, 0, 4080, 3060, 0, 0, 4080, 3060}, /* custom1 */
	{8160, 6144,  240,  912, 7680, 4320, 3840, 2160, 0, 0, 3840, 2160, 0, 0, 3840, 2160}, /* custom2 */
    {8160, 6144,  240,  912, 7680, 4320, 3840, 2160, 0, 0, 3840, 2160, 0, 0, 3840, 2160}, /* custom3 */
	{8160, 6144,    16,   0, 8128, 6144, 2032, 1536, 0, 0, 2032, 1536, 0, 0, 2032, 1536}, /*custom4*/
	{8160, 6144,    0,    0, 8160, 6144, 8160, 6144, 0, 0, 8160, 6144, 0, 0, 8160, 6144}, /* sw remosiac*/
	{8160, 6144,  816,  624, 6528, 4896, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, /* custom6 */
	{8160, 6144,  816, 1236, 6528, 3672, 3264, 1836, 0, 0, 3264, 1836, 0, 0, 3264, 1836}, /* custom7 */
	{8160, 6144,   16,  800, 8128, 4544, 2032, 1136, 0, 0, 2032, 1136, 0, 0, 2032, 1136},  /* custom8 */
};

static struct  SENSOR_RAWINFO_STRUCT imgsensor_raw_info = {
	 4080,//raw_weight
	 3060,//raw_height
	 2,//raw_dataBit
	 BAYER_GRBG,//raw_colorFilterValue
	 64,//raw_blackLevel
	 92,//raw_viewAngle
	 10,//raw_bitWidth
	 64//raw_maxSensorGain
};

#if 1
/*VC1 for HDR(DT=0X35), VC2 for PDAF(DT=0X30), unit : 10bit */
static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[7] =
{/* Preview mode setting 4080*3060*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0FF0, 0x0BF4, 0x00, 0x00, 0x00, 0x00,
	 0x01, 0x30, 0x027C, 0x0BF0, 0x00, 0x00, 0x0000, 0x0000},//VC DT bufW bufH
	/* Normal_Video mode setting 4080*2296*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1200, 0x0BB0, 0x00, 0x00, 0x00, 0x00,
	 0x01, 0x30, 0x027C, 0x08F0, 0x00, 0x00, 0x0000, 0x0000},
	/* custom2 mode setting 3840*2160 60fps*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1200, 0x0A20, 0x00, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x0258, 0x0860, 0x00, 0x00, 0x0000, 0x0000},
	 /* custom6 mode setting 3840*2160 60fps*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0CC0, 0x0990, 0x00, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x01FE, 0x0980, 0x00, 0x00, 0x0000, 0x0000},
	 /* custom7 mode setting 3840*2160 60fps*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0CC0, 0x072C, 0x00, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x01FE, 0x0720, 0x00, 0x00, 0x0000, 0x0000},
	 /* custom8 mode setting 2032*1560 60fps*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x07F0, 0x0600, 0x00, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x027C, 0x02FC, 0x00, 0x00, 0x0000, 0x0000},
	 /* custom8 mode setting 2032*1136 30fps*/
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x07F0, 0x0470, 0x00, 0x00, 0x0000, 0x0000,
	 0x01, 0x30, 0x027C, 0x0234, 0x00, 0x00, 0x0000, 0x0000},
};

/* If mirror flip   4080x3060 ini   0d06 0d08*/
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info =  //4080X3060
{
	.i4OffsetX =  8,
	.i4OffsetY =  2,
	.i4PitchX  =  8,
	.i4PitchY  =  8,
	.i4PairNum  = 4,
	.i4SubBlkW  = 8,
	.i4SubBlkH  = 2,
	.i4PosL = {{9, 2},{11, 5},{15, 6},{13, 9},},
	.i4PosR = {{8, 2},{10, 5},{14, 6},{12, 9},},
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
	.iMirrorFlip = 0,
	.i4Crop = { {0,0}, {0,0}, {0, 382}, {0, 0}, {0, 0}, {0, 0}, {120, 450}, {0, 0}, {0, 0}, {120, 450}, {408,306}, {408,612}}, //4080, 2296
};
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_binning = // 2032X1536
{
    .i4OffsetX = 0,
    .i4OffsetY = 4,
    .i4PitchX = 8,
    .i4PitchY = 4,
    .i4PairNum = 2,
    .i4SubBlkW = 4,
    .i4SubBlkH = 4,
    .i4BlockNumX = 254,
    .i4BlockNumY = 382,
    .i4PosL = {{1, 4},{5, 4},},
    .i4PosR = {{0, 4},{4, 4},},
	.i4Crop = { {0,0}, {0,0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},
				{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 200}
			},
	.iMirrorFlip = 0,
};
#endif
/*hope add otp check start*/
static int vivo_otp_read_when_power_on;
extern int MAIN_38E1_otp_read(void);
extern otp_error_code_t S5KJN1SQ03_OTP_ERROR_CODE;
MUINT32  moduleid_inf_main_s5kjn1sq03[2];  /*0 flag   1-12 data*/
MUINT32  sn_inf_main_s5kjn1sq03[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main_s5kjn1sq03[4];
MUINT32  af_calib_inf_main_s5kjn1sq03pd2282[6];
MUINT32  crosstalk_data_PD2282[8896];  /*0 flag   1-836 data*/
/*hope add otp check end*/
#define HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
//#define READ_FRAMELENGTH_SHUTTER_GAIN
#ifdef READ_FRAMELENGTH_SHUTTER_GAIN
static int s5kjn1_read_framelength;
static int s5kjn1_read_shutter;
static int s5kjn1_read_gain;
#endif

#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
extern u32 sensor_temperature[10];
#endif

static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_16_8(kal_uint16 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);  Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_16_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	 /* kdSetI2CSpeed(imgsensor_info.i2c_speed);Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 1020	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 4

#endif

static kal_uint16 table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
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
			puSendCmd[tosend++] = (char)(data >> 8);
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 4 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 4 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd, tosend, imgsensor.i2c_write_id,
								4, imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2CTiming(puSendCmd, 4, imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
		tosend = 0;
#endif

	}
	return 0;
}



static void set_dummy(void)
{
	PK_DBG("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	 write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	 write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	PK_DBG("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	PK_DBG("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_16(0x6028, 0x4000);
		//write_cmos_sensor_16_16(0x6214, 0xF9F0);
		//write_cmos_sensor_16_16(0x6218, 0xF9F0);
		write_cmos_sensor_16_8(0x0100, 0X01);

		mdelay(10);
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
                    PK_DBG(" Stream Off  at i=%d framecnt =%d ",i,framecnt);
			if ( framecnt == 0xFF) {
				PK_DBG(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		PK_DBG("Stream Off Fail! framecnt= %d.\n", framecnt);
	}
    	PK_DBG("streaming_enable--(0= Sw Standby,1= streaming): %d\n", enable);
	return ERROR_NONE;
}


static void write_shutter(kal_uint32 shutter)// shutter gain n+2  framelength n+1
{

	kal_uint16 realtime_fps = 0;
	int framecnt = 0;

	PK_DBG("shutter =%d\n", shutter);
	if(shutter > 65530) {  //linetime=10160/960000000<< maxshutter=3023622-line=32s
		/*enter long exposure mode */
		kal_uint32 exposure_time;
		kal_uint16 new_framelength;
		kal_uint16 long_shutter=0;
		PK_DBG("enter long exposure mode\n");
		PK_DBG("Calc long exposure  +\n");

		bIsLongExposure = KAL_TRUE;

		PK_DBG("enter long exposure mode shutter = %d, imgsensor.line_length =%d\n", shutter, imgsensor.line_length);

		//exposure_time =(8.65*shutter)/1000000;
		//exposure_time = shutter/(2^6);
		/*exposure_time = shutter*1000*imgsensor.line_length/imgsensor.pclk;ms*/
		//long_shutter = (exposure_time*pclk-256)/lineleght/64 = (shutter*linelength-256)/linelength/pow_shift
		exposure_time = shutter * imgsensor.line_length / imgsensor.pclk *1000 ;
    long_shutter = shutter/64;
		new_framelength = long_shutter+10;

		PK_DBG("Calc long exposure  -\n");

		/*streaming_control(KAL_FALSE);*/
		write_cmos_sensor_16_16(0x6028, 0x4000);
		write_cmos_sensor_16_8(0x0100, 0x00); /*stream off*/
		while (1) {
			//mdelay(2);
			//write_cmos_sensor_16_8(0x0100, 0x00); /*stream off*/
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			PK_DBG(" Stream Off oning at framecnt=0x%x.\n", framecnt);
			if ( framecnt == 0xFF) {
				PK_DBG(" Stream Off OK at framecnt=0x%x.\n", framecnt);
				break;
			}
		}

		write_cmos_sensor_16_16(0x0340, new_framelength);
		write_cmos_sensor_16_16(0x0202, long_shutter);
		write_cmos_sensor_16_16(0x0702, 0x0600);
		write_cmos_sensor_16_16(0x0704, 0x0600);
		write_cmos_sensor_16_8(0x0100, 0x01);  /*stream on*/
		/* Frame exposure mode customization for LE*/
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 1;

		PK_DBG("long exposure  stream on-\n");
	} else {
		imgsensor.current_ae_effective_frame = 1;
		if (bIsLongExposure) {
		PK_DBG("enter normal shutter.\n");
		write_cmos_sensor_16_16(0x6028, 0x4000);
		write_cmos_sensor_16_8(0x0100, 0x00); /*stream off*/
		while (1) {
			//mdelay(2);
			//write_cmos_sensor_16_8(0x0100, 0x00); /*stream off*/
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			PK_DBG(" Stream Off oning at framecnt=0x%x.\n", framecnt);
			if ( framecnt == 0xFF) {
				PK_DBG(" Stream Off OK at framecnt=0x%x.\n", framecnt);
				break;
			}
		}
		write_cmos_sensor_16_16(0x6028, 0x4000);
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
		write_cmos_sensor_16_16(0x0202, 0x0100);
		write_cmos_sensor_16_16(0x0702, 0x0000);
		write_cmos_sensor_16_16(0x0704, 0x0000);
		write_cmos_sensor_16_8(0x0100, 0x01);  /*stream on*/
		bIsLongExposure = KAL_FALSE;
		PK_DBG("===hope enter normal shutter shutter = %d, imgsensor.frame_lengths = 0x%x, imgsensor.line_length = 0x%x\n", shutter,imgsensor.frame_length, imgsensor.line_length);
		}
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (shutter < imgsensor_info.min_shutter) shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);
		else {
		/* Extend frame length*/
	        write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);

	    }
	} else {
		/* Extend frame length*/
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	}

	/* Update Shutter*/
	    write_cmos_sensor_16_16(0x0202, shutter);


	PK_DBG("shutter = %d, framelength = %d 0x0005=0x%x\n", shutter,imgsensor.frame_length,read_cmos_sensor_16_8(0x0005));
	}

#ifdef READ_FRAMELENGTH_SHUTTER_GAIN
	s5kjn1_read_framelength = read_cmos_sensor_16_16(0x0340);
	s5kjn1_read_shutter = read_cmos_sensor_16_16(0x0202);
	PK_DBG(" s5kjn1_read_shutter = %d, s5kjn1_read_framelength = %d \n", s5kjn1_read_shutter,s5kjn1_read_framelength);
#endif
}	/*	write_shutter  */



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
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

/*	write_shutter  */
static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	/*  */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;

	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_16_16(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	/* Update Shutter */
	write_cmos_sensor_16_16(0X0202, shutter & 0xFFFF);

	PK_DBG("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
		frame_length, dummy_line);


}				/*      write_shutter  */


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	 kal_uint16 reg_gain = 0x0;

	reg_gain = gain/2;
	return (kal_uint16)reg_gain;
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

	/*gain= 1024;for test*/
	/*return; for test*/

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_debug("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}


	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	PK_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_8(0x0204, (reg_gain >> 8));
	write_cmos_sensor_16_8(0x0205, (reg_gain & 0xff));

#ifdef READ_FRAMELENGTH_SHUTTER_GAIN
	s5kjn1_read_gain = read_cmos_sensor_16_16(0x0204);
	PK_DBG(" s5kjn1_read_gain = %d \n", s5kjn1_read_gain);
#endif

	return gain;
}	/*	set_gain  */

static kal_uint32 gw3sp_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
#if 0
	PK_DBG("gw3sp_awb_gain: 0x100\n");
	write_cmos_sensor_16_16(0x0D82, 0x100);
	write_cmos_sensor_16_16(0x0D84, 0x100);
	write_cmos_sensor_16_16(0x0D86, 0x100);

#endif
	return ERROR_NONE;
}

/*write AWB gain to sensor*/
static void feedback_awbgain(kal_uint32 r_gain, kal_uint32 b_gain)
{
#if 0
	UINT32 r_gain_int = 0;
	UINT32 b_gain_int = 0;

	r_gain_int = r_gain / 2;
	b_gain_int = b_gain / 2;
	/*write r_gain*/
	write_cmos_sensor_16_16(0x0D82, r_gain_int);
	/*write _gain*/
	write_cmos_sensor_16_16(0x0D86, b_gain_int);
#endif
}

static void gw3sp_set_lsc_reg_setting(
		kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{
#if 0
	int i;
	int startAddr[4] = {0x9D88, 0x9CB0, 0x9BD8, 0x9B00};
	/*0:B,1:Gb,2:Gr,3:R*/
	PK_DBG("E! index:%d, regNum:%d\n", index, regNum);

	write_cmos_sensor_16_8(0x0B00, 0x01); /*lsc enable*/
	write_cmos_sensor_16_8(0x9014, 0x01);
	write_cmos_sensor_16_8(0x4439, 0x01);
	mdelay(1);
	PK_DBG("Addr 0xB870, 0x380D Value:0x%x %x\n",
		read_cmos_sensor_16_8(0xB870), read_cmos_sensor_16_8(0x380D));
	/*define Knot point, 2'b01:u3.7*/
	write_cmos_sensor_16_8(0x9750, 0x01);
	write_cmos_sensor_16_8(0x9751, 0x01);
	write_cmos_sensor_16_8(0x9752, 0x01);
	write_cmos_sensor_16_8(0x9753, 0x01);

	for (i = 0; i < regNum; i++)
		write_cmos_sensor_16_16(startAddr[index] + 2*i, regDa[i]);

	write_cmos_sensor_16_8(0x0B00, 0x00); /*lsc disable*/
#endif
}




static void set_mirror_flip(kal_uint8 image_mirror)
{
	switch (image_mirror) {

	    case IMAGE_NORMAL:

	        write_cmos_sensor_16_8(0x0101,0x00);   /* Gr*/
	        break;

	    case IMAGE_H_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x01);
	        break;

	    case IMAGE_V_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x02);
	        break;

	    case IMAGE_HV_MIRROR:

	        write_cmos_sensor_16_8(0x0101,0x03);/*Gb*/
	        break;
	    default:
	    PK_DBG("Error image_mirror setting\n");
	}
}

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
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

static kal_uint16 addr_data_pair_init[] = {
	0x6028, 0x2400,
	0x602A, 0x1354,
	0x6F12, 0x0100,
	0x6F12, 0x7017,
	0x602A, 0x13B2,
	0x6F12, 0x0000,
	0x602A, 0x1236,
	0x6F12, 0x0000,
	0x602A, 0x1A0A,
	0x6F12, 0x4C0A,
	0x602A, 0x2210,
	0x6F12, 0x3401,
	0x602A, 0x2176,
	0x6F12, 0x6400,
	0x602A, 0x222E,
	0x6F12, 0x0001,
	0x602A, 0x06B6,
	0x6F12, 0x0A00,
	0x602A, 0x06BC,
	0x6F12, 0x1001,
	0x602A, 0x2140,
	0x6F12, 0x0101,
	0x602A, 0x1A0E,
	0x6F12, 0x9600,
	0x6028, 0x4000,
	0xF44E, 0x0011,
	0xF44C, 0x0B0B,
	0xF44A, 0x0006,
	0x0118, 0x0002,
	0x011A, 0x0001,
};
static kal_uint16 addr_data_pair_preview[] = { //4080x3060
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x0000,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x0000,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0000,
	0x0346, 0x000C,
	0x0348, 0x1FFF,
	0x034A, 0x1813,
	0x034C, 0x0FF0,
	0x034E, 0x0BF4,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0067,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0D90,
	0x0342, 0x1510,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x000E,
};
static kal_uint16 addr_data_pair_capture[] = {  //4080x3060
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x0000,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x0000,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0000,
	0x0346, 0x000C,
	0x0348, 0x1FFF,
	0x034A, 0x1813,
	0x034C, 0x0FF0,
	0x034E, 0x0BF4,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0067,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0D90,
	0x0342, 0x1510,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x000E,
};

static kal_uint16 addr_data_pair_normal_video[] = {   //4080x2296
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x1201,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x1F04,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0000,
	0x0346, 0x0308,
	0x0348, 0x1FFF,
	0x034A, 0x1517,
	0x034C, 0x0FF0,
	0x034E, 0x08F8,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x00C0,
	0x0312, 0x0001,
	0x080E, 0x0000,
	0x0340, 0x0C60,
	0x0342, 0x1716,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};
static kal_uint16 addr_data_pair_hs_video[] = {  //1920x1080 120fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0300,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0020,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0x3F00,
	0x602A, 0x19E6,
	0x6F12, 0x0201,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA020,
	0x602A, 0x1A3C,
	0x6F12, 0x5207,
	0x602A, 0x1A48,
	0x6F12, 0x5207,
	0x602A, 0x1444,
	0x6F12, 0x2100,
	0x6F12, 0x2100,
	0x602A, 0x144C,
	0x6F12, 0x4200,
	0x6F12, 0x4200,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x3100,
	0x6F12, 0xF700,
	0x6F12, 0x2600,
	0x6F12, 0xE100,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8600,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x0800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x1100,
	0x602A, 0x09C0,
	0x6F12, 0x9800,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x4001,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x9400,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x4F01,
	0x602A, 0x2080,
	0x6F12, 0x0100,
	0x6F12, 0x7F00,
	0x6F12, 0x0002,
	0x6F12, 0x8000,
	0x6F12, 0x0002,
	0x6F12, 0xC244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x141C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x20BA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0A00,
	0x602A, 0x13AE,
	0x6F12, 0x0102,
	0x602A, 0x0718,
	0x6F12, 0x0005,
	0x602A, 0x0710,
	0x6F12, 0x0004,
	0x6F12, 0x0401,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0300,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0101,
	0x6F12, 0x0101,
	0x602A, 0x1360,
	0x6F12, 0x0000,
	0x602A, 0x1376,
	0x6F12, 0x0200,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0000,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0000,
	0x602A, 0x11F6,
	0x6F12, 0x0010,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x00F0,
	0x0346, 0x0390,
	0x0348, 0x1F0F,
	0x034A, 0x148F,
	0x034C, 0x0780,
	0x034E, 0x0438,
	0x0350, 0x0004,
	0x0352, 0x0004,
	0x0900, 0x0144,
	0x0380, 0x0002,
	0x0382, 0x0006,
	0x0384, 0x0002,
	0x0386, 0x0006,
	0x0110, 0x1002,
	0x0114, 0x0300,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x008A,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005D,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0930,
	0x0342, 0x0844,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0001,
	0x0D04, 0x0002,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_slim_video[] = {  //1920x1080 240fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0300,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0020,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0x3F00,
	0x602A, 0x19E6,
	0x6F12, 0x0201,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA020,
	0x602A, 0x1A3C,
	0x6F12, 0x5207,
	0x602A, 0x1A48,
	0x6F12, 0x5207,
	0x602A, 0x1444,
	0x6F12, 0x2100,
	0x6F12, 0x2100,
	0x602A, 0x144C,
	0x6F12, 0x4200,
	0x6F12, 0x4200,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x3100,
	0x6F12, 0xF700,
	0x6F12, 0x2600,
	0x6F12, 0xE100,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8600,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x0800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x1100,
	0x602A, 0x09C0,
	0x6F12, 0x9800,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x4001,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x9400,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x4F01,
	0x602A, 0x2080,
	0x6F12, 0x0100,
	0x6F12, 0x7F00,
	0x6F12, 0x0002,
	0x6F12, 0x8000,
	0x6F12, 0x0002,
	0x6F12, 0xC244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x141C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x20BA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0A00,
	0x602A, 0x13AE,
	0x6F12, 0x0102,
	0x602A, 0x0718,
	0x6F12, 0x0005,
	0x602A, 0x0710,
	0x6F12, 0x0004,
	0x6F12, 0x0401,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0300,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0101,
	0x6F12, 0x0101,
	0x602A, 0x1360,
	0x6F12, 0x0000,
	0x602A, 0x1376,
	0x6F12, 0x0200,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0000,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0000,
	0x602A, 0x11F6,
	0x6F12, 0x0010,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x00F0,
	0x0346, 0x0390,
	0x0348, 0x1F0F,
	0x034A, 0x148F,
	0x034C, 0x0780,
	0x034E, 0x0438,
	0x0350, 0x0004,
	0x0352, 0x0004,
	0x0900, 0x0144,
	0x0380, 0x0002,
	0x0382, 0x0006,
	0x0384, 0x0002,
	0x0386, 0x0006,
	0x0110, 0x1002,
	0x0114, 0x0300,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x008A,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005D,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x04B8,
	0x0342, 0x0810,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0001,
	0x0D04, 0x0002,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_custom1[] = {  //4080x3060
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x0000,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x0000,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0000,
	0x0346, 0x000C,
	0x0348, 0x1FFF,
	0x034A, 0x1813,
	0x034C, 0x0FF0,
	0x034E, 0x0BF4,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0067,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0D90,
	0x0342, 0x1510,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x000E,
};
static kal_uint16 addr_data_pair_custom2[] = {  //3840x210 60fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x6100,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0xEF03,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x00F0,
	0x0346, 0x0390,
	0x0348, 0x1F0F,
	0x034A, 0x148F,
	0x034C, 0x0F00,
	0x034E, 0x0870,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0087,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x08F4, ///////FLL 30fps-0x11E8  60fps-0x08F4
	0x0342, 0x1000,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_custom3[] = {   //3840x210 30fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x6100,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0xEF03,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x00F0,
	0x0346, 0x0390,
	0x0348, 0x1F0F,
	0x034A, 0x148F,
	0x034C, 0x0F00,
	0x034E, 0x0870,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0087,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x11E8,  ///////FLL 30fps-0x11E8  60fps-0x08F4
	0x0342, 0x1000,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_custom4[] = {  //2032x1536 30fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0300,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0020,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0x3F00,
	0x602A, 0x19E6,
	0x6F12, 0x0201,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA020,
	0x602A, 0x1A3C,
	0x6F12, 0x5207,
	0x602A, 0x1A48,
	0x6F12, 0x5207,
	0x602A, 0x1444,
	0x6F12, 0x2100,
	0x6F12, 0x2100,
	0x602A, 0x144C,
	0x6F12, 0x4200,
	0x6F12, 0x4200,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x3100,
	0x6F12, 0xF700,
	0x6F12, 0x2600,
	0x6F12, 0xE100,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8600,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x0800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x1100,
	0x602A, 0x09C0,
	0x6F12, 0x9800,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x4001,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x9400,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x4F01,
	0x602A, 0x2080,
	0x6F12, 0x0100,
	0x6F12, 0x7F00,
	0x6F12, 0x0002,
	0x6F12, 0x8000,
	0x6F12, 0x0002,
	0x6F12, 0xC244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x141C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x20BA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0A00,
	0x602A, 0x13AE,
	0x6F12, 0x0102,
	0x602A, 0x0718,
	0x6F12, 0x0005,
	0x602A, 0x0710,
	0x6F12, 0x0004,
	0x6F12, 0x0401,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0300,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0101,
	0x6F12, 0x0101,
	0x602A, 0x1360,
	0x6F12, 0x0000,
	0x602A, 0x1376,
	0x6F12, 0x0200,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0000,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0000,
	0x602A, 0x11F6,
	0x6F12, 0x0010,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0010,
	0x0346, 0x0000,
	0x0348, 0x1FEF,
	0x034A, 0x181F,
	0x034C, 0x07F0,
	0x034E, 0x0600,
	0x0350, 0x0004,
	0x0352, 0x0004,
	0x0900, 0x0144,
	0x0380, 0x0002,
	0x0382, 0x0006,
	0x0384, 0x0002,
	0x0386, 0x0006,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x008A,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005D,
	0x0312, 0x0001,
	0x080E, 0x0000,
	0x0340, 0x1038,
	0x0342, 0x12C0,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_custom5[] = {  //remosiac  8160x6144 10fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0400,
	0x602A, 0x139C,
	0x6F12, 0x0100,
	0x602A, 0x13A0,
	0x6F12, 0x0500,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0101,
	0x602A, 0x1A64,
	0x6F12, 0x0001,
	0x6F12, 0x0000,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3403,
	0x602A, 0x19FC,
	0x6F12, 0x0700,
	0x602A, 0x19F4,
	0x6F12, 0x0707,
	0x602A, 0x19F8,
	0x6F12, 0x0B0B,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x8207,
	0x602A, 0x1A48,
	0x6F12, 0x8207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8500,
	0x602A, 0x1A52,
	0x6F12, 0x9800,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x0400,
	0x602A, 0x1480,
	0x6F12, 0x0400,
	0x602A, 0x19F6,
	0x6F12, 0x0404,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0106,
	0x602A, 0x09C0,
	0x6F12, 0x4000,
	0x602A, 0x09C4,
	0x6F12, 0x4000,
	0x602A, 0x19FE,
	0x6F12, 0x0C1C,
	0x602A, 0x4D92,
	0x6F12, 0x0000,
	0x602A, 0x84C8,
	0x6F12, 0x0000,
	0x602A, 0x4D94,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x7306,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x6902,
	0x602A, 0x2080,
	0x6F12, 0x0100,
	0x6F12, 0xFF00,
	0x6F12, 0x0002,
	0x6F12, 0x0001,
	0x6F12, 0x0002,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x101C,
	0x6F12, 0x0D1C,
	0x6F12, 0x54F4,
	0x602A, 0x20BA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0100,
	0x602A, 0x0718,
	0x6F12, 0x0000,
	0x602A, 0x0710,
	0x6F12, 0x0010,
	0x6F12, 0x0201,
	0x6F12, 0x0800,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x1401,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0000,
	0x602A, 0x1376,
	0x6F12, 0x0000,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0400,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x5000,
	0x6F12, 0x5000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x5000,
	0x6F12, 0x5000,
	0x602A, 0x0CB6,
	0x6F12, 0x0000,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0000,
	0x602A, 0x11F6,
	0x6F12, 0x0010,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0000,
	0x0346, 0x0000,
	0x0348, 0x1FFF,
	0x034A, 0x181F,
	0x034C, 0x1FE0,
	0x034E, 0x1800,
	0x0350, 0x0010,
	0x0352, 0x0010,
	0x0900, 0x0111,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0110, 0x1002,
	0x0114, 0x0300,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x006B,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x1920,
	0x0342, 0x21F0,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0100,
	0x0D02, 0x0001,
	0x0D04, 0x0002,
	0x6226, 0x0000,
	0x6028, 0x4000,
	0xB13C, 0x0800,
	0x0810, 0x0000,
};

static kal_uint16 addr_data_pair_custom6[] = { // 3264*2448
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x1201,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x1F04,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0330,
	0x0346, 0x0270,
	0x0348, 0x1CCF,
	0x034A, 0x15AF,
	0x034C, 0x0CC0,
	0x034E, 0x0990,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x007F,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0FF8,
	0x0342, 0x11E8,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
};

static kal_uint16 addr_data_pair_custom7[] = { // 3264*1836
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0100,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0120,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0xFF00,
	0x602A, 0x19E6,
	0x6F12, 0x0200,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA060,
	0x602A, 0x1A3C,
	0x6F12, 0x6207,
	0x602A, 0x1A48,
	0x6F12, 0x6207,
	0x602A, 0x1444,
	0x6F12, 0x2000,
	0x6F12, 0x2000,
	0x602A, 0x144C,
	0x6F12, 0x3F00,
	0x6F12, 0x3F00,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x2F00,
	0x6F12, 0xFA00,
	0x6F12, 0x2400,
	0x6F12, 0xE500,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8A00,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x1800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x0100,
	0x602A, 0x09C0,
	0x6F12, 0x2008,
	0x602A, 0x09C4,
	0x6F12, 0x2000,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x0005,
	0x6F12, 0x000A,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x000A,
	0x6F12, 0x0040,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x8002,
	0x6F12, 0xFD03,
	0x6F12, 0x0010,
	0x6F12, 0x1510,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x1201,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0x1F04,
	0x602A, 0x2080,
	0x6F12, 0x0101,
	0x6F12, 0xFF00,
	0x6F12, 0x7F01,
	0x6F12, 0x0001,
	0x6F12, 0x8001,
	0x6F12, 0xD244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x20BA,
	0x6F12, 0x121C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0200,
	0x602A, 0x13AE,
	0x6F12, 0x0101,
	0x602A, 0x0718,
	0x6F12, 0x0001,
	0x602A, 0x0710,
	0x6F12, 0x0002,
	0x6F12, 0x0804,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0000,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x1360,
	0x6F12, 0x0100,
	0x602A, 0x1376,
	0x6F12, 0x0100,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x0000,
	0x602A, 0x4A94,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0300,
	0x6F12, 0x0000,
	0x6F12, 0x0900,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0100,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0100,
	0x602A, 0x11F6,
	0x6F12, 0x0020,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0330,
	0x0346, 0x04D4,
	0x0348, 0x1CCF,
	0x034A, 0x134B,
	0x034C, 0x0CC0,
	0x034E, 0x072C,
	0x0350, 0x0008,
	0x0352, 0x0008,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000,
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x0082,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x007F,
	0x0312, 0x0000,
	0x080E, 0x0000,
	0x0340, 0x0FF8,
	0x0342, 0x11E8,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
};

static kal_uint16 addr_data_pair_custom8[] = {  //2032x1136 30fps
	0x6028, 0x2400,
	0x602A, 0x1A28,
	0x6F12, 0x4C00,
	0x602A, 0x065A,
	0x6F12, 0x0000,
	0x602A, 0x139E,
	0x6F12, 0x0300,
	0x602A, 0x139C,
	0x6F12, 0x0000,
	0x602A, 0x13A0,
	0x6F12, 0x0A00,
	0x6F12, 0x0020,
	0x602A, 0x2072,
	0x6F12, 0x0000,
	0x602A, 0x1A64,
	0x6F12, 0x0301,
	0x6F12, 0x3F00,
	0x602A, 0x19E6,
	0x6F12, 0x0201,
	0x602A, 0x1A30,
	0x6F12, 0x3401,
	0x602A, 0x19FC,
	0x6F12, 0x0B00,
	0x602A, 0x19F4,
	0x6F12, 0x0606,
	0x602A, 0x19F8,
	0x6F12, 0x1010,
	0x602A, 0x1B26,
	0x6F12, 0x6F80,
	0x6F12, 0xA020,
	0x602A, 0x1A3C,
	0x6F12, 0x5207,
	0x602A, 0x1A48,
	0x6F12, 0x5207,
	0x602A, 0x1444,
	0x6F12, 0x2100,
	0x6F12, 0x2100,
	0x602A, 0x144C,
	0x6F12, 0x4200,
	0x6F12, 0x4200,
	0x602A, 0x7F6C,
	0x6F12, 0x0100,
	0x6F12, 0x3100,
	0x6F12, 0xF700,
	0x6F12, 0x2600,
	0x6F12, 0xE100,
	0x602A, 0x0650,
	0x6F12, 0x0600,
	0x602A, 0x0654,
	0x6F12, 0x0000,
	0x602A, 0x1A46,
	0x6F12, 0x8600,
	0x602A, 0x1A52,
	0x6F12, 0xBF00,
	0x602A, 0x0674,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x6F12, 0x0500,
	0x602A, 0x0668,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x6F12, 0x0800,
	0x602A, 0x0684,
	0x6F12, 0x4001,
	0x602A, 0x0688,
	0x6F12, 0x4001,
	0x602A, 0x147C,
	0x6F12, 0x1000,
	0x602A, 0x1480,
	0x6F12, 0x1000,
	0x602A, 0x19F6,
	0x6F12, 0x0904,
	0x602A, 0x0812,
	0x6F12, 0x0000,
	0x602A, 0x1A02,
	0x6F12, 0x0800,
	0x602A, 0x2148,
	0x6F12, 0x0100,
	0x602A, 0x2042,
	0x6F12, 0x1A00,
	0x602A, 0x0874,
	0x6F12, 0x1100,
	0x602A, 0x09C0,
	0x6F12, 0x9800,
	0x602A, 0x09C4,
	0x6F12, 0x9800,
	0x602A, 0x19FE,
	0x6F12, 0x0E1C,
	0x602A, 0x4D92,
	0x6F12, 0x0100,
	0x602A, 0x84C8,
	0x6F12, 0x0100,
	0x602A, 0x4D94,
	0x6F12, 0x4001,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0004,
	0x6F12, 0x0010,
	0x6F12, 0x0810,
	0x6F12, 0x0810,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0010,
	0x6F12, 0x0010,
	0x602A, 0x3570,
	0x6F12, 0x0000,
	0x602A, 0x3574,
	0x6F12, 0x0000,
	0x602A, 0x21E4,
	0x6F12, 0x0400,
	0x602A, 0x21EC,
	0x6F12, 0xE000,
	0x602A, 0x2080,
	0x6F12, 0x0100,
	0x6F12, 0x7F00,
	0x6F12, 0x0002,
	0x6F12, 0x8000,
	0x6F12, 0x0002,
	0x6F12, 0xC244,
	0x6F12, 0xD244,
	0x6F12, 0x14F4,
	0x6F12, 0x141C,
	0x6F12, 0x111C,
	0x6F12, 0x54F4,
	0x602A, 0x20BA,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x120E,
	0x6F12, 0x1000,
	0x602A, 0x212E,
	0x6F12, 0x0A00,
	0x602A, 0x13AE,
	0x6F12, 0x0102,
	0x602A, 0x0718,
	0x6F12, 0x0005,
	0x602A, 0x0710,
	0x6F12, 0x0004,
	0x6F12, 0x0401,
	0x6F12, 0x0100,
	0x602A, 0x1B5C,
	0x6F12, 0x0300,
	0x602A, 0x0786,
	0x6F12, 0x7701,
	0x602A, 0x2022,
	0x6F12, 0x0101,
	0x6F12, 0x0101,
	0x602A, 0x1360,
	0x6F12, 0x0000,
	0x602A, 0x1376,
	0x6F12, 0x0200,
	0x6F12, 0x6038,
	0x6F12, 0x7038,
	0x6F12, 0x8038,
	0x602A, 0x1386,
	0x6F12, 0x0B00,
	0x602A, 0x06FA,
	0x6F12, 0x1000,
	0x602A, 0x4A94,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0600,
	0x6F12, 0x0000,
	0x6F12, 0x0C00,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x0A76,
	0x6F12, 0x1000,
	0x602A, 0x0AEE,
	0x6F12, 0x1000,
	0x602A, 0x0B66,
	0x6F12, 0x1000,
	0x602A, 0x0BDE,
	0x6F12, 0x1000,
	0x602A, 0x0BE8,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0C56,
	0x6F12, 0x1000,
	0x602A, 0x0C60,
	0x6F12, 0x3000,
	0x6F12, 0x3000,
	0x602A, 0x0CB6,
	0x6F12, 0x0000,
	0x602A, 0x0CF2,
	0x6F12, 0x0001,
	0x602A, 0x0CF0,
	0x6F12, 0x0101,
	0x602A, 0x11B8,
	0x6F12, 0x0000,
	0x602A, 0x11F6,
	0x6F12, 0x0010,
	0x602A, 0x4A74,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0xD8FF,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x602A, 0x218E,
	0x6F12, 0x0000,
	0x602A, 0x2268,
	0x6F12, 0xF279,
	0x602A, 0x5006,
	0x6F12, 0x0000,
	0x602A, 0x500E,
	0x6F12, 0x0100,
	0x602A, 0x4E70,
	0x6F12, 0x2062,
	0x6F12, 0x5501,
	0x602A, 0x06DC,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0xF46A, 0xAE80,
	0x0344, 0x0010,
	0x0346, 0x0320,
	0x0348, 0x1FEF,
	0x034A, 0x14FF,
	0x034C, 0x07F0,
	0x034E, 0x0470,
	0x0350, 0x0004,
	0x0352, 0x0004,
	0x0900, 0x0144,
	0x0380, 0x0002,
	0x0382, 0x0006,
	0x0384, 0x0002,
	0x0386, 0x0006,
	0x0110, 0x1002,
	0x0114, 0x0301,
	0x0116, 0x3000, // pd format 0x2B
	0x0136, 0x1A00,
	0x013E, 0x0000,
	0x0300, 0x0006,
	0x0302, 0x0001,
	0x0304, 0x0004,
	0x0306, 0x008A,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0003,
	0x0310, 0x005D,
	0x0312, 0x0001,
	0x080E, 0x0000,
	0x0340, 0x1038,
	0x0342, 0x12C0,
	0x0702, 0x0000,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x0D00, 0x0101,
	0x0D02, 0x0101,
	0x0D04, 0x0102,
	0x6226, 0x0000,
	0x0816, 0x0000, //for HS-EXIT TIME
};
static void sensor_init(void)
{
	//Global setting
	PK_DBG("start \n");
	write_cmos_sensor_16_16(0x6028,0x4000);
	write_cmos_sensor_16_16(0x0000,0x0003);
	write_cmos_sensor_16_16(0x0000,0x38E1);
	write_cmos_sensor_16_16(0x001E,0x0007);
	write_cmos_sensor_16_16(0x6028,0x4000);
	write_cmos_sensor_16_16(0x6010,0x0001);
	mdelay(5);             /*delay 6ms*/
	write_cmos_sensor_16_16(0x6226,0x0001);
	mdelay(10);             /*delay 10ms*/

	PK_DBG("sensor_init default otp setting \n");
	table_write_cmos_sensor(addr_data_pair_init,
		  sizeof(addr_data_pair_init) / sizeof(kal_uint16));

	bIsLongExposure = KAL_FALSE;

	PK_DBG("end \n");
}	/*	sensor_init  */

static void preview_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	PK_DBG("end \n");
}	/*	preview_setting  */

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps)
{
	PK_DBG("start \n");

	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void normal_video_setting(kal_uint16 currefps)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void hs_video_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void slim_video_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom1_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom2_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom3_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom4_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom5_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom5,
		   sizeof(addr_data_pair_custom5) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom6_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom6,
		   sizeof(addr_data_pair_custom6) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom7_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom7,
		   sizeof(addr_data_pair_custom7) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

static void custom8_setting(void)
{
	PK_DBG("start \n");
	table_write_cmos_sensor(addr_data_pair_custom8,
		   sizeof(addr_data_pair_custom8) / sizeof(kal_uint16));
	PK_DBG("end \n");
}

#if 0
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
#ifdef CONFIG_MTK_CAM_PD2083F_EX

	struct IMGSENSOR_I2C_CFG *pi2c_cfg = NULL;

	if (!params) {
		pr_debug("ois_: sensor device handle error");
		return ERROR_NONE;
	}

	pi2c_cfg = imgsensor_i2c_get_device();
	if (pi2c_cfg && pi2c_cfg->pinst) {
		pr_debug("ois_: fw check start(sensor=%d i2c=%p)", imgsensor_info.sensor_id, pi2c_cfg->pinst->pi2c_client);
		ois_interface_create(pi2c_cfg->pinst->pi2c_client, (struct device *)params, OISDRV_DW9781C);
		//ois_interface_dispatcher(AFIOC_X_OIS_INIT, NULL, OISDRV_DW9781C);
		ois_interface_dispatcher(AFIOC_X_OIS_READY_CHECK, NULL, OISDRV_DW9781C);
		ois_interface_dispatcher(AFIOC_X_OIS_FWUPDATE, NULL, OISDRV_DW9781C);
		//ois_interface_dispatcher(AFIOC_X_OIS_DEINIT, NULL, OISDRV_DW9781C);
		ois_interface_destroy(OISDRV_DW9781C);

		pr_debug("ois_: fw check end");
	}  else {
		pr_debug("ois_: i2c instance fail(cfg=%p, inst=%p)", pi2c_cfg, pi2c_cfg->pinst);
	}

#endif
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
 extern int check_i2c_timeout(u16 addr, u16 i2cid);
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = read_cmos_sensor_16_16(0x0000);
			pr_err("read out sensor id 0x%x \n",*sensor_id);

			if (( *sensor_id == imgsensor_info.sensor_id) || ( *sensor_id == 0xe138) || ( *sensor_id == 0x38ef)) {
				pr_err("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				/*vivo lxd add for CameraEM otp errorcode*/
				pr_debug("start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = MAIN_38E1_otp_read();
				pr_debug("read eeprom ---vivo_otp_read_when_power_on = %d,MAIN_F8D1_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, S5KJN1SQ03_OTP_ERROR_CODE);
				/*vivo lxd add end*/
				return ERROR_NONE;
			}
			pr_err("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF*/
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

	PK_DBG("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = read_cmos_sensor_16_16(0x0000);
			if ((sensor_id == imgsensor_info.sensor_id) || (sensor_id == 0xe138) || (sensor_id == 0x38ef)) {
				PK_DBG("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			pr_err("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if ((sensor_id == imgsensor_info.sensor_id) || (sensor_id == 0xe138) || (sensor_id == 0x38ef)) {
			PK_DBG("Read sensor id success, id: 0x%x\n", sensor_id);
			break;
		}
		retry = 2;
	}

	if ((imgsensor_info.sensor_id != sensor_id) && (sensor_id != 0xe138) && (0x38ef != sensor_id)) {
		pr_err("Read sensor id fail, id: 0x%x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/* initail sequence write in  */
	sensor_init();
#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
	sensor_temperature[1] = 0;
	/*PK_DBG("sensor_temperature[0] = %d\n", sensor_temperature[0]);*/
#endif
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
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
}	/*	open  */



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
	PK_DBG("E\n");
//#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
//	sensor_temperature[1] = 0;
	/*PK_DBG("sensor_temperature[0] = %d\n", sensor_temperature[0]);*/
//#endif

	return ERROR_NONE;
}				/*      close  */


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
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	preview_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      preview   */

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
	PK_DBG("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("%s E\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	imgsensor.pclk = imgsensor_info.custom1.pclk;
	imgsensor.line_length = imgsensor_info.custom1.linelength;
	imgsensor.frame_length = imgsensor_info.custom1.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom1_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom1   */

static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;
	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom2_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom2   */

static kal_uint32 Custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	PK_DBG("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	imgsensor.pclk = imgsensor_info.custom3.pclk;
	imgsensor.line_length = imgsensor_info.custom3.linelength;
	imgsensor.frame_length = imgsensor_info.custom3.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom3_setting();
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom3   */


 static kal_uint32 Custom4(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
 {
	 PK_DBG("E\n");

	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM4;
	 imgsensor.pclk = imgsensor_info.custom4.pclk;
	 imgsensor.line_length = imgsensor_info.custom4.linelength;
	 imgsensor.frame_length = imgsensor_info.custom4.framelength;
	 imgsensor.min_frame_length = imgsensor_info.custom4.framelength;
	 imgsensor.autoflicker_en = KAL_FALSE;
	 spin_unlock(&imgsensor_drv_lock);

	 custom4_setting();
	 set_mirror_flip(imgsensor.mirror);

	 return ERROR_NONE;
 }	 /*  custom4   */

static kal_uint32 Custom5(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
 {
	 PK_DBG("E\n");

	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	 imgsensor.pclk = imgsensor_info.custom5.pclk;
	 imgsensor.line_length = imgsensor_info.custom5.linelength;
	 imgsensor.frame_length = imgsensor_info.custom5.framelength;
	 imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	 imgsensor.autoflicker_en = KAL_FALSE;
	 spin_unlock(&imgsensor_drv_lock);

	 custom5_setting();
	 set_mirror_flip(imgsensor.mirror);

	 return ERROR_NONE;
 }	 /*  custom5   */

static kal_uint32 Custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
 {
	 PK_DBG("E\n");

	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM6;
	 imgsensor.pclk = imgsensor_info.custom6.pclk;
	 imgsensor.line_length = imgsensor_info.custom6.linelength;
	 imgsensor.frame_length = imgsensor_info.custom6.framelength;
	 imgsensor.min_frame_length = imgsensor_info.custom6.framelength;
	 imgsensor.autoflicker_en = KAL_FALSE;
	 spin_unlock(&imgsensor_drv_lock);

	 custom6_setting();
	 set_mirror_flip(imgsensor.mirror);

	 return ERROR_NONE;
 }	 /*  custom6   */

 static kal_uint32 Custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
 {
	 PK_DBG("E\n");

	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM7;
	 imgsensor.pclk = imgsensor_info.custom7.pclk;
	 imgsensor.line_length = imgsensor_info.custom7.linelength;
	 imgsensor.frame_length = imgsensor_info.custom7.framelength;
	 imgsensor.min_frame_length = imgsensor_info.custom7.framelength;
	 imgsensor.autoflicker_en = KAL_FALSE;
	 spin_unlock(&imgsensor_drv_lock);

	 custom7_setting();
	 set_mirror_flip(imgsensor.mirror);

	 return ERROR_NONE;
 }	 /*  custom7   */

 static kal_uint32 Custom8(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
 {
	 PK_DBG("E\n");

	 spin_lock(&imgsensor_drv_lock);
	 imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM8;
	 imgsensor.pclk = imgsensor_info.custom8.pclk;
	 imgsensor.line_length = imgsensor_info.custom8.linelength;
	 imgsensor.frame_length = imgsensor_info.custom8.framelength;
	 imgsensor.min_frame_length = imgsensor_info.custom8.framelength;
	 imgsensor.autoflicker_en = KAL_FALSE;
	 spin_unlock(&imgsensor_drv_lock);

	 custom8_setting();
	 set_mirror_flip(imgsensor.mirror);

	 return ERROR_NONE;
 }	 /*  custom8   */


static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	PK_DBG("%s E\n", __func__);

	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;

	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;

	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;

	sensor_resolution->SensorCustom4Width = imgsensor_info.custom4.grabwindow_width;
	sensor_resolution->SensorCustom4Height =imgsensor_info.custom4.grabwindow_height;

	sensor_resolution->SensorCustom5Width = imgsensor_info.custom5.grabwindow_width;
	sensor_resolution->SensorCustom5Height =imgsensor_info.custom5.grabwindow_height;

	sensor_resolution->SensorCustom6Width = imgsensor_info.custom6.grabwindow_width;
	sensor_resolution->SensorCustom6Height =imgsensor_info.custom6.grabwindow_height;

	sensor_resolution->SensorCustom7Width = imgsensor_info.custom7.grabwindow_width;
	sensor_resolution->SensorCustom7Height =imgsensor_info.custom7.grabwindow_height;

	sensor_resolution->SensorCustom8Width = imgsensor_info.custom8.grabwindow_width;
	sensor_resolution->SensorCustom8Height =imgsensor_info.custom8.grabwindow_height;

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*PK_DBG("get_info -> scenario_id = %d\n", scenario_id);*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

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

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* change pdaf support mode to pdaf VC mode */
	/*0: NO PDAF, 1: PDAF Raw Data mode, 2:PDAF VC mode */
	sensor_info->PDAF_Support = PDAF_SUPPORT_CAMSV;

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

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

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;

	        break;
	    case MSDK_SCENARIO_ID_CUSTOM2:
	        sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
	        sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;

	        sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;

	        break;

	    case MSDK_SCENARIO_ID_CUSTOM3:
			sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;

			break;
	 	case MSDK_SCENARIO_ID_CUSTOM4:
			sensor_info->SensorGrabStartX = imgsensor_info.custom4.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom4.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom4.mipi_data_lp2hs_settle_dc;

			break;

	 	case MSDK_SCENARIO_ID_CUSTOM5:
			sensor_info->SensorGrabStartX = imgsensor_info.custom5.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom5.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom5.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			sensor_info->SensorGrabStartX = imgsensor_info.custom6.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom6.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom6.mipi_data_lp2hs_settle_dc;

			break;

	 	case MSDK_SCENARIO_ID_CUSTOM7:
			sensor_info->SensorGrabStartX = imgsensor_info.custom7.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom7.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom7.mipi_data_lp2hs_settle_dc;

		 	break;

		case MSDK_SCENARIO_ID_CUSTOM8:
			sensor_info->SensorGrabStartX = imgsensor_info.custom8.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom8.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom8.mipi_data_lp2hs_settle_dc;

		 	break;

		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}				/*      get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
		Custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		Custom3(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM4:
		Custom4(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		Custom5(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		Custom6(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		Custom7(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		Custom8(image_window, sensor_config_data);
		break;
	default:
		PK_DBG("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //PK_DBG("framerate = %d\n ", framerate); */
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

static kal_uint32 set_auto_flicker_mode(
	kal_bool enable, UINT16 framerate)
{
	PK_DBG("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)		/* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	PK_DBG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;
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
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
		    / framerate * 10 / imgsensor_info.normal_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
	    (frame_length > imgsensor_info.normal_video.framelength)
	  ? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

		imgsensor.frame_length =
		 imgsensor_info.normal_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		PK_DBG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
		frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
	    set_dummy();
	    break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

		imgsensor.frame_length =
		    imgsensor_info.hs_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		  (frame_length > imgsensor_info.slim_video.framelength)
		? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

		imgsensor.frame_length =
		  imgsensor_info.slim_video.framelength + imgsensor.dummy_line;

		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10 / imgsensor_info.custom1.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? (frame_length - imgsensor_info.custom1.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
		set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10 / imgsensor_info.custom3.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? (frame_length - imgsensor_info.custom3.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;

	case MSDK_SCENARIO_ID_CUSTOM4:
		frame_length = imgsensor_info.custom4.pclk / framerate * 10 / imgsensor_info.custom4.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom4.framelength) ? (frame_length - imgsensor_info.custom4.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom4.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM5:
		frame_length = imgsensor_info.custom5.pclk / framerate * 10 / imgsensor_info.custom5.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom5.framelength) ? (frame_length - imgsensor_info.custom5.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom5.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM6:
		frame_length = imgsensor_info.custom6.pclk / framerate * 10 / imgsensor_info.custom6.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom6.framelength) ? (frame_length - imgsensor_info.custom6.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom6.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM7:
		frame_length = imgsensor_info.custom7.pclk / framerate * 10 / imgsensor_info.custom7.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom7.framelength) ? (frame_length - imgsensor_info.custom7.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom7.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM8:
		frame_length = imgsensor_info.custom8.pclk / framerate * 10 / imgsensor_info.custom8.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line = (frame_length > imgsensor_info.custom8.framelength) ? (frame_length - imgsensor_info.custom8.framelength) : 0;
		imgsensor.frame_length = imgsensor_info.custom8.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

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
		else {
			/*No need to set*/
			PK_DBG("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		PK_DBG("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*PK_DBG("scenario_id = %d\n", scenario_id);*/

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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	PK_DBG("enable: %d\n", enable);

	if (enable) {
/* 0 : Normal, 1 : Solid Color, 2 : Color Bar, 3 : Shade Color Bar, 4 : PN9 */
		write_cmos_sensor_16_16(0x0600, 0x0001);
	} else {
		write_cmos_sensor_16_16(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE  //add for s5kjn1sq03 temperature
static kal_uint32 get_sensor_temperature(void)
{

	static DEFINE_RATELIMIT_STATE(ratelimit, 2 * HZ, 1);
	kal_uint32	TMC_000A_Value=0,TMC_Value =0,TMC_Shift_Value=0;

	write_cmos_sensor_16_16(0x6028, 0x4000);
	TMC_000A_Value=read_cmos_sensor_16_16(0x0020);
	//TMC_Shift_Value =((TMC_000A_Value&0x00FF)<<8)|((TMC_000A_Value&0xFF00)>>8);
	TMC_Shift_Value =TMC_000A_Value;
	//PK_DBG("GW3SP13 TMC 0X000A Value=0x%4x, TMC Shift Value = 0x%4X\n", TMC_000A_Value,TMC_Shift_Value);

	if (TMC_Shift_Value > 0x7FFF ){
		if (__ratelimit(&ratelimit))
			PK_DBG("JN1 TMC Value is < 0 ,TMC is Disable TMC_0020_Value =0x%x\n", TMC_000A_Value);
	}
	else{
		TMC_Value=(TMC_Shift_Value>>8)&0xFF;
		if (TMC_Value > 100){
			PK_DBG("JN1 TMC 0x0020 temperature Value=%d, cause of TMC > 100 \n", TMC_Value);
			TMC_Value = 0;
		}
		sensor_temperature[1] = TMC_Value;

		if (__ratelimit(&ratelimit))
			PK_DBG("JN1 TMC 0x0020 temperature Value=0x%x, sensor_temperature[1] =%d\n", TMC_Value, sensor_temperature[1]);
	}

	return TMC_Value;
}
#endif

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	struct SENSOR_RAWINFO_STRUCT *rawinfo;
#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE //add for s5kjn1sq03 temperature
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
#endif

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    struct SENSOR_VC_INFO_STRUCT *pvcinfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*PK_DBG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(feature_data + 1) = imgsensor_info.min_gain;
		    	*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(feature_data + 1) = imgsensor_info.min_gain;
		    	*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(feature_data + 1) = imgsensor_info.min_gain;
		    	*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(feature_data + 1) = imgsensor_info.min_gain;
		    	*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(feature_data + 1) = imgsensor_info.min_gain;
		    	*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
				break;

		  case MSDK_SCENARIO_ID_CUSTOM2:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
			 break;

			 case MSDK_SCENARIO_ID_CUSTOM3:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
			 break;

			 case MSDK_SCENARIO_ID_CUSTOM4:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = 1024;  //imgsensor_info.max_gain
			 break;
			 case MSDK_SCENARIO_ID_CUSTOM5:
				 *(feature_data + 1) = imgsensor_info.min_gain;
				 *(feature_data + 2) = 1024;  /*imgsensor_info.max_gain*/
			 break;
			 case MSDK_SCENARIO_ID_CUSTOM6:
				*(feature_data + 1) = imgsensor_info.min_gain;
				*(feature_data + 2) = 4096;  //imgsensor_info.max_gain
			 break;
			 case MSDK_SCENARIO_ID_CUSTOM7:
				 *(feature_data + 1) = imgsensor_info.min_gain;
				 *(feature_data + 2) = 4096;  /*imgsensor_info.max_gain*/
			 break;
			 case MSDK_SCENARIO_ID_CUSTOM8:
				 *(feature_data + 1) = imgsensor_info.min_gain;
				 *(feature_data + 2) = 1024;  /*imgsensor_info.max_gain*/
			 break;
			default:
				*(feature_data + 1) = imgsensor_info.min_gain;
			    *(feature_data + 2) = 4096;  //imgsensor_info.max_gain
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
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(feature_data + 1) = 6;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(feature_data + 1) = 6;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;

			case MSDK_SCENARIO_ID_CUSTOM3:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;

			case MSDK_SCENARIO_ID_CUSTOM4:
				*(feature_data + 1) = 6;  /////  imgsensor_info.min_shutter
				break;

			case MSDK_SCENARIO_ID_CUSTOM5:
				*(feature_data + 1) = 5;  /////  imgsensor_info.min_shutter
				break;
			case MSDK_SCENARIO_ID_CUSTOM6:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;

			case MSDK_SCENARIO_ID_CUSTOM7:
			 	*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
			 	break;

			case MSDK_SCENARIO_ID_CUSTOM8:
			 	*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
			 	break;
			default:
				*(feature_data + 1) = 4;  /////  imgsensor_info.min_shutter
				break;
		}

		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 4316666;
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
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
#if 0
		PK_DBG(
			"feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
#endif
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;

	case SENSOR_FEATURE_SET_REGISTER:
		PK_DBG("SENSOR_FEATURE_SET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
		PK_DBG("SENSOR_FEATURE_GET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
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
		set_auto_flicker_mode((BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_DATA\n");
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		PK_DBG("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_RAW_INFO:
		PK_DBG("SENSOR_FEATURE_GET_RAW_INFO scenarioId:%d\n",
			(UINT32) *feature_data);
		rawinfo = (struct SENSOR_RAWINFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		memcpy((void *)rawinfo,
				(void *)&imgsensor_raw_info,
				sizeof(struct SENSOR_RAWINFO_STRUCT));
		break;
	case SENSOR_FEATURE_SET_HDR:
		PK_DBG("ihdr enable :%d\n", (BOOL)*feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;

	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
  		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CUSTOM8:
            *feature_return_para_32 = 4; /*BINNING_NONE,*/
            break;
		case MSDK_SCENARIO_ID_CUSTOM5:
            *feature_return_para_32 = 1; /*BINNING_NONE,*/
			break;
		default:
			*feature_return_para_32 = 4; /*BINNING_NONE,*/
			break;
		}
		PK_DBG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* PK_DBG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		      break;
		case MSDK_SCENARIO_ID_CUSTOM2:
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[6],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		      break;
		case MSDK_SCENARIO_ID_CUSTOM3:
		      memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[7],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
		     break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[8],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			 break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[9],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			 break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[10],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			 break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[11],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			 break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			  memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[12],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			 break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
 #if 1
	case SENSOR_FEATURE_GET_PDAF_INFO:
		PK_DBG("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%lld\n", *feature_data);
		PDAFinfo= (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CUSTOM1:
				imgsensor_pd_info.i4BlockNumX = 508;
				imgsensor_pd_info.i4BlockNumY = 382;
				//imgsensor_pd_info.i4BlockNumY = 430;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;

      		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//case MSDK_SCENARIO_ID_CUSTOM4:
				imgsensor_pd_info.i4BlockNumX = 508;
				imgsensor_pd_info.i4BlockNumY = 286;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));

				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
			case MSDK_SCENARIO_ID_CUSTOM3:
				imgsensor_pd_info.i4BlockNumX = 480;
				imgsensor_pd_info.i4BlockNumY = 268;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;

			case MSDK_SCENARIO_ID_CUSTOM6:
				imgsensor_pd_info.i4BlockNumX = 408;
				imgsensor_pd_info.i4BlockNumY = 304;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;

			case MSDK_SCENARIO_ID_CUSTOM7:
				imgsensor_pd_info.i4BlockNumX = 408;
				imgsensor_pd_info.i4BlockNumY = 228;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_CUSTOM4:
				imgsensor_pd_info_binning.i4BlockNumX = 254;
				imgsensor_pd_info_binning.i4BlockNumY = 382;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_binning,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			case MSDK_SCENARIO_ID_CUSTOM8:
				imgsensor_pd_info_binning.i4BlockNumX = 254;
				imgsensor_pd_info_binning.i4BlockNumY = 282;
				memcpy((void *)PDAFinfo,(void *)&imgsensor_pd_info_binning,sizeof(struct SET_PD_BLOCK_INFO_T));
				break;
			default:
				break;
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		PK_DBG("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%lld\n", *feature_data);
		/*PDAF capacity enable or not, 2p8 only full size support PDAF*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 4080x3060 */
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;  /* 4080x3060 */
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 4080x2296 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 4080x3060 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 3840x2160 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 3840x2160 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 2032*1536 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 3264×2448 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 3264×1836 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1; /* 2032*1136 */
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; /* FULL */
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
			break;
		}
		break;
/*
    case SENSOR_FEATURE_SET_PDAF:
        PK_DBG("PDAF mode :%d\n", *feature_data_16);
        imgsensor.pdaf_mode= *feature_data_16;
        break;
    */
    case SENSOR_FEATURE_GET_VC_INFO:
        PK_DBG("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16)*feature_data);
        pvcinfo = (struct SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_CUSTOM4:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM6:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM7:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[4], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM4:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[5], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CUSTOM8:
		    memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[6], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CUSTOM1:
		default:
		    memcpy((void *)pvcinfo, (void *) &SENSOR_VC_INFO[0], sizeof(struct SENSOR_VC_INFO_STRUCT));
		    break;
		}
		break;
#endif
	#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN_F8D1_OTP_ERROR_CODE:%d \n", *feature_data,S5KJN1SQ03_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			PK_DBG("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)((feature_data+1)) = S5KJN1SQ03_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_main_s5kjn1sq03, sizeof(MUINT32)*13);
				memcpy( feature_data+10, material_inf_main_s5kjn1sq03, sizeof(MUINT32)*4);
				memcpy( feature_data+12, af_calib_inf_main_s5kjn1sq03pd2282, sizeof(MUINT32)*6);
				memcpy( feature_data+15, moduleid_inf_main_s5kjn1sq03, sizeof(MUINT32)*2);
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_main_s5kjn1sq03[%d]= 0x%x\n", i, sn_inf_main_s5kjn1sq03[i]);
						}
				#endif
			}
				break;
			}
			break;
	#endif
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		PK_DBG("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
		break;
#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE  //add for s5kjn1sq03 temperature
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature();
		*feature_para_len = 4;
		break;
#endif
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(feature_data + 1) = 1;
		    	*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(feature_data + 1) = 1;
		    	*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(feature_data + 1) = 1;
		    	*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(feature_data + 1) = 1;
		    	*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(feature_data + 1) = 1;
		    	*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(feature_data + 1) = 1;
				*(feature_data + 2) = 10;   ////imgsensor_info.margin
				 break;
			 case MSDK_SCENARIO_ID_CUSTOM3:
				 *(feature_data + 1) = 1;
				 *(feature_data + 2) = 5;	////imgsensor_info.margin
				 break;

			default:
				*(feature_data + 1) = 1;
				*(feature_data + 2) = 10;   ////imgsensor_info.margin
				break;
		}

		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		PK_DBG("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		PK_DBG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		int type = (kal_uint16)(*feature_data);
		UINT32 i = 0;
		char *data = (char *)(*(feature_data+1));
		/*only copy Cross Talk calibration data*/
		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {
			data[0] = 0xC0;
			data[1] = 0x22;
			for(i = 0; i< 8896 ; i++)
			data[2 + i] = crosstalk_data_PD2282[i];
			pr_debug("read Cross Talk calibration data size= i= %d data=0x%x\n",
				i, data[i]);
		}
	}
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	PK_DBG("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		PK_DBG("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
		*feature_return_para_32 =  imgsensor.current_ae_effective_frame;
		break;


	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.cap.pclk /
			(imgsensor_info.cap.linelength - 80))*
			imgsensor_info.cap.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.normal_video.pclk /
			(imgsensor_info.normal_video.linelength - 80))*
			imgsensor_info.normal_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.hs_video.pclk /
			(imgsensor_info.hs_video.linelength - 80))*
			imgsensor_info.hs_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.slim_video.pclk /
			(imgsensor_info.slim_video.linelength - 80))*
			imgsensor_info.slim_video.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom1.pclk /
			(imgsensor_info.custom1.linelength - 80))*
			imgsensor_info.custom1.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom2.pclk /
			(imgsensor_info.custom2.linelength - 80))*
			imgsensor_info.custom2.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom3.pclk /
			(imgsensor_info.custom3.linelength - 80))*
			imgsensor_info.custom3.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom4.pclk /
			(imgsensor_info.custom4.linelength - 80))*
			imgsensor_info.custom4.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom5.pclk /
			(imgsensor_info.custom5.linelength - 80))*
			imgsensor_info.custom5.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom6.pclk /
			(imgsensor_info.custom6.linelength - 80))*
			imgsensor_info.custom6.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom7.pclk /
			(imgsensor_info.custom7.linelength - 80))*
			imgsensor_info.custom7.grabwindow_width;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom8.pclk /
			(imgsensor_info.custom8.linelength - 80))*
			imgsensor_info.custom8.grabwindow_width;
			break;

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.pre.pclk /
			(imgsensor_info.pre.linelength - 80))*
			imgsensor_info.pre.grabwindow_width;
			break;
		}
		break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom3.mipi_pixel_rate;
			break;

		case MSDK_SCENARIO_ID_CUSTOM4:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom4.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom5.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM6:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom6.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM7:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom7.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM8:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom8.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		/* modify to separate 3hdr and remosaic */
		if (imgsensor.sensor_mode == IMGSENSOR_MODE_CUSTOM5) {
			/*write AWB gain to sensor*/
			feedback_awbgain((UINT32)*(feature_data_32 + 1),
					(UINT32)*(feature_data_32 + 2));
		} else {
			gw3sp_awb_gain(
				(struct SET_SENSOR_AWB_GAIN *) feature_para);
		}
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
	{
		kal_uint8 index =
			*(((kal_uint8 *)feature_para) + (*feature_para_len));

		gw3sp_set_lsc_reg_setting(index, feature_data_16,
					  (*feature_para_len)/sizeof(UINT16));
	}
		break;
#if 0
	/*chenhan add*/
	case SENSOR_FEATURE_OIS_FW_UPDATE:
	{
		ois_fw_check(feature_para);
		break;
	}
	/*chenwenjie add*/
	case SENSOR_FEATURE_SET_SAT_MODE:
	{
		LOG_INF("SENSOR_FEATURE_SET_SAT_MODE %d\n", (UINT16) (*feature_data));
		if (qsc_flag)
			ois_driver_create((UINT16) (*feature_data));
		break;
	}
	/*add end*/
#endif
	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control()  */

//#include "../imgsensor_hop.c"

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close,
	//hop,
  	//do_hop,
};

UINT32 S5KJN1SQ03PD2282_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
