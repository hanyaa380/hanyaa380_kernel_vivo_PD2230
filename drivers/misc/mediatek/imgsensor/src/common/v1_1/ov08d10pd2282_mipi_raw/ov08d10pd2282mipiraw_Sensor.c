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
 *	 ov08d10pd2282mipiraw_Sensor.c
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


#define PFX "OV08D10_camera_sensor"
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
#include "ov08d10pd2282mipiraw_Sensor.h"
#include "ov08d10pd2282_ana_gain_table.h"

#include "../imgsensor_common.h"

//#include "ov08d10pd2282mipiraw_freq.h"
#include "../imgsensor_sensor.h"

#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)

// static int is_standby_flag = 0;
static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = OV08D10PD2282_SENSOR_ID,
	.checksum_value = 0xb1893b4f, /*checksum value for Camera Auto Test*/

	.pre = {
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},
	.cap = {
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},
	.normal_video = {
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},
	.hs_video = { /*hs_video 120fps*/
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
		},
	.slim_video = { /* hs_video 240fps*/
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
		},
	.custom1 = { /*back setting*/
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},

	.custom2 = { /* 3264*1836 30fps */
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},
	 .custom3 = { /* 4608*2592@60fps*/
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},

	.custom4 = { /* 3264*1836 30fps */
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 1836,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},
	.custom5 = { /* 3264*2448 30fps */
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,

	},

	.custom6 = { /* 2688*2016 for bokeh 1x */
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 2688,
		.grabwindow_height = 2016,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,

	},
	.custom7 = { /* 1088*816 for bokeh 2x binning*/
		.pclk = 36075000,
		.linelength  = 460,
		.framelength = 2608,
		.startx = 0,
		.starty = 0,
		.grabwindow_width  = 1344,
		.grabwindow_height = 1008,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 288600000,
	},

	.margin = 20,
	.min_shutter = 4,
	.min_gain = 1 * BASEGAIN, /*1x gain*/
	.max_gain = 15.5 * BASEGAIN, /*15.5x * 64  gain*/
	.min_gain_iso = 50,
	.gain_step = 2, /*minimum step = 2 in 1x~2x gain*/
	.gain_type = 1,
	.max_frame_length = 0x3FFFFC,     /* max framelength by sensor register's limitation */
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 2,	/*sony sensor must be 3,non-sony sensor must be 2 , The delay frame of setting frame length  */
	.ihdr_support = 0,	  /*1, support; 0,not support*/
	.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/
	.sensor_mode_num = 12,			//support sensor mode num

	.cap_delay_frame = 3,		/*enter capture delay frame num*/
	.pre_delay_frame = 3,		/*enter preview delay frame num*/
	.video_delay_frame = 3,		/*enter video delay frame num*/
	.hs_video_delay_frame = 3, /*enter high speed video  delay frame num*/
	.slim_video_delay_frame = 3,/*enter slim video delay frame num*/
	.custom1_delay_frame = 2,
	.custom2_delay_frame = 2,
	.custom3_delay_frame = 2,
	.custom4_delay_frame = 2,
	.custom5_delay_frame = 2,
	.custom6_delay_frame = 2,
	.custom7_delay_frame = 2,
	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_2MA, /*mclk driving current*/

	/*Sensor_interface_type*/
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,

	/*sensor output first pixel color*/
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,

	.mclk = 26,/*mclk value, suggest 24 or 26 for 24Mhz or 26Mhz*/
	.mipi_lane_num = SENSOR_MIPI_2_LANE,/*mipi lane num*/

	/*record sensor support all write id addr, only supprt 4must end with 0xff*/
	.i2c_addr_table = {0x6C, 0xff},
	.i2c_speed = 400, // i2c read/write speed
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,		/*mirrorflip information*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x4C00,			/*current shutter*/
	.gain = 0x200,				/*current gain*/
	.dummy_pixel = 0,			/*current dummypixel*/
	.dummy_line = 0,			/*current dummyline*/

	/*full size current fps : 24fps for PIP, 30fps for Normal or ZSD*/
	.current_fps = 30,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,

	/*current scenario id*/
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/*current scenario id*/
	.ihdr_mode = 0, /*sensor need support LE, SE with HDR feature*/
	.i2c_write_id = 0x6c,
	.current_ae_effective_frame = 2,
	.freq_setting = 0,
  	.present_freq_setting = 0,

};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[12] = {
	{ 3264, 2448, 	0,	     0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // Preview
	{ 3264, 2448, 	0,	     0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // capture
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // video
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // hight speed video
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // slime video
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // custom1
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // custom2
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // custom3
	{ 3264, 2448, 	0,	   306,	 3264, 1836, 3264, 1836, 0, 0, 3264, 1836,  0,  0, 3264, 1836}, // custom4
	{ 3264, 2448, 	0,	     0,	 3264, 2448, 3264, 2448, 0, 0, 3264, 2448,  0,  0, 3264, 2448}, // custom5
	{ 3264, 2448, 288,	   216,	 2688, 2016, 2688, 2016, 0, 0, 2688, 2016,  0,  0, 2688, 2016}, // custom6 for bokeh 1X
	{ 3264, 2448, 960,	   720,	 1344, 1008, 1344, 1008, 0, 0, 1344, 1008,  0,  0, 1344, 1008}, // custom7
};

static struct  SENSOR_RAWINFO_STRUCT imgsensor_raw_info = {
	 3264,//raw_weight
 	 2448,//raw_height
	 2,//raw_dataBit
	 BAYER_BGGR,//raw_colorFilterValue
	 64,//raw_blackLevel
	 120,//raw_viewAngle
	 10,//raw_bitWidth
	 15//raw_maxSensorGain
};

#if 1
/*hope add otp check start*/
static int vivo_otp_read_when_power_on;
extern int vivo_main2_otp_read_ov08d10pd2282(void);
extern otp_error_code_t MAIN2_OTP_ERROR_CODE_OV08D10PD2282;
#endif
MUINT32  moduleid_inf_main2_ov08d10pd2282[2];  /*0 flag   1-12 data*/
MUINT32  sn_inf_main2_ov08d10pd2282[13];  /*0 flag   1-12 data*/
MUINT32  material_inf_main2_ov08d10pd2282[4];
/*hope add otp check end*/

#if 0
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
#endif

static kal_uint16 read_cmos_sensor_8_8(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 1, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[2] = {
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
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
#define I2C_BUFFER_LEN 765	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif

#if 0
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
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}

#if MULTI_WRITE
	if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
		iBurstWriteReg_multi(puSendCmd, tosend,
			imgsensor.i2c_write_id, 3, imgsensor_info.i2c_speed);
			tosend = 0;
	}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif

	}
	return 0;
}
#endif


static void set_dummy(void)
{
	if (imgsensor.frame_length%4 != 0)
		imgsensor.frame_length =
			imgsensor.frame_length - imgsensor.frame_length%4;

	pr_debug("imgsensor.frame_length = %d\n", imgsensor.frame_length);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x05, ((imgsensor.frame_length - 2504) * 2 & 0xFF00) >> 8);
	write_cmos_sensor_8_8(0x06, ((imgsensor.frame_length - 2504) * 2 & 0xFF));
	write_cmos_sensor_8_8(0xfd, 0x01); //page1
	write_cmos_sensor_8_8(0x01, 0x01); //fresh

}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d \n", framerate,min_framelength_en);

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
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		// if(is_standby_flag == 1) {
		// 	write_cmos_sensor_8_8(0xfd, 0x00); // soft powerup start
		// 	write_cmos_sensor_8_8(0xc2, 0x30);
		// 	write_cmos_sensor_8_8(0x21, 0x0e);
		// 	write_cmos_sensor_8_8(0x21, 0x00); // soft powerup end
		// 	is_standby_flag = 0;
		// }
		write_cmos_sensor_8_8(0xfd, 0x01);
		write_cmos_sensor_8_8(0x01, 0x03);
		write_cmos_sensor_8_8(0xfd, 0x00);
		write_cmos_sensor_8_8(0x20, 0x0f);
		write_cmos_sensor_8_8(0xe7, 0x03);
		write_cmos_sensor_8_8(0xe7, 0x00);
		write_cmos_sensor_8_8(0xa0, 0x01);
		write_cmos_sensor_8_8(0xfd, 0x01);

	} else {
		write_cmos_sensor_8_8(0xfd, 0x00);
		write_cmos_sensor_8_8(0x20, 0x0b);
		write_cmos_sensor_8_8(0xa0, 0x00);
		// if(is_standby_flag == 0) {
		// 	write_cmos_sensor_8_8(0xfd, 0x00);
		// 	write_cmos_sensor_8_8(0xc2, 0x32);
		// 	write_cmos_sensor_8_8(0x21, 0x0f);
		// 	is_standby_flag = 1;
		// }
	}
	return ERROR_NONE;
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
	kal_uint16 realtime_fps = 0;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	imgsensor.frame_length =
		imgsensor.frame_length - imgsensor.frame_length%2;
	shutter = (shutter >> 1) << 1;
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

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

	// if(is_standby_flag == 1) {
	// 	LOG_INF("set_shutter exit standby, is_standby_flag: %d\n", is_standby_flag);
	// 	write_cmos_sensor_8_8(0xfd, 0x00); // soft powerup start
	// 	write_cmos_sensor_8_8(0xc2, 0x30);
	// 	write_cmos_sensor_8_8(0x21, 0x0e);
	// 	write_cmos_sensor_8_8(0x21, 0x00); // soft powerup end
	// 	is_standby_flag = 0;
	// }

	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x05, ((imgsensor.frame_length - 2504) * 2 & 0xFF00) >> 8);
	write_cmos_sensor_8_8(0x06, ((imgsensor.frame_length - 2504) * 2 & 0xFF));
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x01, 0x01);

	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x02, ((imgsensor.shutter * 2) >> 16) & 0xFF);
	write_cmos_sensor_8_8(0x03, ((imgsensor.shutter * 2) >> 8) & 0xFF);
	write_cmos_sensor_8_8(0x04, (imgsensor.shutter * 2) & 0xFF);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x01, 0x01);

	pr_debug("shutter = %d, framelength %d", shutter, imgsensor.frame_length);
}

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 target_frame_length, kal_bool auto_extend_en)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);

	shutter = (shutter >> 1) << 1;

	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	dummy_line = target_frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter =
		    (imgsensor_info.max_frame_length - imgsensor_info.margin);

	imgsensor.frame_length =
			imgsensor.frame_length - imgsensor.frame_length % 4;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk
			/ imgsensor.line_length * 10 / imgsensor.frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length*/
				write_cmos_sensor_8_8(0xfd, 0x01);
				write_cmos_sensor_8_8(0x05, ((imgsensor.frame_length - 2504) * 2 & 0xFF00) >> 8);
				write_cmos_sensor_8_8(0x06, ((imgsensor.frame_length - 2504) * 2 & 0xFF));
				write_cmos_sensor_8_8(0xfd, 0x01);
				write_cmos_sensor_8_8(0x01, 0x01);
		}
	} else {
		/* Extend frame length*/
			write_cmos_sensor_8_8(0xfd, 0x01);
			write_cmos_sensor_8_8(0x05, ((imgsensor.frame_length - 2504) * 2 & 0xFF00) >> 8);
			write_cmos_sensor_8_8(0x06, ((imgsensor.frame_length - 2504) * 2 & 0xFF));
			write_cmos_sensor_8_8(0xfd, 0x01);
			write_cmos_sensor_8_8(0x01, 0x01);
	}

	/* Update Shutter*/
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x02, ((imgsensor.shutter * 2) >> 16) & 0xFF);
	write_cmos_sensor_8_8(0x03, ((imgsensor.shutter * 2) >> 8) & 0xFF);
	write_cmos_sensor_8_8(0x04, (imgsensor.shutter * 2) & 0xFF);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x01, 0x01);
	pr_debug("Add for N3D! shutterlzl =%d, framelength =%d\n", shutter, imgsensor.frame_length);
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
	int i = 0;
	kal_int8 reg_gain = 0x10;
	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		pr_info("Error gain setting");
		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	for (i = ARRAY_SIZE(gains) - 1; i > 0; i--) {
		if (gains[i].permile <= gain) {
			reg_gain = gains[i].code;
			break;
		}
	}

	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = gain;
	spin_unlock(&imgsensor_drv_lock);

	// if(is_standby_flag == 1) {
	// 	LOG_INF("set_gain exit standby, is_standby_flag: %d\n", is_standby_flag);
	// 	write_cmos_sensor_8_8(0xfd, 0x00); // soft powerup start
	// 	write_cmos_sensor_8_8(0xc2, 0x30);
	// 	write_cmos_sensor_8_8(0x21, 0x0e);
	// 	write_cmos_sensor_8_8(0x21, 0x00); // soft powerup end
	// 	is_standby_flag = 0;
	// }
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x24, reg_gain);
	write_cmos_sensor_8_8(0xfd, 0x01);	//page1
	write_cmos_sensor_8_8(0x01, 0x01);	//fresh

	LOG_INF("gain = %d, reg_gain = 0x%02x", gain, reg_gain);
	return gain;
}	/*	set_gain  */

static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror %x\n", image_mirror);
	switch (image_mirror) {
	case IMAGE_NORMAL:
		write_cmos_sensor_8_8(0xfd, 0x01);
		write_cmos_sensor_8_8(0x32, 0x00);
		write_cmos_sensor_8_8(0xfd, 0x05);
		write_cmos_sensor_8_8(0x04, 0x40);
		write_cmos_sensor_8_8(0x07, 0x00);
		write_cmos_sensor_8_8(0x0D, 0x01);
		write_cmos_sensor_8_8(0x0F, 0x01);
		write_cmos_sensor_8_8(0x10, 0x00);
		write_cmos_sensor_8_8(0x11, 0x00);
		write_cmos_sensor_8_8(0x12, 0x0c);
		write_cmos_sensor_8_8(0x13, 0xcf);
		write_cmos_sensor_8_8(0x14, 0x00);
		write_cmos_sensor_8_8(0x15, 0x00);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_8_8(0xfd, 0x01);
		write_cmos_sensor_8_8(0x32, 0x01);
		write_cmos_sensor_8_8(0xfd, 0x05);
		write_cmos_sensor_8_8(0x04, 0x40);
		write_cmos_sensor_8_8(0x07, 0x00);
		write_cmos_sensor_8_8(0x0D, 0x01);
		write_cmos_sensor_8_8(0x0F, 0x01);
		write_cmos_sensor_8_8(0x10, 0x0c);
		write_cmos_sensor_8_8(0x11, 0xcf);
		write_cmos_sensor_8_8(0x12, 0x00);
		write_cmos_sensor_8_8(0x13, 0x00);
		write_cmos_sensor_8_8(0x14, 0x00);
		write_cmos_sensor_8_8(0x15, 0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_8_8(0xfd, 0x01);
		write_cmos_sensor_8_8(0x32, 0x02);
		write_cmos_sensor_8_8(0xfd, 0x05);
		write_cmos_sensor_8_8(0x04, 0x40);
		write_cmos_sensor_8_8(0x07, 0x00);
		write_cmos_sensor_8_8(0x0D, 0x01);
		write_cmos_sensor_8_8(0x0F, 0x01);
		write_cmos_sensor_8_8(0x10, 0x00);
		write_cmos_sensor_8_8(0x11, 0x00);
		write_cmos_sensor_8_8(0x12, 0x0c);
		write_cmos_sensor_8_8(0x13, 0xcf);
		write_cmos_sensor_8_8(0x14, 0x09);
		write_cmos_sensor_8_8(0x15, 0x9f);
		break;
	case IMAGE_HV_MIRROR:
		write_cmos_sensor_8_8(0xfd, 0x01);
		write_cmos_sensor_8_8(0x32, 0x03);
		write_cmos_sensor_8_8(0xfd, 0x05);
		write_cmos_sensor_8_8(0x04, 0x40);
		write_cmos_sensor_8_8(0x07, 0x00);
		write_cmos_sensor_8_8(0x0D, 0x01);
		write_cmos_sensor_8_8(0x0F, 0x01);
		write_cmos_sensor_8_8(0x10, 0x0c);
		write_cmos_sensor_8_8(0x11, 0xcf);
		write_cmos_sensor_8_8(0x12, 0x00);
		write_cmos_sensor_8_8(0x13, 0x00);
		write_cmos_sensor_8_8(0x14, 0x09);
		write_cmos_sensor_8_8(0x15, 0x9f);
		break;

	default:
		pr_debug("Error image_mirror setting\n");
		break;
	}
	// if(is_standby_flag == 0) {
	// 	LOG_INF("OV08D10 is_standby_flag %d standby sensor \n", is_standby_flag);
	// 	write_cmos_sensor_8_8(0xfd, 0x00); //soft standby start
	// 	write_cmos_sensor_8_8(0xa0, 0x00);
	// 	write_cmos_sensor_8_8(0xc2, 0x32);
	// 	write_cmos_sensor_8_8(0x21, 0x0f); //soft standby end
	// 	is_standby_flag = 1;
	// }
}

static void sensor_init(void)
{
	LOG_INF("start \n");
	LOG_INF("end \n");
}	/*	sensor_init  */

static void preview_setting(void)
{
	LOG_INF("preview start \n");
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x20, 0x0e);
	write_cmos_sensor_8_8(0x20, 0x0b);
	write_cmos_sensor_8_8(0x10, 0x05);
	write_cmos_sensor_8_8(0x11, 0x2a);
	write_cmos_sensor_8_8(0x12, 0x03);
	write_cmos_sensor_8_8(0x13, 0x05);
	write_cmos_sensor_8_8(0x14, 0x3e);
	write_cmos_sensor_8_8(0x15, 0x02);
	write_cmos_sensor_8_8(0x16, 0x82);
	write_cmos_sensor_8_8(0x17, 0x05);
	write_cmos_sensor_8_8(0x18, 0x6f);
	write_cmos_sensor_8_8(0x19, 0x04);
	write_cmos_sensor_8_8(0x1a, 0x05);
	write_cmos_sensor_8_8(0x1b, 0xde);
	write_cmos_sensor_8_8(0x1c, 0x09);
	write_cmos_sensor_8_8(0x1d, 0x13);
	write_cmos_sensor_8_8(0x1e, 0x23);
	write_cmos_sensor_8_8(0x1f, 0x0f);
	write_cmos_sensor_8_8(0x20, 0x0f);
	write_cmos_sensor_8_8(0x21, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x12, 0x00);
	write_cmos_sensor_8_8(0x02, 0x00);
	write_cmos_sensor_8_8(0x03, 0x12);
	write_cmos_sensor_8_8(0x04, 0x50);
	write_cmos_sensor_8_8(0x05, 0x00);
	write_cmos_sensor_8_8(0x06, 0xd0);
	write_cmos_sensor_8_8(0x07, 0x05);
	write_cmos_sensor_8_8(0x21, 0x02);
	write_cmos_sensor_8_8(0x24, 0x30);
	write_cmos_sensor_8_8(0x33, 0x03);
	write_cmos_sensor_8_8(0x01, 0x03);
	write_cmos_sensor_8_8(0x19, 0x10);
	write_cmos_sensor_8_8(0x42, 0x55);
	write_cmos_sensor_8_8(0x43, 0x00);
	write_cmos_sensor_8_8(0x47, 0x07);
	write_cmos_sensor_8_8(0x48, 0x08);
	write_cmos_sensor_8_8(0x4c, 0x38);
	write_cmos_sensor_8_8(0xb2, 0x7e);
	write_cmos_sensor_8_8(0xb3, 0x7b);
	write_cmos_sensor_8_8(0xbd, 0x08);
	write_cmos_sensor_8_8(0xd2, 0x47);
	write_cmos_sensor_8_8(0xd3, 0x10);
	write_cmos_sensor_8_8(0xd4, 0x0d);
	write_cmos_sensor_8_8(0xd5, 0x08);
	write_cmos_sensor_8_8(0xd6, 0x07);
	write_cmos_sensor_8_8(0xb1, 0x00);
	write_cmos_sensor_8_8(0xb4, 0x00);
	write_cmos_sensor_8_8(0xb7, 0x0a);
	write_cmos_sensor_8_8(0xbc, 0x44);
	write_cmos_sensor_8_8(0xbf, 0x42);
	write_cmos_sensor_8_8(0xc1, 0x10);
	write_cmos_sensor_8_8(0xc3, 0x24);
	write_cmos_sensor_8_8(0xc8, 0x03);
	write_cmos_sensor_8_8(0xc9, 0xf8);
	write_cmos_sensor_8_8(0xe1, 0x33);
	write_cmos_sensor_8_8(0xe2, 0xbb);
	write_cmos_sensor_8_8(0x51, 0x0c);
	write_cmos_sensor_8_8(0x52, 0x0a);
	write_cmos_sensor_8_8(0x57, 0x8c);
	write_cmos_sensor_8_8(0x59, 0x09);
	write_cmos_sensor_8_8(0x5a, 0x08);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x60, 0x02);
	write_cmos_sensor_8_8(0x6d, 0x5c);
	write_cmos_sensor_8_8(0x76, 0x16);
	write_cmos_sensor_8_8(0x7c, 0x11);
	write_cmos_sensor_8_8(0x90, 0x28);
	write_cmos_sensor_8_8(0x91, 0x16);
	write_cmos_sensor_8_8(0x92, 0x1c);
	write_cmos_sensor_8_8(0x93, 0x24);
	write_cmos_sensor_8_8(0x95, 0x48);
	write_cmos_sensor_8_8(0x9c, 0x06);
	write_cmos_sensor_8_8(0xca, 0x0c);
	write_cmos_sensor_8_8(0xce, 0x0d);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x00);
	write_cmos_sensor_8_8(0xdd, 0x18);
	write_cmos_sensor_8_8(0xde, 0x19);
	write_cmos_sensor_8_8(0xdf, 0x32);
	write_cmos_sensor_8_8(0xe0, 0x70);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc2, 0x05);
	write_cmos_sensor_8_8(0xd7, 0x88);
	write_cmos_sensor_8_8(0xd8, 0x77);
	write_cmos_sensor_8_8(0xd9, 0x66);
	write_cmos_sensor_8_8(0xfd, 0x07);
	write_cmos_sensor_8_8(0x00, 0xf8);
	write_cmos_sensor_8_8(0x01, 0x2b);
	write_cmos_sensor_8_8(0x05, 0x40);
	write_cmos_sensor_8_8(0x08, 0x06);
	write_cmos_sensor_8_8(0x09, 0x11);
	write_cmos_sensor_8_8(0x28, 0x6f);
	write_cmos_sensor_8_8(0x2a, 0x20);
	write_cmos_sensor_8_8(0x2b, 0x05);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x52, 0x00);
	write_cmos_sensor_8_8(0x53, 0x80);
	write_cmos_sensor_8_8(0x54, 0x00);
	write_cmos_sensor_8_8(0x55, 0x80);
	write_cmos_sensor_8_8(0x56, 0x00);
	write_cmos_sensor_8_8(0x57, 0x80);
	write_cmos_sensor_8_8(0x58, 0x00);
	write_cmos_sensor_8_8(0x59, 0x80);
	write_cmos_sensor_8_8(0x5c, 0x3f);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0x9a, 0x30);
	write_cmos_sensor_8_8(0xa8, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0x08);
	write_cmos_sensor_8_8(0xa2, 0x09);
	write_cmos_sensor_8_8(0xa3, 0x90);
	write_cmos_sensor_8_8(0xa4, 0x00);
	write_cmos_sensor_8_8(0xa5, 0x08);
	write_cmos_sensor_8_8(0xa6, 0x0c);
	write_cmos_sensor_8_8(0xa7, 0xc0);
	write_cmos_sensor_8_8(0xfd, 0x05);
	write_cmos_sensor_8_8(0x04, 0x40);
	write_cmos_sensor_8_8(0x07, 0x00);
	write_cmos_sensor_8_8(0x0D, 0x01);
	write_cmos_sensor_8_8(0x0F, 0x01);
	write_cmos_sensor_8_8(0x10, 0x00);
	write_cmos_sensor_8_8(0x11, 0x00);
	write_cmos_sensor_8_8(0x12, 0x0C);
	write_cmos_sensor_8_8(0x13, 0xCF);
	write_cmos_sensor_8_8(0x14, 0x00);
	write_cmos_sensor_8_8(0x15, 0x00);
	write_cmos_sensor_8_8(0x18, 0x00);
	write_cmos_sensor_8_8(0x19, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x24, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x1F);
	write_cmos_sensor_8_8(0xc1, 0x08);
	write_cmos_sensor_8_8(0xc2, 0x30);
	write_cmos_sensor_8_8(0x8e, 0x0c);
	write_cmos_sensor_8_8(0x8f, 0xc0);
	write_cmos_sensor_8_8(0x90, 0x09);
	write_cmos_sensor_8_8(0x91, 0x90);
	write_cmos_sensor_8_8(0xb7, 0x02);

	LOG_INF("end \n");
}	/*	preview_setting  */

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture start \n");
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x20, 0x0e);
	write_cmos_sensor_8_8(0x20, 0x0b);
	write_cmos_sensor_8_8(0x10, 0x05);
	write_cmos_sensor_8_8(0x11, 0x2a);
	write_cmos_sensor_8_8(0x12, 0x03);
	write_cmos_sensor_8_8(0x13, 0x05);
	write_cmos_sensor_8_8(0x14, 0x3e);
	write_cmos_sensor_8_8(0x15, 0x02);
	write_cmos_sensor_8_8(0x16, 0x82);
	write_cmos_sensor_8_8(0x17, 0x05);
	write_cmos_sensor_8_8(0x18, 0x6f);
	write_cmos_sensor_8_8(0x19, 0x04);
	write_cmos_sensor_8_8(0x1a, 0x05);
	write_cmos_sensor_8_8(0x1b, 0xde);
	write_cmos_sensor_8_8(0x1c, 0x09);
	write_cmos_sensor_8_8(0x1d, 0x13);
	write_cmos_sensor_8_8(0x1e, 0x23);
	write_cmos_sensor_8_8(0x1f, 0x0f);
	write_cmos_sensor_8_8(0x20, 0x0f);
	write_cmos_sensor_8_8(0x21, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x12, 0x00);
	write_cmos_sensor_8_8(0x02, 0x00);
	write_cmos_sensor_8_8(0x03, 0x12);
	write_cmos_sensor_8_8(0x04, 0x50);
	write_cmos_sensor_8_8(0x05, 0x00);
	write_cmos_sensor_8_8(0x06, 0xd0);
	write_cmos_sensor_8_8(0x07, 0x05);
	write_cmos_sensor_8_8(0x21, 0x02);
	write_cmos_sensor_8_8(0x24, 0x30);
	write_cmos_sensor_8_8(0x33, 0x03);
	write_cmos_sensor_8_8(0x01, 0x03);
	write_cmos_sensor_8_8(0x19, 0x10);
	write_cmos_sensor_8_8(0x42, 0x55);
	write_cmos_sensor_8_8(0x43, 0x00);
	write_cmos_sensor_8_8(0x47, 0x07);
	write_cmos_sensor_8_8(0x48, 0x08);
	write_cmos_sensor_8_8(0x4c, 0x38);
	write_cmos_sensor_8_8(0xb2, 0x7e);
	write_cmos_sensor_8_8(0xb3, 0x7b);
	write_cmos_sensor_8_8(0xbd, 0x08);
	write_cmos_sensor_8_8(0xd2, 0x47);
	write_cmos_sensor_8_8(0xd3, 0x10);
	write_cmos_sensor_8_8(0xd4, 0x0d);
	write_cmos_sensor_8_8(0xd5, 0x08);
	write_cmos_sensor_8_8(0xd6, 0x07);
	write_cmos_sensor_8_8(0xb1, 0x00);
	write_cmos_sensor_8_8(0xb4, 0x00);
	write_cmos_sensor_8_8(0xb7, 0x0a);
	write_cmos_sensor_8_8(0xbc, 0x44);
	write_cmos_sensor_8_8(0xbf, 0x42);
	write_cmos_sensor_8_8(0xc1, 0x10);
	write_cmos_sensor_8_8(0xc3, 0x24);
	write_cmos_sensor_8_8(0xc8, 0x03);
	write_cmos_sensor_8_8(0xc9, 0xf8);
	write_cmos_sensor_8_8(0xe1, 0x33);
	write_cmos_sensor_8_8(0xe2, 0xbb);
	write_cmos_sensor_8_8(0x51, 0x0c);
	write_cmos_sensor_8_8(0x52, 0x0a);
	write_cmos_sensor_8_8(0x57, 0x8c);
	write_cmos_sensor_8_8(0x59, 0x09);
	write_cmos_sensor_8_8(0x5a, 0x08);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x60, 0x02);
	write_cmos_sensor_8_8(0x6d, 0x5c);
	write_cmos_sensor_8_8(0x76, 0x16);
	write_cmos_sensor_8_8(0x7c, 0x11);
	write_cmos_sensor_8_8(0x90, 0x28);
	write_cmos_sensor_8_8(0x91, 0x16);
	write_cmos_sensor_8_8(0x92, 0x1c);
	write_cmos_sensor_8_8(0x93, 0x24);
	write_cmos_sensor_8_8(0x95, 0x48);
	write_cmos_sensor_8_8(0x9c, 0x06);
	write_cmos_sensor_8_8(0xca, 0x0c);
	write_cmos_sensor_8_8(0xce, 0x0d);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x00);
	write_cmos_sensor_8_8(0xdd, 0x18);
	write_cmos_sensor_8_8(0xde, 0x19);
	write_cmos_sensor_8_8(0xdf, 0x32);
	write_cmos_sensor_8_8(0xe0, 0x70);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc2, 0x05);
	write_cmos_sensor_8_8(0xd7, 0x88);
	write_cmos_sensor_8_8(0xd8, 0x77);
	write_cmos_sensor_8_8(0xd9, 0x66);
	write_cmos_sensor_8_8(0xfd, 0x07);
	write_cmos_sensor_8_8(0x00, 0xf8);
	write_cmos_sensor_8_8(0x01, 0x2b);
	write_cmos_sensor_8_8(0x05, 0x40);
	write_cmos_sensor_8_8(0x08, 0x06);
	write_cmos_sensor_8_8(0x09, 0x11);
	write_cmos_sensor_8_8(0x28, 0x6f);
	write_cmos_sensor_8_8(0x2a, 0x20);
	write_cmos_sensor_8_8(0x2b, 0x05);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x52, 0x00);
	write_cmos_sensor_8_8(0x53, 0x80);
	write_cmos_sensor_8_8(0x54, 0x00);
	write_cmos_sensor_8_8(0x55, 0x80);
	write_cmos_sensor_8_8(0x56, 0x00);
	write_cmos_sensor_8_8(0x57, 0x80);
	write_cmos_sensor_8_8(0x58, 0x00);
	write_cmos_sensor_8_8(0x59, 0x80);
	write_cmos_sensor_8_8(0x5c, 0x3f);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0x9a, 0x30);
	write_cmos_sensor_8_8(0xa8, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0x08);
	write_cmos_sensor_8_8(0xa2, 0x09);
	write_cmos_sensor_8_8(0xa3, 0x90);
	write_cmos_sensor_8_8(0xa4, 0x00);
	write_cmos_sensor_8_8(0xa5, 0x08);
	write_cmos_sensor_8_8(0xa6, 0x0c);
	write_cmos_sensor_8_8(0xa7, 0xc0);
	write_cmos_sensor_8_8(0xfd, 0x05);
	write_cmos_sensor_8_8(0x04, 0x40);
	write_cmos_sensor_8_8(0x07, 0x00);
	write_cmos_sensor_8_8(0x0D, 0x01);
	write_cmos_sensor_8_8(0x0F, 0x01);
	write_cmos_sensor_8_8(0x10, 0x00);
	write_cmos_sensor_8_8(0x11, 0x00);
	write_cmos_sensor_8_8(0x12, 0x0C);
	write_cmos_sensor_8_8(0x13, 0xCF);
	write_cmos_sensor_8_8(0x14, 0x00);
	write_cmos_sensor_8_8(0x15, 0x00);
	write_cmos_sensor_8_8(0x18, 0x00);
	write_cmos_sensor_8_8(0x19, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x24, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x1F);
	write_cmos_sensor_8_8(0xc1, 0x08);
	write_cmos_sensor_8_8(0xc2, 0x30);
	write_cmos_sensor_8_8(0x8e, 0x0c);
	write_cmos_sensor_8_8(0x8f, 0xc0);
	write_cmos_sensor_8_8(0x90, 0x09);
	write_cmos_sensor_8_8(0x91, 0x90);
	write_cmos_sensor_8_8(0xb7, 0x02);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}


static void hs_video_setting(void)
{
	LOG_INF("hs_video start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void slim_video_setting(void)
{
	LOG_INF("slim_video start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void custom1_setting(void)
{
	LOG_INF("custom1 start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void custom2_setting(void)
{
	LOG_INF("custom2 start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void custom3_setting(void)
{
	LOG_INF("custom3 start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void custom4_setting(void)
{
	LOG_INF("custom4 start \n");
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x20,0x0e);
	write_cmos_sensor_8_8(0x20,0x0b);
	write_cmos_sensor_8_8(0x10,0x05);
	write_cmos_sensor_8_8(0x11,0x2a);
	write_cmos_sensor_8_8(0x12,0x03);
	write_cmos_sensor_8_8(0x13,0x05);
	write_cmos_sensor_8_8(0x14,0x3e);
	write_cmos_sensor_8_8(0x15,0x02);
	write_cmos_sensor_8_8(0x16,0x82);
	write_cmos_sensor_8_8(0x17,0x05);
	write_cmos_sensor_8_8(0x18,0x6f);
	write_cmos_sensor_8_8(0x19,0x04);
	write_cmos_sensor_8_8(0x1a,0x05);
	write_cmos_sensor_8_8(0x1b,0xde);
	write_cmos_sensor_8_8(0x1c,0x09);
	write_cmos_sensor_8_8(0x1d,0x13);
	write_cmos_sensor_8_8(0x1e,0x23);
	write_cmos_sensor_8_8(0x1f,0x0f);
	write_cmos_sensor_8_8(0x20,0x0f);
	write_cmos_sensor_8_8(0x21,0x00);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0x12,0x00);
	write_cmos_sensor_8_8(0x02,0x00);
	write_cmos_sensor_8_8(0x03,0x12);
	write_cmos_sensor_8_8(0x04,0x50);
	write_cmos_sensor_8_8(0x05,0x00);
	write_cmos_sensor_8_8(0x06,0xd0);
	write_cmos_sensor_8_8(0x07,0x05);
	write_cmos_sensor_8_8(0x21,0x02);
	write_cmos_sensor_8_8(0x24,0x30);
	write_cmos_sensor_8_8(0x33,0x03);
	write_cmos_sensor_8_8(0x01,0x03);
	write_cmos_sensor_8_8(0x19,0x10);
	write_cmos_sensor_8_8(0x42,0x55);
	write_cmos_sensor_8_8(0x43,0x00);
	write_cmos_sensor_8_8(0x47,0x07);
	write_cmos_sensor_8_8(0x48,0x08);
	write_cmos_sensor_8_8(0x4c,0x38);
	write_cmos_sensor_8_8(0xb2,0x7e);
	write_cmos_sensor_8_8(0xb3,0x7b);
	write_cmos_sensor_8_8(0xbd,0x08);
	write_cmos_sensor_8_8(0xd2,0x47);
	write_cmos_sensor_8_8(0xd3,0x10);
	write_cmos_sensor_8_8(0xd4,0x0d);
	write_cmos_sensor_8_8(0xd5,0x08);
	write_cmos_sensor_8_8(0xd6,0x07);
	write_cmos_sensor_8_8(0xb1,0x00);
	write_cmos_sensor_8_8(0xb4,0x00);
	write_cmos_sensor_8_8(0xb7,0x0a);
	write_cmos_sensor_8_8(0xbc,0x44);
	write_cmos_sensor_8_8(0xbf,0x42);
	write_cmos_sensor_8_8(0xc1,0x10);
	write_cmos_sensor_8_8(0xc3,0x24);
	write_cmos_sensor_8_8(0xc8,0x03);
	write_cmos_sensor_8_8(0xc9,0xf8);
	write_cmos_sensor_8_8(0xe1,0x33);
	write_cmos_sensor_8_8(0xe2,0xbb);
	write_cmos_sensor_8_8(0x51,0x0c);
	write_cmos_sensor_8_8(0x52,0x0a);
	write_cmos_sensor_8_8(0x57,0x8c);
	write_cmos_sensor_8_8(0x59,0x09);
	write_cmos_sensor_8_8(0x5a,0x08);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x60,0x02);
	write_cmos_sensor_8_8(0x6d,0x5c);
	write_cmos_sensor_8_8(0x76,0x16);
	write_cmos_sensor_8_8(0x7c,0x11);
	write_cmos_sensor_8_8(0x90,0x28);
	write_cmos_sensor_8_8(0x91,0x16);
	write_cmos_sensor_8_8(0x92,0x1c);
	write_cmos_sensor_8_8(0x93,0x24);
	write_cmos_sensor_8_8(0x95,0x48);
	write_cmos_sensor_8_8(0x9c,0x06);
	write_cmos_sensor_8_8(0xca,0x0c);
	write_cmos_sensor_8_8(0xce,0x0d);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc0,0x00);
	write_cmos_sensor_8_8(0xdd,0x18);
	write_cmos_sensor_8_8(0xde,0x19);
	write_cmos_sensor_8_8(0xdf,0x32);
	write_cmos_sensor_8_8(0xe0,0x70);
	write_cmos_sensor_8_8(0xfd,0x01);
	write_cmos_sensor_8_8(0xc2,0x05);
	write_cmos_sensor_8_8(0xd7,0x88);
	write_cmos_sensor_8_8(0xd8,0x77);
	write_cmos_sensor_8_8(0xd9,0x66);
	write_cmos_sensor_8_8(0xfd,0x07);
	write_cmos_sensor_8_8(0x00,0xf8);
	write_cmos_sensor_8_8(0x01,0x2b);
	write_cmos_sensor_8_8(0x05,0x40);
	write_cmos_sensor_8_8(0x08,0x06);
	write_cmos_sensor_8_8(0x09,0x11);
	write_cmos_sensor_8_8(0x28,0x6f);
	write_cmos_sensor_8_8(0x2a,0x20);
	write_cmos_sensor_8_8(0x2b,0x05);
	write_cmos_sensor_8_8(0x5e,0x10);
	write_cmos_sensor_8_8(0x52,0x00);
	write_cmos_sensor_8_8(0x53,0x80);
	write_cmos_sensor_8_8(0x54,0x00);
	write_cmos_sensor_8_8(0x55,0x80);
	write_cmos_sensor_8_8(0x56,0x00);
	write_cmos_sensor_8_8(0x57,0x80);
	write_cmos_sensor_8_8(0x58,0x00);
	write_cmos_sensor_8_8(0x59,0x80);
	write_cmos_sensor_8_8(0x5c,0x3f);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0x9a,0x30);
	write_cmos_sensor_8_8(0xa8,0x02);
	write_cmos_sensor_8_8(0xfd,0x02);
	write_cmos_sensor_8_8(0xa0,0x01);
	write_cmos_sensor_8_8(0xa1,0x3a);
	write_cmos_sensor_8_8(0xa2,0x07);
	write_cmos_sensor_8_8(0xa3,0x2c);
	write_cmos_sensor_8_8(0xa4,0x00);
	write_cmos_sensor_8_8(0xa5,0x08);
	write_cmos_sensor_8_8(0xa6,0x0c);
	write_cmos_sensor_8_8(0xa7,0xc0);
	write_cmos_sensor_8_8(0xfd,0x05);
	write_cmos_sensor_8_8(0x04,0x40);
	write_cmos_sensor_8_8(0x07,0x00);
	write_cmos_sensor_8_8(0x0D,0x01);
	write_cmos_sensor_8_8(0x0F,0x01);
	write_cmos_sensor_8_8(0x10,0x00);
	write_cmos_sensor_8_8(0x11,0x00);
	write_cmos_sensor_8_8(0x12,0x0C);
	write_cmos_sensor_8_8(0x13,0xCF);
	write_cmos_sensor_8_8(0x14,0x00);
	write_cmos_sensor_8_8(0x15,0x00);
	write_cmos_sensor_8_8(0x18,0x00);
	write_cmos_sensor_8_8(0x19,0x00);
	write_cmos_sensor_8_8(0xfd,0x00);
	write_cmos_sensor_8_8(0x24,0x01);
	write_cmos_sensor_8_8(0xc0,0x1F);
	write_cmos_sensor_8_8(0xc1,0x08);
	write_cmos_sensor_8_8(0xc2,0x30);
	write_cmos_sensor_8_8(0x8e,0x0c);
	write_cmos_sensor_8_8(0x8f,0xc0);
	write_cmos_sensor_8_8(0x90,0x07);
	write_cmos_sensor_8_8(0x91,0x2c);
	write_cmos_sensor_8_8(0xb7,0x02);
	LOG_INF("end \n");
}

static void custom5_setting(void)
{
	LOG_INF("custom5 start \n");
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x20, 0x0e);
	write_cmos_sensor_8_8(0x20, 0x0b);
	write_cmos_sensor_8_8(0x10, 0x05);
	write_cmos_sensor_8_8(0x11, 0x2a);
	write_cmos_sensor_8_8(0x12, 0x03);
	write_cmos_sensor_8_8(0x13, 0x05);
	write_cmos_sensor_8_8(0x14, 0x3e);
	write_cmos_sensor_8_8(0x15, 0x02);
	write_cmos_sensor_8_8(0x16, 0x82);
	write_cmos_sensor_8_8(0x17, 0x05);
	write_cmos_sensor_8_8(0x18, 0x6f);
	write_cmos_sensor_8_8(0x19, 0x04);
	write_cmos_sensor_8_8(0x1a, 0x05);
	write_cmos_sensor_8_8(0x1b, 0xde);
	write_cmos_sensor_8_8(0x1c, 0x09);
	write_cmos_sensor_8_8(0x1d, 0x13);
	write_cmos_sensor_8_8(0x1e, 0x23);
	write_cmos_sensor_8_8(0x1f, 0x0f);
	write_cmos_sensor_8_8(0x20, 0x0f);
	write_cmos_sensor_8_8(0x21, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x12, 0x00);
	write_cmos_sensor_8_8(0x02, 0x00);
	write_cmos_sensor_8_8(0x03, 0x12);
	write_cmos_sensor_8_8(0x04, 0x50);
	write_cmos_sensor_8_8(0x05, 0x00);
	write_cmos_sensor_8_8(0x06, 0xd0);
	write_cmos_sensor_8_8(0x07, 0x05);
	write_cmos_sensor_8_8(0x21, 0x02);
	write_cmos_sensor_8_8(0x24, 0x30);
	write_cmos_sensor_8_8(0x33, 0x03);
	write_cmos_sensor_8_8(0x01, 0x03);
	write_cmos_sensor_8_8(0x19, 0x10);
	write_cmos_sensor_8_8(0x42, 0x55);
	write_cmos_sensor_8_8(0x43, 0x00);
	write_cmos_sensor_8_8(0x47, 0x07);
	write_cmos_sensor_8_8(0x48, 0x08);
	write_cmos_sensor_8_8(0x4c, 0x38);
	write_cmos_sensor_8_8(0xb2, 0x7e);
	write_cmos_sensor_8_8(0xb3, 0x7b);
	write_cmos_sensor_8_8(0xbd, 0x08);
	write_cmos_sensor_8_8(0xd2, 0x47);
	write_cmos_sensor_8_8(0xd3, 0x10);
	write_cmos_sensor_8_8(0xd4, 0x0d);
	write_cmos_sensor_8_8(0xd5, 0x08);
	write_cmos_sensor_8_8(0xd6, 0x07);
	write_cmos_sensor_8_8(0xb1, 0x00);
	write_cmos_sensor_8_8(0xb4, 0x00);
	write_cmos_sensor_8_8(0xb7, 0x0a);
	write_cmos_sensor_8_8(0xbc, 0x44);
	write_cmos_sensor_8_8(0xbf, 0x42);
	write_cmos_sensor_8_8(0xc1, 0x10);
	write_cmos_sensor_8_8(0xc3, 0x24);
	write_cmos_sensor_8_8(0xc8, 0x03);
	write_cmos_sensor_8_8(0xc9, 0xf8);
	write_cmos_sensor_8_8(0xe1, 0x33);
	write_cmos_sensor_8_8(0xe2, 0xbb);
	write_cmos_sensor_8_8(0x51, 0x0c);
	write_cmos_sensor_8_8(0x52, 0x0a);
	write_cmos_sensor_8_8(0x57, 0x8c);
	write_cmos_sensor_8_8(0x59, 0x09);
	write_cmos_sensor_8_8(0x5a, 0x08);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x60, 0x02);
	write_cmos_sensor_8_8(0x6d, 0x5c);
	write_cmos_sensor_8_8(0x76, 0x16);
	write_cmos_sensor_8_8(0x7c, 0x11);
	write_cmos_sensor_8_8(0x90, 0x28);
	write_cmos_sensor_8_8(0x91, 0x16);
	write_cmos_sensor_8_8(0x92, 0x1c);
	write_cmos_sensor_8_8(0x93, 0x24);
	write_cmos_sensor_8_8(0x95, 0x48);
	write_cmos_sensor_8_8(0x9c, 0x06);
	write_cmos_sensor_8_8(0xca, 0x0c);
	write_cmos_sensor_8_8(0xce, 0x0d);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x00);
	write_cmos_sensor_8_8(0xdd, 0x18);
	write_cmos_sensor_8_8(0xde, 0x19);
	write_cmos_sensor_8_8(0xdf, 0x32);
	write_cmos_sensor_8_8(0xe0, 0x70);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc2, 0x05);
	write_cmos_sensor_8_8(0xd7, 0x88);
	write_cmos_sensor_8_8(0xd8, 0x77);
	write_cmos_sensor_8_8(0xd9, 0x66);
	write_cmos_sensor_8_8(0xfd, 0x07);
	write_cmos_sensor_8_8(0x00, 0xf8);
	write_cmos_sensor_8_8(0x01, 0x2b);
	write_cmos_sensor_8_8(0x05, 0x40);
	write_cmos_sensor_8_8(0x08, 0x06);
	write_cmos_sensor_8_8(0x09, 0x11);
	write_cmos_sensor_8_8(0x28, 0x6f);
	write_cmos_sensor_8_8(0x2a, 0x20);
	write_cmos_sensor_8_8(0x2b, 0x05);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x52, 0x00);
	write_cmos_sensor_8_8(0x53, 0x80);
	write_cmos_sensor_8_8(0x54, 0x00);
	write_cmos_sensor_8_8(0x55, 0x80);
	write_cmos_sensor_8_8(0x56, 0x00);
	write_cmos_sensor_8_8(0x57, 0x80);
	write_cmos_sensor_8_8(0x58, 0x00);
	write_cmos_sensor_8_8(0x59, 0x80);
	write_cmos_sensor_8_8(0x5c, 0x3f);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0x9a, 0x30);
	write_cmos_sensor_8_8(0xa8, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0x08);
	write_cmos_sensor_8_8(0xa2, 0x09);
	write_cmos_sensor_8_8(0xa3, 0x90);
	write_cmos_sensor_8_8(0xa4, 0x00);
	write_cmos_sensor_8_8(0xa5, 0x08);
	write_cmos_sensor_8_8(0xa6, 0x0c);
	write_cmos_sensor_8_8(0xa7, 0xc0);
	write_cmos_sensor_8_8(0xfd, 0x05);
	write_cmos_sensor_8_8(0x04, 0x40);
	write_cmos_sensor_8_8(0x07, 0x00);
	write_cmos_sensor_8_8(0x0D, 0x01);
	write_cmos_sensor_8_8(0x0F, 0x01);
	write_cmos_sensor_8_8(0x10, 0x00);
	write_cmos_sensor_8_8(0x11, 0x00);
	write_cmos_sensor_8_8(0x12, 0x0C);
	write_cmos_sensor_8_8(0x13, 0xCF);
	write_cmos_sensor_8_8(0x14, 0x00);
	write_cmos_sensor_8_8(0x15, 0x00);
	write_cmos_sensor_8_8(0x18, 0x00);
	write_cmos_sensor_8_8(0x19, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x24, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x1F);
	write_cmos_sensor_8_8(0xc1, 0x08);
	write_cmos_sensor_8_8(0xc2, 0x30);
	write_cmos_sensor_8_8(0x8e, 0x0c);
	write_cmos_sensor_8_8(0x8f, 0xc0);
	write_cmos_sensor_8_8(0x90, 0x09);
	write_cmos_sensor_8_8(0x91, 0x90);
	write_cmos_sensor_8_8(0xb7, 0x02);
	LOG_INF("end \n");
}

static void custom6_setting(void)
{
	LOG_INF("custom6 start \n");
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x20, 0x0e);
	write_cmos_sensor_8_8(0x20, 0x0b);
	write_cmos_sensor_8_8(0x10, 0x05);
	write_cmos_sensor_8_8(0x11, 0x2a);
	write_cmos_sensor_8_8(0x12, 0x03);
	write_cmos_sensor_8_8(0x13, 0x05);
	write_cmos_sensor_8_8(0x14, 0x3e);
	write_cmos_sensor_8_8(0x15, 0x02);
	write_cmos_sensor_8_8(0x16, 0x82);
	write_cmos_sensor_8_8(0x17, 0x05);
	write_cmos_sensor_8_8(0x18, 0x6f);
	write_cmos_sensor_8_8(0x19, 0x04);
	write_cmos_sensor_8_8(0x1a, 0x05);
	write_cmos_sensor_8_8(0x1b, 0xde);
	write_cmos_sensor_8_8(0x1c, 0x09);
	write_cmos_sensor_8_8(0x1d, 0x13);
	write_cmos_sensor_8_8(0x1e, 0x23);
	write_cmos_sensor_8_8(0x1f, 0x0f);
	write_cmos_sensor_8_8(0x20, 0x0f);
	write_cmos_sensor_8_8(0x21, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x12, 0x00);
	write_cmos_sensor_8_8(0x02, 0x00);
	write_cmos_sensor_8_8(0x03, 0x12);
	write_cmos_sensor_8_8(0x04, 0x50);
	write_cmos_sensor_8_8(0x05, 0x00);
	write_cmos_sensor_8_8(0x06, 0xd0);
	write_cmos_sensor_8_8(0x07, 0x05);
	write_cmos_sensor_8_8(0x21, 0x02);
	write_cmos_sensor_8_8(0x24, 0x30);
	write_cmos_sensor_8_8(0x33, 0x03);
	write_cmos_sensor_8_8(0x01, 0x03);
	write_cmos_sensor_8_8(0x19, 0x10);
	write_cmos_sensor_8_8(0x42, 0x55);
	write_cmos_sensor_8_8(0x43, 0x00);
	write_cmos_sensor_8_8(0x47, 0x07);
	write_cmos_sensor_8_8(0x48, 0x08);
	write_cmos_sensor_8_8(0x4c, 0x38);
	write_cmos_sensor_8_8(0xb2, 0x7e);
	write_cmos_sensor_8_8(0xb3, 0x7b);
	write_cmos_sensor_8_8(0xbd, 0x08);
	write_cmos_sensor_8_8(0xd2, 0x47);
	write_cmos_sensor_8_8(0xd3, 0x10);
	write_cmos_sensor_8_8(0xd4, 0x0d);
	write_cmos_sensor_8_8(0xd5, 0x08);
	write_cmos_sensor_8_8(0xd6, 0x07);
	write_cmos_sensor_8_8(0xb1, 0x00);
	write_cmos_sensor_8_8(0xb4, 0x00);
	write_cmos_sensor_8_8(0xb7, 0x0a);
	write_cmos_sensor_8_8(0xbc, 0x44);
	write_cmos_sensor_8_8(0xbf, 0x42);
	write_cmos_sensor_8_8(0xc1, 0x10);
	write_cmos_sensor_8_8(0xc3, 0x24);
	write_cmos_sensor_8_8(0xc8, 0x03);
	write_cmos_sensor_8_8(0xc9, 0xf8);
	write_cmos_sensor_8_8(0xe1, 0x33);
	write_cmos_sensor_8_8(0xe2, 0xbb);
	write_cmos_sensor_8_8(0x51, 0x0c);
	write_cmos_sensor_8_8(0x52, 0x0a);
	write_cmos_sensor_8_8(0x57, 0x8c);
	write_cmos_sensor_8_8(0x59, 0x09);
	write_cmos_sensor_8_8(0x5a, 0x08);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x60, 0x02);
	write_cmos_sensor_8_8(0x6d, 0x5c);
	write_cmos_sensor_8_8(0x76, 0x16);
	write_cmos_sensor_8_8(0x7c, 0x11);
	write_cmos_sensor_8_8(0x90, 0x28);
	write_cmos_sensor_8_8(0x91, 0x16);
	write_cmos_sensor_8_8(0x92, 0x1c);
	write_cmos_sensor_8_8(0x93, 0x24);
	write_cmos_sensor_8_8(0x95, 0x48);
	write_cmos_sensor_8_8(0x9c, 0x06);
	write_cmos_sensor_8_8(0xca, 0x0c);
	write_cmos_sensor_8_8(0xce, 0x0d);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x00);
	write_cmos_sensor_8_8(0xdd, 0x18);
	write_cmos_sensor_8_8(0xde, 0x19);
	write_cmos_sensor_8_8(0xdf, 0x32);
	write_cmos_sensor_8_8(0xe0, 0x70);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc2, 0x05);
	write_cmos_sensor_8_8(0xd7, 0x88);
	write_cmos_sensor_8_8(0xd8, 0x77);
	write_cmos_sensor_8_8(0xd9, 0x66);
	write_cmos_sensor_8_8(0xfd, 0x07);
	write_cmos_sensor_8_8(0x00, 0xf8);
	write_cmos_sensor_8_8(0x01, 0x2b);
	write_cmos_sensor_8_8(0x05, 0x40);
	write_cmos_sensor_8_8(0x08, 0x06);
	write_cmos_sensor_8_8(0x09, 0x11);
	write_cmos_sensor_8_8(0x28, 0x6f);
	write_cmos_sensor_8_8(0x2a, 0x20);
	write_cmos_sensor_8_8(0x2b, 0x05);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x52, 0x00);
	write_cmos_sensor_8_8(0x53, 0x80);
	write_cmos_sensor_8_8(0x54, 0x00);
	write_cmos_sensor_8_8(0x55, 0x80);
	write_cmos_sensor_8_8(0x56, 0x00);
	write_cmos_sensor_8_8(0x57, 0x80);
	write_cmos_sensor_8_8(0x58, 0x00);
	write_cmos_sensor_8_8(0x59, 0x80);
	write_cmos_sensor_8_8(0x5c, 0x3f);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0x9a, 0x30);
	write_cmos_sensor_8_8(0xa8, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0x08);
	write_cmos_sensor_8_8(0xa2, 0x09);
	write_cmos_sensor_8_8(0xa3, 0x90);
	write_cmos_sensor_8_8(0xa4, 0x00);
	write_cmos_sensor_8_8(0xa5, 0x08);
	write_cmos_sensor_8_8(0xa6, 0x0c);
	write_cmos_sensor_8_8(0xa7, 0xc0);
	write_cmos_sensor_8_8(0xfd, 0x05);
	write_cmos_sensor_8_8(0x04, 0x40);
	write_cmos_sensor_8_8(0x07, 0x00);
	write_cmos_sensor_8_8(0x0D, 0x01);
	write_cmos_sensor_8_8(0x0F, 0x01);
	write_cmos_sensor_8_8(0x10, 0x00);
	write_cmos_sensor_8_8(0x11, 0x00);
	write_cmos_sensor_8_8(0x12, 0x0C);
	write_cmos_sensor_8_8(0x13, 0xCF);
	write_cmos_sensor_8_8(0x14, 0x00);
	write_cmos_sensor_8_8(0x15, 0x00);
	write_cmos_sensor_8_8(0x18, 0x00);
	write_cmos_sensor_8_8(0x19, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x24, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x1F);
	write_cmos_sensor_8_8(0xc1, 0x08);
	write_cmos_sensor_8_8(0xc2, 0x30);
	write_cmos_sensor_8_8(0x8e, 0x0c);
	write_cmos_sensor_8_8(0x8f, 0xc0);
	write_cmos_sensor_8_8(0x90, 0x09);
	write_cmos_sensor_8_8(0x91, 0x90);
	write_cmos_sensor_8_8(0xb7, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0xe0);
	write_cmos_sensor_8_8(0xa2, 0x07);
	write_cmos_sensor_8_8(0xa3, 0xe0);
	write_cmos_sensor_8_8(0xa4, 0x01);
	write_cmos_sensor_8_8(0xa5, 0x28);
	write_cmos_sensor_8_8(0xa6, 0x0a);
	write_cmos_sensor_8_8(0xa7, 0x80);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x8e, 0x0a);
	write_cmos_sensor_8_8(0x8f, 0x80);
	write_cmos_sensor_8_8(0x90, 0x07);
	write_cmos_sensor_8_8(0x91, 0xe0);
	LOG_INF("end \n");
}

static void custom7_setting(void)
{
	LOG_INF("custom7 start \n");
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x20, 0x0e);
	write_cmos_sensor_8_8(0x20, 0x0b);
	write_cmos_sensor_8_8(0x10, 0x05);
	write_cmos_sensor_8_8(0x11, 0x2a);
	write_cmos_sensor_8_8(0x12, 0x03);
	write_cmos_sensor_8_8(0x13, 0x05);
	write_cmos_sensor_8_8(0x14, 0x3e);
	write_cmos_sensor_8_8(0x15, 0x02);
	write_cmos_sensor_8_8(0x16, 0x82);
	write_cmos_sensor_8_8(0x17, 0x05);
	write_cmos_sensor_8_8(0x18, 0x6f);
	write_cmos_sensor_8_8(0x19, 0x04);
	write_cmos_sensor_8_8(0x1a, 0x05);
	write_cmos_sensor_8_8(0x1b, 0xde);
	write_cmos_sensor_8_8(0x1c, 0x09);
	write_cmos_sensor_8_8(0x1d, 0x13);
	write_cmos_sensor_8_8(0x1e, 0x23);
	write_cmos_sensor_8_8(0x1f, 0x0f);
	write_cmos_sensor_8_8(0x20, 0x0f);
	write_cmos_sensor_8_8(0x21, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0x12, 0x00);
	write_cmos_sensor_8_8(0x02, 0x00);
	write_cmos_sensor_8_8(0x03, 0x12);
	write_cmos_sensor_8_8(0x04, 0x50);
	write_cmos_sensor_8_8(0x05, 0x00);
	write_cmos_sensor_8_8(0x06, 0xd0);
	write_cmos_sensor_8_8(0x07, 0x05);
	write_cmos_sensor_8_8(0x21, 0x02);
	write_cmos_sensor_8_8(0x24, 0x30);
	write_cmos_sensor_8_8(0x33, 0x03);
	write_cmos_sensor_8_8(0x01, 0x03);
	write_cmos_sensor_8_8(0x19, 0x10);
	write_cmos_sensor_8_8(0x42, 0x55);
	write_cmos_sensor_8_8(0x43, 0x00);
	write_cmos_sensor_8_8(0x47, 0x07);
	write_cmos_sensor_8_8(0x48, 0x08);
	write_cmos_sensor_8_8(0x4c, 0x38);
	write_cmos_sensor_8_8(0xb2, 0x7e);
	write_cmos_sensor_8_8(0xb3, 0x7b);
	write_cmos_sensor_8_8(0xbd, 0x08);
	write_cmos_sensor_8_8(0xd2, 0x47);
	write_cmos_sensor_8_8(0xd3, 0x10);
	write_cmos_sensor_8_8(0xd4, 0x0d);
	write_cmos_sensor_8_8(0xd5, 0x08);
	write_cmos_sensor_8_8(0xd6, 0x07);
	write_cmos_sensor_8_8(0xb1, 0x00);
	write_cmos_sensor_8_8(0xb4, 0x00);
	write_cmos_sensor_8_8(0xb7, 0x0a);
	write_cmos_sensor_8_8(0xbc, 0x44);
	write_cmos_sensor_8_8(0xbf, 0x42);
	write_cmos_sensor_8_8(0xc1, 0x10);
	write_cmos_sensor_8_8(0xc3, 0x24);
	write_cmos_sensor_8_8(0xc8, 0x03);
	write_cmos_sensor_8_8(0xc9, 0xf8);
	write_cmos_sensor_8_8(0xe1, 0x33);
	write_cmos_sensor_8_8(0xe2, 0xbb);
	write_cmos_sensor_8_8(0x51, 0x0c);
	write_cmos_sensor_8_8(0x52, 0x0a);
	write_cmos_sensor_8_8(0x57, 0x8c);
	write_cmos_sensor_8_8(0x59, 0x09);
	write_cmos_sensor_8_8(0x5a, 0x08);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x60, 0x02);
	write_cmos_sensor_8_8(0x6d, 0x5c);
	write_cmos_sensor_8_8(0x76, 0x16);
	write_cmos_sensor_8_8(0x7c, 0x11);
	write_cmos_sensor_8_8(0x90, 0x28);
	write_cmos_sensor_8_8(0x91, 0x16);
	write_cmos_sensor_8_8(0x92, 0x1c);
	write_cmos_sensor_8_8(0x93, 0x24);
	write_cmos_sensor_8_8(0x95, 0x48);
	write_cmos_sensor_8_8(0x9c, 0x06);
	write_cmos_sensor_8_8(0xca, 0x0c);
	write_cmos_sensor_8_8(0xce, 0x0d);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x00);
	write_cmos_sensor_8_8(0xdd, 0x18);
	write_cmos_sensor_8_8(0xde, 0x19);
	write_cmos_sensor_8_8(0xdf, 0x32);
	write_cmos_sensor_8_8(0xe0, 0x70);
	write_cmos_sensor_8_8(0xfd, 0x01);
	write_cmos_sensor_8_8(0xc2, 0x05);
	write_cmos_sensor_8_8(0xd7, 0x88);
	write_cmos_sensor_8_8(0xd8, 0x77);
	write_cmos_sensor_8_8(0xd9, 0x66);
	write_cmos_sensor_8_8(0xfd, 0x07);
	write_cmos_sensor_8_8(0x00, 0xf8);
	write_cmos_sensor_8_8(0x01, 0x2b);
	write_cmos_sensor_8_8(0x05, 0x40);
	write_cmos_sensor_8_8(0x08, 0x06);
	write_cmos_sensor_8_8(0x09, 0x11);
	write_cmos_sensor_8_8(0x28, 0x6f);
	write_cmos_sensor_8_8(0x2a, 0x20);
	write_cmos_sensor_8_8(0x2b, 0x05);
	write_cmos_sensor_8_8(0x5e, 0x10);
	write_cmos_sensor_8_8(0x52, 0x00);
	write_cmos_sensor_8_8(0x53, 0x80);
	write_cmos_sensor_8_8(0x54, 0x00);
	write_cmos_sensor_8_8(0x55, 0x80);
	write_cmos_sensor_8_8(0x56, 0x00);
	write_cmos_sensor_8_8(0x57, 0x80);
	write_cmos_sensor_8_8(0x58, 0x00);
	write_cmos_sensor_8_8(0x59, 0x80);
	write_cmos_sensor_8_8(0x5c, 0x3f);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0x9a, 0x30);
	write_cmos_sensor_8_8(0xa8, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x00);
	write_cmos_sensor_8_8(0xa1, 0x08);
	write_cmos_sensor_8_8(0xa2, 0x09);
	write_cmos_sensor_8_8(0xa3, 0x90);
	write_cmos_sensor_8_8(0xa4, 0x00);
	write_cmos_sensor_8_8(0xa5, 0x08);
	write_cmos_sensor_8_8(0xa6, 0x0c);
	write_cmos_sensor_8_8(0xa7, 0xc0);
	write_cmos_sensor_8_8(0xfd, 0x05);
	write_cmos_sensor_8_8(0x04, 0x40);
	write_cmos_sensor_8_8(0x07, 0x00);
	write_cmos_sensor_8_8(0x0D, 0x01);
	write_cmos_sensor_8_8(0x0F, 0x01);
	write_cmos_sensor_8_8(0x10, 0x00);
	write_cmos_sensor_8_8(0x11, 0x00);
	write_cmos_sensor_8_8(0x12, 0x0C);
	write_cmos_sensor_8_8(0x13, 0xCF);
	write_cmos_sensor_8_8(0x14, 0x00);
	write_cmos_sensor_8_8(0x15, 0x00);
	write_cmos_sensor_8_8(0x18, 0x00);
	write_cmos_sensor_8_8(0x19, 0x00);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x24, 0x01);
	write_cmos_sensor_8_8(0xc0, 0x1F);
	write_cmos_sensor_8_8(0xc1, 0x08);
	write_cmos_sensor_8_8(0xc2, 0x30);
	write_cmos_sensor_8_8(0x8e, 0x0c);
	write_cmos_sensor_8_8(0x8f, 0xc0);
	write_cmos_sensor_8_8(0x90, 0x09);
	write_cmos_sensor_8_8(0x91, 0x90);
	write_cmos_sensor_8_8(0xb7, 0x02);
	write_cmos_sensor_8_8(0xfd, 0x02);
	write_cmos_sensor_8_8(0xa0, 0x02);
	write_cmos_sensor_8_8(0xa1, 0xd8);
	write_cmos_sensor_8_8(0xa2, 0x03);
	write_cmos_sensor_8_8(0xa3, 0xf0);
	write_cmos_sensor_8_8(0xa4, 0x03);
	write_cmos_sensor_8_8(0xa5, 0xc8);
	write_cmos_sensor_8_8(0xa6, 0x05);
	write_cmos_sensor_8_8(0xa7, 0x40);
	write_cmos_sensor_8_8(0xfd, 0x00);
	write_cmos_sensor_8_8(0x8e, 0x05);
	write_cmos_sensor_8_8(0x8f, 0x40);
	write_cmos_sensor_8_8(0x90, 0x03);
	write_cmos_sensor_8_8(0x91, 0xf0);
	LOG_INF("end \n");
}

static kal_uint32 return_sensor_id()
{
	return ((read_cmos_sensor_8_8(0x00) << 16)
	| (read_cmos_sensor_8_8(0x01) << 8)
	| (read_cmos_sensor_8_8(0x02)));
}

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

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				pr_debug("start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = vivo_main2_otp_read_ov08d10pd2282();
				pr_debug("read eeprom ---vivo_otp_read_when_power_on = %d,MAIN2_885A_OTP_ERROR_CODE=%d\n", vivo_otp_read_when_power_on, MAIN2_OTP_ERROR_CODE_OV08D10PD2282);
				return ERROR_NONE;
			}
			pr_info("Read sensor id fail,write_id:0x%x, id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
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
	kal_uint32 sensor_id = 0;

	LOG_INF("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				pr_info("i2c write id: 0x%x, sensor id: 0x%x\n",
					imgsensor.i2c_write_id, sensor_id);
				break;
			}

			pr_info("Read sensor id fail, write: 0x%x, sensor: 0x%x\n",
			    imgsensor.i2c_write_id, sensor_id);

			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */
	sensor_init();
	mdelay(10);
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
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
	LOG_INF("E\n");

	// if (is_standby_flag == 1) {
	// 	LOG_INF("is_standby_flag: %d\n", is_standby_flag);
	// 	is_standby_flag = 0;
	// }
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
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);
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
	LOG_INF("%s E\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
	LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps);
	mdelay(10);

	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}				/* capture() */

static kal_uint32 normal_video(
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s E\n", __func__);

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
	LOG_INF("%s E\n", __func__);

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
	LOG_INF("%s E\n", __func__);

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
	mdelay(10);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 Custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

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
	LOG_INF("E\n");

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
	LOG_INF("E\n");

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
	 LOG_INF("E\n");

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
	pr_debug("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM5;
	imgsensor.pclk = imgsensor_info.custom5.pclk;
	imgsensor.line_length = imgsensor_info.custom5.linelength;
	imgsensor.frame_length = imgsensor_info.custom5.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom5.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	custom5_setting();/*using preview setting*/
	set_mirror_flip(imgsensor.mirror);

	return ERROR_NONE;
}	/*	custom5   */
static kal_uint32 Custom6(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
}	/*	custom6   */
static kal_uint32 Custom7(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	pr_debug("E\n");

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
}	/*	custom7   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("%s E\n", __func__);

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
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;
       sensor_info->Custom4DelayFrame = imgsensor_info.custom4_delay_frame;
	sensor_info->Custom5DelayFrame = imgsensor_info.custom5_delay_frame;
	sensor_info->Custom6DelayFrame = imgsensor_info.custom6_delay_frame;
	sensor_info->Custom7DelayFrame = imgsensor_info.custom7_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0; /* not use */
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
	sensor_info->PDAF_Support = 0;
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
	default:
		LOG_INF("Error ScenarioId setting");
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
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
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

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

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
			LOG_INF("frame_length %d < shutter %d",
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
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		break;

	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
		LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
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
			LOG_INF("frame_length %d < shutter %d",
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
			LOG_INF("frame_length %d < shutter %d",
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
			LOG_INF("frame_length %d < shutter %d",
				imgsensor.frame_length, imgsensor.shutter);
		}
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	pr_debug("enable: %d\n", enable);

	if (enable) {
		write_cmos_sensor_16_8(0x5000, 0x57);
		write_cmos_sensor_16_8(0x5001, 0x02);
		write_cmos_sensor_16_8(0x5e00, 0x80);
	} else {
		// write_cmos_sensor_16_8(0x5000, 0x77);
		// write_cmos_sensor_16_8(0x5001, 0x0a);
		// write_cmos_sensor_16_8(0x5e00, 0x00);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	//int i = 0;
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/*struct SET_PD_BLOCK_INFO_T *PDAFinfo;*/
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    	/*struct SENSOR_VC_INFO_STRUCT *pvcinfo;*/
		struct SENSOR_RAWINFO_STRUCT *rawinfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*PK_DBG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 0;
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
		LOG_INF(
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
		LOG_INF("SENSOR_FEATURE_SET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		write_cmos_sensor_16_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_8(sensor_reg_data->RegAddr);
		LOG_INF("SENSOR_FEATURE_GET_REGISTER sensor_reg_data->RegAddr = 0x%x, sensor_reg_data->RegData = 0x%x\n", sensor_reg_data->RegAddr, sensor_reg_data->RegData);
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
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) (*feature_data));
		break;

	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {	/*2sum = 2; 4sum = 4; 4avg = 1 not 4cell sensor is 4avg*/

		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM3:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
		case MSDK_SCENARIO_ID_CUSTOM6:
		case MSDK_SCENARIO_ID_CUSTOM7:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
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
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
		case SENSOR_FEATURE_GET_RAW_INFO:
		pr_debug("SENSOR_FEATURE_GET_RAW_INFO scenarioId:%d\n",
			(UINT32) *feature_data);
		rawinfo = (struct SENSOR_RAWINFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		memcpy((void *)rawinfo,
				(void *)&imgsensor_raw_info,
				sizeof(struct SENSOR_RAWINFO_STRUCT));
		break;

#if 1
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  MAIN2_885A_OTP_ERROR_CODE:%d \n", *feature_data,MAIN2_OTP_ERROR_CODE_OV08D10PD2282);
		switch (*feature_data) {
			case 0:    //info type: otp state
			LOG_INF("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = MAIN2_OTP_ERROR_CODE_OV08D10PD2282;//otp_state
				memcpy( feature_data+2, sn_inf_main2_ov08d10pd2282, sizeof(MUINT32)*13);
				memcpy( feature_data+10, material_inf_main2_ov08d10pd2282, sizeof(MUINT32)*4);
				memcpy( feature_data+15, moduleid_inf_main2_ov08d10pd2282, sizeof(MUINT32)*2);
				#if 0
					for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_main2_ov08d10pd2282[%d]= 0x%x\n", i, sn_inf_main2_ov08d10pd2282[i]);
					}
				#endif
			}
				break;
			}
			break;
#endif
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data), (UINT16) (*(feature_data + 1)), (BOOL) (*(feature_data + 2)));
		break;

	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length() support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
	    LOG_INF("SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE\n");
		memcpy(feature_return_para_32,
		&imgsensor.ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		LOG_INF("SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE\n");
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
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
};

UINT32 OV08D10PD2282_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	spin_lock_init(&imgsensor_drv_lock);
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
