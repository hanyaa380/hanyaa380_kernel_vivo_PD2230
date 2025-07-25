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
 *	 s5k3p9spmipiraw_Sensor.c
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


#define PFX "SUB[310A]_camera_sensor"
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
#include "../imgsensor_i2c.h"
#include "s5k3p9sp04pd2279fmipiraw_Sensor.h"




#define LOG_INF(format, args...) pr_debug(PFX "[%s] " format, __func__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5K3P9SP04PD2279F_SENSOR_ID,
	.checksum_value = 0x67b95889,

	.pre = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
		},
	.normal_video = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1296,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 384800000, /////397800000
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 560000000,
		.linelength = 5088,
		.framelength = 917,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 1200,  /*1200 */
	},
	.slim_video = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2320,
		.grabwindow_height = 1744,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.custom1 = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
	    },
	.custom2 = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2320,
		.grabwindow_height = 1308,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
     	},
	.custom3 = {/*sw remosaic*/
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3636,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4608,
		.grabwindow_height = 3456,
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.mipi_pixel_rate = 572000000,
		.max_framerate = 300,
	},
	.custom4 = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
	},
	.custom5 = {
		.pclk = 555286000,
		.linelength = 5088,
		.framelength = 3638,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2304,
		.grabwindow_height = 1728,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 397800000,
		.max_framerate = 300,
	},
		.margin = 3,
		.min_shutter = 3,
		.min_gain = 64,
		.max_gain = 1024,
		.min_gain_iso = 100,
		.gain_step = 2,
		.gain_type = 2,
		.max_frame_length = 0xffff,
		.ae_shut_delay_frame = 0,
		.ae_sensor_gain_delay_frame = 0,
		.ae_ispGain_delay_frame = 2,
		.frame_time_delay_frame = 2,	/*sony sensor must be 3,non-sony sensor must be 2, The delay frame of setting frame length  */
		.ihdr_support = 0,	  /*1, support; 0,not support*/
		.ihdr_le_firstline = 0,  /*1,le first ; 0, se first*/
		.sensor_mode_num = 10,

		.cap_delay_frame = 2,/*3 guanjd modify for cts*/
		.pre_delay_frame = 2,/*3 guanjd modify for cts*/
		.video_delay_frame = 3,
		.hs_video_delay_frame = 3,
		.slim_video_delay_frame = 3,
		.custom1_delay_frame = 2,
		.custom2_delay_frame = 2,
		.custom3_delay_frame = 2,
		.custom4_delay_frame = 2,
		.custom5_delay_frame = 2,

		.isp_driving_current = ISP_DRIVING_6MA,
		.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
		.mipi_sensor_type = MIPI_OPHY_NCSI2, /*0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2*/
		.mipi_settle_delay_mode = 1, /*0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL*/
		.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gb, /*SENSOR_OUTPUT_FORMAT_RAW_Gr*/
		.mclk = 26,
		.mipi_lane_num = SENSOR_MIPI_4_LANE,
		.i2c_addr_table = {0x20, 0xff},
		.i2c_speed = 1000,
};

static struct  SENSOR_RAWINFO_STRUCT imgsensor_raw_info = {
	 2304,//raw_weight
	 1728,//raw_height
	 2,//raw_dataBit
	 BAYER_RGGB,//raw_colorFilterValue
	 64,//raw_blackLevel
	 74.6,//raw_viewAngle
	 10,//raw_bitWidth
	 16//raw_maxSensorGain
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_HV_MIRROR,				/*mirrorflip information IMAGE_NORMAL*/
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
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[10] = {
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8,   8, 2304,  1728,	  0,	0, 2304, 1728}, /*Preview*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8,   8, 2304,  1728,	  0,	0, 2304, 1728}, /*capture*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8, 224, 2304,  1296,	  0,	0, 2304, 1296}, /*video*/
	{ 4640, 3488,  1040, 1024, 2560, 1440, 1280,   720,  0,   0, 1280,   720,	  0,	0, 1280, 720}, /*hs_video,don't use*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  0,   0, 2320,  1744,	  0,	0, 2320, 1744}, /* slim video*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8,   8, 2304,  1728,	  0,	0, 2304, 1728}, /*custom1*/
	{ 4640, 3488,	  0,  436, 4640, 2616, 2320,  1308,  0,   0, 2320,  1308,	  0,	0, 2320, 1308}, /*custom2*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 4640,  3488, 16, 16, 4608,  3456,	  0,	0, 4608, 3456}, /*custom3*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8,   8, 2304,  1728,	  0,	0, 2304, 1728}, /*custom4*/
	{ 4640, 3488,	  0,	0, 4640, 3488, 2320,  1744,  8,   8, 2304,  1728,	  0,	0, 2304, 1728}, /*custom5*/
};


/*no mirror flip*/

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
/*extern void kdSetI2CSpeed(u16 i2cSpeed);*/
/*extern bool read_2l9_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size);*/

/****hope  add for CameraEM otp errorcode****/
extern int SUB310A_otp_read(void);
int vivo_otp_read_when_power_on;
extern unsigned int is_atboot;
extern otp_error_code_t S5K3P9SP04_OTP_ERROR_CODE;
MUINT32  sn_inf_sub_s5k3p9sp[13];  /*0 flag   1-12 data*/
MUINT32 crosstalk_data[2049];  /*0 flag   1-836 data*/
MUINT32  material_inf_sub_s5k3p9sp[4];
MUINT32  module_inf_sub_s5k3p9sp[2];
extern int read_sn_id(void);
/****hope add end****/



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

#define USE_TNP_BURST	1  /*samsung*/
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

#define I2C_BURST_LEN 1000
static bool sensor_register_is_consecutive(kal_uint16 *regs, kal_uint32 i, kal_uint32 size)
{
    if (regs[i] != regs[i + 2]){
        return false;
    }
    return true;
}
static kal_uint16 table_write_cmos_sensor_consecutive(kal_uint16 *regs, kal_uint32 size)
{
    char puSendCmd[I2C_BURST_LEN];
    kal_uint32 i, tosend;
    kal_uint16 addr = 0;
    kal_uint16 data = 0;
    kal_uint32 num_consecutive, IDX, start_IDX;
    kal_uint16 transfer_length;
    transfer_length = 0;
    num_consecutive = 1;
    IDX = 0;
    start_IDX = IDX;
    tosend = 0;
    pr_info("regs size: %d\n", size);
    if (!size || 0 != size % 2 ){
        pr_err("error reg size :%d", size);
        return -1;
    }
    while (IDX < size) {
        start_IDX = IDX;
        while ((IDX+2) < size) {
            if (sensor_register_is_consecutive(regs, IDX, size)){
                num_consecutive++;
                IDX+=2;
            } else {
                break;
            }
        }
        addr = regs[start_IDX];
        puSendCmd[tosend++] = (char)(addr >> 8);
        puSendCmd[tosend++] = (char)(addr & 0xFF);
        if (num_consecutive>1) {
            i = 0;
            while (i<num_consecutive)
            {
                data = regs[start_IDX+i*2+1];
                puSendCmd[tosend++]=(char)(data >> 8);
                puSendCmd[tosend++]=(char)(data & 0xFF);
                i++;
            }
            transfer_length = num_consecutive*2 + 2; //2 bytes for address
            iBurstWriteReg_multi(puSendCmd,
                    transfer_length,
                    imgsensor.i2c_write_id,
                    transfer_length,
                    imgsensor_info.i2c_speed);
            tosend = 0;
            num_consecutive = 1;
        }
        else {
            data = regs[IDX+1];
            puSendCmd[tosend++]=(char)(data >> 8);
            puSendCmd[tosend++]=(char)(data & 0xFF);
            iBurstWriteReg_multi(puSendCmd, tosend,
                          imgsensor.i2c_write_id, 4, imgsensor_info.i2c_speed);
            tosend = 0;
        }
        IDX+=2;
    }
    return 0;
}

static void set_dummy(void)
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	 write_cmos_sensor_16_16(0x0340, imgsensor.frame_length);
	 write_cmos_sensor_16_16(0x0342, imgsensor.line_length);
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

static void write_shutter(kal_uint16 shutter)
{

	kal_uint16 realtime_fps = 0;


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
	LOG_INF("shutter = %d, framelength = %d\n", shutter,imgsensor.frame_length);
}/*	write_shutter  */

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

	LOG_INF("shutter = %d, framelength = %d/%d, dummy_line= %d\n", shutter, imgsensor.frame_length,
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
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_16_16(0x0204,reg_gain);
	/*write_cmos_sensor_16_8(0x0204,(reg_gain>>8));*/
	/*write_cmos_sensor_16_8(0x0205,(reg_gain&0xff));*/

	return gain;
}	/*	set_gain  */

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
	    LOG_INF("Error image_mirror setting\n");
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

static kal_uint32 streaming_control(kal_bool enable)
{
	int timeout = (10000 / imgsensor.current_fps) + 1;
	int i = 0;
	int framecnt = 0;

	LOG_INF("streaming_enable(0= Sw Standby,1= streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_16_8(0x0100, 0X01);

		mdelay(10);
	} else {
		write_cmos_sensor_16_8(0x0100, 0x00);
		for (i = 0; i < timeout; i++) {
			mdelay(5);
			framecnt = read_cmos_sensor_16_8(0x0005);
			if ( framecnt == 0xFF) {
				LOG_INF(" Stream Off OK at i=%d.\n", i);
				return ERROR_NONE;
			}
		}
		LOG_INF("Stream Off Fail! framecnt= %d.\n", framecnt);
	}
	return ERROR_NONE;
}

#if USE_TNP_BURST
static const u16 uTnpArrayA[] = {
0x126F,
0x0000,
0x0000,
0x4906,
0x4805,
0xF8C1,
0x05C4,
0x4905,
0x1A08,
0x4903,
0xF8A1,
0x05C8,
0xF000,
0xBC65,
0x0000,
0x0020,
0x844A,
0x0020,
0xD02E,
0x0020,
0x006C,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0xBA40,
0x4770,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0xBAC0,
0x4770,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0x0000,
0xE92D,
0x47F0,
0x461C,
0x4690,
0x4689,
0x4607,
0x48FE,
0x2200,
0x6800,
0xB286,
0x0C05,
0x4631,
0x4628,
0xF000,
0xFCC5,
0x4623,
0x4642,
0x4649,
0x4638,
0xF000,
0xFCC4,
0x48F8,
0xF890,
0x028B,
0xB188,
0x8A78,
0xF104,
0x5400,
0xEB04,
0x0180,
0xE009,
0x6822,
0xF3C2,
0x60C3,
0xFA90,
0xF0A0,
0xF022,
0x4278,
0xEA42,
0x0050,
0xC401,
0x428C,
0xD1F3,
0x4631,
0x4628,
0xE8BD,
0x47F0,
0x2201,
0xF000,
0xBCA2,
0xE92D,
0x5FFC,
0x4683,
0x48E7,
0x468A,
0x2200,
0x6840,
0x0C01,
0xB280,
0xE9CD,
0x0100,
0x4601,
0x9801,
0xF000,
0xFC93,
0xFBAB,
0x100A,
0x4BE2,
0x4DE0,
0x4AE0,
0xF893,
0x6091,
0xF505,
0x69AA,
0xFB06,
0xF60B,
0x2300,
0x1B89,
0x464D,
0xEB60,
0x0003,
0xC503,
0x461D,
0xFBEB,
0x650A,
0xF502,
0x67AB,
0x463A,
0x4CD6,
0xC260,
0xF8A4,
0x3548,
0xF504,
0x62A9,
0xF894,
0xC4A0,
0xF894,
0x44A1,
0xF44F,
0x58F8,
0xF1BC,
0x0F01,
0xD003,
0xF1BC,
0x0F02,
0xD014,
0xE029,
0xEA08,
0x2304,
0xF043,
0x0311,
0x8013,
0x4623,
0x2200,
0xF000,
0xFC6A,
0xE9C9,
0x0100,
0x4623,
0x2200,
0x4628,
0x4631,
0xF000,
0xFC62,
0xE9C7,
0x0100,
0xE015,
0xEA08,
0x2C04,
0xF04C,
0x0C01,
0xF8A2,
0xC000,
0xFBA1,
0x2C04,
0xFB00,
0xC004,
0xFB01,
0x0103,
0xE9C9,
0x1200,
0xFBA6,
0x0104,
0xFB05,
0x1104,
0xFB06,
0x1103,
0xE9C7,
0x1000,
0x48B8,
0x49B9,
0xF8B0,
0x0548,
0x8008,
0x48B8,
0xC80C,
0xF648,
0x0022,
0xF000,
0xFC43,
0x48B5,
0x3008,
0xC80C,
0xF648,
0x002A,
0xF000,
0xFC3C,
0x4658,
0xF000,
0xFC3E,
0x49AD,
0x2201,
0xF8C1,
0xA568,
0xE9DD,
0x1000,
0xB002,
0xE8BD,
0x5FF0,
0xF000,
0xBC1F,
0x4AA8,
0xF892,
0x25D5,
0xB12A,
0x4AA6,
0x4BA5,
0xF8D2,
0x2568,
0xF8C3,
0x2430,
0x4AA3,
0xF8D2,
0x2430,
0xF000,
0xBC29,
0xB510,
0x49A0,
0x4AA3,
0x4BA4,
0xF8D1,
0x143C,
0x7C94,
0xB10C,
0x8A90,
0xE01B,
0x4A9B,
0xF892,
0x20A2,
0xF1C2,
0x020C,
0x40D1,
0x4348,
0x0A01,
0x489D,
0xF8D0,
0x0084,
0x7902,
0x434A,
0x7941,
0x88C0,
0x40CA,
0xEB00,
0x1012,
0xF44F,
0x2180,
0xFBB1,
0xF0F0,
0x1109,
0x4288,
0xD204,
0x2840,
0xD800,
0x2040,
0x8058,
0xBD10,
0x4608,
0xE7FB,
0x6841,
0x7B4A,
0x4991,
0xF8A1,
0x2382,
0x6842,
0x7B53,
0x2B00,
0xD015,
0xF501,
0x7161,
0x7B92,
0x800A,
0x6840,
0x7BC0,
0x8048,
0x488B,
0xF8B0,
0x20C2,
0x808A,
0xF8B0,
0x20C4,
0x80CA,
0xF810,
0x2FC7,
0x78C0,
0x0852,
0x0840,
0xEA42,
0x0080,
0x8108,
0x4770,
0xE92D,
0x4FFF,
0x4883,
0xB083,
0x461D,
0x79C0,
0xF8DD,
0xB044,
0x4616,
0x460F,
0x2800,
0xD06E,
0xF8DF,
0xA1F4,
0xF10A,
0x0ABA,
0xF1AA,
0x001C,
0xF8B0,
0x9000,
0xF8B0,
0x8004,
0xF000,
0xFBCC,
0x9903,
0x9C10,
0x4308,
0xF104,
0x4480,
0xD007,
0x80A7,
0x80E6,
0xF1AA,
0x001C,
0x8801,
0x8121,
0x8880,
0xE020,
0x4868,
0xF89A,
0x100C,
0xF8B0,
0x01D8,
0x4348,
0x9002,
0xF000,
0xFBBA,
0x2800,
0x9802,
0xD001,
0x1A36,
0xE000,
0x4407,
0x80A7,
0x80E6,
0x4860,
0xF8B0,
0x61DA,
0xF890,
0x028A,
0x4346,
0xF000,
0xFBAF,
0xB110,
0xEBA8,
0x0806,
0xE000,
0x44B1,
0xF8A4,
0x9008,
0x4640,
0x8160,
0x9803,
0xB128,
0x4856,
0xF890,
0x114F,
0xF890,
0x0289,
0xE003,
0xF89A,
0x100D,
0xF89A,
0x000C,
0x010A,
0x4951,
0xF891,
0x114E,
0xEA42,
0x2181,
0xF041,
0x0103,
0x81A1,
0x2101,
0x22FF,
0xEB02,
0x0040,
0xEA41,
0x2000,
0x81E0,
0xA901,
0x4668,
0xF000,
0xFB8B,
0xF89D,
0x0000,
0xF89D,
0x1004,
0xEA40,
0x2001,
0x8220,
0x465F,
0x463E,
0xF000,
0xFB7B,
0x1E79,
0x2800,
0x89A8,
0xD004,
0x1847,
0xF646,
0x10A4,
0xE003,
0xE03F,
0x1846,
0xF646,
0x1024,
0x80A8,
0x8267,
0x82E6,
0x2000,
0x82A0,
0x88A8,
0x8020,
0xF000,
0xFB70,
0x2801,
0xD10C,
0xF000,
0xFB71,
0xB148,
0xF000,
0xFB73,
0xB130,
0xF240,
0x4013,
0x81A0,
0xF240,
0x1001,
0x81E0,
0x8220,
0x6A2B,
0x2100,
0x2083,
0x9A10,
0xF000,
0xFB6A,
0x81E8,
0xF000,
0xFB58,
0x2601,
0x2801,
0xD112,
0xF000,
0xFB58,
0xB178,
0xF000,
0xFB5A,
0xB160,
0x8026,
0x4830,
0x2100,
0xE004,
0x8802,
0x0852,
0xF820,
0x2B02,
0x1C49,
0x89EA,
0xEBB1,
0x0F42,
0xDBF6,
0x89E9,
0x89A8,
0x4281,
0xD900,
0x81E8,
0x8026,
0xB007,
0xE8BD,
0x8FF0,
0xE92D,
0x43F8,
0x481A,
0x2200,
0x6940,
0xB285,
0xEA4F,
0x4810,
0x4629,
0x4640,
0xF000,
0xFAFB,
0xF000,
0xFB3F,
0x4F20,
0xF897,
0x0073,
0xB130,
0x4813,
0xF890,
0x028B,
0xB110,
0x491D,
0x201B,
0x8008,
0x481C,
0x4E0E,
0x3634,
0xF890,
0x46C0,
0x89B0,
0xB998,
0x2000,
0xF8AD,
0x0000,
0x480A,
0x2202,
0x4669,
0xF8B0,
0x0600,
0x302E,
0xF000,
0xFB27,
0xB110,
0xF8BD,
0x0000,
0x81B0,
0x89B0,
0xB910,
0xF44F,
0x6080,
0x81B0,
0xF897,
0x0075,
0xE01D,
0x0020,
0x404A,
0x0020,
0xD02E,
0x0020,
0x200E,
0x0040,
0x3288,
0x0020,
0x2034,
0x0020,
0xA021,
0x0020,
0x403F,
0x0020,
0x703E,
0x0040,
0x00A0,
0x0020,
0xC038,
0x0020,
0x1022,
0x0020,
0x0080,
0x0020,
0x5028,
0x0040,
0x7EF4,
0x0020,
0xE00F,
0xB128,
0x89B0,
0xB118,
0x4360,
0xF500,
0x7000,
0x0A84,
0x48FE,
0xF44F,
0x7280,
0xF8B0,
0x077C,
0x4290,
0xD901,
0x4601,
0xE000,
0x4611,
0x018B,
0xF5A3,
0x4380,
0x4290,
0xD901,
0x4601,
0xE000,
0x4611,
0xFB01,
0x3104,
0x23FF,
0xEBB3,
0x2F11,
0xD90E,
0x4290,
0xD901,
0x4601,
0xE000,
0x4611,
0x0189,
0xF5A1,
0x4180,
0x4290,
0xD800,
0x4610,
0xFB00,
0x1004,
0x0A00,
0xE000,
0x20FF,
0x49EB,
0x8008,
0x4629,
0x4640,
0xE8BD,
0x43F8,
0x2201,
0xF000,
0xBA7A,
0xB570,
0x48E7,
0x2200,
0x6981,
0x0C0C,
0xB28D,
0x4629,
0x4620,
0xF000,
0xFA70,
0xF000,
0xFABE,
0x48E2,
0xF890,
0x1074,
0xB111,
0x2100,
0xF880,
0x1070,
0x48E0,
0xF44F,
0x7180,
0xF890,
0x206F,
0xF44F,
0x4030,
0xF000,
0xFA5E,
0x4629,
0x4620,
0xE8BD,
0x4070,
0x2201,
0xF000,
0xBA57,
0xB570,
0x4604,
0x48D6,
0x4DD7,
0xF890,
0x0408,
0xB1C8,
0x4628,
0xF890,
0x0609,
0xB1A8,
0x4628,
0xF8D5,
0x2384,
0xF8C0,
0x2414,
0xF200,
0x4114,
0x462A,
0xF8D5,
0x0390,
0xF8C2,
0x0420,
0xF8D5,
0x43C0,
0x4610,
0xF8C5,
0x42E4,
0xF8C0,
0x4430,
0x4608,
0xF000,
0xFA8B,
0x49C7,
0xF8B5,
0x22B0,
0x8F08,
0x8F49,
0x1A20,
0x1E40,
0x4411,
0x4281,
0xD900,
0x4608,
0xF8A5,
0x02B2,
0xBD70,
0xE92D,
0x41F0,
0x4606,
0x48BD,
0x2200,
0x6A00,
0xB285,
0x0C04,
0x4629,
0x4620,
0xF000,
0xFA1C,
0x4630,
0xF000,
0xFA73,
0x48BB,
0x4FBB,
0x6800,
0x683B,
0x8B41,
0x0A09,
0xF883,
0x1036,
0x7EC1,
0xF883,
0x1038,
0x49B4,
0xF891,
0x214C,
0x2A00,
0xF8D1,
0x2134,
0xD001,
0x1C52,
0x0852,
0x33CE,
0x0A16,
0x711E,
0x719A,
0xF8B1,
0x213C,
0xF3C2,
0x1257,
0x701A,
0xF891,
0x213D,
0x00D2,
0x709A,
0xF891,
0x214D,
0x3BCE,
0xB122,
0xF8D1,
0x2138,
0x1C52,
0x0856,
0xE001,
0xF8D1,
0x6138,
0x687A,
0xEA4F,
0x2C16,
0xF501,
0x7190,
0xF882,
0xC016,
0x7616,
0x8BCE,
0xF500,
0x70BA,
0xF3C6,
0x1657,
0x7496,
0x7FCE,
0x00F6,
0x7516,
0x8C0E,
0x68CF,
0x08F6,
0x437E,
0x0B36,
0x0A37,
0xF803,
0x7FD6,
0x3277,
0x709E,
0x8806,
0x0A36,
0xF802,
0x6C27,
0x7846,
0xF802,
0x6C25,
0x8886,
0x0A36,
0xF802,
0x6C1F,
0x7946,
0xF802,
0x6C1D,
0x4E8F,
0xF896,
0x6410,
0x71D6,
0x4E8D,
0xF896,
0x6411,
0x7256,
0x4E8B,
0xF896,
0x640B,
0x72D6,
0x4E89,
0xF896,
0x6409,
0x7356,
0xF890,
0x6030,
0x73D6,
0xF890,
0x00DE,
0xF802,
0x0F1F,
0x4884,
0xF200,
0x4672,
0xF890,
0x0472,
0x7490,
0x7830,
0x7510,
0x22A5,
0x70DA,
0x200E,
0x7118,
0xF811,
0x0C7E,
0xF1C0,
0x010C,
0x487C,
0xF8D0,
0x043C,
0x40C8,
0x0A06,
0x719E,
0x7218,
0x2001,
0xF803,
0x0C2C,
0x4877,
0xF8D0,
0x044C,
0x40C8,
0x21AA,
0xF803,
0x1D57,
0x2602,
0x705E,
0x709A,
0x2230,
0x70DA,
0x225A,
0x711A,
0x0A06,
0x715E,
0x719A,
0x71D8,
0x7219,
0x2000,
0x7258,
0x4629,
0x4620,
0xE8BD,
0x41F0,
0x2201,
0xF000,
0xB977,
0xE92D,
0x41F0,
0x4607,
0x4864,
0x460C,
0x2200,
0x6A40,
0xB286,
0x0C05,
0x4631,
0x4628,
0xF000,
0xF96A,
0x4621,
0x4638,
0xF000,
0xF9C5,
0x4860,
0xF890,
0x0297,
0xB910,
0xF000,
0xF997,
0xB120,
0xF104,
0x4480,
0x8AA0,
0x1C40,
0x82A0,
0x4631,
0x4628,
0xE8BD,
0x41F0,
0x2201,
0xF000,
0xB953,
0xE92D,
0x41F0,
0x4607,
0x4852,
0x460E,
0x2200,
0x6A80,
0xB285,
0x0C04,
0x4629,
0x4620,
0xF000,
0xF946,
0x4631,
0x4638,
0xF000,
0xF9A6,
0x4F4B,
0xF24D,
0x260C,
0x3734,
0xF44F,
0x6180,
0x783A,
0x4630,
0xF000,
0xF938,
0x7878,
0xB3C8,
0x2200,
0xF44F,
0x7100,
0x4630,
0xF000,
0xF930,
0x4848,
0x8800,
0x4B48,
0xF8A3,
0x0244,
0x4846,
0x1D00,
0x8800,
0xF8A3,
0x0246,
0xF8B3,
0x0244,
0xF8B3,
0x1246,
0x1842,
0xD002,
0x0280,
0xFBB0,
0xF2F2,
0xB291,
0x4A40,
0xF8A3,
0x1248,
0x8850,
0x8812,
0x4B3D,
0xF8A3,
0x05A6,
0xF8A3,
0x25A8,
0x1880,
0xD005,
0x0292,
0xFBB2,
0xF0F0,
0x461A,
0xF8A2,
0x05AA,
0x4836,
0xF8B0,
0x05AA,
0x180A,
0xFB01,
0x2010,
0xF340,
0x1095,
0x2810,
0xDC06,
0x2800,
0xDA05,
0x2000,
0xE003,
0xE7FF,
0x2201,
0xE7C3,
0x2010,
0x492F,
0x8008,
0x4629,
0x4620,
0xE8BD,
0x41F0,
0x2201,
0xF000,
0xB8EF,
0xB570,
0x4821,
0x2200,
0x6AC1,
0x0C0C,
0xB28D,
0x4629,
0x4620,
0xF000,
0xF8E5,
0x4821,
0x6802,
0xF8B2,
0x0262,
0x0183,
0xF892,
0x0260,
0xF010,
0x0F02,
0xD009,
0x4818,
0x3034,
0x8881,
0x4299,
0xD806,
0x8840,
0xF5A0,
0x4151,
0x3923,
0xD101,
0xF000,
0xF938,
0x4629,
0x4620,
0xE8BD,
0x4070,
0x2201,
0xF000,
0xB8C8,
0xB570,
0x4606,
0x480D,
0x2200,
0x6B01,
0x0C0C,
0xB28D,
0x4629,
0x4620,
0xF000,
0xF8BD,
0x4630,
0xF000,
0xF928,
0x4907,
0x4A11,
0x3134,
0x79CB,
0x68D0,
0x4098,
0x60D0,
0x6810,
0x4098,
0x6010,
0x6888,
0xE019,
0x0020,
0xE00F,
0x0040,
0x74F4,
0x0020,
0x404A,
0x0020,
0x5028,
0x0020,
0x200E,
0x0020,
0xD02E,
0x0020,
0xD008,
0x0020,
0xE036,
0x0040,
0x0494,
0x0020,
0xC038,
0x0040,
0x14D2,
0x0040,
0x10A4,
0x0020,
0x5432,
0x63D0,
0x4629,
0x4620,
0xE8BD,
0x4070,
0x2201,
0xF000,
0xB88C,
0xB510,
0x2200,
0xF6AF,
0x0197,
0x4833,
0xF000,
0xF8F8,
0x4C33,
0x2200,
0xF6AF,
0x013F,
0x6020,
0x4831,
0xF000,
0xF8F0,
0x6060,
0xF2AF,
0x7049,
0x492F,
0x2200,
0x61C8,
0xF2AF,
0x619F,
0x482E,
0xF000,
0xF8E5,
0x2200,
0xF2AF,
0x512D,
0x6120,
0x482B,
0xF000,
0xF8DE,
0x2200,
0xF2AF,
0x4123,
0x6160,
0x4829,
0xF000,
0xF8D7,
0x2200,
0xF2AF,
0x31E9,
0x61A0,
0x4826,
0xF000,
0xF8D0,
0x2200,
0xF2AF,
0x716B,
0x61E0,
0x4824,
0xF000,
0xF8C9,
0x2200,
0xF2AF,
0x7123,
0x60A0,
0x4821,
0xF000,
0xF8C2,
0x2200,
0xF2AF,
0x31B7,
0x60E0,
0x481F,
0xF000,
0xF8BB,
0x2200,
0xF2AF,
0x2161,
0x6220,
0x481C,
0xF000,
0xF8B4,
0x6260,
0x2000,
0xF104,
0x0134,
0x4602,
0x8188,
0xF2AF,
0x2131,
0x4818,
0xF000,
0xF8A9,
0x2200,
0xF2AF,
0x1175,
0x62A0,
0x4815,
0xF000,
0xF8A2,
0x2200,
0xF2AF,
0x1137,
0x62E0,
0x4813,
0xF000,
0xF89B,
0x4912,
0x6320,
0xF640,
0x00F1,
0x6809,
0x8348,
0xBD10,
0x0000,
0x0000,
0x1FDE,
0x0020,
0x404A,
0x0000,
0x3B5F,
0x0020,
0x5008,
0x0000,
0x19D7,
0x0000,
0xFF27,
0x0000,
0xE339,
0x0100,
0xCF32,
0x0100,
0x3B1E,
0x0000,
0x45EC,
0x0000,
0xB967,
0x0000,
0x2BE6,
0x0100,
0x6522,
0x0000,
0x838C,
0x0000,
0x4954,
0x0020,
0xD008,
0xF24A,
0x1C2B,
0xF2C0,
0x0C00,
0x4760,
0xF64D,
0x6C1F,
0xF2C0,
0x0C00,
0x4760,
0xF644,
0x5C65,
0xF2C0,
0x0C01,
0x4760,
0xF645,
0x3C43,
0xF2C0,
0x0C00,
0x4760,
0xF645,
0x6CE3,
0xF2C0,
0x0C00,
0x4760,
0xF246,
0x1C7B,
0xF2C0,
0x0C00,
0x4760,
0xF644,
0x0CD9,
0xF2C0,
0x0C00,
0x4760,
0xF644,
0x1C79,
0xF2C0,
0x0C00,
0x4760,
0xF644,
0x1C81,
0xF2C0,
0x0C00,
0x4760,
0xF244,
0x0CB5,
0xF2C0,
0x0C00,
0x4760,
0xF644,
0x0CE9,
0xF2C0,
0x0C00,
0x4760,
0xF643,
0x5C15,
0xF2C0,
0x0C00,
0x4760,
0xF643,
0x5C1D,
0xF2C0,
0x0C00,
0x4760,
0xF24D,
0x5CC9,
0xF2C0,
0x0C00,
0x4760,
0xF242,
0x7CFF,
0xF2C0,
0x0C00,
0x4760,
0xF248,
0x2C71,
0xF2C0,
0x0C00,
0x4760,
0xF643,
0x1CE3,
0xF2C0,
0x0C00,
0x4760,
0xF243,
0x4C37,
0xF2C0,
0x0C01,
0x4760,
0xF246,
0x7CB9,
0xF2C0,
0x0C00,
0x4760,
0xF24E,
0x6C2B,
0xF2C0,
0x0C00,
0x4760,
0xF242,
0x2C65,
0xF2C0,
0x0C01,
0x4760,
0xF648,
0x4C83,
0xF2C0,
0x0C00,
0x4760,
0xF245,
0x4C49,
0xF2C0,
0x0C00,
0x4760,
0xF24C,
0x1C2D,
0xF2C0,
0x0C00,
0x4760,
};

static kal_uint16 addr_data_pair_init_Abbreviations[] = {
	0x6028, 0x2000,
	0x602A, 0x16F0,
	0x6F12, 0x2929,
	0x602A, 0x16F2,
	0x6F12, 0x2929,
	0x602A, 0x16FA,
	0x6F12, 0x0029,
	0x602A, 0x16FC,
	0x6F12, 0x0029,
	0x602A, 0x1708,
	0x6F12, 0x0029,
	0x602A, 0x170A,
	0x6F12, 0x0029,
	0x602A, 0x1712,
	0x6F12, 0x2929,
	0x602A, 0x1714,
	0x6F12, 0x2929,
	0x602A, 0x1716,
	0x6F12, 0x2929,
	0x602A, 0x1722,
	0x6F12, 0x152A,
	0x602A, 0x1724,
	0x6F12, 0x152A,
	0x602A, 0x172C,
	0x6F12, 0x002A,
	0x602A, 0x172E,
	0x6F12, 0x002A,
	0x602A, 0x1736,
	0x6F12, 0x1500,
	0x602A, 0x1738,
	0x6F12, 0x1500,
	0x602A, 0x1740,
	0x6F12, 0x152A,
	0x602A, 0x1742,
	0x6F12, 0x152A,
	0x602A, 0x16BE,
	0x6F12, 0x1515,
	0x6F12, 0x1515,
	0x602A, 0x16C8,
	0x6F12, 0x0029,
	0x6F12, 0x0029,
	0x602A, 0x16D6,
	0x6F12, 0x0015,
	0x6F12, 0x0015,
	0x602A, 0x16E0,
	0x6F12, 0x2929,
	0x6F12, 0x2929,
	0x6F12, 0x2929,
	0x602A, 0x19B8,
	0x6F12, 0x0100,
	0x602A, 0x2224,
	0x6F12, 0x0100,
	0x602A, 0x0DF8,
	0x6F12, 0x1001,
	0x602A, 0x1EDA,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x6F12, 0x000E,
	0x602A, 0x16A0,
	0x6F12, 0x3D09,
	0x602A, 0x10A8,
	0x6F12, 0x000E,
	0x602A, 0x1198,
	0x6F12, 0x002B,
	0x602A, 0x1002,
	0x6F12, 0x0001,
	0x602A, 0x0F70,
	0x6F12, 0x0101,
	0x6F12, 0x002F,
	0x6F12, 0x007F,
	0x6F12, 0x0030,
	0x6F12, 0x0080,
	0x6F12, 0x000B,
	0x6F12, 0x0009,
	0x6F12, 0xF46E,
	0x602A, 0x0FAA,
	0x6F12, 0x000D,
	0x6F12, 0x0003,
	0x6F12, 0xF464,
	0x602A, 0x1698,
	0x6F12, 0x0D05,
	0x602A, 0x20A0,
	0x6F12, 0x0001,
	0x6F12, 0x0203,
	0x602A, 0x4A74,
	0x6F12, 0x0101,
	0x6F12, 0x0000,
	0x6F12, 0x1F80,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0FEA, 0x1440,
	0x0B06, 0x0101,
	0xF44A, 0x0007,
	0xF456, 0x000A,
	0xF46A, 0xBFA0,
	0x0D80, 0x1388,
	0x0A02, 0x007E,
	0xB134, 0x0080,
	0xB136, 0x0000,
	0xB138, 0x0000,
	//0xB13C, 0x0000,	//mipi_lp_1v2
	0xB13C, 0x0800,		//mipi_lp_1v15
};
#else
static kal_uint16 addr_data_pair_init[] = {
0x6028, 0x2000,
0x602A, 0x3F4C,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0649,
0x6F12, 0x0548,
0x6F12, 0xC1F8,
0x6F12, 0xC405,
0x6F12, 0x0549,
0x6F12, 0x081A,
0x6F12, 0x0349,
0x6F12, 0xA1F8,
0x6F12, 0xC805,
0x6F12, 0x00F0,
0x6F12, 0x65BC,
0x6F12, 0x0000,
0x6F12, 0x2000,
0x6F12, 0x4A84,
0x6F12, 0x2000,
0x6F12, 0x2ED0,
0x6F12, 0x2000,
0x6F12, 0x6C00,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x40BA,
0x6F12, 0x7047,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0xC0BA,
0x6F12, 0x7047,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0x2DE9,
0x6F12, 0xF047,
0x6F12, 0x1C46,
0x6F12, 0x9046,
0x6F12, 0x8946,
0x6F12, 0x0746,
0x6F12, 0xFE48,
0x6F12, 0x0022,
0x6F12, 0x0068,
0x6F12, 0x86B2,
0x6F12, 0x050C,
0x6F12, 0x3146,
0x6F12, 0x2846,
0x6F12, 0x00F0,
0x6F12, 0xC5FC,
0x6F12, 0x2346,
0x6F12, 0x4246,
0x6F12, 0x4946,
0x6F12, 0x3846,
0x6F12, 0x00F0,
0x6F12, 0xC4FC,
0x6F12, 0xF848,
0x6F12, 0x90F8,
0x6F12, 0x8B02,
0x6F12, 0x88B1,
0x6F12, 0x788A,
0x6F12, 0x04F1,
0x6F12, 0x0054,
0x6F12, 0x04EB,
0x6F12, 0x8001,
0x6F12, 0x09E0,
0x6F12, 0x2268,
0x6F12, 0xC2F3,
0x6F12, 0xC360,
0x6F12, 0x90FA,
0x6F12, 0xA0F0,
0x6F12, 0x22F0,
0x6F12, 0x7842,
0x6F12, 0x42EA,
0x6F12, 0x5000,
0x6F12, 0x01C4,
0x6F12, 0x8C42,
0x6F12, 0xF3D1,
0x6F12, 0x3146,
0x6F12, 0x2846,
0x6F12, 0xBDE8,
0x6F12, 0xF047,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0xA2BC,
0x6F12, 0x2DE9,
0x6F12, 0xFC5F,
0x6F12, 0x8346,
0x6F12, 0xE748,
0x6F12, 0x8A46,
0x6F12, 0x0022,
0x6F12, 0x4068,
0x6F12, 0x010C,
0x6F12, 0x80B2,
0x6F12, 0xCDE9,
0x6F12, 0x0001,
0x6F12, 0x0146,
0x6F12, 0x0198,
0x6F12, 0x00F0,
0x6F12, 0x93FC,
0x6F12, 0xABFB,
0x6F12, 0x0A10,
0x6F12, 0xE24B,
0x6F12, 0xE04D,
0x6F12, 0xE04A,
0x6F12, 0x93F8,
0x6F12, 0x9160,
0x6F12, 0x05F5,
0x6F12, 0xAA69,
0x6F12, 0x06FB,
0x6F12, 0x0BF6,
0x6F12, 0x0023,
0x6F12, 0x891B,
0x6F12, 0x4D46,
0x6F12, 0x60EB,
0x6F12, 0x0300,
0x6F12, 0x03C5,
0x6F12, 0x1D46,
0x6F12, 0xEBFB,
0x6F12, 0x0A65,
0x6F12, 0x02F5,
0x6F12, 0xAB67,
0x6F12, 0x3A46,
0x6F12, 0xD64C,
0x6F12, 0x60C2,
0x6F12, 0xA4F8,
0x6F12, 0x4835,
0x6F12, 0x04F5,
0x6F12, 0xA962,
0x6F12, 0x94F8,
0x6F12, 0xA0C4,
0x6F12, 0x94F8,
0x6F12, 0xA144,
0x6F12, 0x4FF4,
0x6F12, 0xF858,
0x6F12, 0xBCF1,
0x6F12, 0x010F,
0x6F12, 0x03D0,
0x6F12, 0xBCF1,
0x6F12, 0x020F,
0x6F12, 0x14D0,
0x6F12, 0x29E0,
0x6F12, 0x08EA,
0x6F12, 0x0423,
0x6F12, 0x43F0,
0x6F12, 0x1103,
0x6F12, 0x1380,
0x6F12, 0x2346,
0x6F12, 0x0022,
0x6F12, 0x00F0,
0x6F12, 0x6AFC,
0x6F12, 0xC9E9,
0x6F12, 0x0001,
0x6F12, 0x2346,
0x6F12, 0x0022,
0x6F12, 0x2846,
0x6F12, 0x3146,
0x6F12, 0x00F0,
0x6F12, 0x62FC,
0x6F12, 0xC7E9,
0x6F12, 0x0001,
0x6F12, 0x15E0,
0x6F12, 0x08EA,
0x6F12, 0x042C,
0x6F12, 0x4CF0,
0x6F12, 0x010C,
0x6F12, 0xA2F8,
0x6F12, 0x00C0,
0x6F12, 0xA1FB,
0x6F12, 0x042C,
0x6F12, 0x00FB,
0x6F12, 0x04C0,
0x6F12, 0x01FB,
0x6F12, 0x0301,
0x6F12, 0xC9E9,
0x6F12, 0x0012,
0x6F12, 0xA6FB,
0x6F12, 0x0401,
0x6F12, 0x05FB,
0x6F12, 0x0411,
0x6F12, 0x06FB,
0x6F12, 0x0311,
0x6F12, 0xC7E9,
0x6F12, 0x0010,
0x6F12, 0xB848,
0x6F12, 0xB949,
0x6F12, 0xB0F8,
0x6F12, 0x4805,
0x6F12, 0x0880,
0x6F12, 0xB848,
0x6F12, 0x0CC8,
0x6F12, 0x48F6,
0x6F12, 0x2200,
0x6F12, 0x00F0,
0x6F12, 0x43FC,
0x6F12, 0xB548,
0x6F12, 0x0830,
0x6F12, 0x0CC8,
0x6F12, 0x48F6,
0x6F12, 0x2A00,
0x6F12, 0x00F0,
0x6F12, 0x3CFC,
0x6F12, 0x5846,
0x6F12, 0x00F0,
0x6F12, 0x3EFC,
0x6F12, 0xAD49,
0x6F12, 0x0122,
0x6F12, 0xC1F8,
0x6F12, 0x68A5,
0x6F12, 0xDDE9,
0x6F12, 0x0010,
0x6F12, 0x02B0,
0x6F12, 0xBDE8,
0x6F12, 0xF05F,
0x6F12, 0x00F0,
0x6F12, 0x1FBC,
0x6F12, 0xA84A,
0x6F12, 0x92F8,
0x6F12, 0xD525,
0x6F12, 0x2AB1,
0x6F12, 0xA64A,
0x6F12, 0xA54B,
0x6F12, 0xD2F8,
0x6F12, 0x6825,
0x6F12, 0xC3F8,
0x6F12, 0x3024,
0x6F12, 0xA34A,
0x6F12, 0xD2F8,
0x6F12, 0x3024,
0x6F12, 0x00F0,
0x6F12, 0x29BC,
0x6F12, 0x10B5,
0x6F12, 0xA049,
0x6F12, 0xA34A,
0x6F12, 0xA44B,
0x6F12, 0xD1F8,
0x6F12, 0x3C14,
0x6F12, 0x947C,
0x6F12, 0x0CB1,
0x6F12, 0x908A,
0x6F12, 0x1BE0,
0x6F12, 0x9B4A,
0x6F12, 0x92F8,
0x6F12, 0xA220,
0x6F12, 0xC2F1,
0x6F12, 0x0C02,
0x6F12, 0xD140,
0x6F12, 0x4843,
0x6F12, 0x010A,
0x6F12, 0x9D48,
0x6F12, 0xD0F8,
0x6F12, 0x8400,
0x6F12, 0x0279,
0x6F12, 0x4A43,
0x6F12, 0x4179,
0x6F12, 0xC088,
0x6F12, 0xCA40,
0x6F12, 0x00EB,
0x6F12, 0x1210,
0x6F12, 0x4FF4,
0x6F12, 0x8021,
0x6F12, 0xB1FB,
0x6F12, 0xF0F0,
0x6F12, 0x0911,
0x6F12, 0x8842,
0x6F12, 0x04D2,
0x6F12, 0x4028,
0x6F12, 0x00D8,
0x6F12, 0x4020,
0x6F12, 0x5880,
0x6F12, 0x10BD,
0x6F12, 0x0846,
0x6F12, 0xFBE7,
0x6F12, 0x4168,
0x6F12, 0x4A7B,
0x6F12, 0x9149,
0x6F12, 0xA1F8,
0x6F12, 0x8223,
0x6F12, 0x4268,
0x6F12, 0x537B,
0x6F12, 0x002B,
0x6F12, 0x15D0,
0x6F12, 0x01F5,
0x6F12, 0x6171,
0x6F12, 0x927B,
0x6F12, 0x0A80,
0x6F12, 0x4068,
0x6F12, 0xC07B,
0x6F12, 0x4880,
0x6F12, 0x8B48,
0x6F12, 0xB0F8,
0x6F12, 0xC220,
0x6F12, 0x8A80,
0x6F12, 0xB0F8,
0x6F12, 0xC420,
0x6F12, 0xCA80,
0x6F12, 0x10F8,
0x6F12, 0xC72F,
0x6F12, 0xC078,
0x6F12, 0x5208,
0x6F12, 0x4008,
0x6F12, 0x42EA,
0x6F12, 0x8000,
0x6F12, 0x0881,
0x6F12, 0x7047,
0x6F12, 0x2DE9,
0x6F12, 0xFF4F,
0x6F12, 0x8348,
0x6F12, 0x83B0,
0x6F12, 0x1D46,
0x6F12, 0xC079,
0x6F12, 0xDDF8,
0x6F12, 0x44B0,
0x6F12, 0x1646,
0x6F12, 0x0F46,
0x6F12, 0x0028,
0x6F12, 0x6ED0,
0x6F12, 0xDFF8,
0x6F12, 0xF4A1,
0x6F12, 0x0AF1,
0x6F12, 0xBA0A,
0x6F12, 0xAAF1,
0x6F12, 0x1C00,
0x6F12, 0xB0F8,
0x6F12, 0x0090,
0x6F12, 0xB0F8,
0x6F12, 0x0480,
0x6F12, 0x00F0,
0x6F12, 0xCCFB,
0x6F12, 0x0399,
0x6F12, 0x109C,
0x6F12, 0x0843,
0x6F12, 0x04F1,
0x6F12, 0x8044,
0x6F12, 0x07D0,
0x6F12, 0xA780,
0x6F12, 0xE680,
0x6F12, 0xAAF1,
0x6F12, 0x1C00,
0x6F12, 0x0188,
0x6F12, 0x2181,
0x6F12, 0x8088,
0x6F12, 0x20E0,
0x6F12, 0x6848,
0x6F12, 0x9AF8,
0x6F12, 0x0C10,
0x6F12, 0xB0F8,
0x6F12, 0xD801,
0x6F12, 0x4843,
0x6F12, 0x0290,
0x6F12, 0x00F0,
0x6F12, 0xBAFB,
0x6F12, 0x0028,
0x6F12, 0x0298,
0x6F12, 0x01D0,
0x6F12, 0x361A,
0x6F12, 0x00E0,
0x6F12, 0x0744,
0x6F12, 0xA780,
0x6F12, 0xE680,
0x6F12, 0x6048,
0x6F12, 0xB0F8,
0x6F12, 0xDA61,
0x6F12, 0x90F8,
0x6F12, 0x8A02,
0x6F12, 0x4643,
0x6F12, 0x00F0,
0x6F12, 0xAFFB,
0x6F12, 0x10B1,
0x6F12, 0xA8EB,
0x6F12, 0x0608,
0x6F12, 0x00E0,
0x6F12, 0xB144,
0x6F12, 0xA4F8,
0x6F12, 0x0890,
0x6F12, 0x4046,
0x6F12, 0x6081,
0x6F12, 0x0398,
0x6F12, 0x28B1,
0x6F12, 0x5648,
0x6F12, 0x90F8,
0x6F12, 0x4F11,
0x6F12, 0x90F8,
0x6F12, 0x8902,
0x6F12, 0x03E0,
0x6F12, 0x9AF8,
0x6F12, 0x0D10,
0x6F12, 0x9AF8,
0x6F12, 0x0C00,
0x6F12, 0x0A01,
0x6F12, 0x5149,
0x6F12, 0x91F8,
0x6F12, 0x4E11,
0x6F12, 0x42EA,
0x6F12, 0x8121,
0x6F12, 0x41F0,
0x6F12, 0x0301,
0x6F12, 0xA181,
0x6F12, 0x0121,
0x6F12, 0xFF22,
0x6F12, 0x02EB,
0x6F12, 0x4000,
0x6F12, 0x41EA,
0x6F12, 0x0020,
0x6F12, 0xE081,
0x6F12, 0x01A9,
0x6F12, 0x6846,
0x6F12, 0x00F0,
0x6F12, 0x8BFB,
0x6F12, 0x9DF8,
0x6F12, 0x0000,
0x6F12, 0x9DF8,
0x6F12, 0x0410,
0x6F12, 0x40EA,
0x6F12, 0x0120,
0x6F12, 0x2082,
0x6F12, 0x5F46,
0x6F12, 0x3E46,
0x6F12, 0x00F0,
0x6F12, 0x7BFB,
0x6F12, 0x791E,
0x6F12, 0x0028,
0x6F12, 0xA889,
0x6F12, 0x04D0,
0x6F12, 0x4718,
0x6F12, 0x46F6,
0x6F12, 0xA410,
0x6F12, 0x03E0,
0x6F12, 0x3FE0,
0x6F12, 0x4618,
0x6F12, 0x46F6,
0x6F12, 0x2410,
0x6F12, 0xA880,
0x6F12, 0x6782,
0x6F12, 0xE682,
0x6F12, 0x0020,
0x6F12, 0xA082,
0x6F12, 0xA888,
0x6F12, 0x2080,
0x6F12, 0x00F0,
0x6F12, 0x70FB,
0x6F12, 0x0128,
0x6F12, 0x0CD1,
0x6F12, 0x00F0,
0x6F12, 0x71FB,
0x6F12, 0x48B1,
0x6F12, 0x00F0,
0x6F12, 0x73FB,
0x6F12, 0x30B1,
0x6F12, 0x40F2,
0x6F12, 0x1340,
0x6F12, 0xA081,
0x6F12, 0x40F2,
0x6F12, 0x0110,
0x6F12, 0xE081,
0x6F12, 0x2082,
0x6F12, 0x2B6A,
0x6F12, 0x0021,
0x6F12, 0x8320,
0x6F12, 0x109A,
0x6F12, 0x00F0,
0x6F12, 0x6AFB,
0x6F12, 0xE881,
0x6F12, 0x00F0,
0x6F12, 0x58FB,
0x6F12, 0x0126,
0x6F12, 0x0128,
0x6F12, 0x12D1,
0x6F12, 0x00F0,
0x6F12, 0x58FB,
0x6F12, 0x78B1,
0x6F12, 0x00F0,
0x6F12, 0x5AFB,
0x6F12, 0x60B1,
0x6F12, 0x2680,
0x6F12, 0x3048,
0x6F12, 0x0021,
0x6F12, 0x04E0,
0x6F12, 0x0288,
0x6F12, 0x5208,
0x6F12, 0x20F8,
0x6F12, 0x022B,
0x6F12, 0x491C,
0x6F12, 0xEA89,
0x6F12, 0xB1EB,
0x6F12, 0x420F,
0x6F12, 0xF6DB,
0x6F12, 0xE989,
0x6F12, 0xA889,
0x6F12, 0x8142,
0x6F12, 0x00D9,
0x6F12, 0xE881,
0x6F12, 0x2680,
0x6F12, 0x07B0,
0x6F12, 0xBDE8,
0x6F12, 0xF08F,
0x6F12, 0x2DE9,
0x6F12, 0xF843,
0x6F12, 0x1A48,
0x6F12, 0x0022,
0x6F12, 0x4069,
0x6F12, 0x85B2,
0x6F12, 0x4FEA,
0x6F12, 0x1048,
0x6F12, 0x2946,
0x6F12, 0x4046,
0x6F12, 0x00F0,
0x6F12, 0xFBFA,
0x6F12, 0x00F0,
0x6F12, 0x3FFB,
0x6F12, 0x204F,
0x6F12, 0x97F8,
0x6F12, 0x7300,
0x6F12, 0x30B1,
0x6F12, 0x1348,
0x6F12, 0x90F8,
0x6F12, 0x8B02,
0x6F12, 0x10B1,
0x6F12, 0x1D49,
0x6F12, 0x1B20,
0x6F12, 0x0880,
0x6F12, 0x1C48,
0x6F12, 0x0E4E,
0x6F12, 0x3436,
0x6F12, 0x90F8,
0x6F12, 0xC046,
0x6F12, 0xB089,
0x6F12, 0x98B9,
0x6F12, 0x0020,
0x6F12, 0xADF8,
0x6F12, 0x0000,
0x6F12, 0x0A48,
0x6F12, 0x0222,
0x6F12, 0x6946,
0x6F12, 0xB0F8,
0x6F12, 0x0006,
0x6F12, 0x2E30,
0x6F12, 0x00F0,
0x6F12, 0x27FB,
0x6F12, 0x10B1,
0x6F12, 0xBDF8,
0x6F12, 0x0000,
0x6F12, 0xB081,
0x6F12, 0xB089,
0x6F12, 0x10B9,
0x6F12, 0x4FF4,
0x6F12, 0x8060,
0x6F12, 0xB081,
0x6F12, 0x97F8,
0x6F12, 0x7500,
0x6F12, 0x1DE0,
0x6F12, 0x2000,
0x6F12, 0x4A40,
0x6F12, 0x2000,
0x6F12, 0x2ED0,
0x6F12, 0x2000,
0x6F12, 0x0E20,
0x6F12, 0x4000,
0x6F12, 0x8832,
0x6F12, 0x2000,
0x6F12, 0x3420,
0x6F12, 0x2000,
0x6F12, 0x21A0,
0x6F12, 0x2000,
0x6F12, 0x3F40,
0x6F12, 0x2000,
0x6F12, 0x3E70,
0x6F12, 0x4000,
0x6F12, 0xA000,
0x6F12, 0x2000,
0x6F12, 0x38C0,
0x6F12, 0x2000,
0x6F12, 0x2210,
0x6F12, 0x2000,
0x6F12, 0x8000,
0x6F12, 0x2000,
0x6F12, 0x2850,
0x6F12, 0x4000,
0x6F12, 0xF47E,
0x6F12, 0x2000,
0x6F12, 0x0FE0,
0x6F12, 0x28B1,
0x6F12, 0xB089,
0x6F12, 0x18B1,
0x6F12, 0x6043,
0x6F12, 0x00F5,
0x6F12, 0x0070,
0x6F12, 0x840A,
0x6F12, 0xFE48,
0x6F12, 0x4FF4,
0x6F12, 0x8072,
0x6F12, 0xB0F8,
0x6F12, 0x7C07,
0x6F12, 0x9042,
0x6F12, 0x01D9,
0x6F12, 0x0146,
0x6F12, 0x00E0,
0x6F12, 0x1146,
0x6F12, 0x8B01,
0x6F12, 0xA3F5,
0x6F12, 0x8043,
0x6F12, 0x9042,
0x6F12, 0x01D9,
0x6F12, 0x0146,
0x6F12, 0x00E0,
0x6F12, 0x1146,
0x6F12, 0x01FB,
0x6F12, 0x0431,
0x6F12, 0xFF23,
0x6F12, 0xB3EB,
0x6F12, 0x112F,
0x6F12, 0x0ED9,
0x6F12, 0x9042,
0x6F12, 0x01D9,
0x6F12, 0x0146,
0x6F12, 0x00E0,
0x6F12, 0x1146,
0x6F12, 0x8901,
0x6F12, 0xA1F5,
0x6F12, 0x8041,
0x6F12, 0x9042,
0x6F12, 0x00D8,
0x6F12, 0x1046,
0x6F12, 0x00FB,
0x6F12, 0x0410,
0x6F12, 0x000A,
0x6F12, 0x00E0,
0x6F12, 0xFF20,
0x6F12, 0xEB49,
0x6F12, 0x0880,
0x6F12, 0x2946,
0x6F12, 0x4046,
0x6F12, 0xBDE8,
0x6F12, 0xF843,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0x7ABA,
0x6F12, 0x70B5,
0x6F12, 0xE748,
0x6F12, 0x0022,
0x6F12, 0x8169,
0x6F12, 0x0C0C,
0x6F12, 0x8DB2,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0x70FA,
0x6F12, 0x00F0,
0x6F12, 0xBEFA,
0x6F12, 0xE248,
0x6F12, 0x90F8,
0x6F12, 0x7410,
0x6F12, 0x11B1,
0x6F12, 0x0021,
0x6F12, 0x80F8,
0x6F12, 0x7010,
0x6F12, 0xE048,
0x6F12, 0x4FF4,
0x6F12, 0x8071,
0x6F12, 0x90F8,
0x6F12, 0x6F20,
0x6F12, 0x4FF4,
0x6F12, 0x3040,
0x6F12, 0x00F0,
0x6F12, 0x5EFA,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0xBDE8,
0x6F12, 0x7040,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0x57BA,
0x6F12, 0x70B5,
0x6F12, 0x0446,
0x6F12, 0xD648,
0x6F12, 0xD74D,
0x6F12, 0x90F8,
0x6F12, 0x0804,
0x6F12, 0xC8B1,
0x6F12, 0x2846,
0x6F12, 0x90F8,
0x6F12, 0x0906,
0x6F12, 0xA8B1,
0x6F12, 0x2846,
0x6F12, 0xD5F8,
0x6F12, 0x8423,
0x6F12, 0xC0F8,
0x6F12, 0x1424,
0x6F12, 0x00F2,
0x6F12, 0x1441,
0x6F12, 0x2A46,
0x6F12, 0xD5F8,
0x6F12, 0x9003,
0x6F12, 0xC2F8,
0x6F12, 0x2004,
0x6F12, 0xD5F8,
0x6F12, 0xC043,
0x6F12, 0x1046,
0x6F12, 0xC5F8,
0x6F12, 0xE442,
0x6F12, 0xC0F8,
0x6F12, 0x3044,
0x6F12, 0x0846,
0x6F12, 0x00F0,
0x6F12, 0x8BFA,
0x6F12, 0xC749,
0x6F12, 0xB5F8,
0x6F12, 0xB022,
0x6F12, 0x088F,
0x6F12, 0x498F,
0x6F12, 0x201A,
0x6F12, 0x401E,
0x6F12, 0x1144,
0x6F12, 0x8142,
0x6F12, 0x00D9,
0x6F12, 0x0846,
0x6F12, 0xA5F8,
0x6F12, 0xB202,
0x6F12, 0x70BD,
0x6F12, 0x2DE9,
0x6F12, 0xF041,
0x6F12, 0x0646,
0x6F12, 0xBD48,
0x6F12, 0x0022,
0x6F12, 0x006A,
0x6F12, 0x85B2,
0x6F12, 0x040C,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0x1CFA,
0x6F12, 0x3046,
0x6F12, 0x00F0,
0x6F12, 0x73FA,
0x6F12, 0xBB48,
0x6F12, 0xBB4F,
0x6F12, 0x0068,
0x6F12, 0x3B68,
0x6F12, 0x418B,
0x6F12, 0x090A,
0x6F12, 0x83F8,
0x6F12, 0x3610,
0x6F12, 0xC17E,
0x6F12, 0x83F8,
0x6F12, 0x3810,
0x6F12, 0xB449,
0x6F12, 0x91F8,
0x6F12, 0x4C21,
0x6F12, 0x002A,
0x6F12, 0xD1F8,
0x6F12, 0x3421,
0x6F12, 0x01D0,
0x6F12, 0x521C,
0x6F12, 0x5208,
0x6F12, 0xCE33,
0x6F12, 0x160A,
0x6F12, 0x1E71,
0x6F12, 0x9A71,
0x6F12, 0xB1F8,
0x6F12, 0x3C21,
0x6F12, 0xC2F3,
0x6F12, 0x5712,
0x6F12, 0x1A70,
0x6F12, 0x91F8,
0x6F12, 0x3D21,
0x6F12, 0xD200,
0x6F12, 0x9A70,
0x6F12, 0x91F8,
0x6F12, 0x4D21,
0x6F12, 0xCE3B,
0x6F12, 0x22B1,
0x6F12, 0xD1F8,
0x6F12, 0x3821,
0x6F12, 0x521C,
0x6F12, 0x5608,
0x6F12, 0x01E0,
0x6F12, 0xD1F8,
0x6F12, 0x3861,
0x6F12, 0x7A68,
0x6F12, 0x4FEA,
0x6F12, 0x162C,
0x6F12, 0x01F5,
0x6F12, 0x9071,
0x6F12, 0x82F8,
0x6F12, 0x16C0,
0x6F12, 0x1676,
0x6F12, 0xCE8B,
0x6F12, 0x00F5,
0x6F12, 0xBA70,
0x6F12, 0xC6F3,
0x6F12, 0x5716,
0x6F12, 0x9674,
0x6F12, 0xCE7F,
0x6F12, 0xF600,
0x6F12, 0x1675,
0x6F12, 0x0E8C,
0x6F12, 0xCF68,
0x6F12, 0xF608,
0x6F12, 0x7E43,
0x6F12, 0x360B,
0x6F12, 0x370A,
0x6F12, 0x03F8,
0x6F12, 0xD67F,
0x6F12, 0x7732,
0x6F12, 0x9E70,
0x6F12, 0x0688,
0x6F12, 0x360A,
0x6F12, 0x02F8,
0x6F12, 0x276C,
0x6F12, 0x4678,
0x6F12, 0x02F8,
0x6F12, 0x256C,
0x6F12, 0x8688,
0x6F12, 0x360A,
0x6F12, 0x02F8,
0x6F12, 0x1F6C,
0x6F12, 0x4679,
0x6F12, 0x02F8,
0x6F12, 0x1D6C,
0x6F12, 0x8F4E,
0x6F12, 0x96F8,
0x6F12, 0x1064,
0x6F12, 0xD671,
0x6F12, 0x8D4E,
0x6F12, 0x96F8,
0x6F12, 0x1164,
0x6F12, 0x5672,
0x6F12, 0x8B4E,
0x6F12, 0x96F8,
0x6F12, 0x0B64,
0x6F12, 0xD672,
0x6F12, 0x894E,
0x6F12, 0x96F8,
0x6F12, 0x0964,
0x6F12, 0x5673,
0x6F12, 0x90F8,
0x6F12, 0x3060,
0x6F12, 0xD673,
0x6F12, 0x90F8,
0x6F12, 0xDE00,
0x6F12, 0x02F8,
0x6F12, 0x1F0F,
0x6F12, 0x8448,
0x6F12, 0x00F2,
0x6F12, 0x7246,
0x6F12, 0x90F8,
0x6F12, 0x7204,
0x6F12, 0x9074,
0x6F12, 0x3078,
0x6F12, 0x1075,
0x6F12, 0xA522,
0x6F12, 0xDA70,
0x6F12, 0x0E20,
0x6F12, 0x1871,
0x6F12, 0x11F8,
0x6F12, 0x7E0C,
0x6F12, 0xC0F1,
0x6F12, 0x0C01,
0x6F12, 0x7C48,
0x6F12, 0xD0F8,
0x6F12, 0x3C04,
0x6F12, 0xC840,
0x6F12, 0x060A,
0x6F12, 0x9E71,
0x6F12, 0x1872,
0x6F12, 0x0120,
0x6F12, 0x03F8,
0x6F12, 0x2C0C,
0x6F12, 0x7748,
0x6F12, 0xD0F8,
0x6F12, 0x4C04,
0x6F12, 0xC840,
0x6F12, 0xAA21,
0x6F12, 0x03F8,
0x6F12, 0x571D,
0x6F12, 0x0226,
0x6F12, 0x5E70,
0x6F12, 0x9A70,
0x6F12, 0x3022,
0x6F12, 0xDA70,
0x6F12, 0x5A22,
0x6F12, 0x1A71,
0x6F12, 0x060A,
0x6F12, 0x5E71,
0x6F12, 0x9A71,
0x6F12, 0xD871,
0x6F12, 0x1972,
0x6F12, 0x0020,
0x6F12, 0x5872,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0xBDE8,
0x6F12, 0xF041,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0x77B9,
0x6F12, 0x2DE9,
0x6F12, 0xF041,
0x6F12, 0x0746,
0x6F12, 0x6448,
0x6F12, 0x0C46,
0x6F12, 0x0022,
0x6F12, 0x406A,
0x6F12, 0x86B2,
0x6F12, 0x050C,
0x6F12, 0x3146,
0x6F12, 0x2846,
0x6F12, 0x00F0,
0x6F12, 0x6AF9,
0x6F12, 0x2146,
0x6F12, 0x3846,
0x6F12, 0x00F0,
0x6F12, 0xC5F9,
0x6F12, 0x6048,
0x6F12, 0x90F8,
0x6F12, 0x9702,
0x6F12, 0x10B9,
0x6F12, 0x00F0,
0x6F12, 0x97F9,
0x6F12, 0x20B1,
0x6F12, 0x04F1,
0x6F12, 0x8044,
0x6F12, 0xA08A,
0x6F12, 0x401C,
0x6F12, 0xA082,
0x6F12, 0x3146,
0x6F12, 0x2846,
0x6F12, 0xBDE8,
0x6F12, 0xF041,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0x53B9,
0x6F12, 0x2DE9,
0x6F12, 0xF041,
0x6F12, 0x0746,
0x6F12, 0x5248,
0x6F12, 0x0E46,
0x6F12, 0x0022,
0x6F12, 0x806A,
0x6F12, 0x85B2,
0x6F12, 0x040C,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0x46F9,
0x6F12, 0x3146,
0x6F12, 0x3846,
0x6F12, 0x00F0,
0x6F12, 0xA6F9,
0x6F12, 0x4B4F,
0x6F12, 0x4DF2,
0x6F12, 0x0C26,
0x6F12, 0x3437,
0x6F12, 0x4FF4,
0x6F12, 0x8061,
0x6F12, 0x3A78,
0x6F12, 0x3046,
0x6F12, 0x00F0,
0x6F12, 0x38F9,
0x6F12, 0x7878,
0x6F12, 0xC8B3,
0x6F12, 0x0022,
0x6F12, 0x4FF4,
0x6F12, 0x0071,
0x6F12, 0x3046,
0x6F12, 0x00F0,
0x6F12, 0x30F9,
0x6F12, 0x4848,
0x6F12, 0x0088,
0x6F12, 0x484B,
0x6F12, 0xA3F8,
0x6F12, 0x4402,
0x6F12, 0x4648,
0x6F12, 0x001D,
0x6F12, 0x0088,
0x6F12, 0xA3F8,
0x6F12, 0x4602,
0x6F12, 0xB3F8,
0x6F12, 0x4402,
0x6F12, 0xB3F8,
0x6F12, 0x4612,
0x6F12, 0x4218,
0x6F12, 0x02D0,
0x6F12, 0x8002,
0x6F12, 0xB0FB,
0x6F12, 0xF2F2,
0x6F12, 0x91B2,
0x6F12, 0x404A,
0x6F12, 0xA3F8,
0x6F12, 0x4812,
0x6F12, 0x5088,
0x6F12, 0x1288,
0x6F12, 0x3D4B,
0x6F12, 0xA3F8,
0x6F12, 0xA605,
0x6F12, 0xA3F8,
0x6F12, 0xA825,
0x6F12, 0x8018,
0x6F12, 0x05D0,
0x6F12, 0x9202,
0x6F12, 0xB2FB,
0x6F12, 0xF0F0,
0x6F12, 0x1A46,
0x6F12, 0xA2F8,
0x6F12, 0xAA05,
0x6F12, 0x3648,
0x6F12, 0xB0F8,
0x6F12, 0xAA05,
0x6F12, 0x0A18,
0x6F12, 0x01FB,
0x6F12, 0x1020,
0x6F12, 0x40F3,
0x6F12, 0x9510,
0x6F12, 0x1028,
0x6F12, 0x06DC,
0x6F12, 0x0028,
0x6F12, 0x05DA,
0x6F12, 0x0020,
0x6F12, 0x03E0,
0x6F12, 0xFFE7,
0x6F12, 0x0122,
0x6F12, 0xC3E7,
0x6F12, 0x1020,
0x6F12, 0x2F49,
0x6F12, 0x0880,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0xBDE8,
0x6F12, 0xF041,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0xEFB8,
0x6F12, 0x70B5,
0x6F12, 0x2148,
0x6F12, 0x0022,
0x6F12, 0xC16A,
0x6F12, 0x0C0C,
0x6F12, 0x8DB2,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0xE5F8,
0x6F12, 0x2148,
0x6F12, 0x0268,
0x6F12, 0xB2F8,
0x6F12, 0x6202,
0x6F12, 0x8301,
0x6F12, 0x92F8,
0x6F12, 0x6002,
0x6F12, 0x10F0,
0x6F12, 0x020F,
0x6F12, 0x09D0,
0x6F12, 0x1848,
0x6F12, 0x3430,
0x6F12, 0x8188,
0x6F12, 0x9942,
0x6F12, 0x06D8,
0x6F12, 0x4088,
0x6F12, 0xA0F5,
0x6F12, 0x5141,
0x6F12, 0x2339,
0x6F12, 0x01D1,
0x6F12, 0x00F0,
0x6F12, 0x38F9,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0xBDE8,
0x6F12, 0x7040,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0xC8B8,
0x6F12, 0x70B5,
0x6F12, 0x0646,
0x6F12, 0x0D48,
0x6F12, 0x0022,
0x6F12, 0x016B,
0x6F12, 0x0C0C,
0x6F12, 0x8DB2,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0x00F0,
0x6F12, 0xBDF8,
0x6F12, 0x3046,
0x6F12, 0x00F0,
0x6F12, 0x28F9,
0x6F12, 0x0749,
0x6F12, 0x114A,
0x6F12, 0x3431,
0x6F12, 0xCB79,
0x6F12, 0xD068,
0x6F12, 0x9840,
0x6F12, 0xD060,
0x6F12, 0x1068,
0x6F12, 0x9840,
0x6F12, 0x1060,
0x6F12, 0x8868,
0x6F12, 0x19E0,
0x6F12, 0x2000,
0x6F12, 0x0FE0,
0x6F12, 0x4000,
0x6F12, 0xF474,
0x6F12, 0x2000,
0x6F12, 0x4A40,
0x6F12, 0x2000,
0x6F12, 0x2850,
0x6F12, 0x2000,
0x6F12, 0x0E20,
0x6F12, 0x2000,
0x6F12, 0x2ED0,
0x6F12, 0x2000,
0x6F12, 0x08D0,
0x6F12, 0x2000,
0x6F12, 0x36E0,
0x6F12, 0x4000,
0x6F12, 0x9404,
0x6F12, 0x2000,
0x6F12, 0x38C0,
0x6F12, 0x4000,
0x6F12, 0xD214,
0x6F12, 0x4000,
0x6F12, 0xA410,
0x6F12, 0x2000,
0x6F12, 0x3254,
0x6F12, 0xD063,
0x6F12, 0x2946,
0x6F12, 0x2046,
0x6F12, 0xBDE8,
0x6F12, 0x7040,
0x6F12, 0x0122,
0x6F12, 0x00F0,
0x6F12, 0x8CB8,
0x6F12, 0x10B5,
0x6F12, 0x0022,
0x6F12, 0xAFF6,
0x6F12, 0x9701,
0x6F12, 0x3348,
0x6F12, 0x00F0,
0x6F12, 0xF8F8,
0x6F12, 0x334C,
0x6F12, 0x0022,
0x6F12, 0xAFF6,
0x6F12, 0x3F01,
0x6F12, 0x2060,
0x6F12, 0x3148,
0x6F12, 0x00F0,
0x6F12, 0xF0F8,
0x6F12, 0x6060,
0x6F12, 0xAFF2,
0x6F12, 0x4970,
0x6F12, 0x2F49,
0x6F12, 0x0022,
0x6F12, 0xC861,
0x6F12, 0xAFF2,
0x6F12, 0x9F61,
0x6F12, 0x2E48,
0x6F12, 0x00F0,
0x6F12, 0xE5F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x2D51,
0x6F12, 0x2061,
0x6F12, 0x2B48,
0x6F12, 0x00F0,
0x6F12, 0xDEF8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x2341,
0x6F12, 0x6061,
0x6F12, 0x2948,
0x6F12, 0x00F0,
0x6F12, 0xD7F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0xE931,
0x6F12, 0xA061,
0x6F12, 0x2648,
0x6F12, 0x00F0,
0x6F12, 0xD0F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x6B71,
0x6F12, 0xE061,
0x6F12, 0x2448,
0x6F12, 0x00F0,
0x6F12, 0xC9F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x2371,
0x6F12, 0xA060,
0x6F12, 0x2148,
0x6F12, 0x00F0,
0x6F12, 0xC2F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0xB731,
0x6F12, 0xE060,
0x6F12, 0x1F48,
0x6F12, 0x00F0,
0x6F12, 0xBBF8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x6121,
0x6F12, 0x2062,
0x6F12, 0x1C48,
0x6F12, 0x00F0,
0x6F12, 0xB4F8,
0x6F12, 0x6062,
0x6F12, 0x0020,
0x6F12, 0x04F1,
0x6F12, 0x3401,
0x6F12, 0x0246,
0x6F12, 0x8881,
0x6F12, 0xAFF2,
0x6F12, 0x3121,
0x6F12, 0x1848,
0x6F12, 0x00F0,
0x6F12, 0xA9F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x7511,
0x6F12, 0xA062,
0x6F12, 0x1548,
0x6F12, 0x00F0,
0x6F12, 0xA2F8,
0x6F12, 0x0022,
0x6F12, 0xAFF2,
0x6F12, 0x3711,
0x6F12, 0xE062,
0x6F12, 0x1348,
0x6F12, 0x00F0,
0x6F12, 0x9BF8,
0x6F12, 0x1249,
0x6F12, 0x2063,
0x6F12, 0x40F6,
0x6F12, 0xF100,
0x6F12, 0x0968,
0x6F12, 0x4883,
0x6F12, 0x10BD,
0x6F12, 0x0000,
0x6F12, 0x0000,
0x6F12, 0xDE1F,
0x6F12, 0x2000,
0x6F12, 0x4A40,
0x6F12, 0x0000,
0x6F12, 0x5F3B,
0x6F12, 0x2000,
0x6F12, 0x0850,
0x6F12, 0x0000,
0x6F12, 0xD719,
0x6F12, 0x0000,
0x6F12, 0x27FF,
0x6F12, 0x0000,
0x6F12, 0x39E3,
0x6F12, 0x0001,
0x6F12, 0x32CF,
0x6F12, 0x0001,
0x6F12, 0x1E3B,
0x6F12, 0x0000,
0x6F12, 0xEC45,
0x6F12, 0x0000,
0x6F12, 0x67B9,
0x6F12, 0x0000,
0x6F12, 0xE62B,
0x6F12, 0x0001,
0x6F12, 0x2265,
0x6F12, 0x0000,
0x6F12, 0x8C83,
0x6F12, 0x0000,
0x6F12, 0x5449,
0x6F12, 0x2000,
0x6F12, 0x08D0,
0x6F12, 0x4AF2,
0x6F12, 0x2B1C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x4DF6,
0x6F12, 0x1F6C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0x655C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x45F6,
0x6F12, 0x433C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x45F6,
0x6F12, 0xE36C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x46F2,
0x6F12, 0x7B1C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0xD90C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0x791C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0x811C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F2,
0x6F12, 0xB50C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x44F6,
0x6F12, 0xE90C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x43F6,
0x6F12, 0x155C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x43F6,
0x6F12, 0x1D5C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x4DF2,
0x6F12, 0xC95C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x42F2,
0x6F12, 0xFF7C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x48F2,
0x6F12, 0x712C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x43F6,
0x6F12, 0xE31C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x43F2,
0x6F12, 0x374C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x46F2,
0x6F12, 0xB97C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x4EF2,
0x6F12, 0x2B6C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x42F2,
0x6F12, 0x652C,
0x6F12, 0xC0F2,
0x6F12, 0x010C,
0x6F12, 0x6047,
0x6F12, 0x48F6,
0x6F12, 0x834C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x45F2,
0x6F12, 0x494C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
0x6F12, 0x4CF2,
0x6F12, 0x2D1C,
0x6F12, 0xC0F2,
0x6F12, 0x000C,
0x6F12, 0x6047,
};
#endif

static kal_uint16 addr_data_pair_preview[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x0900,//2304
	0x034E, 0x06C0,//1728
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_capture[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x0900,//2304
	0x034E, 0x06C0,//1728
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_normal_video[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x01C8,//456
	0x0348, 0x121F,//4639
	0x034A, 0x0BE7,//3047
	0x034C, 0x0900,//2304
	0x034E, 0x0510,//1296
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0094, //0080  0099
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,

};

static kal_uint16 addr_data_pair_hs_video[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0410,
	0x0346, 0x0408,
	0x0348, 0x0E1F,
	0x034A, 0x09A7,
	0x034C, 0x0500,
	0x034E, 0x02D0,
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0006,
	0x0306, 0x00E2,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x00A0, //0080  99
	0x0312, 0x0001,
	0x0340, 0x0395,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_slim_video[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0000,
	0x0346, 0x0008,
	0x0348, 0x122F,
	0x034A, 0x0DA7,
	0x034C, 0x0910,
	0x034E, 0x06D0,
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_custom1[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x0900,//2304
	0x034E, 0x06C0,//1728
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_custom2[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0000,
	0x0346, 0x01BC,  //Y Start
	0x0348, 0x122F,
	0x034A, 0x0BF3,  //Y End
	0x034C, 0x0910,  /// 2320
	0x034E, 0x051C,  //Y Output 1308
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};
static kal_uint16 addr_data_pair_custom3[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0100,
	0x602A, 0x1CF0,
	0x6F12, 0x0100,
	0x602A, 0x0E58,
	0x6F12, 0x0040,
	0x602A, 0x1694,
	0x6F12, 0x1B0F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x0007,
	0x602A, 0x1098,
	0x6F12, 0x000A,
	0x602A, 0x2690,
	0x6F12, 0x0000,
	0x6F12, 0x0055,
	0x602A, 0x16A8,
	0x6F12, 0x38CD,
	0x602A, 0x108C,
	0x6F12, 0x0003,
	0x602A, 0x10CC,
	0x6F12, 0x0008,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0020,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x1200,//4608
	0x034E, 0x0D80,//3456
	0x0350, 0x0008,
	0x0900, 0x0011,
	0x0380, 0x0001,
	0x0382, 0x0001,
	0x0384, 0x0001,
	0x0386, 0x0001,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x006E,
	0x0312, 0x0000,
	0x0340, 0x0E34,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};
static kal_uint16 addr_data_pair_custom4[] = {
	0x6028, 0x4000,
	0x6214, 0x7970,
	0x6218, 0x7150,
	0x6028, 0x2000,
	0x602A, 0x0ED6,
	0x6F12, 0x0000,
	0x602A, 0x1CF0,
	0x6F12, 0x0200,
	0x602A, 0x0E58,
	0x6F12, 0x0023,
	0x602A, 0x1694,
	0x6F12, 0x170F,
	0x602A, 0x16AA,
	0x6F12, 0x009D,
	0x6F12, 0x000F,
	0x602A, 0x1098,
	0x6F12, 0x0012,
	0x602A, 0x2690,
	0x6F12, 0x0100,
	0x6F12, 0x0000,
	0x602A, 0x16A8,
	0x6F12, 0x38C0,
	0x602A, 0x108C,
	0x6F12, 0x0002,
	0x602A, 0x10CC,
	0x6F12, 0x0001,
	0x602A, 0x10D0,
	0x6F12, 0x000F,
	0x602A, 0x0F50,
	0x6F12, 0x0200,
	0x602A, 0x1758,
	0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x0900,//2304
	0x034E, 0x06C0,//1728
	0x0350, 0x0004,
	0x0900, 0x0122,
	0x0380, 0x0002,
	0x0382, 0x0002,
	0x0384, 0x0002,
	0x0386, 0x0002,
	0x0404, 0x1000,
	0x0402, 0x1010,
	0x0400, 0x1010,
	0x0114, 0x0300,
	0x0110, 0x1002,
	0x0136, 0x1A00,
	0x0300, 0x0007,
	0x0302, 0x0001,
	0x0304, 0x0008,
	0x0306, 0x012B,
	0x0308, 0x0008,
	0x030A, 0x0001,
	0x030C, 0x0000,
	0x030E, 0x0004,
	0x0310, 0x0099, //0080
	0x0312, 0x0001,
	0x0340, 0x0E36,
	0x0342, 0x13E0,
	0x0202, 0x0100,
	0x0200, 0x0100,
	0x021E, 0x0000,
	0x0D00, 0x0000,
	0x0D02, 0x0001,
	0x6028, 0x2000,
	0x602A, 0x16A6,
	0x6F12, 0x006B,
};

static kal_uint16 addr_data_pair_custom5[] = {
    0x6028, 0x4000,
    0x6214, 0x7970,
    0x6218, 0x7150,
    0x6028, 0x2000,
    0x602A, 0x0ED6,
    0x6F12, 0x0000,
    0x602A, 0x1CF0,
    0x6F12, 0x0200,
    0x602A, 0x0E58,
    0x6F12, 0x0023,
    0x602A, 0x1694,
    0x6F12, 0x170F,
    0x602A, 0x16AA,
    0x6F12, 0x009D,
    0x6F12, 0x000F,
    0x602A, 0x1098,
    0x6F12, 0x0012,
    0x602A, 0x2690,
    0x6F12, 0x0100,
    0x6F12, 0x0000,
    0x602A, 0x16A8,
    0x6F12, 0x38C0,
    0x602A, 0x108C,
    0x6F12, 0x0002,
    0x602A, 0x10CC,
    0x6F12, 0x0001,
    0x602A, 0x10D0,
    0x6F12, 0x000F,
    0x602A, 0x0F50,
    0x6F12, 0x0200,
    0x602A, 0x1758,
    0x6F12, 0x0000,
	0x6028, 0x4000,
	0x0344, 0x0010,//16
	0x0346, 0x0018,//24
	0x0348, 0x121F,//4639
	0x034A, 0x0D97,//3479
	0x034C, 0x0900,//2304
	0x034E, 0x06C0,//1728
    0x0350, 0x0004,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0404, 0x1000,
    0x0402, 0x1010,
    0x0400, 0x1010,
    0x0114, 0x0300,
    0x0110, 0x1002,
    0x0136, 0x1A00,
    0x0300, 0x0007,
    0x0302, 0x0001,
    0x0304, 0x0008,
    0x0306, 0x012B,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0099,//0080
    0x0312, 0x0001,
    0x0340, 0x0E36,
    0x0342, 0x13E0,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x021E, 0x0000,
    0x0D00, 0x0000,
    0x0D02, 0x0001,
    0x6028, 0x2000,
    0x602A, 0x16A6,
    0x6F12, 0x006B,
};

static void sensor_init(void)
{
	/*Global setting */
	LOG_INF("start \n");
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x0000, 0x0900);
	write_cmos_sensor_16_16(0x0000, 0x310A);
	write_cmos_sensor_16_16(0x6010, 0x0001);
	mdelay(3);
	write_cmos_sensor_16_16(0x6214, 0x7970);
	write_cmos_sensor_16_16(0x6218, 0x7150);

#if USE_TNP_BURST
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x6004, 0x0001);
	write_cmos_sensor_16_16(0x6028, 0x2000);
	write_cmos_sensor_16_16(0x602A, 0x3F4C);
	LOG_INF("Using Burst Mode for TNP (%d)\n",(int)sizeof(uTnpArrayA));
	iWriteRegI2CTiming((u8*)uTnpArrayA , (u16)sizeof(uTnpArrayA), imgsensor.i2c_write_id, imgsensor_info.i2c_speed);
	write_cmos_sensor_16_16(0x6028, 0x4000);
	write_cmos_sensor_16_16(0x6004, 0x0000);
	//global
	table_write_cmos_sensor_consecutive(addr_data_pair_init_Abbreviations,
		   sizeof(addr_data_pair_init_Abbreviations) / sizeof(kal_uint16));
  #else
	table_write_cmos_sensor_consecutive(addr_data_pair_init,
		   sizeof(addr_data_pair_init) / sizeof(kal_uint16));
#endif
	LOG_INF("end \n");
}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_preview,
		   sizeof(addr_data_pair_preview) / sizeof(kal_uint16));
	LOG_INF("end \n");
}	/*	preview_setting  */

// Pll Setting - VCO = 280Mhz
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("start \n");

	table_write_cmos_sensor(addr_data_pair_capture,
		   sizeof(addr_data_pair_capture) / sizeof(kal_uint16));
	LOG_INF("end \n");
}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_normal_video,
		   sizeof(addr_data_pair_normal_video) / sizeof(kal_uint16));
	LOG_INF("end \n");
}


static void hs_video_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_hs_video,
		   sizeof(addr_data_pair_hs_video) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void slim_video_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_slim_video,
		   sizeof(addr_data_pair_slim_video) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void custom1_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_custom1,
		   sizeof(addr_data_pair_custom1) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void custom2_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_custom2,
		   sizeof(addr_data_pair_custom2) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void custom3_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_custom3,
		   sizeof(addr_data_pair_custom3) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void custom4_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor_consecutive(addr_data_pair_custom4,
		   sizeof(addr_data_pair_custom4) / sizeof(kal_uint16));
	LOG_INF("end \n");
}

static void custom5_setting(void)
{
	LOG_INF("start \n");
	table_write_cmos_sensor(addr_data_pair_custom5,
		   sizeof(addr_data_pair_custom5) / sizeof(kal_uint16));
	LOG_INF("end \n");
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
 extern int check_i2c_timeout(u16 addr, u16 i2cid);
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
#if 1
	kal_uint8 timeout = 0;
	timeout = check_i2c_timeout(0x310a, imgsensor.i2c_write_id);
    if (timeout) {
    	 pr_err(PFX "[%s] timeout =null \n",__FUNCTION__,timeout);
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
	}
#endif
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));

			if (*sensor_id == imgsensor_info.sensor_id) {

			 /*vivo hope add for AT cammand start*/
#if 0
				if (is_atboot == 1) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				LOG_INF("AT mode skip now return\n");
				return ERROR_NONE;
				}
#endif
				 /*vivo hope add for AT cammand end*/
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);
				/*vivo hope  add for CameraEM otp errorcode*/

				LOG_INF("cfx_add:start read eeprom ---vivo_otp_read_when_power_on = %d\n", vivo_otp_read_when_power_on);
				vivo_otp_read_when_power_on = SUB310A_otp_read();
				LOG_INF("cfx_add:end read eeprom ---vivo_otp_read_when_power_on = %d,S5K3P9SP04_OTP_ERROR_CODE =%d\n", vivo_otp_read_when_power_on, S5K3P9SP04_OTP_ERROR_CODE);
				/*vivo hope  add end*/
				return ERROR_NONE;

			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
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

	LOG_INF("%s", __func__);

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address*/
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
             sensor_id = ((read_cmos_sensor_16_8(0x0000) << 8) | read_cmos_sensor_16_8(0x0001));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init();

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
	LOG_INF("E\n");

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
	 LOG_INF("E\n");

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
 }
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

	return ERROR_NONE;
}				/*      get_resolution  */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*LOG_INF("get_info -> scenario_id = %d\n", scenario_id);*/

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
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}				/* control() */

static kal_uint32 set_video_mode(UINT16 framerate)
{
	/* //LOG_INF("framerate = %d\n ", framerate); */
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
	/*LOG_INF("scenario_id = %d\n", scenario_id);*/

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
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

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
static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	//INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/*struct SET_PD_BLOCK_INFO_T *PDAFinfo;*/
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_RAWINFO_STRUCT *rawinfo;
    	/*struct SENSOR_VC_INFO_STRUCT *pvcinfo;*/

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_INF("feature_id = %d\n", feature_id);*/
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
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE: 	/*ITS test_sensor_fusion */
  		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 1500000;
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
		write_cmos_sensor_16_16(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;

	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor_16_16(sensor_reg_data->RegAddr);
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
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM4:
		case MSDK_SCENARIO_ID_CUSTOM5:
			*feature_return_para_32 = 4; /*4sum*/
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
		default:
			*feature_return_para_32 = 1; /*BINNING_NONE,*/
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
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
		/* LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
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
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
					break;
	case SENSOR_FEATURE_GET_CUSTOM_INFO:
	    printk("SENSOR_FEATURE_GET_CUSTOM_INFO information type:%lld  S5K3P9SP04_OTP_ERROR_CODE:%d \n", *feature_data,S5K3P9SP04_OTP_ERROR_CODE);
		switch (*feature_data) {
			case 0:    //info type: otp state
			LOG_INF("*feature_para_len = %d, sizeof(MUINT32)*13 + 2 =%ld, \n", *feature_para_len, sizeof(MUINT32)*13 + 2);
			if (*feature_para_len >= sizeof(MUINT32)*13 + 2) {
			    *(MUINT32 *)(uintptr_t)((feature_data+1)) = S5K3P9SP04_OTP_ERROR_CODE;//otp_state
				memcpy( feature_data+2, sn_inf_sub_s5k3p9sp, sizeof(MUINT32)*13);
				memcpy( feature_data+10, material_inf_sub_s5k3p9sp, sizeof(MUINT32)*4);
				memcpy( feature_data+15, module_inf_sub_s5k3p9sp, sizeof(MUINT32)*2);
				#if 0
						for (i = 0 ; i<12 ; i++ ){
						printk("sn_inf_sub_s5k3p9sp[%d]= 0x%x\n", i, sn_inf_sub_s5k3p9sp[i]);
						}
				#endif
			}
				break;
			}
			break;
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
#if 0
	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		int type = (kal_uint16)(*feature_data);
		UINT32 i = 0;
		char *data = (char *)(*(feature_data+1));
		/*only copy Cross Talk calibration data*/
		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {

			data[0] = 0x00;
			data[1] = 0x08;
			for(i = 0; i< 2048 ; i++)
			data[2 + i] = crosstalk_data[1 + i];
		      pr_debug("read Cross Talk calibration data size= %d %d\n",
				data[0], data[1]);
		}
 	}
  	  break;
#endif
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
		case MSDK_SCENARIO_ID_CUSTOM5:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
			(imgsensor_info.custom5.pclk /
			(imgsensor_info.custom5.linelength - 80))*
			imgsensor_info.custom5.grabwindow_width;
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

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K3P9SP04PD2279F_MIPI_RAW_SensorInit(
	struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
