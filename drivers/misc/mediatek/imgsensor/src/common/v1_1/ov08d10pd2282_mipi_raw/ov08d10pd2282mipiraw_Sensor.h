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
 *	 ov08d10pd2282mipiraw_Sensor.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _OV08D10PD2282MIPI_SENSOR_H
#define _OV08D10PD2282MIPI_SENSOR_H


enum IMGSENSOR_MODE {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
    IMGSENSOR_MODE_CUSTOM1,
    IMGSENSOR_MODE_CUSTOM2,
    IMGSENSOR_MODE_CUSTOM3,
    IMGSENSOR_MODE_CUSTOM4,
    IMGSENSOR_MODE_CUSTOM5,
    IMGSENSOR_MODE_CUSTOM6,
    IMGSENSOR_MODE_CUSTOM7,
    IMGSENSOR_MODE_CUSTOM8,
    IMGSENSOR_MODE_CUSTOM9,
    IMGSENSOR_MODE_CUSTOM10,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk;	/* record different mode's pclk */
	kal_uint32 linelength;	/* record different mode's linelength */
	kal_uint32 framelength;	/* record different mode's framelength */

	kal_uint8 startx; /* record different mode's startx of grabwindow */
	kal_uint8 starty; /* record different mode's startx of grabwindow */

	/* record different mode's width of grabwindow */
	kal_uint16 grabwindow_width;

	/* record different mode's height of grabwindow */
	kal_uint16 grabwindow_height;

	/* following for MIPIDataLowPwr2HighSpeedSettleDelayCount
	 * by different scenario
	 */
	kal_uint8 mipi_data_lp2hs_settle_dc;

	/*       following for GetDefaultFramerateByScenario()  */
	kal_uint16 max_framerate;
	kal_uint32 mipi_pixel_rate;

};

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
struct imgsensor_struct {
	kal_uint8 mirror;	/* mirrorflip information */

	kal_uint8 sensor_mode;	/* record IMGSENSOR_MODE enum value */

	kal_uint32 shutter;	/* current shutter */
	kal_uint16 gain;	/* current gain */

	kal_uint32 pclk;	/* current pclk */

	kal_uint32 frame_length;	/* current framelength */
	kal_uint32 line_length;	/* current linelength */

	/* current min	framelength to max framerate */
	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel;	/* current dummypixel */
	kal_uint16 dummy_line;	/* current dummline */

	kal_uint16 current_fps;	/* current max fps */
	kal_bool autoflicker_en; /* record autoflicker enable or disable */
	kal_bool test_pattern;	/* record test pattern mode or not */

	/* current scenario id */
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;

	/* ihdr mode 0: disable, 1: ihdr, 2:mVHDR, 9:zigzag */
	kal_uint8 ihdr_mode;

	kal_uint8 i2c_write_id;	/* record current sensor's i2c write id */
	struct IMGSENSOR_AE_FRM_MODE ae_frm_mode;
	kal_uint8 current_ae_effective_frame;
	kal_uint8  freq_setting;
	kal_uint8  present_freq_setting;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
struct imgsensor_info_struct {
	kal_uint32 sensor_id;	/* record sensor id defined in Kd_imgsensor.h */
	kal_uint32 checksum_value; /* checksum value for Camera Auto Test */

	/* preview scenario relative information */
	struct imgsensor_mode_struct pre;

	/* capture scenario relative information */
	struct imgsensor_mode_struct cap;

	/* capture for PIP 24fps relative information */
	struct imgsensor_mode_struct cap1;

	/* capture for PIP 24fps relative information */
	struct imgsensor_mode_struct cap2;

	/* normal video  scenario relative information */
	struct imgsensor_mode_struct normal_video;

	/* high speed video scenario relative information */
	struct imgsensor_mode_struct hs_video;

	/* slim video for VT scenario relative information */
	struct imgsensor_mode_struct slim_video;
    struct imgsensor_mode_struct custom1;	//custom1 scenario relative information
    struct imgsensor_mode_struct custom2;	//custom2 scenario relative information
    struct imgsensor_mode_struct custom3;	//custom3 scenario relative information
    struct imgsensor_mode_struct custom4;	//custom4 scenario relative information
    struct imgsensor_mode_struct custom5;	//custom5 scenario relative information
    struct imgsensor_mode_struct custom6;	//custom6 scenario relative information
    struct imgsensor_mode_struct custom7;	//custom7 scenario relative information
    struct imgsensor_mode_struct custom8;	//custom8 scenario relative information
    struct imgsensor_mode_struct custom9;	//custom9 scenario relative information
    struct imgsensor_mode_struct custom10;	//custom10 scenario relative information
	kal_uint8 ae_shut_delay_frame;	/* shutter delay frame for AE cycle */

	/* sensor gain delay frame for AE cycle */
	kal_uint8 ae_sensor_gain_delay_frame;

	/* isp gain delay frame for AE cycle */
	kal_uint8 ae_ispGain_delay_frame;
	kal_uint8 frame_time_delay_frame;
	kal_uint8 ihdr_support;	/* 1, support; 0,not support */
	kal_uint8 ihdr_le_firstline;	/* 1,le first ; 0, se first */
	kal_uint8 sensor_mode_num;	/* support sensor mode num */

	kal_uint8 cap_delay_frame;	/* enter capture delay frame num */
	kal_uint8 pre_delay_frame;	/* enter preview delay frame num */
	kal_uint8 video_delay_frame;	/* enter video delay frame num */

	/* enter high speed video  delay frame num */
	kal_uint8 hs_video_delay_frame;

	kal_uint8 slim_video_delay_frame; /* enter slim video delay frame num */
    kal_uint8  custom1_delay_frame;
    kal_uint8  custom2_delay_frame;
    kal_uint8  custom3_delay_frame;
    kal_uint8  custom4_delay_frame;
    kal_uint8  custom5_delay_frame;
    kal_uint8  custom6_delay_frame;
    kal_uint8  custom7_delay_frame;
    kal_uint8  custom8_delay_frame;
    kal_uint8  custom9_delay_frame;
    kal_uint8  custom10_delay_frame;

	kal_uint8 margin;	/* sensor framelength & shutter margin */
	kal_uint32 min_shutter;	/* min shutter */

	/* max framelength by sensor register's limitation */
	kal_uint32 max_frame_length;
	kal_uint32 min_gain;
	kal_uint32 max_gain;
	kal_uint32 min_gain_iso;
	kal_uint32 gain_step;
	kal_uint32 gain_type;
	kal_uint8 isp_driving_current; /* mclk driving current */
	kal_uint8 sensor_interface_type; /* sensor_interface_type */
	kal_uint8 mipi_sensor_type;
	/* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2,
	 * default is NCSI2, don't modify this para
	 */

	kal_uint8 mipi_settle_delay_mode;
	/* 0, high speed signal auto detect;
	 * 1, use settle delay,unit is ns,
	 * default is auto detect, don't modify this para
	 */

	kal_uint8 sensor_output_dataformat;
	kal_uint8 mclk;	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */

	kal_uint8 mipi_lane_num;	/* mipi lane num */

	/* record sensor support all write id addr,
	 * only supprt 4must end with 0xff
	 */
	kal_uint32 i2c_speed;
	kal_uint8 i2c_addr_table[5];

};

struct FREQ {
  kal_uint32 min;
  kal_uint32 max;
};

struct SENSOR_FREQ_LIST {
  struct FREQ *freq_list[4];
  kal_uint16 *setting;
  kal_uint32 setting_size;
};

/* SENSOR READ/WRITE ID */
/* #define IMGSENSOR_WRITE_ID_1 (0x6c) */
/* #define IMGSENSOR_READ_ID_1  (0x6d) */
/* #define IMGSENSOR_WRITE_ID_2 (0x20) */
/* #define IMGSENSOR_READ_ID_2  (0x21) */

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
				u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
/* extern bool read_2T7_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size); */

extern int iReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);




extern int iReadRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
				    u8 *a_pRecvData, u16 a_sizeRecvData,
				    u16 i2cId, u16 timing);


extern int iWriteRegI2CTiming(u8 *a_pSendData, u16 a_sizeSendData,
				     u16 i2cId, u16 timing);

extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
					u16 transfer_length, u16 timing);

struct gain_table {
	kal_int16 permile;
	kal_int8  code;
};

static struct gain_table gains[] = {
	{0x0040,0x10},
	{0x0044,0x11},
	{0x0048,0x12},
	{0x004C,0x13},
	{0x0050,0x14},
	{0x0054,0x15},
	{0x0058,0x16},
	{0x005C,0x17},
	{0x0060,0x18},
	{0x0064,0x19},
	{0x0068,0x1A},
	{0x006C,0x1B},
	{0x0070,0x1C},
	{0x0074,0x1D},
	{0x0078,0x1E},
	{0x007C,0x1F},
	{0x0080,0x20},
	{0x0088,0x22},
	{0x0090,0x24},
	{0x0098,0x26},
	{0x00A0,0x28},
	{0x00A8,0x2A},
	{0x00B0,0x2C},
	{0x00B8,0x2E},
	{0x00C0,0x30},
	{0x00C8,0x32},
	{0x00D0,0x34},
	{0x00D8,0x36},
	{0x00E0,0x38},
	{0x00E8,0x3A},
	{0x00F0,0x3C},
	{0x00F8,0x3E},
	{0x0100,0x40},
	{0x0110,0x44},
	{0x0120,0x48},
	{0x0130,0x4C},
	{0x0140,0x50},
	{0x0150,0x54},
	{0x0160,0x58},
	{0x0170,0x5C},
	{0x0180,0x60},
	{0x0190,0x64},
	{0x01A0,0x68},
	{0x01B0,0x6C},
	{0x01C0,0x70},
	{0x01D0,0x74},
	{0x01E0,0x78},
	{0x01F0,0x7C},
	{0x0200,0x80},
	{0x0220,0x88},
	{0x0240,0x90},
	{0x0260,0x98},
	{0x0280,0xA0},
	{0x02A0,0xA8},
	{0x02C0,0xB0},
	{0x02E0,0xB8},
	{0x0300,0xC0},
	{0x0320,0xC8},
	{0x0340,0xD0},
	{0x0360,0xD8},
	{0x0380,0xE0},
	{0x03A0,0xE8},
	{0x03C0,0xF0},
	{0x03E0,0xF8},
};


#endif
/* _OV08D10PD2282MIPI_SENSOR_H */
