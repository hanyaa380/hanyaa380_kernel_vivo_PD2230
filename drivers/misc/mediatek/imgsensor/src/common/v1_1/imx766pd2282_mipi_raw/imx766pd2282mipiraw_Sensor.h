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
 *     IMX766PD2282mipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _IMX766PD2282MIPI_SENSOR_H
#define _IMX766PD2282MIPI_SENSOR_H


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
	IMGSENSOR_MODE_CUSTOM11,
};

enum {
	OTP_QSC_NONE = 0x0,
	OTP_QSC_INTERNAL,
	OTP_QSC_CUSTOM,
};

struct imgsensor_mode_struct {
	kal_uint32 pclk; /* record different mode's pclk */
	kal_uint32 linelength; /* record different mode's linelength */
	kal_uint32 framelength; /* record different mode's framelength */
	kal_uint32 cit_step_value;
	kal_uint32 cit_min_value;
	kal_uint8 startx; /* record different mode's startx of grabwindow */
	kal_uint8 starty; /* record different mode's startx of grabwindow */

	kal_uint16 grabwindow_width;
	kal_uint16 grabwindow_height;

/* for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario*/
	kal_uint8 mipi_data_lp2hs_settle_dc;

 /*     following for GetDefaultFramerateByScenario()    */
	kal_uint16 max_framerate;
	kal_uint32 mipi_pixel_rate;
};

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
struct imgsensor_struct {
	kal_uint8 mirror; /* mirrorflip information */

	kal_uint8 sensor_mode; /* record IMGSENSOR_MODE enum value */

	kal_uint32 shutter; /* current shutter */
	kal_uint16 gain; /* current gain */

	kal_uint32 pclk; /* current pclk */

	kal_uint32 frame_length; /* current framelength */
	kal_uint32 line_length; /* current linelength */

	kal_uint32 min_frame_length;
	kal_uint16 dummy_pixel; /* current dummypixel */
	kal_uint16 dummy_line; /* current dummline */

	kal_uint16 current_fps; /* current max fps */
	kal_bool autoflicker_en; /* record autoflicker enable or disable */
	kal_bool test_pattern; /* record test pattern mode or not */
	enum MSDK_SCENARIO_ID_ENUM current_scenario_id;
	kal_bool ihdr_en; /* ihdr enable or disable */
	kal_uint8 ihdr_mode; /* ihdr enable or disable */
	kal_uint8 pdaf_mode; /* ihdr enable or disable */
	kal_uint8 i2c_write_id; /* record current sensor's i2c write id */
	struct IMGSENSOR_AE_FRM_MODE ae_frm_mode;
	kal_uint8 current_ae_effective_frame;
};

/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
struct imgsensor_info_struct {
	kal_uint32 sensor_id; /* record sensor id defined in Kd_imgsensor.h */
	kal_uint32 checksum_value; /* checksum value for Camera Auto Test */
	struct imgsensor_mode_struct pre;
	struct imgsensor_mode_struct cap;
	struct imgsensor_mode_struct normal_video;
	struct imgsensor_mode_struct hs_video;
	struct imgsensor_mode_struct slim_video;
	struct imgsensor_mode_struct custom1;
	struct imgsensor_mode_struct custom2;
	struct imgsensor_mode_struct custom3;
	struct imgsensor_mode_struct custom4;
	struct imgsensor_mode_struct custom5;
	struct imgsensor_mode_struct custom6;
	struct imgsensor_mode_struct custom7;
	struct imgsensor_mode_struct custom8;
	struct imgsensor_mode_struct custom9;
	struct imgsensor_mode_struct custom10;
	struct imgsensor_mode_struct custom11;
	kal_uint8 ae_shut_delay_frame; /* shutter delay frame for AE cycle */
	kal_uint8 ae_sensor_gain_delay_frame;
	kal_uint8 ae_ispGain_delay_frame;
	kal_uint8 ihdr_support; /* 1, support; 0,not support */
	kal_uint8 ihdr_le_firstline; /* 1,le first ; 0, se first */
	kal_uint8 temperature_support;	/* 1, support; 0,not support */
	kal_uint8 sensor_mode_num; /* support sensor mode num */

	kal_uint8 cap_delay_frame; /* enter capture delay frame num */
	kal_uint8 pre_delay_frame; /* enter preview delay frame num */
	kal_uint8 video_delay_frame; /* enter video delay frame num */
	kal_uint8 hs_video_delay_frame;
	kal_uint8 slim_video_delay_frame; /* enter slim video delay frame num */
	kal_uint8 custom1_delay_frame; /* enter custom1 delay frame num */
	kal_uint8 custom2_delay_frame; /* enter custom2 delay frame num */
	kal_uint8 custom3_delay_frame; /* enter custom3 delay frame num */
	kal_uint8 custom4_delay_frame; /* enter custom4 delay frame num */
	kal_uint8 custom5_delay_frame; /* enter custom5 delay frame num */
	kal_uint8 custom6_delay_frame; /* enter custom5 delay frame num */
	kal_uint8 custom7_delay_frame; /* enter custom7 delay frame num */
	kal_uint8 custom8_delay_frame; /* enter custom8 delay frame num */
	kal_uint8 custom9_delay_frame; /* enter custom9 delay frame num */
	kal_uint8 custom10_delay_frame; /* enter custom10 delay frame num */
	kal_uint8 custom11_delay_frame; /* enter custom11 delay frame num */
	kal_uint8  frame_time_delay_frame;
	kal_uint8 margin; /* sensor framelength & shutter margin */
	kal_uint32 min_shutter; /* min shutter */
	kal_uint32 max_frame_length;
	kal_uint32 min_gain;
	kal_uint32 max_gain;
	kal_uint32 min_gain_iso;
	kal_uint32 gain_step;
	kal_uint32 exp_step;
	kal_uint32 gain_type;
	kal_uint8 isp_driving_current; /* mclk driving current */
	kal_uint8 sensor_interface_type; /* sensor_interface_type */
	kal_uint8 mipi_sensor_type;
	/* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2, default is NCSI2,
	 * don't modify this para
	 */
	kal_uint8 mipi_settle_delay_mode;
	/* 0, high speed signal auto detect;
	 * 1, use settle delay,unit is ns,
	 * default is auto detect, don't modify this para
	 */
	kal_uint8 sensor_output_dataformat;
	kal_uint8 mclk;	 /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	kal_uint32 i2c_speed; /* i2c speed */
	kal_uint8 mipi_lane_num; /* mipi lane num */
	kal_uint8 i2c_addr_table[5];
};

struct SEAMLESS_SYS_DELAY {
	uint32_t source_scenario;
	uint32_t target_scenario;
	uint32_t sys_delay;
};

struct gain_table {
	u32 permile;
	u16 code1;
	u16 code2;
	u16 code3;
	u16 code4;
//	u16 code5;
//	u16 code6;
//	u16 code7;
//	u16 code8;
};

/* fix OB problem  according to analog_gain */
static struct gain_table gains[] = {
	{640, 0x05, 0xA0, 0x01, 0x50},
	{1024, 0x05, 0xA0, 0x01, 0x50},
//	{7332, 0x01, 0xF6, 0x01, 0x13, 0x01, 0xD6, 0x01, 0xD6},
//	{12744, 0x01, 0xFC, 0x01, 0x13, 0x01, 0xD6, 0x01, 0xD6},
//	{14746, 0x01, 0xD8, 0x01, 0x5E, 0x01, 0xEA, 0x01, 0xEA},
//	{15360, 0x01, 0xD6, 0x01, 0x5E, 0x01, 0xEA, 0x01, 0xEA},
};

/* SENSOR READ/WRITE ID */
/* #define IMGSENSOR_WRITE_ID_1 (0x6c) */
/* #define IMGSENSOR_READ_ID_1  (0x6d) */
/* #define IMGSENSOR_WRITE_ID_2 (0x20) */
/* #define IMGSENSOR_READ_ID_2  (0x21) */

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData,
	u8 *a_pRecvData, u16 a_sizeRecvData,
		       u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);

extern void read_imx230_eeprom(void);
int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
	u16 transfer_length, u16 timing);

extern int iReadReg(u16 a_u2Addr, u8 *a_puBuff, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId,
				u16 transfer_length, u16 timing);
#endif
