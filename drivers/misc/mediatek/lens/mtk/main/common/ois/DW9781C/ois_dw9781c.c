/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ois_dw9781c.h"

#define I2COP_CHECK(cond) \
	if (cond) {                   \
		LOG_OIS_ERR("ois i2c fail(%d)", cond); \
		goto p_err; \
	}

//current specific fw
/*const struct ois_fw_info EXP_FW = {
	.version = 0x0207,
	.type    = FW_VIVO,
	.date    = 0x0130
};*/

#define GAIN_TABLE_SIZE_MAX 100
#if defined(CONFIG_MTK_CAM_PD2215F_EX)
const struct ois_fw_info EXP_FW = {
	.version = 0x0a05,
	.type    = FW_VIVO,
	.date    = 0x0629
};
#define VIVO_OTP_DATA_SIZE 0x2F40
extern unsigned char vivo_otp_data_s5kgw3sp13pd2215f[VIVO_OTP_DATA_SIZE];
#elif defined(CONFIG_MTK_CAM_PD2204F_EX)
//current specific fw
const struct ois_fw_info EXP_FW = {
	.version = 0x080A,
	.type    = FW_VIVO,
	.date    = 0x0414
};
#define VIVO_OTP_DATA_SIZE 0x2F40
extern unsigned char vivo_otp_data_s5kgw1sd03pd2204f[VIVO_OTP_DATA_SIZE];
#elif defined(CONFIG_MTK_CAM_PD2257F_EX)
//current specific fw
const struct ois_fw_info EXP_FW = {
	.version = 0x0f05,
	.type    = FW_VIVO,
	.date    = 0x1216
};
#define VIVO_OTP_DATA_SIZE 0x2EE0
extern unsigned char vivo_otp_data_s5kgw3sp13pd2257f[VIVO_OTP_DATA_SIZE];
#endif

extern int log_ois_level;

static int dw9781c_acc_gain_table[GAIN_TABLE_SIZE_MAX] = {
    0,     0,     0,     0,     0,     0,     0,     0,     27000, 24000,
    21600, 19636, 18000, 16615, 15429, 14400, 13500, 12706, 12000, 11368,
    10800, 10286, 9818,  9391,  9000,  8640,  8308,  8000,  7714,  7448,
    7200,  6968,  6750,  6545,  6353,  6171,  6000,  5838,  5684,  5538,
    5400,  5268,  5143,  5023,  4909,  4800,  4696,  4596,  4500,  4408,
    4320,  4235,  4154,  4075,  4000,  3927,  3857,  3789,  3724,  3661,
    3600,  3541,  3484,  3429,  3375,  3323,  3273,  3224,  3176,  3130,
    3086,  3042,  3000,  2959,  2919,  2880,  2842,  2805,  2769,  2734,
    2700,  2667,  2634,  2602,  2571,  2541,  2512,  2483,  2455,  2427,
    2400,  2374,  2348,  2323,  2298,  2274,  2250,  2227,  2204,  2182
};

#ifdef CONFIG_MTK_CAM_PD2215F_EX
static int dw9781c_acc_gain_table_pd2215[GAIN_TABLE_SIZE_MAX] = {
    0,     0,     0,     0,     0,     0,     0,     0,     0,         0,
    23400, 21273, 19500, 18000, 16714, 15600, 14625, 13765, 13000, 12316,
    11700, 11143, 10636,  10174,  9750,  9360,  9000,  8667,  8357,  8069,
    7800,  7548,  7313,  7091,  6882,  6686,  6500,  6324,  6158,  6000,
    5850,  5707,  5571,  5442,  5318,  5200,  5087,  4979,  4875,  4776,
    4680,  4588,  4500,  4415,  4333,  4255,  4179,  4105,  4034,  3966,
    3900,  3836,  3774,  3714,  3656,  3600,  3545,  3493,  3441,  3391,
    3343,  3296,  3250,  3205,  3162,  3120,  3079,  3039,  3000,  2962,
    2925,  2889,  2854,  2819,  2786,  2753,  2721,  2690,  2659,  2629,
    2600,  2571,  2543,  2516,  2489,  2483,  2412,  2388,  2364,  2340
};
#endif

#ifdef CONFIG_MTK_CAM_PD2257F_EX
static int dw9781c_acc_gain_table_pd2257[GAIN_TABLE_SIZE_MAX] = {
    0,     0,     0,     0,     0,     0,     0,     0,     0,         0,
    23400, 21273, 19500, 18000, 16714, 15600, 14625, 13765, 13000, 12316,
    11700, 11143, 10636,  10174,  9750,  9360,  9000,  8667,  8357,  8069,
    7800,  7548,  7313,  7091,  6882,  6686,  6500,  6324,  6158,  6000,
    5850,  5707,  5571,  5442,  5318,  5200,  5087,  4979,  4875,  4776,
    4680,  4588,  4500,  4415,  4333,  4255,  4179,  4105,  4034,  3966,
    3900,  3836,  3774,  3714,  3656,  3600,  3545,  3493,  3441,  3391,
    3343,  3296,  3250,  3205,  3162,  3120,  3079,  3039,  3000,  2962,
    2925,  2889,  2854,  2819,  2786,  2753,  2721,  2690,  2659,  2629,
    2600,  2571,  2543,  2516,  2489,  2483,  2412,  2388,  2364,  2340
};
#endif

static int dw9781c_log_control(struct ois *ois, int level)
{
	int ret = 0;

	if (level > OIS_LOG_START && level < OIS_LOG_END)
		log_ois_level = level;

	LOG_OIS_INF("log %d", log_ois_level);
	return ret;
}

static int dw9781c_reset(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	/*1.logic circuit reset*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
	I2COP_CHECK(ret);
	mdelay(4);
	/*2.dsp on, wait 25ms for gyro data stable*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, ON);
	I2COP_CHECK(ret);
	mdelay(25);
	/*3.get user write register authority*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_USER_WRITE_PROTECT, USER_WRITE_EN);
	I2COP_CHECK(ret);
	mdelay(10);

	LOG_OIS_INF("ois reset success(%d)", ret);

p_err:
	return ret;
}

#if defined(CONFIG_MTK_CAM_PD2083F_EX)
static int dw9781c_set_spi_mode(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	u16 imu_read = ON, spi_mode = OIS_SPI_MASTER;

	OIS_BUG(!client);

	if (ois->slave_mode) {
		imu_read = OFF;
		spi_mode = OIS_SPI_INPUT;
	}

	LOG_OIS_INF("set read %d spi %04x", imu_read, spi_mode);

	ret = ois_i2c_write(client, DW9781C_REG_GYRO_READ_CTRL, imu_read);
	mdelay(10);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_SPI_MODE, spi_mode);
	I2COP_CHECK(ret);
	mdelay(55);

	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &spi_mode);
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &imu_read);

	LOG_OIS_INF("X actual read %d spi %d", imu_read, spi_mode);
p_err:
	return ret;
};
#endif

static int dw9781c_set_mode(struct ois *ois, int mode, int is_internal)
{
	int ret       = 0;
	u16 exp_mode  = 0x0000;
	u16 old_mode  = 0x0000;
	u16 new_mode  = 0x0000;
	u16 servo     = 0x0000;
	u16 op_status = OPERATE_DONE;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &old_mode);

	if (!is_internal) {
		switch (mode) {
		case OIS_CENTER_MODE: {
			exp_mode = DW9781C_CENTERING_MODE;
			ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINX, OFF);
			ret = ois_i2c_write(client, DW9781C_REG_OIS_ACC_GAINY, OFF);
			I2COP_CHECK(ret);
			break;
		}
		case OIS_STILL_MODE: {
			exp_mode = DW9781C_STILL_MODE;
			break;
		}
		case OIS_VIDEO_MODE: {
			exp_mode = DW9781C_VIDEO_MODE;
			break;
		}
		case OIS_ZOOM_MODE: {
			exp_mode = DW9781C_ZOOM_MODE;
			break;
		}
		case OIS_FIX_MODE: {
			ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, SERVO_ON);
			ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETX, 0x0000);
			I2COP_CHECK(ret);
			ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETY, 0x0000);
			I2COP_CHECK(ret);
			break;
		}
		case OIS_CIRCLEWAVE_MODE: {
			exp_mode = DW9781C_CIRCLEWAVE_MODE;
			break;
		}
		default: {
			LOG_OIS_INF("unsupport ois mode(%d)", mode);
			goto p_err;
		}
		}
	} else
		exp_mode = mode;

	if (0x0000 != exp_mode && old_mode != exp_mode) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, exp_mode);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_CMD_STATUS, OPERATE_START);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CMD_STATUS, &op_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &new_mode);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &servo);
	ois->flash_info->mode = new_mode;
	ois->flash_info->serveOn = servo;

	LOG_OIS_INF("internal(%d) mode(old:0x%04x exp:0x%04x new:0x%04x servo:0x%04x), op status(0x%04x) result(%d)",
		is_internal, old_mode, exp_mode, new_mode, servo, op_status, ret);

p_err:
	return ret;
}

static int dw9781c_start_ois(struct ois *ois)
{
	int ret        = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	//ret = ois_i2c_write(client, DW9781C_REG_OIS_OSC_CTL, REG_DEAFULT_ON);
	//I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_TRIPODE_CTRL, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, OIS_ON);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_MODE, DW9781C_STILL_MODE);
	I2COP_CHECK(ret);

#if defined(CONFIG_MTK_CAM_PD2257F_EX)
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_DERGEEX, DW9781C_PANTILT_1DU);
	I2COP_CHECK(ret);
#endif

	LOG_OIS_INF("ois restart success(%d)", ret);

p_err:
	return ret;
}

static int dw9781c_get_init_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	u16 u16_rxdata = 0x0000;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &u16_rxdata);
	flash_info->fwInfo.version = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &u16_rxdata);
	flash_info->fwInfo.type = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &u16_rxdata);
	flash_info->fwInfo.date = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_CTRL, &u16_rxdata);
	flash_info->hallInfo.pantiltOn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_IMU_SELECT, &u16_rxdata);
	flash_info->imuInfo.imuType = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &s16_rxdata);
	flash_info->imuInfo.gyroOffsetY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINX, &s16_rxdata);
	flash_info->imuInfo.gyroGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINY, &s16_rxdata);
	flash_info->imuInfo.gyroGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &u16_rxdata);
	flash_info->smoothInfo.on = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_STEP, &u16_rxdata);
	flash_info->smoothInfo.step = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_DELAY, &u16_rxdata);
	flash_info->smoothInfo.delay = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_AF_DRIFT_COMP, &s16_rxdata);
	flash_info->driftCompOn = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_CTRL, &u16_rxdata);
	LOG_OIS_INF("len info sample ctrl 0x%04x", u16_rxdata);

	if (OFF != flash_info->imuInfo.accGainX)
		flash_info->accOn = ON;

	ret = copy_to_user(user_buf, flash_info, sizeof(struct ois_flash_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy flash info, ret(%d)\n", ret);
		goto p_err;
	}

	LOG_OIS_INF("flash info:fw(ver:0x%04x date:0x%04x type:0x%04x) gyro(type:0x%04x spi:0x%04x readEn:0x%04x) pantilt(%d,limitx:%d,limity:%d) offset(%d,%d),gain(%d,%d,%d,%d) smooth(%d %d %d)",
		flash_info->fwInfo.version, flash_info->fwInfo.date, flash_info->fwInfo.type,
		flash_info->imuInfo.imuType, flash_info->imuInfo.spiMode, flash_info->imuInfo.imuReadEn,
		flash_info->hallInfo.pantiltOn, flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY,
		flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY, flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		flash_info->smoothInfo.on, flash_info->smoothInfo.step, flash_info->smoothInfo.delay);
	LOG_OIS_INF("control info: chip(0x%04x) dsp(0x%04x) writeAthr(0x%04x) reset(0x%04x) tripod(0x%04x) mode(0x%04x) acc(0x%04x) drift(0x%04x)",
		flash_info->chipEn, flash_info->dspEn, flash_info->writeAuthority, flash_info->logicReset,
		flash_info->tripodFlag, flash_info->imuInfo.spiMode, flash_info->accOn, flash_info->driftCompOn);

    #if defined(CONFIG_MTK_CAM_PD2204F_EX)
	mutex_lock(&ois->op_lock);
	ois->deletefirstdata = 2;
	mutex_unlock(&ois->op_lock);
    #endif

p_err:
	return ret;
}

static int dw9781c_init(struct ois *ois)
{
	int               ret     = 0;
	struct i2c_client *client = ois->client;
//	u16 fw_checksum_flag = 0x0000, checksum_cal = 0x0000, checksum_exp = 0x0000;
	u16 init_fw_ver;

	LOG_OIS_INF("E");

	OIS_BUG(!client);
	OIS_BUG(!(ois->flash_info));
/*#if defined(CONFIG_MTK_CAM_PD2204F_EX)
	code = OIS_BIG_ENDIAN_TRANS4(vivo_otp_data_s5kgw1sd03pd2204f, 0x000F);	
#endif
*/
	ois->client->addr = DW7981C_SLAVE_ADDR >> 1;
	ois->flash_info->chipEn = 0x01;
	ois->deletefirstdata = 0x00;


	/*//1.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//2.flash checksum flag check
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_FLAG, &fw_checksum_flag);
	if (FW_CHECKSUM_FLAG != fw_checksum_flag) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x", fw_checksum_flag);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		mdelay(4);
		I2COP_CHECK(ret);
		ois->flash_info->chipEn = 0x00;
	}

	//3.logic circuit reset
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);
	//4.flash checksum check
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_CAL, &checksum_cal);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_STORED, &checksum_exp);
	if (checksum_cal != checksum_exp) {
		LOG_OIS_ERR("ic checksum ng: flag 0x%04x checksum exp 0x%04x checksum cal 0x%04x",
			fw_checksum_flag, checksum_exp, checksum_cal);
		ois->flash_info->chipEn = 0x00;
	}*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_INIT_FW_VER, &init_fw_ver);
	I2COP_CHECK(ret);
	if(init_fw_ver == EXP_FW.version) {
		ret = dw9781c_reset(ois);
		I2COP_CHECK(ret);
		LOG_OIS_INF("fw ver is ok:%d", init_fw_ver);
	} else {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		I2COP_CHECK(ret);
		mdelay(10);
		ois->flash_info->chipEn = 0x00;
		LOG_OIS_INF("fw ver is bad:%d", init_fw_ver);
	}
	/*6.enable o+e data sample*/
	ret = ois_i2c_write(client, DW9781C_REG_INFO_SAMPLE_CTRL, ((ENABLE_HALL << 8) | DW9871C_DEFAULT_SAMPLE_FREQ));
	I2COP_CHECK(ret);
	/*7.retsart ois*/
	ret = dw9781c_start_ois(ois);
	I2COP_CHECK(ret);
	/*7.pjt name*/
	ois->pjt_name = 0;
#ifdef CONFIG_MTK_CAM_PD2215F_EX
	ois->pjt_name = 2215;
#elif defined(CONFIG_MTK_CAM_PD2204F_EX)
	ois->pjt_name = 2204;
#elif defined(CONFIG_MTK_CAM_PD2257F_EX)
	ois->pjt_name = 2257;
#endif
	LOG_OIS_INF("X chip 0x%04x", ois->flash_info->chipEn);
p_err:
	return ret;
}

static int dw9781c_deinit(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
#ifdef CONFIG_MTK_CAM_PD2083F_EX
	u16 XDD_times = 0x0000;
	u16 YDD_times = 0x0000;
#endif

#if defined(CONFIG_MTK_CAM_PD2215F_EX) || defined(CONFIG_MTK_CAM_PD2257F_EX)
	u16 XDD_times = 0x0000;
	u16 YDD_times = 0x0000;
#endif
	LOG_OIS_INF("E");

	OIS_BUG(!client);
#if defined(CONFIG_MTK_CAM_PD2215F_EX) || defined(CONFIG_MTK_CAM_PD2257F_EX)
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_XCOUNT, &XDD_times);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_YCOUNT, &YDD_times);
	LOG_OIS_INF("Divergence detection times(X: %d Y: %d)", XDD_times, YDD_times);
#endif
/*
#ifdef CONFIG_MTK_CAM_PD2204F_EX
	u16 XDD_times = 0x0000;
	u16 YDD_times = 0x0000;
#endif
	LOG_OIS_INF("E");

	OIS_BUG(!client);
#ifdef CONFIG_MTK_CAM_PD2204F_EX
	ret = ois_i2c_read(client, DW9781C_REG_OIS_DD_XTIMES, &XDD_times);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_DD_YTIMES, &YDD_times);
	LOG_OIS_INF("Divergence detection times(X: %d Y: %d)", XDD_times, YDD_times);
#endif
*/
	/*2.servo off*/
	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, SERVO_OFF);
	I2COP_CHECK(ret);

	ois->flash_info->chipEn = 0x00;

	LOG_OIS_INF("X(%d)", ret);
p_err:
	return ret;
}

static int dw9781c_stream_on(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static int dw9781c_stream_off(struct ois *ois)
{
	int ret = 0;
	return ret;
}

static void dw9781c_vsync_process_fn(struct kthread_work *work)
{
	int result = 0;
	struct ois *ois = NULL;
	struct i2c_client *client = NULL;
	//u16 sample_ready = 0x0000;
	struct ois_lens_info *lens_info = NULL;
	u8 insert_idx = 0;
	u16 group_idx = 0;
	//u8 data[DW9781C_MAX_LENS_INFO_SIZE] = {0, };
	//u16 data_idx = 0, group_idx = 0;
	//u16 valid_size = 0;
	struct timespec start_time, end_time;
	unsigned short bufferReady = 0;
	unsigned short readBuf[62]= {0};


	OIS_BUG_VOID(!work);


	LOG_OIS_VERB("ois vsync processor E");


	ois = container_of(work, struct ois, vsync_work);
	OIS_BUG_VOID(!ois);
	client = ois->client;
	OIS_BUG_VOID(!client);
	OIS_BUG_VOID(!work);
	OIS_BUG_VOID(!(ois->lens_info_buf));

	insert_idx = ois->lens_info_buf->insertidx;
	lens_info = &ois->lens_info_buf->buf[insert_idx];

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	#if defined(CONFIG_MTK_CAM_PD2204F_EX)
	if (ois->deletefirstdata == 2)
	{
		LOG_OIS_INF("ois_deletefirstdata!");
		ois->deletefirstdata = 1;
		goto p_err;
	}
	#endif
	memset(lens_info, 0x00, sizeof(struct ois_lens_info));

	lens_info->validnum = 10;
	lens_info->fcount = ois->vsync_info.ois_vsync_cnt;
	lens_info->timestampboot = ois->vsync_info.sof_timestamp_boot;
	ois->vsync_info.ois_vsync_cnt++;

	//check if data ready and read first packet(62 bytes)
#if defined(CONFIG_MTK_CAM_PD2215F_EX) || defined(CONFIG_MTK_CAM_PD2257F_EX)
	mdelay(1);
#endif
	result = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_READY, &bufferReady);
	result = ois_i2c_read_block(client, DW9781C_REG_LENS_INFO_START, (u8*)(readBuf), 62);
	//I2COP_CHECK(ret);
	LOG_OIS_INF("DW9781C bufferReady=%d readable Sampling data(hall) %04x", bufferReady, readBuf[0]);

	if ((bufferReady & 0x01) == 0 || (!readBuf[0])) {
		LOG_OIS_INF("DW9781 invalid O+E data \n");
		goto p_err;
	}
	for (group_idx = 0; group_idx < lens_info->validnum; ++group_idx) {
		lens_info->hallx[group_idx] = OIS_UINT_TO_INT(OIS_BIG_ENDIAN_TRANS3(readBuf[group_idx * 3 + 2]));
		lens_info->hally[group_idx] = OIS_UINT_TO_INT(OIS_BIG_ENDIAN_TRANS3(readBuf[group_idx * 3 + 3]));
		lens_info->ic_timecount[group_idx] = OIS_BIG_ENDIAN_TRANS3(readBuf[group_idx * 3 + 1]);
		if (lens_info->ic_timecount[group_idx] == 0 && lens_info->hallx[group_idx]== 0) {
			lens_info->validnum = group_idx;
			break;
		}
		LOG_OIS_VERB("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
				lens_info->fcount, group_idx,  lens_info->ic_timecount[group_idx],
				lens_info->hallx[group_idx], lens_info->hally[group_idx]);
	}

	/*result = ois_i2c_read(client, DW9781C_REG_INFO_SAMPLE_READY, &sample_ready);
	result = ois_i2c_read(client, DW9781C_REG_LENS_INFO_START, (u16 *)(&data[0]));
	valid_size = data[0] * 6 + 2;
	if (0x01 != sample_ready || !data[0] || valid_size > DW9781C_MAX_LENS_INFO_SIZE) {
		LOG_OIS_INF("skip: sample_ready=%d valid size=%d", sample_ready, valid_size);
		goto p_err;
	}
	lens_info->validnum = (data[0] > LENS_INFO_GROUPS_MAX) ? LENS_INFO_GROUPS_MAX : data[0];
	lens_info->fcount = ois->vsync_info.ois_vsync_cnt;
	lens_info->timestampboot = ois->vsync_info.sof_timestamp_boot;
	LOG_OIS_VERB("info ready=0x%04x, isp vsync %llu fcount=%llu ts=%llu validnum=%d insert=%d",
		sample_ready, ois->vsync_info.vsync_cnt, lens_info->fcount, lens_info->timestampboot, lens_info->validnum, insert_idx);
	ois->vsync_info.ois_vsync_cnt++;
	//if there are more data
	while (data_idx < valid_size - 1) {
		result = ois_i2c_read_block(client, DW9781C_REG_LENS_INFO_START, &data[data_idx], DW9781C_LENS_PACKET_SIZE);
		result = ois_i2c_write(ois->client, DW9781C_REG_INFO_SAMPLE_READY, OFF);//inform ic one packet read done
		I2COP_CHECK(result);
		mdelay(1);
		data_idx += DW9781C_LENS_PACKET_SIZE;
	}

	//LOG_OIS_INF("read done(idx=%d)", data_idx);

	for (group_idx = 0, data_idx = 2; group_idx < lens_info->validnum; group_idx++) {
		lens_info->ic_timecount[group_idx] = (!group_idx) ? (100 * (OIS_BIG_ENDIAN_TRANS2(data, data_idx))) :
			(OIS_BIG_ENDIAN_TRANS2(data, data_idx));//onvert to us
		data_idx += 2;
		lens_info->hallx[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		lens_info->hally[group_idx] = (s16)(OIS_BIG_ENDIAN_TRANS2(data, data_idx));
		data_idx += 2;
		LOG_OIS_VERB("fcount %lu data[%d] timestamp %lu hallx %d hally %d",
			lens_info->fcount, group_idx,  lens_info->ic_timecount[group_idx],
			lens_info->hallx[group_idx], lens_info->hally[group_idx]);
	}*/

	if (++ois->lens_info_buf->insertidx == LENS_INFO_FRAMES_MAX)
			ois->lens_info_buf->insertidx = 0;

	ktime_get_ts(&end_time);
	LOG_OIS_VERB("ois vsync processor X %llu ms vnum:%d",
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)), lens_info->validnum);
p_err:
	mutex_unlock(&ois->op_lock);
	return;
}

static int dw9781c_vsync_signal(struct ois *ois, void *buf)
{
	int ret = 0;
	struct timespec curtime;
	struct ois_vsync_info *tempInfo = NULL;
	u64 module_idx = 0, vsync_cnt = 0;

	OIS_BUG(!buf);
	OIS_BUG(!(ois->lens_info_buf));

	tempInfo = (struct ois_vsync_info *)buf;
	module_idx = tempInfo->module_idx;
	vsync_cnt = tempInfo->vsync_cnt;

	if (OIS_VSYNC_TYPE_INT_CAM_A_ST != module_idx) {
		LOG_OIS_VERB("module %lu skip vsync %lu process", module_idx, vsync_cnt);
		return ret;
	}
	curtime = ktime_to_timespec(tempInfo->sof_timestamp_boot);
	ois->vsync_info.sof_timestamp_boot = (u64)curtime.tv_sec * 1000000000 + curtime.tv_nsec;
	ois->vsync_info.vsync_cnt = vsync_cnt;
	ois->vsync_info.module_idx = module_idx;

	if (ois->vsync_task != NULL) {
		kthread_queue_work(&ois->vsync_worker, &ois->vsync_work);
	}

	LOG_OIS_VERB("module %lu isp vsync %llu timestamp %llu", module_idx, vsync_cnt, ois->vsync_info.sof_timestamp_boot);

	return ret;
}

static int dw9781c_init_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (ois->vsync_task == NULL) {
		spin_lock_init(&ois->ois_vsync_lock);
		kthread_init_work(&ois->vsync_work, dw9781c_vsync_process_fn);
		kthread_init_worker(&ois->vsync_worker);
		ois->vsync_task = kthread_run(kthread_worker_fn, &ois->vsync_worker, "vsync_processor");
		if (IS_ERR_OR_NULL(ois->vsync_task)) {
			LOG_OIS_ERR("failed to create vsync processor, err(%ld)", PTR_ERR(ois->vsync_task));
			ret = PTR_ERR(ois->vsync_task);
			ois->vsync_task = NULL;
			goto p_err;
		}
	}
	LOG_OIS_INF("start ois vsync processor success");
p_err:
	return ret;
}

static int dw9781c_deinit_vsync_thread(struct ois *ois)
{
	int ret = 0;

	if (!IS_ERR_OR_NULL(ois->vsync_task)) {
		if (kthread_stop(ois->vsync_task)) {
			LOG_OIS_ERR("vsync processor stop fail");
			goto p_err;
		}
		ois->vsync_task = NULL;
	}
	LOG_OIS_INF("stop ois vsync processor success");
p_err:
	return ret;
}

static int dw9781c_get_lens_info(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	//u8  idx = 0;
	struct timespec start_time, end_time;
	//struct ois_lens_info_buf info_buf = {0};

	OIS_BUG(!user_buf);

	//get hal frame id
	//ret = copy_from_user(&info_buf, user_buf, sizeof(struct ois_lens_info_buf));
	//LOG_OIS_INF("read hal frame %llu", info_buf.buf[0].frame_id);

	ktime_get_ts(&start_time);

	mutex_lock(&ois->op_lock);
	ret = copy_to_user(user_buf, ois->lens_info_buf, sizeof(struct ois_lens_info_buf));
	if (ret) {
		LOG_OIS_ERR("fail to copy lens info, ret(%d)\n", ret);
	}

	ktime_get_ts(&end_time);
	/*
    for (idx = 0, idx = 2; idx < LENS_INFO_FRAMES_MAX; idx++)
		LOG_OIS_INF("fcount %lu data[0] timestamp %lu hallx %d hally %d",
			ois->lens_info_buf->buf[idx].fcount, ois->lens_info_buf->buf[idx].ic_timecount[0],
			ois->lens_info_buf->buf[idx].hallx[0], ois->lens_info_buf->buf[idx].hally[0]);
	*/
	LOG_OIS_VERB("lens info copy done %d(%llums)", ret,
		(((u64)end_time.tv_sec * 1000 + end_time.tv_nsec / 1000000) - ((u64)start_time.tv_sec * 1000 + start_time.tv_nsec / 1000000)));

	mutex_unlock(&ois->op_lock);
	return ret;
}

static int dw9781c_get_fw_version(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);

	ret = copy_to_user(user_buf, &flash_info->fwInfo.version, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("fw ver 0x%08x", flash_info->fwInfo.version);

	return ret;
}

static int dw9781c_get_gyro_offset(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	s16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &rxdata);
	flash_info->imuInfo.gyroOffsetX = rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &rxdata);
	flash_info->imuInfo.gyroOffsetY = rxdata;

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroOffsetX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("offset(%d, %d)", flash_info->imuInfo.gyroOffsetX, flash_info->imuInfo.gyroOffsetY);
	return ret;
}

static int dw9781c_get_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	s16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINX, &rxdata);
	flash_info->imuInfo.gyroGainX = rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_GAINY, &rxdata);
	flash_info->imuInfo.gyroGainY = rxdata;

	ret = copy_to_user(user_buf, &flash_info->imuInfo.gyroGainX, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("gyro gain(0x%08x, 0x%08x)", flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY);
	return ret;
}

static int dw9781c_get_mode(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_flash_info *flash_info = ois->flash_info;
	struct i2c_client *client = ois->client;
	u16 rxdata = 0x0000;

	OIS_BUG(!flash_info);
	OIS_BUG(!user_buf);
	OIS_BUG(!client);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &rxdata);
	flash_info->mode = rxdata;

	ret = copy_to_user(user_buf, &flash_info->mode, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("ois mode(0x%x)", flash_info->mode);

	return ret;
}

static int dw9781c_set_smooth(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_smooth_info smooth = { 0, };
	u16 on = 0x0000, step = 0x0000, delay = 0x0000;

	OIS_BUG(!client);

	ret = copy_from_user(&smooth, user_buf, sizeof(struct ois_smooth_info));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &on);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_STEP, &step);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_DELAY, &delay);
	LOG_OIS_INF("E smooth(%d %d %d) before(%d %d %d)",
		on, step, delay, smooth.on, smooth.step, smooth.delay);

	if (MANUAL_ON == smooth.on) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_EN, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_STEP, smooth.step);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_DELAY, (smooth.delay * 3));//3code≈1ms
		I2COP_CHECK(ret);
	} else if (DEFAULT_ON == smooth.on) {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_EN, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_STEP, 102);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_DELAY, 13);
		I2COP_CHECK(ret);
	} else {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_SMOOTH_EN, OFF);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_EN, &on);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_STEP, &step);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SMOOTH_DELAY, &delay);

	LOG_OIS_INF("X smooth(%d %d %d)", on, step, delay);
p_err:
	return ret;
}

static int dw9781c_flash_save(struct ois *ois)
{
	int ret = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);

	//save info
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FLASH_SAVE);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(100);

	//reset ois
	ret = dw9781c_reset(ois);
	I2COP_CHECK(ret);

	//restart ois
	ret = dw9781c_start_ois(ois);

	LOG_OIS_INF("ois flash save success(%d)", ret);
p_err:
	return ret;
}

static int dw9781c_set_gyro_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int gain_param[3] =  { 0, };
	u16 tx_data = 0x0000;
	s16 rx_data = 0x0000;
	u16 addr    = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(gain_param, user_buf, sizeof(gain_param));
	if (ret) {
		LOG_OIS_ERR("copy gain fail(%d)", ret);
		goto p_err;
	}

	if (gain_param[0] > 0) {
		addr = DW9781C_REG_OIS_GYRO_GAINY;
		tx_data = (u16)(gain_param[2]);
	} else {
		addr = DW9781C_REG_OIS_GYRO_GAINX;
		tx_data = (u16)(gain_param[1]);
	}

	ret = ois_i2c_write(client, addr, tx_data);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, addr, &rx_data);

	LOG_OIS_INF("gain param(%d %d %d), set(addr=0x%04x value=%d) read %d(0x%04x)",
		gain_param[0], gain_param[1], gain_param[2], addr, tx_data, rx_data, rx_data);

p_err:
	return ret;
}

static int dw9781c_set_offset_calibration(struct ois *ois)
{
	int ret = 0;
	u8 read_max = 10;
	u8 idx = 0;
	u16 cal_status = 0x0000;
	s16 offsetx = 0x0000;
	s16 offsety = 0x0000;
	s16 gyro_rawx = 0x0000;
	s16 gyro_rawy = 0x0000;
	struct i2c_client *client = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;

	LOG_OIS_INF("E");

	OIS_BUG(!client);
	OIS_BUG(!flash_info);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &offsety);
	LOG_OIS_INF("before cal offset(0x%04x 0x%04x) ret(%d)", offsetx, offsety, ret);

	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_GYRO_OFFSET_CAL);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
	I2COP_CHECK(ret);
	mdelay(100);

	for (idx = 0, read_max = 10; idx < 2; idx++) {
		while (1) {
			ret = ois_i2c_read(client, DW9781C_REG_OIS_GYROOFFSET_CAL_STS, &cal_status);
			if (REG_DEAFULT_ON & cal_status) {
				LOG_OIS_INF("offset calibration done(0x%04x)", cal_status);
				break;
			} else if (--read_max) {
				LOG_OIS_INF("cal status(0x%04x)  gyro(0x%04x 0x%04x) read retry(%d)", cal_status, gyro_rawx, gyro_rawy, read_max);
				mdelay(50);
			} else {
				LOG_OIS_INF("offset cal timeout(%d,%d)", ret, read_max);
				break;
			}
		}
		//if cal fail,try again
		if (cal_status & REG_DEAFULT_ON) {
			break;
		} else {
			LOG_OIS_INF("offset cal failed(try=%d err_msg=0x%04x)", idx, cal_status);
		}
	}

	if (!(cal_status & REG_DEAFULT_ON))
		goto p_err;

	ret = dw9781c_flash_save(ois);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETX, &offsetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_OFFSETY, &offsety);
	flash_info->imuInfo.gyroOffsetX = offsetx;
	flash_info->imuInfo.gyroOffsetY = offsety;

#if defined(CONFIG_MTK_CAM_PD2257F_EX) || defined(CONFIG_MTK_CAM_PD2215F_EX)
	ret = ois_i2c_write(client, DW9781C_REG_OIS_OSC_CTL, OFF);
	I2COP_CHECK(ret);
#endif

	LOG_OIS_INF("X: after cal offset(%d %d) ret(%d)", offsetx, offsety, ret);

p_err:
	return ret;
}

static int dw9781c_set_acc(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct ois_acc_param acc_info = {0};
	u16 acc_gainx = 0x0000, acc_gainy = 0x0000;
	struct i2c_client *client = ois->client;
	u16 tx_data = 0x0000;
	int idx = 0;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);
	OIS_BUG(!(ois->flash_info));
	OIS_BUG(!(ois->flash_info->chipEn));

	ret = copy_from_user(&acc_info, user_buf, sizeof(struct ois_acc_param));
	if (ret) {
		LOG_OIS_ERR("copy_from_user fail(%d)", ret);
		goto p_err;
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &acc_gainx);

	LOG_OIS_VERB("acc (%d %d %d) ret(%d)", acc_info.accOn, acc_info.preFocusDistance, acc_info.currFocusDistance, ret);

	tx_data = (u16)acc_info.currFocusDistance;
	idx = tx_data / 10;
	if (idx > GAIN_TABLE_SIZE_MAX) {
		LOG_OIS_INF("out of gain table, skip");
		goto p_err;
	}
	tx_data = dw9781c_acc_gain_table[idx];

#ifdef CONFIG_MTK_CAM_PD2215F_EX
    tx_data = dw9781c_acc_gain_table_pd2215[idx];
#elif defined(CONFIG_MTK_CAM_PD2257F_EX)
    tx_data = dw9781c_acc_gain_table_pd2257[idx];
#endif
	//convert to acc gain
	if (1 == acc_info.accOn && acc_gainx != tx_data) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINX, tx_data);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINY, tx_data);
		I2COP_CHECK(ret);
	} else if (0 == acc_info.accOn) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINX, OFF);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_ACC_GAINY, OFF);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &acc_gainx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &acc_gainy);

	LOG_OIS_VERB("acc info set(%d %d) write(%d) read(%d %d) ret(%d)",
		acc_info.accOn, acc_info.currFocusDistance, tx_data, acc_gainx, acc_gainy, ret);

p_err:
	return ret;
}

static int dw9781c_af_crosstalk_compensation(struct ois *ois, struct ois_af_drift_param *drift)
{
	int ret = 0;
	u16 tx_data = 0x0000;
	struct i2c_client *client = ois->client;
	s16 drift_on = 0x0000, af_position = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!drift);
	OIS_BUG(!(ois->flash_info));
	OIS_BUG(!(ois->flash_info->chipEn));

	LOG_OIS_VERB("drift info(%d %d %d)", drift->driftCompOn, drift->currFocusDac, drift->preFocusDac);
	if(ois->pjt_name == 2083) {
		LOG_OIS_VERB("PD2083 drift set skip");
		goto p_err;
	}
	if (-1 == drift->driftCompOn) {//hard code -1 for normal af code update case
		ret = ois_i2c_read(client, DW9781C_REG_OIS_AF_DRIFT_COMP, &drift_on);
		if (OFF == drift_on) {
			LOG_OIS_INF("drift set skip", drift_on);
			goto p_err;
		}
	} else {
		tx_data = (drift->driftCompOn == 1) ? (REG_DEAFULT_ON) : (OFF);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_AF_DRIFT_COMP, tx_data);
		I2COP_CHECK(ret);
		LOG_OIS_INF("drift status change(0x%04x)", drift_on);
		goto p_err;
	}

	tx_data = (u16)drift->currFocusDac;

	ret = ois_i2c_write(ois->client, DW9781C_REG_AF_POSITION, tx_data);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_AF_DRIFT_COMP, &drift_on);
	ret = ois_i2c_read(client, DW9781C_REG_AF_POSITION, &af_position);

	LOG_OIS_VERB("drift set(%d) read(0x%02x, %d) ret(%d)", tx_data, drift_on, af_position, ret);

p_err:
	return ret;
}

static int dw9781c_set_tripod(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	int on = 0x0000;
	u16 rx_data = 0x0000;
	u16 tx_data = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&on, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy tripod fail(%d)", ret);
		goto p_err;
	}

	tx_data = (on) ? (REG_DEAFULT_ON) : (OFF);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &rx_data);
	if (tx_data == rx_data) {
		LOG_OIS_INF("config already done(0x%04x)", rx_data);
		goto p_err;
	}

	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_TRIPODE_CTRL, tx_data);
	I2COP_CHECK(ret);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_CTRL, &rx_data);

	LOG_OIS_INF("tripod set(%d 0x%04x) read(0x%04x)", on, tx_data, rx_data);

p_err:
	return ret;
}

static int dw9781c_set_sinewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_sinemode_parameter sine = { 0, };
	u16 mode = 0x0000;
	u16 amp = 0x0000;
	u16 frequency = 0x0000;
	u16 cmd_status = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&sine, user_buf, sizeof(struct ois_sinemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy sine params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("sine set(%d %d %d %d)", sine.axis, sine.frequency, sine.amplitudeX, sine.amplitudeY);

	if (1 == sine.axis)
		mode = 0x8005;
	else if (2 == sine.axis)
		mode = 0x8004;
	else {
		LOG_OIS_INF("invalid axis(%d)", sine.axis);
		goto p_err;
	}

	if (sine.amplitudeX > -1 && sine.amplitudeX <= 100)
		amp = (u16)(0xFFFF * sine.amplitudeX / 200);
	else {
		LOG_OIS_INF("invalid amplitude(%d)", sine.amplitudeX);
		goto p_err;
	}

	frequency = (u16)sine.frequency;

	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_MODE, mode);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_AMP, amp);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_FREQ, frequency);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CMD_STATUS, REG_DEAFULT_ON);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CMD_STATUS, &cmd_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_AMP, &amp);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_FREQ, &frequency);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &mode);

	LOG_OIS_INF("sine read(0x%04x 0x%04x) mode(0x%04x) cmd(0x%04x)", frequency, amp, mode, cmd_status);

p_err:
	return ret;
}

static int dw9781c_set_circlewave(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	struct i2c_client *client = ois->client;
	struct ois_circlemode_parameter circle = { 0, };
	u16 mode = 0x0000;
	u16 amp = 0x0000;
	u16 frequency = 0x0000;
	u16 cmd_status = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&circle, user_buf, sizeof(struct ois_circlemode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy circle params fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("circle set(%d %d %d)", circle.circleFrequency, circle.amplitudeX, circle.amplitudeY);

	if (circle.amplitudeX > -1 && circle.amplitudeX <= 100)
		amp = (u16)(0xFFFF * circle.amplitudeX / 200);
	else {
		LOG_OIS_INF("invalid amplitude(%d)", circle.amplitudeX);
		goto p_err;
	}

	frequency = (u16)circle.circleFrequency;

	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_MODE, DW9781C_CIRCLEWAVE_MODE);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_AMP, amp);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_SINEWAVE_FREQ, frequency);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CMD_STATUS, REG_DEAFULT_ON);
	I2COP_CHECK(ret);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CMD_STATUS, &cmd_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_AMP, &amp);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_SINEWAVE_FREQ, &frequency);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &mode);

	LOG_OIS_INF("circle read(0x%04x 0x%04x) mode(0x%04x) cmd(0x%04x)", frequency, amp, mode, cmd_status);

p_err:
	return ret;
}

static int dw9781c_set_target(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	u16 servo = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_fixmode_parameter fixmode = { 0, };
	int min_code = 0, max_code = 0;
	s16 hallx = 0x0000, hally = 0x0000;
	s16 targetx = 0x0000, targety = 0x0000;
	u16 ampX = 0x0000, ampY = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &servo);
	if (SERVO_ON != servo) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CTRL, SERVO_ON);
		I2COP_CHECK(ret);
		LOG_OIS_INF("switch to servo on");
	}

	ret = copy_from_user(&fixmode, user_buf, sizeof(struct ois_fixmode_parameter));
	if (ret) {
		LOG_OIS_ERR("copy target fail(%d)", ret);
		goto p_err;
	}

	LOG_OIS_INF("target set(%d %d)", fixmode.targetX, fixmode.targetY);

	ampX = ois->flash_info->hallInfo.totalLimitX;
	ampY = ois->flash_info->hallInfo.totalLimitY;
	targetx = (s16)(ampX * fixmode.targetX / 100);
	targety = (s16)(ampY * fixmode.targetY / 100);

	if (abs(targetx) <= ampX && abs(targety) <= ampY) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETX, targetx);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_CL_TARGETY, targety);
		I2COP_CHECK(ret);
	} else {
		LOG_OIS_INF("invalid target(%d, %d)", targetx, targety);
		goto p_err;
	}

	max_code = abs(ois->flash_info->hallInfo.totalLimitX);
	min_code = 0 - max_code;

	mdelay(3);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETX, &targetx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETY, &targety);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSX, &hallx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSY, &hally);

	LOG_OIS_INF("hall read target(0x%04x 0x%04x) hall(0x%04x 0x%04x) amp(0x%04x 0x%04x)",
		targetx, targety, hallx, hally, ampX, ampY);

p_err:
	return ret;
}
/*
static int dw9781c_shutdown(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int servo_param = 0;
	u16 data = 0x0000;
	struct i2c_client *client    = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&servo_param, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy chip control info fail(%d)", ret);
		goto p_err;
	}

	data = (!servo_param) ? OFF : ON;

	ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, data);
	I2COP_CHECK(ret);
	mdelay(5);

	LOG_OIS_INF("chip set 0x%04x", servo_param);
p_err:
	return ret;
}
*/
static int dw9781c_set_servo(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int servo_param = 0;
	u16 data = 0x0000;
	struct i2c_client *client    = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&servo_param, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy servo info fail(%d)", ret);
		goto p_err;
	}

	data = (!servo_param) ? 0x0002 : 0x0001;

	ret = ois_i2c_write(client, DW9781C_REG_OIS_CTRL, data);
	I2COP_CHECK(ret);
	mdelay(5);

	LOG_OIS_INF("servo set 0x%04x", servo_param);
p_err:
	return ret;
}

static int dw9781c_set_pantilt(struct ois *ois, __user void *user_buf)
{
	int ret             = 0;
	u16 pantilt_on      = 0x0000;
	u16 pantilt_degreeX = 0x0000;
	u16 pantilt_degreeY = 0x0000;
	struct i2c_client     *client    = ois->client;
	struct ois_pantilt_param pantilt = { 0, };

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&pantilt, user_buf, sizeof(struct ois_pantilt_param));
	if (ret) {
		LOG_OIS_ERR("copy pantilt fail(%d)", ret);
		goto p_err;
	}

	if (ON == pantilt.on) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_CTRL, DW9781C_PANTILT_ON);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_DERGEEX, pantilt.limitX);
		I2COP_CHECK(ret);
	} else if (OFF == pantilt.on) {
		ret = ois_i2c_write(ois->client, DW9781C_REG_OIS_PANTILT_CTRL, DW9781C_PANTILT_OFF);
		I2COP_CHECK(ret);
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_CTRL, &pantilt_on);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &pantilt_degreeX);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &pantilt_degreeY);

	LOG_OIS_INF("pantilt info(%d %d %d) read(%d %d %d) ret(%d)",
		pantilt.on, pantilt.limitX, pantilt.limitY,
		pantilt_on, pantilt_degreeX, pantilt_degreeY, ret);
p_err:
	return ret;
}

/*write fw to flash*/
static int dw9781c_fw_download(struct ois *ois)
{
	int                   ret              = 0;
	struct i2c_client     *client          = ois->client;
	u16                   idx              = 0;
	u16                   update_flag      = 0;
	const struct firmware *ois_fw          = NULL;
	const u8              *fw_write_data   = NULL;
	u16                   fw_size          = 0;
	size_t                unit_size        = sizeof(u16)/sizeof(u8);
	u8                    *fw_read_data    = NULL;

	OIS_BUG(!client);
	OIS_BUG(!(ois->dev));

	//1.load fw to kernel space
	ret = request_firmware(&ois_fw, DW9781C_OIS_FW_NAME, ois->dev);
	if (ret) {
		LOG_OIS_ERR("load fw failed(%d)", ret);
		goto p_err;
	}
	fw_write_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_write_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	fw_size = ois_fw->size;
	memcpy((void *)fw_write_data, (void *)(ois_fw->data), (ois_fw->size));
	fw_read_data = (u8 *)(kzalloc(ois_fw->size, GFP_KERNEL));
	if (!fw_read_data) {
		LOG_OIS_ERR("ois fw buffer alloc fail");
		goto p_err;
	}
	release_firmware(ois_fw);
	LOG_OIS_INF("ois_fw buffer %p, size %d unit_size %d", fw_write_data, fw_size, unit_size);

	//2.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//3.i2c level adjust
	ret = ois_i2c_write(client, 0xd005, 0x0001);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, 0xdd03, 0x0002);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, 0xdd04, 0x0002);
	I2COP_CHECK(ret);

	//4.erase current exist data of flash(each unit is 4k bytes)
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_0);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_2);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_3);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_4);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_START);
	I2COP_CHECK(ret);
	mdelay(10);
	LOG_OIS_INF("flash erase success!");

	//5.write new fw to flash
	for (idx = 0; idx < fw_size; idx += DW9781C_BLOCK_SIZE) {
		ret = ois_i2c_write_block(client, (MTP_START_ADDR + idx / unit_size), &fw_write_data[idx], DW9781C_BLOCK_SIZE);
		mdelay(10);
		if (ret) {
			update_flag = 1;
			LOG_OIS_ERR("flash[%d] write: addr 0x%04x failed(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
			break;
		} else {
			LOG_OIS_INF("flash[%d] write: addr 0x%04x success(%d)", idx, (MTP_START_ADDR + idx / unit_size), ret);
		}
	}
	if (update_flag)
		goto p_err;

	//6.verify new fw data in flash
	for (idx = 0; idx < fw_size; idx += DW9781C_BLOCK_SIZE) {
		ret = ois_i2c_read_block(client, (MTP_START_ADDR + idx / unit_size), &fw_read_data[idx], DW9781C_BLOCK_SIZE);
	}
	for (idx = 0; idx < fw_size; idx++) {
		if (fw_write_data[idx] != fw_read_data[idx]) {
			LOG_OIS_ERR("fw data mismatch: idx 0x%04x flash=0x%02x expect=0x%02x\n", idx, fw_read_data[idx], fw_write_data[idx]);
			ret = -1;
		}
	}
	/*
	for (idx = 0; idx < fw_size; idx += 8) {
		LOG_OIS_INF("flash read data[%lu]:0x%02x, 0x%02x, 0x%02x, 0x%02x,0x%02x, 0x%02x, 0x%02x, 0x%02x,\n",
			idx, fw_read_data[idx+0], fw_read_data[idx+1], fw_read_data[idx+2], fw_read_data[idx+3], fw_read_data[idx+4], fw_read_data[idx+5], fw_read_data[idx+6], fw_read_data[idx+7]);
	}
	*/
	if (!ret) {
		//ic reset & restart ois
		ret = dw9781c_reset(ois);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		mdelay(10);
		LOG_OIS_INF("fw download success");
	} else {
		//if failed, erase ldt flash and shutdown chip
		ret = ois_i2c_write(client, DW9781C_REG_FLASH_SECTOR_SELECT, SECTOR_PAGE);
		I2COP_CHECK(ret);
		ret = ois_i2c_write(client, DW9781C_REG_FLASH_ERASE_CTRL, DW9781C_ERASE_PAGE);
		I2COP_CHECK(ret);
		mdelay(10);
		ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, OFF);
		I2COP_CHECK(ret);
		LOG_OIS_ERR("fw download failed(%d)", ret);
		goto p_err;
	}

p_err:
	if (fw_write_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_write_data);
		kfree(fw_write_data);
		fw_write_data = NULL;
	}
	if (fw_read_data) {
		LOG_OIS_INF("ois fw buffer(%p) release", fw_read_data);
		kfree(fw_read_data);
		fw_read_data = NULL;
	}
	return ret;
}

static int dw9781c_fw_update(struct ois *ois, void __user *user_buf)
{
	int                   ret               = 0;
	u16                   fw_checksum_flag  = 0x0000;
	u16                   first_chip_id     = 0x0000;
	u16                   second_chip_id    = 0x0000;
	struct i2c_client     *client           = ois->client;
	u16                   fw_date           = 0x0000;
	u16                   fw_type           = 0x0000;
	u16                   curr_fw_version   = 0x0000;
	u16                   checksum_exp      = 0x0000, checksum_cal = 0x0000;
	u16                   exp_fw_version    = EXP_FW.version;
	int                   froce_version     = 0;
	u16                   ois_ready_check_1 = 0x0000;
	u16                   ois_ready_check_2 = 0x0000;

	OIS_BUG(!client);

	LOG_OIS_INF("E");

	ois->client->addr = DW7981C_SLAVE_ADDR >> 1;
/*#if defined(CONFIG_MTK_CAM_PD2204F_EX)
	code = OIS_BIG_ENDIAN_TRANS4(vivo_otp_data_s5kgw1sd03pd2204f, 0x000F);
	exp_fw_version	 = EXP_FW.version;
#endif
	LOG_OIS_INF("module code:%d", code);
*/
	//1. check fw version & type
	if (NULL != user_buf) {
		ret = copy_from_user(&froce_version, user_buf, sizeof(int));
		if (ret) {
			LOG_OIS_ERR("copy_from_user fail(%d)", ret);
			goto p_err;
		}
		exp_fw_version = froce_version;
		LOG_OIS_INF("force ois update(0x%04x)", froce_version);
	}

	//1.disable dsp and release all protection
	ret = ois_i2c_write(client, DW9781C_REG_OIS_DSP_CTRL, OFF);//standby mode
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
	I2COP_CHECK(ret);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
	I2COP_CHECK(ret);

	//2.check checksum flag
	ret = ois_i2c_read(client, DW9781C_REG_OIS_READY_CHECK_1, &ois_ready_check_1);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_READY_CHECK_2, &ois_ready_check_2);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_FLAG, &fw_checksum_flag);
	if ((FW_CHECKSUM_FLAG == fw_checksum_flag) && (OIS_READY_CHECK_1 == ois_ready_check_1) && (OIS_READY_CHECK_2 == ois_ready_check_2)) {
		LOG_OIS_INF("ois checksum flag ok(0x%04x)(0x%04x)(0x%04x)", fw_checksum_flag, ois_ready_check_1, ois_ready_check_2);
		ret = dw9781c_reset(ois);
		if (ret) {
			LOG_OIS_INF("ic reset fail(%d)", ret);
			goto p_err;
		}
	} else {
		ret = ois_i2c_write(client, DW9781C_REG_OIS_LOGIC_RESET, ON);
		mdelay(4);
		I2COP_CHECK(ret);
		LOG_OIS_INF("ois checksum flag ng(0x%04x)", fw_checksum_flag);
	}

	//3.check chip id and download fw
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CHIP_ID, &first_chip_id);
	if (CHIP_ID != first_chip_id) {
		LOG_OIS_ERR("previous ois fw update failed(chip=0x%04x, expect=0x%04x)", first_chip_id, CHIP_ID);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE1, ALL_RELEASE1);
		ret = ois_i2c_write(client, DW9781C_REG_OIS_ALL_RELEASE2, ALL_RELEASE2);
		I2COP_CHECK(ret);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_SECOND_CHIP_ID, &second_chip_id);
		if (DW9781C_SENCOND_CHIP_ID == second_chip_id) {
			LOG_OIS_INF("try to udpate fw again");
			ret = dw9781c_fw_download(ois);
			if (ret) {
				LOG_OIS_ERR("fw download failed");
				goto p_err;
			}
		} else {
			LOG_OIS_ERR("chip id error(0x%04x)", second_chip_id);
			ret = ois_i2c_write(client, DW9781C_REG_CHIP_CTRL, OFF);
			I2COP_CHECK(ret);
			goto p_err;
		}
	} else {
		//fw checksum verify
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_STORED, &checksum_exp);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_MODE, OP_FW_CHECKSUM);
		ret = ois_i2c_write(client, DW9781C_REG_OPERATION_ENABLE, REG_DEAFULT_ON);
		I2COP_CHECK(ret);
		mdelay(10);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_CAL, &checksum_cal);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
		LOG_OIS_INF("flash cal checksum (0x%04x)", checksum_cal);
		if((checksum_exp != checksum_cal)||(fw_type != 0x8001)) {
			LOG_OIS_ERR("fw checksum failed:exp(0x%04x) cal(0x%04x) fwtype(0x%04x)", checksum_exp, checksum_cal, fw_type);
			ret = dw9781c_fw_download(ois);
			if (ret) {
				LOG_OIS_ERR("fw download failed");
				goto p_err;
			}
		}
		//firmware version check
		
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &curr_fw_version);
		ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &fw_date);
		if ((exp_fw_version == curr_fw_version)&&(fw_date == EXP_FW.date)) {
			LOG_OIS_INF("same fw, skip update(0x%04x)", curr_fw_version);
			goto p_err;
		}
		LOG_OIS_INF("current fw(0x%04x) expect fw(0x%04x)", curr_fw_version, exp_fw_version);
		ret = dw9781c_fw_download(ois);
		if (ret) {
			LOG_OIS_ERR("fw download failed");
			goto p_err;
		}
	}

	//5.flash checksum check after download
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_CAL, &checksum_cal);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_STORED, &checksum_exp);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
	if ((checksum_exp != checksum_cal)||(fw_type != 0x8001)) {
		LOG_OIS_ERR("flash checksum expect 0x%04x cal 0x%04x, retry fw downloade", checksum_exp, checksum_cal);
		ret = dw9781c_fw_download(ois);
		if (ret) {
			LOG_OIS_ERR("fw download failed");
			goto p_err;
		} else {
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_CAL, &checksum_cal);
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_CHECKSUM_STORED, &checksum_exp);
			ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);
			if ((checksum_exp != checksum_cal)||(fw_type != 0x8001)) {
				LOG_OIS_ERR("flash checksum expect 0x%04x cal 0x%04x, exit", checksum_exp, checksum_cal);
				goto p_err;
			}
		}
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_VERSION, &curr_fw_version);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_DATE, &fw_date);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_FW_TYPE, &fw_type);

	LOG_OIS_INF("X: fw ver=0x%04x date=0x%04x type=0x%04x update success(%d)", curr_fw_version, fw_date, fw_type, ret);
p_err:
	return ret;
}

static int dw9781c_status_check(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	s16 s16_rxdata = 0x0000;
	u16 u16_rxdata = 0x0000;
	struct i2c_client     *client      = ois->client;
	struct ois_flash_info *flash_info = ois->flash_info;
	s16 drift_on = 0x0000, af_position = 0x0000;
	u16 osc_countx = 0x0000;
	u16 osc_county = 0x0000;
	u16 oscillation_status = 0x0000;
	u16 loopgain_x = 0x0000;
	u16 loopgain_y = 0x0000;

	OIS_BUG(!client);
	OIS_BUG(!flash_info);
	OIS_BUG(!(ois->flash_info->chipEn));

	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_RAWX, &s16_rxdata);
	flash_info->imuInfo.gyroRawX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_RAWY, &s16_rxdata);
	flash_info->imuInfo.gyroRawY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_RAWX, &s16_rxdata);
	flash_info->imuInfo.accRawX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_RAWY, &s16_rxdata);
	flash_info->imuInfo.accRawY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_GYRO_READ_CTRL, &u16_rxdata);
	flash_info->imuInfo.imuReadEn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINX, &s16_rxdata);
	flash_info->imuInfo.accGainX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_GAINY, &s16_rxdata);
	flash_info->imuInfo.accGainY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_FILTERX, &s16_rxdata);
	flash_info->imuInfo.gyrofiltX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_FILTERY, &s16_rxdata);
	flash_info->imuInfo.gyrofiltY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_FILTERX, &s16_rxdata);
	flash_info->imuInfo.accfiltX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_FILTERY, &s16_rxdata);
	flash_info->imuInfo.accfiltY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_TARGETX, &s16_rxdata);
	flash_info->imuInfo.gyrotargetX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_GYRO_TARGETY, &s16_rxdata);
	flash_info->imuInfo.gyrotargetY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_TARGETX, &s16_rxdata);
	flash_info->imuInfo.acctargetX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_ACC_TARGETY, &s16_rxdata);
	flash_info->imuInfo.acctargetY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TOTAL_TARGETX, &s16_rxdata);
	flash_info->imuInfo.totaltargetX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TOTAL_TARGETY, &s16_rxdata);
	flash_info->imuInfo.totaltargetY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_SPI_MODE, &u16_rxdata);
	flash_info->imuInfo.spiMode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSX, &s16_rxdata);
	flash_info->hallInfo.lensPosX = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LENS_POSY, &s16_rxdata);
	flash_info->hallInfo.lensPosY = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_MODE, &u16_rxdata);
	flash_info->mode = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_CTRL, &u16_rxdata);
	flash_info->serveOn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_DSP_CTRL, &u16_rxdata);
	flash_info->dspEn = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOGIC_RESET, &u16_rxdata);
	flash_info->logicReset = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_TRIPODE_STATUS, &u16_rxdata);
	flash_info->tripodFlag = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_AF_DRIFT_COMP, &s16_rxdata);
	drift_on = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_AF_POSITION, &s16_rxdata);
	af_position = s16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEX, &u16_rxdata);
	flash_info->hallInfo.totalLimitX = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_DERGEEY, &u16_rxdata);
	flash_info->hallInfo.totalLimitY = u16_rxdata;
	ret = ois_i2c_read(client, DW9781C_REG_OIS_PANTILT_CTRL, &u16_rxdata);
	flash_info->hallInfo.pantiltOn = u16_rxdata;


	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_XCOUNT, &osc_countx);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_YCOUNT, &osc_county);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_CTL, &oscillation_status);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINX, &loopgain_x);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINY, &loopgain_y);

	if (SERVO_ON == flash_info->serveOn) {
		ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read(client, DW9781C_REG_OIS_CL_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	} else if (OIS_ON == flash_info->serveOn) {
		ret = ois_i2c_read(client, DW9781C_REG_OIS_TARGETX, &s16_rxdata);
		flash_info->targetInfo.totalTargetX = s16_rxdata;
		ret = ois_i2c_read(client, DW9781C_REG_OIS_TARGETY, &s16_rxdata);
		flash_info->targetInfo.totalTargetY = s16_rxdata;
	}

	LOG_OIS_INF("gyroraw: %d %d accraw: %d %d gain 0x%04x 0x%04x gyrofilt:%d %d accfilt:%d %d gyrotarget:%d %d acctarget:%d %d totaltarget: %d %d target: %d %d lensPos: %d %d  gain(%d,%d,%d,%d) oscillationInfo %d %d %d loopGain 0x%x 0x%x",
		flash_info->imuInfo.gyroRawX, flash_info->imuInfo.gyroRawY, flash_info->imuInfo.accRawX, flash_info->imuInfo.accRawY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY,
		flash_info->imuInfo.gyrofiltX, flash_info->imuInfo.gyrofiltY, flash_info->imuInfo.accfiltX, flash_info->imuInfo.accfiltY,
		flash_info->imuInfo.gyrotargetX, flash_info->imuInfo.gyrotargetY, flash_info->imuInfo.acctargetX, flash_info->imuInfo.acctargetY, flash_info->imuInfo.totaltargetX, flash_info->imuInfo.totaltargetY,
		flash_info->targetInfo.totalTargetX, flash_info->targetInfo.totalTargetY,
		flash_info->hallInfo.lensPosX, flash_info->hallInfo.lensPosY,
		flash_info->imuInfo.gyroGainX, flash_info->imuInfo.gyroGainY, flash_info->imuInfo.accGainX, flash_info->imuInfo.accGainY,
		osc_countx, osc_county, oscillation_status, loopgain_x, loopgain_y);

	LOG_OIS_VERB("servo:%d dsp:%d reset:%d mode:%d tripod:%d spi mode:%d read:0x%04x drift:%d %d pantilt:0x%04x %d %d",
		flash_info->serveOn, flash_info->dspEn, flash_info->logicReset, flash_info->mode,
		flash_info->tripodFlag, flash_info->imuInfo.imuReadEn, flash_info->imuInfo.spiMode, drift_on, af_position,
		flash_info->hallInfo.pantiltOn, flash_info->hallInfo.totalLimitX, flash_info->hallInfo.totalLimitY);

	return ret;
}
#if defined(CONFIG_MTK_CAM_PD2257F_EX)
static void format_otp_data_pd2257(struct ois_otp_info *ois_otp, u8 *otp_buf)
{
	int                 ret       = 0;
	u8                  *sn_data  = NULL;
	u8                  *fuse_id  = NULL;
	u8                  data_size = 0;
	u8                  idx       = 0;
	s16                 s16_data  = 0x0000;


	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x2E4A);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x2EA2);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x2EA6);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E92);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E96);
	ois_otp->gyroOffsetY = s16_data;
	// ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9A);
	// ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9E);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E66);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E6A);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E5E);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E62);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E76);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E78);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2EAA);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2EAC);
	ois_otp->icType = DW9781C;
	data_size = 0x0026 - 0x0021 + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0021 + idx]);

	data_size = 0x0051 - 0x0046 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
		LOG_OIS_ERR("sn data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0046 + idx]);


	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(%d,%d,%d,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX, ois_otp->gyroGainY,
		ois_otp->gyroOffsetX, ois_otp->gyroOffsetY, ois_otp->hallXMin, ois_otp->hallXMax,
		ois_otp->hallYMin, ois_otp->hallYMax, 
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);

p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
}
#endif
#if defined(CONFIG_MTK_CAM_PD2215F_EX)
static void format_otp_data_pd2215(struct ois_otp_info *ois_otp, u8 *otp_buf)
{
	int                 ret       = 0;
	u8                  *sn_data  = NULL;
	u8                  *fuse_id  = NULL;
	u8                  data_size = 0;
	u8                  idx       = 0;
	s16                 s16_data  = 0x0000;


	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x7E4A);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x7EA2);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x7EA6);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E92);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E96);
	ois_otp->gyroOffsetY = s16_data;
	// ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9A);
	// ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x2E9E);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E66);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E6A);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E5E);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E62);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E76);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7E78);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7EAA);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x7EAC);
	ois_otp->icType = DW9781C;
	data_size = 0x0026 - 0x0021 + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0021 + idx]);

	data_size = 0x0051 - 0x0046 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
		LOG_OIS_ERR("sn data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0046 + idx]);


	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(%d,%d,%d,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX, ois_otp->gyroGainY,
		ois_otp->gyroOffsetX, ois_otp->gyroOffsetY, ois_otp->hallXMin, ois_otp->hallXMax,
		ois_otp->hallYMin, ois_otp->hallYMax, 
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);

p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
}
#endif
#if defined(CONFIG_MTK_CAM_PD2204F_EX)
static void format_otp_data_pd2204F_EX(struct ois_otp_info *ois_otp, u8 *otp_buf)
{
	int                 ret       = 0;
	u8                  *sn_data  = NULL;
	u8                  *fuse_id  = NULL;
	u8                  data_size = 0;
	u8                  idx       = 0;
	s16                 s16_data  = 0x0000;


	ois_otp->fwVersion = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x6A3E);
	ois_otp->gyroGainX = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x6A66);
	ois_otp->gyroGainY = OIS_BIG_ENDIAN_TRANS4(otp_buf, 0x6A68);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A6A);
	ois_otp->gyroOffsetX = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A6C);
	ois_otp->gyroOffsetY = s16_data;
	ois_otp->hallMechCenterX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A6E);
	ois_otp->hallMechCenterY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A70);
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A76);
	ois_otp->hallXMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A78);
	ois_otp->hallYMin = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A72);
	ois_otp->hallXMax = s16_data;
	s16_data = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A74);
	ois_otp->hallYMax = s16_data;
	ois_otp->tiltSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A7E);
	ois_otp->tiltSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6A80);
	ois_otp->accSRX = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6AFC);
	ois_otp->accSRY = OIS_BIG_ENDIAN_TRANS2(otp_buf, 0x6AFE);
	ois_otp->icType = DW9781C;
	data_size = 0x0026 - 0x0021 + 1;
	fuse_id = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!fuse_id) {
		LOG_OIS_ERR("fuse id kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&fuse_id[idx * 2], "%02x", otp_buf[0x0021 + idx]);

	data_size = 0x0051 - 0x0046 + 1;
	sn_data = kzalloc(sizeof(u8) * data_size * 2 + 2, GFP_KERNEL);
	if (!sn_data) {
		LOG_OIS_ERR("sn data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	for (idx = 0; idx < data_size; idx++)
		sprintf(&sn_data[idx * 2], "%02x", otp_buf[0x0046 + idx]);


	LOG_OIS_INF("ois otp: sn(0x%s) fuseId(0x%s) fwVer(0x%08x) gyroGain(%d, %d) gyroOffset(%d,%d) hall(%d,%d,%d,%d) SR(%d, %d, %d, %d)",
		sn_data, fuse_id, ois_otp->fwVersion, ois_otp->gyroGainX, ois_otp->gyroGainY,
		ois_otp->gyroOffsetX, ois_otp->gyroOffsetY, ois_otp->hallXMin, ois_otp->hallXMax,
		ois_otp->hallYMin, ois_otp->hallYMax, 
		ois_otp->tiltSRX, ois_otp->tiltSRY, ois_otp->accSRX, ois_otp->accSRY);

p_err:
	if (fuse_id)
		kfree(fuse_id);
	if (sn_data)
		kfree(sn_data);
}
#endif
static int dw9781c_format_otp_data(struct ois *ois, void __user *user_buf)
{
	int                 ret       = 0;
	struct ois_otp_info *ois_otp  = NULL;
	u8                  *otp_buf  = NULL;
	OIS_BUG(!user_buf);
	ois_otp = (struct ois_otp_info *)(kzalloc(sizeof(struct ois_otp_info), GFP_KERNEL));
	if (!ois_otp) {
		LOG_OIS_ERR("ois otp data kzalloc failed(%d)\n", ret);
		goto p_err;
	}
	ois_otp->inited = 0x00;

#if defined (CONFIG_MTK_CAM_PD2215F_EX)
	otp_buf = vivo_otp_data_s5kgw3sp13pd2215f;
	format_otp_data_pd2215(ois_otp, otp_buf);
#elif defined(CONFIG_MTK_CAM_PD2204F_EX)
	otp_buf = vivo_otp_data_s5kgw1sd03pd2204f;
	format_otp_data_pd2204F_EX(ois_otp, otp_buf);
#elif defined(CONFIG_MTK_CAM_PD2257F_EX)
	otp_buf = vivo_otp_data_s5kgw3sp13pd2257f;
	format_otp_data_pd2257(ois_otp, otp_buf);
#endif
	ret = copy_to_user(user_buf, ois_otp, sizeof(struct ois_otp_info));
	if (ret) {
		LOG_OIS_ERR("fail to copy otp info, ret(%d)\n", ret);
		goto p_err;
	}
	ois_otp->inited = 0x01;
p_err:
	if (ois_otp)
		kfree(ois_otp);
	return ret;
}

static int dw9781c_set_stroke_limit(struct ois *ois, void __user *user_buf)
{
	int ret = 0;
	return ret;
}
static int dw9781c_get_loopgain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int loopgain[2] = {0,};
	u16 t_loopgain[2] = {0,};
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	OIS_BUG(!user_buf);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINX, &(t_loopgain[0]));
	loopgain[0] = t_loopgain[0];
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINY, &(t_loopgain[1]));
	loopgain[1] = t_loopgain[1];

	ret = copy_to_user(user_buf, loopgain, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("loop gain(%d, %d)", loopgain[0], loopgain[1]);
	return ret;
}

static int dw9781c_get_oscillation_info(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int osc_count[2] = {0,};
	u16 tmp_osc_count[2] = {0,};
	u16 loopgain_x = 0x0000;
	u16 loopgain_y = 0x0000;
	struct i2c_client *client = ois->client;
	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINX, &loopgain_x);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINY, &loopgain_y);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_XCOUNT, &(tmp_osc_count[0]));
	osc_count[0] = tmp_osc_count[0];
	ret = ois_i2c_read(client, DW9781C_REG_OIS_OSC_YCOUNT, &(tmp_osc_count[1]));
	osc_count[1] = tmp_osc_count[1];
	ret = copy_to_user(user_buf, osc_count, (2 * sizeof(int)));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_INF("osc count %d %d loop gain 0x%x 0x%x", osc_count[0], osc_count[1], loopgain_x, loopgain_y);
	return ret;
}

static int dw9781c_set_loop_gain(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	int gain_coef = 0;
	u16 target_gainx = 0, target_gainy = 0;
	u16 cur_gainx = 0, cur_gainy = 0;
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = copy_from_user(&gain_coef, user_buf, sizeof(int));
	if (ret) {
		LOG_OIS_ERR("copy usr info fail(%d)", ret);
		return ret;
	}

	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINX, &cur_gainx);
	target_gainx = (u16)(cur_gainx *  gain_coef / 1000);
	ret = ois_i2c_read(client, DW9781C_REG_OIS_LOOPGAINY, &cur_gainy);
	target_gainy = (u16)(cur_gainy *  gain_coef / 1000);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_LOOPGAINX, target_gainx);
	ret = ois_i2c_write(client, DW9781C_REG_OIS_LOOPGAINY, target_gainy);

	LOG_OIS_INF("coef %d cur gain %d %d target gain %d %d",
		gain_coef, cur_gainx, cur_gainy, target_gainx, target_gainy);

	return ret;
}

static int dw9781c_get_oscd_input(struct ois *ois, __user void *user_buf)
{
	int ret = 0;
	u8  rxdata[4] = { 0 };
	s16 temp_value = 0;
	struct ois_oscd_input input_data = { 0 };
	struct i2c_client *client = ois->client;

	OIS_BUG(!client);
	OIS_BUG(!user_buf);

	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_GYRO_RAWX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.gyrox = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.gyroy = temp_value;
	/*
	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_GYRO_TARGETX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.targetgx = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.targetgy = temp_value;
	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_ACC_TARGETX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.targetax = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.targetay = temp_value;
	*/
	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_ACC_RAWX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.accx = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.accy = temp_value;
	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_TOTAL_TARGETX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.targetx = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.targety = temp_value;
	ret = ois_i2c_read_block(client, DW9781C_REG_OIS_LENS_POSX, &rxdata[0], 4);
	temp_value = ((rxdata[0] << 8) | rxdata[1]);
	input_data.hallx = temp_value;
	temp_value = ((rxdata[2] << 8) | rxdata[3]);
	input_data.hally = temp_value;

	ret = copy_to_user(user_buf, &input_data, sizeof(struct ois_oscd_input));
	if (ret) {
		LOG_OIS_ERR("copy_to_user fail(%d)", ret);
	}

	LOG_OIS_VERB("oscd input gyro %d %d acc %d %d target %d %d hall %d %d",
		input_data.gyrox, input_data.gyroy, input_data.accx, input_data.accy,
		input_data.targetx, input_data.targety, input_data.hallx, input_data.hally);

	return ret;
}


static struct ois_core_ops dw9781c_ois_ops = {
	.ois_init = dw9781c_init,
	.ois_deinit = dw9781c_deinit,
	.ois_stream_on = dw9781c_stream_on,
	.ois_stream_off = dw9781c_stream_off,
	.ois_get_mode = dw9781c_get_mode,
	.ois_set_mode = dw9781c_set_mode,
	.ois_fw_update = dw9781c_fw_update,
	.ois_get_fw_version = dw9781c_get_fw_version,
	.ois_get_gyro_offset = dw9781c_get_gyro_offset,
	.ois_set_offset_calibration = dw9781c_set_offset_calibration,
	.ois_get_gyro_gain = dw9781c_get_gyro_gain,
	.ois_set_gyro_gain = dw9781c_set_gyro_gain,
	.ois_flash_save = dw9781c_flash_save,
	.ois_set_acc = dw9781c_set_acc,
	.ois_set_target = dw9781c_set_target,
	.ois_get_init_info = dw9781c_get_init_info,
	.ois_status_check = dw9781c_status_check,
	.ois_init_vsync_thread = dw9781c_init_vsync_thread,
	.ois_deinit_vsync_thread = dw9781c_deinit_vsync_thread,
	.ois_vsync_signal = dw9781c_vsync_signal,
	.ois_get_lens_info = dw9781c_get_lens_info,
	.ois_format_otp_data = dw9781c_format_otp_data,
	.ois_set_sinewave = dw9781c_set_sinewave,
	.ois_set_stroke_limit =dw9781c_set_stroke_limit,
	.ois_set_pantilt =dw9781c_set_pantilt,
	.ois_reset = dw9781c_reset,
	.ois_set_smooth = dw9781c_set_smooth,
	.ois_set_tripod = dw9781c_set_tripod,
	.ois_set_circlewave = dw9781c_set_circlewave,
	.ois_af_crosstalk_compensation = dw9781c_af_crosstalk_compensation,
	.ois_set_servo = dw9781c_set_servo,
	.ois_log_control = dw9781c_log_control,
	.ois_get_loopgain = dw9781c_get_loopgain,
	.ois_get_oscillation_info = dw9781c_get_oscillation_info,
	.ois_get_oscd_input = dw9781c_get_oscd_input,
	.ois_set_loop_gain = dw9781c_set_loop_gain,
};

/*ic entry expose to ois_core*/
void dw9781c_get_ops(struct ois *ois)
{
	ois->ops = &dw9781c_ois_ops;
}
