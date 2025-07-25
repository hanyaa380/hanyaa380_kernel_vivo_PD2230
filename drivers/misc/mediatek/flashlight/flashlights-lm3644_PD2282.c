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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
/*#define HOPE_ADD_IRQ_FLASH*/
#ifdef HOPE_ADD_IRQ_FLASH
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#endif

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef LM3644_DTNAME
#define LM3644_DTNAME "mediatek,flashlights_lm3644"
#endif
#ifndef LM3644_DTNAME_I2C
#define LM3644_DTNAME_I2C "mediatek,flashlights_lm3644_i2c"
#endif

#define LM3644_NAME "flashlights-lm3644"

/* define registers */
#define LM3644_REG_ENABLE (0x01)
#define LM3644_MASK_ENABLE_LED1 (0x01)
#define LM3644_MASK_ENABLE_LED2 (0x02)
#define LM3644_DISABLE (0x00)
#define LM3644_ENABLE_LED1 (0x01)
#define LM3644_ENABLE_LED1_TORCH (0x09)
#define LM3644_ENABLE_LED1_FLASH (0x0D)
#define LM3644_ENABLE_LED2 (0x02)
#define LM3644_ENABLE_LED2_TORCH (0x0A)
#define LM3644_ENABLE_LED2_FLASH (0x0E)

#define LM3644_REG_TORCH_LEVEL_LED1 (0x05)
#define LM3644_REG_FLASH_LEVEL_LED1 (0x03)
#define LM3644_REG_TORCH_LEVEL_LED2 (0x06)
#define LM3644_REG_FLASH_LEVEL_LED2 (0x04)
#define LM3644_REG_BOOST_CONVERTER (0x07)
#define LM3644_REG_TORCH_PIN_MODE (0x13)
#define REG_FLASH_IC_ID (0x00)
#define REG_TEMP_PROTECT (0x09)

#define AW3644_FLASH_IC_ID_VALUE (0x36)
#define AW36515_FLASH_IC_ID_VALUE (0x30)
#define LM3644_FLASH_IC_ID_VALUE (0x00)
#define LM3644_TEMP_PROTECT_VALUE (0x08)
#define KTD2687_TEMP_PROTECT_VALUE (0x00)

#define LM3644_BOOST_CONVERTER (0x09)
#define LM3644_BOOST_CONVERTER_PASS_MODE (0x0C)

#define LM3644_REG_FLAGS_1 (0x0A)
#define LM3644_REG_FLAGS_2 (0x0B)
#define LM3644_DEVICE_ID (0x0C)

#define LM3644_REG_TIMING_CONF (0x08)
#define LM3644_TORCH_RAMP_TIME (0x10)
#define LM3644_FLASH_TIMEOUT   (0x0F)
#define AW36413_FLASH_TIMEOUT_160MS   (0x03)
#define LM3644_FLASH_TIMEOUT_150MS   (0x0A)
#define LM3644TT_FLASH_TIMEOUT (0x09)

#define LM3644_REG_DEVICE_ID (0x0C)
#define LM3644_REG_BOOST (0x07)
#define LM3644_REG_BOOST_CRRRENT_LIMIT_2_8A (0x09)
#define LM3644_REG_BOOST_CRRRENT_LIMIT_1_8A (0x08)

/* define channel, level */
#define LM3644_CHANNEL_NUM 4
#define LM3644_CHANNEL_CH1 0
#define LM3644_CHANNEL_CH2 1
#define LM3644_CHANNEL_CH3 2
#define LM3644_CHANNEL_CH4 3

/*  TORCH BRT
 *	min 1465uA, step 1465uA, max 187500uA
 */
#define KTD2687_TORCH_BRT_MIN 1465
#define KTD2687_TORCH_BRT_STEP 1465
#define KTD2687_TORCH_BRT_MAX 187500
#define KTD2687_TORCH_BRT_mA_TO_REG(a)	\
	((a) < KTD2687_TORCH_BRT_MIN ? 0 :	\
	 ((a / KTD2687_TORCH_BRT_STEP) - 1))

#define LM3644_LEVEL_NUM 32
#define LM3644_LEVEL_TORCH 7

#define LM3644_HW_TIMEOUT 350 /* ms */

#define LM3644_TURN_ON_LED 1
#define LM3644_TURN_OFF_LED 0

#define LM3644_REGULATOR_FRONT_ENABLE 3
#define LM3644_REGULATOR_REAR_ENABLE 2
#define LM3644_REGULATOR_TYPE_ENABLE 1
#define LM3644_REGULATOR_TYPE_TORCH 0
/* define mutex and work queue */
static DEFINE_MUTEX(lm3644_mutex);

static struct work_struct lm3644_work_ch1;
static struct work_struct lm3644_work_ch2;



#ifdef HOPE_ADD_IRQ_FLASH
#define LM3644_PINCTRL_STATE_FLASH_IRQ "flash_irq"
#endif



#ifdef HOPE_ADD_IRQ_FLASH
static int g_flash_irq_num;
static  unsigned int g_gpio_pin;
static unsigned int g_gpio_headset_deb;
static struct pinctrl_state *lm3644_flash_irq;
static unsigned int g_accdet_eint_type = IRQ_TYPE_LEVEL_LOW;
static struct delayed_work ir_delayed_work;
static unsigned int led_count;
static unsigned int irq_enable_count;
static unsigned int first_led_on;
static ktime_t StartTime;
static struct timer_list flash_delay_off_timer;
static struct work_struct flash_delay_off_work;
static void flash_delay_off_func(struct work_struct *work);
static void set_flash_led_delay_off(unsigned long arg);
unsigned int sof_flag;
#endif


/* define usage count */
static int use_count;
static int lock_touch;  /*hope add*/
static int lock_touch_sub; /*hope add*/

static int fl_ic_number;
/* define i2c */
static struct i2c_client *lm3644_i2c_client;

/* platform data */
struct lm3644_platform_data {
	int channel_num;
	struct regulator *regulator_enable;
	struct regulator *torch_enable;
	struct regulator *front_regulator_enable;
	struct regulator *rear_regulator_enable;
	struct flashlight_device_id *dev_id;
};

/* lm3644 chip data */
struct lm3644_chip_data {
	struct i2c_client *client;
	int device_id;
	unsigned char lm3644_reg_enable;
	int state_pin_short;//20210519 zhujianjia add for I2C short to GND test.
	int state_pin_short_ignore;
	struct lm3644_platform_data *pdata;
	struct mutex lock;
};


/******************************************************************************
 * lm3644 operations
 *****************************************************************************/

/*
 * IC LM3644TT:
 *    Torch:(Code x 2.8) + 1.954 mA
 *    Flash:(Code x 11.725) + 10.9 mA
 * IC LM3644:
 *    Torch:(Code x 1.4) + 0.977 mA
 *    Flash:(Code x 11.725) + 10.9 mA
 * IC KTD2687:
 *    Torch:(Code +1) x 187.5mA / 128
 *    Flash:(Code +1) x 1500 / 128 (mA)
 * IC AW36413, AW3644, AW36515:
 *    Torch:(Code * 2.91) + 2.55 mA
 *    Flash:(Code*11.72mA) + 11.35mA
 */
#if defined(CONFIG_MTK_CAM_PD2282)

static const int lm3644_current[LM3644_LEVEL_NUM] = {  /*current:mA */
	 32,  38,  48,  60,  67, 76, 98, 124,  246, 304,
	351, 398, 445, 503, 550, 597, 625, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100, 1150, 1200, 1250, 1300,
	1350, 1400};

//               48    60    67    76    98    124
static const unsigned char ktd2687_torch_level[LM3644_LEVEL_NUM] = {
	0x15, 0x19, 0x21, 0x29, 0x2E, 0x33, 0x43, 0x43, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};

static const unsigned char lm3644tt_torch_level[LM3644_LEVEL_NUM] = {
	0x0B, 0x0D, 0x11, 0x15, 0x18, 0x23, 0x2B, 0x42, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};

static const unsigned char lm3644_torch_level[LM3644_LEVEL_NUM] = {
	0x16, 0x1B, 0x23, 0x2B, 0x30, 0x46, 0x56, 0x85, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};

static const unsigned char aw36413_torch_level[LM3644_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x21, 0x27, 0x2F, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

//ktd2687  (code + 1)*1500/128
static const unsigned char lm3644_flash_level[LM3644_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x34, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D, 0x63, 0x67, 0x6b, 0x70,
	0x74, 0x78};

#else

static const int lm3644_current[LM3644_LEVEL_NUM] = {  /*current:mA */
	 33,  51,  65,  75,  93, 116, 140, 198,  246, 304,
	351, 398, 445, 503, 550, 597, 656, 703,  750, 796,
	857, 902, 961, 1008, 1067, 1100};

static const unsigned char ktd2687_torch_level[LM3644_LEVEL_NUM] = {
	0x15, 0x1F, 0x2B, 0x32, 0x3F, 0x51, 0x5E, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char lm3644tt_torch_level[LM3644_LEVEL_NUM] = {
	0x0B, 0x10, 0x16, 0x1A, 0x21, 0x2A, 0x31, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char lm3644_torch_level[LM3644_LEVEL_NUM] = {
	0x17, 0x21, 0x2E, 0x35, 0x42, 0x52, 0x63, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static const unsigned char aw36413_torch_level[LM3644_LEVEL_NUM] = {
	0x0A, 0x11, 0x15, 0x19, 0x1F, 0x27, 0x2F, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* torch duty=0x17 << current=33mA */

static const unsigned char lm3644_flash_level[LM3644_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};
#endif


static int lm3644_level_ch1 = -1;
static int lm3644_level_ch2 = -1;

static int lm3644_is_torch(int level)
{
	if(level >= 128) //add for dual tempture led realtime set level
		return 0;

	if (level >= LM3644_LEVEL_TORCH)
		return -1;

	return 0;
}

static int lm3644_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= LM3644_LEVEL_NUM)
		level = LM3644_LEVEL_NUM - 1;

	return level;
}

static int __lm3644_set_power(struct regulator * enable,  bool on)
{
	int ret = 0;

	if (enable) {
		if (on == regulator_is_enabled(enable))
			return ret;
		pr_info("E on(%d) %s ", on, on ? "ON" : "OFF");
		if (on) {
			ret = regulator_enable(enable);
			mdelay(2);
		} else {
			ret = regulator_disable(enable);
		}
	} else {
		pr_info("regulator_enable is null");
		return ret;
	}
	return ret;
}
static int lm3644_set_power(struct i2c_client *client, int regulator_type, int on)
{
	struct lm3644_platform_data  *fled_data = dev_get_platdata(&client->dev);

	if(regulator_type == LM3644_REGULATOR_TYPE_ENABLE) {
		if(IS_ERR(fled_data->regulator_enable))
			return PTR_ERR(fled_data->regulator_enable);
		__lm3644_set_power(fled_data->regulator_enable, on);
	} else if(regulator_type == LM3644_REGULATOR_TYPE_TORCH) {
		if(IS_ERR(fled_data->torch_enable))
			return PTR_ERR(fled_data->torch_enable);
		__lm3644_set_power(fled_data->torch_enable, on);
	} else if(regulator_type == LM3644_REGULATOR_REAR_ENABLE) {
		if(IS_ERR(fled_data->rear_regulator_enable))
			return PTR_ERR(fled_data->rear_regulator_enable);
		__lm3644_set_power(fled_data->rear_regulator_enable, on);
	} else if(regulator_type == LM3644_REGULATOR_FRONT_ENABLE) {
		if(IS_ERR(fled_data->front_regulator_enable))
			return PTR_ERR(fled_data->front_regulator_enable);
		__lm3644_set_power(fled_data->front_regulator_enable, on);
	} else {
		pr_err("regulator type is invalid");
		return 0;
	}
	return 0;
}

static int lm3644_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	if(chip->state_pin_short_ignore == 0){
		if(chip->state_pin_short == 1){ //20210519 zhujianjia add for I2C short to GND test.
	    return 0;
		}
	}
	lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_ON_LED);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);
	chip->state_pin_short = val<0?1:0;
	return val;
}

/* i2c wrapper function */
static int lm3644_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	if(chip->state_pin_short_ignore == 0){
		if(chip->state_pin_short == 1){ //20210519 zhujianjia add for I2C short to GND test.
	    return 0;
		}
	}
	pr_info("state_pin_short =%d\n", chip->state_pin_short);
 	if(chip->state_pin_short == 1){
		pr_info("chip.state_pin_short =%d\n", chip->state_pin_short);
	 	return 0;
 	}

	lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_ON_LED);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if(ret == EIO){
		pr_err("EIO chip.state_pin_short =%d\n",chip->state_pin_short);
		chip->state_pin_short = 1;
	}
	mutex_unlock(&chip->lock);

	if (ret < 0){
		pr_err("failed writing at 0x%02x\n", reg);
		chip->state_pin_short = 1;//20210519 zhujianjia add for I2C short to GND test.
	}
	return ret;
}

static int lm3644_get_device_id(struct i2c_client *client)
{
	unsigned char device_id_val;
    unsigned char flash_ic_id_val;
    unsigned char temp_protect_val;
	//Device ID: Distinguish AW36413 or (LM3644,KTD2687,AW3644,AW36525)
	device_id_val = lm3644_read_reg(client, LM3644_REG_DEVICE_ID);
	flash_ic_id_val = lm3644_read_reg(client, REG_FLASH_IC_ID);
	temp_protect_val = lm3644_read_reg(client, REG_TEMP_PROTECT);

	if(device_id_val == AW36413_DEVIC_ID_VALUE) {
		pr_info("IC is AW36413, DEVICE ID = 0x%x", AW36413_DEVIC_ID_VALUE);
		return AW36413_DEVIC_ID_VALUE;
	} else if(device_id_val == LM3644TT_DEVICE_ID_VALUE) {
		pr_info("IC is LM3644TT, DEVICE ID = 0x%x", LM3644TT_DEVICE_ID_VALUE);
		return LM3644TT_DEVICE_ID_VALUE;
	} else if(device_id_val == DEVICE_ID_DEFAULT_VALUE) {
		if(flash_ic_id_val == AW3644_FLASH_IC_ID_VALUE) {
			pr_info("IC is AW3644, DEVICE ID = 0x%x, Flash IC_ID = 0x%x", DEVICE_ID_DEFAULT_VALUE, flash_ic_id_val);
			return AW3644_DEVIC_ID_VALUE;
		} else if(flash_ic_id_val == AW36515_FLASH_IC_ID_VALUE) {
			pr_info("IC is AW36515, DEVICE ID = 0x%x, Flash IC_ID = 0x%x", DEVICE_ID_DEFAULT_VALUE, flash_ic_id_val);
			return AW36515_DEVIC_ID_VALUE;
		} else if(temp_protect_val == LM3644_TEMP_PROTECT_VALUE) {
			pr_info("IC is LM3644, DEVICE ID = %d, Temp Protect = 0x%x", DEVICE_ID_DEFAULT_VALUE, temp_protect_val);
			return LM3644_DEVICE_ID_VALUE;
		} else {
			pr_info("IC is KTD2687, DEVICE ID = %d, Temp Protect = 0x%x", DEVICE_ID_DEFAULT_VALUE, temp_protect_val);
			return KTD2687_DEVICE_ID_VALUE;
		}
	}
	return -EINVAL;
}
/* flashlight enable function */
static int lm3644_enable_ch1(struct i2c_client *client)
{
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	reg = LM3644_REG_ENABLE;
	if (!lm3644_is_torch(lm3644_level_ch1)) {
		/* torch mode */
		chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
	} else {
		/* flash mode */
		chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_FLASH;
	}
	val = chip->lm3644_reg_enable;

	return lm3644_write_reg(client, reg, val);
}

static int lm3644_enable_ch2(struct i2c_client *client)
{
#ifdef HOPE_ADD_IRQ_FLASH

	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	if (g_flash_irq_num > 0 && irq_enable_count == 0) {
		pr_debug("RED FLASH ON  g_flash_irq_num =%d, device_id = 0x%x\n", g_flash_irq_num, chip->device_id);
		irq_enable_count++;
		led_count = 0;
		StartTime = ktime_get();
		first_led_on = 0;
		sof_flag = 0;
		reg = LM3644_REG_TIMING_CONF;

		if (chip->device_id == AW36413_DEVIC_ID_VALUE)
			val = LM3644_TORCH_RAMP_TIME | AW36413_FLASH_TIMEOUT_160MS; /*setting time_out = 160 ms*/
		else
			val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT_150MS; /*setting time_out = 150 ms*/
		 lm3644_write_reg(client, reg, val);
		 enable_irq(g_flash_irq_num);
		 return 0;
	} else {
		pr_err("lm3644_enable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d, device_id = %d\n", g_flash_irq_num, irq_enable_count, device_id);
		return -1;
	}
#else

	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	if(!chip)
		return 0;

	reg = LM3644_REG_ENABLE;
	if (!lm3644_is_torch(lm3644_level_ch2)) {
		/* torch mode */
		chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
	} else {
		/* flash mode */
		chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_FLASH;
	}
	val = chip->lm3644_reg_enable;
	return lm3644_write_reg(client, reg, val);

#endif
}
static int lm3644_enable(struct i2c_client *client,int channel)
{
	lm3644_write_reg(client, LM3644_REG_BOOST_CONVERTER, LM3644_BOOST_CONVERTER_PASS_MODE);

	if (channel == LM3644_CHANNEL_CH1)
		lm3644_enable_ch1(client);
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_enable_ch2(client);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_enable_store(struct i2c_client *client,int channel)
{
	if (channel == LM3644_CHANNEL_CH1) {
		pr_info("vivo lm3644_enable_store ch1");
		lm3644_enable_ch1(client);
	} else if (channel == LM3644_CHANNEL_CH2) {
		pr_info("vivo lm3644_enable_store ch2");
		lm3644_enable_ch2(client);
	} else if(channel == LM3644_CHANNEL_CH3) {
		pr_info("vivo lm3644_enable_store ch3 front");
		lm3644_enable_ch1(client);
	} else if(channel == LM3644_CHANNEL_CH4) {
		pr_info("vivo lm3644_enable_store ch4 front");
		lm3644_enable_ch2(client);
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight disable function */
static int lm3644_disable_ch1(struct i2c_client *client)
{
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	reg = LM3644_REG_ENABLE;
	if (chip->lm3644_reg_enable & LM3644_MASK_ENABLE_LED2) {
		/* if LED 2 is enable, disable LED 1 */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
	} else {
		/* if LED 2 is disable, disable LED 1 and clear mode */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1_FLASH);
	}
	val = chip->lm3644_reg_enable;

	lm3644_level_ch1 = -1;
	return lm3644_write_reg(client, reg, val);
}

static int lm3644_disable_ch2(struct i2c_client *client)
{

#ifdef HOPE_ADD_IRQ_FLASH
	unsigned char reg, val;
	if (g_flash_irq_num > 0 && irq_enable_count == 1) {
		pr_debug("RED FLASH OFF  g_flash_irq_num =%d\n", g_flash_irq_num);
		disable_irq(g_flash_irq_num);
		irq_enable_count--;
		led_count = 0;
		StartTime = ktime_get();
		first_led_on = 0;
		sof_flag = 0;
		del_timer(&flash_delay_off_timer);
		/* set torch current ramp time and flash timeout */
		reg = LM3644_REG_TIMING_CONF;
		val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;  /*setting time_out = max default*/
		return lm3644_write_reg(client, reg, val);
	}else{
		pr_err("lm3644_disable_ch2 g_flash_irq_num is error g_flash_irq_num = %d, irq_enable_count = %d\n", g_flash_irq_num, irq_enable_count);
		return -1;
	}
#else

	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	reg = LM3644_REG_ENABLE;
	if (chip->lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = chip->lm3644_reg_enable;
	lm3644_level_ch2 = -1;
	return lm3644_write_reg(client, reg, val);
#endif
}


static int lm3644_disable_ch2_store(struct i2c_client *client)
{
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	reg = LM3644_REG_ENABLE;
	if (chip->lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = chip->lm3644_reg_enable;
	return lm3644_write_reg(client, reg, val);
}
static int lm3644_disable(struct i2c_client *client, int channel)
{

	if (channel == LM3644_CHANNEL_CH1)
		lm3644_disable_ch1(client);
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_disable_ch2(client);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;

}

static int lm3644_disable_store(struct i2c_client *client, int channel)
{
	if (channel == LM3644_CHANNEL_CH1) {
		lm3644_disable_ch1(client);
    } else if (channel == LM3644_CHANNEL_CH2) {
		lm3644_disable_ch2_store(client);
	} else if (channel == LM3644_CHANNEL_CH3) {
		lm3644_disable_ch1(client);
	} else if (channel == LM3644_CHANNEL_CH4) {
		lm3644_disable_ch2(client);
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int lm3644_set_level_ch1(struct i2c_client *client, int level)
{
	int ret;
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	level = lm3644_verify_level(level);

	/* set torch brightness level */
	reg = LM3644_REG_TORCH_LEVEL_LED1;
	if (chip->device_id == AW36413_DEVIC_ID_VALUE)
		val = aw36413_torch_level[level];
	else if(chip->device_id == KTD2687_DEVICE_ID_VALUE)
		val = ktd2687_torch_level[level];
	else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE)
		val = lm3644tt_torch_level[level];
	else
		val = lm3644_torch_level[level];
	ret = lm3644_write_reg(client, reg, val);

	lm3644_level_ch1 = level;

	/* set flash brightness level */
	reg = LM3644_REG_FLASH_LEVEL_LED1;
	val = lm3644_flash_level[level];
	ret = lm3644_write_reg(client, reg, val);
	return ret;
}

static int lm3644_set_level_ch2(struct i2c_client *client, int level)
{
	int ret;
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	level = lm3644_verify_level(level);

	/* set torch brightness level */
	reg = LM3644_REG_TORCH_LEVEL_LED2;

	if (chip->device_id == AW36413_DEVIC_ID_VALUE)
		val = aw36413_torch_level[level];
	else if(chip->device_id == KTD2687_DEVICE_ID_VALUE)
		val = ktd2687_torch_level[level];
	else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE)
		val = lm3644tt_torch_level[level];
	else
		val = lm3644_torch_level[level];

	ret = lm3644_write_reg(client, reg, val);

	lm3644_level_ch2 = level;

	/* set flash brightness level */
	reg = LM3644_REG_FLASH_LEVEL_LED2;
	val = lm3644_flash_level[level];
	ret = lm3644_write_reg(client, reg, val);

	return ret;
}

#ifdef HOPE_ADD_IRQ_FLASH
static int lm3644_set_red_flash_level_and_enabe_ch2(struct i2c_client * client)
{
	int ret;
	//unsigned char reg, val;
	/* set torch current ramp time and flash timeout */
	//reg = LM3644_REG_TIMING_CONF;
	//val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT_90MS;
	//ret = lm3644_write_reg(client, reg, val);

	/* set flash brightness level */
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	if(!chip)
		return 0;
	chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_FLASH;

	ret = lm3644_write_reg(client, LM3644_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*write reg5 bit 7 to 0*/
	ret = lm3644_write_reg(client, LM3644_REG_FLASH_LEVEL_LED2, 0x6E);/*6E = 1.3A*/
	ret = lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
	del_timer(&flash_delay_off_timer);
	flash_delay_off_timer.expires = jiffies + msecs_to_jiffies(100);  /*100ms*/
	flash_delay_off_timer.data = 0;
	flash_delay_off_timer.function = set_flash_led_delay_off;
	add_timer(&flash_delay_off_timer);
	return ret;
}
#endif

static int lm3644_set_level(struct i2c_client *client, int channel, int level)
{
	int ret;
	unsigned char reg, val;
	/* set torch current ramp time and flash timeout */

	reg = LM3644_REG_TIMING_CONF;
	val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;
	ret = lm3644_write_reg(client, reg, val);

	if (channel == LM3644_CHANNEL_CH1)
		lm3644_set_level_ch1(client, level);
	else if (channel == LM3644_CHANNEL_CH2)
		lm3644_set_level_ch2(client, level);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int lm3644_init(struct i2c_client *client)
{
	int ret;
	unsigned char reg, val;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
    if(IS_ERR(chip))
		return -EINVAL;

	if (lock_touch == 0 && lock_touch_sub == 0) {
	/* clear enable register */
	reg = LM3644_REG_ENABLE;
	val = LM3644_DISABLE;
	ret = lm3644_write_reg(client, reg, val);

	chip->lm3644_reg_enable = val;
	}

	/* set torch current ramp time and flash timeout */
	reg = LM3644_REG_TIMING_CONF;
	val = LM3644_TORCH_RAMP_TIME | LM3644_FLASH_TIMEOUT;
	ret = lm3644_write_reg(client, reg, val);

#ifdef CONFIG_MTK_CAM_PD2083F_EX
	/* set boost current limit config */
	reg = LM3644_REG_BOOST;
	val = LM3644_REG_BOOST_CRRRENT_LIMIT_1_8A;
	ret = lm3644_write_reg(client, reg, val);
	//mdelay(2);
	//boost_val = lm3644_read_reg(client, LM3644_REG_BOOST);
	//pr_err("lm3644_init LM3644_REG_BOOST =0x%x\n",boost_val);
#endif
	lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
	return ret;
}

/* flashlight uninit */
int lm3644_uninit(struct i2c_client *client)
{
	if (lock_touch == 0 && lock_touch_sub == 0) {
		lm3644_disable(client, LM3644_CHANNEL_CH1);
		lm3644_disable(client, LM3644_CHANNEL_CH2);
		lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		pr_debug("lm3644_uninit LM3644_PINCTRL_PINSTATE_LOW\n");
	}
	return 0;
}

#ifdef HOPE_ADD_IRQ_FLASH
static irqreturn_t vivo_subflash_ISR(int irq, void *dev_id)
{
	/*pr_debug("vivo_subflash_ISR \n");*/
	schedule_delayed_work(&ir_delayed_work, msecs_to_jiffies(0));
	return IRQ_HANDLED;
}
static void ir_delayed_func(struct work_struct *work)
{

	/*int ret;*/
	unsigned char reg, flags_1_val, flags_2_val;
	u64 deltaTime = ktime_us_delta(ktime_get(), StartTime);
	pr_debug("hope  led_count = %d, first_led_on =%d sof_flag = %d\n", led_count, first_led_on, sof_flag);
	pr_debug("hope  deltaTime %lluus\n", ktime_us_delta(ktime_get(), StartTime));
	if(first_led_on == 0 && sof_flag ==1 ){
			first_led_on = 2;
	}

	if(!lm3644_i2c_client ){
		pr_err("i2c client is NULL.\n");
	}else{
		if ((deltaTime > 480000 && led_count < 8) || (first_led_on == 2)) {/*380000*/
			StartTime = ktime_get();
			led_count++;

			reg = LM3644_REG_FLAGS_1;
			flags_1_val = lm3644_read_reg(lm3644_i2c_client,reg);

			reg = LM3644_REG_FLAGS_2;
			flags_2_val = lm3644_read_reg(lm3644_i2c_client,reg);

			if((flags_1_val & 0x7E)||(flags_2_val & 0x1E))
				pr_err("flashlight err flags_1_val = 0x%x, flags_2_val = 0x%x, lm3644_i2c_client->addr = 0x%x, device_id =0x%x\n", flags_1_val, flags_2_val, lm3644_i2c_client->addr, device_id);

				pr_debug("sub flash mode start on \n");
				lm3644_set_red_flash_level_and_enabe_ch2();
		}
		if(first_led_on != 0)
			first_led_on++;
	}
}

static void set_flash_led_delay_off(unsigned long arg)
{
	pr_debug("set_flash_led_delay_off \n");
	schedule_work(&flash_delay_off_work);
	del_timer(&flash_delay_off_timer);
}

static void flash_delay_off_func(struct work_struct *work)
 {
	unsigned char reg, val;
	reg = LM3644_REG_ENABLE;

	pr_err("flash_delay_off_func start\n");

	if (lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
		/* if LED 1 is enable, disable LED 2 */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
	} else {
		/* if LED 1 is disable, disable LED 2 and clear mode */
		lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
	}
	val = lm3644_reg_enable;
	 lm3644_write_reg(lm3644_i2c_client, reg, val);

	 pr_err("flash_delay_off_func end\n");
}

#endif

/*
 * IC LM3644TT:
 *    Torch:(Code x 2.8) + 1.954 mA  100mA-->0x23
 * IC LM3644:
 *    Torch:(Code x 1.4) + 0.977 mA  100mA-->0x46
 * IC KTD2687:
 *    Torch:(Code +1) x 187.5mA / 128  100mA--->0x43
 * IC AW36413, AW3644, AW36515:
 *    Torch:(Code * 2.91) + 2.55 mA   100mA--->0x21
 * Product Current is 100ma/single led
 */
static int set_flashlight_state(struct i2c_client * client, int type, int ct, int part, int state)
{
	int single_led_arg = 0;
	static int back_cold_flag = 0, back_warm_flag = 0;
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	if(IS_ERR(chip))
		return -EINVAL;

	lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_ON_LED);
	lm3644_write_reg(client, LM3644_REG_BOOST_CONVERTER, LM3644_BOOST_CONVERTER_PASS_MODE);
	single_led_arg = (ct << 1) | part;
	pr_info("set_flashlight_state check state:%d single_led_arg:%d\n", state, single_led_arg);
	switch (state) {

	case BACK_TORCH_4LED_ONE_ON:
		pr_info("%s:%d [AT+BK_SNS_FLASHLIGHT=1]\n", __func__, __LINE__);
		lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_ON_LED);
		switch (single_led_arg)
		{
			case BACK_WARMIC_LED1: //1
				pr_info("%s:%d open back led1\n", __func__, __LINE__);
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x43);/*152mA (Code +1) x 187.5mA / 128  Torch*/
				break;
			case BACK_COLDIC_LED1: //2
				pr_info("%s:%d open back led2\n", __func__, __LINE__);
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x43);/*152mA (Code +1) x 187.5mA / 128  Torch*/
				break;
			case BACK_WARMIC_LED2: //3
				pr_info("%s:%d open back led3\n", __func__, __LINE__);
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x43);/*152mA (Code +1) x 187.5mA / 128  Torch*/
				break;
			case BACK_COLDIC_LED2: //0
				pr_info("%s:%d open back led4\n", __func__, __LINE__);
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x43);/*152mA (Code +1) x 187.5mA / 128  Torch*/
				break;
			default:
				break;
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch = 1;
		break;
	case BACK_TORCH_4LED_ONE_OFF:
		pr_info("%s:%d [AT+BK_SNS_FLASHLIGHT=0]\n", __func__, __LINE__);
		switch (single_led_arg)
		{
			case BACK_WARMIC_LED1:
				back_warm_flag = 1;
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				break;
			case BACK_COLDIC_LED1:
				back_cold_flag |= 1;
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				break;
			case BACK_WARMIC_LED2:
				back_warm_flag = 1;
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				break;
			case BACK_COLDIC_LED2:
				back_cold_flag |= 2;
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				break;
			default:
				break;
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		if(part == 1)
			chip->lm3644_reg_enable = 0;
		lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_OFF_LED);
		if(back_cold_flag == 3)
		{
			back_cold_flag = 0;
			lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		}
		if(back_warm_flag == 1)
		{
			back_warm_flag = 0;
			lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		}
		lock_touch = 0;
		break;

	case FRONT_TORCH_3rd_ONE_ON:
		pr_info("FRONT_TORCH_3rd_ONE_ON\n");
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
			if (chip->device_id == AW36413_DEVIC_ID_VALUE) //AW3644,AW36413,AW36515
			{
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x0D);/*(Code * 2.91) + 2.55 mA  Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x0D);/*(Code * 2.91) + 2.55 mA  Torch*/
			} else if(chip->device_id == LM3644_DEVICE_ID_VALUE) { //LM3644
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x1C);/*(Code x 1.4) + 0.977 mA Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x1C);/*(Code x 1.4) + 0.977 mA Torch*/
			} else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE) { //LM3644TT
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x0E);/*(Code x 2.8) + 1.954 mA Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x0E);/*(Code x 2.8) + 1.954 mA Torch*/
			}
			else{ //KTD2687
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x1A);/*(Code +1) x 187.5mA / 128  Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x1A);/*(Code +1) x 187.5mA / 128  Torch*/
			}
		} else {
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
			if (chip->device_id == AW36413_DEVIC_ID_VALUE)
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x22);/*(Brightnees code x 1.4mA)+0.997ma Torch 0x1A = 78mA*/
			else
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*0x47 = 100mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA(Brightnees code x 1.4mA)+0.997ma Torch*/
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch = 1;
		break;
	case FRONT_TORCH_3rd_ONE_OFF:
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		} else {
			if (chip->lm3644_reg_enable & LM3644_MASK_ENABLE_LED2) {
				/* if LED 2 is enable, disable LED 1 */
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
			} else {
				/* disable LED 1 LED 2 and clear mode */
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1_FLASH);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
				lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
			}
		}
		lock_touch = 0;
		pr_info("FRONT_TORCH_3rd_ONE_OFF\n");
		break;
	case FRONT_TORCH_3rd_TWO_ON:
		pr_info("FRONT_TORCH_3rd_TWO_ON\n");
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
		}
        if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
			if (chip->device_id == AW36413_DEVIC_ID_VALUE) //AW3644,AW36413,AW36515
			{
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x0D);/*(Code * 2.91) + 2.55 mA  Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x0D);/*(Code * 2.91) + 2.55 mA  Torch*/
			} else if(chip->device_id == LM3644_DEVICE_ID_VALUE) { //LM3644
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x1C);/*(Code x 1.4) + 0.977 mA Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x1C);/*(Code x 1.4) + 0.977 mA Torch*/
			} else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE) { //LM3644TT
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x0E);/*(Code x 2.8) + 1.954 mA Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x0E);/*(Code x 2.8) + 1.954 mA Torch*/
			}
			else{ //KTD2687
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x36);/*(Code +1) x 187.5mA / 128  Torch*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x36);/*(Code +1) x 187.5mA / 128  Torch*/
			}
		} else {
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
			if (chip->device_id == AW36413_DEVIC_ID_VALUE){
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x22);/*write reg5 bit 7 to 0*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x22);/*default 0x33 = 150mA 0x1A = 78mA (Brightness Code*2.91mA)+2.55mA Torch*/
			}else{
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x35);/*write reg5 bit 7 to 0*/
				lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x35);/*default 2E == 65mA 0x47 = 100mA 0x6A = 150mA lm3644 0x38 = 80mA KTD2687 0x35 = 80mA (Brightnees code x 1.4mA)+0.997ma Torch*/
			}
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch_sub = 1;
		break;
	case FRONT_TORCH_3rd_TWO_OFF:
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		} else {
			if (chip->lm3644_reg_enable & LM3644_MASK_ENABLE_LED1) {
				/* if LED 1 is enable, disable LED 2 */
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
			} else {
				/* disable LED 1 LED 2 and clear mode */
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2_FLASH);
				lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
				lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
			}
		}
		pr_info("FRONT_TORCH_3rd_TWO_OFF\n");
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
		}
		lock_touch_sub = 0;
		break;
	case BBK_TORCH_LOW:
		pr_info("%s:%d [AT+BKBGD=2,1] open front torch LED2 \n", __func__, __LINE__);
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
		}
		mdelay(2);
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			pr_info("BBK_TORCH_LOW, AW36413_REG_TORCH_LEVEL_LED1 and LED2 = 0x43\n");
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
			lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x43);/*130mA (Code +1) x 187.5mA / 128  Torch*/
		}
		mdelay(2);
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch = 1;
		break;
	case BBK_TORCH_OFF:
		pr_info("%s:%d [AT+BKBGD=2,0] close front torch LED2 \n", __func__, __LINE__);
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
			lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		}
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
		}
		lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		lock_touch = 0;
		break;
	case FRONT_TORCH_ON:
		pr_info("%s:%d [AT+BKBGD=1] open front torch LED1 \n", __func__, __LINE__);
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
		}
		mdelay(2);
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
			lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0); /* clear led1 7bit to 0*/
			lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x43);/*130mA (Code +1) x 187.5mA / 128  Torch*/
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch_sub = 1;
		break;
	case FRONT_TORCH_OFF:
		pr_info("%s:%d [AT+BKBGD=0] close front torch LED1 \n", __func__, __LINE__);
		if(fl_ic_number == FRONT_FLASHLIGHT_DOUBLE_LED) {
			chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
			lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		}
		if(type == 1) { //front led
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
		}
		lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		lock_touch_sub = 0;
		break;
	case FRONT_TORCH_ONE_ON:
		lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
	 	switch (single_led_arg)
		{
			case FRONT_WARMIC_LED1: //0
			case FRONT_COLDIC_LED1: //1
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED1_TORCH;
				if (chip->device_id == AW36413_DEVIC_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x21);
				else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x23);
				else if(chip->device_id == LM3644_DEVICE_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x46);
				else
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, 0x43);
				break;
			case FRONT_COLDIC_LED2: //2
			case FRONT_WARMIC_LED2: //3
				chip->lm3644_reg_enable |= LM3644_ENABLE_LED2_TORCH;
				if (chip->device_id == AW36413_DEVIC_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x21);
				else if(chip->device_id == LM3644TT_DEVICE_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x23);
				else if(chip->device_id == LM3644_DEVICE_ID_VALUE)
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x46);
				else
					lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED2, 0x43);
				break;
			default:
				break;
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lock_touch = 1;
		break;
	case FRONT_TORCH_ONE_OFF:
	 	switch (single_led_arg)
		{
			case FRONT_WARMIC_LED1:
			case FRONT_COLDIC_LED1:
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED1);
				break;
			case FRONT_COLDIC_LED2:
			case FRONT_WARMIC_LED2:
				chip->lm3644_reg_enable &= (~LM3644_ENABLE_LED2);
				break;
			default:
				break;
		}
		lm3644_write_reg(client, LM3644_REG_ENABLE, chip->lm3644_reg_enable);
		lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
		lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
		lock_touch = 0;
		break;
	case BBK_FLASH_AT_TEST:
			lm3644_write_reg(client, LM3644_REG_TIMING_CONF, 0x1f);/*vivo liuguangwei change flash time out from 150ms to 400ms*/
			lm3644_write_reg(client, LM3644_REG_FLASH_LEVEL_LED1, 0x54);
			lm3644_write_reg(client, LM3644_REG_ENABLE, LM3644_ENABLE_LED1_FLASH);
		break;
	case BBK_FLASH_AT_OFF:
			lm3644_write_reg(client, LM3644_REG_ENABLE, (~LM3644_ENABLE_LED1_FLASH));
		break;
	default:
		pr_info("set_flashlight_state No such command and arg\n");
		return -ENOTTY;
	}
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer lm3644_timer_ch1;
static struct hrtimer lm3644_timer_ch2;
static unsigned int lm3644_timeout_ms[LM3644_CHANNEL_NUM];

static void lm3644_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	lm3644_disable_ch1(lm3644_i2c_client);
}

static void lm3644_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	lm3644_disable_ch2(lm3644_i2c_client);
}

static enum hrtimer_restart lm3644_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&lm3644_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart lm3644_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&lm3644_work_ch2);
	return HRTIMER_NORESTART;
}

static int lm3644_timer_start(int channel, ktime_t ktime)
{
	if (channel == LM3644_CHANNEL_CH1)
		hrtimer_start(&lm3644_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == LM3644_CHANNEL_CH2)
		hrtimer_start(&lm3644_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_timer_cancel(int channel)
{
	if (channel == LM3644_CHANNEL_CH1)
		hrtimer_cancel(&lm3644_timer_ch1);
	else if (channel == LM3644_CHANNEL_CH2)
		hrtimer_cancel(&lm3644_timer_ch2);
	else {
		pr_err("Error channel\n");
		return -1;
	}

	return 0;
}

static int lm3644_set_current_by_channel(struct i2c_client *client, int channel, unsigned int torch_current)
{
	int ret;
	unsigned char reg;
	unsigned char curret_reg = KTD2687_TORCH_BRT_mA_TO_REG(torch_current * 1000);

	if(curret_reg >= 128) {
		pr_err("Error current reg\n");
		return -1;
	}

	if(channel == LM3644_CHANNEL_CH1) {
		reg = LM3644_REG_TORCH_LEVEL_LED1;
		ret = lm3644_write_reg(client, reg, curret_reg);
	} else if(channel == LM3644_CHANNEL_CH2) {
		reg = LM3644_REG_TORCH_LEVEL_LED2;
		ret = lm3644_write_reg(client, reg, curret_reg);
	} else {
		pr_err("Error channel\n");
		return -1;
	}

	pr_info("channel = %d, torch_current = %u, curret_reg = %u\n", channel, torch_current, curret_reg);
	return 0;
}

static int lm3644_set_realtime_tempture_level(struct i2c_client *client, int level)
{
	unsigned int temp_current;
	unsigned int current_step;
	float current_step_table[] = {1.465, 2.93, 4.395, 5.86, 7.325}; // 1.465 * num(1/2/3/4)

	if((level & 0xFF) > REAR_TEMPERATURE_TORCH_MAX_LEVEL) {
		pr_err(" %s is over level!!!\n", __func__);
	}

	switch(level & 0xFF00) {
		case REAR_CAMERA_AELUX_LEVEL_BASE0:
			current_step = current_step_table[0] * 1000;
		break;
		case REAR_CAMERA_AELUX_LEVEL_BASE1:
			current_step = current_step_table[1] * 1000;
		break;
		case REAR_CAMERA_AELUX_LEVEL_BASE2:
			current_step = current_step_table[2] * 1000;
		break;
		case REAR_CAMERA_AELUX_LEVEL_BASE3:
			current_step = current_step_table[3] * 1000;
		break;
		case REAR_CAMERA_AELUX_LEVEL_BASE4:
			current_step = current_step_table[4] * 1000;
		break;
		default:
			current_step = current_step_table[0] * 1000;
		break;
	}
	temp_current = current_step * (level & 0xFF) / 1000;

	#if defined (CONFIG_SINGLE_FLASH_DUAL_LED)
		lm3644_set_current_by_channel(client, LM3644_CHANNEL_CH1, temp_current);
		lm3644_set_current_by_channel(client, LM3644_CHANNEL_CH2, temp_current);
	#else
		//lm3644_set_current_by_channel(client, channel, temp_current);
	#endif

	pr_info("current_step = %d, level = %d, temp_current = %d\n",
				current_step, level & 0xFF , temp_current);

	return 0;
}

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int lm3644_ioctl(struct i2c_client *client, unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int led_state;
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= LM3644_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		#if defined (CONFIG_SINGLE_FLASH_DUAL_LED)
		lm3644_timeout_ms[LM3644_CHANNEL_CH1] = fl_arg->arg;
		lm3644_timeout_ms[LM3644_CHANNEL_CH2] = fl_arg->arg;
		#else
		lm3644_timeout_ms[channel] = fl_arg->arg;
		#endif
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("FLASH_IOC_SET_DUTY(%d): %d lm3644_current[fl_arg->arg]=%d\n",
				channel, (int)fl_arg->arg,lm3644_current[fl_arg->arg]);
		#if defined (CONFIG_SINGLE_FLASH_DUAL_LED)
		lm3644_set_level(client, LM3644_CHANNEL_CH1, fl_arg->arg);
		lm3644_set_level(client, LM3644_CHANNEL_CH2, fl_arg->arg);
		#else
		lm3644_set_level(client, channel, fl_arg->arg);
		#endif
		break;
	case FLASH_IOC_SET_RT_TEMPTURE_DUTY:
		pr_info("FLASH_IOC_SET_RT_TEMPTURE_DUTY(%d): %d lm3644_current[fl_arg->arg]=%d\n",
				channel, (int)fl_arg->arg, lm3644_current[fl_arg->arg]);
		lm3644_set_realtime_tempture_level(client, fl_arg->arg);
		break;
	case FLASH_IOC_SET_ONOFF:
		pr_info("FLASH_IOC_SET_ONOFF(%d): %d, type:%d part:%d\n", channel, (int)fl_arg->arg, (int)fl_arg->type, (int)fl_arg->part);
		if (fl_arg->arg == 1) {
			if (lm3644_timeout_ms[channel]) {
				s = lm3644_timeout_ms[channel] / 1000;
				ns = lm3644_timeout_ms[channel] % 1000 *
					1000000;
				ktime = ktime_set(s, ns);
				lm3644_timer_start(channel, ktime);
			}
			#if defined (CONFIG_SINGLE_FLASH_DUAL_LED)
			if((int)fl_arg->type == 1) { //vivo front flash
				pr_info("===vivo enable front gpio regulator!\n");
				lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
			}
			if((int)fl_arg->part == 1) { //vivo rear warm
				pr_info("===vivo enable rear gpio regulator!\n");
				lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_ON_LED);
			}
			lm3644_enable(client, LM3644_CHANNEL_CH1);
			lm3644_enable(client, LM3644_CHANNEL_CH2);
			#else
			lm3644_enable(client, channel);
			#endif
		} else {
			if (lock_touch == 0 && lock_touch_sub == 0) {
				#if defined (CONFIG_SINGLE_FLASH_DUAL_LED)
				lm3644_disable(client, LM3644_CHANNEL_CH1);
				lm3644_disable(client, LM3644_CHANNEL_CH2);
				lm3644_timer_cancel(LM3644_CHANNEL_CH1);
				lm3644_timer_cancel(LM3644_CHANNEL_CH2);
				lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
				if((int)fl_arg->type == 1) { //vivo front flash
					pr_info("===vivo disable front gpio regulator!\n");
					lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
				}
				if((int)fl_arg->part == 1) { //vivo rear warm
					pr_info("===vivo disable rear gpio regulator!\n");
					lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_OFF_LED);
				}
				#else
				lm3644_disable(client, channel);
				lm3644_timer_cancel(channel);
				#endif
			}
		}
		break;

    case FLASH_IOCTL_SET_LED_STATE:
		pr_info("FLASH_IOCTL_SET_LED_STATE(channel %d): arg: %d\n",
				channel, (int)fl_arg->arg);
		led_state = (int)fl_arg->arg;
		set_flashlight_state(client, fl_arg->type, fl_arg->ct, fl_arg->part, led_state);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = LM3644_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = LM3644_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = lm3644_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = lm3644_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = LM3644_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int lm3644_open(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3644_release(void)
{
	/* Move to set driver for saving power */
	return 0;
}

static int lm3644_set_driver(struct i2c_client * client, int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&lm3644_mutex);
	if (set) {
		if (!use_count)
			ret = lm3644_init(client);
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = lm3644_uninit(client);
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&lm3644_mutex);

	return ret;
}

static ssize_t lm3644_strobe_store(struct i2c_client * client,struct flashlight_arg arg)
{
	int i;
	if (arg.channel == LM3644_CHANNEL_CH2 && arg.level == 27){
		if(arg.dur == 200)
			set_flashlight_state(client, arg.type, arg.ct, arg.part, 10);
		if(arg.dur == 300)
			set_flashlight_state(client, arg.type, arg.ct, arg.part, 11);
	} else {
		lm3644_set_driver(client, 1);
		if(arg.channel == LM3644_CHANNEL_CH2 && arg.level == 26){
			pr_info("====hope arg.channel = %d, arg.level = %d\n", arg.channel, arg.level);
			lm3644_level_ch2 = arg.level;
			lm3644_write_reg(client, LM3644_REG_TORCH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
			lm3644_write_reg(client, LM3644_REG_FLASH_LEVEL_LED1, (0x7f & 0xbf));/*default value is 0xbf,bit7 should set to 0*/
			lm3644_write_reg(client, LM3644_REG_FLASH_LEVEL_LED2, 0x6E);/*0x6E = 1.3A,0x65 = 1.2A*/
			for (i = 0; i < 10; i++){
				lm3644_enable_store(client, arg.channel);
				msleep(arg.dur);
				lm3644_disable_store(client, arg.channel);
				msleep(330);
				pr_info("====hope arg.dur = %d, disable = 330\n", arg.dur);
			}
		} else {
			dev_info(&(client->dev), "===vivo arg.channel = %d, arg.level = %d, arg.part = %d\n", arg.channel, arg.level, arg.part);
			if(arg.channel == LM3644_CHANNEL_CH3 || arg.channel == LM3644_CHANNEL_CH4) {
				pr_info("===vivo enable front gpio regulator!\n");
				lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_ON_LED);
			} else {
				pr_info("===vivo no nead enable front gpio regulator!\n");
			}
			if (arg.part == 1) { // rear warm
				pr_info("===vivo enable rear gpio regulator!\n");
				lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_ON_LED);
			} else {
				pr_info("===vivo no nead enable rear gpio regulator!\n");
			}
			lm3644_set_level(client,arg.channel, arg.level);
		}

		lm3644_timeout_ms[arg.channel] = 0;
		lm3644_enable_store(client, arg.channel);
		msleep(arg.dur);
		lm3644_disable_store(client, arg.channel);
		lm3644_set_driver(client, 0);
		if(arg.channel == LM3644_CHANNEL_CH3 || arg.channel == LM3644_CHANNEL_CH4) {
			pr_info("===vivo disable front gpio regulator!\n");
			lm3644_set_power(client, LM3644_REGULATOR_FRONT_ENABLE, LM3644_TURN_OFF_LED);
		} else {
			pr_info("===vivo no nead disable front gpio regulator!\n");
		}
		if (arg.part == 1) { // rear warm
			pr_info("===vivo disable rear gpio regulator!\n");
			lm3644_set_power(client, LM3644_REGULATOR_REAR_ENABLE, LM3644_TURN_OFF_LED);
		} else {
			pr_info("===vivo no nead disable rear gpio regulator!\n");
		}
	}
	return 0;
}

static struct flashlight_operations lm3644_ops = {
	lm3644_open,
	lm3644_release,
	lm3644_ioctl,
	lm3644_strobe_store,
	lm3644_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int lm3644_chip_init(struct lm3644_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" for power saving.
	 * lm3644_init();
	 */

	return 0;
}

static int lm3644_parse_dt(struct device *dev,
		struct lm3644_platform_data *pdata)
{
	struct device_node *np, *cnp;
	u32 decouple = 0;
	int i = 0;

	if (!dev || !dev->of_node || !pdata)
		return -ENODEV;

	np = dev->of_node;

	pdata->channel_num = of_get_child_count(np);
	if (!pdata->channel_num) {
		pr_info("Parse no dt, node.\n");
		return 0;
	}
	pr_info("Channel number(%d).\n", pdata->channel_num);

	if (of_property_read_u32(np, "decouple", &decouple))
		pr_info("Parse no dt, decouple.\n");

	pdata->dev_id = devm_kzalloc(dev,
			pdata->channel_num *
			sizeof(struct flashlight_device_id),
			GFP_KERNEL);
	if (!pdata->dev_id)
		return -ENOMEM;

	pdata->regulator_enable = devm_regulator_get(dev, "flash_enable");
  	if (IS_ERR(pdata->regulator_enable)) {
  		pr_info("regulator_enable get failed");
 		return PTR_ERR(pdata->regulator_enable);
  	}
	pdata->torch_enable = devm_regulator_get(dev, "torch_enable");
  	if (IS_ERR(pdata->torch_enable)) {
  		pr_info("torch_enable get failed");
		return PTR_ERR(pdata->torch_enable);
  	}
	pdata->front_regulator_enable = devm_regulator_get(dev, "front_flash_enable");
  	if (IS_ERR(pdata->front_regulator_enable)) {
  		pr_info("front_regulator_enable get failed");
 		return PTR_ERR(pdata->front_regulator_enable);
  	}
	pdata->rear_regulator_enable = devm_regulator_get(dev, "rear_flash_enable");
  	if (IS_ERR(pdata->rear_regulator_enable)) {
  		pr_info("rear_regulator_enable get failed");
 		return PTR_ERR(pdata->rear_regulator_enable);
  	}

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				LM3644_NAME);
		pdata->dev_id[i].channel = i;
		pdata->dev_id[i].decouple = decouple;

		pr_info("Parse dt (type,ct,part,name,channel,decouple)=(%d,%d,%d,%s,%d,%d).\n",
				pdata->dev_id[i].type, pdata->dev_id[i].ct,
				pdata->dev_id[i].part, pdata->dev_id[i].name,
				pdata->dev_id[i].channel,
				pdata->dev_id[i].decouple);
		i++;
	}

	return 0;

err_node_put:
	of_node_put(cnp);
	return -EINVAL;
}

static int lm3644_i2c_probe(
		struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3644_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3644_chip_data *chip;
	int err;
	int i;

	pr_debug("i2c probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct lm3644_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_free;
		}
		client->dev.platform_data = pdata;
		err = lm3644_parse_dt(&client->dev, pdata);
		if (err)
			goto err_free;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	lm3644_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&lm3644_work_ch1, lm3644_work_disable_ch1);
	INIT_WORK(&lm3644_work_ch2, lm3644_work_disable_ch2);

	/* init timer */
	hrtimer_init(&lm3644_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3644_timer_ch1.function = lm3644_timer_func_ch1;
	hrtimer_init(&lm3644_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	lm3644_timer_ch2.function = lm3644_timer_func_ch2;
	lm3644_timeout_ms[LM3644_CHANNEL_CH1] = 0;
	lm3644_timeout_ms[LM3644_CHANNEL_CH2] = 0;

	/* init chip hw */
	lm3644_chip_init(chip);

	/* clear usage count */
	use_count = 0;
	chip->state_pin_short_ignore = 1;
	/* register flashlight device */

	chip->device_id = lm3644_get_device_id(client);
	pr_info("lm3644_i2c_probe lm3644 DEVICE_ID = 0x%x\n", chip->device_id);
	if(chip->device_id  < 0) {
		pr_info("[marklee]i2c failed detect%d\n", chip->device_id);
		err = -EFAULT;
		goto err_free;
	}
	else
	{
		fl_ic_number = fl_ic_number + 1;
		pr_info("[marklee]i2c OK detect%d\n", chip->device_id);
	}
	chip->state_pin_short_ignore = 0;
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id_and_i2c_client(&pdata->dev_id[i], client, &lm3644_ops)) {
				err = -EFAULT;
				goto err_free;
			}
	} else {
		if (flashlight_dev_register(LM3644_NAME, &lm3644_ops)) {
			err = -EFAULT;
			goto err_free;
		}
	}
	/*set_flashlight_state(11); hope for test use */
	lm3644_set_power(client, LM3644_REGULATOR_TYPE_ENABLE, LM3644_TURN_OFF_LED);
	pr_debug("Probe done.\n");

	return 0;

err_free:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

static int lm3644_i2c_remove(struct i2c_client *client)
{
	int i;
	struct lm3644_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3644_chip_data *chip = i2c_get_clientdata(client);
	if(!chip)
		return 0;

	pr_debug("Remove start.\n");

	client->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(LM3644_NAME);

	/* flush work queue */
	flush_work(&lm3644_work_ch1);
	flush_work(&lm3644_work_ch2);

	/* free resource */
	kfree(chip);

	pr_debug("Remove done.\n");

	return 0;
}

static const struct i2c_device_id lm3644_i2c_id[] = {
	{LM3644_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id lm3644_i2c_of_match[] = {
	{.compatible = LM3644_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver lm3644_i2c_driver = {
	.driver = {
		.name = LM3644_NAME,
#ifdef CONFIG_OF
		.of_match_table = lm3644_i2c_of_match,
#endif
	},
	.probe = lm3644_i2c_probe,
	.remove = lm3644_i2c_remove,
	.id_table = lm3644_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int lm3644_probe(struct platform_device *dev)
{
#ifdef HOPE_ADD_IRQ_FLASH
	int ret=0;
	struct device_node *node = NULL;
	u32 ints1[4] = { 0 };
#endif
	pr_debug("Probe start.\n");

	if (i2c_add_driver(&lm3644_i2c_driver)) {
		pr_debug("Failed to add i2c driver.\n");
		return -1;
	}

#ifdef HOPE_ADD_IRQ_FLASH
	pinctrl_select_state(lm3644_pinctrl, lm3644_flash_irq);
	node = of_find_matching_node(node, lm3644_of_match);
	if (node) {
		g_gpio_pin = of_get_named_gpio(node, "deb-gpios", 0);
		ret = of_property_read_u32(node, "debounce", &g_gpio_headset_deb);
		if (ret < 0) {
			pr_debug("debounce time not found\n");
			return ret;
		}
		/*gpio_set_debounce(g_gpio_pin, g_gpio_headset_deb);*/

		g_flash_irq_num = irq_of_parse_and_map(node, 0);
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		g_accdet_eint_type = ints1[1];
		pr_debug("[flash] gpiopin:%d debounce:%d accdet_irq:%d accdet_eint_type:%d\n",
				g_gpio_pin, g_gpio_headset_deb, g_flash_irq_num, g_accdet_eint_type);
		ret = request_irq(g_flash_irq_num, vivo_subflash_ISR, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "flash_irq", NULL);
		if (ret != 0) {
			pr_debug("[flash]EINT IRQ LINE NOT AVAILABLE\n");
			goto ir_irq_err;
		} else {
			pr_debug("[flash]accdet set EINT finished, accdet_irq=%d, headsetdebounce=%d\n",
				     g_flash_irq_num, g_gpio_headset_deb);
		}

		INIT_DELAYED_WORK(&ir_delayed_work, ir_delayed_func);
		INIT_WORK(&flash_delay_off_work, flash_delay_off_func);
		init_timer(&flash_delay_off_timer); // add for flash 100ms off control

		disable_irq_nosync(g_flash_irq_num);

		pr_debug("hope \n");
	} else {
		pr_debug("[flash]%s can't find compatible node\n", __func__);
	}

	return 0;

ir_irq_err:
	free_irq(g_flash_irq_num, NULL);
	return -1;
#endif
	pr_debug("Probe done.\n");

	return 0;
}

static int lm3644_remove(struct platform_device *dev)
{
	pr_debug("Remove start.\n");

	i2c_del_driver(&lm3644_i2c_driver);

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id lm3644_of_match[] = {
	{.compatible = LM3644_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, lm3644_of_match);
#else
static struct platform_device lm3644_platform_device[] = {
	{
		.name = LM3644_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, lm3644_platform_device);
#endif

static struct platform_driver lm3644_platform_driver = {
	.probe = lm3644_probe,
	.remove = lm3644_remove,
	.driver = {
		.name = LM3644_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lm3644_of_match,
#endif
	},
};

static int __init flashlight_lm3644_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&lm3644_platform_device);
	if (ret) {
		pr_err("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&lm3644_platform_driver);
	if (ret) {
		pr_err("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_lm3644_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&lm3644_platform_driver);

	pr_debug("Exit done.\n");
}

late_initcall(flashlight_lm3644_init);
module_exit(flashlight_lm3644_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ziyu Jiang <jiangziyu@meizu.com>");
MODULE_DESCRIPTION("MTK Flashlight LM3644 Driver");

