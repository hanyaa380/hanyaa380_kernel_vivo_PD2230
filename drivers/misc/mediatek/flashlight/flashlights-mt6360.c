// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/power_supply.h>

#include "richtek/rt-flashlight.h"
#include "v1/mtk_charger.h"

#include "flashlight-core.h"
#include "flashlight-dt.h"

/* device tree should be defined in flashlight-dt.h */
#ifndef MT6360_DTNAME
#define MT6360_DTNAME "mediatek,flashlights_mt6360"
#endif

#define MT6360_NAME "flashlights-mt6360"

/* define channel, level */
#define MT6360_CHANNEL_NUM 2
#define MT6360_CHANNEL_CH1 0
#define MT6360_CHANNEL_CH2 1
#define MT6360_CHANNEL_ALL 2

#define MT6360_NONE (-1)
#define MT6360_DISABLE 0
#define MT6360_ENABLE 1
#define MT6360_ENABLE_TORCH 1
#define MT6360_ENABLE_FLASH 2

#define MT6360_LEVEL_NUM 32
#if defined(CONFIG_MTK_CAM_PD2279) || defined (CONFIG_MTK_CAM_PD2279F)
#define MT6360_LEVEL_TORCH 16
#else
#define MT6360_LEVEL_TORCH 18
#endif
#define MT6360_LEVEL_FLASH MT6360_LEVEL_NUM
#define MT6360_WDT_TIMEOUT 1248 /* ms */
#if (defined(MTK_CAM_PD2215F_EX)|| defined(MTK_CAM_PD2257F_EX))
#define MT6360_HW_TIMEOUT 310 /* ms */
#else
#define MT6360_HW_TIMEOUT 400 /* ms */
#endif

/* define mutex, work queue and timer */
static DEFINE_MUTEX(mt6360_mutex);
static struct work_struct mt6360_work_ch1;
static struct work_struct mt6360_work_ch2;
static struct hrtimer mt6360_timer_ch1;
static struct hrtimer mt6360_timer_ch2;
static unsigned int mt6360_timeout_ms[MT6360_CHANNEL_NUM];

/* define usage count */
static int use_count;
static int fd_use_count;

/* define RTK flashlight device */
static struct flashlight_device *flashlight_dev_ch1;
static struct flashlight_device *flashlight_dev_ch2;
#define RT_FLED_DEVICE_CH1  "mt-flash-led1"
#define RT_FLED_DEVICE_CH2  "mt-flash-led2"

/* is decrease voltage */
static int is_decrease_voltage;

static int lock_touch;  /*hope add*/
static int mt6360_set_driver(int set);
static int mt6360_operate(int channel, int enable);

/* platform data */
struct mt6360_platform_data {
	int channel_num;
	struct flashlight_device_id *dev_id;
};

#if defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6833) \
|| defined(CONFIG_MACH_MT6893)
/* define charger consumer */
static struct charger_consumer *flashlight_charger_consumer;
#define CHARGER_SUPPLY_NAME "charger_port1"
#else
/******************************************************************************
 * Charger power supply class
 *****************************************************************************/
static int mt6360_high_voltage_supply(int enable)
{
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	int ret;

	if (chg_psy == NULL)
		chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		prop.intval = enable;
		ret = power_supply_set_property(chg_psy,
			 POWER_SUPPLY_PROP_VOLTAGE_MAX, &prop);
		pr_notice("%s enable_hv:%d\n", __func__, prop.intval);
		power_supply_changed(chg_psy);
	}

	return ret;
}
#endif

/******************************************************************************
 * mt6360 operations
 *****************************************************************************/
#ifdef CONFIG_MTK_CAM_PD2230F
static const int mt6360_current[MT6360_LEVEL_NUM] = {
	  25,   50,  75, 100, 112, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

static const unsigned char mt6360_torch_level[MT6360_LEVEL_TORCH] = {
	0x00, 0x02, 0x04, 0x06, 0x07, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};
#elif defined(CONFIG_MTK_CAM_PD2166G)
static const int mt6360_current[MT6360_LEVEL_NUM] = {
	  25,   50,  75, 100, 125, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

static const unsigned char mt6360_torch_level[MT6360_LEVEL_TORCH] = {
	0x00, 0x02, 0x05, 0x0A, 0x0E, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};

#elif defined(CONFIG_MTK_CAM_PD2279)
static const int mt6360_current[MT6360_LEVEL_NUM] = {
	 25,   50,  75, 100, 125, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

static const unsigned char mt6360_torch_level[MT6360_LEVEL_TORCH] = {
	0x00, 0x02, 0x04, 0x08, 0x0A, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};

#elif defined(CONFIG_MTK_CAM_PD2279F)
static const int mt6360_current[MT6360_LEVEL_NUM] = {
	 25,   50,  75, 100, 125, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

static const unsigned char mt6360_torch_level[MT6360_LEVEL_TORCH] = {
	0x00, 0x02, 0x04, 0x07, 0x0A, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};
#else
static const int mt6360_current[MT6360_LEVEL_NUM] = {
	  25,   50,  75, 100, 125, 150, 175,  200,  225,  250,
	 275,  300, 325, 350, 375, 400, 450,  500,  550,  600,
	 650,  700, 750, 800, 850, 900, 950, 1000, 1050, 1100,
	1150, 1200
};

static const unsigned char mt6360_torch_level[MT6360_LEVEL_TORCH] = {
	0x00, 0x02, 0x04, 0x08, 0x0A, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
	0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E
};
#endif

/* 0x00~0x74 6.25mA/step 0x75~0xB1 12.5mA/step */
static const unsigned char mt6360_strobe_level[MT6360_LEVEL_FLASH] = {
	0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24,
	0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C, 0x44, 0x4C, 0x54, 0x5C,
	0x64, 0x6C, 0x74, 0x78, 0x7C, 0x80, 0x84, 0x88, 0x8C, 0x90,
	0x94, 0x98
};

static int mt6360_decouple_mode;
static int mt6360_en_ch1;
static int mt6360_en_ch2;
static int mt6360_level_ch1;
static int mt6360_level_ch2;

static int mt6360_is_charger_ready(void)
{
	if (flashlight_is_ready(flashlight_dev_ch1) &&
			flashlight_is_ready(flashlight_dev_ch2))
		return FLASHLIGHT_CHARGER_READY;
	else
		return FLASHLIGHT_CHARGER_NOT_READY;
}

static int mt6360_is_torch(int level)
{
	if (level >= MT6360_LEVEL_TORCH)
		return -1;

	return 0;
}

static int mt6360_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= MT6360_LEVEL_NUM)
		level = MT6360_LEVEL_NUM - 1;

	return level;
}

/* flashlight enable function */
static int mt6360_enable(void)
{
	int ret = 0;
	enum flashlight_mode mode = FLASHLIGHT_MODE_TORCH;

	if (!flashlight_dev_ch1 || !flashlight_dev_ch2) {
		pr_info("Failed to enable since no flashlight device.\n");
		return -1;
	}

	/* set flash mode if any channel is flash mode */
	if ((mt6360_en_ch1 == MT6360_ENABLE_FLASH)
			|| (mt6360_en_ch2 == MT6360_ENABLE_FLASH))
		mode = FLASHLIGHT_MODE_FLASH;

	pr_debug("enable(%d,%d), mode:%d.\n",
		mt6360_en_ch1, mt6360_en_ch2, mode);

	/* enable channel 1 and channel 2 */
	if (mt6360_decouple_mode == FLASHLIGHT_SCENARIO_COUPLE &&
			mt6360_en_ch1 != MT6360_DISABLE &&
			mt6360_en_ch2 != MT6360_DISABLE) {
		pr_info("dual flash mode\n");
		if (mode == FLASHLIGHT_MODE_TORCH)
			ret |= flashlight_set_mode(
				flashlight_dev_ch1, FLASHLIGHT_MODE_DUAL_TORCH);
		else
			ret |= flashlight_set_mode(
				flashlight_dev_ch1, FLASHLIGHT_MODE_DUAL_FLASH);
	} else {
		if (mt6360_en_ch1)
			ret |= flashlight_set_mode(
				flashlight_dev_ch1, mode);
		else if (mt6360_decouple_mode == FLASHLIGHT_SCENARIO_COUPLE)
			ret |= flashlight_set_mode(
				flashlight_dev_ch1, FLASHLIGHT_MODE_OFF);
		if (mt6360_en_ch2)
			ret |= flashlight_set_mode(
				flashlight_dev_ch2, mode);
		else if (mt6360_decouple_mode == FLASHLIGHT_SCENARIO_COUPLE)
			ret |= flashlight_set_mode(
				flashlight_dev_ch2, FLASHLIGHT_MODE_OFF);
	}
	if (ret < 0)
		pr_info("Failed to enable.\n");

	return ret;
}

/* flashlight disable function */
static int mt6360_disable_ch1(void)
{
	int ret = 0;

	pr_debug("disable_ch1.\n");

	if (!flashlight_dev_ch1) {
		pr_info("Failed to disable since no flashlight device.\n");
		return -1;
	}

	ret |= flashlight_set_mode(flashlight_dev_ch1, FLASHLIGHT_MODE_OFF);

	if (ret < 0)
		pr_info("Failed to disable.\n");

	return ret;
}

static int mt6360_disable_ch2(void)
{
	int ret = 0;

	pr_debug("disable_ch2.\n");

	if (!flashlight_dev_ch2) {
		pr_info("Failed to disable since no flashlight device.\n");
		return -1;
	}

	ret |= flashlight_set_mode(flashlight_dev_ch2, FLASHLIGHT_MODE_OFF);

	if (ret < 0)
		pr_info("Failed to disable.\n");

	return ret;
}

static int mt6360_disable_all(void)
{
	int ret = 0;

	pr_debug("disable_ch1.\n");

	if (!flashlight_dev_ch1) {
		pr_info("Failed to disable since no flashlight device.\n");
		return -1;
	}

	ret |= flashlight_set_mode(flashlight_dev_ch1,
		FLASHLIGHT_MODE_DUAL_OFF);

	if (ret < 0)
		pr_info("Failed to disable.\n");

	return ret;
}

static int mt6360_disable(int channel)
{
	int ret = 0;

	if (channel == MT6360_CHANNEL_CH1)
		ret = mt6360_disable_ch1();
	else if (channel == MT6360_CHANNEL_CH2)
		ret = mt6360_disable_ch2();
	else if (channel == MT6360_CHANNEL_ALL)
		ret = mt6360_disable_all();
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return ret;
}

/* set flashlight level */
static int mt6360_set_level_ch1(int level)
{
	level = mt6360_verify_level(level);
	mt6360_level_ch1 = level;

	if (!flashlight_dev_ch1) {
		pr_info("Failed to set ht level since no flashlight device.\n");
		return -1;
	}

	/* set brightness level */
	if (!mt6360_is_torch(level))
		flashlight_set_torch_brightness(
				flashlight_dev_ch1, mt6360_torch_level[level]);
	flashlight_set_strobe_brightness(
			flashlight_dev_ch1, mt6360_strobe_level[level]);

	return 0;
}

static int mt6360_set_level_ch2(int level)
{
	level = mt6360_verify_level(level);
	mt6360_level_ch2 = level;

	if (!flashlight_dev_ch2) {
		pr_info("Failed to set lt level since no flashlight device.\n");
		return -1;
	}

	/* set brightness level */
	if (!mt6360_is_torch(level))
		flashlight_set_torch_brightness(
				flashlight_dev_ch2, mt6360_torch_level[level]);
	flashlight_set_strobe_brightness(
			flashlight_dev_ch2, mt6360_strobe_level[level]);

	return 0;
}

static int mt6360_set_level(int channel, int level)
{
	if (channel == MT6360_CHANNEL_CH1)
		mt6360_set_level_ch1(level);
	else if (channel == MT6360_CHANNEL_CH2)
		mt6360_set_level_ch2(level);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

static int mt6360_set_scenario(int scenario)
{
	/* set decouple mode */
	mt6360_decouple_mode = scenario & FLASHLIGHT_SCENARIO_DECOUPLE_MASK;

	/* notify charger to increase or decrease voltage */
	mutex_lock(&mt6360_mutex);
	if (scenario & FLASHLIGHT_SCENARIO_CAMERA_MASK) {
		if (!is_decrease_voltage) {
			pr_info("Decrease voltage level.\n");
#if defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6833) \
|| defined(CONFIG_MACH_MT6893)
			charger_manager_enable_high_voltage_charging(
				flashlight_charger_consumer, false);
#else
			mt6360_high_voltage_supply(0);
#endif
			is_decrease_voltage = 1;
		}
	} else {
		if (is_decrease_voltage) {
			pr_info("Increase voltage level.\n");
#if defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6833) \
|| defined(CONFIG_MACH_MT6893)
			charger_manager_enable_high_voltage_charging(
				flashlight_charger_consumer, true);
#else
			mt6360_high_voltage_supply(1);
#endif
			is_decrease_voltage = 0;
		}
	}
	mutex_unlock(&mt6360_mutex);

	return 0;
}

/* flashlight init */
static int mt6360_init(void)
{
	if (lock_touch == 0) {
	/* clear flashlight state */
	mt6360_en_ch1 = MT6360_NONE;
	mt6360_en_ch2 = MT6360_NONE;

	/* clear decouple mode */
	mt6360_decouple_mode = FLASHLIGHT_SCENARIO_COUPLE;

	/* clear charger status */
	is_decrease_voltage = 0;
	}
	return 0;
}

/* flashlight uninit */
static int mt6360_uninit(void)
{
	int ret;
	if (lock_touch == 0) {
	/* clear flashlight state */
	mt6360_en_ch1 = MT6360_NONE;
	mt6360_en_ch2 = MT6360_NONE;

	/* clear decouple mode */
	mt6360_decouple_mode = FLASHLIGHT_SCENARIO_COUPLE;

	/* clear charger status */
	is_decrease_voltage = 0;

	ret = mt6360_disable(MT6360_CHANNEL_ALL);
	}
	return ret;
}

static int set_flashlight_state(int channel, int state)
{

	pr_err("set_flashlight_state check state:%d, channel:%d\n", state, channel);

	mt6360_set_driver(1);
	switch (state) {
	case BBK_TORCH_LOW:
		mt6360_set_scenario(FLASHLIGHT_SCENARIO_CAMERA |FLASHLIGHT_SCENARIO_DECOUPLE);
#if defined(CONFIG_MTK_CAM_PD2230)|| defined(CONFIG_MTK_CAM_PD2230F)
		mt6360_set_level(channel, 2);
#else
		mt6360_set_level(channel, 3);
#endif
		mt6360_timeout_ms[channel] = 0;
		mt6360_operate(channel, MT6360_ENABLE);
		lock_touch = 1;
		break;
	case BBK_TORCH_OFF:
		mt6360_set_scenario(FLASHLIGHT_SCENARIO_FLASHLIGHT |FLASHLIGHT_SCENARIO_DECOUPLE);
		mt6360_operate(channel, MT6360_DISABLE);
		lock_touch = 0;
		break;
	case BBK_FLASH_AT_TEST:
		mt6360_set_scenario(FLASHLIGHT_SCENARIO_CAMERA |FLASHLIGHT_SCENARIO_DECOUPLE);
		mt6360_set_level(channel, 15);
		mt6360_timeout_ms[channel] = 0;
		mt6360_operate(channel, MT6360_ENABLE);
		mdelay(3000);
		break;
	case BBK_FLASH_AT_OFF:
		mt6360_set_scenario(FLASHLIGHT_SCENARIO_FLASHLIGHT |FLASHLIGHT_SCENARIO_DECOUPLE);
		mt6360_operate(channel, MT6360_DISABLE);
		break;
	default:
		pr_err("set_flashlight_state No such command and arg\n");
		return -ENOTTY;
	}
	mt6360_set_driver(0);
	return 0;
}

/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static void mt6360_work_disable_ch1(struct work_struct *data)
{
	pr_debug("ht work queue callback\n");
	mt6360_disable(MT6360_CHANNEL_CH1);
}

static void mt6360_work_disable_ch2(struct work_struct *data)
{
	pr_debug("lt work queue callback\n");
	mt6360_disable(MT6360_CHANNEL_CH2);
}

static enum hrtimer_restart mt6360_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&mt6360_work_ch1);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart mt6360_timer_func_ch2(struct hrtimer *timer)
{
	schedule_work(&mt6360_work_ch2);
	return HRTIMER_NORESTART;
}

static int mt6360_timer_start(int channel, ktime_t ktime)
{
	if (channel == MT6360_CHANNEL_CH1)
		hrtimer_start(&mt6360_timer_ch1, ktime, HRTIMER_MODE_REL);
	else if (channel == MT6360_CHANNEL_CH2)
		hrtimer_start(&mt6360_timer_ch2, ktime, HRTIMER_MODE_REL);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

static int mt6360_timer_cancel(int channel)
{
	if (channel == MT6360_CHANNEL_CH1)
		hrtimer_cancel(&mt6360_timer_ch1);
	else if (channel == MT6360_CHANNEL_CH2)
		hrtimer_cancel(&mt6360_timer_ch2);
	else {
		pr_info("Error channel\n");
		return -1;
	}

	return 0;
}

/******************************************************************************
 * Flashlight operation wrapper function
 *****************************************************************************/
static int mt6360_operate(int channel, int enable)
{
	ktime_t ktime;
	unsigned int s;
	unsigned int ns;

	/* setup enable/disable */
	if (channel == MT6360_CHANNEL_CH1) {
		mt6360_en_ch1 = enable;
		if (mt6360_en_ch1)
			if (mt6360_is_torch(mt6360_level_ch1))
				mt6360_en_ch1 = MT6360_ENABLE_FLASH;
	} else if (channel == MT6360_CHANNEL_CH2) {
		mt6360_en_ch2 = enable;
		if (mt6360_en_ch2)
			if (mt6360_is_torch(mt6360_level_ch2))
				mt6360_en_ch2 = MT6360_ENABLE_FLASH;
	} else {
		pr_info("Error channel\n");
		return -1;
	}

	/* decouple mode */
	if (mt6360_decouple_mode) {
		if (channel == MT6360_CHANNEL_CH1) {
			mt6360_en_ch2 = MT6360_DISABLE;
			mt6360_timeout_ms[MT6360_CHANNEL_CH2] = 0;
		} else if (channel == MT6360_CHANNEL_CH2) {
			mt6360_en_ch1 = MT6360_DISABLE;
			mt6360_timeout_ms[MT6360_CHANNEL_CH1] = 0;
		}
	}

	pr_debug("en_ch(%d,%d), decouple:%d\n",
		mt6360_en_ch1, mt6360_en_ch2, mt6360_decouple_mode);

	/* operate flashlight and setup timer */
#if defined(CONFIG_MTK_CAM_PD2215F_EX) || defined(CONFIG_MTK_CAM_PD2257F_EX)
	if ((mt6360_en_ch1 != MT6360_NONE)) {
#else
	if ((mt6360_en_ch1 != MT6360_NONE) && (mt6360_en_ch2 != MT6360_NONE)) {
#endif
		if ((mt6360_en_ch1 == MT6360_DISABLE) &&
				(mt6360_en_ch2 == MT6360_DISABLE)) {
			if (mt6360_decouple_mode) {
				if (channel == MT6360_CHANNEL_CH1) {
					mt6360_disable(MT6360_CHANNEL_CH1);
					mt6360_timer_cancel(MT6360_CHANNEL_CH1);
				} else if (channel == MT6360_CHANNEL_CH2) {
					mt6360_disable(MT6360_CHANNEL_CH2);
					mt6360_timer_cancel(MT6360_CHANNEL_CH2);
				}
			} else {
				mt6360_disable(MT6360_CHANNEL_ALL);
				mt6360_timer_cancel(MT6360_CHANNEL_CH1);
				mt6360_timer_cancel(MT6360_CHANNEL_CH2);
			}
		} else {
			if (mt6360_timeout_ms[MT6360_CHANNEL_CH1] &&
				mt6360_en_ch1 != MT6360_DISABLE) {
				s = mt6360_timeout_ms[MT6360_CHANNEL_CH1] /
					1000;
				ns = mt6360_timeout_ms[MT6360_CHANNEL_CH1] %
					1000 * 1000000;
				ktime = ktime_set(s, ns);
				mt6360_timer_start(MT6360_CHANNEL_CH1, ktime);
			}
			if (mt6360_timeout_ms[MT6360_CHANNEL_CH2] &&
				mt6360_en_ch2 != MT6360_DISABLE) {
				s = mt6360_timeout_ms[MT6360_CHANNEL_CH2] /
					1000;
				ns = mt6360_timeout_ms[MT6360_CHANNEL_CH2] %
					1000 * 1000000;
				ktime = ktime_set(s, ns);
				mt6360_timer_start(MT6360_CHANNEL_CH2, ktime);
			}
			mt6360_enable();
		}

		/* clear flashlight state */
		mt6360_en_ch1 = MT6360_NONE;
		mt6360_en_ch2 = MT6360_NONE;
	}

	return 0;
}

/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int mt6360_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int led_state;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= MT6360_CHANNEL_NUM) {
		pr_info("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_debug("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		mt6360_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_debug("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		mt6360_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_SCENARIO:
		pr_debug("FLASH_IOC_SET_SCENARIO(%d): %d\n",
				channel, (int)fl_arg->arg);
		mt6360_set_scenario(fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_debug("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (lock_touch == 0) {
		mt6360_operate(channel, fl_arg->arg);
		}
		break;

	case FLASH_IOC_IS_CHARGER_READY:
		pr_debug("FLASH_IOC_IS_CHARGER_READY(%d)\n", channel);
		fl_arg->arg = mt6360_is_charger_ready();
		pr_debug("FLASH_IOC_IS_CHARGER_READY(%d)\n", fl_arg->arg);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = MT6360_LEVEL_NUM;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = MT6360_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = mt6360_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		fl_arg->arg = mt6360_current[fl_arg->arg];
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = MT6360_HW_TIMEOUT;
		break;

	case FLASH_IOCTL_SET_LED_STATE:
		pr_err("FLASH_IOCTL_SET_LED_STATE(channel %d): arg: %d\n",
				channel, (int)fl_arg->arg);
		led_state = (int)fl_arg->arg;
		set_flashlight_state(channel, led_state);
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int mt6360_open(void)
{
	/* Move to set driver for saving power */
	mutex_lock(&mt6360_mutex);
	fd_use_count++;
	pr_debug("open driver: %d\n", fd_use_count);
	mutex_unlock(&mt6360_mutex);
	return 0;
}

static int mt6360_release(void)
{
	/* Move to set driver for saving power */
	mutex_lock(&mt6360_mutex);
	fd_use_count--;
	pr_debug("close driver: %d\n", fd_use_count);
	/* If camera NE, we need to enable pe by ourselves*/
	if (fd_use_count == 0 && is_decrease_voltage) {
		pr_info("Increase voltage level.\n");
#if defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6833) \
|| defined(CONFIG_MACH_MT6893)
			charger_manager_enable_high_voltage_charging(
				flashlight_charger_consumer, true);
#else
			mt6360_high_voltage_supply(1);
#endif
		is_decrease_voltage = 0;
	}
	mutex_unlock(&mt6360_mutex);
	return 0;
}

static int mt6360_set_driver(int set)
{
	int ret = 0;

	/* set chip and usage count */
	mutex_lock(&mt6360_mutex);
	if (set) {
		if (!use_count)
			ret = mt6360_init();
		use_count++;
		pr_debug("Set driver: %d\n", use_count);
	} else {
		use_count--;
		if (!use_count)
			ret = mt6360_uninit();
		if (use_count < 0)
			use_count = 0;
		pr_debug("Unset driver: %d\n", use_count);
	}
	mutex_unlock(&mt6360_mutex);

	return ret;
}

static ssize_t mt6360_strobe_store(struct flashlight_arg arg)
{
	mt6360_set_driver(1);
	if (arg.decouple)
		mt6360_set_scenario(
			FLASHLIGHT_SCENARIO_CAMERA |
			FLASHLIGHT_SCENARIO_DECOUPLE);
	else
		mt6360_set_scenario(
			FLASHLIGHT_SCENARIO_CAMERA |
			FLASHLIGHT_SCENARIO_COUPLE);
	mt6360_set_level(arg.channel, arg.level);
	mt6360_timeout_ms[arg.channel] = 0;

	if (arg.level < 0)
		mt6360_operate(arg.channel, MT6360_DISABLE);
	else
		mt6360_operate(arg.channel, MT6360_ENABLE);

	msleep(arg.dur);
	if (arg.decouple)
		mt6360_set_scenario(
			FLASHLIGHT_SCENARIO_FLASHLIGHT |
			FLASHLIGHT_SCENARIO_DECOUPLE);
	else
		mt6360_set_scenario(
			FLASHLIGHT_SCENARIO_FLASHLIGHT |
			FLASHLIGHT_SCENARIO_COUPLE);
	mt6360_operate(arg.channel, MT6360_DISABLE);
	mt6360_set_driver(0);

	return 0;
}

static struct flashlight_operations mt6360_ops = {
	mt6360_open,
	mt6360_release,
	mt6360_ioctl,
	mt6360_strobe_store,
	mt6360_set_driver
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int mt6360_parse_dt(struct device *dev,
		struct mt6360_platform_data *pdata)
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

	for_each_child_of_node(np, cnp) {
		if (of_property_read_u32(cnp, "type", &pdata->dev_id[i].type))
			goto err_node_put;
		if (of_property_read_u32(cnp, "ct", &pdata->dev_id[i].ct))
			goto err_node_put;
		if (of_property_read_u32(cnp, "part", &pdata->dev_id[i].part))
			goto err_node_put;
		snprintf(pdata->dev_id[i].name, FLASHLIGHT_NAME_SIZE,
				MT6360_NAME);
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

static int mt6360_probe(struct platform_device *pdev)
{
	struct mt6360_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int ret;
	int i;

	pr_debug("Probe start.\n");

	/* parse dt */
	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		pdev->dev.platform_data = pdata;
		ret = mt6360_parse_dt(&pdev->dev, pdata);
		if (ret)
			return ret;
	}

	/* init work queue */
	INIT_WORK(&mt6360_work_ch1, mt6360_work_disable_ch1);
	INIT_WORK(&mt6360_work_ch2, mt6360_work_disable_ch2);

	/* init timer */
	hrtimer_init(&mt6360_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mt6360_timer_ch1.function = mt6360_timer_func_ch1;
	hrtimer_init(&mt6360_timer_ch2, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mt6360_timer_ch2.function = mt6360_timer_func_ch2;
	mt6360_timeout_ms[MT6360_CHANNEL_CH1] = 600;
	mt6360_timeout_ms[MT6360_CHANNEL_CH2] = 600;

	/* clear attributes */
	use_count = 0;
	fd_use_count = 0;
	is_decrease_voltage = 0;

	/* get RTK flashlight handler */
	flashlight_dev_ch1 = find_flashlight_by_name(RT_FLED_DEVICE_CH1);
	if (!flashlight_dev_ch1) {
		pr_info("Failed to get ht flashlight device.\n");
		return -EFAULT;
	}
	flashlight_dev_ch2 = find_flashlight_by_name(RT_FLED_DEVICE_CH2);
	if (!flashlight_dev_ch2) {
		pr_info("Failed to get lt flashlight device.\n");
		return -EFAULT;
	}

	/* setup strobe mode timeout */
	if (flashlight_set_strobe_timeout(flashlight_dev_ch1,
				MT6360_HW_TIMEOUT, MT6360_HW_TIMEOUT + 200) < 0)
		pr_info("Failed to set strobe timeout.\n");

#if defined(CONFIG_MTK_CAM_PD2230) || defined(CONFIG_MTK_CAM_PD2215F_EX) \
|| defined(CONFIG_MTK_CAM_PD2204F_EX) || defined(CONFIG_MTK_CAM_PD2250) \
||  defined(CONFIG_MTK_CAM_PD2250F_EX) || defined(CONFIG_MTK_CAM_PD2250F)\
|| defined(CONFIG_MTK_CAM_PD2230F) || defined(CONFIG_MTK_CAM_PD2257F_EX) \
|| defined(CONFIG_MTK_CAM_PD2166G) || defined(CONFIG_MTK_CAM_PD2279) \
|| defined(CONFIG_MTK_CAM_PD2279F)
	/* do nothing */
#else
#if defined(CONFIG_MACH_MT6877) || defined(CONFIG_MACH_MT6833) \
|| defined(CONFIG_MACH_MT6893)
	/* get charger consumer manager */

#if	!defined(CONFIG_VIVO_CHARGING_FOR_PD2204) && !defined(CONFIG_VIVO_CHARGING_FOR_PD2282)
	flashlight_charger_consumer = charger_manager_get_by_name(
			&flashlight_dev_ch1->dev, CHARGER_SUPPLY_NAME);
#endif
	if (!flashlight_charger_consumer) {
		pr_info("Failed to get charger manager.\n");
		return -EFAULT;
	}
#endif
#endif

	/* register flashlight device */
	if (pdata->channel_num) {
		for (i = 0; i < pdata->channel_num; i++)
			if (flashlight_dev_register_by_device_id(
						&pdata->dev_id[i],
						&mt6360_ops))
				return -EFAULT;
	} else {
		if (flashlight_dev_register(MT6360_NAME, &mt6360_ops))
			return -EFAULT;
	}

	pr_debug("Probe done.\n");

	return 0;
}

static int mt6360_remove(struct platform_device *pdev)
{
	struct mt6360_platform_data *pdata = dev_get_platdata(&pdev->dev);
	int i;

	pr_debug("Remove start.\n");

	pdev->dev.platform_data = NULL;

	/* unregister flashlight device */
	if (pdata && pdata->channel_num)
		for (i = 0; i < pdata->channel_num; i++)
			flashlight_dev_unregister_by_device_id(
					&pdata->dev_id[i]);
	else
		flashlight_dev_unregister(MT6360_NAME);

	/* flush work queue */
	flush_work(&mt6360_work_ch1);
	flush_work(&mt6360_work_ch2);

	/* clear RTK flashlight device */
	flashlight_dev_ch1 = NULL;
	flashlight_dev_ch2 = NULL;

	pr_debug("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt6360_of_match[] = {
	{.compatible = MT6360_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, mt6360_of_match);
#else
static struct platform_device mt6360_platform_device[] = {
	{
		.name = MT6360_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, mt6360_platform_device);
#endif

static struct platform_driver mt6360_platform_driver = {
	.probe = mt6360_probe,
	.remove = mt6360_remove,
	.driver = {
		.name = MT6360_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt6360_of_match,
#endif
	},
};

static int __init flashlight_mt6360_init(void)
{
	int ret;

	pr_debug("Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&mt6360_platform_device);
	if (ret) {
		pr_info("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&mt6360_platform_driver);
	if (ret) {
		pr_info("Failed to register platform driver\n");
		return ret;
	}

	pr_debug("Init done.\n");

	return 0;
}

static void __exit flashlight_mt6360_exit(void)
{
	pr_debug("Exit start.\n");

	platform_driver_unregister(&mt6360_platform_driver);

	pr_debug("Exit done.\n");
}

/* replace module_init() since conflict in kernel init process */
late_initcall(flashlight_mt6360_init);
module_exit(flashlight_mt6360_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Roger Wang <Roger-HY.Wang@mediatek.com>");
MODULE_DESCRIPTION("MTK Flashlight MT6360 Driver");

