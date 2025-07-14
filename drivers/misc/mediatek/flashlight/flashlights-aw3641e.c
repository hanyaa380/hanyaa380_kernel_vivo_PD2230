/*
* Copyright (C) 2021 Awinic Inc.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <media/soc_camera.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"

#define AW3641E_DRIVER_VERSION "V1.1.0"

#define CONFIG_OF				1
#define AW3641E_NSEC_TIME		1000
#define AW3641E_NAME			"flashlights_aw3641e"

#define AW3641E_CHANNEL_NUM		1
#define AW3641E_CHANNEL_CH1		0
#define AW3641E_LEVEL_FLASH		16
#define AW3641E_LEVEL_TORCH		0
#define AW3641E_HW_TIMEOUT		400 /* ms */

/* add for PD2236 factory flash */
static char node_one_buf[20] = {"0"};

static struct hrtimer timer_close;
static struct work_struct aw3641e_work;
static unsigned int flash_en_gpio;
static unsigned int flash_sel_gpio;
static int flash_level = -1;
static spinlock_t aw3641e_lock;

static int lock_touch;

static unsigned int aw3641e_timeout_ms[AW3641E_CHANNEL_NUM];
//static const unsigned char aw3641e_torch_level[AW3641E_LEVEL_TORCH] = {1};
static const unsigned char aw3641e_flash_level[AW3641E_LEVEL_FLASH + 1] = {
					0, 1, 2, 3, 4, 5, 6, 7, 8,
					9, 10, 11, 12, 13, 14, 15, 16};
static void aw3641e_torch_on(void);
static void aw3641e_flash_on(void);
static void aw3641e_flash_off(void);

static int aw3641e_is_torch(int level)
{
	pr_info("%s.\n", __func__);

	if (level > AW3641E_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw3641e_verify_level(int level)
{
	pr_info("%s.\n", __func__);

	if (level <= 0)
		level = 0;
	else if (level >= AW3641E_LEVEL_FLASH)
		level = AW3641E_LEVEL_FLASH;

	return level;
}

void aw3641e_torch_on(void)
{
	pr_info("%s.\n", __func__);

	gpio_set_value(flash_sel_gpio, 0);
	gpio_set_value(flash_en_gpio, 1);
}

void aw3641e_flash_off(void)
{
	pr_info("%s.\n", __func__);

	gpio_set_value(flash_sel_gpio, 0);
	gpio_set_value(flash_en_gpio, 0);
	udelay(500);
}

void aw3641e_flash_on(void)
{
	int i = 0;

	pr_info("%s.flash_level = %d.\n", __func__, flash_level);

	gpio_set_value(flash_sel_gpio, 1);
	for (i = 0; i < aw3641e_flash_level[flash_level] - 1; i++) {
		gpio_set_value(flash_en_gpio, 1);
		udelay(2);
		gpio_set_value(flash_en_gpio, 0);
		udelay(2);
	}
	gpio_set_value(flash_en_gpio, 1);
}
static void aw3641e_work_disable(struct work_struct *data)
{
	pr_info("%s.\n", __func__);
	aw3641e_flash_off();
}

static enum hrtimer_restart aw3641e_timer_close(struct hrtimer *timer)
{
	schedule_work(&aw3641e_work);
	return HRTIMER_NORESTART;
}

int aw3641e_timer_start(int channel, ktime_t ktime)
{
	pr_info("%s.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1)
		hrtimer_start(&timer_close, ktime, HRTIMER_MODE_REL);
	else {
		pr_err("%s, Error channel\n", __func__);
		return -1;
	}

	return 0;
}

int aw3641e_timer_cancel(int channel)
{
	pr_info("%s.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1)
		hrtimer_cancel(&timer_close);
	else {
		pr_err("Error channel\n");
		return -EINVAL;
	}

	return 0;
}

void aw3641e_enable(int channel)
{
	pr_info("%s,flash_level = %d.\n", __func__, flash_level);

	if (!aw3641e_is_torch(flash_level)) {
		/* torch mode */
		aw3641e_torch_on();
	} else {
		/* flash mode */
		aw3641e_flash_on();
	}

}
void aw3641e_disable(int channel)
{
	pr_info("%s.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1) {
		aw3641e_flash_off();
	} else {
		pr_err("%s, Error channel\n", __func__);
		return;
	}
}
static int aw3641e_set_level(int channel, int level)
{
	pr_info("%s.\n", __func__);

	if (channel == AW3641E_CHANNEL_CH1) {
		flash_level =  aw3641e_flash_level[aw3641e_verify_level(level)];
	} else {
		pr_err("%s, Error channel\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int set_flashlight_state(int state)
{
	ktime_t ktime;
	pr_info("set_flashlight_state check state:%d \n", state);
	switch (state) {
	case BBK_TORCH_LOW:
		aw3641e_set_level(AW3641E_CHANNEL_CH1, 0);
		aw3641e_enable(AW3641E_CHANNEL_CH1);
		lock_touch = 1;
		break;
	case BBK_TORCH_OFF:
		aw3641e_disable(AW3641E_CHANNEL_CH1);
		lock_touch = 0;
		break;
	case BBK_FLASH_AT_TEST:
			aw3641e_set_level(AW3641E_CHANNEL_CH1, 1);
			ktime = ktime_set(300 / 1000, (300 % 1000) * 1000000);
			aw3641e_timer_start(AW3641E_CHANNEL_CH1, ktime);
			aw3641e_enable(AW3641E_CHANNEL_CH1);
		break;
	case BBK_FLASH_AT_OFF:
			aw3641e_disable(AW3641E_CHANNEL_CH1);
			aw3641e_timer_cancel(AW3641E_CHANNEL_CH1);
		break;
	default:
		pr_info("set_flashlight_state No such command and arg\n");
		return -ENOTTY;
	}
	return 0;
}

/****************************************************************
*		flashlights platform interface			*
*****************************************************************/
static int aw3641e_open(void)
{
	return 0;
}

static int aw3641e_release(void)
{
	hrtimer_cancel(&timer_close);
	return 0;
}

static int aw3641e_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	int led_state;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	pr_info("%s.\n", __func__);
	/* modify flash level of fl_arg->arg acrodding your level for test */
	/*
	if (fl_arg->arg == 1)
		fl_arg->arg = 10;
	*/
	/* verify channel */
	if (channel < 0 || channel >= AW3641E_CHANNEL_NUM) {
		pr_err("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		pr_info("%s, FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		aw3641e_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		pr_info("%s,FLASH_IOC_SET_DUTY(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		aw3641e_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		pr_info("%s,FLASH_IOC_SET_ONOFF(%d): %d\n",
				__func__, channel, (int)fl_arg->arg);
		if (fl_arg->arg >= 1) {
			if (aw3641e_timeout_ms[channel]) {
				ktime =
				ktime_set(aw3641e_timeout_ms[channel] / 1000,
				(aw3641e_timeout_ms[channel] % 1000) * 1000000);
				aw3641e_timer_start(channel, ktime);
			}
		spin_lock(&aw3641e_lock);
		aw3641e_enable(channel);
		spin_unlock(&aw3641e_lock);
		} else {
			if ( lock_touch == 0 ) {
				aw3641e_disable(channel);
				aw3641e_timer_cancel(channel);
			}
		}
		break;

	case FLASH_IOCTL_SET_LED_STATE:
		pr_debug("FLASH_IOCTL_SET_LED_STATE(channel %d): arg: %d\n",
				channel, (int)fl_arg->arg);
		led_state = (int)fl_arg->arg;
		set_flashlight_state(led_state);
		break;

	case FLASH_IOC_GET_DUTY_NUMBER:
		pr_debug("FLASH_IOC_GET_DUTY_NUMBER(%d)\n", channel);
		fl_arg->arg = AW3641E_LEVEL_FLASH;
		break;

	case FLASH_IOC_GET_MAX_TORCH_DUTY:
		pr_debug("FLASH_IOC_GET_MAX_TORCH_DUTY(%d)\n", channel);
		fl_arg->arg = AW3641E_LEVEL_TORCH;
		//fl_arg->arg = AW3641E_LEVEL_TORCH - 1;
		break;

	case FLASH_IOC_GET_DUTY_CURRENT:
		fl_arg->arg = aw3641e_verify_level(fl_arg->arg);
		pr_debug("FLASH_IOC_GET_DUTY_CURRENT(%d): %d\n",
				channel, (int)fl_arg->arg);
		break;

	case FLASH_IOC_GET_HW_TIMEOUT:
		pr_debug("FLASH_IOC_GET_HW_TIMEOUT(%d)\n", channel);
		fl_arg->arg = AW3641E_HW_TIMEOUT;
		break;

	default:
		pr_info("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

int aw3641e_set_driver(int set)
{
	/* init chip and set usage count */
	return 0;
}

static ssize_t aw3641e_strobe_store(struct flashlight_arg arg)
{
	aw3641e_set_level(arg.ct, arg.level);
	return 0;
}

/*add for PD2236 factory flash*/
static ssize_t att_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	pr_debug("echo AW3641E_FLASH debug buf, %s ", buf);
	sprintf(node_one_buf, "%s\n", buf);

	if ((strcmp("0", buf) == 0) || (strcmp("0\x0a", buf) == 0)) {
		pr_debug("AW3641E_FLASH  0");
		aw3641e_disable(AW3641E_CHANNEL_CH1);
	} else {
		pr_debug("AW3641E_FLASH  1");
		aw3641e_set_level(AW3641E_CHANNEL_CH1, 0);
		aw3641e_enable(AW3641E_CHANNEL_CH1);
	}

	return count;
}

static ssize_t att_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", node_one_buf);
}

static DEVICE_ATTR(AW3641E_FLASH, 0664, att_show, att_store);

static struct flashlight_operations aw3641e_ops = {
	aw3641e_open,
	aw3641e_release,
	aw3641e_ioctl,
	aw3641e_strobe_store,
	aw3641e_set_driver
};

/********************************************************************
*			driver probe
*********************************************************************/
static int aw3641e_probe(struct platform_device *dev)
{
	struct device_node *node = dev->dev.of_node;

	pr_info("%s Probe start.\n", __func__);

	flash_en_gpio = of_get_named_gpio(node, "flash-en-gpio", 0);
	if ((!gpio_is_valid(flash_en_gpio))) {
		pr_err("%s: dts don't provide flash_en_gpio\n", __func__);
		return -EINVAL;
	}
	pr_info("%s prase dts with flash_en_gpio success.\n", __func__);

	flash_sel_gpio = of_get_named_gpio(node, "flash-sel-gpio", 0);
	if ((!gpio_is_valid(flash_sel_gpio))) {
		pr_err("%s: dts don't provide flash_sel_gpio\n", __func__);
		return -EINVAL;
	}
	pr_info("%s prase dts with flash_sel_gpio success.\n", __func__);

	if (devm_gpio_request_one(&dev->dev, flash_en_gpio,
				  GPIOF_DIR_OUT | GPIOF_INIT_LOW,
		      "Flash-En")) {
		pr_err("%s, gpio Flash-En failed\n", __func__);
		return -1;
	}
	if (devm_gpio_request_one(&dev->dev, flash_sel_gpio,
				  GPIOF_DIR_OUT | GPIOF_INIT_LOW,
		      "Flash-SEL")) {
		pr_err("%s, gpio Flash-SEL failed\n", __func__);
		goto err_gpio;
	}
	/* register flashlight operations */
	if (flashlight_dev_register(AW3641E_NAME, &aw3641e_ops)) {
		pr_err("Failed to register flashlight device.\n");
		goto err_free;
	}

	/* init work queue */
	INIT_WORK(&aw3641e_work, aw3641e_work_disable);

	spin_lock_init(&aw3641e_lock);

	/* init timer close */
	hrtimer_init(&timer_close, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer_close.function = aw3641e_timer_close;
	aw3641e_timeout_ms[AW3641E_CHANNEL_CH1] = 100;

	//add for PD2236 factory flash
	sysfs_create_file(&dev->dev.kobj, &dev_attr_AW3641E_FLASH.attr);

	pr_info("%s Probe finish.\n", __func__);

	return 0;
err_gpio:
	devm_gpio_free(&dev->dev, flash_en_gpio);
err_free:
	devm_gpio_free(&dev->dev, flash_sel_gpio);

	return -EINVAL;
}
static int aw3641e_remove(struct platform_device *dev)
{
	flashlight_dev_unregister(AW3641E_NAME);
	devm_gpio_free(&dev->dev, flash_en_gpio);
	devm_gpio_free(&dev->dev, flash_sel_gpio);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw3641e_of_match[] = {
	{.compatible = "mediatek,flashlights_aw3641e"},
	{},
};
MODULE_DEVICE_TABLE(of, aw3641e_of_match);
#else
static struct platform_device aw3641e_platform_device[] = {
	{
		.name = AW3641E_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw3641e_platform_device);
#endif

static struct platform_driver aw3641e_platform_driver = {
	.probe = aw3641e_probe,
	.remove = aw3641e_remove,
	.driver = {
		.name = AW3641E_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw3641e_of_match,
#endif
	},
};

static int __init aw3641e_flash_init(void)
{
	int ret = 0;

	pr_info("%s driver version %s.\n", __func__, AW3641E_DRIVER_VERSION);

#ifndef CONFIG_OF
	ret = platform_device_register(&aw3641e_platform_device);
	if (ret) {
		pr_err("%s,Failed to register platform device\n", __func__);
		return ret;
	}
#endif
	ret = platform_driver_register(&aw3641e_platform_driver);
	if (ret) {
		pr_err("%s,Failed to register platform driver\n", __func__);
		return ret;
	}
	return 0;
}

static void __exit aw3641e_flash_exit(void)
{
	pr_info("%s.\n", __func__);

	platform_driver_unregister(&aw3641e_platform_driver);
}

module_init(aw3641e_flash_init);
module_exit(aw3641e_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("shiqiang@awinic.com");
MODULE_DESCRIPTION("GPIO Flash driver");

