/**
 * The device control driver for FocalTech's fingerprint sensor.
 *
 * Copyright (C) 2016-2017 FocalTech Systems Co., Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/signal.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/bug.h>
#include <linux/types.h>
#include <linux/param.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/pm_wakeup.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "ff_log.h"
#include "ff_err.h"
#include "ff_ctl.h"

#if USE_PLATFORM_BUS
#include <linux/platform_device.h>
#endif

#if defined(CONFIG_BBK_FP_ID) || defined(CONFIG_BBK_FP_MODULE)
#include "../fp_id.h"
#endif
/*
 * Define the driver version string.
 */
#define FF_DRV_VERSION "v2.1.2"

/*
 * Define the driver name.
 */
#define FF_DRV_NAME "focaltech_fp"

/*
 * Driver context definition and its singleton instance.
 */
typedef struct {
	struct miscdevice miscdev;
	int irq_num;
	struct work_struct work_queue;
	struct fasync_struct *async_queue;
	struct input_dev *input;
	struct notifier_block fb_notifier;
	struct wakeup_source wake_lock;
	bool b_driver_inited;
	bool b_config_dirtied;
} ff_ctl_context_t;
static ff_ctl_context_t *g_context;

/*
 * Driver configuration.
 */
static ff_driver_config_t driver_config;
ff_driver_config_t *g_config;

#if USE_PLATFORM_BUS
struct platform_device *g_dev;
#elif USE_SPI_BUS
struct spi_device *g_dev;
#endif

////////////////////////////////////////////////////////////////////////////////
/// Logging driver to logcat through uevent mechanism.

# undef LOG_TAG
#define LOG_TAG "ff_ctl"

/*
 * Log level can be runtime configurable by 'FF_IOC_SYNC_CONFIG'.
 */
ff_log_level_t g_log_level = __FF_EARLY_LOG_LEVEL;

int ff_log_printf(ff_log_level_t level, const char *tag, const char *fmt, ...)
{
	/* Prepare the storage. */
	va_list ap;
	static char uevent_env_buf[128];
	char *ptr = uevent_env_buf;
	int n, available = sizeof(uevent_env_buf);

	/* Fill logging level. */
	available -= sprintf(uevent_env_buf, "FF_LOG=%1d", level);
	ptr += strlen(uevent_env_buf);

	/* Fill logging message. */
	va_start(ap, fmt);
	vsnprintf(ptr, available, fmt, ap);
	va_end(ap);

	/* Send to ff_device. */
	if (likely(g_context) && likely(g_config) && unlikely(g_config->logcat_driver)) {
		char *uevent_env[2] = {uevent_env_buf, NULL};
		kobject_uevent_env(&g_context->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);
	}

	/* Native output. */
	switch (level) {
	case FF_LOG_LEVEL_ERR:
		n = printk(KERN_ERR FF_DRV_NAME": %s\n", ptr);
		break;
	case FF_LOG_LEVEL_WRN:
		n = printk(KERN_WARNING FF_DRV_NAME": %s\n", ptr);
		break;
	case FF_LOG_LEVEL_INF:
		n = printk(KERN_INFO FF_DRV_NAME": %s\n", ptr);
		break;
	case FF_LOG_LEVEL_DBG:
	case FF_LOG_LEVEL_VBS:
	default:
		n = printk(KERN_DEBUG FF_DRV_NAME": %s\n", ptr);
		break;
	}
	return n;
}

const char *ff_err_strerror(int err)
{
	static char errstr[32] = {'\0', };

	switch (err) {
	case FF_SUCCESS: return "Success";
	case FF_ERR_INTERNAL: return "Internal error";

	/* Base on unix errno. */
	case FF_ERR_NOENT: return "No such file or directory";
	case FF_ERR_INTR:  return "Interrupted";
	case FF_ERR_IO:    return "I/O error";
	case FF_ERR_AGAIN: return "Try again";
	case FF_ERR_NOMEM: return "Out of memory";
	case FF_ERR_BUSY:  return "Resource busy / Timeout";

	/* Common error. */
	case FF_ERR_BAD_PARAMS:   return "Bad parameter(s)";
	case FF_ERR_NULL_PTR:     return "Null pointer";
	case FF_ERR_BUF_OVERFLOW: return "Buffer overflow";
	case FF_ERR_BAD_PROTOCOL: return "Bad protocol";
	case FF_ERR_SENSOR_SIZE:  return "Wrong sensor dimension";
	case FF_ERR_NULL_DEVICE:  return "Device not found";
	case FF_ERR_DEAD_DEVICE:  return "Device is dead";
	case FF_ERR_REACH_LIMIT:  return "Up to the limit";
	case FF_ERR_REE_TEMPLATE: return "Template store in REE";
	case FF_ERR_NOT_TRUSTED: return "Untrusted enrollment";

	default:
		sprintf(errstr, "%d", err);
		break;
	}

	return (const char *)errstr;
}

////////////////////////////////////////////////////////////////////////////////

/* See plat-xxxx.c for platform dependent implementation. */
extern int ff_ctl_parse_dts(void);
extern int ff_ctl_init_pins(int *irq_num);
extern int ff_ctl_free_pins(void);
extern int ff_ctl_enable_spiclk(bool on);
extern int ff_ctl_enable_power(bool on);
extern int ff_ctl_reset_device(int level);
extern int ff_ctl_set_spi_status(bool on);
extern const char *ff_ctl_arch_str(void);
#if 0
static int ff_ctl_enable_irq(bool on)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	FF_LOGD("irq: '%s'.", on ? "enable" : "disabled");

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	if (on) {
		enable_irq(g_context->irq_num);
	} else {
		disable_irq(g_context->irq_num);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}
#endif
static void ff_ctl_device_event(struct work_struct *ws)
{
	ff_ctl_context_t *ctx = container_of(ws, ff_ctl_context_t, work_queue);
	char *uevent_env[2] = {"FF_INTERRUPT", NULL};
	FF_LOGV("'%s' enter.", __func__);
	
	FF_LOGD("%s(irq = %d, ..) toggled.", __func__, ctx->irq_num);
	__pm_wakeup_event(&ctx->wake_lock, 1500);
	kobject_uevent_env(&ctx->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);
	enable_irq(ctx->irq_num);  //add test
	FF_LOGV("'%s' leave.", __func__);
}

static irqreturn_t ff_ctl_device_irq(int irq, void *dev_id)
{
	ff_ctl_context_t *ctx = (ff_ctl_context_t *)dev_id;
	disable_irq_nosync(irq);

	if (likely(irq == ctx->irq_num)) {
		if (g_config && g_config->enable_fasync && ctx->async_queue) {
			kill_fasync(&ctx->async_queue, SIGIO, POLL_IN);
		} else {
			schedule_work(&ctx->work_queue);
		}
	}

	//enable_irq(irq);
	return IRQ_HANDLED;
}

static int ff_ctl_report_key_event(struct input_dev *input, ff_key_event_t *kevent)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	input_report_key(input, kevent->code, kevent->value);
	input_sync(input);

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static const char *ff_ctl_get_version(void)
{
	static char version[FF_DRV_VERSION_LEN] = {'\0', };
	FF_LOGV("'%s' enter.", __func__);

	/* Version info. */
	version[0] = '\0';
	strcat(version, FF_DRV_VERSION);
#ifdef __FF_SVN_REV
	if (strlen(__FF_SVN_REV) > 0) {
		sprintf(version, "%s-r%s", version, __FF_SVN_REV);
	}
#endif
#ifdef __FF_BUILD_DATE
	strcat(version, "-"__FF_BUILD_DATE);
#endif
	sprintf(version, "%s-%s", version, ff_ctl_arch_str());
	FF_LOGD("version: '%s'.", version);

	FF_LOGV("'%s' leave.", __func__);
	return (const char *)version;
}
#if 0
static int ff_ctl_fb_notifier_callback(struct notifier_block *nb, unsigned long action, void *data)
{
	struct fb_event *event;
	int blank;
	char *uevent_env[2];
	ff_ctl_context_t *ctx = container_of(nb, ff_ctl_context_t, fb_notifier);

	/* If we aren't interested in this event, skip it immediately ... */
	if (action != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
		return NOTIFY_DONE;
	}

	FF_LOGV("'%s' enter.", __func__);

	event = (struct fb_event *)data;
	blank = *(int *)event->data;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		uevent_env[0] = "FF_SCREEN_ON";
		break;
	case FB_BLANK_POWERDOWN:
		uevent_env[0] = "FF_SCREEN_OFF";
		break;
	default:
		uevent_env[0] = "FF_SCREEN_??";
		break;
	}
	uevent_env[1] = NULL;
	kobject_uevent_env(&ctx->miscdev.this_device->kobj, KOBJ_CHANGE, uevent_env);

	FF_LOGV("'%s' leave.", __func__);
	return NOTIFY_OK;
}
#endif
static int ff_ctl_register_input(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	/* Allocate the input device. */
	g_context->input = input_allocate_device();
	if (!g_context->input) {
		FF_LOGE("input_allocate_device() failed.");
		return (-ENOMEM);
	}

	/* Register the key event capabilities. */
	if (g_config) {
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_nav_left);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_nav_right);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_nav_up);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_nav_down);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_double_click);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_click);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_long_press);
		input_set_capability(g_context->input, EV_KEY, g_config->keycode_simulation);
	}

	/* Register the allocated input device. */
	g_context->input->name = "ff_key";
	err = input_register_device(g_context->input);
	if (err) {
		FF_LOGE("input_register_device(..) = %d.", err);
		input_free_device(g_context->input);
		g_context->input = NULL;
		return (-ENODEV);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static int ff_ctl_free_driver(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	/* Unregister framebuffer event notifier. */
	//err = fb_unregister_client(&g_context->fb_notifier);

	/* Disable SPI clock. */
	//err = ff_ctl_enable_spiclk(0);

	/* Disable the fingerprint module's power. */
	err = ff_ctl_enable_power(0);

	/* Unregister the spidev device. */
	//ff_spi_unregister_device();

	/* Set spi to GPIO mode. */
	// err = ff_ctl_set_spi_status(0);

	/* De-initialize the input subsystem. */
	if (g_context->input) {
		/*
		 * Once input device was registered use input_unregister_device() and
		 * memory will be freed once last reference to the device is dropped.
		 */
		input_unregister_device(g_context->input);
		g_context->input = NULL;
	}

	/* Release IRQ resource. */
	if (g_context->irq_num > 0) {
		err = disable_irq_wake(g_context->irq_num);
		if (err) {
			FF_LOGE("disable_irq_wake(%d) = %d.", g_context->irq_num, err);
		}
		free_irq(g_context->irq_num, (void *)g_context);
		g_context->irq_num = -1;
	}

	/* Release pins resource. */
	err = ff_ctl_free_pins();

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static int ff_ctl_init_driver(void)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	if (unlikely(!g_context)) {
		return (-ENOSYS);
	}

	do {
		/* Initialize the PWR/SPI/RST/INT pins resource. */
		err = ff_ctl_init_pins(&g_context->irq_num);
		if (err > 0) {
			g_context->b_config_dirtied = true;
		} else \
		if (err) {
			FF_LOGE("ff_ctl_init_pins(..) = %d.", err);
			break;
		}

		/* Register IRQ. */
		err = request_irq(g_context->irq_num, ff_ctl_device_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "ff_irq", (void *)g_context);
		if (err) {
			FF_LOGE("request_irq(..) = %d.", err);
			break;
		}

		/* Wake up the system while receiving the interrupt. */
		err = enable_irq_wake(g_context->irq_num);
		if (err) {
			FF_LOGE("enable_irq_wake(%d) = %d.", g_context->irq_num, err);
		}
	} while (0);

	if (err) {
		ff_ctl_free_driver();
		return err;
	}

	/* Initialize the input subsystem. */
	err = ff_ctl_register_input();
	if (err) {
		FF_LOGE("ff_ctl_init_input() = %d.", err);
		//return err;
	}

	/* Enable the fingerprint module's power at system startup. */
	// err = ff_ctl_enable_power(1);

	/* Set spi to SPI mode. */
	// err = ff_ctl_set_spi_status(1);

	/* Enable SPI clock. */
	//err = ff_ctl_enable_spiclk(1);

	/* Register screen on/off callback. */
	//g_context->fb_notifier.notifier_call = ff_ctl_fb_notifier_callback;
	//err = fb_register_client(&g_context->fb_notifier);

	g_context->b_driver_inited = true;
	FF_LOGV("'%s' leave.", __func__);
	return err;
}

////////////////////////////////////////////////////////////////////////////////
// struct file_operations fields.

static int ff_ctl_fasync(int fd, struct file *filp, int mode)
{
	int err = 0;
	struct miscdevice *dev = (struct miscdevice *)filp->private_data;
	ff_ctl_context_t *ctx = container_of(dev, ff_ctl_context_t, miscdev);
	FF_LOGV("'%s' enter.", __func__);
    
    // By lijing on 210929.
    FF_LOGD("g_context = %p.", g_context);
    FF_LOGD(" async_queue = %p.",  g_context->async_queue);
    FF_LOGD("&async_queue = %p.", &g_context->async_queue);
    
	FF_LOGD("%s: mode = 0x%08x.", __func__, mode);
	err = fasync_helper(fd, filp, mode, &ctx->async_queue);

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static long ff_ctl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct miscdevice *dev = (struct miscdevice *)filp->private_data;
	ff_ctl_context_t *ctx = container_of(dev, ff_ctl_context_t, miscdev);
	FF_LOGV("'%s' enter.", __func__);

#if 1
	if (g_log_level <= FF_LOG_LEVEL_DBG) {
		static const char *cmd_names[] = {
				"FF_IOC_INIT_DRIVER", "FF_IOC_FREE_DRIVER",
				"FF_IOC_RESET_DEVICE",
				"FF_IOC_ENABLE_IRQ", "FF_IOC_DISABLE_IRQ",
				"FF_IOC_ENABLE_SPI_CLK", "FF_IOC_DISABLE_SPI_CLK",
				"FF_IOC_ENABLE_POWER", "FF_IOC_DISABLE_POWER",
				"FF_IOC_REPORT_KEY_EVENT", "FF_IOC_SYNC_CONFIG",
				"FF_IOC_GET_VERSION", "unknown",
		};
		unsigned int _cmd = _IOC_NR(cmd);
		if (_cmd > _IOC_NR(FF_IOC_GET_VERSION)) {
			_cmd = _IOC_NR(FF_IOC_GET_VERSION) + 1;
		}
		FF_LOGD("%s(.., %s, ..) invoke.", __func__, cmd_names[_cmd]);
	}
#endif

	switch (cmd) {
	case FF_IOC_INIT_DRIVER: {
		if (g_context->b_driver_inited) {
			err = ff_ctl_free_driver();
		}
		if (!err) {
			err = ff_ctl_init_driver();
			// TODO: Sync the dirty configuration back to HAL.
		}
		break;
	}
	case FF_IOC_FREE_DRIVER:
		if (g_context->b_driver_inited) {
			err = ff_ctl_free_driver();
			g_context->b_driver_inited = false;
		}
		break;
	case FF_IOC_RESET_DEVICE: {
		err = ff_ctl_reset_device((int)arg);
		break;
	}
	case FF_IOC_ENABLE_IRQ:
		//err = ff_ctl_enable_irq(1);
		enable_irq(ctx->irq_num);
		break;
	case FF_IOC_DISABLE_IRQ:
		//err = ff_ctl_enable_irq(0);
		disable_irq(ctx->irq_num);
		break;
	case FF_IOC_ENABLE_SPI_CLK:
		err = ff_ctl_enable_spiclk(1);
		break;
	case FF_IOC_DISABLE_SPI_CLK:
		err = ff_ctl_enable_spiclk(0);
		break;
	case FF_IOC_ENABLE_POWER:
		err = ff_ctl_enable_power(1);
		break;
	case FF_IOC_DISABLE_POWER:
		err = ff_ctl_enable_power(0);
		break;
	case FF_IOC_REPORT_KEY_EVENT: {
		ff_key_event_t kevent;
		if (copy_from_user(&kevent, (ff_key_event_t *)arg, sizeof(ff_key_event_t))) {
			FF_LOGE("copy_from_user(..) failed.");
			err = (-EFAULT);
			break;
		}
		err = ff_ctl_report_key_event(ctx->input, &kevent);
		break;
	}
	case FF_IOC_SYNC_CONFIG: {
		size_t conf_obj_size = sizeof(ff_driver_config_t);
		FF_LOGD("conf_obj_size = %u.", conf_obj_size);
		if (copy_from_user(&driver_config, (ff_driver_config_t *)arg, conf_obj_size)) {
			FF_LOGE("copy_from_user(..) failed.");
			err = (-EFAULT);
			break;
		}
		g_config = &driver_config;

		/* Take logging level effect. */
		g_log_level = g_config->log_level;
		break;
	}
	case FF_IOC_GET_VERSION: {
		if (copy_to_user((void *)arg, ff_ctl_get_version(), FF_DRV_VERSION_LEN)) {
			FF_LOGE("copy_to_user(..) failed.");
			err = (-EFAULT);
			break;
		}
		break;
	}
	default:
		err = (-EINVAL);
		break;
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static int ff_ctl_open(struct inode *inode, struct file *filp)
{
	FF_LOGD("'%s' nothing to do.", __func__);
	return 0;
}

static int ff_ctl_release(struct inode *inode, struct file *filp)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	/* Remove this filp from the asynchronously notified filp's. */
	// By lijing on 210929.
	if (g_config && g_config->enable_fasync && g_context->async_queue) {
		err = ff_ctl_fasync(-1, filp, 0);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

#ifdef CONFIG_COMPAT
static long ff_ctl_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	err = ff_ctl_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));

	FF_LOGV("'%s' leave.", __func__);
	return err;
}
#endif
///////////////////////////////////////////////////////////////////////////////

static struct file_operations ff_ctl_fops = {
	.owner          = THIS_MODULE,
	.fasync         = ff_ctl_fasync,
	.unlocked_ioctl = ff_ctl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = ff_ctl_compat_ioctl,
#endif
	.open           = ff_ctl_open,
	.release        = ff_ctl_release,
};

static ff_ctl_context_t ff_ctl_context = {
	.miscdev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = FF_DRV_NAME,
		.fops  = &ff_ctl_fops,
	}, 0,
};

#if USE_PLATFORM_BUS
static int ff_probe(struct platform_device *dev)
#elif USE_SPI_BUS
static int ff_probe(struct spi_device *dev)
#endif
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	g_dev = dev;
	FF_CHECK_PTR(g_dev);

	/* Register as a miscellaneous device. */
	err = ff_ctl_parse_dts();
	if (err > 0) {
		FF_LOGI("ff_ctl_parse_dts(..) succeed.");
	} else {
		FF_LOGE("ff_ctl_parse_dts(..) = %d.", err);
		return err;
	}
	/* Register as a miscellaneous device. */
	err = misc_register(&ff_ctl_context.miscdev);
	if (err) {
		FF_LOGE("misc_register(..) = %d.", err);
		return err;
	}

	/* Init the interrupt workqueue. */
	INIT_WORK(&ff_ctl_context.work_queue, ff_ctl_device_event);

	/* Init the wake lock. */
	wakeup_source_init(&ff_ctl_context.wake_lock, "ff_wake_lock");

	/* Assign the context instance. */
	g_context = &ff_ctl_context;
	FF_LOGD("g_context = %p.", g_context);
	g_context->b_driver_inited = false;

	FF_LOGI("FocalTech fingerprint device control driver registered.");
	FF_LOGV("'%s' leave.", __func__);
	return err;
}
#if USE_PLATFORM_BUS
static int ff_remove(struct platform_device *dev)
#elif USE_SPI_BUS
static int ff_remove(struct spi_device *dev)
#endif
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);

	/* Release the HW resources if needs. */
	if (g_context->b_driver_inited) {
		err = ff_ctl_free_driver();
		g_context->b_driver_inited = false;
	}

	/* De-init the wake lock. */
	wakeup_source_trash(&g_context->wake_lock);

	/* Unregister the miscellaneous device. */
	misc_deregister(&g_context->miscdev);

	/* 'g_context' could not use any more. */
	// g_context = NULL; By lijing on 210929.

	FF_LOGI("FocalTech fingerprint device control driver released.");
	FF_LOGV("'%s' leave.", __func__);
	return err;
}

#if USE_PLATFORM_BUS
static int ff_suspend(struct platform_device *dev, pm_message_t state)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	return err;
}

static int ff_resume(struct platform_device *dev)
{
	int err = 0;
	FF_LOGV("'%s' enter.", __func__);
	return err;
}
#endif

static const struct of_device_id ff_of_match[] = {
	{ .compatible = "mediatek,focaltech-fp" },
	{},
};
#if USE_PLATFORM_BUS
static struct platform_driver ff_driver = {
#elif USE_SPI_BUS
static struct spi_driver ff_driver = {
#endif
	.driver = {
		.name = "focal-fp",
#if USE_PLATFORM_BUS
		.bus = &platform_bus_type,
#elif USE_SPI_BUS
		.bus = &spi_bus_type,
#endif
		.owner = THIS_MODULE,
		.of_match_table = ff_of_match,
	},
	.probe  = ff_probe,
	.remove = ff_remove,
#if USE_PLATFORM_BUS
	.suspend = ff_suspend,
	.resume = ff_resume,
#endif
};

extern unsigned int is_atboot;
extern char *get_board_version(void);
static int __init ff_ctl_driver_init(void)
{
	int err = 0;
	struct device_node *node = NULL;
	char *board_version = NULL;
	const char *project_name = NULL;
	FF_LOGV("'%s' enter.", __func__);

	if (is_atboot == 1) {
		FF_LOGI("%s(): is_at_mode, exit\n", __func__);
		return err;
	}

	node = of_find_compatible_node(NULL, NULL, "fp-id");
	if (node) {
		if (of_property_read_bool(node, "vivo,support_soft_fingerprint_id")) {
			FF_LOGI("%s, support software detect fpid.", __func__);
		} else {
			FF_LOGI("%s, not support software detect fpid.", __func__);
			if (get_fp_id() != FOCALTECH_FT9391) {
				FF_LOGI("%s(): wrong fp id, exit\n", __func__);
				return err;
			}
		}
	} else {
		FF_LOGI("%s, fp_id node is NULL", __func__);
		return err;
	}

#if USE_PLATFORM_BUS
	err = platform_driver_register(&ff_driver);
#elif USE_SPI_BUS
	err = spi_register_driver(&ff_driver);
#endif
	if (err < 0) {
		FF_LOGE("register_driver(..) = %d.", err);
	}

	FF_LOGV("'%s' leave.", __func__);
	return err;
}

static void __exit ff_ctl_driver_exit(void)
{
	FF_LOGV("'%s' enter.", __func__);
#if USE_PLATFORM_BUS
	platform_driver_unregister(&ff_driver);
#elif USE_SPI_BUS
	spi_unregister_driver(&ff_driver);
#endif
	FF_LOGV("'%s' leave.", __func__);
}

late_initcall(ff_ctl_driver_init);
module_exit(ff_ctl_driver_exit);

MODULE_DESCRIPTION("The device control driver for FocalTech's fingerprint sensor.");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("FocalTech Fingerprint R&D department");

