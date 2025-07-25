/*
 *  linux/drivers/mmc/core/host.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright (C) 2007-2008 Pierre Ossman
 *  Copyright (C) 2010 Linus Walleij
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC host class device management
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pagemap.h>
#include <linux/export.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/string.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/slot-gpio.h>

#include "core.h"
#include "crypto.h"
#include "host.h"
#include "slot-gpio.h"
#include "pwrseq.h"
#include "sdio_ops.h"

static struct mutex sd_status_mutex;
static int sd_cmd_done_status = -1;

#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

static DEFINE_IDA(mmc_host_ida);

static void mmc_host_classdev_release(struct device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	ida_simple_remove(&mmc_host_ida, host->index);
	kfree(host);
}

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.dev_release	= mmc_host_classdev_release,
};

int mmc_register_host_class(void)
{
	return class_register(&mmc_host_class);
}

void mmc_unregister_host_class(void)
{
	class_unregister(&mmc_host_class);
}

void mmc_retune_enable(struct mmc_host *host)
{
	host->can_retune = 1;
	if (host->retune_period)
		mod_timer(&host->retune_timer,
			  jiffies + host->retune_period * HZ);
}

/*
 * Pause re-tuning for a small set of operations.  The pause begins after the
 * next command and after first doing re-tuning.
 */
void mmc_retune_pause(struct mmc_host *host)
{
	if (!host->retune_paused) {
		host->retune_paused = 1;
		mmc_retune_needed(host);
		mmc_retune_hold(host);
	}
}
EXPORT_SYMBOL(mmc_retune_pause);

void mmc_retune_unpause(struct mmc_host *host)
{
	if (host->retune_paused) {
		host->retune_paused = 0;
		mmc_retune_release(host);
	}
}
EXPORT_SYMBOL(mmc_retune_unpause);

void mmc_retune_disable(struct mmc_host *host)
{
	mmc_retune_unpause(host);
	host->can_retune = 0;
	del_timer_sync(&host->retune_timer);
	host->retune_now = 0;
	host->need_retune = 0;
}

void mmc_retune_timer_stop(struct mmc_host *host)
{
	del_timer_sync(&host->retune_timer);
}
EXPORT_SYMBOL(mmc_retune_timer_stop);

void mmc_retune_hold(struct mmc_host *host)
{
	if (!host->hold_retune)
		host->retune_now = 1;
	host->hold_retune += 1;
}

void mmc_retune_release(struct mmc_host *host)
{
	if (host->hold_retune)
		host->hold_retune -= 1;
	else
		WARN_ON(1);
}
EXPORT_SYMBOL(mmc_retune_release);

int mmc_retune(struct mmc_host *host)
{
	bool return_to_hs400 = false;
	int err;

	if (host->retune_now)
		host->retune_now = 0;
	else
		return 0;

	if (!host->need_retune || host->doing_retune || !host->card)
		return 0;

	host->need_retune = 0;

	host->doing_retune = 1;

	if (host->ios.timing == MMC_TIMING_MMC_HS400) {
		err = mmc_hs400_to_hs200(host->card);
		if (err)
			goto out;

		return_to_hs400 = true;
	}

	err = mmc_execute_tuning(host->card);
	if (err)
		goto out;

	if (return_to_hs400)
		err = mmc_hs200_to_hs400(host->card);
out:
	host->doing_retune = 0;
	if (err)
		host->need_retune = 1;
	return err;
}

static void mmc_retune_timer(struct timer_list *t)
{
	struct mmc_host *host = from_timer(host, t, retune_timer);

	mmc_retune_needed(host);
}

bool is_PD2281F_EX = 0;
/**
 *	mmc_of_parse() - parse host's device-tree node
 *	@host: host whose node should be parsed.
 *
 * To keep the rest of the MMC subsystem unaware of whether DT has been
 * used to to instantiate and configure this host instance or not, we
 * parse the properties and set respective generic mmc-host flags and
 * parameters.
 */
int mmc_of_parse(struct mmc_host *host)
{
	struct device *dev = host->parent;
	u32 bus_width, drv_type, cd_debounce_delay_ms;
	int ret;
	bool cd_cap_invert, cd_gpio_invert = false;
	bool ro_cap_invert, ro_gpio_invert = false;

	if (!dev || !dev_fwnode(dev))
		return 0;

	/* "bus-width" is translated to MMC_CAP_*_BIT_DATA flags */
	if (device_property_read_u32(dev, "bus-width", &bus_width) < 0) {
		dev_dbg(host->parent,
			"\"bus-width\" property is missing, assuming 1 bit.\n");
		bus_width = 1;
	}

	switch (bus_width) {
	case 8:
		host->caps |= MMC_CAP_8_BIT_DATA;
		/* Hosts capable of 8-bit transfers can also do 4 bits */
	case 4:
		host->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
		break;
	default:
		dev_err(host->parent,
			"Invalid \"bus-width\" value %u!\n", bus_width);
		return -EINVAL;
	}

	/* f_max is obtained from the optional "max-frequency" property */
	device_property_read_u32(dev, "max-frequency", &host->f_max);

	/*
	 * Configure CD and WP pins. They are both by default active low to
	 * match the SDHCI spec. If GPIOs are provided for CD and / or WP, the
	 * mmc-gpio helpers are used to attach, configure and use them. If
	 * polarity inversion is specified in DT, one of MMC_CAP2_CD_ACTIVE_HIGH
	 * and MMC_CAP2_RO_ACTIVE_HIGH capability-2 flags is set. If the
	 * "broken-cd" property is provided, the MMC_CAP_NEEDS_POLL capability
	 * is set. If the "non-removable" property is found, the
	 * MMC_CAP_NONREMOVABLE capability is set and no card-detection
	 * configuration is performed.
	 */

	/* Parse Card Detection */
	if (device_property_read_bool(dev, "non-removable")) {
		host->caps |= MMC_CAP_NONREMOVABLE;
	} else {
		cd_cap_invert = device_property_read_bool(dev, "cd-inverted");

		if (device_property_read_u32(dev, "cd-debounce-delay-ms",
					     &cd_debounce_delay_ms))
			cd_debounce_delay_ms = 200;

		if (device_property_read_bool(dev, "broken-cd"))
			host->caps |= MMC_CAP_NEEDS_POLL;

		ret = mmc_gpiod_request_cd(host, "cd", 0, true,
					   cd_debounce_delay_ms * 1000,
					   &cd_gpio_invert);
		if (!ret)
			dev_info(host->parent, "Got CD GPIO\n");
		else if (ret != -ENOENT && ret != -ENOSYS)
			return ret;

		/*
		 * There are two ways to flag that the CD line is inverted:
		 * through the cd-inverted flag and by the GPIO line itself
		 * being inverted from the GPIO subsystem. This is a leftover
		 * from the times when the GPIO subsystem did not make it
		 * possible to flag a line as inverted.
		 *
		 * If the capability on the host AND the GPIO line are
		 * both inverted, the end result is that the CD line is
		 * not inverted.
		 */
		if (cd_cap_invert ^ cd_gpio_invert)
			host->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;
	}

	/* Parse Write Protection */
	ro_cap_invert = device_property_read_bool(dev, "wp-inverted");

	ret = mmc_gpiod_request_ro(host, "wp", 0, false, 0, &ro_gpio_invert);
	if (!ret)
		dev_info(host->parent, "Got WP GPIO\n");
	else if (ret != -ENOENT && ret != -ENOSYS)
		return ret;

	if (device_property_read_bool(dev, "disable-wp"))
		host->caps2 |= MMC_CAP2_NO_WRITE_PROTECT;

	/* See the comment on CD inversion above */
	if (ro_cap_invert ^ ro_gpio_invert)
		host->caps2 |= MMC_CAP2_RO_ACTIVE_HIGH;

	if (device_property_read_bool(dev, "cap-sd-highspeed"))
		host->caps |= MMC_CAP_SD_HIGHSPEED;
	if (device_property_read_bool(dev, "cap-mmc-highspeed"))
		host->caps |= MMC_CAP_MMC_HIGHSPEED;
	if (device_property_read_bool(dev, "sd-uhs-sdr12"))
		host->caps |= MMC_CAP_UHS_SDR12;
	if (device_property_read_bool(dev, "sd-uhs-sdr25"))
		host->caps |= MMC_CAP_UHS_SDR25;
	if (device_property_read_bool(dev, "sd-uhs-sdr50"))
		host->caps |= MMC_CAP_UHS_SDR50;
	if (device_property_read_bool(dev, "sd-uhs-sdr104"))
		host->caps |= MMC_CAP_UHS_SDR104;
	if (device_property_read_bool(dev, "sd-uhs-ddr50"))
		host->caps |= MMC_CAP_UHS_DDR50;
	if (device_property_read_bool(dev, "cap-power-off-card"))
		host->caps |= MMC_CAP_POWER_OFF_CARD;
	if (device_property_read_bool(dev, "cap-mmc-hw-reset"))
		host->caps |= MMC_CAP_HW_RESET;
	if (device_property_read_bool(dev, "cap-sdio-irq"))
		host->caps |= MMC_CAP_SDIO_IRQ;
	if (device_property_read_bool(dev, "full-pwr-cycle"))
		host->caps2 |= MMC_CAP2_FULL_PWR_CYCLE;
	if (device_property_read_bool(dev, "keep-power-in-suspend"))
		host->pm_caps |= MMC_PM_KEEP_POWER;
	if (device_property_read_bool(dev, "wakeup-source") ||
	    device_property_read_bool(dev, "enable-sdio-wakeup")) /* legacy */
		host->pm_caps |= MMC_PM_WAKE_SDIO_IRQ;
	if (device_property_read_bool(dev, "mmc-ddr-3_3v"))
		host->caps |= MMC_CAP_3_3V_DDR;
	if (device_property_read_bool(dev, "mmc-ddr-1_8v"))
		host->caps |= MMC_CAP_1_8V_DDR;
	if (device_property_read_bool(dev, "mmc-ddr-1_2v"))
		host->caps |= MMC_CAP_1_2V_DDR;
	if (device_property_read_bool(dev, "mmc-hs200-1_8v"))
		host->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
	if (device_property_read_bool(dev, "mmc-hs200-1_2v"))
		host->caps2 |= MMC_CAP2_HS200_1_2V_SDR;
	if (device_property_read_bool(dev, "mmc-hs400-1_8v"))
		host->caps2 |= MMC_CAP2_HS400_1_8V | MMC_CAP2_HS200_1_8V_SDR;
	if (device_property_read_bool(dev, "mmc-hs400-1_2v"))
		host->caps2 |= MMC_CAP2_HS400_1_2V | MMC_CAP2_HS200_1_2V_SDR;
	if (device_property_read_bool(dev, "mmc-hs400-enhanced-strobe"))
		host->caps2 |= MMC_CAP2_HS400_ES;
	if (device_property_read_bool(dev, "no-sdio"))
		host->caps2 |= MMC_CAP2_NO_SDIO;
	if (device_property_read_bool(dev, "no-sd"))
		host->caps2 |= MMC_CAP2_NO_SD;
	if (device_property_read_bool(dev, "no-mmc"))
		host->caps2 |= MMC_CAP2_NO_MMC;

	// vivo add for tf card fast_power_off begin
	if (device_property_read_bool(dev, "fast_power_off"))
		host->fast_power_off = true;
	// vivo add for tf card fast_power_off end

	/* Must be after "non-removable" check */
	if (device_property_read_u32(dev, "fixed-emmc-driver-type", &drv_type) == 0) {
		if (host->caps & MMC_CAP_NONREMOVABLE)
			host->fixed_drv_type = drv_type;
		else
			dev_err(host->parent,
				"can't use fixed driver type, media is removable\n");
	}

	if (device_property_read_bool(dev, "is_PD2281F_project"))
		is_PD2281F_EX = 1;
	host->dsr_req = !device_property_read_u32(dev, "dsr", &host->dsr);
	if (host->dsr_req && (host->dsr & ~0xffff)) {
		dev_err(host->parent,
			"device tree specified broken value for DSR: 0x%x, ignoring\n",
			host->dsr);
		host->dsr_req = 0;
	}

	device_property_read_u32(dev, "post-power-on-delay-ms",
				 &host->ios.power_delay_ms);

	return mmc_pwrseq_alloc(host);
}

EXPORT_SYMBOL(mmc_of_parse);

/**
 *	mmc_alloc_host - initialise the per-host structure.
 *	@extra: sizeof private data structure
 *	@dev: pointer to host device model structure
 *
 *	Initialise the per-host structure.
 */
struct mmc_host *mmc_alloc_host(int extra, struct device *dev)
{
	int err;
	struct mmc_host *host;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	int i;
#endif

	host = kzalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (!host)
		return NULL;

	/* scanning will be enabled when we're ready */
	host->rescan_disable = 1;

	err = ida_simple_get(&mmc_host_ida, 0, 0, GFP_KERNEL);
	if (err < 0) {
		kfree(host);
		return NULL;
	}

	host->index = err;

	dev_set_name(&host->class_dev, "mmc%d", host->index);

	host->parent = dev;
	host->class_dev.parent = dev;
	host->class_dev.class = &mmc_host_class;
	device_initialize(&host->class_dev);
	device_enable_async_suspend(&host->class_dev);

	if (mmc_gpio_alloc(host)) {
		put_device(&host->class_dev);
		return NULL;
	}
	host->bad_card = 0;
	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	INIT_DELAYED_WORK(&host->detect, mmc_rescan);
	INIT_DELAYED_WORK(&host->sdio_irq_work, sdio_irq_work);
	timer_setup(&host->retune_timer, mmc_retune_timer, 0);

	/*
	 * By default, hosts do not support SGIO or large requests.
	 * They have to set these according to their abilities.
	 */
	host->max_segs = 1;
	host->max_seg_size = PAGE_SIZE;

	host->max_req_size = PAGE_SIZE;
	host->max_blk_size = 512;
	host->max_blk_count = PAGE_SIZE / 512;

	host->fixed_drv_type = -EINVAL;
	host->ios.power_delay_ms = 10;

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (host->index == 0) {
		for (i = 0; i < EMMC_MAX_QUEUE_DEPTH; i++)
			host->areq_que[i] = NULL;

		atomic_set(&host->areq_cnt, 0);
		host->areq_cur = NULL;
		host->done_mrq = NULL;
		INIT_LIST_HEAD(&host->cmd_que);
		INIT_LIST_HEAD(&host->dat_que);
		spin_lock_init(&host->cmd_que_lock);
		spin_lock_init(&host->dat_que_lock);
		spin_lock_init(&host->que_lock);
		init_waitqueue_head(&host->cmp_que);
		init_waitqueue_head(&host->cmdq_que);
	}
#endif

	return host;
}

EXPORT_SYMBOL(mmc_alloc_host);

static struct device *pdev;

extern int desc_to_gpio(const struct gpio_desc *desc);
struct mmc_gpio {
        bool status;
        struct gpio_desc *ro_gpio;
        struct gpio_desc *cd_gpio;
        bool override_ro_active_level;
        bool override_cd_active_level;
        irqreturn_t (*cd_gpio_isr)(int irq, void *dev_id);
        char *ro_label;
        u32 cd_debounce_delay_ms;
        char cd_label[];
};

void mmc_set_sd_status(int value)
{
	mutex_lock(&sd_status_mutex);
	if (value == 0)
		sd_cmd_done_status = 1;
	else if (value == -1)
		sd_cmd_done_status = -1;
	else
		sd_cmd_done_status = 0;
	mutex_unlock(&sd_status_mutex);
}
EXPORT_SYMBOL(mmc_set_sd_status);

static int mmc_get_sd_status(void)
{
	int ret = 0;

	mutex_lock(&sd_status_mutex);
	ret = sd_cmd_done_status;
	mutex_unlock(&sd_status_mutex);

	return ret;
}

/*
 * Get the current status of sd
 *
 * @ return value
 * 		1		sd card insert
 * 		0		no sd card insert
 * 		-1		wait 3s and try again
 */

static ssize_t sd_status_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct mmc_host *host;
	struct mmc_gpio *ctx;
	int current_sd_status = -1;
	int gpio_num_val = 0;
	int gpio_level_value = 0;
	int _sd_cmd_done_status = 0;
	if(!pdev)
		return -EINVAL;
	host = cls_dev_to_mmc_host(pdev);
	ctx = host->slot.handler_priv;
	if (host->caps & MMC_CAP_NONREMOVABLE) {
		return -1;
	}
	gpio_num_val = desc_to_gpio(ctx->cd_gpio);
	gpio_level_value = gpio_get_value(gpio_num_val);
	_sd_cmd_done_status = mmc_get_sd_status();
	mutex_lock(&sd_status_mutex);
	if (!gpio_level_value)		/* no insert irq */
		current_sd_status = 0;
	else if (gpio_level_value && !_sd_cmd_done_status)		/* insert irq + cmd transfer error */
		current_sd_status = 0;
	else if (gpio_level_value && (_sd_cmd_done_status == -1))	/* insert irq + cmd transfering */
		current_sd_status = -1;
	else if (gpio_level_value && (_sd_cmd_done_status == 1)) 	/* insert irq + cmd transfer successful */
		current_sd_status = 1;
	else
		current_sd_status = -1;

	pr_err("msdc: %s, %d, _sd_cmd_done_status:%d, gpio_level_value:%d, current_sd_status:%d\n", __func__, __LINE__, \
		_sd_cmd_done_status, gpio_level_value, current_sd_status);

	mutex_unlock(&sd_status_mutex);
	return sprintf(buf, "%d\n", current_sd_status);
}
DEVICE_ATTR(sd_status, 0644, sd_status_show, NULL);

static int creat_mmc_debugfs_node(void)
{
	int ret = 0;
	struct class *node_class;
	struct device *node_device;

	node_class = class_create(THIS_MODULE, "mmc_debugfs");
	if (IS_ERR_OR_NULL(node_class)) {
		pr_err("class_create failed! \n");
		return -1;
	}

	node_device = device_create(node_class, NULL, MKDEV(0, 0), NULL, "mmc");
	if (IS_ERR_OR_NULL(node_device)) {
		pr_err("device_create failed! \n");
		return -1;
	}

	ret = sysfs_create_file(&node_device->kobj, &dev_attr_sd_status.attr);
	if (ret < 0) {
		pr_err("create sd_status node failed !\n");
		return -1;
	}

	return 0;
}

static ssize_t show_int_cnt_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);

	return sprintf(buf, "%d\n", host->int_cnt_enable);

}

static ssize_t store_int_cnt_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	int val,ret;
	ret = sscanf(buf, "%d", &val);
	if (ret <= 0)
		return (ret < 0) ? ret : -EINVAL;

	/* We need this check due to input value is u64 */
	if (val < 0)
		return -EINVAL;

	mmc_claim_host(host);
	host->int_cnt_enable = val;
	if (val == 1)
		host->int_cnt = 0;
	mmc_release_host(host);

	return count;
}

static ssize_t show_int_cnt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_host	*host = cls_dev_to_mmc_host(dev);

	return sprintf(buf, "%d\n", host->int_cnt);
}

DEVICE_ATTR(int_cnt_enable, 0644,
		show_int_cnt_enable, store_int_cnt_enable);
DEVICE_ATTR(int_cnt, 0644,
		show_int_cnt, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_int_cnt_enable.attr,
	&dev_attr_int_cnt.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

/**
 *	mmc_add_host - initialise host hardware
 *	@host: mmc host
 *
 *	Register the host with the driver model. The host must be
 *	prepared to start servicing requests before this function
 *	completes.
 */
int mmc_add_host(struct mmc_host *host)
{
	int err;

	mutex_init(&sd_status_mutex);

	WARN_ON((host->caps & MMC_CAP_SDIO_IRQ) &&
		!host->ops->enable_sdio_irq);

	err = device_add(&host->class_dev);
	if (err)
		return err;

	led_trigger_register_simple(dev_name(&host->class_dev), &host->led);

#ifdef CONFIG_DEBUG_FS
	mmc_add_host_debugfs(host);
#endif

	if (!(host->caps & MMC_CAP_NONREMOVABLE)) {
		pdev = &host->class_dev;
		err = creat_mmc_debugfs_node();
		if (err)
			pr_err("%s: failed to sd_status_node with err %d\n",
								__func__, err);
	}
	err = sysfs_create_group(&host->class_dev.kobj, &dev_attr_grp);
	if (err)
		pr_err("%s: failed to create sysfs group with err %d\n",
												__func__, err);

	mmc_start_host(host);
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		mmc_register_pm_notifier(host);

	return 0;
}

EXPORT_SYMBOL(mmc_add_host);

/**
 *	mmc_remove_host - remove host hardware
 *	@host: mmc host
 *
 *	Unregister and remove all cards associated with this host,
 *	and power down the MMC bus. No new requests will be issued
 *	after this function has returned.
 */
void mmc_remove_host(struct mmc_host *host)
{
	if (!(host->pm_flags & MMC_PM_IGNORE_PM_NOTIFY))
		mmc_unregister_pm_notifier(host);
	mmc_stop_host(host);

#ifdef CONFIG_DEBUG_FS
	mmc_remove_host_debugfs(host);
#endif

	sysfs_remove_group(&host->parent->kobj, &dev_attr_grp);
	device_del(&host->class_dev);

	led_trigger_unregister_simple(host->led);

	mutex_destroy(&sd_status_mutex);

}

EXPORT_SYMBOL(mmc_remove_host);

/**
 *	mmc_free_host - free the host structure
 *	@host: mmc host
 *
 *	Free the host once all references to it have been dropped.
 */
void mmc_free_host(struct mmc_host *host)
{
	mmc_crypto_free_host(host);
	mmc_pwrseq_free(host);
	put_device(&host->class_dev);
}

EXPORT_SYMBOL(mmc_free_host);
