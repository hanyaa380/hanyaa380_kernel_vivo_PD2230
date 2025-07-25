/* SPDX-License-Identifier: GPL-2.0 */
/*
* Copyright (C) 2021 MediaTek Inc.
*/

#include <accdet.h>

static struct platform_driver accdet_driver;
static int debug_enable_drv = 1;

long __weak mt_accdet_unlocked_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return 0;
}
static int accdet_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int accdet_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations accdet_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mt_accdet_unlocked_ioctl,
	.open = accdet_open,
	.release = accdet_release,
};

const struct file_operations *accdet_get_fops(void)
{
	return &accdet_fops;
}

static int accdet_probe(struct platform_device *dev)
{
	return 0;
}

static int accdet_remove(struct platform_device *dev)
{
	return 0;
}

const struct of_device_id accdet_of_match[] = {
	{ .compatible = "mediatek,mt8173-accdet", },
	{ .compatible = "mediatek,mt8163-accdet", },
	{ .compatible = "mediatek,pmic-accdet", },
	{ .compatible = "mediatek,mt8167-accdet", },
	{},
};

static struct platform_driver accdet_driver = {
	.probe = accdet_probe,
	.remove = accdet_remove,
	.driver = {
		.name = "Accdet_Driver",
		.of_match_table = accdet_of_match,
	},
};

struct platform_driver accdet_driver_func(void)
{
	return accdet_driver;
}

static int accdet_mod_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&accdet_driver);
	if (ret)
		pr_notice("Accdet platform_driver_register error:(%d)\n", ret);

	pr_info("%s() done!\n", __func__);
	return ret;
}

static void accdet_mod_exit(void)
{
	pr_info("%s()\n", __func__);
	platform_driver_unregister(&accdet_driver);
}

module_init(accdet_mod_init);
module_exit(accdet_mod_exit);

module_param(debug_enable_drv, int, 0644);

MODULE_DESCRIPTION("MTK ACCDET driver");
MODULE_AUTHOR("Luhua <luhua.xu@mediatek.com>");
MODULE_LICENSE("GPL");
