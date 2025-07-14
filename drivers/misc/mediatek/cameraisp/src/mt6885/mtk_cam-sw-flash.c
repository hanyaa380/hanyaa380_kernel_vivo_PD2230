// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2021 MediaTek Inc.
#include <linux/module.h>
#include "mtk_cam-sw-flash.h"


static struct sw_flash_data flash_data;

void mtk_cam_sw_flash_register(int (*flash_cb)(void))
{
	flash_data.is_registered = true;
	flash_data.flash_func_cb = flash_cb;
}
EXPORT_SYMBOL(mtk_cam_sw_flash_register);

void mtk_cam_sw_flash_unregister(void)
{
	flash_data.is_registered = false;
	flash_data.flash_func_cb = NULL;
}
EXPORT_SYMBOL(mtk_cam_sw_flash_unregister);

struct sw_flash_data *mtk_cam_get_swflashdata(void)
{
	return &flash_data;
}


