/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_CAM_SW_FLASH_H
#define __MTK_CAM_SW_FLASH_H

#include <linux/types.h>

struct sw_flash_data {
	bool is_registered;
	int (*flash_func_cb)(void);
};
extern void mtk_cam_sw_flash_register(int (*flash_cb)(void));
extern void mtk_cam_sw_flash_unregister(void);
struct sw_flash_data *mtk_cam_get_swflashdata(void);


#endif /*__MTK_CAM_SW_FLASH_H*/

