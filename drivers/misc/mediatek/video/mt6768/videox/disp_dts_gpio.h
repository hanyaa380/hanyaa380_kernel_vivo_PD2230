/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
*/

#ifndef __DISP_DTS_GPIO_H__
#define __DISP_DTS_GPIO_H__

/*
 * This module helps you to set GPIO pin according to linux device tree (DTS).
 * To use this module, you MUST init this module once before any operation.
 */

#include <linux/platform_device.h>	/* struct platform_device */

/* DTS state */
enum DTS_GPIO_STATE {
#ifdef CONFIG_LCM_PANEL_TYPE_TFT
	DTS_GPIO_STATE_TFT_RST_OUT0 = 0,
	DTS_GPIO_STATE_TFT_RST_OUT1,
	DTS_GPIO_STATE_TFT_ENP_OUT0,
	DTS_GPIO_STATE_TFT_ENP_OUT1,
	DTS_GPIO_STATE_TFT_ENN_OUT0,
	DTS_GPIO_STATE_TFT_ENN_OUT1,
	DTS_GPIO_STATE_TFT_BKG_OUT0,
	DTS_GPIO_STATE_TFT_BKG_OUT1,
	DTS_GPIO_STATE_TFT_TPRST_OUT0,
	DTS_GPIO_STATE_TFT_TPRST_OUT1,
	DTS_GPIO_STATE_MAX,	
#else
	DTS_GPIO_STATE_TE_MODE_GPIO = 0,	/* mode_te_gpio */
	DTS_GPIO_STATE_TE_MODE_TE,	/* mode_te_te */
	DTS_GPIO_STATE_TE1_MODE_TE,	/* mode_te1_te */
	DTS_GPIO_STATE_LCM_RST_OUT0,
	DTS_GPIO_STATE_LCM_RST_OUT1,
	DTS_GPIO_STATE_LCM_VCI_OUT0,
	DTS_GPIO_STATE_LCM_VCI_OUT1,
	DTS_GPIO_STATE_LCM1_RST_OUT0,
	DTS_GPIO_STATE_LCM1_RST_OUT1,
	DTS_GPIO_STATE_LCD_BIAS_ENN,
	DTS_GPIO_STATE_LCD_BIAS_ENP,
	DTS_GPIO_STATE_TP_RST_OUT1,
	DTS_GPIO_STATE_TP_RST_OUT0,
	DTS_GPIO_STATE_6382_RST_OUT1,
	DTS_GPIO_STATE_6382_RST_OUT0,
	DTS_GPIO_STATE_MAX,	/* for array size */
#endif
};

/* this function MUST be called in mtkfb_probe.
 *  @param *pdev    - reference of struct platform_device which contains pinctrl
 *                    state information of GPIO
 *  @return         - 0 for OK, otherwise returns PTR_ERR(pdev).
 */
long disp_dts_gpio_init(struct platform_device *pdev);

/* set gpio according sepcified DTS state.
 *  @notice         - to call this function, you MUST init this module first.
 *                    If not, we will trigger BUG_ON(0).
 *  @param s        - state which describes GPIO statement.
 *  @return         - 0 for OK, otherwise returns PTR_ERR(pdev).
 */
long disp_dts_gpio_select_state(enum DTS_GPIO_STATE s);

/* repo of initialization */
#ifdef CONFIG_MTK_LEGACY
#define disp_dts_gpio_init_repo(x)  (0)
#else
#define disp_dts_gpio_init_repo(x)  (disp_dts_gpio_init(x))
#endif

#endif/*__DISP_DTS_GPIO_H__ */
