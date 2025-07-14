/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __EEPROM_I2C_COMMON_DRIVER_H
#define __EEPROM_I2C_COMMON_DRIVER_H
#include <linux/i2c.h>

#if defined(CONFIG_MTK_CAM_PD2279F)
unsigned int Common_read_region_gc02m1bpd2279f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
unsigned int Common_read_region_gc02m1bpd2279hf(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

#if defined(CONFIG_MTK_CAM_PD2257F_EX)
unsigned int Common_read_region_gc02m1pd2257f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int Common_read_region_gc02m1bpd2257f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int Common_read_region_gc02m1bzpd2257f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

#if defined(CONFIG_MTK_CAM_PD2230)
unsigned int Common_read_region_gc02m1(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
unsigned int Common_read_region_ov02b10pd2230(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

#if defined(CONFIG_MTK_CAM_PD2230F)
unsigned int Common_read_region_gc02m1bpd2230f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

#if defined(CONFIG_MTK_CAM_PD2250F)
unsigned int Common_read_region_gc02m1pd2250f(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

#if defined(CONFIG_MTK_CAM_PD2250)
unsigned int Common_read_region_gc02m1pd2250(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);
#endif

unsigned int Common_read_region(struct i2c_client *client,
				unsigned int addr,
				unsigned char *data,
				unsigned int size);

unsigned int Common_write_region(struct i2c_client *client,
				 unsigned int addr,
				 unsigned char *data,
				 unsigned int size);

unsigned int DW9763_write_region(struct i2c_client *client,
				 unsigned int addr,
				 unsigned char *data,
				 unsigned int size);

unsigned int BL24SA64_write_region(struct i2c_client *client,
				 unsigned int addr,
				 unsigned char *data,
				 unsigned int size);

#endif				/* __CAM_CAL_LIST_H */
