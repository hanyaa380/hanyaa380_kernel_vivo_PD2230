/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "eeprom_utils.h"

/* Include platform define if necessary */
#ifdef EEPROM_PLATFORM_DEFINE
#include "eeprom_platform_def.h"
#endif

/************************************************************
 * I2C read function (Common)
 ************************************************************/

/* add for linux-4.4 */
#ifndef I2C_WR_FLAG
#define I2C_WR_FLAG		(0x1000)
#define I2C_MASK_FLAG	(0x00ff)
#endif

#define EEPROM_I2C_MSG_SIZE_READ 2

#ifndef EEPROM_I2C_READ_MSG_LENGTH_MAX
#define EEPROM_I2C_READ_MSG_LENGTH_MAX 255
#endif
#ifndef EEPROM_I2C_WRITE_MSG_LENGTH_MAX
#define EEPROM_I2C_WRITE_MSG_LENGTH_MAX 32
#endif
#ifndef EEPROM_WRITE_EN
#define EEPROM_WRITE_EN 1
#endif
#if defined(CONFIG_MTK_CAM_PD2257F_EX)
extern unsigned char vivo_otp_data_gc02m1pd2257f[0x14];
extern unsigned char vivo_otp_data_gc02m1bpd2257f[0x14];
extern unsigned char vivo_otp_data_gc02m1bzpd2257f[0x14];
#endif
#if defined(CONFIG_MTK_CAM_PD2230)
extern unsigned char vivo_otp_data_gc02m1pd2230[0x14];
#endif
#if defined(CONFIG_MTK_CAM_PD2230F)
extern unsigned char vivo_otp_data_gc02m1bpd2230f[0x15];
#endif
#if defined(CONFIG_MTK_CAM_PD2279F)
extern unsigned char vivo_otp_data_gc02m1bpd2279f[0x15];
extern unsigned char vivo_otp_data_gc02m1bpd2279hf[0x15];
#endif
#if defined(CONFIG_MTK_CAM_PD2250F)
extern unsigned char vivo_otp_data_gc02m1pd2250f[0x14];
#endif
#if defined(CONFIG_MTK_CAM_PD2250)
extern unsigned char vivo_otp_data_gc02m1pd2250[0x14];
#endif
extern unsigned char vivo_otp_data_ov02b10pd2230[0x13];

static int Read_I2C_CAM_CAL(struct i2c_client *client,
			    u16 a_u2Addr,
			    u32 ui4_length,
			    u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };
	struct i2c_msg msg[EEPROM_I2C_MSG_SIZE_READ];

	if (ui4_length > EEPROM_I2C_READ_MSG_LENGTH_MAX) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 EEPROM_I2C_READ_MSG_LENGTH_MAX);
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = puReadCmd;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = ui4_length;
	msg[1].buf = a_puBuff;

	i4RetValue = i2c_transfer(client->adapter, msg,
				EEPROM_I2C_MSG_SIZE_READ);

	if (i4RetValue != EEPROM_I2C_MSG_SIZE_READ) {
		pr_debug("I2C read data failed!!\n");
		return -1;
	}

	return 0;
}

static int iReadData_CAM_CAL(struct i2c_client *client,
			     unsigned int ui4_offset,
			     unsigned int ui4_length,
			     unsigned char *pinputdata)
{
	int i4ResidueSize;
	u32 u4CurrentOffset, u4Size;
	u8 *pBuff;

	i4ResidueSize = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		u4Size = (i4ResidueSize >= EEPROM_I2C_READ_MSG_LENGTH_MAX)
			? EEPROM_I2C_READ_MSG_LENGTH_MAX : i4ResidueSize;

		if (Read_I2C_CAM_CAL(client, (u16) u4CurrentOffset,
				     u4Size, pBuff) != 0) {
			pr_debug("I2C iReadData failed!!\n");
			return -1;
		}

		i4ResidueSize -= u4Size;
		u4CurrentOffset += u4Size;
		pBuff += u4Size;
	} while (i4ResidueSize > 0);

	return 0;
}

#if EEPROM_WRITE_EN
static int Write_I2C_CAM_CAL(struct i2c_client *client,
			     u16 a_u2Addr,
			     u32 ui4_length,
			     u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puCmd[2 + EEPROM_I2C_WRITE_MSG_LENGTH_MAX];
	struct i2c_msg msg;

	if (ui4_length > EEPROM_I2C_WRITE_MSG_LENGTH_MAX) {
		pr_debug("exceed one transition %d bytes limitation\n",
			 EEPROM_I2C_WRITE_MSG_LENGTH_MAX);
		return -1;
	}

	puCmd[0] = (char)(a_u2Addr >> 8);
	puCmd[1] = (char)(a_u2Addr & 0xFF);
	memcpy(puCmd + 2, a_puBuff, ui4_length);

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 2 + ui4_length;
	msg.buf = puCmd;

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);

	if (i4RetValue != 1) {
		pr_debug("I2C write data failed!!\n");
		return -1;
	}

	/* Wait for write complete */
	mdelay(5);

	return 0;
}

static int iWriteData_CAM_CAL(struct i2c_client *client,
			     unsigned int ui4_offset,
			     unsigned int ui4_length,
			     unsigned char *pinputdata)
{
	int i4ResidueSize;
	u32 u4CurrentOffset, u4Size;
	u8 *pBuff;

	i4ResidueSize = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		u4Size = (i4ResidueSize >= EEPROM_I2C_WRITE_MSG_LENGTH_MAX)
			? EEPROM_I2C_WRITE_MSG_LENGTH_MAX : i4ResidueSize;

		if (Write_I2C_CAM_CAL(client, (u16) u4CurrentOffset,
					u4Size, pBuff) != 0) {
			pr_debug("I2C iWriteData failed!!\n");
			return -1;
		}

		i4ResidueSize -= u4Size;
		u4CurrentOffset += u4Size;
		pBuff += u4Size;
	} while (i4ResidueSize > 0);

	return 0;
}
#endif

unsigned int Common_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;

	EEPROM_PROFILE_INIT(&t);

	if (iReadData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}

#if defined(CONFIG_MTK_CAM_PD2257F_EX)
unsigned int Common_read_region_gc02m1pd2257f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x14) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1pd2257f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}

unsigned int Common_read_region_gc02m1bpd2257f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x14) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1bpd2257f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}

unsigned int Common_read_region_gc02m1bzpd2257f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x14) return -1;


	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1bzpd2257f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

#if defined(CONFIG_MTK_CAM_PD2230)
unsigned int Common_read_region_ov02b10pd2230(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x13) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_ov02b10pd2230[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}

unsigned int Common_read_region_gc02m1(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x14) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1pd2230[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

#if defined(CONFIG_MTK_CAM_PD2230F)
unsigned int Common_read_region_gc02m1bpd2230f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x15) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1bpd2230f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

#if defined(CONFIG_MTK_CAM_PD2279F)
unsigned int Common_read_region_gc02m1bpd2279f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x15) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1bpd2279f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
unsigned int Common_read_region_gc02m1bpd2279hf(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);
	if((addr + size) > 0x15) return -1;

	for (i = addr;i < addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1bpd2279hf[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

#if defined(CONFIG_MTK_CAM_PD2250F)
unsigned int Common_read_region_gc02m1pd2250f(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);

	for (i = addr;i <= addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1pd2250f[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

#if defined(CONFIG_MTK_CAM_PD2250)
unsigned int Common_read_region_gc02m1pd2250(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
	struct timeval t;
	int i = 0;

	EEPROM_PROFILE_INIT(&t);

	for (i = addr;i <= addr + size;i++)
		data[i-addr] = vivo_otp_data_gc02m1pd2250[i];
	ret = size;

	EEPROM_PROFILE(&t, "common_read_time");

	return ret;
}
#endif

unsigned int Common_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	EEPROM_PROFILE_INIT(&t);

	if (iWriteData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "common_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}

unsigned int DW9763_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	int i4RetValue = 0;
	char puCmd[2];
	struct i2c_msg msg;

	EEPROM_PROFILE_INIT(&t);

	puCmd[0] = (char)(0x81);
	puCmd[1] = (char)(0xEE);

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 2;
	msg.buf = puCmd;

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);

	if (i4RetValue != 1) {
		pr_debug("I2C erase data failed!!\n");
		return -1;
	}

	/* Wait for erase complete */
	mdelay(30);

	if (iWriteData_CAM_CAL(client, addr, size, data) == 0)
		ret = size;

	EEPROM_PROFILE(&t, "DW9763_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}

unsigned int BL24SA64_write_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	unsigned int ret = 0;
#if EEPROM_WRITE_EN
	struct timeval t;

	unsigned char test_read = 0x00;
	unsigned char unlock_cmd = 0x40;
	unsigned char lock_cmd = 0x78;
	unsigned char unlock_val = 0x00 | ((client->addr) & 0x7) << 4; // 0x50 -> 0x00
	unsigned char lock_val = unlock_val | 0x0F; // 0x50 -> 0x0F
	unsigned int ori_addr = 0x00; // to store the current address during sending cmd.
	unsigned int exp_addr = 0x00; // to store the expected address after lock EEPROM.
	unsigned int i = 0;

	EEPROM_PROFILE_INIT(&t);

/************ test read EEPROM ************/

	exp_addr = client->addr;
	if (iReadData_CAM_CAL(client, 0x0008, 1, &test_read) < 0) {
		pr_debug("Read EEPROM ID failed\n");
		pr_debug("Start looping slave address 0x50 ~ 0x57\n");
		for (i = 0; i < 8; i++) {
			client->addr = 0x50+i;
			pr_debug("Change slave address to 0x%02x\n", client->addr);
			if (iReadData_CAM_CAL(client, 0x0008, 1, &test_read) == 0) {
				pr_debug("EEPROM ID = 0x%02x\n", test_read);
				break;
			}
		}
	} else
		pr_debug("EEPROM ID = 0x%02x\n", test_read);

	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) < 0) {
		pr_debug("Read register failed\n");
		return -1;
	}
	pr_debug("Register ID = 0x%02x\n", test_read);

/************ unlock EEPROM ************/

	pr_debug("BL24SA64 write unlock 0x%02x\n", unlock_val);

	ori_addr = client->addr;
	client->addr = unlock_cmd;

	iWriteData_CAM_CAL(client, 0x8000, 1, &unlock_val);
	pr_debug("BL24SA64 unlock part1\n");

	client->addr = ori_addr;

	if (iWriteData_CAM_CAL(client, 0x8000, 1, &unlock_val) < 0) {
		pr_debug("Unlock protection failed!!\n");
		return -1;
	}
	pr_debug("BL24SA64 unlock done\n");

/************ test read EEPROM ************/

	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) == 0)
		pr_debug("Register ID = 0x%02x\n", test_read);
	else {
		pr_debug("Read register failed!!\n");
		return -1;
	}

/************ write EEPROM ************/

	if (iWriteData_CAM_CAL(client, addr, size, data) < 0)
		pr_debug("Write EEPROM failed!!\n");
	else
		ret = size;
	pr_debug("Write EEPROM ret = %d\n", ret);

/************ lock EEPROM ************/

	pr_debug("BL24SA64 write lock 0x%02x\n", lock_val);

	ori_addr = client->addr;
	client->addr = lock_cmd;

	iWriteData_CAM_CAL(client, 0x8000, 1, &lock_val);
	pr_debug("BL24SA64 lock part1\n");

	client->addr = ori_addr;

	if (iWriteData_CAM_CAL(client, 0x8000, 1, &lock_val) < 0) {
		pr_debug("Lock protection failed!!\n");
		return -1;
	}
	pr_debug("BL24SA64 lock done\n");

/************ test read EEPROM ************/

	client->addr = exp_addr;
	if (iReadData_CAM_CAL(client, 0x8000, 1, &test_read) == 0)
		pr_debug("Register ID = 0x%02x\n", test_read);
	else {
		pr_debug("Failed to read register!!\n");
		return -1;
	}

/***************************************/

	EEPROM_PROFILE(&t, "BL24SA64_write_time");
#else
	pr_debug("Write operation disabled\n");
#endif

	return ret;
}
