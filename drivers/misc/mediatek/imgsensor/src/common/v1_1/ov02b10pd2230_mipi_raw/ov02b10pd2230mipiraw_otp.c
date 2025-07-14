#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "../imgsensor_i2c.h"
#include "ov02b10pd2230mipiraw_Sensor.h"

#define PFX "MAIN2[ov02b10]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define VIVO_OTP_DATA_SIZE 0x13  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
#define VIVO_EEPROM_WRITE_ID 0x78
#define VIVO_I2C_SPEED 400
#define VIVO_VENDOR_LENS_ID 0x10
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03
#define VIVO_OTP_ADDR_SIZE 0x10

unsigned char vivo_otp_data_ov02b10pd2230[VIVO_OTP_DATA_SIZE];
static unsigned char vivo_otp_data_temp[VIVO_OTP_ADDR_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int SN_addr = 0x00;
static unsigned const int Awb_addr = 0x0C;

#define GOLDEN_RGAIN  743   // R/Gr
#define GOLDEN_GGAIN  1030  // Gb/Gr
#define GOLDEN_BGAIN  766   // B/Gr

static int checksum = 1;
otp_error_code_t OV02B10PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main2_ov02b10pd2230[13];
extern MUINT32  material_inf_main2_ov02b10pd2230[4];
extern MUINT32  module_inf_main2_ov02b10pd2230[2];
static char *vendor = "0STQAFWCYDEGH";
static char *month = "0123456789ABC";

static unsigned char char_to_hex(unsigned char c)
{
	unsigned char hex;
	if ((c >= '0') && (c <= '9'))
		hex = 0x30 + (c- '0');
	else if ((c >= 'A') && (c <= 'Z'))
		hex = 0x41 + (c- 'A');
	else if ((c >= 'a') && (c <= 'z'))
		hex = 0x61 + (c- 'Z');
	else
		hex = 0xff;
	return hex;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	int ret;
	char pu_send_cmd[2] = { (char)(addr & 0xff), (char)(para & 0xff) };

	ret = iWriteRegI2C(pu_send_cmd, 2, VIVO_EEPROM_WRITE_ID);
	if (ret)
		LOG_INF("write cmos sensor fail, ret = %d\n", ret);
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	int ret;
	kal_uint16 get_byte = 0;
	char pu_send_cmd[1] = { (char)(addr & 0xff) };

	ret = iReadRegI2C(pu_send_cmd, 1, (u8 *) &get_byte, 1, VIVO_EEPROM_WRITE_ID);
	if (ret)
		LOG_INF("read cmos sensor fail, ret = %d\n", ret);

	return get_byte;
}

#if 1
static void vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	write_cmos_sensor(0xfd, 0x06);
	*data = read_cmos_sensor(addr);
}
#elif
static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[1] = {(char)(addr & 0xFF)};
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  1,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		LOG_INF("vivo_read_eeprom invalid");
		return false;
	}
	return true;
}
#endif

int MAIN2_002B_otp_read(void)
{
	int i = 0, start_addr = 0x10;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	long long t1, t2, t3, t4, t5, t6, t, temp;
	OV02B10PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

	for (i = 0; i < VIVO_OTP_ADDR_SIZE; i++) {
		vivo_read_eeprom(start_addr + i,  &vivo_otp_data_temp[i]);
		LOG_INF("otp data[0x%x] = 0x%x", start_addr + i, vivo_otp_data_temp[i]);
	}

	if ((vivo_otp_data_temp[0x00] & 0x03) == 0x01)
		check_if_group_valid = 0x01;
	else if ((vivo_otp_data_temp[0x00] & 0x03) == 0x03)
		check_if_group_valid = 0x02;

	if (check_if_group_valid != 0x00) {
		if ((vivo_otp_data_temp[0x00] & 0x0C) == 0x04)
			vivo_otp_data_ov02b10pd2230[0x00] = 0x35;
		else if ((vivo_otp_data_temp[0x00] & 0x0C) == 0x08)
			vivo_otp_data_ov02b10pd2230[0x00] = 0x36;
		else
			LOG_INF("vivo otp no info\n");

		vivo_otp_data_ov02b10pd2230[0x01] = char_to_hex(vendor[vivo_otp_data_temp[0x00] >> 4]);
		vivo_otp_data_ov02b10pd2230[0x02] = vivo_otp_data_temp[0x01];
		vivo_otp_data_ov02b10pd2230[0x03] = vivo_otp_data_temp[0x02];
		vivo_otp_data_ov02b10pd2230[0x04] = (vivo_otp_data_temp[0x03] >> 4) + 0x30;
		vivo_otp_data_ov02b10pd2230[0x05] = char_to_hex(month[vivo_otp_data_temp[0x03] & 0x0F]);
		vivo_otp_data_ov02b10pd2230[0x06] = vivo_otp_data_temp[0x04];
		vivo_otp_data_ov02b10pd2230[0x07] = vivo_otp_data_temp[0x05];
		vivo_otp_data_ov02b10pd2230[0x08] = vivo_otp_data_temp[0x06];
		vivo_otp_data_ov02b10pd2230[0x09] = vivo_otp_data_temp[0x07];
		vivo_otp_data_ov02b10pd2230[0x0A] = vivo_otp_data_temp[0x08];
		vivo_otp_data_ov02b10pd2230[0x0B] = vivo_otp_data_temp[0x09];

		if (check_if_group_valid == 0x01) {
			vivo_otp_data_ov02b10pd2230[0x0C] = vivo_otp_data_temp[0x0A];
			vivo_otp_data_ov02b10pd2230[0x0D] = vivo_otp_data_temp[0x0B];
			vivo_otp_data_ov02b10pd2230[0x0E] = vivo_otp_data_temp[0x0C];
		} else if (check_if_group_valid == 0x02) {
			vivo_otp_data_ov02b10pd2230[0x0C] = vivo_otp_data_temp[0x0D];
			vivo_otp_data_ov02b10pd2230[0x0D] = vivo_otp_data_temp[0x0E];
			vivo_otp_data_ov02b10pd2230[0x0E] = vivo_otp_data_temp[0x0F];
		}

		vivo_otp_data_ov02b10pd2230[0x0F] = 0xFF;
		vivo_otp_data_ov02b10pd2230[0x10] = 0x00;
		vivo_otp_data_ov02b10pd2230[0x11] = 0x0B;
		vivo_otp_data_ov02b10pd2230[0x12] = 0x01;

		/****module info start****/
		module_inf_main2_ov02b10pd2230[0] = vivo_otp_data_ov02b10pd2230[0x01];
		/****module info end****/

		/****material info start****/
		material_inf_main2_ov02b10pd2230[0] = 0x02;
		material_inf_main2_ov02b10pd2230[1] = 0x37;
		material_inf_main2_ov02b10pd2230[2] = 0x15;
		material_inf_main2_ov02b10pd2230[3] = 0x17;
		/*LOG_INF("material_inf_main2_ov02b10pd2230[0] = 0x%x, material_inf_main2_ov02b10pd2230[1] = 0x%x, material_inf_main2_ov02b10pd2230[2] = 0x%x, material_inf_main2_ov02b10pd2230[2] = 0x%x\n",
			material_inf_main2_ov02b10pd2230[0], material_inf_main2_ov02b10pd2230[1], material_inf_main2_ov02b10pd2230[2], material_inf_main2_ov02b10pd2230[3]);*/
		/****material info end****/

		/****sn info start****/
		sn_inf_main2_ov02b10pd2230[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main2_ov02b10pd2230[i+1] = (MUINT32)vivo_otp_data_ov02b10pd2230[i + SN_addr];
			//LOG_INF("sn_inf_main2_ov02b10pd2230[%d] = 0x%x, vivo_otp_data_ov02b10pd2230[0x%x] = 0x%x\n", i+1  , sn_inf_main2_ov02b10pd2230[i+1],  i +SN_addr+1, vivo_otp_data_ov02b10pd2230[i+SN_addr+1]);
		}
		/****sn info end****/

		/****check if awb out of range[5000K high  cct]****/
		R_unit = ((vivo_otp_data_ov02b10pd2230[Awb_addr] & 0xF0) << 4) | (vivo_otp_data_ov02b10pd2230[Awb_addr+2]);
		B_unit = ((vivo_otp_data_ov02b10pd2230[Awb_addr] & 0x0F) << 8) | (vivo_otp_data_ov02b10pd2230[Awb_addr+2]);
		G_unit = 1024;
		R_golden = GOLDEN_RGAIN;
		B_golden = GOLDEN_BGAIN;
		G_golden = GOLDEN_GGAIN;

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (15% * 1024)^2****/
		LOG_INF("lxd_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 23592;//10485
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			OV02B10PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, OV02B10PD2230_OTP_ERROR_CODE);
			return 0;
		}
		OV02B10PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		checksum = 0;
		OV02B10PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!! otp_error_code:%d\n", OV02B10PD2230_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
