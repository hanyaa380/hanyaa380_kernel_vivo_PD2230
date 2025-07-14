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
#include "gc02m1bpd2257fmipimono_Sensor.h"

#define PFX "MAIN2[2E2]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define VIVO_OTP_DATA_SIZE 0x14  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
#define VIVO_EEPROM_WRITE_ID 0x6e
#define VIVO_I2C_SPEED 400
//#define VIVO_VENDOR_SUNNY 0x01
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
//#define VIVO_VENDOR_SUNWIN 0x03
#define VIVO_VENDOR_LENS_ID 0x10
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03
#define VIVO_OTP_ADDR_SIZE 0x11

unsigned char vivo_otp_data_gc02m1bpd2257f[VIVO_OTP_DATA_SIZE];
static unsigned int vivo_otp_data_temp[VIVO_OTP_ADDR_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int SN_addr = 0x00;
static unsigned const int Fpc_addr = 0x0D;

static int checksum = 1;
otp_error_code_t GC02M1BPD2257F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern unsigned char  sn_inf_main2_gc02m1bpd2257f[13];
extern unsigned char  module_inf_main2_gc02m1bpd2257f[13];
extern unsigned int   fpc_inf_main2_gc02m1bpd2257f[4];
static char *vendor = "0STQAFWCYDEGH";
static char *month = "0123456789ABC";
static kal_uint8 addrList[VIVO_OTP_ADDR_SIZE] = {
	0x78, 0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8,
};

struct CAM_MODULE_INFO {
    char MODULE_ID;
    char MODULE_INFO[13];
};

static struct CAM_MODULE_INFO module_info[]={
	{'S', "Sunny"},
	{'T', "Truly"},
	{'Q', "Qtech"},
	{'A', "Semco"},
	{'F', "Ofilm"},
	{'W', "Sunwin"},
	{'C', "Shinephotics"},
	{'Y', "Qtech"},
	{'D', "Sunny"},
	{'E', "Sunwin"},
	{'G', "Ofilm"},
	{'H', "Shinephotics"}
};

unsigned char char_to_hex_gc02m1bpd2257f(unsigned char c)
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

static void vivo_read_eeprom(kal_uint16 addr,  unsigned int *data)
{
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x17, addr);
	write_cmos_sensor(0xf3, 0x34);
	*data = read_cmos_sensor(0x19);
}
extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN2_2e2_otp_read(void)
{
	int i = 0;
	int check_if_group_valid = 0;
	char flag_of_module;
	int module_list_length;
	GC02M1BPD2257F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	#if 1
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip gc02m1bpd2257fmipimono_otp_read\n");
		return 1;
	}
	#endif	

	write_cmos_sensor(0xf3, 0x30);
	for (i = 0; i < VIVO_OTP_ADDR_SIZE; i++) {
		vivo_read_eeprom(addrList[i],  &vivo_otp_data_temp[i]);
		//LOG_INF("read_vivo_eeprom temp addrList:0x%0x Data[0x%0x]:0x%x\n",addrList[i], i, vivo_otp_data_temp[i]);
	}

	if ((vivo_otp_data_temp[0x00] & 0x03) == 0x01)
		check_if_group_valid = 0x01;
	else if ((vivo_otp_data_temp[0x00] & 0x03) == 0x03)
		check_if_group_valid = 0x02;

	if (check_if_group_valid != 0x00) {
		if ((vivo_otp_data_temp[0x00] & 0x0C) == 0x04)
			vivo_otp_data_gc02m1bpd2257f[0x00] = 0x35;
		else if ((vivo_otp_data_temp[0x00] & 0x0C) == 0x08)
			vivo_otp_data_gc02m1bpd2257f[0x00] = 0x36;
		else
			vivo_otp_data_gc02m1bpd2257f[0x00] = vivo_otp_data_temp[0x00];

		vivo_otp_data_gc02m1bpd2257f[0x01] = char_to_hex_gc02m1bpd2257f(vendor[vivo_otp_data_temp[0x00] >> 4]);
		vivo_otp_data_gc02m1bpd2257f[0x02] = vivo_otp_data_temp[0x01];
		vivo_otp_data_gc02m1bpd2257f[0x03] = vivo_otp_data_temp[0x02];
		vivo_otp_data_gc02m1bpd2257f[0x04] = (vivo_otp_data_temp[0x03] >> 4) + 0x30;
		vivo_otp_data_gc02m1bpd2257f[0x05] = char_to_hex_gc02m1bpd2257f(month[vivo_otp_data_temp[0x03] & 0x0F]);
		vivo_otp_data_gc02m1bpd2257f[0x06] = vivo_otp_data_temp[0x04];
		vivo_otp_data_gc02m1bpd2257f[0x07] = vivo_otp_data_temp[0x05];
		vivo_otp_data_gc02m1bpd2257f[0x08] = vivo_otp_data_temp[0x06];
		vivo_otp_data_gc02m1bpd2257f[0x09] = vivo_otp_data_temp[0x07];
		vivo_otp_data_gc02m1bpd2257f[0x0A] = vivo_otp_data_temp[0x08];
		vivo_otp_data_gc02m1bpd2257f[0x0B] = vivo_otp_data_temp[0x09];

		if (check_if_group_valid == 0x01) {
			vivo_otp_data_gc02m1bpd2257f[0x0C] = vivo_otp_data_temp[0x0A];
			vivo_otp_data_gc02m1bpd2257f[0x0D] = vivo_otp_data_temp[0x0B];
			vivo_otp_data_gc02m1bpd2257f[0x0E] = vivo_otp_data_temp[0x0C];
		} else if (check_if_group_valid == 0x02) {
			vivo_otp_data_gc02m1bpd2257f[0x0C] = vivo_otp_data_temp[0x0D];
			vivo_otp_data_gc02m1bpd2257f[0x0D] = vivo_otp_data_temp[0x0E];
			vivo_otp_data_gc02m1bpd2257f[0x0E] = vivo_otp_data_temp[0x0F];
		}

		vivo_otp_data_gc02m1bpd2257f[0x0F] = vivo_otp_data_temp[0x10];
		vivo_otp_data_gc02m1bpd2257f[0x10] = 0xFF;
		vivo_otp_data_gc02m1bpd2257f[0x11] = 0x00;
		vivo_otp_data_gc02m1bpd2257f[0x12] = 0x0B;
		vivo_otp_data_gc02m1bpd2257f[0x13] = 0x01;

		/****sn info start****/
		sn_inf_main2_gc02m1bpd2257f[0] = 0x01;
		for (i = 1; i <= 12; i++) {
			sn_inf_main2_gc02m1bpd2257f[i] = (MUINT32)vivo_otp_data_temp[i];
			//LOG_INF("sn_inf_main2_gc02m1bpd2257f[%d] = 0x%x, vivo_otp_data_temp[0x%x] = 0x%x\n", i, sn_inf_main2_gc02m1bpd2257f[i],  i, vivo_otp_data_temp[i]);
		}
		/****sn info end****/

		/****module info start****/
		flag_of_module = sn_inf_main2_gc02m1bpd2257f[2];
		LOG_INF("flag_of_module = %c", flag_of_module);
		module_list_length = sizeof(module_info)/sizeof(module_info[0]);
		for ( i = 0 ; i < module_list_length; i++){
			if(flag_of_module == module_info[i].MODULE_ID){
				strcpy(module_inf_main2_gc02m1bpd2257f, module_info[i].MODULE_INFO);
				break;
			}
		}
		if(i == module_list_length){
			strcpy(module_inf_main2_gc02m1bpd2257f, "NULL");
		}
		/****module info end****/

		/*fpc info start*/
		for (i = 0 ; i < 4; i++){
			fpc_inf_main2_gc02m1bpd2257f[i] = vivo_otp_data_temp[i + Fpc_addr];
		}
		/*fpc info end*/
		GC02M1BPD2257F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		checksum = 0;
		GC02M1BPD2257F_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!! otp_error_code:%d\n", GC02M1BPD2257F_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
