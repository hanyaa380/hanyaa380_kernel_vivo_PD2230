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
#include "s5k5e9pd2230mipiraw_Sensor.h"

#define PFX "S5K5E9PD2230_EEPROM_OTP"
#define LOG_INF(format,  args...) pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

extern void kdSetI2CSpeed(u16 i2cSpeed);

#define VIVO_OTP_DATA_SIZE 0x143F
#define VIVO_EEPROM_WRITE_ID 0xA2
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x3FFF
#define VIVO_VENDOR_SUNNY 0x01
#define VIVO_VENDOR_TRULY 0x02
#define VIVO_VENDOR_QTECH 0x05
#define VIVO_VENDOR_OFILM 0x09
#define VIVO_VENDOR_LENS_ID 0x11
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03

static unsigned char vivo_otp_data[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x00;
static unsigned const int ModuleInfo_checksum_addr = 0x001F;
static unsigned const int FuseIDInfo_addr = 0x0020;
static unsigned const int FuseIDInfo_checksum_addr = 0x0044;
static unsigned const int SNInfo_addr = 0x0045;
static unsigned const int SNInfo_checksum_addr = 0x0065;

static unsigned const int Awb_addr = 0x0CAB;
static unsigned const int Awb_checksum_addr = 0x0CE0;
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143E;


static int checksum = 0;
otp_error_code_t S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
unsigned  int S5K5E9PD2230_flag;
extern MUINT32  sn_inf_sub_s5k5e9pd2230[13];  /*0 flag   1-12 data*/
extern MUINT32  material_inf_sub_s5k5e9pd2230[4];  /*0 flag   1-12 data*/
extern MUINT32  module_inf_sub_s5k5e9pd2230[2];
static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd, 2, (u8 *)data, 1, VIVO_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}
int S5K5E9PD2230_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, R_golden = 0, B_golden = 0, G_unit = 0, G_golden = 0;

	long long t1,t2,t3,t4,t5,t6,t,temp;
	S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if(!vivo_read_eeprom(offset, &vivo_otp_data[i])){
			LOG_INF("read_vivo_eeprom 0x%0x %d fail\n", offset, vivo_otp_data[i]);
			return 0;
		}
		/*LOG_INF("read_vivo_eeprom vivo_otp_data[0x%0x] =0x%x\n", offset, vivo_otp_data[i]);*/
		offset++;
	}
	#if 1
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", i, vivo_otp_data[i]);
	}
	#endif
	//check_if_group_valid = vivo_otp_data[ModuleInfo_addr] | vivo_otp_data[Awb_addr] | vivo_otp_data[Af_addr] | vivo_otp_data[Lsc_addr] | vivo_otp_data[PD_Proc1_addr] | vivo_otp_data[PD_Proc2_addr];
	LOG_INF("Year-Month-Day:%d-%d-%d",vivo_otp_data[0x03],vivo_otp_data[0x04],vivo_otp_data[0x05]);
	LOG_INF("ModuleInfo_flag = 0x%x,FuseIDInfo_flag = 0x%x,SNInfo_flag = 0x%x,Awb_flag = 0x%x,Lsc_flag = 0x%x",vivo_otp_data[ModuleInfo_addr],vivo_otp_data[FuseIDInfo_addr],vivo_otp_data[SNInfo_addr],vivo_otp_data[Awb_addr],vivo_otp_data[Lsc_addr]);
	if ((0x01 == vivo_otp_data[ModuleInfo_addr]) && (0x01 == vivo_otp_data[FuseIDInfo_addr]) && (0x01 == vivo_otp_data[SNInfo_addr]) && (0x01 == vivo_otp_data[Awb_addr]) && (0x01 == vivo_otp_data[Lsc_addr])) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid. check_if_group_valid:%d\n", check_if_group_valid);
	}
/*
	if (vivo_otp_data[0x09] != 0x00 ){ //0x09 vcm id check
		S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
		S5K5E9PD2230_flag = 0;
		LOG_INF("VCM ID Error!!! S5K5E9PD2230_flag =%d\n", S5K5E9PD2230_flag);
	}else{
		S5K5E9PD2230_flag = 1;
		LOG_INF("S5K5E9PD2230_flag = %d,VCM ID = 0x%x", S5K5E9PD2230_flag,vivo_otp_data[0x09]);
	}
*/
	if (check_if_group_valid == 0x01) {//all the data is valid
/*
		// ModuleInfo
		if (VIVO_VENDOR_QTECH != vivo_otp_data[0x01]) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("ModuleInfo_info error!!! otp_error_code:%d\n", S5K5E9PD2230_OTP_ERROR_CODE);
			return 0;
		} else if ((vivo_otp_data[0x02] != VIVO_VENDOR_PLATFORM_ID) ||(vivo_otp_data[0x08] != VIVO_VENDOR_LENS_ID) ||(vivo_otp_data[0x07] != 0x9B) || (vivo_otp_data[0x06] != 0x55)) {	
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("Platform ID or Sensor or Lens ID Error!!! otp_error_code:%d\n", S5K5E9PD2230_OTP_ERROR_CODE);
			return 0;
		} else if((0xff != vivo_otp_data[0x0B]) || (0x00 != vivo_otp_data[0x0C]) || (0x0b != vivo_otp_data[0x0D]) || (0x01 != vivo_otp_data[0x0E])) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("calibration version Error!!! Read version:0x%2x%2x%2x%2x\n", vivo_otp_data[0x0a],vivo_otp_data[0x0b], vivo_otp_data[0x0c],vivo_otp_data[0x0d]);
			return 0;
		}
*/
		LOG_INF("Read Calibration version:0x%2x%2x%2x%2x\n", vivo_otp_data[0x0B], vivo_otp_data[0x0C], vivo_otp_data[0x0D], vivo_otp_data[0x0E]);
		LOG_INF("Read Code:0x%2x%2x%2x%2x\n", vivo_otp_data[0x0F], vivo_otp_data[0x10], vivo_otp_data[0x11], vivo_otp_data[0x12]);
		LOG_INF("Platform ID = 0x%x, Lens ID = 0x%x, Sensor ID H = 0x%x,Sensor ID L = 0x%x\n", vivo_otp_data[0x02],vivo_otp_data[0x08],vivo_otp_data[0x06],vivo_otp_data[0x07]);
		//ModuleInfo_checksum
		checksum = 0;
        for(i = ModuleInfo_addr + 1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
        }
		checksum = checksum % 0xff + 1;
		if (vivo_otp_data[ModuleInfo_checksum_addr] != checksum) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
            LOG_INF("ModuleInfo_checksum error!!! otp_error_code:%d\n", S5K5E9PD2230_OTP_ERROR_CODE);
			return 0;
        }

        /****module info start****/
        module_inf_sub_s5k5e9pd2230[0] = vivo_otp_data[0x01];
        /****module info end****/

		/****material info start****/
		material_inf_sub_s5k5e9pd2230[0] = (MUINT32)vivo_otp_data[0x0F];
		material_inf_sub_s5k5e9pd2230[1] = (MUINT32)vivo_otp_data[0x10];
		material_inf_sub_s5k5e9pd2230[2] = (MUINT32)vivo_otp_data[0x11];
		material_inf_sub_s5k5e9pd2230[3] = (MUINT32)vivo_otp_data[0x12];
		LOG_INF("material_inf_sub_s5k5e9pd2230[0] = 0x%x, material_inf_sub_s5k5e9pd2230[1] = 0x%x, material_inf_sub_s5k5e9pd2230[2] = 0x%x, material_inf_sub_s5k5e9pd2230[3] = 0x%x\n",
			material_inf_sub_s5k5e9pd2230[0], material_inf_sub_s5k5e9pd2230[1],  material_inf_sub_s5k5e9pd2230[2], material_inf_sub_s5k5e9pd2230[3]);
		/****material info end****/
		//fuseIDInfo_checksum
		checksum = 0;
        for(i = FuseIDInfo_addr + 1; i < FuseIDInfo_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
        }
		checksum = checksum % 0xff + 1;
		if(vivo_otp_data[FuseIDInfo_checksum_addr] != checksum) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			return 0;
        }
		//SNInfo_addr_checksum
		checksum = 0;
        for(i = SNInfo_addr + 1; i < SNInfo_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
        }
		checksum = checksum % 0xff + 1;
		if (vivo_otp_data[SNInfo_checksum_addr] != checksum) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			return 0;
        }
		sn_inf_sub_s5k5e9pd2230[0] = 0x01;
		for(i = 0; i < 12; i++) {
			sn_inf_sub_s5k5e9pd2230[i+1] = (MUINT32)vivo_otp_data[i + SNInfo_addr+1];
			LOG_INF("sn_inf_sub_s5k5e9pd2230[%d] = 0x%x, vivo_otp_data[0x%x] = 0x%x\n", i + 1, sn_inf_sub_s5k5e9pd2230[i+1], i + SNInfo_addr+1, vivo_otp_data[i + SNInfo_addr+1]);
		}
		//AWB_checksum
		checksum = 0;
        for(i = Awb_addr + 1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
        }
		checksum = checksum % 0xff + 1;
		if (vivo_otp_data[Awb_checksum_addr] != checksum) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			return 0;
        }
		//LSC_checksum
		checksum = 0;
        for(i = Lsc_addr + 1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data[i];
        }
		checksum = checksum % 0xff + 1;
		if (vivo_otp_data[Lsc_checksum_addr] != checksum) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			return 0;
		}

		/****check if awb out of range[high cct]****/
		R_unit = vivo_otp_data[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data[Awb_addr+2]);
		B_unit = vivo_otp_data[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data[Awb_addr+4]);
		G_unit = vivo_otp_data[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data[Awb_addr+6]);

		R_golden = vivo_otp_data[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data[Awb_addr+8]);
		B_golden = vivo_otp_data[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data[Awb_addr+10]);
		G_golden = vivo_otp_data[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("add:t1 = %lld ,t2 = %lld ,t3 = %lld ,t4 = %lld ,t5 = %lld ,t6 = %lld ,temp = %lld ,t = %lld\n",t1,t2,t3,t4,t5,t6,temp,t);
		if(t > 0) {
			S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5K5E9PD2230_OTP_ERROR_CODE);
			return 0;
		}
		return 1;
	} else {
		S5K5E9PD2230_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!!   otp_error_code:%d\n", S5K5E9PD2230_OTP_ERROR_CODE);
		return 0;
	}
}
