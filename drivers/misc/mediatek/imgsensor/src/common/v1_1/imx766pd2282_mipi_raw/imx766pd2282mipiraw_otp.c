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
#include "imx766pd2282mipiraw_Sensor.h"

#define PFX "MAIN[0x766]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x4F5C  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
//#define AF_RANGE_GOT  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA0

#define VIVO_I2C_SPEED 1000
#define VIVO_MAX_OFFSET 0x4F5B
//#define VIVO_VENDOR_SUNNY 0x01
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
#define VIVO_VENDOR_LENS_ID 0x07
#define VIVO_VENDOR_VCM_ID 0x07
#define VIVO_VENDOR_DRIVERIC_ID 0x03
#define VIVO_VENDOR_PLATFORM_ID 0x03

unsigned char vivo_otp_data_imx766pd2282[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int Fulse_id_addr = 0x0020;
static unsigned const int Fulse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
static unsigned const int Awb_addr = 0x008C;
static unsigned const int Awb_checksum_addr = 0x00C1;
static unsigned const int Lsc_addr = 0x1FDC;
static unsigned const int Lsc_checksum_addr = 0x2739;
static unsigned const int Af_addr = 0x0066;
static unsigned const int Af_checksum_addr = 0x008B;
static unsigned const int PDAF_proc1_hvbin_addr = 0x273A;
static unsigned const int PDAF_proc1_hvbin_checksum_addr = 0x2B19;
static unsigned const int PDAF_proc2_hvbin_addr = 0x2B1A;
static unsigned const int PDAF_proc2_hvbin_checksum_addr = 0x3179;
static unsigned const int PDAF_proc1_vbin_addr = 0x317A;
static unsigned const int PDAF_proc1_vbin_checksum_addr = 0x3379;
static unsigned const int PDAF_proc2_vbin_addr = 0x337A;
static unsigned const int PDAF_proc2_vbin_checksum_addr = 0x3779;
static unsigned const int PDAF_proc1_full_remosaic_addr = 0x377A;
static unsigned const int PDAF_proc1_full_remosaic_checksum_addr = 0x3B59;
static unsigned const int PDAF_proc2_full_remosaic_addr = 0x3B5A;
static unsigned const int PDAF_proc2_full_remosaic_checksum_addr = 0x41B9;
static unsigned const int QSC_addr = 0x435A;
static unsigned const int QSC_checksum_addr = 0x4F69;

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  moduleid_inf_main_imx766pd2282[2];
extern MUINT32  sn_inf_main_imx766pd2282[13];
extern MUINT32  material_inf_main_imx766pd2282[4];
extern MUINT32  af_calib_inf_main_imx766pd2282[6];


static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{
	int i=0,num=0;
	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
	if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	while(i*4096<VIVO_OTP_DATA_SIZE){
		if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)(data+i*4096),  4096,  VIVO_EEPROM_WRITE_ID) < 0) {
			return false;
		}
		i++;
		num+=4096;
		pu_send_cmd[0]=(char)(num >> 8);
		pu_send_cmd[1]=(char)(num & 0xFF);
	}

	if(num<VIVO_OTP_DATA_SIZE){
		if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)(data+i*4096),  VIVO_OTP_DATA_SIZE-num,  VIVO_EEPROM_WRITE_ID) < 0) {
			return false;
		}
	}
   return true;
}

//extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN_imx766pd2282_otp_read(void)
{
	int i = 0;
	int offset = (ModuleInfo_addr);
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	#if 0
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip imx766pd2282_otp_read\n");
		return 1;
	}
	#endif
	/* guojunzheng add end */

	if (!vivo_read_eeprom(offset,  vivo_otp_data_imx766pd2282)) {
			LOG_INF("[lxd++]read_vivo_eeprom 0x%0x  fail \n", offset);
			return 0;
	}

	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[lxd++] read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_imx766pd2282[i]);
	}
	#endif

	if ((0x01 == vivo_otp_data_imx766pd2282[ModuleInfo_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[Fulse_id_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[SN_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[Awb_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[Lsc_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[Af_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc1_full_remosaic_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc1_hvbin_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc1_vbin_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc2_full_remosaic_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc2_hvbin_addr]) &&
		(0x01 == vivo_otp_data_imx766pd2282[PDAF_proc2_vbin_addr]) ) {
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
		/*
		if ((VIVO_VENDOR_SUNNY != vivo_otp_data_imx766pd2282[0x01]) ) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]Module ID error!!!    otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_VENDOR_LENS_ID != vivo_otp_data_imx766pd2282[0x08]) ||
				(VIVO_VENDOR_VCM_ID != vivo_otp_data_imx766pd2282[0x09]) ||
				(VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_imx766pd2282[0x0A]) ||
				(VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_imx766pd2282[0x02])) {

			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		} else if ((0xff != vivo_otp_data_imx766pd2282[0x0B]) ||
				(0x00 != vivo_otp_data_imx766pd2282[0x0C]) ||
				(0x0b != vivo_otp_data_imx766pd2282[0x0D]) ||
				((0x01 != vivo_otp_data_imx766pd2282[0x0E]) && (0x03 != vivo_otp_data_imx766pd2282[0x0E]) && (0x02 != vivo_otp_data_imx766pd2282[0x0E]))) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n",
				vivo_otp_data_imx766pd2282[0x0B], vivo_otp_data_imx766pd2282[0x0C], vivo_otp_data_imx766pd2282[0x0D], vivo_otp_data_imx766pd2282[0x0E]);
			return 0;
		}
		*/
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[ModuleInfo_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****moduleid info start****/
		moduleid_inf_main_imx766pd2282[0] = (MUINT32)vivo_otp_data_imx766pd2282[0x01];
		LOG_INF("moduleid_inf_main_imx766pd2282 0x%x\n", moduleid_inf_main_imx766pd2282[0]);
		/****moduleid info start****/

		/****material info start****/
			material_inf_main_imx766pd2282[0] = (MUINT32)vivo_otp_data_imx766pd2282[0x0F];
			material_inf_main_imx766pd2282[1] = (MUINT32)vivo_otp_data_imx766pd2282[0x10];
			material_inf_main_imx766pd2282[2] = (MUINT32)vivo_otp_data_imx766pd2282[0x11];
			material_inf_main_imx766pd2282[3] = (MUINT32)vivo_otp_data_imx766pd2282[0x12];
			LOG_INF("material_inf_main_imx766pd2282[0] = 0x%x, material_inf_main_imx766pd2282[1] = 0x%x, material_inf_main_imx766pd2282[2] = 0x%x,material_inf_main_imx766pd2282[3] = 0x%x\n",
				material_inf_main_imx766pd2282[0]  , material_inf_main_imx766pd2282[1],  material_inf_main_imx766pd2282[2], material_inf_main_imx766pd2282[3]);
		/****material info end****/

		/****Fulse id_checksum****/
		checksum = 0;
		for (i = Fulse_id_addr+1; i < Fulse_id_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[Fulse_id_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}

		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data_imx766pd2282[0x%x] = 0x%x\n", i, vivo_otp_data_imx766pd2282[i]);*/
			checksum += vivo_otp_data_imx766pd2282[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[SN_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_main_imx766pd2282[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main_imx766pd2282[i+1] = (MUINT32)vivo_otp_data_imx766pd2282[i + SN_addr+1];
			/*LOG_INF("sn_inf_main_imx766pd2282[%d] = 0x%x, vivo_otp_data_imx766pd2282[0x%x] = 0x%x\n", i+1  , sn_inf_main_imx766pd2282[i+1],  i +SN_addr+1, vivo_otp_data_imx766pd2282[i+SN_addr+1]);*/
		}

		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[Awb_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AWB_checksum error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}


		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}

		checksum = checksum % 0xff + 1;
		if (vivo_otp_data_imx766pd2282[Lsc_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]LSC_checksum error!!! checksum:%d otpdata:%d otp_error_code:%d\n", 
							checksum, vivo_otp_data_imx766pd2282[Lsc_checksum_addr], IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}

		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[Af_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AF_checksum error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}
			af_calib_inf_main_imx766pd2282[0] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 7];
			af_calib_inf_main_imx766pd2282[1] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 8];
			af_calib_inf_main_imx766pd2282[2] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 9];
			af_calib_inf_main_imx766pd2282[3] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 10];
			af_calib_inf_main_imx766pd2282[4] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 11];
			af_calib_inf_main_imx766pd2282[5] = (MUINT32)vivo_otp_data_imx766pd2282[Af_addr + 12];
			LOG_INF("af1 = %d, af2= %d af3= %d af4= %d af5= %d af6= %d\n",af_calib_inf_main_imx766pd2282[0],af_calib_inf_main_imx766pd2282[1],af_calib_inf_main_imx766pd2282[2],af_calib_inf_main_imx766pd2282[3],af_calib_inf_main_imx766pd2282[4],af_calib_inf_main_imx766pd2282[5]);

		/****PDAF Proc1 full remosiac checksum****/
		checksum = 0;
		for (i = PDAF_proc1_full_remosaic_addr+1; i < PDAF_proc1_full_remosaic_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[PDAF_proc1_full_remosaic_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_proc1_full_remosaic_checksum_addr error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF Proc1 hvbin checksum****/
		if (0x01 == vivo_otp_data_imx766pd2282[PDAF_proc1_hvbin_addr]){
			checksum = 0;
			for (i = PDAF_proc1_hvbin_addr+1; i < PDAF_proc1_hvbin_checksum_addr; i++) {
				checksum += vivo_otp_data_imx766pd2282[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_imx766pd2282[PDAF_proc1_hvbin_checksum_addr] != checksum) {
				IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[lxd++]PDAF_proc1_hvbin_checksum_addr error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
				return 0;
				}
		}

		/****PDAF proc1 vbin checksum****/
		checksum = 0;
		for (i = PDAF_proc1_vbin_addr+1; i < PDAF_proc1_vbin_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[PDAF_proc1_vbin_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_proc1_vbin_checksum_addr  error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF proc2 full remosiac checksum****/
		checksum = 0;
		for (i = PDAF_proc2_full_remosaic_addr+1; i < PDAF_proc2_full_remosaic_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[PDAF_proc2_full_remosaic_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_proc2_full_remosaic_checksum_addr  error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF pro2 hvbin checksum****/
		checksum = 0;
		for (i = PDAF_proc2_hvbin_addr+1; i < PDAF_proc2_hvbin_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[PDAF_proc2_hvbin_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_proc2_hvbin_checksum_addr  error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF proc2 vbin checksum****/
		checksum = 0;
		for (i = PDAF_proc2_vbin_addr+1; i < PDAF_proc2_vbin_checksum_addr; i++) {
			checksum += vivo_otp_data_imx766pd2282[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_imx766pd2282[PDAF_proc2_vbin_checksum_addr] != checksum) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDAF_proc2_vbin_checksum_addr  error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
			return 0;
			}

		/****check if awb out of range[5000K high  cct]****/
		R_unit = vivo_otp_data_imx766pd2282[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+2]);
		B_unit = vivo_otp_data_imx766pd2282[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+4]);
		G_unit = vivo_otp_data_imx766pd2282[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+6]);

		R_golden = vivo_otp_data_imx766pd2282[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+8]);
		B_golden = vivo_otp_data_imx766pd2282[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+10]);
		G_golden = vivo_otp_data_imx766pd2282[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("lxd_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[lxd++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}


		/****check if awb out of range[3000K low cct]****/
		R_unit_low = vivo_otp_data_imx766pd2282[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+14]);
		B_unit_low = vivo_otp_data_imx766pd2282[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+16]);
		G_unit_low = vivo_otp_data_imx766pd2282[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+18]);

		R_golden_low = vivo_otp_data_imx766pd2282[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+20]);
		B_golden_low = vivo_otp_data_imx766pd2282[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+22]);
		G_golden_low = vivo_otp_data_imx766pd2282[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_imx766pd2282[Awb_addr+24]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (10% * 1024^2)****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 10485;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, IMX766PD2282_OTP_ERROR_CODE);
			return 0;
		}


		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_imx766pd2282[Af_addr + 9];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_imx766pd2282[Af_addr + 10]);

		AF_mac = vivo_otp_data_imx766pd2282[Af_addr + 11];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_imx766pd2282[Af_addr + 12]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
        /* diff_inf_macro: SUNNY 340±140, QTECH 340±140 */
		if (diff_inf_macro > 480 || diff_inf_macro < 200) {  /*AF code out of range*/
			IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("[lxd++]AF out of range error!!!   otp_error_code:%d    Module ID = 0x%0x   inf = %d    mac = %d\n", IMX766PD2282_OTP_ERROR_CODE, vivo_otp_data_imx766pd2282[0x01], AF_inf, AF_mac);
			return 0;
		}
		#endif

		/*lxd add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("[lxd++]read_IMX766PD2282_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_imx766pd2282[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_imx766pd2282[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_imx766pd2282[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*lxd add for pdaf data end*/
		IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		return 1;
	} else {
		IMX766PD2282_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("ModuleInfo_addr = 0x%x, Fulse_id_addr = 0x%x, SN_addr= 0x%x, Awb_addr = 0x%x, Lsc_addr = 0x%x, Af_addr = 0x%x, \
		PDAF_proc1_full_remosaic_addr = 0x%x, PDAF_proc1_hvbin_addr = 0x%x, PDAF_proc1_vbin_addr = 0x%x, PDAF_proc2_full_remosaic_addr = 0x%x, PDAF_proc2_hvbin_addr = 0x%x, PDAF_proc2_vbin_addr = 0x%x\n",
		vivo_otp_data_imx766pd2282[ModuleInfo_addr],
		vivo_otp_data_imx766pd2282[Fulse_id_addr],
		vivo_otp_data_imx766pd2282[SN_addr],
		vivo_otp_data_imx766pd2282[Awb_addr],
		vivo_otp_data_imx766pd2282[Lsc_addr],
		vivo_otp_data_imx766pd2282[Af_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc1_full_remosaic_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc1_hvbin_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc1_vbin_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc2_full_remosaic_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc2_hvbin_addr],
		vivo_otp_data_imx766pd2282[PDAF_proc2_vbin_addr]);
		LOG_INF("[lxd++]invalid otp data. error!!!   otp_error_code:%d\n", IMX766PD2282_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
