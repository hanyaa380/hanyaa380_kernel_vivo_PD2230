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
#include "s5kjn1sqpd2279fmipiraw_Sensor.h"

#define PFX "MAIN[38E1]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x4000  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
//#define AF_RANGE_GOT  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA0
//#define OIS_WRITE_ID 0xE4
#define VIVO_I2C_SPEED 1000
#define VIVO_MAX_OFFSET 0x3FFF
#define VIVO_VENDOR_SUNNY 0x01
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
#define VIVO_VENDOR_PLATFORM_ID 0x03
#define VIVO_VENDOR_LENS_ID 0x07
#define VIVO_VENDOR_VCM_ID 0x09
#define VIVO_VENDOR_DRIVERIC_ID 0x05

#define XTC_SIZE 2612
#define XTC_OFFSET 0x1A40
#define SensorXTC_SIZE 768
#define SensorXTC_OFFSET 0x2476
#define PDXTC_SIZE 4000
#define PDXTC_OFFSET 0x2778
#define SWGGC_SIZE 626
#define SWGGC_OFFSET 0x371A
#define HWGGC_SIZE 346
#define HWGGC_OFFSET 0x398E

unsigned char vivo_otp_data_s5kjn1sqpd2279f[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int Fulse_id_addr = 0x0020;
static unsigned const int Fulse_id_checksum_addr = 0x0044;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
static unsigned const int Af_addr = 0x0066;
static unsigned const int Af_checksum_addr = 0x008B;
static unsigned const int Awb_addr = 0x0CAB;
static unsigned const int Awb_checksum_addr = 0x0CE0;
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143E;
static unsigned const int PDAF_proc1_addr = 0x143F;
static unsigned const int PDAF_proc1_checksum_addr = 0x1640;
static unsigned const int PDAF_proc2_addr = 0x1641;
static unsigned const int PDAF_proc2_checksum_addr = 0x1A3E;
static unsigned const int XTC_cal_addr = 0x1A3F;
static unsigned const int XTC_cal_checksum_addr = 0x2474;
static unsigned const int SensorXTC_cal_addr = 0x2475;
static unsigned const int SensorXTC_cal_checksum_addr = 0x2776;
static unsigned const int PDXTC_addr = 0x2777;
static unsigned const int PDXTC_checksum_addr = 0x3718;
static unsigned const int SW_GGC_addr = 0x3719;
static unsigned const int SW_GGC_checksum_addr = 0x398C;
static unsigned const int HWGGC_addr = 0x398D;
static unsigned const int HWGGC_addr_checksum_addr = 0x3FFF;
static unsigned const int HWGGC_addr_checksum_addr_SUNNY = 0x3AE8;

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
MUINT32 sn_inf_main_s5kjn1sqpd2279f[13];
MUINT32 material_inf_main_s5kjn1sqpd2279f[4];
MUINT32 af_calib_inf_main_s5kjn1sqpd2279f[6];
MUINT32 module_inf_main_s5kjn1sqpd2279f[2];
unsigned char crosstalk_data_s5kjn1sqpd2279f[XTC_SIZE+SensorXTC_SIZE+PDXTC_SIZE+SWGGC_SIZE];

static bool vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		pr_err("addr > VIVO_MAX_OFFSET 0x%x > 0x%x \n", addr,  VIVO_MAX_OFFSET);
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		pr_err("iReadRegI2C  failed \n");
		return false;
	}
    return true;
}

extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN_38E1_otp_read(void)
{
	int i = 0;
	int offset = (ModuleInfo_addr /*+ 0x5000*/);
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;
	int crosstalk_offset = 0;


	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	#if 1
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip s5kjn1sqpd2279f_otp_read\n");
		return 1;
	}
	#endif
	/* guojunzheng add end */
	//Main_7309_otp_read_setting_init_continuous_read(offset);
	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset, &vivo_otp_data_s5kjn1sqpd2279f[i])) {
			LOG_INF("[lxd++]read_vivo_eeprom 0x%0x %d fail \n", offset, vivo_otp_data_s5kjn1sqpd2279f[i]);
			return 0;
		}
		offset++;
	}
	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_s5kjn1sqpd2279f[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data_s5kjn1sqpd2279f[ModuleInfo_addr] | vivo_otp_data_s5kjn1sqpd2279f[Awb_addr] | vivo_otp_data_s5kjn1sqpd2279f[Af_addr] | vivo_otp_data_s5kjn1sqpd2279f[Lsc_addr] | vivo_otp_data_s5kjn1sqpd2279f[PD_Proc1_addr] | vivo_otp_data_s5kjn1sqpd2279f[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data_s5kjn1sqpd2279f[ModuleInfo_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[Fulse_id_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[SN_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[Af_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[Awb_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[Lsc_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc1_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc2_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[XTC_cal_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[SensorXTC_cal_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[PDXTC_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[SW_GGC_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[HWGGC_addr]) /*&&
		(0x01 == vivo_otp_data_s5kjn1sqpd2279f[OIS_shift_calibration_addr])*/) {

		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
#if 0
		/****ModuleInfo****/
		if ((VIVO_VENDOR_SUNNY != vivo_otp_data_s5kjn1sqpd2279f[0x01]) && (VIVO_VENDOR_QTECH != vivo_otp_data_s5kjn1sqpd2279f[0x01])) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]Module ID error!!!    otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_VENDOR_LENS_ID != vivo_otp_data_s5kjn1sqpd2279f[0x08]) ||
				(VIVO_VENDOR_VCM_ID != vivo_otp_data_s5kjn1sqpd2279f[0x09]) ||
				(VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_s5kjn1sqpd2279f[0x0A]) ||
				(VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_s5kjn1sqpd2279f[0x02])) {

			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		} else if ((0xff != vivo_otp_data_s5kjn1sqpd2279f[0x0B]) ||
				(0x00 != vivo_otp_data_s5kjn1sqpd2279f[0x0C]) ||
				(0x0b != vivo_otp_data_s5kjn1sqpd2279f[0x0D]) ||
				((0x01 != vivo_otp_data_s5kjn1sqpd2279f[0x0E]) && (0x03 != vivo_otp_data_s5kjn1sqpd2279f[0x0E]) && (0x02 != vivo_otp_data_s5kjn1sqpd2279f[0x0E]))) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n",
				vivo_otp_data_s5kjn1sqpd2279f[0x0B], vivo_otp_data_s5kjn1sqpd2279f[0x0C], vivo_otp_data_s5kjn1sqpd2279f[0x0D], vivo_otp_data_s5kjn1sqpd2279f[0x0E]);
			return 0;
		}
		/****ModuleInfo_checksum****/
#endif
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[ModuleInfo_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}

		/****module info start****/
		module_inf_main_s5kjn1sqpd2279f[0] = vivo_otp_data_s5kjn1sqpd2279f[0x01];
		/****module info end****/

		/****material info start****/
			material_inf_main_s5kjn1sqpd2279f[0] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[0x0F];
			material_inf_main_s5kjn1sqpd2279f[1] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[0x10];
			material_inf_main_s5kjn1sqpd2279f[2] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[0x11];
			material_inf_main_s5kjn1sqpd2279f[3] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[0x12];
			LOG_INF("material_inf_main_s5kjn1sqpd2279f[0] = 0x%x, material_inf_main_s5kjn1sqpd2279f[1] = 0x%x, material_inf_main_s5kjn1sqpd2279f[2] = 0x%x,material_inf_main_s5kjn1sqpd2279f[3] = 0x%x\n",
				material_inf_main_s5kjn1sqpd2279f[0]  , material_inf_main_s5kjn1sqpd2279f[1],  material_inf_main_s5kjn1sqpd2279f[2], material_inf_main_s5kjn1sqpd2279f[3]);
		/****material info end****/

		/****Fulse id_checksum****/
		checksum = 0;
		for (i = Fulse_id_addr+1; i < Fulse_id_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[Fulse_id_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}

		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data_s5kjn1sqpd2279f[0x%x] = 0x%x\n", i, vivo_otp_data_s5kjn1sqpd2279f[i]);*/
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[SN_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_main_s5kjn1sqpd2279f[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main_s5kjn1sqpd2279f[i+1] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[i + SN_addr+1];
			/*LOG_INF("sn_inf_main_s5kjn1sqpd2279f[%d] = 0x%x, vivo_otp_data_s5kjn1sqpd2279f[0x%x] = 0x%x\n", i+1  , sn_inf_main_s5kjn1sqpd2279f[i+1],  i +SN_addr+1, vivo_otp_data_s5kjn1sqpd2279f[i+SN_addr+1]);*/
		}


		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[Af_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AF_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}

		af_calib_inf_main_s5kjn1sqpd2279f[0] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 7];
		af_calib_inf_main_s5kjn1sqpd2279f[1] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 8];
		af_calib_inf_main_s5kjn1sqpd2279f[2] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 9];
		af_calib_inf_main_s5kjn1sqpd2279f[3] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 10];
		af_calib_inf_main_s5kjn1sqpd2279f[4] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 11];
		af_calib_inf_main_s5kjn1sqpd2279f[5] = (MUINT32)vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 12];


		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[Awb_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AWB_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}


		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[Lsc_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]LSC_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}

		/****PDAF Proc1_checksum****/
		checksum = 0;
		for (i = PDAF_proc1_addr+1; i < PDAF_proc1_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc1_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF Proc2_checksum****/
		if (0x01 == vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc2_addr]){
			checksum = 0;
			for (i = PDAF_proc2_addr+1; i < PDAF_proc2_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc2_checksum_addr] != checksum) {
				S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
				return 0;
				}
		}


		/****XTC_cal_checksum****/
		checksum = 0;
		for (i = XTC_cal_addr+1; i < XTC_cal_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[XTC_cal_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("XTC_cal_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}
		for (i = 0; i < XTC_SIZE; i++) {
			crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]=vivo_otp_data_s5kjn1sqpd2279f[XTC_OFFSET+i];
			//LOG_INF("XTC[%d] = 0x%x\n", i,  crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]);
		}
		crosstalk_offset += XTC_SIZE;

		/****SensorXTC_cal_checksum****/
		checksum = 0;
		for (i = SensorXTC_cal_addr+1; i < SensorXTC_cal_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[SensorXTC_cal_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("XTC_cal_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}
		for (i = 0; i < SensorXTC_SIZE; i++) {
			crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]=vivo_otp_data_s5kjn1sqpd2279f[SensorXTC_OFFSET+i];
			//LOG_INF("SensorXTC[%d] = 0x%x\n", i,  crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]);
		}
		crosstalk_offset += SensorXTC_SIZE;

		/****PDXTC_checksum****/
		checksum = 0;
		for (i = PDXTC_addr+1; i < PDXTC_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[PDXTC_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDXTC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}
		for (i = 0; i < PDXTC_SIZE; i++) {
			crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]=vivo_otp_data_s5kjn1sqpd2279f[PDXTC_OFFSET+i];
			//LOG_INF("PDXTC[%d] = 0x%x\n", i,  crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]);
		}
		crosstalk_offset += PDXTC_SIZE;

		/****SW_GGC_checksum****/
		checksum = 0;
		for (i = SW_GGC_addr+1; i < SW_GGC_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sqpd2279f[SW_GGC_checksum_addr] != checksum) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SW_GGC_checksum_addr  %x vs %x\n", checksum, vivo_otp_data_s5kjn1sqpd2279f[SW_GGC_checksum_addr]);
			LOG_INF("SW_GGC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
			}
		for (i = 0; i < SWGGC_SIZE; i++) {
			crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]=vivo_otp_data_s5kjn1sqpd2279f[SWGGC_OFFSET+i];
			//LOG_INF("SW_GGC[%d] = 0x%x\n", i,  crosstalk_data_s5kjn1sqpd2279f[crosstalk_offset+i]);
		}
		crosstalk_offset += SWGGC_SIZE;

		/****HW_GGC_checksum****/
		checksum = 0;
		if(VIVO_VENDOR_QTECH == vivo_otp_data_s5kjn1sqpd2279f[0x01])
		{
			for (i = HWGGC_addr+1; i < HWGGC_addr_checksum_addr; i++) {
				//LOG_INF("vivo_otp_data_s5kjn1sqpd2279f[0x%x] = 0x%x\n", i, vivo_otp_data_s5kjn1sqpd2279f[i]);
				checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kjn1sqpd2279f[HWGGC_addr_checksum_addr] != checksum) {
				S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("HW_GGC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
				return 0;
			}
		}
		else if(VIVO_VENDOR_SUNNY == vivo_otp_data_s5kjn1sqpd2279f[0x01])
		{
			for (i = HWGGC_addr+1; i < HWGGC_addr_checksum_addr_SUNNY; i++) {
				//LOG_INF("vivo_otp_data_s5kjn1sqpd2279f[0x%x] = 0x%x\n", i, vivo_otp_data_s5kjn1sqpd2279f[i]);
				checksum += vivo_otp_data_s5kjn1sqpd2279f[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kjn1sqpd2279f[HWGGC_addr_checksum_addr_SUNNY] != checksum) {
				S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("HW_GGC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQPD2279F_OTP_ERROR_CODE);
				return 0;
			}
		}

		/****check if awb out of range[5000K high  cct]****/
		R_unit = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+2]);
		B_unit = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+4]);
		G_unit = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+6]);

		R_golden = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+8]);
		B_golden = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+10]);
		G_golden = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+12]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (12% * 1024)^2****/
		LOG_INF("lxd_add:R_unit=%d, R_golden=%d, B_unit=%d, B_golden=%d, G_unit=%d, G_golden=%d\n", R_unit, R_golden, B_unit, B_golden, G_unit, G_golden);
		t1 = 1024*1024*(R_unit-R_golden)*(R_unit-R_golden);
		t2 = R_golden*R_golden;
		t3 = 1048576*(G_unit-G_golden)*(G_unit-G_golden);
		t4 = G_golden*G_golden;
		t5 = 1048576*(B_unit-B_golden)*(B_unit-B_golden);
		t6 = B_golden*B_golden;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 15100;
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[lxd++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}


		/****check if awb out of range[3000K low cct]****/
		R_unit_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+14]);
		B_unit_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+16]);
		G_unit_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+18]);

		R_golden_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+20]);
		B_golden_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+22]);
		G_golden_low = vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Awb_addr+24]);

		/****awb_range = pow(pow(R_unit/R_golden-1, 2)+pow(B_unit/B_golden-1, 2)+pow(G_unit/G_golden-1, 2), 0.5);****/
		/****t = 1024^2 * (R_unit-R_golden)^2 /(R_golden)^2 + 1024^2 * (B_unit-B_golden)^2 /(B_golden)^2 + 1024^2 * (G_unit-G_golden)^2 /(G_golden)^2 < (12% * 1024)^2****/
		LOG_INF("cfx_add:R_unit_low=%d, R_golden_low=%d, B_unit_low=%d, B_golden_low=%d, G_unit_low=%d, G_golden_low=%d\n", R_unit_low, R_golden_low, B_unit_low, B_golden_low, G_unit_low, G_golden_low);
		t1 = 1024*1024*(R_unit_low-R_golden_low)*(R_unit_low-R_golden_low);
		t2 = R_golden_low*R_golden_low;
		t3 = 1048576*(G_unit_low-G_golden_low)*(G_unit_low-G_golden_low);
		t4 = G_golden_low*G_golden_low;
		t5 = 1048576*(B_unit_low-B_golden_low)*(B_unit_low-B_golden_low);
		t6 = B_golden_low*B_golden_low;
		temp = t1/t2 + t3/t4 + t5/t6 ;
		t = temp - 15100;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			pr_err("[%s] AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n",__FUNCTION__, temp, S5KJN1SQPD2279F_OTP_ERROR_CODE);
			return 0;
		}


		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 9];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 10]);

		AF_mac = vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 11];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_s5kjn1sqpd2279f[Af_addr + 12]);

		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
        /* diff_inf_macro: SUNNY 340±140, QTECH 340±140 */
		if (diff_inf_macro > 480 || diff_inf_macro < 200) {  /*AF code out of range*/
			S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			pr_err("[%s] [lxd++]AF out of range error!!!   otp_error_code:%d    Module ID = 0x%0x   inf = %d    mac = %d\n", __FUNCTION__,S5KJN1SQPD2279F_OTP_ERROR_CODE, vivo_otp_data_s5kjn1sqpd2279f[0x01], AF_inf, AF_mac);
			return 0;
		}
		#endif

		/*lxd add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("[lxd++]read_S5KJN1SQPD2279F_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_s5kjn1sqpd2279f[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_s5kjn1sqpd2279f[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_s5kjn1sqpd2279f[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*lxd add for pdaf data end*/
		S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		pr_err("otp read normally return\n");
		return 1;
	} else {
		S5KJN1SQPD2279F_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		pr_err("[%s] ModuleInfo_addr = 0x%x, Fulse_id_addr = 0x%x, SN_addr= 0x%x, Af_addr = 0x%x, Awb_addr = 0x%x, Lsc_addr = 0x%x, \
		PDAF_proc1_addr = 0x%x, PDAF_proc2_addr = 0x%x, XTC_cal_addr = 0x%x, SensorXTC_cal_addr = 0x%x, PDXTC_addr = 0x%x, SW_GGC_addr = 0x%x, OIS_addr = 0x%x\n",
             __FUNCTION__,
		vivo_otp_data_s5kjn1sqpd2279f[ModuleInfo_addr],
		vivo_otp_data_s5kjn1sqpd2279f[Fulse_id_addr],
		vivo_otp_data_s5kjn1sqpd2279f[SN_addr],
		vivo_otp_data_s5kjn1sqpd2279f[Af_addr],
		vivo_otp_data_s5kjn1sqpd2279f[Awb_addr],
		vivo_otp_data_s5kjn1sqpd2279f[Lsc_addr],
		vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc1_addr],
		vivo_otp_data_s5kjn1sqpd2279f[PDAF_proc2_addr],
		vivo_otp_data_s5kjn1sqpd2279f[XTC_cal_addr],
		vivo_otp_data_s5kjn1sqpd2279f[SensorXTC_cal_addr],
		vivo_otp_data_s5kjn1sqpd2279f[PDXTC_addr],
		vivo_otp_data_s5kjn1sqpd2279f[SW_GGC_addr],
		vivo_otp_data_s5kjn1sqpd2279f[HWGGC_addr]);
		pr_err("[%s][lxd++]invalid otp data. error!!!   otp_error_code:%d\n", __FUNCTION__,S5KJN1SQPD2279F_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
