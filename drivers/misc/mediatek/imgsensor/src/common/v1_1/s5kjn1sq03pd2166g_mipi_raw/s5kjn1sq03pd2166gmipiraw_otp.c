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
#include "imgsensor.h"
#include "s5kjn1sq03pd2166gmipiraw_Sensor.h"

#define PFX "MAIN[38E1]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_info(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x3B50  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
//#define AF_RANGE_GOT  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA0
#define OIS_WRITE_ID 0xE4
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x3FFF
#define VIVO_VENDOR_SUNNY 0x01
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
#define VIVO_VENDOR_PLATFORM_ID 0x03
#define VIVO_VENDOR_LENS_ID 0x11
#define VIVO_VENDOR_VCM_ID 0x13
#define VIVO_VENDOR_DRIVERIC_ID 0x11

unsigned char vivo_otp_data_s5kjn1sq03pd2166g[VIVO_OTP_DATA_SIZE];
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
static unsigned const int HWGGC_addr_checksum_addr = 0x3AE8;
#if 0
static unsigned const int OIS_shift_calibration_addr = 0x2EE0;
static unsigned const int OIS_shift_calibration_checksum_addr = 0x2F3F;
#endif

#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main_s5kjn1sq03pd2166g[13];
extern MUINT32  material_inf_main_s5kjn1sq03pd2166g[4];  
extern MUINT32  af_calib_inf_main_s5kjn1sq03pd2166g[6];
extern MUINT32 crosstalk_data_PD2166G[PDAF_DATA_SIZE - 2];  /*0 flag   1-606 data*/

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
#if 0  /*no use*/
static kal_uint16 read_cmos_sensor_16_16(kal_uint32 addr)
{
	kal_uint16 get_byte= 0;
	char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	 /*kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, OIS_WRITE_ID);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static void write_cmos_sensor_16_16(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	/* kdSetI2CSpeed(imgsensor_info.i2c_speed); Add this func to set i2c speed by each sensor*/
	iWriteRegI2C(pusendcmd , 4, OIS_WRITE_ID);
}


static int Main_7309_otp_read_setting_init_continuous_read(kal_uint16 addr)
{
	LOG_INF("ois reset start\n");
	write_cmos_sensor_16_16(0xD002, 0x0001); /*reset*/
	mdelay(4);
	write_cmos_sensor_16_16(0xD001, 0x0001);/*Active mode(DSP ON)*/
	mdelay(25);/*ST gyro - over wait 25ms,default Servo on*/
	write_cmos_sensor_16_16(0xEBF1, 0x56FA);/*User protection release*/
	mdelay(10);
	/*User protection release*/
	LOG_INF("ois reset end, chip id = 0x%x\n",read_cmos_sensor_16_16(0x7000));
	return 0;
}
#endif 

extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN_38E1_otp_read(void)
{
	int i = 0;
	int index = 0;
	int offset = (ModuleInfo_addr /*+ 0x5000*/);
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	#if 1
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip s5kjn1sq03pd2166g_otp_read\n");
		return 1;
	}
	#endif
	/* guojunzheng add end */
	//Main_7309_otp_read_setting_init_continuous_read(offset);

	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!vivo_read_eeprom(offset,  &vivo_otp_data_s5kjn1sq03pd2166g[i])) {
			LOG_INF("[lxd++]read_vivo_eeprom 0x%0x %d fail \n", offset,  vivo_otp_data_s5kjn1sq03pd2166g[i]);
			return 0;
		}
	//	LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data_s5kjn1sq03pd2166g[i]);
		offset++;
	}
	#if 0
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_s5kjn1sq03pd2166g[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data_s5kjn1sq03pd2166g[ModuleInfo_addr] | vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr] | vivo_otp_data_s5kjn1sq03pd2166g[Af_addr] | vivo_otp_data_s5kjn1sq03pd2166g[Lsc_addr] | vivo_otp_data_s5kjn1sq03pd2166g[PD_Proc1_addr] | vivo_otp_data_s5kjn1sq03pd2166g[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data_s5kjn1sq03pd2166g[ModuleInfo_addr]) && 
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[Fulse_id_addr]) && 
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[SN_addr]) && 
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[Af_addr]) && 
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[Lsc_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc1_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc2_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[XTC_cal_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[SensorXTC_cal_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[PDXTC_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[SW_GGC_addr]) &&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[HWGGC_addr]) /*&&
		(0x01 == vivo_otp_data_s5kjn1sq03pd2166g[OIS_shift_calibration_addr])*/) {
		
		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
#if 0
		if ((VIVO_VENDOR_QTECH != vivo_otp_data_s5kjn1sq03pd2166g[0x01])  && (VIVO_VENDOR_SUNNY != vivo_otp_data_s5kjn1sq03pd2166g[0x01])) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]Module ID error!!!    otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		} else if ((VIVO_VENDOR_LENS_ID != vivo_otp_data_s5kjn1sq03pd2166g[0x08]) || 
				(VIVO_VENDOR_VCM_ID != vivo_otp_data_s5kjn1sq03pd2166g[0x09]) || 
				(VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_s5kjn1sq03pd2166g[0x0A]) || 
				(VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_s5kjn1sq03pd2166g[0x02])) {
				
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);

			return 0;
		} else if ((0xff != vivo_otp_data_s5kjn1sq03pd2166g[0x0B]) || 
				(0x00 != vivo_otp_data_s5kjn1sq03pd2166g[0x0C]) || 
				(0x0b != vivo_otp_data_s5kjn1sq03pd2166g[0x0D]) || 
				((0x01 != vivo_otp_data_s5kjn1sq03pd2166g[0x0E]) && (0x03 != vivo_otp_data_s5kjn1sq03pd2166g[0x0E]) && (0x02 != vivo_otp_data_s5kjn1sq03pd2166g[0x0E]))) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", 
				vivo_otp_data_s5kjn1sq03pd2166g[0x0B], vivo_otp_data_s5kjn1sq03pd2166g[0x0C], vivo_otp_data_s5kjn1sq03pd2166g[0x0D], vivo_otp_data_s5kjn1sq03pd2166g[0x0E]);
			return 0;
		}
#endif
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[ModuleInfo_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}
		
		/****material info start****/
			material_inf_main_s5kjn1sq03pd2166g[0] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[0x0F];
			material_inf_main_s5kjn1sq03pd2166g[1] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[0x10];
			material_inf_main_s5kjn1sq03pd2166g[2] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[0x11];
			material_inf_main_s5kjn1sq03pd2166g[3] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[0x12];
			LOG_INF("material_inf_main_s5kjn1sq03pd2166g[0] = 0x%x, material_inf_main_s5kjn1sq03pd2166g[1] = 0x%x, material_inf_main_s5kjn1sq03pd2166g[2] = 0x%x,material_inf_main_s5kjn1sq03pd2166g[3] = 0x%x\n",
				material_inf_main_s5kjn1sq03pd2166g[0]  , material_inf_main_s5kjn1sq03pd2166g[1],  material_inf_main_s5kjn1sq03pd2166g[2], material_inf_main_s5kjn1sq03pd2166g[3]);
		/****material info end****/
		
		/****Fulse id_checksum****/
		checksum = 0;
		for (i = Fulse_id_addr+1; i < Fulse_id_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[Fulse_id_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data_s5kjn1sq03pd2166g[0x%x] = 0x%x\n", i, vivo_otp_data_s5kjn1sq03pd2166g[i]);*/
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[SN_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_main_s5kjn1sq03pd2166g[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main_s5kjn1sq03pd2166g[i+1] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[i + SN_addr+1];
			/*LOG_INF("sn_inf_main_s5kjn1sq03pd2166g[%d] = 0x%x, vivo_otp_data_s5kjn1sq03pd2166g[0x%x] = 0x%x\n", i+1  , sn_inf_main_s5kjn1sq03pd2166g[i+1],  i +SN_addr+1, vivo_otp_data_s5kjn1sq03pd2166g[i+SN_addr+1]);*/
		}


		/****AF_checksum****/
		checksum = 0;
		for (i = Af_addr+1; i < Af_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[Af_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AF_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		af_calib_inf_main_s5kjn1sq03pd2166g[0] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 7];
		af_calib_inf_main_s5kjn1sq03pd2166g[1] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 8];
		af_calib_inf_main_s5kjn1sq03pd2166g[2] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 9];
		af_calib_inf_main_s5kjn1sq03pd2166g[3] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 10];
		af_calib_inf_main_s5kjn1sq03pd2166g[4] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 11];
		af_calib_inf_main_s5kjn1sq03pd2166g[5] = (MUINT32)vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 12];


		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[Awb_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AWB_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}


		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[Lsc_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]LSC_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		/****PDAF Proc1_checksum****/
		checksum = 0;
		for (i = PDAF_proc1_addr+1; i < PDAF_proc1_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc1_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}

		/****PDAF Proc2_checksum****/
		if (0x01 == vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc2_addr]){
			checksum = 0;
			for (i = PDAF_proc2_addr+1; i < PDAF_proc2_checksum_addr; i++) {
				checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
			}
				checksum = checksum % 0xff+1;
			if (vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc2_checksum_addr] != checksum) {
				S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
				LOG_INF("[lxd++]PDAF_data_checksum error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
				return 0;
				}
		}


		/****XTC_cal_checksum****/
		checksum = 0;
		for (i = XTC_cal_addr+1; i < XTC_cal_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[XTC_cal_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("XTC_cal_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}

		/****XTC_cal_checksum****/
		checksum = 0;
		for (i = SensorXTC_cal_addr+1; i < SensorXTC_cal_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[SensorXTC_cal_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("XTC_cal_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}

		/****PDXTC_checksum****/
		checksum = 0;
		for (i = PDXTC_addr+1; i < PDXTC_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[PDXTC_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("PDXTC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}

		/****SW_GGC_checksum****/
		checksum = 0;
		for (i = SW_GGC_addr+1; i < SW_GGC_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[SW_GGC_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SW_GGC_checksum_addr  %x vs %x\n", checksum, vivo_otp_data_s5kjn1sq03pd2166g[SW_GGC_checksum_addr]);
			LOG_INF("SW_GGC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}
		//Copy XTC、SensorXTC、PDXTC、SWGGC data
		index = 1;
		for (i = 0; i < PDAF_DATA_SIZE - 2; i++) {
			if (i == 2612 || i == 3380 || i == 7380) {
				index += 2; 
			}
			crosstalk_data_PD2166G[i] = vivo_otp_data_s5kjn1sq03pd2166g[XTC_cal_addr + i + index];
		}

		for (i = 0; i < 5; i++) {
			LOG_INF("crosstalk_data_PD2166G[%d] =0x%x\n", i,crosstalk_data_PD2166G[i]);
		}
		for (i = PDAF_DATA_SIZE; i > PDAF_DATA_SIZE -5; i--) {
			LOG_INF("crosstalk_data_PD2166G[%d] =0x%x\n", i,crosstalk_data_PD2166G[i]);
		}
		checksum = 0;
		for (i = HWGGC_addr+1; i < HWGGC_addr_checksum_addr; i++) {
			LOG_INF("vivo_otp_data_s5kjn1sq03pd2166g[0x%x] = 0x%x\n", i, vivo_otp_data_s5kjn1sq03pd2166g[i]);
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[HWGGC_addr_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("HW_GGC_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}

#if 0
		/****OIS_checksum****/
		checksum = 0;
		for (i = OIS_addr+1; i < OIS_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
			LOG_INF("checksum :%d  OIS_checksum=%d\n", checksum,vivo_otp_data_s5kjn1sq03pd2166g[OIS_checksum_addr]);

		if (vivo_otp_data_s5kjn1sq03pd2166g[OIS_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("OIS_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}
#endif
#if 0
		/****OIS_shift_calibration_checksum****/
		checksum = 0;
		for (i = OIS_shift_calibration_addr+1; i < OIS_shift_calibration_checksum_addr; i++) {
			checksum += vivo_otp_data_s5kjn1sq03pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_s5kjn1sq03pd2166g[OIS_shift_calibration_checksum_addr] != checksum) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("OIS_shift_calibration_checksum_addr  error!!!   otp_error_code:%d\n", S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
			}
#endif
		/****check if awb out of range[5000K high  cct]****/
		R_unit = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+2]);
		B_unit = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+4]);
		G_unit = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+6]);

		R_golden = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+8]);
		B_golden = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+10]);
		G_golden = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+12]);

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
		t = temp - 10486;
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[lxd++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		
		/****check if awb out of range[3000K low cct]****/
		R_unit_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+14]);
		B_unit_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+16]);
		G_unit_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+18]);

		R_golden_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+20]);
		B_golden_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+22]);
		G_golden_low = vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr+24]);

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
		t = temp - 10486;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			pr_err("[%s] AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n",__FUNCTION__, temp, S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		
		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 9];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 10]);

		AF_mac = vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 11];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_s5kjn1sq03pd2166g[Af_addr + 12]);
        
		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
        /* diff_inf_macro: SUNNY 340±140, QTECH 340±140 */
		if (diff_inf_macro > 480 || diff_inf_macro < 200) {  /*AF code out of range*/
			S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			pr_err("[%s] [lxd++]AF out of range error!!!   otp_error_code:%d    Module ID = 0x%0x   inf = %d    mac = %d\n", __FUNCTION__,S5KJN1SQ03PD2166G_OTP_ERROR_CODE, vivo_otp_data_s5kjn1sq03pd2166g[0x01], AF_inf, AF_mac);
			return 0;
		}
		#endif

		/*lxd add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("[lxd++]read_S5KJN1SQ03PD2166G_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_s5kjn1sq03pd2166g[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_s5kjn1sq03pd2166g[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_s5kjn1sq03pd2166g[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*lxd add for pdaf data end*/
		S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		pr_err("[%s]otp read normally return\n");
		return 1;
	} else {
		S5KJN1SQ03PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		pr_err("[%s] ModuleInfo_addr = 0x%x, Fulse_id_addr = 0x%x, SN_addr= 0x%x, Af_addr = 0x%x, Awb_addr = 0x%x, Lsc_addr = 0x%x, \
		PDAF_proc1_addr = 0x%x, PDAF_proc2_addr = 0x%x, XTC_cal_addr = 0x%x, SensorXTC_cal_addr = 0x%x, PDXTC_addr = 0x%x, SW_GGC_addr = 0x%x, OIS_addr = 0x%x\n",
             __FUNCTION__,
		vivo_otp_data_s5kjn1sq03pd2166g[ModuleInfo_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[Fulse_id_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[SN_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[Af_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[Awb_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[Lsc_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc1_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[PDAF_proc2_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[XTC_cal_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[SensorXTC_cal_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[PDXTC_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[SW_GGC_addr],
		vivo_otp_data_s5kjn1sq03pd2166g[HWGGC_addr]);
		pr_err("[%s][lxd++]invalid otp data. error!!!   otp_error_code:%d\n", __FUNCTION__,S5KJN1SQ03PD2166G_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
