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
#include "gc02m1pd2166gmipiraw_Sensor.h"

#define PFX "MAIN2[2E0]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
/*unsigned char PDAF_data[1310];*/
#define VIVO_OTP_DATA_SIZE 0x143F  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
//#define AF_RANGE_GOT  /****if got the range of AF_inf and AF_mac , open this define!!!!****/
#define VIVO_EEPROM_WRITE_ID 0xA4
#define VIVO_I2C_SPEED 400
#define VIVO_MAX_OFFSET 0x143E
//#define VIVO_VENDOR_SUNNY 0x01
/*#define VIVO_VENDOR_TRULY 0x02*/
#define VIVO_VENDOR_QTECH 0x05
/*#define VIVO_VENDOR_OFILM 0x09*/
#define VIVO_VENDOR_SUNWIN 0x03
#define VIVO_VENDOR_LENS_ID 0x14
#define VIVO_VENDOR_VCM_ID 0x00
#define VIVO_VENDOR_DRIVERIC_ID 0x00
#define VIVO_VENDOR_PLATFORM_ID 0x03

static unsigned char vivo_otp_data_gc02m1pd2166g[VIVO_OTP_DATA_SIZE];
static unsigned const int ModuleInfo_addr = 0x0000;
static unsigned const int ModuleInfo_checksum_addr = 0x001f;
static unsigned const int SN_addr = 0x0045;
static unsigned const int SN_checksum_addr = 0x0065;
static unsigned const int Awb_addr = 0x0CAB;
static unsigned const int Awb_checksum_addr = 0x0CE0;
static unsigned const int Lsc_addr = 0x0CE1;
static unsigned const int Lsc_checksum_addr = 0x143E;


#ifdef AF_RANGE_GOT
unsigned const int AF_inf_golden = 0;/*320;*/
unsigned const int AF_mac_golden = 0;/*680;*/
#endif
static int checksum;
otp_error_code_t GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main2_gc02m1pd2166g[13];
extern MUINT32  material_inf_main2_gc02m1pd2166g[4];   

bool gc02m1pd2166g_vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{

	char pu_send_cmd[2] = {(char)(addr >> 8),  (char)(addr & 0xFF)};
    if (addr > VIVO_MAX_OFFSET) { /*VIVO_MAX_OFFSET*/
		return false;
	}
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	if (iReadRegI2C(pu_send_cmd,  2,  (u8 *)data,  1,  VIVO_EEPROM_WRITE_ID) < 0) {
		return false;
	}
    return true;
}
//extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN2_2e0_otp_read(void)
{
	int i = 0;
	int offset = ModuleInfo_addr;
	int check_if_group_valid = 0;
	int R_unit = 0, B_unit = 0, G_unit = 0, R_golden = 0, B_golden = 0, G_golden = 0;
	int R_unit_low = 0, B_unit_low = 0, G_unit_low = 0, R_golden_low = 0, B_golden_low = 0, G_golden_low = 0;

	#ifdef AF_RANGE_GOT
	int diff_inf = 0,  diff_mac = 0,  diff_inf_macro = 0;
	int AF_inf = 0,  AF_mac = 0;
	#endif

	long long t1, t2, t3, t4, t5, t6, t, temp;
	GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	#if 0
	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip gc02m1pd2166g_otp_read\n");
		return 1;
	}
	/* guojunzheng add end */
	#endif
	
	for (i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		if (!gc02m1pd2166g_vivo_read_eeprom(offset,  &vivo_otp_data_gc02m1pd2166g[i])) {
			LOG_INF("[lxd++]read_vivo_eeprom 0x%0x %d fail \n", offset,  vivo_otp_data_gc02m1pd2166g[i]);
			return 0;
		}
		//LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", offset,  vivo_otp_data_gc02m1pd2166g[i]);
		offset++;
	}
	#if 1
	for(i = 0; i < VIVO_OTP_DATA_SIZE; i++) {
		LOG_INF("[lxd++]read_vivo_eeprom Data[0x%0x]:0x%x\n", i,  vivo_otp_data_gc02m1pd2166g[i]);
	}
	#endif
	/*check_if_group_valid = vivo_otp_data_gc02m1pd2166g[ModuleInfo_addr] | vivo_otp_data_gc02m1pd2166g[Awb_addr] | vivo_otp_data_gc02m1pd2166g[Af_addr] | vivo_otp_data_gc02m1pd2166g[Lsc_addr] | vivo_otp_data_gc02m1pd2166g[PD_Proc1_addr] | vivo_otp_data_gc02m1pd2166g[PD_Proc2_addr];*/
	if ((0x01 == vivo_otp_data_gc02m1pd2166g[ModuleInfo_addr]) &&  
		(0x01 == vivo_otp_data_gc02m1pd2166g[SN_addr]) && 
		(0x01 == vivo_otp_data_gc02m1pd2166g[Awb_addr]) &&
		(0x01 == vivo_otp_data_gc02m1pd2166g[Lsc_addr])) {

		check_if_group_valid = 0x01;
		LOG_INF("0x01 is valid.check_if_group_valid:%d\n", check_if_group_valid);
	}

	if (check_if_group_valid == 0x01) { /****all the data is valid****/
		/****ModuleInfo****/
#if 0
		if ((VIVO_VENDOR_SUNWIN!= vivo_otp_data_gc02m1pd2166g[0x01]) && (VIVO_VENDOR_QTECH != vivo_otp_data_gc02m1pd2166g[0x01]) ) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]Module ID error!!!    otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		} else if (
				(VIVO_VENDOR_VCM_ID != vivo_otp_data_gc02m1pd2166g[0x09]) || 
				(VIVO_VENDOR_DRIVERIC_ID != vivo_otp_data_gc02m1pd2166g[0x0A]) || 
				(VIVO_VENDOR_PLATFORM_ID != vivo_otp_data_gc02m1pd2166g[0x02])) {
				
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: Platform ID or Lens or VCM ID or Driver_IC ID  Error!!!    otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		} else if ((0xff != vivo_otp_data_gc02m1pd2166g[0x0B]) || 
				(0x00 != vivo_otp_data_gc02m1pd2166g[0x0C]) || 
				(0x0b != vivo_otp_data_gc02m1pd2166g[0x0D]) || 
				((0x01 != vivo_otp_data_gc02m1pd2166g[0x0E]) && (0x03 != vivo_otp_data_gc02m1pd2166g[0x0E]) && (0x02 != vivo_otp_data_gc02m1pd2166g[0x0E]))) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_MODULE_INFO_ERROR;
			LOG_INF("[lxd++]: calibration version  Error!!!    Read version:0x%2x%2x%2x%2x\n", 
				vivo_otp_data_gc02m1pd2166g[0x0B], vivo_otp_data_gc02m1pd2166g[0x0C], vivo_otp_data_gc02m1pd2166g[0x0D], vivo_otp_data_gc02m1pd2166g[0x0E]);
			return 0;
		}
#endif		
		/****ModuleInfo_checksum****/
		checksum = 0;
		for (i = ModuleInfo_addr+1; i < ModuleInfo_checksum_addr; i++) {
			checksum += vivo_otp_data_gc02m1pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_gc02m1pd2166g[ModuleInfo_checksum_addr] != checksum) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]ModuleInfo_checksum error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
			}

		/****material info start****/
			material_inf_main2_gc02m1pd2166g[0] = (MUINT32)vivo_otp_data_gc02m1pd2166g[0x0F];
			material_inf_main2_gc02m1pd2166g[1] = (MUINT32)vivo_otp_data_gc02m1pd2166g[0x10];
			material_inf_main2_gc02m1pd2166g[2] = (MUINT32)vivo_otp_data_gc02m1pd2166g[0x11];
			material_inf_main2_gc02m1pd2166g[3] = (MUINT32)vivo_otp_data_gc02m1pd2166g[0x12];
			LOG_INF("material_inf_main2_gc02m1pd2166g[0] = 0x%x, material_inf_main2_gc02m1pd2166g[1] = 0x%x, material_inf_main2_gc02m1pd2166g[2] = 0x%x,material_inf_main2_gc02m1pd2166g[3] = 0x%x\n",
				material_inf_main2_gc02m1pd2166g[0]  , material_inf_main2_gc02m1pd2166g[1],  material_inf_main2_gc02m1pd2166g[2], material_inf_main2_gc02m1pd2166g[3]);
		/****material info end****/

		/****SN_checksum****/
		checksum = 0;
		for (i = SN_addr+1; i < SN_checksum_addr; i++) {
			/*LOG_INF("vivo_otp_data_gc02m1pd2166g[0x%x] = 0x%x\n", i, vivo_otp_data_gc02m1pd2166g[i]);*/
			checksum += vivo_otp_data_gc02m1pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_gc02m1pd2166g[SN_checksum_addr] != checksum) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("SN_checksum error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		}
		sn_inf_main2_gc02m1pd2166g[0] = 0x01;
		for (i = 0; i < 12; i++) {
			sn_inf_main2_gc02m1pd2166g[i+1] = (MUINT32)vivo_otp_data_gc02m1pd2166g[i + SN_addr+1];
			/*LOG_INF("sn_inf_main2_gc02m1pd2166g[%d] = 0x%x, vivo_otp_data_gc02m1pd2166g[0x%x] = 0x%x\n", i+1  , sn_inf_main2_gc02m1pd2166g[i+1],  i +SN_addr+1, vivo_otp_data_gc02m1pd2166g[i+SN_addr+1]);*/
		}

		/****AWB_checksum****/
		checksum = 0;
		for (i = Awb_addr+1; i < Awb_checksum_addr; i++) {
			checksum += vivo_otp_data_gc02m1pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_gc02m1pd2166g[Awb_checksum_addr] != checksum) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]AWB_checksum error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
			}


		/****LSC_checksum****/
		checksum = 0;
		for (i = Lsc_addr+1; i < Lsc_checksum_addr; i++) {
			checksum += vivo_otp_data_gc02m1pd2166g[i];
			}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_gc02m1pd2166g[Lsc_checksum_addr] != checksum) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("[lxd++]LSC_checksum error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		}

#if 0
		/****Coefficient_of_light_source_checksum****/
		checksum = 0;
		for (i = Coefficient_of_light_source_addr+1; i < Coefficient_of_light_source_checksum_addr; i++) {
			checksum += vivo_otp_data_gc02m1pd2166g[i];
		}
			checksum = checksum % 0xff+1;
		if (vivo_otp_data_gc02m1pd2166g[Coefficient_of_light_source_checksum_addr] != checksum) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_CHECKSUM_ERROR;
			LOG_INF("Coefficient_of_light_source_checksum  error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
			}
#endif
		/****check if awb out of range[5000K high  cct]****/
		R_unit = vivo_otp_data_gc02m1pd2166g[Awb_addr+1];
		R_unit = (R_unit << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+2]);
		B_unit = vivo_otp_data_gc02m1pd2166g[Awb_addr+3];
		B_unit = (B_unit << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+4]);
		G_unit = vivo_otp_data_gc02m1pd2166g[Awb_addr+5];
		G_unit = (G_unit << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+6]);

		R_golden = vivo_otp_data_gc02m1pd2166g[Awb_addr+7];
		R_golden = (R_golden << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+8]);
		B_golden = vivo_otp_data_gc02m1pd2166g[Awb_addr+9];
		B_golden = (B_golden << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+10]);
		G_golden = vivo_otp_data_gc02m1pd2166g[Awb_addr+11];
		G_golden = (G_golden << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+12]);

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
		t = temp - 30303;
		LOG_INF("lxd_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("[lxd++]AWB[high cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		}
		
		/****check if awb out of range[3000K low cct]****/
		R_unit_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+13];
		R_unit_low = (R_unit_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+14]);
		B_unit_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+15];
		B_unit_low = (B_unit_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+16]);
		G_unit_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+17];
		G_unit_low = (G_unit_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+18]);

		R_golden_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+19];
		R_golden_low = (R_golden_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+20]);
		B_golden_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+21];
		B_golden_low = (B_golden_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+22]);
		G_golden_low = vivo_otp_data_gc02m1pd2166g[Awb_addr+23];
		G_golden_low = (G_golden_low << 8) | (vivo_otp_data_gc02m1pd2166g[Awb_addr+24]);

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
		t = temp - 30303;
		LOG_INF("cfx_add:t1 = %lld , t2 = %lld , t3 = %lld , t4 = %lld , t5 = %lld , t6 = %lld , temp = %lld , t = %lld\n", t1, t2, t3, t4, t5, t6, temp, t);
		if (t > 0) {
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AWB_OUT_OF_RANGE;
			LOG_INF("AWB[low cct] out of range error!!!This module range^2 *1024^2 is %lld%%   otp_error_code:%d\n", temp, GC02M1PD2166G_OTP_ERROR_CODE);
			return 0;
		}

		
		#ifdef AF_RANGE_GOT
		/*******check if AF out of range******/

		AF_inf  =  vivo_otp_data_gc02m1pd2166g[Af_addr + 9];
		AF_inf = (AF_inf << 8) | (vivo_otp_data_gc02m1pd2166g[Af_addr + 10]);

		AF_mac = vivo_otp_data_gc02m1pd2166g[Af_addr + 11];
		AF_mac = (AF_mac << 8) | (vivo_otp_data_gc02m1pd2166g[Af_addr + 12]);
        
		diff_inf = (AF_inf - AF_inf_golden) > 0 ? (AF_inf - AF_inf_golden) : (AF_inf_golden - AF_inf);
		diff_mac = (AF_mac - AF_mac_golden) > 0 ? (AF_mac - AF_mac_golden) : (AF_mac_golden - AF_mac);
		diff_inf_macro = AF_mac - AF_inf;
        /* diff_inf_macro: SUNNY 340��140, QTECH 340��140 */
		if (diff_inf_macro > 480 || diff_inf_macro < 200) {  /*AF code out of range*/
			GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_AF_OUT_OF_RANGE;
			LOG_INF("[lxd++]AF out of range error!!!   otp_error_code:%d    Module ID = 0x%0x   inf = %d    mac = %d\n", GC02M1PD2166G_OTP_ERROR_CODE, vivo_otp_data_gc02m1pd2166g[0x01], AF_inf, AF_mac);
			return 0;
		}
		#endif

		/*lxd add for pdaf data start 20161223*/
		/*for(i = 0;i < 1372;i++)
		{
			if(i == 0)
				LOG_INF("[lxd++]read_GC02M1PD2166G_PDAF_data");
			if(i < 496)
				PDAF_data[i] = vivo_otp_data_gc02m1pd2166g[PD_Proc1_addr+i+1];
			else //i >= 496
				PDAF_data[i] = vivo_otp_data_gc02m1pd2166g[PD_Proc1_addr+i+3];
		}*/
		/*copy pdaf data*/
		/*memcpy(PDAF_data, &vivo_otp_data_gc02m1pd2166g[PDAF_addr+1], PDAF_checksum_addr-PDAF_addr-1);*/
		/*lxd add for pdaf data end*/
		GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
		LOG_INF("[lxd++]otp data return normal.\n");
		return 1;
	} else {
		GC02M1PD2166G_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("[lxd++]invalid otp data. error!!!   otp_error_code:%d\n", GC02M1PD2166G_OTP_ERROR_CODE);
		return 0;
	}
}
/*vivo lxd add end*/
