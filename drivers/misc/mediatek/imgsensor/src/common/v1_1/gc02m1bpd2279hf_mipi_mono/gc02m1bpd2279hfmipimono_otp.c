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
#include "gc02m1bpd2279hfmipimono_Sensor.h"

#define PFX "MAIN2[2E2]_EEPROM_OTP"
#define LOG_INF(format,  args...)	pr_debug(PFX "[%s] " format,  __FUNCTION__,  ##args)

/**************/
/****vivo lxd add start****/
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define VIVO_OTP_DATA_SIZE 0x15  /*sizeof(moduleinfo)+sizeof(awb)+sizeof(af)+sizeof(lsc)*/
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

unsigned char vivo_otp_data_gc02m1bpd2279hf[VIVO_OTP_DATA_SIZE];
static unsigned const int SN_addr = 0x01;
static unsigned const int FPC_addr = 0x0D;

static int checksum = 1;
otp_error_code_t GC02M1BPD2279HF_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
extern MUINT32  sn_inf_main2_gc02m1bpd2279hf[13];
extern MUINT32  material_inf_main2_gc02m1bpd2279hf[4];
extern MUINT32  module_inf_main2_gc02m1bpd2279hf[2];
extern MUINT32  fpc_main2_gc02m1bpd2279hf[4];

static kal_uint8 addrList[VIVO_OTP_ADDR_SIZE] = {
	0x78, 0x80, 0x88, 0x90, 0x98, 0xA0, 0xA8, 0xB0, 0xB8, 0xC0, 0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0, 0xF8,
};

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

static void vivo_read_eeprom(kal_uint16 addr,  BYTE *data)
{
	kdSetI2CSpeed(VIVO_I2C_SPEED);

	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0x17, addr);
	write_cmos_sensor(0xf3, 0x34);
	*data = read_cmos_sensor(0x19);
}
extern unsigned int is_atboot;/*guojunzheng add*/
int MAIN2_gc02m1_otp_read(void)
{
	int i = 0;
	unsigned int Fpc_cycle = 0, Fpc_date = 0, Fpc_lot = 0;
	GC02M1BPD2279HF_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;

	/* This operation takes a long time, we need to skip it. guojunzheng add begin */
	#if 1
	if (is_atboot == 1) {
		LOG_INF("[lxd++]AT mode skip gc02m1bpd2279hfmipimono_otp_read\n");
		return 1;
	}
	#endif

	write_cmos_sensor(0xf3, 0x30);
	for (i = 0; i < VIVO_OTP_ADDR_SIZE; i++) {
		vivo_read_eeprom(addrList[i],  &vivo_otp_data_gc02m1bpd2279hf[i]);
		//LOG_INF("read_vivo_eeprom temp addrList:0x%0x Data[0x%0x]:0x%x\n",addrList[i], i, vivo_otp_data_gc02m1bpd2279hf[i]);
	}


	if ((vivo_otp_data_gc02m1bpd2279hf[0x00] & 0x02) == 0x02){
		checksum = 0;
		GC02M1BPD2279HF_OTP_ERROR_CODE = OTP_ERROR_CODE_GROUP_INVALID;
		LOG_INF("invalid otp data. error!!! otp_error_code:%d\n", GC02M1BPD2279HF_OTP_ERROR_CODE);
		return 0;
	}else if ((vivo_otp_data_gc02m1bpd2279hf[0x00] & 0x01) == 0x00){
		checksum = 0;
		GC02M1BPD2279HF_OTP_ERROR_CODE = OTP_ERROR_CODE_EMPTY;
		LOG_INF("OTP data is empty!!! otp_error_code:%d\n", GC02M1BPD2279HF_OTP_ERROR_CODE);
		return 0;
	}

	/****module info start****/
	module_inf_main2_gc02m1bpd2279hf[0] = vivo_otp_data_gc02m1bpd2279hf[0x02];
	/****module info end****/

	vivo_otp_data_gc02m1bpd2279hf[0x11] = 0xFF;
	vivo_otp_data_gc02m1bpd2279hf[0x12] = 0x00;
	vivo_otp_data_gc02m1bpd2279hf[0x13] = 0x0B;
	vivo_otp_data_gc02m1bpd2279hf[0x14] = 0x01;

	material_inf_main2_gc02m1bpd2279hf[0] = 0x02;
	material_inf_main2_gc02m1bpd2279hf[1] = 0x37;
	material_inf_main2_gc02m1bpd2279hf[2] = 0x15;
	material_inf_main2_gc02m1bpd2279hf[3] = 0x18;

	sn_inf_main2_gc02m1bpd2279hf[0] = 0x01;
	for (i = 0; i < 12; i++) {
		sn_inf_main2_gc02m1bpd2279hf[i+1] = (MUINT32)vivo_otp_data_gc02m1bpd2279hf[i + SN_addr];
		LOG_INF("sn_inf_main2_gc02m1bpd2279hf[%d] = 0x%x, vivo_otp_data_gc02m1bpd2279hf[0x%x] = 0x%x\n", i+1  , sn_inf_main2_gc02m1bpd2279hf[i+1],  i +SN_addr+1, vivo_otp_data_gc02m1bpd2279hf[i+SN_addr+1]);
	}

	Fpc_cycle = vivo_otp_data_gc02m1bpd2279hf[FPC_addr];
	Fpc_date = vivo_otp_data_gc02m1bpd2279hf[FPC_addr+1];
	Fpc_lot = ((vivo_otp_data_gc02m1bpd2279hf[FPC_addr+2] & 0x3) * 256) +
				(vivo_otp_data_gc02m1bpd2279hf[FPC_addr + 3] & 0xFF);
	LOG_INF("Fpc_cycle = %d, Fpc_date = %d, Fpc_lot = %d\n", Fpc_cycle, Fpc_date, Fpc_lot);

	// add for fpc data
	fpc_main2_gc02m1bpd2279hf[0] = vivo_otp_data_gc02m1bpd2279hf[FPC_addr];
	fpc_main2_gc02m1bpd2279hf[1] = vivo_otp_data_gc02m1bpd2279hf[FPC_addr + 1];
	fpc_main2_gc02m1bpd2279hf[2] = vivo_otp_data_gc02m1bpd2279hf[FPC_addr + 2];
	fpc_main2_gc02m1bpd2279hf[3] = vivo_otp_data_gc02m1bpd2279hf[FPC_addr + 3] & 0xFF;
	GC02M1BPD2279HF_OTP_ERROR_CODE = OTP_ERROR_CODE_NORMAL;
	return 1;

}
/*vivo lxd add end*/
