/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */



#ifndef _MAIN_LENS_H

#define _MAIN_LENS_H

#include "lens_list.h"
#include <linux/ioctl.h>
/*chenhan add*/
#include "ois_core.h"
/*add end*/

#define MAX_NUM_OF_LENS 45

#define AF_MAGIC 'A'

#ifdef CONFIG_MACH_MT6779
#define SUPPORT_GETTING_LENS_FOLDER_NAME 1
#else
#define SUPPORT_GETTING_LENS_FOLDER_NAME 0
#endif

/* AFDRV_XXXX be the same as AF_DRVNAME in (*af).c */
#define AFDRV_AD5820AF "AD5820AF"
#define AFDRV_AD5823 "AD5823"
#define AFDRV_AD5823AF "AD5823AF"
#define AFDRV_AK7345AF "AK7345AF"
#define AFDRV_AK7371AF "AK7371AF"
#define AFDRV_AK7374AF "AK7374AF"
#define AFDRV_BU63165AF "BU63165AF"
#define AFDRV_BU63169AF "BU63169AF"
#define AFDRV_BU6424AF "BU6424AF"
#define AFDRV_BU64253GWZAF "BU64253GWZAF"
#define AFDRV_BU6429AF "BU6429AF"
#define AFDRV_BU64748AF "BU64748AF"
#define AFDRV_BU64745GWZAF "BU64745GWZAF"
#define AFDRV_DW9714A "DW9714A"
#define AFDRV_DW9714AF "DW9714AF"
#define AFDRV_DW9714V3AF "DW9714V3AF"
#define AFDRV_DW9714VAF "DW9714VAF"
#define AFDRV_DW9714V2AF "DW9714V2AF"
#define AFDRV_DW9714V3AF "DW9714V3AF"
#define AFDRV_DW9714V4AF "DW9714V4AF"
#define AFDRV_DW9718AF "DW9718AF"
#define AFDRV_DW9718SAF "DW9718SAF"
#define AFDRV_DW9800WAF "DW9800WAF"
#define AFDRV_DW9800VAF "DW9800VAF"
#define AFDRV_DW9719TAF "DW9719TAF"
#define AFDRV_DW9763AF "DW9763AF"
#define AFDRV_DW9814AF "DW9814AF"
#define AFDRV_DW9839AF "DW9839AF"
#define AFDRV_DW9781COISAF "DW9781COISAF"
#define AFDRV_FP5510E2AF "FP5510E2AF"
#define AFDRV_FM50AF "FM50AF"
#define AFDRV_GAF001AF "GAF001AF"
#define AFDRV_GAF002AF "GAF002AF"
#define AFDRV_GAF008AF "GAF008AF"
#define AFDRV_GT9764AF "GT9764AF"
#define AFDRV_LC898122AF "LC898122AF"
#define AFDRV_LC898212AF "LC898212AF"
#define AFDRV_LC898212XDAF "LC898212XDAF"
#define AFDRV_LC898212XDAF_TVC700 "LC898212XDAF_TVC700"
#define AFDRV_LC898212XDAF_F "LC898212XDAF_F"
#define AFDRV_LC898214AF "LC898214AF"
#define AFDRV_LC898217AF "LC898217AF"
#define AFDRV_LC898217AFA "LC898217AFA"
#define AFDRV_LC898217AFB "LC898217AFB"
#define AFDRV_LC898217AFC "LC898217AFC"
#define AFDRV_LC898229AF "LC898229AF"
#define AFDRV_MT9P017AF "MT9P017AF"
#define AFDRV_OV8825AF "OV8825AF"
#define AFDRV_WV511AAF "WV511AAF"
#define AFDRV_DW9718TAF "DW9718TAF"
#define AFDRV_GT9772AF "GT9772AF"
#define AFDRV_GT9772V2AF "GT9772V2AF"
#define AFDRV_GT9768AF "GT9768AF"
/*chehnan add*/
#define OISDRV_DW9781C "DW9781C"
#define OISDRV_RUMBAS4SW "RUMBAS4SW"

/*add end*/

#define CONVERT_CCU_TIMESTAMP 0x1000

/* Structures */
struct stAF_MotorInfo {
	/* current position */
	u32 u4CurrentPosition;
	/* macro position */
	u32 u4MacroPosition;
	/* Infinity position */
	u32 u4InfPosition;
	/* Motor Status */
	bool bIsMotorMoving;
	/* Motor Open? */
	bool bIsMotorOpen;
	/* Support SR? */
	bool bIsSupportSR;
};

/* Structures */
struct stAF_MotorCalPos {
	/* macro position */
	u32 u4MacroPos;
	/* Infinity position */
	u32 u4InfPos;
};

#define STRUCT_MOTOR_NAME 32
#define AF_MOTOR_NAME 31

/* Structures */
struct stAF_MotorName {
	u8 uMotorName[STRUCT_MOTOR_NAME];
};

/* Structures */
struct stAF_MotorCmd {
	u32 u4CmdID;
	u32 u4Param;
};

/* Structures */
struct stAF_CtrlCmd {
	long long i8CmdID;
	long long i8Param[2];
};

/* Structures */
struct stAF_MotorOisInfo {
	int i4OISHallPosXum;
	int i4OISHallPosYum;
	int i4OISHallFactorX;
	int i4OISHallFactorY;
};

/* Structures */
#define OIS_DATA_NUM 8
#define OIS_DATA_MASK (OIS_DATA_NUM - 1)
struct stAF_OisPosInfo {
	int64_t TimeStamp[OIS_DATA_NUM];
	int i4OISHallPosX[OIS_DATA_NUM];
	int i4OISHallPosY[OIS_DATA_NUM];
};

/* Structures */
struct stAF_DrvList {
	u8 uEnable;
	u8 uDrvName[32];
	int (*pAF_SetI2Cclient)(struct i2c_client *pstAF_I2Cclient,
				spinlock_t *pAF_SpinLock, int *pAF_Opened);
	long (*pAF_Ioctl)(struct file *a_pstFile, unsigned int a_u4Command,
			  unsigned long a_u4Param);
	int (*pAF_Release)(struct inode *a_pstInode, struct file *a_pstFile);
	int (*pAF_GetFileName)(unsigned char *pFileName);
	int (*pAF_OisGetHallPos)(int *PosX, int *PosY);
};

#define I2CBUF_MAXSIZE 10

struct stAF_CCUI2CFormat {
	u8 I2CBuf[I2CBUF_MAXSIZE];
	u8 BufSize;
};

#define I2CDATA_MAXSIZE 4
/* Structures */
struct stAF_DrvI2CFormat {
	/* Addr Format */
	u8 Addr[I2CDATA_MAXSIZE];
	u8 AddrNum;
	u8 CtrlData[I2CDATA_MAXSIZE]; /* Control Data */
	u8 BitRR[I2CDATA_MAXSIZE];
	u8 Mask1[I2CDATA_MAXSIZE];
	u8 BitRL[I2CDATA_MAXSIZE];
	u8 Mask2[I2CDATA_MAXSIZE];
	u8 DataNum;
};

#define I2CSEND_MAXSIZE 4
/* Structures */
struct stAF_MotorI2CSendCmd {
	u8 Resolution;
	u8 SlaveAddr;
	/* I2C Send */
	struct stAF_DrvI2CFormat I2CFmt[I2CSEND_MAXSIZE];
	u8 I2CSendNum;
};

/* Control commnad */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */
#define AFIOC_G_MOTORINFO _IOR(AF_MAGIC, 0, struct stAF_MotorInfo)

#define AFIOC_T_MOVETO _IOW(AF_MAGIC, 1, u32)

#define AFIOC_T_SETINFPOS _IOW(AF_MAGIC, 2, u32)

#define AFIOC_T_SETMACROPOS _IOW(AF_MAGIC, 3, u32)

#define AFIOC_G_MOTORCALPOS _IOR(AF_MAGIC, 4, struct stAF_MotorCalPos)

#define AFIOC_S_SETPARA _IOW(AF_MAGIC, 5, struct stAF_MotorCmd)

#define AFIOC_G_MOTORI2CSENDCMD _IOR(AF_MAGIC, 6, struct stAF_MotorI2CSendCmd)

#define AFIOC_S_SETDRVNAME _IOW(AF_MAGIC, 10, struct stAF_MotorName)

#define AFIOC_S_SETPOWERDOWN _IOW(AF_MAGIC, 11, u32)

#define AFIOC_G_MOTOROISINFO _IOR(AF_MAGIC, 12, struct stAF_MotorOisInfo)

#define AFIOC_S_SETPOWERCTRL _IOW(AF_MAGIC, 13, u32)

#define AFIOC_S_SETLENSTEST _IOW(AF_MAGIC, 14, u32)

#define AFIOC_G_OISPOSINFO _IOR(AF_MAGIC, 15, struct stAF_OisPosInfo)

#define AFIOC_S_SETDRVINIT _IOW(AF_MAGIC, 16, u32)

#define AFIOC_G_GETDRVNAME _IOWR(AF_MAGIC, 17, struct stAF_MotorName)

#define AFIOC_X_CTRLPARA _IOWR(AF_MAGIC, 18, struct stAF_CtrlCmd)

#define AFIOC_T_SETSTANDBY _IOW(AF_MAGIC, 19, u32)

/*chenhan add for ois*/
#define AFIOC_X_OIS_START           _IOWR('A', 50, int)
#define AFIOC_X_OIS_SETMODE         _IOWR('A', 51, int)
#define AFIOC_X_OIS_SETACC          _IOWR('A', 52, struct ois_acc_param)
#define AFIOC_X_OIS_STATUSCHECK     _IOWR('A', 53, struct ois_flash_info)
#define AFIOC_X_OIS_SETGYROGAIN     _IOWR('A', 54, int[3])
#define AFIOC_X_OIS_SETFIXMODE      _IOWR('A', 55, struct ois_fixmode_parameter)
#define AFIOC_X_OIS_SETSINEMODE     _IOWR('A', 56, struct ois_sinemode_parameter)
#define AFIOC_X_OIS_SETCIRCLEMODE   _IOWR('A', 57, struct ois_circlemode_parameter)
#define AFIOC_X_OIS_SETSTROKELIMIT  _IOWR('A', 58, int)
#define AFIOC_X_OIS_SETPANTILT      _IOWR('A', 59, struct ois_pantilt_param)
#define AFIOC_X_OIS_GETMODE         _IOWR('A', 60, int)
#define AFIOC_X_OIS_GETFWVERSION    _IOWR('A', 61, int)
#define AFIOC_X_OIS_GETGYROOFFSET   _IOWR('A', 62, int[2])
#define AFIOC_X_OIS_GETLENSINFO     _IOWR('A', 63, struct ois_lens_info_buf)
#define AFIOC_X_OIS_GETINITINFO     _IOWR('A', 64, struct ois_flash_info)
#define AFIOC_X_OIS_GETOTPINFO      _IOWR('A', 65, struct ois_otp_info)
#define AFIOC_X_OIS_OFFSETCAL       _IOWR('A', 66, int)
#define AFIOC_X_OIS_FWUPDATE        _IOWR('A', 67, int)
#define AFIOC_X_OIS_FLASHSAVE       _IOWR('A', 68, int)
#define AFIOC_X_OIS_INIT            _IOWR('A', 69, int)
#define AFIOC_X_OIS_DEINIT          _IOWR('A', 70, int)
#define AFIOC_X_OIS_STREAMON        _IOWR('A', 71, int)
#define AFIOC_X_OIS_STREAMOFF       _IOWR('A', 72, int)
#define AFIOC_X_OIS_GETGYROGAIN     _IOWR('A', 73, int[2])
#define AFIOC_X_OIS_SETTRIPOD       _IOWR('A', 74, int)
#define AFIOC_X_OIS_SETSMOOTH       _IOWR('A', 75, struct ois_smooth_info)
#define AFIOC_X_OIS_INIT_SLAVE      _IOWR('A', 76, int)
#define AFIOC_X_OIS_VSYNC           _IOWR('A', 77, int)
#define AFIOC_X_OIS_READY_CHECK     _IOWR('A', 78, int)
#define AFIOC_X_OIS_LOG_LEVEL       _IOWR('A', 79, int)
#define AFIOC_X_OIS_GETLOOPGAIN     _IOWR('A', 80, int[2])
#define AFIOC_X_OIS_GET_OSCINFO     _IOWR('A', 81, int[2])
#define AFIOC_X_OIS_SETAFDRIFT      _IOWR('A', 82, struct ois_af_drift_param)
#define AFIOC_X_OIS_SETSERVO        _IOWR('A', 83, int)
#define AFIOC_X_OIS_ACTSTROKELIMIT  _IOWR('A', 84, int)
#define AFIOC_X_OIS_DEPENDENCY_INIT _IOWR('A', 85, int)
#define AFIOC_X_OIS_GETOSCD_INPUT   _IOWR('A', 86, struct ois_oscd_input)
#define AFIOC_X_OIS_SET_LOOPGAIN    _IOWR('A', 87, int)
#define AFIOC_X_OIS_END             _IOWR('A', 88, int)
#define AFIOC_X_OIS_SUPPORT         _IOWR('A', 100, int)
/*add end*/

#endif
