// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include "kd_imgsensor.h"


#include "imgsensor_hw.h"
#include "imgsensor_cfg_table.h"

/* Legacy design */
struct IMGSENSOR_HW_POWER_SEQ sensor_power_sequence[] = {
#if defined(CONFIG_MTK_CAM_PD2282)
#if defined(IMX766PD2282_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX766PD2282_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 2},
			{AVDD1, Vol_1800, 2},
			{AFVDD_DEF, Vol_2800, 1},
			{DVDD, Vol_1150, 2},
			{OISAVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(S5KJN1SQ03PD2282_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2282_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 7},
			{AFVDD_DEF, Vol_2800, 2},
			{RST, Vol_High, 3},
			{SensorMCLK, Vol_High, 10},
		},
	},
#endif
#if defined(OV08D10PD2282_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV08D10PD2282_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 6},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 8}
		},
	},
#endif
#endif
#if defined(CONFIG_MTK_CAM_PD2279)
#if defined(S5KJNSPD2279_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJNSPD2279_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 0},
			{AVDD1, Vol_High, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 10},
			{AFVDD_DEF, Vol_2800, 2},
		},
	},
#endif
#if defined(S5KJNSV1PD2279_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJNSV1PD2279_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1050, 1},
			{AVDD, Vol_2800, 0},
			{AVDD1, Vol_High, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 10},
			{AFVDD_DEF, Vol_2800, 2},
		},
	},
#endif
#if defined(IMX355PD2279_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355PD2279_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{AVDD1, Vol_High, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#endif
/* End add for PD2279 camera by lq */
#if defined(CONFIG_MTK_CAM_PD2279F)
#if defined(S5KJN1SQPD2279F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQPD2279F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1100, 0},
			{DVDD1, Vol_1000, 2},
			{AVDD, Vol_2800, 7,},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			{AFVDD_DEF, Vol_2800, 2},
		},
	},
#endif
#if defined(S5K3P9SP04PD2279F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP04PD2279F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 0},
			{DVDD1, Vol_1000, 1},
			{AVDD, Vol_2800, 0},
			{AVDD1, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(GC02M1BPD2279F_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1BPD2279F_MIPI_MONO,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1100, 0},
			{DVDD1, Vol_1000, 2},
			{AVDD, Vol_2800, 0},
			{AVDD1, Vol_2800, 7},
			{SensorMCLK, Vol_High,2},
			{RST, Vol_High, 3},
		},
	},
#endif
#if defined(GC02M1BPD2279HF_MIPI_MONO)
		{
			SENSOR_DRVNAME_GC02M1BPD2279HF_MIPI_MONO,
			{
				{I2C, Vol_High,1},
				{RST, Vol_Low, 1},
				{DOVDD, Vol_1800, 2},
				{DVDD, Vol_1100, 0},
				{DVDD1, Vol_1000, 2},
				{AVDD, Vol_2800, 0},
				{AVDD1, Vol_2800, 7},
				{SensorMCLK, Vol_High,2},
				{RST, Vol_High, 3},
			},
		},
#endif
#if defined(S5KJNSPD2279F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJNSPD2279F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1100, 2},	//gpio59
			{AVDD, Vol_2800, 2,},	//gpio55
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 10},
			{AFVDD_DEF, Vol_2800, 2},
		},
	},
#endif
#if defined(IMX355PD2279F_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355PD2279F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 0},	//gpio59
			{DVDD1, Vol_1200, 2},	//vcn13
			{AVDD, Vol_2800, 0},	//gpio55
			{AVDD1, Vol_2800, 1},	//gpio53
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif

#endif
/* Start add for PD2257F camera by hushufen */
#if defined(CONFIG_MTK_CAM_PD2257F_EX)
#if defined(S5KGW3SP13PD2257F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW3SP13PD2257F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 2},
			{OISAVDD, Vol_2800, 7},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5K3P9SP04PD2257F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP04PD2257F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3},
		},
	},
#endif
#if defined(GC02M1PD2257F_MIPI_RAW)
{
		SENSOR_DRVNAME_GC02M1PD2257F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 3},
		},
},
#endif
#if defined(GC02M1BPD2257F_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1BPD2257F_MIPI_MONO,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 3},
		},
	},
#endif
#if defined(GC02M1BZPD2257F_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1BZPD2257F_MIPI_MONO,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 3},
		},
	},
#endif
#endif
/* End add for PD2257F camera by hushufen */
/* Start add for PD2250 camera by sunrey */
#if defined(CONFIG_MTK_CAM_PD2250)
#if defined(OV13B10SUNPD2250_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10SUNPD2250_MIPI_RAW,
			{
			{SensorMCLK, Vol_High,1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{RST, Vol_Low, 2},
			{RST, Vol_High, 2},
			{AFVDD, Vol_2800, 1},
		},
	},
#endif
#if defined(OV13B10QTEPD2250_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10QTEPD2250_MIPI_RAW,
			{
			{SensorMCLK, Vol_High,1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{RST, Vol_Low, 2},
			{RST, Vol_High, 2},
			{AFVDD, Vol_2800, 1},
		},
	},
#endif
#if defined(S5K5E9PD2250_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E9PD2250_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 4},
			{SensorMCLK, Vol_High, 9},
			{RST, Vol_High, 9},
		},
	},
#endif
#if defined(GC02M1PD2250_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M1PD2250_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 2},
		},
	},
#endif
#endif
/* End add for PD2250 camera by sunrey */
/* Start add for PD2250F camera by sunrey */
#if defined(CONFIG_MTK_CAM_PD2250F)
#if defined(S5KJN1SQPD2250F_SENSOR_ID)
	{
		SENSOR_DRVNAME_S5KJN1SQPD2250F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 7},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			//{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(HI1634QPD2250F_MIPI_RAW)
	{
		SENSOR_DRVNAME_HI1634QPD2250F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1100, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 1}

		},
	},
#endif
#if defined(GC02M1PD2250F_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M1PD2250F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 2},
		},
	},
#endif
#endif
/* End add for PD2250F camera by sunrey */

#if defined(CONFIG_MTK_CAM_PD2230)
#if defined(OV13B10PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{SensorMCLK, Vol_High,1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{DVDD1, Vol_1200, 1},
			{RST, Vol_Low, 2},
			{RST, Vol_High, 6},
			{AFVDD_DEF, Vol_2800, 3},
		},
	},
#endif
#if defined(OV13B10V1PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10V1PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{SensorMCLK, Vol_High,1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{DVDD1, Vol_1200, 1},
			{RST, Vol_Low, 2},
			{RST, Vol_High, 6},
			{AFVDD_DEF, Vol_2800, 3},
		},
	},
#endif
#if defined(OV13B10V2PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10V2PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{SensorMCLK, Vol_High,1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 1},
			{DVDD1, Vol_1200, 1},
			{RST, Vol_Low, 2},
			{RST, Vol_High, 6},
			{AFVDD_DEF, Vol_2800, 3},
		},
	},
#endif
#if defined(S5K5E9PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E9PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 4},
			{SensorMCLK, Vol_High, 9},
			{RST, Vol_High, 9},
		},
	},
#endif
#if defined(GC02M1PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M1PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 3},
		},
	},
#endif
#if defined(OV02B10PD2230_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2230_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 6},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 9}
		},
	},
#endif
#endif

#if defined(CONFIG_MTK_CAM_PD2230F)
#if defined(S5KJN1SQPD2230F_SENSOR_ID)
	{
		SENSOR_DRVNAME_S5KJN1SQPD2230F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 7, Vol_Low, 15},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			{AFVDD_DEF, Vol_2800, 2},
		},
	},
#endif
#if defined(HI1634QPD2230F_MIPI_RAW)
	{
		SENSOR_DRVNAME_HI1634QPD2230F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1100, 1},
			{DVDD1, Vol_1100, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}

		},
	},
#endif
#if defined(GC02M1BPD2230F_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1BPD2230F_MIPI_MONO,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High,1},
			{RST, Vol_High, 3},
		},
	},
#endif
#if defined(IMX355PD2230F_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355PD2230F_MIPI_RAW,
		{
			{I2C, Vol_High,1},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{DVDD1, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#endif
/*vivo add by zhangyiteng for PD2224(PD2166G) start*/
#if defined(CONFIG_MTK_CAM_PD2166G)
#if defined(S5KJN1SQ03PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_S5KJN1SQ03PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{DVDD, Vol_1050, 1},
		{AVDD, Vol_2800, 8},
		{SensorMCLK, Vol_High, 2},
		{RST, Vol_High, 3},
		{AFVDD_DEF, Vol_2800, 2},
	},
},
#endif
#if defined(OV13B10PD2224_MIPI_RAW)
{
	SENSOR_DRVNAME_OV13B10PD2224_MIPI_RAW,
	{
		{SensorMCLK, Vol_High,1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
		{RST, Vol_Low, 1, Vol_Low, 2},
		{RST, Vol_High, 2},
		{AFVDD_DEF, Vol_2800, 1},
	},
},
#endif
#if defined(OV13B10V1PD2224_MIPI_RAW)
{
	SENSOR_DRVNAME_OV13B10V1PD2224_MIPI_RAW,
	{
		{SensorMCLK, Vol_High,1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 1},
		{DVDD, Vol_1200, 1},
		{RST, Vol_Low, 1, Vol_Low, 2},
		{RST, Vol_High, 2},
		{AFVDD_DEF, Vol_2800, 1},
	},
},
#endif
#if defined(S5K4H7PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 10},
		{DVDD, Vol_1200, 1},
        {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(S5K4H7V1PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7V1PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 10},
		{DVDD, Vol_1200, 1},
        {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(S5K4H7V2PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7V2PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 10},
		{DVDD, Vol_1200, 1},
        {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(S5K4H7V3PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_S5K4H7V3PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{SensorMCLK, Vol_High, 1},
		{AVDD, Vol_2800, 10},
		{DVDD, Vol_1200, 1},
        {DOVDD, Vol_1800, 1},
		{RST, Vol_High, 5}
	},
},
#endif
#if defined(GC02M1PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_GC02M1PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 10},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
#if defined(GC02M1V1PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_GC02M1V1PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 10},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
#if defined(GC02M1V2PD2166G_MIPI_RAW)
{
	SENSOR_DRVNAME_GC02M1V2PD2166G_MIPI_RAW,
	{
		{RST, Vol_Low, 1},
		{DOVDD, Vol_1800, 1},
		{AVDD, Vol_2800, 10},
		{SensorMCLK, Vol_High, 1},
		{RST, Vol_High, 2}
	},
},
#endif
#endif
/*vivo add by zhangyiteng for PD2224(PD2166G) end*/
#if defined(CONFIG_MTK_CAM_PD2204F_EX)
#if defined(S5KGW1SD03PD2204F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW1SD03PD2204F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1000, 2},
			{AFVDD_DEF, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 2},
			{OISAVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(S5KGD2SMPD2204F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGD2SMPD2204F_MIPI_RAW,
		{
			{RST, Vol_Low, 8},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD_DEF, Vol_2800, 1},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(OV8856PD2204F_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8856PD2204F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV02B10PD2204F_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10PD2204F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{AVDD, Vol_2800, 5},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 9}
		},
	},
#endif
#endif


#if defined(IMX766_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX766_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 3},
#ifdef CONFIG_REGULATOR_RT5133
			{AVDD1, Vol_1800, 0},
#endif
			{AFVDD, Vol_2800, 3},
			{DVDD, Vol_1100, 4},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 6},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5KJD1_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJD1_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DVDD, Vol_1100, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{PDN, Vol_High, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX586_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX586_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
#ifdef CONFIG_REGULATOR_RT5133
			{AVDD1, Vol_1800, 0},
#endif
#if defined(IMGSENSOR_MT6781) || defined(IMGSENSOR_MT6877)
			{AFVDD, Vol_2800, 0},
#endif
			{DVDD, Vol_1100, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 3}
		},
	},
#endif
#if defined(OV48B_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV48B_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 5},
			//{AFVDD, Vol_2800, 2},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K3P9SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P9SP_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 0},
			{SensorMCLK, Vol_High, 0},
			//{AFVDD, Vol_2800, 5},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(IMX319_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX319_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K3M5SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M5SX_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DVDD, Vol_1100, 1},
			{AVDD, Vol_2800, 1},
#if defined(IMGSENSOR_MT6781) || defined(IMGSENSOR_MT6877)
			{AFVDD, Vol_2800, 0},
#endif
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 2},
			{SensorMCLK, Vol_High, 1}
		},
	},
#endif
#if defined(GC02M0_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC02M0_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX519_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX519_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX519DUAL_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX519DUAL_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{AFVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX499_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX499_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(IMX481_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX481_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
#ifdef CONFIG_REGULATOR_RT5133
			//To trigger ex-LDO output 2.8V
			{AVDD, Vol_1800, 0},
#else
			// PMIC output 2.8V
			{AVDD, Vol_2800, 0},
#endif
			{DOVDD, Vol_1800, 0},
#ifdef CONFIG_REGULATOR_RT5133
			//To trigger ex-LDO output 1.1V
			{DVDD, Vol_1800, 0},
#else
			//PMIC output 1.1V
			{DVDD, Vol_1100, 0},
#endif
#if defined(IMGSENSOR_MT6781) || defined(IMGSENSOR_MT6877)
			{AFVDD, Vol_2800, 1},
#endif
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(IMX576_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX576_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 1}, /*data sheet 1050*/
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 8}
		},
	},
#endif
#if defined(IMX350_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX350_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 5},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX398_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX398_MIPI_RAW,
		{
			{SensorMCLK, Vol_Low, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 1},
		},
	},
#endif
#if defined(OV23850_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV23850_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(OV16885_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16885_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_High, 2},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(OV05A20_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV05A20_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 5},
				{DVDD, Vol_1200, 5},
				{RST, Vol_High, 15}
			},
		},
#endif

#if defined(IMX386_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX386_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif
#if defined(IMX386_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX386_MIPI_MONO,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10},
		},
	},
#endif
#if defined(IMX376_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX376_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(IMX338_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX338_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2500, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1100, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K2LQSX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2LQSX_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 4},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K4H7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4H7_MIPI_RAW,
		{
			{PDN, Vol_Low, 1},
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 4},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K4E6_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4E6_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2900, 0},
			{DVDD, Vol_1200, 2},
			{AFVDD, Vol_2800, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K2T7SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2T7SP_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{SensorMCLK, Vol_High, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(S5K3P8SP_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P8SP_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K3P8SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P8SX_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{SensorMCLK, Vol_High, 1},
			{DVDD, Vol_1000, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(S5K3M2_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M2_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3P3SX_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K5E2YA_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K4ECGX_MIPI_YUV)
	{
		SENSOR_DRVNAME_S5K4ECGX_MIPI_YUV,
		{
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV16880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(S5K2P7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1000, 1},
			{DOVDD, Vol_1800, 1},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0},
		},
	},
#endif
#if defined(S5K2P8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 4},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 1},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX258_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX258_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX258_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX258_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX377_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX377_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(OV8858_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8858_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV8856_MIPI_RAW)
	{SENSOR_DRVNAME_OV8856_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 2},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 5},
		},
	},
#endif
#if defined(S5K2X8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2X8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX214_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX214_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX214_MIPI_MONO)
	{
		SENSOR_DRVNAME_IMX214_MIPI_MONO,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX230_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX230_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3L8_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3L8_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(IMX362_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX362_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1200, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K2L7_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K2L7_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 3},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX318_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX318_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{AVDD, Vol_2800, 0},
			{DOVDD, Vol_1800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 5},
			{SensorMCLK, Vol_High, 5},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV8865_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV8865_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(IMX219_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX219_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 10},
			{DOVDD, Vol_1800, 10},
			{DVDD, Vol_1000, 10},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_Low, 0},
			{PDN, Vol_High, 0},
			{RST, Vol_Low, 0},
			{RST, Vol_High, 0}
		},
	},
#endif
#if defined(S5K3M3_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K3M3_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1000, 0},
			{AFVDD, Vol_2800, 1},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV5670_MIPI_RAW_2)
	{
		SENSOR_DRVNAME_OV5670_MIPI_RAW_2,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 5},
			{RST, Vol_Low, 5},
			{DOVDD, Vol_1800, 5},
			{AVDD, Vol_2800, 5},
			{DVDD, Vol_1200, 5},
			{AFVDD, Vol_2800, 5},
			{PDN, Vol_High, 5},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV2281_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV2281_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{RST, Vol_Low, 10},
			{RST, Vol_High, 5},
			{PDN, Vol_Low, 5},
			{PDN, Vol_High, 5},
		},
	},
#endif
#if defined(OV20880_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV20880_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1100, 1},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(OV5645_MIPI_YUV)
	{
		SENSOR_DRVNAME_OV5645_MIPI_YUV,
		{
			{SensorMCLK, Vol_High, 0},
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 0},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 5},
			{PDN, Vol_High, 1},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5K5E9_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K5E9_MIPI_RAW,
		{
			{PDN, Vol_Low, 0},
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{AFVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 2},
			{PDN, Vol_High, 0},
			{RST, Vol_High, 10}
		},
	},
#endif
#if defined(S5KGD1SP_MIPI_RAW)
		{
			SENSOR_DRVNAME_S5KGD1SP_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1100, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(HI846_MIPI_RAW)
		{
			SENSOR_DRVNAME_HI846_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(GC02M0_MIPI_RAW)
		{
			SENSOR_DRVNAME_GC02M0_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(OV02A10_MIPI_MONO)
		{
			SENSOR_DRVNAME_OV02A10_MIPI_MONO,
			{
				{RST, Vol_High, 1},
				{AVDD, Vol_2800, 0},
			/*main3 has no dvdd, compatible with sub2*/
				{DVDD, Vol_1200, 0},
				{DOVDD, Vol_1800, 0},
				{SensorMCLK, Vol_High, 5},
				{RST, Vol_Low, 9}
			},
		},
#endif
#if defined(IMX686_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX686_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2900, 0},
		/*in alph.dts file, pin avdd controls two gpio pins*/
			/*{AVDD_1, Vol_1800, 0},*/
			{DVDD, Vol_1100, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 1}
		},
	},
#endif
#if defined(IMX616_MIPI_RAW)
		{
			SENSOR_DRVNAME_IMX616_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{AVDD, Vol_2900, 0},
				{DVDD, Vol_1100, 0},
				{DOVDD, Vol_1800, 1},
				{SensorMCLK, Vol_High, 1},
				{RST, Vol_High, 2}
			},
		},
#endif
#if defined(IMX355_MIPI_RAW)
	{
		SENSOR_DRVNAME_IMX355_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV13B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 5},
			{SensorMCLK, Vol_High, 1},
		},
	},
#endif
#if defined(OV48C_MIPI_RAW)
		{
			SENSOR_DRVNAME_OV48C_MIPI_RAW,
			{
				{RST, Vol_Low, 1},
				{SensorMCLK, Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 5},
				{RST, Vol_High, 5},
			},
		},
#endif
#if defined(OV16A10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV16A10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{AVDD, Vol_2800, 0},
			{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2},
		},
	},
#endif
#if defined(GC8054_MIPI_RAW)
	{
		SENSOR_DRVNAME_GC8054_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 0},
			{RST, Vol_Low,  1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_1200, 1},
			{AVDD, Vol_2800, 1},
			{RST, Vol_High, 1},
			//{AFVDD, Vol_Low, 5}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M1B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02M1B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 0},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO1)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO1,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02M0B_MIPI_MONO2)
	{
		SENSOR_DRVNAME_GC02M0B_MIPI_MONO2,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(GC02K0B_MIPI_MONO)
	{
		SENSOR_DRVNAME_GC02K0B_MIPI_MONO,
		{
			{RST, Vol_Low, 1},
			//{DVDD, Vol_1200, 0},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 0},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 2}
		},
	},
#endif
#if defined(OV02B10_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV02B10_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{SensorMCLK, Vol_High, 0},
			{AVDD, Vol_2800, 9},
			{RST, Vol_High, 1}
		},
	},
#endif
/*vivo add by zhangyiteng for PD2215 camera bringup 2022.04.21 start */
#if defined(S5KGW3SP13PD2215F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KGW3SP13PD2215F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 2},
			{SensorMCLK, Vol_High, 2},
			{OISAVDD, Vol_2800, 7},
			{RST, Vol_High, 5}
		},
	},
#endif
#if defined(S5KJN1SQ03PD2215F_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5KJN1SQ03PD2215F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 2},
			{DVDD, Vol_1050, 2},
			{AVDD, Vol_2800, 7},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3},
			//{AFVDD, Vol_2800, 2},
		},
	},
#endif
#if defined(HI846PD2215F_MIPI_RAW)
{
		SENSOR_DRVNAME_HI846PD2215F_MIPI_RAW,
		{
			{RST, Vol_Low, 2},
			{AVDD, Vol_2800, 2},
			{DVDD, Vol_1200, 2},
			{DOVDD, Vol_1800, 2},
			{SensorMCLK, Vol_High, 2},
			{RST, Vol_High, 3}
		},
},
#endif
#if defined(GC02M1PD2215F_MIPI_RAW)
{
		SENSOR_DRVNAME_GC02M1PD2215F_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{DOVDD, Vol_1800, 1},
			{AVDD, Vol_2800, 1},
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_High, 5},
		},
},
#endif
/*vivo add by zhangyiteng for PD2215 camera bringup 2022.04.21 end */
/* Onyx */
#if defined(OV13B10LN_MIPI_RAW)
	{
		SENSOR_DRVNAME_OV13B10LN_MIPI_RAW,
		{
			{SensorMCLK, Vol_High, 1},
			{RST, Vol_Low, 1},
			{AVDD, Vol_High, 1},
			{PDN, Vol_High, 1},
			{DOVDD, Vol_1800, 1},
			{DVDD, Vol_High, 5},
			{RST, Vol_High, 1},
		},
	},
#endif

#if defined(S5K4H7LN_MIPI_RAW)
	{
		SENSOR_DRVNAME_S5K4H7LN_MIPI_RAW,
		{
			{RST, Vol_Low, 1},
			{SensorMCLK, Vol_High, 5},
			{AVDD, Vol_High, 1},
			{DVDD, Vol_High, 1},
			{DOVDD, Vol_1800, 1},
			{RST, Vol_High, 1}
		},
	},
#endif
	/* add new sensor before this line */
	{NULL,},
};

