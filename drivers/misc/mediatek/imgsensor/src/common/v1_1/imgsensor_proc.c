// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/string.h>

#include "imgsensor.h"
#include "imgsensor_proc.h"

char mtk_ccm_name[camera_info_size] = { 0 };
char mtk_i2c_dump[camera_info_size] = { 0 };

#define HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
static int pdaf_type_info_read(struct seq_file *m, void *v)
{
#define bufsz 512

	unsigned int len = bufsz;
	char pdaf_type_info[bufsz];

	struct SENSOR_FUNCTION_STRUCT *psensor_func =
	    gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN].pfunc;

	memset(pdaf_type_info, 0, 512);

	if (psensor_func == NULL)
		return 0;

	psensor_func->SensorFeatureControl(
	    SENSOR_FEATURE_GET_PDAF_TYPE,
	    pdaf_type_info,
	    &len);

	seq_printf(m, "%s\n", pdaf_type_info);
	return 0;
};

static int proc_SensorType_open(struct inode *inode, struct file *file)
{
	return single_open(file, pdaf_type_info_read, NULL);
};

static ssize_t proc_SensorType_write(struct file *file,
					const char *buffer, size_t count,
					loff_t *data)
{
	char regBuf[64] = { '\0' };
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);

	struct SENSOR_FUNCTION_STRUCT *psensor_func =
	    gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN].pfunc;

	if (copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (psensor_func)
		psensor_func->SensorFeatureControl(
		    SENSOR_FEATURE_SET_PDAF_TYPE,
		    regBuf,
		   &u4CopyBufSize);

	return count;
};

/*******************************************************************************
 * CAMERA_HW_Reg_Debug()
 * Used for sensor register read/write by proc file
 *****************************************************************************/
static ssize_t CAMERA_HW_Reg_Debug(struct file *file,
				const char *buffer, size_t count,
				loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(
		regBuf, "%x %x", &sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
				sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}
	return count;
}


static ssize_t CAMERA_HW_Reg_Debug2(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_SUB];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
				sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}

	return count;
}

static ssize_t CAMERA_HW_Reg_Debug3(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN2];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
				sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}
	return count;
}

static ssize_t CAMERA_HW_Reg_Debug4(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_SUB2];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
					sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}

	return count;
}

static ssize_t CAMERA_HW_Reg_Debug5(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN3];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
					sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}

	return count;
}

static ssize_t CAMERA_HW_Reg_Debug6(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_SUB3];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
					sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}
	return count;
}

static ssize_t CAMERA_HW_Reg_Debug7(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_MAIN4];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
					sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}

	return count;
}

static ssize_t CAMERA_HW_Reg_Debug8(struct file *file, const char *buffer,
					size_t count, loff_t *data)
{
	char regBuf[64] = { '\0' };
	int ret = 0;
	u32 u4CopyBufSize =
		(count < (sizeof(regBuf) - 1)) ? (count) : (sizeof(regBuf) - 1);
	struct IMGSENSOR_SENSOR *psensor =
		&gimgsensor.sensor[IMGSENSOR_SENSOR_IDX_SUB4];

	MSDK_SENSOR_REG_INFO_STRUCT sensorReg;

	memset(&sensorReg, 0, sizeof(MSDK_SENSOR_REG_INFO_STRUCT));

	if (psensor == NULL || copy_from_user(regBuf, buffer, u4CopyBufSize))
		return -EFAULT;

	if (sscanf(regBuf, "%x %x",
			&sensorReg.RegAddr, &sensorReg.RegData) == 2) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_SET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("write addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr,
			sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			 sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	} else if (kstrtouint(regBuf, 16, &sensorReg.RegAddr) == 0) {
		imgsensor_sensor_feature_control(psensor,
						SENSOR_FEATURE_GET_REGISTER,
						(MUINT8 *) &sensorReg,
			(MUINT32 *) sizeof(MSDK_SENSOR_REG_INFO_STRUCT));
		PK_DBG("read addr = 0x%08x, data = 0x%08x\n",
					sensorReg.RegAddr, sensorReg.RegData);
		ret = snprintf(mtk_i2c_dump, sizeof(mtk_i2c_dump),
			"addr = 0x%08x, data = 0x%08x\n",
			sensorReg.RegAddr, sensorReg.RegData);
		if (ret < 0) {
			pr_info("Error! snprintf allocate 0");
			ret = IMGSENSOR_RETURN_ERROR;
			return ret;
		}
	}

	return count;
}

/* Camera information */
static int subsys_camera_info_read(struct seq_file *m, void *v)
{
	PK_DBG("%s %s\n", __func__, mtk_ccm_name);
	seq_printf(m, "%s\n", mtk_ccm_name);
	return 0;
};

static int subsys_camsensor_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", mtk_i2c_dump);
	return 0;
};

static int proc_camera_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, subsys_camera_info_read, NULL);
};

static int proc_camsensor_open(struct inode *inode, struct file *file)
{
	return single_open(file, subsys_camsensor_read, NULL);
};

static int imgsensor_proc_status_info_read(struct seq_file *m, void *v)
{
	char status_info[IMGSENSOR_STATUS_INFO_LENGTH];
	int ret = 0;

	ret = snprintf(status_info, sizeof(status_info), "ERR_L0, %x\n",
			*((uint32_t *)(&gimgsensor.status)));
	if (ret < 0) {
		pr_info("Error! snprintf allocate 0");
		ret = IMGSENSOR_RETURN_ERROR;
		return ret;
	}
	seq_printf(m, "%s\n", status_info);
	return 0;
};

static int imgsensor_proc_status_info_open(struct inode *inode,
						struct file *file)
{
	return single_open(file, imgsensor_proc_status_info_read, NULL);
};

static const struct file_operations fcamera_proc_fops1 = {
	.owner = THIS_MODULE,
	.open = proc_camera_info_open,
	.read = seq_read,
};

static const struct file_operations fcamera_proc_fops = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug
};

static const struct file_operations fcamera_proc_fops2 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug2
};

static const struct file_operations fcamera_proc_fops3 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug3
};

static const struct file_operations fcamera_proc_fops4 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug4
};

static const struct file_operations fcamera_proc_fops5 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug5
};

static const struct file_operations fcamera_proc_fops6 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug6
};

static const struct file_operations fcamera_proc_fops7 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug7
};

static const struct file_operations fcamera_proc_fops8 = {
	.owner = THIS_MODULE,
	.read = seq_read,
	.open = proc_camsensor_open,
	.write = CAMERA_HW_Reg_Debug8
};

static const struct file_operations fcamera_proc_fops_set_pdaf_type = {
	.owner = THIS_MODULE,
	.open = proc_SensorType_open,
	.read = seq_read,
	.write = proc_SensorType_write
};

static const struct file_operations fcamera_proc_fops_status_info = {
	.owner = THIS_MODULE,
	.open  = imgsensor_proc_status_info_open,
	.read = seq_read,
};

#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
/* hope add begin for store cam_thermal ratio information*/
static struct kobject cam_thermal_kobject;
u32 sensor_temperature[10];

static ssize_t  cam_thermal0_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	uint32_t temp = sensor_temperature[0];
	return snprintf(buf, 10, "%d\n", temp);
}

static ssize_t  cam_thermal1_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	uint32_t temp = sensor_temperature[1];
	return snprintf(buf, 10, "%d\n", temp);
}

static ssize_t  cam_thermal2_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	uint32_t temp = sensor_temperature[2];
	return snprintf(buf, 10, "%d\n", temp);
}

static ssize_t  cam_thermal3_show(struct kobject *kobj,
						struct kobj_attribute *attr, char *buf)
{
	uint32_t temp = sensor_temperature[3];
	return snprintf(buf, 10, "%d\n", temp);
}

static ssize_t cam_thermal_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}

static ssize_t cam_thermal_object_store(struct kobject *k, struct attribute *attr, const char *buf, size_t size)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, sizeof(buf));

	return ret;
}

static struct kobj_attribute cam_thermal0_attribute =
__ATTR(cam_thermal_sensor0, 0644, cam_thermal0_show, NULL);

static struct kobj_attribute cam_thermal1_attribute =
__ATTR(cam_thermal_sensor1, 0644, cam_thermal1_show, NULL);

static struct kobj_attribute cam_thermal2_attribute =
__ATTR(cam_thermal_sensor2, 0644, cam_thermal2_show, NULL);

static struct kobj_attribute cam_thermal3_attribute =
__ATTR(cam_thermal_sensor3, 0644, cam_thermal3_show, NULL);

static const struct sysfs_ops cam_thermal_sysfs_ops = {
	.show = cam_thermal_object_show,
	.store = cam_thermal_object_store,
};

static struct kobj_type cam_thermal_object_type = {
	.sysfs_ops = &cam_thermal_sysfs_ops,
	.release	= NULL,
};

static void cam_thermal_kobj_init(void)
{
	int32_t rc;
	memset(&cam_thermal_kobject, 0x00, sizeof(cam_thermal_kobject));

	if (kobject_init_and_add(&cam_thermal_kobject, &cam_thermal_object_type, NULL, "cam_thermal")) {
		kobject_put(&cam_thermal_kobject);
		return;
	}

	kobject_uevent(&cam_thermal_kobject, KOBJ_ADD);
	rc = sysfs_create_file(&cam_thermal_kobject, &cam_thermal0_attribute.attr);
	rc = sysfs_create_file(&cam_thermal_kobject, &cam_thermal1_attribute.attr);
	rc = sysfs_create_file(&cam_thermal_kobject, &cam_thermal2_attribute.attr);
	rc = sysfs_create_file(&cam_thermal_kobject, &cam_thermal3_attribute.attr);
}
/* hope add end for store cam_thermal ratio information*/
#endif
enum IMGSENSOR_RETURN imgsensor_proc_init(void)
{
	memset(mtk_ccm_name, 0, camera_info_size);

	proc_create("driver/camsensor", 0000, NULL, &fcamera_proc_fops);
	proc_create("driver/camsensor2", 0000, NULL, &fcamera_proc_fops2);
	proc_create("driver/camsensor3", 0000, NULL, &fcamera_proc_fops3);
	proc_create("driver/camsensor4", 0000, NULL, &fcamera_proc_fops4);
	proc_create("driver/camsensor5", 0000, NULL, &fcamera_proc_fops5);
	proc_create("driver/camsensor6", 0000, NULL, &fcamera_proc_fops6);
	proc_create("driver/camsensor7", 0000, NULL, &fcamera_proc_fops7);
	proc_create("driver/camsensor8", 0000, NULL, &fcamera_proc_fops8);
	proc_create("driver/pdaf_type", 0000, NULL,
				&fcamera_proc_fops_set_pdaf_type);
	proc_create("driver/imgsensor_status_info", 0000, NULL,
				&fcamera_proc_fops_status_info);

	/* Camera information */
	proc_create(PROC_CAMERA_INFO, 0000, NULL, &fcamera_proc_fops1);

#ifdef HOPE_ADD_FOR_CAM_TEMPERATURE_NODE
	cam_thermal_kobj_init();
#endif
	return IMGSENSOR_RETURN_SUCCESS;
}
