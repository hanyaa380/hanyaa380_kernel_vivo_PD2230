/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/string.h>

#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"

#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"

#ifdef CONFIG_CAM_I2C_PULLEN_CONTROL
#define MAX_CAM_I2C_NUMBER 12  //相机使用的I2C总线数，等于相机数x2，为了减少内存使用并且一般项目不会超过6颗模组所以设置为12
unsigned int gpio_nums = 0;
unsigned int gpios[MAX_CAM_I2C_NUMBER] = {0};
#endif

/*the index is consistent with enum IMGSENSOR_HW_PIN*/
char * const imgsensor_hw_pin_names[] = {
	"none",
	"pdn",
	"rst",
	"vcama",
#ifdef CONFIG_REGULATOR_RT5133
	"vcama1",
#endif
#if defined(IMGSENSOR_MT6781) || defined(IMGSENSOR_MT6877)
	"vcamaf",
#endif
	"vcamaf_def",
	"vcamd",
	"vcamoisa",
	"vcamio",
#ifdef MIPI_SWITCH
	"mipi_switch_en",
	"mipi_switch_sel",
#endif
	"mclk"
};

/*the index is consistent with enum IMGSENSOR_HW_ID*/
char * const imgsensor_hw_id_names[] = {
	"mclk",
	"regulator",
	"gpio"
};

enum IMGSENSOR_RETURN imgsensor_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info;
	unsigned int i, j, len;
	char str_prop_name[LENGTH_FOR_SNPRINTF];
	const char *pin_hw_id_name;
	struct device_node *of_node
		= of_find_compatible_node(NULL, NULL, "mediatek,imgsensor");

	mutex_init(&phw->common.pinctrl_mutex);

#ifdef CONFIG_CAM_I2C_PULLEN_CONTROL
	if (of_property_read_u32(of_node, "gpio_nums", &gpio_nums)) {
		PK_DBG("imgsensor_hw_init gpio_nums get failed\n");
	}
	if (of_property_read_u32_array(of_node, "gpios", gpios, gpio_nums)) {
		PK_DBG("imgsensor_hw_init gpios get failed\n");
	}
#endif

	/* update the imgsensor_custom_cfg by dts */
	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		PK_DBG("IMGSENSOR_SENSOR_IDX: %d\n", i);
		pcust_pwr_cfg = imgsensor_custom_config;
		while (pcust_pwr_cfg->sensor_idx != i &&
		       pcust_pwr_cfg->sensor_idx != IMGSENSOR_SENSOR_IDX_NONE)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			memset(str_prop_name, 0, sizeof(str_prop_name));
			snprintf(str_prop_name,
				sizeof(str_prop_name),
				"cam%d_pin_%s",
				i,
				imgsensor_hw_pin_names[ppwr_info->pin]);
			if (of_property_read_string(
				of_node, str_prop_name,
				&pin_hw_id_name) == 0) {
				for (j = 0; j < IMGSENSOR_HW_ID_MAX_NUM; j++) {
					len = strlen(imgsensor_hw_id_names[j]);
					if (strncmp(pin_hw_id_name, imgsensor_hw_id_names[j], len)
						== 0) {
						PK_DBG(
							"imgsensor_hw_cfg hw_pin:%s, id name:%s, id:%d\n",
							str_prop_name, pin_hw_id_name, j);
						ppwr_info->id = j;
						break;
					}
				}
			}
			ppwr_info++;
		}
	}
	/* update the imgsensor_custom_cfg by dts END */

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i]) (&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(
				phw->pdev[i]->pinstance, &phw->common);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

		pcust_pwr_cfg = imgsensor_custom_config;
		while (pcust_pwr_cfg->sensor_idx != i &&
		       pcust_pwr_cfg->sensor_idx != IMGSENSOR_SENSOR_IDX_NONE)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0;
				j < IMGSENSOR_HW_ID_MAX_NUM &&
					ppwr_info->id != phw->pdev[j]->id;
				j++) {
			}

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		memset(str_prop_name, 0, sizeof(str_prop_name));
		snprintf(str_prop_name,
					sizeof(str_prop_name),
					"cam%d_%s",
					i,
					"enable_sensor");
		if (of_property_read_string(
			of_node, str_prop_name,
			&phw->enable_sensor_by_index[i]) < 0) {
			PK_DBG("Property cust-sensor not defined\n");
			phw->enable_sensor_by_index[i] = NULL;
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_release_all(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->release != NULL)
			(phw->pdev[i]->release)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

#ifdef CONFIG_CAM_I2C_PULLEN_CONTROL
static void imgsensor_set_i2c_pull(struct IMGSENSOR_HW *phw,enum   IMGSENSOR_SENSOR_IDX sensor_idx, enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	const struct mtk_pin_desc *desc = NULL;
	int ret = 0;
	unsigned int i = 0;
	if(NULL == phw->hw)
		return;

	if(2 * sensor_idx < gpio_nums)
	{
		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
			for(i = 0; i < 2; i++) {
				desc = (const struct mtk_pin_desc *)&phw->hw->soc->pins[gpios[2 * sensor_idx + i]];
				ret |= phw->hw->soc->bias_set_combo(phw->hw, desc, 1, 1);
			}
		}
		else if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
			for(i = 0; i < 2; i++) {
				desc = (const struct mtk_pin_desc *)&phw->hw->soc->pins[gpios[2 * sensor_idx + i]];
				ret |= phw->hw->soc->bias_set_combo(phw->hw, desc, 0, 0);
			}
		}
	}
}
#endif

static enum IMGSENSOR_RETURN imgsensor_hw_power_sequence(
		struct IMGSENSOR_HW             *phw,
		enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
		enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
		struct IMGSENSOR_HW_POWER_SEQ   *ppower_sequence,
		char *pcurr_idx)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr =
					&phw->sensor_pwr[sensor_idx];
	struct IMGSENSOR_HW_POWER_SEQ    *ppwr_seq = ppower_sequence;
	struct IMGSENSOR_HW_POWER_INFO   *ppwr_info;
	struct IMGSENSOR_HW_DEVICE       *pdev;
	int                               pin_cnt = 0;

	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 30);

#ifdef CONFIG_FPGA_EARLY_PORTING  /*for FPGA*/
	if (1) {
		PK_DBG("FPGA return true for power control\n");
		return IMGSENSOR_RETURN_SUCCESS;
	}
#endif

	while (ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		ppwr_seq->name != NULL) {
		if (!strcmp(ppwr_seq->name, PLATFORM_POWER_SEQ_NAME)) {
			if (sensor_idx == ppwr_seq->_idx)
				break;
		} else {
			if (!strcmp(ppwr_seq->name, pcurr_idx))
				break;
		}
		ppwr_seq++;
	}

	if (ppwr_seq->name == NULL)
		return IMGSENSOR_RETURN_ERROR;

	ppwr_info = ppwr_seq->pwr_info;

	while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE &&
	       ppwr_info->pin < IMGSENSOR_HW_PIN_MAX_NUM &&
	       ppwr_info < ppwr_seq->pwr_info + IMGSENSOR_HW_POWER_INFO_MAX) {

		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				phw->pdev[psensor_pwr->id[ppwr_info->pin]];

				if (__ratelimit(&ratelimit))
					pr_info(
					"sensor_idx %d, ppwr_info->pin %d, ppwr_info->pin_state_on %d, delay %u",
					sensor_idx,
					ppwr_info->pin,
					ppwr_info->pin_state_on,
					ppwr_info->pin_on_delay);

#ifdef CONFIG_CAM_I2C_PULLEN_CONTROL
				if (ppwr_info->pin == IMGSENSOR_HW_PIN_I2C) {
					imgsensor_set_i2c_pull(phw, sensor_idx, pwr_status);
					mdelay(ppwr_info->pin_on_delay);
					ppwr_info++;
					pin_cnt++;
					continue;
				}
#endif

				if (pdev->set != NULL)
					pdev->set(pdev->pinstance,
					sensor_idx,
				    ppwr_info->pin, ppwr_info->pin_state_on);
			}

			mdelay(ppwr_info->pin_on_delay);
		}

		ppwr_info++;
		pin_cnt++;
	}

	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
		while (pin_cnt) {
			ppwr_info--;
			pin_cnt--;

			if (__ratelimit(&ratelimit))
				pr_info(
				"sensor_idx %d, ppwr_info->pin %d, ppwr_info->pin_state_off %d, delay %u",
				sensor_idx,
				ppwr_info->pin,
				ppwr_info->pin_state_off,
				ppwr_info->pin_on_delay);

			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				phw->pdev[psensor_pwr->id[ppwr_info->pin]];

#ifdef CONFIG_CAM_I2C_PULLEN_CONTROL
			if (ppwr_info->pin == IMGSENSOR_HW_PIN_I2C) {
				imgsensor_set_i2c_pull(phw, sensor_idx, pwr_status);
				continue;
			}
#endif

				if (pdev->set != NULL)
					pdev->set(pdev->pinstance,
					sensor_idx,
				ppwr_info->pin, ppwr_info->pin_state_off);
			}

#ifdef CONFIG_MTK_CAM_PD2230F
			if(sensor_idx == 0)
				mdelay(ppwr_info->pin_off_delay);
			else
				mdelay(ppwr_info->pin_on_delay);
#else
			mdelay(ppwr_info->pin_on_delay);
#endif
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_power(
		struct IMGSENSOR_HW *phw,
		struct IMGSENSOR_SENSOR *psensor,
		enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	int ret = 0;
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	char *curr_sensor_name = psensor->inst.psensor_list->name;
	char str_index[LENGTH_FOR_SNPRINTF];

	printk(KERN_ERR "sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[(uint32_t)sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[(uint32_t)sensor_idx]);

	if (phw->enable_sensor_by_index[(uint32_t)sensor_idx] &&
	!strstr(phw->enable_sensor_by_index[(uint32_t)sensor_idx], curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;

	ret = snprintf(str_index, sizeof(str_index), "%d", sensor_idx);
	if (ret < 0) {
		pr_info("Error! snprintf allocate 0");
		ret = IMGSENSOR_RETURN_ERROR;
		return ret;
	}
	imgsensor_hw_power_sequence(
			phw,
			sensor_idx,
			pwr_status,
			platform_power_sequence,
			str_index);

	imgsensor_hw_power_sequence(
			phw,
			sensor_idx,
			pwr_status, sensor_power_sequence, curr_sensor_name);

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_dump(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->dump != NULL)
			(phw->pdev[i]->dump)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

