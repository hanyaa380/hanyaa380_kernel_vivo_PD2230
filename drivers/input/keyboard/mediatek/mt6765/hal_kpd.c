// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#ifdef CONFIG_MTK_PMIC_NEW_ARCH
#include <mt-plat/upmu_common.h>
#endif
#include <kpd.h>
#include <hal_kpd.h>
#include <mt-plat/mtk_boot_common.h>

#ifdef CONFIG_MTK_PMIC_NEW_ARCH
static int kpd_enable_lprst = 1;
#endif
static u16 kpd_keymap_state[KPD_NUM_MEMS] = {
	0xffff, 0xffff, 0xffff, 0xffff, 0x00ff
};

#ifdef CONFIG_LONG_PRESS_MODE_EN
struct timer_list Long_press_timer;
atomic_t vol_up_long_press_flag = ATOMIC_INIT(0);
atomic_t pow_key_long_press_flag = ATOMIC_INIT(0);
#endif

static void enable_kpd(int enable)
{
	if (enable == 1) {
		mt_reg_sync_writew((u16) (enable), KP_EN);
		kpd_print("KEYPAD is enabled\n");
	} else if (enable == 0) {
		mt_reg_sync_writew((u16) (enable), KP_EN);
		kpd_print("KEYPAD is disabled\n");
	}
}

void kpd_get_keymap_state(u16 state[])
{
	state[0] = readw(KP_MEM1);
	state[1] = readw(KP_MEM2);
	state[2] = readw(KP_MEM3);
	state[3] = readw(KP_MEM4);
	state[4] = readw(KP_MEM5);
	kpd_print(KPD_SAY "register = %x %x %x %x %x\n",
		state[0], state[1], state[2], state[3], state[4]);
}

void long_press_reboot_function_setting(void)
{
#ifdef CONFIG_MTK_PMIC_NEW_ARCH /*for pmic not ready*/
	if (kpd_enable_lprst && kpd_dts_data.boot_mode == NORMAL_BOOT) {
		kpd_info("Normal Boot long press reboot selection\n");
#ifdef CONFIG_KPD_PMIC_LPRST_TD
		kpd_info("Enable normal mode LPRST\n");
#ifdef CONFIG_ONEKEY_REBOOT_NORMAL_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD,
			CONFIG_KPD_PMIC_LPRST_TD);
#endif

#ifdef CONFIG_TWOKEY_REBOOT_NORMAL_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD,
			CONFIG_KPD_PMIC_LPRST_TD);
#endif
#else
		kpd_info("disable normal mode LPRST\n");
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);

#endif
	} else {
		kpd_info("Other Boot Mode long press reboot selection\n");
#ifdef CONFIG_KPD_PMIC_LPRST_TD
		kpd_info("Enable other mode LPRST\n");
#ifdef CONFIG_ONEKEY_REBOOT_OTHER_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD,
			CONFIG_KPD_PMIC_LPRST_TD);
#endif

#ifdef CONFIG_TWOKEY_REBOOT_OTHER_MODE
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x01);
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_TD,
			CONFIG_KPD_PMIC_LPRST_TD);
#endif
#else
		kpd_info("disable other mode LPRST\n");
		pmic_set_register_value(PMIC_RG_PWRKEY_RST_EN, 0x00);
		pmic_set_register_value(PMIC_RG_HOMEKEY_RST_EN, 0x00);
#endif
	}
#endif
}

/* FM@suspend */
bool __attribute__ ((weak)) ConditionEnterSuspend(void)
{
	return true;
}

/********************************************************************/
void kpd_wakeup_src_setting(int enable)
{
	int is_fm_radio_playing = 0;

	/* If FM is playing, keep keypad as wakeup source */
	if (ConditionEnterSuspend() == true)
		is_fm_radio_playing = 0;
	else
		is_fm_radio_playing = 1;

	if (is_fm_radio_playing == 0) {
		if (enable == 1) {
			kpd_print("enable kpd work!\n");
			enable_kpd(1);
		} else {
			kpd_print("disable kpd work!\n");
			enable_kpd(0);
		}
	}
}

/********************************************************************/
void kpd_init_keymap(u32 keymap[])
{
	int i = 0;

	if (kpd_dts_data.kpd_use_extend_type)
		kpd_keymap_state[4] = 0xffff;
	for (i = 0; i < KPD_NUM_KEYS; i++)
		keymap[i] = kpd_dts_data.kpd_hw_init_map[i];
}

void kpd_init_keymap_state(u16 keymap_state[])
{
	int i = 0;

	for (i = 0; i < KPD_NUM_MEMS; i++)
		keymap_state[i] = kpd_keymap_state[i];
		kpd_info("init_keymap_state done: %x %x %x %x %x!\n",
			keymap_state[0], keymap_state[1], keymap_state[2],
		 keymap_state[3], keymap_state[4]);
}

void kpd_set_debounce(u16 val)
{
	mt_reg_sync_writew((u16) (val & KPD_DEBOUNCE_MASK), KP_DEBOUNCE);
}

void kpd_double_key_enable(int en)
{
	u16 tmp;

	tmp = *(u16 *)KP_SEL;
	if (en != 0)
		writew((u16) (tmp | KPD_DOUBLE_KEY_MASK), KP_SEL);
	else
		writew((u16) (tmp & ~KPD_DOUBLE_KEY_MASK), KP_SEL);
}

#ifdef CONFIG_LONG_PRESS_MODE_EN
void vol_up_long_press(unsigned long pressed)
{
	atomic_set(&vol_up_long_press_flag, 1);
}
#endif
extern void kpd_aee_handler(u32 keycode, u16 pressed);
void kpd_pmic_rstkey_hal(unsigned long pressed)
{
	if (kpd_dts_data.kpd_sw_rstkey != 0) {
		kpd_aee_handler(kpd_dts_data.kpd_sw_rstkey, pressed);
		input_report_key(kpd_input_dev, kpd_dts_data.kpd_sw_rstkey,
				pressed);
		input_sync(kpd_input_dev);
		pr_err("[HW_KEY] keycode: %d state: %s using PMIC\n", kpd_dts_data.kpd_sw_rstkey, pressed ? "pressed" : "released");

#ifdef CONFIG_LONG_PRESS_MODE_EN
		if (pressed) {
			init_timer(&Long_press_timer);
			Long_press_timer.expires = jiffies + 5*HZ;
			Long_press_timer.data = (unsigned long)pressed;
			Long_press_timer.function = vol_up_long_press;
			add_timer(&Long_press_timer);
		} else {
			del_timer_sync(&Long_press_timer);
		}
		if (!pressed && atomic_read(&vol_up_long_press_flag))
			atomic_set(&vol_up_long_press_flag, 0);
#endif
	}
}

void kpd_pmic_pwrkey_hal(unsigned long pressed)
{
	input_report_key(kpd_input_dev, kpd_dts_data.kpd_sw_pwrkey, pressed);
	input_sync(kpd_input_dev);
	pr_err("[HW_KEY] keycode: %d state: %s using PMIC\n", kpd_dts_data.kpd_sw_pwrkey, pressed ? "pressed" : "released");
#ifdef CONFIG_LONG_PRESS_MODE_EN
	if (pressed && atomic_read(&vol_up_long_press_flag) &&
		atomic_read(&vol_down_long_press_flag)) {
		atomic_set(&pow_key_long_press_flag, 1);
	} else if (!pressed && atomic_read(&pow_key_long_press_flag)) {
		atomic_set(&pow_key_long_press_flag, 0);
		BUG_ON(1);
	}
#endif
}

static int mrdump_eint_state;
static int mrdump_ext_rst_irq;
static irqreturn_t mrdump_rst_eint_handler(int irq, void *data)
{
	if (mrdump_eint_state == 0) {
		irq_set_irq_type(mrdump_ext_rst_irq, IRQ_TYPE_LEVEL_HIGH);
		mrdump_eint_state = 1;
	} else {
		irq_set_irq_type(mrdump_ext_rst_irq, IRQ_TYPE_LEVEL_LOW);
		mrdump_eint_state = 0;
	}

	input_report_key(kpd_input_dev, KEY_RESTART, mrdump_eint_state);
	input_sync(kpd_input_dev);

	return IRQ_HANDLED;
}

void mt_eint_register(void)
{
	int ret;
	struct device_node *node;

	/* register EINT handler for MRDUMP_EXT_RST key */
	node = of_find_compatible_node(NULL, NULL,
		"mediatek, mrdump_ext_rst-eint");
	if (!node)
		kpd_print("can't find compatible node\n");
	else {

		mrdump_ext_rst_irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(mrdump_ext_rst_irq, mrdump_rst_eint_handler,
			IRQF_TRIGGER_NONE, "mrdump_ext_rst-eint", NULL);
		if (ret > 0)
			kpd_print("EINT IRQ LINE NOT AVAILABLE\n");
	}
}
