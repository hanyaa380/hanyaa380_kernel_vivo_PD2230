#ifndef __INTERACT_H
#define __INTERACT_H

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/reboot.h>
#include <linux/power/vivo/power_supply_lite.h>
#include <mt-plat/mtk_boot_common.h>
#ifdef CONFIG_VIVO_CHARGING_FOR_PD2226
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <mach/mtk_pmic.h>
#include <mach/mtk_battery_property.h>
#include <mt-plat/mtk_boot.h>
#endif

/*=============================================================================
 * PMIC IRQ ENUM define
 *=============================================================================
 */

/************************************************************
 *
 *   [quote function]
 *
 ***********************************************************/
#ifdef CONFIG_VIVO_CHARGING_FOR_PD2226
extern int pmic_get_auxadc_value(int list);
#endif
//extern int pmic_get_channel_value(u8 list);
//extern int pmic_is_bif_exist(void);

extern unsigned int upmu_get_rgs_chrdet(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
extern bool musb_is_host(void);
extern void kernel_power_off(void);
//extern void mt_power_off(void);
//extern int mtkts_bts_get_hw_temp(void);
//extern int get_pwrap_irq_cnt(void);
extern void charger_connect_judge(char on_or_off);
#if 1
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
#else
extern void dpdm_hiz_init(void);
extern void dpdm_hiz_release(void);
#endif



/************************************************************
 *
 *   [package function]
 *
 ***********************************************************/

extern unsigned int get_boot_mode_ex(void);
extern unsigned int get_boot_mode(void);
extern bool get_atboot(void);
extern bool get_atfirst(void);
extern bool get_atelsp1(void);
extern bool get_bsp_test_mode(void);
extern bool get_bsptest_battery(void);
extern bool get_normal_mode(void);
extern bool get_charging_mode(void);
extern bool get_meta_mode(void);
extern void charge_plug_notify(bool plugin);
extern void platform_usb_connect(bool connect);
extern void platform_dpdm_hiz(bool hiz);
extern bool platform_get_charge_detect(void);
extern int platform_get_charge_voltage(void);
extern int platform_get_charge_type(void);
extern bool platform_otg_host_real(void);
extern void platform_power_off(void);
extern void platform_register_interrupt_callback(void (EINT_FUNC_PTR) (void));



/************************************************************
 *
 *   [export function]
 *
 ***********************************************************/
extern void psl_usb_plug(void);
extern void wake_up_bat(void);
extern void config_otg_mode(bool enable);
extern bool is_power_path_supported(void);
extern int battery_get_capacity(void);
extern int battery_get_ibat(void);
extern int battery_get_vbat(void);
extern int battery_get_tbat(void);
extern int battery_get_vchg(void);
extern bool upmu_is_chr_det(void);
extern void vote_typec_drp(const char *voter, bool drp);
extern const char *is_vote_typec_drp(void);
extern void usboe_power_enable(bool enable);
extern bool usboe_is_enable(void);



#endif/* #ifndef __INTERACT_H */
