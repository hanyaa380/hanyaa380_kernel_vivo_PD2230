// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define LOG_TAG "DSI"

#include <linux/delay.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include "mt-plat/sync_write.h"
#include <debug.h>
#include "disp_drv_log.h"
#include "disp_drv_platform.h"
#include "mtkfb.h"
#include "ddp_drv.h"
#include "ddp_manager.h"
#include "ddp_dump.h"
#include "ddp_irq.h"
#include "ddp_dsi.h"
#include "ddp_log.h"
#include "ddp_mmp.h"
#include "disp_helper.h"
#include "ddp_reg.h"
#include "lcm_drv.h"
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#else
#include "disp_dts_gpio.h"
#endif

#include "ddp_clkmgr.h"
#include "primary_display.h"

#include <asm/arch_timer.h>
/*****************************************************************************/
enum MIPITX_PAD_VALUE {
	PAD_D2P_V = 0,
	PAD_D2N_V,
	PAD_D0P_V,
	PAD_D0N_V,
	PAD_CKP_V,
	PAD_CKN_V,
	PAD_D1P_V,
	PAD_D1N_V,
	PAD_D3P_V,
	PAD_D3N_V,
	PAD_NUM
};

#define DSI_OUTREG32(cmdq, addr, val) DISP_REG_SET(cmdq, addr, val)
#define DSI_BACKUPREG32(cmdq, hSlot, idx, addr) \
	DISP_REG_BACKUP(cmdq, hSlot, idx, addr)
#define DSI_POLLREG32(cmdq, addr, mask, value) \
	DISP_REG_CMDQ_POLLING(cmdq, addr, value, mask)
#define DSI_INREG32(type, addr) INREG32(addr)
#define DSI_READREG32(type, dst, src) mt_reg_sync_writel(INREG32(src), dst)

static int dsi_reg_op_debug;

#define BIT_TO_VALUE(TYPE, bit)  \
do { \
	TYPE r;\
	*(unsigned long *)(&r) = (0x00000000U);\
	r.bit = ~(r.bit);\
	r;\
} while (0)

#define DSI_MASKREG32(cmdq, REG, MASK, VALUE) \
	DISP_REG_MASK((cmdq), (REG), (VALUE), (MASK))

#define DSI_OUTREGBIT(cmdq, TYPE, REG, bit, value)  \
do {\
	TYPE r;\
	TYPE v;\
	if (cmdq) {\
		*(unsigned int *)(&r) = (0x00000000U); \
		r.bit = ~(r.bit);  \
		*(unsigned int *)(&v) = (0x00000000U); \
		v.bit = value; \
		DISP_REG_MASK(cmdq, &REG, AS_UINT32(&v), AS_UINT32(&r)); \
	} else { \
		mt_reg_sync_writel(INREG32(&REG), &r); \
		r.bit = (value); \
		DISP_REG_SET(cmdq, &REG, INREG32(&r)); \
	} \
} while (0)
/*****************************************************************************/

#ifdef CONFIG_FPGA_EARLY_PORTING
#define MIPITX_INREGBIT(addr, field) (0)

#define MIPITX_OUTREG32(addr, val) \
do { \
	if (dsi_reg_op_debug) \
		DDPMSG("[mipitx/reg]%p=0x%08x\n", (void *)addr, val); \
	if (0) \
		mt_reg_sync_writel(val, addr); \
} while (0)

#define MIPITX_OUTREGBIT(addr, field, value)  \
do {	\
	unsigned int val = 0; \
	if (0) \
		val = __raw_readl((unsigned long *)(addr)); \
	val = (val & ~REG_FLD_MASK(field)) | (REG_FLD_VAL((field), (value))); \
	MIPITX_OUTREG32(addr, val);   \
} while (0)

#else
#define MIPITX_INREGBIT(addr, field) DISP_REG_GET_FIELD(field, addr)

#define MIPITX_OUTREG32(addr, val) \
do {\
	if (dsi_reg_op_debug) {	\
		DDPMSG("[mipitx/wreg]%p=0x%08x\n", (void *)addr, val);\
	} \
	mt_reg_sync_writel(val, addr);\
} while (0)

#define MIPITX_OUTREGBIT(addr, field, value)  \
do {	\
	unsigned int val = 0; \
	val = __raw_readl((unsigned long *)(addr)); \
	val = (val & ~REG_FLD_MASK(field)) | (REG_FLD_VAL((field), (value))); \
	MIPITX_OUTREG32(addr, val);	  \
} while (0)
#endif

#define DSI_MODULE_to_ID(x)	(x == DISP_MODULE_DSI0 ? 0 : 1)
#define DIFF_CLK_LANE_LP (0x10)


/*****************************************************************************/
struct t_condition_wq {
	wait_queue_head_t wq;
	atomic_t condition;
};

struct t_dsi_context {
	unsigned int lcm_width; /* config dsi */
	unsigned int lcm_height;
	struct DSI_REGS regBackup; /* backup dsi */
	struct LCM_DSI_PARAMS dsi_params; /* config dsi */
	struct mutex lock; /* init dsi */
	int is_power_on; /* init dsi / suspend / resume */
	struct t_condition_wq cmddone_wq; /* init dsi */
	struct t_condition_wq read_wq; /* init dsi */
	struct t_condition_wq bta_te_wq; /* init dsi */
	struct t_condition_wq ext_te_wq; /* init dsi */
	struct t_condition_wq vm_done_wq; /* init dsi */
	struct t_condition_wq vm_cmd_done_wq; /* init dsi */
	struct t_condition_wq sleep_out_done_wq; /* init dsi */
	struct t_condition_wq sleep_in_done_wq; /* init dsi */
	unsigned int vsa;
	unsigned int vbp;
	unsigned int vfp;
	unsigned int hsa_byte;
	unsigned int hbp_byte;
	unsigned int hfp_byte;
	unsigned int hbllp_byte;

	//high frame rate
	unsigned int data_phy_cycle;
	unsigned int HS_TRAIL;

	/*DynFPS*/
	unsigned int disp_fps;
	unsigned int dynfps_chg_index;
};

struct t_dsi_context _dsi_context[DSI_INTERFACE_NUM];

struct DSI_REGS *DSI_REG[DSI_INTERFACE_NUM];
unsigned long DSI_PHY_REG[DSI_INTERFACE_NUM];
struct DSI_CMDQ_REGS *DSI_CMDQ_REG[DSI_INTERFACE_NUM];
struct DSI_VM_CMDQ_REGS *DSI_VM_CMD_REG[DSI_INTERFACE_NUM];

static int def_data_rate;
static int def_dsi_hbp;
static int dsi_currect_mode;
static int dsi_force_config;
static int dsi0_te_enable = 1;
static struct LCM_UTIL_FUNCS lcm_utils_dsi0;
static struct LCM_UTIL_FUNCS lcm_utils_dsi1;
static struct LCM_UTIL_FUNCS lcm_utils_dsidual;
static cmdqBackupSlotHandle _h_intstat;
static int mipi_clk_change_sta;
unsigned int impendance0[2] = { 0 }; /* MIPITX_DSI_IMPENDANCE0 */
unsigned int impendance1[2] = { 0 }; /* MIPITX_DSI_IMPENDANCE1 */
unsigned int impendance2[2] = { 0 }; /* MIPITX_DSI_IMPENDANCE2 */

unsigned int clock_lane[2] = { 0 }; /* MIPITX_DSI_CLOCK_LANE */
unsigned int data_lane0[2] = { 0 }; /* MIPITX_DSI_DATA_LANE0 */
unsigned int data_lane1[2] = { 0 }; /* MIPITX_DSI_DATA_LANE1 */
unsigned int data_lane2[2] = { 0 }; /* MIPITX_DSI_DATA_LANE2 */
unsigned int data_lane3[2] = { 0 }; /* MIPITX_DSI_DATA_LANE3 */

unsigned int mipitx_impedance_backup[50];
atomic_t dsi_idle_flg = ATOMIC_INIT(0);

static void refill_mipitx_impedance(void *cmdq)
{
	int i = 0, j = 0, cnt = 0;
	unsigned long addr = DSI_PHY_REG[0] + 0x100;

	for (i = 0; i < 5; i++) {
		for (j = 0; j < 4; j++) {
			DSI_OUTREG32(cmdq, addr,
				mipitx_impedance_backup[cnt]);
			cnt++;
			addr += 0x4;
		}
		addr += 0x4; /* 0x114 - 0x110 */
		for (j = 0; j < 4; j++) {
			DSI_OUTREG32(cmdq, addr,
				mipitx_impedance_backup[cnt]);
			cnt++;
			addr += 0x4;
		}
		addr += 0xA4; /* 0x1C8 - 0x124 */
		DSI_OUTREG32(cmdq, addr,
			mipitx_impedance_backup[cnt]);
		cnt++;
		addr += 0x4; /* 0x1CC - 0x1C8 */
		DSI_OUTREG32(cmdq, addr,
			mipitx_impedance_backup[cnt]);
		cnt++;
		addr += 0x34; /* 0x200 - 0x1CC */
	}
}

atomic_t PMaster_enable = ATOMIC_INIT(0);
static int DSI_MODULE_BEGIN(enum DISP_MODULE_ENUM module)
{
	if (module == DISP_MODULE_DSI0)
		return 0;
	else if (module == DISP_MODULE_DSI1)
		return 1;
	else if (module == DISP_MODULE_DSIDUAL)
		return 0;

	return 0;
}

static int DSI_MODULE_END(enum DISP_MODULE_ENUM module)
{
	if (module == DISP_MODULE_DSI0)
		return 0;
	else if (module == DISP_MODULE_DSI1)
		return 1;
	else if (module == DISP_MODULE_DSIDUAL)
		return 1;

	return 0;
}

static void _init_condition_wq(struct t_condition_wq *waitq)
{
	init_waitqueue_head(&(waitq->wq));
	atomic_set(&(waitq->condition), 0);
}

static void _set_condition_and_wake_up(struct t_condition_wq *waitq)
{
	atomic_set(&(waitq->condition), 1);
	wake_up(&(waitq->wq));
}

static const char *_dsi_cmd_mode_parse_state(unsigned int state)
{
	switch (state) {
	case 0x0001:
		return "idle";
	case 0x0002:
		return "Reading command queue for header";
	case 0x0004:
		return "Sending type-0 command";
	case 0x0008:
		return "Waiting frame data from RDMA for type-1 command";
	case 0x0010:
		return "Sending type-1 command";
	case 0x0020:
		return "Sending type-2 command";
	case 0x0040:
		return "Reading command queue for type-2 data";
	case 0x0080:
		return "Sending type-3 command";
	case 0x0100:
		return "Sending BTA";
	case 0x0200:
		return "Waiting RX-read data";
	case 0x0400:
		return "Waiting SW RACK for RX-read data";
	case 0x0800:
		return "Waiting TE";
	case 0x1000:
		return "Get TE";
	case 0x2000:
		return "Waiting SW RACK for TE";
	case 0x4000:
		return "Waiting external TE";
	case 0x8000:
		return "Get external TE";
	default:
		return "unknown";
	}
	return "unknown";
}

static const char *_dsi_vdo_mode_parse_state(unsigned int state)
{
	switch (state) {
	case 0x0001:
		return "Video mode idle";
	case 0x0002:
		return "Sync start packet";
	case 0x0004:
		return "Hsync active";
	case 0x0008:
		return "Sync end packet";
	case 0x0010:
		return "Hsync back porch";
	case 0x0020:
		return "Video data period";
	case 0x0040:
		return "Hsync front porch";
	case 0x0080:
		return "BLLP";
	case 0x0100:
		return "--";
	case 0x0200:
		return "Mix mode using command mode transmission";
	case 0x0400:
		return "Command transmission in BLLP";
	default:
		return "unknown";
	}

	return "unknown";
}

enum DSI_STATUS DSI_DumpRegisters(enum DISP_MODULE_ENUM module, int level)
{
	u32 i = 0, module_num;
	u32 k = 0;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	DDPDUMP("== DISP DSI REGS ==\n");
	if (level >= 0) {
		for (i = 0; i < module_num; i++) {
			unsigned int DSI_DBG8_Status;
			unsigned int DSI_DBG9_Status;
			unsigned long dsi_base_addr =
				(unsigned long)DSI_REG[i];

			if (DSI_REG[0]->DSI_MODE_CTRL.MODE == CMD_MODE) {
				unsigned int DSI_DBG6_Status =
					(INREG32(dsi_base_addr + 0x160)) &
					0xffff;

				DDPDUMP("DSI%d state6(cmd mode):%s\n",
					i, _dsi_cmd_mode_parse_state(
						DSI_DBG6_Status));
			} else {
				unsigned int DSI_DBG7_Status =
					(INREG32(dsi_base_addr + 0x164)) & 0xff;

				DDPDUMP("DSI%d state7(vdo mode):%s\n",
					i, _dsi_vdo_mode_parse_state(
						DSI_DBG7_Status));
			}
			DSI_DBG8_Status =
				(INREG32(dsi_base_addr + 0x168)) & 0x3fff;
			DDPDUMP("DSI%d state8 WORD_COUNTER(cmd mode):%s\n",
				i, _dsi_cmd_mode_parse_state(DSI_DBG8_Status));
			DSI_DBG9_Status =
				(INREG32(dsi_base_addr + 0x16C)) & 0x3fffff;
			DDPDUMP("DSI%d state9 LINE_COUNTER(cmd mode):%s\n",
				i, _dsi_cmd_mode_parse_state(DSI_DBG9_Status));
		}
	}
	if (level >= 1) {
		for (i = 0; i < module_num; i++) {
			unsigned long dsi_base_addr =
				(unsigned long)DSI_REG[i];
#ifndef CONFIG_FPGA_EARLY_PORTING
			unsigned long mipi_base_addr =
				(unsigned long)DSI_PHY_REG[i];
#endif
			unsigned long offset;

			DDPDUMP("== DSI%d REGS ==\n", i);
			for (k = 0; k < sizeof(struct DSI_REGS); k += 16) {
				offset = dsi_base_addr + k;
				DDPDUMP("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
					k, INREG32(offset),
					INREG32(offset + 0x4),
					INREG32(offset + 0x8),
					INREG32(offset + 0xc));
			}

			DDPDUMP("- DSI%d CMD REGS -\n", i);
			/* only dump first 32 bytes cmd */
			for (k = 0; k < 32; k += 16) {
				offset = dsi_base_addr + 0x200 + k;
				DDPDUMP("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
					k, INREG32(offset),
					INREG32(offset + 0x4),
					INREG32(offset + 0x8),
					INREG32(offset + 0xc));
			}

#ifndef CONFIG_FPGA_EARLY_PORTING
			DDPDUMP("== DSI_PHY%d REGS ==\n", i);
			for (k = 0; k < 0x6A0; k += 16) {
				DDPDUMP("0x%04x: 0x%08x 0x%08x 0x%08x 0x%08x\n",
					k, INREG32(mipi_base_addr + k),
					INREG32(mipi_base_addr + k + 0x4),
					INREG32(mipi_base_addr + k + 0x8),
					INREG32(mipi_base_addr + k + 0xc));
			}
#endif
		}
	}

	return DSI_STATUS_OK;
}

void _dump_dsi_params(struct LCM_DSI_PARAMS *dsi_config)
{
	if (dsi_config) {
		switch (dsi_config->mode) {
		case CMD_MODE:
			DISPDBG("[DSI] DSI Mode: CMD_MODE\n");
			break;
		case SYNC_PULSE_VDO_MODE:
			DISPDBG("[DSI] DSI Mode: SYNC_PULSE_VDO_MODE\n");
			break;
		case SYNC_EVENT_VDO_MODE:
			DISPDBG("[DSI] DSI Mode: SYNC_EVENT_VDO_MODE\n");
			break;
		case BURST_VDO_MODE:
			DISPDBG("[DSI] DSI Mode: BURST_VDO_MODE\n");
			break;
		default:
			DISPCHECK("[DSI] DSI Mode: Unknown\n");
			break;
		}

		DISPDBG("[DSI] vact: %d, vbp: %d, vfp: %d, vact_line: %d\n",
			dsi_config->vertical_sync_active,
			dsi_config->vertical_backporch,
			dsi_config->vertical_frontporch,
			dsi_config->vertical_active_line);
		DISPDBG("[DSI] hact: %d, hbp: %d, hfp: %d, hblank: %d\n",
			dsi_config->horizontal_sync_active,
			dsi_config->horizontal_backporch,
			dsi_config->horizontal_frontporch,
			dsi_config->horizontal_blanking_pixel);
		DISPDBG("[DSI] pll_select: %d, pll_div1: %d, pll_div2: %d\n",
			dsi_config->pll_select, dsi_config->pll_div1,
			dsi_config->pll_div2);
		DISPDBG("[DSI] fbk_div: %d,fbk_sel: %d, rg_bir: %d\n",
			dsi_config->fbk_div, dsi_config->fbk_sel,
			dsi_config->rg_bir);
		DISPDBG("[DSI] rg_bic: %d, rg_bp: %d\n",
			dsi_config->rg_bic, dsi_config->rg_bp);
		DISPDBG("[DSI] PLL_CLOCK: %d, dsi_clock: %d, ssc_range: %d\n",
			dsi_config->PLL_CLOCK, dsi_config->dsi_clock,
			dsi_config->ssc_range);
		DISPDBG("[DSI] ssc_disable:%d, com_for_nvk:%d, cont_clk:%d\n",
			dsi_config->ssc_disable,
			dsi_config->compatibility_for_nvk,
			dsi_config->cont_clock);
		DISPDBG("[DSI] lcm_ext_te_en:%d,noncont_clk:%d\n",
			dsi_config->lcm_ext_te_enable,
			dsi_config->noncont_clock);
		DISPDBG("[DSI] noncont_clock_period:%d\n",
			dsi_config->noncont_clock_period);
	}
}

static void _DSI_INTERNAL_IRQ_Handler(enum DISP_MODULE_ENUM module,
	unsigned int param)
{
	int i = 0;
	static unsigned int dsi_underrun_trigger = 1;
	struct DSI_INT_STATUS_REG status;
#if 0
	struct DSI_TXRX_CTRL_REG txrx_ctrl;
#endif

	i = DSI_MODULE_to_ID(module);
	status = *(struct DSI_INT_STATUS_REG *)(&param);

	if (status.RD_RDY)
		_set_condition_and_wake_up(&(_dsi_context[i].read_wq));

	if (status.CMD_DONE)
		wake_up(&(_dsi_context[i].cmddone_wq.wq));

	if (status.VM_DONE)
		wake_up(&(_dsi_context[i].vm_done_wq.wq));

	if (status.VM_CMD_DONE)
		_set_condition_and_wake_up(&(_dsi_context[i].vm_cmd_done_wq));

	if (status.SLEEPOUT_DONE)
		_set_condition_and_wake_up(
			&(_dsi_context[i].sleep_out_done_wq));

	if (status.SLEEPIN_DONE)
		_set_condition_and_wake_up(&(_dsi_context[i].sleep_in_done_wq));

	if (status.BUFFER_UNDERRUN_INT_EN) {
		if (disp_helper_get_option(DISP_OPT_DSI_UNDERRUN_AEE)) {
			if (dsi_underrun_trigger == 1) {
				DDPAEE("%s:buffer underrun,sys_time=%u\n",
					ddp_get_module_name(module),
					(u32)arch_counter_get_cntvct());
				primary_display_diagnose();
				dsi_underrun_trigger = 0;
			}
		}

		DDPERR("%s:buffer underrun\n", ddp_get_module_name(module));
		primary_display_diagnose();
	}

	if (status.INP_UNFINISH_INT_EN)
		DDPERR("%s:input relay unfinish\n",
			ddp_get_module_name(module));
}

static enum DSI_STATUS DSI_Reset(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq)
{
	int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	/* do reset */
	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
			DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
			DSI_REG[i]->DSI_COM_CTRL, DSI_RESET, 0);
	}

	return DSI_STATUS_OK;
}

static enum DSI_STATUS DPHY_Reset(enum DISP_MODULE_ENUM module,
				 struct cmdqRecStruct *cmdq)
{
	int i = 0;

	/* do reset */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
				  DSI_REG[i]->DSI_COM_CTRL, DPHY_RESET, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
				  DSI_REG[i]->DSI_COM_CTRL, DPHY_RESET, 0);
	}

	return DSI_STATUS_OK;
}

static enum DSI_STATUS DSI_SetMode(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, unsigned int mode)
{
	int i = 0, module_num;

	if ((module == DISP_MODULE_DSI0) || (module == DISP_MODULE_DSI1))
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invalid dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_MODE_CTRL_REG,
		DSI_REG[i]->DSI_MODE_CTRL, MODE, mode);
	}

	return DSI_STATUS_OK;
}

void DSI_clk_HS_mode(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
	bool enter)
{
	int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		if (enter) {
			DSI_OUTREGBIT(cmdq, struct DSI_PHY_LCCON_REG,
				DSI_REG[i]->DSI_PHY_LCCON, LC_HS_TX_EN, 1);
		} else if (!enter) {
			DSI_OUTREGBIT(cmdq, struct DSI_PHY_LCCON_REG,
				DSI_REG[i]->DSI_PHY_LCCON, LC_HS_TX_EN, 0);
		}
	}
}

/**
 * DSI_enter_ULPS
 *
 * 1. disable DSI high-speed clock
 * 2. Data lane enter ultra-low power mode
 * 3. Clock lane enter ultra-low power mode
 * 4. wait DSI sleepin irq (timeout interval ?)
 * 5. clear lane_num
 */
void DSI_enter_ULPS(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;
	int ret = 0;
	struct t_condition_wq *waitq;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	DSI_clk_HS_mode(module, NULL, FALSE); /* ??? should be hs disable */
	for (i = 0; i < module_num; i++) {
		ASSERT(DSI_REG[i]->DSI_PHY_LD0CON.L0_ULPM_EN == 0);
		ASSERT(DSI_REG[i]->DSI_PHY_LCCON.LC_ULPM_EN == 0);
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, SLEEPIN_ULPS_INT_EN, 1);

		DSI_OUTREGBIT(NULL, struct DSI_PHY_LD0CON_REG,
			DSI_REG[i]->DSI_PHY_LD0CON, Lx_ULPM_AS_L0, 1);
		DSI_OUTREGBIT(NULL, struct DSI_PHY_LCCON_REG,
			DSI_REG[i]->DSI_PHY_LCCON, LC_ULPM_EN, 1);
		udelay(1);
		DSI_OUTREGBIT(NULL, struct DSI_PHY_LD0CON_REG,
			DSI_REG[i]->DSI_PHY_LD0CON, L0_ULPM_EN, 1);

		waitq = &(_dsi_context[i].sleep_in_done_wq);
		ret = wait_event_timeout(waitq->wq,
			atomic_read(&(waitq->condition)), 2 * HZ);
		atomic_set(&(waitq->condition), 0);
		if (ret == 0) {
			DISPERR("dsi%d wait sleepin timeout\n", i);
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
		}

		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, SLEEPIN_ULPS_INT_EN, 0);
		/* clear lane_num when enter ulps */
	}
}

/**
 * DSI_exit_ULPS
 *
 * 1. set DSI sleep out mode
 * 2. set wakeup prd according to current MIPI frequency
 * 3. recovery lane number
 * 4. sleep out start
 * 4. wait DSI sleepout irq (timeout interval ?)
 */
void DSI_exit_ULPS(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;
	int ret = 0;
	unsigned int lane_num_bitvalue = 0;
	int wake_up_prd = 0;
	struct t_condition_wq *waitq;
	/* wake_up_prd * 1024 * cycle time > 1ms */
	unsigned int data_rate =
		_dsi_context[i].dsi_params.data_rate != 0 ?
			_dsi_context[i].dsi_params.data_rate :
			_dsi_context[i].dsi_params.PLL_CLOCK * 2;

	/*for mipi hopping*/
	if (mipi_clk_change_sta) {
		data_rate = _dsi_context[i].dsi_params.data_rate_dyn != 0 ?
			_dsi_context[i].dsi_params.data_rate_dyn :
			_dsi_context[i].dsi_params.PLL_CLOCK_dyn * 2;

	}
	/*DynFPS*/
	/*exit ulps only happen when power on
	 * and power on need use the default fps,
	 * because init sequence using default fps
	 * no need check dynfps value
	 */

	wake_up_prd = (data_rate * 1000) / (1024 * 8) + 0x1;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(NULL, struct DSI_PHY_LD0CON_REG,
			DSI_REG[i]->DSI_PHY_LD0CON, Lx_ULPM_AS_L0, 1);
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, SLEEPOUT_DONE, 1);

		switch (_dsi_context[i].dsi_params.LANE_NUM) {
		case LCM_ONE_LANE:
			lane_num_bitvalue = 0x1;
			break;
		case LCM_TWO_LANE:
			lane_num_bitvalue = 0x3;
			break;
		case LCM_THREE_LANE:
			lane_num_bitvalue = 0x7;
			break;
		case LCM_FOUR_LANE:
			lane_num_bitvalue = 0xF;
			break;
		default:
			break;
		}

		DSI_OUTREGBIT(NULL, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, LANE_NUM, lane_num_bitvalue);

		DSI_OUTREGBIT(NULL, struct DSI_MODE_CTRL_REG,
			DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 1);
		DSI_OUTREGBIT(NULL, struct DSI_TIME_CON0_REG,
			DSI_REG[i]->DSI_TIME_CON0, UPLS_WAKEUP_PRD,
			wake_up_prd);
		DSI_OUTREGBIT(NULL, struct DSI_START_REG,
			DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(NULL, struct DSI_START_REG,
			DSI_REG[i]->DSI_START, SLEEPOUT_START, 1);

		waitq = &(_dsi_context[i].sleep_out_done_wq);
		ret = wait_event_timeout(waitq->wq,
			atomic_read(&(waitq->condition)), 2 * HZ);
		atomic_set(&(waitq->condition), 0);
		if (ret == 0) {
			DISPERR("dsi%d wait sleepout timeout\n", i);
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
		}
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, SLEEPOUT_DONE, 0);
		DSI_OUTREGBIT(NULL, struct DSI_START_REG,
			DSI_REG[i]->DSI_START, SLEEPOUT_START, 0);
		DSI_OUTREGBIT(NULL, struct DSI_MODE_CTRL_REG,
			DSI_REG[i]->DSI_MODE_CTRL, SLEEP_MODE, 0);
	}
}

static int _check_dsi_mode(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	/* check validity */
	for (i = 0; i < module_num; i++) {
		switch (DSI_REG[i]->DSI_MODE_CTRL.MODE) {
		case CMD_MODE:
		case SYNC_EVENT_VDO_MODE:
		case SYNC_PULSE_VDO_MODE:
		case BURST_VDO_MODE:
			break;
		default:
			return 0;
		}
	}

	/* check coherence */
	if (module == DISP_MODULE_DSIDUAL) {
		if (DSI_REG[0]->DSI_MODE_CTRL.MODE !=
		    DSI_REG[DSI_MODULE_END(module)]->DSI_MODE_CTRL.MODE)
			return 0;
	}

	return 1;
}

static int _is_lcm_cmd_mode(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;
	int ret = 0;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	for (i = 0; i < module_num; i++) {
		if (_dsi_context[i].dsi_params.mode == CMD_MODE)
			ret++;
	}

	return ret;
}

static void _dsi_wait_not_busy_(enum DISP_MODULE_ENUM module,
			      struct cmdqRecStruct *cmdq)
{
	unsigned int loop_cnt = 0;
	int i = 0;

	if (module == DISP_MODULE_DSI0)
		i = 0;
	else if (module == DISP_MODULE_DSI1)
		i = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		i = 0;
	else
		return;

	if (cmdq) {

#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		cmdqRecWait(cmdq,
		(i == 0) ?
		CMDQ_EVENT_DSI0_DONE_EVENT :
		CMDQ_EVENT_DSI1_DONE_EVENT);
		/*avoid influence the stability already achieved
		 * only change to wait for DynFPS
		 */
#else
		DSI_POLLREG32(cmdq, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);
#endif

		DSI_POLLREG32(cmdq, &DSI_REG[i]->DSI_INTSTA, 0x80000000, 0x0);
		return;
	}

	if (!(DSI_REG[i]->DSI_INTSTA.BUSY))
		return;

	while (loop_cnt < 100*1000) {
		if (!(DSI_REG[i]->DSI_INTSTA.BUSY))
			break;
		loop_cnt++;
		udelay(1);
	}
}

static void dsi_wait_not_busy(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq)
{
	int i = 0;

	if (module == DISP_MODULE_DSI0)
		i = 0;
	else if (module == DISP_MODULE_DSI1)
		i = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		i = 0;
	else
		return;

	if (DSI_REG[i]->DSI_MODE_CTRL.MODE) /* only cmd mode can wait cmddone */
		return;

	_dsi_wait_not_busy_(module, cmdq);
}

enum DSI_STATUS DSI_BIST_Pattern_Test(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, bool enable, unsigned int color)
{
	unsigned int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	for (i = 0; i < module_num; i++) {
		if (enable) {
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_PATTERN,
				color);
			DSI_OUTREGBIT(cmdq, struct DSI_BIST_CON_REG,
				DSI_REG[i]->DSI_BIST_CON, SELF_PAT_MODE, 1);

			if (_is_lcm_cmd_mode(module)) {
				struct DSI_T0_INS t0;

				t0.CONFG = 0x09;
				t0.Data_ID = 0x39;
				t0.Data0 = 0x2c;
				t0.Data1 = 0;

				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE,
					1);

				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 0);
				DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_START, 1);
			}
		} else {
			/*
			 * if disable dsi pattern, need enable mutex,
			 * can't just start dsi so we just disable
			 * pattern bit, do not start dsi here
			 */
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BIST_CON, 0x00);
		}

	}
	return DSI_STATUS_OK;
}

int ddp_dsi_porch_setting(enum DISP_MODULE_ENUM module, void *handle,
	enum DSI_PORCH_TYPE type, unsigned int value)
{
	int i, module_num, ret = 0;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	for (i = 0; i < module_num; i++) {
		if (type == DSI_VFP) {
			DISPINFO("set dsi%d vfp to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_VFP_NL, value);
		}
		if (type == DSI_VSA) {
			DISPINFO("set dsi%d vsa to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_VSA_NL, value);
		}
		if (type == DSI_VBP) {
			DISPINFO("set dsi%d vbp to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_VBP_NL, value);
		}
		if (type == DSI_VACT) {
			DISPINFO("set dsi%d vact to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_VACT_NL, value);
		}
		if (type == DSI_HFP) {
			DISPINFO("set dsi%d hfp to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_HFP_WC, value);
		}
		if (type == DSI_HSA) {
			DISPINFO("set dsi%d hsa to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_HSA_WC, value);
		}
		if (type == DSI_HBP) {
			DISPINFO("set dsi%d hbp to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_HBP_WC, value);
		}
		if (type == DSI_BLLP) {
			DISPINFO("set dsi%d bllp to %d\n", i, value);
			DSI_OUTREG32(handle, &DSI_REG[i]->DSI_BLLP_WC, value);
		}
	}
	return ret;
}

static void DSI_Get_Porch_Addr(enum DISP_MODULE_ENUM module,
	unsigned long *pAddr)
{
	int i = 0, module_num;
	unsigned long porch_addr = 0;
	enum DSI_PORCH_TYPE porch_type = DSI_VFP;

	if (pAddr == NULL) {
		DISPERR("%s, NULL pointer!\n", __func__);
		return;
	}

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	porch_type = (enum DSI_PORCH_TYPE)(*pAddr);
	for (i = 0; i < module_num; i++) {
		if (porch_type == DSI_VFP) {
			porch_addr =
				(unsigned long)(&DSI_REG[i]->DSI_VFP_NL);
			DISPINFO("get dsi%d vfp addr_va:%ld\n",
				i, porch_addr);
		}

		if (porch_addr)
			pAddr[i] = disp_addr_convert(porch_addr);
	}
}

void DSI_DPHY_Calc_VDO_Timing(enum DISP_MODULE_ENUM module,
		struct LCM_DSI_PARAMS *dsi_params)
{
	int i;
	unsigned int dsiTmpBufBpp;
	unsigned int t_vfp, t_vbp, t_vsa;
	unsigned int t_hfp, t_hbp, t_hsa;
	unsigned int t_hbllp;

	t_vfp = (mipi_clk_change_sta) ?
		(dsi_params->vertical_frontporch_dyn == 0 ?
		 dsi_params->vertical_frontporch :
		 dsi_params->vertical_frontporch_dyn) :
		dsi_params->vertical_frontporch;
	t_vbp = (mipi_clk_change_sta) ?
		(dsi_params->vertical_backporch_dyn == 0 ?
		 dsi_params->vertical_backporch :
		 dsi_params->vertical_backporch_dyn) :
		dsi_params->vertical_backporch;
	t_vsa = (mipi_clk_change_sta) ?
		(dsi_params->vertical_sync_active_dyn == 0 ?
		 dsi_params->vertical_sync_active :
		 dsi_params->vertical_sync_active_dyn) :
		dsi_params->vertical_sync_active;
	t_hbp = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_backporch_dyn == 0 ?
		 dsi_params->horizontal_backporch :
		 dsi_params->horizontal_backporch_dyn) :
		dsi_params->horizontal_backporch;
	t_hfp = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_frontporch_dyn == 0 ?
		 dsi_params->horizontal_frontporch :
		 dsi_params->horizontal_frontporch_dyn) :
		dsi_params->horizontal_frontporch;
	t_hsa = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_sync_active_dyn == 0 ?
		 dsi_params->horizontal_sync_active :
		 dsi_params->horizontal_sync_active_dyn) :
		dsi_params->horizontal_sync_active;
	if (dsi_params->data_format.format == LCM_DSI_FORMAT_RGB565)
		dsiTmpBufBpp = 2;
	else
		dsiTmpBufBpp = 3;
	if (dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE ||
			dsi_params->switch_mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->switch_mode == BURST_VDO_MODE) {
		t_hsa = ALIGN_TO(t_hsa * dsiTmpBufBpp - 4, 4);
		ASSERT((t_hbp +	t_hsa) * dsiTmpBufBpp > 9);
		t_hbp = ALIGN_TO((t_hbp + t_hsa) * dsiTmpBufBpp - 10, 4);
	} else {
		ASSERT(t_hsa * dsiTmpBufBpp > 9);
		t_hsa = ALIGN_TO(t_hsa * dsiTmpBufBpp - 10, 4);
		ASSERT(t_hbp * dsiTmpBufBpp > 9);
		t_hbp = ALIGN_TO(t_hbp * dsiTmpBufBpp - 10, 4);
	}
	ASSERT(t_hfp * dsiTmpBufBpp > 11);
	t_hfp = ALIGN_TO(t_hfp * dsiTmpBufBpp - 12, 4);
	t_hbllp = ALIGN_TO(dsi_params->horizontal_bllp * dsiTmpBufBpp, 4);
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		_dsi_context[i].vsa = t_vsa;
		_dsi_context[i].vbp = t_vbp;
		_dsi_context[i].vfp = t_vfp;
		_dsi_context[i].hsa_byte = t_hsa;
		_dsi_context[i].hbp_byte = t_hbp;
		_dsi_context[i].hfp_byte = t_hfp;
		_dsi_context[i].hbllp_byte = t_hbllp;
	}
}

void DSI_CPHY_Calc_VDO_Timing(enum DISP_MODULE_ENUM module,
		struct LCM_DSI_PARAMS *dsi_params)
{
	int i;
	unsigned int dsiTmpBufBpp;
	unsigned int t_vfp, t_vbp, t_vsa;
	unsigned int t_hfp, t_hbp, t_hsa;
	unsigned int t_hbllp;
	unsigned int lane_num = 0;
	unsigned int data_phy_cycle;

	t_vfp = (mipi_clk_change_sta) ?
		(dsi_params->vertical_frontporch_dyn == 0 ?
		 dsi_params->vertical_frontporch :
		 dsi_params->vertical_frontporch_dyn) :
		dsi_params->vertical_frontporch;
	t_vbp = (mipi_clk_change_sta) ?
		(dsi_params->vertical_backporch_dyn == 0 ?
		 dsi_params->vertical_backporch :
		 dsi_params->vertical_backporch_dyn) :
		dsi_params->vertical_backporch;
	t_vsa = (mipi_clk_change_sta) ?
		(dsi_params->vertical_sync_active_dyn == 0 ?
		 dsi_params->vertical_sync_active :
		 dsi_params->vertical_sync_active_dyn) :
		dsi_params->vertical_sync_active;

	t_hbp = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_backporch_dyn == 0 ?
		 dsi_params->horizontal_backporch :
		 dsi_params->horizontal_backporch_dyn) :
		dsi_params->horizontal_backporch;
	t_hfp = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_frontporch_dyn == 0 ?
		 dsi_params->horizontal_frontporch :
		 dsi_params->horizontal_frontporch_dyn) :
		dsi_params->horizontal_frontporch;
	t_hsa = (mipi_clk_change_sta) ?
		(dsi_params->horizontal_sync_active_dyn == 0 ?
		 dsi_params->horizontal_sync_active :
		 dsi_params->horizontal_sync_active_dyn) :
		dsi_params->horizontal_sync_active;

	if (dsi_params->data_format.format == LCM_DSI_FORMAT_RGB565)
		dsiTmpBufBpp = 2;
	else
		dsiTmpBufBpp = 3;

	switch (dsi_params->LANE_NUM) {
	case LCM_ONE_LANE:
		lane_num = 1;
		break;
	case LCM_TWO_LANE:
		lane_num = 2;
		break;
	case LCM_THREE_LANE:
		lane_num = 3;
		break;
	case LCM_FOUR_LANE:
		lane_num = 4;
		break;
	default:
		break;
	}

	if (dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE ||
			dsi_params->switch_mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->switch_mode == BURST_VDO_MODE) {
		t_hsa = ALIGN_TO(t_hsa * dsiTmpBufBpp - 4, 4);
		ASSERT((t_hbp +	t_hsa) * dsiTmpBufBpp > 9);

		t_hbp = ALIGN_TO((t_hbp + t_hsa) * dsiTmpBufBpp - 10, 4);
	} else {
	// min:4
		if (t_hsa * dsiTmpBufBpp < 10 * lane_num + 26 + 5)
			t_hsa = 4;
		else
			t_hsa = ALIGN_TO(t_hsa * dsiTmpBufBpp -
				10 * lane_num - 26, 2);

		if (t_hbp * dsiTmpBufBpp < 12 * lane_num + 26 + 5)
			t_hbp = 4;
		else
			t_hbp = ALIGN_TO(t_hbp * dsiTmpBufBpp -
				12 * lane_num - 26, 2);
	}

	//data_phy_cycle = t_hs_zero data + t_hs_prep data +t_hs_exit dafa+5;

	//t_hbllp = ALIGN_TO(dsi_params->horizontal_bllp * dsiTmpBufBpp, 4);
	t_hbllp = ALIGN_TO(lane_num * 16, 4);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		data_phy_cycle = _dsi_context[i].data_phy_cycle;

		if (t_hfp * dsiTmpBufBpp < 8 * lane_num +
			28 + 2 * data_phy_cycle * lane_num + 9)
			t_hfp = 8;
		else if ((t_hfp * dsiTmpBufBpp > 8 * lane_num + 28 +
			2 * data_phy_cycle * lane_num + 8) &&
			(t_hfp * dsiTmpBufBpp < 8 * lane_num +
			28 + 2 * data_phy_cycle * lane_num +
			ALIGN_TO(2 * (_dsi_context[i].HS_TRAIL + 1)*lane_num -
			6 * lane_num - 12, 2)))
			t_hfp = ALIGN_TO(2*(_dsi_context[i].HS_TRAIL + 1)*
			lane_num - 6*lane_num - 12, 2);
		else
			t_hfp = ALIGN_TO(t_hfp * dsiTmpBufBpp - 8 * lane_num -
				28 - 2 * data_phy_cycle * lane_num, 2);

		_dsi_context[i].vsa = t_vsa;
		_dsi_context[i].vbp = t_vbp;
		_dsi_context[i].vfp = t_vfp;
		_dsi_context[i].hsa_byte = t_hsa;
		_dsi_context[i].hbp_byte = t_hbp;
		_dsi_context[i].hfp_byte = t_hfp;
		_dsi_context[i].hbllp_byte = t_hbllp;
	}
}

void DSI_Calc_VDO_Timing(enum DISP_MODULE_ENUM module,
		struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (_dsi_context[i].dsi_params.IsCphy)
			DSI_CPHY_Calc_VDO_Timing(module, dsi_params);
		else
			DSI_DPHY_Calc_VDO_Timing(module, dsi_params);
	}


}

void DSI_Config_VDO_Timing(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0, module_num;
	unsigned int line_byte;
	unsigned int horizontal_sync_active_byte;
	unsigned int horizontal_backporch_byte;
	unsigned int horizontal_frontporch_byte;
	unsigned int horizontal_bllp_byte;
	unsigned int dsiTmpBufBpp;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		if (dsi_params->data_format.format ==
			LCM_DSI_FORMAT_RGB565)
			dsiTmpBufBpp = 2;
		else
			dsiTmpBufBpp = 3;

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VSA_NL,
			dsi_params->vertical_sync_active);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VBP_NL,
			dsi_params->vertical_backporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VFP_NL,
			dsi_params->vertical_frontporch);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_VACT_NL,
			dsi_params->vertical_active_line);

		line_byte =
			(dsi_params->horizontal_sync_active +
			dsi_params->horizontal_backporch +
			dsi_params->horizontal_frontporch +
			dsi_params->horizontal_active_pixel) *
			dsiTmpBufBpp;
		horizontal_sync_active_byte =
			(dsi_params->horizontal_sync_active *
			dsiTmpBufBpp - 4);

		if (dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE ||
		    dsi_params->switch_mode == SYNC_EVENT_VDO_MODE ||
		    dsi_params->switch_mode == BURST_VDO_MODE) {
			ASSERT((dsi_params->horizontal_backporch +
				dsi_params->horizontal_sync_active) *
				dsiTmpBufBpp > 9);

			horizontal_backporch_byte =
				((dsi_params->horizontal_backporch +
				dsi_params->horizontal_sync_active) *
				dsiTmpBufBpp - 10);
		} else {
			ASSERT(dsi_params->horizontal_sync_active *
				dsiTmpBufBpp > 9);

			horizontal_sync_active_byte =
				(dsi_params->horizontal_sync_active *
				dsiTmpBufBpp - 10);

			ASSERT(dsi_params->horizontal_backporch *
				dsiTmpBufBpp > 9);

			horizontal_backporch_byte =
				(dsi_params->horizontal_backporch *
				dsiTmpBufBpp - 10);
		}

		ASSERT(dsi_params->horizontal_frontporch * dsiTmpBufBpp > 11);
		horizontal_frontporch_byte =
		    (dsi_params->horizontal_frontporch * dsiTmpBufBpp - 12);
		horizontal_bllp_byte =
			(dsi_params->horizontal_bllp * dsiTmpBufBpp);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSA_WC,
			ALIGN_TO((horizontal_sync_active_byte), 4));
		if (def_dsi_hbp)
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC,
			 def_dsi_hbp);
		else
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HBP_WC,
				ALIGN_TO((horizontal_backporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC,
			ALIGN_TO((horizontal_frontporch_byte), 4));
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_BLLP_WC,
			ALIGN_TO((horizontal_bllp_byte), 4));
	}
}

void DSI_Set_LFR(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
	unsigned int mode, unsigned int type, unsigned int enable,
	unsigned int skip_num)
{
	/* LFR_MODE 0 disable,1 static mode ,2 dynamic mode 3,both */
	unsigned int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_MODE, mode);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_TYPE, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_VSE_DIS, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_SKIP_NUM, skip_num);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_EN, enable);
	}
}

void DSI_LFR_UPDATE(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq)
{
	unsigned int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_LFR_CON_REG,
			DSI_REG[i]->DSI_LFR_CON, LFR_UPDATE, 1);
	}
}

int DSI_LFR_Status_Check(void)
{
	unsigned int status = 0;
	struct DSI_LFR_STA_REG lfr_skip_sta;

	lfr_skip_sta = DSI_REG[0]->DSI_LFR_STA;
	status = lfr_skip_sta.LFR_SKIP_STA;
	DISPCHECK("LFR_SKIP_CNT 0x%x LFR_SKIP_STA 0x%x,status 0x%x\n",
		  lfr_skip_sta.LFR_SKIP_CNT, lfr_skip_sta.LFR_SKIP_STA, status);

	return status;
}

int _dsi_ps_type_to_bpp(enum LCM_PS_TYPE ps)
{
	switch (ps) {
	case LCM_PACKED_PS_16BIT_RGB565:
		return 2;
	case LCM_LOOSELY_PS_18BIT_RGB666:
		return 3;
	case LCM_PACKED_PS_24BIT_RGB888:
		return 3;
	case LCM_PACKED_PS_18BIT_RGB666:
		return 3;
	}
	return 0;
}

enum DSI_STATUS DSI_PS_Control(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params, int w,
	int h)
{
	int i = 0, module_num;
	unsigned int ps_sel_bitvalue = 0;
	unsigned int ps_wc_adjust = 0;
	unsigned int ps_wc = 0;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	/* TODO: parameter checking */
	ASSERT((int)(dsi_params->PS) <= (int)PACKED_PS_18BIT_RGB666);

	if ((int)(dsi_params->PS) > (int)(LOOSELY_PS_18BIT_RGB666))
		ps_sel_bitvalue = (5 - dsi_params->PS);
	else
		ps_sel_bitvalue = dsi_params->PS;

	if (module == DISP_MODULE_DSIDUAL)
		w = w / 2;

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_VACT_NL_REG,
			DSI_REG[i]->DSI_VACT_NL, VACT_NL, h);
		if (dsi_params->ufoe_enable &&
			dsi_params->ufoe_params.lr_mode_en != 1) {
			if (dsi_params->ufoe_params.compress_ratio == 3) {
				unsigned int ufoe_internal_width = w + w % 4;
				int ps_bpp =
					_dsi_ps_type_to_bpp(dsi_params->PS);

				if (ufoe_internal_width % 3 == 0) {
					ps_wc = (ufoe_internal_width / 3) *
						ps_bpp;
				} else {
					unsigned int temp_w =
						ufoe_internal_width / 3 + 1;
					temp_w = ((temp_w % 2) == 1) ?
						(temp_w + 1) : temp_w;
					ps_wc = temp_w  * ps_bpp;
				}
			} else { /* 1/2 */
				ps_wc = (w + w % 4) / 2 *
					_dsi_ps_type_to_bpp(dsi_params->PS);
			}
		} else if (dsi_params->dsc_enable) {
			ps_wc = dsi_params->word_count;
		} else {
			ps_wc = w * _dsi_ps_type_to_bpp(dsi_params->PS);
		}

		if (ps_wc_adjust)
			ps_wc *= dsi_params->packet_size_mult;

		DSI_OUTREGBIT(cmdq, struct DSI_PSCTRL_REG,
			DSI_REG[i]->DSI_PSCTRL, DSI_PS_WC, ps_wc);
		DSI_OUTREGBIT(cmdq, struct DSI_PSCTRL_REG,
			DSI_REG[i]->DSI_PSCTRL, DSI_PS_SEL, ps_sel_bitvalue);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_SIZE_CON,
			h << 16 | w);
	}

	return DSI_STATUS_OK;
}

enum DSI_STATUS DSI_TXRX_Control(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0, module_num;
	unsigned int lane_num_bitvalue = 0;
	int lane_num = dsi_params->LANE_NUM;
	int vc_num = 0;
	bool null_packet_en = FALSE;
	bool dis_eotp_en = FALSE;
	bool hstx_cklp_en = dsi_params->cont_clock ? FALSE : TRUE;
	int max_return_size = 0;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	switch (lane_num) {
	case LCM_ONE_LANE:
		lane_num_bitvalue = 0x1;
		break;
	case LCM_TWO_LANE:
		lane_num_bitvalue = 0x3;
		break;
	case LCM_THREE_LANE:
		lane_num_bitvalue = 0x7;
		break;
	case LCM_FOUR_LANE:
		lane_num_bitvalue = 0xF;
		break;
	}

	for (i = 0; i < module_num; i++) {
		if (_dsi_context[i].dsi_params.IsCphy)
			dis_eotp_en = TRUE;
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, VC_NUM,
			vc_num);
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, LANE_NUM,
			lane_num_bitvalue);
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, DIS_EOT,
			dis_eotp_en);
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, BLLP_EN,
			null_packet_en);
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, MAX_RTN_SIZE,
			max_return_size);
		DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, HSTX_CKLP_EN,
			hstx_cklp_en);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI,
			DSI_WMEM_CONTI);
		if (dsi_params->mode == CMD_MODE ||
			(dsi_params->mode != CMD_MODE &&
			dsi_params->eint_disable)) {
			if (dsi_params->ext_te_edge ==
				LCM_POLARITY_FALLING) {
				/*use ext te falling edge */
				DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
					DSI_REG[i]->DSI_TXRX_CTRL,
					EXT_TE_EDGE, 1);
			}
			DSI_OUTREGBIT(cmdq, struct DSI_TXRX_CTRL_REG,
				DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 1);
		}

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_MEM_CONTI, DSI_WMEM_CONTI);
	}
	return DSI_STATUS_OK;
}

unsigned long MIPI_BASE_ADDR(enum DISP_MODULE_ENUM dsi_module)
{
	switch (dsi_module) {
	case DISP_MODULE_DSI0:
		return DSI_PHY_REG[0];
	case DISP_MODULE_DSI1:
		return DSI_PHY_REG[1];
	default:
		DDPERR("invalid module=%d\n", dsi_module);
		break;
	}

	return 0;
}

int MIPITX_IsEnabled(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq)
{
	int ret = 0;
	unsigned long base0, base1;

	if (module == DISP_MODULE_DSIDUAL) {
		base0 = MIPI_BASE_ADDR(DISP_MODULE_DSI0) + MIPITX_PLL_CON1;
		base1 = MIPI_BASE_ADDR(DISP_MODULE_DSI1) + MIPITX_PLL_CON1;

		ASSERT(MIPITX_INREGBIT(base0, FLD_RG_DSI_PLL_EN) ==
		       MIPITX_INREGBIT(base1, FLD_RG_DSI_PLL_EN));

		ret = MIPITX_INREGBIT(
			MIPI_BASE_ADDR(DISP_MODULE_DSI0) + MIPITX_PLL_CON1,
				FLD_RG_DSI_PLL_EN);
	} else if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSI1) {
		base0 = MIPI_BASE_ADDR(module) + MIPITX_PLL_CON1;
		ret = MIPITX_INREGBIT(base0, FLD_RG_DSI_PLL_EN);
	}

	return ret;
}

unsigned int dsi_phy_get_clk(enum DISP_MODULE_ENUM module)
{
	return 0;
}

void DSI_PHY_clk_change(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{

}

static int _dsi_get_pcw(int data_rate, int pcw_ratio)
{
	int pcw, tmp, pcw_floor;

	/**
	 * PCW bit 24~30 = floor(pcw)
	 * PCW bit 16~23 = (pcw - floor(pcw))*256
	 * PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
	 * PCW bit 0~7 = (pcw*256*256 - floor(pcw)*256*256)*256
	 */

	pcw = data_rate * pcw_ratio / 26;
	pcw_floor = data_rate * pcw_ratio % 26;
	tmp = ((pcw & 0xFF) << 24) |
		(((256 * pcw_floor / 26) & 0xFF) << 16) |
		(((256 * (256 * pcw_floor % 26) / 26) & 0xFF) << 8) |
		((256 * (256 * (256 * pcw_floor % 26) % 26) / 26) & 0xFF);

	return tmp;
}

void DSI_CPHY_clk_setting(enum DISP_MODULE_ENUM module,
								struct cmdqRecStruct *cmdq,
								struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int pcw_ratio = 0;
	unsigned int pcw = 0;
	unsigned int posdiv = 0;
	unsigned int prediv = 0;
	unsigned int data_Rate = 0;
	unsigned int data_Rate_dyn = 0;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int j = 0;
	struct dfps_info *dfps_params = NULL;
#endif

	/*for mipi hopping*/
	data_Rate = dsi_params->data_rate != 0 ?
					dsi_params->data_rate :
					dsi_params->PLL_CLOCK * 2;

	if (mipi_clk_change_sta) {
		data_Rate_dyn = dsi_params->data_rate_dyn != 0 ?
						dsi_params->data_rate_dyn :
						dsi_params->PLL_CLOCK_dyn * 2;
		if (!data_Rate_dyn)
			data_Rate = data_Rate_dyn;
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/*ToDo, move to a function, dphy also add this function*/
	/*for DynFPS*/
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/*if not power on scenario
		 * if disp_fps !=0 means dynfps happen
		 */
		if (_dsi_context[i].disp_fps &&
			(_dsi_context[i].dynfps_chg_index &
			DYNFPS_DSI_MIPI_CLK)) {
			for (j = 0; j < dsi_params->dfps_num; j++) {
				if ((dsi_params->dfps_params)[j].fps ==
					_dsi_context[i].disp_fps) {
					dfps_params =
						&((dsi_params->dfps_params)[j]);
					DDPMSG("%s,_dsi_context fps:%d\n",
					__func__, _dsi_context[i].disp_fps);
					break;
				}
			}
			if (!mipi_clk_change_sta) {
				if (dfps_params->data_rate != 0 ||
					dfps_params->PLL_CLOCK != 0)
					data_Rate =
						dfps_params->data_rate != 0 ?
						dfps_params->data_rate :
						dfps_params->PLL_CLOCK * 2;
			} else {
				if (dfps_params->data_rate_dyn != 0 ||
					dfps_params->PLL_CLOCK_dyn != 0)
					data_Rate =
					dfps_params->data_rate_dyn != 0 ?
					dfps_params->data_rate_dyn :
					dfps_params->PLL_CLOCK_dyn * 2;

			}
		}
	}
#endif
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		//MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_IMPENDANCE_2, 0x00001010);

		/*set volate*/
		MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_VOLTAGE_SEL, 0x4444236A);

		/*set lane swap*/
		if (dsi_params->lane_swap_en) {
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL0,
				dsi_params->PHY_SEL0);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL1,
				dsi_params->PHY_SEL1);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL2,
				dsi_params->PHY_SEL2);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL3,
				dsi_params->PHY_SEL3);
		} else {
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL0,
				0x65432101);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL1,
				0x24210987);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL2,
				0x68543102);
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PHY_SEL3,
				0x00000007);
		}
	}

	/* MIPI INIT */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* step 0 */
		/* BG_LPF_EN / BG_CORE_EN */
		MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_PLL_CON4,
			0x00FF12E0);
		/* BG_LPF_EN=0 BG_CORE_EN=1 */
		MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
			0x3FFF0088);
		mdelay(1); /* 1us */
		/* BG_LPF_EN=1 */
		MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
			0x3FFF00C8);

		/* step 1 */
		/* SDM_RWR_ON / SDM_ISO_EN */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
			FLD_AD_DSI_PLL_SDM_PWR_ON, 1);
		mdelay(1); /* 1us */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
			FLD_AD_DSI_PLL_SDM_ISO_EN, 0);


		if (data_Rate != 0) {
			unsigned int tmp = 0;

			if (data_Rate > 2500) {
				DISPCHECK("mipitx Data Rate exceed limitation(%d)\n", data_Rate);
				ASSERT(0);
			} else if (data_Rate >= 2000) { /* 2G ~ 2.5G */
				pcw_ratio = 1;
				posdiv    = 0;
				prediv    = 0;
			} else if (data_Rate >= 1000) { /* 1G ~ 2G */
				pcw_ratio = 2;
				posdiv    = 1;
				prediv    = 0;
			} else if (data_Rate >= 500) { /* 500M ~ 1G */
				pcw_ratio = 4;
				posdiv    = 2;
				prediv    = 0;
			} else if (data_Rate > 250) { /* 250M ~ 500M */
				pcw_ratio = 8;
				posdiv    = 3;
				prediv    = 0;
			} else if (data_Rate >= 125) { /* 125M ~ 250M */
				pcw_ratio = 16;
				posdiv    = 4;
				prediv    = 0;
			} else {
				DISPCHECK("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/* step 3 */
			/* PLL PCW config */
			/*
			 * PCW bit 24~30 = floor(pcw)
			 * PCW bit 16~23 = (pcw - floor(pcw))*256
			 * PCW bit 8~15 = (pcw*256 - floor(pcw)*256)*256
			 * PCW bit 8~15 = (pcw*256*256 - floor(pcw)*256*256)*256
			 */
			/* pcw = data_Rate*4*txdiv/(26*2);//Post DIV =4, so need data_Rate*4 */

			pcw = data_Rate * pcw_ratio / 26;
			tmp = ((pcw & 0xFF) << 24) |
				(((256 * (data_Rate * pcw_ratio % 26) / 26) & 0xFF) << 16) |
				(((256 * (256 * (data_Rate * pcw_ratio % 26) %
				26) / 26) & 0xFF) << 8) |
				((256 * (256 * (256 * (data_Rate * pcw_ratio % 26) %
				26) % 26) / 26) & 0xFF);
			MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_PLL_CON0, tmp);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON1,
			FLD_RG_DSI_PLL_POSDIV, posdiv);
		}
		/* step 4 */
		/* PLL EN */
		mdelay(1); /* 30ns */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_CON1, FLD_RG_DSI_PLL_EN, 1);

		mdelay(1); /* 20us */
	}

}

static void DSI_DPHY_clk_setting(enum DISP_MODULE_ENUM module,
								struct cmdqRecStruct *cmdq,
								struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;
	unsigned int j = 0;
	unsigned int data_Rate;
	unsigned int pll_clock;
	unsigned int pcw_ratio = 0;
	unsigned int posdiv = 0;
	unsigned int prediv = 0;
	unsigned int delta1 = 2; /* Delta1 is SSC range, default is 0%~-5% */
	unsigned int pdelta1 = 0;
	enum MIPITX_PHY_LANE_SWAP *swap_base;
	enum MIPITX_PAD_VALUE pad_mapping[MIPITX_PHY_LANE_NUM] = {
					PAD_D0P_V, PAD_D1P_V, PAD_D2P_V,
					PAD_D3P_V, PAD_CKP_V, PAD_CKP_V};

	DISPFUNC();

	data_Rate = (mipi_clk_change_sta == 0) ?
		dsi_params->data_rate : dsi_params->data_rate_dyn;
	pll_clock = (mipi_clk_change_sta == 0) ?
		dsi_params->PLL_CLOCK : dsi_params->PLL_CLOCK_dyn;
	data_Rate = data_Rate != 0 ? data_Rate : pll_clock * 2;

	/* DPHY SETTING */

	/* MIPITX lane swap setting */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* step 0 MIPITX lane swap setting */
		swap_base = dsi_params->lane_swap[i];
		if (unlikely(dsi_params->lane_swap_en)) {
			DISPINFO("MIPITX Lane Swap Enabled for DSI Port %d\n",
				 i);
			DISPINFO(
				"MIPITX Lane Swap mapping: %d|%d|%d|%d|%d|%d\n",
				 swap_base[MIPITX_PHY_LANE_0],
				 swap_base[MIPITX_PHY_LANE_1],
				 swap_base[MIPITX_PHY_LANE_2],
				 swap_base[MIPITX_PHY_LANE_3],
				 swap_base[MIPITX_PHY_LANE_CK],
				 swap_base[MIPITX_PHY_LANE_RX]);

			/* CKMODE_EN */
			for (j = MIPITX_PHY_LANE_0; j < MIPITX_PHY_LANE_CK;
			     j++) {
				if (dsi_params->lane_swap[i][j] ==
				    MIPITX_PHY_LANE_CK)
					break;
			}
			switch (j) {
			case MIPITX_PHY_LANE_0:
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
						 MIPITX_D0_CKMODE_EN,
						 FLD_DSI_D0_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_1:
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
						 MIPITX_D1_CKMODE_EN,
						 FLD_DSI_D1_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_2:
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
						 MIPITX_D2_CKMODE_EN,
						 FLD_DSI_D2_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_3:
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
						 MIPITX_D3_CKMODE_EN,
						 FLD_DSI_D3_CKMODE_EN, 1);
				break;
			case MIPITX_PHY_LANE_CK:
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
						 MIPITX_CK_CKMODE_EN,
						 FLD_DSI_CK_CKMODE_EN, 1);
				break;
			default:
				break;
			}

			/* LANE_0 */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_PHY0_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_0]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_PHY1AB_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_0]] + 1);

			/* LANE_1 */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_PHY1_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_1]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL1,
				FLD_MIPI_TX_PHY2BC_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_1]] + 1);

			/* LANE_2 */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_PHY2_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_2]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_CPHY0BC_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_2]] + 1);

			/* LANE_3 */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL1,
				FLD_MIPI_TX_PHY3_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_3]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL1,
				FLD_MIPI_TX_CPHYXXX_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_3]] + 1);

			/* CK LANE */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_PHYC_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_CK]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL0,
				FLD_MIPI_TX_CPHY1CA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_CK]] + 1);

			/* LPRX SETTING */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL1,
				FLD_MIPI_TX_LPRX0AB_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_RX]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL1,
				FLD_MIPI_TX_LPRX0BC_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_RX]] + 1);

			/* HS_DATA SETTING */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL2,
				FLD_MIPI_TX_PHY2_HSDATA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_2]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL2,
				FLD_MIPI_TX_PHY0_HSDATA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_0]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL2,
				FLD_MIPI_TX_PHYC_HSDATA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_CK]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL2,
				FLD_MIPI_TX_PHY1_HSDATA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_1]]);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PHY_SEL3,
				FLD_MIPI_TX_PHY3_HSDATA_SEL,
				pad_mapping[swap_base[MIPITX_PHY_LANE_3]]);
		} else {
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_CKMODE_EN,
					 FLD_DSI_CK_CKMODE_EN, 1);
		}
	}

	refill_mipitx_impedance(NULL);

	/* MIPI INIT */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		unsigned int tmp = 0;

		/* step 0: RG_DSI0_PLL_IBIAS = 0*/
		MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_PLL_CON4, 0x00FF12E0);
		/* BG_LPF_EN / BG_CORE_EN */
		MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_LANE_CON, 0x3FFF0180);
		udelay(500);
		/* BG_LPF_EN=1,TIEL_SEL=0 */
		MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_LANE_CON, 0x3FFF0080);
		if (atomic_read(&dsi_idle_flg) == 0) {
			/* Switch OFF each Lane */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_CTL_EN,
				FLD_DSI_D0_SW_CTL_EN, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_CTL_EN,
				FLD_DSI_D1_SW_CTL_EN, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_CTL_EN,
				FLD_DSI_D2_SW_CTL_EN, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_CTL_EN,
				FLD_DSI_D3_SW_CTL_EN, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_CTL_EN,
				FLD_DSI_CK_SW_CTL_EN, 0);
		}

		/* step 1: SDM_RWR_ON / SDM_ISO_EN */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_PWR,
				 FLD_AD_DSI_PLL_SDM_PWR_ON, 1);
		udelay(2); /* 1us */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_PWR,
				 FLD_AD_DSI_PLL_SDM_ISO_EN, 0);

		if (!data_Rate)
			continue;


		if (data_Rate > 2500) {
			DISPERR("mipitx Data Rate exceed limitation(%d)\n",
				    data_Rate);
			ASSERT(0);
		} else if (data_Rate >= 2000) { /* 2G ~ 2.5G */
			pcw_ratio = 1;
			posdiv    = 0;
				prediv    = 0;
			} else if (data_Rate >= 1000) { /* 1G ~ 2G */
				pcw_ratio = 2;
				posdiv    = 1;
				prediv    = 0;
			} else if (data_Rate >= 500) { /* 500M ~ 1G */
				pcw_ratio = 4;
				posdiv    = 2;
				prediv    = 0;
			} else if (data_Rate > 250) { /* 250M ~ 500M */
				pcw_ratio = 8;
				posdiv    = 3;
				prediv    = 0;
			} else if (data_Rate >= 125) { /* 125M ~ 250M */
				pcw_ratio = 16;
				posdiv    = 4;
				prediv    = 0;
			} else {
				DISPERR("dataRate is too low(%d)\n", data_Rate);
				ASSERT(0);
			}

			/* step 3 */
			/* PLL PCW config */
			tmp = _dsi_get_pcw(data_Rate, pcw_ratio);
			MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_PLL_CON0, tmp);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON1,
				FLD_RG_DSI_PLL_POSDIV, posdiv);

			/* SSC config */
			if (dsi_params->ssc_disable != 1) {
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON2,
				FLD_RG_DSI_PLL_SDM_SSC_PH_INIT, 1);
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON2,
				FLD_RG_DSI_PLL_SDM_SSC_PRD, 0x1B1);

				delta1 = (dsi_params->ssc_range == 0) ?
				delta1 : dsi_params->ssc_range;
				ASSERT(delta1 <= 8);
				pdelta1 = (delta1 * (data_Rate / 2) * pcw_ratio *
				   262144 + 281664) / 563329;

				MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON3,
				FLD_RG_DSI_PLL_SDM_SSC_DELTA, pdelta1);
				MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON3,
				FLD_RG_DSI_PLL_SDM_SSC_DELTA1,
				pdelta1);
				DDPMSG("PLL config:data_rate=%d,pcw_ratio=%d ",
				data_Rate, pcw_ratio);
				DDPMSG("delta1=%d,pdelta1=0x%x\n",
				delta1, pdelta1);
		}
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (data_Rate && (dsi_params->ssc_disable != 1))
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON2,
					 FLD_RG_DSI_PLL_SDM_SSC_EN, 1);
		else
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON2,
					 FLD_RG_DSI_PLL_SDM_SSC_EN, 0);
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* PLL EN */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON1,
				 FLD_RG_DSI_PLL_EN, 1);

		udelay(50);
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_SW_CTRL_CON4,
				 FLD_MIPI_TX_SW_ANA_CK_EN, 1);
	}
}

static void _DSI_PHY_clk_setting(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (_dsi_context[i].dsi_params.IsCphy)
			DSI_CPHY_clk_setting(module, cmdq, dsi_params);
		else
			DSI_DPHY_clk_setting(module, cmdq, dsi_params);
	}

}

/* DSI_MIPI_clk_change
 */
void DSI_MIPI_clk_change(enum DISP_MODULE_ENUM module, void *cmdq, int clk)
{
	unsigned int chg_status = 0;
	unsigned int pcw_ratio = 0;
	unsigned int pcw = 0;
	unsigned int pcw_floor = 0;
	unsigned int posdiv    = 0;
	unsigned int prediv    = 0;
	unsigned int i = DSI_MODULE_to_ID(module);

	DISPMSG("%s,clk=%d\n", __func__, clk);

	if (_is_power_on_status(module)) {
		if (clk != 0) {
			unsigned int tmp = 0;

			if (clk > 2500) {
				DISPERR("mipitx Data Rate exceed limit(%d)\n",
				 clk);
				ASSERT(0);
			} else if (clk >= 2000) { /* 2G ~ 2.5G */
				pcw_ratio = 1;
				posdiv    = 0;
				prediv    = 0;
			} else if (clk >= 1000) { /* 1G ~ 2G */
				pcw_ratio = 2;
				posdiv    = 1;
				prediv    = 0;
			} else if (clk >= 500) { /* 500M ~ 1G */
				pcw_ratio = 4;
				posdiv    = 2;
				prediv    = 0;
			} else if (clk > 250) { /* 250M ~ 500M */
				pcw_ratio = 8;
				posdiv    = 3;
				prediv    = 0;
			} else if (clk >= 125) { /* 125M ~ 250M */
				pcw_ratio = 16;
				posdiv    = 4;
				prediv    = 0;
			} else {
				DISPERR("dataRate is too low(%d)\n", clk);
				ASSERT(0);
			}

			pcw = clk * pcw_ratio / 26;
			pcw_floor = clk * pcw_ratio % 26;
			tmp = ((pcw & 0xFF) << 24) |
			(((256 * pcw_floor / 26) & 0xFF) << 16) |
			(((256 * (256 * pcw_floor % 26) / 26) & 0xFF) << 8) |
			((256 * (256 * (256 * pcw_floor % 26) % 26) / 26)
			& 0xFF);

			DISP_REG_SET(cmdq, DSI_PHY_REG[i]+MIPITX_PLL_CON0, tmp);

			DISP_REG_SET_FIELD(cmdq,
				FLD_RG_DSI_PLL_POSDIV,
				DSI_PHY_REG[i]+MIPITX_PLL_CON1,
				posdiv);

			chg_status =
				MIPITX_INREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_CON1,
				 FLD_RG_DSI_PLL_SDM_PCW_CHG);

			if (chg_status)
				DISP_REG_SET_FIELD(cmdq,
				 FLD_RG_DSI_PLL_SDM_PCW_CHG,
				 DSI_PHY_REG[i]+MIPITX_PLL_CON1,
				  0);
			else
				DISP_REG_SET_FIELD(cmdq,
				 FLD_RG_DSI_PLL_SDM_PCW_CHG,
				 DSI_PHY_REG[i]+MIPITX_PLL_CON1,
				  1);
		}
	}
}

void DSI_PHY_TIMCONFIG(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params);

int mipi_clk_change(enum DISP_MODULE_ENUM module, int en)
{
	int i, ret;
	struct cmdqRecStruct *handle = NULL;
	struct LCM_DSI_PARAMS *dsi_params;
	bool mod_vfp, mod_vbp, mod_vsa;
	bool mod_hfp, mod_hbp, mod_hsa;
	unsigned int data_rate;
	unsigned int _vfp, _vbp, _vsa;
	unsigned int _hfp, _hbp, _hsa;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int j = 0;
	/*not consider dual pipe in*/
	struct dfps_info *dfps_params = NULL;
#endif
	i = DSI_MODULE_BEGIN(module);
	dsi_params = &_dsi_context[i].dsi_params;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/*DynFPS get current fps info*/
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/*if not power on scenario
		 * if disp_fps !=0 means dynfps happen
		 */
		if (_dsi_context[i].disp_fps) {
			for (j = 0; j < dsi_params->dfps_num; j++) {
				if ((dsi_params->dfps_params)[j].fps ==
					_dsi_context[i].disp_fps) {
					dfps_params =
					&((dsi_params->dfps_params)[j]);
					DISPMSG("%s,disp_fps=%d\n",
					__func__, _dsi_context[i].disp_fps);
					break;
				}
			}
		}
	}
	if (dfps_params &&
		dfps_params->dynamic_switch_mipi == 0) {
		DISPMSG("%s,not support mipi hopping for %d\n",
			__func__, _dsi_context[i].disp_fps);
		return 0;
	}
#endif
	_vfp = dsi_params->vertical_frontporch_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->vertical_frontporch_dyn)
			_vfp = dfps_params->vertical_frontporch_dyn;
	}
#endif
	_vbp = dsi_params->vertical_backporch_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->vertical_backporch_dyn)
			_vbp = dfps_params->vertical_backporch_dyn;
	}
#endif
	_vsa = dsi_params->vertical_sync_active_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->vertical_sync_active_dyn)
			_vsa = dfps_params->vertical_sync_active_dyn;
	}
#endif
	_hfp = dsi_params->horizontal_frontporch_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->horizontal_frontporch_dyn)
			_hfp = dfps_params->horizontal_frontporch_dyn;
	}
#endif
	_hbp = dsi_params->horizontal_backporch_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->horizontal_backporch_dyn)
			_hbp = dfps_params->horizontal_backporch_dyn;
	}
#endif
	_hsa = dsi_params->horizontal_sync_active_dyn;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	if (dfps_params) {
		if (dfps_params->horizontal_sync_active_dyn)
			_hsa = dfps_params->horizontal_sync_active_dyn;
	}
#endif
	mod_vfp = !!_vfp;
	mod_vbp = !!_vbp;
	mod_vsa = !!_vsa;
	mod_hfp = !!_hfp;
	mod_hbp = !!_hbp;
	mod_hsa = !!_hsa;
	if (en) {
		data_rate = dsi_params->data_rate_dyn != 0 ?
			dsi_params->data_rate_dyn :
			dsi_params->PLL_CLOCK_dyn * 2;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		if (dfps_params) {
			if (dfps_params->data_rate_dyn ||
				dfps_params->PLL_CLOCK_dyn)
				data_rate =
				dfps_params->data_rate_dyn != 0 ?
				dfps_params->data_rate_dyn :
				dfps_params->PLL_CLOCK_dyn * 2;
		}
#endif
		mipi_clk_change_sta = 1;
	} else {
		data_rate = dsi_params->data_rate != 0 ?
			dsi_params->data_rate : dsi_params->PLL_CLOCK * 2;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		if (dfps_params) {
			if (dfps_params->data_rate ||
				dfps_params->PLL_CLOCK)
				data_rate =
				dfps_params->data_rate != 0 ?
				dfps_params->data_rate :
				dfps_params->PLL_CLOCK * 2;
		}
#endif
		mipi_clk_change_sta = 0;
	}
	if (_is_power_on_status(module) == 0) {
		DISPMSG("%s,suspend,skip hopping\n", __func__);
		return 0;
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	DISPMSG("%s,mipi_clk_change_sta=%d\n",
		__func__, mipi_clk_change_sta);
	DISPMSG("%s,_vfp:%d,_vbp:%d,_vsa:%d\n",
		__func__, _vfp, _vbp, _vsa);
	DISPMSG("%s,_hfp=%d,_hbp:%d,_hsa:%d\n",
		__func__, _hfp, _hbp, _hsa);
	/*keep mipi hopping use the same GCE  thread as dynfps
	 * can avoid competition between mipi hopping and dynfps in GCE
	 * keep GCE use the same cmd params as CPU calcuated params
	 */
	ret = cmdqRecCreate(CMDQ_SCENARIO_DISP_ESD_CHECK, &handle);
	if (ret) {
		DISPMSG("%s:Fail to create cmdq handle\n", __func__);
		return -1;
	}
	cmdqRecReset(handle);
	i = DSI_MODULE_BEGIN(module);
	/*for keep mipi hopping run after dynfps in GCE,
	 * same as CPU sequence
	 */
	cmdqRecWaitNoClear(handle,
				CMDQ_EVENT_MUTEX0_STREAM_EOF);
#endif
	if (DSI_REG[i]->DSI_MODE_CTRL.MODE) {
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
		/*mipi clock changed, cycle time & ui are also changed
		 * need re-calculate the phy timing
		 * can not change without  stop vdo mode
		 * but for mipi hopping we shouldn't stop vdo mode
		 * so skip this step now
		 */
		/*DSI_PHY_TIMCONFIG(module, handle, dsi_params);*/
#endif
		DISPINFO("%s,re-calc vdo timing\n", __func__);
		DSI_Calc_VDO_Timing(module, dsi_params);
	}
#ifndef CONFIG_MTK_HIGH_FRAME_RATE
	ret = cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
	if (ret) {
		DISPMSG("%s:Fail to create cmdq handle\n", __func__);
		return -1;
	}
	cmdqRecReset(handle);
#endif
	if (DSI_REG[i]->DSI_MODE_CTRL.MODE) {
		if (mod_vfp)
			ddp_dsi_porch_setting(module, handle, DSI_VFP,
					_dsi_context[i].vfp);
		if (mod_vbp)
			ddp_dsi_porch_setting(module, handle, DSI_VBP,
					_dsi_context[i].vbp);
		if (mod_vsa)
			ddp_dsi_porch_setting(module, handle, DSI_VSA,
					_dsi_context[i].vsa);

		if (mod_hbp)
			ddp_dsi_porch_setting(module, handle, DSI_HBP,
					_dsi_context[i].hbp_byte);
		if (mod_hfp)
			ddp_dsi_porch_setting(module, handle, DSI_HFP,
					_dsi_context[i].hfp_byte);
		if (mod_hsa)
			ddp_dsi_porch_setting(module, handle, DSI_HSA,
					_dsi_context[i].hsa_byte);
	}
	DSI_MIPI_clk_change(module, handle, data_rate);
	cmdqRecFlush(handle);
	cmdqRecDestroy(handle);
	DISPMSG("%s,mipi_clk_change_sta=%d done\n",
			__func__, mipi_clk_change_sta);
	return 0;
}


int mipi_clk_change_by_data_rate(int en, int mipi_data_rate)
{
	struct cmdqRecStruct *handle = NULL;

	DISPMSG("%s, mipi_data_rate=%d, en=%d\n",
		__func__, mipi_data_rate, en);

	_primary_path_lock(__func__);

	if (en) {
		def_data_rate = mipi_data_rate;

		/*TODO: ssc_disable = 1 */
		/* if need disable ssc and need re-change hbp or hfp */
		/* and need add parameter to DSI_MIPI_clk_change */
		/* MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON2,*/
		/* FLD_RG_DSI_PLL_SDM_SSC_EN, 0);*/
	} else {
		struct LCM_DSI_PARAMS *dsi_params =
			&(_dsi_context[0].dsi_params);
		unsigned int data_rate = dsi_params->data_rate != 0 ?
			dsi_params->data_rate : dsi_params->PLL_CLOCK * 2;

		def_data_rate = data_rate;
	}

	if (_is_power_on_status(DISP_MODULE_DSI0)) {
		cmdqRecCreate(CMDQ_SCENARIO_PRIMARY_DISP, &handle);
		cmdqRecReset(handle);

		/* 2.wait mutex0_stream_eof: only used for video mode */
		cmdqRecWaitNoClear(handle, CMDQ_EVENT_MUTEX0_STREAM_EOF);

		DSI_MIPI_clk_change(DISP_MODULE_DSI0, handle, def_data_rate);
		cmdqRecFlushAsync(handle);
		cmdqRecDestroy(handle);
	}

	_primary_path_unlock(__func__);

	return 0;
}

/**
 * DSI_PHY_clk_switch
 *
 * mipi init / deinit flow
 */

void DSI_CPHY_clk_switch(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq, int on)
{
	int i = 0;


	/* can't use cmdq for this */
	ASSERT(cmdq == NULL);

	if (on) {
		_DSI_PHY_clk_setting(module, cmdq,
			&(_dsi_context[i].dsi_params));
	} else {
		for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
			/* disable mipi clock */
			/* step 0 */
			/* PLL DISABLE */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_CON1,
				FLD_RG_DSI_PLL_EN,	0);

			/* step 1 */
			/* SDM_RWR_ON / SDM_ISO_EN */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_PWR,
				FLD_AD_DSI_PLL_SDM_ISO_EN, 1);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i]+MIPITX_PLL_PWR,
				FLD_AD_DSI_PLL_SDM_PWR_ON, 0);

			/* step 2 */
			/* BG_LPF_EN=0 */
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_LANE_CON,
				0x3FFF0080);
			/* BG_CORE_EN=1 */
			MIPITX_OUTREG32(DSI_PHY_REG[i]+MIPITX_LANE_CON,
				0x3FFF0000);

			/* mdelay(1); */
		}
	}

}

void DSI_DPHY_clk_switch(enum DISP_MODULE_ENUM module,
			struct cmdqRecStruct *cmdq, int on)
{
	int i = 0;

	/* can't use cmdq for this */
	ASSERT(cmdq == NULL);

	if (on) {
		_DSI_PHY_clk_setting(module, cmdq,
				     &(_dsi_context[i].dsi_params));
		return;
	}

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/* disable mipi clock */
		/* step 0: PLL DISABLE */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_CON1,
				 FLD_RG_DSI_PLL_EN, 0);
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_SW_CTRL_CON4,
				 FLD_MIPI_TX_SW_ANA_CK_EN, 0);

		/* step 1: SDM_RWR_ON / SDM_ISO_EN */
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
				 FLD_AD_DSI_PLL_SDM_ISO_EN, 1);
		MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_PLL_PWR,
				 FLD_AD_DSI_PLL_SDM_PWR_ON, 0);

		if (atomic_read(&dsi_idle_flg) == 1) {
			/* BG_LPF_EN=0, TIEL_SEL=1 */
			MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
				0x3FFF0180);
		} else {
			/* keep LP00 */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
					MIPITX_D2_SW_LPTX_PRE_OE,
					FLD_DSI_D2_SW_LPTX_PRE_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_LPTX_OE,
					FLD_DSI_D2_SW_LPTX_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_LPTX_DP,
					FLD_DSI_D2_SW_LPTX_DP, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_LPTX_DN,
					FLD_DSI_D2_SW_LPTX_DN, 0);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
					MIPITX_D0_SW_LPTX_PRE_OE,
					FLD_DSI_D0_SW_LPTX_PRE_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_LPTX_OE,
					FLD_DSI_D0_SW_LPTX_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_LPTX_DP,
					FLD_DSI_D0_SW_LPTX_DP, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_LPTX_DN,
					FLD_DSI_D0_SW_LPTX_DN, 0);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
					MIPITX_CK_SW_LPTX_PRE_OE,
					FLD_DSI_CK_SW_LPTX_PRE_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_LPTX_OE,
					FLD_DSI_CK_SW_LPTX_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_LPTX_DP,
					FLD_DSI_CK_SW_LPTX_DP, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_LPTX_DN,
					FLD_DSI_CK_SW_LPTX_DN, 0);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
					MIPITX_D1_SW_LPTX_PRE_OE,
					FLD_DSI_D1_SW_LPTX_PRE_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_LPTX_OE,
					FLD_DSI_D1_SW_LPTX_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_LPTX_DP,
					FLD_DSI_D1_SW_LPTX_DP, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_LPTX_DN,
					FLD_DSI_D1_SW_LPTX_DN, 0);

			MIPITX_OUTREGBIT(DSI_PHY_REG[i] +
					MIPITX_D3_SW_LPTX_PRE_OE,
					FLD_DSI_D3_SW_LPTX_PRE_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_LPTX_OE,
					FLD_DSI_D3_SW_LPTX_OE, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_LPTX_DP,
					FLD_DSI_D3_SW_LPTX_DP, 0);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_LPTX_DN,
					FLD_DSI_D3_SW_LPTX_DN, 0);

			/* Switch ON each Lane */
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D0_SW_CTL_EN,
				FLD_DSI_D0_SW_CTL_EN, 1);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D1_SW_CTL_EN,
				FLD_DSI_D1_SW_CTL_EN, 1);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D2_SW_CTL_EN,
				FLD_DSI_D2_SW_CTL_EN, 1);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_D3_SW_CTL_EN,
				FLD_DSI_D3_SW_CTL_EN, 1);
			MIPITX_OUTREGBIT(DSI_PHY_REG[i] + MIPITX_CK_SW_CTL_EN,
				FLD_DSI_CK_SW_CTL_EN, 1);
			/* step 2 */
			/* BG_LPF_EN=0, TIEL_SEL=1 */
			MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
				0x3FFF0180);
			/* BG_CORE_EN=0 */
			MIPITX_OUTREG32(DSI_PHY_REG[i] + MIPITX_LANE_CON,
				0x3FFF0100);
		}
	}
}

/**
 * DSI_PHY_clk_switch
 *
 * mipi init / deinit flow
 */
void DSI_PHY_clk_switch(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, int on)
{
	int i = 0;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (_dsi_context[i].dsi_params.IsCphy)
			DSI_CPHY_clk_switch(module, cmdq, on);
		else
			DSI_DPHY_clk_switch(module, cmdq, on);
	}
}


void DSI_CPHY_TIMCONFIG(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
		       struct LCM_DSI_PARAMS *dsi_params)
{
	struct DSI_PHY_TIMCON0_REG timcon0;
	struct DSI_PHY_TIMCON1_REG timcon1;
	struct DSI_PHY_TIMCON2_REG timcon2;
	struct DSI_PHY_TIMCON3_REG timcon3;
	int i = 0;
	unsigned int lane_no;
	unsigned int cycle_time = 0;
	unsigned int ui = 0;
	unsigned int hs_trail;
	unsigned int _PLL_CLOCK = 0;
	unsigned int _data_rate = 0;
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	unsigned int j = 0;
	struct dfps_info *dfps_params = NULL;
#endif

#ifdef CONFIG_FPGA_EARLY_PORTING
	/* sync from cmm */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, 0x02000102);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, 0x010a0308);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, 0x02000100);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, 0x00010701);

		DISPCHECK("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n", __func__,
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
	return;
#endif
	/*Dynfps, should use data_rate_dyn?
	 *dsi params can be chagned?
	 */
	lane_no = dsi_params->LANE_NUM;
	/*for mipi hopping*/
	if (!mipi_clk_change_sta) {
		_data_rate = dsi_params->data_rate;
		_PLL_CLOCK = dsi_params->PLL_CLOCK;
	} else {
		_data_rate = dsi_params->data_rate_dyn;
		_PLL_CLOCK = dsi_params->PLL_CLOCK_dyn;
	}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
	/*DynFPS, ToDo move to a function */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		/*if not power on scenario
		 * if disp_fps !=0 means dynfps already happened
		 * for example path switch or esd check recovery after dynfps
		 */
		if (_dsi_context[i].disp_fps &&
			(_dsi_context[i].dynfps_chg_index &
				DYNFPS_DSI_MIPI_CLK)) {
			for (j = 0; j < dsi_params->dfps_num; j++) {
				if ((dsi_params->dfps_params)[j].fps ==
					_dsi_context[i].disp_fps) {
					dfps_params =
					&((dsi_params->dfps_params)[j]);
					DISPMSG("%s,disp_fps=%d\n",
					__func__, _dsi_context[i].disp_fps);
					break;
				}
			}
			if (dfps_params) {
				if (!mipi_clk_change_sta) {
					if (dfps_params->data_rate != 0 ||
						dfps_params->PLL_CLOCK != 0)
						_data_rate =
							dfps_params->data_rate;
						_PLL_CLOCK =
							dfps_params->PLL_CLOCK;
				} else {
					if (dfps_params->data_rate_dyn != 0 ||
						dfps_params->PLL_CLOCK_dyn != 0)
						_data_rate =
						dfps_params->data_rate_dyn;
						_PLL_CLOCK =
						dfps_params->PLL_CLOCK_dyn;

				}
			}
		}
	}
#endif
	if (_data_rate != 0) {
		ui = 1000 / _data_rate + 0x01;
		cycle_time = 8000 / _data_rate + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
			"%s, Cycle Time = %d, interval = %d, lane# = %d\n",
			__func__, cycle_time, ui, lane_no);
	} else if (_PLL_CLOCK) {
		ui = 1000 / (_PLL_CLOCK * 2) + 0x01;
		cycle_time = 8000 / (_PLL_CLOCK * 2) + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
			"[DISP]%s, cycle = %d(ns),ui= %d(ns)., lane# = %d\n",
			__func__, cycle_time, ui, lane_no);
	} else {
		DISPCHECK("[dsi_dsi.c] PLL clock should not be 0!\n");
		ASSERT(0);
	}

	if (cycle_time == 0) {
		DISPCHECK("[dsi_dsi.c] cycle_time should not be 0!\n");
		return;
	}


#define NS_TO_CYCLE(n, c)	((n) / (c))

	hs_trail = (dsi_params->HS_TRAIL == 0) ?
		32 : dsi_params->HS_TRAIL;

	/* +3 is recommended from designer becauase of HW latency */
	timcon0.HS_TRAIL = hs_trail;
	//HS_PRPR=DATA_RATE*50.5/7000+1
	timcon0.HS_PRPR = (dsi_params->HS_PRPR == 0) ?
		(NS_TO_CYCLE(_PLL_CLOCK * 101, 7000) + 1) :
		dsi_params->HS_PRPR;

	timcon0.HS_ZERO = (dsi_params->HS_ZERO == 0) ?
		48 : dsi_params->HS_ZERO;

	timcon0.LPX = (dsi_params->LPX == 0) ?
		(NS_TO_CYCLE(_PLL_CLOCK * 2 * 75, 7000)
		+ 0x01) : dsi_params->LPX;

	timcon1.TA_GET = (dsi_params->TA_GET == 0) ?
		(0x5 * timcon0.LPX) : dsi_params->TA_GET;
	timcon1.TA_SURE = (dsi_params->TA_SURE == 0) ?
		(0x3 * timcon0.LPX / 0x2) : dsi_params->TA_SURE;
	timcon1.TA_GO = (dsi_params->TA_GO == 0) ?
		(0x4 * timcon0.LPX) : dsi_params->TA_GO;
	/* --------------------------------------------------------------
	 * NT35510 need fine tune timing
	 * Data_hs_exit = data_rate*112.5/7000+1
	 * Clk_post = 60 ns + 128 UI.
	 * --------------------------------------------------------------
	 */
	timcon1.DA_HS_EXIT  = (dsi_params->DA_HS_EXIT == 0) ?
		(NS_TO_CYCLE(_PLL_CLOCK * 225, 7000) + 1) :
		dsi_params->DA_HS_EXIT;

	timcon2.CLK_TRAIL = ((dsi_params->CLK_TRAIL == 0) ?
		NS_TO_CYCLE(0x60, cycle_time) :
		dsi_params->CLK_TRAIL) + 0x01;
	/* CLK_TRAIL can't be 1. */
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

	timcon2.CONT_DET = dsi_params->CONT_DET;
	timcon2.CLK_ZERO = (dsi_params->CLK_ZERO == 0) ?
		NS_TO_CYCLE(0x190, cycle_time) :
		dsi_params->CLK_ZERO;

	timcon3.CLK_HS_PRPR = (dsi_params->CLK_HS_PRPR == 0) ?
		NS_TO_CYCLE(0x40, cycle_time) :
		dsi_params->CLK_HS_PRPR;

	if (timcon3.CLK_HS_PRPR < 1)
		timcon3.CLK_HS_PRPR = 1;

	timcon3.CLK_HS_EXIT = (dsi_params->CLK_HS_EXIT == 0) ?
		(0x2 * timcon0.LPX) : dsi_params->CLK_HS_EXIT;
	timcon3.CLK_HS_POST = (dsi_params->CLK_HS_POST == 0) ?
		NS_TO_CYCLE((0x60 + 0x34 * ui), cycle_time) :
		dsi_params->CLK_HS_POST;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
		"[DISP] - kernel - %s, HS_TRAIL = %d, HS_ZERO = %d,HS_PRPR = %d, LPX = %d, TA_GET = %d, TA_SURE = %d, TA_GO = %d, CLK_TRAIL = %d, CLK_ZERO = %d, CLK_HS_PRPR = %d\n",
		__func__, timcon0.HS_TRAIL, timcon0.HS_ZERO,
		timcon0.HS_PRPR, timcon0.LPX,
		timcon1.TA_GET, timcon1.TA_SURE,
		timcon1.TA_GO, timcon2.CLK_TRAIL,
		timcon2.CLK_ZERO, timcon3.CLK_HS_PRPR);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		unsigned int value = 0;

		_dsi_context[i].data_phy_cycle = timcon0.HS_PRPR +
			timcon0.HS_ZERO + timcon1.DA_HS_EXIT +
			timcon0.LPX + 5;
		_dsi_context[i].HS_TRAIL = timcon0.HS_TRAIL;

		value = REG_FLD_VAL(FLD_LPX, timcon0.LPX) +
			REG_FLD_VAL(FLD_HS_PRPR, timcon0.HS_PRPR) +
			REG_FLD_VAL(FLD_HS_ZERO, timcon0.HS_ZERO) +
			REG_FLD_VAL(FLD_HS_TRAIL, timcon0.HS_TRAIL);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, value);

		value = REG_FLD_VAL(FLD_TA_GO, timcon1.TA_GO) +
			REG_FLD_VAL(FLD_TA_SURE, timcon1.TA_SURE) +
			REG_FLD_VAL(FLD_TA_GET, timcon1.TA_GET) +
			REG_FLD_VAL(FLD_DA_HS_EXIT, timcon1.DA_HS_EXIT);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, value);

		value = REG_FLD_VAL(FLD_CONT_DET, timcon2.CONT_DET) +
			REG_FLD_VAL(FLD_DA_HS_SYNC, 0x06) + /* default value */
			REG_FLD_VAL(FLD_CLK_ZERO, timcon2.CLK_ZERO) +
			REG_FLD_VAL(FLD_CLK_TRAIL, timcon2.CLK_TRAIL);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, value);

		value = REG_FLD_VAL(FLD_CLK_HS_PRPR, timcon3.CLK_HS_PRPR) +
			REG_FLD_VAL(FLD_CLK_HS_POST, timcon3.CLK_HS_POST) +
			REG_FLD_VAL(FLD_CLK_HS_EXIT, timcon3.CLK_HS_EXIT);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, value);

		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CPHY_CON0, 0x012c0003);

		DISPINFO("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n",
			__func__,
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
}

void DSI_DPHY_TIMCONFIG(enum DISP_MODULE_ENUM module,
			struct cmdqRecStruct *cmdq,
			struct LCM_DSI_PARAMS *dsi_params)
{
	struct DSI_PHY_TIMCON0_REG timcon0;
	struct DSI_PHY_TIMCON1_REG timcon1;
	struct DSI_PHY_TIMCON2_REG timcon2;
	struct DSI_PHY_TIMCON3_REG timcon3;
	int i = 0;
	unsigned int lane_no;
	unsigned int cycle_time = 0;
	unsigned int ui = 0;
	unsigned int hs_trail_m, hs_trail_n;
	unsigned char timcon_temp;
	unsigned int temp_data_rate = 0;

#ifdef CONFIG_FPGA_EARLY_PORTING
	/* sync from cmm */
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, 0x02000102);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, 0x010a0308);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, 0x02000100);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, 0x00010701);

		DISPCHECK("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n", __func__,
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			  INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
	return;
#endif

#if 0
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, 0x140f0708);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, 0x10280c20);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, 0x14280000);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, 0x00101a06);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON4, 0x00023000);
	}
	return;
#endif
	lane_no = dsi_params->LANE_NUM;
	if (dsi_params->data_rate != 0) {
		ui = 1000 / dsi_params->data_rate + 0x01;
		cycle_time = 8000 / dsi_params->data_rate + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
			"%s, Cycle Time = %d, interval = %d, lane# = %d\n",
			__func__, cycle_time, ui, lane_no);
	} else if (dsi_params->PLL_CLOCK) {
		ui = 1000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		cycle_time = 8000 / (dsi_params->PLL_CLOCK * 2) + 0x01;
		DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
			       "[DISP] - kernel - %s, Cycle Time = %d(ns), Unit Interval = %d(ns)., lane# = %d\n",
			       __func__, cycle_time, ui, lane_no);
	} else {
		DISPERR("[dsi_dsi.c] PLL clock should not be 0!\n");
		ASSERT(0);
		return;
	}

#define NS_TO_CYCLE(n, c)	((n) / (c))

	if (dsi_params->data_rate != 0)
		temp_data_rate = dsi_params->data_rate;
	else
		temp_data_rate = dsi_params->PLL_CLOCK * 2;

	hs_trail_m = 1;
	hs_trail_n = (dsi_params->HS_TRAIL == 0) ?
				(NS_TO_CYCLE(((hs_trail_m * 0x4 * ui) + 0x50)
				* temp_data_rate, 0x1F40) + 0x1) :
				dsi_params->HS_TRAIL;
	/* +3 is recommended from designer becauase of HW latency */
	timcon0.HS_TRAIL = (hs_trail_m > hs_trail_n) ? hs_trail_m : hs_trail_n;

	timcon0.HS_PRPR = (dsi_params->HS_PRPR == 0) ?
			(NS_TO_CYCLE((0x40 + 0x5 * ui), cycle_time) + 0x1) :
			dsi_params->HS_PRPR;
	/* HS_PRPR can't be 1. */
	if (timcon0.HS_PRPR < 1)
		timcon0.HS_PRPR = 1;

	timcon0.HS_ZERO = (dsi_params->HS_ZERO == 0) ?
				NS_TO_CYCLE((0xC8 + 0x0a * ui), cycle_time) :
				dsi_params->HS_ZERO;
	timcon_temp = timcon0.HS_PRPR;
	if (timcon_temp < timcon0.HS_ZERO)
		timcon0.HS_ZERO -= timcon0.HS_PRPR;

	timcon0.LPX = (dsi_params->LPX == 0) ?
		(NS_TO_CYCLE(temp_data_rate * 0x4B, 0x1F40)  + 0x1) :
								dsi_params->LPX;
	if (timcon0.LPX < 1)
		timcon0.LPX = 1;

	timcon1.TA_GET = (dsi_params->TA_GET == 0) ?  (0x5 * timcon0.LPX) :
							dsi_params->TA_GET;
	timcon1.TA_SURE = (dsi_params->TA_SURE == 0) ?
				(0x3 * timcon0.LPX / 0x2) : dsi_params->TA_SURE;
	timcon1.TA_GO = (dsi_params->TA_GO == 0) ?  (0x4 * timcon0.LPX) :
							dsi_params->TA_GO;
	/* --------------------------------------------------------------
	 * NT35510 need fine tune timing
	 * Data_hs_exit = 60 ns + 128UI
	 * Clk_post = 60 ns + 128 UI.
	 * --------------------------------------------------------------
	 */
	timcon1.DA_HS_EXIT = (dsi_params->DA_HS_EXIT == 0) ?
				(0x2 * timcon0.LPX) : dsi_params->DA_HS_EXIT;

	timcon2.CLK_TRAIL = ((dsi_params->CLK_TRAIL == 0) ?
				NS_TO_CYCLE(0x64 * temp_data_rate,
				0x1F40) : dsi_params->CLK_TRAIL) + 0x01;
	/* CLK_TRAIL can't be 1. */
	if (timcon2.CLK_TRAIL < 2)
		timcon2.CLK_TRAIL = 2;

	timcon2.CONT_DET = dsi_params->CONT_DET;
	timcon2.CLK_ZERO = (dsi_params->CLK_ZERO == 0) ?
						NS_TO_CYCLE(0x190, cycle_time) :
						dsi_params->CLK_ZERO;

	timcon3.CLK_HS_PRPR = (dsi_params->CLK_HS_PRPR == 0) ?
				NS_TO_CYCLE(0x50 * temp_data_rate,
				0x1F40) : dsi_params->CLK_HS_PRPR;

	if (timcon3.CLK_HS_PRPR < 1)
		timcon3.CLK_HS_PRPR = 1;

	timcon3.CLK_HS_EXIT = (dsi_params->CLK_HS_EXIT == 0) ?
				(0x2 * timcon0.LPX) : dsi_params->CLK_HS_EXIT;
	timcon3.CLK_HS_POST = (dsi_params->CLK_HS_POST == 0) ?
				NS_TO_CYCLE((0x60 + 0x34 * ui), cycle_time) :
				dsi_params->CLK_HS_POST;

	DISP_LOG_PRINT(ANDROID_LOG_INFO, "DSI",
			"[DISP] - kernel - %s, HS_TRAIL = %d, HS_ZERO = %d, HS_PRPR = %d, LPX = %d, TA_GET = %d, TA_SURE = %d, TA_GO = %d, CLK_TRAIL = %d, CLK_ZERO = %d, CLK_HS_PRPR = %d\n",
			__func__, timcon0.HS_TRAIL, timcon0.HS_ZERO,
			timcon0.HS_PRPR, timcon0.LPX,
			timcon1.TA_GET, timcon1.TA_SURE,
			timcon1.TA_GO, timcon2.CLK_TRAIL,
			timcon2.CLK_ZERO, timcon3.CLK_HS_PRPR);

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		unsigned int value = 0;

		value = REG_FLD_VAL(FLD_LPX, timcon0.LPX) +
		    REG_FLD_VAL(FLD_HS_PRPR, timcon0.HS_PRPR) +
		    REG_FLD_VAL(FLD_HS_ZERO, timcon0.HS_ZERO) +
		    REG_FLD_VAL(FLD_HS_TRAIL, timcon0.HS_TRAIL);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON0, value);

		value = REG_FLD_VAL(FLD_TA_GO, timcon1.TA_GO) +
		    REG_FLD_VAL(FLD_TA_SURE, timcon1.TA_SURE) +
		    REG_FLD_VAL(FLD_TA_GET, timcon1.TA_GET) +
		    REG_FLD_VAL(FLD_DA_HS_EXIT, timcon1.DA_HS_EXIT);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON1, value);

		value = REG_FLD_VAL(FLD_CONT_DET, timcon2.CONT_DET) +
		    REG_FLD_VAL(FLD_DA_HS_SYNC, 1) + /* default value */
		    REG_FLD_VAL(FLD_CLK_ZERO, timcon2.CLK_ZERO) +
		    REG_FLD_VAL(FLD_CLK_TRAIL, timcon2.CLK_TRAIL);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON2, value);

		value = REG_FLD_VAL(FLD_CLK_HS_PRPR, timcon3.CLK_HS_PRPR) +
		    REG_FLD_VAL(FLD_CLK_HS_POST, timcon3.CLK_HS_POST) +
		    REG_FLD_VAL(FLD_CLK_HS_EXIT, timcon3.CLK_HS_EXIT);
		DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_PHY_TIMECON3, value);

		DISPINFO("%s, 0x%08x,0x%08x,0x%08x,0x%08x\n",
			__func__,
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON0),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON1),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON2),
			INREG32(&DSI_REG[i]->DSI_PHY_TIMECON3));
	}
}

void DSI_PHY_TIMCONFIG(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{
	int i = 0;

	DDPDUMP("===>%s start\n", __func__);
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (_dsi_context[i].dsi_params.IsCphy)
			DSI_CPHY_TIMCONFIG(module, cmdq, dsi_params);
		else
			DSI_DPHY_TIMCONFIG(module, cmdq, dsi_params);
	}
	DDPDUMP("<===%s done\n", __func__);
}


int DSI_enable_checksum(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq)
{
	int i, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_DEBUG_SEL_REG,
			DSI_REG[i]->DSI_DEBUG_SEL, CHKSUM_REC_EN, 1);
	}
	return 0;
}

enum DSI_STATUS DSI_Start(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq)
{
	if (module == DISP_MODULE_DSI1) {
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[1]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[1]->DSI_START, DSI_START, 1);
	} else {
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, DSI_START, 1);
	}

	return DSI_STATUS_OK;
}

void DSI_Set_VM_CMD(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq)
{
	int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	if (module != DISP_MODULE_DSIDUAL) {
		for (i = 0; i < module_num; i++) {
			DSI_OUTREGBIT(cmdq, struct DSI_VM_CMD_CON_REG,
				DSI_REG[i]->DSI_VM_CMD_CON, TS_VFP_EN, 1);
			DSI_OUTREGBIT(cmdq, struct DSI_VM_CMD_CON_REG,
				DSI_REG[i]->DSI_VM_CMD_CON, VM_CMD_EN, 1);

			DDPMSG("DSI_Set_VM_CMD");
		}
	} else {
		DSI_OUTREGBIT(cmdq, struct DSI_VM_CMD_CON_REG,
			DSI_REG[i]->DSI_VM_CMD_CON, TS_VFP_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_VM_CMD_CON_REG,
			DSI_REG[i]->DSI_VM_CMD_CON, VM_CMD_EN, 1);
	}
}

enum DSI_STATUS DSI_EnableVM_CMD(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq)
{
	int i = 0, module_num;

	if (cmdq)
		DSI_MASKREG32(cmdq, &DSI_REG[0]->DSI_INTSTA,
			0x00000020, 0x00000000);

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	for (i = 0; i < module_num; i++) {
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, VM_CMD_START, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, VM_CMD_START, 1);
	}

	if (cmdq) {
		DSI_POLLREG32(cmdq, &DSI_REG[0]->DSI_INTSTA,
				0x00000020, 0x00000020);
		DSI_MASKREG32(cmdq, &DSI_REG[0]->DSI_INTSTA,
				0x00000020, 0x00000000);
	}
	return DSI_STATUS_OK;
}

/* return value: the data length we got */
UINT32 DSI_dcs_read_lcm_reg_v2(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, UINT8 cmd, UINT8 *buffer,
	UINT8 buffer_size)
{
	unsigned int d = 0;
	UINT32 max_try_count = 5;
	UINT32 recv_data_cnt = 0;
	unsigned char packet_type;
	struct DSI_RX_DATA_REG read_data0;
	struct DSI_RX_DATA_REG read_data1;
	struct DSI_RX_DATA_REG read_data2;
	struct DSI_RX_DATA_REG read_data3;
	struct DSI_T0_INS t0;
	struct DSI_T0_INS t1;
	static const long WAIT_TIMEOUT = 2 * HZ; /* 2 sec */
	long ret;
	unsigned int i;
	struct t_condition_wq *waitq;

	/* illegal parameters */
	ASSERT(cmdq == NULL);
	if (buffer == NULL || buffer_size == 0) {
		DISPWARN("DSI Read Fail: buffer=%p and buffer_size=%d\n",
			buffer, (unsigned int)buffer_size);
		return 0;
	}

	if (module == DISP_MODULE_DSI0)
		d = 0;
	else if (module == DISP_MODULE_DSI1)
		d = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		d = 0;
	else
		return 0;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("DSI%d register addr invalid\n", i);
		return -1;
	}

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) {
		/* only cmd mode can read */
		DISPWARN("DSI Read Fail: DSI Mode is %d\n",
			DSI_REG[d]->DSI_MODE_CTRL.MODE);
		return 0;
	}

	do {

		if (max_try_count == 0) {
			DISPWARN("DSI Read Fail: try 5 times\n");
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}

		max_try_count--;
		recv_data_cnt = 0;

		/* 1. wait dsi not busy => can't read if dsi busy */
		dsi_wait_not_busy(module, NULL);

		/* 2. Check rd_rdy & cmd_done irq */
		if (DSI_REG[d]->DSI_INTEN.RD_RDY == 0) {
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				      DSI_REG[d]->DSI_INTEN, RD_RDY, 1);
		}

		if (DSI_REG[d]->DSI_INTEN.CMD_DONE == 0) {
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				      DSI_REG[d]->DSI_INTEN, CMD_DONE, 1);
		}

		ASSERT(DSI_REG[d]->DSI_INTEN.RD_RDY == 1);
		ASSERT(DSI_REG[d]->DSI_INTEN.CMD_DONE == 1);

		/* dump cmdq & rxdata */
		if (DSI_REG[d]->DSI_INTSTA.RD_RDY != 0 ||
			DSI_REG[d]->DSI_INTSTA.CMD_DONE != 0) {
			DISPERR("Last DSI Read Why not clear irq???\n");
			DISPERR("DSI_CMDQ_SIZE  : %d\n",
				AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
			for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE;
				i++) {
				DISPERR("DSI_CMDQ_DATA%d : 0x%08x\n", i,
					AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
			}
			DISPERR("DSI_RX_DATA0   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
			DISPERR("DSI_RX_DATA1   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
			DISPERR("DSI_RX_DATA2   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
			DISPERR("DSI_RX_DATA3   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));

			/* clear irq */
			DSI_OUTREGBIT(cmdq, struct DSI_INT_STATUS_REG,
				DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_STATUS_REG,
				DSI_REG[d]->DSI_INTSTA, CMD_DONE, 0);
		}

		/* 3. Send cmd */
		t0.CONFG = 0x04; /* BTA */
		t0.Data_ID = (cmd < 0xB0) ?
			DSI_DCS_READ_PACKET_ID :
			DSI_GERNERIC_READ_LONG_PACKET_ID;
		t0.Data0 = cmd;
		t0.Data1 = 0;

		/* set max return size */
		t1.CONFG = 0x00;
		t1.Data_ID = 0x37;
		t1.Data0 = buffer_size <= 10 ? buffer_size : 10;
		t1.Data1 = 0;

		DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
			AS_UINT32(&t1));
		DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[1],
			AS_UINT32(&t0));
		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2);

		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 0);
		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 1);

		/*
		 * the following code is to
		 * 1: wait read ready
		 * 2: read data
		 * 3: ack read ready
		 * 4: wait for CMDQ_DONE(interrupt handler do this op)
		 */
		waitq = &(_dsi_context[d].read_wq);
		ret = wait_event_timeout(waitq->wq,
			atomic_read(&(waitq->condition)), WAIT_TIMEOUT);
		atomic_set(&(waitq->condition), 0);
		if (ret == 0) {
			/* wait read ready timeout */
			DISPERR("DSI Read Fail: dsi wait read ready timeout\n");
			DSI_DumpRegisters(module, 2);

			/* do necessary reset here */
			DSI_OUTREGBIT(cmdq, struct DSI_RACK_REG,
				DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
			DSI_Reset(module, NULL);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}

		/* read data */
		DSI_OUTREG32(cmdq, &read_data0,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
		DSI_OUTREG32(cmdq, &read_data1,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
		DSI_OUTREG32(cmdq, &read_data2,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
		DSI_OUTREG32(cmdq, &read_data3,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));

		DSI_OUTREGBIT(cmdq, struct DSI_RACK_REG,
			DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
		ret = wait_event_timeout(_dsi_context[d].cmddone_wq.wq,
			!(DSI_REG[d]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			/* wait cmddone timeout */
			DISPERR("DSI Read Fail: dsi wait cmddone timeout\n");
			DSI_DumpRegisters(module, 2);
			DSI_Reset(module, NULL);
		}

		DISPDBG("DSI read begin i = %d --------------------\n",
			  5 - max_try_count);
		DISPDBG("DSI_RX_STA     : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_TRIG_STA));
		DISPDBG("DSI_CMDQ_SIZE  : %d\n",
			  AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
		for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
			DISPDBG("DSI_CMDQ_DATA%d : 0x%08x\n", i,
				  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
		}
		DISPDBG("DSI_RX_DATA0   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
		DISPDBG("DSI_RX_DATA1   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
		DISPDBG("DSI_RX_DATA2   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
		DISPDBG("DSI_RX_DATA3   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
		DISPDBG("DSI read end ----------------------------\n");

		packet_type = read_data0.byte0;

		DISPCHECK("DSI read packet_type is 0x%x\n", packet_type);

		/*
		 * 0x02: acknowledge & error report
		 * 0x11: generic short read response(1 byte return)
		 * 0x12: generic short read response(2 byte return)
		 * 0x1a: generic long read response
		 * 0x1c: dcs long read response
		 * 0x21: dcs short read response(1 byte return)
		 * 0x22: dcs short read response(2 byte return)
		 */
		if (packet_type == 0x1A || packet_type == 0x1C) {
			recv_data_cnt =
				read_data0.byte1 + read_data0.byte2 * 16;
			if (recv_data_cnt > 10) {
				DISPCHECK(
					"DSI read long packet data exceeds 10 bytes,size:%d\n",
					recv_data_cnt);
				recv_data_cnt = 10;
			}

			if (recv_data_cnt > buffer_size) {
				DISPCHECK(
					"DSI read long packet data exceeds buffer size,size:%d\n",
					recv_data_cnt);
				recv_data_cnt = buffer_size;
			}
			DISPCHECK("DSI read long packet size: %d\n",
				recv_data_cnt);

			if (recv_data_cnt <= 4) {
				memcpy((void *)buffer,
					(void *)&read_data1, recv_data_cnt);
			} else if (recv_data_cnt <= 8) {
				memcpy((void *)buffer,
					(void *)&read_data1, 4);
				memcpy((void *)buffer + 4,
					(void *)&read_data2,
				    recv_data_cnt - 4);
			} else {
				memcpy((void *)buffer,
					(void *)&read_data1, 4);
				memcpy((void *)buffer + 4,
					(void *)&read_data2, 4);
				memcpy((void *)buffer + 8, (void *)&read_data2,
				    recv_data_cnt - 8);
			}
		} else if (packet_type == 0x11 || packet_type == 0x12 ||
				packet_type == 0x21 || packet_type == 0x22) {
			if (packet_type == 0x11 || packet_type == 0x21)
				recv_data_cnt = 1;
			else
				recv_data_cnt = 2;

			if (recv_data_cnt > buffer_size) {
				DISPCHECK(
					"DSI read short packet data exceeds buffer size:%d\n",
					buffer_size);
				recv_data_cnt = buffer_size;
				memcpy((void *)buffer,
					(void *)&read_data0.byte1,
					recv_data_cnt);
			} else {
				memcpy((void *)buffer,
					(void *)&read_data0.byte1,
					recv_data_cnt);
			}
		} else if (packet_type == 0x02) {
			DISPCHECK("read return type is 0x02, re-read\n");
		} else {
			DISPCHECK("read return type is non-recognite: 0x%x\n",
				  packet_type);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}
	} while (packet_type == 0x02);
	/* here: we may receive a ACK packet which packet type is 0x02
	 * (incdicates some error happened)
	 * therefore we try re-read again until no ACK packet
	 * But: if it is a good way to keep re-trying ???
	 */

	DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN,
		RD_RDY, 0);
	return recv_data_cnt;
}


/* return value:  0 -- error; others -- the data length we got */
UINT32 DSI_dcs_read_lcm_reg_v3(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, char *out, struct dsi_cmd_desc *cmds,
	unsigned int len)
{
	unsigned int d = 0;
	UINT32 max_try_count = 5;
	UINT32 recv_data_cnt = 0;
	unsigned char packet_type;
	struct DSI_RX_DATA_REG read_data0;
	struct DSI_RX_DATA_REG read_data1;
	struct DSI_RX_DATA_REG read_data2;
	struct DSI_RX_DATA_REG read_data3;
	struct DSI_T0_INS t0;
	struct DSI_T0_INS t1;
	static const long WAIT_TIMEOUT = 2 * HZ; /* 2 sec */
	long ret;
	unsigned int i;
	struct t_condition_wq *waitq;
	UINT8 cmd, buffer_size, *buffer;
	unsigned char virtual_channel;

	buffer = out;
	buffer_size = (UINT8)len;
	cmd = (UINT8)cmds->dtype;
	virtual_channel = (unsigned char)cmds->vc;

	virtual_channel = ((virtual_channel << 6) | 0x3F);

	/* illegal parameters */
	ASSERT(cmdq == NULL);
	if (buffer == NULL || buffer_size == 0) {
		DISPWARN("DSI Read Fail: buffer=%p and buffer_size=%d\n",
			buffer, (unsigned int)buffer_size);
		return 0;
	}

	if (module == DISP_MODULE_DSI0)
		d = 0;
	else if (module == DISP_MODULE_DSI1)
		d = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		d = 0;
	else
		return 0;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("DSI%d register addr invalid\n", i);
		return -1;
	}

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) {
		/* only cmd mode can read */
		DISPWARN("DSI Read Fail: DSI Mode is %d\n",
			DSI_REG[d]->DSI_MODE_CTRL.MODE);
		return 0;
	}

	do {

		if (max_try_count == 0) {
			DISPWARN("DSI Read Fail: try 5 times\n");
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}

		max_try_count--;
		recv_data_cnt = 0;

		/* 1. wait dsi not busy => can't read if dsi busy */
		dsi_wait_not_busy(module, NULL);

		/* 2. Check rd_rdy & cmd_done irq */
		if (DSI_REG[d]->DSI_INTEN.RD_RDY == 0) {
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				      DSI_REG[d]->DSI_INTEN, RD_RDY, 1);
		}

		if (DSI_REG[d]->DSI_INTEN.CMD_DONE == 0) {
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				      DSI_REG[d]->DSI_INTEN, CMD_DONE, 1);
		}

		ASSERT(DSI_REG[d]->DSI_INTEN.RD_RDY == 1);
		ASSERT(DSI_REG[d]->DSI_INTEN.CMD_DONE == 1);

		/* dump cmdq & rxdata */
		if (DSI_REG[d]->DSI_INTSTA.RD_RDY != 0 ||
			DSI_REG[d]->DSI_INTSTA.CMD_DONE != 0) {
			DISPERR("Last DSI Read Why not clear irq???\n");
			DISPERR("DSI_CMDQ_SIZE  : %d\n",
				AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
			for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE;
				i++) {
				DISPERR("DSI_CMDQ_DATA%d : 0x%08x\n", i,
					AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
			}
			DISPERR("DSI_RX_DATA0   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
			DISPERR("DSI_RX_DATA1   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
			DISPERR("DSI_RX_DATA2   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
			DISPERR("DSI_RX_DATA3   : 0x%08x\n",
				  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));

			/* clear irq */
			DSI_OUTREGBIT(cmdq, struct DSI_INT_STATUS_REG,
				DSI_REG[d]->DSI_INTSTA, RD_RDY, 0);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_STATUS_REG,
				DSI_REG[d]->DSI_INTSTA, CMD_DONE, 0);
		}

		/* 3. Send cmd */
		t0.CONFG = 0x04; /* BTA */
		t0.Data_ID = (cmd < 0xB0) ?
			DSI_DCS_READ_PACKET_ID :
			DSI_GERNERIC_READ_LONG_PACKET_ID;
		t0.Data_ID = t0.Data_ID & virtual_channel;
		t0.Data0 = cmd;
		t0.Data1 = 0;

		/* set max return size */
		t1.CONFG = 0x00;
		t1.Data_ID = 0x37;
		t1.Data_ID = t1.Data_ID & virtual_channel;
		t1.Data0 = buffer_size <= 10 ? buffer_size : 10;
		t1.Data1 = 0;

		DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
			AS_UINT32(&t1));
		DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[1],
			AS_UINT32(&t0));
		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 2);

		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 0);
		DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 1);

		/*
		 * the following code is to
		 * 1: wait read ready
		 * 2: read data
		 * 3: ack read ready
		 * 4: wait for CMDQ_DONE(interrupt handler do this op)
		 */
		waitq = &(_dsi_context[d].read_wq);
		ret = wait_event_timeout(waitq->wq,
			atomic_read(&(waitq->condition)), WAIT_TIMEOUT);
		atomic_set(&(waitq->condition), 0);
		if (ret == 0) {
			/* wait read ready timeout */
			DISPERR("DSI Read Fail: dsi wait read ready timeout\n");
			DSI_DumpRegisters(module, 2);

			/* do necessary reset here */
			DSI_OUTREGBIT(cmdq, struct DSI_RACK_REG,
				DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
			DSI_Reset(module, NULL);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}

		/* read data */
		DSI_OUTREG32(cmdq, &read_data0,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
		DSI_OUTREG32(cmdq, &read_data1,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
		DSI_OUTREG32(cmdq, &read_data2,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
		DSI_OUTREG32(cmdq, &read_data3,
			AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));

		DSI_OUTREGBIT(cmdq, struct DSI_RACK_REG,
			DSI_REG[d]->DSI_RACK, DSI_RACK, 1);
		ret = wait_event_timeout(_dsi_context[d].cmddone_wq.wq,
			!(DSI_REG[d]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			/* wait cmddone timeout */
			DISPERR("DSI Read Fail: dsi wait cmddone timeout\n");
			DSI_DumpRegisters(module, 2);
			DSI_Reset(module, NULL);
		}

		DISPDBG("DSI read begin i = %d --------------------\n",
			  5 - max_try_count);
		DISPDBG("DSI_RX_STA     : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_TRIG_STA));
		DISPDBG("DSI_CMDQ_SIZE  : %d\n",
			  AS_UINT32(&DSI_REG[d]->DSI_CMDQ_SIZE));
		for (i = 0; i < DSI_REG[d]->DSI_CMDQ_SIZE.CMDQ_SIZE; i++) {
			DISPDBG("DSI_CMDQ_DATA%d : 0x%08x\n", i,
				  AS_UINT32(&DSI_CMDQ_REG[d]->data[i]));
		}
		DISPDBG("DSI_RX_DATA0   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA0));
		DISPDBG("DSI_RX_DATA1   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA1));
		DISPDBG("DSI_RX_DATA2   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA2));
		DISPDBG("DSI_RX_DATA3   : 0x%08x\n",
			  AS_UINT32(&DSI_REG[d]->DSI_RX_DATA3));
		DISPDBG("DSI read end ----------------------------\n");

		packet_type = read_data0.byte0;

		DISPCHECK("DSI read packet_type is 0x%x\n", packet_type);

		/*
		 * 0x02: acknowledge & error report
		 * 0x11: generic short read response(1 byte return)
		 * 0x12: generic short read response(2 byte return)
		 * 0x1a: generic long read response
		 * 0x1c: dcs long read response
		 * 0x21: dcs short read response(1 byte return)
		 * 0x22: dcs short read response(2 byte return)
		 */
		if (packet_type == 0x1A || packet_type == 0x1C) {
			recv_data_cnt =
				read_data0.byte1 + read_data0.byte2 * 16;
			if (recv_data_cnt > 10) {
				DISPCHECK(
					"DSI read long packet data exceeds 10 bytes,size:%d\n",
					recv_data_cnt);
				recv_data_cnt = 10;
			}

			if (recv_data_cnt > buffer_size) {
				DISPCHECK(
					"DSI read long packet data exceeds buffer size,size:%d\n",
					recv_data_cnt);
				recv_data_cnt = buffer_size;
			}
			DISPCHECK("DSI read long packet size: %d\n",
				recv_data_cnt);

			if (recv_data_cnt <= 4) {
				memcpy((void *)buffer,
					(void *)&read_data1, recv_data_cnt);
			} else if (recv_data_cnt <= 8) {
				memcpy((void *)buffer,
					(void *)&read_data1, 4);
				memcpy((void *)buffer + 4,
					(void *)&read_data2,
				    recv_data_cnt - 4);
			} else {
				memcpy((void *)buffer,
					(void *)&read_data1, 4);
				memcpy((void *)buffer + 4,
					(void *)&read_data2, 4);
				memcpy((void *)buffer + 8, (void *)&read_data2,
				    recv_data_cnt - 8);
			}
		} else if (packet_type == 0x11 || packet_type == 0x12 ||
				packet_type == 0x21 || packet_type == 0x22) {
			if (packet_type == 0x11 || packet_type == 0x21)
				recv_data_cnt = 1;
			else
				recv_data_cnt = 2;

			if (recv_data_cnt > buffer_size) {
				DISPCHECK(
					"DSI read short packet data exceeds buffer size:%d\n",
					buffer_size);
				recv_data_cnt = buffer_size;
				memcpy((void *)buffer,
					(void *)&read_data0.byte1,
					recv_data_cnt);
			} else {
				memcpy((void *)buffer,
					(void *)&read_data0.byte1,
					recv_data_cnt);
			}
		} else if (packet_type == 0x02) {
			DISPCHECK("read return type is 0x02, re-read\n");
		} else {
			DISPCHECK("read return type is non-recognite: 0x%x\n",
				  packet_type);
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[d]->DSI_INTEN, RD_RDY, 0);
			return 0;
		}
	} while (packet_type == 0x02);
	/* here: we may receive a ACK packet which packet type is 0x02
	 * (incdicates some error happened)
	 * therefore we try re-read again until no ACK packet
	 * But: if it is a good way to keep re-trying ???
	 */

	DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG, DSI_REG[d]->DSI_INTEN,
		RD_RDY, 0);
	return recv_data_cnt;
}

void DSI_set_cmdq_V2(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
	unsigned int cmd, unsigned char count, unsigned char *para_list,
	unsigned char force_update)
{
	UINT32 i = 0;
	int d = 0;
	unsigned long goto_addr, mask_para, set_para;
	struct DSI_T0_INS t0;
	struct DSI_T2_INS t2;

	memset(&t0, 0, sizeof(struct DSI_T0_INS));
	memset(&t2, 0, sizeof(struct DSI_T2_INS));
	if (module == DISP_MODULE_DSI0)
		d = 0;
	else if (module == DISP_MODULE_DSI1)
		d = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		d = 0;
	else
		return;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("DSI%d register addr invalid\n", i);
		return;
	}

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) { /* vdo cmd */
		struct DSI_VM_CMD_CON_REG vm_cmdq;
		struct DSI_VM_CMDQ *vm_data;

		memset(&vm_cmdq, 0, sizeof(struct DSI_VM_CMD_CON_REG));
		vm_data = DSI_VM_CMD_REG[d]->data;
		DSI_READREG32(struct DSI_VM_CMD_CON_REG *,
			&vm_cmdq, &DSI_REG[d]->DSI_VM_CMD_CON);
		if (cmd < 0xB0) {
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = DSI_DCS_LONG_PACKET_ID;
				vm_cmdq.CM_DATA_0 = count + 1;
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_VM_CMD_CON,
					AS_UINT32(&vm_cmdq));

				goto_addr =
					(unsigned long)(&vm_data[0].byte0);
				mask_para =
					(0xFF << ((goto_addr & 0x3) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & (~0x3), mask_para,
					set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&vm_data[0].byte1) + i;
					mask_para =
						(0xFF <<
						((goto_addr & 0x3) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3),
						mask_para, set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID =
						DSI_DCS_SHORT_PACKET_ID_1;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID =
						DSI_DCS_SHORT_PACKET_ID_0;
					vm_cmdq.CM_DATA_1 = 0;
				}
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));
			}
		} else {
			struct DSI_VM_CMDQ *vm_data;

			vm_data = DSI_VM_CMD_REG[d]->data;
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID =
					DSI_GERNERIC_LONG_PACKET_ID;
				vm_cmdq.CM_DATA_0 = count + 1;
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));

				goto_addr =
					(unsigned long)(&vm_data[0].byte0);
				mask_para =
					(0xFF << ((goto_addr & 0x3) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3) * 8));
				DSI_MASKREG32(cmdq, goto_addr & (~0x3),
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&vm_data[0].byte1) + i;
					mask_para =
						(0xFF <<
						((goto_addr & 0x3) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq,
						goto_addr & (~0x3),
						mask_para, set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_2;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_1;
					vm_cmdq.CM_DATA_1 = 0;
				}
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));
			}
		}
	} else { /* cmd mode */
		dsi_wait_not_busy(module, cmdq);
		if (cmd < 0xB0) {
			struct DSI_CMDQ *cmdq_reg;

			cmdq_reg = DSI_CMDQ_REG[d]->data;
			if (count > 1) {
				t2.CONFG = 2;
				t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
				t2.WC16 = count + 1;

				DSI_OUTREG32(cmdq, &cmdq_reg[0],
					AS_UINT32(&t2));

				goto_addr =
					(unsigned long)(
					&cmdq_reg[1].byte0);
				mask_para =
					(0xFFu <<
					((goto_addr & 0x3u) * 8));
				set_para =
					(cmd <<
					((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & (~0x3UL),
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&cmdq_reg[1].byte1) + i;
					mask_para =
						(0xFFu <<
						((goto_addr & 0x3u) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq,
						goto_addr & (~0x3UL),
						mask_para, set_para);
				}

				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					     2 + (count) / 4);
			} else {
				t0.CONFG = 0;
				t0.Data0 = cmd;
				if (count) {
					t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
					t0.Data1 = para_list[0];
				} else {
					t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
					t0.Data1 = 0;
				}

				DSI_OUTREG32(cmdq,
					&DSI_CMDQ_REG[d]->data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}
		} else {
			struct DSI_CMDQ *cmdq_reg;

			cmdq_reg = DSI_CMDQ_REG[d]->data;
			if (count > 1) {
				t2.CONFG = 2;
				t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
				t2.WC16 = count + 1;
				DSI_OUTREG32(cmdq, &cmdq_reg[0],
					AS_UINT32(&t2));
				goto_addr =
					(unsigned long)(&cmdq_reg[1].byte0);
				mask_para =
					(0xFFu << ((goto_addr & 0x3u) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & ~0x3UL,
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&cmdq_reg[1].byte1) + i;
					mask_para =
						(0xFFu <<
						((goto_addr & 0x3u) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq,
						      goto_addr & (~0x3UL),
						      mask_para, set_para);
				}

				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					     2 + (count) / 4);
			} else {
				t0.CONFG = 0;
				t0.Data0 = cmd;
				if (count) {
					t0.Data_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_2;
					t0.Data1 = para_list[0];
				} else {
					t0.Data_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_1;
					t0.Data1 = 0;
				}
				DSI_OUTREG32(cmdq,
					&DSI_CMDQ_REG[d]->data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}
		}
	}

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) { /* vdo mode */
		/* start DSI VM CMDQ */
		if (force_update)
			DSI_EnableVM_CMD(module, cmdq);
	} else { /* cmd mode */
		if (force_update) {
			DSI_Start(module, cmdq);
			dsi_wait_not_busy(module, cmdq);
		}
	}
}

void DSI_set_cmdq_V3(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
	struct LCM_setting_table_V3 *para_tbl, unsigned int size,
	unsigned char force_update)
{
	UINT32 i;
	/* UINT32 layer, layer_state, lane_num; */
	unsigned long goto_addr, mask_para, set_para;
	/* UINT32 fbPhysAddr, fbVirAddr; */
	struct DSI_T0_INS t0;
	/* DSI_T1_INS t1; */
	struct DSI_T2_INS t2;
	UINT32 index = 0;
	unsigned char data_id, cmd, count;
	unsigned char *para_list;
	UINT32 d;

	memset(&t0, 0, sizeof(struct DSI_T0_INS));
	memset(&t2, 0, sizeof(struct DSI_T2_INS));

	if (module == DISP_MODULE_DSI0)
		d = 0;
	else if (module == DISP_MODULE_DSI1)
		d = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		d = 0;
	else
		return;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("DSI%d register addr invalid\n", i);
		return;
	}

	do {
		data_id = para_tbl[index].id;
		cmd = para_tbl[index].cmd;
		count = para_tbl[index].count;
		para_list = para_tbl[index].para_list;

		if (data_id == REGFLAG_ESCAPE_ID &&
			cmd == REGFLAG_DELAY_MS_V3) {
			udelay(1000 * count);
			DDPMSG("DISP/DSI DSI_set_cmdq_V3[%d]. Delay %d (ms)\n",
			       index, count);

			continue;
		}
		if (DSI_REG[d]->DSI_MODE_CTRL.MODE) { /* vdo mode */
			struct DSI_VM_CMD_CON_REG vm_cmdq;
			struct DSI_VM_CMDQ *dsi_data;

			dsi_data = DSI_VM_CMD_REG[d]->data;
			OUTREG32(&vm_cmdq,
				AS_UINT32(&DSI_REG[d]->DSI_VM_CMD_CON));
			DDPMSG("set cmdq in VDO mode\n");
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = data_id;
				vm_cmdq.CM_DATA_0 = count + 1;
				OUTREG32(&DSI_REG[d]->DSI_VM_CMD_CON,
					AS_UINT32(&vm_cmdq));

				goto_addr = (unsigned long)(&dsi_data[0].byte0);
				mask_para = (0xFF << ((goto_addr & 0x3) * 8));
				set_para = (cmd << ((goto_addr & 0x3) * 8));
				MASKREG32(goto_addr & (~0x3), mask_para,
					set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&dsi_data[0].byte1)
						+ i;
					mask_para =
						(0xFF <<
						((goto_addr & 0x3)
						* 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3) * 8));
					MASKREG32(goto_addr & (~0x3), mask_para,
						set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID = data_id;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID = data_id;
					vm_cmdq.CM_DATA_1 = 0;
				}
				OUTREG32(&DSI_REG[d]->DSI_VM_CMD_CON,
					AS_UINT32(&vm_cmdq));
			}
			/* start DSI VM CMDQ */
			if (force_update)
				DSI_EnableVM_CMD(module, cmdq);
		} else { /* cmd mode */
			struct DSI_CMDQ *dsi_data;

			dsi_wait_not_busy(module, cmdq);
			dsi_data = DSI_CMDQ_REG[d]->data;
			OUTREG32(&dsi_data[0], 0);

			if (count > 1) {
				t2.CONFG = 2;
				t2.Data_ID = data_id;
				t2.WC16 = count + 1;

				DSI_OUTREG32(cmdq, &dsi_data[0].byte0,
					AS_UINT32(&t2));

				goto_addr = (unsigned long)(&dsi_data[1].byte0);
				mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
				set_para = (cmd << ((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq, goto_addr & (~0x3UL),
					      mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&dsi_data[1].byte1) + i;
					mask_para =
						(0xFFu <<
						((goto_addr & 0x3u) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq,
						goto_addr & (~0x3UL),
						mask_para, set_para);
				}

				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					     2 + (count) / 4);
			} else {
				t0.CONFG = 0;
				t0.Data0 = cmd;
				if (count) {
					t0.Data_ID = data_id;
					t0.Data1 = para_list[0];
				} else {
					t0.Data_ID = data_id;
					t0.Data1 = 0;
				}
				DSI_OUTREG32(cmdq, &dsi_data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}

			if (force_update) {
				DSI_Start(module, cmdq);
				dsi_wait_not_busy(module, cmdq);
			}
		}
	} while (++index < size);
}

void DSI_set_cmdq_V4(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq,
	struct dsi_cmd_desc *cmds)
{
	UINT32 i = 0;
	unsigned int d = 0;
	unsigned long goto_addr, mask_para, set_para;
	unsigned int cmd;
	unsigned char count;
	unsigned char *para_list;
	unsigned char virtual_channel;
	struct DSI_T0_INS t0;
	struct DSI_T2_INS t2;

	memset(&t0, 0, sizeof(struct DSI_T0_INS));
	memset(&t2, 0, sizeof(struct DSI_T2_INS));
	if (module == DISP_MODULE_DSI0)
		d = 0;
	else if (module == DISP_MODULE_DSI1)
		d = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		d = 0;
	else
		return;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("DSI%d register addr invalid\n", i);
		return;
	}

	cmd = cmds->dtype;
	count = (unsigned char)cmds->dlen;
	para_list = cmds->payload;
	virtual_channel = (unsigned char)cmds->vc;

	virtual_channel = ((virtual_channel << 6) | 0x3F);

	if (cmds->link_state == 0)
		/* Switch to HS mode*/
		DSI_clk_HS_mode(module, cmdq, TRUE);

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) { /* vdo cmd */
		struct DSI_VM_CMD_CON_REG vm_cmdq;
		struct DSI_VM_CMDQ *vm_data;

		memset(&vm_cmdq, 0, sizeof(struct DSI_VM_CMD_CON_REG));
		vm_data = DSI_VM_CMD_REG[d]->data;
		DSI_READREG32(struct DSI_VM_CMD_CON_REG *,
			&vm_cmdq, &DSI_REG[d]->DSI_VM_CMD_CON);
		if (cmd < 0xB0) {
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID = DSI_DCS_LONG_PACKET_ID;
				vm_cmdq.CM_DATA_ID =
					vm_cmdq.CM_DATA_ID &
					virtual_channel;
				vm_cmdq.CM_DATA_0 = count + 1;
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_VM_CMD_CON,
					AS_UINT32(&vm_cmdq));

				goto_addr =
					(unsigned long)(&vm_data[0].byte0);
				mask_para =
					(0xFF << ((goto_addr & 0x3) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & (~0x3), mask_para,
					set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&vm_data[0].byte1) + i;
					mask_para =
						(0xFF <<
						((goto_addr & 0x3) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq, goto_addr & (~0x3),
						mask_para, set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID =
						DSI_DCS_SHORT_PACKET_ID_1;
					vm_cmdq.CM_DATA_ID =
						vm_cmdq.CM_DATA_ID &
						virtual_channel;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID =
						DSI_DCS_SHORT_PACKET_ID_0;
					vm_cmdq.CM_DATA_ID =
						vm_cmdq.CM_DATA_ID &
						virtual_channel;
					vm_cmdq.CM_DATA_1 = 0;
				}
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));
			}
		} else {
			struct DSI_VM_CMDQ *vm_data;

			vm_data = DSI_VM_CMD_REG[d]->data;
			if (count > 1) {
				vm_cmdq.LONG_PKT = 1;
				vm_cmdq.CM_DATA_ID =
					DSI_GERNERIC_LONG_PACKET_ID;
				vm_cmdq.CM_DATA_ID =
					vm_cmdq.CM_DATA_ID & virtual_channel;
				vm_cmdq.CM_DATA_0 = count + 1;
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));

				goto_addr =
					(unsigned long)(&vm_data[0].byte0);
				mask_para =
					(0xFF << ((goto_addr & 0x3) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3) * 8));
				DSI_MASKREG32(cmdq, goto_addr & (~0x3),
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&vm_data[0].byte1) + i;
					mask_para =
						(0xFF <<
						((goto_addr & 0x3) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3) * 8));
					DSI_MASKREG32(cmdq,
						goto_addr & (~0x3),
						mask_para, set_para);
				}
			} else {
				vm_cmdq.LONG_PKT = 0;
				vm_cmdq.CM_DATA_0 = cmd;
				if (count) {
					vm_cmdq.CM_DATA_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_2;
					vm_cmdq.CM_DATA_ID =
						vm_cmdq.CM_DATA_ID &
						virtual_channel;
					vm_cmdq.CM_DATA_1 = para_list[0];
				} else {
					vm_cmdq.CM_DATA_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_1;
					vm_cmdq.CM_DATA_ID =
						vm_cmdq.CM_DATA_ID &
						virtual_channel;
					vm_cmdq.CM_DATA_1 = 0;
				}
				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_VM_CMD_CON,
					     AS_UINT32(&vm_cmdq));
			}
		}
	} else { /* cmd mode */
		dsi_wait_not_busy(module, cmdq);
		if (cmd < 0xB0) {
			struct DSI_CMDQ *cmdq_reg;

			cmdq_reg = DSI_CMDQ_REG[d]->data;
			if (count > 1) {
				t2.CONFG = 2;
				if (cmds->link_state == 0)
					/* HS Tx transmission */
					t2.CONFG = t2.CONFG | 0x08;
				t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
				t2.Data_ID = t2.Data_ID & virtual_channel;
				t2.WC16 = count + 1;

				DSI_OUTREG32(cmdq, &cmdq_reg[0],
					AS_UINT32(&t2));

				goto_addr =
					(unsigned long)(
					&cmdq_reg[1].byte0);
				mask_para =
					(0xFFu <<
					((goto_addr & 0x3u) * 8));
				set_para =
					(cmd <<
					((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & (~0x3UL),
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&cmdq_reg[1].byte1) + i;
					mask_para =
						(0xFFu <<
						((goto_addr & 0x3u) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq,
						goto_addr & (~0x3UL),
						mask_para, set_para);
				}

				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					     2 + (count) / 4);
			} else {
				t0.CONFG = 0;
				if (cmds->link_state == 0)
					/* HS Tx transmission */
					t0.CONFG = t0.CONFG | 0x08;
				t0.Data0 = cmd;
				if (count) {
					t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
					t0.Data_ID = t0.Data_ID &
						virtual_channel;
					t0.Data1 = para_list[0];
				} else {
					t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
					t0.Data_ID = t0.Data_ID &
						virtual_channel;
					t0.Data1 = 0;
				}

				DSI_OUTREG32(cmdq,
					&DSI_CMDQ_REG[d]->data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}
		} else {
			struct DSI_CMDQ *cmdq_reg;

			cmdq_reg = DSI_CMDQ_REG[d]->data;
			if (count > 1) {
				t2.CONFG = 2;
				if (cmds->link_state == 0)
					/* HS Tx transmission */
					t2.CONFG = t2.CONFG | 0x08;
				t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
				t2.Data_ID = t2.Data_ID & virtual_channel;
				t2.WC16 = count + 1;
				DSI_OUTREG32(cmdq, &cmdq_reg[0],
					AS_UINT32(&t2));
				goto_addr =
					(unsigned long)(&cmdq_reg[1].byte0);
				mask_para =
					(0xFFu << ((goto_addr & 0x3u) * 8));
				set_para =
					(cmd << ((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & ~0x3UL,
					mask_para, set_para);

				for (i = 0; i < count; i++) {
					goto_addr =
						(unsigned long)(
						&cmdq_reg[1].byte1) + i;
					mask_para =
						(0xFFu <<
						((goto_addr & 0x3u) * 8));
					set_para =
						(para_list[i] <<
						((goto_addr & 0x3u) * 8));
					DSI_MASKREG32(cmdq,
						      goto_addr & (~0x3UL),
						      mask_para, set_para);
				}

				DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					     2 + (count) / 4);
			} else {
				t0.CONFG = 0;
				if (cmds->link_state == 0)
					/* HS Tx transmission */
					t0.CONFG = t0.CONFG | 0x08;
				t0.Data0 = cmd;
				if (count) {
					t0.Data_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_2;
					t0.Data_ID =
						t0.Data_ID & virtual_channel;
					t0.Data1 = para_list[0];
				} else {
					t0.Data_ID =
						DSI_GERNERIC_SHORT_PACKET_ID_1;
					t0.Data_ID =
						t0.Data_ID & virtual_channel;
					t0.Data1 = 0;
				}
				DSI_OUTREG32(cmdq,
					&DSI_CMDQ_REG[d]->data[0],
					AS_UINT32(&t0));
				DSI_OUTREG32(cmdq,
					&DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}
		}
	}

	if (DSI_REG[d]->DSI_MODE_CTRL.MODE) { /* vdo mode */
		/* start DSI VM CMDQ */
		DSI_EnableVM_CMD(module, cmdq);
	} else { /* cmd mode */
		DSI_Start(module, cmdq);
		dsi_wait_not_busy(module, cmdq);
	}

	/* Revert to LP mode */
	if (cmds->link_state == 0)
		/* Switch to HS mode*/
		DSI_clk_HS_mode(module, cmdq, FALSE);

}

void DSI_set_cmdq(enum DISP_MODULE_ENUM module, struct cmdqRecStruct *cmdq,
	unsigned int *pdata, unsigned int queue_size,
	unsigned char force_update)
{
	int j = 0;
	int i = 0, module_num;


	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		if (DSI_REG[i]->DSI_MODE_CTRL.MODE) {
			;/* vdo mode */
		} else { /* cmd mode */
			ASSERT(queue_size <= 32);
			dsi_wait_not_busy(module, cmdq);

			for (j = 0; j < queue_size; j++) {
				DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[i]->data[j],
					     AS_UINT32((pdata + j)));
			}

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_CMDQ_SIZE,
				queue_size);
		}
	}

	if (DSI_REG[0]->DSI_MODE_CTRL.MODE) {
		;/* vdo mode */
	} else { /* cmd mode */
		if (force_update) {
			DSI_Start(module, cmdq);
			dsi_wait_not_busy(module, cmdq);
		}
	}
}

int DSI_Send_ROI(enum DISP_MODULE_ENUM module, void *handle, unsigned int x,
	unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];
	unsigned int module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	if (!primary_display_is_video_mode()) {
		data_array[0] = 0x00053902;
		data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) |
			(x0_MSB << 8) | 0x2a;
		data_array[2] = (x1_LSB);
		DSI_set_cmdq(module, handle, data_array, 3, 1);
		data_array[0] = 0x00053902;
		data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) |
			(y0_MSB << 8) | 0x2b;
		data_array[2] = (y1_LSB);
		DSI_set_cmdq(module, handle, data_array, 3, 1);
		DDPMSG("%s(%d,%d,%dx%d)Done!\n",
			__func__, x, y, width, height);
	} else
		DDPDBG("LCM is video mode, no need DSI send ROI!\n");

	return 0;
}

static void lcm_set_reset_pin(UINT32 value)
{
	if (!_is_lcm_cmd_mode(DISP_MODULE_DSI0)) {
		DSI_OUTREG32(NULL, DISP_REG_CONFIG_MMSYS_LCM_RST_B, value);
	} else {
		if (value)
			disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
		else
			disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);
	}
}

static void lcm1_set_reset_pin(UINT32 value)
{
	if (value)
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM1_RST_OUT1);
	else
		disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM1_RST_OUT0);
}

static void lcm1_set_te_pin(void)
{
	disp_dts_gpio_select_state(DTS_GPIO_STATE_TE1_MODE_TE);
}

static void lcm_udelay(UINT32 us)
{
	udelay(us);
}

static void lcm_mdelay(UINT32 ms)
{
	if (ms < 10)
		udelay(ms * 1000);
	else if (ms <= 20)
		usleep_range(ms*1000, (ms+1)*1000);
	else
		usleep_range(ms * 1000 - 100, ms * 1000);
}

void DSI_set_cmdq_V11_wrapper_DSI0(void *cmdq, unsigned int *pdata,
	unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI0, cmdq, pdata, queue_size, force_update);
}

void DSI_set_cmdq_V11_wrapper_DSI1(void *cmdq, unsigned int *pdata,
	unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI1, cmdq, pdata, queue_size, force_update);
}

void DSI_set_cmdq_V2_DSI0(void *cmdq, unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, cmdq, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V2_DSI1(void *cmdq, unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, cmdq, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V2_DSIDual(void *cmdq, unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, cmdq, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V4_DSI0(void *cmdq, struct dsi_cmd_desc *cmds)
{
	DSI_set_cmdq_V4(DISP_MODULE_DSI0, cmdq, cmds);
}

void DSI_set_cmdq_V4_DSI1(void *cmdq, struct dsi_cmd_desc *cmds)
{
	DSI_set_cmdq_V4(DISP_MODULE_DSI1, cmdq, cmds);
}

void DSI_set_cmdq_V4_DSIDual(void *cmdq, struct dsi_cmd_desc *cmds)
{
	DSI_set_cmdq_V4(DISP_MODULE_DSIDUAL, cmdq, cmds);
}

void DSI_set_cmdq_V2_Wrapper_DSI0(unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI0, NULL, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSI1(unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSI1, NULL, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V2_Wrapper_DSIDual(unsigned int cmd, unsigned char count,
	unsigned char *para_list, unsigned char force_update)
{
	DSI_set_cmdq_V2(DISP_MODULE_DSIDUAL, NULL, cmd, count, para_list,
		force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI0(struct LCM_setting_table_V3 *para_tbl,
	unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI0, NULL, para_tbl, size,
		force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSI1(struct LCM_setting_table_V3 *para_tbl,
	unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSI1, NULL, para_tbl, size,
		force_update);
}

void DSI_set_cmdq_V3_Wrapper_DSIDual(struct LCM_setting_table_V3 *para_tbl,
	unsigned int size, unsigned char force_update)
{
	DSI_set_cmdq_V3(DISP_MODULE_DSIDUAL, NULL, para_tbl, size,
		force_update);
}

void DSI_set_cmdq_wrapper_DSI0(unsigned int *pdata,
	unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI0, NULL, pdata, queue_size,
		force_update);
}

void DSI_set_cmdq_wrapper_DSI1(unsigned int *pdata,
	unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSI1, NULL, pdata, queue_size,
		force_update);
}

void DSI_set_cmdq_wrapper_DSIDual(unsigned int *pdata,
	unsigned int queue_size, unsigned char force_update)
{
	DSI_set_cmdq(DISP_MODULE_DSIDUAL, NULL, pdata, queue_size,
		force_update);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI0(UINT8 cmd, UINT8 *buffer,
	UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL, cmd,
		buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSI1(UINT8 cmd, UINT8 *buffer,
	UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI1, NULL, cmd,
		buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v2_wrapper_DSIDUAL(UINT8 cmd, UINT8 *buffer,
	UINT8 buffer_size)
{
	return DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSIDUAL, NULL, cmd,
		buffer, buffer_size);
}

unsigned int DSI_dcs_read_lcm_reg_v3_wrapper_DSI0(char *out,
		struct dsi_cmd_desc *cmds, unsigned int len)
{
	return DSI_dcs_read_lcm_reg_v3(DISP_MODULE_DSI0, NULL,
			out, cmds, len);
}

unsigned int DSI_dcs_read_lcm_reg_v3_wrapper_DSI1(char *out,
		struct dsi_cmd_desc *cmds, unsigned int len)
{
	return DSI_dcs_read_lcm_reg_v3(DISP_MODULE_DSI1, NULL,
			out, cmds, len);
}

unsigned int DSI_dcs_read_lcm_reg_v3_wrapper_DSIDUAL(char *out,
		struct dsi_cmd_desc *cmds, unsigned int len)
{
	return DSI_dcs_read_lcm_reg_v3(DISP_MODULE_DSIDUAL, NULL,
			out, cmds, len);
}

/* remove later */
long lcd_enp_bias_setting(unsigned int value)
{
	long ret = 0;

	return ret;
}

long lcm_vddi_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_VDDI_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_VDDI_OUT0);

	return ret;
}


long lcm_reset_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_RST_OUT0);

	return ret;
}
long lcm_enp_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_ENP_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_ENP_OUT0);

	return ret;
}
long lcm_enn_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_ENN_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_ENN_OUT0);

	return ret;
}

long lcm_bkg_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BKG_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCD_BKG_OUT0);

	return ret;
}

long tp_reset_setting(unsigned int value)
{
	long ret = 0;

	if (value)
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_TP_OUT1);
	else
		ret = disp_dts_gpio_select_state(DTS_GPIO_STATE_LCM_TP_OUT0);

	return ret;
}
int ddp_dsi_set_lcm_utils(enum DISP_MODULE_ENUM module,
	struct LCM_DRIVER *lcm_drv)
{
	struct LCM_UTIL_FUNCS *utils = NULL;

	if (lcm_drv == NULL) {
		DISPERR("lcm_drv is null\n");
		return -1;
	}

	if (module == DISP_MODULE_DSI0) {
		utils = (struct LCM_UTIL_FUNCS *)&lcm_utils_dsi0;
	} else if (module == DISP_MODULE_DSI1) {
		utils = (struct LCM_UTIL_FUNCS *)&lcm_utils_dsi1;
	} else if (module == DISP_MODULE_DSIDUAL) {
		utils = (struct LCM_UTIL_FUNCS *)&lcm_utils_dsidual;
	} else {
		DISPWARN("wrong module: %d\n", module);
		return -1;
	}

	utils->set_reset_pin = lcm_set_reset_pin;
	utils->udelay = lcm_udelay;
	utils->mdelay = lcm_mdelay;
	utils->set_te_pin = NULL;
	if (module == DISP_MODULE_DSI0) {
		utils->dsi_set_cmdq =
			DSI_set_cmdq_wrapper_DSI0;
		utils->dsi_set_cmdq_V2 =
			DSI_set_cmdq_V2_Wrapper_DSI0;
		utils->dsi_set_cmdq_V3 =
			DSI_set_cmdq_V3_Wrapper_DSI0;
		utils->dsi_dcs_read_lcm_reg_v2 =
			DSI_dcs_read_lcm_reg_v2_wrapper_DSI0;
		utils->dsi_set_cmdq_V22 =
			DSI_set_cmdq_V2_DSI0;
		utils->dsi_set_cmdq_V11 =
			DSI_set_cmdq_V11_wrapper_DSI0;
		utils->dsi_set_cmdq_V23 =
			DSI_set_cmdq_V2_DSI0;
		utils->mipi_dsi_cmds_tx =
			DSI_set_cmdq_V4_DSI0;
		utils->mipi_dsi_cmds_rx =
			DSI_dcs_read_lcm_reg_v3_wrapper_DSI0;
	} else if (module == DISP_MODULE_DSI1) {
		utils->set_reset_pin =
			lcm1_set_reset_pin;
		utils->set_te_pin =
			lcm1_set_te_pin;
		utils->dsi_set_cmdq =
			DSI_set_cmdq_wrapper_DSI1;
		utils->dsi_set_cmdq_V2 =
			DSI_set_cmdq_V2_Wrapper_DSI1;
		utils->dsi_set_cmdq_V3 =
			DSI_set_cmdq_V3_Wrapper_DSI1;
		utils->dsi_dcs_read_lcm_reg_v2 =
			DSI_dcs_read_lcm_reg_v2_wrapper_DSI1;
		utils->dsi_set_cmdq_V22 =
			DSI_set_cmdq_V2_DSI1;
		utils->dsi_set_cmdq_V11 =
			DSI_set_cmdq_V11_wrapper_DSI1;
		utils->dsi_set_cmdq_V23 =
			DSI_set_cmdq_V2_DSI1;
		utils->mipi_dsi_cmds_tx =
			DSI_set_cmdq_V4_DSI1;
		utils->mipi_dsi_cmds_rx =
			DSI_dcs_read_lcm_reg_v3_wrapper_DSI1;
	} else if (module == DISP_MODULE_DSIDUAL) {
		struct LCM_PARAMS lcm_param;

		lcm_drv->get_params(&lcm_param);

		if (lcm_param.lcm_cmd_if == LCM_INTERFACE_DSI0) {
			utils->dsi_set_cmdq =
				DSI_set_cmdq_wrapper_DSI0;
			utils->dsi_set_cmdq_V2 =
				DSI_set_cmdq_V2_Wrapper_DSI0;
			utils->dsi_set_cmdq_V3 =
				DSI_set_cmdq_V3_Wrapper_DSI0;
			utils->dsi_dcs_read_lcm_reg_v2 =
				DSI_dcs_read_lcm_reg_v2_wrapper_DSI0;
			utils->dsi_set_cmdq_V22 =
				DSI_set_cmdq_V2_DSI0;
			utils->dsi_set_cmdq_V23 =
				DSI_set_cmdq_V2_DSI0;
			utils->mipi_dsi_cmds_tx =
				DSI_set_cmdq_V4_DSI0;
			utils->mipi_dsi_cmds_rx =
				DSI_dcs_read_lcm_reg_v3_wrapper_DSI0;
		} else if (lcm_param.lcm_cmd_if == LCM_INTERFACE_DSI1) {
			utils->dsi_set_cmdq =
				DSI_set_cmdq_wrapper_DSI1;
			utils->dsi_set_cmdq_V2 =
				DSI_set_cmdq_V2_Wrapper_DSI1;
			utils->dsi_set_cmdq_V3 =
				DSI_set_cmdq_V3_Wrapper_DSI1;
			utils->dsi_dcs_read_lcm_reg_v2 =
				DSI_dcs_read_lcm_reg_v2_wrapper_DSI1;
			utils->dsi_set_cmdq_V22 =
				DSI_set_cmdq_V2_DSI1;
			utils->dsi_set_cmdq_V23 =
				DSI_set_cmdq_V2_DSI1;
			utils->mipi_dsi_cmds_tx =
				DSI_set_cmdq_V4_DSI1;
			utils->mipi_dsi_cmds_rx =
				DSI_dcs_read_lcm_reg_v3_wrapper_DSI1;
		} else {
			utils->dsi_set_cmdq =
				DSI_set_cmdq_wrapper_DSIDual;
			utils->dsi_set_cmdq_V2 =
				DSI_set_cmdq_V2_Wrapper_DSIDual;
			utils->dsi_dcs_read_lcm_reg_v2 =
				DSI_dcs_read_lcm_reg_v2_wrapper_DSIDUAL;
			utils->dsi_set_cmdq_V23 =
				DSI_set_cmdq_V2_DSIDual;
			utils->mipi_dsi_cmds_tx =
				DSI_set_cmdq_V4_DSIDual;
			utils->mipi_dsi_cmds_rx =
				DSI_dcs_read_lcm_reg_v3_wrapper_DSIDUAL;
		}
	}

#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef CONFIG_MTK_LEGACY
	utils->set_gpio_out = mt_set_gpio_out;
	utils->set_gpio_mode = mt_set_gpio_mode;
	utils->set_gpio_dir = mt_set_gpio_dir;
	utils->set_gpio_pull_enable =
		(int (*)(unsigned int, unsigned char))mt_set_gpio_pull_enable;
#else
	utils->set_gpio_lcd_enp_bias = lcd_enp_bias_setting;
#endif
#endif

	utils->lcm_reset_setting = lcm_reset_setting;
	utils->lcm_vddi_setting = lcm_vddi_setting;
	utils->lcm_enp_setting = lcm_enp_setting;
	utils->lcm_enn_setting = lcm_enn_setting;
	utils->lcm_bkg_setting = lcm_bkg_setting;
	utils->tp_reset_setting = tp_reset_setting;
	lcm_drv->set_util_funcs(utils);

	return 0;
}

#if 0
void DSI_ChangeClk(DISP_MODULE_ENUM module, UINT32 clk)
{
	int i = 0;

	if (clk > 1250 || clk < 50)
		return;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		LCM_DSI_PARAMS *dsi_params = &_dsi_context[i].dsi_params;

		dsi_params->PLL_CLOCK = clk;
		DSI_WaitForNotBusy(module, NULL);
		DSI_PHY_clk_setting(module, NULL, dsi_params);
		DSI_PHY_TIMCONFIG(module, NULL, dsi_params);
	}
}
#endif

static void _set_power_on_status(enum DISP_MODULE_ENUM module,
	unsigned int ispoweon)
{
	if (module == DISP_MODULE_DSIDUAL) {
		_dsi_context[0].is_power_on = ispoweon;
		_dsi_context[1].is_power_on = ispoweon;
	} else if (module == DISP_MODULE_DSI0) {
		_dsi_context[0].is_power_on = ispoweon;
	} else if (module == DISP_MODULE_DSI1) {
		_dsi_context[1].is_power_on = ispoweon;
	}
}

int ddp_dsi_init(enum DISP_MODULE_ENUM module, void *cmdq)
{
	int i = 0, j = 0;
	unsigned long addr = 0;
	unsigned int module_num;

	DISPFUNC();

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return DSI_STATUS_ERROR;
	}

	cmdqBackupAllocateSlot(&_h_intstat, 1);

	DSI_REG[0] = (struct DSI_REGS *)DISPSYS_DSI0_BASE;
	DSI_PHY_REG[0] = DISPSYS_MIPITX0_BASE;
	DSI_CMDQ_REG[0] =
		(struct DSI_CMDQ_REGS *)(DISPSYS_DSI0_BASE + 0x200);

	DSI_REG[1] = (struct DSI_REGS *)DISPSYS_DSI1_BASE;
	DSI_PHY_REG[1] = DISPSYS_MIPITX1_BASE;
	DSI_CMDQ_REG[1] =
		(struct DSI_CMDQ_REGS *)(DISPSYS_DSI1_BASE + 0x200);

	DSI_VM_CMD_REG[0] =
		(struct DSI_VM_CMDQ_REGS *)(DISPSYS_DSI0_BASE + 0x134);
	DSI_VM_CMD_REG[1] =
		(struct DSI_VM_CMDQ_REGS *)(DISPSYS_DSI1_BASE + 0x134);
	memset(&_dsi_context, 0, sizeof(_dsi_context));

	for (i = 0; i < module_num; i++) {
		DISPCHECK("dsi%d initializing _dsi_context\n", i);
		mutex_init(&(_dsi_context[i].lock));
		_init_condition_wq(&(_dsi_context[i].cmddone_wq));
		_init_condition_wq(&(_dsi_context[i].read_wq));
		_init_condition_wq(&(_dsi_context[i].bta_te_wq));
		_init_condition_wq(&(_dsi_context[i].ext_te_wq));
		_init_condition_wq(&(_dsi_context[i].vm_done_wq));
		_init_condition_wq(&(_dsi_context[i].vm_cmd_done_wq));
		_init_condition_wq(&(_dsi_context[i].sleep_out_done_wq));
		_init_condition_wq(&(_dsi_context[i].sleep_in_done_wq));
	}

	if (module == DISP_MODULE_DSIDUAL) {
		disp_register_module_irq_callback(DISP_MODULE_DSI0,
			_DSI_INTERNAL_IRQ_Handler);
		disp_register_module_irq_callback(DISP_MODULE_DSI1,
			_DSI_INTERNAL_IRQ_Handler);
	} else {
		disp_register_module_irq_callback(module,
			_DSI_INTERNAL_IRQ_Handler);
	}

	if (MIPITX_IsEnabled(module, cmdq)) {
		_set_power_on_status(module, 1);
		/* enable cg(for ccf) */
		ddp_set_mipi26m(module, 1);

		if (module == DISP_MODULE_DSI0 ||
			module == DISP_MODULE_DSIDUAL) {
			ddp_clk_prepare_enable(CLK_DSI0_MM_CLK);
			ddp_clk_prepare_enable(CLK_DSI0_IF_CLK);
		}

		/* __close_dsi_default_clock(module); */
	}

#if defined(CONFIG_MTK_DUAL_DISPLAY_SUPPORT) && \
	(CONFIG_MTK_DUAL_DISPLAY_SUPPORT == 2)
	if (module == DISP_MODULE_DSI1) {
		/*set DSI1 TE source*/
		DSI_MASKREG32(NULL, DISP_REG_CONFIG_MMSYS_MISC, 0x2, 0x2);
		DISPCHECK("set DISP_REG_CFG_MMSYS_MISC DSI1_TE, value:0x%08x\n",
			INREG32(DISP_REG_CONFIG_MMSYS_MISC));
		/*set GPIO DSI1_TE mode*/
		lcm1_set_te_pin();
	}
#endif

	/* backup mipitx impedance0 which is inited in LK*/
	addr = DSI_PHY_REG[0]+0x100;
	for (i = 0; i < 5; i++) {
		for (j = 0; j < 10; j++) {
			mipitx_impedance_backup[i] |= ((INREG32(addr))<<j);
			addr += 0x4;
		}
		/* 0xD8 = 0x300 - 0x228*/
		addr += 0xD8;
	}

	return DSI_STATUS_OK;
}

int ddp_dsi_deinit(enum DISP_MODULE_ENUM module, void *cmdq_handle)
{
	return DSI_STATUS_OK;
}

static void DSI_PHY_CLK_LP_PerLine_config(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, struct LCM_DSI_PARAMS *dsi_params)
{
	int i, module_num;
	/* LPX */
	struct DSI_PHY_TIMCON0_REG timcon0;
	/* CLK_HS_TRAIL, CLK_HS_ZERO */
	struct DSI_PHY_TIMCON2_REG timcon2;
	/* CLK_HS_EXIT, CLK_HS_POST, CLK_HS_PREP */
	struct DSI_PHY_TIMCON3_REG timcon3;
	struct DSI_HSA_WC_REG hsa;
	struct DSI_HBP_WC_REG hbp;
	struct DSI_HFP_WC_REG hfp, new_hfp;
	struct DSI_BLLP_WC_REG bllp;
	struct DSI_PSCTRL_REG ps;
	UINT32 hstx_ckl_wc, new_hstx_ckl_wc;
	UINT32 v_a, v_b, v_c, lane_num;
	enum LCM_DSI_MODE_CON dsi_mode;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	for (i = 0; i < module_num; i++) {
		lane_num = dsi_params->LANE_NUM;
		dsi_mode = dsi_params->mode;

		if (dsi_mode == CMD_MODE)
			continue;

		/* vdo mode */
		DSI_READREG32(struct DSI_HSA_WC_REG*, &hsa,
			&DSI_REG[i]->DSI_HSA_WC);
		DSI_READREG32(struct DSI_HBP_WC_REG*, &hbp,
			&DSI_REG[i]->DSI_HBP_WC);
		DSI_READREG32(struct DSI_HFP_WC_REG*, &hfp,
			&DSI_REG[i]->DSI_HFP_WC);
		DSI_READREG32(struct DSI_BLLP_WC_REG*, &bllp,
			&DSI_REG[i]->DSI_BLLP_WC);
		DSI_READREG32(struct DSI_PSCTRL_REG*, &ps,
			&DSI_REG[i]->DSI_PSCTRL);
		DSI_READREG32(UINT32*, &hstx_ckl_wc,
			&DSI_REG[i]->DSI_HSTX_CKL_WC);
		DSI_READREG32(struct DSI_PHY_TIMCON0_REG*, &timcon0,
			&DSI_REG[i]->DSI_PHY_TIMECON0);
		DSI_READREG32(struct DSI_PHY_TIMCON2_REG*, &timcon2,
			&DSI_REG[i]->DSI_PHY_TIMECON2);
		DSI_READREG32(struct DSI_PHY_TIMCON3_REG*, &timcon3,
			&DSI_REG[i]->DSI_PHY_TIMECON3);

		if (dsi_mode == SYNC_PULSE_VDO_MODE) {
			/*
			 * 1. sync_pulse_mode
			 * Total    WC(A) = HSA_WC + HBP_WC + HFP_WC +
			 *                  PS_WC + 32
			 * CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP +
			 *                   CLK_HS_ZERO) * lane_num
			 * CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL) *
			 *                   lane_num
			 * HSTX_CKLP_WC = A - B
			 * Limitation: B + C < HFP_WC
			 */
			v_a = hsa.HSA_WC + hbp.HBP_WC + hfp.HFP_WC +
				ps.DSI_PS_WC + 32;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX +
				timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO) *
				lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) *
				lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n",
				(v_a - v_b), hstx_ckl_wc);
			DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",
				(v_b+v_c), hfp.HFP_WC);
			DISPCHECK(
				"===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC,
				(v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_READREG32(struct DSI_HFP_WC_REG*, &new_hfp,
				&DSI_REG[i]->DSI_HFP_WC);
			v_a = hsa.HSA_WC + hbp.HBP_WC + new_hfp.HFP_WC
				+ ps.DSI_PS_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC,
				(v_a - v_b));
			DSI_READREG32(UINT32*, &new_hstx_ckl_wc,
				&DSI_REG[i]->DSI_HSTX_CKL_WC);
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n",
				new_hstx_ckl_wc, new_hfp.HFP_WC);
		} else if (dsi_mode == SYNC_EVENT_VDO_MODE) {
			/*
			 * 2. sync_event_mode
			 * Total    WC(A) = HBP_WC + HFP_WC + PS_WC + 26
			 * CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP +
			 *                  CLK_HS_ZERO) * lane_num
			 * CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL) *
			 *                  lane_num
			 * HSTX_CKLP_WC = A - B
			 * Limitation: B + C < HFP_WC
			 */
			v_a = hbp.HBP_WC + hfp.HFP_WC + ps.DSI_PS_WC + 26;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX +
				timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO) *
				lane_num;
			v_c = (timcon3.CLK_HS_POST + timcon2.CLK_TRAIL) *
				lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n",
				(v_a - v_b), hstx_ckl_wc);
			DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",
				(v_b+v_c), hfp.HFP_WC);
			DISPCHECK(
				"===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC,
				(v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_READREG32(struct DSI_HFP_WC_REG*, &new_hfp,
				&DSI_REG[i]->DSI_HFP_WC);
			v_a = hbp.HBP_WC + new_hfp.HFP_WC + ps.DSI_PS_WC + 26;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC,
				(v_a - v_b));
			DSI_READREG32(UINT32*, &new_hstx_ckl_wc,
				&DSI_REG[i]->DSI_HSTX_CKL_WC);
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n",
				  new_hstx_ckl_wc, new_hfp.HFP_WC);
		} else if (dsi_mode == BURST_VDO_MODE) {
			/*
			 * 3. burst_mode
			 * Total    WC(A) = HBP_WC + HFP_WC + PS_WC +
			 *                  BLLP_WC + 32
			 * CLK init WC(B) = (CLK_HS_EXIT + LPX + CLK_HS_PREP +
			 *                   CLK_HS_ZERO) * lane_num
			 * CLK end  WC(C) = (CLK_HS_POST + CLK_HS_TRAIL) *
			 *                   lane_num
			 * HSTX_CKLP_WC = A - B
			 * Limitation: B + C < HFP_WC
			 */
			v_a = hbp.HBP_WC + hfp.HFP_WC +
				ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			v_b = (timcon3.CLK_HS_EXIT + timcon0.LPX +
				timcon3.CLK_HS_PRPR + timcon2.CLK_ZERO)
				* lane_num;
			v_c = (timcon3.CLK_HS_POST +
				timcon2.CLK_TRAIL) * lane_num;

			DISPCHECK("===>v_a-v_b=0x%x,HSTX_CKLP_WC=0x%x\n",
				(v_a - v_b), hstx_ckl_wc);
			DISPCHECK("===>v_b+v_c=0x%x,HFP_WC=0x%x\n",
				(v_b+v_c), hfp.HFP_WC);
			DISPCHECK(
				"===>Will Reconfig in order to fulfill LP clock lane per line\n");

			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HFP_WC,
				(v_b + v_c + DIFF_CLK_LANE_LP));
			DSI_READREG32(struct DSI_HFP_WC_REG*, &new_hfp,
				&DSI_REG[i]->DSI_HFP_WC);
			v_a = hbp.HBP_WC + new_hfp.HFP_WC +
				ps.DSI_PS_WC + bllp.BLLP_WC + 32;
			DSI_OUTREG32(cmdq, &DSI_REG[i]->DSI_HSTX_CKL_WC,
				(v_a - v_b));
			DSI_READREG32(UINT32*, &new_hstx_ckl_wc,
				&DSI_REG[i]->DSI_HSTX_CKL_WC);
			DISPCHECK("===>new HSTX_CKL_WC=0x%x, HFP_WC=0x%x\n",
				  new_hstx_ckl_wc, new_hfp.HFP_WC);
		}
	}
}

void ddp_dsi_update_partial(enum DISP_MODULE_ENUM module, void *cmdq,
	void *params)
{
	struct disp_rect *roi = (struct disp_rect *)params;
	unsigned int x = roi->x, y = roi->y;

	DSI_PS_Control(module, cmdq, &(_dsi_context[0].dsi_params),
			roi->width, roi->height);

#ifdef CONFIG_MTK_LCM_PHYSICAL_ROTATION_HW
	x = _dsi_context[0].lcm_width - (x + roi->width);
	y = _dsi_context[0].lcm_height - (y + roi->height);
#endif

	DSI_Send_ROI(DISP_MODULE_DSI0, cmdq, x, y,
		roi->width, roi->height);
}

/**
 * _dsi_basic_irq_enable
 *
 * 1. CMD_DONE
 * 2. TE_READY
 * 3. VM_DONE
 * 4. VM_CMD_DONE
 * 5. RD_RDY(enable when read)
 * 6. SLEEPOUT(enable when poweron)
 * 7. SLEEPIN(enable when poweroff)
 * 8. FRAME_DONE(not enable)
 * 9. INP_UNFINISH_IRQ
 */
static void _dsi_basic_irq_enable(enum DISP_MODULE_ENUM module, void *cmdq)
{
	int i = 0;

	if (module == DISP_MODULE_DSIDUAL) {
		/* cmd done */
		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[0]->DSI_INTEN, CMD_DONE, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[1]->DSI_INTEN, CMD_DONE, 1);

		/* vdo mode & disable eint => enable dsi te */
		if (_dsi_context[0].dsi_params.mode != CMD_MODE &&
		    _dsi_context[0].dsi_params.eint_disable == 1) {
			DISPCHECK("Dual DSI Vdo Mode Enable TE\n");
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[0]->DSI_INTEN, TE_RDY, 1);
		}

		/* cmd mode enable dsi te */
		if (_dsi_context[0].dsi_params.mode == CMD_MODE)
			DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
				DSI_REG[0]->DSI_INTEN, TE_RDY, 1);

		if (_dsi_context[0].dsi_params.mode != CMD_MODE ||
		    ((_dsi_context[0].dsi_params.switch_mode_enable == 1) &&
		     (_dsi_context[0].dsi_params.switch_mode != CMD_MODE))) {
			DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
				DSI_REG[0]->DSI_INTEN, VM_DONE, 1);
			DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
				DSI_REG[1]->DSI_INTEN, VM_DONE, 1);
			DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
				DSI_REG[0]->DSI_INTEN, VM_CMD_DONE, 0);
		}

		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[0]->DSI_INTEN, INP_UNFINISH_INT_EN, 1);
		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[1]->DSI_INTEN, INP_UNFINISH_INT_EN, 1);

		return;
	} else if (module == DISP_MODULE_DSI0) {
		i = 0;
	} else if (module == DISP_MODULE_DSI1) {
		i = 1;
	} else {
		return;
	}

	DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG, DSI_REG[i]->DSI_INTEN,
		CMD_DONE, 1);

	/* vdo mode & disable eint => enable dsi te */
	if (_dsi_context[i].dsi_params.mode != CMD_MODE &&
	    _dsi_context[i].dsi_params.eint_disable == 1) {
		DISPCHECK("Dual DSI Vdo Mode Enable TE\n");
		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
	}

	/* cmd mode enable dsi te */
	if (_dsi_context[i].dsi_params.mode == CMD_MODE)
		DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, TE_RDY, 1);

	if (_dsi_context[i].dsi_params.mode != CMD_MODE ||
	    ((_dsi_context[i].dsi_params.switch_mode_enable == 1) &&
	     (_dsi_context[i].dsi_params.switch_mode != CMD_MODE))) {
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, VM_DONE, 1);
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, VM_CMD_DONE, 0);
	}

	DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
		DSI_REG[i]->DSI_INTEN, INP_UNFINISH_INT_EN, 1);
	DSI_OUTREGBIT(cmdq, struct DSI_INT_ENABLE_REG,
		DSI_REG[i]->DSI_INTEN, BUFFER_UNDERRUN_INT_EN, 1);
}

/**
 * config dsi driver:
 * 1._dsi_context get info
 * 2.power init(dsi analogy)
 * 3.dsi irq setting
 */
int ddp_dsi_config(enum DISP_MODULE_ENUM module,
	struct disp_ddp_path_config *config, void *cmdq)
{
	int i = 0, module_num;
	struct LCM_DSI_PARAMS *dsi_config;

	DISPFUNC();

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	if (!config->dst_dirty) {
		/* will be removed */
		if (atomic_read(&PMaster_enable) == 0)
			return 0;
	}

	dsi_config = &(config->dispif_config.dsi);

	for (i = 0; i < module_num; i++) {
		memcpy(&(_dsi_context[i].dsi_params), dsi_config,
			sizeof(struct LCM_DSI_PARAMS));
		_dsi_context[i].lcm_width = config->dst_w;
		_dsi_context[i].lcm_height = config->dst_h;
		_dump_dsi_params(&(_dsi_context[i].dsi_params));
	}

	if (!MIPITX_IsEnabled(module, cmdq) || PanelMaster_is_enable()) {
		DISPCHECK("MIPITX is not inited, will config mipitx clk=%d\n",
			_dsi_context[0].dsi_params.PLL_CLOCK);

		if (PanelMaster_is_enable())
			DISPDBG("===>Pmaster: set clk:%d\n",
				_dsi_context[0].dsi_params.PLL_CLOCK);
		DSI_PHY_clk_switch(module, NULL, false);
		DSI_PHY_clk_switch(module, NULL, true);
	}

	/* c2v */
	if (dsi_config->mode != CMD_MODE)
		dsi_currect_mode = 1;

	_dsi_basic_irq_enable(module, cmdq);
	/* LCM params */
	DSI_TXRX_Control(module, cmdq, dsi_config);
	/* W & H */
	DSI_PS_Control(module, cmdq, dsi_config, config->dst_w, config->dst_h);
	/* PLL */
	DSI_PHY_TIMCONFIG(module, cmdq, dsi_config);

	/* vdo mode params */
	if (dsi_config->mode != CMD_MODE ||
		((dsi_config->switch_mode_enable == 1) &&
		(dsi_config->switch_mode != CMD_MODE))) {
		DSI_Calc_VDO_Timing(module, dsi_config);
		DSI_Config_VDO_Timing(module, cmdq, dsi_config);
		DSI_Set_VM_CMD(module, cmdq);
	}
	/* Enable clk low power per Line ; */
	if (dsi_config->clk_lp_per_line_enable)
		DSI_PHY_CLK_LP_PerLine_config(module, cmdq, dsi_config);

	return 0;
}

int dsi_basic_irq_enable(enum DISP_MODULE_ENUM module, void *cmdq)
{
	_dsi_basic_irq_enable(module, cmdq);

	return 0;
}

/* TUI will use the api */
int dsi_enable_irq(enum DISP_MODULE_ENUM module, void *handle,
	unsigned int enable)
{
	if (module == DISP_MODULE_DSI0)
		DSI_OUTREGBIT(handle, struct DSI_INT_ENABLE_REG,
			DSI_REG[0]->DSI_INTEN, FRAME_DONE, enable);

	return 0;
}


/**
 * start dsi driver:
 * 1.shadow regs setting
 * 2.power init(dsi analogy)
 * 3.dsi irq setting
 */
int _ddp_dsi_start_dual(enum DISP_MODULE_ENUM module, void *cmdq)
{
	int g_lcm_x = disp_helper_get_option(DISP_OPT_FAKE_LCM_X);
	int g_lcm_y = disp_helper_get_option(DISP_OPT_FAKE_LCM_Y);

	DISPFUNC();

	if (module != DISP_MODULE_DSIDUAL)
		return 0;

	DSI_Send_ROI(DISP_MODULE_DSI0, cmdq, g_lcm_x, g_lcm_y,
		     _dsi_context[0].lcm_width, _dsi_context[0].lcm_height);

	DSI_SetMode(module, cmdq, _dsi_context[0].dsi_params.mode);
	DSI_clk_HS_mode(module, cmdq, TRUE);

	return 0;
}

/**
 * start dsi driver:
 * 1.shadow regs setting
 * 2.send roi
 * 3.dsi
 */
int ddp_dsi_start(enum DISP_MODULE_ENUM module, void *cmdq)
{
	int i = 0;
	int g_lcm_x = disp_helper_get_option(DISP_OPT_FAKE_LCM_X);
	int g_lcm_y = disp_helper_get_option(DISP_OPT_FAKE_LCM_Y);

	DISPFUNC();

	if (module == DISP_MODULE_DSIDUAL)
		return _ddp_dsi_start_dual(module, cmdq);
	else if (module == DISP_MODULE_DSI0)
		i = 0;
	else if (module == DISP_MODULE_DSI1)
		i = 1;
	else
		return 0;

	DSI_Send_ROI(module, cmdq, g_lcm_x, g_lcm_y, _dsi_context[i].lcm_width,
		     _dsi_context[i].lcm_height);
	DSI_SetMode(module, cmdq, _dsi_context[i].dsi_params.mode);
	DSI_clk_HS_mode(module, cmdq, TRUE);

	return 0;
}

/**
 * stop dual dsi means:
 * 1.wait frame done/command done(duan_en must = 0)
 * 2.set to command mode
 * 3.dual_en = 0
 * 3.hs clk disable
 *
 * timeout = 1s(fps = 1)
 */
static int _ddp_dsi_stop_dual(enum DISP_MODULE_ENUM module,
	void *cmdq_handle)
{
	int ret = 0;
	static const long WAIT_TIMEOUT = HZ; /* 1 sec */

	DISPFUNC();

	ASSERT(cmdq_handle == NULL);
	ASSERT(_check_dsi_mode(module));

	if (DSI_REG[0]->DSI_MODE_CTRL.MODE == CMD_MODE) {
		DISPMSG("dsi stop: CMD mode\n");

		ret = wait_event_timeout(_dsi_context[0].cmddone_wq.wq,
			!(DSI_REG[0]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi0 wait event for not busy timeout\n");
			DSI_DumpRegisters(DISP_MODULE_DSI0, 1);
			DSI_Reset(DISP_MODULE_DSI0, NULL);
		}

		ret = wait_event_timeout(_dsi_context[1].cmddone_wq.wq,
			!(DSI_REG[1]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi1 wait event for not busy timeout\n");
			DSI_DumpRegisters(DISP_MODULE_DSI1, 1);
			DSI_Reset(DISP_MODULE_DSI1, NULL);
		}
	} else {
		DISPMSG("dsi stop: brust mode(VDO mode lcm)\n");

		/* stop vdo mode */
		DSI_OUTREGBIT(cmdq_handle, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_SetMode(module, cmdq_handle, CMD_MODE);

		ret = wait_event_timeout(_dsi_context[0].vm_done_wq.wq,
			!(DSI_REG[0]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi0 wait event for not busy timeout\n");
			DSI_DumpRegisters(DISP_MODULE_DSI0, 1);
			DSI_Reset(DISP_MODULE_DSI0, NULL);
		}

		ret = wait_event_timeout(_dsi_context[1].vm_done_wq.wq,
			!(DSI_REG[1]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi1 wait event for not busy timeout\n");
			DSI_DumpRegisters(DISP_MODULE_DSI1, 1);
			DSI_Reset(DISP_MODULE_DSI1, NULL);
		}

		DSI_OUTREGBIT(cmdq_handle, struct DSI_COM_CTRL_REG,
			DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
	}

	DSI_clk_HS_mode(module, cmdq_handle, FALSE);

	DSI_OUTREG32(cmdq_handle, &DSI_REG[0]->DSI_INTEN, 0);
	DSI_OUTREG32(cmdq_handle, &DSI_REG[1]->DSI_INTEN, 0);

	return ret;
}

static int dsi_stop_vdo_mode(enum DISP_MODULE_ENUM module,
	void *cmdq_handle)
{
	/* use cmdq to stop dsi vdo mode */
	/* set dsi cmd mode */
	int i = 0;

	if (module == DISP_MODULE_DSI0)
		i = 0;
	else if (module == DISP_MODULE_DSI1)
		i = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		return _ddp_dsi_stop_dual(module, cmdq_handle);
	else
		return 0;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i])) {
		DISPERR("invalid dsi register addr%d\n", i);
		return -1;
	}
	DSI_SetMode(module, cmdq_handle, CMD_MODE);

	/* need do reset DSI_DUAL_EN/DSI_START */
	/* stop vdo mode */
	DSI_OUTREGBIT(cmdq_handle, struct DSI_START_REG,
		DSI_REG[i]->DSI_START, DSI_START, 0);

	/* polling dsi not busy */
	dsi_wait_not_busy(module, cmdq_handle);
	return 0;
}

/**
 * stop dsi means:
 * 1.wait frame done/command done
 * 2.set to command mode
 * 3.hs clk disable
 *
 * timeout = 1s(fps = 1)
 */
int ddp_dsi_stop(enum DISP_MODULE_ENUM module, void *cmdq_handle)
{
	int i = 0;
	int ret = 0;
	static const long WAIT_TIMEOUT = HZ; /* 1 sec */

	DISPFUNC();

	ASSERT(cmdq_handle == NULL);
	ASSERT(_check_dsi_mode(module));

	if (module == DISP_MODULE_DSI0)
		i = 0;
	else if (module == DISP_MODULE_DSI1)
		i = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		return _ddp_dsi_stop_dual(module, cmdq_handle);
	else
		return 0;

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i])) {
		DISPERR("invalid dsi register addr%d\n", i);
		return -1;
	}

	if (DSI_REG[i]->DSI_MODE_CTRL.MODE == CMD_MODE) {
		DISPDBG("dsi stop: command mode\n");
		ret = wait_event_timeout(_dsi_context[i].cmddone_wq.wq,
			!(DSI_REG[i]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi%d wait event for not busy timeout\n", i);
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
		}
	} else {
		DISPDBG("dsi stop: brust mode(vdo mode lcm)\n");
		/* stop vdo mode */
		DSI_OUTREGBIT(cmdq_handle, struct DSI_START_REG,
			DSI_REG[i]->DSI_START, DSI_START, 0);
		DSI_SetMode(module, cmdq_handle, CMD_MODE);

		ret = wait_event_timeout(_dsi_context[i].vm_done_wq.wq,
			!(DSI_REG[i]->DSI_INTSTA.BUSY), WAIT_TIMEOUT);
		if (ret == 0) {
			DISPERR("dsi%d wait event for not busy timeout\n", i);
			DSI_DumpRegisters(module, 1);
			DSI_Reset(module, NULL);
		}
	}

	DSI_clk_HS_mode(module, cmdq_handle, FALSE);
	DSI_OUTREG32(cmdq_handle, &DSI_REG[i]->DSI_INTEN, 0);
	DSI_OUTREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA, 0);

	return 0;
}

int ddp_dsi_switch_mode(enum DISP_MODULE_ENUM module, void *cmdq_handle,
	void *params)
{
	int i = 0, module_num;
	struct LCM_DSI_MODE_SWITCH_CMD lcm_cmd =
		*((struct LCM_DSI_MODE_SWITCH_CMD *) (params));
	struct LCM_DSI_PARAMS *dsi_params = &_dsi_context[0].dsi_params;
	int mode = (int)(lcm_cmd.mode);
	int wait_count = 100;

	if (!cmdq_handle) {
		ASSERT(0);
		return -1;
	}

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	if (dsi_currect_mode == mode) {
		DDPMSG("[%s] no switch mode, current mode = %d, switch to %d\n",
		       __func__, dsi_currect_mode, mode);
		return 0;
	}

	if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI0) {
		i = 0;
	} else if (lcm_cmd.cmd_if == (unsigned int)LCM_INTERFACE_DSI1) {
		i = 1;
	} else {
		DDPMSG("dsi switch not support this cmd IF:%d\n",
			lcm_cmd.cmd_if);
		/* return -1; */
	}

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("invalid dsi register addr%d\n", i);
		return -1;
	}

	if (mode == 0) { /* V2C */
		DISPMSG("[C2V]v2c switch begin\n");
		/* 1.polling dsi idle -- vdo mode over */
		DSI_SetMode(module, cmdq_handle, 0);
		DSI_POLLREG32(cmdq_handle, &DSI_REG[i]->DSI_INTSTA,
			0x80000000, 0x0);

		DISP_REG_SET_FIELD(cmdq_handle, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[i] + MIPITX_PLL_CON2, 1);

		dsi_params->ssc_disable = 0;

		/* 2.mutex setting -- cmd mode */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_RST,
			0x1, 0x1); /* reset mutex for V2C */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_RST,
			0x1, 0x0);
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_SOF,
			0x7, 0x0); /* mutex to cmd  mode */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_SOF,
			0x0, 0x40); /* eof */

		if (disp_helper_get_option(DISP_OPT_MUTEX_EOF_EN_FOR_CMD_MODE))
			DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_SOF,
				0x1c0, 0x40); /* eof */

		/* 3.te_rdy irq enable in dsi config */
		DSI_OUTREGBIT(NULL, struct DSI_INT_ENABLE_REG,
			DSI_REG[i]->DSI_INTEN, TE_RDY, 1);
		DSI_OUTREGBIT(cmdq_handle, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 1);

		/* 4.Set packet_size_mult */
		if (dsi_params->packet_size_mult) {
			unsigned int ps_wc = 0, h = 0;

			h = DSI_INREG32(struct DSI_VACT_NL_REG,
				&DSI_REG[i]->DSI_VACT_NL);
			h /= dsi_params->packet_size_mult;
			DSI_OUTREGBIT(cmdq_handle, struct DSI_VACT_NL_REG,
				DSI_REG[i]->DSI_VACT_NL, VACT_NL, h);
			ps_wc = DSI_INREG32(struct DSI_PSCTRL_REG,
				&DSI_REG[i]->DSI_PSCTRL);
			ps_wc *= dsi_params->packet_size_mult;
			DSI_OUTREGBIT(cmdq_handle, struct DSI_PSCTRL_REG,
				DSI_REG[i]->DSI_PSCTRL, DSI_PS_WC, ps_wc);
		}

		/* 5.Adjust PLL clk */
		DSI_PHY_clk_change(module, cmdq_handle, dsi_params);
		DSI_PHY_TIMCONFIG(module, cmdq_handle, dsi_params);

		/* 6.update one frame */
		DSI_OUTREG32(cmdq_handle, &DSI_CMDQ_REG[i]->data[0],
			0x002c3909);
		DSI_OUTREG32(cmdq_handle, &DSI_REG[i]->DSI_CMDQ_SIZE, 1);
		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DISP_RDMA0_EOF);
		DISP_REG_SET(cmdq_handle, DISP_REG_CONFIG_MUTEX0_EN, 1);
		DSI_Start(module, cmdq_handle);
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_EVENT_DISP_RDMA0_EOF);

		/* 7.blocking flush */
		cmdqRecFlush(cmdq_handle);
		cmdqRecReset(cmdq_handle);

		dsi_analysis(module);
		DSI_DumpRegisters(module, 2);

		DISPMSG("[C2V]v2c switch finished\n");
	} else { /* C2V */
		DISPMSG("[C2V]c2v switch begin\n");
		/* 1. Adjust PLL clk */
		cmdqRecWaitNoClear(cmdq_handle, CMDQ_SYNC_TOKEN_STREAM_EOF);
		DSI_PHY_clk_change(module, cmdq_handle, dsi_params);
		DSI_PHY_TIMCONFIG(module, cmdq_handle, dsi_params);

		/* 2. wait TE */
		cmdqRecClearEventToken(cmdq_handle, CMDQ_EVENT_DSI_TE);
		cmdqRecWait(cmdq_handle, CMDQ_EVENT_DSI_TE);

		/* 3. disable DSI EXT TE, only BTA te could work */
		DSI_OUTREGBIT(cmdq_handle, struct DSI_TXRX_CTRL_REG,
			DSI_REG[i]->DSI_TXRX_CTRL, EXT_TE_EN, 0);
		DISP_REG_SET_FIELD(cmdq_handle, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[i]+MIPITX_PLL_CON2, 0);
		dsi_params->ssc_disable = 1;

		/* 4. change to vdo mode */
		DSI_SetMode(module, cmdq_handle, mode);
		/* DSI_SetSwitchMode(module, cmdq_handle, 1); */
		/* DSI_SetBypassRack(module, cmdq_handle, 1); */

		/* 5. mutex setting */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_SOF,
			0x7, 0x1); /* sof */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_SOF,
				0x1c0, 0x40); /* eof */

		/* 6. Disable packet_size_mult */
		if (dsi_params->packet_size_mult) {
			unsigned int ps_wc = 0, h = 0;

			h = DSI_INREG32(struct DSI_VACT_NL_REG,
				&DSI_REG[i]->DSI_VACT_NL);
			h *= dsi_params->packet_size_mult;
			DSI_OUTREGBIT(cmdq_handle, struct DSI_VACT_NL_REG,
				DSI_REG[i]->DSI_VACT_NL, VACT_NL, h);
			ps_wc = DSI_INREG32(struct DSI_PSCTRL_REG,
				&DSI_REG[i]->DSI_PSCTRL);
			ps_wc /= dsi_params->packet_size_mult;
			DSI_OUTREGBIT(cmdq_handle, struct DSI_PSCTRL_REG,
				DSI_REG[i]->DSI_PSCTRL, DSI_PS_WC, ps_wc);
		}

		/* 7. trigger vdo mode frame update */
		DSI_MASKREG32(cmdq_handle, DISP_REG_CONFIG_MUTEX0_EN,
			0x1, 0x1); /* release mutex for video mode */
		DSI_Start(module, cmdq_handle);

		/* 8. blocking flush */
		cmdqRecFlush(cmdq_handle);
		cmdqRecReset(cmdq_handle);

		DISPINFO("[C2V]after c2v switch, cmdq flushed\n");

		/* THIS IS NOT A GOOD DESIGN!!!!! */
		/* TEMP WORKAROUND FOR ESD/CV SWITCH */
		/**************************************************************/
		while (wait_count) {
			DISPDBG("[C2V]wait loop %d\n", wait_count);
			if (DSI_REG[i]->DSI_STATE_DBG6.CMTRL_STATE == 0x1) {
				DISPDBG("[C2V]after switch, dsi fsm is idle\n");
				break;
			}
			lcm_mdelay(1);
			wait_count--;
		}

		if (wait_count == 0) {
			DISPWARN(
				"[C2V]after c2v switch, dsi state is not idle[0x%08x]\n",
				DSI_REG[i]->DSI_STATE_DBG6.CMTRL_STATE);
			dsi_analysis(module);
			DSI_DumpRegisters(module, 2);
			DSI_Reset(module, NULL);
			DSI_OUTREGBIT(NULL, struct DSI_MODE_CTRL_REG,
				DSI_REG[i]->DSI_MODE_CTRL, C2V_SWITCH_ON, 0);
			DSI_OUTREG32(NULL, &DSI_REG[i]->DSI_CMDQ_SIZE, 0);
			DSI_Start(module, NULL);
		}
		/**************************************************************/

		dsi_analysis(module);
		DSI_DumpRegisters(module, 2);

		/* 8. disable dsi auto rack */
		/* DSI_SetBypassRack(module, NULL, 0); */

		DISPMSG("[C2V]c2v switch finished\n");
	}

	dsi_currect_mode = mode;

	for (i = 0; i < module_num; i++)
		_dsi_context[i].dsi_params.mode = mode;

	return 0;
}

int ddp_dsi_ioctl(enum DISP_MODULE_ENUM module, void *cmdq_handle,
		  enum DDP_IOCTL_NAME ioctl_cmd, void *params)
{
	int ret = 0;
	enum DDP_IOCTL_NAME ioctl = (enum DDP_IOCTL_NAME)ioctl_cmd;

	/* DISPFUNC(); */
	/* DISPCHECK("[ddp_dsi_ioctl] index = %d\n", ioctl); */
	switch (ioctl) {
	case DDP_STOP_VIDEO_MODE:
	{
		dsi_stop_vdo_mode(module, cmdq_handle);
		break;
	}
	case DDP_SWITCH_DSI_MODE:
	{
		ret = ddp_dsi_switch_mode(module, cmdq_handle, params);
		break;
	}
	case DDP_SWITCH_LCM_MODE:
	{
		/* ret = ddp_dsi_switch_lcm_mode(module, params); */
		break;
	}
	case DDP_BACK_LIGHT:
	{
		unsigned int cmd = 0x51;
		unsigned int count = 1;
		unsigned int *plevel = params;
		unsigned int level = *plevel;

		DDPMSG("[ddp_dsi_ioctl] level = %d\n", level);
		DSI_set_cmdq_V2(module, cmdq_handle, cmd, count,
			(unsigned char *)&level, 1);
		break;
	}
	case DDP_DSI_IDLE_CLK_CLOSED:
	{
		break;
	}
	case DDP_DSI_IDLE_CLK_OPEN:
	{
		break;
	}
	case DDP_DSI_PORCH_CHANGE:
	{
		if (params == NULL) {
			DDPERR("[ddp_dsi_ioctl] input pointer is NULL\n");
		} else {
			unsigned int *p = (unsigned int *)params;
			unsigned int vfp = p[0];

			DDPMSG("[ddp_dsi_ioctl] DDP_DSI_PORCH_CHANGE vfp=%d\n",
				vfp);
			ddp_dsi_porch_setting(module, cmdq_handle,
				DSI_VFP, vfp);
		}
		break;
	}
	case DDP_DSI_PORCH_ADDR:
	{
		if (params == NULL) {
			DDPERR("[ddp_dsi_ioctl] input pointer is NULL\n");
		} else {
			unsigned int *p = (unsigned int *)params;
			unsigned int addr = p[0];

			DDPMSG("[ddp_dsi_ioctl] DDP_DSI_PORCH_ADDR addr=0x%x\n",
				addr);
			DSI_Get_Porch_Addr(module, params);
		}
		break;
	}
	case DDP_PHY_CLK_CHANGE:
	{
		struct LCM_DSI_PARAMS *dsi_params =
			&_dsi_context[0].dsi_params;
		unsigned int *p = params;

		dsi_params->PLL_CLOCK = *p;
		/* DSI_WaitForNotBusy(module, cmdq_handle); */
		DSI_PHY_clk_change(module, cmdq_handle, dsi_params);
		DSI_PHY_TIMCONFIG(module, cmdq_handle, dsi_params);
		break;
	}
	case DDP_UPDATE_PLL_CLK_ONLY:
	{
		struct LCM_DSI_PARAMS *dsi_params =
			&_dsi_context[0].dsi_params;
		unsigned int *p = params;

		dsi_params->PLL_CLOCK = *p;
		break;
	}
	case DDP_PARTIAL_UPDATE:
	{
		ddp_dsi_update_partial(module, cmdq_handle, params);
		break;
	}
	case DDP_DSI_ENABLE_TE:
	{
		DISPDBG("[DDPDSI] enable TE\n");
		DSI_OUTREGBIT(cmdq_handle, struct DSI_INT_ENABLE_REG,
			DSI_REG[0]->DSI_INTEN, TE_RDY, 1);
		break;
	}
	case DDP_DSI_ENTER_IDLE:
	{
		DDPDBG("DDP_DSI_ENTER_IDLE\n");
		atomic_set(&dsi_idle_flg, 1);
		break;
	}
	case DDP_DSI_EXIT_IDLE:
	{
		DDPDBG("DDP_DSI_EXIT_IDLE\n");
		atomic_set(&dsi_idle_flg, 0);
		break;
	}

	default:
		break;
	}
	return ret;
}

int ddp_dsi_trigger(enum DISP_MODULE_ENUM module, void *cmdq)
{
	int i = 0;
	unsigned int data_array[16];

	DISPFUNC();
#ifdef CONFIG_FPGA_EARLY_PORTING
	/* reconfig default value of 0x100 for b0384 */
	DSI_OUTREG32(cmdq, &DSI_REG[0]->DSI_PHY_PCPAT, 0x55);
#endif

	/* fhd no split setting */
	DSI_OUTREG32(cmdq, DISP_REG_CONFIG_MMSYS_MISC, 0x14);

	if (_dsi_context[i].dsi_params.mode == CMD_MODE) {
		data_array[0] = 0x002c3909;
		DSI_set_cmdq(module, cmdq, data_array, 1, 0);
	}

	if (module == DISP_MODULE_DSIDUAL) {
		DISPCHECK("dsi0 start = %d\n",
			DSI_REG[0]->DSI_START.DSI_START);
		DSI_OUTREGBIT(cmdq, struct DSI_START_REG,
			DSI_REG[0]->DSI_START, DSI_START, 0);
		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
			DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
	}

	DSI_Start(module, cmdq);

	if (module == DISP_MODULE_DSIDUAL &&
		_dsi_context[i].dsi_params.mode == CMD_MODE) {
		/* Reading one reg is only used for delay
		 * in order to pull down DSI_DUAL_EN.
		 */
		if (cmdq)
			cmdqRecBackupRegisterToSlot(cmdq, _h_intstat, 0,
				disp_addr_convert(
				(unsigned long)(&DSI_REG[0]->DSI_INTSTA)));
		else
			INREG32(&DSI_REG[0]->DSI_INTSTA);

		DSI_OUTREGBIT(cmdq, struct DSI_COM_CTRL_REG,
			DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
	}

	return 0;
}

int ddp_dsi_reset(enum DISP_MODULE_ENUM module, void *cmdq_handle)
{
	DSI_Reset(module, cmdq_handle);

	return 0;
}

unsigned int _is_power_on_status(enum DISP_MODULE_ENUM module)
{
	if (module == DISP_MODULE_DSIDUAL) {
		if (_dsi_context[0].is_power_on == 1 &&
			_dsi_context[1].is_power_on == 1)
			return 1;
		if (_dsi_context[0].is_power_on == 0 &&
			_dsi_context[1].is_power_on == 0)
			return 0;
		ASSERT(0);
	} else if (module == DISP_MODULE_DSI0) {
		return _dsi_context[0].is_power_on;
	} else if (module == DISP_MODULE_DSI1) {
		return _dsi_context[1].is_power_on;
	}

	return 0;
}

/**
 * ddp_dsi_power_on
 *
 * 1.mipi 26m cg
 * 2.mipi init
 * 3.dsi cg
 * 4.dsi exit ulps
 * 5._set_power_on_status
 *
 * 1.mipi 26m cg
 * 2.dsi cg
 * 3._set_power_on_status
 *
 * important: mipi init -> dsi init
 */
int ddp_dsi_power_on(enum DISP_MODULE_ENUM module, void *cmdq_handle)
{
	DISPFUNC();
	if (_is_power_on_status(module))
		return DSI_STATUS_OK;

	ddp_set_mipi26m(module, 1);

	DSI_PHY_clk_switch(module, NULL, true);

	if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
		ddp_clk_prepare_enable(CLK_DSI0_MM_CLK);
		ddp_clk_prepare_enable(CLK_DSI0_IF_CLK);
	}

	DPHY_Reset(module, cmdq_handle);
	/* DSI_RestoreRegisters(module, NULL); */
	if (atomic_read(&dsi_idle_flg) == 0)
		DSI_exit_ULPS(module);
	DSI_Reset(module, NULL);
	_set_power_on_status(module, 1);

	return DSI_STATUS_OK;
}

/**
 * ddp_dsi_power_off
 *
 * 1.enter dsi ulps
 * 2.dsi clk gating
 * 3.mipi deinit
 * 4.mipi 26m cg
 * 5._set_power_on_status
 *
 * important: dsi deinit->mipi deinit
 */
int ddp_dsi_power_off(enum DISP_MODULE_ENUM module, void *cmdq_handle)
{
	uint i = 0;

	DISPFUNC();
	if (!_is_power_on_status(module))
		return DSI_STATUS_OK;

	/* DSI_BackupRegisters(module, NULL); */
	if (atomic_read(&dsi_idle_flg) == 0)
		DSI_enter_ULPS(module);


#ifdef ENABLE_CLK_MGR
	if (module == DISP_MODULE_DSI0 || module == DISP_MODULE_DSIDUAL) {
		ddp_clk_disable_unprepare(CLK_DSI0_MM_CLK);
		ddp_clk_disable_unprepare(CLK_DSI0_IF_CLK);
	}
#endif

	DSI_PHY_clk_switch(module, NULL, false);

#ifdef ENABLE_CLK_MGR
	ddp_set_mipi26m(module, 0);
#endif
	/*DynFPS*/
	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		_dsi_context[i].disp_fps = 0;
		_dsi_context[i].dynfps_chg_index = 0;
	}

	_set_power_on_status(module, 0);
	return DSI_STATUS_OK;
}

int ddp_dsi_is_busy(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;
	int busy = 0;
	struct DSI_INT_STATUS_REG status;

	/* DISPFUNC(); */
	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return -1;
	}

	for (i = 0; i < module_num; i++) {
		status = DSI_REG[i]->DSI_INTSTA;

		if (status.BUSY)
			busy++;
	}

	DISPDBG("%s is %s\n",
		ddp_get_module_name(module), busy ? "busy" : "idle");
	return busy;
}

int ddp_dsi_is_idle(enum DISP_MODULE_ENUM module)
{
	return !ddp_dsi_is_busy(module);
}

static const char *dsi_mode_spy(enum LCM_DSI_MODE_CON mode)
{
	switch (mode) {
	case CMD_MODE:
		return "CMD_MODE";
	case SYNC_PULSE_VDO_MODE:
		return "SYNC_PULSE_VDO_MODE";
	case SYNC_EVENT_VDO_MODE:
		return "SYNC_EVENT_VDO_MODE";
	case BURST_VDO_MODE:
		return "BURST_VDO_MODE";
	default:
		return "unknown";
	}
}

void dsi_analysis(enum DISP_MODULE_ENUM module)
{
	int i = 0, module_num;

	if (module == DISP_MODULE_DSI0 ||
		module == DISP_MODULE_DSI1)
		module_num = 1;
	else if (module == DISP_MODULE_DSIDUAL)
		module_num = 2;
	else {
		DISPERR("invlidate dsi module %d\n", (unsigned int)module);
		return;
	}

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i])) {
		DISPERR("invalid dsi register addr%d\n", i);
		return;
	}

	DDPDUMP("== DISP DSI ANALYSIS ==\n");
	for (i = 0; i < module_num; i++) {
		DSI_REG[i] = (struct DSI_REGS *)DISPSYS_DSI0_BASE;
#ifndef CONFIG_FPGA_EARLY_PORTING
		DDPDUMP("MIPITX Clock: %d\n", dsi_phy_get_clk(module));
#endif
		DDPDUMP("DSI%d Start:%x, Busy:%d, DSI_DUAL_EN:%d\n",
			i, DSI_REG[i]->DSI_START.DSI_START,
			DSI_REG[i]->DSI_INTSTA.BUSY,
			DSI_REG[i]->DSI_COM_CTRL.DSI_DUAL_EN);

		DDPDUMP("DSI%d MODE:%s, High Speed:%d, FSM State:%s\n",
			i, dsi_mode_spy(DSI_REG[i]->DSI_MODE_CTRL.MODE),
			DSI_REG[i]->DSI_PHY_LCCON.LC_HS_TX_EN,
			_dsi_cmd_mode_parse_state(
				DSI_REG[i]->DSI_STATE_DBG6.CMTRL_STATE));

		DDPDUMP("DSI%d IRQ,RD_RDY:%d, CMD_DONE:%d, SLEEPOUT_DONE:%d\n",
			i, DSI_REG[i]->DSI_INTSTA.RD_RDY,
			DSI_REG[i]->DSI_INTSTA.CMD_DONE,
			DSI_REG[i]->DSI_INTSTA.SLEEPOUT_DONE);

		DDPDUMP("DSI%d TE_RDY:%d, VM_CMD_DONE:%d, VM_DONE:%d\n",
			i, DSI_REG[i]->DSI_INTSTA.TE_RDY,
			DSI_REG[i]->DSI_INTSTA.VM_CMD_DONE,
			DSI_REG[i]->DSI_INTSTA.VM_DONE);

		DDPDUMP("DSI%d Lane Num:%d, Ext_TE_EN:%d\n",
			i, DSI_REG[i]->DSI_TXRX_CTRL.LANE_NUM,
			DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EN);

		DDPDUMP("DSI%d Ext_TE_Edge:%d, HSTX_CKLP_EN:%d\n",
			i, DSI_REG[i]->DSI_TXRX_CTRL.EXT_TE_EDGE,
			DSI_REG[i]->DSI_TXRX_CTRL.HSTX_CKLP_EN);

		DDPDUMP("DSI%d LFR En:%d, LFR MODE:%d\n",
			i, DSI_REG[i]->DSI_LFR_CON.LFR_EN,
			DSI_REG[i]->DSI_LFR_CON.LFR_MODE);

		DDPDUMP("DSI%d LFR TYPE:%d, LFR SKIP NUMBER:%d\n",
			i, DSI_REG[i]->DSI_LFR_CON.LFR_TYPE,
			DSI_REG[i]->DSI_LFR_CON.LFR_SKIP_NUM);
	}
}

int ddp_dsi_dump(enum DISP_MODULE_ENUM module, int level)
{
	if (!_is_power_on_status(module)) {
		DISPWARN("sleep dump is invalid\n");
		return 0;
	}

	dsi_analysis(module);
	DSI_DumpRegisters(module, level);

	return 0;
}

int DSI_esd_check_num(struct LCM_DSI_PARAMS *dsi_params)
{
	int i, cnt;

	for (i = 0, cnt = 0; i < 3; i++) {
		if (dsi_params->lcm_esd_check_table[i].cmd)
			cnt++;
	}

	return cnt;
}

int ddp_dsi_build_cmdq(enum DISP_MODULE_ENUM module,
	void *cmdq_trigger_handle, enum CMDQ_STATE state)
{
	int ret = 0;
	unsigned int i = 0, j = 0;
	unsigned int dsi_i = 0;
	struct LCM_DSI_PARAMS *dsi_params = NULL;
	struct DSI_T0_INS t0, t1;
	struct DSI_RX_DATA_REG read_data0;
	struct DSI_RX_DATA_REG read_data1;
	struct DSI_RX_DATA_REG read_data2;
	struct DSI_RX_DATA_REG read_data3;
	unsigned char packet_type;
	unsigned char buffer[30];
	memset((void *)buffer, 0, 30);
	int recv_data_cnt = 0;

	static cmdqBackupSlotHandle hSlot[4] = {0, 0, 0, 0};

	if (module == DISP_MODULE_DSIDUAL)
		dsi_i = 0;
	else
		dsi_i = DSI_MODULE_to_ID(module);

	i = DSI_MODULE_to_ID(module);
	if (!((unsigned long)DSI_REG[i]) ||
		!((unsigned long)DSI_CMDQ_REG[i])) {
		DISPERR("invalid dsi register addr%d\n", i);
		return -1;
	}

	dsi_params = &_dsi_context[dsi_i].dsi_params;

	if (cmdq_trigger_handle == NULL) {
		DISPERR("cmdq_trigger_handle is NULL\n");
		return -1;
	}

	if (state == CMDQ_WAIT_LCM_TE) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0 ||
			module == DISP_MODULE_DSIDUAL) {
			if (dsi0_te_enable == 0)
				return 0;

			if (disp_helper_get_option(DISP_OPT_USE_CMDQ)) {
				ret = cmdqRecClearEventToken(
					cmdq_trigger_handle,
					CMDQ_EVENT_DSI_TE);
				ret = cmdqRecWait(cmdq_trigger_handle,
					CMDQ_EVENT_DSI_TE);
			}
		} else {
			DISPWARN("wrong module: %s\n",
				ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_CHECK_IDLE_AFTER_STREAM_EOF) {
		/* need waiting te */
		if (module == DISP_MODULE_DSI0 ||
			module == DISP_MODULE_DSIDUAL) {
			DSI_POLLREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_INTSTA, 0x80000000, 0);
		} else {
			DISPWARN("wrong module: %s\n",
				ddp_get_module_name(module));
			return -1;
		}
	} else if (state == CMDQ_ESD_CHECK_READ) {
		/* enable dsi interrupt: RD_RDY/CMD_DONE (need do this here?) */
		DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_INT_ENABLE_REG,
			      DSI_REG[dsi_i]->DSI_INTEN, RD_RDY, 1);
		DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_INT_ENABLE_REG,
			      DSI_REG[dsi_i]->DSI_INTEN, CMD_DONE, 1);

		for (i = 0; i < 3; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;

			/* 0. send read lcm command(short packet) */
			t0.CONFG = 0x04; /* /BTA */
			t0.Data0 = dsi_params->lcm_esd_check_table[i].cmd;
			t0.Data_ID =
				(t0.Data0 < 0xB0) ?
				DSI_DCS_READ_PACKET_ID :
				DSI_GERNERIC_READ_LONG_PACKET_ID;
			t0.Data1 = 0;

			t1.CONFG = 0x00;
			t1.Data_ID = 0x37;
			t1.Data0 = dsi_params->lcm_esd_check_table[i].count;
			t1.Data1 = 0;

			/* write DSI CMDQ */
			DSI_OUTREG32(cmdq_trigger_handle,
				&DSI_CMDQ_REG[dsi_i]->data[0], AS_UINT32(&t1));
			DSI_OUTREG32(cmdq_trigger_handle,
				&DSI_CMDQ_REG[dsi_i]->data[1], AS_UINT32(&t0));
			DSI_OUTREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_CMDQ_SIZE, 2);

			/* start DSI */
			DSI_OUTREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_START, 0);
			DSI_OUTREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_START, 1);

			/* 1. wait DSI RD_RDY(must clear,
			 * in case of cpu RD_RDY interrupt handler)
			 */
			if (dsi_i == 0) {
				DSI_POLLREG32(cmdq_trigger_handle,
					&DSI_REG[dsi_i]->DSI_INTSTA,
					0x00000001, 0x1);
				DSI_OUTREGBIT(cmdq_trigger_handle,
					struct DSI_INT_STATUS_REG,
					DSI_REG[dsi_i]->DSI_INTSTA,
					RD_RDY, 0x00000000);
			}
			/* 2. save RX data */
			if (hSlot[0] && hSlot[1] && hSlot[2] && hSlot[3]) {
				DSI_BACKUPREG32(cmdq_trigger_handle,
					hSlot[0], i,
					&DSI_REG[0]->DSI_RX_DATA0);
				DSI_BACKUPREG32(cmdq_trigger_handle,
					hSlot[1], i,
					&DSI_REG[0]->DSI_RX_DATA1);
				DSI_BACKUPREG32(cmdq_trigger_handle,
					hSlot[2], i,
					&DSI_REG[0]->DSI_RX_DATA2);
				DSI_BACKUPREG32(cmdq_trigger_handle,
					hSlot[3], i,
					&DSI_REG[0]->DSI_RX_DATA3);
			}
			/* 3. write RX_RACK */
			DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_RACK_REG,
				DSI_REG[dsi_i]->DSI_RACK, DSI_RACK, 1);

			/* 4. polling not busy(no need clear) */
			if (dsi_i == 0) {
				DSI_POLLREG32(cmdq_trigger_handle,
					&DSI_REG[dsi_i]->DSI_INTSTA,
					0x80000000, 0);
			}
			/* loop: 0~4 */
		}

	} else if (state == CMDQ_ESD_CHECK_CMP) {
		struct LCM_esd_check_item *lcm_esd_tb;
		/* cmp just once and only 1 return value */
		for (i = 0; i < 3; i++) {
			if (dsi_params->lcm_esd_check_table[i].cmd == 0)
				break;

			/* read data */
			if (hSlot[0] && hSlot[1] && hSlot[2] && hSlot[3]) {
				/* read from slot */
				cmdqBackupReadSlot(hSlot[0], i,
					(uint32_t *)&read_data0);
				cmdqBackupReadSlot(hSlot[1], i,
					(uint32_t *)&read_data1);
				cmdqBackupReadSlot(hSlot[2], i,
					(uint32_t *)&read_data2);
				cmdqBackupReadSlot(hSlot[3], i,
					(uint32_t *)&read_data3);
			} else if (i == 0) {
				/* read from dsi, support only one cmd read */
				DSI_OUTREG32(NULL, &read_data0, AS_UINT32(
					&DSI_REG[dsi_i]->DSI_RX_DATA0));
				DSI_OUTREG32(NULL, &read_data1, AS_UINT32(
					&DSI_REG[dsi_i]->DSI_RX_DATA1));
				DSI_OUTREG32(NULL, &read_data2, AS_UINT32(
					&DSI_REG[dsi_i]->DSI_RX_DATA2));
				DSI_OUTREG32(NULL, &read_data3, AS_UINT32(
					&DSI_REG[dsi_i]->DSI_RX_DATA3));
			}

			lcm_esd_tb = &dsi_params->lcm_esd_check_table[i];

			DISPDBG("[DSI]enter cmp read_data0 byte0~1=0x%x~0x%x\n",
				read_data0.byte0, read_data0.byte1);
			DISPDBG("[DSI]enter cmp read_data0 byte2~3=0x%x~0x%x\n",
				read_data0.byte2, read_data0.byte3);
			DISPDBG("[DSI]enter cmp read_data1 byte0~1=0x%x~0x%x\n",
				read_data1.byte0, read_data1.byte1);
			DISPDBG("[DSI]enter cmp read_data1 byte2~3=0x%x~0x%x\n",
				read_data1.byte2, read_data1.byte3);
			DISPDBG("[DSI]enter cmp read_data2 byte0~1=0x%x~0x%x\n",
				read_data2.byte0, read_data2.byte1);
			DISPDBG("[DSI]enter cmp read_data2 byte2~3=0x%x~0x%x\n",
				read_data2.byte2, read_data2.byte3);
			DISPDBG("[DSI]enter cmp read_data3 byte0~1=0x%x~0x%x\n",
				read_data3.byte0, read_data3.byte1);
			DISPDBG("[DSI]enter cmp read_data3 byte2~3=0x%x~0x%x\n",
				read_data3.byte2, read_data3.byte3);

			DISPDBG("[DSI]enter cmp check_tab cmd=0x%x,cnt=0x%x\n",
				lcm_esd_tb->cmd, lcm_esd_tb->count);
			DISPDBG
	("[DSI]para_list[0]=0x%x,para_list[1]=0x%x, para_list[2]=0x%x\n",
				lcm_esd_tb->para_list[0],
				lcm_esd_tb->para_list[1],
				lcm_esd_tb->para_list[2]);
			DISPDBG("[DSI]enter cmp DSI+0x200=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x200));
			DISPDBG("[DSI]enter cmp DSI+0x204=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x204));
			DISPDBG("[DSI]enter cmp DSI+0x60=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x60));
			DISPDBG("[DSI]enter cmp DSI+0x74=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x74));
			DISPDBG("[DSI]enter cmp DSI+0x88=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x88));
			DISPDBG("[DSI]enter cmp DSI+0x0c=0x%x\n",
				AS_UINT32(DISPSYS_DSI0_BASE + 0x0c));

			packet_type = read_data0.byte0;
			/* 0x02: acknowledge & error report */
			/* 0x11: generic short read response(1 byte return) */
			/* 0x12: generic short read response(2 byte return) */
			/* 0x1a: generic long read response */
			/* 0x1c: dcs long read response */
			/* 0x21: dcs short read response(1 byte return) */
			/* 0x22: dcs short read response(2 byte return) */
			if (packet_type == 0x1A || packet_type == 0x1C) {
				recv_data_cnt = read_data0.byte1
					+ read_data0.byte2 * 16;

				if (recv_data_cnt > RT_MAX_NUM) {
					DISPDBG
			("DSI read long packet data exceeds 10 bytes\n");
					recv_data_cnt = RT_MAX_NUM;
				}
				if (recv_data_cnt > lcm_esd_tb->count)
					recv_data_cnt = lcm_esd_tb->count;

				DISPCHECK("DSI read long packet size: %d\n",
					recv_data_cnt);
				if (recv_data_cnt <= 4) {
					memcpy((void *)buffer,
					(void *)&read_data1, recv_data_cnt);
				} else if (recv_data_cnt <= 8) {
					memcpy((void *)buffer,
					(void *)&read_data1, 4);
					memcpy((void *)(buffer + 4),
					(void *)&read_data2, recv_data_cnt - 4);
				} else {
					memcpy((void *)buffer,
						(void *)&read_data1, 4);
					memcpy((void *)(buffer + 4),
						(void *)&read_data2, 4);
					memcpy((void *)(buffer + 8),
					(void *)&read_data3, recv_data_cnt - 8);
				}

			} else if (packet_type == 0x11 || packet_type == 0x21) {
				recv_data_cnt = 1;
				memcpy((void *)buffer,
				(void *)&read_data0.byte1, recv_data_cnt);

			} else if (packet_type == 0x12 || packet_type == 0x22) {
				recv_data_cnt = 2;
				if (recv_data_cnt > lcm_esd_tb->count)
					recv_data_cnt = lcm_esd_tb->count;

				memcpy((void *)buffer,
				(void *)&read_data0.byte1, recv_data_cnt);

			} else if (packet_type == 0x02) {
				DISPCHECK
					("read return type is 0x02, re-read\n");
			} else {
				DISPCHECK
			("read return type is non-recognite, type = 0x%x\n",
					packet_type);
			}
			DISPDBG("[DSI]packet_type~recv_data_cnt = 0x%x~0x%x\n",
				packet_type, recv_data_cnt);
			/*do read data cmp*/
			for (j = 0; j < lcm_esd_tb->count; j++) {

				DISPDBG("buffer[%d]=0x%x\n", j, buffer[j]);
				if (buffer[j] != lcm_esd_tb->para_list[j]) {
					DISPDBG
			("buffer[%d]0x%x != lcm_esd_tb->para_list[%d]0x%x\n",
				j, buffer[j], j, lcm_esd_tb->para_list[j]);

					ret |= 1;/*esd failed*/
					break;
				}
				ret |= 0;/*esd pass*/
				DISPDBG("[DSI]cmp pass cnt = %d\n", j);
			}

			if (ret)/*esd failed*/
				break;
		}
	} else if (state == CMDQ_ESD_ALLC_SLOT) {
		/* create 3 slot */
		unsigned int h = 0, n = 0;

		n = DSI_esd_check_num(dsi_params);
		for (h = 0; h < 4; h++)
			cmdqBackupAllocateSlot(&hSlot[h], n);

	} else if (state == CMDQ_ESD_FREE_SLOT) {
		unsigned int h = 0;

		for (h = 0; h < 4; h++) {
			if (hSlot[h]) {
				cmdqBackupFreeSlot(hSlot[h]);
				hSlot[h] = 0;
			}
		}
	} else if (state == CMDQ_STOP_VDO_MODE) {
		/* use cmdq to stop dsi vdo mode */
		/* 0. set dsi cmd mode */
		DSI_SetMode(module, cmdq_trigger_handle, CMD_MODE);

		/* 2.dual dsi need do reset DSI_DUAL_EN/DSI_START */
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_COM_CTRL_REG,
				 DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_COM_CTRL_REG,
				DSI_REG[1]->DSI_COM_CTRL, DSI_DUAL_EN, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_START_REG,
				DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_START_REG,
				DSI_REG[1]->DSI_START, DSI_START, 0);
		} else if (module == DISP_MODULE_DSI0) {
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_START_REG,
				DSI_REG[0]->DSI_START, DSI_START, 0);
		} else if (module == DISP_MODULE_DSI1) {
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_START_REG,
				DSI_REG[1]->DSI_START, DSI_START, 0);
		}

		/* 1. polling dsi not busy */
		i = DSI_MODULE_BEGIN(module);
		if (i == 0) {
			/* polling dsi busy */
			DSI_POLLREG32(cmdq_trigger_handle,
				&DSI_REG[i]->DSI_INTSTA, 0x80000000, 0);
		}

		i = DSI_MODULE_END(module);
		if (i == 1) /* DUAL */
			DSI_POLLREG32(cmdq_trigger_handle,
				&DSI_REG[i]->DSI_INTSTA, 0x80000000, 0);
	} else if (state == CMDQ_START_VDO_MODE) {
		/* 0. dual dsi set DSI_START/DSI_DUAL_EN */
		if (module == DISP_MODULE_DSIDUAL) {
			DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_START_REG,
				DSI_REG[0]->DSI_START, DSI_START, 0);
			DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_START_REG,
				DSI_REG[1]->DSI_START, DSI_START, 0);

			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_COM_CTRL_REG,
				DSI_REG[0]->DSI_COM_CTRL, DSI_DUAL_EN, 1);
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_COM_CTRL_REG,
				DSI_REG[1]->DSI_COM_CTRL,
				DSI_DUAL_EN, 1);
		} else if (module == DISP_MODULE_DSI0) {
			DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_START_REG,
				DSI_REG[0]->DSI_START, DSI_START, 0);
		} else if (module == DISP_MODULE_DSI1) {
			DSI_OUTREGBIT(cmdq_trigger_handle, struct DSI_START_REG,
				DSI_REG[1]->DSI_START, DSI_START, 0);
		}

		/* 1. set dsi vdo mode */
		DSI_SetMode(module, cmdq_trigger_handle, dsi_params->mode);

	} else if (state == CMDQ_DSI_RESET) {
		DISPCHECK("CMDQ Timeout, Reset DSI\n");
		DSI_DumpRegisters(module, 1);
		DSI_Reset(module, NULL);
	} else if (state == CMDQ_DSI_LFR_MODE) {
		if (dsi_params->lfr_mode == 2 || dsi_params->lfr_mode == 3)
			DSI_LFR_UPDATE(module, cmdq_trigger_handle);
	}

	return ret;
}

int ddp_dsi_read_lcm_cmdq(enum DISP_MODULE_ENUM module,
	cmdqBackupSlotHandle *read_Slot,
	struct cmdqRecStruct *cmdq_trigger_handle,
	struct ddp_lcm_read_cmd_table *read_table)
{
	int ret = 0;
	unsigned int i = 0;
	unsigned int dsi_i = 0;

	struct DSI_T0_INS t0;

	dsi_i = DSI_MODULE_to_ID(module);

	if (dsi_i != 0)
		DISPERR("[DSI]should use dsi0\n");

	if (*read_Slot == 0) {
		ret = -1;
		DISPERR("[DSI]alloc cmdq slot fail\n");
		return ret;
	}
	/* enable dsi interrupt: RD_RDY/CMD_DONE (need do this here?) */
	DSI_OUTREGBIT(cmdq_trigger_handle,
		struct DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN,
			RD_RDY, 1);
	DSI_OUTREGBIT(cmdq_trigger_handle,
		struct DSI_INT_ENABLE_REG, DSI_REG[dsi_i]->DSI_INTEN,
			CMD_DONE, 1);

	for (i = 0; i < 3; i++) {
		if (read_table->cmd[i] == 0)
			break;
		/* 0. send read lcm command(short packet) */
		t0.CONFG = 0x04;	/* /BTA */
		t0.Data0 = read_table->cmd[i];
		/* / 0xB0 is used to distinguish DCS cmd */
		/* or Gerneric cmd, is that Right??? */
		t0.Data_ID =
			(t0.Data0 < 0xB0) ?
			DSI_DCS_READ_PACKET_ID :
			DSI_GERNERIC_READ_LONG_PACKET_ID;
		t0.Data1 = 0;

			/* write DSI CMDQ */
		DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[0],
				    0x00013700);
		DSI_OUTREG32(cmdq_trigger_handle, &DSI_CMDQ_REG[dsi_i]->data[1],
				     AS_UINT32(&t0));
		DSI_OUTREG32(cmdq_trigger_handle,
			&DSI_REG[dsi_i]->DSI_CMDQ_SIZE, 2);

		/* start DSI */
		DSI_OUTREG32(cmdq_trigger_handle,
			&DSI_REG[dsi_i]->DSI_START, 0);
		DSI_OUTREG32(cmdq_trigger_handle,
			&DSI_REG[dsi_i]->DSI_START, 1);

		/* 1. wait DSI RD_RDY(must clear,*/
		/* in case of cpu RD_RDY interrupt handler) */
		if (dsi_i == 0) {
			DSI_POLLREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_INTSTA, 0x00000001, 0x1);
			DSI_OUTREGBIT(cmdq_trigger_handle,
				struct DSI_INT_STATUS_REG,
				DSI_REG[dsi_i]->DSI_INTSTA,
				RD_RDY, 0x00000000);
		}
		/* 2. save RX data */
		if (*read_Slot && dsi_i == 0) {
			DSI_BACKUPREG32(cmdq_trigger_handle,
				*read_Slot, i,
				&DSI_REG[dsi_i]->DSI_RX_DATA0);
		}

		/* 3. write RX_RACK */
		DSI_OUTREGBIT(cmdq_trigger_handle,
			struct DSI_RACK_REG, DSI_REG[dsi_i]->DSI_RACK,
				    DSI_RACK, 1);

		/* 4. polling not busy(no need clear) */
		if (dsi_i == 0) {
			DSI_POLLREG32(cmdq_trigger_handle,
				&DSI_REG[dsi_i]->DSI_INTSTA,
				0x80000000, 0);
		}
		/* loop: 0~2*/
	}
	return ret;
}


int ddp_dsi_write_lcm_cmdq(enum DISP_MODULE_ENUM module,
	struct cmdqRecStruct *cmdq, unsigned  char cmd_char,
	unsigned char count, unsigned char *para_list)
{
	UINT32 i = 0;
	int d = 0;
	int ret = 0;
	unsigned long goto_addr, mask_para, set_para;
	unsigned int cmd = 0;
	struct DSI_T0_INS t0 = {0};
	struct DSI_T2_INS t2 = {0};

	if (module == DISP_MODULE_DSI0)
		d = 0;
	else
		return -1;

	for (i = 0; i < count; i++)
		DISPDBG("ddp_dsi_write_lcm_cmdq list %x\n", para_list[i]);
	cmd = (unsigned int)cmd_char;
	DISPDBG("ddp_dsi_write_lcm_cmdq cmd %x, count = %x\n", cmd, count);

	if (cmdq == NULL)
		return ret;

	DSI_POLLREG32(cmdq, &DSI_REG[d]->DSI_INTSTA, 0x80000000, 0x0);
	if (cmd < 0xB0) {
		if (count > 1) {
			t2.CONFG = 2;
			t2.Data_ID = DSI_DCS_LONG_PACKET_ID;
			t2.WC16 = count + 1;
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
						AS_UINT32(&t2));

			goto_addr =
				(unsigned long)(
				&DSI_CMDQ_REG[d]->data[1].byte0);
			mask_para = (0xFFu <<
				((goto_addr & 0x3u) * 8));
			set_para =
				(cmd << ((goto_addr & 0x3u) * 8));
			DSI_MASKREG32(cmdq,
				goto_addr & (~(0x3u)),
				mask_para, set_para);

			for (i = 0; i < count; i++) {
				goto_addr =
				(unsigned long)(&DSI_CMDQ_REG[d]->data[1].
							byte1) + i;
				mask_para = (0xFFu << ((goto_addr & 0x3u) * 8));
				set_para =
				(para_list[i] << ((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
					goto_addr & (~(0x3u)),
					mask_para, set_para);
			}

			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					2 + (count) / 4);
		} else {
			t0.CONFG = 0;
			t0.Data0 = cmd;
			if (count) {
				t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_1;
				t0.Data1 = para_list[0];
			} else {
				t0.Data_ID = DSI_DCS_SHORT_PACKET_ID_0;
				t0.Data1 = 0;
			}
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
							AS_UINT32(&t0));
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
			}
	} else {
		if (count > 1) {
			t2.CONFG = 2;
			t2.Data_ID = DSI_GERNERIC_LONG_PACKET_ID;
			t2.WC16 = count + 1;
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
					AS_UINT32(&t2));
			goto_addr =
			(unsigned long)(&DSI_CMDQ_REG[d]->data[1].byte0);
			mask_para =
				(0xFFu << ((goto_addr & 0x3u) * 8));
			set_para =
				(cmd << ((goto_addr & 0x3u) * 8));
			DSI_MASKREG32(cmdq, goto_addr & (~(0x3u)),
					mask_para, set_para);

			for (i = 0; i < count; i++) {
				goto_addr =
					(unsigned long)(
					&DSI_CMDQ_REG[d]->data[1].byte1) + i;
				mask_para =
					(0xFFu <<
					((goto_addr & 0x3u) * 8));
				set_para =
					(para_list[i] <<
					((goto_addr & 0x3u) * 8));
				DSI_MASKREG32(cmdq,
						goto_addr & (~(0x3u)),
						mask_para, set_para);
			}

			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE,
					 2 + (count) / 4);

		} else {
			t0.CONFG = 0;
			t0.Data0 = cmd;
			if (count) {
				t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_2;
				t0.Data1 = para_list[0];
			} else {
				t0.Data_ID = DSI_GERNERIC_SHORT_PACKET_ID_1;
				t0.Data1 = 0;
			}
			DSI_OUTREG32(cmdq, &DSI_CMDQ_REG[d]->data[0],
					 AS_UINT32(&t0));
			DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_CMDQ_SIZE, 1);
		}
	}
	/* start DSI */
	DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 0);
	DSI_OUTREG32(cmdq, &DSI_REG[d]->DSI_START, 1);
	DSI_POLLREG32(cmdq, &DSI_REG[d]->DSI_INTSTA, 0x80000000, 0x0);
	return ret;
}


void *get_dsi_params_handle(UINT32 dsi_idx)
{
	if (dsi_idx != PM_DSI1)
		return (void *)(&_dsi_context[0].dsi_params);
	else
		return (void *)(&_dsi_context[1].dsi_params);
}

INT32 DSI_ssc_enable(UINT32 dsi_index, UINT32 en)
{
	UINT32 disable = en ? 0 : 1;

	if (dsi_index == PM_DSI0) {
		DISP_REG_SET_FIELD(NULL, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[0]+MIPITX_PLL_CON2, en);
		_dsi_context[0].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI1) {
		DISP_REG_SET_FIELD(NULL, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[1]+MIPITX_PLL_CON2, en);
		_dsi_context[1].dsi_params.ssc_disable = disable;
	} else if (dsi_index == PM_DSI_DUAL) {
		DISP_REG_SET_FIELD(NULL, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[0]+MIPITX_PLL_CON2, en);
		DISP_REG_SET_FIELD(NULL, FLD_RG_DSI_PLL_SDM_SSC_EN,
			DSI_PHY_REG[1]+MIPITX_PLL_CON2, en);

		_dsi_context[0].dsi_params.ssc_disable =
			_dsi_context[1].dsi_params.ssc_disable = disable;
	}
	return 0;
}

struct DDP_MODULE_DRIVER ddp_driver_dsi0 = {
	.module = DISP_MODULE_DSI0,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};

struct DDP_MODULE_DRIVER ddp_driver_dsi1 = {
	.module = DISP_MODULE_DSI1,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};

struct DDP_MODULE_DRIVER ddp_driver_dsidual = {
	.module = DISP_MODULE_DSIDUAL,
	.init = ddp_dsi_init,
	.deinit = ddp_dsi_deinit,
	.config = ddp_dsi_config,
	.build_cmdq = ddp_dsi_build_cmdq,
	.trigger = ddp_dsi_trigger,
	.start = ddp_dsi_start,
	.stop = ddp_dsi_stop,
	.reset = ddp_dsi_reset,
	.power_on = ddp_dsi_power_on,
	.power_off = ddp_dsi_power_off,
	.is_idle = ddp_dsi_is_idle,
	.is_busy = ddp_dsi_is_busy,
	.dump_info = ddp_dsi_dump,
	.set_lcm_utils = ddp_dsi_set_lcm_utils,
	.ioctl = ddp_dsi_ioctl
};

const struct LCM_UTIL_FUNCS PM_lcm_utils_dsi0 = {
	.set_reset_pin = lcm_set_reset_pin,
	.udelay = lcm_udelay,
	.mdelay = lcm_mdelay,
	.dsi_set_cmdq = DSI_set_cmdq_wrapper_DSI0,
	.dsi_set_cmdq_V2 = DSI_set_cmdq_V2_Wrapper_DSI0
};


/* /////////////////////// Panel Master ////////////////////////////////// */
UINT32 PanelMaster_get_TE_status(UINT32 dsi_idx)
{
	if (dsi_idx == 0)
		return dsi0_te_enable ? 1 : 0;
	/* else */
		/* return dsi1_te_enable ? 1:0 ; */
	return 0;
}

UINT32 PanelMaster_get_CC(UINT32 dsi_idx)
{
	struct DSI_TXRX_CTRL_REG tmp_reg;

	memset(&tmp_reg, 0, sizeof(struct DSI_TXRX_CTRL_REG));

	if ((dsi_idx == PM_DSI0) || (dsi_idx == PM_DSI_DUAL))
		DSI_READREG32(struct DSI_TXRX_CTRL_REG *, &tmp_reg,
			&DSI_REG[0]->DSI_TXRX_CTRL);
	else if (dsi_idx == PM_DSI1)
		DSI_READREG32(struct DSI_TXRX_CTRL_REG *, &tmp_reg,
			&DSI_REG[1]->DSI_TXRX_CTRL);

	return tmp_reg.HSTX_CKLP_EN ? 1 : 0;
}

void PanelMaster_set_CC(UINT32 dsi_index, UINT32 enable)
{
	DDPMSG("set_cc :%d\n", enable);

	if (dsi_index == PM_DSI0) {
		DSI_OUTREGBIT(NULL, struct DSI_TXRX_CTRL_REG,
			DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	} else if (dsi_index == PM_DSI1) {
		DSI_OUTREGBIT(NULL, struct DSI_TXRX_CTRL_REG,
			DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	} else if (dsi_index == PM_DSI_DUAL) {
		DSI_OUTREGBIT(NULL, struct DSI_TXRX_CTRL_REG,
			DSI_REG[0]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
		DSI_OUTREGBIT(NULL, struct DSI_TXRX_CTRL_REG,
			DSI_REG[1]->DSI_TXRX_CTRL, HSTX_CKLP_EN, enable);
	}
}

void PanelMaster_DSI_set_timing(UINT32 dsi_index, struct MIPI_TIMING timing)
{
	UINT32 hbp_byte;
	struct LCM_DSI_PARAMS *dsi_params;
	int fbconfig_dsiTmpBufBpp = 0;

	if (_dsi_context[dsi_index].dsi_params.data_format.format ==
		LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;

	dsi_params = get_dsi_params_handle(dsi_index);
	switch (timing.type) {
	case LPX:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, LPX,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, LPX,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, LPX,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, LPX,
				timing.value);
		}
		break;
	case HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_PRPR,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_PRPR,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_PRPR,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_PRPR,
				timing.value);
		}
		break;
	case HS_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_ZERO,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_ZERO,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_ZERO,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_ZERO,
				timing.value);
		}
		break;
	case HS_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_TRAIL,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_TRAIL,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[0]->DSI_PHY_TIMECON0, HS_TRAIL,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON0_REG,
				DSI_REG[1]->DSI_PHY_TIMECON0, HS_TRAIL,
				timing.value);
		}
		break;
	case TA_GO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_GO,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_GO,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_GO,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_GO,
				timing.value);
		}
		break;
	case TA_SURE:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_SURE,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_SURE,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_SURE,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_SURE,
				timing.value);
		}
		break;
	case TA_GET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_GET,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_GET,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, TA_GET,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, TA_GET,
				timing.value);
		}
		break;
	case DA_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[0]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON1_REG,
				DSI_REG[1]->DSI_PHY_TIMECON1, DA_HS_EXIT,
				timing.value);
		}
		break;
	case CONT_DET:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CONT_DET,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CONT_DET,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CONT_DET,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CONT_DET,
				timing.value);
		}
		break;
	case CLK_ZERO:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CLK_ZERO,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CLK_ZERO,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CLK_ZERO,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CLK_ZERO,
				timing.value);
		}
		break;
	case CLK_TRAIL:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CLK_TRAIL,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CLK_TRAIL,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[0]->DSI_PHY_TIMECON2, CLK_TRAIL,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON2_REG,
				DSI_REG[1]->DSI_PHY_TIMECON2, CLK_TRAIL,
				timing.value);
		}
		break;
	case CLK_HS_PRPR:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_PRPR,
				timing.value);
		}
		break;
	case CLK_HS_POST:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_POST,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_POST,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_POST,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_POST,
				timing.value);
		}
		break;
	case CLK_HS_EXIT:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[0]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_PHY_TIMCON3_REG,
				DSI_REG[1]->DSI_PHY_TIMECON3, CLK_HS_EXIT,
				timing.value);
		}
		break;
	case HPW:
		if (!(dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE)) {
			timing.value *= fbconfig_dsiTmpBufBpp;
			timing.value -= 10;
		}
		timing.value = ALIGN_TO((timing.value), 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC,
				timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC,
				timing.value);
		} else if (dsi_index == PM_DSI_DUAL) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HSA_WC,
				timing.value);
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HSA_WC,
				timing.value);
		}
		break;
	case HFP:
		timing.value = timing.value * fbconfig_dsiTmpBufBpp - 12;
		timing.value = ALIGN_TO(timing.value, 4);
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_HFP_WC_REG,
				DSI_REG[0]->DSI_HFP_WC, HFP_WC, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_HFP_WC_REG,
				DSI_REG[1]->DSI_HFP_WC, HFP_WC, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, struct DSI_HFP_WC_REG,
				DSI_REG[0]->DSI_HFP_WC, HFP_WC, timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_HFP_WC_REG,
				DSI_REG[1]->DSI_HFP_WC, HFP_WC, timing.value);
		}
		break;
	case HBP:
	{
		if (dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE) {
			hbp_byte = timing.value +
				dsi_params->horizontal_sync_active;
			hbp_byte = hbp_byte * fbconfig_dsiTmpBufBpp - 10;
		} else {
			hbp_byte = timing.value * fbconfig_dsiTmpBufBpp - 10;
		}

		if (dsi_index == PM_DSI0) {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC,
				ALIGN_TO((hbp_byte), 4));
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC,
				ALIGN_TO((hbp_byte), 4));
		} else {
			DSI_OUTREG32(NULL, &DSI_REG[0]->DSI_HBP_WC,
				ALIGN_TO((hbp_byte), 4));
			DSI_OUTREG32(NULL, &DSI_REG[1]->DSI_HBP_WC,
				ALIGN_TO((hbp_byte), 4));
		}

		break;
	}
	case VPW:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_VACT_NL_REG,
				DSI_REG[0]->DSI_VACT_NL, VACT_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_VACT_NL_REG,
				DSI_REG[1]->DSI_VACT_NL, VACT_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, struct DSI_VACT_NL_REG,
				DSI_REG[0]->DSI_VACT_NL, VACT_NL, timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_VACT_NL_REG,
				DSI_REG[1]->DSI_VACT_NL, VACT_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VACT_NL,timing.value); */
		break;
	case VFP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_VFP_NL_REG,
				DSI_REG[0]->DSI_VFP_NL, VFP_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_VFP_NL_REG,
				DSI_REG[1]->DSI_VFP_NL, VFP_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, struct DSI_VFP_NL_REG,
				DSI_REG[0]->DSI_VFP_NL, VFP_NL, timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_VFP_NL_REG,
				DSI_REG[1]->DSI_VFP_NL, VFP_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VFP_NL, timing.value); */
		break;
	case VBP:
		if (dsi_index == PM_DSI0) {
			DSI_OUTREGBIT(NULL, struct DSI_VBP_NL_REG,
				DSI_REG[0]->DSI_VBP_NL, VBP_NL, timing.value);
		} else if (dsi_index == PM_DSI1) {
			DSI_OUTREGBIT(NULL, struct DSI_VBP_NL_REG,
				DSI_REG[1]->DSI_VBP_NL, VBP_NL, timing.value);
		} else {
			DSI_OUTREGBIT(NULL, struct DSI_VBP_NL_REG,
				DSI_REG[0]->DSI_VBP_NL, VBP_NL, timing.value);
			DSI_OUTREGBIT(NULL, struct DSI_VBP_NL_REG,
				DSI_REG[1]->DSI_VBP_NL, VBP_NL, timing.value);
		}
		/* OUTREG32(&DSI_REG->DSI_VBP_NL, timing.value); */
		break;
	case SSC_EN:
		DSI_ssc_enable(dsi_index, timing.value);
		break;
	default:
		DDPMSG("fbconfig dsi set timing :no such type!!\n");
		break;
	}
}

UINT32 PanelMaster_get_dsi_timing(UINT32 dsi_index,
	enum MIPI_SETTING_TYPE type)
{
	UINT32 dsi_val;
	struct DSI_REGS *dsi_reg;
	int fbconfig_dsiTmpBufBpp = 0;

	if (_dsi_context[dsi_index].dsi_params.data_format.format ==
		LCM_DSI_FORMAT_RGB565)
		fbconfig_dsiTmpBufBpp = 2;
	else
		fbconfig_dsiTmpBufBpp = 3;

	if ((dsi_index == PM_DSI0) || (dsi_index == PM_DSI_DUAL))
		dsi_reg = DSI_REG[0];
	else
		dsi_reg = DSI_REG[1];

	switch (type) {
	case LPX:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.LPX;
		return dsi_val;
	case HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_PRPR;
		return dsi_val;
	case HS_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_ZERO;
		return dsi_val;
	case HS_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON0.HS_TRAIL;
		return dsi_val;
	case TA_GO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GO;
		return dsi_val;
	case TA_SURE:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_SURE;
		return dsi_val;
	case TA_GET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.TA_GET;
		return dsi_val;
	case DA_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON1.DA_HS_EXIT;
		return dsi_val;
	case CONT_DET:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CONT_DET;
		return dsi_val;
	case CLK_ZERO:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_ZERO;
		return dsi_val;
	case CLK_TRAIL:
		dsi_val = dsi_reg->DSI_PHY_TIMECON2.CLK_TRAIL;
		return dsi_val;
	case CLK_HS_PRPR:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_PRPR;
		return dsi_val;
	case CLK_HS_POST:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_POST;
		return dsi_val;
	case CLK_HS_EXIT:
		dsi_val = dsi_reg->DSI_PHY_TIMECON3.CLK_HS_EXIT;
		return dsi_val;
	case HPW:
	{
		struct DSI_HSA_WC_REG tmp_reg;

		DSI_READREG32((struct DSI_HSA_WC_REG *), &tmp_reg,
			&dsi_reg->DSI_HSA_WC);
		dsi_val = (tmp_reg.HSA_WC + 10) / fbconfig_dsiTmpBufBpp;
		return dsi_val;
	}
	case HFP:
	{
		struct DSI_HFP_WC_REG tmp_hfp;

		DSI_READREG32((struct DSI_HFP_WC_REG *), &tmp_hfp,
			&dsi_reg->DSI_HFP_WC);
		dsi_val = ((tmp_hfp.HFP_WC + 12) / fbconfig_dsiTmpBufBpp);
		return dsi_val;
	}
	case HBP:
	{
		struct DSI_HBP_WC_REG tmp_hbp;
		struct LCM_DSI_PARAMS *dsi_params;

		dsi_params = get_dsi_params_handle(dsi_index);
		OUTREG32(&tmp_hbp, AS_UINT32(&dsi_reg->DSI_HBP_WC));
		if (dsi_params->mode == SYNC_EVENT_VDO_MODE ||
			dsi_params->mode == BURST_VDO_MODE)
			return (tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp -
				dsi_params->horizontal_sync_active;
		else
			return (tmp_hbp.HBP_WC + 10) / fbconfig_dsiTmpBufBpp;
	}
	case VPW:
	{
		struct DSI_VACT_NL_REG tmp_vpw;

		DSI_READREG32((struct DSI_VACT_NL_REG *),
			&tmp_vpw, &dsi_reg->DSI_VACT_NL);
		dsi_val = tmp_vpw.VACT_NL;
		return dsi_val;
	}
	case VFP:
	{
		struct DSI_VFP_NL_REG tmp_vfp;

		DSI_READREG32((struct DSI_VFP_NL_REG *),
			&tmp_vfp, &dsi_reg->DSI_VFP_NL);
		dsi_val = tmp_vfp.VFP_NL;
		return dsi_val;
	}
	case VBP:
	{
		struct DSI_VBP_NL_REG tmp_vbp;

		DSI_READREG32((struct DSI_VBP_NL_REG *),
			&tmp_vbp, &dsi_reg->DSI_VBP_NL);
		dsi_val = tmp_vbp.VBP_NL;
		return dsi_val;
	}
	case SSC_EN:
	{
		if (_dsi_context[dsi_index].dsi_params.ssc_disable)
			dsi_val = 0;
		else
			dsi_val = 1;
		return dsi_val;
	}
	default:
		DDPMSG("fbconfig dsi set timing :no such type!!\n");
		break;
	}

	dsi_val = 0;
	return dsi_val;
}

unsigned int PanelMaster_is_enable(void)
{
	if (atomic_read(&PMaster_enable) == 1)
		return 1;
	else
		return 0;
}

unsigned int PanelMaster_set_PM_enable(unsigned int value)
{
	atomic_set(&PMaster_enable, value);
	return 0;
}

/* ///////////////////////////////No DSI Driver //////////////////////////// */
int DSI_set_roi(int x, int y)
{
	DDPMSG("[DSI](x0,y0,x1,y1)=(%d,%d,%d,%d)\n",
		x, y, _dsi_context[0].lcm_width, _dsi_context[0].lcm_height);
	return DSI_Send_ROI(DISP_MODULE_DSI0, NULL, x, y,
		_dsi_context[0].lcm_width - x, _dsi_context[0].lcm_height - y);
}

int DSI_check_roi(void)
{
	int ret = 0;
	unsigned char read_buf[10] = { 1, 1, 1, 1 };
	unsigned int data_array[16];
	int count;
	int x0;
	int y0;

	data_array[0] = 0x00043700; /* read id return two byte,version and id */
	DSI_set_cmdq(DISP_MODULE_DSI0, NULL, data_array, 1, 1);
	msleep(20);
	count = DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL,
		0x2a, read_buf, 4);
	msleep(20);
	x0 = (read_buf[0] << 8) | read_buf[1];
	DDPMSG("x0=%d count=%d,read:buf[0]=%d,buf[1]=%d,buf[2]=%d,buf[3]=%d\n",
	       x0, count, read_buf[0], read_buf[1], read_buf[2],
	       read_buf[3]);
	if ((count == 0) || (x0 != 0)) {
		DDPMSG("[DSI]x count %d read:buf[0]=%d,buf[1]=%d\n",
		       count, read_buf[0], read_buf[1]);
		DDPMSG("[DSI]x count %d read:buf[2]=%d,buf[3]=%d\n",
		       count, read_buf[2], read_buf[3]);
		return -1;
	}
	msleep(20);
	count = DSI_dcs_read_lcm_reg_v2(DISP_MODULE_DSI0, NULL,
		0x2b, read_buf, 4);
	y0 = (read_buf[0] << 8) | read_buf[1];
	DDPMSG("y0=%d count %d,read:buf[0]=%d,buf[1]=%d,buf[2]=%d,buf[3]=%d\n",
	       y0, count, read_buf[0], read_buf[1], read_buf[2],
	       read_buf[3]);
	if ((count == 0) || (y0 != 0)) {
		DDPMSG("[DSI]y count %d read:buf[0]=%d,buf[1]=%d\n",
		       count, read_buf[0], read_buf[1]);
		DDPMSG("[DSI]y count %d read:buf[2]=%d,buf[3]=%d\n",
		       count, read_buf[2], read_buf[3]);
		return -1;
	}
	return ret;
}

void DSI_ForceConfig(int forceconfig)
{
	if (!disp_helper_get_option(DISP_OPT_CV_BYSUSPEND))
		return;

	if (lcm_mode_status == 0)
		return;

	dsi_force_config = forceconfig;
	/* cv switch by resume */
	if (_dsi_context[0].dsi_params.PLL_CK_CMD == 0)
		_dsi_context[0].dsi_params.PLL_CK_CMD =
			_dsi_context[0].dsi_params.PLL_CLOCK;
	if (_dsi_context[0].dsi_params.PLL_CK_VDO == 0)
		_dsi_context[0].dsi_params.PLL_CK_VDO =
			_dsi_context[0].dsi_params.PLL_CLOCK;
	if (lcm_dsi_mode == CMD_MODE)
		_dsi_context[0].dsi_params.PLL_CLOCK =
			_dsi_context[0].dsi_params.PLL_CK_CMD;
	else if (lcm_dsi_mode == SYNC_PULSE_VDO_MODE ||
		lcm_dsi_mode == SYNC_EVENT_VDO_MODE ||
		 lcm_dsi_mode == BURST_VDO_MODE)
		_dsi_context[0].dsi_params.PLL_CLOCK =
			_dsi_context[0].dsi_params.PLL_CK_VDO;
}
#ifdef CONFIG_MTK_HIGH_FRAME_RATE
/*-------------------------------DynFPS start------------------------------*/
unsigned int ddp_dsi_fps_change_index(
	unsigned int last_dynfps, unsigned int new_dynfps)
{
	struct LCM_DSI_PARAMS *dsi = NULL;
	struct dfps_info *dfps_params_last = NULL;
	struct dfps_info *dfps_params_new = NULL;
	unsigned int i = 0;
	unsigned int fps_chg_index = 0;

	dsi = &_dsi_context[0].dsi_params;

	for (i = 0; i < dsi->dfps_num; i++) {
		if ((dsi->dfps_params)[i].fps == last_dynfps)
			dfps_params_last = &((dsi->dfps_params)[i]);
		if ((dsi->dfps_params)[i].fps == new_dynfps)
			dfps_params_new = &((dsi->dfps_params)[i]);
	}

	if (dfps_params_last == NULL ||
		dfps_params_new == NULL)
		return 0;

	if (!mipi_clk_change_sta) {
		if (dfps_params_last->vertical_frontporch !=
			dfps_params_new->vertical_frontporch) {
			fps_chg_index |= DYNFPS_DSI_VFP;
		}
		if (dfps_params_last->horizontal_frontporch !=
			dfps_params_new->horizontal_frontporch) {
			fps_chg_index |= DYNFPS_DSI_HFP;
		}
		if (dfps_params_last->PLL_CLOCK !=
			dfps_params_new->PLL_CLOCK) {
			fps_chg_index |= DYNFPS_DSI_MIPI_CLK;
		}
		if (dfps_params_last->data_rate !=
			dfps_params_new->data_rate) {
			fps_chg_index |= DYNFPS_DSI_MIPI_CLK;
		}
	} else {
		if (dfps_params_last->vertical_frontporch_dyn !=
			dfps_params_new->vertical_frontporch_dyn) {
			fps_chg_index |= DYNFPS_DSI_VFP;
		}
		if (dfps_params_last->horizontal_frontporch_dyn !=
			dfps_params_new->horizontal_frontporch_dyn) {
			fps_chg_index |= DYNFPS_DSI_HFP;
		}
		if (dfps_params_last->PLL_CLOCK_dyn !=
			dfps_params_new->PLL_CLOCK_dyn) {
			fps_chg_index |= DYNFPS_DSI_MIPI_CLK;
		}
		if (dfps_params_last->data_rate_dyn !=
			dfps_params_new->data_rate_dyn) {
			fps_chg_index |= DYNFPS_DSI_MIPI_CLK;
		}
	}
	DDPMSG("%s,chg %d->%d\n", __func__, last_dynfps, new_dynfps);
	DDPMSG("%s,chg solution:0x%x\n", __func__, fps_chg_index);
	return fps_chg_index;
}

void ddp_dsi_dynfps_chg_fps(
	enum DISP_MODULE_ENUM module, void *handle,
	unsigned int last_fps, unsigned int new_fps, unsigned int chg_index)
{
	struct LCM_DSI_PARAMS *dsi = NULL;
	struct dfps_info *dfps_params_last = NULL;
	struct dfps_info *dfps_params_new = NULL;
	unsigned int i = 0;

	dsi = &_dsi_context[0].dsi_params;

	for (i = 0; i < dsi->dfps_num; i++) {
		if ((dsi->dfps_params)[i].fps == last_fps)
			dfps_params_last = &((dsi->dfps_params)[i]);
		if ((dsi->dfps_params)[i].fps == new_fps)
			dfps_params_new = &((dsi->dfps_params)[i]);
	}
	if (dfps_params_last == NULL ||
		dfps_params_new == NULL)
		return;

	DDPMSG("%s,fps %d->%d\n", __func__, last_fps, new_fps);
	DDPMSG("%s,chg_index=0x%x\n", __func__, chg_index);
	/*we will not change dsi_params
	 *will use this disp_fps to choose right params
	 */
	_dsi_context[i].disp_fps = new_fps;
	_dsi_context[i].dynfps_chg_index = chg_index;

	for (i = DSI_MODULE_BEGIN(module); i <= DSI_MODULE_END(module); i++) {
		if (chg_index & DYNFPS_DSI_MIPI_CLK) {
			DDPMSG("%s, change MIPI Clock\n", __func__);
		} else if (chg_index & DYNFPS_DSI_HFP) {
			DDPMSG("%s, change HFP\n", __func__);

			DSI_PHY_TIMCONFIG(module, handle, dsi);
			DSI_Calc_VDO_Timing(module, dsi);

			ddp_dsi_porch_setting(module, handle, DSI_HFP,
					_dsi_context[i].hfp_byte);

		} else if (chg_index & DYNFPS_DSI_VFP) {
			DDPMSG("%s, change VFP\n", __func__);
			if (!mipi_clk_change_sta)
				_dsi_context[i].vfp =
				dfps_params_new->vertical_frontporch;
			else
				_dsi_context[i].vfp =
				dfps_params_new->vertical_frontporch_dyn;

			ddp_dsi_porch_setting(module, handle, DSI_VFP,
						_dsi_context[i].vfp);
		}

	}

}
void ddp_dsi_dynfps_get_vfp_info(unsigned int disp_fps,
	unsigned int *vfp, unsigned int *vfp_for_lp)
{
	struct LCM_DSI_PARAMS *dsi = NULL;
	struct dfps_info *dfps_params = NULL;
	unsigned int i = 0;
	unsigned int _vfp = 0;
	unsigned int _vfp_for_lp = 0;

	dsi = &_dsi_context[0].dsi_params;
	_vfp = dsi->vertical_frontporch;
	_vfp_for_lp = dsi->vertical_frontporch_for_low_power;

	for (i = 0; i < dsi->dfps_num; i++) {
		if ((dsi->dfps_params)[i].fps == disp_fps)
			dfps_params = &((dsi->dfps_params)[i]);
	}

	if (dfps_params == NULL) {
		if (vfp)
			*vfp = _vfp;
		if (vfp_for_lp)
			*vfp_for_lp = _vfp_for_lp;
		return;
	}

	if (!mipi_clk_change_sta) {
		_vfp = dfps_params->vertical_frontporch ?
				dfps_params->vertical_frontporch :
				dsi->vertical_frontporch;

		_vfp_for_lp =
			dfps_params->vertical_frontporch_for_low_power ?
			dfps_params->vertical_frontporch_for_low_power :
			dsi->vertical_frontporch_for_low_power;

	} else {
		_vfp = dfps_params->vertical_frontporch_dyn ?
				dfps_params->vertical_frontporch_dyn :
				dsi->vertical_frontporch_dyn;

		_vfp_for_lp =
			dfps_params->vertical_frontporch_for_low_power_dyn ?
			dfps_params->vertical_frontporch_for_low_power_dyn :
			dsi->vertical_frontporch_for_low_power;
		/*ToDo, default mipi hopping
		 *whether need add vfp_for_lp_dyn param
		 */
	}
	DDPMSG("%s,fps=%d,vfp:%d,vfp_for_lp:%d]\n",
		__func__, disp_fps, _vfp, _vfp_for_lp);

	if (vfp)
		*vfp = _vfp;
	if (vfp_for_lp)
		*vfp_for_lp = _vfp_for_lp;

	return;

}
/*-------------------------------DynFPS end------------------------------*/
#endif
