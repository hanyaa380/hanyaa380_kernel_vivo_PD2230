/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _PRIMARY_DISPLAY_H_
#define _PRIMARY_DISPLAY_H_

#include "ddp_hal.h"
#include "ddp_manager.h"
#include <linux/types.h>
#include "disp_session.h"
#include "disp_lcm.h"
#include "disp_helper.h"
#ifdef MTK_FB_MMDVFS_SUPPORT
#include <linux/soc/mediatek/mtk-pm-qos.h>
#endif
#include "mt-plat/mtk_smi.h"
#include "mtk_smi.h"


#ifdef MTK_FB_MMDVFS_SUPPORT
extern struct mtk_pm_qos_request primary_display_qos_request;
extern struct mtk_pm_qos_request primary_display_emi_opp_request;
extern struct mtk_pm_qos_request primary_display_mm_freq_request;
#endif

enum DISP_PRIMARY_PATH_MODE {
	DIRECT_LINK_MODE,
	DECOUPLE_MODE,
	SINGLE_LAYER_MODE,
	DEBUG_RDMA1_DSI0_MODE
};

/*enum MTK_SMI_BWC_SCEN {
	SMI_BWC_SCEN_NORMAL,
	SMI_BWC_SCEN_UI_IDLE,
	SMI_BWC_SCEN_VPMJC,
	SMI_BWC_SCEN_FORCE_MMDVFS,
	SMI_BWC_SCEN_HDMI,
	SMI_BWC_SCEN_HDMI4K,
	SMI_BWC_SCEN_WFD,
	SMI_BWC_SCEN_VENC,
	SMI_BWC_SCEN_SWDEC_VP,
	SMI_BWC_SCEN_VP,
	SMI_BWC_SCEN_VP_HIGH_FPS,
	SMI_BWC_SCEN_VP_HIGH_RESOLUTION,
	SMI_BWC_SCEN_VR,
	SMI_BWC_SCEN_VR_SLOW,
	SMI_BWC_SCEN_VSS,
	SMI_BWC_SCEN_CAM_PV,
	SMI_BWC_SCEN_CAM_CP,
	SMI_BWC_SCEN_ICFP,
	SMI_BWC_SCEN_MM_GPU,
	SMI_BWC_SCEN_CNT
};

enum {
	MMDVFS_CAM_MON_SCEN = SMI_BWC_SCEN_CNT, MMDVFS_SCEN_MHL,
	MMDVFS_SCEN_MJC, MMDVFS_SCEN_DISP, MMDVFS_SCEN_ISP,
	MMDVFS_SCEN_VP_HIGH_RESOLUTION, MMDVFS_SCEN_VPU, MMDVFS_MGR,
	MMDVFS_SCEN_VPU_KERNEL, MMDVFS_PMQOS_ISP,
	MMDVFS_SCEN_VP_WFD, MMDVFS_SCEN_COUNT
};
*/

#define UINT8 unsigned char
#define UINT32 unsigned int

#ifndef TRUE
	#define TRUE	(1)
#endif

#ifndef FALSE
	#define FALSE	(0)
#endif

#define ALIGN_TO(x, n)	(((x) + ((n) - 1)) & ~((n) - 1))

#define ASSERT_LAYER    (DDP_OVL_LAYER_MUN-1)
extern unsigned int FB_LAYER;	/* default LCD layer */
#define DISP_DEFAULT_UI_LAYER_ID (DDP_OVL_LAYER_MUN-1)
#define DISP_CHANGED_UI_LAYER_ID (DDP_OVL_LAYER_MUN-2)

#define pgc	_get_context()

extern unsigned int ap_fps_changed;
extern unsigned int arr_fps_backup;
extern unsigned int arr_fps_enable;
extern unsigned int round_corner_offset_enable;
extern bool g_force_cfg;
extern unsigned int g_force_cfg_id;

struct DISP_LAYER_INFO {
	unsigned int id;
	unsigned int curr_en;
	unsigned int next_en;
	unsigned int hw_en;
	int curr_idx;
	int next_idx;
	int hw_idx;
	int curr_identity;
	int next_identity;
	int hw_identity;
	int curr_conn_type;
	int next_conn_type;
	int hw_conn_type;
};

enum reset_and_power_ctrl_tp_type {
	POWER_CTRL_AVDDOFF_AVEEOFF_RST_LOW,
	POWER_CTRL_AVDDON_AVEEON_RST_HIGH,
	POWER_CTRL_AVDDOFF_AVEEOFF_RST_HIGH,
	POWER_CTRL_AVDD_ON,
	POWER_CTRL_AVDD_OFF,
	POWER_CTRL_VDDR_OFF,
	POWER_CTRL_DISP_PANEL_POWER_ON_SEQ,
	POWER_CTRL_DISP_PANEL_POWER_OFF_SEQ,
	POWER_CTRL_DISP_PANEL_POWER_SUHTDOWN_SEQ,
	POWER_CTRL_UNKNOWN
};

enum DISP_STATUS {
	DISP_STATUS_OK = 0,
	DISP_STATUS_NOT_IMPLEMENTED,
	DISP_STATUS_ALREADY_SET,
	DISP_STATUS_ERROR,
};

#if 0
enum DISP_STATE {
	DISP_STATE_IDLE = 0,
	DISP_STATE_BUSY,
};

enum DISP_OP_STATE {
	DISP_OP_PRE = 0,
	DISP_OP_NORMAL,
	DISP_OP_POST,
};
#endif

enum DISP_POWER_STATE {
	DISP_ALIVE = 0xf0,
	DISP_SLEPT,
	DISP_BLANK
};

enum DISP_FRM_SEQ_STATE {
	FRM_CONFIG = 0,
	FRM_TRIGGER,
	FRM_START,
	FRM_END
};

/*add for detect wrong bias devices*/
enum bl_bias_ic_smt_detect_type {
	BL_BIAS_IC_SMT_DETECT_SUCESS,
	BL_IC_SMT_DETECT_FAIL,
	BIAS_IC_SMT_DETECT_FAIL = 0x10,
};

#if 0
enum DISPLAY_HAL_IOCTL {
	DISPLAY_HAL_IOCTL_SET_CMDQ = 0xff00,
	DISPLAY_HAL_IOCTL_ENABLE_CMDQ,
	DISPLAY_HAL_IOCTL_DUMP,
	DISPLAY_HAL_IOCTL_PATTERN,
};
#endif

struct primary_disp_input_config {
	unsigned int layer;
	unsigned int layer_en;
	unsigned int buffer_source;
	unsigned int fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int src_x;
	unsigned int src_y;
	unsigned int src_w;
	unsigned int src_h;
	unsigned int src_pitch;
	unsigned int dst_x;
	unsigned int dst_y;
	unsigned int dst_w;
	unsigned int dst_h;	/* clip region */
	unsigned int keyEn;
	unsigned int key;
	unsigned int aen;
	unsigned char alpha;

	unsigned int sur_aen;
	unsigned int src_alpha;
	unsigned int dst_alpha;
	unsigned int isTdshp;
	unsigned int isDirty;

	unsigned int buff_idx;
	unsigned int identity;
	unsigned int connected_type;
	enum DISP_BUFFER_TYPE security;
	unsigned int dirty;
	unsigned int yuv_range;
};

struct disp_mem_output_config {
	enum UNIFIED_COLOR_FMT fmt;
	unsigned long addr;
	unsigned long addr_sub_u;
	unsigned long addr_sub_v;
	unsigned long vaddr;
	unsigned int x;
	unsigned int y;
	unsigned int w;
	unsigned int h;
	unsigned int pitch;
	unsigned int pitchUV;

	unsigned int buff_idx;
	unsigned int interface_idx;
	enum DISP_BUFFER_TYPE security;
	unsigned int dirty;
	int mode;

	/* night light setting */
	struct disp_ccorr_config m_ccorr_config;
};

#define DISP_INTERNAL_BUFFER_COUNT 3

struct disp_internal_buffer_info {
	struct list_head list;
	struct ion_handle *handle;
	struct sync_fence *pfence;
	void *va;
	uint32_t fence_id;
	uint32_t mva;
	uint32_t size;
	uint32_t output_fence_id;
	uint32_t interface_fence_id;
	unsigned long long timestamp;
};

struct disp_frm_seq_info {
	unsigned int mva;
	unsigned int max_offset;
	unsigned int seq;
	enum DISP_FRM_SEQ_STATE state;
};

struct OPT_BACKUP {
	enum DISP_HELPER_OPT option;
	int value;
};

#define LCM_FPS_ARRAY_SIZE	32
struct lcm_fps_ctx_t {
	int is_inited;
	struct mutex lock;
	unsigned int dsi_mode;
	unsigned int head_idx;
	unsigned int num;
	unsigned long long last_ns;
	unsigned long long array[LCM_FPS_ARRAY_SIZE];
};
/* AOD */
enum lcm_power_state {
	LCM_OFF = 0,
	LCM_ON,
	LCM_ON_LOW_POWER,
	LCM_POWER_STATE_UNKNOWN
};

enum mtkfb_power_mode {
	FB_SUSPEND = 0,
	FB_RESUME,
	DOZE_SUSPEND,
	DOZE,
	MTKFB_POWER_MODE_UNKNOWN,
};

struct display_primary_path_context {
	enum DISP_POWER_STATE state;
	unsigned int lcm_fps;
	unsigned int dynamic_fps;
	int lcm_refresh_rate;
	int max_layer;
	int need_trigger_overlay;
	int need_trigger_ovl1to2;
	int need_trigger_dcMirror_out;
	enum DISP_PRIMARY_PATH_MODE mode;
	unsigned int session_id;
	int session_mode;
	int ovl1to2_mode;
	unsigned int last_vsync_tick;
	unsigned long framebuffer_mva;
	unsigned long framebuffer_va;
	unsigned long framebuffer_pa;
	struct mutex lock;
	struct mutex capture_lock;
	struct mutex switch_dst_lock;
	struct disp_lcm_handle *plcm;
	struct cmdqRecStruct *cmdq_handle_config_esd;
	struct cmdqRecStruct *cmdq_handle_config;
	disp_path_handle dpmgr_handle;
	disp_path_handle ovl2mem_path_handle;
	struct cmdqRecStruct *cmdq_handle_ovl1to2_config;
	struct cmdqRecStruct *cmdq_handle_trigger;
	char *mutex_locker;
	int vsync_drop;
	unsigned int dc_buf_id;
	unsigned int dc_buf[DISP_INTERNAL_BUFFER_COUNT];
	unsigned int force_fps_keep_count;
	unsigned int force_fps_skip_count;
	cmdqBackupSlotHandle cur_config_fence;
	cmdqBackupSlotHandle subtractor_when_free;
	cmdqBackupSlotHandle rdma_buff_info;
	cmdqBackupSlotHandle ovl_status_info;
	cmdqBackupSlotHandle ovl_dummy_info;
	cmdqBackupSlotHandle ovl_config_time;
	cmdqBackupSlotHandle dither_status_info;
	cmdqBackupSlotHandle dsi_vfp_line;
	cmdqBackupSlotHandle dsi_vfp_changed;
	cmdqBackupSlotHandle next_working_fps;
	cmdqBackupSlotHandle night_light_params;
	cmdqBackupSlotHandle hrt_idx_id;
	cmdqBackupSlotHandle trigger_record_slot;

	int is_primary_sec;
	int primary_display_scenario;
#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
	int request_fps;
#endif
	enum mtkfb_power_mode pm;
	enum lcm_power_state lcm_ps;
	/*DynFPS start*/
	unsigned int current_disp_fps;
	unsigned int req_new_disp_fps;
	bool dynfps_update_hrt;
	bool dynfps_update_mmdvfs;
	bool dynfps_update_golden;
	int active_cfg;
	cmdqBackupSlotHandle config_id_slot;
	unsigned int first_cfg;
	struct mutex dynfps_lock;
	struct multi_configs multi_cfg_table;
	/*DynFPS end*/
};

static inline char *lcm_power_state_to_string(enum lcm_power_state ps)
{
	switch (ps) {
	case LCM_OFF:
		return "LCM_OFF";
	case LCM_ON:
		return "LCM_ON";
	case LCM_ON_LOW_POWER:
		return "LCM_ON_LOW_POWER";
	case LCM_POWER_STATE_UNKNOWN:
		return "LCM_POWER_STATE_UNKNOWN";
	}

	return "LCM_POWER_STATE_UNKNOWN";
}

static inline char *power_mode_to_string(enum mtkfb_power_mode pm)
{
	switch (pm) {
	case FB_SUSPEND:
		return "FB_SUSPEND";
	case FB_RESUME:
		return "FB_RESUME";
	case DOZE_SUSPEND:
		return "DOZE_SUSPEND";
	case DOZE:
		return "DOZE";
	case MTKFB_POWER_MODE_UNKNOWN:
		return "MTKFB_POWER_MODE_UNKNOWN";
	}

	return "MTKFB_POWER_MODE_UNKNOWN";
}

typedef int (*PRIMARY_DISPLAY_CALLBACK) (unsigned int user_data);

struct display_primary_path_context *_get_context(void);
void _primary_path_lock(const char *caller);
void _primary_path_unlock(const char *caller);
int primary_display_init(char *lcm_name, unsigned int lcm_fps,
	int is_lcm_inited);
int primary_display_config(unsigned long pa, unsigned long mva);
int primary_display_set_frame_buffer_address(unsigned long va,
	unsigned long mva, unsigned long pa);
unsigned long primary_display_get_frame_buffer_mva_address(void);
unsigned long primary_display_get_frame_buffer_va_address(void);
int primary_display_suspend(void);
int primary_display_resume(void);
int primary_display_ipoh_restore(void);
int primary_display_get_width(void);
int primary_display_get_height(void);
int primary_display_get_virtual_width(void);
int primary_display_get_virtual_height(void);
int primary_display_get_bpp(void);
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
int primary_display_get_corner_full_content(void);
int primary_display_get_corner_pattern_height_bot(void);
int primary_display_get_lcm_corner_en(void);
int primary_display_get_corner_pattern_width(void);
int primary_display_get_corner_pattern_height(void);
void *primary_display_get_corner_pattern_top_va(void);
void *primary_display_get_corner_pattern_bottom_va(void);
#endif
int primary_display_get_pages(void);
int primary_display_set_overlay_layer(struct primary_disp_input_config *input);
int primary_display_is_alive(void);
int primary_display_is_sleepd(void);
int primary_display_wait_for_vsync(void *config);
unsigned int primary_display_get_ticket(void);
int primary_display_user_cmd(unsigned int cmd, unsigned long arg);
int primary_display_trigger(int blocking, void *callback, int need_merge);
int primary_display_switch_mode(int sess_mode, unsigned int session,
	int force);
int primary_display_switch_mode_blocked(int sess_mode, unsigned int session,
	int force);
int primary_display_diagnose(void);

int primary_display_get_info(struct disp_session_info *info);
int primary_display_capture_framebuffer(unsigned long pbuf);
int primary_display_is_video_mode(void);
int primary_is_sec(void);
int do_primary_display_switch_mode(int sess_mode, unsigned int session,
	int need_lock,	struct cmdqRecStruct *handle, int block);
enum DISP_MODE primary_get_sess_mode(void);
unsigned int primary_get_sess_id(void);
enum DISP_POWER_STATE primary_get_state(void);
struct disp_lcm_handle *primary_get_lcm(void);
void *primary_get_dpmgr_handle(void);
void _cmdq_stop_trigger_loop(void);
void _cmdq_start_trigger_loop(void);
void *primary_get_ovl2mem_handle(void);
int primary_display_is_decouple_mode(void);
int primary_display_is_mirror_mode(void);
unsigned int primary_display_get_option(const char *option);
enum CMDQ_SWITCH primary_display_cmdq_enabled(void);
int primary_display_switch_cmdq_cpu(enum CMDQ_SWITCH use_cmdq);
int primary_display_manual_lock(void);
int primary_display_manual_unlock(void);
int primary_display_start(void);
int primary_display_stop(void);
int primary_display_esd_recovery(void);
int primary_display_get_debug_state(char *stringbuf, int buf_len);
void primary_display_set_max_layer(int maxlayer);
void primary_display_reset(void);
void primary_display_esd_check_enable(int enable);
int primary_display_config_input_multiple(
	struct disp_session_input_config *session_input);
int primary_display_frame_cfg(struct disp_frame_cfg_t *cfg);
unsigned int primary_display_force_get_vsync_fps(void);
int primary_display_force_set_vsync_fps(unsigned int fps,
	unsigned int scenario);
unsigned int primary_display_get_fps(void);
unsigned int primary_display_get_fps_nolock(void);
int primary_display_get_original_width(void);
int primary_display_get_original_height(void);
int primary_display_lcm_ATA(void);
int primary_display_setbacklight(unsigned int level);
int primary_display_pause(PRIMARY_DISPLAY_CALLBACK callback,
	unsigned int user_data);
int primary_display_switch_dst_mode(int mode);
int primary_display_get_lcm_index(void);
int primary_display_force_set_fps(unsigned int keep, unsigned int skip);
int primary_display_set_fps(int fps);
int primary_display_get_lcm_max_refresh_rate(void);
int primary_display_set_lcm_refresh_rate(int fps);
int primary_display_get_lcm_refresh_rate(void);
int _display_set_lcm_refresh_rate(int fps);
void primary_display_idlemgr_kick(const char *source, int need_lock);
void primary_display_idlemgr_enter_idle(int need_lock);
void primary_display_update_present_fence(struct cmdqRecStruct *cmdq_handle,
	unsigned int fence_idx);
void primary_display_wakeup_pf_thread(void);
void primary_display_switch_esd_mode(int mode);
int primary_display_cmdq_set_reg(unsigned int addr, unsigned int val);
int primary_display_vsync_switch(int method);
int primary_display_setlcm_cmd(unsigned int *lcm_cmd, unsigned int *lcm_count,
	unsigned int *lcm_value);
int primary_display_mipi_clk_change(unsigned int clk_value);

void _cmdq_insert_wait_frame_done_token_mira(void *handle);
int primary_display_get_max_layer(void);
long primary_display_wait_state(enum DISP_POWER_STATE state, long timeout);
long primary_display_wait_not_state(enum DISP_POWER_STATE state, long timeout);
int do_primary_display_switch_mode(int sess_mode, unsigned int session,
	int need_lock, struct cmdqRecStruct *handle, int block);
int primary_display_check_test(void);
void _primary_path_switch_dst_lock(void);
void _primary_path_switch_dst_unlock(void);

/* AOD */
enum lcm_power_state primary_display_set_power_state(
enum lcm_power_state new_state);
enum lcm_power_state primary_display_get_lcm_power_state(void);
enum mtkfb_power_mode primary_display_set_power_mode(
	enum mtkfb_power_mode new_mode);
enum mtkfb_power_mode primary_display_get_power_mode(void);
enum mtkfb_power_mode primary_display_check_power_mode(void);
void debug_print_power_mode_check(enum mtkfb_power_mode prev,
	enum mtkfb_power_mode cur);
bool primary_is_aod_supported(void);

/* legancy */
struct LCM_PARAMS *DISP_GetLcmPara(void);
struct LCM_DRIVER *DISP_GetLcmDrv(void);
UINT32 DISP_GetVRamSize(void);
UINT32 DISP_GetFBRamSize(void);
UINT32 DISP_GetPages(void);
UINT32 DISP_GetScreenBpp(void);
UINT32 DISP_GetScreenWidth(void);
UINT32 DISP_GetScreenHeight(void);
UINT32 DISP_GetActiveHeight(void);
UINT32 DISP_GetActiveWidth(void);
UINT32 DISP_GetActiveHeightUm(void);
UINT32 DISP_GetActiveWidthUm(void);
UINT32 DISP_GetDensity(void);
unsigned long get_dim_layer_mva_addr(void);
int disp_hal_allocate_framebuffer(phys_addr_t pa_start, phys_addr_t pa_end,
	unsigned long *va, unsigned long *mva);
int Panel_Master_dsi_config_entry(const char *name, void *config_value);
int fbconfig_get_esd_check_test(UINT32 dsi_id, UINT32 cmd, UINT8 *buffer,
	UINT32 num);

/* 0: normal, 1: lcd only, 2: none of lcd and lcm */
extern unsigned int gTriggerDispMode;

/* defined in mtkfb.c should move to mtkfb.h*/
extern unsigned int islcmconnected;

size_t mtkfb_get_fb_size(void);

int primary_fps_ctx_set_wnd_sz(unsigned int wnd_sz);
int primary_fps_ctx_get_fps(unsigned int *fps, int *stable);
int primary_fps_ext_ctx_set_interval(unsigned int interval);

int dynamic_debug_msg_print(unsigned int mva, int w, int h, int pitch,
	int bytes_per_pix);

int display_enter_tui(void);
int display_exit_tui(void);

int primary_display_config_full_roi(struct disp_ddp_path_config *pconfig,
	disp_path_handle disp_handle,
		struct cmdqRecStruct *cmdq_handle);
int primary_display_set_scenario(int scenario);
enum DISP_MODULE_ENUM _get_dst_module_by_lcm(struct disp_lcm_handle *plcm);
extern void check_mm0_clk_sts(void);
int primary_display_is_directlink_mode(void);
#ifdef MTK_FB_MMDVFS_SUPPORT
int primary_display_get_dvfs_last_req(void);
#endif

extern struct lcm_fps_ctx_t lcm_fps_ctx;
int lcm_fps_ctx_init(struct lcm_fps_ctx_t *fps_ctx);
int lcm_fps_ctx_reset(struct lcm_fps_ctx_t *fps_ctx);
int lcm_fps_ctx_update(struct lcm_fps_ctx_t *fps_ctx,
		unsigned long long cur_ns);

unsigned int primary_display_get_current_disp_fps(void);
int primary_display_get_dvfs_last_req(void);
/**************function for DynFPS start************************/
unsigned int primary_display_is_support_DynFPS(void);
unsigned int primary_display_get_default_disp_fps(int need_lock);
unsigned int primary_display_get_def_timing_fps(int need_lock);
int primary_display_get_cfg_fps(
	int config_id, unsigned int *fps, unsigned int *vact_timing_fps);
unsigned int primary_display_get_current_cfg_id(void);
void primary_display_update_cfg_id(int cfg_id);
void primary_display_init_multi_cfg_info(void);
int primary_display_get_multi_configs(struct multi_configs *p_cfgs);
void primary_display_dynfps_chg_fps(int cfg_id);
unsigned int primary_display_is_support_ARR(void);
unsigned int primary_display_get_vfp(unsigned int fps);
unsigned int primary_display_get_current_cfg_id(void);

void primary_display_dynfps_get_vfp_info(
	unsigned int *vfp, unsigned int *vfp_for_lp);

extern unsigned int dump_output;
extern unsigned int dump_output_comp;
extern void *composed_buf;
extern struct completion dump_buf_comp;
#endif
