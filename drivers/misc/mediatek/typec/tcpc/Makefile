subdir-ccflags-y := -Wall $(call cc-disable-warning, unused-but-set-variable)
subdir-ccflags-y += -I$(srctree)/drivers/power/supply
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/usb_boost/
obj-$(CONFIG_TCPC_MT6362)		+= tcpc_mt6362.o
obj-$(CONFIG_TCPC_MT6370)		+= tcpc_mt6370.o
obj-$(CONFIG_TCPC_RT1711H)		+= tcpc_rt1711h.o
obj-$(CONFIG_TCPC_SGM7220)		+= tcpc_sgm7220.o
obj-$(CONFIG_TCPC_WUSB3801)		+= tcpc_wusb3801.o
obj-$(CONFIG_TCPC_CLASS)		+= tcpci_core.o tcpci_typec.o tcpci_timer.o \
		tcpm.o tcpci.o pd_dbg_info.o tcpci_alert.o rt_pd_manager.o

obj-$(CONFIG_USB_POWER_DELIVERY)	+= tcpci_event.o \
		pd_core.o pd_policy_engine.o pd_process_evt.o \
		pd_dpm_core.o pd_dpm_uvdm.o pd_dpm_alt_mode_dp.o pd_dpm_pdo_select.o\
		pd_dpm_reaction.o \
		pd_process_evt_snk.o pd_process_evt_src.o pd_process_evt_vdm.o \
		pd_process_evt_drs.o pd_process_evt_prs.o pd_process_evt_vcs.o \
		pd_process_evt_dbg.o pd_process_evt_tcp.o pd_process_evt_com.o \
		pd_policy_engine_src.o pd_policy_engine_snk.o pd_policy_engine_ufp.o pd_policy_engine_vcs.o \
		pd_policy_engine_dfp.o pd_policy_engine_dr.o pd_policy_engine_drs.o pd_policy_engine_prs.o \
		pd_policy_engine_dbg.o pd_policy_engine_com.o pd_dpm_alt_mode_dc.o

ifeq (y, $(filter y, $(CONFIG_MACH_MT6877) $(CONFIG_MACH_MT6768) $(CONFIG_MACH_MT6781) \
$(CONFIG_MACH_MT6739) $(CONFIG_MACH_MT6833) $(CONFIG_MACH_MT6893) $(CONFIG_MACH_MT6885) \
$(CONFIG_MACH_MT6853) $(CONFIG_MACH_MT6873)))
	ccflags-y += -DADAPT_CHARGER_V1
endif
