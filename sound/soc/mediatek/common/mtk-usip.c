// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Kai Chieh Chuang <kaichieh.chuang@mediatek.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/notifier.h>

#include <mtk_ccci_common.h>
#ifdef CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT
#include <adsp_helper.h>
#include "audio_messenger_ipi.h"
#include "audio_task.h"
#include "adsp_ipi.h"
#include "audio_speech_msg_id.h"
#include "mtk-dsp-common.h"
#endif

#define USIP_EMP_IOC_MAGIC 'D'
#define GET_USIP_EMI_SIZE _IOWR(USIP_EMP_IOC_MAGIC, 0xF0, unsigned long long)
#define GET_USIP_ADSP_PHONE_CALL_ENH_CONFIG _IOWR(USIP_EMP_IOC_MAGIC, 0xF1, unsigned long long)
#define SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG _IOWR(USIP_EMP_IOC_MAGIC, 0xF2, unsigned long long)

#define NUM_MPU_REGION 3

int EMI_TABLE[3][3]
	= {{0, 0, 0x30000}, {1, 0x30000, 0x8000}, {2, 0x38000, 0x28000} };

enum {
	SP_EMI_AP_USIP_PARAMETER,
	SP_EMI_ADSP_USIP_PHONECALL,
	SP_EMI_ADSP_USIP_SMARTPA
};

enum {
	SP_EMI_TYPE,
	SP_EMI_OFFSET,
	SP_EMI_SIZE
};

struct usip_info {
	bool memory_ready;
	size_t memory_size;

	void *memory_area;
	dma_addr_t memory_addr;
	phys_addr_t addr_phy;

	unsigned int adsp_phone_call_enh_config;
};

static struct usip_info usip;

static long usip_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	long size_for_spe = 0;

	pr_info("%s(), cmd 0x%x, arg %lu, memory_size = %ld addr_phy = 0x%llx\n",
		 __func__, cmd, arg, usip.memory_size, usip.addr_phy);

	size_for_spe = EMI_TABLE[SP_EMI_AP_USIP_PARAMETER][SP_EMI_SIZE];
	switch (cmd) {

	case GET_USIP_EMI_SIZE:
		if (!usip.memory_ready) {
			pr_info("no phy addr from ccci");
			ret = -ENODEV;
		} else if (copy_to_user((void __user *)arg, &size_for_spe,
			sizeof(size_for_spe))) {
			pr_warn("Fail copy to user Ptr:%p, r_sz:%zu",
				(char *)&size_for_spe, sizeof(size_for_spe));
			ret = -1;
		}
		break;

	case GET_USIP_ADSP_PHONE_CALL_ENH_CONFIG:
		if (copy_to_user((void __user *)arg, &(usip.adsp_phone_call_enh_config),
			sizeof(usip.adsp_phone_call_enh_config))) {
			pr_info("%s(), Fail copy CALL_ENH_CONFIG to user Ptr: %x",
				__func__,
				usip.adsp_phone_call_enh_config);
			ret = -1;
		}
		break;

	case SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG:
		if (copy_from_user(&(usip.adsp_phone_call_enh_config), (void __user *)arg,
			sizeof(usip.adsp_phone_call_enh_config))) {
			ret = -1;
		}
		pr_info("%s(): in SET_USIP_ADSP_PHONE_CALL_ENH_CONFIG: %d",
			__func__,
			usip.adsp_phone_call_enh_config);
		break;
	default:
		pr_debug("%s(), default\n", __func__);
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long usip_compat_ioctl(struct file *fp, unsigned int cmd,
			      unsigned long arg)
{
	long ret;

	if (!fp->f_op || !fp->f_op->unlocked_ioctl)
		return -ENOTTY;
	ret = fp->f_op->unlocked_ioctl(fp, cmd, arg);
	if (ret < 0)
		pr_err("%s(), fail, ret %ld, cmd 0x%x, arg %lu\n",
				__func__, ret, cmd, arg);
	return ret;
}
#endif

int usip_mmap_data(struct usip_info *usip, struct vm_area_struct *area)
{
	long size;
	unsigned long offset;
	size_t align_bytes = PAGE_ALIGN(usip->memory_size);
	unsigned long pfn;
	int ret;

	pr_info("%s(), memory ready %d, size %zu, align_bytes %zu\n",
		__func__, usip->memory_ready, usip->memory_size, align_bytes);

	if (!usip->memory_ready)
		return -EBADFD;

	size = area->vm_end - area->vm_start;
	offset = area->vm_pgoff << PAGE_SHIFT;

	if ((size_t)size > align_bytes)
		return -EINVAL;
	if (offset > align_bytes - size)
		return -EINVAL;

	/*area->vm_ops = &usip_vm_ops_data;*/
	/*area->vm_private_data = usip;*/

	pfn = usip->addr_phy >> PAGE_SHIFT;
	/* ensure that memory does not get swapped to disk */
	area->vm_flags |= VM_IO;
	/* ensure non-cacheable */
	area->vm_page_prot = pgprot_noncached(area->vm_page_prot);
	ret = remap_pfn_range(area, area->vm_start,
			pfn, size, area->vm_page_prot);
	if (ret)
		pr_err("%s(), ret %d, remap failed 0x%lx, phys_addr %pa -> vm_start 0x%lx\n",
		       __func__, ret, pfn,
		       &usip->addr_phy,
		       area->vm_start);
	/*Comment*/
	smp_mb();

	return ret;
}

static int usip_mmap(struct file *file, struct vm_area_struct *area)
{
	unsigned long offset;
	struct usip_info *usip = file->private_data;

	pr_debug("%s(), vm_flags 0x%lx, vm_pgoff 0x%lx\n",
		 __func__, area->vm_flags, area->vm_pgoff);

	offset = area->vm_pgoff << PAGE_SHIFT;
	switch (offset) {
	default:
		return usip_mmap_data(usip, area);
	}
	return 0;
}
static void usip_get_addr(void)
{
}
#ifdef CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT
static void usip_send_emi_info_to_dsp(void)
{
	int send_result = 0;
	struct ipi_msg_t ipi_msg;
	long long usip_emi_info[2]; //idx0 for addr, idx1 for size
	phys_addr_t offset = 0;

	if ((usip.adsp_phone_call_enh_config & 0x1) == 0) {
		pr_info("%s(), adsp_phone_call_enh_config(%d) is close",
			__func__,
			usip.adsp_phone_call_enh_config);
		return;
	}

	if (!usip.memory_ready) {
		usip_get_addr();
		if (!usip.memory_ready) {
			pr_info("%s(), cannot get emi addr from ccci",
				__func__);
			return;
		}
	}

	offset = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_OFFSET];
	usip_emi_info[0] = usip.addr_phy + offset;
	usip_emi_info[1] = EMI_TABLE[SP_EMI_ADSP_USIP_PHONECALL][SP_EMI_SIZE];

	ipi_msg.magic      = IPI_MSG_MAGIC_NUMBER;
	ipi_msg.task_scene = TASK_SCENE_PHONE_CALL;
	ipi_msg.source_layer  = AUDIO_IPI_LAYER_FROM_KERNEL;
	ipi_msg.target_layer  = AUDIO_IPI_LAYER_TO_DSP;
	ipi_msg.data_type  = AUDIO_IPI_PAYLOAD;
	ipi_msg.ack_type   = AUDIO_IPI_MSG_BYPASS_ACK;
	ipi_msg.msg_id     = IPI_MSG_A2D_GET_EMI_ADDRESS;
	ipi_msg.param1     = sizeof(usip_emi_info);
	ipi_msg.param2     = 0;

	/* Send EMI Address to Hifi3 Via IPI*/
	adsp_register_feature(VOICE_CALL_FEATURE_ID);
	send_result = audio_send_ipi_msg(
					 &ipi_msg, TASK_SCENE_PHONE_CALL,
					 AUDIO_IPI_LAYER_TO_DSP,
					 AUDIO_IPI_PAYLOAD,
					 AUDIO_IPI_MSG_BYPASS_ACK,
					 IPI_MSG_A2D_GET_EMI_ADDRESS,
					 sizeof(usip_emi_info),
					 0,
					 (char *)&usip_emi_info);
	adsp_deregister_feature(VOICE_CALL_FEATURE_ID);

	if (send_result != 0)
		pr_info("%s(), scp_ipi send fail\n", __func__);
	else
		pr_debug("%s(), scp_ipi send succeed\n", __func__);
}

static int audio_call_event_receive(struct notifier_block *this,
				    unsigned long event,
				    void *ptr)
{
	switch (event) {
	case ADSP_EVENT_STOP:
		break;
	case ADSP_EVENT_READY:
		usip_send_emi_info_to_dsp();
		break;
	default:
		pr_info("event %lu err", event);
	}
	return 0;
}

static struct notifier_block audio_call_notifier = {
	.notifier_call = audio_call_event_receive,
	.priority = VOICE_CALL_FEATURE_PRI,
};
#endif /* end of CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT */

static int usip_open(struct inode *inode, struct file *file)
{

	file->private_data = &usip;

	if (!usip.memory_ready) {
		usip_get_addr();
#ifdef CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT
		usip_send_emi_info_to_dsp();
#endif
	}

	return 0;
}

static const struct file_operations usip_fops = {
	.owner = THIS_MODULE,
	.open = usip_open,
	.unlocked_ioctl = usip_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = usip_compat_ioctl,
#endif
	.mmap = usip_mmap,
};

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
static struct miscdevice usip_miscdevice = {
	.minor      = MISC_DYNAMIC_MINOR,
	.name       = "usip",
	.fops       = &usip_fops,
};
#endif


static int __init usip_init(void)
{
	int ret;

	usip.memory_ready = false;
	usip.memory_size = 0;
	usip.addr_phy = 0;

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	ret = misc_register(&usip_miscdevice);
	if (ret) {
		pr_err("%s(), cannot register miscdev on minor %d, ret %d\n",
		       __func__, usip_miscdevice.minor, ret);
		ret = -ENODEV;
	}
#else
	ret = -ENODEV;
#endif

	/* init usip info */
	usip.memory_addr = 0x11220000L;
#ifdef CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT
	usip.adsp_phone_call_enh_config = 1;
#else
	usip.adsp_phone_call_enh_config = 0;
#endif

#ifdef CONFIG_MTK_AURISYS_PHONE_CALL_SUPPORT
#ifdef AUDIO_DSP_V1
	adsp_A_register_notify(&audio_call_notifier);
#else
	adsp_register_notify(&audio_call_notifier);
#endif
#endif

	return ret;
}

static void __exit usip_exit(void)
{
#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)
	misc_deregister(&usip_miscdevice);
#endif
}

module_init(usip_init);
module_exit(usip_exit);

MODULE_DESCRIPTION("Mediatek uSip memory control");
MODULE_LICENSE("GPL v2");

