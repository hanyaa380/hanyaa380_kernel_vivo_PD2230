/*
 *  linux/drivers/mmc/core/core.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/scatterlist.h>
#include <linux/log2.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/suspend.h>
#include <linux/fault-inject.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <uapi/linux/sched/types.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/slot-gpio.h>

#define CREATE_TRACE_POINTS
#include <trace/events/mmc.h>

#include "core.h"
#include "card.h"
#include "queue.h"
#include "bus.h"
#include "host.h"
#include "sdio_bus.h"
#include "pwrseq.h"

#include "mmc_ops.h"
#include "sd_ops.h"
#include "sdio_ops.h"
#include "mtk_mmc_block.h"
#include "slot-gpio.h"
#include "../host/mtk-sd-dbg.h"

extern void mmc_set_sd_status(int value);

/* The max erase timeout, used when host->max_busy_timeout isn't specified */
#define MMC_ERASE_TIMEOUT_MS	(60 * 1000) /* 60 s */

static const unsigned freqs[] = { 400000, 300000, 200000, 100000 };

/*
 * Enabling software CRCs on the data blocks can be a significant (30%)
 * performance cost, and for other reasons may not always be desired.
 * So we allow it it to be disabled.
 */
bool use_spi_crc = 1;
module_param(use_spi_crc, bool, 0);

static int mmc_schedule_delayed_work(struct delayed_work *work,
				     unsigned long delay)
{
	/*
	 * We use the system_freezable_wq, because of two reasons.
	 * First, it allows several works (not the same work item) to be
	 * executed simultaneously. Second, the queue becomes frozen when
	 * userspace becomes frozen during system PM.
	 */
	return queue_delayed_work(system_freezable_wq, work, delay);
}

#ifdef CONFIG_FAIL_MMC_REQUEST

/*
 * Internal function. Inject random data errors.
 * If mmc_data is NULL no errors are injected.
 */
static void mmc_should_fail_request(struct mmc_host *host,
				    struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	static const int data_errors[] = {
		-ETIMEDOUT,
		-EILSEQ,
		-EIO,
	};

	if (!data)
		return;

	if ((cmd && cmd->error) || data->error ||
	    !should_fail(&host->fail_mmc_request, data->blksz * data->blocks))
		return;

	data->error = data_errors[prandom_u32() % ARRAY_SIZE(data_errors)];
	data->bytes_xfered = (prandom_u32() % (data->bytes_xfered >> 9)) << 9;
}

#else /* CONFIG_FAIL_MMC_REQUEST */

static inline void mmc_should_fail_request(struct mmc_host *host,
					   struct mmc_request *mrq)
{
}

#endif /* CONFIG_FAIL_MMC_REQUEST */

static inline void mmc_complete_cmd(struct mmc_request *mrq)
{
	if (mrq->cap_cmd_during_tfr && !completion_done(&mrq->cmd_completion))
		complete_all(&mrq->cmd_completion);
}

void mmc_command_done(struct mmc_host *host, struct mmc_request *mrq)
{
	if (!mrq->cap_cmd_during_tfr)
		return;

	mmc_complete_cmd(mrq);

	pr_debug("%s: cmd done, tfr ongoing (CMD%u)\n",
		 mmc_hostname(host), mrq->cmd->opcode);
}
EXPORT_SYMBOL(mmc_command_done);

/**
 *	mmc_request_done - finish processing an MMC request
 *	@host: MMC host which completed request
 *	@mrq: MMC request which request
 *
 *	MMC drivers should call this function when they have completed
 *	their processing of a request.
 */
void mmc_request_done(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	int err = cmd->error;

	/* Flag re-tuning needed on CRC errors */
	if (cmd->opcode != MMC_SEND_TUNING_BLOCK &&
	    cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200 &&
	    !host->retune_crc_disable &&
	    (err == -EILSEQ || (mrq->sbc && mrq->sbc->error == -EILSEQ) ||
	    (mrq->data && mrq->data->error == -EILSEQ) ||
	    (mrq->stop && mrq->stop->error == -EILSEQ)))
		mmc_retune_needed(host);

	if (err && cmd->retries && mmc_host_is_spi(host)) {
		if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
			cmd->retries = 0;
	}

	if (host->ongoing_mrq == mrq)
		host->ongoing_mrq = NULL;

	mmc_complete_cmd(mrq);

	trace_mmc_request_done(host, mrq);

	/*
	 * We list various conditions for the command to be considered
	 * properly done:
	 *
	 * - There was no error, OK fine then
	 * - We are not doing some kind of retry
	 * - The card was removed (...so just complete everything no matter
	 *   if there are errors or retries)
	 */
	if (!err || !cmd->retries || mmc_card_removed(host->card)) {
		mmc_should_fail_request(host, mrq);

		if (!host->ongoing_mrq)
			led_trigger_event(host->led, LED_OFF);

		if (mrq->sbc) {
			pr_debug("%s: sbc req done <CMD%u>: %d: %08x %08x %08x %08x\n",
				mmc_hostname(host), mrq->sbc->opcode,
				mrq->sbc->error,
				mrq->sbc->resp[0], mrq->sbc->resp[1],
				mrq->sbc->resp[2], mrq->sbc->resp[3]);
		}

		pr_debug("%s: req done (CMD%u): %d: %08x %08x %08x %08x\n",
			mmc_hostname(host), cmd->opcode, err,
			cmd->resp[0], cmd->resp[1],
			cmd->resp[2], cmd->resp[3]);

		if (mrq->data) {
			pr_debug("%s:     %d bytes transferred: %d\n",
				mmc_hostname(host),
				mrq->data->bytes_xfered, mrq->data->error);
		}

		if (mrq->stop) {
			pr_debug("%s:     (CMD%u): %d: %08x %08x %08x %08x\n",
				mmc_hostname(host), mrq->stop->opcode,
				mrq->stop->error,
				mrq->stop->resp[0], mrq->stop->resp[1],
				mrq->stop->resp[2], mrq->stop->resp[3]);
		}
	}

	if(host && cmd)
		dbg_add_host_log(host, 1, cmd->opcode, cmd->resp[0]);

	/*
	 * Request starter must handle retries - see
	 * mmc_wait_for_req_done().
	 */
	if (mrq->done)
		mrq->done(mrq);

	/* mmc_crypto_debug(host); */
}

EXPORT_SYMBOL(mmc_request_done);

static void __mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	/* Assumes host controller has been runtime resumed by mmc_claim_host */
	err = mmc_retune(host);
	if (err) {
		mrq->cmd->error = err;
		mmc_request_done(host, mrq);
		return;
	}

	/*
	 * For sdio rw commands we must wait for card busy otherwise some
	 * sdio devices won't work properly.
	 * And bypass I/O abort, reset and bus suspend operations.
	 */
	if (sdio_is_io_busy(mrq->cmd->opcode, mrq->cmd->arg) &&
	    host->ops->card_busy) {
		int tries = 500; /* Wait aprox 500ms at maximum */

		while (host->ops->card_busy(host) && --tries)
			mmc_delay(1);

		if (tries == 0) {
			mrq->cmd->error = -EBUSY;
			mmc_request_done(host, mrq);
			return;
		}
	}

	if (mrq->cap_cmd_during_tfr) {
		host->ongoing_mrq = mrq;
		/*
		 * Retry path could come through here without having waiting on
		 * cmd_completion, so ensure it is reinitialised.
		 */
		reinit_completion(&mrq->cmd_completion);
	}

	trace_mmc_request_start(host, mrq);

	if (host->cqe_on)
		host->cqe_ops->cqe_off(host);

	host->ops->request(host, mrq);
}

static void mmc_mrq_pr_debug(struct mmc_host *host, struct mmc_request *mrq,
			     bool cqe)
{
	if(host == NULL || mrq == NULL || mrq->data == NULL)
		return;

	if (mrq->sbc) {
		pr_debug("<%s: starting CMD%u arg %08x flags %08x>\n",
			 mmc_hostname(host), mrq->sbc->opcode,
			 mrq->sbc->arg, mrq->sbc->flags);
	}

	if (mrq->cmd) {
		pr_debug("%s: starting %sCMD%u arg %08x flags %08x\n",
			 mmc_hostname(host), cqe ? "CQE direct " : "",
			 mrq->cmd->opcode, mrq->cmd->arg, mrq->cmd->flags);
	} else if (cqe) {
		if (mrq->data->flags & MMC_DATA_WRITE)
			pr_debug("%s: starting CQE transfer for tag %d blkaddr %u, flags=0x%x WRITE\n",
			mmc_hostname(host), mrq->tag, mrq->data->blk_addr, mrq->data->flags);
		else if (mrq->data->flags & MMC_DATA_READ)
			pr_debug("%s: starting CQE transfer for tag %d blkaddr %u, flags=0x%x READ\n",
			mmc_hostname(host), mrq->tag, mrq->data->blk_addr, mrq->data->flags);
	}

	if (mrq->data) {
		pr_debug("%s:     blksz %d blocks %d flags %08x "
			"tsac %d ms nsac %d\n",
			mmc_hostname(host), mrq->data->blksz,
			mrq->data->blocks, mrq->data->flags,
			mrq->data->timeout_ns / 1000000,
			mrq->data->timeout_clks);
	}

	if (mrq->stop) {
		pr_debug("%s:     CMD%u arg %08x flags %08x\n",
			 mmc_hostname(host), mrq->stop->opcode,
			 mrq->stop->arg, mrq->stop->flags);
	}
}

static int mmc_mrq_prep(struct mmc_host *host, struct mmc_request *mrq)
{
	unsigned int i, sz = 0;
	struct scatterlist *sg;

	if (mrq->cmd) {
		mrq->cmd->error = 0;
		mrq->cmd->mrq = mrq;
		mrq->cmd->data = mrq->data;
	}
	if (mrq->sbc) {
		mrq->sbc->error = 0;
		mrq->sbc->mrq = mrq;
	}
	if (mrq->data) {
		if (mrq->data->blksz > host->max_blk_size ||
		    mrq->data->blocks > host->max_blk_count ||
		    mrq->data->blocks * mrq->data->blksz > host->max_req_size)
			return -EINVAL;

		for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i)
			sz += sg->length;
		if (sz != mrq->data->blocks * mrq->data->blksz)
			return -EINVAL;

		mrq->data->error = 0;
		mrq->data->mrq = mrq;
		if (mrq->stop) {
			mrq->data->stop = mrq->stop;
			mrq->stop->error = 0;
			mrq->stop->mrq = mrq;
		}
	}

	return 0;
}

#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
static void mmc_enqueue_queue(struct mmc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	if (mrq->cmd->opcode == MMC_EXECUTE_READ_TASK ||
		mrq->cmd->opcode == MMC_EXECUTE_WRITE_TASK) {

		spin_lock_irqsave(&host->dat_que_lock, flags);
		if (mrq->flags)
			list_add(&mrq->link, &host->dat_que);
		else
			list_add_tail(&mrq->link, &host->dat_que);
		spin_unlock_irqrestore(&host->dat_que_lock, flags);
	} else {

		spin_lock_irqsave(&host->cmd_que_lock, flags);
		if (mrq->flags)
			list_add(&mrq->link, &host->cmd_que);
		else
			list_add_tail(&mrq->link, &host->cmd_que);

		spin_unlock_irqrestore(&host->cmd_que_lock, flags);

	}
}

static void mmc_dequeue_queue(struct mmc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	if (mrq->cmd->opcode == MMC_EXECUTE_READ_TASK ||
		mrq->cmd->opcode == MMC_EXECUTE_WRITE_TASK) {
		spin_lock_irqsave(&host->dat_que_lock, flags);
		list_del_init(&mrq->link);
		spin_unlock_irqrestore(&host->dat_que_lock, flags);
	}
}

static void mmc_clr_dat_mrq_que_flag(struct mmc_host *host)
{
	unsigned int i;

	for (i = 0; i < host->card->ext_csd.cmdq_depth; i++)
		host->data_mrq_queued[i] = false;
}

static void mmc_clr_dat_list(struct mmc_host *host)
{

	unsigned long flags;
	struct mmc_request *mrq = NULL;
	struct mmc_request *mrq_next = NULL;

	spin_lock_irqsave(&host->dat_que_lock, flags);
	list_for_each_entry_safe(mrq, mrq_next, &host->dat_que, link) {
		list_del_init(&mrq->link);
	}
	spin_unlock_irqrestore(&host->dat_que_lock, flags);

	mmc_clr_dat_mrq_que_flag(host);
}

static int mmc_restore_tasks(struct mmc_host *host)
{
	struct mmc_request *mrq_cmd = NULL;
	unsigned int i = 0;
	unsigned int task_id;
	unsigned int tasks;

	tasks = host->task_id_index;
	for (task_id = 0; task_id < host->card->ext_csd.cmdq_depth; task_id++) {
		if (tasks & 0x1) {
			mrq_cmd = host->areq_que[task_id]->mrq_que;
			mmc_enqueue_queue(host, mrq_cmd);
			clear_bit(task_id, &host->task_id_index);
			i++;
		}
		tasks >>= 1;
	}

	return i;
}

static struct mmc_request *mmc_get_cmd_que(struct mmc_host *host)
{
	struct mmc_request *mrq = NULL;

	if (!list_empty(&host->cmd_que)) {
		mrq = list_first_entry(&host->cmd_que,
			struct mmc_request, link);
		list_del_init(&mrq->link);
	}

	return mrq;
}

static struct mmc_request *mmc_get_dat_que(struct mmc_host *host)
{
	struct mmc_request *mrq = NULL;

	if (!list_empty(&host->dat_que)) {
		mrq = list_first_entry(&host->dat_que,
			struct mmc_request, link);
	}
	return mrq;
}

static int mmc_blk_status_check(struct mmc_card *card, unsigned int *status)
{
	struct mmc_command cmd = {0};
	int err, retries = 3;

	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, retries);
	if (err == 0)
		*status = cmd.resp[0];
	else
		pr_info("%s: err %d\n", __func__, err);

	return err;
}

static void mmc_discard_cmdq(struct mmc_host *host)
{
	memset(&host->deq_cmd, 0, sizeof(struct mmc_command));
	memset(&host->deq_mrq, 0, sizeof(struct mmc_request));

	host->deq_cmd.opcode = MMC_CMDQ_TASK_MGMT;
	host->deq_cmd.arg = 1;
	host->deq_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1B | MMC_CMD_AC;
	host->deq_mrq.data = NULL;
	host->deq_mrq.cmd = &host->deq_cmd;

	host->deq_mrq.done = mmc_wait_cmdq_done;
	host->deq_mrq.host = host;
	host->deq_mrq.cmd->retries = 3;
	host->deq_mrq.cmd->error = 0;
	host->deq_mrq.cmd->mrq = &host->deq_mrq;

	while (1) {
		trace_mmc_request_start(host, &host->deq_mrq);
		host->ops->request(host, &host->deq_mrq);

		if (!host->deq_mrq.cmd->error ||
			!host->deq_mrq.cmd->retries)
			break;

		pr_info("%s: req failed (CMD%u): %d, retrying...\n",
			 __func__,
			 host->deq_mrq.cmd->opcode,
			 host->deq_mrq.cmd->error);

		host->deq_mrq.cmd->retries--;
		host->deq_mrq.cmd->error = 0;
	};

	pr_notice("%s: CMDQ send distard (CMD48)\n", __func__);
}

/* add for emmc reset when error happen */
int emmc_resetting_when_cmdq;
static int mmc_reset_for_cmdq(struct mmc_host *host)
{
	int err, ret;

	emmc_resetting_when_cmdq = 1;
	err = mmc_hw_reset(host);
	/* Ensure we switch back to the correct partition */
	if (err != -EOPNOTSUPP) {
		u8 part_config = host->card->ext_csd.part_config;

		part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		/*  only enable cq at user */
		part_config |= 0;

		ret = mmc_switch(host->card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_PART_CONFIG, part_config,
				host->card->ext_csd.part_time);
		if (ret)
			return ret;

		/* enable cmdq at all partition */
		ret = mmc_cmdq_enable(host->card);
		if (ret)
			return ret;

		host->card->ext_csd.part_config = part_config;

	}
	emmc_resetting_when_cmdq = 0;
	return err;
}

/*
 *	check CMDQ QSR
 */
void mmc_do_check(struct mmc_host *host)
{
	memset(&host->que_cmd, 0, sizeof(struct mmc_command));
	memset(&host->que_mrq, 0, sizeof(struct mmc_request));
	host->que_cmd.opcode = MMC_SEND_STATUS;
	host->que_cmd.arg = host->card->rca << 16 | 1 << 15;
	host->que_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	host->que_cmd.data = NULL;
	host->que_mrq.cmd = &host->que_cmd;

	host->que_mrq.done = mmc_wait_cmdq_done;
	host->que_mrq.host = host;
	host->que_mrq.cmd->retries = 3;
	host->que_mrq.cmd->error = 0;
	host->que_mrq.cmd->mrq = &host->que_mrq;
	while (1) {
		trace_mmc_request_start(host, &host->que_mrq);
		host->ops->request(host, &host->que_mrq);

		/* add for emmc reset when error happen */
		if (host->que_mrq.cmd->error && !host->que_mrq.cmd->retries) {
	/* wait data irq handle done otherwice timing issue will happen  */
			msleep(2000);
			if (mmc_reset_for_cmdq(host)) {
				WARN_ON(1);
				pr_info("%s: line=%d [CQ] reinit fail\n",
					__func__, __LINE__);
			}
			mmc_clr_dat_list(host);
			mmc_restore_tasks(host);
			atomic_set(&host->cq_wait_rdy, 0);
			atomic_set(&host->cq_rdy_cnt, 0);
		}

		if (!host->que_mrq.cmd->error ||
			!host->que_mrq.cmd->retries)
			break;

		pr_info("%s: req failed (CMD%u): %d, retrying...\n",
			 __func__,
			 host->que_mrq.cmd->opcode,
			 host->que_mrq.cmd->error);

		host->que_mrq.cmd->retries--;
		host->que_mrq.cmd->error = 0;
	};
}

static void mmc_prep_chk_mrq(struct mmc_host *host)
{
	memset(&host->chk_cmd, 0, sizeof(struct mmc_command));
	memset(&host->chk_mrq, 0, sizeof(struct mmc_request));
	host->chk_cmd.opcode = MMC_SEND_STATUS;
	host->chk_cmd.arg = host->card->rca << 16;
	host->chk_cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
	host->chk_cmd.data = NULL;
	host->chk_mrq.cmd = &host->chk_cmd;

	host->chk_mrq.done = mmc_wait_cmdq_done;
	host->chk_mrq.host = host;
	host->chk_mrq.cmd->error = 0;
	host->chk_mrq.cmd->mrq = &host->chk_mrq;
}

static void mmc_prep_areq_que(struct mmc_host *host,
	struct mmc_async_req *areq_que)
{
	areq_que->mrq->done = mmc_wait_cmdq_done;
	areq_que->mrq->host = host;
	areq_que->mrq->cmd->error = 0;
	areq_que->mrq->cmd->mrq = areq_que->mrq;
	areq_que->mrq->cmd->data =
		areq_que->mrq->data;
	areq_que->mrq->data->error = 0;
	areq_que->mrq->data->mrq = areq_que->mrq;
	if (areq_que->mrq->stop) {
		areq_que->mrq->data->stop =
			areq_que->mrq->stop;
		areq_que->mrq->stop->error = 0;
		areq_que->mrq->stop->mrq = areq_que->mrq;
	}
}

/*
 *	check status register
 */
void mmc_do_status(struct mmc_host *host)
{
	mmc_prep_chk_mrq(host);
	trace_mmc_request_start(host, &host->chk_mrq);
	host->ops->request(host, &host->chk_mrq);
}

/*
 * send stop command
 */
void mmc_do_stop(struct mmc_host *host)
{
	memset(&host->que_cmd, 0, sizeof(struct mmc_command));
	memset(&host->que_mrq, 0, sizeof(struct mmc_request));
	host->que_cmd.opcode = MMC_STOP_TRANSMISSION;
	host->que_cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	host->que_mrq.cmd = &host->que_cmd;
	host->que_mrq.done = mmc_wait_cmdq_done;
	host->que_mrq.host = host;
	host->que_mrq.cmd->retries = 3;
	host->que_mrq.cmd->error = 0;
	host->que_mrq.cmd->mrq = &host->que_mrq;

	while (1) {
		trace_mmc_request_start(host, &host->que_mrq);
		host->ops->request(host, &host->que_mrq);

		if (!host->que_mrq.cmd->error ||
			!host->que_mrq.cmd->retries)
			break;

		pr_info("%s: req failed (CMD%u): %d, retrying...\n",
			__func__,
			host->que_mrq.cmd->opcode,
			host->que_mrq.cmd->error);

		host->que_mrq.cmd->retries--;
		host->que_mrq.cmd->error = 0;
	};
}

static int mmc_wait_tran(struct mmc_host *host)
{
	u32 status;
	int err;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(10 * 1000);
	do {
		err = mmc_blk_status_check(host->card, &status);
		if (err) {
			pr_notice("[CQ] check card status error = %d\n", err);
			return 1;
		}

		if ((R1_CURRENT_STATE(status) == R1_STATE_DATA) ||
			(R1_CURRENT_STATE(status) == R1_STATE_RCV))
			mmc_do_stop(host);

		if (time_after(jiffies, timeout)) {
			pr_info("%s: Card stuck in %d state! %s\n",
				mmc_hostname(host),
				R1_CURRENT_STATE(status), __func__);
			return 1;
		}
	} while (R1_CURRENT_STATE(status) != R1_STATE_TRAN);

	return 0;
}

/*
 * check write
 */
static int mmc_check_write(struct mmc_host *host, struct mmc_request *mrq)
{
	int ret = 0;
	u32 status = 0;
	struct mmc_queue_req *mq_rq;
	struct mmc_async_req *areq_active;

	if (mrq->cmd->opcode == MMC_EXECUTE_WRITE_TASK) {
		ret = mmc_blk_status_check(host->card, &status);

		if ((status & R1_WP_VIOLATION) || host->wp_error) {
			mrq->data->error = -EROFS;
			areq_active =
				host->areq_que[(mrq->cmd->arg >> 16) & 0x1f];
			mq_rq = container_of(areq_active, struct mmc_queue_req,
					areq);
			pr_notice(
	"[%s]: data error = %d, status=0x%x, line:%d, block addr:0x%x\n",
				__func__, mrq->data->error, status,
				__LINE__, mq_rq->brq.que.arg);
		}
		mmc_wait_tran(host);
		mrq->data->error = 0;
		host->wp_error = 0;
		atomic_set(&host->cq_w, false);
	}

	return ret;
}

unsigned long not_ready_time;
void mmc_wait_cmdq_done(struct mmc_request *mrq)
{
	struct mmc_host *host = mrq->host;
	struct mmc_command *cmd = mrq->cmd;
	int done = 0, task_id;

	if (cmd->opcode == MMC_SEND_STATUS ||
		cmd->opcode == MMC_STOP_TRANSMISSION ||
		cmd->opcode == MMC_CMDQ_TASK_MGMT) {
		/* do nothing */
	} else
		mmc_dequeue_queue(host, mrq);

	/* error - request done */
	if (cmd->error) {
		pr_info("%s: cmd%d arg:%x error:%d\n",
			mmc_hostname(host),
			cmd->opcode, cmd->arg,
			cmd->error);
		if ((cmd->opcode == MMC_EXECUTE_READ_TASK) ||
			(cmd->opcode == MMC_EXECUTE_WRITE_TASK)) {
			atomic_set(&host->cq_tuning_now, 1);
			goto clear_end;
		}
		goto request_end;
	}

	/* data error */
	if (mrq->data && mrq->data->error) {
		pr_info("%s: cmd%d arg:%x data error:%d\n",
			mmc_hostname(host),
			cmd->opcode, cmd->arg,
			mrq->data->error);
		atomic_set(&host->cq_tuning_now, 1);
		goto clear_end;
	}

	/* check wp violation */
	if ((cmd->opcode == MMC_QUE_TASK_PARAMS) ||
		(cmd->opcode == MMC_QUE_TASK_ADDR)) {

		if (atomic_read(&host->cq_w)) {
			if (cmd->resp[0] & R1_WP_VIOLATION)
				host->wp_error = 1;
		}
	}

	/* cmd13' - check queue ready & enqueue 46/47 */
	if ((cmd->opcode == MMC_SEND_STATUS) && (cmd->arg & (1 << 15))) {
		int i = 0;
		unsigned int resp = cmd->resp[0];

		if (resp == 0) {
/* Workaround for ALPS03808823: if task not ready over 30s, reinit emmc */
			if (!not_ready_time)
				not_ready_time = jiffies;
			else if (time_after(jiffies, not_ready_time
			+ msecs_to_jiffies(30 * 1000))) {
				pr_info("mmc0: error: task not ready over 30s\n");
				msleep(2000);
				if (mmc_reset_for_cmdq(host)) {
					pr_info("%s: line=%d [CQ] reinit fail\n",
						__func__, __LINE__);
					WARN_ON(1);
				}
				mmc_clr_dat_list(host);
				mmc_restore_tasks(host);
				atomic_set(&host->cq_wait_rdy, 0);
				atomic_set(&host->cq_rdy_cnt, 0);
				not_ready_time = 0;

				//aee_kernel_warning("mmc",
				//	"task not ready over 30s");
			}
			goto request_end;
		}
		not_ready_time = 0;
		do {
			if ((resp & 1) && (!host->data_mrq_queued[i])) {
				if (host->cur_rw_task == i) {
					resp >>= 1;
					i++;
					continue;
				}

				if (!host->areq_que[i]) {
					pr_info("%s: task %d not exist!,QSR:%x\n",
						mmc_hostname(host), i,
						cmd->resp[0]);
					pr_info("%s: task_idx:%08lx\n",
						mmc_hostname(host),
						host->task_id_index);
					pr_info("%s: cnt:%d,wait:%d,rdy:%d\n",
						mmc_hostname(host),
						atomic_read(&host->areq_cnt),
						atomic_read(&host->cq_wait_rdy),
						atomic_read(&host->cq_rdy_cnt));
					/* reset eMMC flow */
					cmd->error = (unsigned int)-ETIMEDOUT;
					cmd->retries = 0;
					goto request_end;
				}

				atomic_dec(&host->cq_wait_rdy);
				atomic_inc(&host->cq_rdy_cnt);

				mmc_prep_areq_que(host, host->areq_que[i]);
				mmc_enqueue_queue(host, host->areq_que[i]->mrq);
				host->data_mrq_queued[i] = true;
			}
			resp >>= 1;
			i++;
		} while (resp && (i < host->card->ext_csd.cmdq_depth));
	}

	/* cmd46 - request done */
	if (cmd->opcode == MMC_EXECUTE_READ_TASK
		|| cmd->opcode == MMC_EXECUTE_WRITE_TASK)
		goto clear_end;

	goto request_end;

clear_end:
	task_id = ((cmd->arg >> 16) & 0x1f);
	clear_bit(task_id, &host->task_id_index);
	host->data_mrq_queued[task_id] = false;
	done = 1;

request_end:
	/* request done when next data transfer */
	if (done) {
		WARN_ON(cmd->opcode != 46 && cmd->opcode != 47);
		WARN_ON(host->done_mrq);
		host->done_mrq = mrq;

		/*
		 * Need to wake up cmdq thread, after done rw.
		 */
		wake_up_interruptible(&host->cmdq_que);
	}
}

static void mmc_wait_for_cmdq_done(struct mmc_host *host)
{
	while (atomic_read(&host->areq_cnt) != 0) {
		wait_event_interruptible(host->cmp_que,
			(atomic_read(&host->areq_cnt) == 0));
	}
}

void mmc_wait_cmdq_empty(struct mmc_host *host)
{
	mmc_wait_for_cmdq_done(host);
}

#define CMD13_TMO_NS (1000 * 1000)
int mmc_run_queue_thread(void *data)
{
	struct mmc_host *host = data;
	struct mmc_request *cmd_mrq = NULL;
	struct mmc_request *dat_mrq = NULL;
	struct mmc_request *done_mrq = NULL;
	unsigned int task_id, areq_cnt_chk, tmo;
	bool is_done = false;
	int err;
	u64 chk_time = 0;

	pr_info("[CQ] start cmdq thread\n");
	mt_bio_queue_alloc(current, NULL, false);

	while (1) {
		mt_biolog_cmdq_check();
		/* End request stage 1/2 */
		if (atomic_read(&host->cq_rw)
		|| (atomic_read(&host->areq_cnt) <= 1)) {
			if (host->done_mrq) {
				done_mrq = host->done_mrq;
				host->done_mrq = NULL;
			}
		}

		if (done_mrq) {
			if (done_mrq->data->error || done_mrq->cmd->error) {
				mmc_wait_tran(host);
				mmc_discard_cmdq(host);
				mmc_wait_tran(host);
				mmc_clr_dat_list(host);
				atomic_set(&host->cq_rdy_cnt, 0);
				if (host->ops->execute_tuning) {
					err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS200);
					if (err && mmc_reset_for_cmdq(host)) {
						pr_info("%s: line=%d ",
							__func__, __LINE__);
						pr_info("[CQ] reinit fail\n");
						WARN_ON(1);
					} else
						pr_notice("[CQ] tuning pass\n");
				}

				host->cur_rw_task = CQ_TASK_IDLE;
				task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
				trace_mmc_request_start(host,
					host->areq_que[task_id]->mrq_que);
				host->ops->request(host,
					host->areq_que[task_id]->mrq_que);
				atomic_set(&host->cq_wait_rdy, 1);
				done_mrq = NULL;
			}

			atomic_set(&host->cq_rw, false);
			if (done_mrq && !done_mrq->data->error
			&& !done_mrq->cmd->error) {
				task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
				mt_biolog_cmdq_dma_end(task_id);
				mmc_check_write(host, done_mrq);
				host->cur_rw_task = CQ_TASK_IDLE;
				is_done = true;
				mmc_complete_mqr_crypto(host);

				if (atomic_read(&host->cq_tuning_now) == 1) {
					mmc_restore_tasks(host);
					atomic_set(&host->cq_tuning_now, 0);
				}
			}
		}

		/* Send Command 46/47 (DMA) */
		if (!atomic_read(&host->cq_rw)) {
			spin_lock_irq(&host->dat_que_lock);
			dat_mrq = mmc_get_dat_que(host);

			spin_unlock_irq(&host->dat_que_lock);

			if (dat_mrq) {
				WARN_ON(
				dat_mrq->cmd->opcode !=
				MMC_EXECUTE_WRITE_TASK
				&& dat_mrq->cmd->opcode !=
				MMC_EXECUTE_READ_TASK);

				if (dat_mrq->cmd->opcode
				== MMC_EXECUTE_WRITE_TASK)
					atomic_set(&host->cq_w, true);

				atomic_set(&host->cq_rw, true);
				task_id = ((dat_mrq->cmd->arg >> 16) & 0x1f);
				host->cur_rw_task = task_id;
				trace_mmc_request_start(host, dat_mrq);
				err = mmc_swcq_prepare_mqr_crypto(host,
					dat_mrq);
				if (err) {
					pr_info("eMMC crypto fail %d\n", err);
					WARN_ON(1);
				}
				host->ops->request(host, dat_mrq);
				mt_biolog_cmdq_dma_start(task_id);
				atomic_dec(&host->cq_rdy_cnt);
				dat_mrq = NULL;
			}
		}

		/* End request stage 2/2 */
		if (is_done) {
			task_id = (done_mrq->cmd->arg >> 16) & 0x1f;
			mt_biolog_cmdq_isdone_start(task_id,
				host->areq_que[task_id]->mrq_que);
			mt_biolog_cmdq_isdone_end(task_id);
			mt_biolog_cmdq_check();
			mmc_blk_end_queued_req(host, done_mrq->areq, task_id);
			done_mrq = NULL;
			is_done = false;
		}

		/* Send Command 44/45 */
		if (atomic_read(&host->cq_tuning_now) == 0) {

			spin_lock_irq(&host->cmd_que_lock);
			cmd_mrq = mmc_get_cmd_que(host);
			spin_unlock_irq(&host->cmd_que_lock);

			while (cmd_mrq) {
				task_id = ((cmd_mrq->sbc->arg >> 16) & 0x1f);
				mt_biolog_cmdq_queue_task(task_id, cmd_mrq);
				if (host->task_id_index & (1 << task_id)) {
					pr_info(
"[%s] BUG!!! task_id %d used, task_id_index 0x%08lx, areq_cnt = %d, cq_wait_rdy = %d\n",
					__func__, task_id, host->task_id_index,
					atomic_read(&host->areq_cnt),
					atomic_read(&host->cq_wait_rdy));
					/* mmc_cmd_dump(host); */
					while (1)
						;
				}
				set_bit(task_id, &host->task_id_index);
				trace_mmc_request_start(host, cmd_mrq);
				host->ops->request(host, cmd_mrq);
				/* add for emmc reset when error happen */
				if ((cmd_mrq->sbc && cmd_mrq->sbc->error)
				|| cmd_mrq->cmd->error) {
		/* wait data irq handle done otherwise timing issue happen*/
					msleep(2000);
					if (mmc_reset_for_cmdq(host)) {
						pr_info("%s: line=%d ",
							__func__, __LINE__);
						pr_info("[CQ] reinit fail\n");
						WARN_ON(1);
					}
					mmc_clr_dat_list(host);
					mmc_restore_tasks(host);
					atomic_set(&host->cq_wait_rdy, 0);
					atomic_set(&host->cq_rdy_cnt, 0);
				} else
					atomic_inc(&host->cq_wait_rdy);

				spin_lock_irq(&host->cmd_que_lock);
				cmd_mrq = mmc_get_cmd_que(host);
				spin_unlock_irq(&host->cmd_que_lock);
			}
		}
		if (atomic_read(&host->cq_rw)) {
			/* wait for event to wakeup */
			/* wake up when new request arrived and dma done */
			areq_cnt_chk = atomic_read(&host->areq_cnt);
			tmo = wait_event_interruptible_timeout(host->cmdq_que,
				host->done_mrq ||
				(atomic_read(&host->areq_cnt) > areq_cnt_chk),
				10 * HZ);
			if (!tmo) {
				pr_info("%s:tmo,mrq(%p),chk(%d),cnt(%d)\n",
					__func__,
					host->done_mrq,
					areq_cnt_chk,
					atomic_read(&host->areq_cnt));
				pr_info("%s:tmo,rw(%d),wait(%d),rdy(%d)\n",
					__func__,
					atomic_read(&host->cq_rw),
					atomic_read(&host->cq_wait_rdy),
					atomic_read(&host->cq_rdy_cnt));
			}
			/* DMA time should not count in polling time */
			chk_time = 0;
		}
		/* Send Command 13' */

		if (atomic_read(&host->cq_wait_rdy) > 0
			&& atomic_read(&host->cq_rdy_cnt) == 0) {
			if (!chk_time)
				/* set check time */
				chk_time = sched_clock();

			/* send cmd13' */
			mmc_do_check(host);

			if (atomic_read(&host->cq_rdy_cnt))
				/* clear when got ready task */
				chk_time = 0;
			else if (sched_clock() - chk_time > CMD13_TMO_NS)
				/* sleep when TMO */
				usleep_range(2000, 5000);
		}

		/* Sleep when nothing to do */
		mt_biolog_cmdq_check();
		set_current_state(TASK_INTERRUPTIBLE);
		if (atomic_read(&host->areq_cnt) == 0)
			schedule();

		set_current_state(TASK_RUNNING);
	}
	mt_bio_queue_free(current);
	return 0;
}
#endif

int mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	init_completion(&mrq->cmd_completion);

	mmc_retune_hold(host);

	if (mmc_card_removed(host->card))
		return -ENOMEDIUM;

	mmc_mrq_pr_debug(host, mrq, false);

	WARN_ON(!host->claimed);

	err = mmc_mrq_prep(host, mrq);
	if (err)
		return err;
#ifdef CONFIG_MTK_EMMC_CQ_SUPPORT
	if (mrq->done == mmc_wait_cmdq_done) {
		mmc_enqueue_queue(host, mrq);
		wake_up_process(host->cmdq_thread);
		led_trigger_event(host->led, LED_FULL);
		return 0;
	}
	if (host->card
		&& host->card->ext_csd.cmdq_support
		&& mrq->cmd->opcode != MMC_SEND_STATUS)
		/* add for emmc reset when error happen */
		/* cannot wait cmdq empty for init requests
		 * when emmc resetting when cmdq
		 */
		if (strncmp(current->comm, "exe_cq", 6)
			|| !emmc_resetting_when_cmdq)
			mmc_wait_cmdq_empty(host);
#endif
	led_trigger_event(host->led, LED_FULL);
	__mmc_start_request(host, mrq);

	return 0;
}
EXPORT_SYMBOL(mmc_start_request);

static void mmc_wait_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

static inline void mmc_wait_ongoing_tfr_cmd(struct mmc_host *host)
{
	struct mmc_request *ongoing_mrq = READ_ONCE(host->ongoing_mrq);

	/*
	 * If there is an ongoing transfer, wait for the command line to become
	 * available.
	 */
	if (ongoing_mrq && !completion_done(&ongoing_mrq->cmd_completion))
		wait_for_completion(&ongoing_mrq->cmd_completion);
}

static int __mmc_start_req(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	mmc_wait_ongoing_tfr_cmd(host);

	init_completion(&mrq->completion);
	mrq->done = mmc_wait_done;

	err = mmc_start_request(host, mrq);
	if (err) {
		mrq->cmd->error = err;
		mmc_complete_cmd(mrq);
		complete(&mrq->completion);
	}

	return err;
}

void mmc_wait_for_req_done(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd;

	while (1) {
		wait_for_completion(&mrq->completion);

		cmd = mrq->cmd;

		/*
		 * If host has timed out waiting for the sanitize
		 * to complete, card might be still in programming state
		 * so let's try to bring the card out of programming
		 * state.
		 */
		if (cmd->sanitize_busy && cmd->error == -ETIMEDOUT) {
			if (!mmc_interrupt_hpi(host->card)) {
				pr_warn("%s: %s: Interrupted sanitize\n",
					mmc_hostname(host), __func__);
				cmd->error = 0;
				break;
			} else {
				pr_err("%s: %s: Failed to interrupt sanitize\n",
				       mmc_hostname(host), __func__);
			}
		}
		if (!cmd->error || !cmd->retries ||
		    mmc_card_removed(host->card))
			break;

		mmc_retune_recheck(host);

		pr_debug("%s: req failed (CMD%u): %d, retrying...\n",
			 mmc_hostname(host), cmd->opcode, cmd->error);
		cmd->retries--;
		cmd->error = 0;
		__mmc_start_request(host, mrq);
	}

	mmc_retune_release(host);
}
EXPORT_SYMBOL(mmc_wait_for_req_done);

/*
 * mmc_cqe_start_req - Start a CQE request.
 * @host: MMC host to start the request
 * @mrq: request to start
 *
 * Start the request, re-tuning if needed and it is possible. Returns an error
 * code if the request fails to start or -EBUSY if CQE is busy.
 */
int mmc_cqe_start_req(struct mmc_host *host, struct mmc_request *mrq)
{
	int err;

	/*
	 * CQE cannot process re-tuning commands. Caller must hold retuning
	 * while CQE is in use.  Re-tuning can happen here only when CQE has no
	 * active requests i.e. this is the first.  Note, re-tuning will call
	 * ->cqe_off().
	 */
	err = mmc_retune(host);
	if (err)
		goto out_err;

	mrq->host = host;

	mmc_mrq_pr_debug(host, mrq, true);

	err = mmc_mrq_prep(host, mrq);
	if (err)
		goto out_err;

	err = host->cqe_ops->cqe_request(host, mrq);

	if (err)
		goto out_err;

	trace_mmc_request_start(host, mrq);
	if(host && mrq && mrq->cmd)
		dbg_add_host_log(host, 5, mrq->cmd->opcode, mrq->cmd->arg);

	if(host && mrq && mrq->data){
		if (mrq->data->flags & MMC_DATA_WRITE)
			dbg_add_host_log(host, 5, MMC_EXECUTE_WRITE_TASK, mrq->data->blocks);//CMD47
		else if (mrq->data->flags & MMC_DATA_READ)
			dbg_add_host_log(host, 5, MMC_EXECUTE_READ_TASK, mrq->data->blocks);//CMD46
	}

	return 0;

out_err:
	if (mrq->cmd) {
		pr_info("%s: failed to start CQE direct CMD%u, error %d\n",
			 mmc_hostname(host), mrq->cmd->opcode, err);
	} else {
		pr_info("%s: failed to start CQE transfer for tag %d, error %d\n",
			 mmc_hostname(host), mrq->tag, err);
	}
	return err;
}
EXPORT_SYMBOL(mmc_cqe_start_req);

/**
 *	mmc_cqe_request_done - CQE has finished processing an MMC request
 *	@host: MMC host which completed request
 *	@mrq: MMC request which completed
 *
 *	CQE drivers should call this function when they have completed
 *	their processing of a request.
 */
void mmc_cqe_request_done(struct mmc_host *host, struct mmc_request *mrq)
{
	mmc_should_fail_request(host, mrq);

	/* Flag re-tuning needed on CRC errors */
	if ((mrq->cmd && mrq->cmd->error == -EILSEQ) ||
	    (mrq->data && mrq->data->error == -EILSEQ))
		mmc_retune_needed(host);

	trace_mmc_request_done(host, mrq);

	if (mrq->cmd) {
		pr_debug("%s: CQE req done (direct CMD%u): %d\n",
			 mmc_hostname(host), mrq->cmd->opcode, mrq->cmd->error);
		dbg_add_host_log(host, 6, mrq->cmd->opcode, mrq->cmd->resp[0]);
	} else {
		pr_debug("%s: CQE transfer done tag %d\n",
			 mmc_hostname(host), mrq->tag);
	}

	if (mrq->data){
		if (mrq->data->flags & MMC_DATA_WRITE){
			pr_debug("%s:     %d bytes transferred: %d WRITE\n",
				 mmc_hostname(host),
				 mrq->data->bytes_xfered, mrq->data->error);
			dbg_add_host_log(host, 6, MMC_EXECUTE_WRITE_TASK, mrq->data->error);//CMD47
		} else if (mrq->data->flags & MMC_DATA_READ){
			pr_debug("%s:     %d bytes transferred: %d READ\n",
				 mmc_hostname(host),
				 mrq->data->bytes_xfered, mrq->data->error);
			dbg_add_host_log(host, 6, MMC_EXECUTE_READ_TASK, mrq->data->error);//CMD46
		}
	}

	mrq->done(mrq);
}
EXPORT_SYMBOL(mmc_cqe_request_done);

/**
 *	mmc_cqe_post_req - CQE post process of a completed MMC request
 *	@host: MMC host
 *	@mrq: MMC request to be processed
 */
void mmc_cqe_post_req(struct mmc_host *host, struct mmc_request *mrq)
{
	if (host->cqe_ops->cqe_post_req)
		host->cqe_ops->cqe_post_req(host, mrq);
}
EXPORT_SYMBOL(mmc_cqe_post_req);

/* Arbitrary 1 second timeout */
#define MMC_CQE_RECOVERY_TIMEOUT	1000

/*
 * mmc_cqe_recovery - Recover from CQE errors.
 * @host: MMC host to recover
 *
 * Recovery consists of stopping CQE, stopping eMMC, discarding the queue in
 * in eMMC, and discarding the queue in CQE. CQE must call
 * mmc_cqe_request_done() on all requests. An error is returned if the eMMC
 * fails to discard its queue.
 */
int mmc_cqe_recovery(struct mmc_host *host)
{
	struct mmc_command cmd;
	int err;

	mmc_retune_hold_now(host);

	/*
	 * Recovery is expected seldom, if at all, but it reduces performance,
	 * so make sure it is not completely silent.
	 */
	pr_warn("%s: running CQE recovery\n", mmc_hostname(host));

	host->cqe_ops->cqe_recovery_start(host);

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode       = MMC_STOP_TRANSMISSION,
	cmd.flags        = MMC_RSP_R1B | MMC_CMD_AC,
	cmd.flags       &= ~MMC_RSP_CRC; /* Ignore CRC */
	cmd.busy_timeout = MMC_CQE_RECOVERY_TIMEOUT,
	mmc_wait_for_cmd(host, &cmd, 0);

	memset(&cmd, 0, sizeof(cmd));
	cmd.opcode       = MMC_CMDQ_TASK_MGMT;
	cmd.arg          = 1; /* Discard entire queue */
	cmd.flags        = MMC_RSP_R1B | MMC_CMD_AC;
	cmd.flags       &= ~MMC_RSP_CRC; /* Ignore CRC */
	cmd.busy_timeout = MMC_CQE_RECOVERY_TIMEOUT,
	err = mmc_wait_for_cmd(host, &cmd, 0);

	host->cqe_ops->cqe_recovery_finish(host);

	mmc_retune_release(host);

	return err;
}
EXPORT_SYMBOL(mmc_cqe_recovery);

/**
 *	mmc_is_req_done - Determine if a 'cap_cmd_during_tfr' request is done
 *	@host: MMC host
 *	@mrq: MMC request
 *
 *	mmc_is_req_done() is used with requests that have
 *	mrq->cap_cmd_during_tfr = true. mmc_is_req_done() must be called after
 *	starting a request and before waiting for it to complete. That is,
 *	either in between calls to mmc_start_req(), or after mmc_wait_for_req()
 *	and before mmc_wait_for_req_done(). If it is called at other times the
 *	result is not meaningful.
 */
bool mmc_is_req_done(struct mmc_host *host, struct mmc_request *mrq)
{
	return completion_done(&mrq->completion);
}
EXPORT_SYMBOL(mmc_is_req_done);

/**
 *	mmc_wait_for_req - start a request and wait for completion
 *	@host: MMC host to start command
 *	@mrq: MMC request to start
 *
 *	Start a new MMC custom command request for a host, and wait
 *	for the command to complete. In the case of 'cap_cmd_during_tfr'
 *	requests, the transfer is ongoing and the caller can issue further
 *	commands that do not use the data lines, and then wait by calling
 *	mmc_wait_for_req_done().
 *	Does not attempt to parse the response.
 */
void mmc_wait_for_req(struct mmc_host *host, struct mmc_request *mrq)
{
	__mmc_start_req(host, mrq);

	if (!mrq->cap_cmd_during_tfr)
		mmc_wait_for_req_done(host, mrq);
}
EXPORT_SYMBOL(mmc_wait_for_req);

/**
 *	mmc_wait_for_cmd - start a command and wait for completion
 *	@host: MMC host to start command
 *	@cmd: MMC command to start
 *	@retries: maximum number of retries
 *
 *	Start a new MMC command for a host, and wait for the command
 *	to complete.  Return any error that occurred while the command
 *	was executing.  Do not attempt to parse the response.
 */
int mmc_wait_for_cmd(struct mmc_host *host, struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq = {};

	WARN_ON(!host->claimed);

	memset(cmd->resp, 0, sizeof(cmd->resp));
	cmd->retries = retries;

	mrq.cmd = cmd;
	cmd->data = NULL;

	mmc_wait_for_req(host, &mrq);

	return cmd->error;
}

EXPORT_SYMBOL(mmc_wait_for_cmd);

/**
 *	mmc_set_data_timeout - set the timeout for a data command
 *	@data: data phase for command
 *	@card: the MMC card associated with the data transfer
 *
 *	Computes the data timeout parameters according to the
 *	correct algorithm given the card type.
 */
void mmc_set_data_timeout(struct mmc_data *data, const struct mmc_card *card)
{
	unsigned int mult;

	/*
	 * SDIO cards only define an upper 1 s limit on access.
	 */
	if (mmc_card_sdio(card)) {
		data->timeout_ns = 1000000000;
		data->timeout_clks = 0;
		return;
	}

	/*
	 * SD cards use a 100 multiplier rather than 10
	 */
	mult = mmc_card_sd(card) ? 100 : 10;

	/*
	 * Scale up the multiplier (and therefore the timeout) by
	 * the r2w factor for writes.
	 */
	if (data->flags & MMC_DATA_WRITE)
		mult <<= card->csd.r2w_factor;

	data->timeout_ns = card->csd.taac_ns * mult;
	data->timeout_clks = card->csd.taac_clks * mult;

	/*
	 * SD cards also have an upper limit on the timeout.
	 */
	if (mmc_card_sd(card)) {
		unsigned int timeout_us, limit_us;

		timeout_us = data->timeout_ns / 1000;
		if (card->host->ios.clock)
			timeout_us += data->timeout_clks * 1000 /
				(card->host->ios.clock / 1000);

		if (data->flags & MMC_DATA_WRITE)
			/*
			 * The MMC spec "It is strongly recommended
			 * for hosts to implement more than 500ms
			 * timeout value even if the card indicates
			 * the 250ms maximum busy length."  Even the
			 * previous value of 300ms is known to be
			 * insufficient for some cards.
			 */
			limit_us = 3000000;
		else
			limit_us = 100000;

		/*
		 * SDHC cards always use these fixed values.
		 */
		if (timeout_us > limit_us) {
			data->timeout_ns = limit_us * 1000;
			data->timeout_clks = 0;
		}

		/* assign limit value if invalid */
		if (timeout_us == 0)
			data->timeout_ns = limit_us * 1000;
	}

	/*
	 * Some cards require longer data read timeout than indicated in CSD.
	 * Address this by setting the read timeout to a "reasonably high"
	 * value. For the cards tested, 600ms has proven enough. If necessary,
	 * this value can be increased if other problematic cards require this.
	 */
	if (mmc_card_long_read_time(card) && data->flags & MMC_DATA_READ) {
		data->timeout_ns = 600000000;
		data->timeout_clks = 0;
	}

	/*
	 * Some cards need very high timeouts if driven in SPI mode.
	 * The worst observed timeout was 900ms after writing a
	 * continuous stream of data until the internal logic
	 * overflowed.
	 */
	if (mmc_host_is_spi(card->host)) {
		if (data->flags & MMC_DATA_WRITE) {
			if (data->timeout_ns < 1000000000)
				data->timeout_ns = 1000000000;	/* 1s */
		} else {
			if (data->timeout_ns < 100000000)
				data->timeout_ns =  100000000;	/* 100ms */
		}
	}
}
EXPORT_SYMBOL(mmc_set_data_timeout);

/**
 *	mmc_align_data_size - pads a transfer size to a more optimal value
 *	@card: the MMC card associated with the data transfer
 *	@sz: original transfer size
 *
 *	Pads the original data size with a number of extra bytes in
 *	order to avoid controller bugs and/or performance hits
 *	(e.g. some controllers revert to PIO for certain sizes).
 *
 *	Returns the improved size, which might be unmodified.
 *
 *	Note that this function is only relevant when issuing a
 *	single scatter gather entry.
 */
unsigned int mmc_align_data_size(struct mmc_card *card, unsigned int sz)
{
	/*
	 * FIXME: We don't have a system for the controller to tell
	 * the core about its problems yet, so for now we just 32-bit
	 * align the size.
	 */
	sz = ((sz + 3) / 4) * 4;

	return sz;
}
EXPORT_SYMBOL(mmc_align_data_size);

/*
 * Allow claiming an already claimed host if the context is the same or there is
 * no context but the task is the same.
 */
static inline bool mmc_ctx_matches(struct mmc_host *host, struct mmc_ctx *ctx,
				   struct task_struct *task)
{
	return host->claimer == ctx ||
	       (!ctx && task && host->claimer->task == task);
}

static inline void mmc_ctx_set_claimer(struct mmc_host *host,
				       struct mmc_ctx *ctx,
				       struct task_struct *task)
{
	if (!host->claimer) {
		if (ctx)
			host->claimer = ctx;
		else
			host->claimer = &host->default_ctx;
	}
	if (task)
		host->claimer->task = task;
}

/**
 *	__mmc_claim_host - exclusively claim a host
 *	@host: mmc host to claim
 *	@ctx: context that claims the host or NULL in which case the default
 *	context will be used
 *	@abort: whether or not the operation should be aborted
 *
 *	Claim a host for a set of operations.  If @abort is non null and
 *	dereference a non-zero value then this will return prematurely with
 *	that non-zero value without acquiring the lock.  Returns zero
 *	with the lock held otherwise.
 */
int __mmc_claim_host(struct mmc_host *host, struct mmc_ctx *ctx,
		     atomic_t *abort)
{
	struct task_struct *task = ctx ? NULL : current;
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int stop;
	bool pm = false;

	might_sleep();

	add_wait_queue(&host->wq, &wait);
	spin_lock_irqsave(&host->lock, flags);
	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		stop = abort ? atomic_read(abort) : 0;
		if (stop || !host->claimed || mmc_ctx_matches(host, ctx, task))
			break;
		spin_unlock_irqrestore(&host->lock, flags);
		schedule();
		spin_lock_irqsave(&host->lock, flags);
	}
	set_current_state(TASK_RUNNING);
	if (!stop) {
		host->claimed = 1;
		mmc_ctx_set_claimer(host, ctx, task);
		host->claim_cnt += 1;
		if (host->claim_cnt == 1)
			pm = true;
	} else
		wake_up(&host->wq);
	spin_unlock_irqrestore(&host->lock, flags);
	remove_wait_queue(&host->wq, &wait);

	if (pm)
		pm_runtime_get_sync(mmc_dev(host));

	return stop;
}
EXPORT_SYMBOL(__mmc_claim_host);

/*
 *     mmc_try_claim_host - try exclusively to claim a host
 *        and keep trying for given time, with a gap of 10ms
 *     @host: mmc host to claim
 *     @dealy_ms: delay in ms
 *
 *     Returns %1 if the host is claimed, %0 otherwise.
 */
int mmc_try_claim_host(struct mmc_host *host, struct mmc_ctx *ctx, unsigned int delay_ms)
{
	struct task_struct *task = ctx ? NULL : current;
	int claimed_host = 0;
	unsigned long flags;
	int retry_cnt = delay_ms/10;
	bool pm = false;

	do {
		spin_lock_irqsave(&host->lock, flags);
		if (!host->claimed || mmc_ctx_matches(host, ctx, task)) {
			host->claimed = 1;
			mmc_ctx_set_claimer(host, ctx, task);
			host->claim_cnt += 1;
			claimed_host = 1;
			if (host->claim_cnt == 1)
				pm = true;
		}
		spin_unlock_irqrestore(&host->lock, flags);
		if (!claimed_host)
			mmc_delay(10);
	} while (!claimed_host && retry_cnt--);

	if (pm)
		pm_runtime_get_sync(mmc_dev(host));

	return claimed_host;
}
EXPORT_SYMBOL(mmc_try_claim_host);

/**
 *	mmc_release_host - release a host
 *	@host: mmc host to release
 *
 *	Release a MMC host, allowing others to claim the host
 *	for their operations.
 */
void mmc_release_host(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->claimed);

	spin_lock_irqsave(&host->lock, flags);
	if (--host->claim_cnt) {
		/* Release for nested claim */
		spin_unlock_irqrestore(&host->lock, flags);
	} else {
		host->claimed = 0;
		host->claimer->task = NULL;
		host->claimer = NULL;
		spin_unlock_irqrestore(&host->lock, flags);
		wake_up(&host->wq);
		pm_runtime_mark_last_busy(mmc_dev(host));
		pm_runtime_put_autosuspend(mmc_dev(host));
	}
}
EXPORT_SYMBOL(mmc_release_host);

/*
 * This is a helper function, which fetches a runtime pm reference for the
 * card device and also claims the host.
 */
void mmc_get_card(struct mmc_card *card, struct mmc_ctx *ctx)
{
	pm_runtime_get_sync(&card->dev);
	__mmc_claim_host(card->host, ctx, NULL);
}
EXPORT_SYMBOL(mmc_get_card);

/*
 * This is a helper function, which releases the host and drops the runtime
 * pm reference for the card device.
 */
void mmc_put_card(struct mmc_card *card, struct mmc_ctx *ctx)
{
	struct mmc_host *host = card->host;

	WARN_ON(ctx && host->claimer != ctx);

	mmc_release_host(host);
	pm_runtime_mark_last_busy(&card->dev);
	pm_runtime_put_autosuspend(&card->dev);
}
EXPORT_SYMBOL(mmc_put_card);

/*
 * Internal function that does the actual ios call to the host driver,
 * optionally printing some debug output.
 */
static inline void mmc_set_ios(struct mmc_host *host)
{
	struct mmc_ios *ios = &host->ios;

	dev_info(host->parent, "%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(host), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 1 << ios->bus_width, ios->timing);

	host->ops->set_ios(host, ios);
}

/*
 * Control chip select pin on a host.
 */
void mmc_set_chip_select(struct mmc_host *host, int mode)
{
	host->ios.chip_select = mode;
	mmc_set_ios(host);
}

/*
 * Sets the host clock to the highest possible frequency that
 * is below "hz".
 */
void mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	WARN_ON(hz && hz < host->f_min);

	if (hz > host->f_max)
		hz = host->f_max;

	host->ios.clock = hz;
	mmc_set_ios(host);
}

int mmc_execute_tuning(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u32 opcode;
	int err;

	if (!host->ops->execute_tuning)
		return 0;

	if (host->cqe_on)
		host->cqe_ops->cqe_off(host);

	if (mmc_card_mmc(card))
		opcode = MMC_SEND_TUNING_BLOCK_HS200;
	else
		opcode = MMC_SEND_TUNING_BLOCK;

	err = host->ops->execute_tuning(host, opcode);

	if (err) {
		pr_info("%s: tuning execution failed: %d\n",
			mmc_hostname(host), err);
	} else {
		pr_info("%s: tuning execution ok: %d\n",
			mmc_hostname(host), err);
		mmc_retune_enable(host);
	}

	return err;
}

/*
 * Change the bus mode (open drain/push-pull) of a host.
 */
void mmc_set_bus_mode(struct mmc_host *host, unsigned int mode)
{
	host->ios.bus_mode = mode;
	mmc_set_ios(host);
}

/*
 * Change data bus width of a host.
 */
void mmc_set_bus_width(struct mmc_host *host, unsigned int width)
{
	host->ios.bus_width = width;
	mmc_set_ios(host);
}

/*
 * Set initial state after a power cycle or a hw_reset.
 */
void mmc_set_initial_state(struct mmc_host *host)
{
	if (host->cqe_on)
		host->cqe_ops->cqe_off(host);

	mmc_retune_disable(host);

	if (mmc_host_is_spi(host))
		host->ios.chip_select = MMC_CS_HIGH;
	else
		host->ios.chip_select = MMC_CS_DONTCARE;
	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	host->ios.drv_type = 0;
	host->ios.enhanced_strobe = false;

	/*
	 * Make sure we are in non-enhanced strobe mode before we
	 * actually enable it in ext_csd.
	 */
	if ((host->caps2 & MMC_CAP2_HS400_ES) &&
	     host->ops->hs400_enhanced_strobe)
		host->ops->hs400_enhanced_strobe(host, &host->ios);

	mmc_set_ios(host);
}

/**
 * mmc_vdd_to_ocrbitnum - Convert a voltage to the OCR bit number
 * @vdd:	voltage (mV)
 * @low_bits:	prefer low bits in boundary cases
 *
 * This function returns the OCR bit number according to the provided @vdd
 * value. If conversion is not possible a negative errno value returned.
 *
 * Depending on the @low_bits flag the function prefers low or high OCR bits
 * on boundary voltages. For example,
 * with @low_bits = true, 3300 mV translates to ilog2(MMC_VDD_32_33);
 * with @low_bits = false, 3300 mV translates to ilog2(MMC_VDD_33_34);
 *
 * Any value in the [1951:1999] range translates to the ilog2(MMC_VDD_20_21).
 */
static int mmc_vdd_to_ocrbitnum(int vdd, bool low_bits)
{
	const int max_bit = ilog2(MMC_VDD_35_36);
	int bit;

	if (vdd < 1650 || vdd > 3600)
		return -EINVAL;

	if (vdd >= 1650 && vdd <= 1950)
		return ilog2(MMC_VDD_165_195);

	if (low_bits)
		vdd -= 1;

	/* Base 2000 mV, step 100 mV, bit's base 8. */
	bit = (vdd - 2000) / 100 + 8;
	if (bit > max_bit)
		return max_bit;
	return bit;
}

/**
 * mmc_vddrange_to_ocrmask - Convert a voltage range to the OCR mask
 * @vdd_min:	minimum voltage value (mV)
 * @vdd_max:	maximum voltage value (mV)
 *
 * This function returns the OCR mask bits according to the provided @vdd_min
 * and @vdd_max values. If conversion is not possible the function returns 0.
 *
 * Notes wrt boundary cases:
 * This function sets the OCR bits for all boundary voltages, for example
 * [3300:3400] range is translated to MMC_VDD_32_33 | MMC_VDD_33_34 |
 * MMC_VDD_34_35 mask.
 */
u32 mmc_vddrange_to_ocrmask(int vdd_min, int vdd_max)
{
	u32 mask = 0;

	if (vdd_max < vdd_min)
		return 0;

	/* Prefer high bits for the boundary vdd_max values. */
	vdd_max = mmc_vdd_to_ocrbitnum(vdd_max, false);
	if (vdd_max < 0)
		return 0;

	/* Prefer low bits for the boundary vdd_min values. */
	vdd_min = mmc_vdd_to_ocrbitnum(vdd_min, true);
	if (vdd_min < 0)
		return 0;

	/* Fill the mask, from max bit to min bit. */
	while (vdd_max >= vdd_min)
		mask |= 1 << vdd_max--;

	return mask;
}
EXPORT_SYMBOL(mmc_vddrange_to_ocrmask);

#ifdef CONFIG_OF

/**
 * mmc_of_parse_voltage - return mask of supported voltages
 * @np: The device node need to be parsed.
 * @mask: mask of voltages available for MMC/SD/SDIO
 *
 * Parse the "voltage-ranges" DT property, returning zero if it is not
 * found, negative errno if the voltage-range specification is invalid,
 * or one if the voltage-range is specified and successfully parsed.
 */
int mmc_of_parse_voltage(struct device_node *np, u32 *mask)
{
	const u32 *voltage_ranges;
	int num_ranges, i;

	voltage_ranges = of_get_property(np, "voltage-ranges", &num_ranges);
	num_ranges = num_ranges / sizeof(*voltage_ranges) / 2;
	if (!voltage_ranges) {
		pr_debug("%pOF: voltage-ranges unspecified\n", np);
		return 0;
	}
	if (!num_ranges) {
		pr_err("%pOF: voltage-ranges empty\n", np);
		return -EINVAL;
	}

	for (i = 0; i < num_ranges; i++) {
		const int j = i * 2;
		u32 ocr_mask;

		ocr_mask = mmc_vddrange_to_ocrmask(
				be32_to_cpu(voltage_ranges[j]),
				be32_to_cpu(voltage_ranges[j + 1]));
		if (!ocr_mask) {
			pr_err("%pOF: voltage-range #%d is invalid\n",
				np, i);
			return -EINVAL;
		}
		*mask |= ocr_mask;
	}

	return 1;
}
EXPORT_SYMBOL(mmc_of_parse_voltage);

#endif /* CONFIG_OF */

static int mmc_of_get_func_num(struct device_node *node)
{
	u32 reg;
	int ret;

	ret = of_property_read_u32(node, "reg", &reg);
	if (ret < 0)
		return ret;

	return reg;
}

struct device_node *mmc_of_find_child_device(struct mmc_host *host,
		unsigned func_num)
{
	struct device_node *node;

	if (!host->parent || !host->parent->of_node)
		return NULL;

	for_each_child_of_node(host->parent->of_node, node) {
		if (mmc_of_get_func_num(node) == func_num)
			return node;
	}

	return NULL;
}

#ifdef CONFIG_REGULATOR

/**
 * mmc_ocrbitnum_to_vdd - Convert a OCR bit number to its voltage
 * @vdd_bit:	OCR bit number
 * @min_uV:	minimum voltage value (mV)
 * @max_uV:	maximum voltage value (mV)
 *
 * This function returns the voltage range according to the provided OCR
 * bit number. If conversion is not possible a negative errno value returned.
 */
static int mmc_ocrbitnum_to_vdd(int vdd_bit, int *min_uV, int *max_uV)
{
	int		tmp;

	if (!vdd_bit)
		return -EINVAL;

	/*
	 * REVISIT mmc_vddrange_to_ocrmask() may have set some
	 * bits this regulator doesn't quite support ... don't
	 * be too picky, most cards and regulators are OK with
	 * a 0.1V range goof (it's a small error percentage).
	 */
	tmp = vdd_bit - ilog2(MMC_VDD_165_195);
	if (tmp == 0) {
		*min_uV = 1650 * 1000;
		*max_uV = 1950 * 1000;
	} else {
		*min_uV = 1900 * 1000 + tmp * 100 * 1000;
		*max_uV = *min_uV + 100 * 1000;
	}

	return 0;
}

/**
 * mmc_regulator_get_ocrmask - return mask of supported voltages
 * @supply: regulator to use
 *
 * This returns either a negative errno, or a mask of voltages that
 * can be provided to MMC/SD/SDIO devices using the specified voltage
 * regulator.  This would normally be called before registering the
 * MMC host adapter.
 */
int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	int			result = 0;
	int			count;
	int			i;
	int			vdd_uV;
	int			vdd_mV;

	count = regulator_count_voltages(supply);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		vdd_uV = regulator_list_voltage(supply, i);
		if (vdd_uV <= 0)
			continue;

		vdd_mV = vdd_uV / 1000;
		result |= mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	if (!result) {
		vdd_uV = regulator_get_voltage(supply);
		if (vdd_uV <= 0)
			return vdd_uV;

		vdd_mV = vdd_uV / 1000;
		result = mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_ocrmask);

/**
 * mmc_regulator_set_ocr - set regulator to match host->ios voltage
 * @mmc: the host to regulate
 * @supply: regulator to use
 * @vdd_bit: zero for power off, else a bit number (host->ios.vdd)
 *
 * Returns zero on success, else negative errno.
 *
 * MMC host drivers may use this to enable or disable a regulator using
 * a particular supply voltage.  This would normally be called from the
 * set_ios() method.
 */
int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit)
{
	int			result = 0;
	int			min_uV, max_uV;

	if (vdd_bit) {
		mmc_ocrbitnum_to_vdd(vdd_bit, &min_uV, &max_uV);

		result = regulator_set_voltage(supply, min_uV, max_uV);
		if (result == 0 && !mmc->regulator_enabled) {
			result = regulator_enable(supply);
			if (!result)
				mmc->regulator_enabled = true;
		}
	} else if (mmc->regulator_enabled) {
		result = regulator_disable(supply);
		if (result == 0)
			mmc->regulator_enabled = false;
	}

	if (result)
		dev_err(mmc_dev(mmc),
			"could not set regulator OCR (%d)\n", result);
	return result;
}
EXPORT_SYMBOL_GPL(mmc_regulator_set_ocr);

static int mmc_regulator_set_voltage_if_supported(struct regulator *regulator,
						  int min_uV, int target_uV,
						  int max_uV)
{
	/*
	 * Check if supported first to avoid errors since we may try several
	 * signal levels during power up and don't want to show errors.
	 */
	if (!regulator_is_supported_voltage(regulator, min_uV, max_uV))
		return -EINVAL;

	return regulator_set_voltage_triplet(regulator, min_uV, target_uV,
					     max_uV);
}

/**
 * mmc_regulator_set_vqmmc - Set VQMMC as per the ios
 *
 * For 3.3V signaling, we try to match VQMMC to VMMC as closely as possible.
 * That will match the behavior of old boards where VQMMC and VMMC were supplied
 * by the same supply.  The Bus Operating conditions for 3.3V signaling in the
 * SD card spec also define VQMMC in terms of VMMC.
 * If this is not possible we'll try the full 2.7-3.6V of the spec.
 *
 * For 1.2V and 1.8V signaling we'll try to get as close as possible to the
 * requested voltage.  This is definitely a good idea for UHS where there's a
 * separate regulator on the card that's trying to make 1.8V and it's best if
 * we match.
 *
 * This function is expected to be used by a controller's
 * start_signal_voltage_switch() function.
 */
int mmc_regulator_set_vqmmc(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct device *dev = mmc_dev(mmc);
	int ret, volt, min_uV, max_uV;

	/* If no vqmmc supply then we can't change the voltage */
	if (IS_ERR(mmc->supply.vqmmc))
		return -EINVAL;

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_120:
		return mmc_regulator_set_voltage_if_supported(mmc->supply.vqmmc,
						1100000, 1200000, 1300000);
	case MMC_SIGNAL_VOLTAGE_180:
		return mmc_regulator_set_voltage_if_supported(mmc->supply.vqmmc,
						1700000, 1800000, 1950000);
	case MMC_SIGNAL_VOLTAGE_330:
		ret = mmc_ocrbitnum_to_vdd(mmc->ios.vdd, &volt, &max_uV);
		if (ret < 0)
			return ret;

		dev_dbg(dev, "%s: found vmmc voltage range of %d-%duV\n",
			__func__, volt, max_uV);

		min_uV = max(volt - 300000, 2700000);
		max_uV = min(max_uV + 200000, 3600000);

		/*
		 * Due to a limitation in the current implementation of
		 * regulator_set_voltage_triplet() which is taking the lowest
		 * voltage possible if below the target, search for a suitable
		 * voltage in two steps and try to stay close to vmmc
		 * with a 0.3V tolerance at first.
		 */
		if (!mmc_regulator_set_voltage_if_supported(mmc->supply.vqmmc,
						min_uV, volt, max_uV))
			return 0;

		return mmc_regulator_set_voltage_if_supported(mmc->supply.vqmmc,
						2700000, volt, 3600000);
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL_GPL(mmc_regulator_set_vqmmc);

#endif /* CONFIG_REGULATOR */

/**
 * mmc_regulator_get_supply - try to get VMMC and VQMMC regulators for a host
 * @mmc: the host to regulate
 *
 * Returns 0 or errno. errno should be handled, it is either a critical error
 * or -EPROBE_DEFER. 0 means no critical error but it does not mean all
 * regulators have been found because they all are optional. If you require
 * certain regulators, you need to check separately in your driver if they got
 * populated after calling this function.
 */
int mmc_regulator_get_supply(struct mmc_host *mmc)
{
	struct device *dev = mmc_dev(mmc);
	int ret;

	mmc->supply.vmmc = devm_regulator_get_optional(dev, "vmmc");
	mmc->supply.vqmmc = devm_regulator_get_optional(dev, "vqmmc");

	if (IS_ERR(mmc->supply.vmmc)) {
		if (PTR_ERR(mmc->supply.vmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_dbg(dev, "No vmmc regulator found\n");
	} else {
		ret = mmc_regulator_get_ocrmask(mmc->supply.vmmc);
		if (ret > 0)
			mmc->ocr_avail = ret;
		else
			dev_warn(dev, "Failed getting OCR mask: %d\n", ret);
	}

	if (IS_ERR(mmc->supply.vqmmc)) {
		if (PTR_ERR(mmc->supply.vqmmc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_dbg(dev, "No vqmmc regulator found\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_regulator_get_supply);

/*
 * Mask off any voltages we don't support and select
 * the lowest voltage
 */
u32 mmc_select_voltage(struct mmc_host *host, u32 ocr)
{
	int bit;

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		dev_warn(mmc_dev(host),
		"card claims to support voltages below defined range\n");
		ocr &= ~0x7F;
	}

	ocr &= host->ocr_avail;
	if (!ocr) {
		dev_warn(mmc_dev(host), "no support for card's volts\n");
		return 0;
	}

	if (host->caps2 & MMC_CAP2_FULL_PWR_CYCLE) {
		bit = ffs(ocr) - 1;
		ocr &= 3 << bit;
		mmc_power_cycle(host, ocr);
	} else {
		bit = fls(ocr) - 1;
		ocr &= 3 << bit;
		if (bit != host->ios.vdd)
			dev_warn(mmc_dev(host), "exceeding card's volts\n");
	}

	return ocr;
}

int mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage)
{
	int err = 0;
	int old_signal_voltage = host->ios.signal_voltage;

	host->ios.signal_voltage = signal_voltage;
	if (host->ops->start_signal_voltage_switch)
		err = host->ops->start_signal_voltage_switch(host, &host->ios);

	if (err)
		host->ios.signal_voltage = old_signal_voltage;

	return err;

}

void mmc_set_initial_signal_voltage(struct mmc_host *host)
{
	/* Try to set signal voltage to 3.3V but fall back to 1.8v or 1.2v */
	if (!mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330))
		dev_dbg(mmc_dev(host), "Initial signal voltage of 3.3v\n");
	else if (!mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180))
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.8v\n");
	else if (!mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120))
		dev_dbg(mmc_dev(host), "Initial signal voltage of 1.2v\n");
}

int mmc_host_set_uhs_voltage(struct mmc_host *host)
{
	u32 clock;

	/*
	 * During a signal voltage level switch, the clock must be gated
	 * for 5 ms according to the SD spec
	 */
	clock = host->ios.clock;
	host->ios.clock = 0;
	mmc_set_ios(host);

	if (mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180))
		return -EAGAIN;

	/* Keep clock gated for at least 10 ms, though spec only says 5 ms */
	mmc_delay(10);
	host->ios.clock = clock;
	mmc_set_ios(host);

	return 0;
}

int mmc_set_uhs_voltage(struct mmc_host *host, u32 ocr)
{
	struct mmc_command cmd = {};
	int err = 0;

	/*
	 * If we cannot switch voltages, return failure so the caller
	 * can continue without UHS mode
	 */
	if (!host->ops->start_signal_voltage_switch)
		return -EPERM;
	if (!host->ops->card_busy)
		pr_warn("%s: cannot verify signal voltage switch\n",
			mmc_hostname(host));

	cmd.opcode = SD_SWITCH_VOLTAGE;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err)
		goto power_cycle;

	if (!mmc_host_is_spi(host) && (cmd.resp[0] & R1_ERROR))
		return -EIO;

	/*
	 * The card should drive cmd and dat[0:3] low immediately
	 * after the response of cmd11, but wait 1 ms to be sure
	 */
	mmc_delay(1);
	if (host->ops->card_busy && !host->ops->card_busy(host)) {
		err = -EAGAIN;
		goto power_cycle;
	}

	if (mmc_host_set_uhs_voltage(host)) {
		/*
		 * Voltages may not have been switched, but we've already
		 * sent CMD11, so a power cycle is required anyway
		 */
		err = -EAGAIN;
		goto power_cycle;
	}

	/* Wait for at least 1 ms according to spec */
	mmc_delay(1);

	/*
	 * Failure to switch is indicated by the card holding
	 * dat[0:3] low
	 */
	if (host->ops->card_busy && host->ops->card_busy(host))
		err = -EAGAIN;

power_cycle:
	if (err) {
		pr_debug("%s: Signal voltage switch failed, "
			"power cycling card\n", mmc_hostname(host));
		mmc_power_cycle(host, ocr);
	}

	return err;
}

/*
 * Select timing parameters for host.
 */
void mmc_set_timing(struct mmc_host *host, unsigned int timing)
{
	host->ios.timing = timing;
	mmc_set_ios(host);
}

/*
 * Select appropriate driver type for host.
 */
void mmc_set_driver_type(struct mmc_host *host, unsigned int drv_type)
{
	host->ios.drv_type = drv_type;
	mmc_set_ios(host);
}

int mmc_select_drive_strength(struct mmc_card *card, unsigned int max_dtr,
			      int card_drv_type, int *drv_type)
{
	struct mmc_host *host = card->host;
	int host_drv_type = SD_DRIVER_TYPE_B;

	*drv_type = 0;

	if (!host->ops->select_drive_strength)
		return 0;

	/* Use SD definition of driver strength for hosts */
	if (host->caps & MMC_CAP_DRIVER_TYPE_A)
		host_drv_type |= SD_DRIVER_TYPE_A;

	if (host->caps & MMC_CAP_DRIVER_TYPE_C)
		host_drv_type |= SD_DRIVER_TYPE_C;

	if (host->caps & MMC_CAP_DRIVER_TYPE_D)
		host_drv_type |= SD_DRIVER_TYPE_D;

	/*
	 * The drive strength that the hardware can support
	 * depends on the board design.  Pass the appropriate
	 * information and let the hardware specific code
	 * return what is possible given the options
	 */
	return host->ops->select_drive_strength(card, max_dtr,
						host_drv_type,
						card_drv_type,
						drv_type);
}

/*
 * Apply power to the MMC stack.  This is a two-stage process.
 * First, we enable power to the card without the clock running.
 * We then wait a bit for the power to stabilise.  Finally,
 * enable the bus drivers and clock to the card.
 *
 * We must _NOT_ enable the clock prior to power stablising.
 *
 * If a host does all the power sequencing itself, ignore the
 * initial MMC_POWER_UP stage.
 */
void mmc_power_up(struct mmc_host *host, u32 ocr)
{
	if (host->ios.power_mode == MMC_POWER_ON)
		return;

	if (host->fast_power_off && (mmc_gpio_get_status(host) == 0)) {
		pr_err("%s: do not power up in fast_power_off mode\n", mmc_hostname(host));
		return;
	}

	mmc_pwrseq_pre_power_on(host);

	host->ios.vdd = fls(ocr) - 1;
	host->ios.power_mode = MMC_POWER_UP;
	/* Set initial state and call mmc_set_ios */
	mmc_set_initial_state(host);

	mmc_set_initial_signal_voltage(host);

	/*
	 * This delay should be sufficient to allow the power supply
	 * to reach the minimum voltage.
	 */
	mmc_delay(host->ios.power_delay_ms);

	mmc_pwrseq_post_power_on(host);

	host->ios.clock = host->f_init;

	host->ios.power_mode = MMC_POWER_ON;
	mmc_set_ios(host);

	/*
	 * This delay must be at least 74 clock sizes, or 1 ms, or the
	 * time required to reach a stable voltage.
	 */
	mmc_delay(host->ios.power_delay_ms);
}

void mmc_power_off(struct mmc_host *host)
{
	if (host->ios.power_mode == MMC_POWER_OFF)
		return;

	mmc_pwrseq_power_off(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;

	host->ios.power_mode = MMC_POWER_OFF;
	/* Set initial state and call mmc_set_ios */
	mmc_set_initial_state(host);

	/*
	 * Some configurations, such as the 802.11 SDIO card in the OLPC
	 * XO-1.5, require a short delay after poweroff before the card
	 * can be successfully turned on again.
	 */
	mmc_delay(1);
}

void mmc_power_cycle(struct mmc_host *host, u32 ocr)
{
	mmc_power_off(host);
	/* Wait at least 1 ms according to SD spec */
	mmc_delay(1);
	mmc_power_up(host, ocr);
}

/*
 * Cleanup when the last reference to the bus operator is dropped.
 */
static void __mmc_release_bus(struct mmc_host *host)
{
	WARN_ON(!host->bus_dead);

	host->bus_ops = NULL;
}

/*
 * Increase reference count of bus operator
 */
static inline void mmc_bus_get(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs++;
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Decrease reference count of bus operator and free it if
 * it is the last reference.
 */
static inline void mmc_bus_put(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs--;
	if ((host->bus_refs == 0) && host->bus_ops)
		__mmc_release_bus(host);
	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Assign a mmc bus handler to a host. Only one bus handler may control a
 * host at any given time.
 */
void mmc_attach_bus(struct mmc_host *host, const struct mmc_bus_ops *ops)
{
	unsigned long flags;

	WARN_ON(!host->claimed);

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->bus_ops);
	WARN_ON(host->bus_refs);

	host->bus_ops = ops;
	host->bus_refs = 1;
	host->bus_dead = 0;

	spin_unlock_irqrestore(&host->lock, flags);
}

/*
 * Remove the current bus handler from a host.
 */
void mmc_detach_bus(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->claimed);
	WARN_ON(!host->bus_ops);

	spin_lock_irqsave(&host->lock, flags);

	host->bus_dead = 1;

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_put(host);
}

static void _mmc_detect_change(struct mmc_host *host, unsigned long delay,
				bool cd_irq)
{
	int ret = 0;
	/*
	 * If the device is configured as wakeup, we prevent a new sleep for
	 * 5 s to give provision for user space to consume the event.
	 */
	if (cd_irq && !(host->caps & MMC_CAP_NEEDS_POLL) &&
		device_can_wakeup(mmc_dev(host)))
		pm_wakeup_event(mmc_dev(host), 5000);

	host->detect_change = 1;
	ret = mmc_schedule_delayed_work(&host->detect, delay);
	printk(KERN_ERR "%s: mmc_schedule_delayed_work ret=%d delay=%ld\n", mmc_hostname(host), ret, delay);
}

/**
 *	mmc_detect_change - process change of state on a MMC socket
 *	@host: host which changed state.
 *	@delay: optional delay to wait before detection (jiffies)
 *
 *	MMC drivers should call this when they detect a card has been
 *	inserted or removed. The MMC layer will confirm that any
 *	present card is still functional, and initialize any newly
 *	inserted.
 */
void mmc_detect_change(struct mmc_host *host, unsigned long delay)
{
	_mmc_detect_change(host, delay, true);
}
EXPORT_SYMBOL(mmc_detect_change);

void mmc_init_erase(struct mmc_card *card)
{
	unsigned int sz;

	if (is_power_of_2(card->erase_size))
		card->erase_shift = ffs(card->erase_size) - 1;
	else
		card->erase_shift = 0;

	/*
	 * It is possible to erase an arbitrarily large area of an SD or MMC
	 * card.  That is not desirable because it can take a long time
	 * (minutes) potentially delaying more important I/O, and also the
	 * timeout calculations become increasingly hugely over-estimated.
	 * Consequently, 'pref_erase' is defined as a guide to limit erases
	 * to that size and alignment.
	 *
	 * For SD cards that define Allocation Unit size, limit erases to one
	 * Allocation Unit at a time.
	 * For MMC, have a stab at ai good value and for modern cards it will
	 * end up being 4MiB. Note that if the value is too small, it can end
	 * up taking longer to erase. Also note, erase_size is already set to
	 * High Capacity Erase Size if available when this function is called.
	 */
	if (mmc_card_sd(card) && card->ssr.au) {
		card->pref_erase = card->ssr.au;
		card->erase_shift = ffs(card->ssr.au) - 1;
	} else if (card->erase_size) {
		sz = (card->csd.capacity << (card->csd.read_blkbits - 9)) >> 11;
		if (sz < 128)
			card->pref_erase = 512 * 1024 / 512;
		else if (sz < 512)
			card->pref_erase = 1024 * 1024 / 512;
		else if (sz < 1024)
			card->pref_erase = 2 * 1024 * 1024 / 512;
		else
			card->pref_erase = 4 * 1024 * 1024 / 512;
		if (card->pref_erase < card->erase_size)
			card->pref_erase = card->erase_size;
		else {
			sz = card->pref_erase % card->erase_size;
			if (sz)
				card->pref_erase += card->erase_size - sz;
		}
	} else
		card->pref_erase = 0;
}

static unsigned int mmc_mmc_erase_timeout(struct mmc_card *card,
				          unsigned int arg, unsigned int qty)
{
	unsigned int erase_timeout;

	if (arg == MMC_DISCARD_ARG ||
	    (arg == MMC_TRIM_ARG && card->ext_csd.rev >= 6)) {
		erase_timeout = card->ext_csd.trim_timeout;
	} else if (card->ext_csd.erase_group_def & 1) {
		/* High Capacity Erase Group Size uses HC timeouts */
		if (arg == MMC_TRIM_ARG)
			erase_timeout = card->ext_csd.trim_timeout;
		else
			erase_timeout = card->ext_csd.hc_erase_timeout;
	} else {
		/* CSD Erase Group Size uses write timeout */
		unsigned int mult = (10 << card->csd.r2w_factor);
		unsigned int timeout_clks = card->csd.taac_clks * mult;
		unsigned int timeout_us;

		/* Avoid overflow: e.g. taac_ns=80000000 mult=1280 */
		if (card->csd.taac_ns < 1000000)
			timeout_us = (card->csd.taac_ns * mult) / 1000;
		else
			timeout_us = (card->csd.taac_ns / 1000) * mult;

		/*
		 * ios.clock is only a target.  The real clock rate might be
		 * less but not that much less, so fudge it by multiplying by 2.
		 */
		timeout_clks <<= 1;
		timeout_us += (timeout_clks * 1000) /
			      (card->host->ios.clock / 1000);

		erase_timeout = timeout_us / 1000;

		/*
		 * Theoretically, the calculation could underflow so round up
		 * to 1ms in that case.
		 */
		if (!erase_timeout)
			erase_timeout = 1;
	}

	/* Multiplier for secure operations */
	if (arg & MMC_SECURE_ARGS) {
		if (arg == MMC_SECURE_ERASE_ARG)
			erase_timeout *= card->ext_csd.sec_erase_mult;
		else
			erase_timeout *= card->ext_csd.sec_trim_mult;
	}

	erase_timeout *= qty;

	/*
	 * Ensure at least a 1 second timeout for SPI as per
	 * 'mmc_set_data_timeout()'
	 */
	if (mmc_host_is_spi(card->host) && erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_sd_erase_timeout(struct mmc_card *card,
					 unsigned int arg,
					 unsigned int qty)
{
	unsigned int erase_timeout;

	if (card->ssr.erase_timeout) {
		/* Erase timeout specified in SD Status Register (SSR) */
		erase_timeout = card->ssr.erase_timeout * qty +
				card->ssr.erase_offset;
	} else {
		/*
		 * Erase timeout not specified in SD Status Register (SSR) so
		 * use 250ms per write block.
		 */
		erase_timeout = 250 * qty;
	}

	/* Must not be less than 1 second */
	if (erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_erase_timeout(struct mmc_card *card,
				      unsigned int arg,
				      unsigned int qty)
{
	if (mmc_card_sd(card))
		return mmc_sd_erase_timeout(card, arg, qty);
	else
		return mmc_mmc_erase_timeout(card, arg, qty);
}

static int mmc_do_erase(struct mmc_card *card, unsigned int from,
			unsigned int to, unsigned int arg)
{
	struct mmc_command cmd = {};
	unsigned int qty = 0, busy_timeout = 0;
	bool use_r1b_resp = false;
	unsigned long timeout;
	int loop_udelay=64, udelay_max=32768;
	int err;

	mmc_retune_hold(card->host);

	/*
	 * qty is used to calculate the erase timeout which depends on how many
	 * erase groups (or allocation units in SD terminology) are affected.
	 * We count erasing part of an erase group as one erase group.
	 * For SD, the allocation units are always a power of 2.  For MMC, the
	 * erase group size is almost certainly also power of 2, but it does not
	 * seem to insist on that in the JEDEC standard, so we fall back to
	 * division in that case.  SD may not specify an allocation unit size,
	 * in which case the timeout is based on the number of write blocks.
	 *
	 * Note that the timeout for secure trim 2 will only be correct if the
	 * number of erase groups specified is the same as the total of all
	 * preceding secure trim 1 commands.  Since the power may have been
	 * lost since the secure trim 1 commands occurred, it is generally
	 * impossible to calculate the secure trim 2 timeout correctly.
	 */
	if (card->erase_shift)
		qty += ((to >> card->erase_shift) -
			(from >> card->erase_shift)) + 1;
	else if (mmc_card_sd(card))
		qty += to - from + 1;
	else
		qty += ((to / card->erase_size) -
			(from / card->erase_size)) + 1;

	if (!mmc_card_blockaddr(card)) {
		from <<= 9;
		to <<= 9;
	}

	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_START;
	else
		cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.arg = from;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group start error %d, "
		       "status %#x\n", err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_END;
	else
		cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.arg = to;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group end error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.arg = arg;
	busy_timeout = mmc_erase_timeout(card, arg, qty);
	/*
	 * If the host controller supports busy signalling and the timeout for
	 * the erase operation does not exceed the max_busy_timeout, we should
	 * use R1B response. Or we need to prevent the host from doing hw busy
	 * detection, which is done by converting to a R1 response instead.
	 * Note, some hosts requires R1B, which also means they are on their own
	 * when it comes to deal with the busy timeout.
	 */
	if (!(card->host->caps & MMC_CAP_NEED_RSP_BUSY) &&
	    card->host->max_busy_timeout &&
	    busy_timeout > card->host->max_busy_timeout) {
		cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	} else {
		cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
		cmd.busy_timeout = busy_timeout;
		use_r1b_resp = true;
	}

	/* for erase request, force use r1b type */
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	cmd.busy_timeout = busy_timeout;
	use_r1b_resp = true;

	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: erase error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_host_is_spi(card->host))
		goto out;

	/*
	 * In case of when R1B + MMC_CAP_WAIT_WHILE_BUSY is used, the polling
	 * shall be avoided.
	 */
	if ((card->host->caps & MMC_CAP_WAIT_WHILE_BUSY) && use_r1b_resp)
		goto out;

	timeout = jiffies + msecs_to_jiffies(busy_timeout);
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		/* Do not retry else we can't see errors */
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (err || R1_STATUS(cmd.resp[0])) {
			pr_err("error %d requesting status %#x\n",
				err, cmd.resp[0]);
			err = -EIO;
			goto out;
		}

		/* Timeout if the device never becomes ready for data and
		 * never leaves the program state.
		 */
		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(card->host), __func__);
			err =  -EIO;
			goto out;
		}
		if ((cmd.resp[0] & R1_READY_FOR_DATA) &&
		    R1_CURRENT_STATE(cmd.resp[0]) != R1_STATE_PRG)
			break;

		usleep_range(loop_udelay, loop_udelay*2);
		if (loop_udelay < udelay_max)
			loop_udelay *= 2;
	} while (1);

out:
	mmc_retune_release(card->host);
	return err;
}

static unsigned int mmc_align_erase_size(struct mmc_card *card,
					 unsigned int *from,
					 unsigned int *to,
					 unsigned int nr)
{
	unsigned int from_new = *from, nr_new = nr, rem;

	/*
	 * When the 'card->erase_size' is power of 2, we can use round_up/down()
	 * to align the erase size efficiently.
	 */
	if (is_power_of_2(card->erase_size)) {
		unsigned int temp = from_new;

		from_new = round_up(temp, card->erase_size);
		rem = from_new - temp;

		if (nr_new > rem)
			nr_new -= rem;
		else
			return 0;

		nr_new = round_down(nr_new, card->erase_size);
	} else {
		rem = from_new % card->erase_size;
		if (rem) {
			rem = card->erase_size - rem;
			from_new += rem;
			if (nr_new > rem)
				nr_new -= rem;
			else
				return 0;
		}

		rem = nr_new % card->erase_size;
		if (rem)
			nr_new -= rem;
	}

	if (nr_new == 0)
		return 0;

	*to = from_new + nr_new;
	*from = from_new;

	return nr_new;
}

/**
 * mmc_erase - erase sectors.
 * @card: card to erase
 * @from: first sector to erase
 * @nr: number of sectors to erase
 * @arg: erase command argument (SD supports only %MMC_ERASE_ARG)
 *
 * Caller must claim host before calling this function.
 */
int mmc_erase(struct mmc_card *card, unsigned int from, unsigned int nr,
	      unsigned int arg)
{
	unsigned int rem, to = from + nr;
	int err;

	if (!(card->host->caps & MMC_CAP_ERASE) ||
	    !(card->csd.cmdclass & CCC_ERASE))
		return -EOPNOTSUPP;

	if (!card->erase_size)
		return -EOPNOTSUPP;

	if (mmc_card_sd(card) && arg != MMC_ERASE_ARG)
		return -EOPNOTSUPP;

	if ((arg & MMC_SECURE_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN))
		return -EOPNOTSUPP;

	if ((arg & MMC_TRIM_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN))
		return -EOPNOTSUPP;

	if (arg == MMC_SECURE_ERASE_ARG) {
		if (from % card->erase_size || nr % card->erase_size)
			return -EINVAL;
	}

	if (arg == MMC_ERASE_ARG)
		nr = mmc_align_erase_size(card, &from, &to, nr);

	if (nr == 0)
		return 0;

	if (to <= from)
		return -EINVAL;

	/* 'from' and 'to' are inclusive */
	to -= 1;

	/*
	 * Special case where only one erase-group fits in the timeout budget:
	 * If the region crosses an erase-group boundary on this particular
	 * case, we will be trimming more than one erase-group which, does not
	 * fit in the timeout budget of the controller, so we need to split it
	 * and call mmc_do_erase() twice if necessary. This special case is
	 * identified by the card->eg_boundary flag.
	 */
	rem = card->erase_size - (from % card->erase_size);
	if ((arg & MMC_TRIM_ARGS) && (card->eg_boundary) && (nr > rem)) {
		err = mmc_do_erase(card, from, from + rem - 1, arg);
		from += rem;
		if ((err) || (to <= from))
			return err;
	}

	return mmc_do_erase(card, from, to, arg);
}
EXPORT_SYMBOL(mmc_erase);

int mmc_can_erase(struct mmc_card *card)
{
	if ((card->host->caps & MMC_CAP_ERASE) &&
	    (card->csd.cmdclass & CCC_ERASE) && card->erase_size)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_erase);

int mmc_can_trim(struct mmc_card *card)
{
	if ((card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN) &&
	    (!(card->quirks & MMC_QUIRK_TRIM_BROKEN)))
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_trim);

int mmc_can_discard(struct mmc_card *card)
{
	/*
	 * As there's no way to detect the discard support bit at v4.5
	 * use the s/w feature support filed.
	 */
	if (card->ext_csd.feature_support & MMC_DISCARD_FEATURE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_discard);

int mmc_can_sanitize(struct mmc_card *card)
{
	if (!mmc_can_trim(card) && !mmc_can_erase(card))
		return 0;
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_SANITIZE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_sanitize);

int mmc_can_secure_erase_trim(struct mmc_card *card)
{
	if ((card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN) &&
	    !(card->quirks & MMC_QUIRK_SEC_ERASE_TRIM_BROKEN))
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_secure_erase_trim);

int mmc_erase_group_aligned(struct mmc_card *card, unsigned int from,
			    unsigned int nr)
{
	if (!card->erase_size)
		return 0;
	if (from % card->erase_size || nr % card->erase_size)
		return 0;
	return 1;
}
EXPORT_SYMBOL(mmc_erase_group_aligned);

static unsigned int mmc_do_calc_max_discard(struct mmc_card *card,
					    unsigned int arg)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, x, y, qty = 0, max_qty, min_qty, timeout;
	unsigned int last_timeout = 0;
	unsigned int max_busy_timeout = host->max_busy_timeout ?
			host->max_busy_timeout : MMC_ERASE_TIMEOUT_MS;

	if (card->erase_shift) {
		max_qty = UINT_MAX >> card->erase_shift;
		min_qty = card->pref_erase >> card->erase_shift;
	} else if (mmc_card_sd(card)) {
		max_qty = UINT_MAX;
		min_qty = card->pref_erase;
	} else {
		max_qty = UINT_MAX / card->erase_size;
		min_qty = card->pref_erase / card->erase_size;
	}

	/*
	 * We should not only use 'host->max_busy_timeout' as the limitation
	 * when deciding the max discard sectors. We should set a balance value
	 * to improve the erase speed, and it can not get too long timeout at
	 * the same time.
	 *
	 * Here we set 'card->pref_erase' as the minimal discard sectors no
	 * matter what size of 'host->max_busy_timeout', but if the
	 * 'host->max_busy_timeout' is large enough for more discard sectors,
	 * then we can continue to increase the max discard sectors until we
	 * get a balance value. In cases when the 'host->max_busy_timeout'
	 * isn't specified, use the default max erase timeout.
	 */
	do {
		y = 0;
		for (x = 1; x && x <= max_qty && max_qty - x >= qty; x <<= 1) {
			timeout = mmc_erase_timeout(card, arg, qty + x);

			if (qty + x > min_qty && timeout > max_busy_timeout)
				break;

			if (timeout < last_timeout)
				break;
			last_timeout = timeout;
			y = x;
		}
		qty += y;
	} while (y);

	if (!qty)
		return 0;

	/*
	 * When specifying a sector range to trim, chances are we might cross
	 * an erase-group boundary even if the amount of sectors is less than
	 * one erase-group.
	 * If we can only fit one erase-group in the controller timeout budget,
	 * we have to care that erase-group boundaries are not crossed by a
	 * single trim operation. We flag that special case with "eg_boundary".
	 * In all other cases we can just decrement qty and pretend that we
	 * always touch (qty + 1) erase-groups as a simple optimization.
	 */
	if (qty == 1)
		card->eg_boundary = 1;
	else
		qty--;

	/* Convert qty to sectors */
	if (card->erase_shift)
		max_discard = qty << card->erase_shift;
	else if (mmc_card_sd(card))
		max_discard = qty + 1;
	else
		max_discard = qty * card->erase_size;

	return max_discard;
}

unsigned int mmc_calc_max_discard(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, max_trim;

	if (!host->max_busy_timeout)
		return UINT_MAX;
	/*
	 * Without erase_group_def set, MMC erase timeout depends on clock
	 * frequence which can change.  In that case, the best choice is
	 * just the preferred erase size.
	 */
	if (mmc_card_mmc(card) && !(card->ext_csd.erase_group_def & 1))
		return card->pref_erase;

	max_discard = mmc_do_calc_max_discard(card, MMC_ERASE_ARG);
	if (mmc_can_trim(card)) {
		max_trim = mmc_do_calc_max_discard(card, MMC_TRIM_ARG);
		if (max_trim < max_discard || max_discard == 0)
			max_discard = max_trim;
	} else if (max_discard < card->erase_size) {
		max_discard = 0;
	}
	pr_debug("%s: calculated max. discard sectors %u for timeout %u ms\n",
		mmc_hostname(host), max_discard, host->max_busy_timeout ?
		host->max_busy_timeout : MMC_ERASE_TIMEOUT_MS);
	return max_discard;
}
EXPORT_SYMBOL(mmc_calc_max_discard);

bool mmc_card_is_blockaddr(struct mmc_card *card)
{
	return card ? mmc_card_blockaddr(card) : false;
}
EXPORT_SYMBOL(mmc_card_is_blockaddr);

int mmc_set_blocklen(struct mmc_card *card, unsigned int blocklen)
{
	struct mmc_command cmd = {};

	if (mmc_card_blockaddr(card) || mmc_card_ddr52(card) ||
	    mmc_card_hs400(card) || mmc_card_hs400es(card))
		return 0;

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blocklen);

int mmc_set_blockcount(struct mmc_card *card, unsigned int blockcount,
			bool is_rel_write)
{
	struct mmc_command cmd = {};

	cmd.opcode = MMC_SET_BLOCK_COUNT;
	cmd.arg = blockcount & 0x0000FFFF;
	if (is_rel_write)
		cmd.arg |= 1 << 31;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blockcount);

static void mmc_hw_reset_for_init(struct mmc_host *host)
{
	mmc_pwrseq_reset(host);

	if (!(host->caps & MMC_CAP_HW_RESET) || !host->ops->hw_reset)
		return;
	host->ops->hw_reset(host);
}

int mmc_hw_reset(struct mmc_host *host)
{
	int ret;

	if (!host->card)
		return -EINVAL;

	mmc_bus_get(host);
	if (!host->bus_ops || host->bus_dead || !host->bus_ops->hw_reset) {
		mmc_bus_put(host);
		return -EOPNOTSUPP;
	}

	ret = host->bus_ops->hw_reset(host);
	mmc_bus_put(host);

	if (ret)
		pr_warn("%s: tried to HW reset card, got error %d\n",
			mmc_hostname(host), ret);

	return ret;
}
EXPORT_SYMBOL(mmc_hw_reset);

int mmc_sw_reset(struct mmc_host *host)
{
	int ret;

	if (!host->card)
		return -EINVAL;

	mmc_bus_get(host);
	if (!host->bus_ops || host->bus_dead || !host->bus_ops->sw_reset) {
		mmc_bus_put(host);
		return -EOPNOTSUPP;
	}

	ret = host->bus_ops->sw_reset(host);
	mmc_bus_put(host);

	if (ret)
		pr_warn("%s: tried to SW reset card, got error %d\n",
			mmc_hostname(host), ret);

	return ret;
}
EXPORT_SYMBOL(mmc_sw_reset);

static int mmc_rescan_try_freq(struct mmc_host *host, unsigned freq)
{
	host->f_init = freq;

	pr_debug("%s: %s: trying to init card at %u Hz\n",
		mmc_hostname(host), __func__, host->f_init);

	mmc_power_up(host, host->ocr_avail);

	/*
	 * Some eMMCs (with VCCQ always on) may not be reset after power up, so
	 * do a hardware reset if possible.
	 */
	mmc_hw_reset_for_init(host);

	/*
	 * sdio_reset sends CMD52 to reset card.  Since we do not know
	 * if the card is being re-initialized, just send it.  CMD52
	 * should be ignored by SD/eMMC cards.
	 * Skip it if we already know that we do not support SDIO commands
	 */
	if (!(host->caps2 & MMC_CAP2_NO_SDIO))
		sdio_reset(host);

	mmc_go_idle(host);

	if (!(host->caps2 & MMC_CAP2_NO_SD))
		mmc_send_if_cond(host, host->ocr_avail);

	/* Order's important: probe SDIO, then SD, then MMC */
	if (!(host->caps2 & MMC_CAP2_NO_SDIO))
		if (!mmc_attach_sdio(host))
			return 0;

	if (!(host->caps2 & MMC_CAP2_NO_SD))
		if (!mmc_attach_sd(host))
			return 0;

	if (!(host->caps2 & MMC_CAP2_NO_MMC))
		if (!mmc_attach_mmc(host))
			return 0;

	mmc_power_off(host);
	return -EIO;
}

int _mmc_detect_card_removed(struct mmc_host *host)
{
	int ret;

	if (!host->card || mmc_card_removed(host->card))
		return 1;

	ret = host->bus_ops->alive(host);

	/*
	 * Card detect status and alive check may be out of sync if card is
	 * removed slowly, when card detect switch changes while card/slot
	 * pads are still contacted in hardware (refer to "SD Card Mechanical
	 * Addendum, Appendix C: Card Detection Switch"). So reschedule a
	 * detect work 200ms later for this case.
	 */
	if (!ret && host->ops->get_cd && !host->ops->get_cd(host)) {
		mmc_detect_change(host, msecs_to_jiffies(200));
		pr_debug("%s: card removed too slowly\n", mmc_hostname(host));
	}

	if (ret) {
		mmc_card_set_removed(host->card);
		pr_debug("%s: card remove detected\n", mmc_hostname(host));
	}

	return ret;
}

int mmc_detect_card_removed(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int ret;

	WARN_ON(!host->claimed);

	if (!card)
		return 1;

	if (!mmc_card_is_removable(host))
		return 0;

	ret = mmc_card_removed(card);
	/*
	 * The card will be considered unchanged unless we have been asked to
	 * detect a change or host requires polling to provide card detection.
	 */
	if (!host->detect_change && !(host->caps & MMC_CAP_NEEDS_POLL))
		return ret;

	host->detect_change = 0;
	if (!ret) {
		ret = _mmc_detect_card_removed(host);
		if (ret && (host->caps & MMC_CAP_NEEDS_POLL)) {
			/*
			 * Schedule a detect work as soon as possible to let a
			 * rescan handle the card removal.
			 */
			cancel_delayed_work(&host->detect);
			_mmc_detect_change(host, 0, false);
		}
	}

	return ret;
}
EXPORT_SYMBOL(mmc_detect_card_removed);

void mmc_rescan(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, detect.work);
	int i;

	printk(KERN_ERR "%s === %s === rescan_disable=%d bus_dead=%d fast_power_off=%d\n",
    mmc_hostname(host),  __func__, host->rescan_disable, host->bus_dead, host->fast_power_off);

	mmc_set_sd_status(-1);

	if (host->rescan_disable)
		return;

	/* If there is a non-removable card registered, only scan once */
	if (!mmc_card_is_removable(host) && host->rescan_entered)
		return;
	host->rescan_entered = 1;

	if (host->trigger_card_event && host->ops->card_event) {
		mmc_claim_host(host);
		host->ops->card_event(host);
		mmc_release_host(host);
		host->trigger_card_event = false;
	}

	mmc_bus_get(host);

	/*
	 * if there is a _removable_ card registered, check whether it is
	 * still present
	 */
	if (host->bus_ops && !host->bus_dead && mmc_card_is_removable(host))
		host->bus_ops->detect(host);

	host->detect_change = 0;

	/*
	 * Let mmc_bus_put() free the bus/bus_ops if we've found that
	 * the card is no longer present.
	 */
	mmc_bus_put(host);
	mmc_bus_get(host);

	/* if there still is a card present, stop here */
	if (host->bus_ops != NULL) {
		mmc_bus_put(host);
		goto out;
	}

	/*
	 * Only we can add a new handler, so it's safe to
	 * release the lock here.
	 */
	mmc_bus_put(host);

	mmc_claim_host(host);
	if (mmc_card_is_removable(host) && host->ops->get_cd &&
			host->ops->get_cd(host) == 0) {
		mmc_power_off(host);
		mmc_release_host(host);
		printk(KERN_ERR "%s: going out.\n", mmc_hostname(host));
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(freqs); i++) {
		if (!mmc_rescan_try_freq(host, max(freqs[i], host->f_min)))
			break;
		if (freqs[i] <= host->f_min)
			break;
	}
	mmc_release_host(host);

 out:
	if (host->caps & MMC_CAP_NEEDS_POLL)
		mmc_schedule_delayed_work(&host->detect, HZ);
}

void mmc_start_host(struct mmc_host *host)
{
	int card_inserted = 1;
	int fast_power_off = 0;

	host->f_init = max(freqs[0], host->f_min);
	host->rescan_disable = 0;
	host->int_cnt_enable = 0;
	host->ios.power_mode = MMC_POWER_UNDEFINED;

	card_inserted = (host->index == 1) && mmc_gpio_get_status(host);
	fast_power_off = host->fast_power_off && !card_inserted;

	mmc_claim_host(host);
	if (host->caps2 & MMC_CAP2_NO_PRESCAN_POWERUP)
		mmc_power_off(host);
	else {
		if (!fast_power_off)
		mmc_power_up(host, host->ocr_avail);
	}
	mmc_release_host(host);

	mmc_gpiod_request_cd_irq(host);
	_mmc_detect_change(host, 0, false);
}

void mmc_stop_host(struct mmc_host *host)
{
	if (host->slot.cd_irq >= 0) {
		mmc_gpio_set_cd_wake(host, false);
		disable_irq(host->slot.cd_irq);
	}

	host->rescan_disable = 1;
	cancel_delayed_work_sync(&host->detect);

	/* clear pm flags now and let card drivers set them as needed */
	host->pm_flags = 0;


	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		/* Calling bus_ops->remove() with a claimed host can deadlock */
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_bus_put(host);
		return;
	}
	mmc_bus_put(host);

	mmc_claim_host(host);
	mmc_power_off(host);
	mmc_release_host(host);
}

#ifdef CONFIG_PM_SLEEP
/* Do the card removal on suspend if card is assumed removeable
 * Do that in pm notifier while userspace isn't yet frozen, so we will be able
   to sync the card.
*/
static int mmc_pm_notify(struct notifier_block *notify_block,
			unsigned long mode, void *unused)
{
	struct mmc_host *host = container_of(
		notify_block, struct mmc_host, pm_notify);
	unsigned long flags;
	int err = 0;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
	case PM_RESTORE_PREPARE:
		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 1;
		spin_unlock_irqrestore(&host->lock, flags);
		cancel_delayed_work_sync(&host->detect);

		if (!host->bus_ops)
			break;

		/* Validate prerequisites for suspend */
		if (host->bus_ops->pre_suspend)
			err = host->bus_ops->pre_suspend(host);
		if (!err)
			break;

		if (!mmc_card_is_removable(host)) {
			dev_warn(mmc_dev(host),
				 "pre_suspend failed for non-removable host: "
				 "%d\n", err);
			/* Avoid removing non-removable hosts */
			break;
		}

		/* Calling bus_ops->remove() with a claimed host can deadlock */
		host->bus_ops->remove(host);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		host->pm_flags = 0;
		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:

		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 0;
		spin_unlock_irqrestore(&host->lock, flags);
		_mmc_detect_change(host, 0, false);

	}

	return 0;
}

void mmc_register_pm_notifier(struct mmc_host *host)
{
	host->pm_notify.notifier_call = mmc_pm_notify;
	register_pm_notifier(&host->pm_notify);
}

void mmc_unregister_pm_notifier(struct mmc_host *host)
{
	unregister_pm_notifier(&host->pm_notify);
}
#endif

static int __init mmc_init(void)
{
	int ret;

	ret = mmc_register_bus();
	if (ret)
		return ret;

	ret = mmc_register_host_class();
	if (ret)
		goto unregister_bus;

	ret = sdio_register_bus();
	if (ret)
		goto unregister_host_class;

	return 0;

unregister_host_class:
	mmc_unregister_host_class();
unregister_bus:
	mmc_unregister_bus();
	return ret;
}

static void __exit mmc_exit(void)
{
	sdio_unregister_bus();
	mmc_unregister_host_class();
	mmc_unregister_bus();
}

subsys_initcall(mmc_init);
module_exit(mmc_exit);

MODULE_LICENSE("GPL");
