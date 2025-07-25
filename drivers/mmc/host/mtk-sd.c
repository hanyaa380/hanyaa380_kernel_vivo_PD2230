/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Chaotian.Jing <chaotian.jing@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include "../core/card.h"
#include <linux/mmc/card.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#ifdef CONFIG_MACH_MT8173
#include "../../misc/mediatek/include/mt-plat/mtk_chip.h"
#include "mt8173_top_io.h"
#endif

#include "cqhci.h"
#include "../../char/rpmb/rpmb-mtk.h"
#include "../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#include "mtk-msdc.h"
#include "mtk-sd-dbg.h"

void msdc_dump_info(struct mmc_host *mmc);

#define MAX_BD_NUM          1024

/*--------------------------------------------------------------------------*/
/* Common Definition                                                        */
/*--------------------------------------------------------------------------*/
#define MSDC_BUS_1BITS          0x0
#define MSDC_BUS_4BITS          0x1
#define MSDC_BUS_8BITS          0x2

#define MSDC_BURST_64B          0x6

#define MSDC_EMMC          (0)
#define MSDC_SD            (1)
#define MSDC_SDIO          (2)

/*--------------------------------------------------------------------------*/
/* Register Offset                                                          */
/*--------------------------------------------------------------------------*/
#define MSDC_CFG         0x0
#define MSDC_IOCON       0x04
#define MSDC_PS          0x08
#define MSDC_INT         0x0c
#define MSDC_INTEN       0x10
#define MSDC_FIFOCS      0x14
#define SDC_CFG          0x30
#define SDC_CMD          0x34
#define SDC_ARG          0x38
#define SDC_STS          0x3c
#define SDC_RESP0        0x40
#define SDC_RESP1        0x44
#define SDC_RESP2        0x48
#define SDC_RESP3        0x4c
#define SDC_BLK_NUM      0x50
#define SDC_ADV_CFG0     0x64
#define EMMC_IOCON       0x7c
#define SDC_ACMD_RESP    0x80
#define DMA_SA_H4BIT     0x8c
#define MSDC_DMA_SA      0x90
#define MSDC_DMA_CTRL    0x98
#define MSDC_DMA_CFG     0x9c
#define MSDC_PATCH_BIT   0xb0
#define MSDC_PATCH_BIT1  0xb4
#define MSDC_PATCH_BIT2  0xb8
#define MSDC_PAD_TUNE    0xec
#define MSDC_PAD_TUNE0   0xf0
#define PAD_DS_TUNE      0x188
#define PAD_CMD_TUNE     0x18c
#define EMMC50_CFG0      0x208
#define EMMC50_CFG3      0x220
#define SDC_FIFO_CFG     0x228

/*--------------------------------------------------------------------------*/
/* Top Pad Register Offset                                                  */
/*--------------------------------------------------------------------------*/
#define EMMC_TOP_CONTROL	0x00
#define EMMC_TOP_CMD		0x04
#define EMMC50_PAD_DS_TUNE	0x0c

/*--------------------------------------------------------------------------*/
/* Register Mask                                                            */
/*--------------------------------------------------------------------------*/

/* MSDC_CFG mask */
#define MSDC_CFG_MODE           (0x1 << 0)	/* RW */
#define MSDC_CFG_CKPDN          (0x1 << 1)	/* RW */
#define MSDC_CFG_RST            (0x1 << 2)	/* RW */
#define MSDC_CFG_PIO            (0x1 << 3)	/* RW */
#define MSDC_CFG_CKDRVEN        (0x1 << 4)	/* RW */
#define MSDC_CFG_BV18SDT        (0x1 << 5)	/* RW */
#define MSDC_CFG_BV18PSS        (0x1 << 6)	/* R  */
#define MSDC_CFG_CKSTB          (0x1 << 7)	/* R  */
#define MSDC_CFG_CKDIV          (0xff << 8)	/* RW */
#define MSDC_CFG_CKMOD          (0x3 << 16)	/* RW */
#define MSDC_CFG_HS400_CK_MODE  (0x1 << 18)	/* RW */
#define MSDC_CFG_HS400_CK_MODE_EXTRA  (0x1 << 22)	/* RW */
#define MSDC_CFG_CKDIV_EXTRA    (0xfff << 8)	/* RW */
#define MSDC_CFG_CKMOD_EXTRA    (0x3 << 20)	/* RW */

/* MSDC_IOCON mask */
#define MSDC_IOCON_SDR104CKS    (0x1 << 0)	/* RW */
#define MSDC_IOCON_RSPL         (0x1 << 1)	/* RW */
#define MSDC_IOCON_DSPL         (0x1 << 2)	/* RW */
#define MSDC_IOCON_DDLSEL       (0x1 << 3)	/* RW */
#define MSDC_IOCON_DDR50CKD     (0x1 << 4)	/* RW */
#define MSDC_IOCON_DSPLSEL      (0x1 << 5)	/* RW */
#define MSDC_IOCON_W_DSPL       (0x1 << 8)	/* RW */
#define MSDC_IOCON_D0SPL        (0x1 << 16)	/* RW */
#define MSDC_IOCON_D1SPL        (0x1 << 17)	/* RW */
#define MSDC_IOCON_D2SPL        (0x1 << 18)	/* RW */
#define MSDC_IOCON_D3SPL        (0x1 << 19)	/* RW */
#define MSDC_IOCON_D4SPL        (0x1 << 20)	/* RW */
#define MSDC_IOCON_D5SPL        (0x1 << 21)	/* RW */
#define MSDC_IOCON_D6SPL        (0x1 << 22)	/* RW */
#define MSDC_IOCON_D7SPL        (0x1 << 23)	/* RW */
#define MSDC_IOCON_RISCSZ       (0x3 << 24)	/* RW */

/* MSDC_PS mask */
#define MSDC_PS_CDEN            (0x1 << 0)	/* RW */
#define MSDC_PS_CDSTS           (0x1 << 1)	/* R  */
#define MSDC_PS_CDDEBOUNCE      (0xf << 12)	/* RW */
#define MSDC_PS_DAT             (0xff << 16)	/* R  */
#define MSDC_PS_CMD             (0x1 << 24)	/* R  */
#define MSDC_PS_WP              (0x1 << 31)	/* R  */

/* MSDC_INT mask */
#define MSDC_INT_MMCIRQ         (0x1 << 0)	/* W1C */
#define MSDC_INT_CDSC           (0x1 << 1)	/* W1C */
#define MSDC_INT_ACMDRDY        (0x1 << 3)	/* W1C */
#define MSDC_INT_ACMDTMO        (0x1 << 4)	/* W1C */
#define MSDC_INT_ACMDCRCERR     (0x1 << 5)	/* W1C */
#define MSDC_INT_DMAQ_EMPTY     (0x1 << 6)	/* W1C */
#define MSDC_INT_SDIOIRQ        (0x1 << 7)	/* W1C */
#define MSDC_INT_CMDRDY         (0x1 << 8)	/* W1C */
#define MSDC_INT_CMDTMO         (0x1 << 9)	/* W1C */
#define MSDC_INT_RSPCRCERR      (0x1 << 10)	/* W1C */
#define MSDC_INT_CSTA           (0x1 << 11)	/* R */
#define MSDC_INT_XFER_COMPL     (0x1 << 12)	/* W1C */
#define MSDC_INT_DXFER_DONE     (0x1 << 13)	/* W1C */
#define MSDC_INT_DATTMO         (0x1 << 14)	/* W1C */
#define MSDC_INT_DATCRCERR      (0x1 << 15)	/* W1C */
#define MSDC_INT_ACMD19_DONE    (0x1 << 16)	/* W1C */
#define MSDC_INT_DMA_BDCSERR    (0x1 << 17)	/* W1C */
#define MSDC_INT_DMA_GPDCSERR   (0x1 << 18)	/* W1C */
#define MSDC_INT_DMA_PROTECT    (0x1 << 19)	/* W1C */
#define MSDC_INT_CMDQ           (0x1 << 28)	/* W1C */

/* MSDC_INTEN mask */
#define MSDC_INTEN_MMCIRQ       (0x1 << 0)	/* RW */
#define MSDC_INTEN_CDSC         (0x1 << 1)	/* RW */
#define MSDC_INTEN_ACMDRDY      (0x1 << 3)	/* RW */
#define MSDC_INTEN_ACMDTMO      (0x1 << 4)	/* RW */
#define MSDC_INTEN_ACMDCRCERR   (0x1 << 5)	/* RW */
#define MSDC_INTEN_DMAQ_EMPTY   (0x1 << 6)	/* RW */
#define MSDC_INTEN_SDIOIRQ      (0x1 << 7)	/* RW */
#define MSDC_INTEN_CMDRDY       (0x1 << 8)	/* RW */
#define MSDC_INTEN_CMDTMO       (0x1 << 9)	/* RW */
#define MSDC_INTEN_RSPCRCERR    (0x1 << 10)	/* RW */
#define MSDC_INTEN_CSTA         (0x1 << 11)	/* RW */
#define MSDC_INTEN_XFER_COMPL   (0x1 << 12)	/* RW */
#define MSDC_INTEN_DXFER_DONE   (0x1 << 13)	/* RW */
#define MSDC_INTEN_DATTMO       (0x1 << 14)	/* RW */
#define MSDC_INTEN_DATCRCERR    (0x1 << 15)	/* RW */
#define MSDC_INTEN_ACMD19_DONE  (0x1 << 16)	/* RW */
#define MSDC_INTEN_DMA_BDCSERR  (0x1 << 17)	/* RW */
#define MSDC_INTEN_DMA_GPDCSERR (0x1 << 18)	/* RW */
#define MSDC_INTEN_DMA_PROTECT  (0x1 << 19)	/* RW */

/* MSDC_FIFOCS mask */
#define MSDC_FIFOCS_RXCNT       (0xff << 0)	/* R */
#define MSDC_FIFOCS_TXCNT       (0xff << 16)	/* R */
#define MSDC_FIFOCS_CLR         (0x1 << 31)	/* RW */

/* SDC_CFG mask */
#define SDC_CFG_SDIOINTWKUP     (0x1 << 0)	/* RW */
#define SDC_CFG_INSWKUP         (0x1 << 1)	/* RW */
#define SDC_CFG_WRDTOC          (0x1fff  << 2)  /* RW */
#define SDC_CFG_BUSWIDTH        (0x3 << 16)	/* RW */
#define SDC_CFG_SDIO            (0x1 << 19)	/* RW */
#define SDC_CFG_SDIOIDE         (0x1 << 20)	/* RW */
#define SDC_CFG_INTATGAP        (0x1 << 21)	/* RW */
#define SDC_CFG_DTOC            (0xff << 24)	/* RW */

/* SDC_STS mask */
#define SDC_STS_SDCBUSY         (0x1 << 0)	/* RW */
#define SDC_STS_CMDBUSY         (0x1 << 1)	/* RW */
#define SDC_STS_SWR_COMPL       (0x1 << 31)	/* RW */

/* SDC_ADV_CFG0 mask */
#define SDC_RX_ENHANCE_EN	(0x1 << 20)	/* RW */

/* DMA_SA_H4BIT mask */
#define DMA_ADDR_HIGH_4BIT      (0xf << 0)      /* RW */

/* MSDC_DMA_CTRL mask */
#define MSDC_DMA_CTRL_START     (0x1 << 0)	/* W */
#define MSDC_DMA_CTRL_STOP      (0x1 << 1)	/* W */
#define MSDC_DMA_CTRL_RESUME    (0x1 << 2)	/* W */
#define MSDC_DMA_CTRL_MODE      (0x1 << 8)	/* RW */
#define MSDC_DMA_CTRL_LASTBUF   (0x1 << 10)	/* RW */
#define MSDC_DMA_CTRL_BRUSTSZ   (0x7 << 12)	/* RW */

/* MSDC_DMA_CFG mask */
#define MSDC_DMA_CFG_STS        (0x1 << 0)	/* R */
#define MSDC_DMA_CFG_DECSEN     (0x1 << 1)	/* RW */
#define MSDC_DMA_CFG_AHBHPROT2  (0x2 << 8)	/* RW */
#define MSDC_DMA_CFG_ACTIVEEN   (0x2 << 12)	/* RW */
#define MSDC_DMA_CFG_CS12B16B   (0x1 << 16)	/* RW */

/* MSDC_PATCH_BIT mask */
#define MSDC_PATCH_BIT_ODDSUPP    (0x1 <<  1)	/* RW */
#define MSDC_INT_DAT_LATCH_CK_SEL (0x7 <<  7)
#define MSDC_CKGEN_MSDC_DLY_SEL   (0x1f << 10)
#define MSDC_PATCH_BIT_IODSSEL    (0x1 << 16)	/* RW */
#define MSDC_PATCH_BIT_IOINTSEL   (0x1 << 17)	/* RW */
#define MSDC_PATCH_BIT_BUSYDLY    (0xf << 18)	/* RW */
#define MSDC_PATCH_BIT_WDOD       (0xf << 22)	/* RW */
#define MSDC_PATCH_BIT_IDRTSEL    (0x1 << 26)	/* RW */
#define MSDC_PATCH_BIT_CMDFSEL    (0x1 << 27)	/* RW */
#define MSDC_PATCH_BIT_INTDLSEL   (0x1 << 28)	/* RW */
#define MSDC_PATCH_BIT_SPCPUSH    (0x1 << 29)	/* RW */
#define MSDC_PATCH_BIT_DECRCTMO   (0x1 << 30)	/* RW */
#define MSDC_PATCH_BIT1_WRTA      (0x7 << 0)    /* RW */

#define MSDC_PB1_BUSY_CHECK_SEL   (0x1 << 7)    /* RW */
#define MSDC_PATCH_BIT1_CMDTA     (0x7 << 3)    /* RW */
#define MSDC_PATCH_BIT1_STOP_DLY  (0xf << 8)    /* RW */

#define MSDC_PATCH_BIT2_CFGRESP   (0x1 << 15)   /* RW */
#define MSDC_PATCH_BIT2_CFGCRCSTS (0x1 << 28)   /* RW */
#define MSDC_PB2_SUPPORT_64G      (0x1 << 1)    /* RW */
#define MSDC_PB2_RESPWAIT         (0x3 << 2)    /* RW */
#define MSDC_PB2_RESPSTSENSEL     (0x7 << 16)   /* RW */
#define MSDC_PB2_CRCSTSENSEL      (0x7 << 29)   /* RW */

#define MSDC_PAD_TUNE_DATWRDLY	  (0x1f <<  0)	/* RW */
#define MSDC_PAD_TUNE_DATRRDLY	  (0x1f <<  8)	/* RW */
#define MSDC_PAD_TUNE_CMDRDLY	  (0x1f << 16)  /* RW */
#define MSDC_PAD_TUNE_CMDRRDLY	  (0x1f << 22)	/* RW */
#define MSDC_PAD_TUNE_CLKTDLY	  (0x1f << 27)  /* RW */
#define MSDC_PAD_TUNE_RXDLYSEL	  (0x1 << 15)   /* RW */
#define MSDC_PAD_TUNE_RD_SEL	  (0x1 << 13)   /* RW */
#define MSDC_PAD_TUNE_CMD_SEL	  (0x1 << 21)   /* RW */

#define PAD_DS_TUNE_DLY_SEL       (0x1 << 0)	/* RW */
#define PAD_DS_TUNE_DLY1	  (0x1f << 2)   /* RW */
#define PAD_DS_TUNE_DLY2	  (0x1f << 7)   /* RW */
#define PAD_DS_TUNE_DLY3	  (0x1f << 12)  /* RW */

#define PAD_CMD_TUNE_RX_DLY3	  (0x1f << 1)  /* RW */

#define EMMC50_CFG_PADCMD_LATCHCK (0x1 << 0)   /* RW */
#define EMMC50_CFG_CRCSTS_EDGE    (0x1 << 3)   /* RW */
#define EMMC50_CFG_CFCSTS_SEL     (0x1 << 4)   /* RW */

#define EMMC50_CFG3_OUTS_WR       (0x1f << 0)  /* RW */

#define SDC_FIFO_CFG_WRVALIDSEL   (0x1 << 24)  /* RW */
#define SDC_FIFO_CFG_RDVALIDSEL   (0x1 << 25)  /* RW */
/* MT8173 E2 top register 0x10000404 */
#define MSDC1_DAT0_DELAY (0x1f << 10)
#define MSDC1_DAT1_DELAY (0x1f << 15)
#define MSDC1_DAT2_DELAY (0x1f << 20)
#define MSDC1_DAT3_DELAY (0x1f << 25)

/* MT8173 E2 top register 0x10000408 */
#define MSDC1_CMD_DELAY	 (0x1f << 25)

/* EMMC_TOP_CONTROL mask */
#define PAD_RXDLY_SEL           (0x1 << 0)      /* RW */
#define DELAY_EN                (0x1 << 1)      /* RW */
#define PAD_DAT_RD_RXDLY2       (0x1f << 2)     /* RW */
#define PAD_DAT_RD_RXDLY        (0x1f << 7)     /* RW */
#define PAD_DAT_RD_RXDLY2_SEL   (0x1 << 12)     /* RW */
#define PAD_DAT_RD_RXDLY_SEL    (0x1 << 13)     /* RW */
#define DATA_K_VALUE_SEL        (0x1 << 14)     /* RW */
#define SDC_RX_ENH_EN           (0x1 << 15)     /* TW */

/* EMMC_TOP_CMD mask */
#define PAD_CMD_RXDLY2          (0x1f << 0)     /* RW */
#define PAD_CMD_RXDLY           (0x1f << 5)     /* RW */
#define PAD_CMD_RD_RXDLY2_SEL   (0x1 << 10)     /* RW */
#define PAD_CMD_RD_RXDLY_SEL    (0x1 << 11)     /* RW */
#define PAD_CMD_TX_DLY          (0x1f << 12)    /* RW */

/* EMMC50_PAD_DS_TUNE mask */
#define PAD_DS_DLY_SEL		(0x1 << 16)	/* RW */
#define PAD_DS_DLY1		(0x1f << 10)	/* RW */
#define PAD_DS_DLY3		(0x1f << 0)	/* RW */

#define REQ_CMD_EIO  (0x1 << 0)
#define REQ_CMD_TMO  (0x1 << 1)
#define REQ_DAT_ERR  (0x1 << 2)
#define REQ_STOP_EIO (0x1 << 3)
#define REQ_STOP_TMO (0x1 << 4)
#define REQ_CMD_BUSY (0x1 << 5)

#define MSDC_PREPARE_FLAG (0x1 << 0)
#define MSDC_ASYNC_FLAG (0x1 << 1)
#define MSDC_MMAP_FLAG (0x1 << 2)

#define MTK_MMC_AUTOSUSPEND_DELAY	50
#define CMD_TIMEOUT         (HZ/10 * 5)	/* 100ms x5 */
#define DAT_TIMEOUT         (HZ    * 5)	/* 1000ms x5 */

#define PAD_DELAY_MAX	32 /* PAD delay cells */

/*--------------------------------------------------------------------------*/
/* SDCard error handler                                                     */
/*--------------------------------------------------------------------------*/
/* if continuous data timeout reach the limit */
/* driver will force remove card */
#define MSDC_MAX_DATA_TIMEOUT_CONTINUOUS (100)

/* if continuous power cycle fail reach the limit */
/* driver will force remove card */
#define MSDC_MAX_POWER_CYCLE_FAIL_CONTINUOUS (3)

/* count of bad sd detecter (or bad sd condition kinds),
 * we can add it here if has other condition
 */
#define BAD_SD_DETECTER_COUNT 1

/* we take it as bad sd when the bad sd condition occurs
 * out of tolerance
 */
u32 bad_sd_tolerance[BAD_SD_DETECTER_COUNT] = {10};

/* bad sd condition occur times
 */
u32 bad_sd_detecter[BAD_SD_DETECTER_COUNT] = {0};

/* bad sd condition occur times will reset to zero by self
 * when reach the forget time (when set to 0, means not
 * reset to 0 by self), unit:s
 */
u32 bad_sd_forget[BAD_SD_DETECTER_COUNT] = {3};

/* the latest occur time of the bad sd condition,
 * unit: clock
 */
unsigned long bad_sd_timer[BAD_SD_DETECTER_COUNT] = {0};


/*--------------------------------------------------------------------------*/
/* Descriptor Structure                                                     */
/*--------------------------------------------------------------------------*/
struct mtk_mmc_compatible {
	u8 clk_div_bits;
	bool hs400_tune; /* only used for MT8173 */
	u32 pad_tune_reg;
	bool async_fifo;
	bool data_tune;
	bool busy_check;
	bool stop_clk_fix;
	bool enhance_rx;
	bool support_64g;
};

struct msdc_delay_phase {
	u8 maxlen;
	u8 start;
	u8 final_phase;
};

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

#ifdef CONFIG_MACH_MT8173
spinlock_t msdc_top_lock;
static bool msdc_top_lock_inited;
#endif
static const struct mtk_mmc_compatible mt8135_compat = {
	.clk_div_bits = 8,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt8173_compat = {
	.clk_div_bits = 8,
	.hs400_tune = true,
	.pad_tune_reg = MSDC_PAD_TUNE,
	.async_fifo = false,
	.data_tune = false,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt8183_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt2701_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = false,
	.stop_clk_fix = false,
	.enhance_rx = false,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt2712_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt7622_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = false,
};

static const struct mtk_mmc_compatible mt6779_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6768_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6781_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6785_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6877_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt8666_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6833_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6853_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct mtk_mmc_compatible mt6873_compat = {
	.clk_div_bits = 12,
	.hs400_tune = false,
	.pad_tune_reg = MSDC_PAD_TUNE0,
	.async_fifo = true,
	.data_tune = true,
	.busy_check = true,
	.stop_clk_fix = true,
	.enhance_rx = true,
	.support_64g = true,
};

static const struct of_device_id msdc_of_ids[] = {
	{ .compatible = "mediatek,mt8135-mmc", .data = &mt8135_compat},
	{ .compatible = "mediatek,mt8173-mmc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt8183-mmc", .data = &mt8183_compat},
	{ .compatible = "mediatek,mt2701-mmc", .data = &mt2701_compat},
	{ .compatible = "mediatek,mt2712-mmc", .data = &mt2712_compat},
	{ .compatible = "mediatek,mt7622-mmc", .data = &mt7622_compat},
	{ .compatible = "mediatek,mt6779-mmc", .data = &mt6779_compat},
	{ .compatible = "mediatek,mt6768-mmc", .data = &mt6768_compat},
	{ .compatible = "mediatek,mt6781-mmc", .data = &mt6781_compat},
	{ .compatible = "mediatek,mt6785-mmc", .data = &mt6785_compat},
	{ .compatible = "mediatek,mt6877-mmc", .data = &mt6877_compat},
	{ .compatible = "mediatek,mt8666-mmc", .data = &mt8666_compat},
	{ .compatible = "mediatek,mt6833-mmc", .data = &mt6833_compat},
	{ .compatible = "mediatek,mt6853-mmc", .data = &mt6853_compat},
	{ .compatible = "mediatek,mt6873-mmc", .data = &mt6873_compat},
	{}
};
MODULE_DEVICE_TABLE(of, msdc_of_ids);

static void sdr_set_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val |= bs;
	writel(val, reg);
}

static void sdr_clr_bits(void __iomem *reg, u32 bs)
{
	u32 val = readl(reg);

	val &= ~bs;
	writel(val, reg);
}

static void sdr_set_field(void __iomem *reg, u32 field, u32 val)
{
	unsigned int tv = readl(reg);

	tv &= ~field;
	tv |= ((val) << (ffs((unsigned int)field) - 1));
	writel(tv, reg);
}

static void sdr_get_field(void __iomem *reg, u32 field, u32 *val)
{
	unsigned int tv = readl(reg);

	*val = ((tv & field) >> (ffs((unsigned int)field) - 1));
}

static void msdc_retry(struct msdc_host *host, int addr, int val, int expr, int retry, int cnt,
	struct mmc_host *mmc)
{

	do {
		int backup = cnt;

		while (retry) {
			sdr_set_bits(host->base + addr, val);
			if (!(expr))
				break;
			if (cnt-- == 0) {
				retry--; mdelay(1); cnt = backup;
			}
		}
		if (retry == 0) {
			dev_info(mmc->parent, "%s\n", __func__);
			msdc_dump_info(mmc);
		}
		WARN_ON(retry == 0);
	} while (0);
}

static void msdc_reset_hw(struct msdc_host *host)
{
	u32 val;
	u32 count = 0;
	//dev_info(host->mmc->parent, "%s\n",__func__);

	//sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_RST);
	msdc_retry(host, MSDC_CFG, MSDC_CFG_RST,
		readl(host->base + MSDC_CFG) & MSDC_CFG_RST,
		10, 1000, host->mmc);

	//sdr_set_bits(host->base + MSDC_FIFOCS, MSDC_FIFOCS_CLR);
	msdc_retry(host, MSDC_FIFOCS, MSDC_FIFOCS_CLR,
		readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_CLR,
		10, 1000, host->mmc);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd);

static const u32 cmd_ints_mask = MSDC_INTEN_CMDRDY | MSDC_INTEN_RSPCRCERR |
			MSDC_INTEN_CMDTMO | MSDC_INTEN_ACMDRDY |
			MSDC_INTEN_ACMDCRCERR | MSDC_INTEN_ACMDTMO;
static const u32 data_ints_mask = MSDC_INTEN_XFER_COMPL | MSDC_INTEN_DATTMO |
			MSDC_INTEN_DATCRCERR | MSDC_INTEN_DMA_BDCSERR |
			MSDC_INTEN_DMA_GPDCSERR | MSDC_INTEN_DMA_PROTECT;

static u8 msdc_dma_calcs(u8 *buf, u32 len)
{
	u32 i, sum = 0;

	for (i = 0; i < len; i++)
		sum += buf[i];
	return 0xff - (u8) sum;
}

static inline void msdc_dma_setup(struct msdc_host *host, struct msdc_dma *dma,
		struct mmc_data *data)
{
	unsigned int j, dma_len;
	dma_addr_t dma_address;
	u32 dma_ctrl;
	struct scatterlist *sg;
	struct mt_gpdma_desc *gpd;
	struct mt_bdma_desc *bd;

	sg = data->sg;

	gpd = dma->gpd;
	bd = dma->bd;

	/* modify gpd */
	gpd->gpd_info |= GPDMA_DESC_HWO;
	gpd->gpd_info |= GPDMA_DESC_BDP;
	/* need to clear first. use these bits to calc checksum */
	gpd->gpd_info &= ~GPDMA_DESC_CHECKSUM;
	gpd->gpd_info |= msdc_dma_calcs((u8 *) gpd, 16) << 8;

	/* modify bd */
	for_each_sg(data->sg, sg, data->sg_count, j) {
		dma_address = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);

		/* init bd */
		bd[j].bd_info &= ~BDMA_DESC_BLKPAD;
		bd[j].bd_info &= ~BDMA_DESC_DWPAD;
		bd[j].ptr = lower_32_bits(dma_address);
		if (host->dev_comp->support_64g) {
			bd[j].bd_info &= ~BDMA_DESC_PTR_H4;
			bd[j].bd_info |= (upper_32_bits(dma_address) & 0xf)
					 << 28;
		}

		if (host->dev_comp->support_64g) {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN_EXT;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN_EXT);
		} else {
			bd[j].bd_data_len &= ~BDMA_DESC_BUFLEN;
			bd[j].bd_data_len |= (dma_len & BDMA_DESC_BUFLEN);
		}

		if (j == data->sg_count - 1) /* the last bd */
			bd[j].bd_info |= BDMA_DESC_EOL;
		else
			bd[j].bd_info &= ~BDMA_DESC_EOL;

		/* checksume need to clear first */
		bd[j].bd_info &= ~BDMA_DESC_CHECKSUM;
		bd[j].bd_info |= msdc_dma_calcs((u8 *)(&bd[j]), 16) << 8;
	}

	sdr_set_field(host->base + MSDC_DMA_CFG, MSDC_DMA_CFG_DECSEN, 1);
	dma_ctrl = readl_relaxed(host->base + MSDC_DMA_CTRL);
	dma_ctrl &= ~(MSDC_DMA_CTRL_BRUSTSZ | MSDC_DMA_CTRL_MODE);
	dma_ctrl |= (MSDC_BURST_64B << 12 | 1 << 8);
	writel_relaxed(dma_ctrl, host->base + MSDC_DMA_CTRL);
	if (host->dev_comp->support_64g)
		sdr_set_field(host->base + DMA_SA_H4BIT, DMA_ADDR_HIGH_4BIT,
			      upper_32_bits(dma->gpd_addr) & 0xf);
	writel(lower_32_bits(dma->gpd_addr), host->base + MSDC_DMA_SA);
}

static void msdc_prepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (!(data->host_cookie & MSDC_PREPARE_FLAG)) {
		data->host_cookie |= MSDC_PREPARE_FLAG;
		data->sg_count = dma_map_sg(host->dev, data->sg, data->sg_len,
					    mmc_get_dma_dir(data));
	}
}

static void msdc_unprepare_data(struct msdc_host *host, struct mmc_request *mrq)
{
	struct mmc_data *data = mrq->data;

	if (data->host_cookie & MSDC_ASYNC_FLAG)
		return;

	if (data->host_cookie & MSDC_PREPARE_FLAG) {
		dma_unmap_sg(host->dev, data->sg, data->sg_len,
			     mmc_get_dma_dir(data));
		data->host_cookie &= ~MSDC_PREPARE_FLAG;
	}
}

static u64 msdc_timeout_cal(struct msdc_host *host, u64 ns, u64 clks)
{
	u64 timeout, clk_ns;
	u32 mode = 0;

	host->timeout_ns = ns;
	host->timeout_clks = clks;
	if (host->mmc->actual_clock == 0) {
		timeout = 0;
	} else {
		clk_ns  = 1000000000UL / host->mmc->actual_clock;
		timeout = (ns + clk_ns - 1) / clk_ns + clks;
		/* in 1048576 sclk cycle unit */
		timeout = DIV_ROUND_UP(timeout, (0x1 << 20));
		if (host->dev_comp->clk_div_bits == 8)
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD, &mode);
		else
			sdr_get_field(host->base + MSDC_CFG,
				      MSDC_CFG_CKMOD_EXTRA, &mode);
		/*DDR mode will double the clk cycles for data timeout */
		timeout = mode >= 2 ? timeout * 2 : timeout;
		timeout = timeout > 1 ? timeout - 1 : 0;
	}
	return timeout;
}

/* clock control primitives */
static void msdc_set_timeout(struct msdc_host *host, u64 ns, u64 clks)
{
	u64 timeout;

	host->timeout_ns = ns;
	host->timeout_clks = clks;

	timeout = msdc_timeout_cal(host, ns, clks);
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC,
		(u32)(timeout > 255 ? 255 : timeout));
}

static void msdc_set_busy_timeout(struct msdc_host *host, u64 ns, u64 clks)
{
	u64 timeout;

	timeout = msdc_timeout_cal(host, ns, clks);
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_WRDTOC,
		(u32)(timeout > 8191 ? 8191 : timeout));
}

void msdc_gate_clock(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	if (!host)
		return;

	if (mmc->caps2 & MMC_CAP2_CRYPTO)
		clk_disable_unprepare(host->crypto_clk);
	clk_disable_unprepare(host->src_clk_cg);
	clk_disable_unprepare(host->src_clk);
	clk_disable_unprepare(host->bus_clk);
	clk_disable_unprepare(host->h_clk);
}

void msdc_ungate_clock(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	if (!host)
		return;

	clk_prepare_enable(host->h_clk);
	clk_prepare_enable(host->bus_clk);
	clk_prepare_enable(host->src_clk);
	clk_prepare_enable(host->src_clk_cg);
	if (mmc->caps2 & MMC_CAP2_CRYPTO)
		clk_prepare_enable(host->crypto_clk);

	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB))
		cpu_relax();
}

static void msdc_set_mclk(struct msdc_host *host, unsigned char timing, u32 hz)
{
	struct mmc_host *mmc = host->mmc;
	u32 mode;
	u32 flags;
	u32 div;
	u32 sclk;
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (!hz) {
		dev_dbg(host->dev, "set mclk to 0\n");
		host->mclk = 0;
		host->mmc->actual_clock = 0;
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
		return;
	}

	flags = readl(host->base + MSDC_INTEN);
	sdr_clr_bits(host->base + MSDC_INTEN, flags);
	if (host->dev_comp->clk_div_bits == 8)
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_HS400_CK_MODE);
	else
		sdr_clr_bits(host->base + MSDC_CFG,
			     MSDC_CFG_HS400_CK_MODE_EXTRA);
	if (timing == MMC_TIMING_UHS_DDR50 ||
	    timing == MMC_TIMING_MMC_DDR52 ||
	    timing == MMC_TIMING_MMC_HS400) {
		if (timing == MMC_TIMING_MMC_HS400)
			mode = 0x3;
		else
			mode = 0x2; /* ddr mode and use divisor */

		if (hz >= (host->src_clk_freq >> 2)) {
			div = 0; /* mean div = 1/4 */
			sclk = host->src_clk_freq >> 2; /* sclk = clk / 4 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
			div = (div >> 1);
		}

		if (timing == MMC_TIMING_MMC_HS400 &&
		    hz >= (host->src_clk_freq >> 1)) {
			if (host->dev_comp->clk_div_bits == 8)
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE);
			else
				sdr_set_bits(host->base + MSDC_CFG,
					     MSDC_CFG_HS400_CK_MODE_EXTRA);
			sclk = host->src_clk_freq >> 1;
			div = 0; /* div is ignore when bit18 is set */
		}
	} else if (hz >= host->src_clk_freq) {
		mode = 0x1; /* no divisor */
		div = 0;
		sclk = host->src_clk_freq;
	} else {
		mode = 0x0; /* use divisor */
		if (hz >= (host->src_clk_freq >> 1)) {
			div = 0; /* mean div = 1/2 */
			sclk = host->src_clk_freq >> 1; /* sclk = clk / 2 */
		} else {
			div = (host->src_clk_freq + ((hz << 2) - 1)) / (hz << 2);
			sclk = (host->src_clk_freq >> 2) / div;
		}
	}
	//sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	/*
	 * As src_clk/HCLK use the same bit to gate/ungate,
	 * So if want to only gate src_clk, need gate its parent(mux).
	 */
	if (host->src_clk_cg)
		clk_disable_unprepare(host->src_clk_cg);
	else
		clk_disable_unprepare(clk_get_parent(host->src_clk));
	if (host->dev_comp->clk_div_bits == 8)
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD | MSDC_CFG_CKDIV,
			      (mode << 8) | div);
	else
		sdr_set_field(host->base + MSDC_CFG,
			      MSDC_CFG_CKMOD_EXTRA | MSDC_CFG_CKDIV_EXTRA,
			      (mode << 12) | div);
	if (host->src_clk_cg)
		clk_prepare_enable(host->src_clk_cg);
	else
		clk_prepare_enable(clk_get_parent(host->src_clk));

	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB))
		cpu_relax();
	if (host->mclk == 0 && (mmc->caps2 & MMC_CAP2_NO_MMC)
		&& mmc->ios.signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		dev_info(host->dev, "[%s]:enable clk free run 1ms+ for switch to 1.8v\n",
			__func__);
		sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
		usleep_range(1000, 1500);
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	}

#ifdef CONFIG_MACH_MT8173
	host->sclk = sclk;
#endif
	host->mmc->actual_clock = sclk;
	host->mclk = hz;
	host->timing = timing;
	/* need because clk changed. */
	msdc_set_timeout(host, host->timeout_ns, host->timeout_clks);
	sdr_set_bits(host->base + MSDC_INTEN, flags);

	/*
	 * mmc_select_hs400() will drop to 50Mhz and High speed mode,
	 * tune result of hs200/200Mhz is not suitable for 50Mhz
	 */
	if (host->mmc->actual_clock <= 52000000) {
		writel(host->def_tune_para.iocon, host->base + MSDC_IOCON);
		if (host->top_base) {
			writel(host->def_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->def_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		} else {
			writel(host->def_tune_para.pad_tune,
			       host->base + tune_reg);
		}
	} else {
		writel(host->saved_tune_para.iocon, host->base + MSDC_IOCON);
		writel(host->saved_tune_para.pad_cmd_tune,
		       host->base + PAD_CMD_TUNE);
		if (host->top_base) {
			writel(host->saved_tune_para.emmc_top_control,
			       host->top_base + EMMC_TOP_CONTROL);
			writel(host->saved_tune_para.emmc_top_cmd,
			       host->top_base + EMMC_TOP_CMD);
		} else {
			writel(host->saved_tune_para.pad_tune,
			       host->base + tune_reg);
		}
	}

	if (timing == MMC_TIMING_MMC_HS400 &&
	    host->dev_comp->hs400_tune)
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs400_cmd_int_delay);
	dev_info(host->dev, "sclk: %d, timing: %d\n", host->mmc->actual_clock,
		timing);
}

static inline u32 msdc_cmd_find_resp(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 resp;

	switch (mmc_resp_type(cmd)) {
		/* Actually, R1, R5, R6, R7 are the same */
	case MMC_RSP_R1:
		resp = 0x1;
		break;
	case MMC_RSP_R1B:
		resp = 0x7;
		break;
	case MMC_RSP_R2:
		resp = 0x2;
		break;
	case MMC_RSP_R3:
		resp = 0x3;
		break;
	case MMC_RSP_NONE:
	default:
		resp = 0x0;
		break;
	}

	return resp;
}

static inline u32 msdc_cmd_prepare_raw_cmd(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	/* rawcmd :
	 * vol_swt << 30 | auto_cmd << 28 | blklen << 16 | go_irq << 15 |
	 * stop << 14 | rw << 13 | dtype << 11 | rsptyp << 7 | brk << 6 | opcode
	 */
	u32 opcode = cmd->opcode;
	u32 resp = msdc_cmd_find_resp(host, mrq, cmd);
	u32 rawcmd = (opcode & 0x3f) | ((resp & 0x7) << 7);
	u32 blksz = (readl(host->base + SDC_CMD) >> 16) & 0xFFF;

	host->cmd_rsp = resp;

	if ((opcode == SD_IO_RW_DIRECT && cmd->flags == (unsigned int) -1) ||
	    opcode == MMC_STOP_TRANSMISSION)
		rawcmd |= (0x1 << 14);
	else if (opcode == SD_SWITCH_VOLTAGE)
		rawcmd |= (0x1 << 30);
	else if (opcode == SD_APP_SEND_SCR ||
		 opcode == SD_APP_SEND_NUM_WR_BLKS ||
		 (opcode == SD_SWITCH && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == SD_APP_SD_STATUS && mmc_cmd_type(cmd) == MMC_CMD_ADTC) ||
		 (opcode == MMC_SEND_EXT_CSD && mmc_cmd_type(cmd) == MMC_CMD_ADTC))
		rawcmd |= (0x1 << 11);

	if (cmd->data) {
		struct mmc_data *data = cmd->data;

		if (mmc_op_multi(opcode)) {
			if (mmc_card_mmc(host->mmc->card) && mrq->sbc &&
			    !(mrq->sbc->arg & 0xFFFF0000))
				rawcmd |= 0x2 << 28; /* AutoCMD23 */
			else if (mmc_card_sd(host->mmc->card) && (host->autocmd & MSDC_AUTOCMD12))
				rawcmd |= 0x1 << 28; /* AutoCMD12 */
		}

		rawcmd |= ((data->blksz & 0xFFF) << 16);
		if (data->flags & MMC_DATA_WRITE)
			rawcmd |= (0x1 << 13);
		if (data->blocks > 1)
			rawcmd |= (0x2 << 11);
		else
			rawcmd |= (0x1 << 11);
		/* Always use dma mode */
		sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_PIO);

		if (host->timeout_ns != data->timeout_ns ||
		    host->timeout_clks != data->timeout_clks)
			msdc_set_timeout(host, data->timeout_ns,
					data->timeout_clks);

		writel(data->blocks, host->base + SDC_BLK_NUM);
	} else
		rawcmd |= blksz << 16;

	return rawcmd;
}

void msdc_dump_info_2(struct msdc_host *host,struct mmc_command *cmd){
	int i;
	int offset;

	if(cmd){
		if(cmd->opcode != 25 || cmd->opcode != 18)
			return;
		pr_info("[%s %d]cmd=%d\n",__func__,__LINE__,cmd->opcode);
	} else
		pr_info("[%s %d]\n",__func__,__LINE__);

	for(i=0;i<4;i++){
		pr_info("dmagpd[%08X]=%08X \n",i*4,readl(phys_to_virt(host->dma.gpd_addr + i*4)));
	}
}

static void msdc_start_data(struct msdc_host *host, struct mmc_request *mrq,
			    struct mmc_command *cmd, struct mmc_data *data)
{
	bool read;
	int i;

	WARN_ON(host->data);
	host->data = data;
	read = data->flags & MMC_DATA_READ;

	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	msdc_dma_setup(host, &host->dma, data);
	sdr_set_bits(host->base + MSDC_INTEN, data_ints_mask);
	sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_START, 1);
	dev_dbg(host->dev, "DMA start\n");
	dev_dbg(host->dev, "%s: cmd=%d DMA data: %d blocks; read=%d\n",
			__func__, cmd->opcode, data->blocks, read);
}

static int msdc_auto_cmd_done(struct msdc_host *host, int events,
		struct mmc_command *cmd)
{
	u32 *rsp = cmd->resp;

	rsp[0] = readl(host->base + SDC_ACMD_RESP);

	if (events & MSDC_INT_ACMDRDY) {
		cmd->error = 0;
		if (host->need_tune == TUNE_AUTOK_PASS) {
			host->need_tune = TUNE_NONE;
			host->retune_times = 0;
		}
	} else {
		msdc_reset_hw(host);
		if (events & MSDC_INT_ACMDCRCERR) {
			cmd->error = -EILSEQ;
			host->error |= REQ_STOP_EIO;
			host->need_tune = TUNE_CMD_CRC;
#ifdef CONFIG_MACH_MT8173
			host->tune_response_valid = false;
#endif
		} else if (events & MSDC_INT_ACMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_STOP_TMO;
			host->need_tune = TUNE_CMD_TMO;
		}
		dev_err(host->dev,
			"%s: AUTO_CMD%d arg=%08X; rsp %08X; cmd_error=%d\n",
			__func__, cmd->opcode, cmd->arg, rsp[0], cmd->error);
	}
	return cmd->error;
}

static void msdc_track_cmd_data(struct msdc_host *host,
				struct mmc_command *cmd, struct mmc_data *data)
{
	if (host->error)
		dev_dbg(host->dev, "%s: cmd=%d arg=%08X; host->error=0x%08X\n",
			__func__, cmd->opcode, cmd->arg, host->error);
}

static void msdc_request_done(struct msdc_host *host, struct mmc_request *mrq)
{
	unsigned long flags;

	/*
	 * No need check the return value of cancel_delayed_work, as only ONE
	 * path will go here!
	 */
	cancel_delayed_work(&host->req_timeout);

	spin_lock_irqsave(&host->lock, flags);
	host->mrq = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	msdc_track_cmd_data(host, mrq->cmd, mrq->data);
	if (mrq->data)
		msdc_unprepare_data(host, mrq);
	mmc_request_done(host->mmc, mrq);
}

/* returns true if command is fully handled; returns false otherwise */
static bool msdc_cmd_done(struct msdc_host *host, int events,
			  struct mmc_request *mrq, struct mmc_command *cmd)
{
	bool done = false;
	bool sbc_error;
	unsigned long flags;
	u32 *rsp;

	if (mrq->sbc && cmd == mrq->cmd &&
	    (events & (MSDC_INT_ACMDRDY | MSDC_INT_ACMDCRCERR
				   | MSDC_INT_ACMDTMO)))
		msdc_auto_cmd_done(host, events, mrq->sbc);

	sbc_error = mrq->sbc && mrq->sbc->error;

	if (!sbc_error && !(events & (MSDC_INT_CMDRDY
					| MSDC_INT_RSPCRCERR
					| MSDC_INT_CMDTMO)))
		return done;

	spin_lock_irqsave(&host->lock, flags);
	done = !host->cmd;
	host->cmd = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return true;
	rsp = cmd->resp;

	sdr_clr_bits(host->base + MSDC_INTEN, cmd_ints_mask);

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			rsp[0] = readl(host->base + SDC_RESP3);
			rsp[1] = readl(host->base + SDC_RESP2);
			rsp[2] = readl(host->base + SDC_RESP1);
			rsp[3] = readl(host->base + SDC_RESP0);
		} else {
			rsp[0] = readl(host->base + SDC_RESP0);
		}
	}

	if (!sbc_error && !(events & MSDC_INT_CMDRDY)) {
		if (events & MSDC_INT_CMDTMO ||
		    (cmd->opcode != MMC_SEND_TUNING_BLOCK &&
		     cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200 &&
		     !host->hs400_tuning))
			/*
			 * should not clear fifo/interrupt as the tune data
			 * may have alreay come when cmd19/cmd21 gets response
			 * CRC error.
			 */
			msdc_reset_hw(host);
		if (events & MSDC_INT_RSPCRCERR) {
#ifdef CONFIG_MACH_MT8173
			if (cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200)
				host->tune_response_valid = false;
#endif
			cmd->error = -EILSEQ;
			host->error |= REQ_CMD_EIO;
			host->need_tune = TUNE_CMD_CRC;
		} else if (events & MSDC_INT_CMDTMO) {
			cmd->error = -ETIMEDOUT;
			host->error |= REQ_CMD_TMO;
			host->need_tune = TUNE_CMD_TMO;
		}
	} else {
		if (host->need_tune == TUNE_AUTOK_PASS) {
			host->need_tune = TUNE_NONE;
			host->retune_times = 0;
		}
	}
	if (cmd->error && cmd->opcode != MMC_SEND_TUNING_BLOCK &&
	    cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200)
		dev_dbg(host->dev,
				"%s: cmd=%d arg=%08X; rsp %08X; cmd_error=%d\n",
				__func__, cmd->opcode, cmd->arg, rsp[0],
				cmd->error);

	msdc_cmd_next(host, mrq, cmd);
	return true;
}

/* It is the core layer's responsibility to ensure card status
 * is correct before issue a request. but host design do below
 * checks recommended.
 */
static inline bool msdc_cmd_is_ready(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	/* The max busy time we can endure is 500ms */
	unsigned long tmo = jiffies + msecs_to_jiffies(CMD_TIMEOUT);

	if (cmd->opcode == MMC_SEND_STATUS) {
		while ((readl(host->base + SDC_STS) & SDC_STS_CMDBUSY) &&
				time_before(jiffies, tmo))
			cpu_relax();
		if (readl(host->base + SDC_STS) & SDC_STS_CMDBUSY) {
			dev_err(host->dev, "CMD bus busy detected\n");
			msdc_dump_info(host->mmc);
			host->error |= REQ_CMD_BUSY;
			msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
			return false;
		}
	} else {
		tmo = jiffies + msecs_to_jiffies(DAT_TIMEOUT);
		/* R1B or with data, should check SDCBUSY */
		while ((readl(host->base + SDC_STS) & SDC_STS_SDCBUSY) &&
				time_before(jiffies, tmo))
			cpu_relax();
		if (readl(host->base + SDC_STS) & SDC_STS_SDCBUSY) {
			dev_err(host->dev, "Controller busy detected\n");
			dev_info(host->dev, "%s: cmd=%d DMA data\n",
					__func__, cmd->opcode);
			msdc_dump_info_2(host,cmd);
			msdc_dump_info(host->mmc);
			host->error |= REQ_CMD_BUSY;
			msdc_cmd_done(host, MSDC_INT_CMDTMO, mrq, cmd);
			return false;
		}
	}
	return true;
}

static void msdc_start_command(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	u32 rawcmd;
	unsigned long flags;

	WARN_ON(host->cmd);
	host->cmd = cmd;

	mod_delayed_work(system_wq, &host->req_timeout, DAT_TIMEOUT);
	if (!msdc_cmd_is_ready(host, mrq, cmd))
		return;

	if ((readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_TXCNT) >> 16 ||
	    readl(host->base + MSDC_FIFOCS) & MSDC_FIFOCS_RXCNT) {
		dev_err(host->dev, "TX/RX FIFO non-empty before start of IO. Reset\n");
		msdc_reset_hw(host);
	}

	cmd->error = 0;
	rawcmd = msdc_cmd_prepare_raw_cmd(host, mrq, cmd);

	spin_lock_irqsave(&host->lock, flags);
	sdr_set_bits(host->base + MSDC_INTEN, cmd_ints_mask);
	spin_unlock_irqrestore(&host->lock, flags);

	dbg_add_host_log(host->mmc, 0, rawcmd, cmd->arg);

	//if(host && host->mmc && cmd)
	//	dbg_add_host_log(host->mmc, 0, cmd->opcode, cmd->arg);

	writel(cmd->arg, host->base + SDC_ARG);
	writel(rawcmd, host->base + SDC_CMD);
}

static void msdc_cmd_next(struct msdc_host *host,
		struct mmc_request *mrq, struct mmc_command *cmd)
{
	if ((cmd->error &&
	    !(cmd->error == -EILSEQ &&
	      (cmd->opcode == MMC_SEND_TUNING_BLOCK ||
	       cmd->opcode == MMC_SEND_TUNING_BLOCK_HS200 ||
	       host->hs400_tuning))) ||
	    (mrq->sbc && mrq->sbc->error))
		msdc_request_done(host, mrq);
	else if (cmd == mrq->sbc)
		msdc_start_command(host, mrq, mrq->cmd);
	else if (!cmd->data)
		msdc_request_done(host, mrq);
	else
		msdc_start_data(host, mrq, cmd, cmd->data);
}

static void msdc_ops_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->error = 0;
	WARN_ON(host->mrq);
	host->mrq = mrq;

	if (mrq->data)
		msdc_prepare_data(host, mrq);

	/* if SBC is required, we have HW option and SW option.
	 * if HW option is enabled, and SBC does not have "special" flags,
	 * use HW option,  otherwise use SW option
	 */
	if (mrq->sbc && (!mmc_card_mmc(mmc->card) ||
	    (mrq->sbc->arg & 0xFFFF0000)))
		msdc_start_command(host, mrq, mrq->sbc);
	else
		msdc_start_command(host, mrq, mrq->cmd);
}

static void msdc_pre_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;

	if (!data)
		return;

	msdc_prepare_data(host, mrq);
	data->host_cookie |= MSDC_ASYNC_FLAG;
}

static void msdc_post_req(struct mmc_host *mmc, struct mmc_request *mrq,
		int err)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct mmc_data *data;

	data = mrq->data;
	if (!data)
		return;
	if (data->host_cookie) {
		data->host_cookie &= ~MSDC_ASYNC_FLAG;
		msdc_unprepare_data(host, mrq);
	}
}

static void msdc_data_xfer_next(struct msdc_host *host,
				struct mmc_request *mrq, struct mmc_data *data)
{
	if (mmc_op_multi(mrq->cmd->opcode) && mrq->stop && !mrq->stop->error &&
	    !mrq->sbc && !(host->autocmd & MSDC_AUTOCMD12))
		msdc_start_command(host, mrq, mrq->stop);
	else
		msdc_request_done(host, mrq);
}

static bool msdc_data_xfer_done(struct msdc_host *host, u32 events,
				struct mmc_request *mrq, struct mmc_data *data)
{
	struct mmc_command *stop;
	unsigned long flags;
	bool done;
	unsigned int check_data = events &
	    (MSDC_INT_XFER_COMPL | MSDC_INT_DATCRCERR | MSDC_INT_DATTMO
	     | MSDC_INT_DMA_BDCSERR | MSDC_INT_DMA_GPDCSERR
	     | MSDC_INT_DMA_PROTECT);

	spin_lock_irqsave(&host->lock, flags);
	done = !host->data;
	if (check_data)
		host->data = NULL;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		return true;
	stop = data->stop;

	if (check_data || (stop && stop->error)) {
		dev_dbg(host->dev, "DMA status: 0x%8X\n",
				readl(host->base + MSDC_DMA_CFG));
		sdr_set_field(host->base + MSDC_DMA_CTRL, MSDC_DMA_CTRL_STOP,
				1);
		while (readl(host->base + MSDC_DMA_CTRL) & MSDC_DMA_CTRL_STOP)
			cpu_relax();
		sdr_clr_bits(host->base + MSDC_INTEN, data_ints_mask);
		dev_dbg(host->dev, "DMA stop\n");

		if ((events & MSDC_INT_XFER_COMPL) && (!stop || !stop->error)) {
			data->bytes_xfered = data->blocks * data->blksz;
			if (host->need_tune == TUNE_AUTOK_PASS) {
				host->need_tune = TUNE_NONE;
				host->retune_times = 0;
			}
		} else {
			dev_dbg(host->dev, "interrupt events: %x\n", events);
			msdc_reset_hw(host);

			if (mrq->data->flags & MMC_DATA_WRITE)
				host->need_tune = TUNE_DATA_WRITE;
			else
				host->need_tune = TUNE_DATA_READ;

			host->error |= REQ_DAT_ERR;
			data->bytes_xfered = 0;

			if (events & MSDC_INT_DATTMO) {
				data->error = -ETIMEDOUT;
				host->data_timeout_cont++;
			} else if (events & MSDC_INT_DATCRCERR){
				data->error = -EILSEQ;
				host->data_timeout_cont = 0;
			}
			if (mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK &&
			    mrq->cmd->opcode != MMC_SEND_TUNING_BLOCK_HS200) {
				dev_dbg(host->dev, "%s: cmd=%d; blocks=%d",
					__func__, mrq->cmd->opcode,
					data->blocks);
				dev_dbg(host->dev, "data_error=%d xf_size=%d\n",
					(int)data->error, data->bytes_xfered);
			}
		}

		msdc_data_xfer_next(host, mrq, data);
		done = true;
	}
	return done;
}

static void msdc_set_buswidth(struct msdc_host *host, u32 width)
{
	u32 val = readl(host->base + SDC_CFG);

	val &= ~SDC_CFG_BUSWIDTH;

	switch (width) {
	default:
	case MMC_BUS_WIDTH_1:
		val |= (MSDC_BUS_1BITS << 16);
		break;
	case MMC_BUS_WIDTH_4:
		val |= (MSDC_BUS_4BITS << 16);
		break;
	case MMC_BUS_WIDTH_8:
		val |= (MSDC_BUS_8BITS << 16);
		break;
	}

	writel(val, host->base + SDC_CFG);
	dev_dbg(host->dev, "Bus Width = %d", width);
}

extern char	mmc_prod_name_g[8];
static bool msdc_is_0810201_0810202_emmc(void)
{
	/* 0810201: 0x435554423432 equals to CUTB42
	 * 0810202：0x445554423432 equals to DUTB42
	 */

	if (((mmc_prod_name_g[0] == 0x43) || (mmc_prod_name_g[0] == 0x44))
		&& (mmc_prod_name_g[1] == 0x55) && (mmc_prod_name_g[2] == 0x54)
		&& (mmc_prod_name_g[3] == 0x42) && (mmc_prod_name_g[4] == 0x34)
		&& (mmc_prod_name_g[5] == 0x32))
		return true;

	return false;
}
extern bool is_PD2281F_EX;
extern unsigned int mmc_manfid_g;
static int msdc_ops_switch_volt(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int vdd_tmp = 0;
	int ret = 0;

	if (!IS_ERR(mmc->supply.vqmmc)) {
		if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330 &&
		    ios->signal_voltage != MMC_SIGNAL_VOLTAGE_180) {
			dev_err(host->dev, "Unsupported signal voltage!\n");
			return -EINVAL;
		}

		// vivo add for vmc vdd to 3.0v begin
		/* mtk msdc support max volt is 3.15v
		 * origin code set volt to 3.5v
		 * this code force volt form 3.5v to 3.0v
		 */
		if ((ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) &&
		    (mmc->ios.vdd > 18) && !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
			vdd_tmp = mmc->ios.vdd;
			mmc->ios.vdd = 18;
		}
		// vivo add for vmc vdd to 3.0v end
		ret = mmc_regulator_set_vqmmc(mmc, ios);
		if (ret) {
			dev_dbg(host->dev, "Regulator set error %d (%d)\n",
				ret, ios->signal_voltage);
		} else {
			/* Apply different pinctrl settings for different signal voltage */
			if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
				pinctrl_select_state(host->pinctrl, host->pins_uhs);
			else
				pinctrl_select_state(host->pinctrl, host->pins_default);
		}

		// vivo add for vmc vdd to 3.0v begin
		if ((ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) &&
		    (vdd_tmp > 18) && !(mmc->caps & MMC_CAP_NONREMOVABLE)) {
			mmc->ios.vdd = vdd_tmp;
		}
		// vivo add for vmc vdd to 3.0v end
	}
	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		/* 0x9b is YMTC 0x15 is samsung*/
		if (!is_PD2281F_EX) {
			if ((mmc_manfid_g == 0x9b) || msdc_is_0810201_0810202_emmc())
				pinctrl_select_state(host->pinctrl, host->pins_uhs_ymtc);
			else
				pinctrl_select_state(host->pinctrl, host->pins_uhs);
		} else {
			if (mmc_manfid_g == 0x9b)
				pinctrl_select_state(host->pinctrl, host->pins_uhs_ymtc);
			else if (mmc_manfid_g == 0x15)
				pinctrl_select_state(host->pinctrl, host->pins_uhs_samsung);
			else
				pinctrl_select_state(host->pinctrl, host->pins_uhs);
		}
	}

	return ret;
}

static int msdc_card_busy(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 status = readl(host->base + MSDC_PS);

	/* only check if data0 is low */
	return !(status & BIT(16));
}

static void msdc_request_timeout(struct work_struct *work)
{
	struct msdc_host *host = container_of(work, struct msdc_host,
			req_timeout.work);

	/* simulate HW timeout status */
	dev_err(host->dev, "%s: aborting cmd/data/mrq\n", __func__);
	if (host->mrq) {
		dev_err(host->dev, "%s: aborting mrq=%p cmd=%d\n", __func__,
				host->mrq, host->mrq->cmd->opcode);
		if (host->cmd) {
			dev_err(host->dev, "%s: aborting cmd=%d\n",
					__func__, host->cmd->opcode);
			msdc_cmd_done(host, MSDC_INT_CMDTMO, host->mrq,
					host->cmd);
		} else if (host->data) {
			dev_err(host->dev, "%s: abort data: cmd%d; %d blocks\n",
					__func__, host->mrq->cmd->opcode,
					host->data->blocks);
			msdc_data_xfer_done(host, MSDC_INT_DATTMO, host->mrq,
					host->data);
		}
	}
}

static void __msdc_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	unsigned long flags;
	struct msdc_host *host = mmc_priv(mmc);

	spin_lock_irqsave(&host->lock, flags);
	if (enb)
		sdr_set_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
	else
		sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INTEN_SDIOIRQ);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void msdc_enable_sdio_irq(struct mmc_host *mmc, int enb)
{
	struct msdc_host *host = mmc_priv(mmc);

	__msdc_enable_sdio_irq(mmc, enb);

	if (enb)
		pm_runtime_get_noresume(host->dev);
	else
		pm_runtime_put_noidle(host->dev);
}

#ifdef CONFIG_MMC_CQHCI
static irqreturn_t msdc_cmdq_irq(struct msdc_host *host, u32 intsts)
{
	int cmd_err = 0, dat_err = 0;

	if (intsts & MSDC_INT_RSPCRCERR) {
		cmd_err = (unsigned int)-EILSEQ;
		pr_err("XXX CMD CRC");
	} else if (intsts & MSDC_INT_CMDTMO) {
		cmd_err = (unsigned int)-ETIMEDOUT;
		pr_err("XXX CMD TIMEOUT");
	}

	if (intsts & MSDC_INT_DATCRCERR) {
		dat_err = (unsigned int)-EILSEQ;
		pr_err("XXX DATA CRC");
	} else if (intsts & MSDC_INT_DATTMO) {
		dat_err = (unsigned int)-ETIMEDOUT;
		pr_err("XXX DATA TIMEOUT");
	}

	if (cmd_err || dat_err) {
		pr_err("cmd_err = %d, dat_err =%d, intsts = 0x%x",
			cmd_err, dat_err, intsts);
		writel(intsts, host->base + MSDC_INT); /* clear interrupts */
	}

	return cqhci_irq(host->mmc, 0, cmd_err, dat_err);
}
#endif

static irqreturn_t msdc_irq(int irq, void *dev_id)
{
	struct msdc_host *host = (struct msdc_host *) dev_id;

	while (true) {
		unsigned long flags;
		struct mmc_request *mrq;
		struct mmc_command *cmd;
		struct mmc_data *data;
		u32 events, event_mask;

		spin_lock_irqsave(&host->lock, flags);
		events = readl(host->base + MSDC_INT);
		event_mask = readl(host->base + MSDC_INTEN);
		/* clear interrupts */
		writel(events & event_mask, host->base + MSDC_INT);

		mrq = host->mrq;
		cmd = host->cmd;
		data = host->data;
		spin_unlock_irqrestore(&host->lock, flags);

		if ((events & event_mask) & MSDC_INT_SDIOIRQ) {
			__msdc_enable_sdio_irq(host->mmc, 0);
			sdio_signal_irq(host->mmc);
		}

		if (!(events & (event_mask & ~MSDC_INT_SDIOIRQ)))
			break;

#ifdef CONFIG_MMC_CQHCI
		if (host->cqhci && (events & MSDC_INT_CMDQ)) {
			msdc_cmdq_irq(host, events);
			/* clear interrupts */
			writel(events, host->base + MSDC_INT);
			return IRQ_HANDLED;
		}
#endif

		if (!mrq) {
			dev_err(host->dev,
				"%s: MRQ=NULL; events=%08X; event_mask=%08X\n",
				__func__, events, event_mask);
			WARN_ON(1);
			break;
		}

		dev_dbg(host->dev, "%s: events=%08X,event_mask=%08X\n", __func__, events, event_mask);

		if (cmd)
			msdc_cmd_done(host, events, mrq, cmd);
		else if (data)
			msdc_data_xfer_done(host, events, mrq, data);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_MACH_MT8173
/* MT8173E1 Chip do not support SD data tune */
static bool is_e1_chip(void)
{
	bool e1_chip = false;

	if (mt_get_chip_sw_ver() == CHIP_SW_VER_01)
		e1_chip = true;

	return e1_chip;
}
#endif
static void msdc_init_hw(struct msdc_host *host)
{
	u32 val;
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	/* Configure to MMC/SD mode */
	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_MODE);

	/* Reset */
	msdc_reset_hw(host);

	/* Disable card detection */
	sdr_clr_bits(host->base + MSDC_PS, MSDC_PS_CDEN);

	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);
	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);

	if (host->top_base) {
		writel(0, host->top_base + EMMC_TOP_CONTROL);
		writel(0, host->top_base + EMMC_TOP_CMD);
	} else {
		writel(0, host->base + tune_reg);
	}
	writel(0, host->base + MSDC_IOCON);
	sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_DDLSEL, 0);
	/* modify CKGEN delay also need protect, or will data timeout */
	sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	/*
	 * As src_clk/HCLK use the same bit to gate/ungate,
	 * So if want to only gate src_clk, need gate its parent(mux).
	 */
	if (host->src_clk_cg)
		clk_disable_unprepare(host->src_clk_cg);
	else
		clk_disable_unprepare(clk_get_parent(host->src_clk));
	writel(0x403c0046, host->base + MSDC_PATCH_BIT);
	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_CKGEN_MSDC_DLY_SEL, 1);

	if (host->src_clk_cg)
		clk_prepare_enable(host->src_clk_cg);
	else
		clk_prepare_enable(clk_get_parent(host->src_clk));

	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB))
		cpu_relax();
	sdr_set_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	writel(0xfffe4089, host->base + MSDC_PATCH_BIT1);
	sdr_set_bits(host->base + EMMC50_CFG0, EMMC50_CFG_CFCSTS_SEL);

	if (host->dev_comp->stop_clk_fix) {
		sdr_set_field(host->base + MSDC_PATCH_BIT1,
			      MSDC_PATCH_BIT1_STOP_DLY, 3);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_WRVALIDSEL);
		sdr_clr_bits(host->base + SDC_FIFO_CFG,
			     SDC_FIFO_CFG_RDVALIDSEL);
	}

	if (host->dev_comp->busy_check)
		sdr_clr_bits(host->base + MSDC_PATCH_BIT1, (1 << 7));

	if (host->dev_comp->async_fifo) {
		sdr_set_field(host->base + MSDC_PATCH_BIT2,
			      MSDC_PB2_RESPWAIT, 3);
		if (host->dev_comp->enhance_rx) {
			if (host->top_base)
				sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
					     SDC_RX_ENH_EN);
			else
				sdr_set_bits(host->base + SDC_ADV_CFG0,
					     SDC_RX_ENHANCE_EN);
		} else {
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_RESPSTSENSEL, 2);
			sdr_set_field(host->base + MSDC_PATCH_BIT2,
				      MSDC_PB2_CRCSTSENSEL, 2);
		}
		/* use async fifo, then no need tune internal delay */
		sdr_clr_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGRESP);
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PATCH_BIT2_CFGCRCSTS);
	}

	if (host->dev_comp->support_64g)
		sdr_set_bits(host->base + MSDC_PATCH_BIT2,
			     MSDC_PB2_SUPPORT_64G);
	if (host->dev_comp->data_tune) {
		if (host->top_base) {
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_DAT_RD_RXDLY_SEL);
			sdr_clr_bits(host->top_base + EMMC_TOP_CONTROL,
				     DATA_K_VALUE_SEL);
			sdr_set_bits(host->top_base + EMMC_TOP_CMD,
				     PAD_CMD_RD_RXDLY_SEL);
		} else {
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RD_SEL |
				     MSDC_PAD_TUNE_CMD_SEL);
		}
	} else {
		/* choose clock tune */
		if (host->top_base)
			sdr_set_bits(host->top_base + EMMC_TOP_CONTROL,
				     PAD_RXDLY_SEL);
		else
			sdr_set_bits(host->base + tune_reg,
				     MSDC_PAD_TUNE_RXDLYSEL);
	}

	/* Configure to enable SDIO mode.
	 * it's must otherwise sdio cmd5 failed
	 */
	sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIO);

	/* Config SDIO device detect interrupt function */
	if (host->mmc->caps & MMC_CAP_SDIO_IRQ)
		sdr_set_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);
	else
		sdr_clr_bits(host->base + SDC_CFG, SDC_CFG_SDIOIDE);

	/* Configure to default data timeout */
	sdr_set_field(host->base + SDC_CFG, SDC_CFG_DTOC, 3);

	host->def_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	if (host->top_base) {
		host->def_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->def_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->saved_tune_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
	} else {
		host->def_tune_para.pad_tune = readl(host->base + tune_reg);
		host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	}
#ifdef CONFIG_MACH_MT8173
	if (!is_e1_chip() && host->host_id == 1) {
		/* SD use data tune */
		top_sdr_set_bits(host->top_base, BIT(31));
		top_sdr_set_field(host->top_base + 8, MSDC1_CMD_DELAY, 0);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT0_DELAY, 0);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT1_DELAY, 0);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT2_DELAY, 0);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT3_DELAY, 0);
	}
#endif
	if (host->need_tune & TUNE_FAIL_RETUNE) {
		host->need_tune &= ~TUNE_FAIL_RETUNE;
		dev_info(host->dev, "%s: skip reset need_tune: %d and retune_times: %d\n",
			__func__, host->need_tune, host->retune_times);
	} else {
		host->need_tune = TUNE_NONE;
		host->retune_times = 0;
	}

	dev_dbg(host->dev, "init hardware done!");
}

static void msdc_deinit_hw(struct msdc_host *host)
{
	u32 val;
	/* Disable and clear all interrupts */
	writel(0, host->base + MSDC_INTEN);

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);
}

/* init gpd and bd list in msdc_drv_probe */
static void msdc_init_gpd_bd(struct msdc_host *host, struct msdc_dma *dma)
{
	struct mt_gpdma_desc *gpd = dma->gpd;
	struct mt_bdma_desc *bd = dma->bd;
	dma_addr_t dma_addr;
	int i;

	memset(gpd, 0, sizeof(struct mt_gpdma_desc) * 2);

	dma_addr = dma->gpd_addr + sizeof(struct mt_gpdma_desc);
	gpd->gpd_info = GPDMA_DESC_BDP; /* hwo, cs, bd pointer */
	/* gpd->next is must set for desc DMA
	 * That's why must alloc 2 gpd structure.
	 */
	gpd->next = lower_32_bits(dma_addr);
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;

	dma_addr = dma->bd_addr;
	gpd->ptr = lower_32_bits(dma->bd_addr); /* physical address */
	if (host->dev_comp->support_64g)
		gpd->gpd_info |= (upper_32_bits(dma_addr) & 0xf) << 28;

	memset(bd, 0, sizeof(struct mt_bdma_desc) * MAX_BD_NUM);
	for (i = 0; i < (MAX_BD_NUM - 1); i++) {
		dma_addr = dma->bd_addr + sizeof(*bd) * (i + 1);
		bd[i].next = lower_32_bits(dma_addr);
		if (host->dev_comp->support_64g)
			bd[i].bd_info |= (upper_32_bits(dma_addr) & 0xf) << 24;
	}
}

static int msdc_get_cd(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	int val;

	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		host->card_inserted = 1;
		goto end;
	} else if (!host->internal_cd) {
		host->card_inserted = mmc_gpio_get_cd(mmc);
	} else {
		val = readl(host->base + MSDC_PS) & MSDC_PS_CDSTS;
		if (mmc->caps2 & MMC_CAP2_CD_ACTIVE_HIGH)
			host->card_inserted = !!val;
		else
			host->card_inserted = !val;
	}

	if (host->block_bad_card)
		host->card_inserted = 0;
end:
	pr_info(
		"%s:card status:%s block bad card<%d> trigger card event<%d>",
		__func__, host->card_inserted ? "inserted" : "removed",
		host->block_bad_card, mmc->trigger_card_event);

	return host->card_inserted;
}

static void msdc_reset_bad_sd_detecter(struct msdc_host *host)
{
	u32 i = 0;

	if (host == NULL) {
		pr_info("WARN: host is NULL at %s\n", __func__);
		return;
	}

	host->block_bad_card = 0;
	for (i = 0; i < BAD_SD_DETECTER_COUNT; i++)
		bad_sd_detecter[i] = 0;
}

static void msdc_ops_card_event(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	host->power_cycle_cnt = 0;
	host->data_timeout_cont = 0;
	host->is_autok_done = 0;
	msdc_reset_bad_sd_detecter(host);
	msdc_get_cd(mmc);
}

void msdc_set_bad_card_and_remove(struct msdc_host *host)
{
	struct mmc_host *mmc = host->mmc;

	if (host->card_inserted) {
		host->block_bad_card = 1;
		host->card_inserted = 0;
	}

	if ((mmc == NULL) || (mmc->card == NULL)) {
		pr_info("WARN: mmc or card is NULL");
		return;
	}

	if (mmc->card) {
		mmc_card_set_removed(mmc->card);
		pr_info("%s, schedule mmc_rescan\n", __func__);
		mmc_detect_change(mmc, msecs_to_jiffies(1000));
		if (host->block_bad_card)
			pr_info("%s, remove the bad card, block_bad_card=%d, card_inserted=%d",
				__func__, host->block_bad_card, host->card_inserted);
	}
}

int sdcard_hw_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;

	host->card_inserted = msdc_get_cd(mmc);

	if (!(host->card_inserted)) {
		pr_info("card is not inserted!\n");
		msdc_set_bad_card_and_remove(host);
		ret = -1;
		return ret;
	}

	ret = mmc_hw_reset(mmc);
	if (ret) {
		if (++host->power_cycle_cnt
			> MSDC_MAX_POWER_CYCLE_FAIL_CONTINUOUS)
			msdc_set_bad_card_and_remove(host);
		pr_info(
			"msdc%d power reset (%d) failed, block_bad_card = %d\n",
			host->mmc->index, host->power_cycle_cnt, host->block_bad_card);
	} else {
		host->power_cycle_cnt = 0;
		pr_info("msdc%d power reset success\n", host->mmc->index);
	}

	return ret;
}

int msdc_data_timeout_cont_chk(struct msdc_host *host)
{
	if ((host->mmc->host_function == MSDC_SD) &&
		(host->data_timeout_cont >= MSDC_MAX_DATA_TIMEOUT_CONTINUOUS)) {
		pr_info("force remove bad card, data timeout continuous %d",
			host->data_timeout_cont);
		msdc_set_bad_card_and_remove(host);
		return 1;
	}

	return 0;
}

int sdcard_reset_tuning_host(struct mmc_host *mmc)
{
	char *remove_cap;

	if (mmc->ios.timing == MMC_TIMING_UHS_SDR104
		&& mmc->caps & MMC_CAP_UHS_SDR104) {
		mmc->caps &= ~MMC_CAP_UHS_SDR104;
		remove_cap = "UHS_SDR104";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_DDR50
		&& mmc->caps & MMC_CAP_UHS_DDR50) {
		mmc->caps &= ~MMC_CAP_UHS_DDR50;
		remove_cap = "UHS_DDR50";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_SDR50
		&& mmc->caps & MMC_CAP_UHS_SDR50) {
		mmc->caps &= ~MMC_CAP_UHS_SDR50;
		remove_cap = "UHS_SDR50";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_SDR25
		&& mmc->caps & MMC_CAP_UHS_SDR25) {
		mmc->caps &= ~MMC_CAP_UHS_SDR25;
		remove_cap = "UHS_SDR25";
	} else {
		remove_cap = "none";
	}

	pr_info("msdc%d: remove %s mode from host\n", mmc->index, remove_cap);
	return 0;
}

int sdcard_reset_tuning_card(struct mmc_host *mmc)
{
	char *remove_cap;

	if (mmc->ios.timing == MMC_TIMING_UHS_SDR104
		&& mmc->card->sw_caps.sd3_bus_mode & SD_MODE_UHS_SDR104) {
		mmc->card->sw_caps.sd3_bus_mode &= ~SD_MODE_UHS_SDR104;
		remove_cap = "UHS_SDR104";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_DDR50
		&& mmc->card->sw_caps.sd3_bus_mode & SD_MODE_UHS_DDR50) {
		mmc->card->sw_caps.sd3_bus_mode &= ~SD_MODE_UHS_DDR50;
		remove_cap = "UHS_DDR50";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_SDR50
		&& mmc->card->sw_caps.sd3_bus_mode & SD_MODE_UHS_SDR50) {
		mmc->card->sw_caps.sd3_bus_mode &= ~SD_MODE_UHS_SDR50;
		remove_cap = "UHS_SDR50";
	} else if (mmc->ios.timing == MMC_TIMING_UHS_SDR25
		&& mmc->card->sw_caps.sd3_bus_mode & SD_MODE_UHS_SDR25) {
		mmc->card->sw_caps.sd3_bus_mode &= ~SD_MODE_UHS_SDR25;
		remove_cap = "UHS_SDR25";
	} else {
		remove_cap = "none";
	}

	pr_info("msdc%d: remove %s mode then reinit card\n", mmc->index, remove_cap);
	return 0;
}

/* SDcard will change speed mode and power reset
 * UHS card
 *    UHS_SDR104 --> UHS_DDR50 --> UHS_SDR50 --> UHS_SDR25
 * HS card
 *    50MHz --> 25MHz --> 12.5MHz --> 6.25MHz
 */
int sdcard_reset_tuning(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret = 0;

    pr_notice("%s: msdc%d reset tuning to down mode from mode %d\n",
               __func__, mmc->index, mmc->ios.timing);
	if (!mmc->card) {
               sdcard_reset_tuning_host(mmc);
       } else if (mmc_card_uhs(mmc->card)) {
               sdcard_reset_tuning_card(mmc);
	} else if (mmc_card_hs(mmc->card)) {
		if (mmc->card->sw_caps.hs_max_dtr >= HIGH_SPEED_MAX_DTR / 4)
			mmc->card->sw_caps.hs_max_dtr /= 2;
		pr_info("msdc%d: set hs speed %dhz then reinit card\n",
			host->mmc->index, mmc->card->sw_caps.hs_max_dtr);
	} else {
		pr_info("msdc%d: ds card just reinit card\n", host->mmc->index);
	}

	/* force remove card for continuous data timeout */
	ret = msdc_data_timeout_cont_chk(host);
	if (ret) {
		ret = -1;
		goto done;
	}

	/* power cycle sdcard */
	ret = sdcard_hw_reset(mmc);
	if (ret) {
		ret = -1;
		goto done;
	}

done:
	return ret;
}

static int msdc_detect_bad_sd(struct msdc_host *host, u32 condition)
{
	unsigned long time_current = jiffies;
	int ret = 0;

	if (host == NULL) {
		pr_info("WARN: host is NULL at %s\n", __func__);
		ret = -1;
		goto end;
	}

	if (condition >= BAD_SD_DETECTER_COUNT) {
		pr_info("msdc1: BAD_SD_DETECTER_COUNT is %d, need check it's definition at %s\n",
			BAD_SD_DETECTER_COUNT, __func__);
		ret = -1;
		goto end;
	}

	if (bad_sd_forget[condition]
		&& time_after(time_current,
		(bad_sd_timer[condition] + bad_sd_forget[condition] * HZ)))
		bad_sd_detecter[condition] = 0;
	bad_sd_timer[condition] = time_current;

	if (++(bad_sd_detecter[condition]) >= bad_sd_tolerance[condition]) {
		msdc_set_bad_card_and_remove(host);
		ret = -1;
	}
	pr_info("%s:bad_sd_detecter:%d\n", __func__, bad_sd_detecter[condition]);

end:
	return ret;
}

static void msdc_ops_init_card(struct mmc_host *mmc, struct mmc_card *card)
{
	struct device *dev = mmc->parent;
	struct msdc_host *host = mmc_priv(mmc);

	if (memcmp(host->last_cid, card->raw_cid, sizeof(host->last_cid)) != 0) {
		pr_info("msdc%d: SD card is changed, reset mmc->caps\n", mmc->index);
		memcpy(host->last_cid, card->raw_cid, sizeof(card->raw_cid));
		if (device_property_read_bool(dev, "sd-uhs-sdr12"))
			mmc->caps |= MMC_CAP_UHS_SDR12;
		if (device_property_read_bool(dev, "sd-uhs-sdr25"))
			mmc->caps |= MMC_CAP_UHS_SDR25;
		if (device_property_read_bool(dev, "sd-uhs-sdr50"))
			mmc->caps |= MMC_CAP_UHS_SDR50;
		if (device_property_read_bool(dev, "sd-uhs-sdr104"))
			mmc->caps |= MMC_CAP_UHS_SDR104;
		if (device_property_read_bool(dev, "sd-uhs-ddr50"))
			mmc->caps |= MMC_CAP_UHS_DDR50;
	}
}

void msdc_ops_set_bad_card_and_remove(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	msdc_set_bad_card_and_remove(host);
}

static void msdc_ops_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;

	msdc_set_buswidth(host, ios->bus_width);
	dev_err(host->dev, "set power_mode = %d\n", ios->power_mode);

	/* Suspend/Resume will do power off/on */
	switch (ios->power_mode) {
	case MMC_POWER_UP:
		if (!IS_ERR(mmc->supply.vmmc)) {
			msdc_init_hw(host);
			ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc,
					ios->vdd);
			if (ret) {
				dev_err(host->dev, "Failed to set vmmc power!\n");
				return;
			}
		}
		break;
	case MMC_POWER_ON:
		if (!IS_ERR(mmc->supply.vqmmc) && !host->vqmmc_enabled) {
			ret = regulator_enable(mmc->supply.vqmmc);
			if (ret)
				dev_err(host->dev, "Failed to set vqmmc power!\n");
			else
				host->vqmmc_enabled = true;
		}
		break;
	case MMC_POWER_OFF:
		if (!IS_ERR(mmc->supply.vmmc))
			mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);

		if (!IS_ERR(mmc->supply.vqmmc) && host->vqmmc_enabled) {
			regulator_disable(mmc->supply.vqmmc);
			host->vqmmc_enabled = false;
		}
		break;
	default:
		break;
	}

	if (host->mclk != ios->clock || host->timing != ios->timing)
		msdc_set_mclk(host, ios->timing, ios->clock);
}

static u32 test_delay_bit(u32 delay, u32 bit)
{
	bit %= PAD_DELAY_MAX;
	return delay & (1 << bit);
}

static int get_delay_len(u32 delay, u32 start_bit)
{
	int i;

	for (i = 0; i < (PAD_DELAY_MAX - start_bit); i++) {
		if (test_delay_bit(delay, start_bit + i) == 0)
			return i;
	}
	return PAD_DELAY_MAX - start_bit;
}

static struct msdc_delay_phase get_best_delay(struct msdc_host *host, u32 delay)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xff;
	struct msdc_delay_phase delay_phase = { 0, };

	if (delay == 0) {
		dev_err(host->dev, "phase error: [map:%x]\n", delay);
		delay_phase.final_phase = final_phase;
		return delay_phase;
	}

	while (start < PAD_DELAY_MAX) {
		len = get_delay_len(delay, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
		if (len >= 12 && start_final < 4)
			break;
	}

	/* The rule is that to find the smallest delay cell */
	if (start_final == 0)
		final_phase = (start_final + len_final / 3) % PAD_DELAY_MAX;
	else
		final_phase = (start_final + len_final / 2) % PAD_DELAY_MAX;
	dev_info(host->dev, "phase: [map:%x] [maxlen:%d] [final:%d]\n",
		 delay, len_final, final_phase);

	delay_phase.maxlen = len_final;
	delay_phase.start = start_final;
	delay_phase.final_phase = final_phase;
	return delay_phase;
}
#ifdef CONFIG_MACH_MT8173
static int sd_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 cmd_delay = 0;
	struct msdc_delay_phase final_cmd_delay = { 0,};
	u8 final_delay;
	u32 tune_reg = MSDC_PAD_TUNE;
	int cmd_err;
	int i, j;

	if (mmc->ios.timing == MMC_TIMING_UHS_SDR104) {
		/* set cmd ta to 4 */
		sdr_set_field(host->base + MSDC_PATCH_BIT1,
			      MSDC_PATCH_BIT1_CMDTA, 4);
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);
		sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_RSPL,
			      host->hs200_cmd_resp_sel);
	}

	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		top_sdr_set_field(host->top_base + 8, MSDC1_CMD_DELAY, i);
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				cmd_delay |= (1 << i);
			} else {
				cmd_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_cmd_delay = get_best_delay(host, cmd_delay);
	final_delay = final_cmd_delay.final_phase;
	top_sdr_set_field(host->top_base + 8, MSDC1_CMD_DELAY, final_delay);

	dev_info(host->dev, "Final cmd pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int sd_tune_data(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 data_delay = 0;
	struct msdc_delay_phase final_data_delay = { 0, };
	u8 final_delay;
	u32 tune_reg = MSDC_PAD_TUNE;
	int i, ret = 0;
	u32 edge = host->hs200_cmd_resp_sel;

	sdr_set_field(host->base + MSDC_PATCH_BIT,
		      MSDC_INT_DAT_LATCH_CK_SEL, 1);
	if (mmc->ios.timing == MMC_TIMING_UHS_SDR104) {
		/* use same latch edge as response */
		sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_DSPL, edge);
		sdr_set_field(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL, edge);
		sdr_set_field(host->base + MSDC_PATCH_BIT1,
			      MSDC_PATCH_BIT1_WRTA, 3);
		/* write internal delay is the same as cmd internal delay */
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_DATWRDLY,
			      host->hs200_cmd_int_delay);
	}

	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT0_DELAY, i);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT1_DELAY, i);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT2_DELAY, i);
		top_sdr_set_field(host->top_base + 4, MSDC1_DAT3_DELAY, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			data_delay |= (1 << i);
	}
	final_data_delay = get_best_delay(host, data_delay);
	final_delay = final_data_delay.final_phase;
	top_sdr_set_field(host->top_base + 4, MSDC1_DAT0_DELAY, final_delay);
	top_sdr_set_field(host->top_base + 4, MSDC1_DAT1_DELAY, final_delay);
	top_sdr_set_field(host->top_base + 4, MSDC1_DAT2_DELAY, final_delay);
	top_sdr_set_field(host->top_base + 4, MSDC1_DAT3_DELAY, final_delay);

	dev_info(host->dev, "edge: %d, data pad delay: 0x%x\n",
		 edge, data_delay);
	return final_delay == 0xff ? -EIO : 0;
}
#endif

static inline void msdc_set_cmd_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->top_base)
		sdr_set_field(host->top_base + EMMC_TOP_CMD, PAD_CMD_RXDLY,
			      value);
	else
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRDLY,
			      value);
}

static inline void msdc_set_data_delay(struct msdc_host *host, u32 value)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	if (host->top_base)
		sdr_set_field(host->top_base + EMMC_TOP_CONTROL,
			      PAD_DAT_RD_RXDLY, value);
	else
		sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_DATRRDLY,
			      value);
}

static int msdc_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	struct msdc_delay_phase internal_delay_phase;
	u8 final_delay, final_maxlen;
	u32 internal_delay = 0;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	int cmd_err = 0;
	int i, j;

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		msdc_set_cmd_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				rise_delay |= (1 << i);
			} else {
				rise_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
	if (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4))
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		msdc_set_cmd_delay(host, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 3; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				fall_delay |= (1 << i);
			} else {
				fall_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_fall_delay.maxlen >= 12 && final_fall_delay.start < 4)
		final_maxlen = final_fall_delay.maxlen;
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		final_delay = final_fall_delay.final_phase;
	}
	msdc_set_cmd_delay(host, final_delay);

	if (host->dev_comp->async_fifo || host->hs200_cmd_int_delay)
		goto skip_internal;

	for (i = 0; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + tune_reg,
			      MSDC_PAD_TUNE_CMDRRDLY, i);
		mmc_send_tuning(mmc, opcode, &cmd_err);
		if (!cmd_err)
			internal_delay |= (1 << i);
	}
	dev_dbg(host->dev, "Final internal delay: 0x%x\n", internal_delay);
	internal_delay_phase = get_best_delay(host, internal_delay);
	sdr_set_field(host->base + tune_reg, MSDC_PAD_TUNE_CMDRRDLY,
		      internal_delay_phase.final_phase);
skip_internal:
	dev_dbg(host->dev, "Final cmd pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int hs400_tune_response(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 cmd_delay = 0;
	struct msdc_delay_phase final_cmd_delay = { 0,};
	u8 final_delay;
	int cmd_err = 0;
	int i, j;

	/* select EMMC50 PAD CMD tune */
	sdr_set_bits(host->base + PAD_CMD_TUNE, BIT(0));
	sdr_set_field(host->base + MSDC_PATCH_BIT1, MSDC_PATCH_BIT1_CMDTA, 2);

	if (mmc->ios.timing == MMC_TIMING_MMC_HS200 ||
	    mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		sdr_set_field(host->base + MSDC_PAD_TUNE,
			      MSDC_PAD_TUNE_CMDRRDLY,
			      host->hs200_cmd_int_delay);

	if (host->hs400_cmd_resp_sel_rising)
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	else
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
#ifdef CONFIG_MACH_MT8173
	/*
	 * As Now tune 10 times for each pad macro, will tune 320 times for
	 * all steps and will cost more than 20ms, so do not re-tune if current
	 * result is valid and no response CRC error occurs.
	 */
	if (host->tune_response_valid)
		goto skip_tune;
#endif
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		sdr_set_field(host->base + PAD_CMD_TUNE,
			      PAD_CMD_TUNE_RX_DLY3, i);
		/*
		 * Using the same parameters, it may sometimes pass the test,
		 * but sometimes it may fail. To make sure the parameters are
		 * more stable, we test each set of parameters 3 times.
		 */
		for (j = 0; j < 10; j++) {
			mmc_send_tuning(mmc, opcode, &cmd_err);
			if (!cmd_err) {
				cmd_delay |= (1 << i);
			} else {
				cmd_delay &= ~(1 << i);
				break;
			}
		}
	}
	final_cmd_delay = get_best_delay(host, cmd_delay);
	sdr_set_field(host->base + PAD_CMD_TUNE, PAD_CMD_TUNE_RX_DLY3,
		      final_cmd_delay.final_phase);
	final_delay = final_cmd_delay.final_phase;

#ifdef CONFIG_MACH_MT8173
	if (cmd_delay != 0xffffffff && final_delay != 0xff) {
		host->tune_response_delay = final_delay;
		host->tune_response_valid = true;
	}
#endif
	dev_dbg(host->dev, "Final cmd pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
#ifdef CONFIG_MACH_MT8173
skip_tune:
	sdr_set_field(host->base + PAD_CMD_TUNE, PAD_CMD_TUNE_RX_DLY3,
		      host->tune_response_delay);
#endif
	return 0;
}

static int msdc_tune_data(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	u8 final_delay, final_maxlen;
	int i, ret;

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		msdc_set_data_delay(host, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			rise_delay |= (1 << i);
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
	if (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4))
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		msdc_set_data_delay(host, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			fall_delay |= (1 << i);
	}
	final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_DSPL);
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_W_DSPL);
		final_delay = final_fall_delay.final_phase;
	}
	msdc_set_data_delay(host, final_delay);

	dev_dbg(host->dev, "Final data pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

/*
 * MSDC IP which supports data tune + async fifo can do CMD/DAT tune
 * together, which can save the tuning time.
 */
static int msdc_tune_together(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 rise_delay = 0, fall_delay = 0;
	struct msdc_delay_phase final_rise_delay, final_fall_delay = { 0,};
	u8 final_delay, final_maxlen;
	int i, ret;

	sdr_set_field(host->base + MSDC_PATCH_BIT, MSDC_INT_DAT_LATCH_CK_SEL,
		      host->latch_ck);

	sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_clr_bits(host->base + MSDC_IOCON,
		     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
	for (i = 0 ; i < PAD_DELAY_MAX; i++) {
		msdc_set_cmd_delay(host, i);
		msdc_set_data_delay(host, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			rise_delay |= (1 << i);
	}
	final_rise_delay = get_best_delay(host, rise_delay);
	/* if rising edge has enough margin, then do not scan falling edge */
	if (final_rise_delay.maxlen >= 12 ||
	    (final_rise_delay.start == 0 && final_rise_delay.maxlen >= 4))
		goto skip_fall;

	sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
	sdr_set_bits(host->base + MSDC_IOCON,
		     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		msdc_set_cmd_delay(host, i);
		msdc_set_data_delay(host, i);
		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (!ret)
			fall_delay |= (1 << i);
	}
	final_fall_delay = get_best_delay(host, fall_delay);

skip_fall:
	final_maxlen = max(final_rise_delay.maxlen, final_fall_delay.maxlen);
	if (final_maxlen == final_rise_delay.maxlen) {
		sdr_clr_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_clr_bits(host->base + MSDC_IOCON,
			     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
		final_delay = final_rise_delay.final_phase;
	} else {
		sdr_set_bits(host->base + MSDC_IOCON, MSDC_IOCON_RSPL);
		sdr_set_bits(host->base + MSDC_IOCON,
			     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
		final_delay = final_fall_delay.final_phase;
	}

	msdc_set_cmd_delay(host, final_delay);
	msdc_set_data_delay(host, final_delay);

	dev_dbg(host->dev, "Final pad delay: %x\n", final_delay);
	return final_delay == 0xff ? -EIO : 0;
}

static int msdc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct msdc_host *host = mmc_priv(mmc);
	int ret;
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	dev_info(host->dev, "%s\n", __func__);

	if (host->dev_comp->data_tune && host->dev_comp->async_fifo) {
		if (host->need_tune && (mmc->host_function == MSDC_SD)) {
			if (msdc_get_cd(mmc) == 0)
				goto end;

			ret = msdc_detect_bad_sd(host, 0);
			if (ret)
				goto end;
			pr_info("msdc%d: %s retune %d times\n",
 				mmc->index, mmc->host_function? "SD" : "eMMC",
 				host->retune_times);
 			++host->retune_times;
		}
		pr_info("mmc%d: need_tune %d:, retune times: %d\n", mmc->index, host->need_tune, host->retune_times);
		ret = msdc_tune_together(mmc, opcode);

		if (mmc->host_function == MSDC_SD) {
			if (bad_sd_detecter[0] >= 2) {
				pr_info("msdc%d reset tuning since bad_sd_detecter:%d\n",
					mmc->index, bad_sd_detecter[0]);
				msdc_reset_bad_sd_detecter(host);
				sdcard_reset_tuning(mmc);
				ret = -EAGAIN;
			} else if (!ret) {
				pr_info("msdc%d autok pass\n", mmc->index);
				host->need_tune = TUNE_AUTOK_PASS;
			} else if (host->retune_times >= 4) {
				pr_info("msdc%d reset tuning since tuning times more than 4\n",
					mmc->index);
				sdcard_reset_tuning(mmc);
				ret = -EAGAIN;
			} else if (!mmc->card && mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
				pr_info("msdc%d reset tuning since tuning fail DDR50\n", mmc->index);
				sdcard_reset_tuning(mmc);
				ret = -EAGAIN;
			} else {
				pr_info("msdc%d tuning fail\n", mmc->index);
				host->need_tune |= TUNE_FAIL_RETUNE;
				ret = -EAGAIN;
			}
		}

		if (host->hs400_mode) {
			sdr_clr_bits(host->base + MSDC_IOCON,
				     MSDC_IOCON_DSPL | MSDC_IOCON_W_DSPL);
			msdc_set_data_delay(host, 0);
		}
		goto tune_done;
	}
	if (host->hs400_mode &&
	    host->dev_comp->hs400_tune)
		ret = hs400_tune_response(mmc, opcode);
	else
		ret = msdc_tune_response(mmc, opcode);
	if (ret == -EIO) {
		dev_info(host->dev, "Tune response fail!\n");
		msdc_dump_info(mmc);
		return ret;
	}
	if (host->hs400_mode == false) {
		ret = msdc_tune_data(mmc, opcode);
		if (ret == -EIO)
			dev_err(host->dev, "Tune data fail!\n");
	}
#ifdef CONFIG_MACH_MT8173
	/*
	 * Note that I assume that the code go here is running at MT8173
	 */
	if (host->hs400_mode &&
	    host->dev_comp->hs400_tune) {
		ret = hs400_tune_response(mmc, opcode);
	} else {
		if (host->host_id == 0 || is_e1_chip())
			ret = msdc_tune_response(mmc, opcode);
		else
			ret = sd_tune_response(mmc, opcode);
	}
	if (ret == -EIO) {
		dev_info(host->dev, "Tune response fail!\n");
		msdc_dump_info(mmc);
		return ret;
	}
	if (host->hs400_mode == false) {
		if (host->host_id == 0 || is_e1_chip())
			ret = msdc_tune_data(mmc, opcode);
		else
			ret = sd_tune_data(mmc, opcode);
		if (ret == -EIO){
			dev_info(host->dev, "Tune data fail!\n");
			msdc_dump_info(mmc);
		}
	}
#endif

tune_done:
	host->saved_tune_para.iocon = readl(host->base + MSDC_IOCON);
	host->saved_tune_para.pad_tune = readl(host->base + tune_reg);
	host->saved_tune_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
#ifndef CONFIG_MACH_MT8173
	if (host->top_base) {
		host->saved_tune_para.emmc_top_control = readl(host->top_base +
				EMMC_TOP_CONTROL);
		host->saved_tune_para.emmc_top_cmd = readl(host->top_base +
				EMMC_TOP_CMD);
	}
#endif
end:
	return ret;
}

static int msdc_prepare_hs400_tuning(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msdc_host *host = mmc_priv(mmc);
	host->hs400_mode = true;

	if (host->top_base)
		writel(host->hs400_ds_delay,
		       host->top_base + EMMC50_PAD_DS_TUNE);
	else
		writel(host->hs400_ds_delay, host->base + PAD_DS_TUNE);
	/* hs400 mode must set it to 0 */
	sdr_clr_bits(host->base + MSDC_PATCH_BIT2, MSDC_PATCH_BIT2_CFGCRCSTS);
	/* to improve read performance, set outstanding to 2 */
	sdr_set_field(host->base + EMMC50_CFG3, EMMC50_CFG3_OUTS_WR, 2);

	return 0;
}

static int msdc_execute_hs400_tuning(struct mmc_host *mmc, struct mmc_card *card)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct msdc_delay_phase dly1_delay;
	u32 val, result_dly1 = 0;
	u8 *ext_csd;
	int i, ret;

	if (host->top_base) {
		sdr_set_bits(host->top_base + EMMC50_PAD_DS_TUNE,
			     PAD_DS_DLY_SEL);
		if (host->hs400_ds_dly3)
			sdr_set_field(host->top_base + EMMC50_PAD_DS_TUNE,
				      PAD_DS_DLY3, host->hs400_ds_dly3);
	} else {
		sdr_set_bits(host->base + PAD_DS_TUNE, PAD_DS_TUNE_DLY_SEL);
		if (host->hs400_ds_dly3)
			sdr_set_field(host->base + PAD_DS_TUNE,
				      PAD_DS_TUNE_DLY3, host->hs400_ds_dly3);
	}

	host->hs400_tuning = true;
	for (i = 0; i < PAD_DELAY_MAX; i++) {
		if (host->top_base)
			sdr_set_field(host->top_base + EMMC50_PAD_DS_TUNE,
				      PAD_DS_DLY1, i);
		else
			sdr_set_field(host->base + PAD_DS_TUNE,
				      PAD_DS_TUNE_DLY1, i);
		ret = mmc_get_ext_csd(card, &ext_csd);
		if (!ret) {
			result_dly1 |= (1 << i);
			kfree(ext_csd);
		}
	}
	host->hs400_tuning = false;

	dly1_delay = get_best_delay(host, result_dly1);
	if (dly1_delay.maxlen == 0) {
		dev_info(host->dev, "Failed to get DLY1 delay!\n");
		goto fail;
	}
	if (host->top_base)
		sdr_set_field(host->top_base + EMMC50_PAD_DS_TUNE,
			      PAD_DS_DLY1, dly1_delay.final_phase);
	else
		sdr_set_field(host->base + PAD_DS_TUNE,
			      PAD_DS_TUNE_DLY1, dly1_delay.final_phase);

	if (host->top_base) {
		val = readl(host->top_base + EMMC50_PAD_DS_TUNE);
		dev_info(host->dev, "Fianl EMMC50_PAD_DS_TUNE: 0x%x\n", val);
	} else {
		val = readl(host->base + PAD_DS_TUNE);
		dev_info(host->dev, "Fianl PAD_DS_TUNE: 0x%x\n", val);
	}

	return 0;

fail:
	dev_info(host->dev, "Failed to tuning DS pin delay!\n");
	return -EIO;
}

static void msdc_hw_reset(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	sdr_set_bits(host->base + EMMC_IOCON, 1);
	udelay(10); /* 10us is enough */
	sdr_clr_bits(host->base + EMMC_IOCON, 1);
}

static void msdc_ack_sdio_irq(struct mmc_host *mmc)
{
	__msdc_enable_sdio_irq(mmc, 1);
}

static void msdc_cqe_enable(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);

	/* enable cmdq irq */
	writel(MSDC_INT_CMDQ, host->base + MSDC_INTEN);
	/* enable busy check */
	sdr_set_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);
	/* default write data / busy timeout 20s */
	msdc_set_busy_timeout(host, 20 * 1000000000ULL, 0);
	/* default read data timeout 100ms */
	msdc_set_timeout(host, 100 * 1000000, 0);
}

void msdc_cqe_disable(struct mmc_host *mmc, bool recovery)
{
	struct msdc_host *host = mmc_priv(mmc);
	u32 val;

	val = readl(host->base + MSDC_INT);
	writel(val, host->base + MSDC_INT);

	/* disable cmdq irq */
	sdr_clr_bits(host->base + MSDC_INTEN, MSDC_INT_CMDQ);
	/* disable busy check */
	sdr_clr_bits(host->base + MSDC_PATCH_BIT1, MSDC_PB1_BUSY_CHECK_SEL);

	if (recovery) {
		sdr_set_field(host->base + MSDC_DMA_CTRL,
			MSDC_DMA_CTRL_STOP, 1);
		while (readl(host->base + MSDC_DMA_CTRL) & MSDC_DMA_CTRL_STOP)
			cpu_relax();
		msdc_reset_hw(host);
	}
}

static const struct mmc_host_ops mt_msdc_ops = {
	.post_req = msdc_post_req,
	.pre_req = msdc_pre_req,
	.request = msdc_ops_request,
	.set_ios = msdc_ops_set_ios,
	.get_ro = mmc_gpio_get_ro,
	.get_cd = msdc_get_cd,
	.enable_sdio_irq = msdc_enable_sdio_irq,
	.ack_sdio_irq = msdc_ack_sdio_irq,
	.start_signal_voltage_switch = msdc_ops_switch_volt,
	.card_busy = msdc_card_busy,
	.execute_tuning = msdc_execute_tuning,
	.prepare_hs400_tuning = msdc_prepare_hs400_tuning,
	.execute_hs400_tuning = msdc_execute_hs400_tuning,
	.hw_reset = msdc_hw_reset,
	.card_event = msdc_ops_card_event,
	.remove_bad_sdcard = msdc_ops_set_bad_card_and_remove,
	.init_card = msdc_ops_init_card,
};

#define MSDC_DEBUG_REGISTER_COUNT               0x63
void msdc_dump_info(struct mmc_host *mmc)
{
	struct msdc_host *host = mmc_priv(mmc);
	struct cqhci_host *cq_host = mmc->cqe_private;
	int i;
	dev_info(mmc->parent,"%s\n",__func__);

	/* dump clock */
	if (host->crypto_clk_base)
		dev_info(host->dev, "crypto_clk=%08x, bit29 should 2b0",
			readl(host->crypto_clk_base));

	/* dump nomral regs */
	for (i = 0; i <= 64; i++){
		dev_info(host->dev, "[%08X]=%08X", i*4,
			readl(host->base + i*4));
		udelay(1);
	}
	dev_info(host->dev, "[00000228]=%08x",
			readl(host->base + 0x228));

	/* dump debug register */
	pr_notice("msdc debug register a0:a4dump [set:out]\n");
	for (i = 0; i < MSDC_DEBUG_REGISTER_COUNT + 1; i++) {
		writel(i, host->base + 0xA0);
		pr_notice("[%.3hx:%.8x]", i, readl(host->base + 0xA4));
	}

	writel(0x27, host->base + 0xA0);

	pr_notice("msdc debug register dump 0x224 [set:out]\n");
	for (i = 0; i < 12; i++) {
		writel(i, host->base + 0x224);
		pr_notice("[%.3hx:%.8x]", i, readl(host->base + 0xA4));
	}
}
EXPORT_SYMBOL_GPL(msdc_dump_info);

static const struct cqhci_host_ops msdc_cmdq_ops = {
	.enable         = msdc_cqe_enable,
	.disable        = msdc_cqe_disable,
	.dumpregs       = msdc_dump_info,
};

static void msdc_of_property_parse(struct platform_device *pdev,
				   struct msdc_host *host)
{
	unsigned int msdc_crypto_clk_base;

	of_property_read_u32(pdev->dev.of_node, "mediatek,latch-ck",
			     &host->latch_ck);

	of_property_read_u32(pdev->dev.of_node, "hs400-ds-delay",
			     &host->hs400_ds_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs400-ds-dly3",
				&host->hs400_ds_dly3);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs200-cmd-int-delay",
			     &host->hs200_cmd_int_delay);

	of_property_read_u32(pdev->dev.of_node, "mediatek,hs400-cmd-int-delay",
			     &host->hs400_cmd_int_delay);

	if (of_property_read_bool(pdev->dev.of_node,
				  "mediatek,hs400-cmd-resp-sel-rising"))
		host->hs400_cmd_resp_sel_rising = true;
	else
		host->hs400_cmd_resp_sel_rising = false;

	if (of_property_read_bool(pdev->dev.of_node,
				  "mediatek,cqhci"))
		host->cqhci = true;
	else
		host->cqhci = false;

	if (!of_property_read_u32(pdev->dev.of_node, "msdc_crypto_clk_base",
			     &msdc_crypto_clk_base))
		host->crypto_clk_base = ioremap(msdc_crypto_clk_base, 4);
#ifdef CONFIG_MACH_MT8173
	if (!of_property_read_u32(pdev->dev.of_node, "cmd_resp_sel",
				  &host->hs200_cmd_resp_sel))
		dev_dbg(&pdev->dev, "host->hs200_cmd_resp_sel: %x\n",
				host->hs200_cmd_resp_sel);
#endif
}

static int check_boot_type(struct platform_device *pdev)
{
	struct tag_bootmode *tags = NULL;
	struct device_node *node = NULL;
	unsigned long size = 0;
	int ret = BOOTDEV_UFS;

	node = of_find_node_by_path("/chosen");
	if (!node)
		node = of_find_node_by_path("/chosen@0");

	if (node) {
		tags = (struct tag_bootmode *)of_get_property(node,
				"atag,boot", (int *)&size);
	} else
		dev_info(&pdev->dev, "of_chosen not found\n");

	if (tags) {
		ret = tags->boottype;
		if ((ret > 2) || (ret < 0))
			ret = BOOTDEV_SDMMC;
	} else {
		dev_info(&pdev->dev, "atag,boot is not found\n");
	}

	return ret;
}

static int msdc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;
	struct resource *res;
	const char *dup_name;
#ifdef CONFIG_MMC_CQHCI
	struct arm_smccc_res smccc_res;
#endif
	int ret;
	int boot_type = 0;
	int host_index = -1;

#ifdef CONFIG_MACH_MT8173
	if (msdc_top_lock_inited == false) {
		spin_lock_init(&msdc_top_lock);
		msdc_top_lock_inited = true;
	}
#endif
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	/* Add check_boot_type check and return ENODEV if not eMMC boot */
	if (device_property_read_u32(&pdev->dev, "host-index", &host_index) < 0) {
		dev_info(&pdev->dev, "index property is missing \n");
		host_index = -1;
	}
	boot_type = check_boot_type(pdev);
	if ((boot_type != BOOTDEV_SDMMC) && (host_index == 0)) {
		dev_info(&pdev->dev, "no eMMC boot\n");
		return -ENODEV;
	}

	/* Allocate MMC host for this device */
	mmc = mmc_alloc_host(sizeof(struct msdc_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	ret = mmc_of_parse(mmc);

	msdc_debug_proc_init();

	/* fix uaf(use afer free) issue:backup pdev->name,
	 * device_rename will free pdev->name
	 */
	pdev->name = kstrdup(pdev->name, GFP_KERNEL);
	/* device rename */
	if (boot_type == BOOTDEV_SDMMC){
		if ((mmc->index == 0) && !device_rename(mmc->parent, "bootdevice"))
			dev_info(&pdev->dev, "[msdc%d] rename to bootdevice.\n", mmc->index);
		else if ((mmc->index == 1) && !device_rename(mmc->parent, "externdevice"))
			dev_info(&pdev->dev, "[msdc%d] rename to externdevice.\n",mmc->index);
		else if ((mmc->index == 0) || (mmc->index == 1))
			dev_info(&pdev->dev, "[msdc%d] error: rename faile.\n", mmc->index);
	} else if (boot_type == BOOTDEV_UFS){
		if ((mmc->index == 0) && !device_rename(mmc->parent, "externdevice"))
			dev_info(&pdev->dev, "[msdc%d] rename to externdevice.\n",mmc->index);
		else
			dev_info(&pdev->dev, "[msdc%d] error: rename faile.\n", mmc->index);
	}

	dup_name = pdev->name;
	pdev->name = pdev->dev.kobj.name;
	kfree_const(dup_name);

	if (ret)
		goto host_free;

	host->mmc = mmc;
#ifdef CONFIG_MMC_CRYPTO
	if (host->mmc->caps2 & MMC_CAP2_NO_SD)
		host->mmc->caps2 |= MMC_CAP2_CRYPTO;
#endif

	if (device_property_read_u32(&pdev->dev, "host-function", &mmc->host_function) <0){
		dev_info(&pdev->dev, "host_function isn't found in device tree\n");
		mmc->host_function = -1;
	}

	if (mmc->host_function == MSDC_SD)
		mmc->caps |= MMC_CAP_AGGRESSIVE_PM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto host_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	host->top_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->top_base))
		host->top_base = NULL;
#ifdef CONFIG_MACH_MT8173
	host->top_base = of_iomap(pdev->dev.of_node, 1);
	if (WARN_ON(!host->top_base)) {
		ret = -EINVAL;
		goto host_free;
	}

	if (of_property_read_u32(pdev->dev.of_node, "host_id",
	    &host->host_id)) {
		dev_info(&pdev->dev, "Please add host_id at DTS!\n");
		ret = -EINVAL;
		goto host_free;
	}
#endif

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto host_free;

	host->src_clk = devm_clk_get(&pdev->dev, "source");
	if (IS_ERR(host->src_clk)) {
		ret = PTR_ERR(host->src_clk);
		goto host_free;
	}

	host->h_clk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(host->h_clk)) {
		ret = PTR_ERR(host->h_clk);
		goto host_free;
	}

	host->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(host->bus_clk))
		host->bus_clk = NULL;
	/*source clock control gate is optional clock*/
	host->src_clk_cg = devm_clk_get(&pdev->dev, "source_cg");
	if (IS_ERR(host->src_clk_cg))
		host->src_clk_cg = NULL;

	if (host->mmc->caps2 & MMC_CAP2_CRYPTO) {
		host->crypto_clk = devm_clk_get(&pdev->dev, "crypto_clk");
		if (IS_ERR(host->crypto_clk))
			host->crypto_clk = NULL;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = -EINVAL;
		goto host_free;
	}

	host->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(host->pinctrl)) {
		ret = PTR_ERR(host->pinctrl);
		dev_err(&pdev->dev, "Cannot find pinctrl!\n");
		goto host_free;
	}

	host->pins_default = pinctrl_lookup_state(host->pinctrl, "default");
	if (IS_ERR(host->pins_default)) {
		ret = PTR_ERR(host->pins_default);
		dev_err(&pdev->dev, "Cannot find pinctrl default!\n");
		goto host_free;
	}

	host->pins_uhs = pinctrl_lookup_state(host->pinctrl, "state_uhs");
	if (IS_ERR(host->pins_uhs)) {
		ret = PTR_ERR(host->pins_uhs);
		dev_err(&pdev->dev, "Cannot find pinctrl uhs!\n");
		goto host_free;
	}

	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		host->pins_uhs_samsung = pinctrl_lookup_state(host->pinctrl, "state_uhs_samsung");
		if (IS_ERR(host->pins_uhs_samsung)) {
			ret = PTR_ERR(host->pins_uhs_samsung);
			dev_err(&pdev->dev, "Cannot find pinctrl uhs samsung!\n");
		}
	}

	if (mmc->caps & MMC_CAP_NONREMOVABLE) {
		host->pins_uhs_ymtc = pinctrl_lookup_state(host->pinctrl, "state_uhs_ymtc");
		if (IS_ERR(host->pins_uhs_ymtc)) {
			ret = PTR_ERR(host->pins_uhs_ymtc);
			dev_err(&pdev->dev, "Cannot find pinctrl uhs ymtc!\n");
		}
	}

	msdc_of_property_parse(pdev, host);

	host->dev = &pdev->dev;
	host->dev_comp = of_device_get_match_data(&pdev->dev);
#ifdef CONFIG_MACH_MT8173
	host->mmc = mmc;
#endif
	host->src_clk_freq = clk_get_rate(host->src_clk);
	/* Set host parameters to mmc */
	mmc->ops = &mt_msdc_ops;
	if (host->dev_comp->clk_div_bits == 8)
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 255);
	else
		mmc->f_min = DIV_ROUND_UP(host->src_clk_freq, 4 * 4095);

	if (mmc->caps & MMC_CAP_SDIO_IRQ)
		mmc->caps2 |= MMC_CAP2_SDIO_IRQ_NOTHREAD;

	if (mmc->host_function == MSDC_SD){
		mmc->caps |= MMC_CAP_ERASE;
		host->autocmd = MSDC_AUTOCMD12;
	}else{
		mmc->caps |= MMC_CAP_ERASE | MMC_CAP_CMD23;
		host->autocmd = 0;
	}
	if (host->cqhci)
		mmc->caps2 |= MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD;
	/* MMC core transfer sizes tunable parameters */
	mmc->max_segs = MAX_BD_NUM;
	if (host->dev_comp->support_64g)
		mmc->max_seg_size = BDMA_DESC_BUFLEN_EXT;
	else
		mmc->max_seg_size = BDMA_DESC_BUFLEN;
	mmc->max_blk_size = 2048;
	mmc->max_req_size = 512 * 1024;
	mmc->max_blk_count = mmc->max_req_size / 512;
	if (host->dev_comp->support_64g)
		host->dma_mask = DMA_BIT_MASK(36);
	else
		host->dma_mask = DMA_BIT_MASK(32);
	mmc_dev(mmc)->dma_mask = &host->dma_mask;

	msdc_ungate_clock(mmc);
#ifdef CONFIG_MMC_CQHCI
	if (host->cqhci) {
		host->cq_host = devm_kzalloc(host->mmc->parent,
			       sizeof(*host->cq_host), GFP_KERNEL);
		host->cq_host->caps |= CQHCI_TASK_DESC_SZ_128;
		host->cq_host->quirks |= CQHCI_QUIRK_DIS_BEFORE_NON_CQ_CMD;
		host->cq_host->mmio = host->base + 0x800;
		host->cq_host->ops = &msdc_cmdq_ops;
		cqhci_init(host->cq_host, mmc, true);
		host->cq_host->mmc->max_segs = 128;
		/* cqhci 16bit length */
		/* 0 size, means 65536 so we don't have to -1 here */
		host->cq_host->mmc->max_seg_size = 64 * 1024;
	}
	/*
	 * 1: MSDC_AES_CTL_INIT
	 * 4: cap_id, no-meaning
	 * 1: cfg_id, we choose the second cfg group
	 */
	if (host->mmc->caps2 & MMC_CAP2_CRYPTO)
		arm_smccc_smc(MTK_SIP_KERNEL_HW_FDE_MSDC_CTL,
			1, 4, 1, 0, 0, 0, 0, &smccc_res);
#endif

	host->timeout_clks = 3 * 1048576;
	host->dma.gpd = dma_alloc_coherent(&pdev->dev,
				2 * sizeof(struct mt_gpdma_desc),
				&host->dma.gpd_addr, GFP_KERNEL);
	host->dma.bd = dma_alloc_coherent(&pdev->dev,
				MAX_BD_NUM * sizeof(struct mt_bdma_desc),
				&host->dma.bd_addr, GFP_KERNEL);
	if (!host->dma.gpd || !host->dma.bd) {
		ret = -ENOMEM;
		goto release_mem;
	}
	msdc_init_gpd_bd(host, &host->dma);
	INIT_DELAYED_WORK(&host->req_timeout, msdc_request_timeout);
	spin_lock_init(&host->lock);

	platform_set_drvdata(pdev, mmc);
#ifdef CONFIG_MACH_MT8173
	msdc_ungate_clock(mmc);
#endif
	msdc_init_hw(host);

	ret = devm_request_irq(&pdev->dev, host->irq, msdc_irq,
		IRQF_TRIGGER_NONE | IRQF_ONESHOT, pdev->name, host);
	if (ret)
		goto release;

#if defined(CONFIG_MACH_MT6768) && defined(CONFIG_MTK_PMQOS)
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		mtk_pm_qos_add_request(&host->pm_qos, MTK_PM_QOS_VCORE_OPP, 1);
#endif

	pm_runtime_set_active(host->dev);
	pm_runtime_set_autosuspend_delay(host->dev, MTK_MMC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(host->dev);
	pm_runtime_enable(host->dev);

	if (mmc->host_function == MSDC_EMMC) {
		mmc->ocr_avail_mmc = MMC_VDD_EMMC;
		mmc->ocr_avail = mmc->ocr_avail_mmc;
	} else if (mmc->host_function == MSDC_SD) {
		mmc->ocr_avail_sd = MMC_VDD_SD;
		mmc->ocr_avail = mmc->ocr_avail_sd;
	}

	ret = mmc_add_host(mmc);

	if (ret)
		goto end;


	msdc_debug_set_host(mmc);

	return 0;
end:
	pm_runtime_disable(host->dev);
#if defined(CONFIG_MACH_MT6768) && defined(CONFIG_MTK_PMQOS)
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		mtk_pm_qos_remove_request(&host->pm_qos);
#endif
release:
	platform_set_drvdata(pdev, NULL);
	msdc_deinit_hw(host);
	msdc_gate_clock(mmc);
release_mem:
	if (host->dma.gpd)
		dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	if (host->dma.bd)
		dma_free_coherent(&pdev->dev,
			MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);
host_free:
	mmc_free_host(mmc);

	return ret;
}

static int msdc_drv_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct msdc_host *host;

	mmc = platform_get_drvdata(pdev);
	host = mmc_priv(mmc);

	pm_runtime_get_sync(host->dev);

	platform_set_drvdata(pdev, NULL);
	mmc_remove_host(host->mmc);
	msdc_deinit_hw(host);
	msdc_gate_clock(mmc);

	pm_runtime_disable(host->dev);
	pm_runtime_put_noidle(host->dev);
	dma_free_coherent(&pdev->dev,
			2 * sizeof(struct mt_gpdma_desc),
			host->dma.gpd, host->dma.gpd_addr);
	dma_free_coherent(&pdev->dev, MAX_BD_NUM * sizeof(struct mt_bdma_desc),
			host->dma.bd, host->dma.bd_addr);

#if defined(CONFIG_MACH_MT6768) && defined(CONFIG_MTK_PMQOS)
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		mtk_pm_qos_remove_request(&host->pm_qos);
#endif

	mmc_free_host(host->mmc);

	return 0;
}

#ifdef CONFIG_PM
static void msdc_save_reg(struct msdc_host *host)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;

	host->save_para.msdc_cfg = readl(host->base + MSDC_CFG);
	host->save_para.iocon = readl(host->base + MSDC_IOCON);
	host->save_para.sdc_cfg = readl(host->base + SDC_CFG);
	host->save_para.patch_bit0 = readl(host->base + MSDC_PATCH_BIT);
	host->save_para.patch_bit1 = readl(host->base + MSDC_PATCH_BIT1);
	host->save_para.patch_bit2 = readl(host->base + MSDC_PATCH_BIT2);
	host->save_para.pad_ds_tune = readl(host->base + PAD_DS_TUNE);
	host->save_para.pad_cmd_tune = readl(host->base + PAD_CMD_TUNE);
	host->save_para.emmc50_cfg0 = readl(host->base + EMMC50_CFG0);
	host->save_para.emmc50_cfg3 = readl(host->base + EMMC50_CFG3);
	host->save_para.sdc_fifo_cfg = readl(host->base + SDC_FIFO_CFG);
	if (host->top_base) {
		host->save_para.emmc_top_control =
			readl(host->top_base + EMMC_TOP_CONTROL);
		host->save_para.emmc_top_cmd =
			readl(host->top_base + EMMC_TOP_CMD);
		host->save_para.emmc50_pad_ds_tune =
			readl(host->top_base + EMMC50_PAD_DS_TUNE);
	} else {
		host->save_para.pad_tune = readl(host->base + tune_reg);
	}
}

static void msdc_restore_reg(struct msdc_host *host)
{
	u32 tune_reg = host->dev_comp->pad_tune_reg;
	unsigned long tmo;

	sdr_clr_bits(host->base + MSDC_CFG, MSDC_CFG_CKPDN);
	/*
	 * As src_clk/HCLK use the same bit to gate/ungate,
	 * So if want to only gate src_clk, need gate its parent(mux).
	 */
	if (host->src_clk_cg)
		clk_disable_unprepare(host->src_clk_cg);
	else
		clk_disable_unprepare(clk_get_parent(host->src_clk));
	/*
	 * As modify MSDC_CFG may change the clk mode, so MUST do it
	 * like msdc_set_mclk().
	 */
	writel(host->save_para.msdc_cfg, host->base + MSDC_CFG);
	if (host->src_clk_cg)
		clk_prepare_enable(host->src_clk_cg);
	else
		clk_prepare_enable(clk_get_parent(host->src_clk));

	tmo = jiffies + msecs_to_jiffies(20);

	while (!(readl(host->base + MSDC_CFG) & MSDC_CFG_CKSTB) &&
		time_before(jiffies, tmo))
		cpu_relax();

	writel(host->save_para.iocon, host->base + MSDC_IOCON);
	writel(host->save_para.sdc_cfg, host->base + SDC_CFG);

	writel(host->save_para.patch_bit0, host->base + MSDC_PATCH_BIT);
	writel(host->save_para.patch_bit1, host->base + MSDC_PATCH_BIT1);
	writel(host->save_para.patch_bit2, host->base + MSDC_PATCH_BIT2);
	writel(host->save_para.pad_ds_tune, host->base + PAD_DS_TUNE);
	writel(host->save_para.pad_cmd_tune, host->base + PAD_CMD_TUNE);
	writel(host->save_para.emmc50_cfg0, host->base + EMMC50_CFG0);
	writel(host->save_para.emmc50_cfg3, host->base + EMMC50_CFG3);
	writel(host->save_para.sdc_fifo_cfg, host->base + SDC_FIFO_CFG);
	if (host->top_base) {
		writel(host->save_para.emmc_top_control,
		       host->top_base + EMMC_TOP_CONTROL);
		writel(host->save_para.emmc_top_cmd,
		       host->top_base + EMMC_TOP_CMD);
		writel(host->save_para.emmc50_pad_ds_tune,
		       host->top_base + EMMC50_PAD_DS_TUNE);
	} else {
		writel(host->save_para.pad_tune, host->base + tune_reg);
	}
}

static int msdc_runtime_suspend(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	u32 val;

#ifdef CONFIG_MMC_CQHCI
	if (mmc->caps2 & MMC_CAP2_CQE) {
		cqhci_suspend(mmc);
		val = readl(host->base + MSDC_INT);
		writel(val, host->base + MSDC_INT);
	}
#endif

	msdc_save_reg(host);
	msdc_gate_clock(mmc);
#if defined(CONFIG_MACH_MT6768) && defined(CONFIG_MTK_PMQOS)
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		mtk_pm_qos_update_request(&host->pm_qos, MTK_PM_QOS_VCORE_OPP_DEFAULT_VALUE);
#endif
	return 0;
}

static int msdc_runtime_resume(struct device *dev)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	struct msdc_host *host = mmc_priv(mmc);
	struct arm_smccc_res smccc_res;

#if defined(CONFIG_MACH_MT6768) && defined(CONFIG_MTK_PMQOS)
	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		mtk_pm_qos_update_request(&host->pm_qos, 1);
#endif

	msdc_ungate_clock(mmc);
	msdc_restore_reg(host);

#ifdef CONFIG_MMC_CQHCI
	if (mmc->caps2 & MMC_CAP2_CQE)
		cqhci_resume(mmc);
#endif
	/*
	 * 1: MSDC_AES_CTL_INIT
	 * 4: cap_id, no-meaning
	 * 1: cfg_id, we choose the second cfg group
	 */
	if (host->mmc->caps2 & MMC_CAP2_CRYPTO)
		arm_smccc_smc(MTK_SIP_KERNEL_HW_FDE_MSDC_CTL,
			1, 4, 1, 0, 0, 0, 0, &smccc_res);
	return 0;
}
#endif

static const struct dev_pm_ops msdc_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(msdc_runtime_suspend, msdc_runtime_resume, NULL)
};

static struct platform_driver mt_msdc_driver = {
	.probe = msdc_drv_probe,
	.remove = msdc_drv_remove,
	.driver = {
		.name = "mtk-msdc",
		.of_match_table = msdc_of_ids,
		.pm = &msdc_dev_pm_ops,
	},
};

module_platform_driver(mt_msdc_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver");
