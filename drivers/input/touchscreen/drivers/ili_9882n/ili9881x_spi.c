/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ili9881x.h"

struct touch_bus_info {
	struct spi_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_ts_data *ilits;

#ifdef CONFIG_SPI_MT65XX
const struct mtk_chip_config spi_ctrl_data = {
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.cs_pol = 0,
	.sample_sel = 0,
	.cs_setuptime = 25,
	.cs_holdtime = 12,
	.cs_idletime = 12,
};
#endif

#if SPI_DMA_TRANSFER_SPLIT
#define DMA_TRANSFER_MAX_CHUNK		64   // number of chunks to be transferred.
#define DMA_TRANSFER_MAX_LEN		1024 // length of a chunk.

int ili_spi_write_then_read_split(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1, duplex_len = 0;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	int offset = 0;
	u8 cmd = 0;
	struct spi_message message;
	struct spi_transfer *xfer = NULL;

	if (n_rx > SPI_RX_BUF_SIZE) {
		ILI_ERR("Rx length is greater than spi local buf, abort\n");
		return -ENOMEM;
	}

	xfer = kzalloc(DMA_TRANSFER_MAX_CHUNK * sizeof(struct spi_transfer), GFP_KERNEL);
	if (ERR_ALLOC_MEM(xfer)) {
		ILI_ERR("spi_transfer kzalloc failed! \n");
		return = -ENOMEM;
	}

	spi_message_init(&message);
	memset(ilits->spi_tx, 0x0, SPI_TX_BUF_SIZE);
	memset(ilits->spi_rx, 0x0, SPI_RX_BUF_SIZE);

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		if (n_tx % DMA_TRANSFER_MAX_LEN)
			xferloop = (n_tx / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = n_tx / DMA_TRANSFER_MAX_LEN;

		xferlen = n_tx;
		memcpy(ilits->spi_tx, (u8 *)txbuf, xferlen);

		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = ilits->spi_tx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		if (n_tx > DMA_TRANSFER_MAX_LEN) {
			ILI_ERR("Tx length must be lower than dma length (%d).\n", DMA_TRANSFER_MAX_LEN);
			status = -EINVAL;
			break;
		}

		if (!atomic_read(&ilits->ice_stat))
			offset = 2;

		memcpy(ilits->spi_tx, txbuf, n_tx);
		duplex_len = n_tx + n_rx + offset;

		if (duplex_len % DMA_TRANSFER_MAX_LEN)
			xferloop = (duplex_len / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = duplex_len / DMA_TRANSFER_MAX_LEN;

		xferlen = duplex_len;
		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = ilits->spi_tx;
			xfer[xfercnt].rx_buf = ilits->spi_rx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = duplex_len - (xfercnt + 1) * DMA_TRANSFER_MAX_LEN;
		}

		status = spi_sync(spi, &message);
		if (status == 0) {
			if (ilits->spi_rx[1] != SPI_ACK && !atomic_read(&ilits->ice_stat)) {
				status = DO_SPI_RECOVER;
				ILI_ERR("Do spi recovery: rxbuf[1] = 0x%x, ice = %d\n", ilits->spi_rx[1], atomic_read(&ilits->ice_stat));
				break;
			}

			memcpy((u8 *)rxbuf, ilits->spi_rx + offset + 1, n_rx);
		} else {
			ILI_ERR("spi read fail, status = %d\n", status);
		}
		break;
	default:
		ILI_INFO("Unknown command 0x%x\n", cmd);
		break;
	}
	
	kfree(xfer);
	return status;
}
#else
int ili_spi_write_then_read_direct(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1, duplex_len = 0;
	int offset = 0;
	u8 cmd = 0;
	struct spi_message message;
	struct spi_transfer xfer;

	if (ilits->shutdown == true) {
		VTE("device has been shutdown, return now!");
		goto out;
	}

	if (n_rx > SPI_RX_BUF_SIZE) {
		ILI_ERR("Rx length is greater than spi local buf, abort\n");
		status = -ENOMEM;
		goto out;
	}

	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		xfer.len = n_tx;
		xfer.tx_buf = txbuf;
		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		if (!atomic_read(&ilits->ice_stat))
			offset = 2;

		duplex_len = n_tx + n_rx + offset;
		if ((duplex_len > SPI_TX_BUF_SIZE) ||
			(duplex_len > SPI_RX_BUF_SIZE)) {
			ILI_ERR("duplex_len is over than dma buf, abort\n");
			status = -ENOMEM;
			break;
		}

		/*memset(ilits->spi_tx, 0x0, SPI_TX_BUF_SIZE);
		memset(ilits->spi_rx, 0x0, SPI_RX_BUF_SIZE);*/

		xfer.len = duplex_len;
		memcpy(ilits->spi_tx, txbuf, n_tx);
		xfer.tx_buf = ilits->spi_tx;
		xfer.rx_buf = ilits->spi_rx;

		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		if (status == 0) {
			if (ilits->spi_rx[1] != SPI_ACK && !atomic_read(&ilits->ice_stat)) {
				status = DO_SPI_RECOVER;
				ILI_ERR("Do spi recovery: rxbuf[1] = 0x%x, ice = %d\n", ilits->spi_rx[1], atomic_read(&ilits->ice_stat));
				break;
			}

			memcpy((u8 *)rxbuf, ilits->spi_rx + offset + 1, n_rx);
		} else {
			ILI_ERR("spi read fail, status = %d\n", status);
		}
		break;
	default:
		ILI_INFO("Unknown command 0x%x\n", cmd);
		break;
	}

out:
	return status;
}
#endif

static int ili_spi_mp_pre_cmd(u8 cdc)
{
	u8 pre[5] = {0};

	if (!atomic_read(&ilits->mp_stat) || cdc != P5_X_SET_CDC_INIT ||
		ilits->chip->core_ver >= CORE_VER_1430)
		return 0;

	ILI_DBG("mp test with pre commands\n");

	pre[0] = SPI_WRITE;
	pre[1] = 0x0;/*dummy byte*/
	pre[2] = 0x2;/*Write len byte*/
	pre[3] = P5_X_READ_DATA_CTRL;
	pre[4] = P5_X_GET_CDC_DATA;
	if (ilits->spi_write_then_read(ilits->spi, pre, 5, NULL, 0) < 0) {
		ILI_ERR("Failed to write pre commands\n");
		return -1;
	}

	pre[0] = SPI_WRITE;
	pre[1] = 0x0;/*dummy byte*/
	pre[2] = 0x1;/*Write len byte*/
	pre[3] = P5_X_GET_CDC_DATA;
	if (ilits->spi_write_then_read(ilits->spi, pre, 4, NULL, 0) < 0) {
		ILI_ERR("Failed to write pre commands\n");
		return -1;
	}
	return 0;
}

static int ili_spi_pll_clk_wakeup(void)
{
	int index = 0;
	u8 wdata[32] = {0};
	u8 wakeup[9] = {0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	u32 wlen = sizeof(wakeup);

	wdata[0] = SPI_WRITE;
	wdata[1] = wlen >> 8;
	wdata[2] = wlen & 0xff;
	index = 3;
	

	ipio_memcpy(&wdata[index], wakeup, wlen, wlen);
	wlen += index;

	ILI_INFO("Write dummy to wake up spi pll clk\n");
	if (ilits->spi_write_then_read(ilits->spi, wdata, wlen, NULL, 0) < 0) {
		ILI_INFO("spi slave write error\n");
		return -1;
	}
	usleep_range(1000, 2000);
	return 0;
}

static int ili_spi_wrapper(u8 *txbuf, u32 wlen, u8 *rxbuf, u32 rlen, bool spi_irq, bool i2c_irq)
{
	int ret = 0;
	int mode = 0, index = 0;
	u8 wdata[128] = {0};
	u8 checksum = 0;
	bool ice = atomic_read(&ilits->ice_stat);

	if (wlen > 0) {
		if (!txbuf) {
			ILI_ERR("txbuf is null\n");
			return -ENOMEM;
		}

		/* 3 bytes data consist of length and header */
		if ((wlen + 3) > sizeof(wdata)) {
			ILI_ERR("WARNING! wlen(%d) > wdata(%d), using wdata length to transfer\n", wlen, (int)sizeof(wdata));
			wlen = sizeof(wdata) - 3;
		}
	}

	if (rlen > 0) {
		if (!rxbuf) {
			ILI_ERR("rxbuf is null\n");
			return -ENOMEM;
		}
	}

	if (rlen > 0 && !wlen)
		mode = SPI_READ;
	else
		mode = SPI_WRITE;

	if (ilits->int_pulse)
		ilits->detect_int_stat = ili_ic_check_int_pulse;
	else
		ilits->detect_int_stat = ili_ic_check_int_level;

	if (spi_irq)
		atomic_set(&ilits->cmd_int_check, ENABLE);

	switch (mode) {
	case SPI_WRITE:
#if (PLL_CLK_WAKEUP_TP_RESUME == ENABLE)
		if (ilits->pll_clk_wakeup == true) {
#else
		if ((ilits->pll_clk_wakeup == true) && (ilits->tp_suspend == true)) {
#endif
			ret = ili_spi_pll_clk_wakeup();
			if (ret < 0) {
				ILI_ERR("Wakeup pll clk error\n");
				break;
			}
		}
		if (ice) {
			wdata[0] = SPI_WRITE;
			index = 1;
		} else {
			wdata[0] = SPI_WRITE;
			wdata[1] = wlen >> 8;
			wdata[2] = wlen & 0xff;
			index = 3;
		}

		ipio_memcpy(&wdata[index], txbuf, wlen, wlen);

		wlen += index;

		/*
		* NOTE: If TP driver is doing MP test and commanding 0xF1 to FW, we add a checksum
		* to the last index and plus 1 with size.
		*/
		if (atomic_read(&ilits->mp_stat) && wdata[index] == P5_X_SET_CDC_INIT) {
			checksum = ili_calc_packet_checksum(&wdata[index], wlen - index);
			wdata[wlen] = checksum;
			wlen++;
			wdata[1] = (wlen - index) >> 8;
			wdata[2] = (wlen - index) & 0xff;
			ili_dump_data(wdata, 8, wlen, 0, "mp cdc cmd with checksum");
		}

		ret = ilits->spi_write_then_read(ilits->spi, wdata, wlen, txbuf, 0);

		if (!ice) {
			ILI_INFO("send cmd delay 1ms\n");
			usleep_range(1000, 2000);
		}

		if (ret < 0) {
			ILI_INFO("spi-wrapper write error\n");
			break;
		}

		/* Won't break if it needs to read data following with writing. */
		if (!rlen)
			break;
	case SPI_READ:
		if (!ice && spi_irq) {
			if (ilits->mpstatus == false) {
				ILI_INFO("mdelay\n");
				mdelay(5);
			} else {
				/* Check INT triggered by FW when sending cmds. */
				if (ilits->detect_int_stat(false) < 0) {
					ILI_ERR("ERROR! Check INT timeout\n");
					ret = -ETIME;
					break;
				}
			}
		}

		ret = ili_spi_mp_pre_cmd(wdata[3]);
		if (ret < 0)
			ILI_ERR("spi-wrapper mp pre cmd error\n");

		wdata[0] = SPI_READ;

		ret = ilits->spi_write_then_read(ilits->spi, wdata, 1, rxbuf, rlen);
		if (ret < 0)
			ILI_ERR("spi-wrapper read error\n");
		break;
	default:
		ILI_ERR("Unknown spi mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}

	if (spi_irq)
		atomic_set(&ilits->cmd_int_check, DISABLE);

	return ret;
}

int ili_core_spi_setup(int num)
{
	u32 freq[] = {
		TP_SPI_CLK_1M,
		TP_SPI_CLK_2M,
		TP_SPI_CLK_3M,
		TP_SPI_CLK_4M,
		TP_SPI_CLK_5M,
		TP_SPI_CLK_6M,
		TP_SPI_CLK_7M,
		TP_SPI_CLK_8M,
		TP_SPI_CLK_9M,
		TP_SPI_CLK_10M,
		TP_SPI_CLK_11M,
		TP_SPI_CLK_12M,
		TP_SPI_CLK_13M,
		TP_SPI_CLK_14M,
		TP_SPI_CLK_15M
	};

	if (num > sizeof(freq)) {
		ILI_ERR("Invaild clk freq, set default clk freq\n");
		num = 7;
	}

	ILI_INFO("spi clock = %d\n", ilits->spi->max_speed_hz);

	ilits->spi->mode = SPI_MODE_0;
	ilits->spi->bits_per_word = 8;
	//ilits->spi->max_speed_hz = freq[num];

	if (spi_setup(ilits->spi) < 0) {
		ILI_ERR("Failed to setup spi device\n");
		return -ENODEV;
	}

	ILI_INFO("name = %s, bus_num = %d,cs = %d, mode = %d, speed = %d\n",
			ilits->spi->modalias,
			ilits->spi->master->bus_num,
			ilits->spi->chip_select,
			ilits->spi->mode,
			ilits->spi->max_speed_hz);
	return 0;
}

extern int vts_ilitek_setup (void);
static int ilitek_spi_probe(struct spi_device *spi, struct device_node *np)
{
	struct touch_bus_info *info =
	container_of(to_spi_driver(spi->dev.driver),
		struct touch_bus_info, bus_driver);
	int ret = 0;

	ILI_INFO("ilitek spi probe\n");

	if (!spi) {
		ILI_ERR("spi device is NULL\n");
		return -ENODEV;
	}

	ilits = devm_kzalloc(&spi->dev, sizeof(struct ilitek_ts_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ilits)) {
		ILI_ERR("Failed to allocate ts memory, %ld\n", PTR_ERR(ilits));
		return -ENOMEM;
	}

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		ILI_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_duplex;
	}

	ilits->update_buf = kzalloc(MAX_HEX_FILE_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->update_buf)) {
		ILI_ERR("fw kzalloc error\n");
		ret = -ENOMEM;
		goto err_kzalloc_update_buf;
	}

	/* Used for receiving touch data only, do not mix up with others. */
	ilits->tr_buf = kzalloc(TR_BUF_SIZE, GFP_ATOMIC);
	if (ERR_ALLOC_MEM(ilits->tr_buf)) {
		ILI_ERR("failed to allocate touch report buffer\n");
		ret = -ENOMEM;
		goto err_kvzalloc_tr_buf;
	}

	ilits->mutual_delta_buf = kzalloc(TR_BUF_SIZE, GFP_ATOMIC);
	if (ERR_ALLOC_MEM(ilits->mutual_delta_buf)) {
		ILI_ERR("Failed to allocate mutual delta buffer\n");
		ret = -ENOMEM;
		goto err_kvzalloc_mutual_delta_buf;
	}

	ilits->p_rawdata = kcalloc(2048, sizeof(u8), GFP_ATOMIC);
	if (ERR_ALLOC_MEM(ilits->p_rawdata)) {
		ILI_ERR("Failed to allocate packet memory\n");
		ret = -ENOMEM;
		goto err_kcalloc_p_rawdata;
	}

	ilits->spi_tx = kzalloc(SPI_TX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->spi_tx)) {
		ILI_ERR("Failed to allocate spi tx buffer\n");
		ret = -ENOMEM;
		goto err_kzalloc_spi_tx;
	}

	ilits->spi_rx = kzalloc(SPI_RX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->spi_rx)) {
		ILI_ERR("Failed to allocate spi rx buffer\n");
		ret = -ENOMEM;
		goto err_kzalloc_spi_rx;
	}

	ilits->gcoord = kzalloc(sizeof(struct gesture_coordinate), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ilits->gcoord)) {
		ILI_ERR("Failed to allocate gresture coordinate buffer\n");
		ret = -ENOMEM;
		goto err_kvzalloc_gcoord;
	}

	ilits->i2c = NULL;
	ilits->spi = spi;
	ilits->dev = &spi->dev;
	ilits->hwif = info->hwif;
	ilits->phys = "SPI";
	ilits->wrapper = ili_spi_wrapper;
	ilits->detect_int_stat = ili_ic_check_int_pulse;
	ilits->int_pulse = true;
	ilits->mp_retry = false;
	ilits->node = np;
	ilits->dev->of_node = np;

#if SPI_DMA_TRANSFER_SPLIT
	ilits->spi_write_then_read = ili_spi_write_then_read_split;
#else
	ilits->spi_write_then_read = ili_spi_write_then_read_direct;
#endif

	ilits->actual_tp_mode = P5_X_FW_AP_MODE;
	ilits->tp_data_format = DATA_FORMAT_DEMO;
	ilits->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;

	if (TDDI_RST_BIND)
		ilits->reset = TP_IC_WHOLE_RST;
	else
		ilits->reset = TP_HW_RST_ONLY;

	ilits->rst_edge_delay = 10;
	ilits->fw_open = REQUEST_FIRMWARE;
	ilits->fw_upgrade_mode = UPGRADE_IRAM;
	ilits->mp_move_code = ili_move_mp_code_iram;
	ilits->gesture_move_code = ili_move_gesture_code_iram;
	ilits->esd_recover = ili_wq_esd_spi_check;
	ilits->ges_recover = ili_touch_esd_gesture_iram;
	ilits->gesture_mode = DATA_FORMAT_GESTURE_INFO;
	ilits->gesture_demo_ctrl = DISABLE;
	ilits->wtd_ctrl = OFF;
	ilits->report = ENABLE;
	ilits->netlink = DISABLE;
	ilits->dnp = DISABLE;
	ilits->irq_tirgger_type = IRQF_TRIGGER_FALLING;
	ilits->info_from_hex = ENABLE;
	ilits->wait_int_timeout = AP_INT_TIMEOUT;

#if ENABLE_GESTURE
	ilits->gesture = DISABLE;
	ilits->ges_sym.double_tap = DOUBLE_TAP;
	ilits->ges_sym.alphabet_line_2_top = ALPHABET_LINE_2_TOP;
	ilits->ges_sym.alphabet_line_2_bottom = ALPHABET_LINE_2_BOTTOM;
	ilits->ges_sym.alphabet_line_2_left = ALPHABET_LINE_2_LEFT;
	ilits->ges_sym.alphabet_line_2_right = ALPHABET_LINE_2_RIGHT;
	ilits->ges_sym.alphabet_m = ALPHABET_M;
	ilits->ges_sym.alphabet_w = ALPHABET_W;
	ilits->ges_sym.alphabet_c = ALPHABET_C;
	ilits->ges_sym.alphabet_E = ALPHABET_E;
	ilits->ges_sym.alphabet_V = ALPHABET_V;
	ilits->ges_sym.alphabet_O = ALPHABET_O;
	ilits->ges_sym.alphabet_S = ALPHABET_S;
	ilits->ges_sym.alphabet_Z = ALPHABET_Z;
	ilits->ges_sym.alphabet_V_down = ALPHABET_V_DOWN;
	ilits->ges_sym.alphabet_V_left = ALPHABET_V_LEFT;
	ilits->ges_sym.alphabet_V_right = ALPHABET_V_RIGHT;
	ilits->ges_sym.alphabet_two_line_2_bottom = ALPHABET_TWO_LINE_2_BOTTOM;
	ilits->ges_sym.alphabet_F = ALPHABET_F;
	ilits->ges_sym.alphabet_AT = ALPHABET_AT;
#endif
	spi_set_drvdata(spi, ilits);

#ifdef CONFIG_SPI_MT65XX
	memcpy(&ilits->spi_ctrl, &spi_ctrl_data, sizeof(struct mtk_chip_config));
	ilits->spi->controller_data = (void *)&ilits->spi_ctrl;
#endif

	/* get spi property */
	ret = of_property_read_u32(np, "spi-max-frequency",
			&ilits->spi->max_speed_hz);
	if (ret) {
		VTE("set default spi-max-frequency 8M");
		ilits->spi->max_speed_hz = 8*1000*1000;
	}

	ilits->reset_poweroff = of_property_read_bool(np, "ilits,vts-reset-poweroff");
	if (ilits->reset_poweroff) {
		VTI("reset is poweroff in sleep mode");
	} else {
		VTI("reset no poweroff in sleep mode");
	}

	/* get 240HZ support-highrate */
	ret = of_property_read_u32(np, "ilits,support-highrate",&ilits->support_highrate);
	VTI("support_high:%d",ilits->support_highrate);
	if(ret < 0){
		VTE("support-highrate get failed");
	}

	ilits->vddi_poweroff = of_property_read_bool(np, "mtk,vts-vddi-poweroff");
	if (ilits->vddi_poweroff) {
		VTI("cs is poweroff in sleep mode");
	} else {
		VTI("cs no poweroff in sleep mode");
	}

	if (ili_core_spi_setup(SPI_CLK) < 0)
		return -EINVAL;

	ret = vts_ilitek_setup();
	if (ret) {
		ILI_INFO("fail to set up vts !");
		goto err_ilitek_setup;
	}
	ILI_INFO("ili probe end !");
	return ret;

err_ilitek_setup:
	kfree(ilits->gcoord);
err_kvzalloc_gcoord:
	kfree(ilits->spi_rx);
err_kzalloc_spi_rx:
	kfree(ilits->spi_tx);
err_kzalloc_spi_tx:
	kfree(ilits->p_rawdata);
err_kcalloc_p_rawdata:
	kfree(ilits->mutual_delta_buf);
err_kvzalloc_mutual_delta_buf:
	kfree(ilits->tr_buf);
err_kvzalloc_tr_buf:
	kfree(ilits->update_buf);
err_kzalloc_update_buf:
err_duplex:
	devm_kfree(&spi->dev,ilits);

	return ret;
}

/*static struct spi_device_id tp_spi_id[] = {
	{TDDI_DEV_ID, 0},
	{},
};*/

static int ilitek_spi_remove(struct spi_device *spi, struct device_node *np)
{
	struct touch_bus_info *info = (struct touch_bus_info *)ilits->hwif->info;
	ILI_INFO();
	ILI_INFO("remove spi dev\n");
	kfree(ilits->update_buf);
	kfree(ilits->spi_tx);
	kfree(ilits->spi_rx);
	ipio_kfree((void **)&info);
	return 0;
}

static void ilitek_spi_shutdown(struct spi_device *spi)
{
	ILI_INFO("ilitek_spi_shutdown dev\n");
	ilits->shutdown = true;
	if (gpio_is_valid(ilits->tp_rst)) 
		gpio_set_value(ilits->tp_rst, 0);
	else 
		ILI_INFO(" spi tp_rst not valid\n");
#if defined(QCOM_FOR_VIVO_TS)
	if (gpio_is_valid(ilits->tp_cs)) 
		gpio_set_value(ilits->tp_cs, 0);
	else 
		ILI_INFO(" spi tp_cs not valid\n");
#endif
#if defined(MTK_FOR_VIVO_TS)
	if (ilits->vddi_poweroff) {
		if (!IS_ERR_OR_NULL(ilits->spi_cs_sleep_pulllow)) {
			pinctrl_select_state(ilits->pinctrl, ilits->spi_cs_sleep_pulllow);
			VTI("spi_cs_sleep_pulllow");
		}
	}
#endif
	msleep(5);
	vts_dsi_panel_reset_power_ctrl(8);
}

struct vts_spi_driver ili9882n_spi_driver = {
	.probe		= ilitek_spi_probe,
	.remove		= ilitek_spi_remove,
	.shutdown	= ilitek_spi_shutdown,
	.compatible = "tchip,ilitek",
};

int ili_interface_dev_init(struct ilitek_hwif_info *hwif)
{
	/*struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ILI_ERR("faied to allocate spi_driver\n");
		return -ENOMEM;
	}

	if (hwif->bus_type != BUS_SPI) {
		ILI_ERR("Not SPI dev\n");
		ipio_kfree((void **)&info);
		return -EINVAL;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;
	info->bus_driver.driver.pm = hwif->pm;

	info->bus_driver.probe = ilitek_spi_probe;
	info->bus_driver.remove = ilitek_spi_remove;
	//info->bus_driver.id_table = tp_spi_id;

	info->hwif = hwif;
	vts_spi_drv_reigster(info->bus_driver);*/
	return 0;
}

void ili_interface_dev_exit(struct ilitek_ts_data *ts)
{
	struct touch_bus_info *info = (struct touch_bus_info *)ilits->hwif->info;

	ILI_INFO("remove spi dev\n");
	kfree(ilits->update_buf);
	kfree(ilits->spi_tx);
	kfree(ilits->spi_rx);
	vts_spi_drv_reigster(&ili9882n_spi_driver);
	kfree(info);
}

static const int ic_numbers[] = {VTS_IC_ILI_9882N};
module_vts_driver(ili_9882n, ic_numbers, vts_spi_drv_reigster(&ili9882n_spi_driver), vts_spi_drv_reigster(&ili9882n_spi_driver));

