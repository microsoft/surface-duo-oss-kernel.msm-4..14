/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include "rsi_sdio.h"
#include "rsi_common.h"
#include "rsi_hal.h"
#include "rsi_hci.h"

/* Default operating mode is Wi-Fi alone */
#ifdef CONFIG_CARACALLA_BOARD
#if defined (CONFIG_VEN_RSI_COEX) || defined(CONFIG_VEN_RSI_BT_ALONE)
u16 dev_oper_mode = DEV_OPMODE_STA_BT_DUAL;
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
module_param(dev_oper_mode, ushort, S_IRUGO);
MODULE_PARM_DESC(dev_oper_mode,
		 "1 -	Wi-Fi Alone \
		  4 -	BT Alone \
		  8 -	BT LE Alone \
		  5 -	Wi-Fi STA + BT classic \
		  9 -	Wi-Fi STA + BT LE \
		  13 -	Wi-Fi STA + BT classic + BT LE \
		  6 -	AP + BT classic \
		  14 -	AP + BT classic + BT LE");


/**
 * rsi_sdio_set_cmd52_arg() - This function prepares cmd 52 read/write arg.
 * @rw: Read/write
 * @func: function number
 * @raw: indicates whether to perform read after write
 * @address: address to which to read/write
 * @writedata: data to write
 *
 * Return: argument
 */
static u32 rsi_sdio_set_cmd52_arg(bool rw,
				  u8 func,
				  u8 raw,
				  u32 address,
				  u8 writedata)
{
	return ((rw & 1) << 31) | ((func & 0x7) << 28) |
		((raw & 1) << 27) | (1 << 26) |
		((address & 0x1FFFF) << 9) | (1 << 8) |
		(writedata & 0xFF);
}

/**
 * rsi_cmd52writebyte() - This function issues cmd52 byte write onto the card.
 * @card: Pointer to the mmc_card.
 * @address: Address to write.
 * @byte: Data to write.
 *
 * Return: Write status.
 */
static int rsi_cmd52writebyte(struct mmc_card *card,
			      u32 address,
			      u8 byte)
{
	struct mmc_command io_cmd;
	u32 arg;

	memset(&io_cmd, 0, sizeof(io_cmd));
	arg = rsi_sdio_set_cmd52_arg(1, 0, 0, address, byte);
	io_cmd.opcode = SD_IO_RW_DIRECT;
	io_cmd.arg = arg;
	io_cmd.flags = /*MMC_RSP_R5 | */MMC_CMD_AC;

	return mmc_wait_for_cmd(card->host, &io_cmd, 0);
}

/**
 * rsi_cmd52readbyte() - This function issues cmd52 byte read onto the card.
 * @card: Pointer to the mmc_card.
 * @address: Address to read from.
 * @byte: Variable to store read value.
 *
 * Return: Read status.
 */
static int rsi_cmd52readbyte(struct mmc_card *card,
			     u32 address,
			     u8 *byte)
{
	struct mmc_command io_cmd;
	u32 arg;
	int err;

	memset(&io_cmd, 0, sizeof(io_cmd));
	arg = rsi_sdio_set_cmd52_arg(0, 0, 0, address, 0);
	io_cmd.opcode = SD_IO_RW_DIRECT;
	io_cmd.arg = arg;
	io_cmd.flags = /*MMC_RSP_R5 | */MMC_CMD_AC;

	err = mmc_wait_for_cmd(card->host, &io_cmd, 0);
	if ((!err) && (byte))
		*byte =  io_cmd.resp[0] & 0xFF;
	return err;
}

/**
 * rsi_issue_sdiocommand() - This function issues sdio commands.
 * @func: Pointer to the sdio_func structure.
 * @opcode: Opcode value.
 * @arg: Arguments to pass.
 * @flags: Flags which are set.
 * @resp: Pointer to store response.
 *
 * Return: err: command status as 0 or -1.
 */
static int rsi_issue_sdiocommand(struct sdio_func *func,
				 u32 opcode,
				 u32 arg,
				 u32 flags,
				 u32 *resp)
{
	struct mmc_command cmd;
	struct mmc_host *host;
	int err;

	host = func->card->host;

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = flags;
	err = mmc_wait_for_cmd(host, &cmd, 3);

	if ((!err) && (resp))
		*resp = cmd.resp[0];

	return err;
}

static void rsi_dummy_isr(struct sdio_func *function)
{
	return;
}

/**
 * rsi_handle_interrupt() - This function is called upon the occurrence
 *			    of an interrupt.
 * @function: Pointer to the sdio_func structure.
 *
 * Return: None.
 */
static void rsi_handle_interrupt(struct sdio_func *function)
{
	struct rsi_hw *adapter = sdio_get_drvdata(function);
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;

	dev->sdio_irq_task = current;
	rsi_interrupt_handler(adapter);
	dev->sdio_irq_task = NULL;
}

void rsi_gspi_init(struct rsi_hw *adapter)
{
	unsigned long gspi_ctrl_reg0_val;
	
	/* Programming gspi frequency = soc_frequency / 2 */
	/* Warning : ULP seemed to be not working
	 * well at high frequencies. Modify accordingly */
	gspi_ctrl_reg0_val = 0x4;
	/* csb_setup_time [5:4] */
	gspi_ctrl_reg0_val |= 0x10; 
	/* csb_hold_time [7:6] */
	gspi_ctrl_reg0_val |= 0x40; 
	/* csb_high_time [9:8] */
	gspi_ctrl_reg0_val |= 0x100; 
	/* spi_mode [10] */
	gspi_ctrl_reg0_val |= 0x000; 
	/* clock_phase [11] */
	gspi_ctrl_reg0_val |= 0x000; 
	/* Initializing GSPI for ULP read/writes */
	rsi_sdio_master_reg_write(adapter,
				  GSPI_CTRL_REG0,
				  gspi_ctrl_reg0_val,
				  2);
}

void ulp_read_write(struct rsi_hw *adapter, u16 addr, u16 *data, u16 len_in_bits)
{
	rsi_sdio_master_reg_write(adapter,
				  GSPI_DATA_REG1,
				  ((addr << 6) | (data[1] & 0x3f)),
				  2);
	rsi_sdio_master_reg_write(adapter,
				  GSPI_DATA_REG0,
				  *(u16 *)&data[0],
				  2);
	rsi_gspi_init(adapter);
	rsi_sdio_master_reg_write(adapter,
				  GSPI_CTRL_REG1,
				  ((len_in_bits - 1) | GSPI_TRIG),
				  2);
	msleep(10);
}

static void rsi_reset_chip(struct rsi_hw *adapter)
{
	u16 temp[4] = {0};
	u32 data;
	u8 sdio_interrupt_status = 0;
	u8 request = 1;

	ven_rsi_dbg(INFO_ZONE, "Writing disable to wakeup register\n");
	if (rsi_sdio_write_register(adapter,
				0,
				    SDIO_WAKEUP_REG,
				    &request) < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to Write SDIO WAKEUP REG\n", __func__);
		return;
	}
	msleep(3);
	if (rsi_sdio_read_register(adapter,
				   RSI_FN1_INT_REGISTER,
				   &sdio_interrupt_status) < 0) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to Read Intr Status Register\n",
			__func__);
		return;
	}
	ven_rsi_dbg(INFO_ZONE, "%s: Intr Status Register value = %d \n",
		__func__, sdio_interrupt_status);

	/* Put TA on hold */
	if (rsi_sdio_master_access_msword(adapter, 0x2200)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to set ms word to common reg\n",
			__func__);
		return ;
	}

	data = TA_HOLD_THREAD_VALUE;
	if (rsi_sdio_write_register_multiple(adapter,
					TA_HOLD_THREAD_REG | SD_REQUEST_MASTER,
					(u8 *)&data,
					4)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to hold TA threads\n", __func__);
		return ;
	}

	/* This msleep will ensure TA processor to go to hold and any pending dma
	 * transfers to rf spi in device to finish */
	msleep(100);

	*(u32 *)temp = 0;
	ulp_read_write(adapter, ULP_RESET_REG, temp, 32);
	*(u32 *)temp = 2;
	ulp_read_write(adapter, WATCH_DOG_TIMER_1, temp, 32);
	*(u32 *)temp = 0;
	ulp_read_write(adapter, WATCH_DOG_TIMER_2, temp, 32);
	*(u32 *)temp = 50;
	ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_1, temp, 32);
	*(u32 *)temp = 0;
	ulp_read_write(adapter, WATCH_DOG_DELAY_TIMER_2, temp, 32);
	*(u32 *)temp = ((0xaa000) | RESTART_WDT | BYPASS_ULP_ON_WDT);
	ulp_read_write(adapter, WATCH_DOG_TIMER_ENABLE, temp, 32);
	msleep(1000);
}

/**
 * rsi_reset_card() - This function resets and re-initializes the card.
 * @pfunction: Pointer to the sdio_func structure.
 *
 * Return: None.
 */
static void rsi_reset_card(struct sdio_func *pfunction)
{
	int err;
	struct mmc_card *card = pfunction->card;
	struct mmc_host *host = card->host;
	s32 bit = (fls(host->ocr_avail) - 1);
	u8 cmd52_resp = 0;
	u32 clock, resp, i;
	u16 rca;
	u32 cmd_delay = 0;

#ifdef CONFIG_CARACALLA_BOARD
	/* Reset 9110 chip */
	err = rsi_cmd52writebyte(pfunction->card,
				 SDIO_CCCR_ABORT,
				 (1 << 3));

	/* Card will not send any response as it is getting reset immediately
	 * Hence expect a timeout status from host controller
	 */
	if (err != -ETIMEDOUT)
		ven_rsi_dbg(ERR_ZONE, "%s: Reset failed : %d\n", __func__, err);

	cmd_delay = 20;
#else
	cmd_delay = 2;
#endif

	/* Wait for few milli seconds to get rid of residue charges if any */
	msleep(cmd_delay);

	/* Initialize the SDIO card */
	host->ios.vdd = bit;
	host->ios.chip_select = MMC_CS_DONTCARE;
	host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	host->ios.power_mode = MMC_POWER_UP;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	host->ops->set_ios(host, &host->ios);

	/*
	 * This delay should be sufficient to allow the power supply
	 * to reach the minimum voltage.
	 */
	msleep(cmd_delay);

	host->ios.clock = host->f_min;
	host->ios.power_mode = MMC_POWER_ON;
	host->ops->set_ios(host, &host->ios);

	/*
	 * This delay must be at least 74 clock sizes, or 1 ms, or the
	 * time required to reach a stable voltage.
	 */
	msleep(cmd_delay);

	/* Issue CMD0. Goto idle state */
	host->ios.chip_select = MMC_CS_HIGH;
	host->ops->set_ios(host, &host->ios);
	msleep(cmd_delay);
	err = rsi_issue_sdiocommand(pfunction,
				    MMC_GO_IDLE_STATE,
				    0,
				    (MMC_RSP_NONE | MMC_CMD_BC),
				    NULL);
	host->ios.chip_select = MMC_CS_DONTCARE;
	host->ops->set_ios(host, &host->ios);
	msleep(cmd_delay);
	host->use_spi_crc = 0;

	if (err)
		ven_rsi_dbg(ERR_ZONE, "%s: CMD0 failed : %d\n", __func__, err);

#ifdef CONFIG_CARACALLA_BOARD
	if (!host->ocr_avail) {
#else
	if (1) {
#endif
		/* Issue CMD5, arg = 0 */
		err = rsi_issue_sdiocommand(pfunction,
					    SD_IO_SEND_OP_COND,
					    0,
					    (MMC_RSP_R4 | MMC_CMD_BCR),
					    &resp);
		if (err)
			ven_rsi_dbg(ERR_ZONE, "%s: CMD5 failed : %d\n",
				__func__, err);
#ifdef CONFIG_CARACALLA_BOARD
		host->ocr_avail = resp;
#else
		card->ocr = resp;
#endif
	}

	/* Issue CMD5, arg = ocr. Wait till card is ready  */
	for (i = 0; i < 100; i++) {
		err = rsi_issue_sdiocommand(pfunction,
					    SD_IO_SEND_OP_COND,
#ifdef CONFIG_CARACALLA_BOARD
					    host->ocr_avail,
#else
					    card->ocr,
#endif
					    (MMC_RSP_R4 | MMC_CMD_BCR),
					    &resp);
		if (err) {
			ven_rsi_dbg(ERR_ZONE, "%s: CMD5 failed : %d\n",
				__func__, err);
			break;
		}

		if (resp & MMC_CARD_BUSY)
			break;
		msleep(cmd_delay);
	}

	if ((i == 100) || (err)) {
		ven_rsi_dbg(ERR_ZONE, "%s: card in not ready : %d %d\n",
			__func__, i, err);
		return;
	}

	/* Issue CMD3, get RCA */
	err = rsi_issue_sdiocommand(pfunction,
				    SD_SEND_RELATIVE_ADDR,
				    0,
				    (MMC_RSP_R6 | MMC_CMD_BCR),
				    &resp);
	if (err) {
		ven_rsi_dbg(ERR_ZONE, "%s: CMD3 failed : %d\n", __func__, err);
		return;
	}
	rca = resp >> 16;
	host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	host->ops->set_ios(host, &host->ios);

	/* Issue CMD7, select card  */
	err = rsi_issue_sdiocommand(pfunction,
				    MMC_SELECT_CARD,
				    (rca << 16),
				    (MMC_RSP_R1 | MMC_CMD_AC),
				    NULL);
	if (err) {
		ven_rsi_dbg(ERR_ZONE, "%s: CMD7 failed : %d\n", __func__, err);
		return;
	}

	/* Enable high speed */
	if (card->host->caps & MMC_CAP_SD_HIGHSPEED) {
		ven_rsi_dbg(ERR_ZONE, "%s: Set high speed mode\n", __func__);
		err = rsi_cmd52readbyte(card, SDIO_CCCR_SPEED, &cmd52_resp);
		if (err) {
			ven_rsi_dbg(ERR_ZONE, "%s: CCCR speed reg read failed: %d\n",
				__func__, err);
		} else {
			err = rsi_cmd52writebyte(card,
						 SDIO_CCCR_SPEED,
						 (cmd52_resp | SDIO_SPEED_EHS));
			if (err) {
				ven_rsi_dbg(ERR_ZONE,
					"%s: CCR speed regwrite failed %d\n",
					__func__, err);
				return;
			}
			host->ios.timing = MMC_TIMING_SD_HS;
			host->ops->set_ios(host, &host->ios);
		}
	}

	/* Set clock */
	if (mmc_card_hs(card))
		clock = 50000000;
	else
		clock = card->cis.max_dtr;

	if (clock > host->f_max)
		clock = host->f_max;

	host->ios.clock = clock;
	host->ops->set_ios(host, &host->ios);

	if (card->host->caps & MMC_CAP_4_BIT_DATA) {
		/* CMD52: Set bus width & disable card detect resistor */
		err = rsi_cmd52writebyte(card,
					 SDIO_CCCR_IF,
					 (SDIO_BUS_CD_DISABLE |
					  SDIO_BUS_WIDTH_4BIT));
		if (err) {
			ven_rsi_dbg(ERR_ZONE, "%s: Set bus mode failed : %d\n",
				__func__, err);
			return;
		}
		host->ios.bus_width = MMC_BUS_WIDTH_4;
		host->ops->set_ios(host, &host->ios);
	}
	mdelay(cmd_delay);
}

/**
 * rsi_setclock() - This function sets the clock frequency.
 * @adapter: Pointer to the adapter structure.
 * @freq: Clock frequency.
 *
 * Return: None.
 */
static void rsi_setclock(struct rsi_hw *adapter, u32 freq)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	struct mmc_host *host = dev->pfunction->card->host;
	u32 clock;

	clock = freq * 1000;
	if (clock > host->f_max)
		clock = host->f_max;
	host->ios.clock = clock;
	host->ops->set_ios(host, &host->ios);
}

/**
 * rsi_setblocklength() - This function sets the host block length.
 * @adapter: Pointer to the adapter structure.
 * @length: Block length to be set.
 *
 * Return: status: 0 on success, -1 on failure.
 */
static int rsi_setblocklength(struct rsi_hw *adapter, u32 length)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	int status;

	ven_rsi_dbg(INIT_ZONE, "%s: Setting the block length\n", __func__);

	status = sdio_set_block_size(dev->pfunction, length);
	dev->pfunction->max_blksize = 256;

	ven_rsi_dbg(INFO_ZONE,
		"%s: Operational blk length is %d\n", __func__, length);
	return status;
}

/**
 * rsi_setupcard() - This function queries and sets the card's features.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: status: 0 on success, -1 on failure.
 */
static int rsi_setupcard(struct rsi_hw *adapter)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	int status = 0;

	rsi_setclock(adapter, 50000);

	dev->tx_blk_size = 256;
	adapter->tx_blk_size = dev->tx_blk_size;
	status = rsi_setblocklength(adapter, dev->tx_blk_size);
	if (status)
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to set block length\n", __func__);
	return status;
}

/**
 * rsi_sdio_read_register() - This function reads one byte of information
 *			      from a register.
 * @adapter: Pointer to the adapter structure.
 * @addr: Address of the register.
 * @data: Pointer to the data that stores the data read.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_read_register(struct rsi_hw *adapter,
			   u32 addr,
			   u8 *data)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	u8 fun_num = 0;
	int status;

	if (likely(dev->sdio_irq_task != current))
		sdio_claim_host(dev->pfunction);

	if (fun_num == 0)
		*data = sdio_f0_readb(dev->pfunction, addr, &status);
	else
		*data = sdio_readb(dev->pfunction, addr, &status);

	if (likely(dev->sdio_irq_task != current))
		sdio_release_host(dev->pfunction);

	return status;
}

/**
 * rsi_sdio_write_register() - This function writes one byte of information
 *			       into a register.
 * @adapter: Pointer to the adapter structure.
 * @function: Function Number.
 * @addr: Address of the register.
 * @data: Pointer to the data tha has to be written.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_write_register(struct rsi_hw *adapter,
			    u8 function,
			    u32 addr,
			    u8 *data)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	int status = 0;

	if (likely(dev->sdio_irq_task != current))
		sdio_claim_host(dev->pfunction);

	if (function == 0)
		sdio_f0_writeb(dev->pfunction, *data, addr, &status);
	else
		sdio_writeb(dev->pfunction, *data, addr, &status);

	if (likely(dev->sdio_irq_task != current))
		sdio_release_host(dev->pfunction);

	return status;
}

/**
 * rsi_sdio_ack_intr() - This function acks the interrupt received.
 * @adapter: Pointer to the adapter structure.
 * @int_bit: Interrupt bit to write into register.
 *
 * Return: None.
 */
void rsi_sdio_ack_intr(struct rsi_hw *adapter, u8 int_bit)
{
	int status;

	status = rsi_sdio_write_register(adapter,
					 1,
					 (SDIO_FUN1_INTR_CLR_REG |
					  SD_REQUEST_MASTER),
					 &int_bit);
	if (status)
		ven_rsi_dbg(ERR_ZONE, "%s: unable to send ack\n", __func__);
}

/**
 * rsi_sdio_read_register_multiple() - This function read multiple bytes of
 *				       information from the SD card.
 * @adapter: Pointer to the adapter structure.
 * @addr: Address of the register.
 * @count: Number of multiple bytes to be read.
 * @data: Pointer to the read data.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_read_register_multiple(struct rsi_hw *adapter,
				    u32 addr,
				    u8 *data,
				    u16 count)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	u32 status = 0;

	if (likely(dev->sdio_irq_task != current))
		sdio_claim_host(dev->pfunction);

	status =  sdio_readsb(dev->pfunction, data, addr, count);

	if (likely(dev->sdio_irq_task != current))
		sdio_release_host(dev->pfunction);

	if (status != 0)
		ven_rsi_dbg(ERR_ZONE, "%s: Synch Cmd53 read failed\n", __func__);
	return status;
}

/**
 * rsi_sdio_write_register_multiple() - This function writes multiple bytes of
 *					information to the SD card.
 * @adapter: Pointer to the adapter structure.
 * @addr: Address of the register.
 * @data: Pointer to the data that has to be written.
 * @count: Number of multiple bytes to be written.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_write_register_multiple(struct rsi_hw *adapter,
				     u32 addr,
				     u8 *data,
				     u16 count)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	int status;

	if (dev->write_fail > 1) {
		ven_rsi_dbg(ERR_ZONE, "%s: Stopping card writes\n", __func__);
		return 0;
	} else if (dev->write_fail == 1) {
		/**
		 * Assuming it is a CRC failure, we want to allow another
		 *  card write
		 */
		ven_rsi_dbg(ERR_ZONE, "%s: Continue card writes\n", __func__);
		dev->write_fail++;
	}

	if (likely(dev->sdio_irq_task != current))
		sdio_claim_host(dev->pfunction);

	status = sdio_writesb(dev->pfunction, addr, data, count);

	if (likely(dev->sdio_irq_task != current))
		sdio_release_host(dev->pfunction);

	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Synch Cmd53 write failed %d\n",
			__func__, status);
		dev->write_fail = 2;
	} else {
		memcpy(dev->prev_desc, data, FRAME_DESC_SZ);
	}
	return status;
}

int rsi_sdio_load_data_master_write(struct rsi_hw *adapter,
				    u32 base_address,
				    u32 instructions_sz,
				    u16 block_size,
				    u8 *ta_firmware)
{
	u32 num_blocks;
	u16 msb_address;
	u32 offset, ii;
	u8 temp_buf[block_size];
	u16 lsb_address;

	num_blocks = instructions_sz / block_size;
	msb_address = base_address >> 16;

	ven_rsi_dbg(INFO_ZONE, "ins_size: %d\n", instructions_sz);
	ven_rsi_dbg(INFO_ZONE, "num_blocks: %d\n", num_blocks);

	/* Loading DM ms word in the sdio slave */
	if (rsi_sdio_master_access_msword(adapter, msb_address)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to set ms word reg\n", __func__);
		return -EIO;
	}

	for (offset = 0, ii = 0; ii < num_blocks; ii++, offset += block_size) {
		memset(temp_buf, 0, block_size);
		memcpy(temp_buf, ta_firmware + offset, block_size);
		lsb_address = (u16)base_address;
		if (rsi_sdio_write_register_multiple(adapter,
					lsb_address | SD_REQUEST_MASTER,
					temp_buf, block_size)) {
			ven_rsi_dbg(ERR_ZONE, "%s: failed to write\n", __func__);
			return -EIO;
		}
		ven_rsi_dbg(INFO_ZONE, "%s: loading block: %d\n", __func__, ii);
		base_address += block_size;

		if ((base_address >> 16) != msb_address) {
			msb_address += 1;

			/* Loading DM ms word in the sdio slave */
			if (rsi_sdio_master_access_msword(adapter,
							  msb_address)) {
				ven_rsi_dbg(ERR_ZONE,
					"%s: Unable to set ms word reg\n",
					__func__);
				return -EIO;
			}
		}
	}

	if (instructions_sz % block_size) {
		memset(temp_buf, 0, block_size);
		memcpy(temp_buf,
		       ta_firmware + offset,
		       instructions_sz % block_size);
		lsb_address = (u16)base_address;
		if (rsi_sdio_write_register_multiple(adapter,
						lsb_address | SD_REQUEST_MASTER,
						temp_buf,
						instructions_sz % block_size)) {
			return -EIO;
		}
		ven_rsi_dbg(INFO_ZONE,
			"Written Last Block in Address 0x%x Successfully\n",
			offset | SD_REQUEST_MASTER);
	}
	return 0;
}

int rsi_sdio_master_reg_read(struct rsi_hw *adapter, u32 addr,
			     u32 *read_buf, u16 size)
{
	u32 *data = NULL;
	u16 ms_addr = 0;
	u32 align[2] = {};
	u32 addr_on_bus;

	data = PTR_ALIGN(&align[0], 8);

	ms_addr = (addr >> 16);
	if (rsi_sdio_master_access_msword(adapter, ms_addr)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to set ms word to common reg\n",
			__func__);
		return -EIO;
	}
	addr = addr & 0xFFFF;

	addr_on_bus = (addr & 0xFF000000);
	if ((addr_on_bus == (FLASH_SIZE_ADDR & 0xFF000000)) ||
	    (addr_on_bus == 0x0)) {
		addr_on_bus = (addr & ~(0x3));
	} else
		addr_on_bus = addr;

	/* Bring TA out of reset */
	if (rsi_sdio_read_register_multiple(adapter,
					    (addr_on_bus | SD_REQUEST_MASTER),
					    (u8 *)data, 4)) {
		ven_rsi_dbg(ERR_ZONE, "%s: AHB register read failed\n", __func__);
		return -EIO;
	}
	if (size == 2) {
		if ((addr & 0x3) == 0)
			*read_buf = *data;
		else
			*read_buf  = (*data >> 16);
		*read_buf = (*read_buf & 0xFFFF);
	} else if (size == 1) {
		if ((addr & 0x3) == 0)
			*read_buf = *data;
		else if ((addr & 0x3) == 1)
			*read_buf = (*data >> 8);
		else if ((addr & 0x3) == 2)
			*read_buf = (*data >> 16);
		else
			*read_buf = (*data >> 24);
		*read_buf = (*read_buf & 0xFF);
	} else { /*size is 4 */
		*read_buf = *data;
	}

	return 0;
}

int rsi_sdio_master_reg_write(struct rsi_hw *adapter,
			      unsigned long addr,
			      unsigned long data,
			      u16 size)
{
	unsigned long data1[2];
	unsigned long *data_aligned;

	data_aligned = PTR_ALIGN(&data1[0], 8);

	if (size == 2) {
		*data_aligned = ((data << 16) | (data & 0xFFFF));
	} else if (size == 1) {
		u32 temp_data;

		temp_data = (data & 0xFF);
		*data_aligned = ((temp_data << 24) |
				  (temp_data << 16) |
				  (temp_data << 8) |
				  (temp_data));
	} else {
		*data_aligned = data;
	}
	size = 4;

	if (rsi_sdio_master_access_msword(adapter, (addr >> 16))) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to set ms word to common reg\n",
			__func__);
		return -EIO;
	}
	addr = addr & 0xFFFF;

	/* Bring TA out of reset */
	if (rsi_sdio_write_register_multiple(adapter,
					     (addr | SD_REQUEST_MASTER),
					     (u8 *)data_aligned, size)) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Unable to do AHB reg write\n", __func__);
		return -EIO;
	}
	return 0;
}

/**
 * rsi_sdio_host_intf_write_pkt() - This function writes the packet to device.
 * @adapter: Pointer to the adapter structure.
 * @pkt: Pointer to the data to be written on to the device.
 * @len: length of the data to be written on to the device.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_host_intf_write_pkt(struct rsi_hw *adapter,
				 u8 *pkt,
				 u32 len)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	u32 block_size = dev->tx_blk_size;
	u32 num_blocks, address, length;
	u32 queueno;
	int status;

	queueno = ((pkt[1] >> 4) & 0xf);
	if ((queueno == RSI_BT_DATA_Q) || (queueno == RSI_BT_MGMT_Q))
		queueno = RSI_BT_Q;

	num_blocks = len / block_size;

	if (len % block_size)
		num_blocks++;

	address = (num_blocks * block_size | (queueno << 12));
	length  = num_blocks * block_size;

	status = rsi_sdio_write_register_multiple(adapter,
						  address,
						  pkt,
						  length);
	if (status < 0)
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to write onto the card: %d\n",
			__func__, status);
	ven_rsi_dbg(DATA_TX_ZONE, "%s: Successfully written onto card\n", __func__);
	return status;
}

/**
 * rsi_sdio_host_intf_read_pkt() - This function reads the packet
				   from the device.
 * @adapter: Pointer to the adapter data structure.
 * @pkt: Pointer to the packet data to be read from the the device.
 * @length: Length of the data to be read from the device.
 *
 * Return: 0 on success, -1 on failure.
 */
int rsi_sdio_host_intf_read_pkt(struct rsi_hw *adapter,
				u8 *pkt,
				u32 length)
{
	int status = -EINVAL;

	if (!length) {
		ven_rsi_dbg(ERR_ZONE, "%s: Pkt size is zero\n", __func__);
		return status;
	}

	status = rsi_sdio_read_register_multiple(adapter,
						 length,
						 (u8 *)pkt,
						 length);

	if (status)
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to read frame: %d\n", __func__,
			status);
	return status;
}

/**
 * rsi_init_sdio_interface() - This function does init specific to SDIO.
 *
 * @adapter: Pointer to the adapter data structure.
 * @pkt: Pointer to the packet data to be read from the the device.
 *
 * Return: 0 on success, -1 on failure.
 */

static int rsi_init_sdio_interface(struct rsi_hw *adapter,
				   struct sdio_func *pfunction)
{
	struct rsi_91x_sdiodev *rsi_91x_dev;
	int status = -ENOMEM;

	rsi_91x_dev = kzalloc(sizeof(*rsi_91x_dev), GFP_KERNEL);
	if (!rsi_91x_dev)
		return status;

	adapter->rsi_dev = rsi_91x_dev;
	rsi_91x_dev->sdio_irq_task = NULL;

	sdio_claim_host(pfunction);

	pfunction->enable_timeout = 100;
	status = sdio_enable_func(pfunction);
	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to enable interface\n", __func__);
		sdio_release_host(pfunction);
		return status;
	}

	ven_rsi_dbg(INIT_ZONE, "%s: Enabled the interface\n", __func__);

	rsi_91x_dev->pfunction = pfunction;
	adapter->device = &pfunction->dev;

	sdio_set_drvdata(pfunction, adapter);

	status = rsi_setupcard(adapter);
	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to setup card\n", __func__);
		goto fail;
	}

	ven_rsi_dbg(INIT_ZONE, "%s: Setup card successfully\n", __func__);

	status = rsi_init_sdio_slave_regs(adapter);
	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to init slave regs\n", __func__);
		goto fail;
	}
	sdio_release_host(pfunction);

	adapter->determine_event_timeout = rsi_sdio_determine_event_timeout;
	adapter->process_isr_hci = rsi_interrupt_handler;
	adapter->check_intr_status_reg = rsi_read_intr_status_reg;
	
#ifdef CONFIG_VEN_RSI_DEBUGFS
	adapter->num_debugfs_entries = MAX_DEBUGFS_ENTRIES;
#endif
	return status;
fail:
	sdio_disable_func(pfunction);
	sdio_release_host(pfunction);
	return status;
}

static struct rsi_host_intf_ops sdio_host_intf_ops = {
	.write_pkt		= rsi_sdio_host_intf_write_pkt,
	.read_pkt		= rsi_sdio_host_intf_read_pkt,
	.master_access_msword	= rsi_sdio_master_access_msword,
	.master_reg_read	= rsi_sdio_master_reg_read,
	.master_reg_write	= rsi_sdio_master_reg_write,
	.read_reg_multiple	= rsi_sdio_read_register_multiple,
	.write_reg_multiple	= rsi_sdio_write_register_multiple,
	.load_data_master_write	= rsi_sdio_load_data_master_write,
	.check_hw_queue_status	= rsi_sdio_check_buffer_status,
};

/**
 * rsi_probe() - This function is called by kernel when the driver provided
 *		 Vendor and device IDs are matched. All the initialization
 *		 work is done here.
 * @pfunction: Pointer to the sdio_func structure.
 * @id: Pointer to sdio_device_id structure.
 *
 * Return: 0 on success, 1 on failure.
 */
static int rsi_probe(struct sdio_func *pfunction,
		     const struct sdio_device_id *id)
{
	struct rsi_hw *adapter;

	ven_rsi_dbg(INIT_ZONE, "%s: Init function called\n", __func__);

	adapter = ven_rsi_91x_init();
	if (!adapter) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to init os intf ops\n",
			__func__);
		return 1;
	}
	adapter->rsi_host_intf = RSI_HOST_INTF_SDIO;
	adapter->host_intf_ops = &sdio_host_intf_ops;
	adapter->priv->oper_mode = dev_oper_mode;

	if (rsi_init_sdio_interface(adapter, pfunction)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to init sdio interface\n",
			__func__);
		goto fail;
	}
#ifdef CONFIG_SDIO_INTR_POLL
	init_sdio_intr_status_poll_thread(adapter->priv);
#endif
	sdio_claim_host(pfunction);
//	if (sdio_claim_irq(pfunction, rsi_handle_interrupt)) {
	if (sdio_claim_irq(pfunction, rsi_dummy_isr)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to request IRQ\n", __func__);
		sdio_release_host(pfunction);
		goto fail;
	}
	sdio_release_host(pfunction);
	ven_rsi_dbg(INIT_ZONE, "%s: Registered Interrupt handler\n", __func__);

	if (rsi_hal_device_init(adapter)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in device init\n", __func__);
		sdio_claim_host(pfunction);
		sdio_release_irq(pfunction);
		sdio_disable_func(pfunction);
		sdio_release_host(pfunction);
		goto fail;
	}
	ven_rsi_dbg(INFO_ZONE, "===> RSI Device Init Done <===\n");
	
	if (rsi_sdio_master_access_msword(adapter, MISC_CFG_BASE_ADDR)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to set ms word reg\n", __func__);
		return -EIO;
	}
	ven_rsi_dbg(INIT_ZONE, "%s: Setting ms word to 0x41050000\n", __func__);

	sdio_claim_host(pfunction);
		sdio_release_irq(pfunction);
	mdelay(10);
	if (sdio_claim_irq(pfunction, rsi_handle_interrupt)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to request IRQ\n", __func__);
		sdio_release_host(pfunction);
		goto fail;
	}
	sdio_release_host(pfunction);
	adapter->priv->hibernate_resume = false;

	return 0;

fail:
#ifdef CONFIG_SDIO_INTR_POLL
	rsi_kill_thread(&adapter->priv->sdio_intr_poll_thread);
#endif
	ven_rsi_91x_deinit(adapter);
	ven_rsi_dbg(ERR_ZONE, "%s: Failed in probe...Exiting\n", __func__);
	return 1;
}

/**
 * rsi_disconnect() - This function performs the reverse of the probe function.
 * @pfunction: Pointer to the sdio_func structure.
 *
 * Return: void.
 */
static void rsi_disconnect(struct sdio_func *pfunction)
{
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);
	struct rsi_91x_sdiodev *dev;

	if (!adapter)
		return;

	dev = (struct rsi_91x_sdiodev *)adapter->rsi_dev;

#ifdef CONFIG_SDIO_INTR_POLL
	rsi_kill_thread(&adapter->priv->sdio_intr_poll_thread);
#endif
	sdio_claim_host(pfunction);
	sdio_release_irq(pfunction);
	sdio_release_host(pfunction);

	ven_rsi_mac80211_detach(adapter);

#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	rsi_hci_detach(adapter->priv);
#endif

	if (!adapter->priv->hibernate_resume) {
		/* Reset Chip */
		rsi_reset_chip(adapter);

		/* Resetting to take care of the case, where-in driver
		 * is re-loaded */
		sdio_claim_host(pfunction);
		rsi_reset_card(pfunction);
		sdio_disable_func(pfunction);
		sdio_release_host(pfunction);
	}
	dev->write_fail = 2;
	ven_rsi_91x_deinit(adapter);
	ven_rsi_dbg(ERR_ZONE, "##### RSI SDIO device disconnected #####\n");
}

#ifdef CONFIG_PM
int rsi_set_sdio_pm_caps(struct rsi_hw *adapter)
{
	struct rsi_91x_sdiodev *dev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;
	struct sdio_func *func = dev->pfunction;
	int ret;

	/* Keep Power to the MMC while suspend */
	ret = sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
	if (ret) {
		ven_rsi_dbg(ERR_ZONE, "set sdio keep pwr flag failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static int rsi_sdio_disable_interrupts(struct sdio_func *pfunction)
{
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);
	u8 isr_status = 0, data = 0;
	int ret;

	ven_rsi_dbg(ERR_ZONE, "Waiting for interrupts to be cleared..");
	do {
		rsi_sdio_read_register(adapter,
				       RSI_FN1_INT_REGISTER,
				       &isr_status);
		ven_rsi_dbg(ERR_ZONE, ".");
	} while (isr_status);
	ven_rsi_dbg(ERR_ZONE, "\nInterrupts cleared");

	sdio_claim_host(pfunction);
	ret = rsi_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to read INTR_EN register\n",
			__func__);
		sdio_release_host(pfunction);
		return ret;
	}
	ven_rsi_dbg(INFO_ZONE, "INTR_EN reg content = %x\n", data);

	/* And bit0 and b1 */
	data &= 0xfc;

	ret = rsi_cmd52writebyte(pfunction->card, 0x04, data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to Write to INTR_EN register\n",
			__func__);
		sdio_release_host(pfunction);
		return ret;
	}
	ret = rsi_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to read INTR_EN register\n",
			__func__);
		sdio_release_host(pfunction);
		return ret;
	}
	ven_rsi_dbg(INFO_ZONE, "INTR_EN reg content. = %x\n", data);

	sdio_release_host(pfunction);
	
	return 0;
}

static int rsi_sdio_enable_interrupts(struct sdio_func *pfunction)
{
	u8 data;
	int ret;

	sdio_claim_host(pfunction);
	ret = rsi_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to read INTR_EN register\n", __func__);
		sdio_release_host(pfunction);
		return ret;
	}
	ven_rsi_dbg(INFO_ZONE, "INTR_EN reg content1 = %x\n", data);

	/* Enable b1 and b0 */
	data |= 0x03;

	ret = rsi_cmd52writebyte(pfunction->card, 0x04, data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to Write to INTR_EN register\n",
			__func__);
		sdio_release_host(pfunction);
		return ret;
	}
	
        ret = rsi_cmd52readbyte(pfunction->card, 0x04, &data);
	if (ret < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Failed to read INTR_EN register\n", __func__);
		sdio_release_host(pfunction);
		return ret;
	}
	ven_rsi_dbg(INFO_ZONE, "INTR_EN reg content1.. = %x\n", data);
	sdio_release_host(pfunction);

	return ret;
}

static int rsi_suspend(struct device *dev)
{
	int ret = 0;
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);
	struct rsi_common *common = adapter->priv;
	struct rsi_91x_sdiodev *sdev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;

	ven_rsi_dbg(ERR_ZONE, "SDIO Bus suspend ===>\n");

	if (!adapter) {
		ven_rsi_dbg(ERR_ZONE, "Device is not ready\n");
		return -ENODEV;
	}

	common->suspend_in_prog = true;
#ifdef CONFIG_VEN_RSI_WOW
	if (common->wow_flags & RSI_WOW_ENABLED) {
		if (common->wow_flags & RSI_WOW_NO_CONNECTION)
			ven_rsi_dbg(ERR_ZONE,
				"##### Device can not wake up through WLAN\n");

#if 0
#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
		if ((common->coex_mode == 2) || (common->coex_mode == 4)) {
			/* Deregister BT protocol */
			rsi_hci_detach(common);
		}
#endif
#endif
	}
#endif

	ret = rsi_sdio_disable_interrupts(pfunction);

	if (sdev->write_fail)
		ven_rsi_dbg(INFO_ZONE, "###### Device is not ready #######\n");

	ret = rsi_set_sdio_pm_caps(adapter);
	if (ret)
		ven_rsi_dbg(INFO_ZONE,
			"Setting power management caps failed\n");

	common->fsm_state = FSM_CARD_NOT_READY;
	ven_rsi_dbg(INFO_ZONE, "***** SDIO BUS SUSPEND DONE ******\n");

	return 0;
}

int rsi_resume(struct device *dev)
{
	int ret = 0;
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);
	struct rsi_common *common = adapter->priv;
        
	ven_rsi_dbg(INFO_ZONE, "***** BUS RESUME ******\n");

	common->suspend_in_prog = false;
	common->fsm_state = FSM_MAC_INIT_DONE;

	ret = rsi_sdio_enable_interrupts(pfunction);

#if 0
#ifdef CONFIG_VEN_RSI_WOW
#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	if ((common->wow_flags & RSI_WOW_ENABLED) &&
	    ((common->coex_mode == 2) || (common->coex_mode == 4))) {
		/* Register BT protocol */
		rsi_hci_attach(common);
	}
#endif
        adapter->priv->wow_flags = 0;
#endif
#endif

	ven_rsi_dbg(INFO_ZONE, "***** RSI module resumed *****\n");
	return 0;
}

static int rsi_freeze(struct device *dev)
{
	int ret = 0;
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);
	struct rsi_common *common = adapter->priv;
	struct rsi_91x_sdiodev *sdev =
		(struct rsi_91x_sdiodev *)adapter->rsi_dev;

	ven_rsi_dbg(INFO_ZONE, "SDIO Bus freeze ===>\n");

	if (!adapter) {
		ven_rsi_dbg(ERR_ZONE, "Device is not ready\n");
		return -ENODEV;
	}

	common->suspend_in_prog = true;
#ifdef CONFIG_VEN_RSI_WOW
	if (common->wow_flags & RSI_WOW_ENABLED) {
		if (common->wow_flags & RSI_WOW_NO_CONNECTION)
			ven_rsi_dbg(ERR_ZONE,
				"##### Device can not wake up through WLAN\n");

#if 0
#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
		if ((common->coex_mode == 2) || (common->coex_mode == 4)) {
			/* Deregister BT protocol */
			rsi_deregister_bt(common);
		}
#endif
#endif
	}
#endif

	ret = rsi_sdio_disable_interrupts(pfunction);

	if (sdev->write_fail)
		ven_rsi_dbg(INFO_ZONE, "###### Device is not ready #######\n");
	
	ret = rsi_set_sdio_pm_caps(adapter);
	if (ret)
		ven_rsi_dbg(INFO_ZONE, "Setting power management caps failed\n");
	
	return 0;
}

int rsi_thaw(struct device *dev)
{
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);

	ven_rsi_dbg(ERR_ZONE, "***** BUS THAW ******\n");

//	adapter->priv->suspend_in_prog = false;
	adapter->priv->hibernate_resume = true;
	adapter->priv->fsm_state = FSM_CARD_NOT_READY;
	adapter->priv->bt_fsm_state = BT_DEVICE_NOT_READY;
	adapter->priv->iface_down = true;

	rsi_sdio_enable_interrupts(pfunction);

	ven_rsi_dbg(INFO_ZONE, "RSI module resumed\n");
	return 0;
}

static int rsi_poweroff(struct device *dev)
{
	return rsi_freeze(dev);
}

static void rsi_shutdown(struct device *dev)
{
	rsi_freeze(dev);
}

int rsi_restore(struct device *dev)
{
	struct sdio_func *pfunction = dev_to_sdio_func(dev);
	struct rsi_hw *adapter = sdio_get_drvdata(pfunction);

	adapter->priv->suspend_in_prog = false;
	adapter->priv->hibernate_resume = true;
	adapter->priv->fsm_state = FSM_CARD_NOT_READY;
	adapter->priv->bt_fsm_state = BT_DEVICE_NOT_READY;
	adapter->priv->iface_down = true;

	ven_rsi_dbg(INFO_ZONE, "RSI module restored\n");

	return 0;
}

static const struct dev_pm_ops rsi_pm_ops = {
	.suspend = rsi_suspend,
	.resume = rsi_resume,
	.freeze = rsi_freeze,
	.thaw = rsi_thaw,
	.poweroff = rsi_poweroff,
	.restore = rsi_restore,
};
#endif

static const struct sdio_device_id rsi_dev_table[] =  {
#if 0
	{ SDIO_DEVICE(0x303, 0x100) },
	{ SDIO_DEVICE(0x041B, 0x0301) },
	{ SDIO_DEVICE(0x041B, 0x0201) },
#endif
	{ SDIO_DEVICE(0x041B, 0x9330) },
	{ /* Blank */},
};

static struct sdio_driver rsi_driver = {
	.name       = "RSI-SDIO WLAN",
	.probe      = rsi_probe,
	.remove     = rsi_disconnect,
	.id_table   = rsi_dev_table,
#ifdef CONFIG_PM
	.drv = {
		.pm = &rsi_pm_ops,
	        .shutdown   = rsi_shutdown,
	}
#endif
};

/**
 * rsi_module_init() - This function registers the sdio module.
 * @void: Void.
 *
 * Return: 0 on success.
 */
static int rsi_module_init(void)
{
	int ret;

	ret = sdio_register_driver(&rsi_driver);
	ven_rsi_dbg(INIT_ZONE, "%s: Registering driver\n", __func__);
	return ret;
}

/**
 * rsi_module_exit() - This function unregisters the sdio module.
 * @void: Void.
 *
 * Return: None.
 */
static void rsi_module_exit(void)
{
	sdio_unregister_driver(&rsi_driver);
	ven_rsi_dbg(INFO_ZONE, "%s: Unregistering driver\n", __func__);
}

module_init(rsi_module_init);
module_exit(rsi_module_exit);

MODULE_AUTHOR("Redpine Signals Inc");
MODULE_DESCRIPTION("Common SDIO layer for RSI drivers");
MODULE_SUPPORTED_DEVICE("RSI-91x");
MODULE_DEVICE_TABLE(sdio, rsi_dev_table);
MODULE_FIRMWARE(FIRMWARE_RSI9113);
MODULE_VERSION(DRV_VER);
MODULE_LICENSE("Dual BSD/GPL");
