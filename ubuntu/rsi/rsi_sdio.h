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

#ifndef __RSI_SDIO_INTF__
#define __RSI_SDIO_INTF__

#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio_ids.h>
#include "rsi_main.h"

/* Interrupt Bit Related Macros */
#define PKT_BUFF_AVAILABLE		1
#define FW_ASSERT_IND			2

#define RSI_DEVICE_BUFFER_STATUS_REGISTER 0xf3
#define RSI_FN1_INT_REGISTER		0xf9
#define SD_REQUEST_MASTER		0x10000

/* FOR SD CARD ONLY */
#define SDIO_RX_NUM_BLOCKS_REG		0x000F1
#define SDIO_FW_STATUS_REG		0x000F2
#define SDIO_NXT_RD_DELAY2		0x000F5
#define SDIO_MASTER_ACCESS_MSBYTE	0x000FA
#define SDIO_MASTER_ACCESS_LSBYTE	0x000FB
#define SDIO_READ_START_LVL		0x000FC
#define SDIO_READ_FIFO_CTL		0x000FD
#define SDIO_WRITE_FIFO_CTL		0x000FE
#define SDIO_WAKEUP_REG			0x000FF

#define SDIO_FUN1_INTR_CLR_REG		0x0008
#define SDIO_REG_HIGH_SPEED		0x0013
#define TA_SOFT_RESET_REG		0x0004
#define TA_TH0_PC_REG			0x0400
#define TA_HOLD_THREAD_REG		0x0844       
#define TA_RELEASE_THREAD_REG		0x0848
#define TA_POLL_BREAK_STATUS_REG	0x085C


#define RSI_GET_SDIO_INTERRUPT_TYPE(_I, TYPE)      \
	{					   \
		TYPE =                             \
		(_I & (1 << PKT_BUFF_AVAILABLE)) ? \
		BUFFER_AVAILABLE :		   \
		(_I & (1 << MSDU_PKT_PENDING)) ?   \
		MSDU_PACKET_PENDING :              \
		(_I & (1 << FW_ASSERT_IND)) ?      \
		FIRMWARE_ASSERT_IND : UNKNOWN_INT; \
	}

/* common registers in SDIO function1 */
#define TA_SOFT_RESET_REG		0x0004
#define TA_TH0_PC_REG			0x0400
#define TA_HOLD_THREAD_REG		0x0844
#define TA_RELEASE_THREAD_REG		0x0848

#define TA_SOFT_RST_CLR			0
#define TA_SOFT_RST_SET			BIT(0)
#define TA_PC_ZERO			0
#define TA_HOLD_THREAD_VALUE		cpu_to_le32(0xF)
#define TA_RELEASE_THREAD_VALUE		cpu_to_le32(0xF)
#define TA_BASE_ADDR			0x2200
#define MISC_CFG_BASE_ADDR		0x4105

enum sdio_interrupt_type {
	BUFFER_FULL		= 0x0,
	BUFFER_AVAILABLE	= 0x2,
	FIRMWARE_ASSERT_IND	= 0x3,
	MSDU_PACKET_PENDING	= 0x4,
	UNKNOWN_INT		= 0XE
};

struct receive_info {
	bool buffer_full;
	bool semi_buffer_full;
	bool mgmt_buffer_full;
	u32 mgmt_buf_full_counter;
	u32 buf_semi_full_counter;
	u8 watch_bufferfull_count;
	u32 sdio_intr_status_zero;
	u32 sdio_int_counter;
	u32 total_sdio_msdu_pending_intr;
	u32 total_sdio_unknown_intr;
	u32 buf_full_counter;
	u32 buf_available_counter;
};

struct rsi_91x_sdiodev {
	struct sdio_func *pfunction;
	struct task_struct *sdio_irq_task;
	struct receive_info rx_info;
	u32 next_read_delay;
	u32 sdio_high_speed_enable;
	u8 sdio_clock_speed;
	u32 cardcapability;
	u8 prev_desc[16];
	u32 tx_blk_size;
	u8 write_fail;
	u8 buff_status_updated;
};

void rsi_interrupt_handler(struct rsi_hw *adapter);
int rsi_init_sdio_slave_regs(struct rsi_hw *adapter);
int rsi_sdio_device_init(struct rsi_common *common);
int rsi_sdio_read_register(struct rsi_hw *adapter, u32 addr, u8 *data);
int rsi_sdio_write_register(struct rsi_hw *adapter, u8 function,
			    u32 addr, u8 *data);
int rsi_sdio_host_intf_read_pkt(struct rsi_hw *adapter, u8 *pkt, u32 length);
int rsi_sdio_host_intf_write_pkt(struct rsi_hw *adapter, u8 *pkt, u32 len);
int rsi_sdio_read_register_multiple(struct rsi_hw *adapter, u32 addr,
				    u8 *data, u16 count);
int rsi_sdio_write_register_multiple(struct rsi_hw *adapter, u32 addr,
				     u8 *data, u16 count);
int rsi_sdio_master_access_msword(struct rsi_hw *adapter,
				  u16 ms_word);
int rsi_sdio_load_data_master_write(struct rsi_hw *adapter,
				    u32 base_address, u32 instructions_sz,
				    u16 block_size, u8 *ta_firmware);
int rsi_sdio_master_reg_read(struct rsi_hw *adapter, u32 addr,
			     u32 *read_buf, u16 size);
int rsi_sdio_master_reg_write(struct rsi_hw *adapter,
			      unsigned long addr,
			      unsigned long data,
			      u16 size);
void rsi_sdio_ack_intr(struct rsi_hw *adapter, u8 int_bit);
int rsi_sdio_determine_event_timeout(struct rsi_hw *adapter);
int rsi_sdio_check_buffer_status(struct rsi_hw *adapter, u8 q_num);
int rsi_read_intr_status_reg(struct rsi_hw *adapter);
#endif
