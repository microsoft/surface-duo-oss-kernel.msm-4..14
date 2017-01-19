/**
 * Copyright (c) 2014 Redpine Signals Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __RSI_HAL_H__
#define __RSI_HAL_H__

#define TA_LOAD_ADDRESS			0x00
#define FIRMWARE_RSI9113		"rsi_91x.fw"
#define FLASH_WRITE_CHUNK_SIZE		(4 * 1024)
#define USB_FLASH_READ_CHUNK_SIZE	((2 * 1024) - 4)
#define SDIO_FLASH_READ_CHUNK_SIZE	(2 * 1024)
#define FLASH_SECTOR_SIZE		(4 * 1024)
#define STARTING_BLOCK_INDEX		0
#define FLASH_BLOCK_SIZE		(32 * 1024)

#define FLASH_SIZE_ADDR			0x04000016
#define PING_BUFFER_ADDRESS		0x19000
#define PONG_BUFFER_ADDRESS		0x1a000
#define SWBL_REGIN			0x41050034
#define SWBL_REGOUT			0x4105003c
#define PING_WRITE			0x1
#define PONG_WRITE			0x2

#define BL_CMD_TIMEOUT			2000
#define BL_BURN_TIMEOUT			(50 * 1000)

#define MASTER_READ_MODE		1
#define EEPROM_READ_MODE		2

#define REGIN_VALID			0xA
#define REGIN_INPUT			0xA0
#define REGOUT_VALID			0xAB
#define REGOUT_INVALID			(~0xAB)
#define CMD_PASS			0xAA
#define CMD_FAIL			0xCC
#define INVALID_ADDR			0x4C

#define BURN_BL				0x23
#define LOAD_HOSTED_FW			'A'
#define BURN_HOSTED_FW			'B'
#define PING_VALID			'I'
#define PONG_VALID			'O'
#define PING_AVAIL			'I'
#define PONG_AVAIL			'O'
#define EOF_REACHED			'E'
#define CHECK_CRC			'K'
#define POLLING_MODE			'P'
#define CONFIG_AUTO_READ_MODE		'R'
#define JUMP_TO_ZERO_PC			'J'
#define FW_LOADING_SUCCESSFUL		'S'
#define LOADING_INITIATED		'1'

/* Boot loader commands */
#define HOST_INTF_REG_OUT		0x4105003C
#define HOST_INTF_REG_IN		0x41050034
#define BOARD_READY			0xABCD
#define REG_READ			0xD1
#define REG_WRITE			0xD2
#define SEND_RPS_FILE			'2'
#define BOOTUP_OPTIONS_LAST_CONFIG_NOT_SAVED 0xF1
#define BOOTUP_OPTIONS_CHECKSUM_FAIL 0xF2
#define INVALID_OPTION			0xF3
#define CHECKSUM_SUCCESS		0xAA
#define CHECKSUM_FAILURE		0xCC
#define CHECKSUM_INVALID_ADDRESS	0x4C

#define EEPROM_VERSION_OFFSET		77
#define CALIB_CRC_OFFSET		4092
#define MAGIC_WORD			0x5A
#define MAGIC_WORD_OFFSET_1		40
#define MAGIC_WORD_OFFSET_2		424
#define FW_IMAGE_MIN_ADDRESS		(68 * 1024)
#define FLASH_MAX_ADDRESS		(4 * 1024 * 1024) //4MB
#define MAX_FLASH_FILE_SIZE		(400 * 1024) //400K
#define FLASHING_START_ADDRESS		16
#define CALIB_VALUES_START_ADDR		16
#define SOC_FLASH_ADDR			0x04000000
#define EEPROM_DATA_SIZE		4096
#define CALIB_DATA_SIZE		(EEPROM_DATA_SIZE - CALIB_VALUES_START_ADDR)
#define BL_HEADER			32

#define BT_CARD_READY_IND		0x89
#define WLAN_CARD_READY_IND		0x0
#define COMMON_HAL_CARD_READY_IND	0x0
#define ZIGB_CARD_READY_IND		0xff

#define COMMAN_HAL_WAIT_FOR_CARD_READY	1
#define COMMON_HAL_SEND_CONFIG_PARAMS	2
#define COMMON_HAL_TX_ACCESS		3
#define COMMON_HAL_WAIT_FOR_PROTO_CARD_READY 4
#define HEX_FILE			1
#define BIN_FILE			0
#define UNIX_FILE_TYPE			8
#define DOS_FILE_TYPE			9
#define LMAC_INSTRUCTIONS_SIZE		(16  * 1024) /* 16Kbytes */

#define ULP_RESET_REG			0x161
#define WATCH_DOG_TIMER_1		0x16c
#define WATCH_DOG_TIMER_2		0x16d
#define WATCH_DOG_DELAY_TIMER_1		0x16e
#define WATCH_DOG_DELAY_TIMER_2		0x16f
#define WATCH_DOG_TIMER_ENABLE		0x170

#define RESTART_WDT			BIT(11)
#define BYPASS_ULP_ON_WDT		BIT(1)

#define RF_SPI_PROG_REG_BASE_ADDR	0x40080000

#define GSPI_CTRL_REG0			(RF_SPI_PROG_REG_BASE_ADDR)
#define GSPI_CTRL_REG1			(RF_SPI_PROG_REG_BASE_ADDR + 0x2)
#define GSPI_DATA_REG0			(RF_SPI_PROG_REG_BASE_ADDR + 0x4)
#define GSPI_DATA_REG1			(RF_SPI_PROG_REG_BASE_ADDR + 0x6)
#define GSPI_DATA_REG2			(RF_SPI_PROG_REG_BASE_ADDR + 0x8)

#define GSPI_DMA_MODE			BIT(13)

#define GSPI_2_ULP			BIT(12)
#define GSPI_TRIG			BIT(7)
#define GSPI_READ			BIT(6)
#define GSPI_RF_SPI_ACTIVE		BIT(8)

struct bl_header {
	u32 flags;
	u32 image_no;
	u32 check_sum;
	u32 flash_start_address;
	u32 flash_len;
} __packed;

struct ta_metadata {
	char *name;
	unsigned int address;
};

int rsi_prepare_mgmt_desc(struct rsi_common *common, struct sk_buff *skb);
int rsi_prepare_data_desc(struct rsi_common *common, struct sk_buff *skb);
int rsi_hal_device_init(struct rsi_hw *adapter);
int rsi_send_data_pkt(struct rsi_common *common, struct sk_buff *skb);
int rsi_send_bt_pkt(struct rsi_common *common, struct sk_buff *skb);
int rsi_send_beacon(struct rsi_common *common);

#endif
