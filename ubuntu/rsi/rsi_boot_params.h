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

#ifndef __RSI_BOOTPARAMS_HEADER_H__
#define __RSI_BOOTPARAMS_HEADER_H__

#define CRYSTAL_GOOD_TIME		BIT(0)
#define BOOTUP_MODE_INFO                BIT(1)
#define WIFI_TAPLL_CONFIGS              BIT(5)
#define WIFI_PLL960_CONFIGS             BIT(6)
#define WIFI_AFEPLL_CONFIGS             BIT(7)
#define WIFI_SWITCH_CLK_CONFIGS         BIT(8)

#define TAPLL_M_VAL_20			9
#define TAPLL_N_VAL_20			0
#define TAPLL_P_VAL_20			4

#define PLL960_M_VAL_20                 0x14
#define PLL960_N_VAL_20                 0
#define PLL960_P_VAL_20                 5

#define UMAC_CLK_40MHZ                  80

#define TAPLL_M_VAL_40                  9
#define TAPLL_N_VAL_40                  0
#define TAPLL_P_VAL_40                  4

#define PLL960_M_VAL_40                 0x14
#define PLL960_N_VAL_40                 0
#define PLL960_P_VAL_40                 5

#define UMAC_CLK_20BW \
	(((TAPLL_M_VAL_20 + 1) * 40) / \
	 ((TAPLL_N_VAL_20 + 1) * (TAPLL_P_VAL_20 + 1)))
#define VALID_20 \
	(WIFI_TAPLL_CONFIGS | WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | \
	 WIFI_SWITCH_CLK_CONFIGS | BOOTUP_MODE_INFO | CRYSTAL_GOOD_TIME)

#define UMAC_CLK_40BW   \
	(((TAPLL_M_VAL_40 + 1) * 40) / \
	 ((TAPLL_N_VAL_40 + 1) * (TAPLL_P_VAL_40 + 1)))
#define VALID_40 \
	(WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | WIFI_SWITCH_CLK_CONFIGS | \
	 WIFI_TAPLL_CONFIGS | CRYSTAL_GOOD_TIME | BOOTUP_MODE_INFO)

/* TAPLL programming configurations */
struct tapll_info {
	__le16 pll_reg_1;
	__le16 pll_reg_2;
} __packed;

/* PLL960 programming configurations */
struct pll960_info {
	__le16 pll_reg_1;
	__le16 pll_reg_2;
	__le16 pll_reg_3;
} __packed;

/* AFEPLL programming configurations */
struct afepll_info {
	__le16 pll_reg;
} __packed;

/* PLL configurations */
struct pll_config {
	struct tapll_info tapll_info_g;
	struct pll960_info pll960_info_g;
	struct afepll_info afepll_info_g;
} __packed;

/* UMAC clk programming configurations */
struct switch_clk {
	__le16 switch_umac_clk : 1; /* If set rest is valid */
	__le16 switch_qspi_clk : 1; /* If set qspi clk will be changed */
	__le16 switch_slp_clk_2_32 : 1;
	__le16 switch_bbp_lmac_clk_reg : 1;
	__le16 switch_mem_ctrl_cfg : 1;
	__le16 reserved : 11;

	/* If switch_bbp_lmac_clk_reg is set then this value will be programmed
	 * into reg
	 */
	__le16 bbp_lmac_clk_reg_val;
	/* if switch_umac_clk is set then this value will be programmed */
	__le16 umac_clock_reg_config;
	/* if switch_qspi_clk is set then this value will be programmed */
	__le16 qspi_uart_clock_reg_config;
} __packed;

struct device_clk_info {
	struct pll_config pll_config_g;
	struct switch_clk switch_clk_g;
} __packed;

struct bootup_params {
	__le16 magic_number;
#define LOADED_TOKEN  0x5AA5   /* Bootup params are installed by host
				* or OTP/FLASH (Bootloader)
				*/
#define ROM_TOKEN     0x55AA   /* Bootup params are taken from ROM
				* itself in MCU mode.
				*/
	__le16 crystal_good_time;
	__le32 valid;
#define CRYSTAL_GOOD_TIME                BIT(0)
#define BOOTUP_MODE_INFO                 BIT(1)
#define DIGITAL_LOOP_BACK_PARAMS         BIT(2)
#define RTLS_TIMESTAMP_EN                BIT(3)
#define HOST_SPI_INTR_CFG                BIT(4)
#define WIFI_TAPLL_CONFIGS               BIT(5)
#define WIFI_PLL960_CONFIGS              BIT(6)
#define WIFI_AFEPLL_CONFIGS              BIT(7)
#define WIFI_SWITCH_CLK_CONFIGS          BIT(8)
#define BT_TAPLL_CONFIGS                 BIT(9)
#define BT_PLL960_CONFIGS                BIT(10)
#define BT_AFEPLL_CONFIGS                BIT(11)
#define BT_SWITCH_CLK_CONFIGS            BIT(12)
#define ZB_TAPLL_CONFIGS                 BIT(13)
#define ZB_PLL960_CONFIGS                BIT(14)
#define ZB_AFEPLL_CONFIGS                BIT(15)
#define ZB_SWITCH_CLK_CONFIGS            BIT(16)
#define BUCKBOOST_WAIT_INFO              BIT(17)
#define PMU_WAKEUP_SHUTDOWN_W            BIT(18)
#define WDT_PROG_VALUES                  BIT(19)
#define WDT_RESET_DELAY_VALUE            BIT(20)
#define DCDC_OPERATION_MODE_VALID        BIT(21)
#define PMU_SLP_CLKOUT_SEL               BIT(22)
#define SOC_RESET_WAIT_CNT               BIT(23)
	__le32 reserved_for_valids;
	__le16 bootup_mode_info;
#define BT_COEXIST                       BIT(0)
#define BOOTUP_MODE                     (BIT(2) | BIT(1))
#define CUR_DEV_MODE                    (bootup_params.bootup_mode_info >> 1)
	__le16 digital_loop_back_params;
	__le16 rtls_timestamp_en;
	__le16 host_spi_intr_cfg;
	struct device_clk_info device_clk_info[3];
	/* ulp buckboost wait time  */
	__le32 buckboost_wakeup_cnt;
	/* pmu wakeup wait time & WDT EN info */
	__le16 pmu_wakeup_wait;
	u8 shutdown_wait_time;
	/* Sleep clock source selection */
	u8 pmu_slp_clkout_sel;
	/* WDT programming values */
	__le32 wdt_prog_value;
	/* WDT soc reset delay */
	__le32 wdt_soc_rst_delay;
	/* dcdc modes configs */
	__le32 dcdc_operation_mode;
	__le32 soc_reset_wait_cnt;
	__le32 waiting_time_at_fresh_sleep;
	__le32 max_threshold_to_avoid_sleep;
	u8 beacon_resedue_alg_en;
} __packed;

#endif
