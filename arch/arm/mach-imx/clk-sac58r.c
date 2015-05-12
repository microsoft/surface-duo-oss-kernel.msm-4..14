/*
 * Copyright 2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/sac58r-clock.h>

#include "clk.h"

#define CCM_CCR					(ccm_base + 0x10C)
#define CCM_CSR					(ccm_base + 0x110)
#define CCM_CCSR				(ccm_base + 0x114)
#define CCM_CLPCR				(ccm_base + 0x118)
#define CCM_MUXED_FAST_OSC_CLK_REG		(ccm_base + 0x00)
#define CCM_MUXED_SLOW_OSC_CLK_REG		(ccm_base + 0x04)
#define CCM_SXOXC_IPG_SYNCED_REG		(ccm_base + 0x08)
#define CCM_A7_CLK_REG				(ccm_base + 0x0C)
#define CCM_QOS_DDR_ROOT_REG			(ccm_base + 0x10)
#define CCM_QOS301_CLK_REG			(ccm_base + 0x14)
#define CCM_DDR_400M_CLK_REG			(ccm_base + 0x18)
#define CCM_BUS2X_CLK_REG			(ccm_base + 0x1C)
#define CCM_BUS_CLK_REG				(ccm_base + 0x20)
#define CCM_PER_CLK_REG				(ccm_base + 0x24)
#define CCM_DAP_CLK_REG				(ccm_base + 0x28)
#define CCM_SNVS_CLK_REG			(ccm_base + 0x2C)
#define CCM_AUDIO0_PLL_DIV_CLK_REG		(ccm_base + 0x30)
#define CCM_AUDIO1_PLL_DIV_CLK_REG		(ccm_base + 0x34)
#define CCM_VIDEO_PLL_DIV_CLK_REG		(ccm_base + 0x38)
#define CCM_RUBY_CLK_REG			(ccm_base + 0x3C)
#define CCM_AXIQ_CLK_REG			(ccm_base + 0x40)
#define CCM_GPC_CLK_REG				(ccm_base + 0x44)
#define CCM_DCU_LDI_PIX_CLK_SRC_REG		(ccm_base + 0x48)
#define CCM_GCC_CLK2X_CLK_REG			(ccm_base + 0x4C)
#define CCM_ANACATUM_PROC_CLK_REG		(ccm_base + 0x50)
#define CCM_ALBACORE_CLK60_REG			(ccm_base + 0x54)
#define CCM_ALBACORE_CLK13_REG			(ccm_base + 0x58)
#define CCM_ALBACORE_CLK27_REG			(ccm_base + 0x5C)
#define CCM_FLEXTIMER0_FF_EXTCLK_REG		(ccm_base + 0x60)
#define CCM_FLEXTIMER0_FF_CLK_REG		(ccm_base + 0x64)
#define CCM_ENET_TIME_CLK_REG			(ccm_base + 0x68)
#define CCM_ENET_MII_CLK_REG			(ccm_base + 0x6C)
#define CCM_USDHC0_PERCLK_REG			(ccm_base + 0x70)
#define CCM_USDHC1_PERCLK_REG			(ccm_base + 0x74)
#define CCM_USDHC2_PERCLK_REG			(ccm_base + 0x78)
#define CCM_QSPI_4X_CLK_REG			(ccm_base + 0x7C)
#define CCM_NFC_FLASH_CLK_DIV_REG		(ccm_base + 0x80)
#define CCM_AUD_CLK_0_REG			(ccm_base + 0x84)
#define CCM_AUD_CLK_1_REG			(ccm_base + 0x88)
#define CCM_SAI0_MCLK_REG			(ccm_base + 0x8C)
#define CCM_SAI1_MCLK_REG			(ccm_base + 0x90)
#define CCM_SAI2_MCLK_REG			(ccm_base + 0x94)
#define CCM_SAI3_MCLK_REG			(ccm_base + 0x98)
#define CCM_ESAI0_MCLK_REG			(ccm_base + 0x9C)
#define CCM_ESAI1_MCLK_REG			(ccm_base + 0xA0)
#define CCM_SPDIF_TX_CLK_REG			(ccm_base + 0xA4)
#define CCM_ASRC0_MUX_CLK_REG			(ccm_base + 0xB0)
#define CCM_ASRC1_MUX_CLK_REG			(ccm_base + 0xB4)
#define CCM_CODEC_SAI_BCLK0_REG			(ccm_base + 0xC4)
#define CCM_CODEC_SAI_BCLK1_REG			(ccm_base + 0xC8)
#define CCM_CODEC_MCLK_REG			(ccm_base + 0xD4)
#define CCM_VIU_CLK_REG				(ccm_base + 0xD8)
#define CCM_VPU_CLK_REG				(ccm_base + 0xDC)
#define CCM_AUDIO_CODEC_IPG_CLK_REG		(ccm_base + 0xF0)
#define CCM_MIPI_24M_REF_CLK_REG		(ccm_base + 0xF4)
#define CCM_MIPI_ESC_MODE_CLK_REG		(ccm_base + 0xF8)
#define CCM_VIU_INTF_CLK_REG			(ccm_base + 0xFC)
#define CCM_SPDIF1_TX_CLK_REG			(ccm_base + 0x100)
#define CCM_ADC_CONV_CLK_REG			(ccm_base + 0x104)
#define CCM_XRDC_IPG_CLK_REG			(ccm_base + 0x108)

#define PLL1_CTRL_REG				(anatop_base + 0x270)
#define PLL2_CTRL_REG				(anatop_base + 0x30)
#define PLL3_CTRL_REG				(anatop_base + 0x10)
#define PLL5_CTRL_REG				(anatop_base + 0xE0)
#define PLL7_CTRL_REG				(anatop_base + 0x20)
#define PFD_PLL2_REG				(anatop_base + 0x100)
#define PFD_PLL3_REG				(anatop_base + 0x0F0)

#define OFFPF_PCTLn(n)				((n) * 2)

#define GPC_AIPS0_ONPF_PCTL0		(gpc_base + 0x80)
#define GPC_AIPS0_OFFPF_PCTL0		(gpc_base + 0x88)
#define GPC_AIPS0_OFFPF_PCTL1		(gpc_base + 0x8C)
#define GPC_AIPS0_OFFPF_PCTL2		(gpc_base + 0x90)
#define GPC_AIPS0_OFFPF_PCTL3		(gpc_base + 0x94)
#define GPC_AIPS0_OFFPF_PCTL4		(gpc_base + 0x98)

#define GPC_AIPS1_ONPF_PCTL0		(gpc_base + 0x100)
#define GPC_AIPS1_OFFPF_PCTL0		(gpc_base + 0x108)
#define GPC_AIPS1_OFFPF_PCTL1		(gpc_base + 0x10C)
#define GPC_AIPS1_OFFPF_PCTL2		(gpc_base + 0x110)
#define GPC_AIPS1_OFFPF_PCTL3		(gpc_base + 0x114)
#define GPC_AIPS1_OFFPF_PCTL4		(gpc_base + 0x118)

#define GPC_AIPS2_OFFPF_PCTL0		(gpc_base + 0x188)
#define GPC_AIPS2_OFFPF_PCTL1		(gpc_base + 0x18C)
#define GPC_AIPS2_OFFPF_PCTL2		(gpc_base + 0x190)
#define GPC_AIPS2_OFFPF_PCTL3		(gpc_base + 0x194)
#define GPC_AIPS2_OFFPF_PCTL4		(gpc_base + 0x198)
#define GPC_AIPS2_OFFPF_PCTL5		(gpc_base + 0x19C)

static void __iomem *anatop_base;
static void __iomem *ccm_base;
static void __iomem *gpc_base;


/* sources for multiplexer clocks, this is used multiple times */
static const char const *fast_sels[]	= { "firc", "fxosc", };
static const char const *slow_sels[]	= { "sirc_32k", "sxosc", };
#if 0 /* Unused so disable to remove warnings */
static const char const *pll1_sels[]	= { "pll1_main", "pll1_pfd1", "pll1_pfd2", "pll1_pfd3", "pll1_pfd4", };
static const char const *pll2_sels[]	= { "pll2_main", "pll2_pfd1", "pll2_pfd2", "pll2_pfd3", "pll2_pfd4", };
#endif
static const char const *a7_sels[] 	= { "dummy", "firc", "fxosc", "pll1_main", "pll2_main", "pll2_pfd1", "pll2_pfd2", "pll2_pfd3", "pll2_pfd4", 
						"pll3_main", "pll3_pfd1", "pll1_div2", };

static const char const *audio0_pll_sels[] = { "pll4_main", "pll4_div2", };
static const char const *audio1_pll_sels[] = { "pll8_main", "pll4_div2", };
static const char const *video_pll_sels[] = { "pll6_main", "pll6_div2", };
static const char const *dcu_ldi_sels[]	= { "pll2_pfd1", "pll2_pfd4", 
					    "pll3_main", "pll3_pfd2", "pll3_pfd4", 
					    "pll4_main", "pll8_main", "pll6_main",
					    "qos301", };
static const char const *gcc_sels[]	= { "dummy", "firc", "pll1_main",
						"pll4_main", "pll8_main", "pll6_main",
					    "pll1_div2", };
static const char const *anacatum_sels[] = { "pll3_main",
					     "pll4_main", "pll8_main", "pll6_main",
					    "pll1_div2", };
static const char const *usdhc_sels[]	= { "pll1_main", "pll2_main", "pll2_pfd2", "pll3_main", "pll3_pfd1", "pll1_div2", "qos301", };
static const char const *qspi_sels[]	= { "pll2_pfd1", "pll2_pfd2", "pll2_pfd3", "pll3_pfd4", "per_clk", };
static const char const *nfc_sels[]	= { "pll2_pfd1", "pll2_pfd2", "pll2_pfd3", "pll2_pfd4", "pll3_main", "pll3_pfd3", "pll3_pfd4", "qos301", };

static const char const *aud_clk0_sels[] = { "dummy", "dummy", "dummy", };
static const char const *aud_clk1_sels[] = { "dummy", "dummy", "dummy", };

static const char const *sai_sels[]	= {"dummy", "aud_clk_0", "aud_clk_1", "audio0_pll_div", "audio1_pll_div", "video_pll_div", };
static const char const *esai_sels[] = { "dummy", "aud_clk_0", "aud_clk_1", "audio0_pll_div", "audio1_pll_div", "video_pll_div", "bus_clk", };
#if 0 /* Unused so disable to remove warnings */
static const char const *asrc_sels[]	= { "esai0_sckr", "esai0_hckr", "tun34_rx_bclk_div", };
static const char const *codec_sai_sels[] = { "dummy", "pll4_main", "pll8_main", "audio0_pll_div", "audio1_pll_div", "video_pll_div", };
#endif
static const char const *viu_sels[] = { "pll1_main", 
					"pll2_main", "pll2_pfd1", "pll2_pfd2", "pll2_pfd3", "pll2_pfd4",
					"pll3_main", "pll3_pfd1", "pll3_pfd2", "pll3_pfd3", "pll3_pfd4",
					"pll4_main", "pll8_main", "pll1_div2",
					"qos301", };

static const char const *mipi24m_sels[]	= { "fxosc", "ipp_tuner_mclk", };

static struct clk *clk[SAC58R_CLK_END];
static struct clk_onecell_data clk_data;



static void __init sac58r_clocks_init(struct device_node *ccm_node)
{
	struct device_node *np;

	clk[SAC58R_CLK_DUMMY] = imx_clk_fixed("dummy", 0);
	clk[SAC58R_CLK_SIRC_128K] = imx_clk_fixed("sirc_128k", 128000);
	clk[SAC58R_CLK_SIRC_32K] = imx_clk_fixed("sirc_32k", 32000);
	clk[SAC58R_CLK_FIRC] = imx_clk_fixed("firc", 24000000);

	clk[SAC58R_CLK_SXOSC] = imx_obtain_fixed_clock("sxosc", 0);
	clk[SAC58R_CLK_FXOSC] = imx_obtain_fixed_clock("fxosc", 0);

	clk[SAC58R_CLK_FXOSC_HALF] = imx_clk_fixed_factor("fxosc_half", "fxosc", 1, 2);

	np = of_find_compatible_node(NULL, NULL, "fsl,sac58r-anatop");
	anatop_base = of_iomap(np, 0);
	BUG_ON(!anatop_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,sac58r-gpc");
	gpc_base = of_iomap(np, 0);
	BUG_ON(!gpc_base);

	np = ccm_node;
	ccm_base = of_iomap(np, 0);
	BUG_ON(!ccm_base);

	clk[SAC58R_CLK_SLOW_CLK_SEL] = imx_clk_mux("slow_clk_sel",
		CCM_MUXED_SLOW_OSC_CLK_REG, 0, 1, slow_sels, ARRAY_SIZE(slow_sels));
	clk[SAC58R_CLK_FASK_CLK_SEL] = imx_clk_mux("fast_clk_sel",
		CCM_MUXED_FAST_OSC_CLK_REG, 0, 1, fast_sels, ARRAY_SIZE(fast_sels));

	/* PLL1: ARM_PLL/CORE_PLL */
	clk[SAC58R_CLK_PLL1_MAIN] = imx_clk_pllv3(IMX_PLLV3_SYS,
		"pll1_main", "fast_clk_sel", PLL1_CTRL_REG, 0x7f);
	clk[SAC58R_CLK_PLL1_MAIN_DIV2] = imx_clk_fixed_factor("pll1_div2", "pll1_main", 1, 2);

	/* PLL2: SYS_PLL */
	clk[SAC58R_CLK_PLL2_MAIN] = imx_clk_pllv3(IMX_PLLV3_GENERIC, "pll2_main", "fast_clk_sel", PLL2_CTRL_REG, 0x1);	
	clk[SAC58R_CLK_PLL2_PFD1] = imx_clk_pfd("pll2_pfd1", "pll2_main", PFD_PLL2_REG, 0);
	clk[SAC58R_CLK_PLL2_PFD2] = imx_clk_pfd("pll2_pfd2", "pll2_main", PFD_PLL2_REG, 1);
	clk[SAC58R_CLK_PLL2_PFD3] = imx_clk_pfd("pll2_pfd3", "pll2_main", PFD_PLL2_REG, 2);
	clk[SAC58R_CLK_PLL2_PFD4] = imx_clk_pfd("pll2_pfd4", "pll2_main", PFD_PLL2_REG, 3);

	/* PLL3: USB0_PLL */
	clk[SAC58R_CLK_PLL3_MAIN] = imx_clk_fixed_factor("pll3_main", "fast_clk_sel", 20, 1);
	clk[SAC58R_CLK_PLL3_PFD1] = imx_clk_pfd("pll3_pfd1", "pll3_main", PFD_PLL3_REG, 0);
	clk[SAC58R_CLK_PLL3_PFD2] = imx_clk_pfd("pll3_pfd2", "pll3_main", PFD_PLL3_REG, 1);
	clk[SAC58R_CLK_PLL3_PFD3] = imx_clk_pfd("pll3_pfd3", "pll3_main", PFD_PLL3_REG, 2);
	clk[SAC58R_CLK_PLL3_PFD4] = imx_clk_pfd("pll3_pfd4", "pll3_main", PFD_PLL3_REG, 3);

	/* pll4 (AUDIO0 PLL)  */
	clk[SAC58R_CLK_PLL4_MAIN] = imx_clk_fixed_factor("pll4_main", "fast_clk_sel", 25, 1);
	clk[SAC58R_CLK_PLL4_MAIN_DIV2] = imx_clk_fixed_factor("pll4_div2", "pll4_main", 1, 2);
	
	/* PLL5 (ENET_PLL)  fixed 500 Mhz */
	clk[SAC58R_CLK_PLL5_MAIN] = imx_clk_pllv3(IMX_PLLV3_ENET, "pll5_main", "fast_clk_sel", PLL5_CTRL_REG, 0x1);
	clk[SAC58R_CLK_ENET_50M] = imx_clk_fixed_factor("enet_50M", "pll5_main", 1, 10);
	
	/* PLL6 (VIDEO PLL): default 960Mhz */
	clk[SAC58R_CLK_PLL6_MAIN] = imx_clk_fixed_factor("pll6_main", "fast_clk_sel", 40, 1);
	clk[SAC58R_CLK_PLL6_MAIN_DIV2] = imx_clk_fixed_factor("pll6_div2", "pll6_main", 1, 2);
	
	/* PLL7 (USB1 PLL): default 480Mhz */
	clk[SAC58R_CLK_PLL7_MAIN] = imx_clk_fixed_factor("pll7_main", "fast_clk_sel", 20, 1);
	
	/* PLL8 (AUDIO1 PLL): default 1176 Mhz */
	clk[SAC58R_CLK_PLL8_MAIN] = imx_clk_fixed_factor("pll8_main", "fast_clk_sel", 20, 1);
	clk[SAC58R_CLK_PLL8_MAIN_DIV2] = imx_clk_fixed_factor("pll8_div2", "pll8_main", 1, 2);


	clk[SAC58R_CLK_A7_SEL] = imx_clk_mux("a7_clk_sel", CCM_A7_CLK_REG, 0, 4, a7_sels, 11);
	clk[SAC58R_CLK_A7_DIV] = imx_clk_divider("a7_clk", "a7_clk_sel", CCM_A7_CLK_REG, 16, 3);

	clk[SAC58R_CLK_QOS_DDR_SEL] = imx_clk_mux("qos_ddr_root_sel", CCM_QOS_DDR_ROOT_REG, 0, 4, a7_sels, 11);
	clk[SAC58R_CLK_QOS_DDR_DIV] = imx_clk_divider("qos_ddr_root", "qos_ddr_root_sel", CCM_QOS_DDR_ROOT_REG, 16, 3);
	
	clk[SAC58R_CLK_QOS301_DIV] = imx_clk_divider("qos301", "qos_ddr_root", CCM_QOS301_CLK_REG, 16, 3);

	clk[SAC58R_CLK_SYS_BUS2X] = imx_clk_divider("sys_bus2x", "qos301", CCM_BUS2X_CLK_REG, 16, 3);
	clk[SAC58R_CLK_SYS_BUS] = imx_clk_divider("sys_bus", "qos301", CCM_BUS_CLK_REG, 16, 3);
	
	clk[SAC58R_CLK_PER] = imx_clk_divider("per", "sys_bus", CCM_PER_CLK_REG, 16, 3);

	clk[SAC58R_CLK_PIT] = imx_clk_gate2("pit", "sys_bus", GPC_AIPS0_OFFPF_PCTL3, 16);
	clk[SAC58R_CLK_UART0] = imx_clk_gate2("uart0", "per", GPC_AIPS1_OFFPF_PCTL1, 22);

	clk[SAC58R_CLK_PORTA] = imx_clk_gate2("port_a", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 20);
	clk[SAC58R_CLK_PORTB] = imx_clk_gate2("port_b", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 22);
	clk[SAC58R_CLK_PORTC] = imx_clk_gate2("port_c", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 24);
	clk[SAC58R_CLK_PORTD] = imx_clk_gate2("port_d", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 26);
	clk[SAC58R_CLK_PORTE] = imx_clk_gate2("port_e", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 28);
	clk[SAC58R_CLK_PORTF] = imx_clk_gate2("port_f", "sys_bus", GPC_AIPS0_OFFPF_PCTL0, 30);
	clk[SAC58R_CLK_PORTG] = imx_clk_gate2("port_g", "sys_bus", GPC_AIPS0_OFFPF_PCTL1, 0);
	clk[SAC58R_CLK_PORTH] = imx_clk_gate2("port_h", "sys_bus", GPC_AIPS0_OFFPF_PCTL1, 2);
	clk[SAC58R_CLK_PORTJ] = imx_clk_gate2("port_j", "sys_bus", GPC_AIPS0_OFFPF_PCTL1, 6);
	clk[SAC58R_CLK_PORTK] = imx_clk_gate2("port_k", "sys_bus", GPC_AIPS0_OFFPF_PCTL1, 8);
	clk[SAC58R_CLK_PORTL] = imx_clk_gate2("port_l", "sys_bus", GPC_AIPS0_OFFPF_PCTL1, 10);

	clk[SAC58R_CLK_FLEXCAN0] = imx_clk_gate2("flexcan0", "sys_bus", GPC_AIPS0_OFFPF_PCTL2, 0);
	clk[SAC58R_CLK_FLEXCAN1] = imx_clk_gate2("flexcan1", "sys_bus", GPC_AIPS0_OFFPF_PCTL2, 2);
	clk[SAC58R_CLK_FLEXCAN2] = imx_clk_gate2("flexcan2", "sys_bus", GPC_AIPS0_OFFPF_PCTL2, 4);


	clk[SAC58R_CLK_DSPI0] = imx_clk_gate2("spi0", "per", GPC_AIPS0_OFFPF_PCTL2, 10);
	clk[SAC58R_CLK_DSPI1] = imx_clk_gate2("spi1", "per", GPC_AIPS1_OFFPF_PCTL1, 16);
	clk[SAC58R_CLK_DMACH_MUX0] = imx_clk_gate2("dmamux0", "per", GPC_AIPS0_OFFPF_PCTL2, 24);
	clk[SAC58R_CLK_DMACH_MUX1] = imx_clk_gate2("dmamux1", "per", GPC_AIPS0_OFFPF_PCTL2, 26);

	clk[SAC58R_CLK_UART1] = imx_clk_gate2("uart1", "per", GPC_AIPS1_OFFPF_PCTL1, 24);
	clk[SAC58R_CLK_UART2] = imx_clk_gate2("uart2", "per", GPC_AIPS0_OFFPF_PCTL2, 16);
	clk[SAC58R_CLK_UART3] = imx_clk_gate2("uart3", "per", GPC_AIPS2_OFFPF_PCTL0, 0);
	clk[SAC58R_CLK_UART4] = imx_clk_gate2("uart4", "per", GPC_AIPS2_OFFPF_PCTL0, 2);
	clk[SAC58R_CLK_UART5] = imx_clk_gate2("uart5", "per", GPC_AIPS2_OFFPF_PCTL0, 4);


	clk[SAC58R_CLK_I2C0] = imx_clk_gate2("i2c0", "per", GPC_AIPS1_OFFPF_PCTL1, 30);
	clk[SAC58R_CLK_I2C1] = imx_clk_gate2("i2c1", "per", GPC_AIPS1_OFFPF_PCTL2, 0);
	clk[SAC58R_CLK_I2C2] = imx_clk_gate2("i2c2", "per", GPC_AIPS2_OFFPF_PCTL0, 8);
	clk[SAC58R_CLK_I2C3] = imx_clk_gate2("i2c3", "per", GPC_AIPS2_OFFPF_PCTL0, 10);

	clk[SAC58R_CLK_AUDIO0_PLL_SEL] = imx_clk_mux("audio0_pll_sel", CCM_AUDIO0_PLL_DIV_CLK_REG, 0, 1, audio0_pll_sels, 2);
	clk[SAC58R_CLK_AUDIO0_PLL_EN] = imx_clk_gate("audio0_pll_en", "audio0_pll_sel", CCM_AUDIO0_PLL_DIV_CLK_REG, 31);
	clk[SAC58R_CLK_AUDIO0_PLL_DIV] = imx_clk_divider("audio0_pll_div", "audio0_pll_en", CCM_AUDIO0_PLL_DIV_CLK_REG, 16, 3); 

	clk[SAC58R_CLK_AUDIO1_PLL_SEL] = imx_clk_mux("audio1_pll_sel", CCM_AUDIO1_PLL_DIV_CLK_REG, 0, 1, audio1_pll_sels, 2);
	clk[SAC58R_CLK_AUDIO1_PLL_EN] = imx_clk_gate("audio1_pll_en", "audio1_pll_sel", CCM_AUDIO1_PLL_DIV_CLK_REG, 31);
	clk[SAC58R_CLK_AUDIO1_PLL_DIV] = imx_clk_divider("audio1_pll_div", "audio1_pll_en", CCM_AUDIO1_PLL_DIV_CLK_REG, 16, 3);

	clk[SAC58R_CLK_VIDEO_PLL_SEL] = imx_clk_mux("video_pll_sel", CCM_VIDEO_PLL_DIV_CLK_REG, 0, 1, video_pll_sels, 2);
	clk[SAC58R_CLK_VIDEO_PLL_EN] = imx_clk_gate("video_pll_en", "video_pll_sel", CCM_VIDEO_PLL_DIV_CLK_REG, 31);
	clk[SAC58R_CLK_VIDEO_PLL_DIV] = imx_clk_divider("video_pll_div", "video_pll_en", CCM_VIDEO_PLL_DIV_CLK_REG, 16, 3);

	clk[SAC58R_CLK_DCU_LDI_SEL] = imx_clk_mux("dcu_ldi_sel", CCM_DCU_LDI_PIX_CLK_SRC_REG, 0, 4, dcu_ldi_sels, 9);
	clk[SAC58R_CLK_DCU_LDI_EN] = imx_clk_gate("dcu_ldi_en", "dcu_ldi_sel", CCM_DCU_LDI_PIX_CLK_SRC_REG, 31);
	clk[SAC58R_CLK_DCU_LDI_DIV] = imx_clk_divider("dcu_ldi", "dcu_ldi_en", CCM_DCU_LDI_PIX_CLK_SRC_REG, 16, 3);

	clk[SAC58R_CLK_GCC_SEL] = imx_clk_mux("gcc_sel", CCM_GCC_CLK2X_CLK_REG, 0, 3, gcc_sels, 6);
	clk[SAC58R_CLK_GCC_EN] = imx_clk_gate("gcc_en", "gcc_sel", CCM_GCC_CLK2X_CLK_REG, 31);
	clk[SAC58R_CLK_GCC_DIV] = imx_clk_divider("gcc_div", "gcc_en", CCM_GCC_CLK2X_CLK_REG, 16, 3);

	/* Video ADC and Video decoder clocks */
	clk[SAC58R_CLK_ANACATUM_SEL] = imx_clk_mux("anac_sel", CCM_ANACATUM_PROC_CLK_REG, 0, 3, anacatum_sels, 6);
	clk[SAC58R_CLK_ANACATUM_EN] = imx_clk_gate("anac_en", "anac_sel", CCM_ANACATUM_PROC_CLK_REG, 31);
	clk[SAC58R_CLK_ANACATUM_DIV] = imx_clk_divider("anac_div", "anac_en", CCM_ANACATUM_PROC_CLK_REG, 16, 3);


	clk[SAC58R_CLK_USDHC0_SEL] = imx_clk_mux("usdhc0_sel", CCM_USDHC0_PERCLK_REG, 0, 3, usdhc_sels, 7);
	clk[SAC58R_CLK_USDHC0_EN] = imx_clk_gate("usdhc0_en", "usdhc0_sel", CCM_USDHC0_PERCLK_REG, 31);
	clk[SAC58R_CLK_USDHC0_DIV] = imx_clk_divider("usdhc0_div", "usdhc0_en", CCM_USDHC0_PERCLK_REG, 16, 3);
	clk[SAC58R_CLK_USDHC0] = imx_clk_gate2("usdhc0", "usdhc0_div", GPC_AIPS2_OFFPF_PCTL4, 24);

	clk[SAC58R_CLK_USDHC1_SEL] = imx_clk_mux("usdhc1_sel", CCM_USDHC1_PERCLK_REG, 0, 3, usdhc_sels, 7);
	clk[SAC58R_CLK_USDHC1_EN] = imx_clk_gate("usdhc1_en", "usdhc1_sel", CCM_USDHC1_PERCLK_REG, 31);
	clk[SAC58R_CLK_USDHC1_DIV] = imx_clk_divider("usdhc1_div", "usdhc1_en", CCM_USDHC1_PERCLK_REG, 16, 3);
	clk[SAC58R_CLK_USDHC1] = imx_clk_gate2("usdhc1", "usdhc1_div", GPC_AIPS2_OFFPF_PCTL4, 26);

	clk[SAC58R_CLK_USDHC2_SEL] = imx_clk_mux("usdhc2_sel", CCM_USDHC2_PERCLK_REG, 0, 3, usdhc_sels, 7);
	clk[SAC58R_CLK_USDHC2_EN] = imx_clk_gate("usdhc2_en", "usdhc2_sel", CCM_USDHC2_PERCLK_REG, 31);
	clk[SAC58R_CLK_USDHC2_DIV] = imx_clk_divider("usdhc2_div", "usdhc2_en", CCM_USDHC2_PERCLK_REG, 16, 3);
	clk[SAC58R_CLK_USDHC2] = imx_clk_gate2("usdhc2", "usdhc2_div", GPC_AIPS2_OFFPF_PCTL4, 28);

	clk[SAC58R_CLK_QSPI_SEL] = imx_clk_mux("qspi_sel", CCM_QSPI_4X_CLK_REG, 0, 3, qspi_sels, 5);
	clk[SAC58R_CLK_QSPI_EN] = imx_clk_gate("qspi_en", "qspi_sel", CCM_QSPI_4X_CLK_REG, 31);
	clk[SAC58R_CLK_QSPI_DIV] = imx_clk_divider("qspi_div", "qspi_en", CCM_QSPI_4X_CLK_REG, 16, 3);
	clk[SAC58R_CLK_QSPI] = imx_clk_gate2("qspi", "qspi_div", GPC_AIPS2_OFFPF_PCTL4, 20);

	clk[SAC58R_CLK_NFC_SEL] = imx_clk_mux("nfc_sel", CCM_NFC_FLASH_CLK_DIV_REG, 0, 3, nfc_sels, 8);
	clk[SAC58R_CLK_NFC_EN] = imx_clk_gate("nfc_en", "nfc_sel", CCM_NFC_FLASH_CLK_DIV_REG, 31);
	clk[SAC58R_CLK_NFC_DIV] = imx_clk_divider("nfc_div", "nfc_en", CCM_NFC_FLASH_CLK_DIV_REG, 16, 3);

	clk[SAC58R_CLK_AUD_CLK0_SEL] = imx_clk_mux("aud_clk_0", CCM_AUD_CLK_0_REG, 0, 5, aud_clk0_sels, 25);
	clk[SAC58R_CLK_AUD_CLK1_SEL] = imx_clk_mux("aud_clk_1", CCM_AUD_CLK_1_REG, 0, 3, aud_clk1_sels, 23);


	clk[SAC58R_CLK_SAI0_SEL] = imx_clk_mux("sai0_sel", CCM_SAI0_MCLK_REG, 0, 3, sai_sels, 5);
	clk[SAC58R_CLK_SAI0] = imx_clk_gate2("sai0", "sai0_sel", GPC_AIPS2_OFFPF_PCTL0, 18);
	clk[SAC58R_CLK_SAI1_SEL] = imx_clk_mux("sai1_sel", CCM_SAI1_MCLK_REG, 0, 3, sai_sels, 5);
	clk[SAC58R_CLK_SAI1] = imx_clk_gate2("sai1", "sai1_sel", GPC_AIPS2_OFFPF_PCTL0, 20);
	clk[SAC58R_CLK_SAI2_SEL] = imx_clk_mux("sai2_sel", CCM_SAI2_MCLK_REG, 0, 3, sai_sels, 5);
	clk[SAC58R_CLK_SAI2] = imx_clk_gate2("sai2", "sai2_sel", GPC_AIPS2_OFFPF_PCTL0, 22);
	clk[SAC58R_CLK_SAI3_SEL] = imx_clk_mux("sai3_sel", CCM_SAI3_MCLK_REG, 0, 3, sai_sels, 5);
	clk[SAC58R_CLK_SAI3] = imx_clk_gate2("sai3", "sai3_sel", GPC_AIPS2_OFFPF_PCTL0, 24);

	/* SAI[4..11] clock gating */
	clk[SAC58R_CLK_SAI4] = imx_clk_gate2("sai4", "sys_bus", GPC_AIPS1_OFFPF_PCTL0, 12);
	clk[SAC58R_CLK_SAI5] = imx_clk_gate2("sai5", "sys_bus", GPC_AIPS1_OFFPF_PCTL0, 14);
	clk[SAC58R_CLK_SAI6] = imx_clk_gate2("sai6", "sys_bus", GPC_AIPS1_OFFPF_PCTL0, 24);
	clk[SAC58R_CLK_SAI7] = imx_clk_gate2("sai7", "sys_bus", GPC_AIPS1_OFFPF_PCTL0, 26);

	clk[SAC58R_CLK_SAI8]  = imx_clk_gate2("sai8",  "sys_bus", GPC_AIPS1_OFFPF_PCTL4, 16);
	clk[SAC58R_CLK_SAI9]  = imx_clk_gate2("sai9",  "sys_bus", GPC_AIPS1_OFFPF_PCTL4, 18);
	clk[SAC58R_CLK_SAI10] = imx_clk_gate2("sai10", "sys_bus", GPC_AIPS1_OFFPF_PCTL4, 20);
	clk[SAC58R_CLK_SAI11] = imx_clk_gate2("sai11", "sys_bus", GPC_AIPS1_OFFPF_PCTL4, 22);

	/* Audio Codec clock gating. Here we define 4 different clocks, even though only one
		is actually used. It is because we need to enable 4 differents slots in AIPS to enable
		the module */
	clk[SAC58R_CLK_AUD_A]  = imx_clk_gate2("aud_a",  "sys_bus", GPC_AIPS1_OFFPF_PCTL0, 0);
	clk[SAC58R_CLK_AUD_B]  = imx_clk_gate2("aud_b",  "aud_a", GPC_AIPS1_OFFPF_PCTL0, 2);
	clk[SAC58R_CLK_AUD_C]  = imx_clk_gate2("aud_c",  "aud_b", GPC_AIPS1_OFFPF_PCTL0, 4);
	clk[SAC58R_CLK_AUD_ADC_DAC]  = imx_clk_gate2("aud_adc_dac",  "aud_c", GPC_AIPS1_OFFPF_PCTL0, 6);

	clk[SAC58R_CLK_ESAI0_SEL] = imx_clk_mux("esai0_sel", CCM_ESAI0_MCLK_REG, 0, 3, esai_sels, 6);
	clk[SAC58R_CLK_ESAI0] = imx_clk_gate2("esai0", "esai0_sel", GPC_AIPS1_OFFPF_PCTL1, 6);
	clk[SAC58R_CLK_ESAI1_SEL] = imx_clk_mux("esai1_sel", CCM_ESAI1_MCLK_REG, 0, 3, esai_sels, 6);
	clk[SAC58R_CLK_ESAI1] = imx_clk_gate2("esai1", "esai1_sel", GPC_AIPS2_OFFPF_PCTL0, 14);

	clk[SAC58R_CLK_SAI_BCLK0_SEL] = imx_clk_mux("sai_bclk0_sel", CCM_CODEC_SAI_BCLK0_REG, 0, 3, esai_sels, 6);
	clk[SAC58R_CLK_SAI_BCLK0_EN] = imx_clk_gate("sai_bclk0_en", "sai_bclk0_sel", CCM_CODEC_SAI_BCLK0_REG, 31);
	clk[SAC58R_CLK_SAI_BCLK0_DIV] = imx_clk_divider("sai_bclk0_div", "sai_bclk0_en", CCM_CODEC_SAI_BCLK0_REG, 16, 3);

	clk[SAC58R_CLK_SAI_BCLK1_SEL] = imx_clk_mux("sai_bclk1_sel", CCM_CODEC_SAI_BCLK1_REG, 0, 3, esai_sels, 6);
	clk[SAC58R_CLK_SAI_BCLK1_EN] = imx_clk_gate("sai_bclk1_en", "sai_bclk1_sel", CCM_CODEC_SAI_BCLK1_REG, 31);
	clk[SAC58R_CLK_SAI_BCLK1_DIV] = imx_clk_divider("sai_bclk1_div", "sai_bclk1_en", CCM_CODEC_SAI_BCLK1_REG, 16, 3);

	clk[SAC58R_CLK_VIU_SEL] = imx_clk_mux("viu_sel", CCM_VIU_CLK_REG, 0, 4, viu_sels, 16);
	clk[SAC58R_CLK_VIU_EN] = imx_clk_gate("viu_en", "viu_sel", CCM_VIU_CLK_REG, 31);
	clk[SAC58R_CLK_VIU_DIV] = imx_clk_divider("viu_div", "viu_en", CCM_VIU_CLK_REG, 16, 3);
	clk[SAC58R_CLK_VIU] = imx_clk_gate2("viu", "viu_div", GPC_AIPS2_OFFPF_PCTL1, 16);

	/* VPU clock input is system bus clock */
	clk[SAC58R_CLK_VPU_DIV] = imx_clk_divider("vpu_div", "sys_bus", CCM_VPU_CLK_REG, 16, 3);
	clk[SAC58R_CLK_VPU] = imx_clk_gate2("vpu", "vpu_div", GPC_AIPS2_OFFPF_PCTL3, 16);

	clk[SAC58R_CLK_MIPI_24M_SEL] = imx_clk_mux("mipi24m_sel", CCM_MIPI_24M_REF_CLK_REG, 0, 2, mipi24m_sels, 2);

	clk[SAC58R_CLK_MIPI_ESC_EN] = imx_clk_gate("mipi_esc_en", "sys_bus", CCM_MIPI_ESC_MODE_CLK_REG, 31);
	clk[SAC58R_CLK_MIPI_ESC] = imx_clk_fixed_factor("mipi_esc", "mipi_esc_en", 1, 2); /* BUS clock divided by 2 */

	clk[SAC58R_CLK_VIU_INTF_EN] = imx_clk_gate("viu_intf_en", "viu_div", CCM_VIU_INTF_CLK_REG, 31);
	clk[SAC58R_CLK_VIU_INTF_DIV] = imx_clk_divider("viu_intf_div", "viu_intf_en", CCM_VIU_INTF_CLK_REG, 16, 3);

	clk[SAC58R_CLK_ENET] = imx_clk_gate2("enet", "sys_bus", GPC_AIPS2_OFFPF_PCTL4, 0);

	clk[SAC58R_CLK_USBC0] = imx_clk_gate2("usbc0", "pll3_main", GPC_AIPS2_OFFPF_PCTL4, 6);
	clk[SAC58R_CLK_USBC1] = imx_clk_gate2("usbc1", "pll7_main", GPC_AIPS2_OFFPF_PCTL4, 8);

	clk[SAC58R_CLK_USBC2] = imx_clk_gate2("usbc2", "sys_bus", GPC_AIPS2_OFFPF_PCTL4, 10);

	clk[SAC58R_CLK_USBPHY0] = imx_clk_gate("usbphy1", "pll3_main",	PLL3_CTRL_REG, 6);
	clk[SAC58R_CLK_USBPHY1] = imx_clk_gate("usbphy2", "pll7_main",	PLL7_CTRL_REG, 6);

	clk[SAC58R_CLK_SWT0] = imx_clk_gate2("swt0", "sirc_128k", GPC_AIPS0_ONPF_PCTL0, 6);
	clk[SAC58R_CLK_SWT1] = imx_clk_gate2("swt1", "sirc_128k", GPC_AIPS1_ONPF_PCTL0, 2);
	clk[SAC58R_CLK_SWT2] = imx_clk_gate2("swt2", "sirc_128k", GPC_AIPS1_ONPF_PCTL0, 6);

	/* When SWT modules are disabled, it is not possible to reset the platform.
		Force them to be used, so that they do not get gated by clock framework */
	clk_prepare_enable(clk[SAC58R_CLK_SWT0]);
	clk_prepare_enable(clk[SAC58R_CLK_SWT1]);
	clk_prepare_enable(clk[SAC58R_CLK_SWT2]);

	clk[SAC58R_CLK_SNVS_WDOG] = imx_clk_gate2("snvs_wdog", "per", GPC_AIPS0_OFFPF_PCTL1, 30);

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(sac58r, "fsl,sac58r-ccm", sac58r_clocks_init);
