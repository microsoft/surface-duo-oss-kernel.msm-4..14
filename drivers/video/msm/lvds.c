/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "msm_fb.h"
#include "mdp4.h"

#define LVDS_CH2_CLK_LANE_EN	BIT(17)
#define LVDS_CH1_CLK_LANE_EN	BIT(16)
#define LVDS_CH2_DATA_LANE3_EN	BIT(15)
#define LVDS_CH2_DATA_LANE2_EN	BIT(14)
#define LVDS_CH2_DATA_LANE1_EN	BIT(13)
#define LVDS_CH2_DATA_LANE0_EN	BIT(12)
#define LVDS_CH1_DATA_LANE3_EN	BIT(11)
#define LVDS_CH1_DATA_LANE2_EN	BIT(10)
#define LVDS_CH1_DATA_LANE1_EN	BIT(9)
#define LVDS_CH1_DATA_LANE0_EN	BIT(8)
#define LVDS_EN			BIT(7)
#define LVDS_CH2_RES_BIT		BIT(6)
#define LVDS_CH1_RES_BIT		BIT(5)
#define LVDS_CH_SWAP			BIT(4)
#define LVDS_RGB_OUT			BIT(3)
#define LVDS_MODE_SEL		BIT(2)

/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_3_TO_0 */
#define LVDS_D2_BIT_0_SEL (0x1A <<  0)	/* Bits  4: 0 - 0x1A: DE */
#define LVDS_D2_BIT_1_SEL (0x19 <<  8)	/* Bits 12: 8 - 0x19: VS */
#define LVDS_D2_BIT_2_SEL (0x18 << 16)	/* Bits 20:16 - 0x18: HS */
#define LVDS_D2_BIT_3_SEL (0x17 << 24)	/* Bits 28:24 - 0x17: B7 */
/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_6_TO_4 */
#define LVDS_D2_BIT_4_SEL (0x16 <<  0)	/* Bits  4: 0 - 0x16: B6 */
#define LVDS_D2_BIT_5_SEL (0x15 <<  8)	/* Bits 12: 8 - 0x15: B5 */
#define LVDS_D2_BIT_6_SEL (0x14 << 16)	/* Bits 20:16 - 0x14: B4 */
/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_3_TO_0 */
#define LVDS_D1_BIT_0_SEL (0x13 <<  0)	/* Bits  4: 0 - 0x13: B3 */
#define LVDS_D1_BIT_1_SEL (0x12 <<  8)	/* Bits 12: 8 - 0x12: B2 */
#define LVDS_D1_BIT_2_SEL (0x0F << 16)	/* Bits 20:16 - 0x0F: G7 */
#define LVDS_D1_BIT_3_SEL (0x0E << 24)	/* Bits 28:24 - 0x0E: G6 */
/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_6_TO_4 */
#define LVDS_D1_BIT_4_SEL (0x0D <<  0)	/* Bits  4: 0 - 0x0D: G5 */
#define LVDS_D1_BIT_5_SEL (0x0C <<  8)	/* Bits 12: 8 - 0x0C: G4 */
#define LVDS_D1_BIT_6_SEL (0x0B << 16)	/* Bits 20:16 - 0x0B: G3 */
/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_3_TO_0 */
#define LVDS_D0_BIT_0_SEL (0x0A <<  0)	/* Bits  4: 0 - 0x0A: G2 */
#define LVDS_D0_BIT_1_SEL (0x07 <<  8)	/* Bits 12: 8 - 0x07: R7 */
#define LVDS_D0_BIT_2_SEL (0x06 << 16)	/* Bits 20:16 - 0x06: R6 */
#define LVDS_D0_BIT_3_SEL (0x05 << 24)	/* Bits 28:24 - 0x05: R5 */
/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_6_TO_4 */
#define LVDS_D0_BIT_4_SEL (0x04 <<  0)	/* Bits  4: 0 - 0x04: R4 */
#define LVDS_D0_BIT_5_SEL (0x03 <<  8)	/* Bits 12: 8 - 0x03: R3 */
#define LVDS_D0_BIT_6_SEL (0x02 << 16)	/* Bits 20:16 - 0x02: R2 */

#define LVDS_PIXEL_MAP_PATTERN_2	2

static int lvds_probe(struct platform_device *pdev);
static int lvds_remove(struct platform_device *pdev);

static int lvds_off(struct platform_device *pdev);
static int lvds_on(struct platform_device *pdev);

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

static struct clk *lvds_clk;

static struct platform_driver lvds_driver = {
	.probe = lvds_probe,
	.remove = lvds_remove,
	.suspend = NULL,
	.resume = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "lvds",
		   },
};

static struct lcdc_platform_data *lvds_pdata;

static void lvds_init(struct msm_fb_data_type *mfd)
{
	unsigned int lvds_intf = 0, lvds_phy_cfg0 = 0;
	mdp_clk_ctrl(1);

	MDP_OUTP(MDP_BASE + 0xc2034, 0x33);
	usleep(1000);

	/* LVDS PHY PLL configuration */
	if (mfd->panel_info.clk_rate == 74250000) {
		MDP_OUTP(MDP_BASE + 0xc3000, 0x08);
		MDP_OUTP(MDP_BASE + 0xc3004, 0x4c);
		MDP_OUTP(MDP_BASE + 0xc3008, 0x30);
		MDP_OUTP(MDP_BASE + 0xc300c, 0xc3);
		MDP_OUTP(MDP_BASE + 0xc3014, 0x10);
		MDP_OUTP(MDP_BASE + 0xc3018, 0x04);
		MDP_OUTP(MDP_BASE + 0xc301c, 0x62);
		MDP_OUTP(MDP_BASE + 0xc3020, 0x41);
		MDP_OUTP(MDP_BASE + 0xc3024, 0x0d);
		MDP_OUTP(MDP_BASE + 0xc3028, 0x07);
		MDP_OUTP(MDP_BASE + 0xc302c, 0x00);
		MDP_OUTP(MDP_BASE + 0xc3030, 0x1c);
		MDP_OUTP(MDP_BASE + 0xc3034, 0x01);
		MDP_OUTP(MDP_BASE + 0xc3038, 0x00);
		MDP_OUTP(MDP_BASE + 0xc3040, 0xC0);
		MDP_OUTP(MDP_BASE + 0xc3044, 0x00);
		MDP_OUTP(MDP_BASE + 0xc3048, 0x30);
		MDP_OUTP(MDP_BASE + 0xc304c, 0x00);

		MDP_OUTP(MDP_BASE + 0xc3000, 0x11);
		MDP_OUTP(MDP_BASE + 0xc3064, 0x05);
		MDP_OUTP(MDP_BASE + 0xc3050, 0x20);
	} else {
		MDP_OUTP(MDP_BASE + 0xc3004, 0x8f);
		MDP_OUTP(MDP_BASE + 0xc3008, 0x30);
		MDP_OUTP(MDP_BASE + 0xc300c, 0xc6);
		MDP_OUTP(MDP_BASE + 0xc3014, 0x10);
		MDP_OUTP(MDP_BASE + 0xc3018, 0x07);
		MDP_OUTP(MDP_BASE + 0xc301c, 0x62);
		MDP_OUTP(MDP_BASE + 0xc3020, 0x41);
		MDP_OUTP(MDP_BASE + 0xc3024, 0x0d);
	}

	MDP_OUTP(MDP_BASE + 0xc3000, 0x01);
	/* Wait until LVDS PLL is locked and ready */
	while (!readl_relaxed(MDP_BASE + 0xc3080))
		cpu_relax();

	writel_relaxed(0x00, mmss_cc_base + 0x0264);
	writel_relaxed(0x00, mmss_cc_base + 0x0094);

	writel_relaxed(0x02, mmss_cc_base + 0x00E4);

	writel_relaxed((0x80 | readl_relaxed(mmss_cc_base + 0x00E4)),
	       mmss_cc_base + 0x00E4);
	usleep(1000);
	writel_relaxed((~0x80 & readl_relaxed(mmss_cc_base + 0x00E4)),
	       mmss_cc_base + 0x00E4);

	writel_relaxed(0x05, mmss_cc_base + 0x0094);
	writel_relaxed(0x02, mmss_cc_base + 0x0264);
	/* Wait until LVDS pixel clock output is enabled */
	mb();

	if (mfd->panel_info.bpp == 24) {
		if (lvds_pdata &&
		    lvds_pdata->lvds_pixel_remap &&
		    lvds_pdata->lvds_pixel_remap()) {
			if (lvds_pdata->lvds_pixel_remap() ==
				LVDS_PIXEL_MAP_PATTERN_2) {
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc2014, 0x070A1B1B);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2018, 0x00040506);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc201c, 0x12131B1B);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2020, 0x000B0C0D);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc2024, 0x191A1B1B);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2028, 0x00141518);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc202c, 0x171B1B1B);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2030, 0x000e0f16);
			} else {
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc2014, 0x05080001);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2018, 0x00020304);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc201c, 0x1011090a);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2020, 0x000b0c0d);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc2024, 0x191a1213);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2028, 0x00141518);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_3_TO_0 */
				MDP_OUTP(MDP_BASE +  0xc202c, 0x171b0607);
				/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_6_TO_4 */
				MDP_OUTP(MDP_BASE +  0xc2030, 0x000e0f16);
			}
		} else {
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_3_TO_0 */
			MDP_OUTP(MDP_BASE +  0xc2014, 0x03040508);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_6_TO_4 */
			MDP_OUTP(MDP_BASE +  0xc2018, 0x00000102);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_3_TO_0 */
			MDP_OUTP(MDP_BASE +  0xc201c, 0x0c0d1011);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_6_TO_4 */
			MDP_OUTP(MDP_BASE +  0xc2020, 0x00090a0b);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_3_TO_0 */
			MDP_OUTP(MDP_BASE +  0xc2024, 0x1518191a);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_6_TO_4 */
			MDP_OUTP(MDP_BASE +  0xc2028, 0x00121314);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_3_TO_0 */
			MDP_OUTP(MDP_BASE +  0xc202c, 0x0f16171b);
			/* MDP_LCDC_LVDS_MUX_CTL_FOR_D3_6_TO_4 */
			MDP_OUTP(MDP_BASE +  0xc2030, 0x0006070e);
		}
		if (mfd->panel_info.lvds.channel_mode ==
			LVDS_DUAL_CHANNEL_MODE) {
			lvds_intf = 0x0003ff80;
			lvds_phy_cfg0 = BIT(6) | BIT(7);
			if (mfd->panel_info.lvds.channel_swap)
				lvds_intf |= BIT(4);
		} else {
			lvds_intf = 0x00010f84;
			lvds_phy_cfg0 = BIT(6);
		}
	} else if (mfd->panel_info.bpp == 18) {

		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_3_TO_0 */
		MDP_OUTP(MDP_BASE +  0xc2014, LVDS_D0_BIT_3_SEL |
						  LVDS_D0_BIT_2_SEL |
						  LVDS_D0_BIT_1_SEL |
						  LVDS_D0_BIT_0_SEL);
		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D0_6_TO_4 */
		MDP_OUTP(MDP_BASE +  0xc2018, LVDS_D0_BIT_6_SEL |
						  LVDS_D0_BIT_5_SEL |
						  LVDS_D0_BIT_4_SEL);
		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_3_TO_0 */
		MDP_OUTP(MDP_BASE +  0xc201c, LVDS_D1_BIT_3_SEL |
						  LVDS_D1_BIT_2_SEL |
						  LVDS_D1_BIT_1_SEL |
						  LVDS_D1_BIT_0_SEL);
		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D1_6_TO_4 */
		MDP_OUTP(MDP_BASE +  0xc2020, LVDS_D1_BIT_6_SEL |
						  LVDS_D1_BIT_5_SEL |
						  LVDS_D1_BIT_4_SEL);
		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_3_TO_0 */
		MDP_OUTP(MDP_BASE +  0xc2024, LVDS_D2_BIT_3_SEL |
						  LVDS_D2_BIT_2_SEL |
						  LVDS_D2_BIT_1_SEL |
						  LVDS_D2_BIT_0_SEL);
		/* MDP_LCDC_LVDS_MUX_CTL_FOR_D2_6_TO_4 */
		MDP_OUTP(MDP_BASE +  0xc2028, LVDS_D2_BIT_6_SEL |
						  LVDS_D2_BIT_5_SEL |
						  LVDS_D2_BIT_4_SEL);

		if (mfd->panel_info.lvds.channel_mode ==
			LVDS_DUAL_CHANNEL_MODE) {
			lvds_intf = 0x00037788;
			lvds_phy_cfg0 = BIT(6) | BIT(7);
			if (mfd->panel_info.lvds.channel_swap)
				lvds_intf |= BIT(4);
		} else {
			lvds_intf = LVDS_CH1_CLK_LANE_EN |
				     LVDS_CH1_DATA_LANE2_EN |
				     LVDS_CH1_DATA_LANE1_EN |
				     LVDS_CH1_DATA_LANE0_EN |
				     LVDS_EN | LVDS_RGB_OUT |
				     LVDS_MODE_SEL;
			/**
			 * Shuts down all data lanes, LN0 to LN3,
			 * and clock lane, LNCK0, power which includes
			 * gating all digital clocks and disables analog
			 * block.(Active low)
			 */
			lvds_phy_cfg0 = BIT(6);
		}
	} else {
		BUG();
	}

	/* MDP_LVDSPHY_CFG0 */
	MDP_OUTP(MDP_BASE +  0xc3100, lvds_phy_cfg0);
	/* MDP_LCDC_LVDS_INTF_CTL */
	MDP_OUTP(MDP_BASE +  0xc2000, lvds_intf);
	MDP_OUTP(MDP_BASE +  0xc3108, 0x30);
	lvds_phy_cfg0 |= BIT(4);

	/* Wait until LVDS PHY registers are configured */
	mb();
	usleep(1);
	/* MDP_LVDSPHY_CFG0, enable serialization */
	MDP_OUTP(MDP_BASE +  0xc3100, lvds_phy_cfg0);
	mdp_clk_ctrl(0);
}

static int lvds_off(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	ret = panel_next_off(pdev);

	if (lvds_clk)
		clk_disable_unprepare(lvds_clk);

	mdp_clk_ctrl(1);
	MDP_OUTP(MDP_BASE +  0xc3100, 0x0);
	MDP_OUTP(MDP_BASE + 0xc3000, 0x0);
	usleep(10);
	mdp_clk_ctrl(0);

	if (lvds_pdata && lvds_pdata->lcdc_power_save)
		lvds_pdata->lcdc_power_save(0);

	if (lvds_pdata && lvds_pdata->lcdc_gpio_config)
		ret = lvds_pdata->lcdc_gpio_config(0);

	return ret;
}

static int lvds_on(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_data_type *mfd;
	unsigned long panel_pixclock_freq = 0;
	mfd = platform_get_drvdata(pdev);

	if (lvds_pdata && lvds_pdata->lcdc_get_clk)
		panel_pixclock_freq = lvds_pdata->lcdc_get_clk();

	if (!panel_pixclock_freq)
		panel_pixclock_freq = mfd->fbi->var.pixclock;
	mfd = platform_get_drvdata(pdev);

	if (lvds_clk) {
		mfd->fbi->var.pixclock = clk_round_rate(lvds_clk,
			mfd->fbi->var.pixclock);
		ret = clk_set_rate(lvds_clk, mfd->fbi->var.pixclock);
		if (ret) {
			pr_err("%s: Can't set lvds clock to rate %u\n",
				__func__, mfd->fbi->var.pixclock);
			goto out;
		}
		clk_prepare_enable(lvds_clk);
	}

	if (lvds_pdata && lvds_pdata->lcdc_power_save)
		lvds_pdata->lcdc_power_save(1);
	if (lvds_pdata && lvds_pdata->lcdc_gpio_config)
		ret = lvds_pdata->lcdc_gpio_config(1);

	lvds_init(mfd);
	ret = panel_next_on(pdev);

out:
	return ret;
}

static int lvds_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	struct platform_device *mdp_dev = NULL;
	struct msm_fb_panel_data *pdata = NULL;
	int rc;

	if (pdev->id == 0) {
		lvds_pdata = pdev->dev.platform_data;

		lvds_clk = clk_get(&pdev->dev, "lvds_clk");
		if (IS_ERR_OR_NULL(lvds_clk)) {
			pr_err("Couldnt find lvds_clk\n");
			lvds_clk = NULL;
		}
		return 0;
	}

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mdp_dev = platform_device_alloc("mdp", pdev->id);
	if (!mdp_dev)
		return -ENOMEM;

	/*
	 * link to the latest pdev
	 */
	mfd->pdev = mdp_dev;
	mfd->dest = DISPLAY_LCDC;

	/*
	 * alloc panel device data
	 */
	if (platform_device_add_data
	    (mdp_dev, pdev->dev.platform_data,
	     sizeof(struct msm_fb_panel_data))) {
		pr_err("lvds_probe: platform_device_add_data failed!\n");
		platform_device_put(mdp_dev);
		return -ENOMEM;
	}
	/*
	 * data chain
	 */
	pdata = (struct msm_fb_panel_data *)mdp_dev->dev.platform_data;
	pdata->on = lvds_on;
	pdata->off = lvds_off;
	pdata->next = pdev;

	/*
	 * get/set panel specific fb info
	 */
	mfd->panel_info = pdata->panel_info;

	if (mfd->index == 0)
		mfd->fb_imgType = MSMFB_DEFAULT_TYPE;
	else
		mfd->fb_imgType = MDP_RGB_565;

	fbi = mfd->fbi;
	if (lvds_clk) {
		fbi->var.pixclock = clk_round_rate(lvds_clk,
			mfd->panel_info.clk_rate);
	}

	fbi->var.left_margin = mfd->panel_info.lcdc.h_back_porch;
	fbi->var.right_margin = mfd->panel_info.lcdc.h_front_porch;
	fbi->var.upper_margin = mfd->panel_info.lcdc.v_back_porch;
	fbi->var.lower_margin = mfd->panel_info.lcdc.v_front_porch;
	fbi->var.hsync_len = mfd->panel_info.lcdc.h_pulse_width;
	fbi->var.vsync_len = mfd->panel_info.lcdc.v_pulse_width;

	/*
	 * set driver data
	 */
	platform_set_drvdata(mdp_dev, mfd);
	/*
	 * register in mdp driver
	 */
	rc = platform_device_add(mdp_dev);
	if (rc)
		goto lvds_probe_err;

	pdev_list[pdev_list_cnt++] = pdev;

	return 0;

lvds_probe_err:
	platform_device_put(mdp_dev);
	return rc;
}

static int lvds_remove(struct platform_device *pdev)
{
	return 0;
}

static int lvds_register_driver(void)
{
	return platform_driver_register(&lvds_driver);
}

static int __init lvds_driver_init(void)
{
	return lvds_register_driver();
}

module_init(lvds_driver_init);
