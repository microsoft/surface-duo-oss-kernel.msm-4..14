/*
 * SN65DSI83 DSI-to-LVDS bridge IC driver
 *
 * Copyright (C) 2017 CompuLab Ltd.
 * Author: Valentin Raevsky <valentin@compulab.co.il>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include "mipi_dsi.h"
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define DRV_NAME "sn65dsi83"

#define SN65DSI83_SOFT_RESET_REG		0x09

#define SN65DSI83_CLOCK_REG			0x0A
#define SN65DSI83_LVDS_CLK_RANGE_MASK		0xE
#define SN65DSI83_LVDS_CLK_RANGE_SHIFT		1
#define SN65DSI83_HS_CLK_SRC_MASK		0x1
#define SN65DSI83_HS_CLK_SRC_SHIFT		0

#define SN65DSI83_PLL_DIV_REG			0x0B
#define SN65DSI83_DSI_CLK_DIV_MASK		0xF8
#define SN65DSI83_DSI_CLK_DIV_SHIFT		3
#define SN65DSI83_REFCLK_MULTIPLIER_MASK	0x3
#define SN65DSI83_REFCLK_MULTIPLIER_SHIFT	0

#define SN65DSI83_PLL_EN_REG			0x0D

#define SN65DSI83_DSI_CFG_REG			0x10
#define SN65DSI83_CHA_DSI_LANES_MASK		0x18
#define SN65DSI83_CHA_DSI_LANES_SHIFT		3
#define SN65DSI83_SOT_ERR_TOL_DIS_MASK		0x1
#define SN65DSI83_SOT_ERR_TOL_DIS_SHIFT		0

#define SN65DSI83_DSI_EQ_REG			0x11

#define SN65DSI83_CHA_DSI_CLK_RANGE_REG		0x12
#define SN65DSI83_CHA_DSI_CLK_RANGE_MASK	0xFF
#define SN65DSI83_CHA_DSI_CLK_RANGE_SHIFT	0xFF

#define SN65DSI83_CHB_DSI_CLK_RNG_REG		0x13

#define SN65DSI83_LVDS_MODE_REG			0x18
#define SN65DSI83_DE_NEG_POLARITY_SHIFT		7
#define SN65DSI83_HS_NEG_POLARITY_SHIFT		6
#define SN65DSI83_VS_NEG_POLARITY_SHIFT		5
#define SN65DSI83_LVDS_LINK_CFG_SHIFT		4
#define SN65DSI83_CHA_24BPP_MODE_SHIFT		3
#define SN65DSI83_CHA_24BPP_FMT1_SHIFT		1

#define SN65DSI83_LVDS_SIGN_REG			0x19

#define SN65DSI83_LVDS_TERM_REG			0x1A
#define SN65DSI83_CHA_REVERSE_LVDS_MASK		0x20
#define SN65DSI83_CHA_REVERSE_LVDS_SHIFT	5
#define SN65DSI83_CHA_LVDS_TERM_MASK		0x1
#define SN65DSI83_CHA_LVDS_TERM_SHIFT		0

#define SN65DSI83_LVDS_CM_ADJ_REG		0x1B
#define SN65DSI83_CHA_LINE_LEN_LO_REG		0x20
#define SN65DSI83_CHA_LINE_LEN_HI_REG		0x21
#define SN65DSI83_CHB_LINE_LEN_LO_REG		0x22
#define SN65DSI83_CHB_LINE_LEN_HI_REG		0x23
#define SN65DSI83_CHA_VERT_LINES_LO_REG		0x24
#define SN65DSI83_CHA_VERT_LINES_HI_REG		0x25
#define SN65DSI83_CHB_VERT_LINES_LO_REG		0x26
#define SN65DSI83_CHB_VERT_LINES_HI_REG		0x27
#define SN65DSI83_CHA_SYNC_DELAY_LO_REG		0x28
#define SN65DSI83_CHA_SYNC_DELAY_HI_REG		0x29
#define SN65DSI83_CHB_SYNC_DELAY_LO_REG		0x2A
#define SN65DSI83_CHB_SYNC_DELAY_HI_REG		0x2B
#define SN65DSI83_CHA_HSYNC_WIDTH_LO_REG	0x2C
#define SN65DSI83_CHA_HSYNC_WIDTH_HI_REG	0x2D
#define SN65DSI83_CHB_HSYNC_WIDTH_LO_REG	0x2E
#define SN65DSI83_CHB_HSYNC_WIDTH_HI_REG	0x2F
#define SN65DSI83_CHA_VSYNC_WIDTH_LO_REG	0x30
#define SN65DSI83_CHA_VSYNC_WIDTH_HI_REG	0x31
#define SN65DSI83_CHB_VSYNC_WIDTH_LO_REG	0x32
#define SN65DSI83_CHB_VSYNC_WIDTH_HI_REG	0x33
#define SN65DSI83_CHA_HORZ_BACKPORCH_REG	0x34
#define SN65DSI83_CHB_HORZ_BACKPORCH_REG	0x35
#define SN65DSI83_CHA_VERT_BACKPORCH_REG	0x36
#define SN65DSI83_CHB_VERT_BACKPORCH_REG	0x37
#define SN65DSI83_CHA_HORZ_FRONTPORCH_REG	0x38
#define SN65DSI83_CHB_HORZ_FRONTPORCH_REG	0x39
#define SN65DSI83_CHA_VERT_FRONTPORCH_REG	0x3A
#define SN65DSI83_CHB_VERT_FRONTPORCH_REG	0x3B
#define SN65DSI83_TEST_PATTERN_REG		0x3C
#define SN65DSI83_3D_REG			0x3D
#define SN65DSI83_3E_REG			0x3E
#define SN65DSI83_CHA_TEST_PATTERN_SHIFT	4

#define SN65DSI83_CHA_ERR_REG			0xE5

/* vm structure shortcuts */
#define PIXCLK vm->pixelclock
#define HACTIVE vm->hactive
#define HFP vm->hfront_porch
#define HBP vm->hback_porch
#define HPW vm->hsync_len
#define VACTIVE vm->vactive
#define VFP vm->vfront_porch
#define VBP vm->vback_porch
#define VPW vm->vsync_len
#define FLAGS vm->flags
/* Calculation macros  */
#define HIGH(A) (((A) >> 8) & 0xFF)
#define LOW(A)  ((A)  & 0xFF)
#define ABS(X) ((X) < 0 ? (-1 * (X)) : (X))

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num	= 2,
	.max_phy_clk	= 800,
	.dpi_fmt	= MIPI_RGB888,
};

static struct panel_drv_data {
	struct i2c_client *client;
	struct videomode vm;
	struct fb_videomode fb_vm;
	int pixclk_src;
	u32 bpp;
	u32 format;
	struct gpio_desc *enable_gpio;
} sn65dsi_panel;

static void dump_fb_videomode(struct fb_videomode *m)
{
	pr_info("fb_videomode = %d %d %d %u %d %d %d %d %d %d 0x%x %d %d\n",
		m->refresh, m->xres, m->yres, m->pixclock, m->left_margin,
		m->right_margin, m->upper_margin, m->lower_margin,
		m->hsync_len, m->vsync_len, m->sync, m->vmode, m->flag);
}

static void dump_videomode(struct videomode *m)
{
	pr_info("videomode = %lu %d %d %d %d %d %d %d %d %d\n",
		m->pixelclock,
		m->hactive, m->hfront_porch, m->hback_porch, m->hsync_len,
		m->vactive, m->vfront_porch, m->vback_porch, m->vsync_len,
		m->flags);
}

static int sn65dsi_write(u8 reg, u8 val)
{
	struct i2c_client *client = sn65dsi_panel.client;
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret)
		dev_err(&client->dev, "failed to write at 0x%02x", reg);

	dev_dbg(&client->dev, "%s: write reg 0x%02x data 0x%02x", __func__,
		reg, val);

	return ret;
}

static int sn65dsi_read(u8 reg)
{
	struct i2c_client *client = sn65dsi_panel.client;
	int ret;

	dev_notice(&client->dev, "client 0x%p", client);
	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x", reg);
		return ret;
	}

	dev_dbg(&client->dev, "%s: read reg 0x%02x data 0x%02x", __func__,
		reg, ret);

	return ret;
}

static int sn65dsi_probe_of(void)
{
	struct device_node *np = sn65dsi_panel.client->dev.of_node;
	struct device_node *np_panel;
	struct device dev = sn65dsi_panel.client->dev;
	struct display_timing of_timing;
	struct gpio_desc *gpio;
	int ret;

	gpio = devm_gpiod_get(&dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		dev_err(&dev, "failed to parse enable gpio");
		return PTR_ERR(gpio);
	}
	sn65dsi_panel.enable_gpio = gpio;
	dev_err(&dev, "enable gpio found");
	dev_notice(&dev, "node %s", np->name);

	np_panel = of_get_child_by_name(np, "lvds_panel");
	if (!np_panel)
		dev_err(&dev, "failed to find lvds_panel node");

	memset(&sn65dsi_panel.vm, 0, sizeof(sn65dsi_panel.vm));
	memset(&sn65dsi_panel.fb_vm, 0, sizeof(sn65dsi_panel.fb_vm));
	ret = of_get_display_timing(np_panel, "display-timings", &of_timing);
	if (ret) {
		dev_err(&dev, "failed to find display-timings");
		return -ENODEV;
	}
	videomode_from_timing(&of_timing, &sn65dsi_panel.vm);
	dump_videomode(&sn65dsi_panel.vm);
	ret = fb_videomode_from_videomode(&sn65dsi_panel.vm,
					  &sn65dsi_panel.fb_vm);
	if (ret)
		return ret;

	/* Update MXC frame buffer flags */
	if (sn65dsi_panel.vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		sn65dsi_panel.fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
	dump_fb_videomode(&sn65dsi_panel.fb_vm);

	ret = of_property_read_u32(np_panel, "lvds-bpp", &sn65dsi_panel.bpp);
	if (ret)
		sn65dsi_panel.bpp = 24;

	ret = of_property_read_u32(np_panel, "lvds-format",
				&sn65dsi_panel.format);
	if (ret || (sn65dsi_panel.format != 1 && sn65dsi_panel.format != 2))
		sn65dsi_panel.format = 2;

	ret = of_property_read_u32(np_panel, "lvds-pixclk-src",
				   &sn65dsi_panel.pixclk_src);
	if (ret)
		sn65dsi_panel.pixclk_src = 0;

	return 0;
}

static int sn65dsi_power_on(void)
{

	gpiod_set_value_cansleep(sn65dsi_panel.enable_gpio, 1);
	/* Wait for 1ms for the internal voltage regulator to stabilize */
	msleep(1);

	return 0;
}

static void sn65dsi_power_off(void)
{
	gpiod_set_value_cansleep(sn65dsi_panel.enable_gpio, 0);
	/*
	 * The EN pin must be held low for at least 10 ms
	 * before being asserted high
	 */
	msleep(10);
}

static int sn65dsi_start_stream(void)
{
	struct device dev = sn65dsi_panel.client->dev;
	int regval;

	/* Set the PLL_EN bit (CSR 0x0D.0) */
	sn65dsi_write(SN65DSI83_PLL_EN_REG, 0x1);
	/* Wait for the PLL_LOCK bit to be set (CSR 0x0A.7) */
	msleep(200);

	/* Perform SW reset to apply changes */
	sn65dsi_write(SN65DSI83_SOFT_RESET_REG, 0x01);

	/* Read CHA Error register */
	regval = sn65dsi_read(SN65DSI83_CHA_ERR_REG);
	dev_info(&dev, "CHA (0x%02x) = 0x%02x",
		 SN65DSI83_CHA_ERR_REG, regval);

	return 0;
}

static void sn65dsi_stop_stream(void)
{
	/* Clear the PLL_EN bit (CSR 0x0D.0) */
	sn65dsi_write(SN65DSI83_PLL_EN_REG, 0x00);
}

static int sn65dsi83_calk_clk_range(int min_regval, int max_regval,
				    unsigned long min_clk, unsigned long inc,
				    unsigned long target_clk)
{
	int regval = min_regval;
	unsigned long clk = min_clk;

	while (regval <= max_regval) {
		if ((clk <= target_clk) && (target_clk < (clk + inc)))
			return regval;

		regval++;
		clk += inc;
	}

	return -1;
}

static int sn65dsi83_calk_div(int min_regval, int max_regval, int min_div,
			      int inc, unsigned long source_clk,
			      unsigned long target_clk)
{
	int regval = min_regval;
	int div = min_div;
	unsigned long curr_delta;
	unsigned long prev_delta = ABS(DIV_ROUND_UP(source_clk, div) -
					target_clk);

	while (regval <= max_regval) {
		curr_delta = DIV_ROUND_UP(source_clk, div);
		if (curr_delta > target_clk)
			curr_delta = curr_delta - target_clk;
		else
			curr_delta = target_clk - curr_delta;

		if (curr_delta > prev_delta)
			return --regval;
		prev_delta = curr_delta;
		regval++;
		div += inc;
	}

	return -1;
}

static int sn65dsi83_calk_dsi_bpp(void)
{
	switch (lcd_config.dpi_fmt) {
	case MIPI_RGB565_PACKED:
	case MIPI_RGB565_LOOSELY:
	case MIPI_RGB565_CONFIG3:
		return 16;
	case MIPI_RGB666_PACKED:
	case MIPI_RGB666_LOOSELY:
		return 18;
	case MIPI_RGB888:
	default:
		return 24;
	}
}

static int sn65dsi_configure(void)
{
	int regval = 0;
	int dsi_bpp = sn65dsi83_calk_dsi_bpp();
	struct videomode *vm = &sn65dsi_panel.vm;
	struct i2c_client *client = sn65dsi_panel.client;
	u32 dsi_clk = (((PIXCLK * dsi_bpp) /
			lcd_config.data_lane_num) >> 1);

	dev_info(&client->dev, "DSI clock [ %u ] Hz\n", dsi_clk);
	dev_info(&client->dev, "GeoMetry [ %d x %d ] Hz\n", HACTIVE, VACTIVE);

	/* Reset PLL_EN and SOFT_RESET registers */
	sn65dsi_write(SN65DSI83_SOFT_RESET_REG, 0x00);
	sn65dsi_write(SN65DSI83_PLL_EN_REG, 0x00);

	/* LVDS clock setup */
	if  ((25000000 <= PIXCLK) && (PIXCLK < 37500000))
		regval = 0;
	else
		regval = sn65dsi83_calk_clk_range(0x01, 0x05, 37500000,
						  25000000, PIXCLK);

	if (regval < 0) {
		dev_err(&client->dev, "failed to configure LVDS clock");
		return -EINVAL;
	}

	regval = (regval << SN65DSI83_LVDS_CLK_RANGE_SHIFT);
	regval |= (1 << SN65DSI83_HS_CLK_SRC_SHIFT); /* Use DSI clock */
	sn65dsi_write(SN65DSI83_CLOCK_REG, regval);

	/* DSI clock range */
	regval = sn65dsi83_calk_clk_range(0x08, 0x64, 40000000, 5000000,
					  dsi_clk);
	if (regval < 0) {
		dev_err(&client->dev, "failed to configure DSI clock range\n");
		return -EINVAL;
	}
	sn65dsi_write(SN65DSI83_CHA_DSI_CLK_RANGE_REG, regval);

	/* DSI clock divider */
	regval = sn65dsi83_calk_div(0x0, 0x18, 1, 1, dsi_clk, PIXCLK);
	if (regval < 0) {
		dev_err(&client->dev, "failed to calculate DSI clock divider");
		return -EINVAL;
	}
	regval = regval << SN65DSI83_DSI_CLK_DIV_SHIFT;
	sn65dsi_write(SN65DSI83_PLL_DIV_REG, regval);

	/* Configure DSI_LANES  */
	regval = sn65dsi_read(SN65DSI83_DSI_CFG_REG);
	regval &= ~(3 << SN65DSI83_CHA_DSI_LANES_SHIFT);
	regval |= ((4 - lcd_config.data_lane_num) <<
		   SN65DSI83_CHA_DSI_LANES_SHIFT);
	sn65dsi_write(SN65DSI83_DSI_CFG_REG, regval);

	/* CHA_DSI_DATA_EQ - No Equalization */
	/* CHA_DSI_CLK_EQ  - No Equalization */
	sn65dsi_write(SN65DSI83_DSI_EQ_REG, 0x00);

	/* Video formats */
	regval = 0;
	if (FLAGS & DISPLAY_FLAGS_HSYNC_LOW)
		regval |= (1 << SN65DSI83_HS_NEG_POLARITY_SHIFT);
	if (FLAGS & DISPLAY_FLAGS_VSYNC_LOW)
		regval |= (1 << SN65DSI83_VS_NEG_POLARITY_SHIFT);
	if (FLAGS & DISPLAY_FLAGS_DE_LOW)
		regval |= (1 << SN65DSI83_DE_NEG_POLARITY_SHIFT);
	if (sn65dsi_panel.bpp == 24)
		regval |= (1 << SN65DSI83_CHA_24BPP_MODE_SHIFT);
	if (sn65dsi_panel.format == 1)
		regval |= (1 << SN65DSI83_CHA_24BPP_FMT1_SHIFT);
	regval |= (1 << SN65DSI83_LVDS_LINK_CFG_SHIFT);
	sn65dsi_write(SN65DSI83_LVDS_MODE_REG, regval);

	/* Voltage and pins */
	sn65dsi_write(SN65DSI83_LVDS_SIGN_REG, 0x00);
	sn65dsi_write(SN65DSI83_LVDS_TERM_REG, 0x03);
	sn65dsi_write(SN65DSI83_LVDS_CM_ADJ_REG, 0x00);

	/* Configure sync delay to minimal allowed value */
	sn65dsi_write(SN65DSI83_CHA_SYNC_DELAY_LO_REG, 0x21);
	sn65dsi_write(SN65DSI83_CHA_SYNC_DELAY_HI_REG, 0x00);

	/* Geometry */
	sn65dsi_write(SN65DSI83_CHA_LINE_LEN_LO_REG, LOW(HACTIVE));
	sn65dsi_write(SN65DSI83_CHA_LINE_LEN_HI_REG, HIGH(HACTIVE));

	sn65dsi_write(SN65DSI83_CHA_VERT_LINES_LO_REG, LOW(VACTIVE));
	sn65dsi_write(SN65DSI83_CHA_VERT_LINES_HI_REG, HIGH(VACTIVE));

	sn65dsi_write(SN65DSI83_CHA_HSYNC_WIDTH_LO_REG, LOW(HPW));
	sn65dsi_write(SN65DSI83_CHA_HSYNC_WIDTH_HI_REG, HIGH(HPW));

	sn65dsi_write(SN65DSI83_CHA_VSYNC_WIDTH_LO_REG, LOW(VPW));
	sn65dsi_write(SN65DSI83_CHA_VSYNC_WIDTH_HI_REG, HIGH(VPW));

	sn65dsi_write(SN65DSI83_CHA_HORZ_BACKPORCH_REG, LOW(HBP));
	sn65dsi_write(SN65DSI83_CHA_VERT_BACKPORCH_REG, LOW(VBP));

	sn65dsi_write(SN65DSI83_CHA_HORZ_FRONTPORCH_REG, LOW(HFP));
	sn65dsi_write(SN65DSI83_CHA_VERT_FRONTPORCH_REG, LOW(VFP));

	sn65dsi_write(SN65DSI83_TEST_PATTERN_REG, 0x00);
	sn65dsi_write(SN65DSI83_3D_REG, 0x00);
	sn65dsi_write(SN65DSI83_3E_REG, 0x00);

	/* Mute channel B */
	sn65dsi_write(SN65DSI83_CHB_DSI_CLK_RNG_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_LINE_LEN_LO_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_LINE_LEN_HI_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VERT_LINES_LO_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VERT_LINES_HI_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_SYNC_DELAY_LO_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_SYNC_DELAY_HI_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_HSYNC_WIDTH_LO_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_HSYNC_WIDTH_HI_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VSYNC_WIDTH_LO_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VSYNC_WIDTH_HI_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_HORZ_BACKPORCH_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VERT_BACKPORCH_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_HORZ_FRONTPORCH_REG, 0x00);
	sn65dsi_write(SN65DSI83_CHB_VERT_FRONTPORCH_REG, 0x00);
	return 0;
}

int sn65dsi83_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;

	dev_notice(&dev, "MIPI DSI LCD setup.\n");

	sn65dsi_power_on();
	sn65dsi_configure();

	return 0;
}

int sn65dsi83_lcd_start(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;

	dev_notice(&dev, "MIPI DSI LCD start.\n");

	sn65dsi_start_stream();

	return 0;
}

int sn65dsi83_lcd_stop(struct mipi_dsi_info *mipi_dsi)
{
	struct device dev = mipi_dsi->pdev->dev;

	dev_notice(&dev, "MIPI DSI LCD stop.\n");

	sn65dsi_stop_stream();
	sn65dsi_power_off();

	return 0;
}

void sn65dsi83_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data)
{
	*mode = &sn65dsi_panel.fb_vm;
	*size = 1;
	*data = &lcd_config;
}

static int sn65dsi_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;

	dev_notice(&client->dev, "probe, client 0x%p\n", client);
	memset(&sn65dsi_panel, 0, sizeof(sn65dsi_panel));

	sn65dsi_panel.client = client;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	ret = sn65dsi_probe_of();
	if (ret) {
		dev_err(&client->dev, "failed to parse DT data");
		return -1;
	}

	sn65dsi_power_off();
	sn65dsi_power_on();

	/* Soft Reset reg value at power on should be 0x00 */
	ret = sn65dsi_read(SN65DSI83_SOFT_RESET_REG);
	if (ret != 0x00)
		return -ENODEV;

	sn65dsi_power_off();

	return ret;
}

static int __exit sn65dsi_remove(struct i2c_client *client)
{
	sn65dsi_stop_stream();
	sn65dsi_power_off();
	return 0;
}

static const struct i2c_device_id sn65dsi_id[] = {
	{ DRV_NAME, 0},
	{ },
};

MODULE_DEVICE_TABLE(i2c, sn65dsi_id);

static const struct of_device_id sn65dsi_of_match[] = {
	{ .compatible = "ti,sn65dsi83", },
	{ }
};

MODULE_DEVICE_TABLE(of, sn65dsi_of_match);

static struct i2c_driver sn65dsi_driver = {
	.probe	= sn65dsi_probe,
	.remove	= __exit_p(sn65dsi_remove),
	.id_table = sn65dsi_id,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = sn65dsi_of_match,
	},
};

module_i2c_driver(sn65dsi_driver);

MODULE_AUTHOR("Valentin Raevsky <valentin@compulab.co.il>");
MODULE_DESCRIPTION("SN65DSI83 DSI-to-LVDS bridge IC driver");
MODULE_LICENSE("GPL");
