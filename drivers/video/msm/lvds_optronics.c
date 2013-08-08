/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "msm_fb.h"
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <mach/gpio.h>

static struct lvds_panel_platform_data *optronics_pdata;
static struct platform_device *optronics_fbpdev;

static int lvds_optronics_panel_on(struct platform_device *pdev)
{
	return 0;
}

static int lvds_optronics_panel_off(struct platform_device *pdev)
{
	return 0;
}

static int __devinit lvds_optronics_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (pdev->id == 0) {
		optronics_pdata = pdev->dev.platform_data;
		if (optronics_pdata != NULL) {
			pr_info("LVDS Optronics probe <%s>(%d)\n",
				 __func__, __LINE__);
		}
		return 0;
	}

	optronics_fbpdev = msm_fb_add_device(pdev);
	if (!optronics_fbpdev) {
		dev_err(&pdev->dev, "failed to add msm_fb device\n");
		rc = -ENODEV;
	}
	pr_info("LVDS FB device is added <%s>(%d)\n",
		 __func__, __LINE__);

	return rc;
}

static int __devexit lvds_optronics_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = lvds_optronics_probe,
	.remove = lvds_optronics_remove,
	.driver = {
		.name   = "lvds_optronics",
	},
};

static struct msm_fb_panel_data lvds_optronics_panel_data = {
	.on = lvds_optronics_panel_on,
	.off = lvds_optronics_panel_off
};

static struct platform_device this_device = {
	.name   = "lvds_optronics",
	.id	= 1,
	.dev	= {
		.platform_data = &lvds_optronics_panel_data,
	}
};

static int __init lvds_optronics_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	if (msm_fb_detect_client("lvds_optronics"))
		return 0;

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &lvds_optronics_panel_data.panel_info;
	pinfo->xres = 1366;
	pinfo->yres = 768;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = LVDS_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 72000000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_front_porch	= 20;		/* thfp */
	pinfo->lcdc.h_back_porch	= 0;		/* thb */
	pinfo->lcdc.h_pulse_width	= 70;		/* thpw */

	pinfo->lcdc.v_front_porch	= 14;		/* tvfp */
	pinfo->lcdc.v_back_porch	= 0;		/* tvb */
	pinfo->lcdc.v_pulse_width	= 42;		/* tvpw */

	pinfo->lcdc.border_clr	= 0;		/* black */
	pinfo->lcdc.underflow_clr	= 0xff;	/* blue */
	pinfo->lcdc.hsync_skew	= 0;

	pinfo->lvds.channel_mode = LVDS_SINGLE_CHANNEL_MODE;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lvds_optronics_init);
