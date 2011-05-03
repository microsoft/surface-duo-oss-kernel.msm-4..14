/*
 * sdp4430-hdmi.c
 *
 * OMAP ALSA SoC machine driver for TI OMAP4 HDMI
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Ricardo Neri <ricardo.neri@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>

#define OMAP4_HDMI_SND_DEV_ID 1

static struct snd_soc_dai_link omap4_hdmi_dai = {
		.name = "HDMI",
		.stream_name = "HDMI",
		.cpu_dai_name = "hdmi-audio-dai",
		.platform_name = "omap-pcm-audio",
		.codec_name = "omapdss_hdmi",
		.codec_dai_name = "omap4-hdmi-audio-codec"
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_omap4_hdmi = {
	.name = "SDP4430HDMI",
	/* .long_name = "TI OMAP4 HDMI Board", */
	.dai_link = &omap4_hdmi_dai,
	.num_links = 1,
};

static struct platform_device *omap4_hdmi_snd_device;

static int __init omap4_hdmi_soc_init(void)
{
	int ret;

	if (!(machine_is_omap_4430sdp() || machine_is_omap4_panda()))
		return -ENODEV;
	printk(KERN_INFO "OMAP4 HDMI audio SoC init\n");

	if (machine_is_omap4_panda())
		snd_soc_omap4_hdmi.name = "PandaHDMI";

	omap4_hdmi_snd_device = platform_device_alloc("soc-audio",
		OMAP4_HDMI_SND_DEV_ID);
	if (!omap4_hdmi_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(omap4_hdmi_snd_device, &snd_soc_omap4_hdmi);

	ret = platform_device_add(omap4_hdmi_snd_device);
	if (ret)
		goto err;

	return 0;
err:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(omap4_hdmi_snd_device);
	return ret;
}
module_init(omap4_hdmi_soc_init);

static void __exit omap4_hdmi_soc_exit(void)
{
	platform_device_unregister(omap4_hdmi_snd_device);
}
module_exit(omap4_hdmi_soc_exit);

MODULE_AUTHOR("Ricardo Neri <ricardo.neri@ti.com>");
MODULE_DESCRIPTION("ALSA SoC OMAP4 HDMI AUDIO");
MODULE_LICENSE("GPL");
