/*
 * Copyright (C) 2010-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <video/edid.h>

/**********************************************************
 * Macros for tracing
 **********************************************************/
//#define __LOG_TRACE__ 1

#ifndef EDID_LENGTH
	#define EDID_LENGTH 0x80
#endif

#ifdef __LOG_TRACE__
#define __TRACE__ printk(KERN_INFO "[ HDMI info ] %s\n", __func__);
#define __MSG_TRACE__(string, args...) printk(KERN_INFO "[ HDMI info ] "\
		" %s : %d : " string, __FUNCTION__, __LINE__, ##args)
#else
	#define __TRACE__ ;
	#define __MSG_TRACE__(string, args...) ;
#endif

struct sii902x_edid_cfg {
	bool cea_underscan;
	bool cea_basicaudio;
	bool cea_ycbcr444;
	bool cea_ycbcr422;
	bool hdmi_cap;
};

struct sii902x_data {
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct sii902x_edid_cfg edid_cfg;
	struct regmap *regmap;
	unsigned int irq;
	u8 cable_plugin;
} *sii902x;

static void sii902x_poweron(void);
static void sii902x_poweroff(void);

/**********************************************************
 * GLOBAL: monitor settings
 **********************************************************/
struct fb_info global_fbi;

/**********************************************************
 * FUNCTION: sii902x_to_i2c
 **********************************************************/
static struct i2c_client *sii902x_to_i2c(struct sii902x_data *sii902x)
{
	__TRACE__;
	return sii902x->client;
}

/**********************************************************
 * FUNCTION: sii902x_write
 **********************************************************/
static s32 sii902x_write(const struct i2c_client *client,
			u8 command, u8 value)
{
	__TRACE__;
	return i2c_smbus_write_byte_data(client, command, value);
}

/**********************************************************
 * FUNCTION: sii902x_read
 **********************************************************/
static s32 sii902x_read(const struct i2c_client *client, u8 command)
{
	int val;

	__TRACE__;
	val = i2c_smbus_read_word_data(client, command);

	return val & 0xff;
}

/**********************************************************
 * FUNCTION: __sii902x_read_edid
 **********************************************************/
static int __sii902x_read_edid(struct i2c_adapter *adp,
			unsigned char *edid, u8 *buf)
{
	unsigned short addr = 0x50;
	int ret;

	struct i2c_msg msg[2] = {
		{
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= buf,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	__TRACE__;

	if (adp == NULL)
		return -EINVAL;

	memset(edid, 0, EDID_LENGTH);

	ret = i2c_transfer(adp, msg, 2);
	if (ret < 0)
		return ret;

	/* If 0x50 fails, try 0x37. */
	if (edid[1] == 0x00) {
		msg[0].addr = msg[1].addr = 0x37;
		ret = i2c_transfer(adp, msg, 2);
		if (ret < 0)
			return ret;
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	return 0;
}

/**********************************************************
 * FUNCTION: __sii902x_get_edid
 * make sure edid has 256 bytes
 **********************************************************/
static int __sii902x_get_edid(struct i2c_adapter *adp,
		struct sii902x_edid_cfg *cfg, struct fb_info *fbi)
{
	u8 *edid;
	u8 buf[2] = {0, 0};
	int num, ret;

	__TRACE__;

	edid = kzalloc(EDID_LENGTH, GFP_KERNEL);
	if (!edid)
		return -ENOMEM;

	ret = __sii902x_read_edid(adp, edid, buf);
	if (ret)
		return ret;

	memset(cfg, 0, sizeof(struct sii902x_edid_cfg));
	/* edid first block parsing */
	memset(&fbi->monspecs, 0, sizeof(fbi->monspecs));
	fb_edid_to_monspecs(edid, &fbi->monspecs);

	/* need read ext block? Only support one more blk now*/
	num = edid[0x7E];
	if (num) {
		if (num > 1)
			printk("Edid has %d ext block, \
				but now only support 1 ext blk\n", num);

		buf[0] = 0x80;
		ret = __sii902x_read_edid(adp, edid, buf);
		if (ret)
			return ret;

		fb_edid_add_monspecs(edid, &fbi->monspecs);
	}

	kfree(edid);
	return 0;
}

/**********************************************************
 * FUNCTION: sii902x_get_edid
 **********************************************************/
static int sii902x_get_edid(struct fb_info *fbi)
{
	int old, dat, ret, cnt = 100;

	__TRACE__;

	old = sii902x_read(sii902x->client, 0x1A);

	sii902x_write(sii902x->client, 0x1A, old | 0x4);
	do {
		cnt--;
		msleep(10);
		dat = sii902x_read(sii902x->client, 0x1A);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	sii902x_write(sii902x->client, 0x1A, old | 0x06);

	/* edid reading */
	ret = __sii902x_get_edid(sii902x->client->adapter,
			&sii902x->edid_cfg, fbi);

	cnt = 100;
	do {
		cnt--;
		sii902x_write(sii902x->client, 0x1A, old & ~0x6);
		msleep(10);
		dat = sii902x_read(sii902x->client, 0x1A);
	} while ((dat & 0x6) && cnt);

	if (!cnt)
		ret = -1;

done:
	sii902x_write(sii902x->client, 0x1A, old);
	return ret;
}

/**********************************************************
 * FUNCTION: sii902x_power_up_tx
 **********************************************************/
static void sii902x_power_up_tx(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	__TRACE__;

	val = sii902x_read(client, 0x1E);
	val &= ~0x3;
	sii902x_write(client, 0x1E, val);
}

/**********************************************************
 * FUNCTION: sii902x_chip_id
 **********************************************************/
static void sii902x_chip_id(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	__TRACE__;

	/* read device ID */
	val = sii902x_read(client, 0x1B);
	printk("Sii902x: read id = 0x%02X", val);
	val = sii902x_read(client, 0x1C);
	printk("-0x%02X", val);
	val = sii902x_read(client, 0x1D);
	printk("-0x%02X", val);
	val = sii902x_read(client, 0x30);
	printk("-0x%02X\n", val);
}


/**********************************************************
 * FUNCTION: sii902x_initialize
 **********************************************************/
static int sii902x_initialize(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int ret, cnt;

	__TRACE__;

	for (cnt = 0; cnt < 5; cnt++) {
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
		ret = sii902x_write(client, 0xC7, 0x00);
		if (ret < 0)
			break;
	}
	if (0 != ret)
		dev_err(&client->dev, "cound not find device\n");

	return ret;
}

/**********************************************************
 * FUNCTION: sii902x_enable_source
 **********************************************************/
static void sii902x_enable_source(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	__TRACE__;

	sii902x_write(client, 0xBC, 0x01);
	sii902x_write(client, 0xBD, 0x82);
	val = sii902x_read(client, 0xBE);
	val |= 0x1;
	sii902x_write(client, 0xBE, val);
}

/**********************************************************
 * FUNCTION: fsl_dcu_num_layers
 * INFO: number of layers is based on max blending layers
 **********************************************************/

/* FIXME: DCU - HDMI communication of EDID parameters, Rayleigh */
struct fb_monspecs sii902x_get_monspecs(void)
{
	__TRACE__;

	/* update monitor settings */
	sii902x_get_edid(&global_fbi);

	return global_fbi.monspecs;
}
EXPORT_SYMBOL_GPL(sii902x_get_monspecs);

/**********************************************************
 * FUNCTION: sii902x_probe
 **********************************************************/

/* FIXME: SII9022A adapted minimal probe/init for Rayleigh EVB */
static int sii902x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	int err;

	__TRACE__;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

	sii902x = devm_kzalloc(&client->dev, sizeof(*sii902x), GFP_KERNEL);
	if (!sii902x)
		return -ENOMEM;

	sii902x->client = client;
	i2c_set_clientdata(client, sii902x);

	err = sii902x_initialize(sii902x);
	if (err)
		return err;

	sii902x_write(client, 0xC7, 0x0);
	sii902x_chip_id(sii902x);
	sii902x_power_up_tx(sii902x);
	sii902x_enable_source(sii902x);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	sii902x_write(client, 0x8, 0x70);
	/* Set input format to RGB */
	sii902x_write(client, 0x9, 0x0);
	/* set output format to RGB */
	sii902x_write(client, 0xA, 0x0);
	/* audio setup */
	sii902x_write(client, 0x25, 0x0);
	sii902x_write(client, 0x26, 0x81);
	sii902x_write(client, 0x27, 0x50);

	sii902x_poweron();

	// detect resolution
	sii902x_get_edid(&global_fbi);

	return 0;
}

/**********************************************************
 * FUNCTION: sii902x_remove
 **********************************************************/
static int sii902x_remove(struct i2c_client *client)
{
	__TRACE__;

	sii902x_poweroff();
	return 0;
}

/**********************************************************
 * FUNCTION: sii902x_poweron
 **********************************************************/
static void sii902x_poweron(void)
{
	__TRACE__;

	/* Turn on DVI or HDMI */
	if (sii902x->edid_cfg.hdmi_cap){
		sii902x_write(sii902x->client, 0x1A, 0x01);
	}
	else {
		sii902x_write(sii902x->client, 0x1A, 0x00);
	}
	return;
}

/**********************************************************
 * FUNCTION: sii902x_poweroff
 **********************************************************/
static void sii902x_poweroff(void)
{
	__TRACE__;

	/* disable tmds before changing resolution */
	if (sii902x->edid_cfg.hdmi_cap)
		sii902x_write(sii902x->client, 0x1A, 0x11);
	else
		sii902x_write(sii902x->client, 0x1A, 0x10);
	return;
}

/**********************************************************
 * DRIVER configuration
 **********************************************************/
static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static const struct of_device_id sii902x_dt_ids[] = {
	{ .compatible = "fsl,sii902x", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sii902x_dt_ids);

static struct i2c_driver sii902x_i2c_driver = {
	.driver = {
		.name = "sii902x",
		.owner = THIS_MODULE,
		.of_match_table = sii902x_dt_ids,
	},
	.probe = sii902x_probe,
	.remove = sii902x_remove,
	.id_table = sii902x_id,
};
module_i2c_driver(sii902x_i2c_driver);


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");
