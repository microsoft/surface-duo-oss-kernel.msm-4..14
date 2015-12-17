/*
 * Copyright (C) 2015 CompuLab LTD.
 * All Rights Reserved.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>

struct sil164_encoder_params {
	enum {
		SIL164_INPUT_EDGE_FALLING = 0,
		SIL164_INPUT_EDGE_RISING
	} input_edge;

	enum {
		SIL164_INPUT_WIDTH_12BIT = 0,
		SIL164_INPUT_WIDTH_24BIT
	} input_width;

	enum {
		SIL164_INPUT_SINGLE_EDGE = 0,
		SIL164_INPUT_DUAL_EDGE
	} input_dual;

	enum {
		SIL164_PLL_FILTER_ON = 0,
		SIL164_PLL_FILTER_OFF,
	} pll_filter;

	int input_skew; /** < Allowed range [-4, 3], use 0 for no de-skew. */
	int duallink_skew; /** < Allowed range [-4, 3]. */
};

#define sil164_info(client, format, ...)		\
	dev_info(&client->dev, format, __VA_ARGS__)
#define sil164_err(client, format, ...)			\
	dev_err(&client->dev, format, __VA_ARGS__)

/* HW register definitions */

#define SIL164_VENDOR_LO			0x0
#define SIL164_VENDOR_HI			0x1
#define SIL164_DEVICE_LO			0x2
#define SIL164_DEVICE_HI			0x3
#define SIL164_REVISION				0x4
#define SIL164_FREQ_MIN				0x6
#define SIL164_FREQ_MAX				0x7

#define SIL164_CONTROL0				0x8
#define SIL164_CONTROL0_POWER_ON		0x01
#define SIL164_CONTROL0_EDGE_RISING		0x02
#define SIL164_CONTROL0_INPUT_24BIT		0x04
#define SIL164_CONTROL0_DUAL_EDGE		0x08
#define SIL164_CONTROL0_HSYNC_ON		0x10
#define SIL164_CONTROL0_VSYNC_ON		0x20

#define SIL164_DETECT				0x9
#define SIL164_DETECT_INTR_STAT			0x01
#define SIL164_DETECT_HOTPLUG_STAT		0x02
#define SIL164_DETECT_RECEIVER_STAT		0x04
#define SIL164_DETECT_INTR_MODE_RECEIVER	0x00
#define SIL164_DETECT_INTR_MODE_HOTPLUG		0x08
#define SIL164_DETECT_OUT_MODE_HIGH		0x00
#define SIL164_DETECT_OUT_MODE_INTR		0x10
#define SIL164_DETECT_OUT_MODE_RECEIVER		0x20
#define SIL164_DETECT_OUT_MODE_HOTPLUG		0x30
#define SIL164_DETECT_VSWING_STAT		0x80

#define SIL164_CONTROL1				0xa
#define SIL164_CONTROL1_DESKEW_ENABLE		0x10
#define SIL164_CONTROL1_DESKEW_INCR_SHIFT	5

#define SIL164_GPIO				0xb

#define SIL164_CONTROL2				0xc
#define SIL164_CONTROL2_FILTER_ENABLE		0x01
#define SIL164_CONTROL2_FILTER_SETTING_SHIFT	1
#define SIL164_CONTROL2_DUALLINK_MASTER		0x40
#define SIL164_CONTROL2_SYNC_CONT		0x80

#define SIL164_DUALLINK				0xd
#define SIL164_DUALLINK_ENABLE			0x10
#define SIL164_DUALLINK_SKEW_SHIFT		5

#define SIL164_PLLZONE				0xe
#define SIL164_PLLZONE_STAT			0x08
#define SIL164_PLLZONE_FORCE_ON			0x10
#define SIL164_PLLZONE_FORCE_HIGH		0x20

/* HW access functions */

static void
sil164_write(struct i2c_client *client, uint8_t addr, uint8_t val)
{
	uint8_t buf[] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		sil164_err(client, "Error %d writing to subaddress 0x%x\n",
			   ret, addr);
}

static uint8_t
sil164_read(struct i2c_client *client, uint8_t addr)
{
	uint8_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, &val, sizeof(val));
	if (ret < 0)
		goto fail;

	return val;

fail:
	sil164_err(client, "Error %d reading from subaddress 0x%x\n",
		   ret, addr);
	return 0;
}

static void
sil164_set_power_state(struct i2c_client *client, bool on)
{
	uint8_t control0 = sil164_read(client, SIL164_CONTROL0);

	if (on)
		control0 |= SIL164_CONTROL0_POWER_ON;
	else
		control0 &= ~SIL164_CONTROL0_POWER_ON;

	sil164_write(client, SIL164_CONTROL0, control0);
}

static void
sil164_init_state(struct i2c_client *client,
		  struct sil164_encoder_params *config,
		  bool duallink)
{
	sil164_write(client, SIL164_CONTROL0,
		     SIL164_CONTROL0_HSYNC_ON |
		     SIL164_CONTROL0_VSYNC_ON |
		     (config->input_edge ? SIL164_CONTROL0_EDGE_RISING : 0) |
		     (config->input_width ? SIL164_CONTROL0_INPUT_24BIT : 0) |
		     (config->input_dual ? SIL164_CONTROL0_DUAL_EDGE : 0));

	sil164_write(client, SIL164_DETECT,
		     SIL164_DETECT_INTR_STAT |
		     SIL164_DETECT_OUT_MODE_RECEIVER);

	sil164_write(client, SIL164_CONTROL1,
		     (config->input_skew ? SIL164_CONTROL1_DESKEW_ENABLE : 0) |
		     (((config->input_skew + 4) & 0x7)
		      << SIL164_CONTROL1_DESKEW_INCR_SHIFT));

	sil164_write(client, SIL164_CONTROL2,
		     SIL164_CONTROL2_SYNC_CONT |
		     (config->pll_filter ? 0 : SIL164_CONTROL2_FILTER_ENABLE) |
		     (4 << SIL164_CONTROL2_FILTER_SETTING_SHIFT));

	sil164_write(client, SIL164_PLLZONE, 0);

	if (duallink)
		sil164_write(client, SIL164_DUALLINK,
			     SIL164_DUALLINK_ENABLE |
			     (((config->duallink_skew + 4) & 0x7)
			      << SIL164_DUALLINK_SKEW_SHIFT));
	else
		sil164_write(client, SIL164_DUALLINK, 0);
}

/* I2C driver functions */

static int
sil164_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sil164_encoder_params *config;

	int vendor = sil164_read(client, SIL164_VENDOR_HI) << 8 |
		sil164_read(client, SIL164_VENDOR_LO);
	int device = sil164_read(client, SIL164_DEVICE_HI) << 8 |
		sil164_read(client, SIL164_DEVICE_LO);
	int rev = sil164_read(client, SIL164_REVISION);

	if (vendor != 0x1 || device != 0x6) {
		sil164_info(client, "Unknown device %x:%x.%x\n",
			   vendor, device, rev);
		return -ENODEV;
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->input_width = SIL164_CONTROL0_INPUT_24BIT;

	sil164_init_state(client, config, 0);

	sil164_set_power_state(client, 1);

	sil164_info(client, "Detected device %x:%x.%x\n",
		    vendor, device, rev);

	kfree(config);

	return 0;
}

static int
sil164_remove(struct i2c_client *client)
{
	sil164_set_power_state(client, 0);
	return 0;
}

static struct i2c_device_id sil164_ids[] = {
	{ "sil164_simple", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sil164_ids);

static struct i2c_driver sil164_simple_driver = {
		.probe = sil164_probe,
		.remove = sil164_remove,
		.driver = {
			.name = "sil164_simple",
		},
		.id_table = sil164_ids,
};

module_i2c_driver(sil164_simple_driver);

MODULE_AUTHOR("CompuLab Ltd.");
MODULE_DESCRIPTION("Silicon Image sil164 TMDS transmitter simple driver");
MODULE_LICENSE("GPL and additional rights");
