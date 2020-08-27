// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * Copyright (c) 2019-2020. Linaro Limited.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <sound/hdmi-codec.h>

#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_probe_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_print.h>

#define EDID_BLOCK_SIZE	128
#define EDID_NUM_BLOCKS 2

struct lt9611uxc {
	struct device *dev;
	struct drm_bridge bridge;

	struct regmap *regmap;
	/* Protects all accesses to registers by stopping the on-chip MCU */
	struct mutex ocm_lock;

	struct device_node *dsi0_node;
	struct device_node *dsi1_node;
	struct mipi_dsi_device *dsi0;
	struct mipi_dsi_device *dsi1;
	struct platform_device *audio_pdev;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	struct regulator_bulk_data supplies[2];

	struct i2c_client *client;

	bool hpd_supported;
	struct display_timings *timings;
	u8 edid_buf[EDID_BLOCK_SIZE * EDID_NUM_BLOCKS];
};

#define LT9611_PAGE_CONTROL	0xff

static const struct regmap_range_cfg lt9611uxc_ranges[] = {
	{
		.name = "register_range",
		.range_min =  0,
		.range_max = 0xd0ff,
		.selector_reg = LT9611_PAGE_CONTROL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt9611uxc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
	.ranges = lt9611uxc_ranges,
	.num_ranges = ARRAY_SIZE(lt9611uxc_ranges),
};

struct lt9611uxc_mode {
	u16 hdisplay;
	u16 vdisplay;
	u8 vrefresh;
	u8 lanes;
	u8 intfs;
};

static struct lt9611uxc_mode lt9611uxc_modes[] = {
#if 0
	{ 3840, 2160, 60, 4, 2 }, /* 3840x2160 24bit 60Hz 4Lane 2ports */
	{ 3840, 2160, 30, 4, 2 }, /* 3840x2160 24bit 30Hz 4Lane 2ports */
	{ 1920, 1080, 60, 4, 1 }, /* 1080P 24bit 60Hz 4lane 1port */
	{ 1920, 1080, 30, 3, 1 }, /* 1080P 24bit 30Hz 3lane 1port */
	{ 1920, 1080, 24, 3, 1 },
#endif
	{ 1024, 768, 60, 4, 1},
	{ 800, 600, 60, 4, 1},
	{ 720, 480, 60, 4, 1 },
	{ 720, 576, 50, 2, 1 },
	{ 640, 480, 60, 2, 1 },
};

static struct lt9611uxc *bridge_to_lt9611uxc(struct drm_bridge *bridge)
{
	return container_of(bridge, struct lt9611uxc, bridge);
}

static void lt9611uxc_lock(struct lt9611uxc *lt9611uxc)
{
	mutex_lock(&lt9611uxc->ocm_lock);
	regmap_write(lt9611uxc->regmap, 0x80ee, 0x01);
}

static void lt9611uxc_unlock(struct lt9611uxc *lt9611uxc)
{
	regmap_write(lt9611uxc->regmap, 0x80ee, 0x00);
	msleep(50);
	mutex_unlock(&lt9611uxc->ocm_lock);
}

static irqreturn_t lt9611uxc_irq_thread_handler(int irq, void *dev_id)
{
	struct lt9611uxc *lt9611uxc = dev_id;
	unsigned int irq_status = 0;
	unsigned int hpd_status = 0;

	lt9611uxc_lock(lt9611uxc);

	regmap_read(lt9611uxc->regmap, 0xb022, &irq_status);
	if (irq_status) {
		regmap_write(lt9611uxc->regmap, 0xb022, 0);
		regmap_read(lt9611uxc->regmap, 0xb023, &hpd_status);
	}

	lt9611uxc_unlock(lt9611uxc);

	if (irq_status & 0x3 && lt9611uxc->bridge.dev)
		drm_kms_helper_hotplug_event(lt9611uxc->bridge.dev);

	return IRQ_HANDLED;
}

static void lt9611uxc_reset(struct lt9611uxc *lt9611uxc)
{
	gpiod_set_value_cansleep(lt9611uxc->reset_gpio, 1);
	msleep(20);

	gpiod_set_value_cansleep(lt9611uxc->reset_gpio, 0);
	msleep(20);

	gpiod_set_value_cansleep(lt9611uxc->reset_gpio, 1);
	msleep(180);
}

static void lt9611uxc_assert_5v(struct lt9611uxc *lt9611uxc)
{
	if (!lt9611uxc->enable_gpio)
		return;

	gpiod_set_value_cansleep(lt9611uxc->enable_gpio, 1);
	msleep(20);
}

static int lt9611uxc_regulator_init(struct lt9611uxc *lt9611uxc)
{
	int ret;

	lt9611uxc->supplies[0].supply = "vdd";
	lt9611uxc->supplies[1].supply = "vcc";

	ret = devm_regulator_bulk_get(lt9611uxc->dev, 2, lt9611uxc->supplies);
	if (ret < 0)
		return ret;

	return regulator_set_load(lt9611uxc->supplies[0].consumer, 200000);
}

static int lt9611uxc_regulator_enable(struct lt9611uxc *lt9611uxc)
{
	int ret;

	ret = regulator_enable(lt9611uxc->supplies[0].consumer);
	if (ret < 0)
		return ret;

	usleep_range(1000, 10000); /* 50000 according to dtsi */

	ret = regulator_enable(lt9611uxc->supplies[1].consumer);
	if (ret < 0) {
		regulator_disable(lt9611uxc->supplies[0].consumer);
		return ret;
	}

	return 0;
}

static struct lt9611uxc_mode *lt9611uxc_find_mode(const struct drm_display_mode *mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lt9611uxc_modes); i++) {
		if (lt9611uxc_modes[i].hdisplay == mode->hdisplay &&
		    lt9611uxc_modes[i].vdisplay == mode->vdisplay &&
		    lt9611uxc_modes[i].vrefresh == drm_mode_vrefresh(mode)) {
			return &lt9611uxc_modes[i];
		}
	}

	return NULL;
}

static void lt9611uxc_bridge_enable(struct drm_bridge *bridge)
{
}

static void lt9611uxc_bridge_disable(struct drm_bridge *bridge)
{
}

static struct mipi_dsi_device *lt9611uxc_attach_dsi(struct lt9611uxc *lt9611uxc,
						 struct device_node *dsi_node)
{
	const struct mipi_dsi_device_info info = { "lt9611uxc", 0, NULL };
	struct mipi_dsi_device *dsi;
	struct mipi_dsi_host *host;
	int ret;

	host = of_find_mipi_dsi_host_by_node(dsi_node);
	if (!host) {
		dev_err(lt9611uxc->dev, "failed to find dsi host\n");
		return ERR_PTR(-EPROBE_DEFER);
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(lt9611uxc->dev, "failed to create dsi device\n");
		return dsi;
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(lt9611uxc->dev, "failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
		return ERR_PTR(ret);
	}

	return dsi;
}

static void lt9611uxc_bridge_detach(struct drm_bridge *bridge)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);

	if (lt9611uxc->dsi1) {
		mipi_dsi_detach(lt9611uxc->dsi1);
		mipi_dsi_device_unregister(lt9611uxc->dsi1);
	}

	mipi_dsi_detach(lt9611uxc->dsi0);
	mipi_dsi_device_unregister(lt9611uxc->dsi0);
}

static int lt9611uxc_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)) {
		dev_err(lt9611uxc->dev, "Fix bridge driver to make connector optional!");
		return -EINVAL;
	}

	/* Attach primary DSI */
	lt9611uxc->dsi0 = lt9611uxc_attach_dsi(lt9611uxc, lt9611uxc->dsi0_node);
	if (IS_ERR(lt9611uxc->dsi0))
		return PTR_ERR(lt9611uxc->dsi0);

	/* Attach secondary DSI, if specified */
	if (lt9611uxc->dsi1_node) {
		lt9611uxc->dsi1 = lt9611uxc_attach_dsi(lt9611uxc, lt9611uxc->dsi1_node);
		if (IS_ERR(lt9611uxc->dsi1)) {
			ret = PTR_ERR(lt9611uxc->dsi1);
			goto err_unregister_dsi0;
		}
	}

	return 0;

err_unregister_dsi0:
	lt9611uxc_bridge_detach(bridge);
	mipi_dsi_device_unregister(lt9611uxc->dsi0);

	return ret;
}

static enum drm_mode_status
lt9611uxc_bridge_mode_valid(struct drm_bridge *bridge,
		const struct drm_display_info *info,
		const struct drm_display_mode *mode)
{
	struct lt9611uxc_mode *lt9611uxc_mode;

	lt9611uxc_mode = lt9611uxc_find_mode(mode);

	return lt9611uxc_mode ? MODE_OK : MODE_BAD;
}

static void lt9611uxc_bridge_post_disable(struct drm_bridge *bridge)
{
}

static void lt9611uxc_video_setup(struct lt9611uxc *lt9611uxc,
		const struct drm_display_mode *mode)
{
	u32 h_total, hactive, hsync_len, hfront_porch, hsync_porch;
	u32 v_total, vactive, vsync_len, vfront_porch, vsync_porch;

	h_total = mode->htotal;
	v_total = mode->vtotal;

	hactive = mode->hdisplay;
	hsync_len = mode->hsync_end - mode->hsync_start;
	hfront_porch = mode->hsync_start - mode->hdisplay;
	hsync_porch = hsync_len + mode->htotal - mode->hsync_end;

	vactive = mode->vdisplay;
	vsync_len = mode->vsync_end - mode->vsync_start;
	vfront_porch = mode->vsync_start - mode->vdisplay;
	vsync_porch = vsync_len + mode->vtotal - mode->vsync_end;

	regmap_write(lt9611uxc->regmap, 0xd00d, (u8)(v_total / 256));
	regmap_write(lt9611uxc->regmap, 0xd00e, (u8)(v_total % 256));

	regmap_write(lt9611uxc->regmap, 0xd00f, (u8)(vactive / 256));
	regmap_write(lt9611uxc->regmap, 0xd010, (u8)(vactive % 256));

	regmap_write(lt9611uxc->regmap, 0xd011, (u8)(h_total / 256));
	regmap_write(lt9611uxc->regmap, 0xd012, (u8)(h_total % 256));

	regmap_write(lt9611uxc->regmap, 0xd013, (u8)(hactive / 256));
	regmap_write(lt9611uxc->regmap, 0xd014, (u8)(hactive % 256));

	regmap_write(lt9611uxc->regmap, 0xd015, (u8)(vsync_len % 256));

	regmap_update_bits(lt9611uxc->regmap, 0xd016, 0xf, (u8)(hsync_len / 256));
	regmap_write(lt9611uxc->regmap, 0xd017, (u8)(hsync_len % 256));

	regmap_update_bits(lt9611uxc->regmap, 0xd018, 0xf, (u8)(vfront_porch / 256));
	regmap_write(lt9611uxc->regmap, 0xd019, (u8)(vfront_porch % 256));

	regmap_update_bits(lt9611uxc->regmap, 0xd01a, 0xf, (u8)(hfront_porch / 256));
	regmap_write(lt9611uxc->regmap, 0xd01b, (u8)(hfront_porch % 256));
}

static void lt9611uxc_bridge_mode_set(struct drm_bridge *bridge,
				   const struct drm_display_mode *mode,
				   const struct drm_display_mode *adj_mode)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);

	lt9611uxc_lock(lt9611uxc);
	lt9611uxc_video_setup(lt9611uxc, mode);
	lt9611uxc_unlock(lt9611uxc);
}

static enum drm_connector_status lt9611uxc_bridge_detect(struct drm_bridge *bridge)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);
	unsigned int reg_val = 0;
	int ret;
	int connected = 1;

	if (lt9611uxc->hpd_supported) {
		lt9611uxc_lock(lt9611uxc);
		ret = regmap_read(lt9611uxc->regmap, 0xb023, &reg_val);
		lt9611uxc_unlock(lt9611uxc);

		if (ret)
			dev_err(lt9611uxc->dev, "failed to read hpd status: %d\n", ret);
		else
			connected  = reg_val & BIT(1);
	}

	return connected ?  connector_status_connected :
				connector_status_disconnected;
}

static int lt9611uxc_bridge_get_modes(struct drm_bridge *bridge,
		struct drm_connector *connector)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);
	struct display_timings *timings = lt9611uxc->timings;
	int i;

	for (i = 0; i < timings->num_timings; i++) {
		struct drm_display_mode *mode = drm_mode_create(bridge->dev);
		struct videomode vm;

		if (videomode_from_timings(timings, &vm, i))
			break;

		drm_display_mode_from_videomode(&vm, mode);

		mode->type = DRM_MODE_TYPE_DRIVER;

		if (timings->native_mode == i)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int lt9611uxc_read_edid(struct lt9611uxc *lt9611uxc)
{
	int ret = 0;
	int i;

	/* memset to clear old buffer, if any */
	memset(lt9611uxc->edid_buf, 0, sizeof(lt9611uxc->edid_buf));

	lt9611uxc_lock(lt9611uxc);

	regmap_write(lt9611uxc->regmap, 0xb00b, 0x10);

#define EDID_SEG 16
	for (i = 0; i < 2 * EDID_BLOCK_SIZE; i += EDID_SEG) {
		regmap_write(lt9611uxc->regmap, 0xb00a, i);
		ret = regmap_noinc_read(lt9611uxc->regmap, 0xb0b0, &lt9611uxc->edid_buf[i], EDID_SEG);
		if (ret < 0)
			break;
	}

	lt9611uxc_unlock(lt9611uxc);
	return ret;
}

static int lt9611uxc_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	struct lt9611uxc *lt9611uxc = data;
	int ret;

	if (len > EDID_BLOCK_SIZE)
		return -EINVAL;

	if (block >= EDID_NUM_BLOCKS)
		return -EINVAL;

	if (block == 0) {
		ret = lt9611uxc_read_edid(lt9611uxc);
		if (ret) {
			dev_err(lt9611uxc->dev, "edid read failed\n");
			return ret;
		}
	}

	memcpy(buf, lt9611uxc->edid_buf + block * EDID_BLOCK_SIZE, len);
	return 0;
};

struct edid* lt9611uxc_bridge_get_edid(struct drm_bridge *bridge,
		struct drm_connector *connector)
{
	struct lt9611uxc *lt9611uxc = bridge_to_lt9611uxc(bridge);

	return drm_do_get_edid(connector, lt9611uxc_get_edid_block, lt9611uxc);
}

static const struct drm_bridge_funcs lt9611uxc_bridge_funcs = {
	.attach = lt9611uxc_bridge_attach,
	.detach = lt9611uxc_bridge_detach,
	.mode_valid = lt9611uxc_bridge_mode_valid,
	.enable = lt9611uxc_bridge_enable,
	.disable = lt9611uxc_bridge_disable,
	.post_disable = lt9611uxc_bridge_post_disable,
	.mode_set = lt9611uxc_bridge_mode_set,
	.detect = lt9611uxc_bridge_detect,
	.get_modes = lt9611uxc_bridge_get_modes,
	.get_edid = lt9611uxc_bridge_get_edid,
};

static int lt9611uxc_parse_dt(struct device *dev,
			   struct lt9611uxc *lt9611uxc)
{
	lt9611uxc->timings = of_get_display_timings(dev->of_node);
	if (!lt9611uxc->timings)
		dev_info(dev, "no display timings provided\n");

	lt9611uxc->dsi0_node = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!lt9611uxc->dsi0_node) {
		dev_err(lt9611uxc->dev, "failed to get remote node for primary dsi\n");
		return -ENODEV;
	}

	lt9611uxc->dsi1_node = of_graph_get_remote_node(dev->of_node, 2, -1);

	return 0;
}

static int lt9611uxc_gpio_init(struct lt9611uxc *lt9611uxc)
{
	struct device *dev = lt9611uxc->dev;

	lt9611uxc->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(lt9611uxc->reset_gpio)) {
		dev_err(dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt9611uxc->reset_gpio);
	}

	lt9611uxc->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						      GPIOD_OUT_LOW);
	if (IS_ERR(lt9611uxc->enable_gpio)) {
		dev_err(dev, "failed to acquire enable gpio\n");
		return PTR_ERR(lt9611uxc->enable_gpio);
	}

	return 0;
}

static int lt9611uxc_read_device_rev(struct lt9611uxc *lt9611uxc)
{
	unsigned int rev0, rev1, rev2;
	int ret;

	lt9611uxc_lock(lt9611uxc);

	ret = regmap_read(lt9611uxc->regmap, 0x8100, &rev0);
	ret |= regmap_read(lt9611uxc->regmap, 0x8101, &rev1);
	ret |= regmap_read(lt9611uxc->regmap, 0x8102, &rev2);
	if (ret)
		dev_err(lt9611uxc->dev, "failed to read revision: %d\n", ret);
	else
		dev_info(lt9611uxc->dev, "LT9611 revision: 0x%02x.%02x.%02x\n", rev0, rev1, rev2);

	lt9611uxc_unlock(lt9611uxc);

	return ret;
}

static int lt9611uxc_read_version(struct lt9611uxc *lt9611uxc)
{
	unsigned int rev;
	int ret;

	lt9611uxc_lock(lt9611uxc);

	ret = regmap_read(lt9611uxc->regmap, 0xb021, &rev);
	if (ret)
		dev_err(lt9611uxc->dev, "failed to read revision: %d\n", ret);
	else
		dev_info(lt9611uxc->dev, "LT9611 version: 0x%02x\n", rev);

	lt9611uxc_unlock(lt9611uxc);

	return ret < 0 ? ret : rev;
}

static int lt9611uxc_hdmi_hw_params(struct device *dev, void *data,
				 struct hdmi_codec_daifmt *fmt,
				 struct hdmi_codec_params *hparms)
{
	/* LT9611UXC will automatically detect rate and bitness, so no need to
	 * setup anything here. */
	return 0;
}

static void lt9611uxc_audio_shutdown(struct device *dev, void *data)
{
}

static int lt9611uxc_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
				      struct device_node *endpoint)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/*
	 * HDMI sound should be located as reg = <2>
	 * Then, it is sound port 0
	 */
	if (of_ep.port == 2)
		return 0;

	return -EINVAL;
}

static const struct hdmi_codec_ops lt9611uxc_codec_ops = {
	.hw_params	= lt9611uxc_hdmi_hw_params,
	.audio_shutdown = lt9611uxc_audio_shutdown,
	.get_dai_id	= lt9611uxc_hdmi_i2s_get_dai_id,
};

static int lt9611uxc_audio_init(struct device *dev, struct lt9611uxc *lt9611uxc)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &lt9611uxc_codec_ops,
		.max_i2s_channels = 2,
		.i2s = 1,
		.data = lt9611uxc,
	};

	lt9611uxc->audio_pdev =
		platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					      PLATFORM_DEVID_AUTO,
					      &codec_data, sizeof(codec_data));

	return PTR_ERR_OR_ZERO(lt9611uxc->audio_pdev);
}

static void lt9611uxc_audio_exit(struct lt9611uxc *lt9611uxc)
{
	if (lt9611uxc->audio_pdev) {
		platform_device_unregister(lt9611uxc->audio_pdev);
		lt9611uxc->audio_pdev = NULL;
	}
}

static int lt9611uxc_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lt9611uxc *lt9611uxc;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	lt9611uxc = devm_kzalloc(dev, sizeof(*lt9611uxc), GFP_KERNEL);
	if (!lt9611uxc)
		return -ENOMEM;

	lt9611uxc->dev = &client->dev;
	lt9611uxc->client = client;
	mutex_init(&lt9611uxc->ocm_lock);

	lt9611uxc->regmap = devm_regmap_init_i2c(client, &lt9611uxc_regmap_config);
	if (IS_ERR(lt9611uxc->regmap)) {
		dev_err(lt9611uxc->dev, "regmap i2c init failed\n");
		return PTR_ERR(lt9611uxc->regmap);
	}

	ret = lt9611uxc_parse_dt(&client->dev, lt9611uxc);
	if (ret) {
		dev_err(dev, "failed to parse device tree\n");
		return ret;
	}

	ret = lt9611uxc_gpio_init(lt9611uxc);
	if (ret < 0)
		goto err_of_put;

	ret = lt9611uxc_regulator_init(lt9611uxc);
	if (ret < 0)
		goto err_of_put;

	lt9611uxc_assert_5v(lt9611uxc);

	ret = lt9611uxc_regulator_enable(lt9611uxc);
	if (ret)
		goto err_of_put;

	lt9611uxc_reset(lt9611uxc);

	ret = lt9611uxc_read_device_rev(lt9611uxc);
	if (ret) {
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	ret = lt9611uxc_read_version(lt9611uxc);
	if (ret < 0) {
		dev_err(dev, "failed to read FW version\n");
		goto err_disable_regulators;
	} else if (ret == 0) {
		dev_err(dev, "FW version 0, FW update not supported\n");
		ret = -EOPNOTSUPP;
		goto err_disable_regulators;
	} else if (ret < 0x40) {
		dev_info(dev, "FW version 0x%x, HPD not supported\n", ret);
	} else {
		lt9611uxc->hpd_supported = true;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					lt9611uxc_irq_thread_handler,
					IRQF_ONESHOT, "lt9611uxc", lt9611uxc);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto err_disable_regulators;
	}

	i2c_set_clientdata(client, lt9611uxc);

	lt9611uxc->bridge.funcs = &lt9611uxc_bridge_funcs;
	lt9611uxc->bridge.of_node = client->dev.of_node;
	lt9611uxc->bridge.ops = DRM_BRIDGE_OP_DETECT;
	if (lt9611uxc->timings)
		lt9611uxc->bridge.ops |= DRM_BRIDGE_OP_MODES;
	else
		lt9611uxc->bridge.ops |= DRM_BRIDGE_OP_EDID;
	if (lt9611uxc->hpd_supported)
		lt9611uxc->bridge.ops |= DRM_BRIDGE_OP_HPD;
	lt9611uxc->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	drm_bridge_add(&lt9611uxc->bridge);

	return lt9611uxc_audio_init(dev, lt9611uxc);

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(lt9611uxc->supplies), lt9611uxc->supplies);

err_of_put:
	of_node_put(lt9611uxc->dsi1_node);
	of_node_put(lt9611uxc->dsi0_node);

	return ret;
}

static int lt9611uxc_remove(struct i2c_client *client)
{
	struct lt9611uxc *lt9611uxc = i2c_get_clientdata(client);

	disable_irq(client->irq);
	lt9611uxc_audio_exit(lt9611uxc);
	drm_bridge_remove(&lt9611uxc->bridge);

	mutex_destroy(&lt9611uxc->ocm_lock);

	regulator_bulk_disable(ARRAY_SIZE(lt9611uxc->supplies), lt9611uxc->supplies);

	of_node_put(lt9611uxc->dsi1_node);
	of_node_put(lt9611uxc->dsi0_node);

	return 0;
}

static struct i2c_device_id lt9611uxc_id[] = {
	{ "lontium,lt9611uxc", 0 },
	{}
};

static const struct of_device_id lt9611uxc_match_table[] = {
	{ .compatible = "lontium,lt9611uxc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt9611uxc_match_table);

static struct i2c_driver lt9611uxc_driver = {
	.driver = {
		.name = "lt9611uxc",
		.of_match_table = lt9611uxc_match_table,
	},
	.probe = lt9611uxc_probe,
	.remove = lt9611uxc_remove,
	.id_table = lt9611uxc_id,
};
module_i2c_driver(lt9611uxc_driver);

MODULE_LICENSE("GPL v2");
