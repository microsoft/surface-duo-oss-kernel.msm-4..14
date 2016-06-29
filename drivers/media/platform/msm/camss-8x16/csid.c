/*
 * csid.c
 *
 * Qualcomm MSM Camera Subsystem - CSID Module
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2016 Linaro Ltd.
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
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "csid.h"
#include "camss.h"

#define MSM_CSID_NAME "msm_csid"

#define CAMSS_CSID_HW_VERSION		0x0
#define CAMSS_CSID_CORE_CTRL_0		0x004
#define CAMSS_CSID_CORE_CTRL_1		0x008
#define CAMSS_CSID_RST_CMD		0x00c
#define CAMSS_CSID_CID_LUT_VC_n(n)	(0x010 + 0x4 * (n))
#define CAMSS_CSID_CID_n_CFG(n)		(0x020 + 0x4 * (n))
#define CAMSS_CSID_IRQ_CLEAR_CMD	0x060
#define CAMSS_CSID_IRQ_MASK		0x064
#define CAMSS_CSID_IRQ_STATUS		0x068
#define CAMSS_CSID_TG_CTRL		0x0a0
#define CAMSS_CSID_TG_VC_CFG		0x0a4
#define CAMSS_CSID_TG_VC_CFG_H_BLANKING		0x3ff
#define CAMSS_CSID_TG_VC_CFG_V_BLANKING		0x7f
#define CAMSS_CSID_TG_DT_n_CGG_0(n)	(0x0ac + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_1(n)	(0x0b0 + 0xc * (n))
#define CAMSS_CSID_TG_DT_n_CGG_2(n)	(0x0b4 + 0xc * (n))

/*
 * csid_isr - CSID module interrupt handler
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq, void *dev)
{
	struct csid_device *csid = dev;
	u32 value;

	value = readl(csid->base + CAMSS_CSID_IRQ_STATUS);
	writel(value, csid->base + CAMSS_CSID_IRQ_CLEAR_CMD);

	if ((value >> 11) & 0x1)
		complete(&csid->reset_complete);

	return IRQ_HANDLED;
}

/*
 * csid_enable_clocks - Enable clocks for CSID module and
 * set clock rates where needed
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_enable_clocks(int nclocks, struct clk **clock, s32 *clock_rate)
{
	long clk_rate;
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
		if (clock_rate[i]) {
			clk_rate = clk_round_rate(clock[i], clock_rate[i]);
			if (clk_rate < 0) {
				pr_err("clock round rate failed\n");
				ret = clk_rate;
				goto error;
			}
			ret = clk_set_rate(clock[i], clk_rate);
			if (ret < 0) {
				pr_err("clock set rate failed\n");
				goto error;
			}
		}
		ret = clk_prepare_enable(clock[i]);
		if (ret) {
			pr_err("clock enable failed\n");
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		clk_disable_unprepare(clock[i]);

	return ret;
}

/*
 * csid_disable_clocks - Disable clocks for CSID module
 * @csid: CSID device
 */
static void csid_disable_clocks(int nclocks, struct clk **clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i]);
}

/*
 * csid_set_power - Power on/off CSID module
 * @sd: CSID V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_power(struct v4l2_subdev *sd, int on)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	int ret;

	dev_err(csid->camss->dev, "%s: Enter, csid%d on = %d\n",
		__func__, csid->id, on);

	if (on) {
		u32 hw_version;

		ret = regulator_enable(csid->vdda);
		if (ret < 0)
			return ret;

		ret = csid_enable_clocks(csid->nclocks, csid->clock,
					 csid->clock_rate);
		if (ret < 0)
			return ret;

		enable_irq(csid->irq);

		hw_version = readl(csid->base + CAMSS_CSID_HW_VERSION);
		dev_err(csid->camss->dev, "CSID HW Version = 0x%08x\n", hw_version);
	} else {
		disable_irq(csid->irq);

		csid_disable_clocks(csid->nclocks, csid->clock);

		ret = regulator_disable(csid->vdda);
		if (ret < 0)
			return ret;
	}

	dev_err(csid->camss->dev, "%s: Exit, csid%d on = %d\n",
		__func__, csid->id, on);

	return 0;
}

#define DATA_TYPE_YUV422_8BIT 0x1e

/*
 * csid_get_data_type - map media but format to data type
 * @fmt media bus format code
 *
 * Return data type code
 */
static u8 csid_get_data_type(u32 fmt)
{
	switch (fmt) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		return DATA_TYPE_YUV422_8BIT;
	}

	return 0;
}

#define DECODE_FORMAT_UNCOMPRESSED_8_BIT 0x1

/*
 * csid_get_decode_format - map media but format to decode format
 * @fmt media bus format code
 *
 * Return decode format code
 */
static u8 csid_get_decode_format(u32 fmt)
{
	switch (fmt) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		return DECODE_FORMAT_UNCOMPRESSED_8_BIT;
	}

	return 0;
}

/*
 * csid_set_stream - Enable/disable streaming on CSID module
 * @sd: CSID V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of CSID module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct csid_testgen_config *tg = &csid->testgen;

	dev_err(csid->camss->dev, "%s: Enter, csid%d enable = %d\n",
		__func__, csid->id, enable);

	if (enable) {
		u8 vc = 0; /* TODO: How to get this from sensor? */
		u8 cid = vc * 4;
		u8 dt, dt_shift, df;
		u32 val;
		int ret;

		ret = v4l2_ctrl_handler_setup(&csid->ctrls);
		if (ret < 0) {
			dev_err(csid->camss->dev,
				"could not sync v4l2 controls\n");
			return ret;
		}

		if (!tg->enabled &&
		    !media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK])) {
			return -ENOLINK;
		}

		/* Reset */
		writel(0x7FFF, csid->base + CAMSS_CSID_RST_CMD);
		wait_for_completion(&csid->reset_complete);

		dt = csid_get_data_type(csid->fmt[MSM_CSID_PAD_SRC].code);

		if (tg->enabled) {
			/* Config Test Generator */
			u32 num_bytes_per_line =
					csid->fmt[MSM_CSID_PAD_SRC].width * 2;
			u32 num_lines = csid->fmt[MSM_CSID_PAD_SRC].height;

			/* 31:24 V blank, 23:13 H blank, 3:2 num of active DT */
			/* 1:0 VC */
			val = ((CAMSS_CSID_TG_VC_CFG_V_BLANKING & 0xff) << 24) |
			      ((CAMSS_CSID_TG_VC_CFG_H_BLANKING & 0x7ff) << 13);
			writel(val, csid->base + CAMSS_CSID_TG_VC_CFG);

			/* 28:16 bytes per lines, 12:0 num of lines */
			val = ((num_bytes_per_line & 0x1FFF) << 16) |
			      (num_lines & 0x1FFF);
			writel(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_0(0));

			/* 5:0 data type */
			val = dt;
			writel(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_1(0));

			/* 2:0 output random */
			val = tg->payload_mode;
			writel(val, csid->base + CAMSS_CSID_TG_DT_n_CGG_2(0));
		} else {
			struct csid_phy_config *phy = &csid->phy;

			val = phy->lane_cnt - 1;
			val |= phy->lane_assign << 4;

			writel(val, csid->base + CAMSS_CSID_CORE_CTRL_0);

			val = phy->csiphy_id << 17;
			val |= 0x9;

			writel(val, csid->base + CAMSS_CSID_CORE_CTRL_1);
		}

		/* Config LUT */

		dt_shift = (cid % 4) * 8;
		df = csid_get_decode_format(csid->fmt[MSM_CSID_PAD_SINK].code);

		val = readl(csid->base + CAMSS_CSID_CID_LUT_VC_n(vc));
		val &= ~(0xff << dt_shift);
		val |= dt << dt_shift;
		writel(val, csid->base + CAMSS_CSID_CID_LUT_VC_n(vc));

		val = (df << 4) | 0x3;
		writel(val, csid->base + CAMSS_CSID_CID_n_CFG(cid));

		if (tg->enabled) {
			val = 0x00a06437;
			writel(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	} else {
		if (tg->enabled) {
			u32 val = 0x00a06436;
			writel(val, csid->base + CAMSS_CSID_TG_CTRL);
		}
	}

	return 0;
}

/*
 * __csid_get_format - Get pointer to format structure
 * @csid: CSID device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__csid_get_format(struct csid_device *csid,
		  struct v4l2_subdev_pad_config *cfg,
		  unsigned int pad,
		  enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&csid->subdev, cfg, pad);

	return &csid->fmt[pad];
}

/*
 * csid_get_format - Handle get format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_get_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csid_get_format(csid, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * csid_set_format - Handle set format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_set_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad == MSM_CSID_PAD_SINK) {
		/* Set format on sink pad */
		format = __csid_get_format(csid, cfg, fmt->pad,
					   fmt->which);
		if (format == NULL)
			return -EINVAL;

		if (fmt->format.field == V4L2_FIELD_ANY)
			fmt->format.field = V4L2_FIELD_NONE;

		*format = fmt->format;

		/* Reset format on source pad */
		format = __csid_get_format(csid, cfg, MSM_CSID_PAD_SRC,
					   fmt->which);
		if (format == NULL)
			return -EINVAL;

		*format = fmt->format;
	} else {
		if (media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK])) {
			/* CSID is linked to CSIPHY */
			/* Reset format on source pad to sink pad format */

			format = __csid_get_format(csid, cfg, MSM_CSID_PAD_SINK,
						   fmt->which);
			if (format == NULL)
				return -EINVAL;

			fmt->format = *format;

			format = __csid_get_format(csid, cfg, fmt->pad,
						   fmt->which);
			if (format == NULL)
				return -EINVAL;

			*format = fmt->format;
		} else {
			/* CSID is not linked to CSIPHY */
			/* Set format on source pad to allow */
			/* test generator usage */

			format = __csid_get_format(csid, cfg, fmt->pad,
						   fmt->which);
			if (format == NULL)
				return -EINVAL;

			/* Accept only YUV422 format */
			fmt->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
			fmt->format.field = V4L2_FIELD_NONE;

			*format = fmt->format;
		}
	}

	return 0;
}

static const char * const csid_test_pattern_menu[] = {
	"Disabled",
	"Incrementing",
	"Alternating 55/AA",
	"All Zeros",
	"All Ones",
	"Random Data",
};

static int csid_set_test_pattern(struct csid_device *csid, s32 value)
{
	struct csid_testgen_config *tg = &csid->testgen;

	/* If CSID is linked to CSIPHY, do not allow to enable test generator */
	if (value && media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK]))
		return -EBUSY;

	tg->enabled = !!value;

	switch (value) {
	case 1:
		tg->payload_mode = CSID_PAYLOAD_MODE_INCREMENTING;
		break;
	case 2:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALTERNATING_55_AA;
		break;
	case 3:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ZEROES;
		break;
	case 4:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ONES;
		break;
	case 5:
		tg->payload_mode = CSID_PAYLOAD_MODE_RANDOM;
		break;
	}

	return 0;
}

static int csid_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct csid_device *csid = container_of(ctrl->handler,
						struct csid_device, ctrls);
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ret = csid_set_test_pattern(csid, ctrl->val);
		break;
	}

	return ret;
}

static struct v4l2_ctrl_ops csid_ctrl_ops = {
	.s_ctrl = csid_s_ctrl,
};

/*
 * msm_csid_subdev_init - Initialize CSID device structure and resources
 * @csid: CSID device
 * @camss: Camera sub-system structure
 * @res: CSID module resources table
 * @id: CSID module id
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_subdev_init(struct csid_device *csid, struct camss *camss,
			 struct resources *res, u8 id)
{
	struct device *dev = camss->dev;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	csid->camss = camss;

	csid->id = id;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	csid->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(csid->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csid->base);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt[0]);
	csid->irq = r->start;
	if (IS_ERR_VALUE(csid->irq))
		return csid->irq;

	ret = devm_request_irq(dev, csid->irq, csid_isr,
		IRQF_TRIGGER_RISING, dev_name(dev), csid);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	disable_irq(csid->irq);

	/* Clocks */

	i = 0;
	csid->nclocks = 0;
	while (res->clock[i++])
		csid->nclocks++;

	csid->clock = devm_kzalloc(dev, csid->nclocks * sizeof(*csid->clock),
				    GFP_KERNEL);
	if (!csid->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	csid->clock_rate = devm_kzalloc(dev, csid->nclocks *
					sizeof(*csid->clock_rate), GFP_KERNEL);
	if (!csid->clock_rate) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < csid->nclocks; i++) {
		csid->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(csid->clock[i]))
			return PTR_ERR(csid->clock[i]);
		csid->clock_rate[i] = res->clock_rate[i];
	}

	/* Regulator */

	csid->vdda = devm_regulator_get(dev, res->regulator[0]);
	if (IS_ERR(csid->vdda)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(csid->vdda);
	}

	init_completion(&csid->reset_complete);

	return 0;
}

/*
 * csid_get_lane_assign - Calculate CSI2 lane assign configuration parameter
 * @lane_cfg - CSI2 lane configuration
 *
 * Return lane assign
 */
static u32 csid_get_lane_assign(struct camss_csiphy_lanes_cfg *lanecfg)
{
	u32 lane_assign = 0;
	int i;

	for (i = 0; i < lanecfg->num_data; i++)
		lane_assign |= lanecfg->data[i].pos << (i * 4);

	return lane_assign;
}

/*
 * csid_link_setup - Setup CSID connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Rreturn 0 on success
 */
static int csid_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SINK) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct csid_device *csid;
		struct csiphy_device *csiphy;
		struct camss_csiphy_lanes_cfg *lanecfg;

		sd = container_of(entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		/* If test generator is enabled
		 * do not allow a link from CSIPHY to CSID */
		if (csid->testgen_mode->cur.val != 0)
			return -EBUSY;

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csiphy = v4l2_get_subdevdata(sd);

		/* If a sensor is not linked to CSIPHY
		 * do no allow a link from CSIPHY to CSID */
		if (!csiphy->cfg.csi2)
			return -EPERM;

		csid->phy.csiphy_id = csiphy->id;

		lanecfg = &csiphy->cfg.csi2->lanecfg;
		csid->phy.lane_cnt = lanecfg->num_data;
		csid->phy.lane_assign = csid_get_lane_assign(lanecfg);
	}

	return 0;
}

static const struct v4l2_subdev_core_ops csid_core_ops = {
	.s_power = csid_set_power,
};

static const struct v4l2_subdev_video_ops csid_video_ops = {
	.s_stream = csid_set_stream,
};

static const struct v4l2_subdev_pad_ops csid_pad_ops = {
	.get_fmt = csid_get_format,
	.set_fmt = csid_set_format,
};

static const struct v4l2_subdev_ops csid_v4l2_ops = {
	.core = &csid_core_ops,
	.video = &csid_video_ops,
	.pad = &csid_pad_ops,
};

static const struct v4l2_subdev_internal_ops csid_v4l2_internal_ops;

static const struct media_entity_operations csid_media_ops = {
	.link_setup = csid_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_csid_register_entities - Register subdev node for CSID module
 * @csid: CSID device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_register_entities(struct csid_device *csid,
			       struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &csid->subdev;
	struct media_pad *pads = csid->pads;
	int ret;

	v4l2_subdev_init(sd, &csid_v4l2_ops);
	sd->internal_ops = &csid_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_CSID_NAME, csid->id);
	v4l2_set_subdevdata(sd, csid);

	v4l2_ctrl_handler_init(&csid->ctrls, 1);
	csid->testgen_mode = v4l2_ctrl_new_std_menu_items(&csid->ctrls,
				&csid_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(csid_test_pattern_menu) - 1, 0, 0,
				csid_test_pattern_menu);

	if (csid->ctrls.error) {
		dev_err(csid->camss->dev, "failed to init ctrl: %d\n",
			csid->ctrls.error);
		ret = csid->ctrls.error;
		goto free_ctrl;
	}

	csid->subdev.ctrl_handler = &csid->ctrls;

	pads[MSM_CSID_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_CSID_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &csid_media_ops;
	ret = media_entity_init(&sd->entity, MSM_CSID_PADS_NUM, pads, 0);
	if (ret < 0) {
		dev_err(csid->camss->dev, "failed to init media entity");
		goto free_ctrl;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(csid->camss->dev, "failed to register subdev");
		goto media_cleanup;
	}

	return 0;

media_cleanup:
	media_entity_cleanup(&sd->entity);
free_ctrl:
	v4l2_ctrl_handler_free(&csid->ctrls);

	return ret;
}

/*
 * msm_csid_unregister_entities - Unregister CSID module subdev node
 * @csid: CSID device
 */
void msm_csid_unregister_entities(struct csid_device *csid)
{
	v4l2_device_unregister_subdev(&csid->subdev);
	v4l2_ctrl_handler_free(&csid->ctrls);
}
