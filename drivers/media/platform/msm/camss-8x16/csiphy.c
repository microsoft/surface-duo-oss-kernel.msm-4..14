/*
 * csiphy.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY Module
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 Linaro Ltd.
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "csiphy.h"
#include "camss.h"

#define MSM_CSIPHY_NAME "msm_csiphy"

#define CAMSS_CSI_PHY_LNn_CFG2(n)		(0x004 + 0x40 * (n))
#define CAMSS_CSI_PHY_LNn_CFG3(n)		(0x008 + 0x40 * (n))
#define CAMSS_CSI_PHY_LNn_MISC1(n)		(0x028 + 0x40 * (n))
#define CAMSS_CSI_PHY_LNn_TEST_IMP(n)		(0x01c + 0x40 * (n))
#define CAMSS_CSI_PHY_GLBL_RESET		0x140
#define CAMSS_CSI_PHY_GLBL_PWR_CFG		0x144
#define CAMSS_CSI_PHY_GLBL_IRQ_CMD		0x164
#define CAMSS_CSI_PHY_HW_VERSION		0x188
#define CAMSS_CSI_PHY_INTERRUPT_STATUSn(n)	(0x18c + 0x4 * (n))
#define CAMSS_CSI_PHY_INTERRUPT_MASKn(n)	(0x1ac + 0x4 * (n))
#define CAMSS_CSI_PHY_INTERRUPT_CLEARn(n)	(0x1cc + 0x4 * (n))
#define CAMSS_CSI_PHY_GLBL_T_INIT_CFG0		0x1ec
#define CAMSS_CSI_PHY_T_WAKEUP_CFG0		0x1f4

/*
 * csiphy_isr - CSIPHY module interrupt handler
 * @irq: Interrupt line
 * @dev: CSIPHY device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csiphy_isr(int irq, void *dev)
{
	struct csiphy_device *csiphy = dev;
	u8 val[8];
	u8 i;

	for (i = 0; i < 8; i++) {
		val[i] = readl_relaxed(csiphy->base +
				       CAMSS_CSI_PHY_INTERRUPT_STATUSn(i));
		writel_relaxed(val[i], csiphy->base +
			       CAMSS_CSI_PHY_INTERRUPT_CLEARn(i));
		writel_relaxed(0x1, csiphy->base + CAMSS_CSI_PHY_GLBL_IRQ_CMD);
		writel_relaxed(0x0, csiphy->base + CAMSS_CSI_PHY_GLBL_IRQ_CMD);
		writel_relaxed(0x0, csiphy->base +
			       CAMSS_CSI_PHY_INTERRUPT_CLEARn(i));
	}

	return IRQ_HANDLED;
}

/*
 * csiphy_reset - Perform software reset on CSIPHY module
 * @csiphy: CSIPHY device
 */
static void csiphy_reset(struct csiphy_device *csiphy)
{
	writel_relaxed(0x1, csiphy->base + CAMSS_CSI_PHY_GLBL_RESET);
	usleep_range(5000, 8000);
	writel_relaxed(0x0, csiphy->base + CAMSS_CSI_PHY_GLBL_RESET);
}

/*
 * csiphy_enable_clocks - Enable clocks for CSIPHY module and
 * set clock rates where needed
 * @csiphy: CSIPHY device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csiphy_enable_clocks(struct csiphy_device *csiphy)
{
	long clk_rate;
	int ret;
	int i;

	for (i = 0; i < csiphy->nclocks; i++) {
		if (csiphy->clock_rate[i]) {
			clk_rate = clk_round_rate(csiphy->clock[i],
						  csiphy->clock_rate[i]);
			if (clk_rate < 0) {
				dev_err(csiphy->camss->dev, "round failed\n");
				ret = clk_rate;
				goto error;
			}
			ret = clk_set_rate(csiphy->clock[i], clk_rate);
			if (ret < 0) {
				dev_err(csiphy->camss->dev, "set rate failed\n");
				goto error;
			}
		}
		ret = clk_prepare_enable(csiphy->clock[i]);
		if (ret) {
			dev_err(csiphy->camss->dev, "clk enable failed\n");
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		clk_disable_unprepare(csiphy->clock[i]);

	return ret;
}

/*
 * csiphy_disable_clocks - Disable clocks for CSIPHY module
 * @csiphy: CSIPHY device
 */
static void csiphy_disable_clocks(struct csiphy_device *csiphy)
{
	int i;

	for (i = csiphy->nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(csiphy->clock[i]);
}

/*
 * csiphy_set_power - Power on/off CSIPHY module
 * @sd: CSIPHY V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csiphy_set_power(struct v4l2_subdev *sd, int on)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	int ret;

	dev_err(csiphy->camss->dev, "%s: Enter, csiphy%d on = %d\n",
		__func__, csiphy->id, on);

	if (on) {
		u8 hw_version;

		ret = csiphy_enable_clocks(csiphy);
		if (ret < 0)
			return ret;

		enable_irq(csiphy->irq);

		csiphy_reset(csiphy);

		hw_version = readl(csiphy->base + CAMSS_CSI_PHY_HW_VERSION);
		dev_err(csiphy->camss->dev, "CSIPHY HW Version = 0x%02x\n",
			hw_version);
	} else {
		disable_irq(csiphy->irq);

		csiphy_disable_clocks(csiphy);
	}

	dev_err(csiphy->camss->dev, "%s: Exit csiphy%d on = %d\n",
		__func__, csiphy->id, on);

	return 0;
}

/*
 * csiphy_get_lane_mask - Calculate CSI2 lane mask configuration parameter
 * @lane_cfg - CSI2 lane configuration
 *
 * Return lane mask
 */
static int csiphy_get_lane_mask(struct camss_csiphy_lanes_cfg *lane_cfg)
{
	u16 lane_mask;
	int i;

	lane_mask = 1 << lane_cfg->clk.pos;

	for (i = 0; i < lane_cfg->num_data; i++)
		lane_mask |= 1 << lane_cfg->data[i].pos;

	return lane_mask;
}

/*
 * csiphy_set_stream - Enable/disable streaming on CSIPHY module
 * @sd: CSIPHY V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of CSIPHY module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csiphy_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct csiphy_config *cfg = &csiphy->cfg;
	u16 lane_mask = csiphy_get_lane_mask(&cfg->csi2->lanecfg);
	u8 i;
	u8 val;

	dev_err(csiphy->camss->dev, "%s: Enter, csiphy%d enable = %d\n",
		__func__, csiphy->id, enable);

	if (enable) {
		val = readl(csiphy->base_clk_mux);
		if (cfg->combo_mode && (lane_mask & 0x18) == 0x18) {
			val &= ~0xf0;
			val |= cfg->csid_id << 4;
		} else {
			val &= ~0xf;
			val |= cfg->csid_id;
		}
		writel_relaxed(val, csiphy->base_clk_mux);

		writel_relaxed(0x1, csiphy->base +
			       CAMSS_CSI_PHY_GLBL_T_INIT_CFG0);
		writel_relaxed(0x1, csiphy->base +
			       CAMSS_CSI_PHY_T_WAKEUP_CFG0);

		val = 0x1;
		val |= lane_mask << 1;
		writel_relaxed(val, csiphy->base + CAMSS_CSI_PHY_GLBL_PWR_CFG);

		val = cfg->combo_mode << 4;
		writel_relaxed(val, csiphy->base + CAMSS_CSI_PHY_GLBL_RESET);

		lane_mask &= 0x1f;
		i = 0;
		while (lane_mask & 0x1f) {
			if (!(lane_mask & 0x1)) {
				i++;
				lane_mask >>= 1;
				continue;
			}

			writel_relaxed(0x10, csiphy->base +
				       CAMSS_CSI_PHY_LNn_CFG2(i));
			writel_relaxed(cfg->csi2->settle_cnt, csiphy->base +
				       CAMSS_CSI_PHY_LNn_CFG3(i));

			writel_relaxed(0x3f, csiphy->base +
				       CAMSS_CSI_PHY_INTERRUPT_MASKn(i));
			writel_relaxed(0x3f, csiphy->base +
				       CAMSS_CSI_PHY_INTERRUPT_CLEARn(i));

			i++;
			lane_mask >>= 1;
		}
	} else {
		i = 0;
		while (lane_mask) {
			if (lane_mask & 0x1) {
				writel_relaxed(0x0, csiphy->base +
					       CAMSS_CSI_PHY_LNn_CFG2(i));
				writel_relaxed(0x0, csiphy->base +
					       CAMSS_CSI_PHY_LNn_MISC1(i));
				writel_relaxed(0x0, csiphy->base +
					       CAMSS_CSI_PHY_LNn_TEST_IMP(i));
			}

			lane_mask >>= 1;
			i++;
		}

		writel_relaxed(0x0, csiphy->base + CAMSS_CSI_PHY_LNn_CFG2(4));
		writel_relaxed(0x0, csiphy->base + CAMSS_CSI_PHY_GLBL_PWR_CFG);
	}

	return 0;
}

/*
 * __csiphy_get_format - Get pointer to format structure
 * @csiphy: CSIPHY device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__csiphy_get_format(struct csiphy_device *csiphy,
		    struct v4l2_subdev_pad_config *cfg,
		    unsigned int pad,
		    enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&csiphy->subdev, cfg, pad);

	return &csiphy->fmt[pad];
}

/*
 * csiphy_get_format - Handle get format by pads subdev method
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csiphy_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __csiphy_get_format(csiphy, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * csiphy_set_format - Handle set format by pads subdev method
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csiphy_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (fmt->pad == MSM_CSIPHY_PAD_SINK) {
		/* Set format on sink pad */
		format = __csiphy_get_format(csiphy, cfg, fmt->pad,
					     fmt->which);
		if (format == NULL)
			return -EINVAL;

		if (fmt->format.field == V4L2_FIELD_ANY)
			fmt->format.field = V4L2_FIELD_NONE;

		*format = fmt->format;

		/* Reset format on source pad */
		format = __csiphy_get_format(csiphy, cfg, MSM_CSIPHY_PAD_SRC,
					     fmt->which);
		if (format == NULL)
			return -EINVAL;

		*format = fmt->format;
	} else {
		/* Source pad format must be same as sink pad format, */
		/* so just return source pad format */
		format = __csiphy_get_format(csiphy, cfg, fmt->pad, fmt->which);
		if (format == NULL)
			return -EINVAL;

		fmt->format = *format;
	}

	return 0;
}

/*
 * msm_csiphy_subdev_init - Initialize CSIPHY device structure and resources
 * @csiphy: CSIPHY device
 * @camss: Camera sub-system structure
 * @res: CSIPHY module resources table
 * @id: CSIPHY module id
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csiphy_subdev_init(struct csiphy_device *csiphy, struct camss *camss,
			   struct resources *res, u8 id)
{
	struct device *dev = camss->dev;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	csiphy->camss = camss;

	dev_err(csiphy->camss->dev, "%s: Enter\n", __func__);

	csiphy->id = id;

	csiphy->cfg.combo_mode = 0;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	csiphy->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(csiphy->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csiphy->base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[1]);
	csiphy->base_clk_mux = devm_ioremap_resource(dev, r);
	if (IS_ERR(csiphy->base_clk_mux)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csiphy->base_clk_mux);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt[0]);
	csiphy->irq = r->start;
	if (IS_ERR_VALUE(csiphy->irq))
		return csiphy->irq;

	ret = devm_request_irq(dev, csiphy->irq, csiphy_isr,
		IRQF_TRIGGER_RISING, dev_name(dev), csiphy);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	disable_irq(csiphy->irq);

	/* Clocks */

	i = 0;
	csiphy->nclocks = 0;
	while (res->clock[i++])
		csiphy->nclocks++;

	csiphy->clock = devm_kzalloc(dev, csiphy->nclocks *
				     sizeof(*csiphy->clock), GFP_KERNEL);
	if (!csiphy->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	csiphy->clock_rate = devm_kzalloc(dev, csiphy->nclocks *
					  sizeof(*csiphy->clock_rate), GFP_KERNEL);
	if (!csiphy->clock_rate) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < csiphy->nclocks; i++) {
		csiphy->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(csiphy->clock[i]))
			return PTR_ERR(csiphy->clock[i]);
		csiphy->clock_rate[i] = res->clock_rate[i];
	}

	dev_err(csiphy->camss->dev, "%s: Exit\n", __func__);

	return 0;
}

/*
 * csiphy_link_setup - Setup CSIPHY connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Rreturn 0 on success
 */
static int csiphy_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SOURCE) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct csiphy_device *csiphy;
		struct csid_device *csid;

		sd = container_of(entity, struct v4l2_subdev, entity);
		csiphy = v4l2_get_subdevdata(sd);

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		csiphy->cfg.csid_id = csid->id;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops csiphy_core_ops = {
	.s_power = csiphy_set_power,
};

static const struct v4l2_subdev_video_ops csiphy_video_ops = {
	.s_stream = csiphy_set_stream,
};

static const struct v4l2_subdev_pad_ops csiphy_pad_ops = {
	.get_fmt = csiphy_get_format,
	.set_fmt = csiphy_set_format,
};

static const struct v4l2_subdev_ops csiphy_v4l2_ops = {
	.core = &csiphy_core_ops,
	.video = &csiphy_video_ops,
	.pad = &csiphy_pad_ops,
};

static const struct v4l2_subdev_internal_ops csiphy_v4l2_internal_ops;

static const struct media_entity_operations csiphy_media_ops = {
	.link_setup = csiphy_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_csiphy_register_entities - Register subdev node for CSIPHY module
 * @csiphy: CSIPHY device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csiphy_register_entities(struct csiphy_device *csiphy,
				 struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &csiphy->subdev;
	struct media_pad *pads = csiphy->pads;
	int ret;

	v4l2_subdev_init(sd, &csiphy_v4l2_ops);
	sd->internal_ops = &csiphy_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_CSIPHY_NAME, csiphy->id);
	v4l2_set_subdevdata(sd, csiphy);

	pads[MSM_CSIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_CSIPHY_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &csiphy_media_ops;
	ret = media_entity_init(&sd->entity, MSM_CSIPHY_PADS_NUM, pads, 0);
	if (ret < 0) {
		pr_err("Fail to init media entity");
		return ret;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		pr_err("Fail to register subdev");
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

/*
 * msm_csiphy_unregister_entities - Unregister CSIPHY module subdev node
 * @csiphy: CSIPHY device
 */
void msm_csiphy_unregister_entities(struct csiphy_device *csiphy)
{
	v4l2_device_unregister_subdev(&csiphy->subdev);
}
