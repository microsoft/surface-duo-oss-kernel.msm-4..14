/*
 * ispif.c
 *
 * Qualcomm MSM Camera Subsystem - ISPIF Module
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ispif.h"
#include "camss.h"

#define MSM_ISPIF_NAME "msm_ispif"

#define ISPIF_RST_CMD_0			0x008
#define ISPIF_IRQ_GLOBAL_CLEAR_CMD	0x01c
#define ISPIF_VFE_m_CTRL_0(m)		(0x200 + 0x200 * (m))
#define ISPIF_VFE_m_CTRL_0_PIX0_LINE_BUF_EN	(1 << 6)
#define ISPIF_VFE_m_IRQ_MASK_0(m)	(0x208 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_0_ENABLE   0x0a493249
#define ISPIF_VFE_m_IRQ_MASK_1(m)	(0x20c + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_1_ENABLE   0x02493249
#define ISPIF_VFE_m_IRQ_MASK_2(m)	(0x210 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_2_ENABLE   0x00001249
#define ISPIF_VFE_m_IRQ_STATUS_0(m)	(0x21c + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_STATUS_1(m)	(0x220 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_STATUS_2(m)	(0x224 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_CLEAR_0(m)	(0x230 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_CLEAR_1(m)	(0x234 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_CLEAR_2(m)	(0x238 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_INPUT_SEL(m)	(0x244 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_CMD_0(m)	(0x248 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_CMD_1(m)	(0x24c + 0x200 * (m))
#define ISPIF_VFE_m_PIX_INTF_n_CID_MASK(m, n)	(0x254 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_RDI_INTF_n_CID_MASK(m, n)	(0x264 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_PIX_INTF_n_STATUS(m, n)	(0x2c0 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_RDI_INTF_n_STATUS(m, n)	(0x2d0 + 0x200 * (m) + 0x4 * (n))

#define CSI_RDI_CLK_MUX_SEL		0x008

#define ISPIF_TIMEOUT_SLEEP_US		1000
#define ISPIF_TIMEOUT_ALL_US		1000000

enum ispif_intf {
	PIX0,
	RDI0,
	PIX1,
	RDI1,
	RDI2
};

enum ispif_intf_cmd {
	CMD_DISABLE_FRAME_BOUNDARY = 0x0,
	CMD_ENABLE_FRAME_BOUNDARY = 0x1,
	CMD_DISABLE_IMMEDIATELY = 0x2,
	CMD_ALL_DISABLE_IMMEDIATELY = 0xaaaaaaaa,
	CMD_ALL_NO_CHANGE = 0xffffffff,
};

/*
 * ispif_isr - ISPIF module interrupt handler
 * @irq: Interrupt line
 * @dev: ISPIF device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t ispif_isr(int irq, void *dev)
{
	struct ispif_device *ispif = dev;
	u32 value0, value1, value2;

	value0 = readl(ispif->base + ISPIF_VFE_m_IRQ_STATUS_0(0));
	value1 = readl(ispif->base + ISPIF_VFE_m_IRQ_STATUS_1(0));
	value2 = readl(ispif->base + ISPIF_VFE_m_IRQ_STATUS_2(0));

	writel(value0, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(0));
	writel(value1, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(0));
	writel(value2, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(0));

	wmb();
	writel(0x1, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD);
	wmb();

	if ((value0 >> 27) & 0x1)
		complete(&ispif->reset_complete);

	return IRQ_HANDLED;
}

/*
 * ispif_enable_clocks - Enable clocks for ISPIF module
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_enable_clocks(int nclocks, struct clk **clock,
			       u8 *clock_for_reset, u8 reset)
{
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
		if (clock_for_reset[i] == reset) {
			ret = clk_prepare_enable(clock[i]);
			if (ret) {
				pr_err("clock enable failed\n");
				goto error;
			}
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		if (clock_for_reset[i] == reset)
			clk_disable_unprepare(clock[i]);

	return ret;
}

/*
 * ispif_disable_clocks - Disable clocks for ISPIF module
 */
static void ispif_disable_clocks(int nclocks, struct clk **clock,
				 u8 *clock_for_reset, u8 reset)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		if (clock_for_reset[i] == reset)
			clk_disable_unprepare(clock[i]);
}

/*
 * ispif_set_power - Power on/off ISPIF module
 * @sd: ISPIF V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_set_power(struct v4l2_subdev *sd, int on)
{
	struct ispif_device *ispif = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_err(ispif->camss->dev, "%s: Enter, on = %d\n",
		__func__, on);

	if (on)
		ret = ispif_enable_clocks(ispif->nclocks, ispif->clock,
					  ispif->clock_for_reset, 0);
	else
		ispif_disable_clocks(ispif->nclocks, ispif->clock,
				     ispif->clock_for_reset, 0);

	dev_err(ispif->camss->dev, "%s: Exit, on = %d\n",
		__func__, on);

	return ret;
}

static int ispif_reset(struct ispif_device *ispif)
{
	int ret;

	ret = ispif_enable_clocks(ispif->nclocks, ispif->clock,
				  ispif->clock_for_reset, 1);
	if (ret < 0)
		goto exit;

	writel(0xfe0f1fff, ispif->base + ISPIF_RST_CMD_0);
	wait_for_completion(&ispif->reset_complete);

	ispif_disable_clocks(ispif->nclocks, ispif->clock,
			     ispif->clock_for_reset, 1);

exit:
	return ret;
}

static void ispif_reset_sw(struct ispif_device *ispif, u8 vfe)
{

	writel_relaxed(ISPIF_VFE_m_CTRL_0_PIX0_LINE_BUF_EN,
		       ispif->base + ISPIF_VFE_m_CTRL_0(vfe));
	writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
	writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
	writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
	writel_relaxed(0xffffffff, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
	writel_relaxed(0xffffffff, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
	writel_relaxed(0xffffffff, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(vfe));

	writel_relaxed(0, ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));

	writel_relaxed(CMD_ALL_NO_CHANGE,
		       ispif->base + ISPIF_VFE_m_INTF_CMD_0(vfe));
	writel_relaxed(CMD_ALL_NO_CHANGE,
		       ispif->base + ISPIF_VFE_m_INTF_CMD_1(vfe));

	writel_relaxed(0,
		       ispif->base + ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 0));
	writel_relaxed(0,
		       ispif->base + ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 1));
	writel_relaxed(0,
		       ispif->base + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 0));
	writel_relaxed(0,
		       ispif->base + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 1));
	writel_relaxed(0,
		       ispif->base + ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 2));

	wmb();
	writel_relaxed(0x1, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD);
	wmb();
}

static void ispif_select_clk_mux(struct ispif_device *ispif,
				 enum ispif_intf intf, u8 csid, u8 vfe)
{
	u32 val = 0;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base_clk_mux);
		val &= ~(0xf << (vfe * 8));
		val |= (csid << (vfe * 8));
		writel_relaxed(val, ispif->base_clk_mux);
		break;

	case RDI0:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (vfe * 12));
		val |= (csid << (vfe * 12));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case PIX1:
		val = readl_relaxed(ispif->base_clk_mux);
		val &= ~(0xf << (4 + (vfe * 8)));
		val |= (csid << (4 + (vfe * 8)));
		writel_relaxed(val, ispif->base_clk_mux);
		break;

	case RDI1:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (4 + (vfe * 12)));
		val |= (csid << (4 + (vfe * 12)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case RDI2:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (8 + (vfe * 12)));
		val |= (csid << (8 + (vfe * 12)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;
	}

	mb();
}

static int ispif_validate_intf_status(struct ispif_device *ispif,
				      enum ispif_intf intf, u8 vfe)
{
	int ret = 0;
	u32 val;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 0));
		break;
	case RDI0:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 0));
		break;
	case PIX1:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 1));
		break;
	case RDI1:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 1));
		break;
	case RDI2:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 2));
		break;
	}

	if ((val & 0xf) != 0xf)
		ret = -EBUSY;

	return ret;
}

static void ispif_select_csid(struct ispif_device *ispif,
			      enum ispif_intf intf, u8 csid, u8 vfe)
{
	u32 val;

	val = readl_relaxed(ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
	switch (intf) {
	case PIX0:
		val &= ~(BIT(1) | BIT(0));
		val |= csid;
		break;
	case RDI0:
		val &= ~(BIT(5) | BIT(4));
		val |= (csid << 4);
		break;
	case PIX1:
		val &= ~(BIT(9) | BIT(8));
		val |= (csid << 8);
		break;
	case RDI1:
		val &= ~(BIT(13) | BIT(12));
		val |= (csid << 12);
		break;
	case RDI2:
		val &= ~(BIT(21) | BIT(20));
		val |= (csid << 20);
		break;
	}

	wmb();
	writel_relaxed(val, ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
	wmb();
}

static void ispif_enable_cid(struct ispif_device *ispif, enum ispif_intf intf,
			     u16 cid_mask, u8 vfe, u8 enable)
{
	u32 addr, val;

	switch (intf) {
	case PIX0:
		addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 0);
		break;
	case RDI0:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 0);
		break;
	case PIX1:
		addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 1);
		break;
	case RDI1:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 1);
		break;
	case RDI2:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 2);
		break;
	}

	val = readl_relaxed(ispif->base + addr);
	if (enable)
		val |= cid_mask;
	else
		val &= ~cid_mask;

	wmb();
	writel_relaxed(val, ispif->base + addr);
	wmb();
}

static void ispif_config_irq(struct ispif_device *ispif, u8 vfe)
{
	u32 val;

	val = ISPIF_VFE_m_IRQ_MASK_0_ENABLE;
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
	val = ISPIF_VFE_m_IRQ_MASK_1_ENABLE;
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
	val = ISPIF_VFE_m_IRQ_MASK_2_ENABLE;
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
	writel(val, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(vfe));
	wmb();
	writel(0x1, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD);
	wmb();
}

static void ispif_intf_cmd(struct ispif_device *ispif, u8 cmd,
			   enum ispif_intf intf, u8 vfe, u8 vc)
{
	u32 val = CMD_ALL_NO_CHANGE;

	if (intf == RDI2) {
		val &= ~(0x3 << (vc * 2 + 8));
		val |= (cmd << (vc * 2 + 8));
		wmb();
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_INTF_CMD_1(vfe));
		wmb();
	} else {
		val &= ~(0x3 << (vc * 2 + intf * 8));
		val |= (cmd << (vc * 2 + intf * 8));
		wmb();
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_INTF_CMD_0(vfe));
		wmb();
	}
}

/*
 * ispif_set_stream - Enable/disable streaming on ISPIF module
 * @sd: ISPIF V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of ISPIF module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ispif_device *ispif = v4l2_get_subdevdata(sd);
	enum ispif_intf ispif_intf = RDI0;
	u8 vfe = 0;
	u8 vc = 0; /* TODO: How to get this from sensor? */
	u8 cid = vc * 4;

	int ret;

	dev_err(ispif->camss->dev, "%s: Enter, enable = %d\n",
		__func__, enable);

	if (enable) {
		u8 csid = ispif->csid_id;

		if (!media_entity_remote_pad(
					&ispif->pads[MSM_ISPIF_PAD_SINK])) {
			return -ENOLINK;
		}

		/* Reset */

		ret = ispif_reset(ispif);
		if (ret < 0)
			return ret;

		/* Config */

		ispif_reset_sw(ispif, vfe);

		ispif_select_clk_mux(ispif, ispif_intf, csid, vfe);

		ret = ispif_validate_intf_status(ispif, ispif_intf, vfe);
		if (ret < 0)
			return ret;

		ispif_select_csid(ispif, ispif_intf, csid, vfe);

		ispif_enable_cid(ispif, ispif_intf, 1 << cid, vfe, 1);

		ispif_config_irq(ispif, vfe);

		ispif_intf_cmd(ispif, CMD_ENABLE_FRAME_BOUNDARY, ispif_intf, vfe, vc);
	} else {
		u32 stop_flag = 0;

		ispif_intf_cmd(ispif, CMD_DISABLE_FRAME_BOUNDARY, ispif_intf, vfe, vc);

		ret = readl_poll_timeout(ispif->base + ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 0),
					 stop_flag,
					 (stop_flag & 0xf) == 0xf,
					 ISPIF_TIMEOUT_SLEEP_US,
					 ISPIF_TIMEOUT_ALL_US);
		if (ret < 0)
			return ret;

		ispif_enable_cid(ispif, ispif_intf, 1 << cid, vfe, 0);
	}

	return 0;
}

/*
 * msm_ispif_subdev_init - Initialize ISPIF device structure and resources
 * @ispif: ISPIF device
 * @camss: Camera sub-system structure
 * @res: ISPIF module resources table
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_ispif_subdev_init(struct ispif_device *ispif, struct camss *camss,
			  struct resources_ispif *res)
{
	struct device *dev = camss->dev;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	ispif->camss = camss;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	ispif->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(ispif->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(ispif->base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[1]);
	ispif->base_clk_mux = devm_ioremap_resource(dev, r);
	if (IS_ERR(ispif->base_clk_mux)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(ispif->base_clk_mux);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt);
	ispif->irq = r->start;
	if (IS_ERR_VALUE(ispif->irq))
		return ispif->irq;

	ret = devm_request_irq(dev, ispif->irq, ispif_isr,
			       IRQF_TRIGGER_RISING, "ispif", ispif);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	/* Clocks */

	i = 0;
	ispif->nclocks = 0;
	while (res->clock[i++])
		ispif->nclocks++;

	ispif->clock = devm_kzalloc(dev, ispif->nclocks * sizeof(*ispif->clock),
				    GFP_KERNEL);
	if (!ispif->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	ispif->clock_for_reset = devm_kzalloc(dev, ispif->nclocks *
			sizeof(*ispif->clock_for_reset), GFP_KERNEL);
	if (!ispif->clock_for_reset) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < ispif->nclocks; i++) {
		ispif->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(ispif->clock[i]))
			return PTR_ERR(ispif->clock[i]);
		ispif->clock_for_reset[i] = res->clock_for_reset[i];
	}

	init_completion(&ispif->reset_complete);

	return 0;
}

static int ispif_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SINK) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct ispif_device *ispif;
		struct csid_device *csid;

		sd = container_of(entity, struct v4l2_subdev, entity);
		ispif = v4l2_get_subdevdata(sd);

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		ispif->csid_id = csid->id;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ispif_core_ops = {
	.s_power = ispif_set_power,
};

static const struct v4l2_subdev_video_ops ispif_video_ops = {
	.s_stream = ispif_set_stream,
};

static const struct v4l2_subdev_pad_ops ispif_pad_ops;

static const struct v4l2_subdev_ops ispif_v4l2_ops = {
	.core = &ispif_core_ops,
	.video = &ispif_video_ops,
	.pad = &ispif_pad_ops,
};

static const struct v4l2_subdev_internal_ops ispif_v4l2_internal_ops;

static const struct media_entity_operations ispif_media_ops = {
	.link_setup = ispif_link_setup,
};

/*
 * msm_ispif_register_entities - Register subdev node for ISPIF module
 * @ispif: ISPIF device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_ispif_register_entities(struct ispif_device *ispif,
				struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &ispif->subdev;
	struct media_pad *pads = ispif->pads;
	int ret;

	v4l2_subdev_init(sd, &ispif_v4l2_ops);
	sd->internal_ops = &ispif_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), MSM_ISPIF_NAME);
	v4l2_set_subdevdata(sd, ispif);

	pads[MSM_ISPIF_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_ISPIF_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &ispif_media_ops;
	ret = media_entity_init(&sd->entity, MSM_ISPIF_PADS_NUM, pads, 0);
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
 * msm_ispif_unregister_entities - Unregister ISPIF module subdev node
 * @ispif: ISPIF device
 */
void msm_ispif_unregister_entities(struct ispif_device *ispif)
{
	v4l2_device_unregister_subdev(&ispif->subdev);
}
