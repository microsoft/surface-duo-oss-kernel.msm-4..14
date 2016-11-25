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
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ispif.h"
#include "camss.h"

#define MSM_ISPIF_NAME "msm_ispif"

#define ispif_line_array(ptr_line)	\
	((const struct ispif_line (*)[]) &(ptr_line[-(ptr_line->id)]))

#define to_ispif(ptr_line)	\
	container_of(ispif_line_array(ptr_line), struct ispif_device, ptr_line)

#define ISPIF_RST_CMD_0			0x008
#define ISPIF_IRQ_GLOBAL_CLEAR_CMD	0x01c
#define ISPIF_VFE_m_CTRL_0(m)		(0x200 + 0x200 * (m))
#define ISPIF_VFE_m_CTRL_0_PIX0_LINE_BUF_EN	(1 << 6)
#define ISPIF_VFE_m_IRQ_MASK_0(m)	(0x208 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_0_PIX0_MASK	0x00001fff
#define ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE	0x02492000
#define ISPIF_VFE_m_IRQ_MASK_0_RDI0_MASK	0x03ffe000
#define ISPIF_VFE_m_IRQ_MASK_1(m)	(0x20c + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_1_PIX1_MASK	0x00001fff
#define ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE	0x02492000
#define ISPIF_VFE_m_IRQ_MASK_1_RDI1_MASK	0x03ffe000
#define ISPIF_VFE_m_IRQ_MASK_2(m)	(0x210 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_2_RDI2_MASK	0x00001fff
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

enum ispif_intf_cmd {
	CMD_DISABLE_FRAME_BOUNDARY = 0x0,
	CMD_ENABLE_FRAME_BOUNDARY = 0x1,
	CMD_DISABLE_IMMEDIATELY = 0x2,
	CMD_ALL_DISABLE_IMMEDIATELY = 0xaaaaaaaa,
	CMD_ALL_NO_CHANGE = 0xffffffff,
};

static const u32 ispif_formats[] = {
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_VYUY8_2X8,
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_YVYU8_2X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
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

	value0 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_0(0));
	value1 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_1(0));
	value2 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_2(0));

	writel_relaxed(value0, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(0));
	writel_relaxed(value1, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(0));
	writel_relaxed(value2, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(0));

	writel(0x1, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD);

	if ((value0 >> 27) & 0x1)
		complete(&ispif->reset_complete);

	return IRQ_HANDLED;
}

/*
 * ispif_enable_clocks - Enable clocks for ISPIF module
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_enable_clocks(int nclocks, struct clk **clock)
{
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
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
 * ispif_disable_clocks - Disable clocks for ISPIF module
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 */
static void ispif_disable_clocks(int nclocks, struct clk **clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i]);
}

static int ispif_reset(struct ispif_device *ispif)
{
	int ret;

	ret = ispif_enable_clocks(ispif->nclocks_for_reset,
				  ispif->clock_for_reset);
	if (ret < 0)
		goto exit;

	writel_relaxed(0x000f1fff, ispif->base + ISPIF_RST_CMD_0);
	wait_for_completion(&ispif->reset_complete);

	ispif_disable_clocks(ispif->nclocks_for_reset, ispif->clock_for_reset);

exit:
	return ret;
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
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct ispif_device *ispif = to_ispif(line);
	int ret = 0;

	dev_dbg(to_device(ispif), "%s: Enter, ispif%d on = %d\n",
		__func__, line->id, on);

	if (on) {
		mutex_lock(&ispif->power_lock);
		if (ispif->power_count) {
			/* Power is already on */
			ispif->power_count++;
			goto exit;
		}

		ret = ispif_enable_clocks(ispif->nclocks, ispif->clock);
		if (ret < 0)
			goto exit;

		ret = ispif_reset(ispif);
		if (ret < 0) {
			ispif_disable_clocks(ispif->nclocks, ispif->clock);
			goto exit;
		}

		ispif->intf_cmd[0].cmd_0 = CMD_ALL_NO_CHANGE;
		ispif->intf_cmd[0].cmd_1 = CMD_ALL_NO_CHANGE;

		ispif->power_count++;
	} else {
		mutex_lock(&ispif->power_lock);
		if (ispif->power_count == 0) {
			dev_err(to_device(ispif),
				"%s: set_power off on power_count == 0 !!!\n",
				__func__);
			goto exit;
		} else if (ispif->power_count == 1) {
			ispif_disable_clocks(ispif->nclocks, ispif->clock);
		}

		ispif->power_count--;
	}

exit:
	mutex_unlock(&ispif->power_lock);

	dev_dbg(to_device(ispif), "%s: Exit, ispif%d on = %d\n",
		__func__, line->id, on);

	return ret;
}

static void ispif_select_clk_mux(struct ispif_device *ispif,
				 enum ispif_intf intf, u8 csid,
				 u8 vfe, u8 enable)
{
	u32 val = 0;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base_clk_mux);
		val &= ~(0xf << (vfe * 8));
		if (enable)
			val |= (csid << (vfe * 8));
		writel_relaxed(val, ispif->base_clk_mux);
		break;

	case RDI0:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (vfe * 12));
		if (enable)
			val |= (csid << (vfe * 12));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case PIX1:
		val = readl_relaxed(ispif->base_clk_mux);
		val &= ~(0xf << (4 + (vfe * 8)));
		if (enable)
			val |= (csid << (4 + (vfe * 8)));
		writel_relaxed(val, ispif->base_clk_mux);
		break;

	case RDI1:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (4 + (vfe * 12)));
		if (enable)
			val |= (csid << (4 + (vfe * 12)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case RDI2:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (8 + (vfe * 12)));
		if (enable)
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
	u32 val = 0;

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

	if ((val & 0xf) != 0xf) {
		dev_err(to_device(ispif), "%s: ispif is busy: 0x%x\n",
			__func__, val);
		ret = -EBUSY;
	}

	return ret;
}

static int ispif_wait_for_stop(struct ispif_device *ispif,
			       enum ispif_intf intf, u8 vfe)
{
	int ret;
	u32 addr = 0;
	u32 stop_flag = 0;

	switch (intf) {
	case PIX0:
		addr = ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 0);
		break;
	case RDI0:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 0);
		break;
	case PIX1:
		addr = ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 1);
		break;
	case RDI1:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 1);
		break;
	case RDI2:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 2);
		break;
	}

	ret = readl_poll_timeout(ispif->base + addr,
				 stop_flag,
				 (stop_flag & 0xf) == 0xf,
				 ISPIF_TIMEOUT_SLEEP_US,
				 ISPIF_TIMEOUT_ALL_US);

	if (ret < 0)
		dev_err(to_device(ispif), "%s: ispif stop timeout\n",
			__func__);

	return ret;
}

static void ispif_select_csid(struct ispif_device *ispif, enum ispif_intf intf,
			      u8 csid, u8 vfe, u8 enable)
{
	u32 val;

	val = readl_relaxed(ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
	switch (intf) {
	case PIX0:
		val &= ~(BIT(1) | BIT(0));
		if (enable)
			val |= csid;
		break;
	case RDI0:
		val &= ~(BIT(5) | BIT(4));
		if (enable)
			val |= (csid << 4);
		break;
	case PIX1:
		val &= ~(BIT(9) | BIT(8));
		if (enable)
			val |= (csid << 8);
		break;
	case RDI1:
		val &= ~(BIT(13) | BIT(12));
		if (enable)
			val |= (csid << 12);
		break;
	case RDI2:
		val &= ~(BIT(21) | BIT(20));
		if (enable)
			val |= (csid << 20);
		break;
	}

	writel(val, ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
}

static void ispif_enable_cid(struct ispif_device *ispif, enum ispif_intf intf,
			     u16 cid_mask, u8 vfe, u8 enable)
{
	u32 addr = 0, val;

	switch (intf) {
	case PIX0:
		addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 0);
		break;
	case RDI0:
	default:
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

	writel(val, ispif->base + addr);
}

static void ispif_config_irq(struct ispif_device *ispif, enum ispif_intf intf,
			     u8 vfe, u8 enable)
{
	u32 val;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_0_PIX0_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
		break;
	case RDI0:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_0_RDI0_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
		break;
	case PIX1:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_1_PIX1_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
		break;
	case RDI1:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_1_RDI1_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
		break;
	case RDI2:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_2_RDI2_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(vfe));
		break;
	}

	writel(0x1, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD);
}

static void ispif_set_intf_cmd(struct ispif_device *ispif, u8 cmd,
			       enum ispif_intf intf, u8 vfe, u8 vc)
{
	u32 *val;

	if (intf == RDI2) {
		val = &ispif->intf_cmd[0].cmd_1;
		*val &= ~(0x3 << (vc * 2 + 8));
		*val |= (cmd << (vc * 2 + 8));
		wmb();
		writel_relaxed(*val, ispif->base + ISPIF_VFE_m_INTF_CMD_1(vfe));
		wmb();
	} else {
		val = &ispif->intf_cmd[0].cmd_0;
		*val &= ~(0x3 << (vc * 2 + intf * 8));
		*val |= (cmd << (vc * 2 + intf * 8));
		wmb();
		writel_relaxed(*val, ispif->base + ISPIF_VFE_m_INTF_CMD_0(vfe));
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
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct ispif_device *ispif = to_ispif(line);
	enum ispif_intf intf = line->interface;
	u8 csid = line->csid_id;
	u8 vfe = line->vfe_id;
	u8 vc = 0; /* TODO: How to get this from sensor? */
	u8 cid = vc * 4;

	int ret;

	dev_dbg(to_device(ispif), "%s: Enter, ispif%d enable = %d\n",
		__func__, line->id, enable);

	if (enable) {
		if (!media_entity_remote_pad(
					&line->pads[MSM_ISPIF_PAD_SINK])) {
			return -ENOLINK;
		}

		/* Config */

		mutex_lock(&ispif->config_lock);
		ispif_select_clk_mux(ispif, intf, csid, vfe, 1);

		ret = ispif_validate_intf_status(ispif, intf, vfe);
		if (ret < 0) {
			mutex_unlock(&ispif->config_lock);
			return ret;
		}

		ispif_select_csid(ispif, intf, csid, vfe, 1);
		ispif_enable_cid(ispif, intf, 1 << cid, vfe, 1);
		ispif_config_irq(ispif, intf, vfe, 1);
		ispif_set_intf_cmd(ispif, CMD_ENABLE_FRAME_BOUNDARY, intf, vfe, vc);
	} else {
		mutex_lock(&ispif->config_lock);
		ispif_set_intf_cmd(ispif, CMD_DISABLE_FRAME_BOUNDARY, intf, vfe, vc);
		mutex_unlock(&ispif->config_lock);

		ret = ispif_wait_for_stop(ispif, intf, vfe);
		if (ret < 0)
			return ret;

		mutex_lock(&ispif->config_lock);
		ispif_config_irq(ispif, intf, vfe, 0);
		ispif_enable_cid(ispif, intf, 1 << cid, vfe, 0);
		ispif_select_csid(ispif, intf, csid, vfe, 0);
		ispif_select_clk_mux(ispif, intf, csid, vfe, 0);
	}

	mutex_unlock(&ispif->config_lock);

	return 0;
}

/*
 * __ispif_get_format - Get pointer to format structure
 * @ispif: ISPIF line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__ispif_get_format(struct ispif_line *line,
		   struct v4l2_subdev_pad_config *cfg,
		   unsigned int pad,
		   enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&line->subdev, cfg, pad);

	return &line->fmt[pad];
}

/*
 * ispif_try_format - Handle try format by pad subdev method
 * @ispif: ISPIF line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void ispif_try_format(struct ispif_line *line,
			     struct v4l2_subdev_pad_config *cfg,
			     unsigned int pad,
			     struct v4l2_mbus_framefmt *fmt,
			     enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_ISPIF_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(ispif_formats); i++)
			if (fmt->code == ispif_formats[i])
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(ispif_formats))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		break;

	case MSM_ISPIF_PAD_SRC:
		/* Set and return a format same as sink pad */

		*fmt = *__ispif_get_format(line, cfg, MSM_ISPIF_PAD_SINK,
					   which);

		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * ispif_enum_mbus_code - Handle pixel format enumeration
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int ispif_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (code->pad == MSM_ISPIF_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(ispif_formats))
			return -EINVAL;

		code->code = ispif_formats[code->index];
	} else {
		if (code->index > 0)
			return -EINVAL;

		format = __ispif_get_format(line, cfg, MSM_ISPIF_PAD_SINK,
					    code->which);

		code->code = format->code;
	}

	return 0;
}

/*
 * ispif_enum_frame_size - Handle frame size enumeration
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int ispif_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ispif_try_format(line, cfg, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ispif_try_format(line, cfg, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * ispif_get_format - Handle get format by pads subdev method
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int ispif_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ispif_get_format(line, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * ispif_set_format - Handle set format by pads subdev method
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int ispif_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ispif_get_format(line, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	ispif_try_format(line, cfg, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_ISPIF_PAD_SINK) {
		format = __ispif_get_format(line, cfg, MSM_ISPIF_PAD_SRC,
					    fmt->which);

		*format = fmt->format;
		ispif_try_format(line, cfg, MSM_ISPIF_PAD_SRC, format,
				 fmt->which);
	}

	return 0;
}

/*
 * ispif_init_formats - Initialize formats on all pads
 * @sd: ISPIF V4L2 subdevice
 *
 * Initialize all pad formats with default values.
 */
static int ispif_init_formats(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_format format;

	memset(&format, 0, sizeof(format));
	format.pad = MSM_ISPIF_PAD_SINK;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;
	ispif_set_format(sd, NULL, &format);

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
int msm_ispif_subdev_init(struct ispif_device *ispif,
			  struct resources_ispif *res)
{
	struct device *dev = to_device(ispif);
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++)
		ispif->line[i].id = i;

	mutex_init(&ispif->power_lock);
	ispif->power_count = 0;

	mutex_init(&ispif->config_lock);

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

	ispif->nclocks = 0;
	while (res->clock[ispif->nclocks])
		ispif->nclocks++;

	ispif->clock = devm_kzalloc(dev, ispif->nclocks * sizeof(*ispif->clock),
				    GFP_KERNEL);
	if (!ispif->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < ispif->nclocks; i++) {
		ispif->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(ispif->clock[i]))
			return PTR_ERR(ispif->clock[i]);
	}

	ispif->nclocks_for_reset = 0;
	while (res->clock_for_reset[ispif->nclocks_for_reset])
		ispif->nclocks_for_reset++;


	ispif->clock_for_reset = devm_kzalloc(dev, ispif->nclocks_for_reset *
			sizeof(*ispif->clock_for_reset), GFP_KERNEL);
	if (!ispif->clock_for_reset) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < ispif->nclocks_for_reset; i++) {
		ispif->clock_for_reset[i] = devm_clk_get(dev,
						res->clock_for_reset[i]);
		if (IS_ERR(ispif->clock_for_reset[i]))
			return PTR_ERR(ispif->clock_for_reset[i]);
	}

	init_completion(&ispif->reset_complete);

	return 0;
}

static enum ispif_intf ispif_get_intf(enum vfe_line_id line_id)
{
	switch (line_id) {
	case (VFE_LINE_RDI0):
		return RDI0;
	case (VFE_LINE_RDI1):
		return RDI1;
	case (VFE_LINE_RDI2):
		return RDI2;
	default:
		return RDI0;
	}
}

static int ispif_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SINK) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct ispif_line *line;

		sd = container_of(entity, struct v4l2_subdev, entity);
		line = v4l2_get_subdevdata(sd);

		msm_csid_get_csid_id(remote->entity, &line->csid_id);
	} else if ((local->flags & MEDIA_PAD_FL_SOURCE) &&
		   (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct ispif_line *line;
		enum vfe_line_id id;

		sd = container_of(entity, struct v4l2_subdev, entity);
		line = v4l2_get_subdevdata(sd);

		msm_vfe_get_vfe_id(remote->entity, &line->vfe_id);
		msm_vfe_get_vfe_line_id(remote->entity, &id);
		line->interface = ispif_get_intf(id);
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ispif_core_ops = {
	.s_power = ispif_set_power,
};

static const struct v4l2_subdev_video_ops ispif_video_ops = {
	.s_stream = ispif_set_stream,
};

static const struct v4l2_subdev_pad_ops ispif_pad_ops = {
	.enum_mbus_code = ispif_enum_mbus_code,
	.enum_frame_size = ispif_enum_frame_size,
	.get_fmt = ispif_get_format,
	.set_fmt = ispif_set_format,
};

static const struct v4l2_subdev_ops ispif_v4l2_ops = {
	.core = &ispif_core_ops,
	.video = &ispif_video_ops,
	.pad = &ispif_pad_ops,
};

static const struct v4l2_subdev_internal_ops ispif_v4l2_internal_ops;

static const struct media_entity_operations ispif_media_ops = {
	.link_setup = ispif_link_setup,
	.link_validate = v4l2_subdev_link_validate,
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
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;
		struct media_pad *pads = ispif->line[i].pads;

		v4l2_subdev_init(sd, &ispif_v4l2_ops);
		sd->internal_ops = &ispif_v4l2_internal_ops;
		sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
			 MSM_ISPIF_NAME, i);
		v4l2_set_subdevdata(sd, &ispif->line[i]);

		ispif_init_formats(sd);

		pads[MSM_ISPIF_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
		pads[MSM_ISPIF_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

		sd->entity.ops = &ispif_media_ops;
		ret = media_entity_init(&sd->entity, MSM_ISPIF_PADS_NUM,
					pads, 0);
		if (ret < 0) {
			pr_err("Fail to init media entity");
			goto error;
		}

		ret = v4l2_device_register_subdev(v4l2_dev, sd);
		if (ret < 0) {
			pr_err("Fail to register subdev");
			media_entity_cleanup(&sd->entity);
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;

		v4l2_device_unregister_subdev(sd);
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
	int i;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;

		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
}
