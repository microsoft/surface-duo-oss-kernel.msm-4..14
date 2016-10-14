/*
 * vfe.c
 *
 * Qualcomm MSM Camera Subsystem - VFE Module
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
#include <asm/dma-iommu.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/msm-bus.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/qcom_iommu.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "vfe.h"
#include "camss.h"

#define MSM_VFE_NAME "msm_vfe"
#define MSM_VFE_VIDEO_NAME "msm_vfe_video"

#define vfe_line_array(ptr_line)	\
	((const struct vfe_line (*)[]) &(ptr_line[-(ptr_line->id)]))

#define to_vfe(ptr_line)	\
	container_of(vfe_line_array(ptr_line), struct vfe_device, ptr_line)

#define VFE_0_HW_VERSION		0x000

#define VFE_0_GLOBAL_RESET_CMD		0x00c
#define VFE_0_GLOBAL_RESET_CMD_CORE	(1 << 0)
#define VFE_0_GLOBAL_RESET_CMD_CAMIF	(1 << 1)
#define VFE_0_GLOBAL_RESET_CMD_BUS	(1 << 2)
#define VFE_0_GLOBAL_RESET_CMD_BUS_BDG	(1 << 3)
#define VFE_0_GLOBAL_RESET_CMD_REGISTER	(1 << 4)
#define VFE_0_GLOBAL_RESET_CMD_TIMER	(1 << 5)
#define VFE_0_GLOBAL_RESET_CMD_PM	(1 << 6)
#define VFE_0_GLOBAL_RESET_CMD_BUS_MISR	(1 << 7)
#define VFE_0_GLOBAL_RESET_CMD_TESTGEN	(1 << 8)

#define VFE_0_IRQ_CMD			0x024
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	(1 << 0)

#define VFE_0_IRQ_MASK_0		0x028
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	(1 << ((n) + 8))
#define VFE_0_IRQ_MASK_0_RESET_ACK			(1 << 31)
#define VFE_0_IRQ_MASK_1		0x02c
#define VFE_0_IRQ_MASK_1_VIOLATION			(1 << 7)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		(1 << 8)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	(1 << ((n) + 9))

#define VFE_0_IRQ_CLEAR_0		0x030
#define VFE_0_IRQ_CLEAR_1		0x034

#define VFE_0_IRQ_STATUS_0		0x038
#define VFE_0_IRQ_STATUS_0_RDIn_REG_UPDATE(n)		(1 << ((n) + 5))
#define VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(n)	(1 << ((n) + 8))
#define VFE_0_IRQ_STATUS_0_RESET_ACK			(1 << 31)
#define VFE_0_IRQ_STATUS_1		0x03c
#define VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK		(1 << 8)
#define VFE_0_IRQ_STATUS_1_RDI0_SOF			(1 << 29)

#define VFE_0_BUS_CMD			0x4c
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	(1 << (x))

#define VFE_0_BUS_CFG			0x050

#define VFE_0_BUS_XBAR_CFG_x(x)		(0x58 + 0x4 * ((x) / 2))
#define VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT		8
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0	5
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1	6
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2	7

#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x06c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT	1
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x070 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x074 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x078 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT	2
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK	(0x1F << 2)

#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x07c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT	16
#define VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(n)	(0x088 + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(n)	(0x08c + 0x24 * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF	0xffffffff

#define VFE_0_BUS_PING_PONG_STATUS	0x268
#define VFE_0_BUS_OPERATION_STATUS	0x26c

#define VFE_0_BUS_BDG_CMD		0x2c0
#define VFE_0_BUS_BDG_CMD_HALT_REQ	1

#define VFE_0_BUS_BDG_QOS_CFG_0		0x2c4
#define VFE_0_BUS_BDG_QOS_CFG_1		0x2c8
#define VFE_0_BUS_BDG_QOS_CFG_2		0x2cc
#define VFE_0_BUS_BDG_QOS_CFG_3		0x2d0
#define VFE_0_BUS_BDG_QOS_CFG_4		0x2d4
#define VFE_0_BUS_BDG_QOS_CFG_5		0x2d8
#define VFE_0_BUS_BDG_QOS_CFG_6		0x2dc
#define VFE_0_BUS_BDG_QOS_CFG_7		0x2e0

#define VFE_0_RDI_CFG_x(x)		(0x2e8 + (0x4 * (x)))
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT	28
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK	(0xf << 28)
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT	4
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK		(0xf << 4)
#define VFE_0_RDI_CFG_x_RDI_EN_BIT		(1 << 2)
#define VFE_0_RDI_CFG_x_MIPI_EN_BITS		0x3
#define VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(r)	(1 << (16 + (r)))

#define VFE_0_REG_UPDATE			0x378
#define VFE_0_REG_UPDATE_RDIn(n)		(1 << (1 + (n)))
#define VFE_0_REG_UPDATE_RDI1			(1 << 2)
#define VFE_0_REG_UPDATE_RDI2			(1 << 3)

#define VFE_0_CGC_OVERRIDE_1			0x974
#define VFE_0_CGC_OVERRIDE_1_IMAGE_Mx_CGC_OVERRIDE(x)	(1 << (x))

/* Vfe reset timeout */
#define VFE_RESET_TIMEOUT_MS 50
/* Vfe halt timeout */
#define VFE_HALT_TIMEOUT_MS 100
/* Max number of frame drop updates per frame */
#define VFE_FRAME_DROP_UPDATES 5
/* Frame drop value NOTE it VAL + UPDATES should not exceed 31 */
#define VFE_FRAME_DROP_VAL 20

static const u32 vfe_formats[] = {
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

static inline void vfe_reg_clr(struct vfe_device *vfe, u32 reg, u32 clr_bits)
{
	u32 bits = readl(vfe->base + reg);

	writel(bits & ~clr_bits, vfe->base + reg);
}

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl(vfe->base + reg);

	writel(bits | set_bits, vfe->base + reg);
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	u32 reset_bits = VFE_0_GLOBAL_RESET_CMD_TESTGEN		|
			 VFE_0_GLOBAL_RESET_CMD_BUS_MISR	|
			 VFE_0_GLOBAL_RESET_CMD_PM		|
			 VFE_0_GLOBAL_RESET_CMD_TIMER		|
			 VFE_0_GLOBAL_RESET_CMD_REGISTER	|
			 VFE_0_GLOBAL_RESET_CMD_BUS_BDG		|
			 VFE_0_GLOBAL_RESET_CMD_BUS		|
			 VFE_0_GLOBAL_RESET_CMD_CAMIF		|
			 VFE_0_GLOBAL_RESET_CMD_CORE;

	writel(reset_bits, vfe->base + VFE_0_GLOBAL_RESET_CMD);
}

static void vfe_wm_enable(struct vfe_device *vfe, u32 wm, u32 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm), 1);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm), 1);
}

static void vfe_wm_frame_based(struct vfe_device *vfe, u32 wm, u32 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
}

static void vfe_wm_set_framedrop_period(struct vfe_device *vfe, u32 wm, u32 per)
{
	u32 reg;

	reg = readl(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	reg &= ~(VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK);

	reg |= (per << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT)
		& VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK;

	writel(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));
}

static void vfe_wm_set_framedrop_pattern(struct vfe_device *vfe, u32 wm,
					 u32 pattern)
{
	writel(pattern,
	       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(wm));
}

static void vfe_wm_set_ub_cfg(struct vfe_device *vfe, u32 wm, u16 offset,
			      u16 depth)
{
	u32 reg;

	reg = (offset << VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT) |
		depth;
	writel(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
}

static void vfe_bus_reload_wm(struct vfe_device *vfe, u32 wm)
{
	writel(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
}

static void vfe_wm_set_ping_addr(struct vfe_device *vfe, u32 wm, u32 addr)
{
	writel(addr, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
}

static void vfe_wm_set_pong_addr(struct vfe_device *vfe, u32 wm, u32 addr)
{
	writel(addr, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
}

static int vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u32 wm)
{
	u32 reg;

	reg = readl(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (reg >> wm) & 0x1;
}

static void vfe_bus_enable_wr_if(struct vfe_device *vfe, u32 enable)
{
	if (enable)
		writel(0x10000009, vfe->base + VFE_0_BUS_CFG);
	else
		writel(0, vfe->base + VFE_0_BUS_CFG);
}

static int vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u32 wm, enum vfe_line_id id)
{
	u32 reg;

	if (id != VFE_LINE_RDI0 && id != VFE_LINE_RDI1 && id != VFE_LINE_RDI2)
		return -EINVAL;

	reg = VFE_0_RDI_CFG_x_MIPI_EN_BITS;
	reg |= VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(id);
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	reg |= ((3 * id) << VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT) &
		VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK;
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	default:
		dev_err(to_device(vfe), "Invalid rdi %d\n", id);
		return -EINVAL;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_set(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);

	writel(VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF,
	       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(wm));

	return 0;
}

static int vfe_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u32 wm, enum vfe_line_id id)
{
	u32 reg;

	if (id != VFE_LINE_RDI0 && id != VFE_LINE_RDI1 && id != VFE_LINE_RDI2)
		return -EINVAL;

	reg = VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(id);
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	default:
		dev_err(to_device(vfe), "Invalid rdi %d\n", id);
		return -EINVAL;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_clr(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);

	return 0;
}

static int vfe_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id, u8 cid)
{
	if (id != VFE_LINE_RDI0 && id != VFE_LINE_RDI1 && id != VFE_LINE_RDI2)
		return -EINVAL;

	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id),
		    VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK);

	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id),
		    cid << VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT);

	return 0;
}

static int vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= VFE_0_REG_UPDATE_RDIn(line_id);
	writel(vfe->reg_update, vfe->base + VFE_0_REG_UPDATE);

	return 0;
}

static void vfe_enable_irq_wm(struct vfe_device *vfe, u32 wm_idx, u8 enable) {
	u32 irq_en0 = readl_relaxed(vfe->base + VFE_0_IRQ_MASK_0);
	u32 irq_en1 = readl_relaxed(vfe->base + VFE_0_IRQ_MASK_1);

	if (enable) {
		irq_en0 |= VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm_idx);
		irq_en1 |= VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(wm_idx);
	} else {
		irq_en0 &= ~VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm_idx);
		irq_en1 &= ~VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(wm_idx);
	}

	writel_relaxed(irq_en0, vfe->base + VFE_0_IRQ_MASK_0);
	writel_relaxed(irq_en1, vfe->base + VFE_0_IRQ_MASK_1);
}

static void vfe_enable_irq_common(struct vfe_device *vfe)
{
	u32 irq_en0 = readl_relaxed(vfe->base + VFE_0_IRQ_MASK_0);
	u32 irq_en1 = readl_relaxed(vfe->base + VFE_0_IRQ_MASK_1);

	irq_en0 |= VFE_0_IRQ_MASK_0_RESET_ACK;

	irq_en1 |= VFE_0_IRQ_MASK_1_VIOLATION;
	irq_en1 |= VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK;

	writel(irq_en0, vfe->base + VFE_0_IRQ_MASK_0);
	writel(irq_en1, vfe->base + VFE_0_IRQ_MASK_1);
}

static void vfe_isr_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	unsigned long flags;

	spin_lock_irqsave(&vfe->output_lock, flags);
	vfe->reg_update &= ~VFE_0_REG_UPDATE_RDIn(line_id);
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static void vfe_isr_wm_done(struct vfe_device *vfe, u32 wm_idx);

static irqreturn_t vfe_subdev_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 value0, value1;
	int i;

	value0 = readl(vfe->base + VFE_0_IRQ_STATUS_0);
	value1 = readl(vfe->base + VFE_0_IRQ_STATUS_1);

	writel(value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel(value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();
	writel(0x1, vfe->base + VFE_0_IRQ_CMD); // Apply IRQ Clear[01]

	if (value0 & VFE_0_IRQ_STATUS_0_RESET_ACK)
		complete_all(&vfe->reset_completion);

	if (value1 & VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK) {
		complete_all(&vfe->halt_completion);
		writel(0x0, vfe->base + VFE_0_BUS_BDG_CMD);
	}

	for (i = VFE_LINE_RDI0; i < VFE_LINE_RDI2 + 1; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_RDIn_REG_UPDATE(i))
			vfe_isr_reg_update(vfe, i);

	for (i = 0; i < 7; i++)
		if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_n_PING_PONG(i))
			vfe_isr_wm_done(vfe, i);

	return IRQ_HANDLED;
}

static int vfe_reset(struct vfe_device *vfe)
{
	unsigned long time;

	init_completion(&vfe->reset_completion);

	vfe_global_reset(vfe);

	time = wait_for_completion_timeout(&vfe->reset_completion,
		msecs_to_jiffies(VFE_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(to_device(vfe), "Vfe reset timeout\n");
		return -EIO;
	}

	return 0;
}

static int vfe_halt(struct vfe_device *vfe)
{
	unsigned long time;

	init_completion(&vfe->halt_completion);

	writel(VFE_0_BUS_BDG_CMD_HALT_REQ, vfe->base + VFE_0_BUS_BDG_CMD);

	time = wait_for_completion_timeout(&vfe->halt_completion,
		msecs_to_jiffies(VFE_HALT_TIMEOUT_MS));
	if (!time) {
		dev_err(to_device(vfe), "Vfe halt timeout\n");
		return -EIO;
	}

	return 0;
}

static void vfe_init_outputs(struct vfe_device *vfe)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->line); i++) {
		struct msm_vfe_output *output = &vfe->line[i].output;

		output->state = MSM_VFE_OUTPUT_OFF;
		output->buf[0] = NULL;
		output->buf[1] = NULL;
		INIT_LIST_HEAD(&output->pending_bufs);
	}
}

static void vfe_reset_output_maps(struct vfe_device *vfe)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&vfe->output_lock, flags);

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++)
		vfe->wm_output_map[i] = VFE_LINE_NONE;

	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static void vfe_set_qos(struct vfe_device *vfe)
{
	u32 val = 0xaaa5aaa5;
	u32 val7 = 0x0001aaa5;

	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_0);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_1);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_2);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_3);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_4);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_5);
	writel(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_6);
	writel(val7, vfe->base + VFE_0_BUS_BDG_QOS_CFG_7);
}

static void vfe_set_cgc_override(struct vfe_device *vfe, u32 wm_idx, u8 enable)
{
	u32 val = VFE_0_CGC_OVERRIDE_1_IMAGE_Mx_CGC_OVERRIDE(wm_idx);

	if (enable)
		vfe_reg_set(vfe, VFE_0_CGC_OVERRIDE_1, val);
	else
		vfe_reg_clr(vfe, VFE_0_CGC_OVERRIDE_1, val);

	wmb();
}

static void vfe_output_init_addrs(struct vfe_device *vfe,
				  struct msm_vfe_output *output,
				  int sync)
{
	u32 ping_addr = 0;
	u32 pong_addr = 0;

	output->active_buf = 0;

	if (output->buf[0])
		ping_addr = output->buf[0]->addr;

	if (output->buf[1])
		pong_addr = output->buf[1]->addr;
	else
		pong_addr = ping_addr;

	vfe_wm_set_ping_addr(vfe, output->wm_idx, ping_addr);
	vfe_wm_set_pong_addr(vfe, output->wm_idx, pong_addr);
	if (sync)
		vfe_bus_reload_wm(vfe, output->wm_idx);
}

static void vfe_output_reset_addrs(struct vfe_device *vfe,
				       struct msm_vfe_output *output)
{
	vfe_wm_set_ping_addr(vfe, output->wm_idx, 0x00);
	vfe_wm_set_pong_addr(vfe, output->wm_idx, 0x00);
}

static void vfe_output_update_ping_addr(struct vfe_device *vfe,
					struct msm_vfe_output *output,
					int sync)
{
	u32 addr = 0;

	if (output->buf[0])
		addr = output->buf[0]->addr;

	vfe_wm_set_ping_addr(vfe, output->wm_idx, addr);
	if (sync)
		vfe_bus_reload_wm(vfe, output->wm_idx);
}

static void vfe_output_update_pong_addr(struct vfe_device *vfe,
					struct msm_vfe_output *output,
					int sync)
{
	u32 addr = 0;

	if (output->buf[1])
		addr = output->buf[1]->addr;

	vfe_wm_set_pong_addr(vfe, output->wm_idx, addr);
	if (sync)
		vfe_bus_reload_wm(vfe, output->wm_idx);

}

static int __vfe_reserve_wm(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	int ret = -EBUSY;
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++) {
		if (vfe->wm_output_map[i] == VFE_LINE_NONE) {
			vfe->wm_output_map[i] = line_id;
			ret = i;
			break;
		}
	}

	return ret;
}

static int __vfe_release_wm(struct vfe_device *vfe, u8 wm_idx)
{
	if (wm_idx > ARRAY_SIZE(vfe->wm_output_map))
		return -EINVAL;

	vfe->wm_output_map[wm_idx] = VFE_LINE_NONE;

	return 0;
}

/* Vfe hw buffer operations */
static struct msm_video_buffer *
__vfe_get_next_output_buf(struct msm_vfe_output *output)
{
	struct msm_video_buffer *buffer = NULL;

	if (!list_empty(&output->pending_bufs)) {
		buffer = list_first_entry(&output->pending_bufs,
					  struct msm_video_buffer,
					  dma_queue);
		list_del(&buffer->dma_queue);
	}

	return buffer;
}

/*
 * vfe_output_frame_drop - Set frame drop pattern per given output
 * @vfe: Pointer to vfe device.
 * @output: Pointer to vfe output.
 * @drop_pattern: Kept (1) or dropped (0). The pattern starts from bit 0
 *   and progresses to bit 31.
 */
static void vfe_output_frame_drop(struct vfe_device *vfe,
				  struct msm_vfe_output *output,
				  u32 drop_pattern)
{
	u32 drop_period;

	/* We need to toggle update period to be valid on next frame */
	output->drop_update_idx++;
	output->drop_update_idx %= VFE_FRAME_DROP_UPDATES;
	drop_period = VFE_FRAME_DROP_VAL + output->drop_update_idx;

	vfe_wm_set_framedrop_period(vfe, output->wm_idx, drop_period);
	wmb();
	vfe_wm_set_framedrop_pattern(vfe, output->wm_idx, drop_pattern);
	wmb();
	vfe_reg_update(vfe, container_of(output, struct vfe_line, output)->id);

}

/*
 * __vfe_add_output_buf - Add output buffer to vfe output
 * @output: Pointer to vfe output.
 * @buffer: Pointer to video buffer.
 *
 * NOTE: Should be called with vfe locked.
 */
static void __vfe_add_output_buf(struct msm_vfe_output *output,
				 struct msm_video_buffer *buffer)
{
	INIT_LIST_HEAD(&buffer->dma_queue);
	list_add_tail(&buffer->dma_queue, &output->pending_bufs);
}

/*
 * __vfe_flush_output_bufs - Flush all pending out buffers.
 * @output: Pointer to vfe output.
 *
 * NOTE: Should be called with vfe locked.
 */
static void __vfe_flush_output_bufs(struct msm_vfe_output *output)
{
	struct msm_video_buffer *buf;
	struct msm_video_buffer *t;

	list_for_each_entry_safe(buf, t, &output->pending_bufs, dma_queue) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->dma_queue);
	}
}

static void __vfe_update_wm_on_next_buf(struct vfe_device *vfe,
					struct msm_vfe_output *output)
{
	switch (output->state) {
	case MSM_VFE_OUTPUT_CONTINUOUS:
		vfe_output_frame_drop(vfe, output, 3);
		break;
	case MSM_VFE_OUTPUT_SINGLE:
		dev_err_ratelimited(to_device(vfe),
				    "Next buf in single state!\n");
		break;
	default:
		return;
	}
}

static void __vfe_update_wm_on_last_buf(struct vfe_device *vfe,
					struct msm_vfe_output *output)
{
	switch (output->state) {
	case MSM_VFE_OUTPUT_CONTINUOUS:
		output->state = MSM_VFE_OUTPUT_SINGLE;
		vfe_output_frame_drop(vfe, output, 1);
		break;
	case MSM_VFE_OUTPUT_SINGLE:
		output->state = MSM_VFE_OUTPUT_IDLE;
		vfe_output_frame_drop(vfe, output, 0);
		vfe_output_reset_addrs(vfe, output);
		break;
	default:
		dev_err_ratelimited(to_device(vfe),
				    "Last buff in wrong state! %d\n",
				    output->state);
		return;
	}
}

static void __vfe_update_wm_on_new_buf(struct vfe_device *vfe,
				       struct msm_vfe_output *output,
				       struct msm_video_buffer *new_buf)
{
	int inactive_idx;

	switch (output->state) {

	case MSM_VFE_OUTPUT_SINGLE:
		inactive_idx = !output->active_buf;

		if (!output->buf[inactive_idx]) {
			output->buf[inactive_idx] = new_buf;

			if (inactive_idx)
				vfe_output_update_pong_addr(vfe, output, 0);
			else
				vfe_output_update_ping_addr(vfe, output, 0);

			vfe_output_frame_drop(vfe, output, 3);
			output->state = MSM_VFE_OUTPUT_CONTINUOUS;
		} else {
			__vfe_add_output_buf(output, new_buf);
			dev_err_ratelimited(to_device(vfe),
					    "Inactive buffer is busy\n");
		}
		break;

	case MSM_VFE_OUTPUT_IDLE:
		if (!output->buf[0]) {
			output->buf[0] = new_buf;

			vfe_output_init_addrs(vfe, output, 1);

			/* After wm reload we can not skip second frame.
			 * Capture only second frame to avoid iommu fault */
			vfe_output_frame_drop(vfe, output, 2);
			output->state = MSM_VFE_OUTPUT_SINGLE;
		} else {
			__vfe_add_output_buf(output, new_buf);
			dev_err_ratelimited(to_device(vfe),
					    "Output idle with buffer set!\n");
		}
		break;

	case MSM_VFE_OUTPUT_CONTINUOUS:

	default:
		__vfe_add_output_buf(output, new_buf);
		return;
	}
}

static int vfe_get_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct msm_vfe_output *output;
	unsigned long flags;
	int wm_idx;

	spin_lock_irqsave(&vfe->output_lock, flags);

	output = &line->output;
	if (output->state != MSM_VFE_OUTPUT_OFF) {
		dev_err(to_device(vfe), "Output is running\n");
		goto error;
	}
	output->state = MSM_VFE_OUTPUT_RESERVED;

	output->active_buf = 0;

	/* We will use only one wm per output for now */
	wm_idx = __vfe_reserve_wm(vfe, line->id);
	if (wm_idx < 0) {
		dev_err(to_device(vfe), "Can not reserve wm\n");
		goto error_get_wm;
	}
	output->drop_update_idx = 0;
	output->wm_idx = wm_idx;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	dev_dbg(to_device(vfe), "%s: RDI%d -> WM%d\n",
		__func__, line->id, wm_idx);

	return 0;

error_get_wm:
	output->state = MSM_VFE_OUTPUT_OFF;
error:
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return -EINVAL;
}

static int vfe_put_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct msm_vfe_output *output = &line->output;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&vfe->output_lock, flags);

	ret = __vfe_release_wm(vfe, output->wm_idx);
	if (ret < 0)
		goto out;

	output->state = MSM_VFE_OUTPUT_OFF;

out:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
	return ret;
}

static int vfe_enable_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct msm_vfe_output *output = &line->output;
	unsigned long flags;
	u32 ub_size;

	switch (vfe->id) {
	case 0:
		ub_size = MSM_VFE_VFE0_UB_SIZE_RDI;
		break;
	case 1:
		ub_size = MSM_VFE_VFE1_UB_SIZE_RDI;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&vfe->output_lock, flags);

	vfe->reg_update &= ~VFE_0_REG_UPDATE_RDIn(line->id);

	if (output->state != MSM_VFE_OUTPUT_RESERVED) {
		dev_err(to_device(vfe), "Output is not in reserved state %d\n",
			output->state);
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EINVAL;
	}
	output->state = MSM_VFE_OUTPUT_IDLE;

	output->buf[0] = __vfe_get_next_output_buf(output);
	if (output->buf[0])
		output->state = MSM_VFE_OUTPUT_SINGLE;

	output->buf[1] = __vfe_get_next_output_buf(output);
	if (output->buf[1])
		output->state = MSM_VFE_OUTPUT_CONTINUOUS;

	switch (output->state) {
	case MSM_VFE_OUTPUT_SINGLE:
		/* After wm reload we can not skip second frame.
		 * Capture only second frame to avoid iommu fault */
		/* Skip 4 bad frames from sensor TODO: get number from sensor */
		vfe_output_frame_drop(vfe, output, 2 << 4);
		break;
	case MSM_VFE_OUTPUT_CONTINUOUS:
		/* Skip 4 bad frames from sensor TODO: get number from sensor */
		vfe_output_frame_drop(vfe, output, 3 << 4);
		break;
	default:
		vfe_output_frame_drop(vfe, output, 0);
		break;
	}

	vfe_output_init_addrs(vfe, output, 0);

	vfe_set_cgc_override(vfe, output->wm_idx, 1);

	vfe_enable_irq_wm(vfe, output->wm_idx, 1);

	vfe_bus_connect_wm_to_rdi(vfe, output->wm_idx, line->id);

	vfe_set_rdi_cid(vfe, line->id, 0);

	vfe_wm_set_ub_cfg(vfe, output->wm_idx,
			  (ub_size + 1) * output->wm_idx, ub_size);

	vfe_wm_frame_based(vfe, output->wm_idx, 1);
	vfe_wm_enable(vfe, output->wm_idx, 1);

	vfe_bus_reload_wm(vfe, output->wm_idx);

	vfe_reg_update(vfe, line->id);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int vfe_disable_output(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct msm_vfe_output *output = &line->output;
	unsigned long flags;

	spin_lock_irqsave(&vfe->output_lock, flags);

	vfe_wm_enable(vfe, output->wm_idx, 0);
	vfe_bus_disconnect_wm_from_rdi(vfe, output->wm_idx, line->id);
	vfe_reg_update(vfe, line->id);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int vfe_enable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	mutex_lock(&vfe->stream_lock);

	if (!vfe->stream_count) {
		vfe_enable_irq_common(vfe);

		vfe_bus_enable_wr_if(vfe, 1);

		vfe_set_qos(vfe);
	}

	vfe->stream_count++;

	mutex_unlock(&vfe->stream_lock);

	ret = vfe_get_output(line);
	if (ret < 0)
		goto error_get_output;

	ret = vfe_enable_output(line);
	if (ret < 0)
		goto error_enable_output;

	return 0;


error_enable_output:
	vfe_put_output(line);

error_get_output:
	mutex_lock(&vfe->stream_lock);

	if (vfe->stream_count == 1)
		vfe_bus_enable_wr_if(vfe, 0);

	vfe->stream_count--;

	mutex_unlock(&vfe->stream_lock);

	return ret;
}

static int vfe_disable(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);

	mutex_lock(&vfe->stream_lock);

	if (vfe->stream_count == 1) {
		vfe_bus_enable_wr_if(vfe, 0);

	}

	vfe->stream_count--;

	mutex_unlock(&vfe->stream_lock);

	vfe_disable_output(line);

	vfe_put_output(line);

	return 0;
}

static void vfe_isr_wm_done(struct vfe_device *vfe, u32 wm_idx)
{
	struct msm_video_buffer *ready_buf;
	struct msm_vfe_output *output;
	dma_addr_t new_addr;
	unsigned long flags;
	u32 active_index;

	active_index = vfe_wm_get_ping_pong_status(vfe, wm_idx);

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm_idx] == VFE_LINE_NONE) {
		dev_err_ratelimited(to_device(vfe),
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->line[vfe->wm_output_map[wm_idx]].output;

	if (output->active_buf == active_index) {
		dev_err_ratelimited(to_device(vfe),
				    "Active buffer mismatch!\n");
		goto out_unlock;
	}
	output->active_buf = active_index;

	ready_buf = output->buf[!active_index];
	if (!ready_buf) {
		dev_err_ratelimited(to_device(vfe),
				    "Missing ready buf %d %d!\n",
				    !active_index, output->state);
		goto out_unlock;
	}

	/* Get next buffer */
	output->buf[!active_index] = __vfe_get_next_output_buf(output);
	if (!output->buf[!active_index]) {
		new_addr = 0;
		__vfe_update_wm_on_last_buf(vfe, output);
	} else {
		new_addr = output->buf[!active_index]->addr;
		__vfe_update_wm_on_next_buf(vfe, output);
	}

	if (active_index)
		vfe_wm_set_ping_addr(vfe, wm_idx, new_addr);
	else
		vfe_wm_set_pong_addr(vfe, wm_idx, new_addr);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (ready_buf)
		vb2_buffer_done(&ready_buf->vb, VB2_BUF_STATE_DONE);
	else
		dev_err_ratelimited(to_device(vfe),
				    "Received wm without buffer\n");

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static int vfe_bus_request(struct vfe_device *vfe)
{
	int ret;

	vfe->bus_client = msm_bus_scale_register_client(vfe->bus_scale_table);
	if (!vfe->bus_client) {
		dev_err(to_device(vfe), "Failed to register bus client\n");
		return -ENOENT;
	}

	ret = msm_bus_scale_client_update_request(vfe->bus_client, 1);
	if (ret < 0) {
		dev_err(to_device(vfe), "Failed bus scale update %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static void vfe_bus_release(struct vfe_device *vfe)
{
	if (vfe->bus_client) {
		msm_bus_scale_unregister_client(vfe->bus_client);
		vfe->bus_client = 0;
	}
}

/*
 * vfe_enable_clocks - Enable clocks for VFE module and
 * set clock rates where needed
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 * @clock_rate: Clock rates array
 *
 * Return 0 on success or a negative error code otherwise
 */
static int vfe_enable_clocks(int nclocks, struct clk **clock, s32 *clock_rate)
{
	long clk_rate;
	int i;
	int ret;

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
 * vfe_disable_clocks - Disable clocks for VFE module
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 */
static void vfe_disable_clocks(int nclocks, struct clk **clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i]);
}

static int vfe_get(struct vfe_device *vfe)
{
	int ret;

	mutex_lock(&vfe->power_lock);

	if (vfe->power_count == 0) {
		vfe_reset_output_maps(vfe); /* TODO: Move? */

		ret = vfe_bus_request(vfe);
		if (ret < 0) {
			dev_err(to_device(vfe), "Fail bus request\n");
			goto error_clocks;
		}

		ret = vfe_enable_clocks(vfe->nclocks, vfe->clock,
					vfe->clock_rate);
		if (ret < 0) {
			dev_err(to_device(vfe), "Fail to enable clocks\n");
			goto error_clocks;
		}

		ret = vfe_reset(vfe);
		if (ret < 0) {
			dev_err(to_device(vfe), "Fail to reset vfe\n");
			goto error_reset;
		}
	}
	vfe->power_count++;

	mutex_unlock(&vfe->power_lock);

	return 0;

error_reset:
	vfe_disable_clocks(vfe->nclocks, vfe->clock);

error_clocks:
	mutex_unlock(&vfe->power_lock);
	return ret;
}

static void vfe_put(struct vfe_device *vfe)
{
	mutex_lock(&vfe->power_lock);
	BUG_ON(vfe->power_count == 0);

	if (--vfe->power_count == 0) {
//		vfe_init_outputs(vfe); /* TODO */
		vfe_halt(vfe);
		vfe_bus_release(vfe);
		vfe_disable_clocks(vfe->nclocks, vfe->clock);
	}
	mutex_unlock(&vfe->power_lock);
}

static struct vfe_line
*vfe_video_pad_to_line(struct media_pad *pad)
{
	struct media_pad *vfe_pad;
	struct v4l2_subdev *subdev;

	vfe_pad = media_entity_remote_pad(pad);
	if (pad == NULL)
		return NULL;

	subdev = media_entity_to_v4l2_subdev(vfe_pad->entity);

	return container_of(subdev, struct vfe_line, subdev);
}

static int vfe_queue_dmabuf(struct camss_video *vid,
			    struct msm_video_buffer *buf)
{
	struct vfe_device *vfe = &vid->camss->vfe;
	struct vfe_line *line;
	struct msm_vfe_output *output;
	unsigned long flags;

	line = vfe_video_pad_to_line(&vid->pad);
	if (!line) {
		dev_err(to_device(vfe), "Can not queue dma buf\n");
		return -1;
	}
	output = &line->output;

	spin_lock_irqsave(&vfe->output_lock, flags);

	__vfe_update_wm_on_new_buf(vfe, output, buf);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int vfe_flush_dmabufs(struct camss_video *vid)
{
	struct vfe_device *vfe = &vid->camss->vfe;
	struct vfe_line *line;
	struct msm_vfe_output *output;
	unsigned long flags;

	line = vfe_video_pad_to_line(&vid->pad);
	if (!line) {
		dev_err(to_device(vfe),	"Can not flush dma buf\n");
		return -1;
	}
	output = &line->output;

	spin_lock_irqsave(&vfe->output_lock, flags);

	__vfe_flush_output_bufs(output);

	if (output->buf[0])
		vb2_buffer_done(&output->buf[0]->vb, VB2_BUF_STATE_ERROR);

	if (output->buf[1])
		vb2_buffer_done(&output->buf[1]->vb, VB2_BUF_STATE_ERROR);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int vfe_set_power(struct v4l2_subdev *sd, int on)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct vfe_device *vfe = to_vfe(line);
	int ret;

	dev_dbg(to_device(vfe), "%s: Enter, rdi%d on = %d\n",
		__func__, line->id, on);

	if (on) {
		u32 hw_version;

		ret = vfe_get(vfe);
		if (ret < 0)
			return ret;

		hw_version = readl(vfe->base);
		dev_dbg(to_device(vfe),
			"VFE HW Version = 0x%08x\n", hw_version);
	} else {
		vfe_put(vfe);
	}

	dev_dbg(to_device(vfe), "%s: Exit, rdi%d on = %d\n",
		__func__, line->id, on);

	return 0;
}

static int vfe_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct vfe_device *vfe = to_vfe(line);
	int ret = 0;

	dev_dbg(to_device(vfe), "%s: Enter, rdi%d enable = %d\n",
		__func__, line->id, enable);

	if (enable) {
		ret = vfe_enable(line);
		if (ret < 0)
			dev_err(to_device(vfe),
				"Fail to enable vfe outputs\n");
	} else {
		ret = vfe_disable(line);
		if (ret < 0)
			dev_err(to_device(vfe),
				"Fail to disable vfe outputs\n");
	}

	return 0;
}

/*
 * __vfe_get_format - Get pointer to format structure
 * @vfe: VFE line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__vfe_get_format(struct vfe_line *line,
		 struct v4l2_subdev_pad_config *cfg,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(&line->subdev, cfg, pad);

	return &line->fmt[pad];
}


/*
 * vfe_try_format - Handle try format by pad subdev method
 * @vfe: VFE line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void vfe_try_format(struct vfe_line *line,
			   struct v4l2_subdev_pad_config *cfg,
			   unsigned int pad,
			   struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_VFE_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(vfe_formats); i++)
			if (fmt->code == vfe_formats[i])
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(vfe_formats))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		break;

	case MSM_VFE_PAD_SRC:
		/* Set and return a format same as sink pad */

		*fmt = *__vfe_get_format(line, cfg, MSM_VFE_PAD_SINK,
					 which);

		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * vfe_enum_mbus_code - Handle pixel format enumeration
 * @sd: VFE V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int vfe_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	if (code->pad == MSM_VFE_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(vfe_formats))
			return -EINVAL;

		code->code = vfe_formats[code->index];
	} else {
		if (code->index > 0)
			return -EINVAL;

		format = __vfe_get_format(line, cfg, MSM_VFE_PAD_SINK,
					  code->which);

		code->code = format->code;
	}

	return 0;
}

/*
 * vfe_enum_frame_size - Handle frame size enumeration
 * @sd: VFE V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int vfe_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	vfe_try_format(line, cfg, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	vfe_try_format(line, cfg, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * vfe_get_format - Handle get format by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_get_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __vfe_get_format(line, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * vfe_set_format - Handle set format by pads subdev method
 * @sd: VFE V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int vfe_set_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct vfe_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __vfe_get_format(line, cfg, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	vfe_try_format(line, cfg, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_VFE_PAD_SINK) {
		format = __vfe_get_format(line, cfg, MSM_VFE_PAD_SRC,
					  fmt->which);

		*format = fmt->format;
		vfe_try_format(line, cfg, MSM_VFE_PAD_SRC, format,
			       fmt->which);
	}

	return 0;
}

/*
 * vfe_init_formats - Initialize formats on all pads
 * @sd: VFE V4L2 subdevice
 *
 * Initialize all pad formats with default values.
 */
static int vfe_init_formats(struct v4l2_subdev *sd)
{
	struct v4l2_subdev_format format;

	memset(&format, 0, sizeof(format));
	format.pad = MSM_VFE_PAD_SINK;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;
	vfe_set_format(sd, NULL, &format);

	return 0;
}

int msm_vfe_subdev_init(struct vfe_device *vfe, struct resources *res)
{
	struct device *dev = to_device(vfe);
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct resource *r;
	struct dma_iommu_mapping *mapping;
	struct camss *camss = to_camss(vfe);
	int i;
	int ret;

	mutex_init(&vfe->power_lock);
	vfe->power_count = 0;

	mutex_init(&vfe->stream_lock);
	vfe->stream_count = 0;

	spin_lock_init(&vfe->output_lock);

	vfe->id = 0;
	vfe->reg_update = 0;

	for (i = 0; i < ARRAY_SIZE(vfe->line); i++) {
		vfe->line[i].video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vfe->line[i].video_out.camss = camss;
		vfe->line[i].id = i;
	}

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	vfe->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(vfe->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(vfe->base);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt[0]);
	vfe->irq = r->start;
	if (IS_ERR_VALUE(vfe->irq))
		return vfe->irq;

	ret = devm_request_irq(dev, vfe->irq, vfe_subdev_isr,
			       IRQF_TRIGGER_RISING, "vfe", vfe);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	/* Clocks */

	vfe->nclocks = 0;
	while (res->clock[vfe->nclocks])
		vfe->nclocks++;

	vfe->clock = devm_kzalloc(dev, vfe->nclocks * sizeof(*vfe->clock),
				  GFP_KERNEL);
	if (!vfe->clock) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	vfe->clock_rate = devm_kzalloc(dev, vfe->nclocks *
				       sizeof(*vfe->clock_rate), GFP_KERNEL);
	if (!vfe->clock_rate) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < vfe->nclocks; i++) {
		vfe->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(vfe->clock[i]))
			return PTR_ERR(vfe->clock[i]);
		vfe->clock_rate[i] = res->clock_rate[i];
	}

	/* IOMMU */

	camss->iommu_dev = msm_iommu_get_ctx("vfe");
	if (IS_ERR(camss->iommu_dev)) {
		dev_err(dev, "Cannot find iommu nonsecure ctx\n");
		return PTR_ERR(camss->iommu_dev);
	}

	mapping = arm_iommu_create_mapping(&platform_bus_type,
					   0x40000000, 0xC0000000, 0);
	if (IS_ERR_OR_NULL(mapping))
		return PTR_ERR(mapping) ?: -ENODEV;

	ret = arm_iommu_attach_device(camss->iommu_dev, mapping);
	if (ret)
		return -1;

	/* MSM Bus */

	vfe->bus_scale_table = msm_bus_cl_get_pdata(pdev);
	if (!vfe->bus_scale_table) {
		dev_err(dev, "bus scaling is disabled\n");
		return -1;
	}

	vfe_init_outputs(vfe);

	return 0;
}

void msm_vfe_get_vfe_id(struct media_entity *entity, u8 *id)
{
	struct v4l2_subdev *sd;
	struct vfe_line *line;
	struct vfe_device *vfe;

	sd = container_of(entity, struct v4l2_subdev, entity);
	line = v4l2_get_subdevdata(sd);
	vfe = to_vfe(line);

	*id = vfe->id;
}

void msm_vfe_get_vfe_line_id(struct media_entity *entity, enum vfe_line_id *id)
{
	struct v4l2_subdev *sd;
	struct vfe_line *line;

	sd = container_of(entity, struct v4l2_subdev, entity);
	line = v4l2_get_subdevdata(sd);

	*id = line->id;
}

static const struct v4l2_subdev_core_ops vfe_core_ops = {
	.s_power = vfe_set_power,
};

static const struct v4l2_subdev_video_ops vfe_video_ops = {
	.s_stream = vfe_set_stream,
};

static const struct v4l2_subdev_pad_ops vfe_pad_ops = {
	.enum_mbus_code = vfe_enum_mbus_code,
	.enum_frame_size = vfe_enum_frame_size,
	.get_fmt = vfe_get_format,
	.set_fmt = vfe_set_format,
};

static const struct v4l2_subdev_ops vfe_v4l2_ops = {
	.core = &vfe_core_ops,
	.video = &vfe_video_ops,
	.pad = &vfe_pad_ops,
};

static const struct v4l2_subdev_internal_ops vfe_v4l2_internal_ops;

static const struct media_entity_operations vfe_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static struct msm_video_ops rdi_video_ops = {
	.queue_dmabuf = vfe_queue_dmabuf,
	.flush_dmabufs = vfe_flush_dmabufs,
};

int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd;
	struct media_pad *pads;
	struct camss_video *video_out;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->line); i++) {
		char name[32];

		sd = &vfe->line[i].subdev;
		pads = vfe->line[i].pads;
		video_out = &vfe->line[i].video_out;


		v4l2_subdev_init(sd, &vfe_v4l2_ops);
		sd->internal_ops = &vfe_v4l2_internal_ops;
		sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d_%s%d",
			 MSM_VFE_NAME, vfe->id, "rdi", i);
		v4l2_set_subdevdata(sd, &vfe->line[i]);

		vfe_init_formats(sd);

		pads[MSM_VFE_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
		pads[MSM_VFE_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

		sd->entity.ops = &vfe_media_ops;
		ret = media_entity_init(&sd->entity, MSM_VFE_PADS_NUM, pads, 0);
		if (ret < 0) {
			pr_err("Fail to init media entity");
			goto error_init_entity;
		}

		ret = v4l2_device_register_subdev(v4l2_dev, sd);
		if (ret < 0) {
			pr_err("Fail to register subdev");
			goto error_reg_subdev;
		}

		video_out->ops = &rdi_video_ops;
		snprintf(name, ARRAY_SIZE(name), "%s%d", MSM_VFE_VIDEO_NAME, i);
		ret = msm_video_register(video_out, v4l2_dev, name);
		if (ret < 0) {
			pr_err("Failed to register video node");
			goto error_reg_video;
		}

		ret = media_entity_create_link(
				&sd->entity, MSM_VFE_PAD_SRC,
				&video_out->vdev->entity, 0,
				MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
		if (ret < 0) {
			pr_err("Fail to link %s->%s entities\n",
			       sd->entity.name, video_out->vdev->entity.name);
			goto error_link;
		}
	}

	return 0;

error_link:
	msm_video_unregister(video_out);

error_reg_video:
	v4l2_device_unregister_subdev(sd);

error_reg_subdev:
	media_entity_cleanup(&sd->entity);

error_init_entity:
	for (i--; i >= 0; i--) {
		sd = &vfe->line[i].subdev;
		video_out = &vfe->line[i].video_out;

		media_entity_remove_links(&sd->entity);
		msm_video_unregister(video_out);
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

void msm_vfe_unregister_entities(struct vfe_device *vfe)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->line); i++) {
		struct v4l2_subdev *sd = &vfe->line[i].subdev;
		struct camss_video *video_out = &vfe->line[i].video_out;

		media_entity_remove_links(&sd->entity);
		msm_video_unregister(video_out);
		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
}
