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

#define MSM_VFE_DRV_NAME "msm_vfe"
#define MSM_VFE_VIDEO_NAME "msm_vfe_video"

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
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_0_PING_PONG	(1 << 8)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_1_PING_PONG	(1 << 9)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_2_PING_PONG	(1 << 10)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_3_PING_PONG	(1 << 11)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_4_PING_PONG	(1 << 12)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_5_PING_PONG	(1 << 13)
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_6_PING_PONG	(1 << 14)
#define VFE_0_IRQ_MASK_0_RESET_ACK			(1 << 31)
#define VFE_0_IRQ_MASK_1		0x02c
#define VFE_0_IRQ_MASK_1_VIOLATION			(1 << 7)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		(1 << 8)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_0_BUS_OVERFLOW	(1 << 9)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_1_BUS_OVERFLOW	(1 << 10)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_2_BUS_OVERFLOW	(1 << 11)

#define VFE_0_IRQ_CLEAR_0		0x030
#define VFE_0_IRQ_CLEAR_0_ALL		0xffffffff

#define VFE_0_IRQ_CLEAR_1		0x034
#define VFE_0_IRQ_CLEAR_1_ALL		0xffffffff

#define VFE_0_IRQ_STATUS_0		0x038
#define VFE_0_IRQ_STATUS_0_IMAGE_MASTER_0_PING_PONG	(1 << 8)
#define VFE_0_IRQ_STATUS_0_RESET_ACK			(1 << 31)
#define VFE_0_IRQ_STATUS_1		0x03c
#define VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK		(1 << 8)
#define VFE_0_IRQ_STATUS_1_RDI0_SOF			(1 << 29)

#define VFE_0_BUS_CMD			0x4c
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	(1 << (x))

#define VFE_0_BUS_CFG			0x050

#define VFE_0_BUS_XBAR_CFG_x(x)		(0x58 + 0x4 * (x))
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
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK	(0xF << 28)
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT	4
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK		(0xF << 4)
#define VFE_0_RDI_CFG_x_RDI_EN_BIT		(1 << 2)
#define VFE_0_RDI_CFG_x_MIPI_EN_BITS		0x3
#define VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(r)	(1 << (16 + (r)))

#define VFE_0_REG_UPDATE			0x378
#define VFE_0_REG_UPDATE_RDI0			(1 << 1)
#define VFE_0_REG_UPDATE_RDI1			(1 << 2)
#define VFE_0_REG_UPDATE_RDI2			(1 << 3)

#define VFE_0_CGC_OVERRIDE_1			0x974
#define VFE_0_CGC_OVERRIDE_1_IMAGE_M0_CGC_OVERRIDE	1

/* Vfe reset timeout */
#define MSM_VFE_RESET_TIMEOUT_MS 50
/* Vfe halt timeout */
#define MSM_VFE_HALT_TIMEOUT_MS 100
/* Max number of frame drop updates per frame */
#define MSM_VFE_FRAME_DROP_UPDATES 5
/* Frame drop value NOTE it VAL + UPDATES should not exceed 31 */
#define MSM_VFE_FRAME_DROP_VAL 20


static char *clocks[] = {
	"camss_top_ahb_clk",
	"vfe_clk_src",
	"camss_vfe_vfe_clk",
	"camss_csi_vfe_clk",
	"iface_clk",
	"bus_clk",
	"camss_ahb_clk"
};

static char *reg = "vfe0";
static char *reg_vbif = "vfe0_vbif";

static char *interrupt = "vfe0";

static inline void msm_vfe_reg_clr(struct vfe_device *vfe,
	u32 reg, u32 clr_bits)
{
	u32 bits = readl(vfe->base + reg);

	writel(bits & ~clr_bits, vfe->base + reg);
}

static inline void msm_vfe_reg_set(struct vfe_device *vfe,
	u32 reg, u32 set_bits)
{
	u32 bits = readl(vfe->base + reg);

	writel(bits | set_bits, vfe->base + reg);
}

static void msm_vfe_global_reset(struct vfe_device *vfe)
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

static void msm_vfe_wm_enable(struct vfe_device *vfe, u32 wm, u32 enable)
{
	if (enable)
		msm_vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm), 1);
	else
		msm_vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm), 1);
}

static void msm_vfe_wm_frame_based(struct vfe_device *vfe, u32 wm, u32 enable)
{
	if (enable) {
		msm_vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
	} else {
		msm_vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_FRM_BASED_SHIFT);
	}
}

static void msm_vfe_wm_set_framedrop_period(struct vfe_device *vfe,
					    u32 wm, u32 per)
{
	u32 reg;

	reg = readl(vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	reg &= ~(VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK);

	reg |= (per << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT) &
		VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK;

	writel(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));
}

static void msm_vfe_wm_set_framedrop_pattern(struct vfe_device *vfe,
					     u32 wm, u32 pat)
{
	writel(pat, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(wm));
}

static void msm_vfe_wm_set_ub_cfg(struct vfe_device *vfe, u32 wm,
				u16 offset, u16 depth)
{
	u32 reg;

	reg = (offset << VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT) | depth;
	writel(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
}

static void msm_vfe_bus_reload_wm(struct vfe_device *vfe, u32 wm)
{
	writel(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
}

static void msm_vfe_wm_set_ping_addr(struct vfe_device *vfe, u32 wm, u32 addr)
{
	writel(addr, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
}

static void msm_vfe_wm_set_pong_addr(struct vfe_device *vfe, u32 wm, u32 addr)
{
	writel(addr, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
}

static int msm_vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u32 wm)
{
	u32 reg;

	reg = readl(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (reg >> wm) & 0x1;
}

static void msm_vfe_bus_enable_wr_if(struct vfe_device *vfe, u32 enable)
{
	if (enable)
		writel(0x10000009, vfe->base + VFE_0_BUS_CFG);
	else
		writel(0, vfe->base + VFE_0_BUS_CFG);
}

static int msm_vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u32 wm, u32 rdi)
{
	u32 reg;

	reg = VFE_0_RDI_CFG_x_MIPI_EN_BITS;
	reg |= VFE_0_RDI_CFG_x_RDI_Mr_FRAME_BASED_EN(rdi);
	msm_vfe_reg_set(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	reg |= (wm << VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT) &
		VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK;
	msm_vfe_reg_set(vfe, VFE_0_RDI_CFG_x(rdi), reg);

	switch (rdi) {
	case 0:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case 1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	case 2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M0_SINGLE_STREAM_SEL_SHIFT;
		break;
	default:
		dev_err(vfe->camss->dev, "Invalid rdi %d\n", rdi);
		return -EINVAL;
	}

	writel(reg, vfe->base + VFE_0_BUS_XBAR_CFG_x(wm));

	writel(VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF,
	       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(wm));

	return 0;
}

static int msm_vfe_bus_dicconnect_wm_from_rdi(struct vfe_device *vfe, u32 rdi)
{
	writel(0x0, vfe->base + VFE_0_RDI_CFG_x(rdi));

	return 0;
}

static void msm_vfe_set_rdi_cid(struct vfe_device *vfe, u32 rdi_idx, u8 cid)
{
	msm_vfe_reg_clr(vfe,
		VFE_0_RDI_CFG_x(rdi_idx),
		VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK);

	msm_vfe_reg_set(vfe,
		VFE_0_RDI_CFG_x(rdi_idx),
		cid << VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT);
}

static int msm_vfe_reg_update(struct vfe_device *vfe, int rdi_idx)
{
	switch (rdi_idx) {
	case 0:
		writel(VFE_0_REG_UPDATE_RDI0, vfe->base + VFE_0_REG_UPDATE);
		break;
	case 1:
		writel(VFE_0_REG_UPDATE_RDI1, vfe->base + VFE_0_REG_UPDATE);
		break;
	case 2:
		writel(VFE_0_REG_UPDATE_RDI2, vfe->base + VFE_0_REG_UPDATE);
		break;
	default:
		dev_err(vfe->camss->dev, "Invalid vfe interface %d\n", rdi_idx);
		return -EINVAL;
	}

	return 0;
}

static void msm_vfe_irq_clear(struct vfe_device *vfe, u32 clr_0, u32 clr_1)
{
	writel(clr_0, vfe->base + VFE_0_IRQ_CLEAR_0);

	writel(clr_1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();

	writel(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
}

static void msm_vfe_enable_irq_0(struct vfe_device *vfe)
{
	u32 irq_en = VFE_0_IRQ_MASK_0_IMAGE_MASTER_0_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_1_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_2_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_3_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_4_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_5_PING_PONG |
		     VFE_0_IRQ_MASK_0_IMAGE_MASTER_6_PING_PONG |
		     VFE_0_IRQ_MASK_0_RESET_ACK;

	writel(irq_en, vfe->base + VFE_0_IRQ_MASK_0);
}

static void msm_vfe_enable_irq_1(struct vfe_device *vfe)
{
	u32 irq_en = VFE_0_IRQ_MASK_1_VIOLATION |
		     VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK |
		     VFE_0_IRQ_MASK_1_IMAGE_MASTER_0_BUS_OVERFLOW |
		     VFE_0_IRQ_MASK_1_IMAGE_MASTER_1_BUS_OVERFLOW |
		     VFE_0_IRQ_MASK_1_IMAGE_MASTER_2_BUS_OVERFLOW;

	writel(irq_en, vfe->base + VFE_0_IRQ_MASK_1);
}

static void msm_vfe_enable_irq_all(struct vfe_device *vfe)
{
	msm_vfe_irq_clear(vfe, VFE_0_IRQ_CLEAR_0_ALL, VFE_0_IRQ_CLEAR_1_ALL);
	msm_vfe_enable_irq_0(vfe);
	msm_vfe_enable_irq_1(vfe);
}

static void msm_vfe_disable_irq_all(struct vfe_device *vfe)
{

	writel(0x0, vfe->base + VFE_0_IRQ_MASK_0);
	writel(0x0, vfe->base + VFE_0_IRQ_MASK_1);
	msm_vfe_irq_clear(vfe, VFE_0_IRQ_CLEAR_0_ALL, VFE_0_IRQ_CLEAR_1_ALL);
}

static void msm_vfe_isr_wm_done(struct vfe_device *vfe, u32 wm_idx);

static irqreturn_t msm_vfe_subdev_isr(int irq, void *dev)
{
	struct vfe_device *vfe = dev;
	u32 value0, value1;

	value0 = readl(vfe->base + VFE_0_IRQ_STATUS_0);
	value1 = readl(vfe->base + VFE_0_IRQ_STATUS_1);

	writel(value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel(value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();
	writel(0x1, vfe->base + VFE_0_IRQ_CMD); // Apply IRQ Clear[01]

	if (value0 & VFE_0_IRQ_STATUS_0_RESET_ACK)
		complete_all(&vfe->reset_completion);

	if (value1 & VFE_0_IRQ_STATUS_1_BUS_BDG_HALT_ACK)
		complete_all(&vfe->halt_completion);

	if (value0 & VFE_0_IRQ_STATUS_0_IMAGE_MASTER_0_PING_PONG)
		msm_vfe_isr_wm_done(vfe, 0);

	return IRQ_HANDLED;
}

static int msm_vfe_reset(struct vfe_device *vfe)
{
	unsigned long time;

	init_completion(&vfe->reset_completion);

	msm_vfe_global_reset(vfe);

	time = wait_for_completion_timeout(&vfe->reset_completion,
		msecs_to_jiffies(MSM_VFE_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(vfe->camss->dev, "Vfe reset timeout\n");
		return -EIO;
	}

	return 0;
}

static int msm_vfe_halt(struct vfe_device *vfe)
{
	unsigned long time;

	init_completion(&vfe->halt_completion);

	writel(VFE_0_BUS_BDG_CMD_HALT_REQ, vfe->base + VFE_0_BUS_BDG_CMD);

	time = wait_for_completion_timeout(&vfe->halt_completion,
		msecs_to_jiffies(MSM_VFE_HALT_TIMEOUT_MS));
	if (!time) {
		dev_err(vfe->camss->dev, "Vfe halt timeout\n");
		return -EIO;
	}

	return 0;
}

static void msm_vfe_init_outputs(struct vfe_device *vfe)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->output); i++) {
		vfe->output[i].active_wm = 0;
		vfe->output[i].state = MSM_VFE_OUTPUT_OFF;
		vfe->output[i].buf[0] = NULL;
		vfe->output[i].buf[1] = NULL;
		INIT_LIST_HEAD(&vfe->output[i].pending_bufs);
	}
}

static void msm_vfe_reset_output_maps(struct vfe_device *vfe)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&vfe->output_lock, flags);

	for (i = 0; i < ARRAY_SIZE(vfe->rdi_output_map); i++)
		vfe->rdi_output_map[i] = -1;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++)
		vfe->wm_output_map[i] = -1;

	for (i = 0; i < ARRAY_SIZE(vfe->composite_output_map); i++)
		vfe->composite_output_map[i] = -1;

	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static void msm_vfe_set_qos(struct vfe_device *vfe)
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

static void msm_vfe_im_cgc_override(struct vfe_device *vfe)
{
	u32 val = VFE_0_CGC_OVERRIDE_1_IMAGE_M0_CGC_OVERRIDE;

	writel(val, vfe->base + VFE_0_CGC_OVERRIDE_1);
}

static void msm_vfe_output_init_addrs(struct vfe_device *vfe,
				      struct msm_vfe_output *output,
				      int sync)
{
	u32 ping_addr = 0;
	u32 pong_addr = 0;
	int i;

	output->active_buf = 0;

	if (output->buf[0])
		ping_addr = output->buf[0]->addr;

	if (output->buf[1])
		pong_addr = output->buf[1]->addr;
	else
		pong_addr = ping_addr;

	for (i = 0; i < output->active_wm; i++) {
		dev_err(vfe->camss->dev, "init_addrs: wm[%d], ping = 0x%08x, pong = 0x%08x\n",
			i, ping_addr, pong_addr);
		msm_vfe_wm_set_ping_addr(vfe, output->wm[i].wm_idx, ping_addr);
		msm_vfe_wm_set_pong_addr(vfe, output->wm[i].wm_idx, pong_addr);
		if (sync)
			msm_vfe_bus_reload_wm(vfe, output->wm[i].wm_idx);
	}
}

static void msm_vfe_output_reset_addrs(struct vfe_device *vfe,
				       struct msm_vfe_output *output)
{
	int i;

	for (i = 0; i < output->active_wm; i++) {
		msm_vfe_wm_set_ping_addr(vfe, output->wm[i].wm_idx, 0x00);
		msm_vfe_wm_set_pong_addr(vfe, output->wm[i].wm_idx, 0x00);
	}
}

static void msm_vfe_output_update_ping_addr(struct vfe_device *vfe,
					    struct msm_vfe_output *output,
					    int sync)
{
	u32 addr = 0;
	int i;

	if (output->buf[0])
		addr = output->buf[0]->addr;

	for (i = 0; i < output->active_wm; i++) {
		msm_vfe_wm_set_ping_addr(vfe, output->wm[i].wm_idx, addr);
		if (sync)
			msm_vfe_bus_reload_wm(vfe, output->wm[i].wm_idx);
	}
}

static void msm_vfe_output_update_pong_addr(struct vfe_device *vfe,
					    struct msm_vfe_output *output,
					    int sync)
{
	u32 addr = 0;
	int i;

	if (output->buf[1])
		addr = output->buf[1]->addr;

	for (i = 0; i < output->active_wm; i++) {
		msm_vfe_wm_set_pong_addr(vfe, output->wm[i].wm_idx, addr);
		if (sync)
			msm_vfe_bus_reload_wm(vfe, output->wm[i].wm_idx);
	}
}

static int __msm_vfe_reserve_rdi(struct vfe_device *vfe, u32 output_idx)
{
	int ret = -EBUSY;
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->rdi_output_map); i++) {
		if (vfe->rdi_output_map[i] < 0) {
			vfe->rdi_output_map[i] = output_idx;
			ret = i;
			break;
		}
	}
	return ret;
}

static int __msm_vfe_release_rdi(struct vfe_device *vfe, u32 rdi_idx)
{
	if (rdi_idx > ARRAY_SIZE(vfe->rdi_output_map))
		return -EINVAL;

	vfe->rdi_output_map[rdi_idx] = -1;

	return 0;
}

static int __msm_vfe_reserve_wm(struct vfe_device *vfe, u32 output_idx)
{
	int ret = -EBUSY;
	int i;

	for (i = 0; i < ARRAY_SIZE(vfe->wm_output_map); i++) {
		if (vfe->wm_output_map[i] < 0) {
			vfe->wm_output_map[i] = output_idx;
			ret = i;
			break;
		}
	}

	return ret;
}

static int __msm_vfe_release_wm(struct vfe_device *vfe, u32 wm_idx)
{
	if (wm_idx > ARRAY_SIZE(vfe->wm_output_map))
		return -EINVAL;

	vfe->wm_output_map[wm_idx] = -1;

	return 0;
}

/* Vfe hw buffer operations */
static struct msm_video_buffer *
__msm_vfe_get_next_output_buf(struct msm_vfe_output *output)
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
 * msm_vfe_output_frame_drop - Set frame drop pattern per given output
 * @vfe: Pointer to vfe device.
 * @output: Pointer to vfe output.
 * @drop_pattern: Kept (1) or dropped (0). The pattern starts from bit 0
 *   and progresses to bit 31.
 */
static void msm_vfe_output_frame_drop(struct vfe_device *vfe,
				      struct msm_vfe_output *output,
				      u32 drop_pattern)
{
	u32 drop_period;
	int i;

	/* We need to toggle update period to be valid on next frame */
	output->drop_update_idx++;
	output->drop_update_idx %= MSM_VFE_FRAME_DROP_UPDATES;
	drop_period = MSM_VFE_FRAME_DROP_VAL + output->drop_update_idx;

	for (i = 0; i < output->active_wm; i++) {
		msm_vfe_wm_set_framedrop_period(vfe, output->wm[i].wm_idx,
						drop_period);
		wmb();
		msm_vfe_wm_set_framedrop_pattern(vfe, output->wm[i].wm_idx,
						drop_pattern);
		wmb();
		msm_vfe_reg_update(vfe, output->wm[i].rdi_idx);
	}
}

/*
 * __msm_vfe_add_output_buf - Add output buffer to vfe output
 * @output: Pointer to vfe output.
 * @buffer: Pointer to video buffer.
 *
 * NOTE: Should be called with vfe locked.
 */
void __msm_vfe_add_output_buf(struct msm_vfe_output *output,
	struct msm_video_buffer *buffer)
{
	INIT_LIST_HEAD(&buffer->dma_queue);
	list_add_tail(&buffer->dma_queue, &output->pending_bufs);
}

/*
 * __msm_vfe_flush_output_bufs - Flush all pending out buffers.
 * @output: Pointer to vfe output.
 *
 * NOTE: Should be called with vfe locked.
 */
void __msm_vfe_flush_output_bufs(struct msm_vfe_output *output)
{
	struct msm_video_buffer *buf;
	struct msm_video_buffer *t;

	list_for_each_entry_safe(buf, t, &output->pending_bufs, dma_queue) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->dma_queue);
	}
}

static void __msm_vfe_update_wm_on_next_buf(struct vfe_device *vfe,
					    struct msm_vfe_output *output)
{
	switch (output->state) {
	case MSM_VFE_OUTPUT_CONTINUOUS:
		msm_vfe_output_frame_drop(vfe, output, 3);
		break;
	case MSM_VFE_OUTPUT_SINGLE:
		dev_err_ratelimited(vfe->camss->dev,
				    "Next buf in single state!\n");
		break;
	default:
		return;
	}
}

static void __msm_vfe_update_wm_on_last_buf(struct vfe_device *vfe,
					    struct msm_vfe_output *output)
{
	switch (output->state) {
	case MSM_VFE_OUTPUT_CONTINUOUS:
		output->state = MSM_VFE_OUTPUT_SINGLE;
		msm_vfe_output_frame_drop(vfe, output, 1);
		break;
	case MSM_VFE_OUTPUT_SINGLE:
		output->state = MSM_VFE_OUTPUT_IDLE;
		msm_vfe_output_frame_drop(vfe, output, 0);
		msm_vfe_output_reset_addrs(vfe, output);
		break;
	default:
		dev_err_ratelimited(vfe->camss->dev,
				    "Last buff in wrong state! %d\n",
				    output->state);
		return;
	}
}

static void __msm_vfe_update_wm_on_new_buf(struct vfe_device *vfe,
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
				msm_vfe_output_update_pong_addr(vfe,
								output, 0);
			else
				msm_vfe_output_update_ping_addr(vfe,
								output, 0);

			msm_vfe_output_frame_drop(vfe, output, 3);
			output->state = MSM_VFE_OUTPUT_CONTINUOUS;
		} else {
			__msm_vfe_add_output_buf(output, new_buf);
			dev_err_ratelimited(vfe->camss->dev,
					    "Inactive buffer is busy\n");
		}
		break;

	case MSM_VFE_OUTPUT_IDLE:
		if (!output->buf[0]) {
			output->buf[0] = new_buf;

			msm_vfe_output_init_addrs(vfe, output, 1);

			/* After wm reload we can not skip second frame.
			 * Capture only second frame to avoid iommu fault */
			msm_vfe_output_frame_drop(vfe, output, 2);
			output->state = MSM_VFE_OUTPUT_SINGLE;
		} else {
			__msm_vfe_add_output_buf(output, new_buf);
			dev_err_ratelimited(vfe->camss->dev,
					    "Output idle with buffer set!\n");
		}
		break;

	case MSM_VFE_OUTPUT_CONTINUOUS:

	default:
		__msm_vfe_add_output_buf(output, new_buf);
		return;
	}
}

static struct msm_vfe_output* msm_vfe_get_output(struct vfe_device *vfe,
						 u32 output_idx)
{
	struct msm_vfe_output *output;
	unsigned long flags;
	int wm_idx;
	int rdi_idx;

	if (output_idx > ARRAY_SIZE(vfe->output))
		return ERR_PTR(-EINVAL);

	spin_lock_irqsave(&vfe->output_lock, flags);

	output = &vfe->output[output_idx];
	if (output->state != MSM_VFE_OUTPUT_OFF) {
		dev_err(vfe->camss->dev, "Output is running\n");
		goto error;
	}
	output->state = MSM_VFE_OUTPUT_RESERVED;

	output->active_buf = 0;

	rdi_idx = __msm_vfe_reserve_rdi(vfe, output_idx);
	if (rdi_idx < 0) {
		dev_err(vfe->camss->dev, "Can not reserve rdi\n");
		goto error_get_rdi;
	}

	/* We will use only one wm per output for now */
	wm_idx = __msm_vfe_reserve_wm(vfe, output_idx);
	if (wm_idx < 0) {
		dev_err(vfe->camss->dev, "Can not reserve wm\n");
		goto error_get_wm;
	}
	output->active_wm = 1;
	output->drop_update_idx = 0;
	output->wm[0].wm_idx = wm_idx;
	output->wm[0].rdi_idx = rdi_idx;

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return output;

error_get_wm:
	__msm_vfe_release_rdi(vfe, rdi_idx);
error_get_rdi:
	output->state = MSM_VFE_OUTPUT_OFF;
error:
	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return ERR_PTR(-EINVAL);
}

static int msm_vfe_put_output(struct vfe_device *vfe,
			      struct msm_vfe_output *output)
{
	struct msm_vfe_wm *wm;
	unsigned long flags;
	int ret;
	int i;

	spin_lock_irqsave(&vfe->output_lock, flags);

	for (i = 0; i < output->active_wm; i++) {
		wm = &output->wm[i];

		ret = __msm_vfe_release_wm(vfe, wm->wm_idx);
		if (ret < 0)
			goto out;

		ret = __msm_vfe_release_rdi(vfe, wm->rdi_idx);
		if (ret < 0)
			goto out;
	}

	output->state = MSM_VFE_OUTPUT_OFF;
	output->active_wm = 0;

out:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
	return ret;
}

static int msm_vfe_enable_output(struct vfe_device *vfe,
				 struct msm_vfe_output *output,
				 u32 ub_size)
{
	struct msm_vfe_wm *wm;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (output->state != MSM_VFE_OUTPUT_RESERVED) {
		dev_err(vfe->camss->dev, "Output is not in reserved state %d\n",
			output->state);
		spin_unlock_irqrestore(&vfe->output_lock, flags);
		return -EINVAL;
	}
	output->state = MSM_VFE_OUTPUT_IDLE;

	output->buf[0] = __msm_vfe_get_next_output_buf(output);
	if (output->buf[0])
		output->state = MSM_VFE_OUTPUT_SINGLE;

	output->buf[1] = __msm_vfe_get_next_output_buf(output);
	if (output->buf[1])
		output->state = MSM_VFE_OUTPUT_CONTINUOUS;

	msm_vfe_set_qos(vfe);

	switch (output->state) {
	case MSM_VFE_OUTPUT_SINGLE:
		/* After wm reload we can not skip second frame.
		 * Capture only second frame to avoid iommu fault */
		/* Skip 4 bad frames from sensor TODO: get number from sensor */
		msm_vfe_output_frame_drop(vfe, output, 2 << 4);
		break;
	case MSM_VFE_OUTPUT_CONTINUOUS:
		/* Skip 4 bad frames from sensor TODO: get number from sensor */
		msm_vfe_output_frame_drop(vfe, output, 3 << 4);
		break;
	default:
		msm_vfe_output_frame_drop(vfe, output, 0);
		break;
	}

	msm_vfe_output_init_addrs(vfe, output, 0);

	for (i = 0; i < output->active_wm; i++) {
		wm = &output->wm[i];

		msm_vfe_bus_connect_wm_to_rdi(vfe, wm->wm_idx, wm->rdi_idx);

		msm_vfe_set_rdi_cid(vfe, wm->rdi_idx, wm->rdi_idx);

		msm_vfe_wm_set_ub_cfg(vfe, wm->wm_idx,
			(ub_size * wm->wm_idx), ub_size);

		msm_vfe_wm_frame_based(vfe, wm->wm_idx, 1);
		msm_vfe_wm_enable(vfe, wm->wm_idx, 1);

		msm_vfe_bus_reload_wm(vfe, output->wm[i].wm_idx);

		msm_vfe_reg_update(vfe, wm->rdi_idx);
	}

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int msm_vfe_disable_output(struct vfe_device *vfe,
				  struct msm_vfe_output *output)
{
	struct msm_vfe_wm *wm;
	int i;

	for (i = 0; i <  output->active_wm; i++) {
		wm = &output->wm[i];
		msm_vfe_wm_enable(vfe, wm->wm_idx, 0);
		msm_vfe_bus_dicconnect_wm_from_rdi(vfe, wm->rdi_idx);
		msm_vfe_reg_update(vfe, wm->rdi_idx);
	}

	return 0;
}

static int msm_vfe_enable_all_outputs(struct vfe_device *vfe)
{
	struct msm_vfe_output *output;
	u32 ub_size;
	int ret;
	int i;

	mutex_lock(&vfe->mutex);

	if (!vfe->stream_cnt)
		return -EINVAL;

	switch (vfe->hw_id) {
	case 0:
		ub_size = MSM_VFE_UB_MAX_SIZE_VFE0;
		break;
	case 1:
		ub_size = MSM_VFE_UB_MAX_SIZE_VFE1;
		break;
	default:
		return -EINVAL;
	}
	ub_size /= vfe->stream_cnt;

	msm_vfe_im_cgc_override(vfe);
	wmb();

	/* Bus interface should be enabled first */
	msm_vfe_bus_enable_wr_if(vfe, 1);

	for (i = 0; i < vfe->stream_cnt; i++) {
		output = msm_vfe_get_output(vfe, i);
		if (IS_ERR_OR_NULL(output))
			goto error;

		ret = msm_vfe_enable_output(vfe, output, ub_size);
		if (ret < 0)
			goto error;
	}
	vfe->active_outputs = i;

	mutex_unlock(&vfe->mutex);

	return 0;

error:
	msm_vfe_bus_enable_wr_if(vfe, 0);

	for (; i > 0; i--)
		msm_vfe_put_output(vfe, &vfe->output[i - 1]);

	mutex_unlock(&vfe->mutex);

	return ret;
}

static int msm_vfe_disable_all_outputs(struct vfe_device *vfe)
{
	int i;

	mutex_lock(&vfe->mutex);

	msm_vfe_bus_enable_wr_if(vfe, 0);

	for (i = 0; i < vfe->active_outputs; i++) {
		msm_vfe_disable_output(vfe, &vfe->output[i]);
		msm_vfe_put_output(vfe, &vfe->output[i]);
	}

	msm_vfe_halt(vfe);

	vfe->active_outputs = 0;

	mutex_unlock(&vfe->mutex);

	return 0;
}

static void msm_vfe_isr_wm_done(struct vfe_device *vfe, u32 wm_idx)
{
	struct msm_video_buffer *ready_buf;
	struct msm_vfe_output *output;
	dma_addr_t new_addr;
	unsigned long flags;
	u32 active_index;

	active_index = msm_vfe_wm_get_ping_pong_status(vfe, wm_idx);

	spin_lock_irqsave(&vfe->output_lock, flags);

	if (vfe->wm_output_map[wm_idx] < 0) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm done for unmapped index\n");
		goto out_unlock;
	}
	output = &vfe->output[vfe->wm_output_map[wm_idx]];

	if (output->active_buf == active_index) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Active buffer mismatch!\n");
		goto out_unlock;
	}
	output->active_buf = active_index;

	ready_buf = output->buf[!active_index];
	if (!ready_buf) {
		dev_err_ratelimited(vfe->camss->dev,
				    "Missing ready buf %d %d!\n",
				    !active_index, output->state);
		goto out_unlock;
	}

	/* Get next buffer */
	output->buf[!active_index] = __msm_vfe_get_next_output_buf(output);
	if (!output->buf[!active_index]) {
		new_addr = 0;
		__msm_vfe_update_wm_on_last_buf(vfe, output);
	} else {
		new_addr = output->buf[!active_index]->addr;
		__msm_vfe_update_wm_on_next_buf(vfe, output);
	}

	if (active_index)
		msm_vfe_wm_set_ping_addr(vfe, wm_idx, new_addr);
	else
		msm_vfe_wm_set_pong_addr(vfe, wm_idx, new_addr);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	if (ready_buf)
		vb2_buffer_done(&ready_buf->vb, VB2_BUF_STATE_DONE);
	else
		dev_err_ratelimited(vfe->camss->dev,
				    "Received wm without buffer\n");

	return;

out_unlock:
	spin_unlock_irqrestore(&vfe->output_lock, flags);
}

static int msm_vfe_bus_request(struct vfe_device *vfe)
{
	int ret;

	vfe->bus_client = msm_bus_scale_register_client(vfe->bus_scale_table);
	if (!vfe->bus_client) {
		dev_err(vfe->camss->dev, "Failed to register bus client\n");
		return -ENOENT;
	}

	ret = msm_bus_scale_client_update_request(vfe->bus_client, 1);
	if (ret < 0) {
		dev_err(vfe->camss->dev, "Failed bus scale update %d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static void msm_vfe_bus_release(struct vfe_device *vfe)
{
	if (vfe->bus_client) {
		msm_bus_scale_unregister_client(vfe->bus_client);
		vfe->bus_client = 0;
	}
}

static int msm_vfe_set_clock_rate(struct vfe_device *vfe)
{
	int ret;
	long clk_rate;

	// TODO
	clk_rate = clk_round_rate(vfe->clocks[1].clk, 465000000);
	if (clk_rate < 0) {
		dev_err(vfe->camss->dev, "clk round failed\n");
		return -EINVAL;
	}
	ret = clk_set_rate(vfe->clocks[1].clk, clk_rate);
	if (ret < 0) {
		dev_err(vfe->camss->dev, "clk set rate failed\n");
		return -EINVAL;
	}

	return 0;
}

static int msm_vfe_enable_clocks(struct vfe_device *vfe)
{
	int i;
	int ret;

	for (i = 0; i < vfe->nclocks; i++) {
		ret = clk_prepare_enable(vfe->clocks[i].clk);
		if (ret < 0) {
			dev_err(vfe->camss->dev,
				"clock prepare_enable failed %d\n", i);
			goto error;
		}
	}

	return 0;

error:
	for (; i > 0; i--) {
		clk_disable_unprepare(vfe->clocks[i - 1].clk);
	}
	return ret;
}

static void msm_vfe_disable_clocks(struct vfe_device *vfe)
{
	int i;

	for (i = vfe->nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(vfe->clocks[i].clk);
}

static int msm_vfe_get(struct vfe_device *vfe)
{
	int ret;

	mutex_lock(&vfe->mutex);

	if (vfe->ref_count == 0) {
		msm_vfe_reset_output_maps(vfe);

		ret = msm_vfe_bus_request(vfe);
		if (ret < 0) {
			dev_err(vfe->camss->dev, "Fail bus request\n");
			goto error_clocks;
		}

		ret = msm_vfe_set_clock_rate(vfe);
		if (ret < 0) {
			dev_err(vfe->camss->dev, "Fail to set clocks rate\n");
			goto error_clocks;
		}

		ret = msm_vfe_enable_clocks(vfe);
		if (ret < 0) {
			dev_err(vfe->camss->dev, "Fail to enable clocks\n");
			goto error_clocks;
		}
	}
	vfe->ref_count++;

	mutex_unlock(&vfe->mutex);

	return 0;

error_clocks:
	mutex_unlock(&vfe->mutex);
	return ret;
}

static void msm_vfe_put(struct vfe_device *vfe)
{
	mutex_lock(&vfe->mutex);
	BUG_ON(vfe->ref_count == 0);

	if (--vfe->ref_count == 0) {
		msm_vfe_disable_irq_all(vfe);
		msm_vfe_init_outputs(vfe);
		msm_vfe_bus_release(vfe);
		msm_vfe_disable_clocks(vfe);
	}
	mutex_unlock(&vfe->mutex);
}

static int msm_vfe_queue_dmabuf(struct camss_video *vid,
				struct msm_video_buffer *buf)
{
	struct vfe_device *vfe = &vid->camss->vfe;
	struct msm_vfe_output *output;
	unsigned long flags;
	int idx;

	idx = 0; // TODO: msm_vfe_pad_to_output(vfe, vid->pad_idx);
	if (idx < 0) {
		dev_err(vfe->camss->dev,
			"Can not queue dma buf invalid pad idx\n");
		return idx;
	}
	output = &vfe->output[idx];

	spin_lock_irqsave(&vfe->output_lock, flags);

	__msm_vfe_update_wm_on_new_buf(vfe, output, buf);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int msm_vfe_flush_dmabufs(struct camss_video *vid)
{
	struct vfe_device *vfe = &vid->camss->vfe;
	struct msm_vfe_output *output;
	unsigned long flags;
	int idx;

	idx = 0; // TODO: msm_vfe_pad_to_output(vfe, vid->pad_idx);
	if (idx < 0) {
		dev_err(vfe->camss->dev,
			"Can not flush dma buf invalid pad idx\n");
		return idx;
	}
	output = &vfe->output[idx];

	spin_lock_irqsave(&vfe->output_lock, flags);

	__msm_vfe_flush_output_bufs(output);

	if (output->buf[0])
		vb2_buffer_done(&output->buf[0]->vb, VB2_BUF_STATE_ERROR);

	if (output->buf[1])
		vb2_buffer_done(&output->buf[1]->vb, VB2_BUF_STATE_ERROR);

	spin_unlock_irqrestore(&vfe->output_lock, flags);

	return 0;
}

static int msm_vfe_subdev_set_power(struct v4l2_subdev *sd, int on)
{
	struct vfe_device *vfe = v4l2_get_subdevdata(sd);
	int ret;

	dev_err(vfe->camss->dev, "%s: Enter, on = %d\n",
		__func__, on);

	if (on) {
		u32 hw_version;

		ret = msm_vfe_get(vfe);
		if (ret < 0)
			return ret;


		hw_version = readl(vfe->base);
		dev_err(vfe->camss->dev,
			"VFE HW Version = 0x%08x\n", hw_version);
	} else {
		msm_vfe_put(vfe);
	}

	dev_err(vfe->camss->dev, "%s: Exit, on = %d\n",
		__func__, on);

	return 0;
}

static int msm_vfe_subdev_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct vfe_device *vfe = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_err(vfe->camss->dev, "%s: Enter, enable = %d\n",
		__func__, enable);

	if (enable) {
		mutex_lock(&vfe->mutex);

		ret = msm_vfe_reset(vfe);
		if (ret < 0) {
			dev_err(vfe->camss->dev, "Fail to reset vfe\n");
			return ret;
		}

		msm_vfe_enable_irq_all(vfe);

		mutex_unlock(&vfe->mutex);

		ret = msm_vfe_enable_all_outputs(vfe);
		if (ret < 0)
			dev_err(vfe->camss->dev,
				"Fail to enable vfe outputs\n");
	} else {
		ret = msm_vfe_disable_all_outputs(vfe);
		if (ret < 0)
			dev_err(vfe->camss->dev,
				"Fail to disable vfe outputs\n");
	}

	return 0;
}

int msm_vfe_subdev_init(struct vfe_device *vfe, struct camss *camss,
			struct vfe_init *init)
{
	struct device *dev = camss->dev;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct resource *r;
	struct dma_iommu_mapping *mapping;
	int i;
	int ret;

	mutex_init(&vfe->mutex);
	spin_lock_init(&vfe->output_lock);

	vfe->hw_id = 0; // TODO

	vfe->camss = camss;
	vfe->init = *init;

	vfe->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vfe->video_out.camss = camss;

	// Temp:
#define FMT_WIDTH 1920
#define FMT_HEIGHT 1080
	vfe->stream_cnt = 1;
	vfe->video_out.active_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vfe->video_out.active_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	vfe->video_out.active_fmt.fmt.pix.width = FMT_WIDTH;
	vfe->video_out.active_fmt.fmt.pix.height = FMT_HEIGHT;
	vfe->video_out.active_fmt.fmt.pix.bytesperline = FMT_WIDTH * 2;
	vfe->video_out.active_fmt.fmt.pix.sizeimage = FMT_WIDTH * FMT_HEIGHT * 2;
	vfe->video_out.active_fmt.fmt.pix.field = V4L2_FIELD_NONE;
	vfe->video_out.active_fmt.fmt.pix.colorspace = V4L2_COLORSPACE_JPEG;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, reg);
	vfe->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(vfe->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(vfe->base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, reg_vbif);
	vfe->base_vbif = devm_ioremap_resource(dev, r);
	if (IS_ERR(vfe->base_vbif)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(vfe->base_vbif);
	}

	/* Clocks */

	vfe->nclocks = ARRAY_SIZE(clocks);
	vfe->clocks = devm_kzalloc(dev, vfe->nclocks * sizeof(*vfe->clocks),
				    GFP_KERNEL);
	if (!vfe->clocks) {
		dev_err(dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < vfe->nclocks; i++) {
		vfe->clocks[i].name = clocks[i];
	}

	for (i = 0; i < vfe->nclocks; i++) {
		vfe->clocks[i].clk = devm_clk_get(dev, vfe->clocks[i].name);
		if (IS_ERR(vfe->clocks[i].clk))
			return PTR_ERR(vfe->clocks[i].clk);
	}

	/* IOMMU */

	vfe->camss->iommu_dev = msm_iommu_get_ctx("vfe");
	if (IS_ERR(vfe->camss->iommu_dev)) {
		dev_err(dev, "Cannot find iommu nonsecure ctx\n");
		return PTR_ERR(vfe->camss->iommu_dev);
	}

	mapping = arm_iommu_create_mapping(&platform_bus_type,
					   0x40000000, 0xC0000000, 0);
	if (IS_ERR_OR_NULL(mapping))
		return PTR_ERR(mapping) ?: -ENODEV;

	ret = arm_iommu_attach_device(vfe->camss->iommu_dev, mapping);
	if (ret)
		return -1;

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, interrupt);
	vfe->irq = r->start;
	if (IS_ERR_VALUE(vfe->irq))
		return vfe->irq;

	ret = devm_request_irq(dev, vfe->irq, msm_vfe_subdev_isr,
			       IRQF_TRIGGER_RISING, "vfe", vfe);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	/* MSM Bus */

	vfe->bus_scale_table = msm_bus_cl_get_pdata(pdev);
	if (!vfe->bus_scale_table) {
		dev_err(dev, "bus scaling is disabled\n");
		return -1;
	}

	msm_vfe_init_outputs(vfe);

	return 0;
}

static const struct v4l2_subdev_core_ops msm_vfe_core_ops = {
	.s_power = msm_vfe_subdev_set_power,
};

static const struct v4l2_subdev_video_ops msm_vfe_video_ops = {
	.s_stream = msm_vfe_subdev_set_stream,
};

static const struct v4l2_subdev_pad_ops msm_vfe_pad_ops;

static const struct v4l2_subdev_ops msm_vfe_ops = {
	.core = &msm_vfe_core_ops,
	.video = &msm_vfe_video_ops,
	.pad = &msm_vfe_pad_ops,
};

static const struct v4l2_subdev_internal_ops msm_vfe_internal_ops;

static struct msm_video_ops rdi_video_ops = {
	.queue_dmabuf = msm_vfe_queue_dmabuf,
	.flush_dmabufs = msm_vfe_flush_dmabufs,
};


int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &vfe->subdev;
	struct media_pad *pads = vfe->pads;
	int ret;

	v4l2_subdev_init(sd, &msm_vfe_ops);
	sd->internal_ops = &msm_vfe_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), MSM_VFE_DRV_NAME);
	v4l2_set_subdevdata(sd, vfe);

	pads[MSM_VFE_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_VFE_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

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

	vfe->video_out.ops = &rdi_video_ops;
	ret = msm_video_register(&vfe->video_out, v4l2_dev, MSM_VFE_VIDEO_NAME);
	if (ret < 0)
		goto error_reg_video;

	ret = media_entity_create_link(
			&vfe->subdev.entity, MSM_VFE_PAD_SRC,
			&vfe->video_out.video.entity, 0,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret < 0) {
		pr_err("Fail to link %s->%s entities\n",
			vfe->subdev.entity.name,
			vfe->video_out.video.entity.name);
		goto error_link;
	}

	return 0;

error_link:
	msm_video_unregister(&vfe->video_out);
error_reg_video:
	v4l2_device_unregister_subdev(sd);
error_reg_subdev:
	media_entity_cleanup(&sd->entity);
error_init_entity:

	return ret;
}

void msm_vfe_unregister_entities(struct vfe_device *vfe)
{
	v4l2_device_unregister_subdev(&vfe->subdev);
	msm_video_unregister(&vfe->video_out);
}
