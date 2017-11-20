/*
 * camss-vfe-4-7.c
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v4.7
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2017 Linaro Ltd.
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

#include <linux/iopoll.h>

#include "camss-vfe.h"


#define VFE_0_HW_VERSION		0x000

#define VFE_0_GLOBAL_RESET_CMD		0x018
#define VFE_0_GLOBAL_RESET_CMD_CORE	(1 << 0)
#define VFE_0_GLOBAL_RESET_CMD_CAMIF	(1 << 1)
#define VFE_0_GLOBAL_RESET_CMD_BUS	(1 << 2)
#define VFE_0_GLOBAL_RESET_CMD_BUS_BDG	(1 << 3)
#define VFE_0_GLOBAL_RESET_CMD_REGISTER	(1 << 4)
#define VFE_0_GLOBAL_RESET_CMD_PM	(1 << 5)
#define VFE_0_GLOBAL_RESET_CMD_BUS_MISR	(1 << 6)
#define VFE_0_GLOBAL_RESET_CMD_TESTGEN	(1 << 7)
#define VFE_0_GLOBAL_RESET_CMD_DSP	(1 << 8)
#define VFE_0_GLOBAL_RESET_CMD_IDLE_CGC	(1 << 9)

#define VFE_0_IRQ_CMD			0x058
#define VFE_0_IRQ_CMD_GLOBAL_CLEAR	(1 << 0)

#define VFE_0_IRQ_MASK_0		0x05c
#define VFE_0_IRQ_MASK_0_RDIn_REG_UPDATE(n)		(1 << ((n) + 5))
#define VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(n)		\
	((n) == VFE_LINE_PIX ? (1 << 4) : VFE_0_IRQ_MASK_0_RDIn_REG_UPDATE(n))
#define VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(n)	(1 << ((n) + 8))
#define VFE_0_IRQ_MASK_0_RESET_ACK			(1 << 31)
#define VFE_0_IRQ_MASK_1		0x060
#define VFE_0_IRQ_MASK_1_VIOLATION			(1 << 7)
#define VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK		(1 << 8)
#define VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(n)	(1 << ((n) + 9))
#define VFE_0_IRQ_MASK_1_RDIn_SOF(n)			(1 << ((n) + 29))

#define VFE_0_IRQ_CLEAR_0		0x064
#define VFE_0_IRQ_CLEAR_1		0x068

#define VFE_0_IRQ_STATUS_0		0x06c
#define VFE_0_IRQ_STATUS_1		0x070

#define VFE_0_VIOLATION_STATUS		0x07c

#define VFE_0_BUS_CMD			0x80
#define VFE_0_BUS_CMD_Mx_RLD_CMD(x)	(1 << (x))

#define VFE_0_BUS_CFG			0x084

#define VFE_0_BUS_XBAR_CFG_x(x)		(0x90 + 0x4 * ((x) / 2))
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT		8
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0	0xc
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1	0xd
#define VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2	0xe

#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(n)		(0x0a0 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT	0
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(n)	(0x0a4 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(n)	(0x0ac + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(n)		(0x0b4 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_BASED_SHIFT	1
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT	2
#define VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK	(0x1f << 2)
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(n)		(0x0b8 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT	16
#define VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(n)	\
							(0x0c4 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(n)	\
							(0x0c8 + 0x2c * (n))
#define VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF	0xffffffff

#define VFE_0_BUS_PING_PONG_STATUS	0x338

#define VFE_0_BUS_BDG_CMD		0x400
#define VFE_0_BUS_BDG_CMD_HALT_REQ	1

#define VFE_0_BUS_BDG_QOS_CFG_0		0x404
#define VFE_0_BUS_BDG_QOS_CFG_0_CFG	0xaaa9aaa9
#define VFE_0_BUS_BDG_QOS_CFG_1		0x408
#define VFE_0_BUS_BDG_QOS_CFG_2		0x40c
#define VFE_0_BUS_BDG_QOS_CFG_3		0x410
#define VFE_0_BUS_BDG_QOS_CFG_4		0x414
#define VFE_0_BUS_BDG_QOS_CFG_5		0x418
#define VFE_0_BUS_BDG_QOS_CFG_6		0x41c
#define VFE_0_BUS_BDG_QOS_CFG_7		0x420
#define VFE_0_BUS_BDG_QOS_CFG_7_CFG	0x0001aaa9

#define VFE_0_BUS_BDG_DS_CFG_0		0x424
#define VFE_0_BUS_BDG_DS_CFG_0_CFG	0xcccc0011
#define VFE_0_BUS_BDG_DS_CFG_1		0x428
#define VFE_0_BUS_BDG_DS_CFG_2		0x42c
#define VFE_0_BUS_BDG_DS_CFG_3		0x430
#define VFE_0_BUS_BDG_DS_CFG_4		0x434
#define VFE_0_BUS_BDG_DS_CFG_5		0x438
#define VFE_0_BUS_BDG_DS_CFG_6		0x43c
#define VFE_0_BUS_BDG_DS_CFG_7		0x440
#define VFE_0_BUS_BDG_DS_CFG_8		0x444
#define VFE_0_BUS_BDG_DS_CFG_9		0x448
#define VFE_0_BUS_BDG_DS_CFG_10		0x44c
#define VFE_0_BUS_BDG_DS_CFG_11		0x450
#define VFE_0_BUS_BDG_DS_CFG_12		0x454
#define VFE_0_BUS_BDG_DS_CFG_13		0x458
#define VFE_0_BUS_BDG_DS_CFG_14		0x45c
#define VFE_0_BUS_BDG_DS_CFG_15		0x460
#define VFE_0_BUS_BDG_DS_CFG_16		0x464
#define VFE_0_BUS_BDG_DS_CFG_16_CFG	0x40000103

#define VFE_0_RDI_CFG_x(x)		(0x46c + (0x4 * (x)))
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT	28
#define VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK	(0xf << 28)
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT	4
#define VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK		(0xf << 4)
#define VFE_0_RDI_CFG_x_RDI_EN_BIT		(1 << 2)
#define VFE_0_RDI_CFG_x_MIPI_EN_BITS		0x3

#define VFE_0_REG_UPDATE		0x4ac
#define VFE_0_REG_UPDATE_RDIn(n)		(1 << (1 + (n)))
#define VFE_0_REG_UPDATE_line_n(n)		\
			((n) == VFE_LINE_PIX ? 1 : VFE_0_REG_UPDATE_RDIn(n))

#define MSM_VFE_VFE0_UB_SIZE 2047
#define MSM_VFE_VFE0_UB_SIZE_RDI (MSM_VFE_VFE0_UB_SIZE / 3)
#define MSM_VFE_VFE1_UB_SIZE 1535
#define MSM_VFE_VFE1_UB_SIZE_RDI (MSM_VFE_VFE1_UB_SIZE / 3)


static void vfe_hw_version_read(struct vfe_device *vfe, struct device *dev)
{
	u32 hw_version = readl_relaxed(vfe->base + VFE_0_HW_VERSION);

	dev_err(dev, "VFE HW Version = 0x%08x\n", hw_version);
}

static u16 vfe_get_ub_size(u8 vfe_id)
{
	if (vfe_id == 0)
		return MSM_VFE_VFE0_UB_SIZE_RDI;
	else if (vfe_id == 1)
		return MSM_VFE_VFE1_UB_SIZE_RDI;

	return 0;
}

static inline void vfe_reg_clr(struct vfe_device *vfe, u32 reg, u32 clr_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits & ~clr_bits, vfe->base + reg);
}

static inline void vfe_reg_set(struct vfe_device *vfe, u32 reg, u32 set_bits)
{
	u32 bits = readl_relaxed(vfe->base + reg);

	writel_relaxed(bits | set_bits, vfe->base + reg);
}

static void vfe_global_reset(struct vfe_device *vfe)
{
	u32 reset_bits = VFE_0_GLOBAL_RESET_CMD_IDLE_CGC	|
			 VFE_0_GLOBAL_RESET_CMD_DSP		|
			 VFE_0_GLOBAL_RESET_CMD_TESTGEN		|
			 VFE_0_GLOBAL_RESET_CMD_BUS_MISR	|
			 VFE_0_GLOBAL_RESET_CMD_PM		|
			 VFE_0_GLOBAL_RESET_CMD_REGISTER	|
			 VFE_0_GLOBAL_RESET_CMD_BUS_BDG		|
			 VFE_0_GLOBAL_RESET_CMD_BUS		|
			 VFE_0_GLOBAL_RESET_CMD_CAMIF		|
			 VFE_0_GLOBAL_RESET_CMD_CORE;

	writel_relaxed(BIT(31), vfe->base + VFE_0_IRQ_MASK_0);
	wmb();
	writel_relaxed(reset_bits, vfe->base + VFE_0_GLOBAL_RESET_CMD);
}

static void vfe_halt_request(struct vfe_device *vfe)
{
	writel_relaxed(VFE_0_BUS_BDG_CMD_HALT_REQ,
		       vfe->base + VFE_0_BUS_BDG_CMD);
}

static void vfe_halt_clear(struct vfe_device *vfe)
{
	writel_relaxed(0x0, vfe->base + VFE_0_BUS_BDG_CMD);
}

static void vfe_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			    1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_CFG(wm),
			    1 << VFE_0_BUS_IMAGE_MASTER_n_WR_CFG_WR_PATH_SHIFT);
}

static void vfe_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable)
{
	if (enable)
		vfe_reg_set(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_BASED_SHIFT);
	else
		vfe_reg_clr(vfe, VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm),
			1 << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_BASED_SHIFT);
}

static void vfe_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm, u8 per)
{
	u32 reg;

	reg = readl_relaxed(vfe->base +
			    VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));

	reg &= ~(VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK);

	reg |= (per << VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_SHIFT)
		& VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG_FRM_DROP_PER_MASK;

	writel_relaxed(reg,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_ADDR_CFG(wm));
}

static void vfe_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm,
					 u32 pattern)
{
	writel_relaxed(pattern,
	       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_FRAMEDROP_PATTERN(wm));
}

static void vfe_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm,
			      u16 offset, u16 depth)
{
	u32 reg;

	reg = (offset << VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG_OFFSET_SHIFT) |
		depth;
	writel_relaxed(reg, vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_UB_CFG(wm));
}

static void vfe_bus_reload_wm(struct vfe_device *vfe, u8 wm)
{
	wmb();
	writel_relaxed(VFE_0_BUS_CMD_Mx_RLD_CMD(wm), vfe->base + VFE_0_BUS_CMD);
	wmb();
}

static void vfe_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PING_ADDR(wm));
}

static void vfe_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr)
{
	writel_relaxed(addr,
		       vfe->base + VFE_0_BUS_IMAGE_MASTER_n_WR_PONG_ADDR(wm));
}

static int vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm)
{
	u32 reg;

	reg = readl_relaxed(vfe->base + VFE_0_BUS_PING_PONG_STATUS);

	return (reg >> wm) & 0x1;
}

static void vfe_bus_enable_wr_if(struct vfe_device *vfe, u8 enable)
{
	if (enable)
		writel_relaxed(0x101, vfe->base + VFE_0_BUS_CFG);
	else
		writel_relaxed(0, vfe->base + VFE_0_BUS_CFG);
}

static void vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u8 wm,
				      enum vfe_line_id id)
{
	u32 reg;

	reg = VFE_0_RDI_CFG_x_MIPI_EN_BITS;
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(0), reg);

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	reg |= ((3 * id) << VFE_0_RDI_CFG_x_RDI_STREAM_SEL_SHIFT) &
		VFE_0_RDI_CFG_x_RDI_STREAM_SEL_MASK;
	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
	default:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_set(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);
}

static void vfe_wm_set_subsample(struct vfe_device *vfe, u8 wm)
{
	writel_relaxed(VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN_DEF,
	       vfe->base +
	       VFE_0_BUS_IMAGE_MASTER_n_WR_IRQ_SUBSAMPLE_PATTERN(wm));
}

static void vfe_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
					   enum vfe_line_id id)
{
	u32 reg;

	reg = VFE_0_RDI_CFG_x_RDI_EN_BIT;
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id), reg);

	switch (id) {
	case VFE_LINE_RDI0:
	default:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI0 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI1:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI1 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	case VFE_LINE_RDI2:
		reg = VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_VAL_RDI2 <<
		      VFE_0_BUS_XBAR_CFG_x_M_SINGLE_STREAM_SEL_SHIFT;
		break;
	}

	if (wm % 2 == 1)
		reg <<= 16;

	vfe_reg_clr(vfe, VFE_0_BUS_XBAR_CFG_x(wm), reg);
}

static void vfe_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id, u8 cid)
{
	vfe_reg_clr(vfe, VFE_0_RDI_CFG_x(id),
		    VFE_0_RDI_CFG_x_RDI_M0_SEL_MASK);

	vfe_reg_set(vfe, VFE_0_RDI_CFG_x(id),
		    cid << VFE_0_RDI_CFG_x_RDI_M0_SEL_SHIFT);
}

static void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id)
{
	vfe->reg_update |= VFE_0_REG_UPDATE_line_n(line_id);
	wmb();
	writel_relaxed(vfe->reg_update, vfe->base + VFE_0_REG_UPDATE);
	wmb();
}

static inline void vfe_reg_update_clear(struct vfe_device *vfe,
					enum vfe_line_id line_id)
{
	vfe->reg_update &= ~VFE_0_REG_UPDATE_line_n(line_id);
}

static void vfe_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
				   enum vfe_line_id line_id, u8 enable)
{
	u32 irq_en0 = VFE_0_IRQ_MASK_0_IMAGE_MASTER_n_PING_PONG(wm) |
		      VFE_0_IRQ_MASK_0_line_n_REG_UPDATE(line_id);
	u32 irq_en1 = VFE_0_IRQ_MASK_1_IMAGE_MASTER_n_BUS_OVERFLOW(wm) |
		      VFE_0_IRQ_MASK_1_RDIn_SOF(line_id);

	if (enable) {
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	} else {
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_0, irq_en0);
		vfe_reg_clr(vfe, VFE_0_IRQ_MASK_1, irq_en1);
	}
}

static void vfe_enable_irq_common(struct vfe_device *vfe)
{
	u32 irq_en0 = VFE_0_IRQ_MASK_0_RESET_ACK;
	u32 irq_en1 = VFE_0_IRQ_MASK_1_VIOLATION |
		      VFE_0_IRQ_MASK_1_BUS_BDG_HALT_ACK;

	vfe_reg_set(vfe, VFE_0_IRQ_MASK_0, irq_en0);
	vfe_reg_set(vfe, VFE_0_IRQ_MASK_1, irq_en1);
}

static void vfe_set_qos(struct vfe_device *vfe)
{
	u32 val = VFE_0_BUS_BDG_QOS_CFG_0_CFG;
	u32 val7 = VFE_0_BUS_BDG_QOS_CFG_7_CFG;

	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_0);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_1);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_2);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_3);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_4);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_5);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_QOS_CFG_6);
	writel_relaxed(val7, vfe->base + VFE_0_BUS_BDG_QOS_CFG_7);
}

static void vfe_set_ds(struct vfe_device *vfe)
{
	u32 val = VFE_0_BUS_BDG_DS_CFG_0_CFG;
	u32 val16 = VFE_0_BUS_BDG_DS_CFG_16_CFG;

	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_0);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_1);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_2);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_3);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_4);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_5);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_6);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_7);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_8);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_9);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_10);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_11);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_12);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_13);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_14);
	writel_relaxed(val, vfe->base + VFE_0_BUS_BDG_DS_CFG_15);
	writel_relaxed(val16, vfe->base + VFE_0_BUS_BDG_DS_CFG_16);
}

static void vfe_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable)
{
	/* empty */
}

static void vfe_isr_read(struct vfe_device *vfe, u32 *value0, u32 *value1)
{
	*value0 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_0);
	*value1 = readl_relaxed(vfe->base + VFE_0_IRQ_STATUS_1);

	writel_relaxed(*value0, vfe->base + VFE_0_IRQ_CLEAR_0);
	writel_relaxed(*value1, vfe->base + VFE_0_IRQ_CLEAR_1);

	wmb();
	writel_relaxed(VFE_0_IRQ_CMD_GLOBAL_CLEAR, vfe->base + VFE_0_IRQ_CMD);
}

static void vfe_violation_read(struct vfe_device *vfe, struct device *dev)
{
	u32 violation = readl_relaxed(vfe->base + VFE_0_VIOLATION_STATUS);

	dev_err_ratelimited(dev, "VFE: violation = 0x%08x\n", violation);
}

const struct vfe_hw_ops vfe_ops_4_7 = {
	.hw_version_read = vfe_hw_version_read,
	.get_ub_size = vfe_get_ub_size,
	.global_reset = vfe_global_reset,
	.halt_request = vfe_halt_request,
	.halt_clear = vfe_halt_clear,
	.wm_enable = vfe_wm_enable,
	.wm_frame_based = vfe_wm_frame_based,
	.wm_set_framedrop_period = vfe_wm_set_framedrop_period,
	.wm_set_framedrop_pattern = vfe_wm_set_framedrop_pattern,
	.wm_set_ub_cfg = vfe_wm_set_ub_cfg,
	.bus_reload_wm = vfe_bus_reload_wm,
	.wm_set_ping_addr = vfe_wm_set_ping_addr,
	.wm_set_pong_addr = vfe_wm_set_pong_addr,
	.wm_get_ping_pong_status = vfe_wm_get_ping_pong_status,
	.bus_enable_wr_if = vfe_bus_enable_wr_if,
	.bus_connect_wm_to_rdi = vfe_bus_connect_wm_to_rdi,
	.wm_set_subsample = vfe_wm_set_subsample,
	.bus_disconnect_wm_from_rdi = vfe_bus_disconnect_wm_from_rdi,
	.set_rdi_cid = vfe_set_rdi_cid,
	.reg_update = vfe_reg_update,
	.reg_update_clear = vfe_reg_update_clear,
	.enable_irq_wm_line = vfe_enable_irq_wm_line,
	.enable_irq_common = vfe_enable_irq_common,
	.set_qos = vfe_set_qos,
	.set_ds = vfe_set_ds,
	.set_cgc_override = vfe_set_cgc_override,
	.isr_read = vfe_isr_read,
	.violation_read = vfe_violation_read,
};
