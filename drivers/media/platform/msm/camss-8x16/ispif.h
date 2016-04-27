/*
 * ispif.h
 *
 * Qualcomm MSM Camera Subsystem - ISPIF Module
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#ifndef QC_MSM_CAMSS_ISPIF_H
#define QC_MSM_CAMSS_ISPIF_H

#include <linux/clk.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define MSM_ISPIF_PAD_SINK 0
#define MSM_ISPIF_PAD_SRC 1
#define MSM_ISPIF_PADS_NUM 2

struct camss;

struct ispif_device {
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_ISPIF_PADS_NUM];
	struct camss *camss;
	void __iomem *base;
	void __iomem *base_clk_mux;
	u32 irq;
	struct clk **clock;
	u8 *clock_for_reset;
	int nclocks;
	struct completion reset_complete;
	u8 csid_id;
};

struct resources_ispif;

int msm_ispif_subdev_init(struct ispif_device *ispif, struct camss *camss,
			  struct resources_ispif *res);

int msm_ispif_register_entities(struct ispif_device *ispif,
				struct v4l2_device *v4l2_dev);

void msm_ispif_unregister_entities(struct ispif_device *ispif);

#endif /* QC_MSM_CAMSS_ISPIF_H */
