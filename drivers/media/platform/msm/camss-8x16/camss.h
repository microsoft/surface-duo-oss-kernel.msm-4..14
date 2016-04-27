/*
 * camss.h
 *
 * Qualcomm MSM Camera Subsystem - Core
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#ifndef QC_MSM_CAMSS_H
#define QC_MSM_CAMSS_H

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/media-device.h>
#include <linux/device.h>

#include "csid.h"
#include "csiphy.h"
#include "ispif.h"
#include "vfe.h"

#define CAMSS_VERSION KERNEL_VERSION(0, 1, 0)

#define CAMSS_RES_MAX 15

struct resources {
	char *regulator[CAMSS_RES_MAX];
	char *clock[CAMSS_RES_MAX];
	s32 clock_rate[CAMSS_RES_MAX];
	char *reg[CAMSS_RES_MAX];
	char *interrupt[CAMSS_RES_MAX];
};

struct resources_ispif {
	char *clock[CAMSS_RES_MAX];
	u8 clock_for_reset[CAMSS_RES_MAX];
	char *reg[CAMSS_RES_MAX];
	char *interrupt;
};

struct camss {
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;
	struct device *dev;
	int csiphy_num;
	struct csiphy_device *csiphy;
	int csid_num;
	struct csid_device *csid;
	struct ispif_device ispif;
	struct vfe_device vfe;
	struct vfe_init vfe_init;
	struct device *iommu_dev;
};

enum camss_csiphy {
	CAMSS_CSIPHY0 = 0,
	CAMSS_CSIPHY1
};

struct camss_csiphy_lane {
	u8 pos;
	u8 pol;
};

struct camss_csiphy_lanes_cfg {
	int num_data;
	struct camss_csiphy_lane *data;
	struct camss_csiphy_lane clk;
};

struct camss_csi2_cfg {
	int settle_cnt;
	struct camss_csiphy_lanes_cfg lanecfg;
};

struct camss_camera_interface {
	enum camss_csiphy id;
	struct camss_csi2_cfg csi2;
};

struct camss_async_subdev {
	struct camss_camera_interface interface;
	struct v4l2_async_subdev asd;
};

int msm_camss_pipeline_pm_use(struct media_entity *entity, int use);

#endif /* QC_MSM_CAMSS_H */
