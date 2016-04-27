/*
 * vfe.h
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
#ifndef QC_MSM_CAMSS_VFE_H
#define QC_MSM_CAMSS_VFE_H

#include <linux/clk.h>
#include <linux/spinlock_types.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "video.h"

#define MSM_VFE_MAX_CID_NUM 4

#define MSM_VFE_PAD_SINK 0
#define MSM_VFE_PAD_SRC 1
#define MSM_VFE_PADS_NUM 2

/* Output limitations */
#define MSM_VFE_MAX_WM_PER_OUTPUT 4
#define MSM_VFE_MAX_OUTPUTS 1

/* Hw definitions */
#define MSM_VFE_NUM_RDI 3
#define MSM_VFE_IMAGE_MASTERS_NUM 7
#define MSM_VFE_IMAGE_COMPOSITE_NUM 4

#define MSM_VFE_UB_MAX_SIZE_VFE0 827
#define MSM_VFE_UB_MAX_SIZE_VFE1 (1535)

enum msm_vfe_output_state {
	MSM_VFE_OUTPUT_OFF,
	MSM_VFE_OUTPUT_RESERVED,
	MSM_VFE_OUTPUT_SINGLE,
	MSM_VFE_OUTPUT_CONTINUOUS,
	MSM_VFE_OUTPUT_IDLE,
};

struct msm_vfe_wm {
	u8 rdi_idx;
	u8 wm_idx;
	u32 bytesperline;
};

struct msm_vfe_output {
	u16 active_wm;
	struct msm_vfe_wm wm[MSM_VFE_MAX_WM_PER_OUTPUT];

	int active_buf;
	struct msm_video_buffer *buf[2];
	struct list_head pending_bufs;

	int drop_update_idx;

	enum msm_vfe_output_state state;
};

struct vfe_init {
	int num_cids;
	unsigned int cid[MSM_VFE_MAX_CID_NUM];
};

struct clock_info {
	const char *name;
	struct clk *clk;
};

struct vfe_device {
	int hw_id;
	struct vfe_init init;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_VFE_PADS_NUM];
	struct camss *camss;
	struct camss_video video_out;
	void __iomem *base;
	void __iomem *base_vbif;
	u32 irq;
	struct clock_info *clocks;
	int nclocks;
	struct completion reset_completion;
	struct completion halt_completion;
	struct mutex mutex;
	int ref_count;
	spinlock_t output_lock;
	int rdi_output_map[MSM_VFE_NUM_RDI];
	int wm_output_map[MSM_VFE_IMAGE_MASTERS_NUM];
	int composite_output_map[MSM_VFE_IMAGE_COMPOSITE_NUM];
	int stream_cnt;
	int active_outputs;
	struct msm_vfe_output output[MSM_VFE_MAX_OUTPUTS];
	struct msm_bus_scale_pdata *bus_scale_table;
	uint32_t bus_client;
};

int msm_vfe_subdev_init(struct vfe_device *vfe, struct camss *camss,
			struct vfe_init *init);

int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev);

void msm_vfe_unregister_entities(struct vfe_device *vfe);

#endif /* QC_MSM_CAMSS_VFE_H */
