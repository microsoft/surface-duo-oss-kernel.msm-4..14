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

#define MSM_VFE_VFE0_UB_SIZE 1023
#define MSM_VFE_VFE0_UB_SIZE_RDI (MSM_VFE_VFE0_UB_SIZE / 3)
#define MSM_VFE_VFE1_UB_SIZE 1535
#define MSM_VFE_VFE1_UB_SIZE_RDI (MSM_VFE_VFE1_UB_SIZE / 3)

enum msm_vfe_output_state {
	MSM_VFE_OUTPUT_OFF,
	MSM_VFE_OUTPUT_RESERVED,
	MSM_VFE_OUTPUT_SINGLE,
	MSM_VFE_OUTPUT_CONTINUOUS,
	MSM_VFE_OUTPUT_IDLE,
};

enum vfe_line_id {
	VFE_LINE_NONE = -1,
	VFE_LINE_MIN = 0,
	VFE_LINE_RDI0 = 0,
	VFE_LINE_RDI1 = 1,
	VFE_LINE_RDI2 = 2,
	VFE_LINE_MAX = VFE_LINE_RDI2,
	VFE_LINE_PIX /* TODO: implement */
};

struct msm_vfe_output {
	u8 wm_idx;

	int active_buf;
	struct msm_video_buffer *buf[2];
	struct list_head pending_bufs;

	int drop_update_idx;

	enum msm_vfe_output_state state;
};

struct vfe_line {
	enum vfe_line_id id;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_VFE_PADS_NUM];
	struct v4l2_mbus_framefmt fmt[MSM_VFE_PADS_NUM];
	struct camss_video video_out;
	struct msm_vfe_output output;
};

struct vfe_device {
	u8 id;
	void __iomem *base;
	u32 irq;
	struct clk **clock;
	s32 *clock_rate;
	int nclocks;
	struct completion reset_completion;
	struct completion halt_completion;
	struct mutex power_lock;
	int power_count;
	struct mutex stream_lock;
	int stream_count;
	spinlock_t output_lock;
	enum vfe_line_id wm_output_map[MSM_VFE_IMAGE_MASTERS_NUM];
	struct msm_bus_scale_pdata *bus_scale_table;
	uint32_t bus_client;
	struct vfe_line line[VFE_LINE_MAX + 1];
	u32 reg_update;
};

struct resources;

int msm_vfe_subdev_init(struct vfe_device *vfe, struct resources *res);

int msm_vfe_register_entities(struct vfe_device *vfe,
			      struct v4l2_device *v4l2_dev);

void msm_vfe_unregister_entities(struct vfe_device *vfe);

void msm_vfe_get_vfe_id(struct media_entity *entity, u8 *id);
void msm_vfe_get_vfe_line_id(struct media_entity *entity, enum vfe_line_id *id);

#endif /* QC_MSM_CAMSS_VFE_H */
