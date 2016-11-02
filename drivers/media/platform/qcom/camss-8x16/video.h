/*
 * video.h
 *
 * Qualcomm MSM Camera Subsystem - V4L2 device node
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
#ifndef QC_MSM_CAMSS_VIDEO_H
#define QC_MSM_CAMSS_VIDEO_H

#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf2-v4l2.h>

#define camss_video_call(f, op, args...)			\
	(!(f) ? -ENODEV : (((f)->ops && (f)->ops->op) ? \
			    (f)->ops->op((f), ##args) : -ENOIOCTLCMD))

struct camss_buffer {
	struct vb2_v4l2_buffer vb;
	dma_addr_t addr;
	struct list_head queue;
};

struct camss_video;

struct camss_video_ops {
	int (*queue_buffer)(struct camss_video *vid, struct camss_buffer *buf);
	int (*flush_buffers)(struct camss_video *vid);
};

struct camss_video {
	struct camss *camss;
	void *alloc_ctx;
	struct vb2_queue vb2_q;
	struct video_device *vdev;
	struct media_pad pad;
	struct v4l2_format active_fmt;
	enum v4l2_buf_type type;
	struct media_pipeline pipe;
	struct camss_video_ops *ops;
};

struct camss_video_fh {
	struct v4l2_fh vfh;
	struct camss_video *video;
};

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name);

void msm_video_unregister(struct camss_video *video);

#endif /* QC_MSM_CAMSS_VIDEO_H */
