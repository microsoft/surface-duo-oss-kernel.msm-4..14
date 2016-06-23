/*
 * video.c
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
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-core.h>
#include <media/videobuf2-dma-contig.h>

#include "video.h"
#include "camss.h"

static int video_queue_setup(struct vb2_queue *q, const void *parg,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], void *alloc_ctxs[])
{
	struct camss_video *video = vb2_get_drv_priv(q);
	const struct v4l2_format *fmt = parg;

	*num_planes = 1;

	if (NULL == fmt)
		sizes[0] = video->active_fmt.fmt.pix.sizeimage;
	else
		sizes[0] = fmt->fmt.pix.sizeimage;

	alloc_ctxs[0] = video->alloc_ctx;

	return 0;
}

static int video_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

static int video_buf_prepare(struct vb2_buffer *vb)
{
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct msm_video_buffer *buffer = container_of(vb,
						struct msm_video_buffer, vb);

	buffer->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	vb2_set_plane_payload(vb, 0, video->active_fmt.fmt.pix.sizeimage);

	return 0;
}

static void video_buf_finish(struct vb2_buffer *vb)
{
}

static void video_buf_queue(struct vb2_buffer *vb)
{
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct msm_video_buffer *buffer = container_of(vb,
						struct msm_video_buffer, vb);

	msm_video_call(video, queue_dmabuf, buffer);
}

static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{

	return 0;
}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);

	msm_video_call(video, flush_dmabufs);
}

static struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.buf_init        = video_buf_init,
	.buf_prepare     = video_buf_prepare,
	.buf_finish      = video_buf_finish,
	.buf_queue       = video_buf_queue,
	.start_streaming = video_start_streaming,
	.stop_streaming  = video_stop_streaming,
};

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	struct camss_video *video = video_drvdata(file);

	strlcpy(cap->driver, video->video.name, sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));
	cap->version = CAMSS_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
							V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int video_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;

	if (f->index)
		return -EINVAL;

	f->pixelformat = video->active_fmt.fmt.pix.pixelformat;

	return 0;
}

static int video_enum_framesizes(struct file *file, void *fh,
				 struct v4l2_frmsizeenum *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->pixel_format != video->active_fmt.fmt.pix.pixelformat)
		return -EINVAL;

	if (f->index)
		return -EINVAL;

	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	f->discrete.width = video->active_fmt.fmt.pix.width;
	f->discrete.height = video->active_fmt.fmt.pix.height;

	return 0;
}

static int video_enum_frameintervals(struct file *file, void *fh,
				     struct v4l2_frmivalenum *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->pixel_format != video->active_fmt.fmt.pix.pixelformat ||
	    f->width != video->active_fmt.fmt.pix.width ||
	    f->height != video->active_fmt.fmt.pix.height)
		return -EINVAL;

	if (f->index)
		return -EINVAL;

	f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	f->discrete.numerator = 1;
	f->discrete.denominator = 30;

	return 0;
}

static int video_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;

	*f = video->active_fmt;

	return 0;
}

static int video_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;

	*f = video->active_fmt;

	return 0;
}

static int video_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;

	*f = video->active_fmt;

	return 0;
}

static int video_reqbufs(struct file *file, void *fh,
			 struct v4l2_requestbuffers *b)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	ret = vb2_reqbufs(&video->vb2_q, b);

	return ret;
}

static int video_querybuf(struct file *file, void *fh,
			  struct v4l2_buffer *b)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	ret = vb2_querybuf(&video->vb2_q, b);

	return ret;
}

static int video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	ret = vb2_qbuf(&video->vb2_q, b);

	return ret;
}

static int video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct camss_video *video = video_drvdata(file);
	int ret;

	ret = vb2_dqbuf(&video->vb2_q, b, file->f_flags & O_NONBLOCK);

	return ret;
}

static int video_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct video_device *video_dev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;

	if (type != video->type)
		return -EINVAL;

	ret = media_entity_pipeline_start(&video->video.entity, &video->pipe);
	if (ret < 0)
		return ret;

	ret = vb2_streamon(&video->vb2_q, type);
	if (ret < 0)
		goto pipeline_stop;

	entity = &video_dev->entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			goto streamoff;
	}

	return 0;

pipeline_stop:
	media_entity_pipeline_stop(&video->video.entity);
streamoff:
	vb2_streamoff(&video->vb2_q, type);

	return ret;
}

static int video_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct video_device *video_dev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;

	if (type != video->type)
		return -EINVAL;

	entity = &video_dev->entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		v4l2_subdev_call(subdev, video, s_stream, 0);
	}

	ret = vb2_streamoff(&video->vb2_q, type);
	if (ret)
		return ret;

	media_entity_pipeline_stop(&video->video.entity);

	return 0;
}

static const struct v4l2_ioctl_ops msm_vid_ioctl_ops = {
	.vidioc_querycap          = video_querycap,
	.vidioc_enum_fmt_vid_cap  = video_enum_fmt,
	.vidioc_enum_framesizes   = video_enum_framesizes,
	.vidioc_enum_frameintervals = video_enum_frameintervals,
	.vidioc_g_fmt_vid_cap     = video_g_fmt,
	.vidioc_s_fmt_vid_cap     = video_s_fmt,
	.vidioc_try_fmt_vid_cap   = video_try_fmt,
	.vidioc_reqbufs           = video_reqbufs,
	.vidioc_querybuf          = video_querybuf,
	.vidioc_qbuf              = video_qbuf,
	.vidioc_dqbuf             = video_dqbuf,
	.vidioc_streamon          = video_streamon,
	.vidioc_streamoff         = video_streamoff,
};

static int video_open(struct file *file)
{
	struct video_device *video_dev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct vb2_queue *q;
	int ret;

	video->alloc_ctx = vb2_dma_contig_init_ctx(video->camss->iommu_dev);
	if (IS_ERR(video->alloc_ctx)) {
		dev_err(&video->video.dev, "Failed to init vb2 dma ctx\n");
		return PTR_ERR(video->alloc_ctx);
	}

	v4l2_fh_init(&video->fh, video_dev);
	v4l2_fh_add(&video->fh);
	file->private_data = &video->fh;

	ret = msm_camss_pipeline_pm_use(&video_dev->entity, 1);
	if (ret < 0) {
		dev_err(&video_dev->dev, "pipeline power-up failed\n");
		goto error;
	}

	q = &video->vb2_q;
	q->drv_priv = video;
	q->mem_ops = &vb2_dma_contig_memops;
	q->ops = &msm_video_vb2_q_ops;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // TODO: MPLANE
	q->io_modes = VB2_MMAP;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct msm_video_buffer);
	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(&video_dev->dev, "vb2 queue init failed\n");
		goto error;
	}

	return 0;

error:
	file->private_data = NULL;
	v4l2_fh_del(&video->fh);
	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);

	return ret;
}

static int video_release(struct file *file)
{
	struct video_device *video_dev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);

	vb2_queue_release(&video->vb2_q);

	msm_camss_pipeline_pm_use(&video_dev->entity, 0);

	file->private_data = NULL;
	v4l2_fh_del(&video->fh);

	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);

	return 0;
}

static unsigned int video_poll(struct file *file,
				   struct poll_table_struct *wait)
{
	struct camss_video *video = video_drvdata(file);

	return vb2_poll(&video->vb2_q, file, wait);
}

static int video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct camss_video *video = video_drvdata(file);

	return vb2_mmap(&video->vb2_q, vma);
}

static const struct v4l2_file_operations msm_vid_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = video_open,
	.release        = video_release,
	.poll           = video_poll,
	.mmap		= video_mmap,
};

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name)
{
	struct media_pad *pad = &video->pad;
	struct video_device *video_dev = &video->video;
	int ret;

	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&video_dev->entity, 1, pad, 0);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to init video entity\n");
		return ret;
	}

	video_dev->fops = &msm_vid_fops;
	video_dev->ioctl_ops = &msm_vid_ioctl_ops;
	video_dev->release = video_device_release; // TODO: implement
	video_dev->v4l2_dev = v4l2_dev;
	video_dev->vfl_dir = VFL_DIR_RX;
	strlcpy(video_dev->name, name, sizeof(video_dev->name));

	ret = video_register_device(video_dev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register video device\n");
		return ret;
	}

	video_set_drvdata(video_dev, video);

	return 0;
}

void msm_video_unregister(struct camss_video *video)
{
	video_unregister_device(&video->video);
}
