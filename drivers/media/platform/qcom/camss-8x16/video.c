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

/*
 * struct format_info - ISP media bus format information
 * @code: V4L2 media bus format code
 * @pixelformat: V4L2 pixel format FCC identifier
 * @bpp: Bits per pixel when stored in memory
 */
static const struct format_info {
	u32 code;
	u32 pixelformat;
	unsigned int bpp;
} formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_PIX_FMT_UYVY, 16 },
	{ MEDIA_BUS_FMT_VYUY8_2X8, V4L2_PIX_FMT_VYUY, 16 },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_PIX_FMT_YUYV, 16 },
	{ MEDIA_BUS_FMT_YVYU8_2X8, V4L2_PIX_FMT_YVYU, 16 },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, 8 },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, 8 },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, 8 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, 8 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10P, 10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10P, 10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10P, 10 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10P, 10 },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_PIX_FMT_SRGGB12P, 12 },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, V4L2_PIX_FMT_SGBRG12P, 12 },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_PIX_FMT_SGRBG12P, 12 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, V4L2_PIX_FMT_SRGGB12P, 12 }
};

/* -----------------------------------------------------------------------------
 * Helper functions
 */

/*
 * video_mbus_to_pix - Convert v4l2_mbus_framefmt to v4l2_pix_format
 * @mbus: v4l2_mbus_framefmt format (input)
 * @pix: v4l2_pix_format format (output)
 *
 * Fill the output pix structure with information from the input mbus format.
 *
 * Return 0 on success or a negative error code otherwise
 */
static unsigned int video_mbus_to_pix(const struct v4l2_mbus_framefmt *mbus,
				      struct v4l2_pix_format *pix)
{
	unsigned int i;

	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].code == mbus->code)
			break;
	}

	if (WARN_ON(i == ARRAY_SIZE(formats)))
		return -EINVAL;

	pix->pixelformat = formats[i].pixelformat;
	pix->bytesperline = pix->width * formats[i].bpp / 8;
	pix->bytesperline = ALIGN(pix->bytesperline, 8);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;

	return 0;
}

static struct v4l2_subdev *video_remote_subdev(struct camss_video *video,
					       u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&video->pad);

	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int video_get_subdev_format(struct camss_video *video,
				   struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	format->type = video->type;
	return video_mbus_to_pix(&fmt.format, &format->fmt.pix);
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

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

static int video_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);

	vb2_set_plane_payload(vb, 0, video->active_fmt.fmt.pix.sizeimage);
	if (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
		return -EINVAL;

	vbuf->field = V4L2_FIELD_NONE;

	buffer->addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	return 0;
}

static void video_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);

	camss_video_call(video, queue_buffer, buffer);
}


static int video_check_format(struct camss_video *video)
{
	struct v4l2_pix_format *pix = &video->active_fmt.fmt.pix;
	struct v4l2_format format;
	int ret;

	ret = video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	if (pix->pixelformat != format.fmt.pix.pixelformat ||
	    pix->height != format.fmt.pix.height ||
	    pix->width != format.fmt.pix.width ||
	    pix->bytesperline != format.fmt.pix.bytesperline ||
	    pix->sizeimage != format.fmt.pix.sizeimage ||
	    pix->field != format.fmt.pix.field)
		return -EINVAL;

	return 0;
}

static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;

	ret = media_entity_pipeline_start(&vdev->entity, &video->pipe);
	if (ret < 0)
		return ret;

	ret = video_check_format(video);
	if (ret < 0)
		goto error;

	entity = &vdev->entity;
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
			goto error;
	}

	return 0;

error:
	media_entity_pipeline_stop(&vdev->entity);

	return ret;
}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	struct v4l2_subdev *subdev_vfe;

	entity = &vdev->entity;
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

		if (strstr(subdev->name, "vfe")) {
			subdev_vfe = subdev;
		} else if (strstr(subdev->name, "ispif")) {
			v4l2_subdev_call(subdev, video, s_stream, 0);
			v4l2_subdev_call(subdev_vfe, video, s_stream, 0);
		} else {
			v4l2_subdev_call(subdev, video, s_stream, 0);
		}
	}

	media_entity_pipeline_stop(&vdev->entity);

	camss_video_call(video, flush_buffers);
}

static struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.buf_prepare     = video_buf_prepare,
	.buf_queue       = video_buf_queue,
	.start_streaming = video_start_streaming,
	.stop_streaming  = video_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strlcpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:qcom-camss", sizeof(cap->bus_info));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
							V4L2_CAP_DEVICE_CAPS;
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int video_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct camss_video *video = video_drvdata(file);
	struct v4l2_format format;
	int ret;

	if (f->type != video->type)
		return -EINVAL;

	if (f->index)
		return -EINVAL;

	ret = video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	f->pixelformat = format.fmt.pix.pixelformat;

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
	int ret;

	if (f->type != video->type)
		return -EINVAL;

	ret = video_get_subdev_format(video, f);
	if (ret < 0)
		return ret;

	video->active_fmt = *f;

	return 0;
}

static int video_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;

	return video_get_subdev_format(video, f);
}

static int video_enum_input(struct file *file, void *fh,
			    struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops msm_vid_ioctl_ops = {
	.vidioc_querycap          = video_querycap,
	.vidioc_enum_fmt_vid_cap  = video_enum_fmt,
	.vidioc_g_fmt_vid_cap     = video_g_fmt,
	.vidioc_s_fmt_vid_cap     = video_s_fmt,
	.vidioc_try_fmt_vid_cap   = video_try_fmt,
	.vidioc_reqbufs           = vb2_ioctl_reqbufs,
	.vidioc_querybuf          = vb2_ioctl_querybuf,
	.vidioc_qbuf              = vb2_ioctl_qbuf,
	.vidioc_dqbuf             = vb2_ioctl_dqbuf,
	.vidioc_streamon          = vb2_ioctl_streamon,
	.vidioc_streamoff         = vb2_ioctl_streamoff,
	.vidioc_enum_input        = video_enum_input,
	.vidioc_g_input           = video_g_input,
	.vidioc_s_input           = video_s_input,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

/*
 * video_init_format - Helper function to initialize format
 *
 * Initialize all pad formats with default values.
 */
static int video_init_format(struct file *file, void *fh)
{
	struct v4l2_format format;

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return video_s_fmt(file, fh, &format);
}

static int video_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct camss_video_fh *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	v4l2_fh_init(&handle->vfh, video->vdev);
	v4l2_fh_add(&handle->vfh);

	handle->video = video;
	file->private_data = &handle->vfh;

	ret = msm_camss_pipeline_pm_use(&vdev->entity, 1);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to power up pipeline\n");
		goto error_pm_use;
	}

	ret = video_init_format(file, &handle->vfh);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to init format\n");
		goto error_init_format;
	}

	return 0;

error_init_format:
	msm_camss_pipeline_pm_use(&vdev->entity, 0);

error_pm_use:
	v4l2_fh_del(&handle->vfh);
	kfree(handle);

	return ret;
}

static int video_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct camss_video_fh *handle = container_of(vfh, struct camss_video_fh,
						     vfh);

	vb2_ioctl_streamoff(file, vfh, video->type);

	msm_camss_pipeline_pm_use(&vdev->entity, 0);

	v4l2_fh_del(vfh);
	kfree(handle);
	file->private_data = NULL;

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

/* -----------------------------------------------------------------------------
 * CAMSS video core
 */

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name)
{
	struct media_pad *pad = &video->pad;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	vdev = video_device_alloc();
	if (vdev == NULL) {
		dev_err(v4l2_dev->dev, "Failed to allocate video device\n");
		return -ENOMEM;
	}

	video->vdev = vdev;

	video->alloc_ctx = vb2_dma_contig_init_ctx(video->camss->dev);
	if (IS_ERR(video->alloc_ctx)) {
		dev_err(v4l2_dev->dev, "Failed to init vb2 dma ctx\n");
		return PTR_ERR(video->alloc_ctx);
	}

	q = &video->vb2_q;
	q->drv_priv = video;
	q->mem_ops = &vb2_dma_contig_memops;
	q->ops = &msm_video_vb2_q_ops;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct camss_buffer);
	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init vb2 queue\n");
		goto error_vb2_init;
	}

	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vdev->entity, 1, pad, 0);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init video entity\n");
		goto error_media_init;
	}

	vdev->fops = &msm_vid_fops;
	vdev->ioctl_ops = &msm_vid_ioctl_ops;
	vdev->release = video_device_release;
	vdev->v4l2_dev = v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = &video->vb2_q;
	strlcpy(vdev->name, name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to register video device\n");
		goto error_video_register;
	}

	video_set_drvdata(vdev, video);

	return 0;

error_video_register:
	media_entity_cleanup(&vdev->entity);
error_media_init:
	vb2_queue_release(&video->vb2_q);
error_vb2_init:
	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);

	return ret;
}

void msm_video_unregister(struct camss_video *video)
{
	video_unregister_device(video->vdev);
	media_entity_cleanup(&video->vdev->entity);
	vb2_queue_release(&video->vb2_q);
	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);
}
