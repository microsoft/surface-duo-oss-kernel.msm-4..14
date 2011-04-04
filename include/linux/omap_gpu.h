/*
 * linux/include/linux/omap_gpu.h
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __OMAP_GPU_H__
#define __OMAP_GPU_H__

#include <linux/module.h>
#include <drm/drmP.h>

/* interface that plug-in drivers (for now just PVR) can implement */
struct omap_gpu_plugin {
	const char *name;

	/* drm functions */
	int (*open)(struct drm_device *dev, struct drm_file *file);
	int (*load)(struct drm_device *dev, unsigned long flags);
	int (*unload)(struct drm_device *dev);
	int (*release)(struct drm_device *dev, struct drm_file *file);

	/* file-ops */
	int (*mmap)(struct file *file, struct vm_area_struct *vma);

	struct drm_ioctl_desc *ioctls;
	int num_ioctls;
	int ioctl_start;

	struct list_head list;  /* note, this means struct can't be const.. */
};

int omap_gpu_register_plugin(struct omap_gpu_plugin *plugin);
int omap_gpu_unregister_plugin(struct omap_gpu_plugin *plugin);
struct fb_info * omap_gpu_get_fbdev(struct drm_device *dev);

enum omap_dss_update_mode omap_connector_get_update_mode(
		struct drm_connector *connector);
int omap_connector_set_update_mode(struct drm_connector *connector,
		enum omap_dss_update_mode mode);
int omap_connector_sync(struct drm_connector *connector);

int omap_encoder_wait_for_vsync(struct drm_encoder *encoder);

struct drm_connector * omap_fbdev_get_next_connector(
		struct fb_info *fbi, struct drm_connector *from);
void omap_fbdev_flush(struct fb_info *fbi, int x, int y, int w, int h);

struct drm_connector * omap_framebuffer_get_next_connector(
		struct drm_framebuffer *fb, struct drm_connector *from);
void omap_framebuffer_flush(struct drm_framebuffer *fb,
		int x, int y, int w, int h);


/* optional platform data to configure the default configuration of which
 * pipes/overlays/CRTCs are used.. if this is not provided, then instead the
 * first CONFIG_DRM_OMAP_NUM_CRTCS are used, and they are each connected to
 * one manager, with priority given to managers that are connected to
 * detected devices.  This should be a good default behavior for most cases,
 * but yet there still might be times when you wish to do something different.
 */
struct omap_gpu_platform_data {
	int ovl_cnt;
	const int *ovl_ids;
	int mgr_cnt;
	const int *mgr_ids;
	int dev_cnt;
	const char **dev_names;
};

#endif /* __OMAP_GPU_H__ */
