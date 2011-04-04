/*
 * linux/drivers/gpu/drm/omap/omap_fb.c
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

#include <plat/vram.h>

#include <linux/omap_gpu.h>
#include "omap_gpu_priv.h"

#include "drm_crtc.h"
#include "drm_crtc_helper.h"


static char *def_vram;
module_param_named(vram, def_vram, charp, 0);

/*
 * framebuffer funcs
 */

#define to_omap_framebuffer(x) container_of(x, struct omap_framebuffer, base)

struct omap_framebuffer {
	struct drm_framebuffer base;

	/* framebuffer size/phys-addr/virt-addr */
	int size;
	unsigned long paddr;
	void __iomem *vaddr;
};


/* copied from omapfb-main.c to preserve vram param syntax */
static int parse_vram_param(const char *param, int max_entries,
		unsigned long *sizes, unsigned long *paddrs)
{
	int fbnum;
	unsigned long size;
	unsigned long paddr = 0;
	char *p, *start;

	DBG("vram: %s", param);

	start = (char *)param;

	while (1) {
		p = start;

		fbnum = simple_strtoul(p, &p, 10);

		if ((p == param) || (*p != ':') || (fbnum >= max_entries))
			return -EINVAL;

		size = memparse(p + 1, &p);

		if (!size)
			return -EINVAL;

		paddr = 0;

		if (*p == '@') {
			paddr = simple_strtoul(p + 1, &p, 16);

			if (!paddr)
				return -EINVAL;
		}

		paddrs[fbnum] = paddr;
		sizes[fbnum] = size;

		if (*p == 0)
			break;

		if (*p != ',')
			return -EINVAL;

		++p;

		start = p;
	}

	return 0;
}

#define MAX_FBS  10

static int allocate_vram(struct drm_framebuffer *fb, int idx, int size)
{
	struct omap_framebuffer *omap_fb = to_omap_framebuffer(fb);
	struct drm_device *dev = fb->dev;
	unsigned long sizes[MAX_FBS] = {0};
	unsigned long paddrs[MAX_FBS] = {0};
	unsigned long paddr = 0;
	int ret = ret;

	if (idx >= MAX_FBS) {
		dev_err(dev->dev, "invalid fb number: %d\n", idx);
		goto fail;
	}

	if (def_vram) {
		if (parse_vram_param(def_vram, MAX_FBS, sizes, paddrs)) {
			dev_err(dev->dev, "failed to parse vram parameter\n");
			memset(&sizes, 0, sizeof(sizes));
			memset(&paddrs, 0, sizeof(paddrs));
		} else {
			size = sizes[idx];
			paddr = paddrs[idx];
		}
	}

	size = PAGE_ALIGN(size);

	if (paddr) {
		DBG("reserving %d bytes at %lx for fb %d", size, paddr, idx);
		ret = omap_vram_reserve(paddr, size);
	} else {
		DBG("allocating %d bytes for fb %d", size, idx);
		ret = omap_vram_alloc(OMAP_VRAM_MEMTYPE_SDRAM, size, &paddr);
	}

	if (ret) {
		dev_err(dev->dev, "failed to allocate vram\n");
		goto fail;
	}

	omap_fb->size = size;
	omap_fb->paddr = paddr;
	omap_fb->vaddr = ioremap_wc(paddr, size);

	if (!omap_fb->vaddr) {
		dev_err(dev->dev, "failed to ioremap framebuffer\n");
		ret = -ENOMEM;
		goto fail;
	}

	//memset(vaddr, 0, size);

	return 0;

fail:
	if (omap_fb->paddr) {
		omap_vram_free(paddr, size);
	}

	omap_fb->size = 0;
	omap_fb->vaddr = NULL;
	omap_fb->paddr = 0;

	return ret;
}

static int omap_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	struct omap_framebuffer *omap_fb = to_omap_framebuffer(fb);
	DBG("framebuffer: get handle: %p", omap_fb);

	// TODO, I suppose this really should be some sort of GEM handle
	// to the framebuffer object, in case it needs to be mapped or
	// something.  Right now this will go-exist badly with PVR, who
	// implements the mmap() fxn.. need to think about how to handle
	// this..

	*handle = 42;

	return 0;
}

static void omap_framebuffer_destroy(struct drm_framebuffer *fb)
{
	/* omap_vram_free() doesn't really do what you'd think.. (or at
	 * least not if you think it'd return vram to the pool), so
	 * disabling this for now until there is a better way to alloc
	 * and free coherant..
	 */
	DBG("destroy: FB ID: %d (%p)", fb->base.id, fb);
}

static int omap_framebuffer_dirty(struct drm_framebuffer *fb,
		struct drm_file *file_priv, unsigned flags, unsigned color,
		struct drm_clip_rect *clips, unsigned num_clips)
{
	int i;

	for (i = 0; i < num_clips; i++) {
		omap_framebuffer_flush(fb, clips[i].x1, clips[i].y1,
					clips[i].x2 - clips[i].x1,
					clips[i].y2 - clips[i].y1);
	}

	return 0;
}

static const struct drm_framebuffer_funcs omap_framebuffer_funcs = {
	.create_handle = omap_framebuffer_create_handle,
	.destroy = omap_framebuffer_destroy,
	.dirty = omap_framebuffer_dirty,
};

int omap_framebuffer_get_buffer(struct drm_framebuffer *fb, int x, int y,
		void **vaddr, unsigned long *paddr, int *screen_width)
{
	struct omap_framebuffer *omap_fb = to_omap_framebuffer(fb);
	int bpp = 4; //XXX fb->depth / 8;
	unsigned long offset;

	offset = (x * bpp) + (y * fb->pitch);

	*vaddr = omap_fb->vaddr + offset;
	*paddr = omap_fb->paddr + offset;
	*screen_width = fb->pitch / bpp;

	return omap_fb->size;
}

/* iterate thru all the connectors, returning ones that are attached
 * to the same fb..
 */
struct drm_connector * omap_framebuffer_get_next_connector(
		struct drm_framebuffer *fb, struct drm_connector *from)
{
	struct drm_device *dev = fb->dev;
	struct list_head *connector_list = &dev->mode_config.connector_list;
	struct drm_connector *connector = from;

	if (!from) {
		return list_first_entry(connector_list, typeof(*from), head);
	}

	list_for_each_entry_from(connector, connector_list, head) {
		if (connector != from) {
			struct drm_encoder *encoder = connector->encoder;
			struct drm_crtc *crtc = encoder ? encoder->crtc : NULL;
			if (crtc && crtc->fb == fb) {
				return connector;
			}
		}
	}

	return NULL;
}
EXPORT_SYMBOL(omap_framebuffer_get_next_connector);

/* flush an area of the framebuffer (in case of manual update display that
 * is not automatically flushed)
 */
void omap_framebuffer_flush(struct drm_framebuffer *fb,
		int x, int y, int w, int h)
{
	struct drm_connector *connector = NULL;

	VERB("flush: %d,%d %dx%d, fb=%p", x, y, w, h, fb);

	while ((connector =
			omap_framebuffer_get_next_connector(fb, connector))) {
		/* only consider connectors that are part of a chain */
		if (connector->encoder && connector->encoder->crtc) {
			/* TODO: maybe this should propagate thru the crtc who
			 * could do the coordinate translation..
			 */
			struct drm_crtc *crtc = connector->encoder->crtc;
			int cx = max(0, x - crtc->x);
			int cy = max(0, y - crtc->y);
			int cw = w + (x - crtc->x) - cx;
			int ch = h + (y - crtc->y) - cy;

			omap_connector_flush(connector, cx, cy, cw, ch);
		}
	}
}
EXPORT_SYMBOL(omap_framebuffer_flush);

struct drm_framebuffer * omap_framebuffer_create(struct drm_device *dev,
        struct drm_file *file, struct drm_mode_fb_cmd *mode_cmd)
{
	return omap_framebuffer_init(dev, mode_cmd);
}

struct drm_framebuffer * omap_framebuffer_init(struct drm_device *dev,
		struct drm_mode_fb_cmd *mode_cmd)
{
	struct omap_gpu_private *priv = dev->dev_private;
	struct omap_framebuffer *omap_fb = NULL;
	struct drm_framebuffer *fb;
	int ret;

	/* in case someone tries to feed us a completely bogus stride: */
	mode_cmd->pitch = max(mode_cmd->pitch,
			mode_cmd->width * mode_cmd->bpp / 8);

	/* pvr needs to have a stride that is a multiple of 8 pixels: */
	mode_cmd->pitch = ALIGN(mode_cmd->pitch, 8 * (mode_cmd->bpp / 8));

	/* for now, we can only support a single fb.. so we need to resize
	 * the old one instead of creating a new one of different size.  If
	 * we can eventually get rid of the vram pool and had a better way
	 * to allocate contiguous memory after boot time, this restriction
	 * should be lifted.
	 */
	if (priv->fb) {
		fb = priv->fb;
		DBG("recycle: FB ID: %d (%p)", fb->base.id, fb);
		goto recycle;  /* ugg, we need CMA! */
	}

	DBG("create framebuffer: dev=%p, mode_cmd=%p (%dx%d)", dev,
			mode_cmd, mode_cmd->width, mode_cmd->height);

	omap_fb = kzalloc(sizeof(*omap_fb), GFP_KERNEL);
	if (!omap_fb) {
		dev_err(dev->dev, "could not allocate fb\n");
		goto fail;
	}

	fb = &omap_fb->base;
	ret = drm_framebuffer_init(dev, fb, &omap_framebuffer_funcs);
	if (ret) {
		dev_err(dev->dev, "framebuffer init failed: %d\n", ret);
		goto fail;
	}

	DBG("create: FB ID: %d (%p)", fb->base.id, fb);

	ret = allocate_vram(fb, dev->primary->index,
			mode_cmd->pitch * mode_cmd->height);
	if (ret) {
		dev_err(dev->dev, "failed to allocate framebuffer\n");
		goto fail;
	}

recycle:
	drm_helper_mode_fill_fb_struct(fb, mode_cmd);

	priv->fb = fb;

	if (priv->fbdev) {
		/* if fbdev is already created, we need to update it to
		 * be attached to the new fb
		 */
		omap_fbdev_update(priv->fbdev, fb);
	}

	return fb;

fail:
	if (omap_fb) {
		kfree(omap_fb);
	}
	return NULL;
}

