/*
 * linux/drivers/gpu/drm/omap/omap_connector.c
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

#include <linux/omap_gpu.h>
#include "omap_gpu_priv.h"

#include "drm_crtc.h"
#include "drm_crtc_helper.h"

/*
 * connector funcs
 */

#define to_omap_connector(x) container_of(x, struct omap_connector, base)

struct omap_connector {
	struct drm_connector base;
	struct omap_dss_device *dssdev;
	struct drm_display_mode *native_mode;
};

static inline void copy_timings_omap_to_drm(struct drm_display_mode *mode,
		struct omap_video_timings *timings)
{
	mode->clock = timings->pixel_clock;

	mode->hdisplay = timings->x_res;
	mode->hsync_start = mode->hdisplay + timings->hfp;
	mode->hsync_end = mode->hsync_start + timings->hsw;
	mode->htotal = mode->hsync_end + timings->hbp;

	mode->vdisplay = timings->y_res;
	mode->vsync_start = mode->vdisplay + timings->vfp;
	mode->vsync_end = mode->vsync_start + timings->vsw;
	mode->vtotal = mode->vsync_end + timings->vbp;

	/* note: whether or not it is interlaced, +/- h/vsync, etc,
	 * which should be set in the mode flags, is not exposed in
	 * the omap_video_timings struct.. but hdmi driver tracks
	 * those separately so all we have to have to set the mode
	 * is the way to recover these timings values, and the
	 * omap_dss_driver would do the rest.
	 */
}

static inline void copy_timings_drm_to_omap(struct omap_video_timings *timings,
		struct drm_display_mode *mode)
{
	timings->pixel_clock = mode->clock;

	timings->x_res = mode->hdisplay;
	timings->hfp = mode->hsync_start - mode->hdisplay;
	timings->hsw = mode->hsync_end - mode->hsync_start;
	timings->hbp = mode->htotal - mode->hsync_end;

	timings->y_res = mode->vdisplay;
	timings->vfp = mode->vsync_start - mode->vdisplay;
	timings->vsw = mode->vsync_end - mode->vsync_start;
	timings->vbp = mode->vtotal - mode->vsync_end;
}

void omap_connector_dpms(struct drm_connector *connector, int mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;

	/* TODO: add API in DSS to suspend/resume individual displays.. */

	DBG("%s: %d", dssdev->name, mode);
}

enum drm_connector_status omap_connector_detect(
		struct drm_connector *connector, bool force)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	enum drm_connector_status ret;

	if (dssdrv->is_detected(dssdev)) {
		ret = connector_status_connected;
	} else {
		ret = connector_status_disconnected;
	}

	DBG("%s: %d (force=%d)", omap_connector->dssdev->name, ret, force);

	return ret;
}

static void omap_connector_destroy(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);

	DBG("%s", omap_connector->dssdev->name);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(omap_connector);
}

static struct drm_display_mode * omap_connector_native_mode(
			struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode, *largest = NULL;
	int high_w = 0, high_h = 0, high_v = 0;

	list_for_each_entry(mode, &omap_connector->base.probed_modes, head) {
		mode->vrefresh = drm_mode_vrefresh(mode);
		if (mode->flags & DRM_MODE_FLAG_INTERLACE)
			continue;

		/* Use preferred mode if there is one */
		if (mode->type & DRM_MODE_TYPE_PREFERRED) {
			DBG("native mode from preferred: %dx%d@%d",
				mode->hdisplay, mode->vdisplay, mode->vrefresh);
			return drm_mode_duplicate(dev, mode);
		}

		/* Otherwise, take the resolution with the largest width, then
		 * height, then vertical refresh
		 */
		if (mode->hdisplay < high_w)
			continue;

		if (mode->hdisplay == high_w && mode->vdisplay < high_h)
			continue;

		if (mode->hdisplay == high_w && mode->vdisplay == high_h &&
				mode->vrefresh < high_v)
			continue;

		high_w = mode->hdisplay;
		high_h = mode->vdisplay;
		high_v = mode->vrefresh;
		largest = mode;
	}

	DBG("native mode from largest: %dx%d@%d", high_w, high_h, high_v);
	return largest ? drm_mode_duplicate(dev, largest) : NULL;
}

struct moderec {
	int hdisplay;
	int vdisplay;
};

static struct moderec scaler_modes[] = {
	{ 1920, 1200 },
	{ 1920, 1080 },
	{ 1680, 1050 },
	{ 1600, 1200 },
	{ 1400, 1050 },
	{ 1400, 900 },
	{ 1280, 1024 },
	{ 1280, 960 },
	{ 1280, 720 },
	{ 1152, 768 },
	{ 1024, 768 },
	{ 800, 600 },
	{ 720, 480 },
	{ 640, 480 },
	{}
};

static int omap_connector_scaler_modes_add(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct drm_display_mode *native = omap_connector->native_mode, *m;
	struct drm_device *dev = connector->dev;
	struct moderec *mode = &scaler_modes[0];
	int modes = 0;

	if (!native)
		return 0;

	while (mode->hdisplay) {
		if (mode->hdisplay <= native->hdisplay &&
				mode->vdisplay <= native->vdisplay) {
			m = drm_cvt_mode(dev, mode->hdisplay, mode->vdisplay,
					60, true, false, false);
			if (!m)
				continue;

			m->type |= DRM_MODE_TYPE_DRIVER;

			DBG("adding scaler mode: %dx%d@%d", mode->hdisplay,
				 mode->vdisplay, drm_mode_vrefresh(m));
			drm_mode_probed_add(connector, m);
			modes++;
		}
		mode++;
	}

	return modes;
}

#define MAX_EDID  256

static int omap_connector_get_modes(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	struct drm_device *dev = connector->dev;
	int n = 0;

	DBG("%s", omap_connector->dssdev->name);

	if (omap_connector->native_mode) {
		drm_mode_destroy(dev, omap_connector->native_mode);
		omap_connector->native_mode = NULL;
	}

	/* if display exposes EDID, then we parse that in the normal way to
	 * build table of supported modes.. otherwise (ie. fixed resolution
	 * LCD panels) we just return a single mode corresponding to the
	 * currently configured timings:
	 */
	if (dssdrv->get_edid) {
		void *edid = kzalloc(MAX_EDID, GFP_KERNEL);

		if ((dssdrv->get_edid(dssdev, edid, MAX_EDID) == 0) &&
				drm_edid_is_valid(edid)) {
			drm_mode_connector_update_edid_property(connector, edid);
			n = drm_add_edid_modes(connector, edid);
			omap_connector->native_mode =
					omap_connector_native_mode(connector);
			n += omap_connector_scaler_modes_add(connector);
			kfree(connector->display_info.raw_edid);
			connector->display_info.raw_edid = edid;
		} else {
			drm_mode_connector_update_edid_property(connector, NULL);
			connector->display_info.raw_edid = NULL;
			kfree(edid);
		}
	} else {
		struct drm_display_mode *mode = drm_mode_create(dev);
		struct omap_video_timings timings;

		dssdrv->get_timings(dssdev, &timings);

		copy_timings_omap_to_drm(mode, &timings);

		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);

		n = 1;
	}

	return n;
}

static int omap_connector_mode_valid(struct drm_connector *connector,
				 struct drm_display_mode *mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	struct omap_video_timings timings = {0};
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *new_mode;
	int ret = MODE_BAD;

	copy_timings_drm_to_omap(&timings, mode);
	mode->vrefresh = drm_mode_vrefresh(mode);

	if (!dssdrv->check_timings(dssdev, &timings)) {
		/* check if vrefresh is still valid */
		new_mode = drm_mode_duplicate(dev, mode);
		new_mode->clock = timings.pixel_clock;
		new_mode->vrefresh = 0;
		if (mode->vrefresh == drm_mode_vrefresh(new_mode))
			ret = MODE_OK;
		drm_mode_destroy(dev, new_mode);
	}

	DBG("connector: mode %s: "
			"%d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			(ret == MODE_OK) ? "valid" : "invalid",
			mode->base.id, mode->name, mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal, mode->type, mode->flags);

	return ret;
}

struct drm_encoder * omap_connector_attached_encoder(
		struct drm_connector *connector)
{
	int i;
	struct omap_connector *omap_connector = to_omap_connector(connector);

	for (i = 0; i < DRM_CONNECTOR_MAX_ENCODER; i++) {
		struct drm_mode_object *obj;

		if (connector->encoder_ids[i] == 0)
			break;

		obj = drm_mode_object_find(connector->dev,
				connector->encoder_ids[i],
				DRM_MODE_OBJECT_ENCODER);

		if (obj) {
			struct drm_encoder *encoder = obj_to_encoder(obj);
			struct omap_overlay_manager *mgr =
					omap_encoder_get_manager(encoder);
			DBG("%s: found %s", omap_connector->dssdev->name,
					mgr->name);
			return encoder;
		}
	}

	DBG("%s: no encoder", omap_connector->dssdev->name);

	return NULL;
}

static const struct drm_connector_funcs omap_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = omap_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = omap_connector_destroy,
};

static const struct drm_connector_helper_funcs omap_connector_helper_funcs = {
	.get_modes = omap_connector_get_modes,
	.mode_valid = omap_connector_mode_valid,
	.best_encoder = omap_connector_attached_encoder,
};

/* called from encoder when mode is set, to propagate settings to the dssdev */
void omap_connector_mode_set(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	struct drm_device *dev = connector->dev;
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	struct omap_video_timings timings;

	copy_timings_drm_to_omap(&timings, mode);

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			omap_connector->dssdev->name,
			mode->base.id, mode->name, mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal, mode->type, mode->flags);

	if (dssdrv->check_timings(dssdev, &timings)) {
		dev_err(dev->dev, "could not set timings\n");
		return;
	}

	dssdrv->set_timings(dssdev, &timings);
}

enum omap_dss_update_mode omap_connector_get_update_mode(
		struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s", omap_connector->dssdev->name);

	if (dssdrv->get_update_mode) {
		return dssdrv->get_update_mode(dssdev);
	}

	return -1;
}
EXPORT_SYMBOL(omap_connector_get_update_mode);

int omap_connector_set_update_mode(struct drm_connector *connector,
		enum omap_dss_update_mode mode)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s: %d", omap_connector->dssdev->name, mode);

	if (dssdrv->set_update_mode) {
		return dssdrv->set_update_mode(dssdev, mode);
	}

	return -EINVAL;
}
EXPORT_SYMBOL(omap_connector_set_update_mode);

int omap_connector_sync(struct drm_connector *connector)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);
	struct omap_dss_device *dssdev = omap_connector->dssdev;
	struct omap_dss_driver *dssdrv = dssdev->driver;

	DBG("%s", omap_connector->dssdev->name);

	if (dssdrv->sync) {
		return dssdrv->sync(dssdev);
	}

	return -EINVAL;
}
EXPORT_SYMBOL(omap_connector_sync);

/* flush an area of the framebuffer (in case of manual update display that
 * is not automatically flushed)
 */
void omap_connector_flush(struct drm_connector *connector,
		int x, int y, int w, int h)
{
	struct omap_connector *omap_connector = to_omap_connector(connector);

	/* TODO: enable when supported in dss */
	VERB("%s: %d,%d, %dx%d", omap_connector->dssdev->name, x, y, w, h);
}

/* initialize connector */
struct drm_connector * omap_connector_init(struct drm_device *dev,
		int connector_type, struct omap_dss_device *dssdev)
{
	struct drm_connector *connector = NULL;
	struct omap_connector *omap_connector;

	DBG("%s", dssdev->name);

	omap_connector = kzalloc(sizeof(struct omap_connector), GFP_KERNEL);
	if (!omap_connector) {
		dev_err(dev->dev, "could not allocate connector\n");
		goto fail;
	}

	omap_connector->dssdev = dssdev;
	connector = &omap_connector->base;

	drm_connector_init(dev, connector, &omap_connector_funcs,
				connector_type);
	drm_connector_helper_add(connector, &omap_connector_helper_funcs);

	if (dssdev->caps & OMAP_DSS_DISPLAY_CAP_HPD) {
		connector->polled = 0;
	} else {
		connector->polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;
	}

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_sysfs_connector_add(connector);

	/* store resume info for suspended displays */
	switch (dssdev->state) {
	case OMAP_DSS_DISPLAY_SUSPENDED:
		dssdev->activate_after_resume = true;
		break;
	case OMAP_DSS_DISPLAY_DISABLED:
		if (dssdev->driver) {
			int ret = dssdev->driver->enable(dssdev);
			if (ret) {
				DBG("%s: failed to enable: %d",
						dssdev->name, ret);
				dssdev->driver->disable(dssdev);
			}
		}
		break;
	default:
		break;
	}

	return connector;

fail:
	if (connector) {
		drm_connector_cleanup(connector);
		kfree(omap_connector);
	}

	return NULL;
}
