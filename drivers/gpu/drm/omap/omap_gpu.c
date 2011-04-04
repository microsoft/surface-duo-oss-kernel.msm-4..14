/*
 * linux/drivers/gpu/drm/omap/omap_gpu.c
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

#include "drm_crtc_helper.h"
#include "drm_fb_helper.h"

#define DRIVER_NAME		MODULE_NAME
#define DRIVER_DESC		"OMAP GPU"
#define DRIVER_DATE		"20110403"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

struct drm_device *drm_device;

/* TODO: think about how to handle more than one plugin.. ie. some ops
 * me might want to stop on the first plugin that doesn't return an
 * error, etc..
 */
LIST_HEAD(plugin_list);

/* keep track of whether we are already loaded.. we may need to call
 * plugin's load() if they register after we are already loaded
 */
static bool loaded = false;

/*
 * mode config funcs
 */

/* Notes about mapping DSS and DRM entities:
 *    CRTC:        overlay
 *    encoder:     manager.. with some extension to allow one primary CRTC
 *                 and zero or more video CRTC's to be mapped to one encoder?
 *    connector:   dssdev.. manager can be attached/detached from different
 *                 devices
 */

static void omap_fb_output_poll_changed(struct drm_device *dev)
{
	struct omap_gpu_private *priv = dev->dev_private;
	DBG("dev=%p", dev);
	if (priv->fbdev) {
		drm_fb_helper_hotplug_event(priv->fbdev);
	}
}

static struct drm_mode_config_funcs omap_mode_config_funcs = {
	.fb_create = omap_framebuffer_create,
	.output_poll_changed = omap_fb_output_poll_changed,
};

static int get_connector_type(struct omap_dss_device *dssdev)
{
	switch (dssdev->type) {
	case OMAP_DISPLAY_TYPE_DPI:
		if (!strcmp(dssdev->name, "dvi"))
			return DRM_MODE_CONNECTOR_DVID;
	default:
		return DRM_MODE_CONNECTOR_Unknown;
	}
}

static int omap_gpu_notifier(struct notifier_block *nb,
		unsigned long evt, void *arg)
{
	switch (evt) {
	case OMAP_DSS_SIZE_CHANGE:
	case OMAP_DSS_HOTPLUG_CONNECT:
	case OMAP_DSS_HOTPLUG_DISCONNECT: {
		struct drm_device *dev = drm_device;
		DBG("hotplug event: evt=%d, dev=%p", evt, dev);
		if (dev) {
			drm_sysfs_hotplug_event(dev);
		}
		return NOTIFY_OK;
	}
	default:  /* don't care about other events for now */
		return NOTIFY_DONE;
	}
}

static void dump_video_chains(void)
{
	int i;

	DBG("dumping video chains: ");
	for (i = 0; i < omap_dss_get_num_overlays(); i++) {
		struct omap_overlay *ovl = omap_dss_get_overlay(i);
		struct omap_overlay_manager *mgr = ovl->manager;
		struct omap_dss_device *dssdev = mgr ? mgr->device : NULL;
		if (dssdev) {
			DBG("%d: %s -> %s -> %s", i, ovl->name, mgr->name,
						dssdev->name);
		} else if (mgr) {
			DBG("%d: %s -> %s", i, ovl->name, mgr->name);
		} else {
			DBG("%d: %s", i, ovl->name);
		}
	}
}

static int omap_modeset_init(struct drm_device *dev)
{
	const struct omap_gpu_platform_data *pdata = dev->dev->platform_data;
	struct omap_gpu_private *priv = dev->dev_private;
	struct omap_dss_device *dssdev = NULL;
	int i, j;
	unsigned int connected_connectors = 0;

	/* create encoders for each manager */
	int create_encoder(int i) {
		struct omap_overlay_manager *mgr =
				omap_dss_get_overlay_manager(i);
		struct drm_encoder *encoder = omap_encoder_init(dev, mgr);

		if (!encoder) {
			dev_err(dev->dev, "could not create encoder\n");
			return -ENOMEM;
		}

		priv->encoders[priv->num_encoders++] = encoder;

		return 0;
	}

	/* create connectors for each display device */
	int create_connector(struct omap_dss_device *dssdev) {
		static struct notifier_block *notifier;
		struct drm_connector *connector;

		if (!dssdev->driver) {
			dev_warn(dev->dev, "%s has no driver.. skipping it\n",
					dssdev->name);
			return 0;
		}

		if (!(dssdev->driver->get_timings ||
					dssdev->driver->get_edid)) {
			dev_warn(dev->dev, "%s driver does not support "
				"get_timings or get_edid.. skipping it!\n",
				dssdev->name);
			return 0;
		}

		connector = omap_connector_init(dev,
				get_connector_type(dssdev), dssdev);

		if (!connector) {
			dev_err(dev->dev, "could not create connector\n");
			return -ENOMEM;
		}

		/* track what is already connected.. rather than looping thru
		 * all connectors twice later, first for connected then for
		 * remainder (which could be a race condition if connected
		 * status changes)
		 */
		if (omap_connector_detect(connector, true) ==
				connector_status_connected) {
			connected_connectors |= (1 << priv->num_connectors);
		}

		priv->connectors[priv->num_connectors++] = connector;

		notifier = kzalloc(sizeof(struct notifier_block), GFP_KERNEL);
		notifier->notifier_call = omap_gpu_notifier;
		omap_dss_add_notify(dssdev, notifier);

		for (j = 0; j < priv->num_encoders; j++) {
			struct omap_overlay_manager *mgr =
				omap_encoder_get_manager(priv->encoders[j]);
			if (mgr->device == dssdev) {
				drm_mode_connector_attach_encoder(connector,
						priv->encoders[j]);
			}
		}

		return 0;
	}

	/* create up to max_overlays CRTCs mapping to overlays.. by default,
	 * connect the overlays to different managers/encoders, giving priority
	 * to encoders connected to connectors with a detected connection
	 */
	int create_crtc(int i) {
		struct omap_overlay *ovl = omap_dss_get_overlay(i);
		struct omap_overlay_manager *mgr = NULL;
		struct drm_crtc *crtc;

		if (ovl->manager) {
			DBG("disconnecting %s from %s", ovl->name,
						ovl->manager->name);
			ovl->unset_manager(ovl);
		}

		/* find next best connector, ones with detected connection first
		 */
		while (j < priv->num_connectors && !mgr) {
			if (connected_connectors & (1 << j)) {
				struct drm_encoder * encoder =
					omap_connector_attached_encoder(
							priv->connectors[j]);
				if (encoder) {
					mgr = omap_encoder_get_manager(encoder);
				}
			}
			j++;
		}

		/* if we couldn't find another connected connector, lets start
		 * looking at the unconnected connectors:
		 */
		while (j < 2 * priv->num_connectors && !mgr) {
			int idx = j - priv->num_connectors;
			if (!(connected_connectors & (1 << idx))) {
				struct drm_encoder * encoder =
					omap_connector_attached_encoder(
							priv->connectors[idx]);
				if (encoder) {
					mgr = omap_encoder_get_manager(encoder);
				}
			}
			j++;
		}

		if (mgr) {
			DBG("connecting %s to %s", ovl->name, mgr->name);
			ovl->set_manager(ovl, mgr);
		}

		crtc = omap_crtc_init(dev, ovl);

		if (!crtc) {
			dev_err(dev->dev, "could not create CRTC\n");
			return -ENOMEM;
		}

		priv->crtcs[priv->num_crtcs++] = crtc;

		return 0;
	}

	drm_mode_config_init(dev);

	if (pdata) {
		/* if platform data is provided by the board file, use it to
		 * control which overlays, managers, and devices we own.
		 */
		for (i = 0; i < pdata->mgr_cnt; i++) {
			if (create_encoder(pdata->mgr_ids[i])) {
				goto fail;
			}
		}

		for (i = 0; i < pdata->dev_cnt; i++) {
			int m(struct omap_dss_device *dssdev, void *data) {
				return ! strcmp(dssdev->name, data);
			}
			struct omap_dss_device *dssdev =
				omap_dss_find_device(
					(void *)pdata->dev_names[i], m);
			if (!dssdev) {
				dev_warn(dev->dev, "no such dssdev: %s\n",
						pdata->dev_names[i]);
				continue;
			}
			if (create_connector(dssdev)) {
				goto fail;
			}
		}

		j = 0;
		for (i = 0; i < pdata->ovl_cnt; i++) {
			if (create_crtc(pdata->ovl_ids[i])) {
				goto fail;
			}
		}
	} else {
		/* otherwise just grab up to CONFIG_DRM_OMAP_NUM_CRTCS and try
		 * to make educated guesses about everything else
		 */
		int max_overlays = min(omap_dss_get_num_overlays(),
					CONFIG_DRM_OMAP_NUM_CRTCS);

		for (i = 0; i < omap_dss_get_num_overlay_managers(); i++) {
			if (create_encoder(i)) {
				goto fail;
			}
		}

		for_each_dss_dev(dssdev) {
			if (create_connector(dssdev)) {
				goto fail;
			}
		}

		j = 0;
		for (i = 0; i < max_overlays; i++) {
			if (create_crtc(i)) {
				goto fail;
			}
		}
	}

	/* for now keep the mapping of CRTCs and encoders static.. */
	for (i = 0; i < priv->num_encoders; i++) {
		struct drm_encoder *encoder = priv->encoders[i];
		struct omap_overlay_manager *mgr =
				omap_encoder_get_manager(encoder);

		encoder->possible_crtcs = 0;

		for (j = 0; j < priv->num_crtcs; j++) {
			struct omap_overlay *ovl =
					omap_crtc_get_overlay(priv->crtcs[j]);
			if (ovl->manager == mgr) {
				encoder->possible_crtcs |= (1 << j);
			}
		}

		DBG("%s: possible_crtcs=%08x", mgr->name,
					encoder->possible_crtcs);
	}

	dump_video_chains();

	dev->mode_config.min_width = 640;
	dev->mode_config.min_height = 480;

	/* note: pvr can't currently handle dst surfaces larger than 2k by 2k */
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.funcs = &omap_mode_config_funcs;

	drm_kms_helper_poll_init(dev);

	return 0;

fail:
	/* TODO: cleanup what has been created so far */
	return -EINVAL;
}

/*
 * drm driver funcs
 */

/**
 * load - setup chip and create an initial config
 * @dev: DRM device
 * @flags: startup flags
 *
 * The driver load routine has to do several things:
 *   - initialize the memory manager
 *   - allocate initial config memory
 *   - setup the DRM framebuffer with the allocated memory
 */
static int dev_load(struct drm_device *dev, unsigned long flags)
{
	struct omap_gpu_private *priv;
	struct omap_gpu_plugin *plugin;
	int ret;

	DBG("load: dev=%p", dev);

	drm_device = dev;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev->dev, "could not allocate priv\n");
		return -1;
	}

	dev->dev_private = priv;

	ret = omap_modeset_init(dev);
	if (ret) {
		dev_err(dev->dev, "omap_modeset_init failed: ret=%d\n", ret);
		// hmm
		//return ret;
	}

	priv->fbdev = omap_fbdev_init(dev);
	if (!priv->fbdev) {
		dev_err(dev->dev, "omap_fbdev_init failed\n");
		ret = -ENOMEM;
		// hmm
		//return ret;
	}

	loaded = true;

	list_for_each_entry(plugin, &plugin_list, list) {
		ret = plugin->load(dev, flags);
	}

	return 0;
}

static int dev_unload(struct drm_device *dev)
{
	struct omap_gpu_plugin *plugin;
	int ret;

	DBG("unload: dev=%p", dev);

	list_for_each_entry(plugin, &plugin_list, list) {
		ret = plugin->unload(dev);
	}

	drm_kms_helper_poll_fini(dev);

	loaded = false;

	return 0;
}

static int dev_open(struct drm_device *dev, struct drm_file *file)
{
	struct omap_gpu_plugin *plugin;
	bool found_pvr = false;
	int ret;

	file->driver_priv = NULL;

	DBG("open: dev=%p, file=%p", dev, file);

	list_for_each_entry(plugin, &plugin_list, list) {
		if (!strcmp(DRIVER_NAME "_pvr", plugin->name)) {
			found_pvr = true;
			break;
		}
	}

	if (!found_pvr) {
		DBG("open: PVR submodule not loaded.. let's try now");
		request_module(DRIVER_NAME "_pvr");
	}

	list_for_each_entry(plugin, &plugin_list, list) {
		ret = plugin->open(dev, file);
	}

	return 0;
}

static int dev_firstopen(struct drm_device *dev)
{
	DBG("firstopen: dev=%p", dev);
	return 0;
}

/**
 * lastclose - clean up after all DRM clients have exited
 * @dev: DRM device
 *
 * Take care of cleaning up after all DRM clients have exited.  In the
 * mode setting case, we want to restore the kernel's initial mode (just
 * in case the last client left us in a bad state).
 *
 * Additionally, in the non-mode setting case, we'll tear down the AGP
 * and DMA structures, since the kernel won't be using them, and clean
 * up any GEM state.
 */
static void dev_lastclose(struct drm_device * dev)
{
	DBG("lastclose: dev=%p", dev);
}

static void dev_preclose(struct drm_device * dev, struct drm_file *file)
{
	DBG("preclose: dev=%p", dev);
}

static void dev_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct omap_gpu_plugin *plugin;
	int ret;

	DBG("postclose: dev=%p, file=%p", dev, file);

	list_for_each_entry(plugin, &plugin_list, list) {
		ret = plugin->release(dev, file);
	}

	return;
}

/**
 * enable_vblank - enable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Enable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 *
 * RETURNS
 * Zero on success, appropriate errno if the given @crtc's vblank
 * interrupt cannot be enabled.
 */
static int dev_enable_vblank(struct drm_device *dev, int crtc)
{
	DBG("enable_vblank: dev=%p, crtc=%d", dev, crtc);
	return 0;
}

/**
 * disable_vblank - disable vblank interrupt events
 * @dev: DRM device
 * @crtc: which irq to enable
 *
 * Disable vblank interrupts for @crtc.  If the device doesn't have
 * a hardware vblank counter, this routine should be a no-op, since
 * interrupts will have to stay on to keep the count accurate.
 */
static void dev_disable_vblank(struct drm_device *dev, int crtc)
{
	DBG("disable_vblank: dev=%p, crtc=%d", dev, crtc);
}

/**
 * Called by \c drm_device_is_agp.  Typically used to determine if a
 * card is really attached to AGP or not.
 *
 * \param dev  DRM device handle
 *
 * \returns
 * One of three values is returned depending on whether or not the
 * card is absolutely \b not AGP (return of 0), absolutely \b is AGP
 * (return of 1), or may or may not be AGP (return of 2).
 */
static int dev_device_is_agp(struct drm_device *dev)
{
	return 0;
}

static irqreturn_t dev_irq_handler(DRM_IRQ_ARGS)
{
	return IRQ_HANDLED;
}

static void dev_irq_preinstall(struct drm_device *dev)
{
	DBG("irq_preinstall: dev=%p", dev);
}

static int dev_irq_postinstall(struct drm_device *dev)
{
	DBG("irq_postinstall: dev=%p", dev);
	return 0;
}

static void dev_irq_uninstall(struct drm_device *dev)
{
	DBG("irq_uninstall: dev=%p", dev);
}

static int fop_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap_gpu_plugin *plugin;
	int ret = 0;

	list_for_each_entry(plugin, &plugin_list, list) {
		ret = plugin->mmap(file, vma);
		if (!ret) {
			/* on first plugin that succeeds, bail out of iteration */
			return ret;
		}
	}

	return ret;
}

static struct drm_driver omap_gpu_driver;

static int pdev_suspend(struct platform_device *pDevice, pm_message_t state)
{
	DBG("pdev_suspend");
	return 0;
}

static int pdev_resume(struct platform_device *device)
{
	DBG("pdev_resume");
	return 0;
}

static void pdev_shutdown(struct platform_device *device)
{
	DBG("pdev_shutdown");
}

static int pdev_probe(struct platform_device *device)
{
	DBG("pdev_probe: %s", device->name);
	return drm_get_platform_dev(device, &omap_gpu_driver);
}

static int pdev_remove(struct platform_device *device)
{
	DBG("pdev_remove");
	drm_put_dev(drm_device);
	return 0;
}

struct drm_ioctl_desc ioctls[DRM_COMMAND_END - DRM_COMMAND_BASE] = {{0}};

static struct drm_driver omap_gpu_driver = {
		.driver_features = DRIVER_HAVE_IRQ |
			DRIVER_USE_PLATFORM_DEVICE | DRIVER_MODESET,
		.load = dev_load,
		.unload = dev_unload,
		.open = dev_open,
		.firstopen = dev_firstopen,
		.lastclose = dev_lastclose,
		.preclose = dev_preclose,
		.postclose = dev_postclose,
		.get_vblank_counter = drm_vblank_count,
		.enable_vblank = dev_enable_vblank,
		.disable_vblank = dev_disable_vblank,
		.device_is_agp = dev_device_is_agp,
		.irq_preinstall = dev_irq_preinstall,
		.irq_postinstall = dev_irq_postinstall,
		.irq_uninstall = dev_irq_uninstall,
		.irq_handler = dev_irq_handler,
		.reclaim_buffers = drm_core_reclaim_buffers,
		.ioctls = ioctls,
		.num_ioctls = 0,
		.fops = {
				.owner = THIS_MODULE,
				.open = drm_open,
				.unlocked_ioctl = drm_ioctl,
				.release = drm_release,
				.mmap = fop_mmap,
				.poll = drm_poll,
				.fasync = drm_fasync,
				.read = drm_read,
		},
		.platform_driver = {
			.driver = {
				.name = DRIVER_NAME,
			},
			.probe = pdev_probe,
			.remove = pdev_remove,
			.suspend = pdev_suspend,
			.resume = pdev_resume,
			.shutdown = pdev_shutdown,
		},
		.name = DRIVER_NAME,
		.desc = DRIVER_DESC,
		.date = DRIVER_DATE,
		.major = DRIVER_MAJOR,
		.minor = DRIVER_MINOR,
		.patchlevel = DRIVER_PATCHLEVEL,
};

int omap_gpu_register_plugin(struct omap_gpu_plugin *plugin)
{
	struct drm_device *dev = drm_device;
	int i;

	DBG("register plugin: %p (%s)", plugin, plugin->name);

	/* XXX: PVR code uses drm_file->driver_priv...
	 * need to come up with some sane way to handle this.
	 */

	list_add_tail(&plugin->list, &plugin_list);

	/* register the plugin's ioctl's */
	for (i = 0; i < plugin->num_ioctls; i++) {
		int nr = i + plugin->ioctl_start;

		/* check for out of bounds ioctl nr or already registered ioctl */
		if (nr > ARRAY_SIZE(ioctls) || ioctls[nr].func) {
			dev_err(dev->dev, "invalid ioctl: %d (nr=%d)\n", i, nr);
			return -EINVAL;
		}

		DBG("register ioctl: %d %08x", nr, plugin->ioctls[i].cmd);

		ioctls[nr] = plugin->ioctls[i];

		if (nr >= omap_gpu_driver.num_ioctls) {
			omap_gpu_driver.num_ioctls = nr + 1;
		}
	}

	if (loaded) {
		plugin->load(dev, 0);
	}

	return 0;
}
EXPORT_SYMBOL(omap_gpu_register_plugin);

int omap_gpu_unregister_plugin(struct omap_gpu_plugin *plugin)
{
	list_del(&plugin->list);
	return 0;
}
EXPORT_SYMBOL(omap_gpu_unregister_plugin);

struct fb_info * omap_gpu_get_fbdev(struct drm_device *dev)
{
	struct omap_gpu_private *priv = dev->dev_private;
	return priv->fbdev->fbdev;
}
EXPORT_SYMBOL(omap_gpu_get_fbdev);

static int __init omap_gpu_init(void)
{
	DBG("init");
	return drm_init(&omap_gpu_driver);
}

static void __exit omap_gpu_fini(void)
{
	DBG("fini");
	drm_exit(&omap_gpu_driver);
}

/* need late_initcall() so we load after dss_driver's are loaded */
late_initcall(omap_gpu_init);
module_exit(omap_gpu_fini);

MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_DESCRIPTION("OMAP DRM Display Driver");
MODULE_LICENSE("GPL v2");
