/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"[drm-shp] %s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/component.h>
#include <linux/of_irq.h>
#include <linux/kthread.h>
#include <uapi/linux/sched/types.h>
#include "sde_connector.h"
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc.h>
#include <drm/drm_vblank.h>

#include "msm_drv.h"
#include "msm_kms.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include "sde_crtc.h"
#include "sde_plane.h"
#include "shp_drm.h"

#ifdef CONFIG_DRM_SDE_SHD
#include "../shd/shd_drm.h"
#else
static bool shd_is_crtc_shared(struct drm_crtc *crtc1,
		struct drm_crtc *crtc2, bool check_roi)
{
	return false;
}
static void shd_update_shared_plane(struct drm_plane *plane,
		struct drm_crtc *crtc)
{
}
#endif

enum shp_seamless_mode {
	SHP_SEAMLESS_MODE_RESTRICT = 0,
	SHP_SEAMLESS_MODE_SHARED,
	SHP_SEAMLESS_MODE_NONE,
};

struct shp_plane;

struct shp_pool {
	struct shp_plane *active;
	struct list_head plane_list;
	int index;
};

struct shp_plane_state {
	bool handoff;
	bool active_changed_flag;
	bool detach_attach_flag;
	bool handoff_changed;
	bool handoff_done;
	uint32_t possible_crtcs;
	uint32_t pending_crtcs;
};

struct shp_plane {
	struct shp_pool *pool;
	struct list_head head;
	bool is_shared;
	enum shp_seamless_mode seamless_mode;
	bool detach_handoff;
	bool init_active;

	struct drm_plane_funcs funcs;
	const struct drm_plane_funcs *funcs_orig;

	struct shp_plane *master;
	struct drm_plane *plane;
	uint32_t default_crtcs;

	struct shp_plane_state new_state;
	struct shp_plane_state state;
};

struct shp_device {
	struct drm_device *dev;

	struct shp_plane *planes;
	int num_planes;

	struct drm_property *handoff_prop;

	struct msm_kms_funcs kms_funcs;
	const struct msm_kms_funcs *orig_kms_funcs;

	struct mutex dev_lock;
	uint32_t pending_plane_mask[MAX_CRTCS];
};

struct shp_device g_shp_device;

static void shp_plane_send_uevent(struct drm_device *dev)
{
	char *envp[2] = {"PLANE_POSSIBLE_CRTCS_UPDATED=1", NULL};

	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE,
				envp);

	SDE_DEBUG("possible crtcs update uevent\n");
}

static void shp_plane_update_pending(struct shp_plane *plane)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct drm_crtc *crtc;
	struct shp_plane_state *shp_state = &plane->new_state;
	uint32_t plane_mask = (1 << drm_plane_index(plane->plane));
	uint32_t crtc_idx;

	mutex_lock(&shp_dev->dev_lock);

	drm_for_each_crtc(crtc, shp_dev->dev) {
		crtc_idx = drm_crtc_index(crtc);

		if (shp_state->pending_crtcs & (1 << crtc_idx))
			shp_dev->pending_plane_mask[crtc_idx] |= plane_mask;
		else
			shp_dev->pending_plane_mask[crtc_idx] &= ~plane_mask;
	}

	mutex_unlock(&shp_dev->dev_lock);
}

static int shp_plane_validate(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane = &shp_dev->planes[plane->index];
	struct shp_plane *p;
	struct drm_crtc *crtc;
	struct drm_plane_state *plane_state;
	struct shp_plane_state *shp_state = &shp_plane->new_state;
	struct shp_plane_state *pstate;
	uint32_t crtc_mask;

	if (shp_state->handoff_done) {
		list_for_each_entry(p, &shp_plane->pool->plane_list,
					head) {
			plane_state = drm_atomic_get_plane_state(state->state,
						p->plane);
			if (IS_ERR(plane_state))
				return PTR_ERR(plane_state);

			pstate = &p->new_state;

			if (p->master != shp_plane->master)
				pstate->possible_crtcs = 0;
			else
				pstate->possible_crtcs = p->default_crtcs;

			pstate->pending_crtcs = 0;
			pstate->handoff = false;

			SDE_DEBUG("plane%d: 0x%x m=%d h=%d c=%d 0x%x\n",
				p->plane->base.id,
				pstate->possible_crtcs,
				p->master->plane->base.id,
				pstate->handoff,
				plane_state->crtc ?
					plane_state->crtc->base.id : 0,
				pstate->pending_crtcs);
		}
	} else if (shp_state->handoff_changed) {
		if (!state->crtc) {
			crtc_mask = 0xFFFFFFFF;
			shp_state->pending_crtcs = 0;
			shp_state->handoff = false;
		} else {
			bool check_roi;

			crtc_mask = 0;
			switch (shp_plane->seamless_mode) {
			case SHP_SEAMLESS_MODE_RESTRICT:
				check_roi = true;
				break;
			case SHP_SEAMLESS_MODE_SHARED:
				check_roi = false;
				break;
			default:
				goto next;
			}

			/* Due to the H/W limitation, here we only allow crtcs
			 * that share the same display to be handed off.
			 */
			drm_for_each_crtc(crtc, state->plane->dev) {
				if (shd_is_crtc_shared(state->crtc,
						crtc, check_roi)) {
					crtc_mask |= drm_crtc_mask(crtc);
					SDE_DEBUG("add pending crtc%d\n",
						crtc->base.id);
				}
			}
next:
			shp_state->pending_crtcs = crtc_mask;
		}

		list_for_each_entry(p, &shp_plane->pool->plane_list,
					head) {
			plane_state = drm_atomic_get_plane_state(state->state,
						p->plane);
			if (IS_ERR(plane_state))
				return PTR_ERR(plane_state);

			pstate = &p->new_state;

			if (p->master != shp_plane->master)
				pstate->possible_crtcs =
					crtc_mask & p->default_crtcs;
			else if (state->crtc ||
				(shp_plane->seamless_mode ==
					SHP_SEAMLESS_MODE_NONE))
				pstate->possible_crtcs = p->default_crtcs;
			else
				pstate->possible_crtcs = 0;

			SDE_DEBUG("plane%d: 0x%x m=%d h=%d c=%d 0x%x\n",
				p->plane->base.id,
				pstate->possible_crtcs,
				p->master->plane->base.id,
				pstate->handoff,
				plane_state->crtc ?
					plane_state->crtc->base.id : 0,
				pstate->pending_crtcs);
		}
	} else {
		list_for_each_entry(p, &shp_plane->pool->plane_list, head) {
			plane_state = drm_atomic_get_plane_state(state->state,
						p->plane);
			if (IS_ERR(plane_state))
				return PTR_ERR(plane_state);

			pstate = &p->new_state;

			if (p->master != shp_plane->master)
				pstate->possible_crtcs = p->default_crtcs;

			SDE_DEBUG("plane%d: 0x%x m=%d h=%d c=%d 0x%x\n",
				p->plane->base.id,
				pstate->possible_crtcs,
				p->master->plane->base.id,
				pstate->handoff,
				plane_state->crtc ?
					plane_state->crtc->base.id : 0,
				pstate->pending_crtcs);
		}
	}

	if (shp_plane->pool->active != shp_plane && state->crtc) {
		SDE_DEBUG("plane%d set to active\n", plane->base.id);
		shp_state->active_changed_flag = true;

		p = shp_plane->pool->active;
		if (p && p->plane->state->crtc) {
			SDE_DEBUG("old active plane%d crtc%d\n",
				p->plane->base.id,
				p->plane->state->crtc->base.id);
			p->new_state.detach_attach_flag = true;
		}
	}

	return 0;
}

static int shp_atomic_check(struct drm_atomic_state *state)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane, *p;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state, *cstate;
	struct drm_plane_state *plane_state;
	struct shp_plane_state *new_state, *old_state;
	struct shp_plane_state *pstate;
	uint32_t pending_crtcs, pending_planes;
	uint32_t plane_mask = 0;
	int index;
	int i, ret;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		mutex_lock(&shp_dev->dev_lock);
		pending_planes =
			shp_dev->pending_plane_mask[drm_crtc_index(crtc)];
		mutex_unlock(&shp_dev->dev_lock);

		/* check pending planes */
		if (!pending_planes)
			continue;

		/* check active crtc */
		if (!crtc_state->active || !crtc_state->plane_mask)
			continue;

		SDE_DEBUG("crtc%d pending plane_mask=0x%x\n",
			crtc->base.id, pending_planes);

		drm_for_each_plane_mask(plane, state->dev, pending_planes) {
			shp_plane = &shp_dev->planes[plane->index];
			new_state = &shp_plane->new_state;
			old_state = &shp_plane->state;

			plane_state = drm_atomic_get_existing_plane_state(
					state, plane);
			if (plane_state) {
				pending_crtcs = new_state->pending_crtcs;

				if (!(pending_crtcs & drm_crtc_mask(crtc)))
					continue;
			} else {
				/* lock and test plane state first */
				ret = drm_modeset_lock(&plane->mutex,
							state->acquire_ctx);
				if (ret)
					return ret;

				pending_crtcs = new_state->pending_crtcs;

				/* skip if crtc is no longer pending */
				if (!(pending_crtcs & drm_crtc_mask(crtc))) {
					drm_modeset_unlock(&plane->mutex);
					continue;
				}

				/* continue adding plane into state */
				plane_state =
					plane->funcs->atomic_duplicate_state(
							plane);
				if (!plane_state)
					return -ENOMEM;

				/* update plane in state */
				index = drm_plane_index(plane);
				state->planes[index].state = plane_state;
				state->planes[index].ptr = plane;
				state->planes[index].old_state = plane->state;
				state->planes[index].new_state = plane_state;
				plane_state->state = state;

				/* add affected crtc */
				if (plane_state->crtc) {
					cstate = drm_atomic_get_crtc_state(
						state, plane_state->crtc);
					if (IS_ERR(cstate))
						return PTR_ERR(cstate);
				}
			}

			SDE_DEBUG("update crtc%d pending plane%d\n",
				crtc->base.id,
				plane->base.id);

			/* detach pending planes */
			list_for_each_entry(p, &shp_plane->pool->plane_list,
							head) {
				plane_state = drm_atomic_get_plane_state(state,
						p->plane);
				if (IS_ERR(plane_state))
					return PTR_ERR(plane_state);

				pstate = &p->new_state;

				drm_atomic_set_fb_for_plane(plane_state,
						NULL);
				drm_atomic_set_crtc_for_plane(plane_state,
						NULL);

				pstate->possible_crtcs = 0;
			}

			new_state->pending_crtcs = 0;
			new_state->handoff = false;

			/* add to check list */
			plane_mask |= (1 << drm_plane_index(plane));
		}
	}

	/* mark updated planes */
	for_each_plane_in_state(state, plane, plane_state, i) {
		shp_plane = &shp_dev->planes[plane->index];
		new_state = &shp_plane->new_state;
		old_state = &shp_plane->state;

		/* allow changes only when possible_crtcs is valid */
		if (!shp_plane->is_shared || !new_state->possible_crtcs ||
			shp_plane != shp_plane->master)
			continue;

		/* check detach_handoff flag */
		if (shp_plane->detach_handoff) {
			if (plane_state->crtc && !plane->state->crtc)
				new_state->handoff_done = true;
			else if (!plane_state->crtc && plane->state->crtc)
				new_state->handoff_changed = true;

		/* check active change */
		} else if (plane_state->crtc &&
			shp_plane->pool->active != shp_plane->master) {
			if (new_state->handoff)
				new_state->handoff_changed = true;
			else
				new_state->handoff_done = true;
			SDE_DEBUG("activate plane%d\n", plane->base.id);

		/* check handoff set */
		} else if (new_state->handoff && (!old_state->handoff ||
			plane_state->crtc != plane->state->crtc)) {
			new_state->handoff_changed = true;
			SDE_DEBUG("set plane%d handoff\n", plane->base.id);

		/* check handoff clear */
		} else if (!new_state->handoff && old_state->handoff) {
			new_state->handoff_done = true;
			SDE_DEBUG("clear plane%d handoff\n", plane->base.id);
		}

		/* add state changed planes into mask */
		if (new_state->handoff_done || new_state->handoff_changed) {
			if (shp_plane->pool->active)
				plane_mask &= ~(1 << drm_plane_index(
					shp_plane->pool->active->plane));
			plane_mask |= (1 << drm_plane_index(plane));
		}
	}

	/* validate all affected planes */
	drm_for_each_plane_mask(plane, state->dev, plane_mask) {
		plane_state = drm_atomic_get_existing_plane_state(
					state, plane);
		ret = shp_plane_validate(plane, plane_state);
		if (ret)
			return ret;
	}

	return 0;
}

static int shp_kms_atomic_check(struct msm_kms *kms,
			struct drm_atomic_state *state)
{
	struct shp_device *shp_dev = &g_shp_device;
	int ret;

	ret = shp_atomic_check(state);
	if (ret)
		return ret;

	ret = shp_dev->orig_kms_funcs->atomic_check(kms, state);
	if (ret)
		return ret;

	return 0;
}

static void shp_kms_post_swap(struct msm_kms *kms,
			struct drm_atomic_state *state)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	struct drm_plane *plane;
	struct shp_plane_state *new_state, *old_state;
	bool update = false;
	int i;

	for_each_oldnew_plane_in_state(state, plane, old_plane_state,
				new_plane_state, i) {
		shp_plane = &shp_dev->planes[plane->index];
		new_state = &shp_plane->new_state;
		old_state = &shp_plane->state;

		if (!shp_plane->is_shared)
			continue;

		if (plane->possible_crtcs != new_state->possible_crtcs) {
			update = true;
			SDE_DEBUG("plane%d crtcs 0x%x to 0x%x\n",
				shp_plane->plane->base.id,
				plane->possible_crtcs,
				new_state->possible_crtcs);
			plane->possible_crtcs = new_state->possible_crtcs;
		}

		if (new_state->pending_crtcs != old_state->pending_crtcs) {
			SDE_DEBUG("plane%d pending_crtcs 0x%x to 0x%x\n",
				plane->base.id,
				old_state->pending_crtcs,
				new_state->pending_crtcs);
			shp_plane_update_pending(shp_plane);
			old_state->pending_crtcs =
				new_state->pending_crtcs;
		}

		if (new_state->detach_attach_flag) {
			if (!old_plane_state->crtc) {
				SDE_ERROR("plane%d invalid detach state\n",
					plane->base.id);
			} else {
				shd_update_shared_plane(plane,
					old_plane_state->crtc);
				SDE_DEBUG("plane%d skip detach\n",
					plane->base.id);
			}
		}

		if (new_state->active_changed_flag) {
			SDE_DEBUG("active plane%d to plane%d\n",
				shp_plane->pool->active ?
				shp_plane->pool->active->plane->base.id : 0,
				plane->base.id);
			shp_plane->pool->active = shp_plane;
		}

		old_state->handoff = new_state->handoff;
	}

	if (shp_dev->orig_kms_funcs->prepare_fence)
		shp_dev->orig_kms_funcs->prepare_fence(kms, state);

	if (update)
		shp_plane_send_uevent(state->dev);
}

static int shp_plane_atomic_set_property(struct drm_plane *plane,
				   struct drm_plane_state *state,
				   struct drm_property *property,
				   uint64_t val)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane;
	struct shp_plane_state *shp_state;

	if (plane->index >= shp_dev->num_planes) {
		SDE_ERROR("invalid plane index %d\n", plane->index);
		return -EINVAL;
	}

	shp_plane = &shp_dev->planes[plane->index];
	shp_state = &shp_plane->new_state;

	if (property == shp_dev->handoff_prop) {
		if (is_sde_plane_virtual(plane)) {
			SDE_ERROR("virtual plane doesn't support handoff\n");
			return -EINVAL;
		}

		shp_state->handoff = !!val;
		return 0;
	}

	return shp_plane->funcs_orig->atomic_set_property(plane,
			state, property, val);
}

static int shp_plane_atomic_get_property(struct drm_plane *plane,
				   const struct drm_plane_state *state,
				   struct drm_property *property,
				   uint64_t *val)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane;
	struct shp_plane_state *shp_state;

	if (plane->index >= shp_dev->num_planes) {
		SDE_ERROR("invalid plane index %d\n", plane->index);
		return -EINVAL;
	}

	shp_plane = &shp_dev->planes[plane->index];
	shp_state = &shp_plane->new_state;

	if (property == shp_dev->handoff_prop) {
		*val = shp_state->handoff;
		return 0;
	}

	return shp_plane->funcs_orig->atomic_get_property(plane,
			state, property, val);
}

struct drm_plane_state *shp_plane_atomic_duplicate_state(
				struct drm_plane *plane)
{
	struct shp_device *shp_dev = &g_shp_device;
	struct shp_plane *shp_plane;
	struct shp_plane_state *new_state, *old_state;

	if (plane->index >= shp_dev->num_planes) {
		SDE_ERROR("invalid plane index %d\n", plane->index);
		return NULL;
	}

	shp_plane = &shp_dev->planes[plane->index];
	new_state = &shp_plane->new_state;
	old_state = &shp_plane->state;

	if (shp_plane->is_shared) {
		new_state->pending_crtcs = old_state->pending_crtcs;
		new_state->possible_crtcs = plane->possible_crtcs;
		new_state->handoff = old_state->handoff;
		new_state->active_changed_flag = false;
		new_state->detach_attach_flag = false;
		new_state->handoff_changed = false;
		new_state->handoff_done = false;
	}

	return shp_plane->funcs_orig->atomic_duplicate_state(plane);
}

static int shp_parse(struct platform_device *pdev, struct shp_device *shp)
{
	struct drm_device *dev = shp->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct sde_kms *sde_kms = to_sde_kms(priv->kms);
	struct device_node *of_node, *parent_node;
	struct shp_pool *shp_pool;
	struct shp_plane *shp_plane, *parent, *p;
	struct shp_plane_state *shp_state;
	struct drm_plane *plane;
	enum sde_sspp sspp;
	const char *name;
	int dup_count, system_count, total_count, i, j;
	int index;
	int rc = 0;

	parent_node = of_get_child_by_name(pdev->dev.of_node,
			"qcom,add-planes");
	if (!parent_node) {
		SDE_ERROR("no planes defined\n");
		return -ENODEV;
	}

	dup_count = of_get_child_count(parent_node);
	if (!dup_count) {
		SDE_ERROR("no duplicated planes defined\n");
		return -EINVAL;
	}

	mutex_lock(&dev->mode_config.mutex);
	sde_power_resource_enable(&priv->phandle,
			sde_kms->core_client, true);

	system_count = priv->num_planes;
	total_count = dup_count + system_count;
	shp->planes = devm_kzalloc(&pdev->dev,
			sizeof(*shp_plane) * total_count, GFP_KERNEL);
	if (!shp->planes) {
		rc = -ENOMEM;
		goto out;
	}

	shp_pool = devm_kzalloc(&pdev->dev,
			sizeof(*shp_pool) * system_count, GFP_KERNEL);
	if (!shp_pool) {
		rc = -ENOMEM;
		goto out;
	}

	/* init swap state function */
	shp->orig_kms_funcs = priv->kms->funcs;
	shp->kms_funcs = *priv->kms->funcs;
	shp->kms_funcs.prepare_fence = shp_kms_post_swap;
	shp->kms_funcs.atomic_check = shp_kms_atomic_check;
	priv->kms->funcs = &shp->kms_funcs;

	/* init system planes states */
	index = system_count;
	for_each_child_of_node(parent_node, of_node) {
		rc = of_property_read_string(of_node,
				"qcom,plane-parent", &name);
		if (rc) {
			SDE_ERROR("failed to get parent name\n");
			goto out;
		}

		parent = NULL;
		for (j = 0; j < system_count; j++) {
			if (!strcmp(priv->planes[j]->name, name)) {
				parent = &shp->planes[j];
				if (!parent->is_shared) {
					parent->is_shared = true;
					parent->plane = priv->planes[j];
					parent->pool = &shp_pool[j];
					parent->pool->index = j;
					INIT_LIST_HEAD(
						&parent->pool->plane_list);
				}
				break;
			}
		}

		if (!parent) {
			SDE_ERROR("parent name %s is not found\n", name);
			rc = -EINVAL;
			goto out;
		}

		if (is_sde_plane_virtual(priv->planes[j])) {
			SDE_ERROR("virtual plane %s is not supported\n", name);
			rc = -EINVAL;
			goto out;
		}

		sspp = sde_plane_pipe(parent->plane);
		plane = sde_plane_init(dev, sspp,
				parent->plane->type,
				parent->plane->possible_crtcs, 0);

		if (!plane) {
			SDE_ERROR("failed to init plane %d\n", plane->index);
			rc = -EINVAL;
			goto out;
		}

		plane->possible_crtcs = 0;
		BUG_ON(plane->index != index);
		shp_plane = &shp->planes[index++];
		shp_plane->plane = plane;
		shp_plane->master = shp_plane;
		shp_plane->pool = parent->pool;
		shp_plane->default_crtcs = parent->plane->possible_crtcs;
		list_add_tail(&shp_plane->head, &shp_plane->pool->plane_list);
		drm_object_attach_property(&shp_plane->plane->base,
				shp->handoff_prop, 0);
		shp_plane->is_shared = true;
		shp_state = &shp_plane->state;

		if (!of_property_read_string(of_node,
				"qcom,plane-name", &name))
			plane->name = kasprintf(GFP_KERNEL, "%s", name);

		shp_state->handoff = of_property_read_bool(of_node,
				"qcom,plane-init-handoff");

		shp_plane->detach_handoff = of_property_read_bool(of_node,
				"qcom,plane-detach-handoff");

		shp_plane->init_active = of_property_read_bool(of_node,
				"qcom,plane-init-active");

		if (!of_property_read_string(of_node,
				"qcom,plane-seamless-mode", &name)) {
			if (!strcmp(name, "shared"))
				shp_plane->seamless_mode =
					SHP_SEAMLESS_MODE_SHARED;
			else if (!strcmp(name, "restrict"))
				shp_plane->seamless_mode =
					SHP_SEAMLESS_MODE_RESTRICT;
			else
				shp_plane->seamless_mode =
					SHP_SEAMLESS_MODE_NONE;

			/* update parent's seamless mode */
			parent->seamless_mode = shp_plane->seamless_mode;
		}

		/* reset plane */
		if (plane->funcs->reset)
			plane->funcs->reset(plane);
	}

	shp->num_planes = index;

	/* init system planes states to active state */
	for (i = 0; i < system_count; i++) {
		shp_plane = &shp->planes[i];
		shp_plane->plane = priv->planes[i];
		BUG_ON(shp_plane->plane->index != i);

		if (is_sde_plane_virtual(shp_plane->plane)) {
			for (j = 0; j < i; j++) {
				if (sde_plane_pipe(shp_plane->plane) ==
					sde_plane_pipe(priv->planes[j])) {
					if (!shp->planes[j].is_shared)
						break;
					shp_plane->pool = &shp_pool[j];
					shp_plane->master = &shp->planes[j];
					shp_plane->is_shared = true;
					break;
				}
			}
			if (!shp_plane->master)
				continue;
		} else {
			if (!shp_plane->is_shared)
				continue;

			shp_plane->master = shp_plane;
			shp_plane->pool->active = shp_plane;
			drm_object_attach_property(&shp_plane->plane->base,
				shp->handoff_prop, 0);
		}

		shp_plane->default_crtcs = shp_plane->plane->possible_crtcs;
		list_add_tail(&shp_plane->head, &shp_plane->pool->plane_list);
	}

	/* setup all planes with new atomic check */
	for (i = 0; i < shp->num_planes; i++) {
		shp_plane = &shp->planes[i];
		if (!shp_plane->is_shared)
			continue;

		shp_plane->funcs = *shp_plane->plane->funcs;
		shp_plane->funcs_orig = shp_plane->plane->funcs;
		shp_plane->funcs.atomic_set_property =
				shp_plane_atomic_set_property;
		shp_plane->funcs.atomic_get_property =
				shp_plane_atomic_get_property;
		shp_plane->funcs.atomic_duplicate_state =
				shp_plane_atomic_duplicate_state;
		shp_plane->plane->funcs = &shp_plane->funcs;
	}

	/* update init-handoff cases */
	for (i = system_count; i < shp->num_planes; i++) {
		shp_plane = &shp->planes[i];
		shp_state = &shp_plane->state;
		if (!shp_state->handoff && !shp_plane->init_active)
			continue;

		/* set plane to active state */
		shp_plane->pool->active = shp_plane;

		/* update possible crtcs */
		list_for_each_entry(p,
			&shp_plane->pool->plane_list, head) {

			/* only one plane can set to init_active or
			 * init_handoff state
			 */
			if (p != shp_plane &&
				(p->init_active || p->state.handoff)) {
				SDE_ERROR("both %s and %s set to active\n",
					shp_plane->plane->name,
					p->plane->name);
				rc = -EINVAL;
				goto out;
			}

			if (shp_state->handoff || p->master == shp_plane)
				p->plane->possible_crtcs = p->default_crtcs;
			else
				p->plane->possible_crtcs = 0;
		}
	}

out:
	sde_power_resource_enable(&priv->phandle,
				sde_kms->core_client, false);
	mutex_unlock(&dev->mode_config.mutex);

	/* dump all the planes */
	for (i = 0; i < shp->num_planes; i++) {
		shp_plane = &shp->planes[i];
		if (!shp_plane->is_shared)
			continue;

		shp_state = &shp_plane->state;
		SDE_DEBUG("%s: crtcs=0x%x/0x%x pool=%d virt=%d handoff=%d\n",
			shp_plane->plane->name,
			shp_plane->plane->possible_crtcs,
			shp_plane->default_crtcs,
			shp_plane->pool->index,
			is_sde_plane_virtual(shp_plane->plane),
			shp_state->handoff);
	}

	return rc;
}

#ifdef CONFIG_DRM_SDE_SHD
static int sde_shp_match_unprobed_name(struct device *dev, void *data)
{
	struct device_driver *drv = data;

	return drv->bus->match(dev, drv) && !dev_get_drvdata(dev);
}

static bool sde_shp_has_unprobed_device(const char *drv_name)
{
	struct device *dev;
	struct device_driver *drv;

	drv = driver_find(drv_name, &platform_bus_type);
	if (!drv)
		return false;

	dev = bus_find_device(&platform_bus_type, NULL,
			(void *)drv, sde_shp_match_unprobed_name);

	return (dev != NULL);
}
#endif

static int sde_shp_probe(struct platform_device *pdev)
{
	struct shp_device *shp_dev;
	struct drm_minor *minor;
	struct drm_device *dev;
	int ret;

	/* defer until primary drm is created */
	minor = drm_minor_acquire(0);
	if (IS_ERR(minor))
		return -EPROBE_DEFER;

	dev = minor->dev;
	drm_minor_release(minor);
	if (!dev)
		return -EPROBE_DEFER;

#ifdef CONFIG_DRM_SDE_SHD
	if (sde_shp_has_unprobed_device("sde_shd"))
		return -EPROBE_DEFER;
#endif

	shp_dev = &g_shp_device;
	if (shp_dev->dev) {
		pr_err("only single device is supported\n");
		return -EEXIST;
	}

	shp_dev->dev = dev;
	mutex_init(&shp_dev->dev_lock);
	shp_dev->handoff_prop = drm_property_create_range(dev,
			DRM_MODE_PROP_ATOMIC, "handoff", 0, 1);

	if (!shp_dev->handoff_prop)
		return -ENOMEM;

	ret = shp_parse(pdev, shp_dev);
	if (ret) {
		SDE_ERROR("failed to parse shared plane device\n");
		return ret;
	}

	platform_set_drvdata(pdev, shp_dev);

	return 0;
}

static int sde_shp_remove(struct platform_device *pdev)
{
	struct shp_plane *shd_dev;

	shd_dev = platform_get_drvdata(pdev);
	if (!shd_dev)
		return 0;

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "qcom,sde-shared-plane"},
	{},
};

static struct platform_driver sde_shp_driver = {
	.probe = sde_shp_probe,
	.remove = sde_shp_remove,
	.driver = {
		.name = "sde_shp",
		.of_match_table = dt_match,
		.suppress_bind_attrs = true,
	},
};

static int __init sde_shp_register(void)
{
	return platform_driver_register(&sde_shp_driver);
}

static void __exit sde_shp_unregister(void)
{
	platform_driver_unregister(&sde_shp_driver);
}

module_init(sde_shp_register);
module_exit(sde_shp_unregister);
