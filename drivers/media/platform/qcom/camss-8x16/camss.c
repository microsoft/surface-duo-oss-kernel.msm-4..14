/*
 * camss.c
 *
 * Qualcomm MSM Camera Subsystem - Core
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>

#include "camss.h"

static struct resources csiphy_res[] = {
	/* CSIPHY0 */
	{
		.regulator = { NULL },
		.clock = { "csiphy0_timer_src_clk", "csiphy0_timer_clk", "camss_ahb_src" },
		.clock_rate = { 200000000, 0, 0 },
		.reg = { "csiphy0", "csiphy0_clk_mux" },
		.interrupt = { "csiphy0" }
	},

	/* CSIPHY1 */
	{
		.regulator = { NULL },
		.clock = { "csiphy1_timer_src_clk", "csiphy1_timer_clk", "camss_ahb_src" },
		.clock_rate = { 200000000, 0, 0 },
		.reg = { "csiphy1", "csiphy1_clk_mux" },
		.interrupt = { "csiphy1" }
	}
};

static struct resources csid_res[] = {
	/* CSID0 */
	{
		.regulator = { "vdda" },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk", "csi0_ahb_clk",
			   "csi0_src_clk", "csi0_clk", "csi0_phy_clk",
			   "csi0_pix_clk", "csi0_rdi_clk", "camss_ahb_clk" },
		.clock_rate = { 0, 0, 0, 200000000, 0, 0, 0, 0, 0 },
		.reg = { "csid0" },
		.interrupt = { "csid0" }
	},

	/* CSID1 */
	{
		.regulator = { "vdda" },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk", "csi1_ahb_clk",
			   "csi1_src_clk", "csi1_clk", "csi1_phy_clk",
			   "csi1_pix_clk", "csi1_rdi_clk", "camss_ahb_clk" },
		.clock_rate = { 0, 0, 0, 200000000, 0, 0, 0, 0, 0 },
		.reg = { "csid1" },
		.interrupt = { "csid1" }
	},
};

static struct resources_ispif ispif_res = {
	/* ISPIF */
	.clock = { "camss_ahb_src", "ispif_ahb_clk" },
	.clock_for_reset = {
		"csi0_src_clk", "csi0_clk", "csi0_pix_clk", "csi0_rdi_clk",
		"csi1_src_clk", "csi1_clk", "csi1_pix_clk", "csi1_rdi_clk",
		"vfe_clk_src", "camss_vfe_vfe_clk", "camss_csi_vfe_clk",
		"camss_top_ahb_clk", "camss_ahb_clk" },
	.reg = { "ispif", "csi_clk_mux" },
	.interrupt = "ispif"

};

static struct resources vfe_res = {
	/* VFE0 */
	.regulator = { NULL },
	.clock = { "camss_top_ahb_clk", "vfe_clk_src", "camss_vfe_vfe_clk",
		   "camss_csi_vfe_clk", "iface_clk", "bus_clk",
		   "camss_ahb_clk" },
	.clock_rate = { 0, 320000000, 0, 0, 0, 0, 0, 0, 0 },
	.reg = { "vfe0", "vfe0_vbif" },
	.interrupt = { "vfe0" }
};

/*
 * camss_pipeline_pm_use_count - Count the number of users of a pipeline
 * @entity: The entity
 *
 * Return the total number of users of all video device nodes in the pipeline.
 */
static int camss_pipeline_pm_use_count(struct media_entity *entity)
{
	struct media_entity_graph graph;
	int use = 0;

	media_entity_graph_walk_start(&graph, entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_DEVNODE)
			use += entity->use_count;
	}

	return use;
}

/*
 * camss_pipeline_pm_power_one - Apply power change to an entity
 * @entity: The entity
 * @change: Use count change
 *
 * Change the entity use count by @change. If the entity is a subdev update its
 * power state by calling the core::s_power operation when the use count goes
 * from 0 to != 0 or from != 0 to 0.
 *
 * Return 0 on success or a negative error code on failure.
 */
static int camss_pipeline_pm_power_one(struct media_entity *entity, int change)
{
	struct v4l2_subdev *subdev;
	int ret;

	subdev = media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV
	       ? media_entity_to_v4l2_subdev(entity) : NULL;

	if (entity->use_count == 0 && change > 0 && subdev != NULL) {
		ret = v4l2_subdev_call(subdev, core, s_power, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
	}

	entity->use_count += change;
	WARN_ON(entity->use_count < 0);

	if (entity->use_count == 0 && change < 0 && subdev != NULL)
		v4l2_subdev_call(subdev, core, s_power, 0);

	return 0;
}

/*
 * camss_pipeline_pm_power - Apply power change to all entities in a pipeline
 * @entity: The entity
 * @change: Use count change
 *
 * Walk the pipeline to update the use count and the power state of all non-node
 * entities.
 *
 * Return 0 on success or a negative error code on failure.
 */
static int camss_pipeline_pm_power(struct media_entity *entity, int change)
{
	struct media_entity_graph graph;
	struct media_entity *first = entity;
	int ret = 0;

	if (!change)
		return 0;

	media_entity_graph_walk_start(&graph, entity);

	while (!ret && (entity = media_entity_graph_walk_next(&graph)))
		if (media_entity_type(entity) != MEDIA_ENT_T_DEVNODE)
			ret = camss_pipeline_pm_power_one(entity, change);

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

	while ((first = media_entity_graph_walk_next(&graph))
	       && first != entity)
		if (media_entity_type(first) != MEDIA_ENT_T_DEVNODE)
			camss_pipeline_pm_power_one(first, -change);

	return ret;
}

/*
 * msm_camss_pipeline_pm_use - Update the use count of an entity
 * @entity: The entity
 * @use: Use (1) or stop using (0) the entity
 *
 * Update the use count of all entities in the pipeline and power entities on or
 * off accordingly.
 *
 * Return 0 on success or a negative error code on failure. Powering entities
 * off is assumed to never fail. No failure can occur when the use parameter is
 * set to 0.
 */
int msm_camss_pipeline_pm_use(struct media_entity *entity, int use)
{
	int change = use ? 1 : -1;
	int ret;

	mutex_lock(&entity->parent->graph_mutex);

	/* Apply use count to node. */
	entity->use_count += change;
	WARN_ON(entity->use_count < 0);

	/* Apply power change to connected non-nodes. */
	ret = camss_pipeline_pm_power(entity, change);
	if (ret < 0)
		entity->use_count -= change;

	mutex_unlock(&entity->parent->graph_mutex);

	return ret;
}

/*
 * camss_pipeline_link_notify - Link management notification callback
 * @link: The link
 * @flags: New link flags that will be applied
 * @notification: The link's state change notification type (MEDIA_DEV_NOTIFY_*)
 *
 * React to link management on powered pipelines by updating the use count of
 * all entities in the source and sink sides of the link. Entities are powered
 * on or off accordingly.
 *
 * Return 0 on success or a negative error code on failure. Powering entities
 * off is assumed to never fail. This function will not fail for disconnection
 * events.
 */
static int camss_pipeline_link_notify(struct media_link *link, u32 flags,
				    unsigned int notification)
{
	struct media_entity *source = link->source->entity;
	struct media_entity *sink = link->sink->entity;
	int source_use = camss_pipeline_pm_use_count(source);
	int sink_use = camss_pipeline_pm_use_count(sink);
	int ret;

	if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH &&
	    !(flags & MEDIA_LNK_FL_ENABLED)) {
		/* Powering off entities is assumed to never fail. */
		camss_pipeline_pm_power(source, -sink_use);
		camss_pipeline_pm_power(sink, -source_use);
		return 0;
	}

	if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH &&
		(flags & MEDIA_LNK_FL_ENABLED)) {

		ret = camss_pipeline_pm_power(source, sink_use);
		if (ret < 0)
			return ret;

		ret = camss_pipeline_pm_power(sink, source_use);
		if (ret < 0)
			camss_pipeline_pm_power(source, -sink_use);

		return ret;
	}

	return 0;
}

static int camss_of_parse_node(struct device *dev, struct device_node *node,
			       struct camss_async_subdev *csd)
{
	struct csiphy_lanes_cfg *lncfg = &csd->interface.csi2.lane_cfg;
	int *settle_cnt = &csd->interface.csi2.settle_cnt;
	struct v4l2_of_endpoint vep;
	unsigned int i;

	v4l2_of_parse_endpoint(node, &vep);

	dev_dbg(dev, "parsing endpoint %s\n", node->full_name);

	csd->interface.csiphy_id = vep.base.port;

	lncfg->clk.pos = vep.bus.mipi_csi2.clock_lane;
	lncfg->clk.pol = vep.bus.mipi_csi2.lane_polarities[0];
	dev_dbg(dev, "clock lane polarity %u, pos %u\n",
		lncfg->clk.pol, lncfg->clk.pos);

	lncfg->num_data = vep.bus.mipi_csi2.num_data_lanes;

	lncfg->data = devm_kzalloc(dev, lncfg->num_data * sizeof(*lncfg->data),
								GFP_KERNEL);
	if (!lncfg->data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	for (i = 0; i < lncfg->num_data; i++) {
		lncfg->data[i].pos = vep.bus.mipi_csi2.data_lanes[i];
		lncfg->data[i].pol =
				vep.bus.mipi_csi2.lane_polarities[i + 1];
		dev_dbg(dev, "data lane %u polarity %u, pos %u\n", i,
			lncfg->data[i].pol, lncfg->data[i].pos);
	}

	of_property_read_u32(node, "qcom,settle-cnt", settle_cnt);

	return 0;
}

static int camss_of_parse_nodes(struct device *dev,
				struct v4l2_async_notifier *notifier)
{
	struct device_node *node = NULL;
	int size, i;
	int ret;

	while ((node = of_graph_get_next_endpoint(dev->of_node, node))) {
		notifier->num_subdevs++;
	}
	dev_err(dev, "notifier->num_subdevs = %u\n", notifier->num_subdevs);

	size = sizeof(*notifier->subdevs) * notifier->num_subdevs;
	notifier->subdevs = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!notifier->subdevs) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i = 0;
	while ((node = of_graph_get_next_endpoint(dev->of_node, node))) {
		struct camss_async_subdev *csd;

		csd = devm_kzalloc(dev, sizeof(*csd), GFP_KERNEL);
		if (!csd) {
			of_node_put(node);
			dev_err(dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		notifier->subdevs[i++] = &csd->asd;

		ret = camss_of_parse_node(dev, node, csd);
		if (ret < 0) {
			of_node_put(node);
			return ret;
		}

		csd->asd.match.of.node = of_graph_get_remote_port_parent(node);
		of_node_put(node);
		if (!csd->asd.match.of.node) {
			dev_warn(dev, "bad remote port parent\n");
			return -EINVAL;
		}

		csd->asd.match_type = V4L2_ASYNC_MATCH_OF;
	}

	return notifier->num_subdevs;
}

static int camss_init_subdevices(struct camss *camss)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		ret = msm_csiphy_subdev_init(&camss->csiphy[i],
					     &csiphy_res[i], i);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to init csiphy[%d] sub-device\n", i);
			return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++) {
		ret = msm_csid_subdev_init(&camss->csid[i],
					   &csid_res[i], i);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to init csid[%d] sub-device\n", i);
			return ret;
		}
	}

	ret = msm_ispif_subdev_init(&camss->ispif, &ispif_res);
	if (ret < 0) {
		dev_err(camss->dev, "Failed to init ispif sub-device\n");
		return ret;
	}

	ret = msm_vfe_subdev_init(&camss->vfe, &vfe_res);
	if (ret < 0) {
		dev_err(camss->dev, "Fail to init vfe sub-device\n");
		return ret;
	}

	return 0;
}

static int camss_register_entities(struct camss *camss)
{
	int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		ret = msm_csiphy_register_entities(&camss->csiphy[i],
						   &camss->v4l2_dev);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to register csiphy[%d] entity\n", i);
			goto err_reg_csiphy;
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		ret = msm_csid_register_entities(&camss->csid[i],
						 &camss->v4l2_dev);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to register csid[%d] entity\n", i);
			goto err_reg_csid;
		}
	}

	ret = msm_ispif_register_entities(&camss->ispif, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(camss->dev, "Fail to register ispif entities\n");
		goto err_reg_ispif;
	}

	ret = msm_vfe_register_entities(&camss->vfe, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(camss->dev, "Fail to register vfe entities\n");
		goto err_reg_vfe;
	}

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->csid); j++) {
			ret = media_entity_create_link(
				&camss->csiphy[i].subdev.entity,
				MSM_CSIPHY_PAD_SRC,
				&camss->csid[j].subdev.entity,
				MSM_CSID_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Fail to link %s->%s entities\n",
					camss->csiphy[i].subdev.entity.name,
					camss->csid[j].subdev.entity.name);
				goto err_link;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->ispif.line); j++) {
			ret = media_entity_create_link(
				&camss->csid[i].subdev.entity,
				MSM_CSID_PAD_SRC,
				&camss->ispif.line[j].subdev.entity,
				MSM_ISPIF_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Fail to link %s->%s entities\n",
					camss->csid[i].subdev.entity.name,
					camss->ispif.line[j].subdev.entity.name);
				goto err_link;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->ispif.line); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->vfe.line); j++) {
			ret = media_entity_create_link(
				&camss->ispif.line[i].subdev.entity,
				MSM_ISPIF_PAD_SRC,
				&camss->vfe.line[j].subdev.entity,
				MSM_VFE_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Fail to link %s->%s entities\n",
					camss->ispif.line[i].subdev.entity.name,
					camss->vfe.line[j].subdev.entity.name);
				goto err_link;
			}
		}
	}

	return 0;

err_link:
	msm_vfe_unregister_entities(&camss->vfe);
err_reg_vfe:
	msm_ispif_unregister_entities(&camss->ispif);
err_reg_ispif:

	i = ARRAY_SIZE(camss->csid);
err_reg_csid:
	for (i--; i >= 0; i--) {
		msm_csid_unregister_entities(&camss->csid[i]);
	}

	i = ARRAY_SIZE(camss->csiphy);
err_reg_csiphy:
	for (i--; i >= 0; i--) {
		msm_csiphy_unregister_entities(&camss->csiphy[i]);
	}

	return ret;
}

static void camss_unregister_entities(struct camss *camss)
{
	int i;

	/* TODO: Remove links? */

	for (i = ARRAY_SIZE(camss->csiphy) - 1; i >= 0; i--)
		msm_csiphy_unregister_entities(&camss->csiphy[i]);

	for (i = ARRAY_SIZE(camss->csid) - 1; i >= 0; i--)
		msm_csid_unregister_entities(&camss->csid[i]);

	msm_ispif_unregister_entities(&camss->ispif);
	msm_vfe_unregister_entities(&camss->vfe);
}

static int camss_subdev_notifier_bound(struct v4l2_async_notifier *async,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct media_entity *sensor = &subdev->entity;
	struct camss *camss = container_of(async, struct camss, notifier);
	struct camss_async_subdev *csd =
		container_of(asd, struct camss_async_subdev, asd);
	u8 id = csd->interface.csiphy_id;
	struct csiphy_device *csiphy = &camss->csiphy[id];
	struct media_entity *input = &csiphy->subdev.entity;
	unsigned int flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED;
	unsigned int pad = MSM_CSIPHY_PAD_SINK;
	unsigned int i;
	int ret;

	for (i = 0; i < sensor->num_pads; i++) {
		if (sensor->pads[i].flags & MEDIA_PAD_FL_SOURCE)
			break;
	}
	if (i == sensor->num_pads) {
		dev_err(camss->dev, "%s: no source pad in external entity\n",
			__func__);
		return -EINVAL;
	}

	ret = media_entity_create_link(sensor, i, input, pad, flags);
	if (ret < 0) {
		dev_err(camss->dev, "Fail to link %s->%s entities\n",
			sensor->name, input->name);
		return ret;
	}

	csiphy->cfg.csi2 = &csd->interface.csi2;

	return 0;
}

static int camss_subdev_notifier_complete(struct v4l2_async_notifier *async)
{
	struct camss *camss = container_of(async, struct camss, notifier);

	return v4l2_device_register_subdev_nodes(&camss->v4l2_dev);
}

static int camss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct camss *camss;
	int ret;

	dev_dbg(dev, "Enter\n");

	camss = devm_kzalloc(dev, sizeof(*camss), GFP_KERNEL);
	if (!camss) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	camss->dev = dev;
	platform_set_drvdata(pdev, camss);

	ret = camss_of_parse_nodes(dev, &camss->notifier);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return -ENODEV;

	ret = camss_init_subdevices(camss);
	if (ret < 0)
		return ret;

	camss->media_dev.dev = camss->dev;
	strlcpy(camss->media_dev.model, "QC MSM CAMSS",
		sizeof(camss->media_dev.model));
	camss->media_dev.driver_version = CAMSS_VERSION;
	camss->media_dev.link_notify = camss_pipeline_link_notify;
	ret = media_device_register(&camss->media_dev);
	if (ret < 0) {
		dev_err(dev, "%s: Media device registration failed (%d)\n",
			__func__, ret);
		return ret;
	}

	camss->v4l2_dev.mdev = &camss->media_dev;
	ret = v4l2_device_register(camss->dev, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(dev, "%s: V4L2 device registration failed (%d)\n",
			__func__, ret);
		goto err_register_v4l2;
	}

	ret = camss_register_entities(camss);
	if (ret < 0)
		goto err_register_entities;

	if (camss->notifier.num_subdevs) {
		camss->notifier.bound = camss_subdev_notifier_bound;
		camss->notifier.complete = camss_subdev_notifier_complete;

		ret = v4l2_async_notifier_register(&camss->v4l2_dev,
						   &camss->notifier);
		if (ret) {
			dev_err(dev,
				"%s: V4L2 async notifier registration failed (%d)\n",
				__func__, ret);
			goto err_register_subdevs;
		}
	} else {
		ret = v4l2_device_register_subdev_nodes(&camss->v4l2_dev);
		if (ret < 0) {
			dev_err(dev,
				"%s: V4L2 subdev nodes registration failed (%d)\n",
				__func__, ret);
			goto err_register_subdevs;
		}
	}

	dev_dbg(dev, "camss driver registered successfully!\n");

	return 0;

err_register_subdevs:
	camss_unregister_entities(camss);
err_register_entities:
	v4l2_device_unregister(&camss->v4l2_dev);
err_register_v4l2:
	media_device_unregister(&camss->media_dev);

	return ret;
}

static int camss_remove(struct platform_device *pdev)
{
	struct camss *camss = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&camss->notifier);
	camss_unregister_entities(camss);
	v4l2_device_unregister(&camss->v4l2_dev);
	media_device_unregister(&camss->media_dev);

	return 0;
}

static const struct of_device_id camss_dt_match[] = {
	{ .compatible = "qcom,msm-camss" },
	{ }
};

MODULE_DEVICE_TABLE(of, camss_dt_match);

static struct platform_driver qcom_camss_driver = {
	.probe = camss_probe,
	.remove = camss_remove,
	.driver = {
		.name = "qcom-camss",
		.of_match_table = camss_dt_match,
	},
};

module_platform_driver(qcom_camss_driver);

MODULE_ALIAS("platform:qcom-camss");
MODULE_DESCRIPTION("Qualcomm camera subsystem driver");
MODULE_LICENSE("GPL");
