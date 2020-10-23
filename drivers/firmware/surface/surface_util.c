/*
 * surface-util.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "surface_common.h"
#include "fuse_check.h"
#include "mcfg_check.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>


#define MODULE_NAME "surface_factory_util"


typedef struct _module {
	char* name;
	bool enabled;
	lib_initialize init;
	lib_deinitialize deinit;
}module;

module modules[] = {
	{"fuse_check", true, &fuse_check_init, &fuse_check_deinit},
	{"mcfg_check", true, &mcfg_check_init, &mcfg_check_deinit}
};

static const struct of_device_id factory_util_match_table[] = {
	{},
};

static void deinitiatize_libraries(struct kobject* kobj)
{
	int itr = 0, count = sizeof(modules)/sizeof(module);
	int ret;
	for(itr=0; itr< count; itr++)
	{
		if(modules[itr].enabled){
			ret = (*modules[itr].deinit)(kobj);
			if(ret != 0)
				pr_err("%s: failed to deinitialize library  %s error %d\n", __func__, modules[itr].name, ret);
		}
	}
}

static void initiatize_libraries(struct kobject* kobj)
{
	int itr = 0, count = sizeof(modules)/sizeof(module);
	int ret;
	for(itr=0; itr< count; itr++)
	{
		if(modules[itr].enabled){
			ret = (*modules[itr].init)(kobj);
			if(ret != 0)
				pr_err("%s: failed to initialize library  %s error %d\n", __func__, modules[itr].name, ret);
		}
	}
}

void update(const char* name){
	int itr, count = sizeof(modules)/sizeof(module);;
	for(itr=0; itr<count; itr++ ){
		if(strncpy(modules[itr].name, name, strlen(name))){
			pr_err("%s: Enabled library %s\n", __func__, modules[itr].name);
			modules[itr].enabled = true;
			break;
		}
	}
}

static void update_config(struct platform_device* pdev)
{
	const char *mod_name;
	int count;
	if (pdev->dev.of_node){
		int itr;
		count = of_property_count_strings(pdev->dev.of_node,
				"module-names");
		for(itr=0; itr<count; itr++ ){
			if(of_property_read_string_index(pdev->dev.of_node, "module-names",
							itr, &mod_name) == 0)
				update(mod_name);

		}
	}
}

static int surface_util_probe(struct platform_device *pdev)
{
	update_config(pdev);
	initiatize_libraries(&pdev->dev.kobj);
	return 0;
}

static int surface_util_remove(struct platform_device *pdev)
{
	deinitiatize_libraries(&(pdev->dev.kobj));
	return 0;
}

static const struct of_device_id surface_util_match[] = {
	{	.compatible = "surface,surface_util",
	},
	{}
};

static struct platform_driver surface_util_driver = {
	.probe = surface_util_probe,
	.remove = surface_util_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = surface_util_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

module_platform_driver(surface_util_driver);
MODULE_DESCRIPTION("Surface Utility driver");
MODULE_LICENSE("Surface Util V1");
