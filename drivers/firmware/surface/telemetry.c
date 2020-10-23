/*
 * telemetry.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/soc/surface/surface_utils.h>

struct kobject *telemetry_kobj = NULL;
EXPORT_SYMBOL_GPL(telemetry_kobj);

int telemetry_init()
{
    if(!telemetry_kobj)
    {
        telemetry_kobj = kobject_create_and_add("telemetry", kernel_kobj);
        if (!telemetry_kobj)
        {
            kobject_put(telemetry_kobj);
            return -ENOMEM;
        }
    }
    return 0;
}
EXPORT_SYMBOL(telemetry_init);