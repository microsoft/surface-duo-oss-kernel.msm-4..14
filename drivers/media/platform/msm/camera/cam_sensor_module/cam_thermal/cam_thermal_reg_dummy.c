/*
 * cam_thermal_reg_dummy.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/device.h>

#include "cam_sensor_dev.h"
#include "cam_thermal_reg.h"

int cam_thermal_reg_register_tzd(struct cam_sensor_ctrl_t *s_ctrl)
{
	(void *)s_ctrl;
	return 0;
}

int cam_thermal_reg_unregister_tzd(void)
{
	return 0;
}
