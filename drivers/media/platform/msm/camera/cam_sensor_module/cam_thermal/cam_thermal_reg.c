/*
 * cam_thermal_reg.c
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
#include <linux/gfp.h>
#include <linux/mutex.h>

#include "cam_sensor_dev.h"
#include "cam_thermal_reg.h"
#include "cam_thermal_sensor.h"

DEFINE_MUTEX(thermal_reg_mutex);

#define SUPPORTED_SENSOR_ID 0x351

struct thermal_zone_device *tzd = NULL;
struct cam_sensor_ctrl_t *s_cam_ctrl = NULL;

static int cam_thermal_reg_get_temp(void *data, int *temp)
{
	struct cam_sensor_ctrl_t *s_ctrl = data;
	mutex_lock(&thermal_reg_mutex);
	*temp = cam_thermal_sensor_get_temp(s_ctrl);
	mutex_unlock(&thermal_reg_mutex);
	CAM_INFO(CAM_THERMAL, "temperature: %d", *temp);

	return 0;
}

static int cam_thermal_is_sensor_supported()
{
	if(SUPPORTED_SENSOR_ID == s_cam_ctrl->sensordata->slave_info.sensor_id)
	{
		CAM_INFO(CAM_THERMAL, "sensor_id:0x%x is supported",
			s_cam_ctrl->sensordata->slave_info.sensor_id);
		return 1;
	}
	else
	{
		CAM_ERR(CAM_THERMAL, "sensor_id:0x%x is NOT supported",
			s_cam_ctrl->sensordata->slave_info.sensor_id);
		return 0;
	}
}

static struct thermal_zone_of_device_ops tzone_ops = {
	.get_temp = cam_thermal_reg_get_temp,
};

int cam_thermal_reg_register_tzd(struct cam_sensor_ctrl_t *s_ctrl)
{
	s_cam_ctrl = s_ctrl;

	if(!cam_thermal_is_sensor_supported())
	{
		CAM_ERR(CAM_THERMAL, "Sensor not supported, not registering thermal device.");
		return -EINVAL;
	}

	CAM_INFO(CAM_THERMAL,
		"camera sensor data: slot:%d,slave_addr:0x%x,sensor_id:0x%x",
		s_ctrl->soc_info.index,
		s_ctrl->sensordata->slave_info.sensor_slave_addr,
		s_ctrl->sensordata->slave_info.sensor_id);

	cam_thermal_sensor_enable_temp_readings(s_cam_ctrl);
	tzd = thermal_zone_of_sensor_register(&(s_ctrl->pdev)->dev, 0,
					      s_ctrl, &tzone_ops);
	if (IS_ERR(tzd)) {
		CAM_ERR(CAM_THERMAL, "Error registering TZ zone: %ld for dt_ch: %d\n",
		    PTR_ERR(tzd), 0);
		return -EINVAL;
	}
	CAM_INFO(CAM_THERMAL, "Registration successful - thermal zone: %s",
		 tzd->type);

	return 0;
}

int cam_thermal_reg_unregister_tzd()
{
	CAM_INFO(CAM_THERMAL, "device_name: %s",
		s_cam_ctrl->device_name);

	mutex_lock(&thermal_reg_mutex);
	thermal_zone_of_sensor_unregister(&(s_cam_ctrl->pdev)->dev, tzd);
	mutex_unlock(&thermal_reg_mutex);
	tzd = NULL;
	s_cam_ctrl = NULL;

	return 0;
}
