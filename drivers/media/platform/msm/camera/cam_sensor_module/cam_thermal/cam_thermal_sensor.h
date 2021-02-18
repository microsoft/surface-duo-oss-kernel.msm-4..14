/*
 * cam_thermal_sensor.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _CAM_THERMAL_SENSOR_H_
#define _CAM_THERMAL_SENSOR_H_

#include "cam_sensor_dev.h"

int32_t cam_thermal_sensor_enable_temp_readings(struct cam_sensor_ctrl_t *s_ctrl);
int cam_thermal_sensor_get_temp(struct cam_sensor_ctrl_t *s_ctrl);

#endif // _CAM_THERMAL_SENSOR_H