/*
 * cam_thermal_reg.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _CAM_THERMAL_REG_H_
#define _CAM_THERMAL_REG_H_

#include "cam_sensor_dev.h"

int cam_thermal_reg_register_tzd(struct cam_sensor_ctrl_t *s_ctrl);
int cam_thermal_reg_unregister_tzd(void);

#endif // _CAM_THERMAL_REG_H_