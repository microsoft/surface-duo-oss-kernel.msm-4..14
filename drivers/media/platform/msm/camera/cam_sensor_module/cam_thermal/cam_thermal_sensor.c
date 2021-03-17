/*
 * cam_thermal_sensor.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include "cam_sensor_dev.h"
#include "cam_sensor_io.h"
#include "cam_thermal_sensor.h"

#define TEMP_SEN_CTL 0x0138
#define TEMP_SEN_OUT 0x013a
#define TEMP_SEN_ENABLE 0x01

/*
Temperature sensor output
0x81~0xEC -20 deg
0xED -19 deg
...
0x00
0 deg
...
0x4F 79 deg
0x50~0x7F 80 deg
*/

static int cam_thermal_sensor_convert_temp_to_celsius(uint8_t temp)
{
	int temperature = (int8_t)temp;

	if(temperature<-20)
		temperature=-20;
	else if(temperature>80)
		temperature=80;

	/* temperature under /sys/class/thermal is reported in degrees C * 1000 */
	temperature*=1000;

	CAM_INFO(CAM_THERMAL, "temperature: %x -> %d", temp, temperature);
	return temperature;
}

static struct cam_sensor_i2c_reg_array reg_data = {
	.reg_addr = TEMP_SEN_CTL,
	.reg_data = TEMP_SEN_ENABLE,
	.delay = 0,
	.data_mask = TEMP_SEN_ENABLE,
};

static struct cam_sensor_i2c_reg_setting reg_setting = {
	.reg_setting = &reg_data,
	.size = 1,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	.delay = 0,
};

int32_t cam_thermal_sensor_enable_temp_readings(struct cam_sensor_ctrl_t *s_ctrl)
{
	uint32_t rc = 0;

	rc = camera_io_dev_write(&(s_ctrl->io_master_info), &reg_setting);
	if (rc < 0)
	    CAM_ERR(CAM_THERMAL, "camera_io_dev_write Fail: %d", rc);

	return rc;
}

int cam_thermal_sensor_get_temp(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t data;
	uint8_t temp;
	rc = camera_io_dev_read(&(s_ctrl->io_master_info), TEMP_SEN_OUT, &data, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0)
	    CAM_ERR(CAM_THERMAL, "camera_io_dev_read Fail: %d", rc);
	temp = (data & 0xFF);
	CAM_INFO(CAM_THERMAL, "temperature: %d", temp);

	return cam_thermal_sensor_convert_temp_to_celsius(temp);
}
