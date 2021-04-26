/* Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CAM_SENSOR_SERDES_DEV_H_
#define _CAM_SENSOR_SERDES_DEV_H_

#include <soc/qcom/boot_stats.h>

#define CAMX_SENSOR_SERDES_DEV_NAME "cam-sensor-serdes-driver"

/**
 * struct cam_bchip_ctrl_t: Camera Bridgechip control structure
 * @device_name: Sensor device name
 * @pdev: Platform device
 * @of_node: Of node ptr
 * @cam_sensor_mutex: Bridgechip mutex
 * @is_probe_succeed: Probe succeeded or not
 * @id: Cell Index
 * @v4l2_dev_str: V4L2 device structure
 * @s_ctrl: Camera Sensor control structure
 * @bchip_init_task: Bridgechip init task structure
 */

struct cam_bchip_ctrl_t {
	char device_name[CAM_CTX_DEV_NAME_MAX_LENGTH];
	struct platform_device *pdev;
	struct device_node *of_node;
	struct mutex cam_bchip_mutex;
	uint8_t is_probe_succeed;
	uint32_t id;
	struct cam_subdev v4l2_dev_str;
	struct cam_sensor_ctrl_t *s_ctrl;
	struct task_struct *bchip_init_task;
};

static struct ais_sensor_i2c_wr_payload io_expander_reset[] = {
	{0x007d, 0x12, 0},
	{0x007d, 0x34, 100000},
};

static struct ais_sensor_i2c_wr_payload serializer_alias[] = {
	{0x0000, 0x82, 0},
};

static struct ais_sensor_i2c_wr_payload serializer_link_reset[] = {
	{0x0010, 0x31, 100000},
	{0x0042, 0xe4, 0},
	{0x0043, 0xba, 0},
	{0x0044, 0x8e, 0},
	{0x0045, 0x82, 0},
};

static struct ais_sensor_i2c_wr_payload serializer_addr_chng_A[] = {
	{0x006b, 0x10, 0},
	{0x0073, 0x11, 0},
	{0x007b, 0x30, 0},
	{0x0083, 0x30, 0},
	{0x0093, 0x30, 0},
	{0x009b, 0x30, 0},
	{0x00a3, 0x30, 0},
	{0x00ab, 0x30, 0},
	{0x008b, 0x30, 0},
};

static struct ais_sensor_i2c_wr_payload deserializer_intr_init[] = {
	{0x001c, 0x89, 0},
};

static struct ais_sensor_i2c_wr_payload serializer_init_8bit_regs_0[] = {
	{0x0002, 0x03, 0},
	{0x0100, 0x60, 0},
	{0x0101, 0x0a, 0},
	{0x01b0, 0x02, 0},
	{0x01b1, 0x03, 0},
	{0x01b2, 0x04, 0},
	{0x01b3, 0x05, 0},
	{0x01b4, 0x06, 0},
	{0x01b5, 0x07, 0},
	{0x01b6, 0x08, 0},
	{0x01b7, 0x09, 0},
	{0x01b8, 0x00, 0},
	{0x01b9, 0x01, 0},
	{0x0053, 0x00, 0},
	{0x0102, 0x0e, 0},
	{0x0311, 0x10, 0},
};

static struct ais_sensor_cmd_i2c_wr_array i2c_write_sequence[] = {
	{
		.i2c_config = {
			.slave_addr = 0x7c,
			.i2c_freq_mode = I2C_CUSTOM_MODE,
			.cmd_type = 0,
		},
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.reserved = 0,
		.count = 2,
		.wr_array = io_expander_reset,
	},
        {
                .i2c_config = {
                        .slave_addr = 0xc4,
                        .i2c_freq_mode = I2C_CUSTOM_MODE,
                        .cmd_type = 0,
                },
                .addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
                .data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
                .reserved = 0,
                .count = 1,
               .wr_array = serializer_alias,
        },
	{
		.i2c_config = {
			.slave_addr = 0x82,
			.i2c_freq_mode = I2C_CUSTOM_MODE,
			.cmd_type = 0,
		},
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.reserved = 0,
		.count = 5,
		.wr_array = serializer_link_reset,
	},
	{
		.i2c_config = {
			.slave_addr = 0x82,
			.i2c_freq_mode = I2C_CUSTOM_MODE,
			.cmd_type = 0,
		},
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.reserved = 0,
		.count = 9,
		.wr_array = serializer_addr_chng_A,
	},
	{
		.i2c_config = {
			.slave_addr = 0x90,
			.i2c_freq_mode = I2C_CUSTOM_MODE,
			.cmd_type = 0,
		},
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.reserved = 0,
		.count = 1,
		.wr_array = deserializer_intr_init,
	},
	{
		.i2c_config = {
			.slave_addr = 0x82,
			.i2c_freq_mode = I2C_CUSTOM_MODE,
			.cmd_type = 0,
		},
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.reserved = 0,
		.count = 16,
		.wr_array = serializer_init_8bit_regs_0,
	},
};

static struct ais_sensor_probe_cmd sensor_probe_cmd = {
	.i2c_config = {
		.slave_addr = 0x90,
		.i2c_freq_mode = I2C_CUSTOM_MODE,
		.cmd_type = 0,
	},
	.power_config = {
		.size_up = 3,
		.power_up_setting = {
			{
				.power_seq_type = 2,
				.reserved = 0,
				.config_val_low = 0,
				.config_val_high = 0,
				.delay = 0,
			},
			{
				.power_seq_type = 8,
				.reserved = 0,
				.config_val_low = 0,
				.config_val_high = 0,
				.delay = 1,
			},
			{
				.power_seq_type = 8,
				.reserved = 0,
				.config_val_low = 2,
				.config_val_high = 0,
				.delay = 20,
			},
		},
		.size_down = 2,
		.power_down_setting = {
			{
				.power_seq_type = 8,
				.reserved = 0,
				.config_val_low = 0,
				.config_val_high = 0,
				.delay = 1,
			},
			{
				.power_seq_type = 2,
				.reserved = 0,
				.config_val_low = 0,
				.config_val_high = 0,
				.delay = 0,
			},
		},
	},
};

/**
 * @b_ctrl: Camera Bridgechip ctrl structure
 * @arg:    Camera control command argument
 *
 * This API handles the camera control argument reached to sensor SerDes
 */
int32_t cam_sensor_serdes_driver_cmd(struct cam_bchip_ctrl_t *b_ctrl, void *arg);

#endif /* _CAM_SENSOR_SERDES_DEV_H_ */
