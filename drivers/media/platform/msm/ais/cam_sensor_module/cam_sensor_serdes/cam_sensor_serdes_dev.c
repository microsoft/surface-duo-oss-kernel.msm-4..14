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

#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_core.h"
#include "cam_sensor_serdes_dev.h"

void cam_serdes_shutdown(struct cam_bchip_ctrl_t *b_ctrl)
{
	(void *)b_ctrl;
}

static long cam_sensor_serdes_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_bchip_ctrl_t *b_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		CAM_ERR(CAM_SENSOR, "serdes cam control ioctl cmd");
		rc = cam_sensor_serdes_driver_cmd(b_ctrl, arg);
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd");
		rc = -EINVAL;
		break;
	}
	return rc;
}

int32_t cam_sensor_serdes_driver_cmd(struct cam_bchip_ctrl_t *b_ctrl,
        void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;

        if (!b_ctrl || !cmd) {
                CAM_ERR(CAM_SENSOR, "Invalid Args");
                return -EINVAL;
        }

	switch (cmd->op_code) {
	case AIS_SERDES_INIT_STATUS: {
		struct ais_serdes_cfg_status serdes_cfg_status;

		serdes_cfg_status.serdes_init_status = b_ctrl->is_probe_succeed;
		rc = copy_to_user((void __user *) cmd->handle,
				&serdes_cfg_status, sizeof(serdes_cfg_status));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed Copying to user");
			rc = -EFAULT;
		}
		CAM_ERR(CAM_SENSOR, "RRR AIS_SERDES_INIT_STATUS %d",
				serdes_cfg_status.serdes_init_status);
	}
		break;
	}

	return rc;
}

static int cam_sensor_serdes_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{

	struct cam_bchip_ctrl_t *b_ctrl =
		v4l2_get_subdevdata(sd);

	if (!b_ctrl) {
		CAM_ERR(CAM_SENSOR, "b_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(b_ctrl->cam_bchip_mutex));
	cam_serdes_shutdown(b_ctrl);
	mutex_unlock(&(b_ctrl->cam_bchip_mutex));

	return 0;
}

static struct v4l2_subdev_core_ops cam_sensor_serdes_subdev_core_ops = {
	.ioctl = cam_sensor_serdes_subdev_ioctl,
};

static struct v4l2_subdev_ops cam_sensor_serdes_subdev_ops = {
	.core = &cam_sensor_serdes_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_sensor_serdes_internal_ops = {
	.close = cam_sensor_serdes_subdev_close,
};

static int cam_sensor_serdes_init_subdev(struct cam_bchip_ctrl_t *b_ctrl)
{
	int rc = 0;

	b_ctrl->v4l2_dev_str.internal_ops =
		&cam_sensor_serdes_internal_ops;
	b_ctrl->v4l2_dev_str.ops =
		&cam_sensor_serdes_subdev_ops;
	strlcpy(b_ctrl->device_name, CAMX_SENSOR_SERDES_DEV_NAME,
		sizeof(b_ctrl->device_name));
	b_ctrl->v4l2_dev_str.name =
		b_ctrl->device_name;
	b_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	b_ctrl->v4l2_dev_str.ent_function =
		CAM_SERDES_DEVICE_TYPE;
	b_ctrl->v4l2_dev_str.token = b_ctrl;

	rc = cam_register_subdev(&(b_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_SENSOR, "Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int sensor_probe(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct ais_sensor_probe_cmd *probe_cmd = &sensor_probe_cmd;

	rc = cam_sensor_update_i2c_info(&probe_cmd->i2c_config, s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed in Updating the i2c Info");
	}

	rc = ais_sensor_update_power_settings(probe_cmd,
			&s_ctrl->sensordata->power_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed copy power settings");
	}

	/* Parse and fill vreg params for powerup settings */
	rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_setting,
			s_ctrl->sensordata->power_info.power_setting_size);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PUP rc %d",
				rc);
	}

	/* Parse and fill vreg params for powerdown settings*/
	rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_down_setting,
			s_ctrl->sensordata->power_info.power_down_setting_size);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PDOWN rc %d",
				rc);
	}

	CAM_WARN(CAM_SENSOR,
			"Probe Success,slot:%d,slave_addr:0x%x",
			s_ctrl->soc_info.index,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);

	/*
	 * Set probe succeeded flag to 1 so that no other camera shall
	 * probed on this slot
	 */
	s_ctrl->is_probe_succeed = 1;
	s_ctrl->sensor_state = CAM_SENSOR_INIT;

	return rc;
}

static int sensor_powerup(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	CAM_ERR(CAM_SENSOR, "sensor power up start");
	if ((s_ctrl->is_probe_succeed == 0) ||
			(s_ctrl->sensor_state != CAM_SENSOR_INIT)) {
		CAM_WARN(CAM_SENSOR,
				"Not in right state to powerup %d (%d)",
				s_ctrl->soc_info.index,
				s_ctrl->sensor_state);
		rc = -EINVAL;
	}

	CAM_WARN(CAM_SENSOR, "powering up %d", s_ctrl->soc_info.index);
	rc = cam_sensor_power_up(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power up failed");
	}

	s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
	CAM_INFO(CAM_SENSOR,
			"SENSOR_POWER_UP Success %d",
			s_ctrl->soc_info.index);
	return rc;
}

static int sensor_powerdown(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	CAM_ERR(CAM_SENSOR, "sensor power down start");
	if (s_ctrl->sensor_state == CAM_SENSOR_START) {
		rc = -EINVAL;
		CAM_WARN(CAM_SENSOR,
				"Not in right state to release %d (%d)",
				s_ctrl->soc_info.index,
				s_ctrl->sensor_state);
	}

	CAM_WARN(CAM_SENSOR, "powering down %d",
			s_ctrl->soc_info.index);
	rc = cam_sensor_power_down(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "power down failed");
	}

	s_ctrl->sensor_state = CAM_SENSOR_INIT;
	CAM_INFO(CAM_SENSOR,
			"SENSOR_POWER_DOWN Success %d",
			s_ctrl->soc_info.index);
	return rc;
}

static int i2c_write_array(struct cam_sensor_ctrl_t *s_ctrl, struct ais_sensor_cmd_i2c_wr_array *i2c_write)
{
	int32_t rc = 0;
	int i = 0;
	struct cam_sensor_i2c_reg_setting write_setting;
	struct cam_sensor_i2c_reg_array *reg_setting;
	struct cam_sensor_i2c_slave_info slave_info;

	if (s_ctrl->sensor_state != CAM_SENSOR_ACQUIRE) {
		CAM_WARN(CAM_SENSOR,
				"%d Not in right state to aquire %d",
				s_ctrl->soc_info.index,
				s_ctrl->sensor_state);
		rc = -EINVAL;
		goto exit;
	}

	if (!i2c_write->count ||
			i2c_write->count > CCI_I2C_MAX_WRITE) {
		CAM_ERR(CAM_SENSOR, "invalid i2c array size");
		rc = -EINVAL;
		goto exit;
	}

	reg_setting = kcalloc(i2c_write->count,
			(sizeof(struct cam_sensor_i2c_reg_array)),
			GFP_KERNEL);
	if (!reg_setting) {
		rc = -ENOMEM;
		goto exit;
	}

	write_setting.reg_setting = reg_setting;
	write_setting.size = i2c_write->count;
	write_setting.delay = 0;
	write_setting.addr_type = i2c_write->addr_type;
	write_setting.data_type = i2c_write->data_type;

	for (i = 0; i < i2c_write->count; i++) {
		reg_setting[i].reg_addr = i2c_write->wr_array[i].reg_addr;
		reg_setting[i].reg_data = i2c_write->wr_array[i].reg_data;
		reg_setting[i].delay = i2c_write->wr_array[i].delay;
	}

	slave_info.slave_addr = i2c_write->i2c_config.slave_addr;
	slave_info.i2c_freq_mode = i2c_write->i2c_config.i2c_freq_mode;
	rc = cam_sensor_update_i2c_slave_info(&(s_ctrl->io_master_info),
			&slave_info);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed to update slave info");
		kfree(reg_setting);
	}

	CAM_DBG(CAM_SENSOR,
			"Write 0x%x, %d regs [%d, %d]",
			i2c_write->i2c_config.slave_addr,
			i2c_write->count,
			i2c_write->addr_type, i2c_write->data_type);

	rc = camera_io_dev_write(&(s_ctrl->io_master_info),
			&write_setting);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR, "Failed to write array to 0x%x %d",
				slave_info.slave_addr,
				write_setting.size);

	(void)cam_sensor_restore_slave_info(s_ctrl);

	kfree(reg_setting);
exit:
	return rc;
}

static int init_sequence(struct cam_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	int32_t i = 0;
	struct ais_sensor_cmd_i2c_wr_array *i2c_write = i2c_write_sequence;

	for (i = 0; i < ARRAY_SIZE(i2c_write_sequence); i++) {
		rc = i2c_write_array(s_ctrl, &i2c_write[i]);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "init sequence %d", i);
			goto error;
		}
	}

error:
	return rc;
}

static int cam_sensor_serdes_init(void *data)
{
	int32_t rc = 0;
	char boot_marker[40];
	struct cam_bchip_ctrl_t *b_ctrl = (struct cam_bchip_ctrl_t *)data;

	snprintf(boot_marker, sizeof(boot_marker), "M - DRIVER BC%d init",
		b_ctrl->s_ctrl->soc_info.index);
	place_marker(boot_marker);
	rc = sensor_probe(b_ctrl->s_ctrl);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "Bridgechip%d Probe Failed",
			b_ctrl->s_ctrl->soc_info.index);
		goto error;
	}

	rc = sensor_powerup(b_ctrl->s_ctrl);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "Bridgechip%d Powerup Failed",
			b_ctrl->s_ctrl->soc_info.index);
		goto error;
	}

	init_sequence(b_ctrl->s_ctrl);

	rc = sensor_powerdown(b_ctrl->s_ctrl);
	if (rc) {
		CAM_ERR(CAM_SENSOR, "Bridgechip%d Powerdown Failed",
			b_ctrl->s_ctrl->soc_info.index);
		goto error;
	}

	snprintf(boot_marker, sizeof(boot_marker), "M - DRIVER BC%d init Ready",
		b_ctrl->s_ctrl->soc_info.index);
	place_marker(boot_marker);

	b_ctrl->is_probe_succeed = 1;

error:
	return rc;
}

static const struct of_device_id cam_sensor_serdes_dt_match[] = {
	{.compatible = "qcom,cam-serdes"},
	{}
};

static int32_t cam_sensor_serdes_platform_probe(struct platform_device *pdev)
{
	int index = 0;
	int rc = 0;
	struct cam_bchip_ctrl_t *b_ctrl;

	b_ctrl = devm_kzalloc(&pdev->dev, sizeof(struct cam_bchip_ctrl_t), GFP_KERNEL);
	if (!b_ctrl)
		return -ENOMEM;

	b_ctrl->s_ctrl = cam_sensor_get_subdevdata(index);

	b_ctrl->of_node = pdev->dev.of_node;
	b_ctrl->is_probe_succeed = 0;
	b_ctrl->pdev = pdev;

	rc = cam_sensor_serdes_init_subdev(b_ctrl);
	if (rc)
		goto err;

	b_ctrl->bchip_init_task = kthread_create(cam_sensor_serdes_init,
			b_ctrl, "bridgechip%d_init", index);
	if (!IS_ERR(b_ctrl->bchip_init_task)) {
		wake_up_process(b_ctrl->bchip_init_task);
		CAM_ERR(CAM_SENSOR, "starting Bridgechip%d thread", index);
	} else {
		CAM_ERR(CAM_SENSOR, "Couldn't start bridgechip%d thread",
				index);
	}

	platform_set_drvdata(pdev, b_ctrl);

err:
	return rc;
}

static int cam_sensor_serdes_platform_remove(struct platform_device *pdev)
{
	struct cam_bchip_ctrl_t *b_ctrl = platform_get_drvdata(pdev);
	if (!b_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	kthread_stop(b_ctrl->bchip_init_task);

        CAM_INFO(CAM_SENSOR, "platform remove invoked");
        mutex_lock(&(b_ctrl->cam_bchip_mutex));
        cam_serdes_shutdown(b_ctrl);
        mutex_unlock(&(b_ctrl->cam_bchip_mutex));
        cam_unregister_subdev(&(b_ctrl->v4l2_dev_str));

	v4l2_set_subdevdata(&b_ctrl->v4l2_dev_str.sd, NULL);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, b_ctrl);

	return 0;
}

MODULE_DEVICE_TABLE(of, cam_sensor_serdes_dt_match);

static struct platform_driver cam_sensor_serdes_platform_driver = {
	.probe = cam_sensor_serdes_platform_probe,
	.driver = {
		.name = "cam-serdes",
		.owner = THIS_MODULE,
		.of_match_table = cam_sensor_serdes_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_sensor_serdes_platform_remove,
};

static int __init cam_sensor_serdes_driver_init(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_sensor_serdes_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "platform_driver_register Failed: rc = %d",
			rc);
		return rc;
	}

	return rc;
}

static void __exit cam_sensor_serdes_driver_exit(void)
{
	platform_driver_unregister(&cam_sensor_serdes_platform_driver);
}

fs_initcall(cam_sensor_serdes_driver_init);
module_exit(cam_sensor_serdes_driver_exit);
MODULE_DESCRIPTION("Camera Sensor SerDes Driver");
MODULE_LICENSE("GPL v2");
