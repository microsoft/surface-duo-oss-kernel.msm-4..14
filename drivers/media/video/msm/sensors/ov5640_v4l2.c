#include "msm_sensor.h"
#include "ov5640_v4l2.h"
#define SENSOR_NAME "ov5640"
#define PLATFORM_DRIVER_NAME "msm_camera_ov5640"
#define ov5640_obj ov5640_##obj

DEFINE_MUTEX(ov5640_mut);
static struct msm_sensor_ctrl_t ov5640_s_ctrl;

static struct msm_camera_i2c_reg_conf ov5640_start_settings[] = {
	{0x3008, 0x02},
};

static struct msm_camera_i2c_reg_conf ov5640_stop_settings[] = {
	{0x3008, 0x42},
};

static struct msm_camera_i2c_reg_conf ov5640_groupon_settings[] = {
	{0x3212, 0x00},
};

static struct msm_camera_i2c_reg_conf ov5640_groupoff_settings[] = {
	{0x3212, 0x10},
	{0x3212, 0xA0},
};


static struct msm_camera_i2c_reg_conf ov5640_prev_settings[] = {
	{0x3103, 0x11},
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3103, 0x03},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x3034, 0x18},
	{0x3035, 0x11},
	{0x3036, 0x54},
	{0x3037, 0x13},
	{0x3108, 0x01},
	{0x3630, 0x36},
	{0x3631, 0x0e},
	{0x3632, 0xe2},
	{0x3633, 0x12},
	{0x3621, 0xe0},
	{0x3704, 0xa0},
	{0x3703, 0x5a},
	{0x3715, 0x78},
	{0x3717, 0x01},
	{0x370b, 0x60},
	{0x3705, 0x1a},
	{0x3905, 0x02},
	{0x3906, 0x10},
	{0x3901, 0x0a},
	{0x3731, 0x12},
	{0x3600, 0x08},
	{0x3601, 0x33},
	{0x302d, 0x60},
	{0x3620, 0x52},
	{0x371b, 0x20},
	{0x471c, 0x50},
	{0x3a13, 0x43},
	{0x3a18, 0x02},
	{0x3a19, 0x00},
	{0x3635, 0x13},
	{0x3636, 0x03},
	{0x3634, 0x40},
	{0x3622, 0x01},
	{0x3c01, 0x34},
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c06, 0x00},
	{0x3c07, 0x07},
	{0x3c08, 0x00},
	{0x3c09, 0x1c},
	{0x3c0a, 0x9c},
	{0x3c0b, 0x40},
	{0x3820, 0x41},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x4800, 0x24},
	{0x3800, 0x01},
	{0x3801, 0x50},
	{0x3802, 0x01},
	{0x3803, 0xb2},
	{0x3804, 0x08},
	{0x3805, 0xef},
	{0x3806, 0x05},
	{0x3807, 0xfa},
	{0x3808,0x07},
	{0x3809,0x80},
	{0x380a,0x04},
	{0x380b,0x38},
	{0x380c, 0x09},
	{0x380d, 0xc4},
	{0x380e, 0x04},
	{0x380f, 0x60},
	{0x3810, 0x00},
	{0x3811, 0x10},
	{0x3812, 0x00},
	{0x3813, 0x04},
	{0x3618, 0x04},
	{0x3612, 0x2b},
	{0x3708, 0x62},
	{0x3709, 0x12},
	{0x370c, 0x00},
	{0x3a02, 0x04},
	{0x3a03, 0x60},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0e, 0x03},
	{0x3a0d, 0x04},
	{0x3a14, 0x04},
	{0x3a15, 0x60},
	{0x4001, 0x02},
	{0x4004, 0x06},
	{0x3000, 0x00},
	{0x3002, 0x1c},
	{0x3004, 0xff},
	{0x3006, 0xc3},
	{0x300e, 0x45},
	{0x302e, 0x08},
	{0x4300, 0x30},
	{0x501f, 0x00},
	{0x4713, 0x02},
	{0x4407, 0x04},
	{0x440e, 0x00},
	{0x460b, 0x37},
	{0x460c, 0x20},
	{0x4837, 0x0a},
	{0x3824, 0x04},
	{0x5000, 0xa7},
	{0x5001, 0x83},
	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x09},
	{0x5187, 0x09},
	{0x5188, 0x09},
	{0x5189, 0x75},
	{0x518a, 0x54},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x3d},
	{0x518f, 0x56},
	{0x5190, 0x46},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},
	{0x5381, 0x1e},
	{0x5382, 0x5b},
	{0x5383, 0x08},
	{0x5384, 0x0a},
	{0x5385, 0x7e},
	{0x5386, 0x88},
	{0x5387, 0x7c},
	{0x5388, 0x6c},
	{0x5389, 0x10},
	{0x538a, 0x01},
	{0x538b, 0x98},
	{0x5300, 0x08},
	{0x5301, 0x30},
	{0x5302, 0x10},
	{0x5303, 0x00},
	{0x5304, 0x08},
	{0x5305, 0x30},
	{0x5306, 0x08},
	{0x5307, 0x16},
	{0x5309, 0x08},
	{0x530a, 0x30},
	{0x530b, 0x04},
	{0x530c, 0x06},
	{0x5480, 0x01},
	{0x5481, 0x08},
	{0x5482, 0x14},
	{0x5483, 0x28},
	{0x5484, 0x51},
	{0x5485, 0x65},
	{0x5486, 0x71},
	{0x5487, 0x7d},
	{0x5488, 0x87},
	{0x5489, 0x91},
	{0x548a, 0x9a},
	{0x548b, 0xaa},
	{0x548c, 0xb8},
	{0x548d, 0xcd},
	{0x548e, 0xdd},
	{0x548f, 0xea},
	{0x5490, 0x1d},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x40},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0xf8},
	{0x5800, 0x23},
	{0x5801, 0x14},
	{0x5802, 0x0f},
	{0x5803, 0x0f},
	{0x5804, 0x12},
	{0x5805, 0x26},
	{0x5806, 0x0c},
	{0x5807, 0x08},
	{0x5808, 0x05},
	{0x5809, 0x05},
	{0x580a, 0x08},
	{0x580b, 0x0d},
	{0x580c, 0x08},
	{0x580d, 0x03},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x03},
	{0x5811, 0x09},
	{0x5812, 0x07},
	{0x5813, 0x03},
	{0x5814, 0x00},
	{0x5815, 0x01},
	{0x5816, 0x03},
	{0x5817, 0x08},
	{0x5818, 0x0d},
	{0x5819, 0x08},
	{0x581a, 0x05},
	{0x581b, 0x06},
	{0x581c, 0x08},
	{0x581d, 0x0e},
	{0x581e, 0x29},
	{0x581f, 0x17},
	{0x5820, 0x11},
	{0x5821, 0x11},
	{0x5822, 0x15},
	{0x5823, 0x28},
	{0x5824, 0x46},
	{0x5825, 0x26},
	{0x5826, 0x08},
	{0x5827, 0x26},
	{0x5828, 0x64},
	{0x5829, 0x26},
	{0x582a, 0x24},
	{0x582b, 0x22},
	{0x582c, 0x24},
	{0x582d, 0x24},
	{0x582e, 0x06},
	{0x582f, 0x22},
	{0x5830, 0x40},
	{0x5831, 0x42},
	{0x5832, 0x24},
	{0x5833, 0x26},
	{0x5834, 0x24},
	{0x5835, 0x22},
	{0x5836, 0x22},
	{0x5837, 0x26},
	{0x5838, 0x44},
	{0x5839, 0x24},
	{0x583a, 0x26},
	{0x583b, 0x28},
	{0x583c, 0x42},
	{0x583d, 0xce},
	{0x5025, 0x00},
	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x26},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},

};
static struct msm_camera_i2c_reg_conf ov5640_recommend_settings[] = {

};

static struct v4l2_subdev_info ov5640_subdev_info[] = {
	{
	.code  = V4L2_MBUS_FMT_YUYV8_2X8,//V4L2_MBUS_FMT_YUYV8_1X16,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt   = 1,
	.order = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array ov5640_init_conf[] = {
	{&ov5640_recommend_settings[0],
	ARRAY_SIZE(ov5640_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}

};

static struct msm_camera_i2c_conf_array ov5640_confs[] = {
	{&ov5640_prev_settings[0],
	ARRAY_SIZE(ov5640_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},

};

static struct msm_sensor_output_info_t ov5640_dimensions[] = {
	{
		.x_output = 1920,
		.y_output = 1088,
		.line_length_pclk = 0x9c4,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 256000000,
		.op_pixel_clk = 256000000,
		.binning_factor = 1,
	},
	{
		.x_output = 1920,
		.y_output = 1088,
		.line_length_pclk = 0x9c4,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 256000000,
		.op_pixel_clk = 256000000,
		.binning_factor = 1,
	},

};

static struct msm_sensor_output_reg_addr_t ov5640_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov5640_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x5640,
};

static struct msm_sensor_exp_gain_info_t ov5640_exp_gain_info = {
	.coarse_int_time_addr = 0x3500,
	.global_gain_addr = 0x350A,
	.vert_offset = 4,
};

static int32_t ov5640_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		int_time[0], MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
		int_time[1], MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr+1,
		int_time[2], MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static const struct i2c_device_id ov5640_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov5640_s_ctrl},
	{ }
};

static struct i2c_driver ov5640_i2c_driver = {
	.id_table = ov5640_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov5640_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


static const struct of_device_id ov5640_dt_match[] = {
	{.compatible = "qcom,ov5640", .data = &ov5640_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov5640_dt_match);

static struct platform_driver ov5640_platform_driver = {
	.driver = {
		.name = "qcom,ov5640",
		.owner = THIS_MODULE,
		.of_match_table = ov5640_dt_match,
	},
};

static int32_t ov5640_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov5640_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init msm_sensor_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_probe(&ov5640_platform_driver,
		ov5640_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit msm_sensor_exit_module(void)
{
	if (ov5640_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5640_s_ctrl);
		platform_driver_unregister(&ov5640_platform_driver);
	} else
		i2c_del_driver(&ov5640_i2c_driver);
	return;
}

static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core = &ov5640_subdev_core_ops,
	.video  = &ov5640_subdev_video_ops,
};

static struct msm_sensor_fn_t ov5640_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov5640_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov5640_write_exp_gain,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines2,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t ov5640_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov5640_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov5640_start_settings),
	.stop_stream_conf = ov5640_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov5640_stop_settings),
	.group_hold_on_conf = ov5640_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov5640_groupon_settings),
	.group_hold_off_conf = ov5640_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(ov5640_groupoff_settings),
	.init_settings = &ov5640_init_conf[0],
	.init_size = ARRAY_SIZE(ov5640_init_conf),
	.mode_settings = &ov5640_confs[0],
	.output_settings = &ov5640_dimensions[0],
	.num_conf = ARRAY_SIZE(ov5640_confs),
};

static struct msm_sensor_ctrl_t ov5640_s_ctrl = {
	.msm_sensor_reg = &ov5640_regs,
	.sensor_i2c_client = &ov5640_sensor_i2c_client,
	.sensor_i2c_addr = 0x78,
	.sensor_output_reg_addr = &ov5640_reg_addr,
	.sensor_id_info = &ov5640_id_info,
	.sensor_exp_gain_info = &ov5640_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &ov5640_mut,
	.sensor_i2c_driver = &ov5640_i2c_driver,
	.sensor_v4l2_subdev_info = ov5640_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov5640_subdev_info),
	.sensor_v4l2_subdev_ops = &ov5640_subdev_ops,
	.func_tbl = &ov5640_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
module_exit(msm_sensor_exit_module);
MODULE_DESCRIPTION("Omnivision 5MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
