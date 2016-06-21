/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5x06_ts.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/device.h>
#define FT5X0X_DEBUG    1
/*#define SUPPORT_MULTI_TOUCH  1 */

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	/* touch event:
	 * 0 -- down;
	 * 1-- contact;
	 * 2 -- contact
	 */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ft5x0x_platform_data *pdata;
#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
#endif
};

#define FTS_POINT_UP		0x01
#define FTS_POINT_DOWN		0x00
#define FTS_POINT_CONTACT	0x02

#ifdef CONFIG_DRM_ICN_6201
#define FT5X06_X_MAX	1024
#define FT5X06_Y_MAX	600
#else
#define FT5X06_X_MAX	800
#define FT5X06_Y_MAX	1280
#endif


/*
*ft5x0x_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);
	return ret;
}

/*release the point*/
static void ft5x0x_ts_release(struct ft5x0x_ts_data *data)
{
#ifdef SUPPORT_MULTI_TOUCH
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
#endif
	input_sync(data->input_dev);
}

/*
*report the point information
*/
static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i = 0;

	for (i = 0; i < event->touch_point; i++) {
		/* LCD view area */
#ifdef SUPPORT_MULTI_TOUCH
		if (event->au16_x[i] < data->x_max
		    && event->au16_y[i] < data->y_max) {
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 event->au16_y[i]);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					 event->au8_finger_id[i]);
			if (event->au8_touch_event[i] == FTS_POINT_DOWN
			|| event->au8_touch_event[i] == FTS_POINT_CONTACT) {
				input_report_abs(data->input_dev,
					ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev,
					ABS_MT_PRESSURE, event->pressure);
				input_report_key(data->input_dev,
						BTN_TOUCH, 1);
			} else {
				input_report_abs(data->input_dev,
						ABS_MT_TOUCH_MAJOR, 0);
				input_report_abs(data->input_dev,
						ABS_MT_PRESSURE, 0);
				input_report_key(data->input_dev,
						BTN_TOUCH, 0);
			}
		}
		input_mt_sync(data->input_dev);
#else
		if (event->au16_x[i] < data->x_max
		    && event->au16_y[i] < data->y_max) {
			input_report_abs(data->input_dev, ABS_X,
					event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_Y,
					event->au16_y[i]);
			if (event->au8_touch_event[i] == FTS_POINT_DOWN
			|| event->au8_touch_event[i] == FTS_POINT_CONTACT) {
				input_report_abs(data->input_dev,
						ABS_PRESSURE, 1);
				input_report_key(data->input_dev,
						BTN_TOUCH, 1);
			} else {
				input_report_abs(data->input_dev,
						ABS_PRESSURE, 0);
				input_report_key(data->input_dev,
						BTN_TOUCH, 0);
			}
		}

		input_sync(data->input_dev);
#endif
	}
	input_sync(data->input_dev);

	if (event->touch_point == 0)
		ft5x0x_ts_release(data);

}

/*Read touch point information when the interrupt  is asserted.*/
static int ft5x0x_read_Touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
		    (s16)(buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F)
			 << 8 | (s16)buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
		    (s16)(buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F)
			 << 8 | (s16)buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}

	event->pressure = FT_PRESS;

	return 0;
}


/*The ft5x0x device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	int ret = 0;

	ret = ft5x0x_read_Touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(ft5x0x_ts);

	return IRQ_HANDLED;
}

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;
	/*unsigned int irq_no;*/
	struct device *dev = &client->dev;
	struct gpio_desc *gpio_rst;


	gpio_rst = devm_gpiod_get_optional(dev, "rst", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_rst))
		return PTR_ERR(gpio_rst);

	if (gpio_rst) {
		mdelay(5);
		gpiod_set_value_cansleep(gpio_rst, 0);
		mdelay(5);
		gpiod_set_value_cansleep(gpio_rst, 1);
		mdelay(50);
		gpiod_set_value_cansleep(gpio_rst, 0);

	}


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = devm_kzalloc(dev, sizeof(*ft5x0x_ts), GFP_KERNEL);

	if (!ft5x0x_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ft5x0x_ts);

	ft5x0x_ts->irq = client->irq;
	ft5x0x_ts->client = client;

	ft5x0x_ts->x_max = FT5X06_X_MAX;
	ft5x0x_ts->y_max = FT5X06_Y_MAX;

	if (client->irq) {
		err = devm_request_threaded_irq(dev, client->irq, NULL,
						ft5x0x_ts_interrupt,
						IRQF_ONESHOT, dev_name(dev),
						ft5x0x_ts);
		if (err)
			goto exit_irq_request_failed;
	}


	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

#ifdef SUPPORT_MULTI_TOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_PRESSURE, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
				ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
				ft5x0x_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
				PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0,
				PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
				CFG_MAX_TOUCH_POINTS, 0, 0);
#else
	set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_X, 0, FT5X06_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, FT5X06_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

#endif

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	input_dev->name = FT5X0X_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: failed to register: %s\n", __func__,
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

	/*make sure CTP already finish startup process */
	msleep(150);


	/*get some register information */
	uc_reg_addr = FT5x0x_REG_FW_VER;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] Firmware version = 0x%x\n", uc_reg_value);

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] report rate is %dHz.\n",
		uc_reg_value * 10);

	uc_reg_addr = FT5X0X_REG_THGROUP;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_dbg(&client->dev, "[FTS] touch threshold is %d.\n",
		uc_reg_value * 4);

#ifdef FT5X0X_DEBUG
	dev_dbg(&client->dev, "ft5x0x_ts_probe succssss!\n");
#endif

	/*enable_irq(irq_no);*/
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	/*free_irq(client->irq, ft5x0x_ts);*/

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	/*kfree(ft5x0x_ts);*/

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}


static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts;

	ft5x0x_ts = i2c_get_clientdata(client);
	input_unregister_device(ft5x0x_ts->input_dev);
	#ifdef CONFIG_PM
	/*gpio_free(ft5x0x_ts->pdata->reset);*/
	#endif
	/*free_irq(client->irq, ft5x0x_ts);*/
	/*kfree(ft5x0x_ts);*/
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{FT5X0X_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static const struct of_device_id lemaker_tp_of_match[] = {
	{ .compatible = "FocalTech,ft5506", },
	{ }
};
MODULE_DEVICE_TABLE(of, lemaker_tp_of_match);


static struct i2c_driver ft5x0x_ts_driver = {
	.probe = ft5x0x_ts_probe,
	.remove = ft5x0x_ts_remove,
	.id_table = ft5x0x_ts_id,
	.driver = {
		   .name = FT5X0X_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = lemaker_tp_of_match,
		   },
};

static int __init ft5x0x_ts_init(void)
{
	int ret;

	ret = i2c_add_driver(&ft5x0x_ts_driver);
	if (ret) {
		pr_err("Adding ft5x0x driver failed errno = %d\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ft5x0x_ts_driver.driver.name);
	}

	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}


module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<luowj>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL v2");
