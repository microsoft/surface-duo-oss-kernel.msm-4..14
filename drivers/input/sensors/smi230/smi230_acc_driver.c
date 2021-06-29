// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include "smi230_driver.h"
#include "smi230_data_sync.h"

#define MODULE_TAG MODULE_NAME
#include "smi230_log.h"
#include "smi230.h"

#define SMI230_ACC_ENABLE_INT2 1
#define SMI230_MIN_VALUE      -32768
#define SMI230_MAX_VALUE      32767

#ifdef CONFIG_SMI230_ACC_FIFO
#define SMI230_MAX_ACC_FIFO_BYTES 1024
/*1024 bytes fifo host max 147 data frames*/
#define SMI230_MAX_ACC_FIFO_FRAME 147

static uint8_t fifo_buf[SMI230_MAX_ACC_FIFO_BYTES];
#endif


struct smi230_client_data {
	struct device *dev;
	struct input_dev *input;
	int IRQ;
	uint8_t gpio_pin;
	struct work_struct irq_work;
};

static struct smi230_dev *p_smi230_dev;

static ssize_t smi230_acc_reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;

	for (i = 0; i <= 0x7e; i++) {
		err = smi230_acc_get_regs(i, &data, 1, p_smi230_dev);
		if (err) {
			PERR("falied");
			return err;
		}
		printk("0x%x = 0x%x", i, data);
		if ( i % 15 == 0 )
			printk("\n");
	}

	return 0;
}

static ssize_t smi230_acc_show_chip_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t chip_id[2] = {0};
	int err = 0;

	err = smi230_acc_get_regs(SMI230_ACCEL_CHIP_ID_REG, chip_id, 2, p_smi230_dev);
	if (err) {
		PERR("falied");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "chip_id=0x%x rev_id=0x%x\n",
		chip_id[0], chip_id[1]);
}

static ssize_t smi230_acc_show_fifo_wm(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	uint16_t fifo_wm;

	err = smi230_acc_get_fifo_wm(&fifo_wm, p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "fifo water mark is %u\n", fifo_wm);
}

static ssize_t smi230_acc_store_fifo_wm(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	uint16_t fifo_wm;

	err = kstrtou16(buf, 10, &fifo_wm);
	err |= smi230_acc_set_fifo_wm(fifo_wm, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("set fifo wm faild");
		return err;
	}

	PDEBUG("set fifo wm to %u", fifo_wm);

	return count;
}

static ssize_t smi230_acc_show_acc_pwr_cfg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	err = smi230_acc_get_power_mode(p_smi230_dev);
	if (err) {
		PERR("read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE, "%x (0:active 3:suspend)\n", p_smi230_dev->accel_cfg.power);
}

static ssize_t smi230_acc_store_acc_pwr_cfg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	unsigned long pwr_cfg;

	err = kstrtoul(buf, 10, &pwr_cfg);
	if (err)
		return err;
	if (pwr_cfg == 3) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
		err = smi230_acc_set_power_mode(p_smi230_dev);
	}
	else if (pwr_cfg == 0) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
		err = smi230_acc_set_power_mode(p_smi230_dev);
	}

	PDEBUG("set power cfg to %ld, err %d", pwr_cfg, err);

	if (err) {
		PERR("failed");
		return err;
	}
	return count;
}

static ssize_t smi230_acc_show_acc_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_sensor_data data = {0};

	err = smi230_acc_get_data(&data, p_smi230_dev);
	if (err < 0)
		return err;
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n",
			data.x, data.y, data.z);
}

static ssize_t smi230_acc_show_driver_version(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE,
		"Driver version: %s\n", DRIVER_VERSION);
}

#ifdef CONFIG_SMI230_DATA_SYNC
static ssize_t smi230_acc_show_sync_data(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;

	err = smi230_get_synchronized_data(&accel_data, &gyro_data, p_smi230_dev);
	if (err != SMI230_OK)
		return err;

	return snprintf(buf, PAGE_SIZE, "acc: %hd %hd %hd gyro: %hd %hd %hd\n",
			accel_data.x, accel_data.y, accel_data.z,
			gyro_data.x, gyro_data.y, gyro_data.z
			);
}

static ssize_t smi230_acc_store_datasync_odr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0, odr;
	struct smi230_data_sync_cfg sync_cfg;

	err = kstrtoint(buf, 10, &odr);
	if (err) {
		PERR("invalid params");
		return err;
	}

	switch(odr) {
	case 2000:
		sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
		break;
	case 1000:
		sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_1000HZ;
		break;
	case 400:
		sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_400HZ;
		break;
	default:
		PERR("ODR not supported");
		return count;
	}

	err = smi230_configure_data_synchronization(sync_cfg, p_smi230_dev);

	PDEBUG("set ODR to %d, err %d", odr, err);

	if (err) {
		PERR("setting datasync ODR failed");
		return err;
	}
	return count;
}
#else
static ssize_t smi230_acc_show_odr(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err, odr = 0;

        err = smi230_acc_get_meas_conf(p_smi230_dev);
	if (err) {
		PERR("read ODR failed");
		return err;
	}

	switch(p_smi230_dev->accel_cfg.odr) {
	case SMI230_ACCEL_ODR_12_5_HZ:
		odr = 12;
		break;
	case SMI230_ACCEL_ODR_25_HZ:
		odr = 25;
		break;
	case SMI230_ACCEL_ODR_50_HZ:
		odr = 50;
		break;
	case SMI230_ACCEL_ODR_100_HZ:
		odr = 100;
		break;
	case SMI230_ACCEL_ODR_200_HZ:
		odr = 200;
		break;
	case SMI230_ACCEL_ODR_400_HZ:
		odr = 400;
		break;
	case SMI230_ACCEL_ODR_800_HZ:
		odr = 800;
		break;
	case SMI230_ACCEL_ODR_1600_HZ:
		odr = 1600;
		break;
	default:
		PERR("Wrong ODR read");
	}
	if(odr == 12)
		return snprintf(buf, PAGE_SIZE, "%s\n", "12.5");
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", odr);
}

static ssize_t smi230_acc_store_odr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	int odr;

	err = kstrtoint(buf, 10, &odr);
	if (err) {
		PERR("invalid params");
		return err;
	}

	switch(odr) {
	case 12:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_12_5_HZ;
		break;
	case 25:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_25_HZ;
		break;
	case 50:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_50_HZ;
		break;
	case 100:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
		break;
	case 200:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_200_HZ;
		break;
	case 400:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
		break;
	case 800:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
		break;
	case 1600:
		p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
		break;
	default:
		PERR("ODR not supported");
		return count;
	}

        err |= smi230_acc_set_meas_conf(p_smi230_dev);

	PDEBUG("set ODR to %d, err %d", odr, err);

	if (err) {
		PERR("setting ODR failed");
		return err;
	}
	return count;
}
#endif

static ssize_t smi230_acc_show_bw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;

        err = smi230_acc_get_meas_conf(p_smi230_dev);
	if (err) {
		PERR("read BW failed");
		return err;
	}

	switch(p_smi230_dev->accel_cfg.bw) {
	case SMI230_ACCEL_BW_OSR2:
		return snprintf(buf, PAGE_SIZE, "%s\n", "osr2");
	case SMI230_ACCEL_BW_OSR4:
		return snprintf(buf, PAGE_SIZE, "%s\n", "osr4");
	case SMI230_ACCEL_BW_NORMAL:
		return snprintf(buf, PAGE_SIZE, "%s\n", "normal");
	default:
		return snprintf(buf, PAGE_SIZE, "%s\n", "error");
	}
}

static ssize_t smi230_acc_store_bw(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;

	if(strncmp(buf, "normal", 6) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	else if(strncmp(buf, "osr2", 4) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR2;
	else if(strncmp(buf, "osr4", 4) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR4;
	else{
		PERR("invalid params");
		return count;
	}

        err |= smi230_acc_set_meas_conf(p_smi230_dev);

	PDEBUG("set bw to %s, err %d", buf, err);

	if (err) {
		PERR("setting BW failed");
		return err;
	}
	return count;
}

static ssize_t smi230_acc_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err, range = 0;

        err = smi230_acc_get_meas_conf(p_smi230_dev);
	if (err) {
		PERR("read range failed");
		return err;
	}

	switch(p_smi230_dev->accel_cfg.range) {
	case SMI230_ACCEL_RANGE_2G:
		range = 2;
		break;
	case SMI230_ACCEL_RANGE_4G:
		range = 4;
		break;
	case SMI230_ACCEL_RANGE_8G:
		range = 8;
		break;
	case SMI230_ACCEL_RANGE_16G:
		range = 16;
		break;
	default:
		PERR("wrong range read");
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", range);
}

static ssize_t smi230_acc_store_range(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0, range;

	err = kstrtoint(buf, 10, &range);
	if (err) {
		PERR("invalid params");
		return err;
	}

	switch(range) {
	case 2:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_2G;
		break;
	case 4:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;
		break;
	case 8:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_8G;
		break;
	case 16:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;
		break;
	default:
		PERR("range not supported");
		return count;
	}

        err |= smi230_acc_set_meas_conf(p_smi230_dev);

	PDEBUG("set range to %d, err %d", range, err);

	if (err) {
		PERR("setting range failed");
		return err;
	}
	return count;
}


static ssize_t smi230_acc_show_sensor_temperature(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int err;
	int32_t sensor_temp;

	err = smi230_acc_get_sensor_temperature(p_smi230_dev, &sensor_temp);
	if (err != SMI230_OK)
		return err;

	return snprintf(buf, PAGE_SIZE, "acc temperature: %d\n", sensor_temp);
}

static ssize_t smi230_acc_store_fifo_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;

	if(strncmp(buf, "reset", 5) == 0)
		err = smi230_acc_fifo_reset(p_smi230_dev);
	else {
		PDEBUG("error");
		return count;
	}

	if (err) {
		PERR("failed");
		return err;
	}
	PDEBUG("fifo reseted");

	return count;
}

static DEVICE_ATTR(regs_dump, S_IRUGO,
	smi230_acc_reg_dump, NULL);
static DEVICE_ATTR(chip_id, S_IRUGO,
	smi230_acc_show_chip_id, NULL);
static DEVICE_ATTR(fifo_wm, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_fifo_wm, smi230_acc_store_fifo_wm);
static DEVICE_ATTR(pwr_cfg, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_acc_pwr_cfg, smi230_acc_store_acc_pwr_cfg);
static DEVICE_ATTR(bw, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_bw, smi230_acc_store_bw);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_range, smi230_acc_store_range);
#ifdef CONFIG_SMI230_DATA_SYNC
static DEVICE_ATTR(data_sync, S_IRUGO,
	smi230_acc_show_sync_data, NULL);
static DEVICE_ATTR(datasync_odr, S_IWUSR|S_IWGRP,
	NULL, smi230_acc_store_datasync_odr);
#else
static DEVICE_ATTR(odr, S_IRUGO|S_IWUSR|S_IWGRP,
	smi230_acc_show_odr, smi230_acc_store_odr);
#endif
static DEVICE_ATTR(acc_value, S_IRUGO,
	smi230_acc_show_acc_value, NULL);
static DEVICE_ATTR(fifo_reset, S_IWUSR|S_IWGRP,
	NULL, smi230_acc_store_fifo_reset);
static DEVICE_ATTR(temp, S_IRUGO,
	smi230_acc_show_sensor_temperature, NULL);
static DEVICE_ATTR(driver_version, S_IRUGO,
	smi230_acc_show_driver_version, NULL);

static struct attribute *smi230_attributes[] = {
	&dev_attr_regs_dump.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_fifo_wm.attr,
	&dev_attr_pwr_cfg.attr,
	&dev_attr_bw.attr,
	&dev_attr_range.attr,
#ifdef CONFIG_SMI230_DATA_SYNC
	&dev_attr_data_sync.attr,
	&dev_attr_datasync_odr.attr,
#else
	&dev_attr_odr.attr,
#endif
	&dev_attr_acc_value.attr,
	&dev_attr_fifo_reset.attr,
	&dev_attr_temp.attr,
	&dev_attr_driver_version.attr,
	NULL
};

static struct attribute_group smi230_attribute_group = {
	.attrs = smi230_attributes
};

static int smi230_input_init(struct smi230_client_data *client_data)
{
	int err = 0;
	struct input_dev *dev = input_allocate_device();

	if (dev == NULL)
		return -ENOMEM;

	dev->id.bustype = BUS_I2C;
	dev->name = SENSOR_ACC_NAME;
	dev_set_name(&dev->dev, SENSOR_ACC_NAME);
	input_set_drvdata(dev, client_data);
	client_data->input = dev;

	input_set_capability(dev, EV_MSC, MSC_RAW);
	input_set_capability(dev, EV_MSC, MSC_GESTURE);
	input_set_abs_params(dev, ABS_X, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Y, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);
	input_set_abs_params(dev, ABS_Z, SMI230_MIN_VALUE, SMI230_MAX_VALUE, 0, 0);

	err = input_register_device(dev);
	if (err)
		input_free_device(dev);
	return err;
}

#ifdef CONFIG_SMI230_DATA_SYNC
static void smi230_data_sync_ready_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_sensor_data accel_data;
	struct smi230_sensor_data gyro_data;
	struct smi230_sensor_data accel_sync;
	int err = 0;

	err = smi230_acc_get_data(&accel_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;
	err = smi230_get_synchronized_data(&accel_sync, &gyro_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;

	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_sync.z);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)gyro_data.z);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.z);

	input_sync(client_data->input);
}
#endif

#ifdef CONFIG_SMI230_ACC_FIFO
static struct smi230_sensor_data fifo_accel_data[SMI230_MAX_ACC_FIFO_FRAME];

static void smi230_acc_fifo_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_fifo_frame fifo;
	int err = 0, i;
	uint16_t fifo_length;

	err = smi230_acc_get_fifo_length(&fifo.length, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO get length error!");
		return;
	}

#ifdef CONFIG_SMI230_DEBUG
	PINFO("ACC FIFO length %d", fifo.length);
#endif

	fifo.data = fifo_buf;
	err = smi230_acc_read_fifo_data(&fifo, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("FIFO read data error!");
		return;
	}

#ifdef CONFIG_SMI230_DEBUG
	PINFO("====================");
	PINFO("ACC FIFO data %d", fifo.data[0]);
	PINFO("ACC FIFO data %d", fifo.data[1]);
	PINFO("ACC FIFO data %d", fifo.data[2]);
	PINFO("ACC FIFO data %d", fifo.data[3]);
	PINFO("ACC FIFO data %d", fifo.data[4]);
	PINFO("ACC FIFO data %d", fifo.data[5]);
	PINFO("ACC FIFO data %d", fifo.data[6]);
	PINFO("--------------------");
#endif

#if 0
	/* this event shall never be mixed with sensor data  */
	/* this event here is to indicate IRQ timing if needed */
	input_event(client_data->input, EV_MSC, MSC_RAW, (int)fifo.length);
	input_sync(client_data->input);
#endif

	/* make sure all frames are read out,
	 * the actual frame numbers will be returned
	 * through fifo_length itself*/
	fifo_length = SMI230_MAX_ACC_FIFO_FRAME;
	err = smi230_acc_extract_accel(fifo_accel_data,
                            &fifo_length,
                            &fifo,
                            p_smi230_dev);

	for (i = 0; i < fifo_length; i++) {
		input_event(client_data->input, EV_ABS, ABS_X, (int)fifo_accel_data[i].x);
		input_event(client_data->input, EV_ABS, ABS_Y, (int)fifo_accel_data[i].y);
		input_event(client_data->input, EV_ABS, ABS_Z, (int)fifo_accel_data[i].z);
		input_sync(client_data->input);
	}
}

#else /* new data */

static void smi230_new_data_ready_handle(
	struct smi230_client_data *client_data)
{
	struct smi230_sensor_data accel_data;
	int err = 0;

	err = smi230_acc_get_data(&accel_data, p_smi230_dev);
	if (err != SMI230_OK)
		return;

	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.x);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.y);
	input_event(client_data->input, EV_MSC, MSC_GESTURE, (int)accel_data.z);

	input_sync(client_data->input);
}
#endif

static void smi230_irq_work_func(struct work_struct *work)
{
	struct smi230_client_data *client_data =
		container_of(work, struct smi230_client_data, irq_work);

#ifdef CONFIG_SMI230_ACC_FIFO
	smi230_acc_fifo_handle(client_data);
#else
	smi230_new_data_ready_handle(client_data);
#endif

#ifdef CONFIG_SMI230_DATA_SYNC
	smi230_data_sync_ready_handle(client_data);
#endif
}

static irqreturn_t smi230_irq_handle(int irq, void *handle)
{
	struct smi230_client_data *client_data = handle;
	int err = 0;

	err = schedule_work(&client_data->irq_work);
	if (err < 0)
		PERR("schedule_work failed\n");

	return IRQ_HANDLED;
}

static void smi230_free_irq(struct smi230_client_data *client_data)
{
	cancel_work_sync(&client_data->irq_work);
	free_irq(client_data->IRQ, client_data);
	gpio_free(client_data->gpio_pin);
}

static int smi230_request_irq(struct smi230_client_data *client_data)
{
	int err = 0;

	INIT_WORK(&client_data->irq_work, smi230_irq_work_func);

	client_data->gpio_pin = of_get_named_gpio_flags(
		client_data->dev->of_node,
		"gpio_irq", 0, NULL);
	PINFO("SMI230_ACC gpio number:%d\n", client_data->gpio_pin);
	err = gpio_request_one(client_data->gpio_pin,
				GPIOF_IN, "smi230_acc_interrupt");
	if (err < 0) {
		PDEBUG("gpio_request_one\n");
		return err;
	}
	err = gpio_direction_input(client_data->gpio_pin);
	if (err < 0) {
		PDEBUG("gpio_direction_input\n");
		return err;
	}
	client_data->IRQ = gpio_to_irq(client_data->gpio_pin);
	err = request_irq(client_data->IRQ, smi230_irq_handle,
			IRQF_TRIGGER_RISING,
			SENSOR_ACC_NAME, client_data);
	if (err < 0) {
		PDEBUG("request_irq\n");
		return err;
	}
	return err;
}

static void smi230_input_destroy(struct smi230_client_data *client_data)
{
	struct input_dev *dev = client_data->input;
	input_unregister_device(dev);
	/* to avoid underflow of refcount, do a checck before call free device*/
	if (dev->devres_managed)
		input_free_device(dev);
}

int smi230_acc_remove(struct device *dev)
{
	int err = 0;
	struct smi230_client_data *client_data = dev_get_drvdata(dev);

	if (NULL != client_data) {
		smi230_free_irq(client_data);
		sysfs_remove_group(&client_data->input->dev.kobj,
				&smi230_attribute_group);
		smi230_input_destroy(client_data);
		kfree(client_data);
	}
	return err;
}

int smi230_acc_probe(struct device *dev, struct smi230_dev *smi230_dev)
{
	int err = 0;
	struct smi230_client_data *client_data = NULL;
#ifdef CONFIG_SMI230_DATA_SYNC
	struct smi230_data_sync_cfg sync_cfg;
#endif
#ifdef CONFIG_SMI230_ACC_FIFO
	struct accel_fifo_config fifo_config;
#endif
	struct smi230_int_cfg int_config;

	if (dev == NULL || smi230_dev == NULL)
		return -EINVAL;

	p_smi230_dev = smi230_dev;

	client_data = kzalloc(sizeof(struct smi230_client_data),
						GFP_KERNEL);
	if (NULL == client_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_directly;
	}

	client_data->dev = dev;

	/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
	err |= smi230_acc_soft_reset(p_smi230_dev);

	/*! Max read/write length (maximum supported length is 32).
	 To be set by the user */
	/*set accel power mode */
	p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	err |= smi230_acc_set_power_mode(p_smi230_dev);

#ifdef CONFIG_SMI230_ACC_INT1
	/*configure int1 as accel FIFO interrupt pin */
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	/*disable acc int2*/
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
#endif

#ifdef CONFIG_SMI230_ACC_INT2
	/*disable acc int1*/
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;

	/*configure int2 as accel FIFO interrupt pin */
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;
#endif

	p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
	p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;

        err |= smi230_acc_set_meas_conf(p_smi230_dev);
	smi230_delay(100);

#ifdef CONFIG_SMI230_ACC_FIFO
	PINFO("ACC FIFO is enabled");

	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;

	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
#ifdef CONFIG_SMI230_ACC_FIFO_WM
	PINFO("ACC FIFO watermark is enabled");
	int_config.accel_int_config_1.int_type = SMI230_FIFO_WM_INT;
	int_config.accel_int_config_2.int_type = SMI230_FIFO_WM_INT;

	err |= smi230_acc_set_fifo_wm(91, p_smi230_dev);
#endif
#ifdef CONFIG_SMI230_ACC_FIFO_FULL
	PINFO("ACC FIFO full is enabled");
	int_config.accel_int_config_2.int_type = SMI230_FIFO_FULL_INT;
#endif

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_2, p_smi230_dev);

	fifo_config.mode = SMI230_ACC_FIFO_MODE;
	fifo_config.accel_en = 1;
	err |= smi230_acc_set_fifo_config(&fifo_config, p_smi230_dev);

	if (err != SMI230_OK)
	{
		PERR("FIFO HW init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);

#else /* new data */

	PINFO("ACC new data is enabled");

#ifdef CONFIG_SMI230_ACC_INT1
	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = SMI230_ACCEL_DATA_RDY_INT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_1, p_smi230_dev);
#endif
#ifdef CONFIG_SMI230_ACC_INT2
	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_ACCEL_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_2, p_smi230_dev);
#endif

	if (err != SMI230_OK)
	{
		PERR("ACC new data init failed");
		goto exit_free_client_data;
	}

	smi230_delay(100);

#endif

#ifdef CONFIG_SMI230_DATA_SYNC
	PINFO("DATA sync is enabled");
	/* API uploads the smi230 config file onto the device and wait for 150ms 
	   to enable the data synchronization - delay taken care inside the function */
	err |= smi230_apply_config_file(p_smi230_dev);
        if (err == SMI230_OK)
		PINFO("Configuration file transfer succeed!");

	else {
		PERR("Configuration file transfer error!");
		goto exit_free_client_data;
	}

	/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
	sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
	err |= smi230_configure_data_synchronization(sync_cfg, p_smi230_dev);

	/*set accel interrupt pin configuration*/
	/*configure host data ready interrupt */
	int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
	int_config.accel_int_config_1.int_type = SMI230_ACCEL_SYNC_INPUT;
	int_config.accel_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	/*configure Accel syncronization input interrupt pin */
	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_ACCEL_SYNC_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	/*set gyro interrupt pin configuration*/
	int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = SMI230_ENABLE;
	int_config.gyro_int_config_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;

	/* Enable synchronization interrupt pin */
	err |= smi230_set_data_sync_int_config(&int_config, p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("Data sync HW init failed");
		goto exit_free_client_data;
	}
#endif

#ifndef CONFIG_SMI230_DATA_SYNC
#ifndef CONFIG_SMI230_DEBUG
	/*if not for the convenience of debuging, sensors should be disabled at startup*/
	p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	err |= smi230_acc_set_power_mode(p_smi230_dev);
	if (err != SMI230_OK) {
		PERR("set power mode failed");
		goto exit_free_client_data;
	}
#endif
#endif

	/*acc input device init */
	err = smi230_input_init(client_data);
	if (err < 0) {
		PERR("input init failed");
		goto exit_free_client_data;
	}

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
			&smi230_attribute_group);
	if (err < 0) {
		PERR("sysfs create failed");
		goto exit_cleanup_input;
	}

	/*request irq and config*/
	err = smi230_request_irq(client_data);
	if (err < 0) {
		PERR("Request irq failed");
		goto exit_cleanup_sysfs;
	}

	PINFO("Sensor %s was probed successfully", SENSOR_ACC_NAME);

	return 0;
exit_cleanup_sysfs:
	sysfs_remove_group(&client_data->input->dev.kobj,
		&smi230_attribute_group);
exit_cleanup_input:
	smi230_input_destroy(client_data);
exit_free_client_data:
	if (client_data != NULL)
		kfree(client_data);
exit_directly:
	return err;
}
