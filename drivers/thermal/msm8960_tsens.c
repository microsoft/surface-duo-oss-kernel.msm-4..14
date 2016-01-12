/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/of.h>

#define TSENS_CNTL_ADDR				0x3620
#define TSENS_EN				BIT(0)
#define TSENS_SW_RST				BIT(1)
#define TSENS_ADC_CLK_SEL			BIT(2)
#define SENSOR0_EN				BIT(3)
#define TSENS_SENSOR0_SHIFT			3
#define TSENS_MASK1					1
#define SENSORS_EN_MASK(n)			GENMASK((n + 2), 3)

#define TSENS_TH_ADDR				0x3624
#define TSENS_TH_MAX_LIMIT_SHIFT		24
#define TSENS_TH_MIN_LIMIT_SHIFT		16
#define TSENS_TH_UPPER_LIMIT_SHIFT		8
#define TSENS_TH_LOWER_LIMIT_SHIFT		0
#define TSENS_TH_MAX_LIMIT_MASK			GENMASK(31, 24)
#define TSENS_TH_MIN_LIMIT_MASK			GENMASK(23, 16)
#define TSENS_TH_UPPER_LIMIT_MASK		GENMASK(15, 8)
#define TSENS_TH_LOWER_LIMIT_MASK		GENMASK(7, 0)

#define TSENS_S0_STATUS_ADDR			0x3628
#define TSENS_STATUS_ADDR_OFFSET		2

#define TSENS_INT_STATUS_ADDR			0x363c
#define TSENS_LOWER_INT_MASK			BIT(1)
#define TSENS_UPPER_INT_MASK			BIT(2)
#define TSENS_MAX_INT_MASK			BIT(3)
#define TSENS_TRDY_MASK				BIT(7)

#define TSENS_TH_MAX_CODE			0xff
#define TSENS_TH_MIN_CODE			0
#define TSENS_TRDY_RDY_MIN_TIME			1000
#define TSENS_TRDY_RDY_MAX_TIME			1100

#define TSENS_MEASURE_PERIOD				1
/* Initial temperature threshold values */
#define TSENS_LOWER_LIMIT_TH			0x50
#define TSENS_UPPER_LIMIT_TH			0xdf
#define TSENS_MIN_LIMIT_TH			0x0
#define TSENS_MAX_LIMIT_TH			0xff

#define TSENS_MIN_STATUS_MASK(offset)		BIT((offset))
#define TSENS_LOWER_STATUS_CLR(offset)		BIT((offset + 1))
#define TSENS_UPPER_STATUS_CLR(offset)		BIT((offset + 2))
#define TSENS_MAX_STATUS_MASK(offset)		BIT((offset + 3))

/* QFPROM addresses */
#define TSENS_8960_QFPROM_ADDR0			0x0
#define TSENS_8960_QFPROM_SPARE_OFFSET		0x10
/* 8960 Specifics */
#define TSENS_8960_CONFIG			0x9b
#define TSENS_8960_CONFIG_MASK			GENMASK(7, 0)
#define TSENS_8960_CONFIG_ADDR			0x3640
#define TSENS_8064_STATUS_CNTL			0x3660
#define TSENS_8064_SEQ_SENSORS			5
#define TSENS_8064_S4_S5_OFFSET			40

#define TSENS_CAL_MILLI_DEGC			30000
#define TSENS_MAX_SENSORS			11

/* Trips: from very hot to very cold */
enum tsens_trip_type {
	TSENS_TRIP_STAGE3 = 0,
	TSENS_TRIP_STAGE2,
	TSENS_TRIP_STAGE1,
	TSENS_TRIP_STAGE0,
	TSENS_TRIP_NUM,
};

struct tsens_tm_device;

struct tsens_tm_device_sensor {
	struct thermal_zone_device	*tzone;
	enum thermal_device_mode	mode;
	struct tsens_tm_device		*tmdev;
	unsigned int			sensor_num;
	int				offset;
	uint32_t			slope;
	bool				user_zone;
};

struct tsens_variant_data {
	int	slope[TSENS_MAX_SENSORS];
	u32	nsensors;
	/* enable to tsens slp clock */
	u32	slp_clk_ena;
	u32	sctrl_reg;
	u32	sctrl_offset;
	u32	config_reg;
	u32	config_mask;
	u32	config;
	int (*calib_sensors)(struct tsens_tm_device *tmdev);
};

struct tsens_tm_device {
	struct regmap			*base;
	struct regmap			*qfprom_base;
	//void __iomem			*qfprom_base;
	struct tsens_variant_data	*data;
	struct work_struct		tsens_work;
	int				qfprom_offset;
	int				qfprom_size;
	int				nsensors;
	int				irq;
	/* tsens ready bit for reading valid temperature */
	bool				trdy;
	struct tsens_tm_device_sensor	sensor[0];
};

/* Temperature on y axis and ADC-code on x-axis */
static inline int tsens_code_to_degc(struct tsens_tm_device_sensor *s,
				     int adc_code)
{
	return adc_code * s->slope + s->offset;
}

static inline int tsens_degc_to_code(struct tsens_tm_device_sensor *s, int degc)
{
	return (degc - s->offset + s->slope / 2) / s->slope;
}

static int __tsens_get_temp(void *data, long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = data;
	unsigned int code, offset = 0;
	struct tsens_tm_device *tmdev = tm_sensor->tmdev;
	int sensor_num	= tm_sensor->sensor_num;
	struct regmap *base = tmdev->base;

	if (!tmdev->trdy) {
		u32 val;

		regmap_read(base, TSENS_INT_STATUS_ADDR, &val);

		while (!(val & TSENS_TRDY_MASK)) {
			usleep_range(TSENS_TRDY_RDY_MIN_TIME,
				TSENS_TRDY_RDY_MAX_TIME);
			regmap_read(base, TSENS_INT_STATUS_ADDR, &val);
		}
		tmdev->trdy = true;
	}
//FIXME
	if (sensor_num >= TSENS_8064_SEQ_SENSORS)
		offset = TSENS_8064_S4_S5_OFFSET;

	regmap_read(base, TSENS_S0_STATUS_ADDR + offset +
			(sensor_num << TSENS_STATUS_ADDR_OFFSET), &code);
	*temp = tsens_code_to_degc(tm_sensor, code);

	return 0;
}

static int tsens_get_temp(struct thermal_zone_device *thermal,
			     unsigned long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;

	if (!tm_sensor || tm_sensor->mode != THERMAL_DEVICE_ENABLED)
		return -EINVAL;

	return __tsens_get_temp(tm_sensor, temp);

}

static int tsens_get_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode *mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;

	if (tm_sensor)
		*mode = tm_sensor->mode;

	return 0;
}

/**
 * Function to enable the mode.
 * If the main sensor is disabled all the sensors are disable and
 * the clock is disabled.
 * If the main sensor is not enabled and sub sensor is enabled
 * returns with an error stating the main sensor is not enabled.
 **/

static int tsens_set_mode(struct thermal_zone_device *thermal,
			      enum thermal_device_mode mode)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	struct tsens_tm_device *tmdev = tm_sensor->tmdev;
	struct tsens_variant_data *data = tmdev->data;
	struct regmap *base = tmdev->base;
	unsigned int reg, mask, i;

	if (!tm_sensor)
		return -EINVAL;

	if (mode != tm_sensor->mode) {
		regmap_read(base, TSENS_CNTL_ADDR, &reg);
		mask = BIT(tm_sensor->sensor_num + TSENS_SENSOR0_SHIFT);
		if (mode == THERMAL_DEVICE_ENABLED) {
			if ((mask != SENSOR0_EN) && !(reg & SENSOR0_EN)) {
				pr_info("Main sensor not enabled\n");
				return -EINVAL;
			}
			regmap_write(base, TSENS_CNTL_ADDR, reg | TSENS_SW_RST);
			reg |= mask | data->slp_clk_ena | TSENS_EN;
			tmdev->trdy = false;
		} else {
			reg &= ~mask;
			if (!(reg & SENSOR0_EN)) {
				reg &= ~(SENSORS_EN_MASK(tmdev->nsensors) |
					data->slp_clk_ena |
					TSENS_EN);

				for (i = 1; i < tmdev->nsensors; i++)
					tmdev->sensor[i].mode = mode;

			}
		}
		regmap_write(base, TSENS_CNTL_ADDR, reg);
	}
	tm_sensor->mode = mode;

	return 0;
}

static int tsens_get_trip_type(struct thermal_zone_device *thermal,
				   int trip, enum thermal_trip_type *type)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;

	if (!tm_sensor || trip < 0 || !type)
		return -EINVAL;

	switch (trip) {
	case TSENS_TRIP_STAGE3:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	case TSENS_TRIP_STAGE2:
		*type = THERMAL_TRIP_HOT;
		break;
	case TSENS_TRIP_STAGE1:
		*type = THERMAL_TRIP_PASSIVE;
		break;
	case TSENS_TRIP_STAGE0:
		*type = THERMAL_TRIP_ACTIVE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tsens_get_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long *temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	struct tsens_tm_device *tmdev = tm_sensor->tmdev;
	unsigned int reg;

	if (!tm_sensor || trip < 0 || !temp)
		return -EINVAL;

	regmap_read(tmdev->base, TSENS_TH_ADDR, &reg);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		reg = (reg & TSENS_TH_MAX_LIMIT_MASK)
					>> TSENS_TH_MAX_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE2:
		reg = (reg & TSENS_TH_UPPER_LIMIT_MASK)
					>> TSENS_TH_UPPER_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE1:
		reg = (reg & TSENS_TH_LOWER_LIMIT_MASK)
					>> TSENS_TH_LOWER_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE0:
		reg = (reg & TSENS_TH_MIN_LIMIT_MASK)
					>> TSENS_TH_MIN_LIMIT_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	*temp = tsens_code_to_degc(tm_sensor, reg);

	return 0;
}

static int tsens_set_trip_temp(struct thermal_zone_device *thermal,
				   int trip, unsigned long temp)
{
	struct tsens_tm_device_sensor *tm_sensor = thermal->devdata;
	unsigned int reg_th, reg_cntl;
	int code, hi_code, lo_code, code_err_chk;
	struct tsens_tm_device *tmdev = tm_sensor->tmdev;
	struct regmap *base = tmdev->base;
	struct tsens_variant_data *data = tmdev->data;
	u32 offset = data->sctrl_offset;

	code_err_chk = code = tsens_degc_to_code(tm_sensor, temp);
	if (!tm_sensor || trip < 0)
		return -EINVAL;

	lo_code = TSENS_TH_MIN_CODE;
	hi_code = TSENS_TH_MAX_CODE;

	regmap_read(base, data->sctrl_reg, &reg_cntl);

	regmap_read(base, TSENS_TH_ADDR, &reg_th);
	switch (trip) {
	case TSENS_TRIP_STAGE3:
		code <<= TSENS_TH_MAX_LIMIT_SHIFT;
		reg_th &= ~TSENS_TH_MAX_LIMIT_MASK;

		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR(offset)))
			lo_code = (reg_th & TSENS_TH_UPPER_LIMIT_MASK)
					>> TSENS_TH_UPPER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_LOWER_STATUS_CLR(offset)))
			lo_code = (reg_th & TSENS_TH_LOWER_LIMIT_MASK)
					>> TSENS_TH_LOWER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK(offset)))
			lo_code = (reg_th & TSENS_TH_MIN_LIMIT_MASK)
					>> TSENS_TH_MIN_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE2:
		code <<= TSENS_TH_UPPER_LIMIT_SHIFT;
		reg_th &= ~TSENS_TH_UPPER_LIMIT_MASK;

		if (!(reg_cntl & TSENS_MAX_STATUS_MASK(offset)))
			hi_code = (reg_th & TSENS_TH_MAX_LIMIT_MASK)
					>> TSENS_TH_MAX_LIMIT_SHIFT;
		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR(offset)))
			lo_code = (reg_th & TSENS_TH_LOWER_LIMIT_MASK)
					>> TSENS_TH_LOWER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_MIN_STATUS_MASK(offset)))
			lo_code = (reg_th & TSENS_TH_MIN_LIMIT_MASK)
					>> TSENS_TH_MIN_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE1:
		code <<= TSENS_TH_LOWER_LIMIT_SHIFT;
		reg_th &= ~TSENS_TH_LOWER_LIMIT_MASK;

		if (!(reg_cntl & TSENS_MIN_STATUS_MASK(offset)))
			lo_code = (reg_th & TSENS_TH_MIN_LIMIT_MASK)
					>> TSENS_TH_MIN_LIMIT_SHIFT;
		if (!(reg_cntl & TSENS_UPPER_STATUS_CLR(offset)))
			hi_code = (reg_th & TSENS_TH_UPPER_LIMIT_MASK)
					>> TSENS_TH_UPPER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK(offset)))
			hi_code = (reg_th & TSENS_TH_MAX_LIMIT_MASK)
					>> TSENS_TH_MAX_LIMIT_SHIFT;
		break;
	case TSENS_TRIP_STAGE0:
		code <<= TSENS_TH_MIN_LIMIT_SHIFT;
		reg_th &= ~TSENS_TH_MIN_LIMIT_MASK;

		if (!(reg_cntl & TSENS_LOWER_STATUS_CLR(offset)))
			hi_code = (reg_th & TSENS_TH_LOWER_LIMIT_MASK)
					>> TSENS_TH_LOWER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_UPPER_STATUS_CLR(offset)))
			hi_code = (reg_th & TSENS_TH_UPPER_LIMIT_MASK)
					>> TSENS_TH_UPPER_LIMIT_SHIFT;
		else if (!(reg_cntl & TSENS_MAX_STATUS_MASK(offset)))
			hi_code = (reg_th & TSENS_TH_MAX_LIMIT_MASK)
					>> TSENS_TH_MAX_LIMIT_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	if (code_err_chk < lo_code || code_err_chk > hi_code)
		return -EINVAL;

	regmap_write(base, TSENS_TH_ADDR, reg_th | code);

	return 0;
}

static int tsens_get_crit_temp(struct thermal_zone_device *thermal,
				  unsigned long *temp)
{
	return tsens_get_trip_temp(thermal, TSENS_TRIP_STAGE3, temp);
}

static struct thermal_zone_device_ops tsens_thermal_zone_ops = {
	.get_temp = tsens_get_temp,
	.get_mode = tsens_get_mode,
	.set_mode = tsens_set_mode,
	.get_trip_type = tsens_get_trip_type,
	.get_trip_temp = tsens_get_trip_temp,
	.set_trip_temp = tsens_set_trip_temp,
	.get_crit_temp = tsens_get_crit_temp,
};

static void tsens_scheduler_fn(struct work_struct *work)
{
	struct tsens_tm_device *tmdev = container_of(work,
					struct tsens_tm_device, tsens_work);
	unsigned int threshold, threshold_low, i, code, reg, sensor, mask;
	unsigned int sensor_addr;
	bool upper_th_x, lower_th_x;
	struct regmap *base = tmdev->base;
	struct tsens_variant_data *data = tmdev->data;
	u32 offset = data->sctrl_offset;

	regmap_read(base, data->sctrl_reg, &reg);
	regmap_write(base, data->sctrl_reg, reg |
					    TSENS_LOWER_STATUS_CLR(offset) |
					    TSENS_UPPER_STATUS_CLR(offset));

	mask = ~(TSENS_LOWER_STATUS_CLR(offset) | TSENS_UPPER_STATUS_CLR(offset));
	regmap_read(tmdev->base, TSENS_TH_ADDR, &threshold);
	threshold_low = (threshold & TSENS_TH_LOWER_LIMIT_MASK)
					>> TSENS_TH_LOWER_LIMIT_SHIFT;
	threshold = (threshold & TSENS_TH_UPPER_LIMIT_MASK)
					>> TSENS_TH_UPPER_LIMIT_SHIFT;

	regmap_read(base, TSENS_CNTL_ADDR, &sensor);
	sensor &= SENSORS_EN_MASK(tmdev->nsensors);
	sensor >>= TSENS_SENSOR0_SHIFT;
	sensor_addr = TSENS_S0_STATUS_ADDR;
	for (i = 0; i < tmdev->nsensors; i++) {
		if (i == TSENS_8064_SEQ_SENSORS)
			sensor_addr += TSENS_8064_S4_S5_OFFSET;
		if (sensor & TSENS_MASK1) {
			regmap_read(base, sensor_addr, &code);
			upper_th_x = code >= threshold;
			lower_th_x = code <= threshold_low;
			if (upper_th_x)
				mask |= TSENS_UPPER_STATUS_CLR(offset);
			if (lower_th_x)
				mask |= TSENS_LOWER_STATUS_CLR(offset);
			if (upper_th_x || lower_th_x)
				thermal_zone_device_update(tmdev->sensor[i].tzone);
		}
		sensor >>= 1;
		sensor_addr += 4;
	}
	regmap_read(base, data->sctrl_reg, &reg);
	regmap_write(base, data->sctrl_reg, reg & mask);
}

static irqreturn_t tsens_isr(int irq, void *dev)
{
	struct tsens_tm_device *tmdev = dev;

	schedule_work(&tmdev->tsens_work);

	return IRQ_HANDLED;
}

static void tsens_disable_mode(struct tsens_tm_device *tmdev)
{
	unsigned int reg_cntl = 0;
	struct regmap *base = tmdev->base;
	struct tsens_variant_data	*data = tmdev->data;

	regmap_read(base, TSENS_CNTL_ADDR, &reg_cntl);
	regmap_write(base, TSENS_CNTL_ADDR, reg_cntl &
			~(SENSORS_EN_MASK(tmdev->nsensors) | data->slp_clk_ena
			| TSENS_EN));
}

static void tsens_hw_init(struct tsens_tm_device *tmdev)
{
	unsigned int reg_cntl = 0, reg_cfg = 0, reg_thr = 0;
	unsigned int reg_status_cntl = 0;
	struct regmap *base = tmdev->base;
	struct tsens_variant_data	*data = tmdev->data;
	u32 offset = data->sctrl_offset;

	regmap_read(base, TSENS_CNTL_ADDR, &reg_cntl);
	regmap_write(base, TSENS_CNTL_ADDR, reg_cntl | TSENS_SW_RST);

	reg_cntl |= data->slp_clk_ena |
		(TSENS_MEASURE_PERIOD << 18) |
		SENSORS_EN_MASK(tmdev->nsensors);
	regmap_write(base, TSENS_CNTL_ADDR, reg_cntl);

	regmap_read(base, data->sctrl_reg, &reg_status_cntl);
	reg_status_cntl |= TSENS_LOWER_STATUS_CLR(offset) |
		TSENS_UPPER_STATUS_CLR(offset) |
		TSENS_MIN_STATUS_MASK(offset) |
		TSENS_MAX_STATUS_MASK(offset);

	regmap_write(base, data->sctrl_reg, reg_status_cntl);
		

	reg_cntl |= TSENS_EN;
	regmap_write(base, TSENS_CNTL_ADDR, reg_cntl);

	regmap_read(base, data->config_reg, &reg_cfg);
	reg_cfg = (reg_cfg & ~data->config_mask) | data->config;
	regmap_write(base, data->config_reg, reg_cfg);

	reg_thr |= (TSENS_LOWER_LIMIT_TH << TSENS_TH_LOWER_LIMIT_SHIFT) |
		(TSENS_UPPER_LIMIT_TH << TSENS_TH_UPPER_LIMIT_SHIFT) |
		(TSENS_MIN_LIMIT_TH << TSENS_TH_MIN_LIMIT_SHIFT) |
		(TSENS_MAX_LIMIT_TH << TSENS_TH_MAX_LIMIT_SHIFT);
	regmap_write(base, TSENS_TH_ADDR, reg_thr);
}

static int tsens_8960_calib_sensors(struct tsens_tm_device *tmdev)
{

	int rc, i;
	uint8_t calib_data, calib_data_backup;
	int sz	= round_up(tmdev->qfprom_size, sizeof(int));
	uint8_t *cdata = kzalloc(sz, GFP_KERNEL | GFP_ATOMIC);

	rc = regmap_bulk_read(tmdev->qfprom_base, tmdev->qfprom_offset, cdata, sz/sizeof(int));
	if (rc < 0)
		return rc;
	for (i = 0; i < tmdev->nsensors; i++) {
		calib_data = cdata[i];
		calib_data_backup = cdata[TSENS_8960_QFPROM_SPARE_OFFSET + i];
		if (calib_data_backup)
			calib_data = calib_data_backup;
		
		if (!calib_data) {
			pr_err("QFPROM TSENS calibration data not present\n");
			return -ENODEV;
		}
		tmdev->sensor[i].offset = TSENS_CAL_MILLI_DEGC - (calib_data * tmdev->sensor[i].slope);
		tmdev->trdy = false;
	}
	kfree(cdata);
	return 0;
}

static int tsens_calib_sensors(struct tsens_tm_device *tmdev)
{
	if (tmdev->data->calib_sensors)
		return tmdev->data->calib_sensors(tmdev);

	return -ENODEV;
}

struct tsens_variant_data msm8960_data =
{
	/**
	 * Slope for a thermocouple in a given part is always same
	 * for desired range of temperature measurements
	 **/
	.slope			= {910, 910, 910, 910, 910},
	.nsensors		= 5,
	.slp_clk_ena		= BIT(26),
	.sctrl_reg		= TSENS_CNTL_ADDR,
	.sctrl_offset		= 8,
	.config_reg		= TSENS_8960_CONFIG_ADDR,
	.config_mask		= TSENS_8960_CONFIG_MASK,
	.config			= TSENS_8960_CONFIG,
	.calib_sensors		= tsens_8960_calib_sensors,
};

struct tsens_variant_data apq8064_data = {
	/**
	 * Slope for a thermocouple in a given part is always same
	 * for desired range of temperature measurements
	 **/
	.slope			= {1176, 1176, 1154, 1176, 1111, 1132,
				   1132, 1199, 1132, 1199, 1132},
	.nsensors		= 11,
	.slp_clk_ena		= BIT(26),
	.sctrl_reg		= TSENS_8064_STATUS_CNTL,
	.sctrl_offset		= 0,
	.config_reg		= TSENS_8960_CONFIG_ADDR,
	.config_mask		= TSENS_8960_CONFIG_MASK,
	.config			= TSENS_8960_CONFIG,
	.calib_sensors		= tsens_8960_calib_sensors,
};

static struct of_device_id qcom_tsens_of_match[] = {
	{ .compatible = "qcom,msm8960-tsens", .data = &msm8960_data },
	{ .compatible = "qcom,apq8064-tsens", .data = &apq8064_data },
};
MODULE_DEVICE_TABLE(of, qcom_tsens_of_match);

static int tsens_tm_probe(struct platform_device *pdev)
{
	int rc, i;
	struct device *dev = &pdev->dev;
	struct tsens_tm_device *tmdev;
	const struct tsens_variant_data *data;
	struct device_node *np = pdev->dev.of_node;

	if (!np) {
		dev_err(dev, "Non DT not supported\n");
		return -EINVAL;
	}

	data = of_match_node(qcom_tsens_of_match, np)->data;
	tmdev = devm_kzalloc(dev, sizeof(struct tsens_tm_device) +
			data->nsensors *
			sizeof(struct tsens_tm_device_sensor),
			GFP_ATOMIC);
	if (tmdev == NULL) {
		pr_err("%s: kzalloc() failed.\n", __func__);
		return -ENOMEM;
	}

	tmdev->qfprom_base = syscon_regmap_lookup_by_phandle(np, "qcom,calib-data");
	if (IS_ERR(tmdev->qfprom_base))
		return PTR_ERR(tmdev->qfprom_base);
	
	rc = of_property_read_u32_index(np, "qcom,calib-data", 1, &tmdev->qfprom_offset);
	if (rc) {
		dev_info(dev, "Could not read qfprom_offset from qcom,calib-data!\n");
		return -EINVAL;
	}

	rc = of_property_read_u32_index(np, "qcom,calib-data", 2, &tmdev->qfprom_size);
	if (rc) {
		dev_info(dev, "Could not read qfprom_size from qcom,calib-data!\n");
		return -EINVAL;
	}

	for (i = 0; i < data->nsensors; i++)
		tmdev->sensor[i].slope = data->slope[i];

	tmdev->data =(struct tsens_variant_data *) data;
	tmdev->nsensors = data->nsensors;
	tmdev->base = dev_get_regmap(dev->parent, NULL);

	if (!tmdev->base) {
		dev_err(&pdev->dev, "Parent regmap unavailable.\n");
		return -ENXIO;
	}

	rc = tsens_calib_sensors(tmdev);
	if (rc < 0)
		return rc;

	tsens_hw_init(tmdev);

	for (i = 0; i < tmdev->nsensors; i++) {
		char name[18];
		snprintf(name, sizeof(name), "tsens_sensor%d", i);
		tmdev->sensor[i].mode = THERMAL_DEVICE_ENABLED;
		tmdev->sensor[i].sensor_num = i;
		tmdev->sensor[i].tmdev = tmdev;

		tmdev->sensor[i].tzone = thermal_zone_of_sensor_register(dev, i,
					&tmdev->sensor[i], __tsens_get_temp, NULL);

		if (IS_ERR(tmdev->sensor[i].tzone)) {
			tmdev->sensor[i].tzone = thermal_zone_device_register(
					name,
					TSENS_TRIP_NUM,
					GENMASK(TSENS_TRIP_NUM - 1 , 0),
					&tmdev->sensor[i],
					&tsens_thermal_zone_ops, NULL, 0, 0);
			if (IS_ERR(tmdev->sensor[i].tzone)) {
				dev_err(dev, "thermal_zone_device_register() failed.\n");
				rc = -ENODEV;
				goto fail;
			}
			tmdev->sensor[i].user_zone = true;
		}
	}

	tmdev->irq = platform_get_irq_byname(pdev, "tsens-ul");
	if (tmdev->irq > 0) {
		rc = devm_request_irq(dev, tmdev->irq, tsens_isr,
					IRQF_TRIGGER_RISING,
					"tsens_interrupt", tmdev);
		if (rc < 0) {
			pr_err("%s: request_irq FAIL: %d\n", __func__, rc);
			for (i = 0; i < tmdev->nsensors; i++)
				if (tmdev->sensor[i].user_zone)
					thermal_zone_device_unregister(
						tmdev->sensor[i].tzone);
				else
					thermal_zone_of_sensor_unregister(dev,
						tmdev->sensor[i].tzone);
			goto fail;
		}
		INIT_WORK(&tmdev->tsens_work, tsens_scheduler_fn);
	}
	platform_set_drvdata(pdev, tmdev);

	dev_info(dev, "Probed sucessfully\n");

	return 0;
fail:
	tsens_disable_mode(tmdev);

	return rc;
}

static int tsens_tm_remove(struct platform_device *pdev)
{
	int i;
	struct tsens_tm_device *tmdev = platform_get_drvdata(pdev);
	struct tsens_tm_device_sensor *s;

	tsens_disable_mode(tmdev);
	
	for (i = 0; i < tmdev->nsensors; i++) {
		s = &tmdev->sensor[i];
		if (s->user_zone)
			thermal_zone_device_unregister(s->tzone);
		else
			thermal_zone_of_sensor_unregister(&pdev->dev,
							  s->tzone);
	}

	return 0;
}

static struct platform_driver tsens_tm_driver = {
	.probe = tsens_tm_probe,
	.remove = tsens_tm_remove,
	.driver = {
		.name = "tsens8960-tm",
		.owner = THIS_MODULE,
		.of_match_table = qcom_tsens_of_match,
	},
};

module_platform_driver(tsens_tm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM8960 Temperature Sensor driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:tsens8960-tm");
