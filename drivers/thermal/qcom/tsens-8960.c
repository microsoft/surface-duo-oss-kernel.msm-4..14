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
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/regmap.h>
#include "tsens.h"

#define CAL_MDEGC		30000

#define CONFIG_ADDR		0x3640
#define CONFIG_ADDR_8660	0x3620
/* CONFIG_ADDR bitmasks */
#define CONFIG			0x9b
#define CONFIG_MASK		0xf
#define CONFIG_8660		1
#define CONFIG_SHIFT_8660	28
#define CONFIG_MASK_8660	(3 << CONFIG_SHIFT_8660)

#define STATUS_CNTL_ADDR_8064	0x3660
#define CNTL_ADDR		0x3620
/* CNTL_ADDR bitmasks */
#define EN			BIT(0)
#define SW_RST			BIT(1)
#define SENSOR0_EN		BIT(3)
#define SLP_CLK_ENA		BIT(26)
#define SLP_CLK_ENA_8660	BIT(24)
#define MEASURE_PERIOD		1
#define SENSOR0_SHIFT		3

/* INT_STATUS_ADDR bitmasks */
#define MIN_STATUS_MASK		BIT(0)
#define LOWER_STATUS_CLR	BIT(1)
#define UPPER_STATUS_CLR	BIT(2)
#define MAX_STATUS_MASK		BIT(3)

#define THRESHOLD_ADDR		0x3624
/* THRESHOLD_ADDR bitmasks */
#define THRESHOLD_MAX_LIMIT_SHIFT	24
#define THRESHOLD_MIN_LIMIT_SHIFT	16
#define THRESHOLD_UPPER_LIMIT_SHIFT	8
#define THRESHOLD_LOWER_LIMIT_SHIFT	0

/* Initial temperature threshold values */
#define LOWER_LIMIT_TH		0x50
#define UPPER_LIMIT_TH		0xdf
#define MIN_LIMIT_TH		0x0
#define MAX_LIMIT_TH		0xff

#define S0_STATUS_ADDR		0x3628
#define INT_STATUS_ADDR		0x363c
#define TRDY_MASK		BIT(7)

static int suspend_8960(struct tsens_device *tmdev)
{
	unsigned int mask;
	struct regmap *map = tmdev->map;

	regmap_read(map, THRESHOLD_ADDR, &tmdev->pm_tsens_thr_data);
	regmap_read(map, CNTL_ADDR, &tmdev->pm_tsens_cntl);

	if (tmdev->num_sensors > 1)
		mask = SLP_CLK_ENA | EN;
	else
		mask = SLP_CLK_ENA_8660 | EN;

	regmap_update_bits(map, CNTL_ADDR, mask, 0);
	tmdev->prev_reading_avail = 0;

	return 0;
}

static int resume_8960(struct tsens_device *tmdev)
{
	unsigned long reg_cntl;
	struct regmap *map = tmdev->map;

	regmap_update_bits(map, CNTL_ADDR, SW_RST, SW_RST);
	regmap_field_update_bits(tmdev->status_field,
				 MIN_STATUS_MASK | MAX_STATUS_MASK,
				 MIN_STATUS_MASK | MAX_STATUS_MASK);
	/*
	 * Separate CONFIG restore is not needed only for 8660 as
	 * config is part of CTRL Addr and its restored as such
	 */
	if (tmdev->num_sensors > 1)
		regmap_update_bits(map, CONFIG_ADDR, CONFIG_MASK, CONFIG);

	regmap_write(map, THRESHOLD_ADDR, tmdev->pm_tsens_thr_data);
	regmap_write(map, CNTL_ADDR, tmdev->pm_tsens_cntl);

	reg_cntl = tmdev->pm_tsens_cntl;
	reg_cntl >>= SENSOR0_SHIFT;

	return 0;
}

static int enable_8960(struct tsens_device *tmdev, int id)
{
	u32 reg, mask;

	regmap_read(tmdev->map, CNTL_ADDR, &reg);
	mask = BIT(id + SENSOR0_SHIFT);
	regmap_write(tmdev->map, CNTL_ADDR, reg | SW_RST);

	if (tmdev->num_sensors > 1)
		reg |= mask | SLP_CLK_ENA | EN;
	else
		reg |= mask | SLP_CLK_ENA_8660 | EN;

	tmdev->prev_reading_avail = false;
	regmap_write(tmdev->map, CNTL_ADDR, reg);

	return 0;
}

static void disable_8960(struct tsens_device *tmdev)
{
	u32 reg_cntl;
	u32 mask;

	mask = GENMASK(tmdev->num_sensors - 1, 0);
	mask <<= SENSOR0_SHIFT;
	mask |= EN;

	regmap_read(tmdev->map, CNTL_ADDR, &reg_cntl);
	reg_cntl &= ~mask;

	if (tmdev->num_sensors > 1)
		reg_cntl &= ~SLP_CLK_ENA;
	else
		reg_cntl &= ~SLP_CLK_ENA_8660;

	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);
}

static int init_8960(struct tsens_device *tmdev)
{
	u32 reg_cntl, reg_thr;
	struct reg_field *field;
	static struct reg_field status_0 = {
		.reg = STATUS_CNTL_ADDR_8064,
		.lsb = 0,
		.msb = 3,
	};
	static struct reg_field status_8 = {
		.reg = CNTL_ADDR,
		.lsb = 8,
		.msb = 11,
	};
	int i;

	tmdev->map = dev_get_regmap(tmdev->dev->parent, NULL);
	if (!tmdev->map)
		return -ENODEV;

	/* Status bits move when the sensor bits next to them overlap */
	if (tmdev->num_sensors > 5)
		field = &status_0;
	else
		field = &status_8;

	tmdev->status_field = devm_regmap_field_alloc(tmdev->dev, tmdev->map,
						      *field);
	if (IS_ERR(tmdev->status_field)) {
		dev_err(tmdev->dev, "regmap alloc failed\n");
		return PTR_ERR(tmdev->status_field);
	}

	/*
	 * The status registers for each sensor are discontiguous
	 * because some SoCs have 5 sensors while others have more
	 * but the control registers stay in the same place, i.e
	 * directly after the first 5 status registers.
	 */
	for (i = 0; i < tmdev->num_sensors; i++) {
		if (i >= 5)
			tmdev->sensor[i].status = S0_STATUS_ADDR + 40;
		tmdev->sensor[i].status += i * 4;
	}

	reg_cntl = SW_RST;
	regmap_update_bits(tmdev->map, CNTL_ADDR, SW_RST, reg_cntl);

	if (tmdev->num_sensors > 1) {
		reg_cntl |= SLP_CLK_ENA | (MEASURE_PERIOD << 18);
		reg_cntl &= ~SW_RST;
		regmap_update_bits(tmdev->map, CONFIG_ADDR,
				   CONFIG_MASK, CONFIG);
	} else {
		reg_cntl |= SLP_CLK_ENA_8660 | (MEASURE_PERIOD << 16);
		reg_cntl &= ~CONFIG_MASK_8660;
		reg_cntl |= CONFIG_8660 << CONFIG_SHIFT_8660;
	}

	reg_cntl |= GENMASK(tmdev->num_sensors - 1, 0) << SENSOR0_SHIFT;
	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);

	regmap_field_update_bits(tmdev->status_field,
				 LOWER_STATUS_CLR | UPPER_STATUS_CLR |
				 MIN_STATUS_MASK | MAX_STATUS_MASK,
				 LOWER_STATUS_CLR | UPPER_STATUS_CLR |
				 MIN_STATUS_MASK | MAX_STATUS_MASK);

	reg_cntl |= EN;
	regmap_write(tmdev->map, CNTL_ADDR, reg_cntl);

	reg_thr = (LOWER_LIMIT_TH << THRESHOLD_LOWER_LIMIT_SHIFT) |
		(UPPER_LIMIT_TH << THRESHOLD_UPPER_LIMIT_SHIFT) |
		(MIN_LIMIT_TH << THRESHOLD_MIN_LIMIT_SHIFT) |
		(MAX_LIMIT_TH << THRESHOLD_MAX_LIMIT_SHIFT);
	regmap_write(tmdev->map, THRESHOLD_ADDR, reg_thr);

	return 0;
}

static int calibrate_8960(struct tsens_device *tmdev)
{
	int i;
	char *data;

	ssize_t num_read = tmdev->num_sensors;
	struct tsens_sensor *s = tmdev->sensor;

	data = qfprom_read(tmdev->dev, "calib");
	if (IS_ERR(data))
		data = qfprom_read(tmdev->dev, "backup_calib");
	if (IS_ERR(data))
		return PTR_ERR(data);

	for (i = 0; i < num_read; i++, s++)
		s->offset = CAL_MDEGC - s->slope * data[i];

	return 0;
}

/* Temperature on y axis and ADC-code on x-axis */
static inline int code_to_mdegC(u32 adc_code, const struct tsens_sensor *s)
{
	return adc_code * s->slope + s->offset;
}

static int get_temp_8960(struct tsens_device *tmdev, int id, long *temp)
{
	u32 code, trdy;
	const struct tsens_sensor *s = &tmdev->sensor[id];

	if (!tmdev->prev_reading_avail) {
		while (!regmap_read(tmdev->map, INT_STATUS_ADDR, &trdy) &&
		       !(trdy & TRDY_MASK))
			usleep_range(1000, 1100);
		tmdev->prev_reading_avail = true;
	}

	regmap_read(tmdev->map, s->status, &code);
	*temp = code_to_mdegC(code, s);

	dev_dbg(tmdev->dev, "Sensor%d temp is: %ld", id, *temp);

	return 0;
}

const struct tsens_ops ops_8960 = {
	.init		= init_8960,
	.calibrate	= calibrate_8960,
	.get_temp	= get_temp_8960,
	.enable		= enable_8960,
	.disable	= disable_8960,
	.suspend	= suspend_8960,
	.resume		= resume_8960,
};
