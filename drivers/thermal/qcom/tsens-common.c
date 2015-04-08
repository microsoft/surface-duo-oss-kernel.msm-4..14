
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
#include <linux/eeprom-consumer.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include "tsens.h"

#define S0_ST_ADDR(n)		((n) + 0x1030)
#define TRDY_ADDR(n)		((n) + 0x105c)
#define SN_ADDR_OFFSET		0x4
#define SN_ST_TEMP_MASK		0x3ff
#define CAL_DEGC_PT1		30
#define CAL_DEGC_PT2		120
#define SLOPE_FACTOR		1000

char *qfprom_read(struct device *dev, const char *cname)
{
	struct eeprom_cell *cell;
	ssize_t data;
	char *ret;

	cell = of_eeprom_cell_get_byname(dev, cname);
	if (IS_ERR(cell))
		return ERR_CAST(cell);

	ret = eeprom_cell_read(cell, &data);
	eeprom_cell_put(cell);
	return ret;
}

void compute_intercept_slope(struct tsens_device *tmdev, u32 *p1,
			     u32 *p2, u32 mode)
{
	int i;
	int num, den;

	for (i = 0; i < tmdev->num_sensors; i++) {
		dev_dbg(tmdev->dev,
			"sensor%d - data_point1:%#x data_point2:%#x\n",
			i, p1[i], p2[i]);

		if (mode == TWO_PT_CALIB) {
			/* slope (m) = adc_code2 - adc_code1 (y2 - y1)/
				temp_120_degc - temp_30_degc (x2 - x1) */
			num = p2[i] - p1[i];
			num *= SLOPE_FACTOR;
			den = CAL_DEGC_PT2 - CAL_DEGC_PT1;
			tmdev->sensor[i].slope = num / den;
		}

		tmdev->sensor[i].offset = (p1[i] * SLOPE_FACTOR) -
				(CAL_DEGC_PT1 *
				tmdev->sensor[i].slope);
		dev_dbg(tmdev->dev, "offset:%d\n", tmdev->sensor[i].offset);
	}
}

static inline int code_to_degc(u32 adc_code, const struct tsens_sensor *s)
{
	int degc, num, den;

	num = (adc_code * SLOPE_FACTOR) - s->offset;
	den = s->slope;

	if (num > 0)
		degc = num + (den / 2);
	else if (num < 0)
		degc = num - (den / 2);
	else
		degc = num;

	degc /= den;

	return degc;
}

int get_temp_common(struct tsens_device *tmdev, int id, long *temp)
{
	struct tsens_sensor *s = &tmdev->sensor[id];
	unsigned int code;
	void __iomem *sensor_addr;
	int last_temp = 0;

	sensor_addr = S0_ST_ADDR(tmdev->base) + s->hw_id * SN_ADDR_OFFSET;
	code = readl_relaxed(sensor_addr);
	last_temp = code & SN_ST_TEMP_MASK;

	*temp = code_to_degc(last_temp, s);

	return 0;
}

int init_common(struct tsens_device *tmdev)
{
	tmdev->base = of_iomap(tmdev->dev->of_node, 0);
	if (IS_ERR(tmdev->base))
		return -EINVAL;
	return 0;
}
