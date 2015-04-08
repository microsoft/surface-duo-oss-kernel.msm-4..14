/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __QCOM_TSENS_H__
#define __QCOM_TSENS_H__

struct tsens_device;

struct tsens_sensor {
	struct tsens_device		*tmdev;
	struct thermal_zone_device	*tzd;
	int				offset;
	int				id;
	int				hw_id;
	u32				slope;
	u32				status;
};

struct tsens_ops {
	/* mandatory callbacks */
	int (*init)(struct tsens_device *);
	int (*calibrate)(struct tsens_device *);
	int (*get_temp)(struct tsens_device *, int, long *);
	/* optional callbacks */
	int (*enable)(struct tsens_device *, int);
	void (*disable)(struct tsens_device *);
	int (*suspend)(struct tsens_device *);
	int (*resume)(struct tsens_device *);
	int (*get_trend)(struct tsens_device *, int, long *);
};

struct tsens_device {
	struct device			*dev;
	u32				num_sensors;
	void __iomem			*base;
	struct regmap			*map;
	struct regmap_field		*status_field;
	int				pm_tsens_thr_data;
	int				pm_tsens_cntl;
	bool				prev_reading_avail;
	const struct tsens_ops		*ops;
	struct tsens_sensor		sensor[0];
};

#endif /* __QCOM_TSENS_H__ */
