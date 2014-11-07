/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2014 ARM Limited
 */

#define DRVNAME "v2m-juno-hwmon"
#define pr_fmt(fmt) DRVNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

struct v2m_juno_hwmon_type {
	const char *name;
	const struct attribute_group *attr_groups;
};

static void __iomem *base;

struct v2m_juno_hwmon_dev {
	struct v2m_juno_hwmon_type *type;
	struct device *hwmon_dev;
	const char *name;
	u32 offset;
	u32 mult;
	u32 div;
};

static ssize_t v2m_juno_hwmon_name_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct v2m_juno_hwmon_dev *hdev = dev_get_drvdata(dev);

	return sprintf(buffer, "%s\n", hdev->name);
}

static ssize_t v2m_juno_hwmon_u32_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct v2m_juno_hwmon_dev *hdev = dev_get_drvdata(dev);
	u64 value;

	value = readl(base + hdev->offset);
	value *= hdev->mult;
	return snprintf(buffer, PAGE_SIZE, "%llu\n", value / hdev->div);
}

static ssize_t v2m_juno_hwmon_u64_show(struct device *dev,
		struct device_attribute *dev_attr, char *buffer)
{
	struct v2m_juno_hwmon_dev *hdev = dev_get_drvdata(dev);
	u64 value;

	value = readq(base + hdev->offset);
	value *= hdev->mult;
	return snprintf(buffer, PAGE_SIZE, "%llu\n", value / hdev->div);
}

static DEVICE_ATTR(name, S_IRUGO, v2m_juno_hwmon_name_show, NULL);

#define JUNO_HWMON_ATTRS(_name, _input_attr)			\
struct attribute *v2m_juno_hwmon_attrs_##_name[] = {		\
	&dev_attr_name.attr,					\
	&sensor_dev_attr_##_input_attr.dev_attr.attr,		\
	NULL							\
}

#if !defined(CONFIG_REGULATOR_VEXPRESS)
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, v2m_juno_hwmon_u32_show,
		NULL, 1);
static	JUNO_HWMON_ATTRS(volt, in1_input);
static struct attribute_group v2m_juno_hwmon_group_volt = {
	.attrs = v2m_juno_hwmon_attrs_volt,
};
static struct v2m_juno_hwmon_type v2m_juno_hwmon_volt = {
	.name = "v2m_juno_volt",
	.attr_groups = &v2m_juno_hwmon_group_volt,
};
#endif

static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, v2m_juno_hwmon_u32_show,
		NULL, 1);
static JUNO_HWMON_ATTRS(amp, curr1_input);
static struct attribute_group v2m_juno_hwmon_group_amp = {
	.attrs = v2m_juno_hwmon_attrs_amp,
};
static struct v2m_juno_hwmon_type v2m_juno_hwmon_amp = {
	.name = "v2m_juno_amp",
	.attr_groups = &v2m_juno_hwmon_group_amp,
};

static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, v2m_juno_hwmon_u32_show,
		NULL, 1);
static JUNO_HWMON_ATTRS(power, power1_input);
static struct attribute_group v2m_juno_hwmon_group_power = {
	.attrs = v2m_juno_hwmon_attrs_power,
};
static struct v2m_juno_hwmon_type v2m_juno_hwmon_power = {
	.name = "v2m_juno_power",
	.attr_groups = &v2m_juno_hwmon_group_power,
};

static SENSOR_DEVICE_ATTR(energy1_input, S_IRUGO, v2m_juno_hwmon_u64_show,
		NULL, 1);
static JUNO_HWMON_ATTRS(energy, energy1_input);
static struct attribute_group v2m_juno_hwmon_group_energy = {
	.attrs = v2m_juno_hwmon_attrs_energy,
};
static struct v2m_juno_hwmon_type v2m_juno_hwmon_energy = {
	.name = "v2m_juno_energy",
	.attr_groups = &v2m_juno_hwmon_group_energy,
};

/* current adc channels */
#define	SYS_ADC_CH0_PM1_SYS	0xd0
#define	SYS_ADC_CH1_PM2_A57	0xd4
#define	SYS_ADC_CH2_PM3_A53	0xd8
#define	SYS_ADC_CH3_PM4_GPU	0xdc

/* voltage adc channels */
#define	SYS_ADC_CH4_VSYS	0xe0
#define	SYS_ADC_CH5_VA57	0xe4
#define	SYS_ADC_CH6_VA53	0xe8
#define	SYS_ADC_CH7_VGPU	0xec

/* power adc channels */
#define	SYS_EN_CH04_SYS		0xf0
#define	SYS_EN_CH15_A57		0xf4
#define	SYS_EN_CH26_A53		0xf8
#define	SYS_EN_CH37_GPU		0xfc

/* energy adc channels */
#define SYS_ENM_CH0_L_SYS	0x100
#define SYS_ENM_CH0_H_SYS	0x104
#define SYS_ENM_CH1_L_A57	0x108
#define SYS_ENM_CH1_H_A57	0x10c
#define SYS_ENM_CH0_L_A53	0x110
#define SYS_ENM_CH0_H_A53	0x114
#define SYS_ENM_CH0_L_GPU	0x118
#define SYS_ENM_CH0_H_GPU	0x11c

struct v2m_juno_hwmon_dev v2m_juno_hwmon_curr_sys = {
	.type = &v2m_juno_hwmon_amp,
	.offset = SYS_ADC_CH0_PM1_SYS,
	.name = "sys_curr",
	.mult = 1000,
	.div = 761
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_curr_a57 = {
	.type = &v2m_juno_hwmon_amp,
	.offset = SYS_ADC_CH1_PM2_A57,
	.name = "a57_curr",
	.mult = 1000,
	.div = 381,
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_curr_a53 = {
	.type = &v2m_juno_hwmon_amp,
	.offset = SYS_ADC_CH2_PM3_A53,
	.name = "a53_curr",
	.mult = 1000,
	.div = 761
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_curr_gpu = {
	.type = &v2m_juno_hwmon_amp,
	.offset = SYS_ADC_CH3_PM4_GPU,
	.name = "gpu_curr",
	.mult = 1000,
	.div = 381
};

#if !defined(CONFIG_REGULATOR_VEXPRESS)
struct v2m_juno_hwmon_dev v2m_juno_hwmon_volt_sys = {
	.type = &v2m_juno_hwmon_volt,
	.offset = SYS_ADC_CH4_VSYS,
	.name = "sys_volt",
	.mult = 1000,
	.div = 1622
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_volt_a57 = {
	.type = &v2m_juno_hwmon_volt,
	.offset = SYS_ADC_CH5_VA57,
	.name = "a57_volt",
	.mult = 1000,
	.div = 1622
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_volt_a53 = {
	.type = &v2m_juno_hwmon_volt,
	.offset = SYS_ADC_CH6_VA53,
	.name = "a53_volt",
	.mult = 1000,
	.div = 1622
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_volt_gpu = {
	.type = &v2m_juno_hwmon_volt,
	.offset = SYS_ADC_CH7_VGPU,
	.name = "gpu_volt",
	.mult = 1000,
	.div = 1622
};
#endif

struct v2m_juno_hwmon_dev v2m_juno_hwmon_power_sys = {
	.type = &v2m_juno_hwmon_power,
	.offset = SYS_EN_CH04_SYS,
	.name = "sys_power",
	.mult = 1000,
	.div = 1234803

};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_power_a57 = {
	.type = &v2m_juno_hwmon_power,
	.offset = SYS_EN_CH15_A57,
	.name = "a57_power",
	.mult = 1000,
	.div = 617402
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_power_a53 = {
	.type = &v2m_juno_hwmon_power,
	.offset = SYS_EN_CH26_A53,
	.name = "a53_power",
	.mult = 1000,
	.div = 1234803,
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_power_gpu = {
	.type = &v2m_juno_hwmon_power,
	.offset = SYS_EN_CH37_GPU,
	.mult = 1000,
	.name = "gpu_power",
	.div = 617402
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_energy_sys = {
	.type = &v2m_juno_hwmon_energy,
	.offset = SYS_ENM_CH0_L_SYS,
	.name = "sys_energy",
	.mult = 100,
	.div = 1234803,
};
struct v2m_juno_hwmon_dev v2m_juno_hwmon_energy_a57 = {
	.type = &v2m_juno_hwmon_energy,
	.offset = SYS_ENM_CH1_L_A57,
	.name = "a57_energy",
	.mult = 100,
	.div = 617402
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_energy_a53 = {
	.type = &v2m_juno_hwmon_energy,
	.offset = SYS_ENM_CH0_L_A53,
	.name = "a53_energy",
	.mult = 100,
	.div = 1234803,
};

struct v2m_juno_hwmon_dev v2m_juno_hwmon_energy_gpu = {
	.type = &v2m_juno_hwmon_energy,
	.offset = SYS_ENM_CH0_L_GPU,
	.name = "gpu_energy",
	.mult = 100,
	.div = 617402
};

static struct v2m_juno_hwmon_dev *v2m_juno_hwmon_devices[] = {
#if !defined(CONFIG_REGULATOR_VEXPRESS)
	&v2m_juno_hwmon_volt_sys,
	&v2m_juno_hwmon_volt_a57,
	&v2m_juno_hwmon_volt_a53,
	&v2m_juno_hwmon_volt_gpu,
#endif
	&v2m_juno_hwmon_energy_sys,
	&v2m_juno_hwmon_energy_a57,
	&v2m_juno_hwmon_energy_a53,
	&v2m_juno_hwmon_energy_gpu,
	&v2m_juno_hwmon_power_sys,
	&v2m_juno_hwmon_power_a57,
	&v2m_juno_hwmon_power_a53,
	&v2m_juno_hwmon_power_gpu,
	&v2m_juno_hwmon_curr_sys,
	&v2m_juno_hwmon_curr_a57,
	&v2m_juno_hwmon_curr_a53,
	&v2m_juno_hwmon_curr_gpu,
};

static struct of_device_id v2m_juno_hwmon_of_match[] = {
	{
		.compatible = "arm,v2m-juno-meters",
	},
	{}
};
MODULE_DEVICE_TABLE(of, v2m_juno_hwmon_of_match);

static int v2m_juno_hwmon_probe(struct platform_device *pdev)
{
	int i, err;
	const struct of_device_id *match;
	struct v2m_juno_hwmon_dev *dev;
	struct resource *mem;

	match = of_match_device(v2m_juno_hwmon_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem);

	for (i = 0; i < ARRAY_SIZE(v2m_juno_hwmon_devices); i++) {
		dev = v2m_juno_hwmon_devices[i];
		dev->hwmon_dev = hwmon_device_register(&pdev->dev);

		if (IS_ERR(dev->hwmon_dev)) {
			err = PTR_ERR(dev->hwmon_dev);
			goto error;
		}
		err = sysfs_create_group(&dev->hwmon_dev->kobj,
					 dev->type->attr_groups);
		if (err)
			goto error_sysfs;

		dev_set_drvdata(dev->hwmon_dev, dev);
	}

	return 0;

error:
	while (--i >= 0) {
		dev = v2m_juno_hwmon_devices[i];
		sysfs_remove_group(&dev->hwmon_dev->kobj,
					dev->type->attr_groups);
error_sysfs:
		hwmon_device_unregister(dev->hwmon_dev);
	}

	return err;
}

static int v2m_juno_hwmon_remove(struct platform_device *pdev)
{
	int i;
	struct v2m_juno_hwmon_dev *dev;

	for (i = 0; i < ARRAY_SIZE(v2m_juno_hwmon_devices); i++) {
		dev = v2m_juno_hwmon_devices[i];
		hwmon_device_unregister(dev->hwmon_dev);
		sysfs_remove_group(&dev->hwmon_dev->kobj,
				   dev->type->attr_groups);
	}

	return 0;
}

static struct platform_driver v2m_juno_hwmon_driver = {
	.probe = v2m_juno_hwmon_probe,
	.remove = v2m_juno_hwmon_remove,
	.driver	= {
		.name = DRVNAME,
		.owner = THIS_MODULE,
		.of_match_table = v2m_juno_hwmon_of_match,
	},
};

module_platform_driver(v2m_juno_hwmon_driver);

MODULE_AUTHOR("Lorenzo Pieralisi <lorenzo.pieralisi@arm.com>");
MODULE_DESCRIPTION("V2M Juno hwmon sensors driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:v2m-juno-hwmon");
