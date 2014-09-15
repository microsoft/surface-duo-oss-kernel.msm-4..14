/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <dt-bindings/pinctrl/qcom,pmic-mpp.h>
#include <dt-bindings/pinctrl/qcom,pmic-gpio.h>

#include "../core.h"
#include "../pinctrl-utils.h"

/*
 * Mode select - indicates whether the pin should be input, output, or both
 * for GPIOs. MPP pins also support bidirectional, analog input, analog output
 * and current sink.
 */
#define QPNP_PIN_MODE_DIG_IN			0
#define QPNP_PIN_MODE_DIG_OUT			1
#define QPNP_PIN_MODE_DIG_IN_OUT		2
#define QPNP_PIN_MODE_BIDIR			3
#define QPNP_PIN_MODE_AIN			4
#define QPNP_PIN_MODE_AOUT			5
#define QPNP_PIN_MODE_SINK			6

/*
 * Voltage select (GPIO, MPP) - specifies the voltage level when the output
 * is set to 1. For an input GPIO specifies the voltage level at which
 * the input is interpreted as a logical 1
 * To be used with "power-func = <>"
 */
#define QPNP_PIN_VIN_4CH_INVALID		5
#define QPNP_PIN_VIN_8CH_INVALID		8

/*
 * Analog Output - Set the analog output reference.
 * See PM8XXX_MPP_AOUT_XXX. To be used with "qcom,aout = <>"
 */
#define QPNP_MPP_AOUT_INVALID			8

/*
 * Analog Input - Set the func for analog input.
 * See PM8XXX_MPP_AIN_XXX. To be used with "qcom,ain = <>"
 */
#define QPNP_MPP_AIN_INVALID			8

/*
 * Output type - indicates pin should be configured as CMOS or
 * open drain.
 */
#define QPNP_GPIO_OUT_BUF_CMOS			0
#define QPNP_GPIO_OUT_BUF_OPEN_DRAIN_NMOS	1
#define QPNP_GPIO_OUT_BUF_OPEN_DRAIN_PMOS	2

/*
 * Pull Up Values - it indicates whether a pull up or pull down
 * should be applied. If a pull-up is required the current strength needs
 * to be specified. Current values of 30uA, 1.5uA, 31.5uA, 1.5uA with 30uA
 * boost are supported.
 * Note that the hardware ignores this configuration if the GPIO is not set
 * to input or output open-drain mode.
 */
#define QPNP_GPIO_PULL_UP_30			0
#define QPNP_GPIO_PULL_UP_1P5			1
#define QPNP_GPIO_PULL_UP_31P5			2
#define QPNP_GPIO_PULL_UP_1P5_30		3
#define QPNP_GPIO_PULL_DN			4
#define QPNP_GPIO_PULL_NO			5

/*
 * Pull Up Values - it indicates whether a pull-up should be
 * applied for bidirectional mode only. The hardware ignores the
 * configuration when operating in other modes.
 */
#define QPNP_MPP_PULL_UP_0P6KOHM		0
#define QPNP_MPP_PULL_UP_10KOHM			1
#define QPNP_MPP_PULL_UP_30KOHM			2
#define QPNP_MPP_PULL_UP_OPEN			3

/* Out Strength (GPIO) - the amount of current supplied for an output GPIO */
#define QPNP_GPIO_STRENGTH_LOW			1
#define QPNP_GPIO_STRENGTH_MED			2
#define QPNP_GPIO_STRENGTH_HIGH			3

/*
 * Master enable (GPIO, MPP) - Enable features within the pin block based on
 * configurations. QPNP_PIN_MASTER_DISABLE = Completely disable the pin
 * lock and let the pin float with high impedance regardless of other settings.
 */
#define QPNP_PIN_MASTER_DISABLE                 0
#define QPNP_PIN_MASTER_ENABLE			1

/* revision registers base address offsets */
#define QPNP_REG_DIG_MINOR_REV			0x0
#define QPNP_REG_DIG_MAJOR_REV			0x1
#define QPNP_REG_ANA_MINOR_REV			0x2

/* type registers base address offsets */
#define QPNP_REG_TYPE				0x4
#define QPNP_REG_SUBTYPE			0x5

/* GPIO peripheral type and subtype values */
#define QPNP_GPIO_TYPE				0x10
#define QPNP_GPIO_SUBTYPE_GPIO_4CH		0x1
#define QPNP_GPIO_SUBTYPE_GPIOC_4CH		0x5
#define QPNP_GPIO_SUBTYPE_GPIO_8CH		0x9
#define QPNP_GPIO_SUBTYPE_GPIOC_8CH		0xd

/* mpp peripheral type and subtype values */
#define QPNP_MPP_TYPE				0x11
#define QPNP_MPP_SUBTYPE_4CH_NO_ANA_OUT		0x3
#define QPNP_MPP_SUBTYPE_ULT_4CH_NO_ANA_OUT	0x4
#define QPNP_MPP_SUBTYPE_4CH_NO_SINK		0x5
#define QPNP_MPP_SUBTYPE_ULT_4CH_NO_SINK	0x6
#define QPNP_MPP_SUBTYPE_4CH_FULL_FUNC		0x7
#define QPNP_MPP_SUBTYPE_8CH_FULL_FUNC		0xf

#define QPNP_REG_STATUS1			0x8
#define QPNP_REG_STATUS1_VAL_MASK		0x1
#define QPNP_REG_STATUS1_GPIO_EN_REV0_MASK	0x2
#define QPNP_REG_STATUS1_GPIO_EN_MASK		0x80
#define QPNP_REG_STATUS1_MPP_EN_MASK		0x80

/* control register base address offsets */
#define QPNP_REG_MODE_CTL			0x40
#define QPNP_REG_DIG_VIN_CTL			0x41
#define QPNP_REG_DIG_PULL_CTL			0x42
#define QPNP_REG_DIG_IN_CTL			0x43
#define QPNP_REG_DIG_OUT_CTL			0x45
#define QPNP_REG_EN_CTL				0x46
#define QPNP_REG_AOUT_CTL			0x4b
#define QPNP_REG_AIN_CTL			0x4a
#define QPNP_REG_SINK_CTL			0x4c

/* QPNP_REG_MODE_CTL */
#define QPNP_REG_MODE_INVERT_SHIFT		0
#define QPNP_REG_MODE_INVERT_MASK		0x1
#define QPNP_REG_MODE_FUNCTION_SHIFT		1
#define QPNP_REG_MODE_FUNCTION_MASK		0x6
#define QPNP_REG_MODE_SEL_SHIFT			4
#define QPNP_REG_MODE_SEL_MASK			0x70

/* QPNP_REG_DIG_VIN_CTL */
#define QPNP_REG_VIN_SHIFT			0
#define QPNP_REG_VIN_MASK			0x7

/* QPNP_REG_DIG_PULL_CTL */
#define QPNP_REG_PULL_SHIFT			0
#define QPNP_REG_PULL_MASK			0x7

/* QPNP_REG_DIG_OUT_CTL */
#define QPNP_REG_OUT_STRENGTH_SHIFT		0
#define QPNP_REG_OUT_STRENGTH_MASK		0x3
#define QPNP_REG_OUT_TYPE_SHIFT			4
#define QPNP_REG_OUT_TYPE_MASK			0x30

/* QPNP_REG_EN_CTL */
#define QPNP_REG_MASTER_EN_SHIFT		7
#define QPNP_REG_MASTER_EN_MASK			0x80

/* QPNP_REG_AOUT_CTL */
#define QPNP_REG_AOUT_REF_SHIFT			0
#define QPNP_REG_AOUT_REF_MASK			0x7

/* QPNP_REG_AIN_CTL */
#define QPNP_REG_AIN_ROUTE_SHIFT		0
#define QPNP_REG_AIN_ROUTE_MASK			0x7

/* QPNP_REG_SINK_CTL */
#define QPNP_REG_CS_OUT_SHIFT			0
#define QPNP_REG_CS_OUT_MASK			0x7

#define QPNP_PIN_PHYSICAL_OFFSET		1

/* Qualcomm specific pin configurations */
#define QPNP_PINCONF_PULL_UP			(PIN_CONFIG_END + 1)
#define QPNP_PINCONF_STRENGTH			(PIN_CONFIG_END + 2)
#define QPNP_PINCONF_AMUX_ROUTE			(PIN_CONFIG_END + 3)
#define QPNP_PINCONF_VREFENCE			(PIN_CONFIG_END + 4)
#define QPNP_PINCONF_MODE			(PIN_CONFIG_END + 5)

struct qpnp_chipinfo {
	unsigned npins;
	unsigned base;
	unsigned type;
};

struct qpnp_padinfo {
	u16 offset;		/* address offset in SPMI device */
	int irq;
	const char *name;	/* pin name */
	unsigned int modes;	/* supported modes: DI, DO, DIO, AI, AO... */
	unsigned int type;	/* peripheral hardware type */
	unsigned int subtype;	/* peripheral hardware subtype */
	unsigned int major;	/* digital major version */
};

#define QPNP_REG_ADDR(pad, reg) ((pad)->offset + reg)
#define QPNP_GET(buff, shift, mask) ((buff & mask) >> shift)

struct qpnp_pinctrl {
	struct device *dev;
	struct regmap *map;
	struct pinctrl_dev *ctrl;
	struct pinctrl_desc desc;
	struct gpio_chip chip;

	struct qpnp_padinfo *pads;
	const struct qpnp_chipinfo *info;
	const char *const *groups;
	const char *const *functions;
};

static inline struct qpnp_pinctrl *to_qpnp_pinctrl(struct gpio_chip *chip)
{
	return container_of(chip, struct qpnp_pinctrl, chip);
};

struct qpnp_pinbindings {
	const char *property;
	unsigned param;
	u32 default_value;
};

struct qpnp_pinattrib {
	unsigned addr;
	unsigned shift;
	unsigned mask;
	unsigned val;
};

static struct qpnp_pinbindings qpnp_pinbindings[] = {
	/* QCOM_BIAS_PULL_UP_30...  */
	{"qcom,pull-up-strength", QPNP_PINCONF_PULL_UP, 0},
	/* QCOM_DRIVE_STRENGTH_NO... */
	{"qcom,drive-strength",	QPNP_PINCONF_STRENGTH, 0},
	/* PMIC_MPP_AMUX_ROUTE_CH5 ... */
	{"qcom,amux-route",	QPNP_PINCONF_AMUX_ROUTE, 0},
	/* PMIC_MPP_VREFERENCE_1V25 ... */
	{"qcom,vrefence",	QPNP_PINCONF_VREFENCE, 0},
	/* PMIC_MPP_MODE.. */
	{"qcom,mode",	        QPNP_PINCONF_MODE, 0},
};

static const char * const qpnp_gpio_groups[] = {
	"gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7", "gpio8",
	"gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14", "gpio15",
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21", "gpio22",
	"gpio23", "gpio24", "gpio25", "gpio26", "gpio27", "gpio28", "gpio29",
	"gpio30", "gpio31", "gpio32", "gpio33", "gpio34", "gpio35", "gpio36",
};

static const char * const qpnp_mpp_groups[] = {
	"mpp1", "mpp2", "mpp3", "mpp4", "mpp5", "mpp6", "mpp7", "mpp8",
};

static const char * const qpnp_gpio_functions[] = {
	PMIC_GPIO_FUNC_NORMAL, PMIC_GPIO_FUNC_PAIRED,
	PMIC_GPIO_FUNC_FUNC1, PMIC_GPIO_FUNC_FUNC2,
	PMIC_GPIO_FUNC_DTEST1, PMIC_GPIO_FUNC_DTEST2,
	PMIC_GPIO_FUNC_DTEST3, PMIC_GPIO_FUNC_DTEST4,
};

static const char * const qpnp_mpp_functions[] = {
	PMIC_MPP_FUNC_NORMAL, PMIC_MPP_FUNC_PAIRED,
	"reserved1", "reserved2",
	PMIC_MPP_FUNC_DTEST1, PMIC_MPP_FUNC_DTEST2,
	PMIC_MPP_FUNC_DTEST3, PMIC_MPP_FUNC_DTEST4,
};

static inline struct qpnp_padinfo *qpnp_get_desc(struct qpnp_pinctrl *qctrl,
						 unsigned pin)
{
	if (pin >= qctrl->desc.npins) {
		dev_warn(qctrl->dev, "invalid pin number %d", pin);
		return NULL;
	}

	return &qctrl->pads[pin];
}

static int qpnp_conv_to_pin(struct qpnp_pinctrl *qctrl,
			   struct qpnp_padinfo *pad, unsigned param,
			   unsigned val)
{
	struct qpnp_pinattrib attr[3];
	unsigned int type, subtype;
	int nattrs = 1, idx, ret;

	type = pad->type;
	subtype = pad->subtype;

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		attr[0].addr  = QPNP_REG_DIG_OUT_CTL;
		attr[0].shift = QPNP_REG_OUT_TYPE_SHIFT;
		attr[0].mask  = QPNP_REG_OUT_TYPE_MASK;
		attr[0].val   = QPNP_GPIO_OUT_BUF_CMOS;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		if (subtype == QPNP_GPIO_SUBTYPE_GPIOC_4CH ||
		    subtype == QPNP_GPIO_SUBTYPE_GPIOC_8CH)
			return -EINVAL;
		attr[0].addr  = QPNP_REG_DIG_OUT_CTL;
		attr[0].shift = QPNP_REG_OUT_TYPE_SHIFT;
		attr[0].mask  = QPNP_REG_OUT_TYPE_MASK;
		attr[0].val   = QPNP_GPIO_OUT_BUF_OPEN_DRAIN_NMOS;
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		if (subtype == QPNP_GPIO_SUBTYPE_GPIOC_4CH ||
		    subtype == QPNP_GPIO_SUBTYPE_GPIOC_8CH)
			return -EINVAL;
		attr[0].addr  = QPNP_REG_DIG_OUT_CTL;
		attr[0].shift = QPNP_REG_OUT_TYPE_SHIFT;
		attr[0].mask  = QPNP_REG_OUT_TYPE_MASK;
		attr[0].val   = QPNP_GPIO_OUT_BUF_OPEN_DRAIN_PMOS;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		attr[0].addr  = QPNP_REG_DIG_PULL_CTL;
		attr[0].shift = QPNP_REG_PULL_SHIFT;
		attr[0].mask  = QPNP_REG_PULL_MASK;
		if (type == QPNP_GPIO_TYPE)
			attr[0].val = QPNP_GPIO_PULL_NO;
		else
			attr[0].val = QPNP_MPP_PULL_UP_OPEN;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (type != QPNP_MPP_TYPE)
			return -EINVAL;
		switch (val) {
		case 0:
			val = QPNP_MPP_PULL_UP_OPEN;
			break;
		case 600:
			val = QPNP_MPP_PULL_UP_0P6KOHM;
			break;
		case 10000:
			val = QPNP_MPP_PULL_UP_10KOHM;
			break;
		case 30000:
			val = QPNP_MPP_PULL_UP_30KOHM;
			break;
		default:
			return -EINVAL;
		}
		attr[0].addr  = QPNP_REG_DIG_PULL_CTL;
		attr[0].shift = QPNP_REG_PULL_SHIFT;
		attr[0].mask  = QPNP_REG_PULL_MASK;
		attr[0].val   = val;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (type != QPNP_GPIO_TYPE)
			return -EINVAL;
		attr[0].addr  = QPNP_REG_DIG_PULL_CTL;
		attr[0].shift = QPNP_REG_PULL_SHIFT;
		attr[0].mask  = QPNP_REG_PULL_MASK;
		if (val)
			attr[0].val = QPNP_GPIO_PULL_DN;
		else
			attr[0].val = QPNP_GPIO_PULL_NO;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		if (val >= QPNP_PIN_VIN_8CH_INVALID)
			return -EINVAL;
		if (val >= QPNP_PIN_VIN_4CH_INVALID) {
			if (type == QPNP_GPIO_TYPE &&
			   (subtype == QPNP_GPIO_SUBTYPE_GPIO_4CH ||
			    subtype == QPNP_GPIO_SUBTYPE_GPIOC_4CH))
				return -EINVAL;
			if (type == QPNP_MPP_TYPE &&
			   (subtype == QPNP_MPP_SUBTYPE_4CH_NO_ANA_OUT ||
			    subtype == QPNP_MPP_SUBTYPE_4CH_NO_SINK ||
			    subtype == QPNP_MPP_SUBTYPE_4CH_FULL_FUNC ||
			    subtype == QPNP_MPP_SUBTYPE_ULT_4CH_NO_ANA_OUT ||
			    subtype == QPNP_MPP_SUBTYPE_ULT_4CH_NO_SINK))
				return -EINVAL;
		}
		attr[0].addr  = QPNP_REG_DIG_VIN_CTL;
		attr[0].shift = QPNP_REG_VIN_SHIFT;
		attr[0].mask  = QPNP_REG_VIN_MASK;
		attr[0].val   = val;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		if (type != QPNP_MPP_TYPE)
			return -EINVAL;
		if (subtype == QPNP_MPP_SUBTYPE_4CH_NO_SINK ||
		    subtype == QPNP_MPP_SUBTYPE_ULT_4CH_NO_SINK)
			return -ENXIO;
		if (val > 50)	/* mA */
			return -EINVAL;
		attr[0].addr  = QPNP_REG_SINK_CTL;
		attr[0].shift = QPNP_REG_CS_OUT_SHIFT;
		attr[0].mask  = QPNP_REG_CS_OUT_MASK;
		attr[0].val   = (val / 5) - 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		nattrs = 2;
		attr[0].addr  = QPNP_REG_MODE_CTL;
		attr[0].shift = QPNP_REG_MODE_SEL_SHIFT;
		attr[0].mask  = QPNP_REG_MODE_SEL_MASK;
		attr[0].val   = QPNP_PIN_MODE_DIG_IN;
		attr[1].addr  = QPNP_REG_EN_CTL;
		attr[1].shift = QPNP_REG_MASTER_EN_SHIFT;
		attr[1].mask  = QPNP_REG_MASTER_EN_MASK;
		attr[1].val   = 1;
		if (val)
			break;
	/* Fallthrough */
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		attr[1].addr  = QPNP_REG_EN_CTL;
		attr[1].shift = QPNP_REG_MASTER_EN_SHIFT;
		attr[1].mask  = QPNP_REG_MASTER_EN_MASK;
		attr[1].val   = 0;
		break;
	case PIN_CONFIG_OUTPUT:
		nattrs = 3;
		attr[0].addr  = QPNP_REG_MODE_CTL;
		attr[0].shift = QPNP_REG_MODE_INVERT_SHIFT;
		attr[0].mask  = QPNP_REG_MODE_INVERT_MASK;
		attr[0].val   = !!val;
		attr[1].addr  = QPNP_REG_MODE_CTL;
		attr[1].shift = QPNP_REG_MODE_SEL_SHIFT;
		attr[1].mask  = QPNP_REG_MODE_SEL_MASK;
		attr[1].val   = QPNP_PIN_MODE_DIG_OUT;
		attr[2].addr  = QPNP_REG_EN_CTL;
		attr[2].shift = QPNP_REG_MASTER_EN_SHIFT;
		attr[2].mask  = QPNP_REG_MASTER_EN_MASK;
		attr[2].val   = 1;
		break;
	case QPNP_PINCONF_PULL_UP:
		if (type != QPNP_GPIO_TYPE)
			return -EINVAL;
		switch (val) {
		default:
			return -EINVAL;
		case 0:
			val = QPNP_GPIO_PULL_NO;
			break;
		case PMIC_GPIO_PULL_UP_30:
			val = QPNP_GPIO_PULL_UP_30;
			break;
		case PMIC_GPIO_PULL_UP_1P5:
			val = QPNP_GPIO_PULL_UP_1P5;
			break;
		case PMIC_GPIO_PULL_UP_31P5:
			val = QPNP_GPIO_PULL_UP_31P5;
			break;
		case PMIC_GPIO_PULL_UP_1P5_30:
			val = QPNP_GPIO_PULL_UP_1P5_30;
			break;
		}
		attr[0].addr  = QPNP_REG_DIG_PULL_CTL;
		attr[0].shift = QPNP_REG_PULL_SHIFT;
		attr[0].mask  = QPNP_REG_PULL_MASK;
		attr[0].val   = val;
		break;
	case QPNP_PINCONF_STRENGTH:
		if (type != QPNP_GPIO_TYPE)
			return -EINVAL;
		switch (val) {
		default:
		case PMIC_GPIO_STRENGTH_NO:
			return -EINVAL;
		case PMIC_GPIO_STRENGTH_LOW:
			attr[0].val = QPNP_GPIO_STRENGTH_LOW;
			break;
		case PMIC_GPIO_STRENGTH_MED:
			attr[0].val = QPNP_GPIO_STRENGTH_MED;
			break;
		case PMIC_GPIO_STRENGTH_HIGH:
			attr[0].val = QPNP_GPIO_STRENGTH_HIGH;
			break;
		}
		attr[0].addr  = QPNP_REG_DIG_OUT_CTL;
		attr[0].shift = QPNP_REG_OUT_STRENGTH_SHIFT;
		attr[0].mask  = QPNP_REG_OUT_STRENGTH_MASK;
		break;
	case QPNP_PINCONF_VREFENCE:
		if (type != QPNP_MPP_TYPE)
			return -ENXIO;
		if (val >= QPNP_MPP_AOUT_INVALID)
			return -EINVAL;
		if (subtype == QPNP_MPP_SUBTYPE_4CH_NO_ANA_OUT ||
		    subtype == QPNP_MPP_SUBTYPE_ULT_4CH_NO_ANA_OUT)
			return -ENXIO;
		attr[0].addr  = QPNP_REG_AOUT_CTL;
		attr[0].shift = QPNP_REG_AOUT_REF_SHIFT;
		attr[0].mask  = QPNP_REG_AOUT_REF_MASK;
		attr[0].val   = val;
		break;
	case QPNP_PINCONF_AMUX_ROUTE:
		if (type != QPNP_MPP_TYPE)
			return -ENXIO;
		if (val >= QPNP_MPP_AIN_INVALID)
			return -EINVAL;
		attr[0].addr  = QPNP_REG_AIN_CTL;
		attr[0].shift = QPNP_REG_AIN_ROUTE_SHIFT;
		attr[0].mask  = QPNP_REG_AIN_ROUTE_MASK;
		attr[0].val   = val;
		break;
	case QPNP_PINCONF_MODE:
		if ((pad->modes & BIT(val)) == 0)
			return -ENXIO;
		attr[0].addr  = QPNP_REG_MODE_CTL;
		attr[0].shift = QPNP_REG_MODE_SEL_SHIFT;
		attr[0].mask  = QPNP_REG_MODE_SEL_MASK;
		attr[0].val   = val;
		break;
	default:
		return -EINVAL;
	}

	for (idx = 0; idx < nattrs; idx++) {
		/* add base offset */
		attr[idx].addr = QPNP_REG_ADDR(pad, attr[idx].addr);
		ret = regmap_update_bits(qctrl->map, attr[idx].addr,
					 attr[idx].mask,
					 attr[idx].val << attr[idx].shift);
		if (ret < 0)
			return ret;
	}

	return 0;
}


static int qpnp_conv_from_pin(struct qpnp_pinctrl *qctrl,
			     struct qpnp_padinfo *pad,
			     unsigned param, unsigned *val)
{
	struct qpnp_pinattrib attr;
	unsigned int type, subtype, field;
	unsigned int addr, buff;
	int ret;

	*val = 0;
	type = pad->type;
	subtype = pad->subtype;

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		attr.addr  = QPNP_REG_DIG_OUT_CTL;
		attr.shift = QPNP_REG_OUT_TYPE_SHIFT;
		attr.mask  = QPNP_REG_OUT_TYPE_MASK;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
	/* Fallthrough */
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP:
		attr.addr  = QPNP_REG_DIG_PULL_CTL;
		attr.shift = QPNP_REG_PULL_SHIFT;
		attr.mask  = QPNP_REG_PULL_MASK;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		attr.addr  = QPNP_REG_EN_CTL;
		attr.shift = QPNP_REG_MASTER_EN_SHIFT;
		attr.mask  = QPNP_REG_MASTER_EN_MASK;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		attr.addr  = QPNP_REG_DIG_VIN_CTL;
		attr.shift = QPNP_REG_VIN_SHIFT;
		attr.mask  = QPNP_REG_VIN_MASK;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		if (type != QPNP_MPP_TYPE)
			return -ENXIO;
		attr.addr  = QPNP_REG_SINK_CTL;
		attr.shift = QPNP_REG_CS_OUT_SHIFT;
		attr.mask  = QPNP_REG_CS_OUT_MASK;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		attr.addr  = QPNP_REG_EN_CTL;
		attr.shift = QPNP_REG_MASTER_EN_SHIFT;
		attr.mask  = QPNP_REG_MASTER_EN_MASK;
		break;
	case PIN_CONFIG_OUTPUT:
		attr.addr  = QPNP_REG_MODE_CTL;
		attr.shift = QPNP_REG_MODE_INVERT_SHIFT;
		attr.mask  = QPNP_REG_MODE_INVERT_MASK;
		break;
	case QPNP_PINCONF_PULL_UP:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		attr.addr  = QPNP_REG_DIG_PULL_CTL;
		attr.shift = QPNP_REG_PULL_SHIFT;
		attr.mask  = QPNP_REG_PULL_MASK;
		break;
	case QPNP_PINCONF_STRENGTH:
		if (type != QPNP_GPIO_TYPE)
			return -ENXIO;
		attr.addr  = QPNP_REG_DIG_OUT_CTL;
		attr.shift = QPNP_REG_OUT_STRENGTH_SHIFT;
		attr.mask  = QPNP_REG_OUT_STRENGTH_MASK;
		break;
	case QPNP_PINCONF_VREFENCE:
		if (type != QPNP_MPP_TYPE)
			return -ENXIO;
		if (subtype == QPNP_MPP_SUBTYPE_4CH_NO_ANA_OUT ||
		    subtype == QPNP_MPP_SUBTYPE_ULT_4CH_NO_ANA_OUT)
			return -ENXIO;
		attr.addr  = QPNP_REG_AOUT_CTL;
		attr.shift = QPNP_REG_AOUT_REF_SHIFT;
		attr.mask  = QPNP_REG_AOUT_REF_MASK;
		break;
	case QPNP_PINCONF_AMUX_ROUTE:
		if (type != QPNP_MPP_TYPE)
			return -ENXIO;
		attr.addr  = QPNP_REG_AIN_CTL;
		attr.shift = QPNP_REG_AIN_ROUTE_SHIFT;
		attr.mask  = QPNP_REG_AIN_ROUTE_MASK;
		break;
	case QPNP_PINCONF_MODE:
		attr.addr  = QPNP_REG_MODE_CTL;
		attr.shift = QPNP_REG_MODE_SEL_SHIFT;
		attr.mask  = QPNP_REG_MODE_SEL_MASK;
		break;
	default:
		return -EINVAL;
	}

	addr = QPNP_REG_ADDR(pad, attr.addr);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0)
		return ret;

	field = QPNP_GET(buff, attr.shift, attr.mask);

	switch (param) {
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		if (field == QPNP_GPIO_OUT_BUF_CMOS)
			*val = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (field == QPNP_GPIO_OUT_BUF_OPEN_DRAIN_NMOS)
			*val = 1;
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
		if (field == QPNP_GPIO_OUT_BUF_OPEN_DRAIN_PMOS)
			*val = 1;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		if (type == QPNP_GPIO_TYPE) {
			if (field == QPNP_GPIO_PULL_NO)
				*val = 1;
		} else {
			if (field == QPNP_MPP_PULL_UP_OPEN)
				*val = 1;
		}
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		switch (field) {
		default:
		case QPNP_MPP_PULL_UP_OPEN:
			*val = 0;
			break;
		case QPNP_MPP_PULL_UP_0P6KOHM:
			*val = 600;
			break;
		case QPNP_MPP_PULL_UP_10KOHM:
			*val = 10000;
			break;
		case QPNP_MPP_PULL_UP_30KOHM:
			*val = 30000;
			break;
		}
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (field == QPNP_GPIO_PULL_DN)
			*val = 1;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		*val = field;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		*val = (field + 1) * 5;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		*val = field;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		if (field == QPNP_PIN_MASTER_DISABLE)
			*val = 1;
		break;
	case PIN_CONFIG_OUTPUT:
		*val = field;
		break;
	case QPNP_PINCONF_PULL_UP:
		switch (field) {
		case QPNP_GPIO_PULL_NO:
			field = 0;
			break;
		case QPNP_GPIO_PULL_UP_30:
			field = PMIC_GPIO_PULL_UP_30;
			break;
		case QPNP_GPIO_PULL_UP_1P5:
			field = PMIC_GPIO_PULL_UP_1P5;
			break;
		case QPNP_GPIO_PULL_UP_31P5:
			field = PMIC_GPIO_PULL_UP_31P5;
			break;

		case QPNP_GPIO_PULL_UP_1P5_30:
			field = PMIC_GPIO_PULL_UP_1P5_30;
			break;
		}
		*val = field;
		break;
	case QPNP_PINCONF_STRENGTH:
		switch (field) {
		case QPNP_GPIO_STRENGTH_HIGH:
			field = PMIC_GPIO_STRENGTH_HIGH;
			break;
		case QPNP_GPIO_STRENGTH_MED:
			field = PMIC_GPIO_STRENGTH_MED;
			break;
		case QPNP_GPIO_STRENGTH_LOW:
			field = PMIC_GPIO_STRENGTH_LOW;
			break;
		}
		*val = field;
		break;
	case QPNP_PINCONF_MODE:
	case QPNP_PINCONF_VREFENCE:
	case QPNP_PINCONF_AMUX_ROUTE:
		*val = field;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int qpnp_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct qpnp_pinctrl *qpctrl = pinctrl_dev_get_drvdata(pctldev);

	/* Every PIN is a group */
	return qpctrl->desc.npins;
}

static const char *qpnp_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned pin)
{
	struct qpnp_pinctrl *qpctrl = pinctrl_dev_get_drvdata(pctldev);

	/* Every PIN is a group */
	return qpctrl->desc.pins[pin].name;
}

static int qpnp_get_group_pins(struct pinctrl_dev *pctldev,
			      unsigned pin,
			      const unsigned **pins,
			      unsigned *num_pins)
{
	struct qpnp_pinctrl *qpctrl = pinctrl_dev_get_drvdata(pctldev);

	/* Every PIN is a group */
	*pins = &qpctrl->desc.pins[pin].number;
	*num_pins = 1;
	return 0;
}

static int qpnp_parse_dt_config(struct device_node *np,
			      struct pinctrl_dev *pctldev,
			      unsigned long **configs, unsigned int *nconfs)
{
	struct qpnp_pinbindings *par;
	unsigned long cfg;
	int ret, idx;
	u32 val;

	for (idx = 0; idx < ARRAY_SIZE(qpnp_pinbindings); idx++) {

		par = &qpnp_pinbindings[idx];
		ret = of_property_read_u32(np, par->property, &val);

		/* property not found */
		if (ret == -EINVAL)
			continue;

		/* use default value, when no value is specified */
		if (ret)
			val = par->default_value;

		dev_dbg(pctldev->dev, "found %s with value %u\n",
			par->property, val);

		cfg = pinconf_to_config_packed(par->param, val);

		ret = pinctrl_utils_add_config(pctldev, configs, nconfs, cfg);
		if (ret)
			return ret;
	}

	return 0;
}

static int qpnp_dt_subnode_to_map(struct pinctrl_dev *pctldev,
				  struct device_node *np,
				  struct pinctrl_map **map,
				  unsigned *reserv, unsigned *nmaps,
				  enum pinctrl_map_type type)
{
	unsigned long *configs = NULL;
	unsigned nconfs = 0;
	struct property *prop;
	const char *group;
	int ret;

	ret = qpnp_parse_dt_config(np, pctldev, &configs, &nconfs);
	if (ret < 0)
		return ret;

	if (!nconfs)
		return 0;

	ret = of_property_count_strings(np, "pins");
	if (ret < 0)
		goto exit;

	ret = pinctrl_utils_reserve_map(pctldev, map, reserv,
					nmaps, ret);
	if (ret < 0)
		goto exit;

	of_property_for_each_string(np, "pins", prop, group) {
		ret = pinctrl_utils_add_map_configs(pctldev, map,
				reserv, nmaps, group, configs, nconfs, type);
		if (ret < 0)
			break;
	}
exit:
	kfree(configs);
	return ret;
}

static int qpnp_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np_config,
			       struct pinctrl_map **map,
			       unsigned *nmaps)
{
	struct device_node *np;
	enum pinctrl_map_type type;
	unsigned reserv;
	int ret;

	ret = 0;
	*map = NULL;
	*nmaps = 0;
	reserv = 0;
	type = PIN_MAP_TYPE_CONFIGS_GROUP;

	for_each_child_of_node(np_config, np) {

		ret = pinconf_generic_dt_subnode_to_map(pctldev, np, map,
							&reserv, nmaps, type);
		if (ret)
			break;

		ret = qpnp_dt_subnode_to_map(pctldev, np, map, &reserv,
					     nmaps, type);
		if (ret)
			break;
	}

	if (ret < 0)
		pinctrl_utils_dt_free_map(pctldev, *map, *nmaps);

	return ret;
}

static const struct pinctrl_ops qpnp_pinctrl_ops = {
	.get_groups_count	= qpnp_get_groups_count,
	.get_group_name		= qpnp_get_group_name,
	.get_group_pins		= qpnp_get_group_pins,
	.dt_node_to_map		= qpnp_dt_node_to_map,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int qpnp_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(qpnp_gpio_functions);
}

static const char *qpnp_get_function_name(struct pinctrl_dev *pctldev,
					 unsigned function)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);

	return qctrl->functions[function];
}

static int qpnp_get_function_groups(struct pinctrl_dev *pctldev,
				  unsigned function,
				  const char *const **groups,
				  unsigned *const num_qgroups)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = qctrl->groups;
	*num_qgroups = qctrl->desc.npins;
	return 0;
}

static int qpnp_pinmux_enable(struct pinctrl_dev *pctldev,
			     unsigned function,
			     unsigned pin)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);
	struct qpnp_padinfo *pad;
	unsigned int addr, val, mask;
	int ret;

	pad = qpnp_get_desc(qctrl, pin);
	if (!pad)
		return -EINVAL;

	addr = QPNP_REG_ADDR(pad, QPNP_REG_MODE_CTL);
	val = function << QPNP_REG_MODE_FUNCTION_SHIFT;
	mask = QPNP_REG_MODE_FUNCTION_MASK;
	ret = regmap_update_bits(qctrl->map, addr, mask, val);
	if (ret)
		return ret;

	addr = QPNP_REG_ADDR(pad, QPNP_REG_EN_CTL);
	val = BIT(QPNP_REG_MASTER_EN_SHIFT);
	mask = QPNP_REG_MASTER_EN_MASK;
	ret = regmap_update_bits(qctrl->map, addr, mask, val);

	return ret;
}

static const struct pinmux_ops qpnp_pinmux_ops = {
	.get_functions_count	= qpnp_get_functions_count,
	.get_function_name	= qpnp_get_function_name,
	.get_function_groups	= qpnp_get_function_groups,
	.enable			= qpnp_pinmux_enable,
};

static int qpnp_get(struct gpio_chip *chip, unsigned offset)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	struct qpnp_padinfo *pad;
	unsigned int val, en_mask, buff, addr;
	int ret;

	pad = qpnp_get_desc(qctrl, offset);
	if (!pad)
		return -EINVAL;

	addr = QPNP_REG_ADDR(pad, QPNP_REG_MODE_CTL);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0)
		return ret;

	/* GPIO val is from RT status if input is enabled */
	if ((buff & QPNP_REG_MODE_SEL_MASK) ==
	    (QPNP_PIN_MODE_DIG_IN << QPNP_REG_MODE_SEL_SHIFT)) {

		addr = QPNP_REG_ADDR(pad, QPNP_REG_STATUS1);
		ret = regmap_read(qctrl->map, addr, &val);
		if (ret < 0)
			return ret;

		if (pad->type == QPNP_GPIO_TYPE && pad->major == 0)
			en_mask = QPNP_REG_STATUS1_GPIO_EN_REV0_MASK;
		else if (pad->type == QPNP_GPIO_TYPE &&
			 pad->major > 0)
			en_mask = QPNP_REG_STATUS1_GPIO_EN_MASK;
		else		/* MPP */
			en_mask = QPNP_REG_STATUS1_MPP_EN_MASK;

		if (!(val & en_mask))
			return -EPERM;

		ret = val & QPNP_REG_STATUS1_VAL_MASK;

	} else {
		ret = buff & QPNP_REG_MODE_INVERT_MASK;
		ret = ret >> QPNP_REG_MODE_INVERT_SHIFT;
	}

	return !!ret;
}

static void qpnp_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	struct qpnp_padinfo *pad;
	unsigned int addr, buff;

	pad = qpnp_get_desc(qctrl, offset);
	if (!pad)
		return;

	addr = QPNP_REG_ADDR(pad, QPNP_REG_MODE_CTL);
	buff = !!value << QPNP_REG_MODE_INVERT_SHIFT;

	regmap_update_bits(qctrl->map, addr, QPNP_REG_MODE_INVERT_MASK, buff);
}

static int qpnp_config_get(struct pinctrl_dev *pctldev,
			  unsigned int pin,
			  unsigned long *config)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned param = pinconf_to_config_param(*config);
	struct qpnp_padinfo *pad;
	unsigned arg;
	int ret;

	pad = qpnp_get_desc(qctrl, pin);
	if (!pad)
		return -EINVAL;

	/* Convert pinconf values to register values */
	ret = qpnp_conv_from_pin(qctrl, pad, param, &arg);
	if (ret)
		return ret;

	/* Convert register value to pinconf value */
	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int qpnp_config_set(struct pinctrl_dev *pctldev, unsigned int pin,
			  unsigned long *configs, unsigned nconfs)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);
	struct qpnp_padinfo *pad;
	unsigned param;
	unsigned arg;
	int idx, ret;

	pad = qpnp_get_desc(qctrl, pin);
	if (!pad)
		return -EINVAL;

	for (idx = 0; idx < nconfs; idx++) {
		param = pinconf_to_config_param(configs[idx]);
		arg = pinconf_to_config_argument(configs[idx]);

		/* Convert pinconf values to register values */
		ret = qpnp_conv_to_pin(qctrl, pad, param, arg);
		if (ret < 0) {
			dev_err(pctldev->dev, "config %u = %u failed\n",
				param, arg);
			return ret;
		}
	}

	return 0;
}

static void qpnp_config_dbg_show(struct pinctrl_dev *pctldev,
				 struct seq_file *seq, unsigned pin)
{
	struct qpnp_pinctrl *qctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int en, buff, pull, out, drive, addr, mode, func, inv;
	struct qpnp_padinfo *pad;
	const char *name;
	int ret;

	static const char *const modes[] = {
		"di", "do", "dio", "ai", "ao", "aio", "cs"
	};
	static const char * const biases[] = {
		"pull-up 30uA", "pull-up 1.5uA", "pull-up 31.5uA",
		"pull-up 1.5uA + 30uA boost", "pull-down 10uA", "no pull"
	};
	static const char * const buffer_types[] = {
		"push-pull", "open-drain", "open-func"
	};
	static const char * const strengths[] = {
		"no", "low", "medium", "high"
	};

	pad = qpnp_get_desc(qctrl, pin);
	if (!pad)
		return;

	name = pad->name;

	addr = QPNP_REG_ADDR(pad, QPNP_REG_MODE_CTL);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0) {
		seq_printf(seq, " %-8s: read error %d", name, ret);
		return;
	}

	mode = QPNP_GET(buff, QPNP_REG_MODE_SEL_SHIFT,
			QPNP_REG_MODE_SEL_MASK);
	func = QPNP_GET(buff, QPNP_REG_MODE_FUNCTION_SHIFT,
			QPNP_REG_MODE_FUNCTION_MASK);
	inv  = QPNP_GET(buff, QPNP_REG_MODE_INVERT_SHIFT,
			QPNP_REG_MODE_INVERT_MASK);

	addr = QPNP_REG_ADDR(pad, QPNP_REG_DIG_PULL_CTL);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0) {
		seq_printf(seq, " %-8s: read error %d", name, ret);
		return;
	}
	pull = QPNP_GET(buff, QPNP_REG_PULL_SHIFT, QPNP_REG_PULL_MASK);

	addr = QPNP_REG_ADDR(pad, QPNP_REG_DIG_OUT_CTL);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0) {
		seq_printf(seq, " %-8s: read error %d", name, ret);
		return;
	}
	out = QPNP_GET(buff, QPNP_REG_OUT_TYPE_SHIFT, QPNP_REG_OUT_TYPE_MASK);
	drive = QPNP_GET(buff, QPNP_REG_OUT_STRENGTH_SHIFT,
			 QPNP_REG_OUT_STRENGTH_MASK);

	addr = QPNP_REG_ADDR(pad, QPNP_REG_EN_CTL);
	ret = regmap_read(qctrl->map, addr, &buff);
	if (ret < 0) {
		seq_printf(seq, " %-8s: read error %d", name, ret);
		return;
	}
	en = QPNP_GET(buff, QPNP_REG_MASTER_EN_SHIFT, QPNP_REG_MASTER_EN_MASK);

	seq_printf(seq, " %-8s: %-4s", name, modes[mode]);
	seq_printf(seq, " %-7s", qctrl->functions[func]);
	seq_printf(seq, " %-24s %-12s", biases[pull], buffer_types[out]);
	seq_printf(seq, " %-6s %-4s", strengths[drive], inv ? "inv" : "");
	seq_printf(seq, " %s", !en ? "high-Z" : "");

}

static const struct pinconf_ops qpnp_pinconf_ops = {
	.pin_config_group_get	= qpnp_config_get,
	.pin_config_group_set	= qpnp_config_set,
	.pin_config_group_dbg_show = qpnp_config_dbg_show,
};

static int qpnp_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);

	return qpnp_config_set(qctrl->ctrl, offset, &config, 1);
}

static int qpnp_direction_output(struct gpio_chip *chip,
			      unsigned offset, int val)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, val);
	return qpnp_config_set(qctrl->ctrl, offset, &config, 1);
}

static int qpnp_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void qpnp_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static int qpnp_of_xlate(struct gpio_chip *chip,
		       const struct of_phandle_args *gpio_desc, u32 *flags)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	struct qpnp_padinfo *pad;
	unsigned pin = gpio_desc->args[0] - QPNP_PIN_PHYSICAL_OFFSET;

	if (chip->of_gpio_n_cells < 2)
		return -EINVAL;

	pad = qpnp_get_desc(qctrl, pin);
	if (!pad)
		return -EINVAL;

	if (flags)
		*flags = gpio_desc->args[1];

	return pin;
}

static int qpnp_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	struct qpnp_padinfo *pad;

	pad = qpnp_get_desc(qctrl, offset);
	if (!pad)
		return -EINVAL;

	return pad->irq;
}

static void qpnp_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct qpnp_pinctrl *qctrl = to_qpnp_pinctrl(chip);
	unsigned idx;

	for (idx = 0; idx < chip->ngpio; idx++) {
		qpnp_config_dbg_show(qctrl->ctrl, s, idx);
		seq_puts(s, "\n");
	}
}

static const struct gpio_chip qpnp_gpio_template = {
	.direction_input  = qpnp_direction_input,
	.direction_output = qpnp_direction_output,
	.get              = qpnp_get,
	.set              = qpnp_set,
	.request          = qpnp_request,
	.free             = qpnp_free,
	.of_xlate	  = qpnp_of_xlate,
	.to_irq		  = qpnp_to_irq,
	.dbg_show         = qpnp_dbg_show,
};

static int qpnp_control_init(struct qpnp_pinctrl *qctrl,
			  struct qpnp_padinfo *pad)
{
	pad->modes = 0;

	if (pad->type == QPNP_GPIO_TYPE) {
		switch (pad->subtype) {
		case QPNP_GPIO_SUBTYPE_GPIO_4CH:
		case QPNP_GPIO_SUBTYPE_GPIOC_4CH:
		case QPNP_GPIO_SUBTYPE_GPIO_8CH:
		case QPNP_GPIO_SUBTYPE_GPIOC_8CH:

			/* only GPIO is supported*/
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN_OUT);
			break;
		default:
			dev_err(qctrl->dev, "invalid GPIO subtype 0x%x\n",
				pad->subtype);
			return -EINVAL;
		}

	} else if (pad->type == QPNP_MPP_TYPE) {

		switch (pad->subtype) {
		case QPNP_MPP_SUBTYPE_4CH_NO_SINK:
		case QPNP_MPP_SUBTYPE_ULT_4CH_NO_SINK:

			/* Current sink not supported*/
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_BIDIR);
			pad->modes |= BIT(QPNP_PIN_MODE_AIN);
			pad->modes |= BIT(QPNP_PIN_MODE_AOUT);
			break;
		case QPNP_MPP_SUBTYPE_4CH_NO_ANA_OUT:
		case QPNP_MPP_SUBTYPE_ULT_4CH_NO_ANA_OUT:

			/* Analog output not supported */
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_BIDIR);
			pad->modes |= BIT(QPNP_PIN_MODE_AIN);
			pad->modes |= BIT(QPNP_PIN_MODE_SINK);
			break;
		case QPNP_MPP_SUBTYPE_4CH_FULL_FUNC:
		case QPNP_MPP_SUBTYPE_8CH_FULL_FUNC:

			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_DIG_IN_OUT);
			pad->modes |= BIT(QPNP_PIN_MODE_BIDIR);
			pad->modes |= BIT(QPNP_PIN_MODE_AIN);
			pad->modes |= BIT(QPNP_PIN_MODE_AOUT);
			pad->modes |= BIT(QPNP_PIN_MODE_SINK);
			break;
		default:
			dev_err(qctrl->dev, "invalid MPP subtype 0x%x\n",
				pad->subtype);
			return -EINVAL;
		}
	} else {
		dev_err(qctrl->dev, "invalid type 0x%x\n", pad->type);
		return -EINVAL;
	}

	return 0;
}

static int qpnp_discover(struct platform_device *pdev,
			struct qpnp_pinctrl *qctrl)
{
	struct device *dev = qctrl->dev;
	struct pinctrl_pin_desc *desc, *descs;
	struct qpnp_padinfo *pad, *pads;
	unsigned int addr, npins;
	int idx, ret;

	npins = qctrl->info->npins;
	pads = devm_kcalloc(dev, npins, sizeof(*pads), GFP_KERNEL);
	if (!pads)
		return -ENOMEM;

	descs = devm_kcalloc(dev, npins, sizeof(*descs), GFP_KERNEL);
	if (!descs)
		return -ENOMEM;

	for (idx = 0; idx < npins; idx++) {

		pad = &pads[idx];
		desc = &descs[idx];

		pad->irq = platform_get_irq(pdev, idx);
		if (pad->irq < 0)
			return pad->irq;

		pad->offset = qctrl->info->base + (idx * 0x100);

		addr = QPNP_REG_ADDR(pad, QPNP_REG_DIG_MAJOR_REV);
		ret = regmap_read(qctrl->map, addr, &pad->major);
		if (ret < 0)
			return ret;

		addr = QPNP_REG_ADDR(pad, QPNP_REG_TYPE);
		ret = regmap_read(qctrl->map, addr, &pad->type);
		if (ret < 0)
			return ret;

		if (pad->type != qctrl->info->type) {
			dev_err(dev, "Expected %x, found %x\n",
				qctrl->info->type, pad->type);
			return -EINVAL;
		}

		addr = QPNP_REG_ADDR(pad, QPNP_REG_SUBTYPE);
		ret = regmap_read(qctrl->map, addr, &pad->subtype);
		if (ret < 0)
			return ret;

		ret = qpnp_control_init(qctrl, pad);
		if (ret)
			return ret;

		pad->name = qctrl->groups[idx];
		desc->number = idx;
		desc->name = pad->name;
	}

	qctrl->pads = pads;

	qctrl->chip = qpnp_gpio_template;
	qctrl->chip.dev = dev;
	qctrl->chip.base = -1;
	qctrl->chip.ngpio = qctrl->info->npins;
	qctrl->chip.label = dev_name(dev);
	qctrl->chip.of_gpio_n_cells = 2;
	qctrl->chip.can_sleep = true;

	qctrl->desc.pctlops = &qpnp_pinctrl_ops,
	qctrl->desc.pmxops = &qpnp_pinmux_ops,
	qctrl->desc.confops = &qpnp_pinconf_ops,
	qctrl->desc.owner = THIS_MODULE,
	qctrl->desc.name = dev_name(dev);
	qctrl->desc.pins = descs;
	qctrl->desc.npins = npins;

	qctrl->ctrl = pinctrl_register(&qctrl->desc, dev, qctrl);
	if (!qctrl->ctrl)
		return -ENODEV;

	ret = gpiochip_add(&qctrl->chip);
	if (ret) {
		dev_err(qctrl->dev, "can't add gpio chip\n");
		goto err_chip;
	}

	ret = gpiochip_add_pin_range(&qctrl->chip, dev_name(dev),
				     0, 0, npins);
	if (ret) {
		dev_err(dev, "failed to add pin range\n");
		goto err_range;
	}

	return 0;

err_chip:
	pinctrl_unregister(qctrl->ctrl);

err_range:
	gpiochip_remove(&qctrl->chip);

	return ret;

}

static const struct of_device_id qpnp_pinctrl_of_match[];

static int qpnp_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct qpnp_chipinfo *qchip;
	struct qpnp_pinctrl *qctrl;

	qctrl = devm_kzalloc(dev, sizeof(*qctrl), GFP_KERNEL);
	if (!qctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, qctrl);

	qchip = of_match_node(qpnp_pinctrl_of_match, dev->of_node)->data;

	qctrl->info = qchip;
	qctrl->dev = &pdev->dev;
	qctrl->map = dev_get_regmap(dev->parent, NULL);

	if (qchip->type == QPNP_GPIO_TYPE) {
		if (WARN_ON(qchip->npins > ARRAY_SIZE(qpnp_gpio_groups)))
			return -EINVAL;
		qctrl->groups = qpnp_gpio_groups;
		qctrl->functions = qpnp_gpio_functions;
	} else {
		if (WARN_ON(qchip->npins > ARRAY_SIZE(qpnp_mpp_groups)))
			return -EINVAL;
		qctrl->groups = qpnp_mpp_groups;
		qctrl->functions = qpnp_mpp_functions;
	}

	return qpnp_discover(pdev, qctrl);
}

static int qpnp_pinctrl_remove(struct platform_device *pdev)
{
	struct qpnp_pinctrl *qctrl = platform_get_drvdata(pdev);

	gpiochip_remove(&qctrl->chip);
	pinctrl_unregister(qctrl->ctrl);

	return 0;
}

static const struct qpnp_chipinfo qpnp_pm8841_mpp_info = {
	.npins	= 4,
	.base	= 0xa000,
	.type   = QPNP_MPP_TYPE,
};

static const struct qpnp_chipinfo qpnp_pm8941_gpio_info = {
	.npins	= 36,
	.base	= 0xc000,
	.type   = QPNP_GPIO_TYPE,
};

static const struct qpnp_chipinfo qpnp_pm8941_mpp_info = {
	.npins	= 8,
	.base	= 0xa000,
	.type   = QPNP_MPP_TYPE,
};

static const struct qpnp_chipinfo qpnp_pma8084_mpp_info = {
	.npins	= 4,
	.base	= 0xa000,
	.type   = QPNP_MPP_TYPE,
};

static const struct qpnp_chipinfo qpnp_pma8084_gpio_info = {
	.npins	= 22,
	.base	= 0xc000,
	.type   = QPNP_GPIO_TYPE,
};

static const struct of_device_id qpnp_pinctrl_of_match[] = {
	{ .compatible = "qcom,pm8941-gpio", .data = &qpnp_pm8941_gpio_info },
	{ .compatible = "qcom,pm8941-mpp", .data = &qpnp_pm8941_mpp_info },
	{ .compatible = "qcom,pm8841-mpp", .data = &qpnp_pm8841_mpp_info },
	{ .compatible = "qcom,pma8084-gpio", .data = &qpnp_pma8084_gpio_info },
	{ .compatible = "qcom,pma8084-mpp", .data = &qpnp_pma8084_mpp_info },
	{ },
};
MODULE_DEVICE_TABLE(of, qpnp_pinctrl_of_match);

static struct platform_driver qpnp_pinctrl_driver = {
	.driver = {
		.name = "spmi-pmic-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = qpnp_pinctrl_of_match,
	},
	.probe = qpnp_pinctrl_probe,
	.remove = qpnp_pinctrl_remove,
};
module_platform_driver(qpnp_pinctrl_driver);

MODULE_AUTHOR("Ivan T. Ivanov <iivanov@mm-sol.com>");
MODULE_DESCRIPTION("Qualcomm SPMI PMIC pin control driver");
MODULE_ALIAS("platform:spmi-pmic-pinctrl");
MODULE_LICENSE("GPL v2");
