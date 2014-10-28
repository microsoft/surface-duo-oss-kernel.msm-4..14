/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/mfd/pm8921-core.h>

#include <dt-bindings/pinctrl/qcom,pmic-gpio.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"

/* direction */
#define PM8XXX_GPIO_DIR_OUT		BIT(0)
#define PM8XXX_GPIO_DIR_IN		BIT(1)

/* output buffer */
#define PM8XXX_GPIO_PUSH_PULL		0
#define PM8XXX_GPIO_OPEN_DRAIN		1

/* bias */
#define PM8XXX_GPIO_BIAS_PU_30		0
#define PM8XXX_GPIO_BIAS_PU_1P5		1
#define PM8XXX_GPIO_BIAS_PU_31P5	2
#define PM8XXX_GPIO_BIAS_PU_1P5_30	3
#define PM8XXX_GPIO_BIAS_PD		4
#define PM8XXX_GPIO_BIAS_NP		5

/* GPIO registers */
#define SSBI_REG_ADDR_GPIO_BASE		0x150
#define SSBI_REG_ADDR_GPIO(n)		(SSBI_REG_ADDR_GPIO_BASE + n)

#define	PM8XXX_GPIO_MODE_ENABLE		BIT(0)
#define PM8XXX_GPIO_WRITE		BIT(7)

#define PM8XXX_MAX_GPIOS		44

/* Qualcomm specific pin configurations */
#define PM8XXX_PINCONF_PULL_UP		(PIN_CONFIG_END + 1)
#define PM8XXX_PINCONF_STRENGTH		(PIN_CONFIG_END + 2)

struct pm8xxx_pinbindings {
	const char *property;
	unsigned param;
	u32 default_value;
};
static struct pm8xxx_pinbindings pm8xxx_pinbindings[] = {
	/* PMIC_GPIO_PULL_UP_30...  */
	{"qcom,pull-up-strength",	PM8XXX_PINCONF_PULL_UP, 0},
	/* PMIC_GPIO_STRENGTH_NO... */
	{"qcom,drive-strength",		PM8XXX_PINCONF_STRENGTH, 0},
};

struct pm8xxx_gpio_pin {
	int irq;

	u8 power_source;
	u8 direction;
	u8 output_buffer;
	u8 output_value;
	u8 bias;
	u8 output_strength;
	u8 disable;
	u8 function;
	u8 non_inverted;
};

struct pm8xxx_gpio_data {
	int ngpio;
};

struct pm8xxx_gpio {
	struct device *dev;
	struct regmap *regmap;
	struct pinctrl_dev *pctrl;
	struct gpio_chip chip;

	const struct pm8xxx_gpio_data *data;

	struct pm8xxx_gpio_pin pins[PM8XXX_MAX_GPIOS];
};

static inline struct pm8xxx_gpio *to_pm8xxx_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct pm8xxx_gpio, chip);
};

static const char * const pm8xxx_gpio_groups[PM8XXX_MAX_GPIOS] = {
	"gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7", "gpio8",
	"gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14", "gpio15",
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21", "gpio22",
	"gpio23", "gpio24", "gpio25", "gpio26", "gpio27", "gpio28", "gpio29",
	"gpio30", "gpio31", "gpio32", "gpio33", "gpio34", "gpio35", "gpio36",
	"gpio37", "gpio38", "gpio39", "gpio40", "gpio41", "gpio42", "gpio43",
	"gpio44",
};

static const char * const pm8xxx_gpio_functions[] = {
	PMIC_GPIO_FUNC_NORMAL, PMIC_GPIO_FUNC_PAIRED,
	PMIC_GPIO_FUNC_FUNC1, PMIC_GPIO_FUNC_FUNC2,
	PMIC_GPIO_FUNC_DTEST1, PMIC_GPIO_FUNC_DTEST2,
	PMIC_GPIO_FUNC_DTEST3, PMIC_GPIO_FUNC_DTEST4,
};

static int pm8xxx_gpio_read(struct pm8xxx_gpio *pctrl, int pin, int bank)
{
	int reg = SSBI_REG_ADDR_GPIO(pin);
	unsigned int val = bank << 4;
	int ret;

	ret = regmap_write(pctrl->regmap, reg, val);
	if (ret) {
		dev_err(pctrl->dev,
			"failed to select bank %d of pin %d\n", bank, pin);
		return ret;
	}

	ret = regmap_read(pctrl->regmap, reg, &val);
	if (ret) {
		dev_err(pctrl->dev,
			"failed to read register %d of pin %d\n", bank, pin);
		return ret;
	}

	return val;
}

static int pm8xxx_gpio_write(struct pm8xxx_gpio *pctrl,
			     int pin, int bank, u8 val)
{
	int ret;

	val |= PM8XXX_GPIO_WRITE;
	val |= bank << 4;

	ret = regmap_write(pctrl->regmap, SSBI_REG_ADDR_GPIO(pin), val);
	if (ret)
		dev_err(pctrl->dev, "failed to write register\n");

	return ret;
}

static int pm8xxx_gpio_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct pm8xxx_gpio *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->ngpio;
}

static const char *pm8xxx_gpio_get_group_name(struct pinctrl_dev *pctldev,
				      unsigned group)
{
	return pm8xxx_gpio_groups[group];
}

static int pm8xxx_parse_dt_config(struct device *dev, struct device_node *np,
			unsigned long **configs, unsigned int *nconfigs)
{
	struct pm8xxx_pinbindings *par;
	unsigned long cfg[ARRAY_SIZE(pm8xxx_pinbindings)];
	unsigned int ncfg = 0;
	int ret, idx;
	u32 val;

	if (!np)
		return -EINVAL;

	for (idx = 0; idx < ARRAY_SIZE(pm8xxx_pinbindings); idx++) {
		par = &pm8xxx_pinbindings[idx];
		ret = of_property_read_u32(np, par->property, &val);

		/* property not found */
		if (ret == -EINVAL)
			continue;

		/* use default value, when no value is specified */
		if (ret)
			val = par->default_value;

		dev_dbg(dev, "found %s with value %u\n", par->property, val);
		cfg[ncfg] = pinconf_to_config_packed(par->param, val);
		ncfg++;
	}

	ret = 0;

	/* no configs found at qchip->npads */
	if (ncfg == 0) {
		*configs = NULL;
		*nconfigs = 0;
		goto out;
	}

	/*
	 * Now limit the number of configs to the real number of
	 * found properties.
	 */
	*configs = kcalloc(ncfg, sizeof(unsigned long), GFP_KERNEL);
	if (!*configs) {
		ret = -ENOMEM;
		goto out;
	}

	memcpy(*configs, cfg, ncfg * sizeof(unsigned long));
	*nconfigs = ncfg;

out:
	return ret;
}

static int pm8xxx_dt_subnode_to_map(struct pinctrl_dev *pctldev,
				  struct device_node *np,
				  struct pinctrl_map **map,
				  unsigned *reserv, unsigned *nmaps,
				  enum pinctrl_map_type type)
{
	unsigned long *configs = NULL;
	unsigned num_configs = 0;
	struct property *prop;
	const char *group;
	int ret;

	ret = pm8xxx_parse_dt_config(pctldev->dev, np, &configs, &num_configs);
	if (ret < 0)
		return ret;

	if (!num_configs)
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
				reserv, nmaps, group, configs,
				num_configs, type);
		if (ret < 0)
			break;
	}
exit:
	kfree(configs);
	return ret;
}

static int pm8xxx_dt_node_to_map(struct pinctrl_dev *pctldev,
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

		ret = pm8xxx_dt_subnode_to_map(pctldev, np, map, &reserv,
					     nmaps, type);
		if (ret)
			break;
	}

	if (ret < 0)
		pinctrl_utils_dt_free_map(pctldev, *map, *nmaps);

	return ret;
}

static const struct pinctrl_ops pm8xxx_gpio_pinctrl_ops = {
	.get_groups_count	= pm8xxx_gpio_get_groups_count,
	.get_group_name		= pm8xxx_gpio_get_group_name,
	.dt_node_to_map		= pm8xxx_dt_node_to_map,
	.dt_free_map		= pinctrl_utils_dt_free_map,
};

static int pm8xxx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(pm8xxx_gpio_functions);
}

static const char *pm8xxx_get_function_name(struct pinctrl_dev *pctldev,
					 unsigned function)
{
	return pm8xxx_gpio_functions[function];
}

static int pm8xxx_get_function_groups(struct pinctrl_dev *pctldev,
				   unsigned function,
				   const char * const **groups,
				   unsigned * const num_groups)
{
	struct pm8xxx_gpio *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pm8xxx_gpio_groups;
	*num_groups = pctrl->data->ngpio;
	return 0;
}

static int pm8xxx_pinmux_enable(struct pinctrl_dev *pctldev,
			     unsigned function,
			     unsigned group)
{
	struct pm8xxx_gpio *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[group];
	u8 val;

	pin->function = function;
	val = pin->function << 1;

	pm8xxx_gpio_write(pctrl, group, 4, val);

	return 0;
}

static const struct pinmux_ops pm8xxx_pinmux_ops = {
	.get_functions_count	= pm8xxx_get_functions_count,
	.get_function_name	= pm8xxx_get_function_name,
	.get_function_groups	= pm8xxx_get_function_groups,
	.set_mux		= pm8xxx_pinmux_enable,
};

static int pm8xxx_gpio_config_get(struct pinctrl_dev *pctldev,
			  unsigned int offset,
			  unsigned long *config)
{
	struct pm8xxx_gpio *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset];
	unsigned param = pinconf_to_config_param(*config);
	unsigned arg;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		arg = pin->bias == PM8XXX_GPIO_BIAS_NP;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = pin->bias == PM8XXX_GPIO_BIAS_PD;
		break;
	case PM8XXX_PINCONF_PULL_UP:
		if (pin->bias >= PM8XXX_GPIO_BIAS_PU_30 &&
		    pin->bias <= PM8XXX_GPIO_BIAS_PU_1P5_30)
			arg = PMIC_GPIO_PULL_UP_30 + pin->bias;
		else
			arg = 0;
		break;
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
		arg = pin->disable;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		arg = pin->direction == PM8XXX_GPIO_DIR_IN;
		break;
	case PIN_CONFIG_OUTPUT:
		arg = pin->output_value;
		break;
	case PIN_CONFIG_POWER_SOURCE:
		arg = pin->power_source;
		break;
	case PM8XXX_PINCONF_STRENGTH:
		arg = pin->output_strength;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		arg = pin->output_buffer == PM8XXX_GPIO_PUSH_PULL;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		arg = pin->output_buffer == PM8XXX_GPIO_OPEN_DRAIN;
		break;
	default:
		dev_err(pctrl->dev,
			"unsupported config parameter: %x\n",
			param);
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int pm8xxx_gpio_config_set(struct pinctrl_dev *pctldev,
				  unsigned int offset,
				  unsigned long *configs,
				  unsigned num_configs)
{
	struct pm8xxx_gpio *pctrl = pinctrl_dev_get_drvdata(pctldev);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset];
	unsigned param;
	unsigned arg;
	unsigned i;
	u8 banks = 0;
	u8 val;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			pin->bias = PM8XXX_GPIO_BIAS_NP;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pin->bias = PM8XXX_GPIO_BIAS_PD;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PM8XXX_PINCONF_PULL_UP:
			if (arg < PMIC_GPIO_PULL_UP_30 ||
			    arg > PMIC_GPIO_PULL_UP_1P5_30) {
				dev_err(pctrl->dev, "invalid pull-up level\n");
				return -EINVAL;
			}
			pin->bias = arg - PM8XXX_GPIO_BIAS_PU_30;
			banks |= BIT(2);
			pin->disable = 0;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
			pin->disable = 1;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			pin->direction = PM8XXX_GPIO_DIR_IN;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_OUTPUT:
			pin->direction = PM8XXX_GPIO_DIR_OUT;
			pin->output_value = !!arg;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_POWER_SOURCE:
			pin->power_source = arg;
			banks |= BIT(0);
			break;
		case PM8XXX_PINCONF_STRENGTH:
			if (arg > PMIC_GPIO_STRENGTH_LOW) {
				dev_err(pctrl->dev, "invalid drive strength\n");
				return -EINVAL;
			}
			pin->output_strength = arg;
			banks |= BIT(3);
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			pin->output_buffer = PM8XXX_GPIO_PUSH_PULL;
			banks |= BIT(1);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			pin->output_buffer = PM8XXX_GPIO_OPEN_DRAIN;
			banks |= BIT(1);
			break;
		default:
			dev_err(pctrl->dev,
					"unsupported config parameter: %x\n",
					param);
			return -EINVAL;
		}
	}

	if (banks & BIT(0))
		pm8xxx_gpio_write(pctrl, offset, 0, pin->power_source << 1 |
				  PM8XXX_GPIO_MODE_ENABLE);

	if (banks & BIT(1)) {
		val = pin->direction << 2;
		val |= pin->output_buffer << 1;
		val |= pin->output_value;
		pm8xxx_gpio_write(pctrl, offset, 1, val);
	}

	if (banks & BIT(2)) {
		val = pin->bias << 1;
		pm8xxx_gpio_write(pctrl, offset, 2, val);
	}

	if (banks & BIT(3)) {
		val = pin->output_strength << 2;
		val |= pin->disable;
		pm8xxx_gpio_write(pctrl, offset, 3, val);
	}

	if (banks & BIT(4)) {
		val = pin->function << 1;
		pm8xxx_gpio_write(pctrl, offset, 4, val);
	}

	if (banks & BIT(5)) {
		val = 0;
		if (pin->non_inverted)
			val |= BIT(3);
		pm8xxx_gpio_write(pctrl, offset, 5, val);
	}

	return 0;
}

static const struct pinconf_ops pm8xxx_gpio_pinconf_ops = {
	.pin_config_group_get = pm8xxx_gpio_config_get,
	.pin_config_group_set = pm8xxx_gpio_config_set,
};

static struct pinctrl_desc pm8xxx_gpio_desc = {
	.pctlops = &pm8xxx_gpio_pinctrl_ops,
	.pmxops = &pm8xxx_pinmux_ops,
	.confops = &pm8xxx_gpio_pinconf_ops,
	.owner = THIS_MODULE,
};

static int pm8xxx_gpio_direction_input(struct gpio_chip *chip,
				       unsigned offset)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(chip);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset - 1];
	u8 val;

	pin->direction = PM8XXX_GPIO_DIR_IN;
	val = pin->direction << 2;

	pm8xxx_gpio_write(pctrl, offset, 1, val);

	return 0;
}

static int pm8xxx_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset,
					int value)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(chip);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset - 1];
	u8 val;

	pin->direction = PM8XXX_GPIO_DIR_OUT;
	pin->output_value = !!value;

	val = pin->direction << 2;
	val |= pin->output_buffer << 1;
	val |= pin->output_value;

	pm8xxx_gpio_write(pctrl, offset, 1, val);

	return 0;
}

static int pm8xxx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(chip);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset - 1];

	if (pin->direction == PM8XXX_GPIO_DIR_OUT)
		return pin->output_value;

	return pm8xxx_read_irq_status(pin->irq);
}

static void pm8xxx_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(gc);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset - 1];
	u8 val;

	pin->output_value = !!value;

	val = pin->direction << 2;
	val |= pin->output_buffer << 1;
	val |= pin->output_value;

	pm8xxx_gpio_write(pctrl, offset, 1, val);
}

static int pm8xxx_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(chip);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset - 1];

	return pin->irq;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static void pm8xxx_gpio_dbg_show_one(struct seq_file *s,
				  struct pinctrl_dev *pctldev,
				  struct gpio_chip *chip,
				  unsigned offset,
				  unsigned gpio)
{
	struct pm8xxx_gpio *pctrl = to_pm8xxx_gpio(chip);
	struct pm8xxx_gpio_pin *pin = &pctrl->pins[offset];

	static const char * const directions[] = {
		"off", "out", "in", "both"
	};
	static const char * const biases[] = {
		"pull-up 30uA", "pull-up 1.5uA", "pull-up 31.5uA",
		"pull-up 1.5uA + 30uA boost", "pull-down 10uA", "no pull"
	};
	static const char * const buffer_types[] = {
		"push-pull", "open-drain"
	};
	static const char * const strengths[] = {
		"no", "high", "medium", "low"
	};

	seq_printf(s, " gpio%-2d:", offset + 1);
	if (pin->disable) {
		seq_puts(s, " ---");
	} else {
		seq_printf(s, " %-4s", directions[pin->direction]);
		seq_printf(s, " %-7s", pm8xxx_gpio_functions[pin->function]);
		seq_printf(s, " VIN%d", pin->power_source);
		seq_printf(s, " %-27s", biases[pin->bias]);
		seq_printf(s, " %-10s", buffer_types[pin->output_buffer]);
		seq_printf(s, " %-4s", pin->output_value ? "high" : "low");
		seq_printf(s, " %-7s", strengths[pin->output_strength]);
		if (!pin->non_inverted)
			seq_puts(s, " inverted");
	}
}

static void pm8xxx_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned gpio = chip->base;
	unsigned i;

	for (i = 0; i < chip->ngpio; i++, gpio++) {
		pm8xxx_gpio_dbg_show_one(s, NULL, chip, i, gpio);
		seq_puts(s, "\n");
	}
}

#else
#define msm_gpio_dbg_show NULL
#endif

static struct gpio_chip pm8xxx_gpio_template = {
	.direction_input = pm8xxx_gpio_direction_input,
	.direction_output = pm8xxx_gpio_direction_output,
	.get = pm8xxx_gpio_get,
	.set = pm8xxx_gpio_set,
	.to_irq = pm8xxx_gpio_to_irq,
	.dbg_show = pm8xxx_gpio_dbg_show,
	.owner = THIS_MODULE,
};

static int pm8xxx_gpio_populate(struct pm8xxx_gpio *pctrl)
{
	struct pm8xxx_gpio_pin *pin;
	int val;
	int i;

	for (i = 0; i < pctrl->data->ngpio; i++) {
		pin = &pctrl->pins[i];

		val = pm8xxx_gpio_read(pctrl, i, 0);
		if (val < 0)
			return val;

		pin->power_source = (val >> 1) & 0x7;

		val = pm8xxx_gpio_read(pctrl, i, 1);
		if (val < 0)
			return val;

		pin->direction = (val >> 2) & 0x3;
		pin->output_buffer = !!(val & BIT(1));
		pin->output_value = val & BIT(0);

		val = pm8xxx_gpio_read(pctrl, i, 2);
		if (val < 0)
			return val;

		pin->bias = (val >> 1) & 0x7;

		val = pm8xxx_gpio_read(pctrl, i, 3);
		if (val < 0)
			return val;

		pin->output_strength = (val >> 2) & 0x3;
		pin->disable = val & BIT(0);

		val = pm8xxx_gpio_read(pctrl, i, 4);
		if (val < 0)
			return val;

		pin->function = (val >> 1) & 0x7;

		val = pm8xxx_gpio_read(pctrl, i, 5);
		if (val < 0)
			return val;

		pin->non_inverted = !!(val & BIT(3));
	}

	return 0;
}

static const struct pm8xxx_gpio_data pm8018_gpio_data = {
	.ngpio = 6,
};

static const struct pm8xxx_gpio_data pm8038_gpio_data = {
	.ngpio = 12,
};

static const struct pm8xxx_gpio_data pm8058_gpio_data = {
	.ngpio = 40,
};
static const struct pm8xxx_gpio_data pm8917_gpio_data = {
	.ngpio = 38,
};

static const struct pm8xxx_gpio_data pm8921_gpio_data = {
	.ngpio = 44,
};

static const struct of_device_id pm8xxx_gpio_of_match[] = {
	{ .compatible = "qcom,pm8018-gpio", .data = &pm8018_gpio_data },
	{ .compatible = "qcom,pm8038-gpio", .data = &pm8038_gpio_data },
	{ .compatible = "qcom,pm8058-gpio", .data = &pm8058_gpio_data },
	{ .compatible = "qcom,pm8917-gpio", .data = &pm8917_gpio_data },
	{ .compatible = "qcom,pm8921-gpio", .data = &pm8921_gpio_data },
	{ },
};
MODULE_DEVICE_TABLE(of, pm8xxx_gpio_of_match);

static int pm8xxx_gpio_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct pm8xxx_gpio *pctrl;
	int ret;
	int i;

	match = of_match_node(pm8xxx_gpio_of_match, pdev->dev.of_node);
	if (!match)
		return -ENXIO;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->dev = &pdev->dev;
	pctrl->data = match->data;

	BUG_ON(pctrl->data->ngpio > PM8XXX_MAX_GPIOS);

	pctrl->chip = pm8xxx_gpio_template;
	pctrl->chip.base = -1;
	pctrl->chip.dev = &pdev->dev;
	pctrl->chip.of_node = pdev->dev.of_node;
	pctrl->chip.label = dev_name(pctrl->dev);
	pctrl->chip.ngpio = pctrl->data->ngpio;

	pctrl->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pctrl->regmap) {
		dev_err(&pdev->dev, "parent regmap unavailable\n");
		return -ENXIO;
	}

	for (i = 0; i < pctrl->data->ngpio; i++) {
		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"missing interrupts for pin %d\n", i);
			return ret;
		}

		pctrl->pins[i].irq = ret;
	}

	ret = pm8xxx_gpio_populate(pctrl);
	if (ret)
		return ret;

	pm8xxx_gpio_desc.name = dev_name(&pdev->dev);
	pctrl->pctrl = pinctrl_register(&pm8xxx_gpio_desc, &pdev->dev, pctrl);
	if (!pctrl->pctrl) {
		dev_err(&pdev->dev, "couldn't register pm8xxx gpio driver\n");
		return -ENODEV;
	}

	ret = gpiochip_add(&pctrl->chip);
	if (ret) {
		dev_err(&pdev->dev, "failed register gpiochip\n");
		goto unregister_pinctrl;
	}

	ret = gpiochip_add_pin_range(&pctrl->chip,
				     dev_name(pctrl->dev),
				     1, 0, pctrl->data->ngpio);
	if (ret) {
		dev_err(pctrl->dev, "failed to add pin range\n");
		goto unregister_gpiochip;
	}

	platform_set_drvdata(pdev, pctrl);

	dev_dbg(&pdev->dev, "Qualcomm pm8xxx gpio driver probed\n");

	return 0;

unregister_pinctrl:
	pinctrl_unregister(pctrl->pctrl);

unregister_gpiochip:
	gpiochip_remove(&pctrl->chip);

	return ret;
}

static int pm8xxx_gpio_remove(struct platform_device *pdev)
{
	struct pm8xxx_gpio *pctrl = platform_get_drvdata(pdev);

	gpiochip_remove(&pctrl->chip);

	pinctrl_unregister(pctrl->pctrl);

	return 0;
}

static struct platform_driver pm8xxx_gpio_driver = {
	.driver = {
		.name = "ssbi-pmic-gpio",
		.owner = THIS_MODULE,
		.of_match_table = pm8xxx_gpio_of_match,
	},
	.probe = pm8xxx_gpio_probe,
	.remove = pm8xxx_gpio_remove,
};

static int pm8xxx_gpio_init(void)
{
	return platform_driver_register(&pm8xxx_gpio_driver);
}
subsys_initcall(pm8xxx_gpio_init);

MODULE_AUTHOR("Bjorn Andersson <bjorn.andersson@sonymobile.com>");
MODULE_DESCRIPTION("Qualcomm SSBI PMIC GPIO driver");
MODULE_LICENSE("GPL v2");
