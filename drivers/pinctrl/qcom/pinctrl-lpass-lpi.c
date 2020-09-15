// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/bitops.h>

#include "../core.h"
#include "../pinctrl-utils.h"

#define LPI_GPIO_REG_VAL_CTL             0x00
#define LPI_GPIO_REG_DIR_CTL             0x04
#define LPI_SLEW_REG_VAL_CTL             0x00
#define LPI_SLEW_RATE_MAX                0x03
#define LPI_SLEW_BITS_SIZE               0x02

#define LPI_GPIO_REG_PULL_SHIFT		0x0
#define LPI_GPIO_REG_PULL_MASK		GENMASK(1, 0)

#define LPI_GPIO_REG_FUNCTION_SHIFT	0x2
#define LPI_GPIO_REG_FUNCTION_MASK	GENMASK(5, 2)

#define LPI_GPIO_REG_OUT_STRENGTH_SHIFT	0x6
#define LPI_GPIO_REG_OUT_STRENGTH_MASK	GENMASK(8, 6)

#define LPI_GPIO_REG_OE_SHIFT		0x9
#define LPI_GPIO_REG_OE_MASK		BIT(9)

#define LPI_GPIO_REG_DIR_SHIFT		0x1
#define LPI_GPIO_REG_DIR_MASK		0x2

#define LPI_GPIO_BIAS_DISABLE		0x0
#define LPI_GPIO_PULL_DOWN		0x1
#define LPI_GPIO_KEEPER			0x2
#define LPI_GPIO_PULL_UP		0x3

#define LPI_GPIO_FUNC_GPIO		"gpio"
#define LPI_GPIO_FUNC_FUNC1		"func1"
#define LPI_GPIO_FUNC_FUNC2		"func2"
#define LPI_GPIO_FUNC_FUNC3		"func3"
#define LPI_GPIO_FUNC_FUNC4		"func4"
#define LPI_GPIO_FUNC_FUNC5		"func5"

/* The index of each function in lpi_gpio_functions[] array */
enum lpi_gpio_func_index {
	LPI_GPIO_FUNC_INDEX_GPIO = 0x00,
	LPI_GPIO_FUNC_INDEX_FUNC1,
	LPI_GPIO_FUNC_INDEX_FUNC2,
	LPI_GPIO_FUNC_INDEX_FUNC3,
	LPI_GPIO_FUNC_INDEX_FUNC4,
	LPI_GPIO_FUNC_INDEX_FUNC5,
};

struct lpi_pinctrl_variant_data {
	int tlmm_reg_offset;
	const struct pinctrl_pin_desc *pins;
	int npins;
	const char *const *groups;
	int ngroups;
	int *slew_reg_pin_offsets;
};

struct lpi_pinctrl {
	struct device       *dev;
	struct pinctrl_dev  *ctrl;
	struct gpio_chip     chip;
	struct pinctrl_desc desc;
	char __iomem        *tlmm_base;
	char __iomem        *slew_base;
	struct clk          *core_vote;
	struct clk          *audio_vote;
	struct mutex         slew_access_lock;
	const struct lpi_pinctrl_variant_data *data;
};

/* sm8250 variant specific data */
#define SM8250_LPASS_PINS	14

static const char *const sm8250_gpio_groups[SM8250_LPASS_PINS] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4",
	"gpio5", "gpio6", "gpio7", "gpio8", "gpio9",
	"gpio10", "gpio11", "gpio12", "gpio13"
};

static const struct pinctrl_pin_desc sm8250_lpi_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
};

static int sm8250_slew_reg_offsets[] = {
		0x0, 0x2, 0x4, 0x8, 0xa,
		0xc, 0x0, 0x0, 0x0, 0x0,
		0x10, 0x12, 0x0, 0x0,
};

static struct lpi_pinctrl_variant_data sm8250_lpi_data = {
	.tlmm_reg_offset = 0x1000,
	.pins = sm8250_lpi_pins,
	.npins = ARRAY_SIZE(sm8250_lpi_pins),
	.slew_reg_pin_offsets = sm8250_slew_reg_offsets,
	.groups = sm8250_gpio_groups,
	.ngroups = ARRAY_SIZE(sm8250_gpio_groups),
};

static const char *const lpi_gpio_functions[] = {
	[LPI_GPIO_FUNC_INDEX_GPIO]	= LPI_GPIO_FUNC_GPIO,
	[LPI_GPIO_FUNC_INDEX_FUNC1]	= LPI_GPIO_FUNC_FUNC1,
	[LPI_GPIO_FUNC_INDEX_FUNC2]	= LPI_GPIO_FUNC_FUNC2,
	[LPI_GPIO_FUNC_INDEX_FUNC3]	= LPI_GPIO_FUNC_FUNC3,
	[LPI_GPIO_FUNC_INDEX_FUNC4]	= LPI_GPIO_FUNC_FUNC4,
	[LPI_GPIO_FUNC_INDEX_FUNC5]	= LPI_GPIO_FUNC_FUNC5,
};

static int lpi_gpio_read(struct lpi_pinctrl *state, unsigned int pin,
			 unsigned int addr)
{
	return ioread32(state->tlmm_base +
			state->data->tlmm_reg_offset * pin + addr);
}

static int lpi_gpio_write(struct lpi_pinctrl *state, unsigned int pin,
			  unsigned int addr, unsigned int val)
{
	iowrite32(val, state->tlmm_base +
		  state->data->tlmm_reg_offset * pin + addr);
	pr_err("DEBUG: %s: %x-> %x \n", __func__,
	       state->data->tlmm_reg_offset * pin,  val);

	return 0;
}

static int lpi_gpio_get_groups_count(struct pinctrl_dev *pctldev)
{
	/* Every PIN is a group */
	return pctldev->desc->npins;
}

static const char *lpi_gpio_get_group_name(struct pinctrl_dev *pctldev,
					   unsigned int pin)
{
	struct lpi_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	return pctrl->data->groups[pin];
}

static int lpi_gpio_get_group_pins(struct pinctrl_dev *pctldev,
				   unsigned int pin,
				   const unsigned int **pins,
				   unsigned int *num_pins)
{
	*pins = &pctldev->desc->pins[pin].number;
	*num_pins = 1;

	return 0;
}

static const struct pinctrl_ops lpi_gpio_pinctrl_ops = {
	.get_groups_count	= lpi_gpio_get_groups_count,
	.get_group_name		= lpi_gpio_get_group_name,
	.get_group_pins		= lpi_gpio_get_group_pins,
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_group,
	.dt_free_map		= pinctrl_utils_free_map,
};

static int lpi_gpio_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(lpi_gpio_functions);
}

static const char *lpi_gpio_get_function_name(struct pinctrl_dev *pctldev,
					      unsigned int function)
{
	return lpi_gpio_functions[function];
}

static int lpi_gpio_get_function_groups(struct pinctrl_dev *pctldev,
					unsigned int function,
					const char *const **groups,
					unsigned *const num_qgroups)
{
	struct lpi_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = pctrl->data->groups;
	*num_qgroups = pctrl->data->ngroups;

	return 0;
}

static int lpi_gpio_set_mux(struct pinctrl_dev *pctldev, unsigned int function,
			    unsigned int pin)
{
	struct lpi_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int val;

	val = lpi_gpio_read(pctrl, pin, LPI_GPIO_REG_VAL_CTL);
	val &= ~(LPI_GPIO_REG_FUNCTION_MASK);
	val |= function << LPI_GPIO_REG_FUNCTION_SHIFT;
	lpi_gpio_write(pctrl, pin, LPI_GPIO_REG_VAL_CTL, val);

	return 0;
}

static const struct pinmux_ops lpi_gpio_pinmux_ops = {
	.get_functions_count	= lpi_gpio_get_functions_count,
	.get_function_name	= lpi_gpio_get_function_name,
	.get_function_groups	= lpi_gpio_get_function_groups,
	.set_mux		= lpi_gpio_set_mux,
};

static int lpi_config_get(struct pinctrl_dev *pctldev,
			  unsigned int pin, unsigned long *config)
{
	unsigned int param = pinconf_to_config_param(*config);
	struct lpi_pinctrl *state = dev_get_drvdata(pctldev->dev);
	unsigned int arg = 0;
	int is_out;
	int pull;
	u32 ctl_reg;

	ctl_reg = lpi_gpio_read(state, pin, LPI_GPIO_REG_DIR_CTL);

	is_out = (ctl_reg & LPI_GPIO_REG_DIR_MASK) >> LPI_GPIO_REG_DIR_SHIFT;

	ctl_reg = lpi_gpio_read(state, pin, LPI_GPIO_REG_VAL_CTL);

	pull = (ctl_reg & LPI_GPIO_REG_PULL_MASK) >> LPI_GPIO_REG_PULL_SHIFT;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		if (pull == LPI_GPIO_BIAS_DISABLE)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (pull == LPI_GPIO_PULL_DOWN)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (pull == LPI_GPIO_KEEPER)
			arg = 1;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pull == LPI_GPIO_PULL_UP)
			arg = 1;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		if (is_out)
			arg = 1;
		break;
	default:
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static unsigned int lpi_drive_to_regval(u32 arg)
{
	return (arg/2 - 1);
}

static int lpi_config_set(struct pinctrl_dev *pctldev, unsigned int pin,
			  unsigned long *configs, unsigned int nconfs)
{
	unsigned int param, arg;
	int i, ret = 0;
	volatile unsigned long val;
	struct lpi_pinctrl *state = dev_get_drvdata(pctldev->dev);
	bool            output_enabled;
	unsigned int    pullup;
	unsigned int    strength;
	unsigned int	offset;
	bool            value;

	for (i = 0; i < nconfs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			pullup = LPI_GPIO_BIAS_DISABLE;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			pullup = LPI_GPIO_PULL_DOWN;
			break;
		case PIN_CONFIG_BIAS_BUS_HOLD:
			pullup = LPI_GPIO_KEEPER;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			pullup = LPI_GPIO_PULL_UP;
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			output_enabled = false;
			break;
		case PIN_CONFIG_OUTPUT:
			output_enabled = true;
			lpi_gpio_write(state, pin, LPI_GPIO_REG_DIR_CTL,
			output_enabled << LPI_GPIO_REG_DIR_SHIFT);
			value = arg;
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			strength = arg;
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (arg > LPI_SLEW_RATE_MAX) {
				dev_err(pctldev->dev, "%s: invalid slew rate %u for pin: %d\n",
					__func__, arg, pin);
				goto set_gpio;
			}

			mutex_lock(&state->slew_access_lock);

			val = ioread32(state->slew_base + LPI_SLEW_REG_VAL_CTL);

			offset = state->data->slew_reg_pin_offsets[pin];
			for (i = 0; i < LPI_SLEW_BITS_SIZE; i++) {
				if (arg & 0x01)
					set_bit(offset, &val);
				else
					clear_bit(offset, &val);
				offset++;
				arg = arg >> 1;
			}

			iowrite32(val, state->slew_base + LPI_SLEW_REG_VAL_CTL);

			mutex_unlock(&state->slew_access_lock);
			break;
		default:
			ret = -EINVAL;
			goto done;
		}
	}

set_gpio:

	val = lpi_gpio_read(state, pin, LPI_GPIO_REG_VAL_CTL);
	val &= ~(LPI_GPIO_REG_PULL_MASK | LPI_GPIO_REG_OUT_STRENGTH_MASK |
		 LPI_GPIO_REG_OE_MASK);
	val |= pullup << LPI_GPIO_REG_PULL_SHIFT;
	val |= lpi_drive_to_regval(strength) << LPI_GPIO_REG_OUT_STRENGTH_SHIFT;
	if (output_enabled)
		val |= value << LPI_GPIO_REG_OE_SHIFT;

	lpi_gpio_write(state, pin, LPI_GPIO_REG_VAL_CTL, val);
	lpi_gpio_write(state, pin, LPI_GPIO_REG_DIR_CTL,
		       output_enabled << LPI_GPIO_REG_DIR_SHIFT);
done:
	return ret;
}

static const struct pinconf_ops lpi_gpio_pinconf_ops = {
	.is_generic			= true,
	.pin_config_group_get		= lpi_config_get,
	.pin_config_group_set		= lpi_config_set,
};

static int lpi_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	struct lpi_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_INPUT_ENABLE, 1);

	return lpi_config_set(state->ctrl, pin, &config, 1);
}

static int lpi_gpio_direction_output(struct gpio_chip *chip,
				     unsigned int pin, int val)
{
	struct lpi_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, val);

	return lpi_config_set(state->ctrl, pin, &config, 1);
}

static int lpi_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	struct lpi_pinctrl *state = gpiochip_get_data(chip);
	int value;

	value = lpi_gpio_read(state, pin, LPI_GPIO_REG_VAL_CTL);
	return value;
}

static void lpi_gpio_set(struct gpio_chip *chip, unsigned int pin, int value)
{
	struct lpi_pinctrl *state = gpiochip_get_data(chip);
	unsigned long config;

	config = pinconf_to_config_packed(PIN_CONFIG_OUTPUT, value);

	lpi_config_set(state->ctrl, pin, &config, 1);
}
#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static unsigned int lpi_regval_to_drive(u32 val)
{
	return (val + 1) * 2;
}

static void lpi_gpio_dbg_show_one(struct seq_file *s,
				  struct pinctrl_dev *pctldev,
				  struct gpio_chip *chip,
				  unsigned int offset,
				  unsigned int gpio)
{
	struct lpi_pinctrl *state = gpiochip_get_data(chip);
	struct pinctrl_pin_desc pindesc;
	unsigned int func;
	int is_out;
	int drive;
	int pull;
	u32 ctl_reg;

	static const char * const pulls[] = {
		"no pull",
		"pull down",
		"keeper",
		"pull up"
	};

	pctldev = pctldev ? : state->ctrl;
	pindesc = pctldev->desc->pins[offset];
	ctl_reg = lpi_gpio_read(state, offset, LPI_GPIO_REG_DIR_CTL);
	is_out = (ctl_reg & LPI_GPIO_REG_DIR_MASK) >> LPI_GPIO_REG_DIR_SHIFT;
	ctl_reg = lpi_gpio_read(state, offset, LPI_GPIO_REG_VAL_CTL);

	func = (ctl_reg & LPI_GPIO_REG_FUNCTION_MASK) >>
		LPI_GPIO_REG_FUNCTION_SHIFT;
	drive = (ctl_reg & LPI_GPIO_REG_OUT_STRENGTH_MASK) >>
		 LPI_GPIO_REG_OUT_STRENGTH_SHIFT;
	pull = (ctl_reg & LPI_GPIO_REG_PULL_MASK) >> LPI_GPIO_REG_PULL_SHIFT;

	seq_printf(s, " %-8s: %-3s %d",
		   pindesc.name, is_out ? "out" : "in", func);
	seq_printf(s, " %dmA", lpi_regval_to_drive(drive));
	seq_printf(s, " %s", pulls[pull]);
}

static void lpi_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	unsigned int gpio = chip->base;
	unsigned int i;

	for (i = 0; i < chip->ngpio; i++, gpio++) {
		lpi_gpio_dbg_show_one(s, NULL, chip, i, gpio);
		seq_puts(s, "\n");
	}
}

#else
#define lpi_gpio_dbg_show NULL
#endif

static const struct gpio_chip lpi_gpio_template = {
	.direction_input	= lpi_gpio_direction_input,
	.direction_output	= lpi_gpio_direction_output,
	.get			= lpi_gpio_get,
	.set			= lpi_gpio_set,
	.request		= gpiochip_generic_request,
	.free			= gpiochip_generic_free,
	.dbg_show		= lpi_gpio_dbg_show,
};

static int lpi_pinctrl_probe(struct platform_device *pdev)
{
	int ret, npins;
	struct clk *core_vote = NULL;
	struct clk *audio_vote = NULL;

	struct lpi_pinctrl *pctrl;
	const struct lpi_pinctrl_variant_data *data;
	struct device *dev = &pdev->dev;
	struct resource *res;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pctrl);

	data = of_device_get_match_data(dev);
	pctrl->data = data;
	pctrl->dev = &pdev->dev;
	npins = data->npins;

	/* Register LPASS core hw vote */
	core_vote = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(core_vote)) {
		dev_dbg(&pdev->dev, "%s: clk get %s failed %d\n",
			__func__, "core_vote", ret);
		return PTR_ERR(core_vote);
	}
	pctrl->core_vote = core_vote;

	/* Register LPASS audio hw vote */
	audio_vote = devm_clk_get(&pdev->dev, "audio");
	if (IS_ERR(audio_vote)) {
		dev_dbg(&pdev->dev, "%s: clk get %s failed %d\n",
			__func__, "audio_vote", ret);
		return PTR_ERR(audio_vote);
	}

	pctrl->audio_vote = audio_vote;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pctrl->tlmm_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctrl->tlmm_base))
		return PTR_ERR(pctrl->tlmm_base);

	clk_prepare_enable(pctrl->core_vote);
	clk_prepare_enable(pctrl->audio_vote);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pctrl->slew_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctrl->slew_base))
		return PTR_ERR(pctrl->slew_base);

	pctrl->desc.pctlops = &lpi_gpio_pinctrl_ops;
	pctrl->desc.pmxops = &lpi_gpio_pinmux_ops;
	pctrl->desc.confops = &lpi_gpio_pinconf_ops;

	pctrl->desc.owner = THIS_MODULE;
	pctrl->desc.name = dev_name(dev);
	pctrl->desc.pins = data->pins;
	pctrl->desc.npins = data->npins;

	pctrl->chip = lpi_gpio_template;
	pctrl->chip.parent = dev;
	pctrl->chip.base = -1;
	pctrl->chip.ngpio = npins;
	pctrl->chip.label = dev_name(dev);
	pctrl->chip.of_gpio_n_cells = 2;
	pctrl->chip.can_sleep = false;

	mutex_init(&pctrl->slew_access_lock);

	pctrl->ctrl = devm_pinctrl_register(dev, &pctrl->desc, pctrl);
	if (IS_ERR(pctrl->ctrl))
		return PTR_ERR(pctrl->ctrl);

	ret = gpiochip_add_data(&pctrl->chip, pctrl);
	if (ret) {
		dev_err(pctrl->dev, "can't add gpio chip\n");
		goto err_chip;
	}

	ret = gpiochip_add_pin_range(&pctrl->chip, dev_name(dev), 0, 0, npins);
	if (ret) {
		dev_err(dev, "failed to add pin range\n");
		goto err_range;
	}

	return 0;

err_range:
	gpiochip_remove(&pctrl->chip);
err_chip:
	mutex_destroy(&pctrl->slew_access_lock);
	return ret;
}

static int lpi_pinctrl_remove(struct platform_device *pdev)
{
	struct lpi_pinctrl *state = platform_get_drvdata(pdev);

	gpiochip_remove(&state->chip);
	mutex_destroy(&state->slew_access_lock);

	return 0;
}

static const struct of_device_id lpi_pinctrl_of_match[] = {
	{
	       .compatible = "qcom,sm8250-lpass-lpi-pinctrl",
	       .data = &sm8250_lpi_data,
	},
	{ },
};

MODULE_DEVICE_TABLE(of, lpi_pinctrl_of_match);

static struct platform_driver lpi_pinctrl_driver = {
	.driver = {
		   .name = "qcom-lpass-lpi-pinctrl",
		   .of_match_table = lpi_pinctrl_of_match,
	},
	.probe = lpi_pinctrl_probe,
	.remove = lpi_pinctrl_remove,
};

module_platform_driver(lpi_pinctrl_driver);

MODULE_DESCRIPTION("QTI LPI GPIO pin control driver");
MODULE_LICENSE("GPL v2");
