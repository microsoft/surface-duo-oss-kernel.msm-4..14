/*
 * Copyright (c) 2014, Sony Mobile Communications AB.
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/hwspinlock.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/mfd/syscon.h>

#include <linux/delay.h>

#include <linux/soc/qcom/smem.h>

#define SMSM_APPS_STATE 0
#define SMEM_SMSM_SHARED_STATE 85

#define SMSM_MAX_STATES 8

struct qcom_smsm_state {
	unsigned state_id;
	struct gpio_chip chip;

	int irq;

	struct regmap *ipc_regmap;
	int ipc_bit;
	int ipc_offset;
};

struct qcom_smsm {
	struct device *dev;

	u32 *shared_state;
	size_t shared_state_size;

	struct qcom_smsm_state states[SMSM_MAX_STATES];
};

#if 0
int qcom_smsm_change_state(struct qcom_smsm *smsm, u32 clear_mask, u32 set_mask)
{
	u32 state;

	dev_dbg(smsm->dev, "SMSM_APPS_STATE clear 0x%x set 0x%x\n", clear_mask, set_mask);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	state = readl(&smsm->shared_state[SMSM_APPS_STATE]);
	state &= ~clear_mask;
	state |= set_mask;
	writel(state, &smsm->shared_state[SMSM_APPS_STATE]);

	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	// qcom_smem_signal(-1, smsm->smem, smsm->signal_offset, smsm->signal_bit);

	return 0;
}
EXPORT_SYMBOL(qcom_smsm_change_state);
#endif

static struct qcom_smsm_state *to_smsm_state(struct gpio_chip *chip)
{
	return container_of(chip, struct qcom_smsm_state, chip);
}

static struct qcom_smsm *to_smsm(struct qcom_smsm_state *state)
{
	return container_of(state, struct qcom_smsm, states[state->state_id]);
}

static int smsm_gpio_direction_input(struct gpio_chip *chip,
				     unsigned offset)
{
	struct qcom_smsm_state *state = to_smsm_state(chip);

	if (state->state_id == SMSM_APPS_STATE)
		return -EINVAL;
	return 0;
}

static int smsm_gpio_direction_output(struct gpio_chip *chip,
				      unsigned offset,
				      int value)
{
	struct qcom_smsm_state *ipc_state;
	struct qcom_smsm_state *state = to_smsm_state(chip);
	struct qcom_smsm *smsm = to_smsm(state);
	unsigned word;
	unsigned bit;
	u32 val;
	int i;

	/* Only SMSM_APPS_STATE supports writing */
	if (state->state_id != SMSM_APPS_STATE)
		return -EINVAL;

	offset += state->state_id * 32;

	word = ALIGN(offset / 32, 4);
	bit = offset % 32;

	val = readl(smsm->shared_state + word);
	if (value)
		val |= BIT(bit);
	else
		val &= ~BIT(bit);
	writel(val, smsm->shared_state + word);

	/* XXX: send out interrupts */
	for (i = 0; i < SMSM_MAX_STATES; i++) {
		ipc_state = &smsm->states[i];
		if (!ipc_state->ipc_regmap)
			continue;

		regmap_write(ipc_state->ipc_regmap, ipc_state->ipc_offset, BIT(ipc_state->ipc_bit));
	}

	dev_err(smsm->dev, "set %d %d\n", offset, value);

	return 0;
}

static int smsm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct qcom_smsm_state *state = to_smsm_state(chip);
	struct qcom_smsm *smsm = to_smsm(state);
	unsigned word;
	unsigned bit;
	u32 val;

	offset += state->state_id * 32;

	word = ALIGN(offset / 32, 4);
	bit = offset % 32;

	val = readl(smsm->shared_state + word);

	return !!(val & BIT(bit));
}

static void smsm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	smsm_gpio_direction_output(chip, offset, value);
}

static int smsm_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return -EINVAL;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static void smsm_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct qcom_smsm_state *state = to_smsm_state(chip);
	struct qcom_smsm *smsm = to_smsm(state);
	unsigned i;
	u32 val;

	val = readl(smsm->shared_state + state->state_id * 4);

	for (i = 0; i < 32; i++) {
		if (val & BIT(i))
			seq_puts(s, "1");
		else
			seq_puts(s, "0");

		if (i == 7 || i == 15 || i == 23)
			seq_puts(s, " ");
	}
	seq_puts(s, "\n");
}

#else
#define smsm_gpio_dbg_show NULL
#endif

static struct gpio_chip smsm_gpio_template = {
	.direction_input = smsm_gpio_direction_input,
	.direction_output = smsm_gpio_direction_output,
	.get = smsm_gpio_get,
	.set = smsm_gpio_set,
	.to_irq = smsm_gpio_to_irq,
	.dbg_show = smsm_gpio_dbg_show,
	.owner = THIS_MODULE,
};

static int qcom_smsm_probe(struct platform_device *pdev)
{
	struct qcom_smsm_state *state;
	struct device_node *syscon_np;
	struct device_node *node;
	struct qcom_smsm *smsm;
	char *key;
	u32 sid;
	int ret;

	smsm = devm_kzalloc(&pdev->dev, sizeof(*smsm), GFP_KERNEL);
	if (!smsm)
		return -ENOMEM;
	smsm->dev = &pdev->dev;

	ret = qcom_smem_alloc(-1, SMEM_SMSM_SHARED_STATE, 8 * sizeof(uint32_t));
	if (ret < 0 && ret != -EEXIST) {
		dev_err(&pdev->dev, "unable to allocate shared state entry\n");
		return ret;
	}

	ret = qcom_smem_get(-1, SMEM_SMSM_SHARED_STATE,
			    (void**)&smsm->shared_state,
			    &smsm->shared_state_size);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to acquire shared state entry\n");
		return ret;
	}

	dev_err(smsm->dev, "SMEM_SMSM_SHARED_STATE: %d, %zu\n", ret, smsm->shared_state_size);
	print_hex_dump(KERN_DEBUG, "raw data: ", DUMP_PREFIX_OFFSET, 16, 1, smsm->shared_state, smsm->shared_state_size, true);

	for_each_child_of_node(pdev->dev.of_node, node) {
		key = "reg";
		ret = of_property_read_u32(node, key, &sid);
		if (ret || sid >= SMSM_MAX_STATES) {
			dev_err(&pdev->dev, "smsm state missing %s\n", key);
			return -EINVAL;
		}
		state = &smsm->states[sid];
		state->state_id = sid;

		state->chip = smsm_gpio_template;
		state->chip.base = -1;
		state->chip.dev = &pdev->dev;
		state->chip.of_node = node;
		state->chip.label = node->name;
		state->chip.ngpio = 8 * sizeof(u32);
		ret = gpiochip_add(&state->chip);
		if (ret) {
			dev_err(&pdev->dev, "failed register gpiochip\n");
			// goto wooha;
		}

		/* The remaining properties are only for non-apps state */
		if (sid == SMSM_APPS_STATE)
			continue;

		state->irq = irq_of_parse_and_map(node, 0);
		if (state->irq < 0 && state->irq != -EINVAL) {
			dev_err(&pdev->dev, "failed to parse smsm interrupt\n");
			return -EINVAL;
		}

		syscon_np = of_parse_phandle(node, "qcom,ipc", 0);
		if (!syscon_np) {
			dev_err(&pdev->dev, "no qcom,ipc node\n");
			return -ENODEV;
		}

		state->ipc_regmap = syscon_node_to_regmap(syscon_np);
		if (IS_ERR(state->ipc_regmap))
			return PTR_ERR(state->ipc_regmap);

		key = "qcom,ipc";
		ret = of_property_read_u32_index(node, key, 1, &state->ipc_offset);
		if (ret < 0) {
			dev_err(&pdev->dev, "no offset in %s\n", key);
			return -EINVAL;
		}

		ret = of_property_read_u32_index(node, key, 2, &state->ipc_bit);
		if (ret < 0) {
			dev_err(&pdev->dev, "no bit in %s\n", key);
			return -EINVAL;
		}

	}

	return 0;
}

static const struct of_device_id qcom_smsm_of_match[] = {
	{ .compatible = "qcom,smsm" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_smsm_of_match);

static struct platform_driver qcom_smsm_driver = {
	.probe          = qcom_smsm_probe,
	.driver  = {
		.name  = "qcom_smsm",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smsm_of_match,
	},
};

static int __init qcom_smsm_init(void)
{
	return platform_driver_register(&qcom_smsm_driver);
}
arch_initcall(qcom_smsm_init);

static void __exit qcom_smsm_exit(void)
{
	platform_driver_unregister(&qcom_smsm_driver);
}
module_exit(qcom_smsm_exit)

MODULE_DESCRIPTION("Qualcomm Shared Memory Signaling Mechanism");
MODULE_LICENSE("GPLv2");
