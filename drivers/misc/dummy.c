#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

struct vreg {
	const char *name;
	int minvolt;
	int maxvolt;
} vregs [8] = {
	{ "vddio", 1800000, 1800000 },
	{ "vddaon", 950000, 950000 },
	{ "vddpmu", 950000, 950000 },
	{ "vddrfa1", 950000, 950000 },
	{ "vddrfa2", 1380000, 1380000 },
	{ "vddrfa3", 2000000, 2000000 },
	{ "vddpcie1", 1350000, 1350000 },
	{ "vddpcie2", 2000000, 2000000 },
};

static int dummy_probe(struct platform_device *pdev)
{
	struct regulator_bulk_data regulators[8];
	struct device *dev = &pdev->dev;
	int i, ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *wlan_en_active;
	struct pinctrl_state *bt_en_active;

	for (i = 0; i < ARRAY_SIZE(regulators); i++)
		regulators[i].supply = vregs[i].name;
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(regulators), regulators);
	if (ret < 0)
		return ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(regulators), regulators);
	if (ret)
		return ret;

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
                ret = PTR_ERR(pinctrl);
                pr_err("Failed to get pinctrl, err = %d\n", ret);
                return ret;
        }

	wlan_en_active = pinctrl_lookup_state(pinctrl, "wlan_en_active");
	if (IS_ERR_OR_NULL(wlan_en_active)) {
		ret = PTR_ERR(wlan_en_active);
		pr_err("Failed to get wlan_en_active, err = %d\n", ret);
		return ret;
	}

	bt_en_active = pinctrl_lookup_state(pinctrl, "bt_en_active");
	if (IS_ERR_OR_NULL(bt_en_active)) {
		ret = PTR_ERR(bt_en_active);
		pr_err("Failed to get bt_en_active, err = %d\n", ret);
		return ret;
	}

	ret = pinctrl_select_state(pinctrl, wlan_en_active);
	if (ret) {
		pr_err("Failed to select wlan pinctrl state");
		return ret;
	}
	udelay(1000);

	ret = pinctrl_select_state(pinctrl, bt_en_active);
	if (ret) {
		pr_err("Failed to select bt pinctrl state");
		return ret;
	}
	udelay(1000);

	return 0;
}

static int dummy_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dummy_of_match[] = {
	{ .compatible = "qcom,dummy" },
};

static struct platform_driver dummy_driver = {
	.probe = dummy_probe,
	.remove = dummy_remove,
	.driver = {
		.name = "dummy",
		.of_match_table = dummy_of_match,
	},
};

static int __init dummy_driver_init(void)
{
	return platform_driver_register(&dummy_driver);
}

postcore_initcall(dummy_driver_init);
MODULE_LICENSE("GPL v2");
