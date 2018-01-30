#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slimbus/slimbus.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>

#include "wcd9335.h"
#include "wcd9335_registers.h"


extern struct regmap_config wcd9335_regmap_config;

static const struct regmap_irq wcd9335_irqs[] = {
	/* INTR_REG 0 */
	[WCD9335_IRQ_SLIMBUS] = {
		.reg_offset = 0,
		.mask = BIT(0),
	},
	[WCD9335_IRQ_FLL_LOCK_LOSS] = {
		.reg_offset = 0,
		.mask = BIT(1),
	},
	[WCD9335_IRQ_HPH_PA_OCPL_FAULT] = {
		.reg_offset = 0,
		.mask = BIT(2),
	},
	[WCD9335_IRQ_HPH_PA_OCPR_FAULT] = {
		.reg_offset = 0,
		.mask = BIT(3),
	},
	[WCD9335_IRQ_EAR_PA_OCP_FAULT] = {
		.reg_offset = 0,
		.mask = BIT(4),
	},
	[WCD9335_IRQ_HPH_PA_CNPL_COMPLETE] = {
		.reg_offset = 0,
		.mask = BIT(5),
	},
	[WCD9335_IRQ_HPH_PA_CNPR_COMPLETE] = {
		.reg_offset = 0,
		.mask = BIT(6),
	},
	[WCD9335_IRQ_EAR_PA_CNP_COMPLETE] = {
		.reg_offset = 0,
		.mask = BIT(7),
	},
	/* INTR_REG 1 */
	[WCD9335_IRQ_MBHC_SW_DET] = {
		.reg_offset = 1,
		.mask = BIT(0),
	},
	[WCD9335_IRQ_MBHC_ELECT_INS_REM_DET] = {
		.reg_offset = 1,
		.mask = BIT(1),
	},
	[WCD9335_IRQ_MBHC_BUTTON_PRESS_DET] = {
		.reg_offset = 1,
		.mask = BIT(2),
	},
	[WCD9335_IRQ_MBHC_BUTTON_RELEASE_DET] = {
		.reg_offset = 1,
		.mask = BIT(3),
	},
	[WCD9335_IRQ_MBHC_ELECT_INS_REM_LEG_DET] = {
		.reg_offset = 1,
		.mask = BIT(4),
	},
	/* INTR_REG 2 */
	[WCD9335_IRQ_LINE_PA1_CNP_COMPLETE] = {
		.reg_offset = 2,
		.mask = BIT(0),
	},
	[WCD9335_IRQ_LINE_PA2_CNP_COMPLETE] = {
		.reg_offset = 2,
		.mask = BIT(1),
	},
	[WCD9335_IRQ_LINE_PA3_CNP_COMPLETE] = {
		.reg_offset = 2,
		.mask = BIT(2),
	},
	[WCD9335_IRQ_LINE_PA4_CNP_COMPLETE] = {
		.reg_offset = 2,
		.mask = BIT(3),
	},
	[WCD9335_IRQ_SOUNDWIRE] = {
		.reg_offset = 2,
		.mask = BIT(4),
	},
	[WCD9335_IRQ_VDD_DIG_RAMP_COMPLETE] = {
		.reg_offset = 2,
		.mask = BIT(5),
	},
	[WCD9335_IRQ_RCO_ERROR] = {
		.reg_offset = 2,
		.mask = BIT(6),
	},
	[WCD9335_IRQ_SVA_ERROR] = {
		.reg_offset = 2,
		.mask = BIT(7),
	},
	/* INTR_REG 3 */
	[WCD9335_IRQ_MAD_AUDIO] = {
		.reg_offset = 3,
		.mask = BIT(0),
	},
	[WCD9335_IRQ_MAD_BEACON] = {
		.reg_offset = 3,
		.mask = BIT(1),
	},
	[WCD9335_IRQ_MAD_ULTRASOUND] = {
		.reg_offset = 3,
		.mask = BIT(2),
	},
	[WCD9335_IRQ_VBAT_ATTACK] = {
		.reg_offset = 3,
		.mask = BIT(3),
	},
	[WCD9335_IRQ_VBAT_RESTORE] = {
		.reg_offset = 3,
		.mask = BIT(4),
	},
	[WCD9335_IRQ_SVA_OUTBOX1] = {
		.reg_offset = 3,
		.mask = BIT(5),
	},
	[WCD9335_IRQ_SVA_OUTBOX2] = {
		.reg_offset = 3,
		.mask = BIT(6),
	},
};

static const struct regmap_irq_chip wcd9335_regmap_irq1_chip = {
        .name = "wcd9335_pin1_irq",
        .status_base = WCD9335_INTR_PIN1_STATUS0,
        .mask_base = WCD9335_INTR_PIN1_MASK0,
        .ack_base = WCD9335_INTR_PIN1_CLEAR0,
        .type_base = WCD9335_INTR_LEVEL0,
        .num_regs = 4,
        .irqs = wcd9335_irqs,
        .num_irqs = ARRAY_SIZE(wcd9335_irqs),
};

static const struct regmap_irq_chip wcd9335_regmap_irq2_chip = {
        .name = "wcd9335_pin2_irq",
        .status_base = WCD9335_INTR_PIN2_STATUS0,
        .mask_base = WCD9335_INTR_PIN2_MASK0,
        .ack_base = WCD9335_INTR_PIN2_CLEAR0,
        .type_base = WCD9335_INTR_LEVEL0,
        .num_regs = 4,
        .irqs = wcd9335_irqs,
        .num_irqs = ARRAY_SIZE(wcd9335_irqs),
};
static struct regmap_config wcd9335_ifd_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
//	.can_multi_write = true,
};

static int wcd9335_slim_get_laddr(struct slim_device *sb,
				  const u8 *e_addr, u8 e_len, u8 *laddr)
{
	int ret;
	const unsigned long timeout = jiffies +
				      msecs_to_jiffies(SLIMBUS_PRESENT_TIMEOUT);

	do {
		ret = slim_get_logical_addr(sb, e_addr, e_len, laddr);
		if (!ret)
			break;
		/* Give SLIMBUS time to report present and be ready. */
		usleep_range(1000, 1100);
		pr_debug_ratelimited("%s: retyring get logical addr\n",
				     __func__);
	} while time_before(jiffies, timeout);

	return ret;
}

static int wcd9335_parse_slim_ifd_info(struct device *dev, struct slim_device *slim_ifd)
{
	int ret = 0;
	struct property *prop;

	ret = of_property_read_string(dev->of_node, "qcom,cdc-slim-ifd",
				      &slim_ifd->name);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"qcom,cdc-slim-ifd-dev", dev->of_node->full_name);
		return -ENODEV;
	}
	prop = of_find_property(dev->of_node,
			"qcom,cdc-slim-ifd-elemental-addr", NULL);
	if (!prop) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"qcom,cdc-slim-ifd-elemental-addr",
			dev->of_node->full_name);
		return -ENODEV;
	} else if (prop->length != 6) {
		dev_err(dev, "invalid codec slim ifd addr. addr length = %d\n",
			      prop->length);
		return -ENODEV;
	}
	memcpy(slim_ifd->e_addr, prop->value, 6);

	return 0;
}

static int wcd9335_parse_dt(struct wcd9335 *wcd)
{
	struct device *dev = wcd->dev;
	struct device_node *np = dev->of_node;
	int ret;

	wcd->reset_gpio = of_get_named_gpio(np,	"qcom,cdc-reset-gpio", 0);
	if (wcd->reset_gpio < 0) {
		dev_err(dev, "Reset gpio missing in DT\n");
		return -EINVAL;
	}

	wcd->irq_gpio = of_get_named_gpio(np, "qcom,gpio-int2", 0);
	if (!gpio_is_valid(wcd->irq_gpio)) {
		dev_err(dev, "IRQ gpio missing in DT\n");
		return -EINVAL;
	}

	wcd->clk1_gpio = of_get_named_gpio(np, "qcom,clk1-gpio", 0);
	if (!gpio_is_valid(wcd->clk1_gpio)) {
		dev_err(dev, "CLK gpio missing in DT\n");
		return -EINVAL;
	}

	gpio_request(wcd->clk1_gpio, "CLK1");
	gpio_direction_output(wcd->clk1_gpio, 0);

	//FIXME should go in to machine driver 
	ret = of_property_read_u32(np, "qcom,cdc-mclk-clk-rate", &wcd->mclk_rate);
	if (ret) {
		dev_err(dev, "Reset mclk rate missing in DT\n");
		return -EINVAL;
	}

	if (wcd->mclk_rate != WCD9XXX_MCLK_CLK_9P6HZ &&
	    wcd->mclk_rate != WCD9XXX_MCLK_CLK_12P288MHZ) {
		dev_err(dev, "Invalid mclk_rate = %u\n", wcd->mclk_rate);
		return -EINVAL;
	}
	wcd->ext_clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(wcd->ext_clk)) {
		dev_err(dev, "Unable to find external clk\n");
		return -EINVAL;
	}

	wcd->native_clk = devm_clk_get(dev, "native");
	if (IS_ERR(wcd->native_clk)) {
		dev_err(dev, "Unable to find native clk\n");
		return -EINVAL;
	}
	
	return 0;
}

void wcd9335_reset(struct wcd9335 *wcd)
{
	gpio_direction_output(wcd->reset_gpio, 0);
	msleep(20);
	gpio_set_value(wcd->reset_gpio, 1);
	msleep(20);
}

int wcd9335_power_up(struct wcd9335 *wcd)
{
	struct device *dev = wcd->dev;
	int ret;

	wcd->num_of_supplies = 5;
	wcd->supplies[0].supply = "vdd-buck";
	wcd->supplies[1].supply = "buck-sido";
	wcd->supplies[2].supply = "vdd-tx-h";
	wcd->supplies[3].supply = "vdd-rx-h";
	wcd->supplies[4].supply = "vddpx-1";

	ret = regulator_bulk_get(dev, wcd->num_of_supplies, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(wcd->num_of_supplies, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	/*
	 * For WCD9335, it takes about 600us for the Vout_A and
	 * Vout_D to be ready after BUCK_SIDO is powered up.
	 * SYS_RST_N shouldn't be pulled high during this time
	 */
	usleep_range(600, 650);


	return 0;
}

static int wcd9335_init(struct wcd9335 *wcd)
{
	int ret;
	
	wcd->irq = gpio_to_irq(wcd->irq_gpio);
	if (wcd->irq < 0) {
		pr_err("Unable to configure irq\n");
		return wcd->irq;
	}
	ret = regmap_add_irq_chip(wcd->regmap, wcd->irq,
			   	 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				 0, &wcd9335_regmap_irq1_chip,
				 &wcd->irq_data);
	if (ret != 0) {
       	        pr_err("Failed to register IRQ chip: %d\n", ret);
               	return ret;
       	}

	return 0;
}

static int wcd9335_bring_up(struct wcd9335 *wcd)
{
	int val, byte0;
	int ret = 0;

	regmap_read(wcd->regmap,
				 WCD9335_CHIP_TIER_CTRL_EFUSE_VAL_OUT0, &val);
	regmap_read(wcd->regmap,
				   WCD9335_CHIP_TIER_CTRL_CHIP_ID_BYTE0, &byte0);

	if ((val < 0) || (byte0 < 0)) {
		dev_err(wcd->dev, "%s: tasha codec version detection fail!\n",
			__func__);
		return -EINVAL;
	}


	if ((val & 0x80) && (byte0 == 0x0)) {
		dev_info(wcd->dev, "%s: wcd9335 codec version is v1.1\n",
			 __func__);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x01);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_CCL_2, 0xFC);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_CCL_4, 0x21);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x5);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x7);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x3);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x3);
	} else if (byte0 == 0x1) {
		dev_info(wcd->dev, "%s: wcd9335 codec version is v2.0\n",
			 __func__);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x01);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_TEST_2, 0x00);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_CCL_8, 0x6F);
		regmap_write(wcd->regmap, WCD9335_BIAS_VBG_FINE_ADJ, 0x65);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x5);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x7);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x3);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x3);
	} else if ((byte0 == 0) && (!(val & 0x80))) {
		dev_info(wcd->dev, "%s: wcd9335 codec version is v1.0\n",
			 __func__);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x01);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_CCL_2, 0xFC);
		regmap_write(wcd->regmap, WCD9335_SIDO_SIDO_CCL_4, 0x21);
		regmap_write(wcd->regmap,
				    WCD9335_CODEC_RPM_PWR_CDC_DIG_HM_CTL, 0x3);
		regmap_write(wcd->regmap, WCD9335_CODEC_RPM_RST_CTL, 0x3);
	} else {
		dev_err(wcd->dev, "%s: tasha codec version unknown\n",
			__func__);
		ret = -EINVAL;
	}
	return ret;
}

static int wcd9335_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcd9335 *wcd;
	int ret = 0;

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return	-ENOMEM;

	wcd->slim = slim;
	wcd->dev = dev;


	ret = wcd9335_power_up(wcd);
	if (ret) {
		dev_err(dev, "Error parsing DT\n");
		return ret;
	}

	ret = wcd9335_parse_dt(wcd);

	ret = wcd9335_parse_slim_ifd_info(dev, &wcd->slim_ifd);
	if (ret) {
		dev_err(&slim->dev, "Error, parsing slim interface\n");
		return ret;
	}




	wcd->regmap = regmap_init_slimbus(slim, &wcd9335_regmap_config);

	if (IS_ERR(wcd->regmap)) {
		ret = PTR_ERR(wcd->regmap);
		dev_err(&slim->dev, "%s: Failed to allocate register map: %d\n",
			__func__, ret);
		return ret;
	}


	slim_set_clientdata(slim, wcd);

	usleep_range(600, 650);
	wcd9335_reset(wcd);

	ret = wcd9335_slim_get_laddr(wcd->slim, wcd->slim->e_addr,
				     ARRAY_SIZE(wcd->slim->e_addr),
				     &wcd->slim->laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, wcd->slim->name, ret);
		ret = -EPROBE_DEFER;
		return ret;
	}

	wcd9335_init(wcd);

	ret = slim_add_device(slim->ctrl, &wcd->slim_ifd);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		return ret;
	}

	ret = wcd9335_slim_get_laddr(&wcd->slim_ifd,
				     wcd->slim_ifd.e_addr,
				     ARRAY_SIZE(wcd->slim_ifd.e_addr),
				     &wcd->slim_ifd.laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, wcd->slim->name, ret);
		ret = -EPROBE_DEFER;
		return ret;
	}

	wcd->ifd_regmap = regmap_init_slimbus(&wcd->slim_ifd, &wcd9335_ifd_regmap_config);
	if (IS_ERR(wcd->ifd_regmap)) {
		ret = PTR_ERR(wcd->ifd_regmap);
		dev_err(dev, "%s: Failed to allocate register map: %d\n",
			__func__, ret);
		return ret;
	}


	dev_set_drvdata(dev, wcd);
	//FIXME..
	
	ret = wcd9335_bring_up(wcd);
	if (ret) {
		pr_err("DEBUG:: %s: %d\n", __func__, __LINE__);
	}


	wcd->version = 2;
	wcd->dev = dev;
	wcd->slim_slave = &wcd->slim_ifd;
	wcd->intf_type = WCD_INTERFACE_TYPE_SLIMBUS;

#if 1
	wcd->slim_data.slim = slim;
	wcd->slim_data.slim_slave = &wcd->slim_ifd;
	wcd->slim_data.regmap = wcd->regmap;
	wcd->slim_data.if_regmap = wcd->ifd_regmap;
	//FIXME
	wcd->slim_data.rx_port_ch_reg_base = 0x180 - (WCD9335_RX_SLAVE_PORTS * 4);
	wcd->slim_data.port_rx_cfg_reg_base = 0x040 - WCD9335_RX_SLAVE_PORTS;
	wcd->slim_data.port_tx_cfg_reg_base = 0x050;

#endif
	//FIXME
//	ret = wcd9335_regmap_register_patch(wcd->regmap, wcd->version);
//	if (ret) {
//		dev_err(&slim->dev, "Failed to patch register map\n");
//		return ret;
//	}

	return of_platform_populate(wcd->dev->of_node, NULL, NULL, wcd->dev);
}

static int wcd9335_slim_remove(struct slim_device *sdev)
{
	return 0;
}

static const struct slim_device_id wcd9335_slim_id[] = {
	{"tasha-slim-pgd", 0},
	{}
};

static struct slim_driver wcd9335_slim_driver = {
	.driver = {
		.name = "wcd9335-slim",
		.owner = THIS_MODULE,
	},
	.probe = wcd9335_slim_probe,
	.remove = wcd9335_slim_remove,
	.id_table = wcd9335_slim_id,
};

static int __init wcd_init(void)
{
	return  slim_driver_register(&wcd9335_slim_driver);
}
module_init(wcd_init);

static void __exit wcd_exit(void)
{
}
module_exit(wcd_exit);

MODULE_DESCRIPTION("Codec core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
