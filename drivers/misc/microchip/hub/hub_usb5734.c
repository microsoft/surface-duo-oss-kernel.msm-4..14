/*
 * otgid_gpio_hub.c
 *
 * Copyright (c) Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/param.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/hisi/log/hisi_log.h>
#include <linux/hisi/usb/hisi_usb.h>
#include <linux/tifm.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include "hub_usb5734.h"

#define DEVICE_DRIVER_NAME "gpio_hub_for_usb5734"
#define DEVICE_TREE_COMPATIBLE_NAME "hisilicon,gpio_hub"

#define GPIO_HUB_OTG_HOST 1
#define GPIO_HUB_OTG_DEVICE 0
#define GPIO_TYPEC_VBUS_POWER 0
#define GPIO_TYPEC_NO_POWER 1
#define GPIO_HUB_VBUS_POWER 1
#define GPIO_HUB_VBUS_NO_POWER 0
#define GPIO_HUB_HUB_VBUS_POWER 1

/* 寄存器说明：外设时钟使能寄存器1。
   位域定义UNION结构:  SOC_CRGPERIPH_PEREN1_UNION */
#define SOC_CRGPERIPH_PEREN1_ADDR(base)               ((base) + (0x010))

#define HISILOG_TAG GPIO_HUB
HISILOG_REGIST();

struct gpio_hub_info
{
   int otg_switch_gpio;
   int typec_vbus_gpio;
   int hub_vbus_gpio;
};
static struct gpio_hub_info gpio_hub_driver_info = {
    .otg_switch_gpio = -1,
    .typec_vbus_gpio = -1,
    .hub_vbus_gpio = -1,
};

static void gpio_hub_gpio_free(void)
{
    gpio_free(gpio_hub_driver_info.hub_vbus_gpio);
    gpio_hub_driver_info.hub_vbus_gpio = -1;

    gpio_free(gpio_hub_driver_info.typec_vbus_gpio);
    gpio_hub_driver_info.typec_vbus_gpio = -1;
}

static void gpio_hub_vbus_power_on(void)
{
    struct device_node * root;
    int vbus_gpio_status;
    int set_vbus_gpio;
    int set_hub_vbus_gpio;
    unsigned int vdd33;
    void __iomem *crgctrl;
    struct device_node *np;
    hisilog_info("%s: step in\n", __func__);

    np = of_find_compatible_node(NULL, NULL, "hisilicon,crgctrl");
    if (!np) {
           hisilog_info("%s:get crgctrl error.\n", __func__);
	    return -ENXIO;
    }
    crgctrl = of_iomap(np, 0);
    writel(0x1, SOC_CRGPERIPH_PEREN1_ADDR(crgctrl));

    root = of_find_compatible_node(NULL, NULL,
        DEVICE_TREE_COMPATIBLE_NAME);
    //void *reg = ioremap(0xfff35018, 4);
    void *reg = ioremap(0xE896C120, 4);
    vdd33 = ioread32(reg);
    hisilog_info("%s: step in:0x%x\n", __func__, vdd33);
    //iowrite32(reg, vdd33 | ((unsigned int)1<< 30));
    vdd33 = ioread32(reg);
    hisilog_info("%s: step in:0x%x\n", __func__, vdd33);
    return;
    //writel(reg, );
    if(root)
    {
        gpio_hub_driver_info.hub_vbus_gpio = of_get_named_gpio(root,
        "hub_vbus_int_gpio", 0);

        if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
        {
            hisilog_info("%s: hub_vbus_int_gpio address %d\n", __func__,
                gpio_hub_driver_info.hub_vbus_gpio);
        }else
        {
            hisilog_err("%s: hub gpio address is not configure\n",
                 __func__);
            return;
        }

    }else
    {
        hisilog_err("%s: gpio hub compatible device node is not"
            " configure\n", __func__);
        return;
    }

    vbus_gpio_status= gpio_request(gpio_hub_driver_info.hub_vbus_gpio,
        "hub_vbus_int_gpio");
    if(vbus_gpio_status < 0)
    {
        hisilog_err("%s: hub vbus gpio request err\n", __func__);
        return;
    }

    set_vbus_gpio = gpio_direction_output(
        gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_HUB_VBUS_POWER);
    if(set_vbus_gpio)
    {
        gpio_free(gpio_hub_driver_info.hub_vbus_gpio);
        gpio_hub_driver_info.hub_vbus_gpio = -1;
        hisilog_err("%s: gpio hub otg switch gpio set err\n", __func__);
        return;
    }

    set_hub_vbus_gpio = gpio_direction_output(
        gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_HUB_VBUS_POWER);
    if(set_hub_vbus_gpio)
    {
        gpio_free(gpio_hub_driver_info.hub_vbus_gpio);
        gpio_hub_driver_info.hub_vbus_gpio = -1;
        hisilog_err("%s: gpio hub typec vbus gpio set err", __func__);
        return;
    }
    hisilog_info("%s gpio hub step out", __func__);
    return;
}
static int gpio_hub_probe(struct platform_device *pdev)
{
    int set_hub_vbus_gpio;
    int otg_switch_status;
    int typec_vbus_status;
    int set_typec_vbus_gpio;
    int hub_vbus_status;
    struct device_node * root;

    hisilog_err("%s: step in\n", __func__);
    if(!pdev)
    {
      return -EBUSY;
    }

    root = of_find_compatible_node(NULL, NULL,
        DEVICE_TREE_COMPATIBLE_NAME);
    if(root)
    {
        gpio_hub_driver_info.hub_vbus_gpio = of_get_named_gpio(root,
            "hub_vdd33_en_gpio", 0);
        gpio_hub_driver_info.typec_vbus_gpio = of_get_named_gpio(root,
            "typc_vbus_int_gpio,typec-gpios", 0);

        if(/*  gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio)
            && */gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
        {
            hisilog_info("%s: hub vbus gpio address:%d\n", __func__,
                gpio_hub_driver_info.hub_vbus_gpio);
            hisilog_info("%s: typec vbus gpio address:%d\n", __func__,
            gpio_hub_driver_info.typec_vbus_gpio);
        }else
        {
            hisilog_err("%s: hub gpio address is not configure\n",
                 __func__);
            return -ENXIO;
        }

    }else
    {
        hisilog_err("%s: gpio hub compatible device node is not"
            " configure\n", __func__);
        return -ENODEV;
    }

    hub_vbus_status = gpio_request(gpio_hub_driver_info.hub_vbus_gpio,
        "hub_vbus_int_gpio");
    if(hub_vbus_status < 0)
    {
        //gpio_free(gpio_hub_driver_info.hub_vbus_gpio);
        gpio_hub_driver_info.hub_vbus_gpio = -1;
	hisilog_err("%s: hub gpio request err:%d\n", __func__, hub_vbus_status);
        //return -EBUSY;
    }

    typec_vbus_status = gpio_request(gpio_hub_driver_info.typec_vbus_gpio,
        "typec_vbus_int_gpio");
    if(typec_vbus_status < 0)
    {
        gpio_free(gpio_hub_driver_info.typec_vbus_gpio);
	gpio_hub_driver_info.typec_vbus_gpio = -1;
        hisilog_err("%s: gpio hub typec vbus detect err:%d\n", __func__,
            typec_vbus_status);
        return -EBUSY;
    }

    set_hub_vbus_gpio = gpio_direction_output(
        gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_VBUS_POWER);
    if(set_hub_vbus_gpio)
    {
        //gpio_hub_gpio_free();
	hisilog_err("%s: gpio hub vbus gpio set err\n", __func__);
        //return -EBUSY;
    }

    set_typec_vbus_gpio = gpio_direction_output(
        gpio_hub_driver_info.typec_vbus_gpio, GPIO_TYPEC_VBUS_POWER);
    if(set_typec_vbus_gpio)
    {
        gpio_hub_gpio_free();
	hisilog_err("%s: gpio hub typec vbus gpio set err", __func__);
	return -EBUSY;
    }

    hisilog_info("%s gpio hub typec vbus gpio value:%d\n", __func__,
		    gpio_get_value(gpio_hub_driver_info.typec_vbus_gpio));
    hisilog_info("%s gpio hub vbus gpio value:%d\n", __func__,
    		    gpio_get_value(gpio_hub_driver_info.hub_vbus_gpio));
    hisilog_info("%s gpio hub step out", __func__);
    return 0;
}

static int  gpio_hub_remove(struct platform_device *pdev)
{
    if(gpio_is_valid(gpio_hub_driver_info.otg_switch_gpio))
    {
        gpio_free(gpio_hub_driver_info.otg_switch_gpio);
	gpio_hub_driver_info.otg_switch_gpio = -1;
    }

    if(gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
    {
        gpio_free(gpio_hub_driver_info.typec_vbus_gpio);
	gpio_hub_driver_info.typec_vbus_gpio = -1;
    }

    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_free(gpio_hub_driver_info.hub_vbus_gpio);
	    gpio_hub_driver_info.hub_vbus_gpio = -1;
    }
    return 0;
}

static const struct of_device_id id_table_for_gpio_hub[] = {
    {.compatible = DEVICE_TREE_COMPATIBLE_NAME},
    {},
};

static struct platform_driver gpio_hub_driver = {
    .probe = gpio_hub_probe,
    .remove = gpio_hub_remove,
    .driver = {
        .name = DEVICE_DRIVER_NAME,
	.of_match_table = of_match_ptr(id_table_for_gpio_hub),

     },
};

static int __init gpio_hub_init(void)
{
    int ret = platform_driver_register(&gpio_hub_driver);
    hisilog_info("%s:gpio hub init status:%d", __func__, ret);
    return ret;
}

static void __exit gpio_hub_exit(void)
{
    platform_driver_unregister(&gpio_hub_driver);
}

void gpio_hub_host_attach(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio,
	    GPIO_HUB_VBUS_NO_POWER);
	hisilog_info("%s: gpio hub hub vbus no power set sucess", __func__);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus no power set err", __func__);
    }


    if(gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.typec_vbus_gpio,
            GPIO_TYPEC_NO_POWER);
        hisilog_info("%s: gpio hub typc vbus gpio no power set sucess", __func__);
    }else
    {
        hisilog_err("%s: gpio hub typec vbus gpio no power set err", __func__);
    }
}

void gpio_hub_device_unattach(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio,
	    GPIO_HUB_VBUS_POWER);
	hisilog_info("%s: gpio hub hub vbus power set sucess", __func__);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus set err", __func__);
    }


    if(gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.typec_vbus_gpio,
            GPIO_TYPEC_VBUS_POWER);
	hisilog_info("%s: gpio hub typc vbus gpio power set sucess", __func__);
    }else
    {
       hisilog_err("%s: gpio hub typec vbus gpio power set err", __func__);
    }
}

void gpio_hub_device_attach(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_VBUS_POWER);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus set err", __func__);
    }

    if(gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.typec_vbus_gpio,
	    GPIO_TYPEC_VBUS_POWER);
    }else
    {
        hisilog_err("%s: gpio hub typec vbus gpio power set err", __func__);
    }
}
void gpio_hub_host_unattach(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_VBUS_POWER);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus set err", __func__);
    }

    if(gpio_is_valid(gpio_hub_driver_info.typec_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.typec_vbus_gpio,
	    GPIO_TYPEC_VBUS_POWER);
    }else
    {
        hisilog_err("%s: gpio hub typec vbus gpio power set err", __func__);
    }
}

void gpio_hub_power_off(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio,
	    GPIO_HUB_VBUS_NO_POWER);
	hisilog_info("%s: gpio hub hub vbus no power set sucess", __func__);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus no power set err", __func__);
    }

}
void gpio_hub_power_on(void)
{
    if(gpio_is_valid(gpio_hub_driver_info.hub_vbus_gpio))
    {
        gpio_set_value(gpio_hub_driver_info.hub_vbus_gpio, GPIO_HUB_VBUS_POWER);
    }else
    {
        hisilog_err("%s: gpio hub hub vbus set err", __func__);
    }
}
EXPORT_SYMBOL_GPL(gpio_hub_host_attach);
EXPORT_SYMBOL_GPL(gpio_hub_device_attach);
EXPORT_SYMBOL_GPL(gpio_hub_host_unattach);
EXPORT_SYMBOL_GPL(gpio_hub_device_unattach);

module_init(gpio_hub_init);
module_exit(gpio_hub_exit);

MODULE_AUTHOR("wangbinghui<wangbinghui@hisilicon.com>");
MODULE_DESCRIPTION("HUB GPIO FOR OTG ID driver");
MODULE_LICENSE("GPL v2");
