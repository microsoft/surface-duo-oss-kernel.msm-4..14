/*
 * For Hisilicon Hi6220 SoC, the reset of some hosts (e.g. UART) should be disabled
 * before using them, this driver will handle the host chip reset disable.
 *
 * Copyright (C) 2015 Hisilicon Ltd.
 * Author: Bintian Wang <bintian.wang@huawei.com>
 *
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define PMUSSI_REG_EX(pmu_base, reg_addr) (((reg_addr) << 2) + (char *)pmu_base)

static int __init hi6220_sysconf(void)
{
        static void __iomem *base1 = NULL;
        struct device_node *node1;
	unsigned char ret;

        node1 = of_find_compatible_node(NULL, NULL, "hisilicon,hi655x-pmic");
        if (!node1)
                return -ENOENT;

        base1 = of_iomap(node1, 0);
        if (base1 == NULL) {
                printk(KERN_ERR "hi6220: pmic reg iomap failed!\n");
                return -ENOMEM;
        }

	/*enable clk for BT/WIFI*/
	ret = *(volatile unsigned char*)PMUSSI_REG_EX(base1, 0x1c);
	ret |= 0x40;
	*(volatile unsigned char*)PMUSSI_REG_EX(base1, 0x1c) = ret;

        iounmap(base1);
        return 0;
}
postcore_initcall(hi6220_sysconf);
