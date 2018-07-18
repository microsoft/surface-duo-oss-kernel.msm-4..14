/*
 * Copyright 2011 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/reset-controller.h>
#include <linux/smp.h>
#include <asm/smp_plat.h>
#include "common.h"

#define SRC_SCR				0x000
#define SRC_GPR1			0x020

#define BP_SRC_SCR_WARM_RESET_ENABLE	0
#define BP_SRC_SCR_SW_GPU_RST		1
#define BP_SRC_SCR_SW_VPU_RST		2
#define BP_SRC_SCR_SW_IPU1_RST		3
#define BP_SRC_SCR_SW_OPEN_VG_RST	4
#define BP_SRC_SCR_SW_IPU2_RST		12
#define BP_SRC_SCR_CORE1_RST		14
#define BP_SRC_SCR_CORE1_ENABLE		22

/* SAC58R SRC registers definition */
#define SAC58R_SRC_GPR2				0x028
#define SAC58R_SRC_SCR_CORE1_ENABLE	31
#define SRC_SCR_SW_RESET			12


enum {
	IMX_SRC_DEVTYPE_IMX6,
	IMX_SRC_DEVTYPE_SAC58R,
};

static int imx_src_devtype;

static void __iomem *src_base;
static DEFINE_SPINLOCK(scr_lock);

static const int sw_reset_bits[5] = {
	BP_SRC_SCR_SW_GPU_RST,
	BP_SRC_SCR_SW_VPU_RST,
	BP_SRC_SCR_SW_IPU1_RST,
	BP_SRC_SCR_SW_OPEN_VG_RST,
	BP_SRC_SCR_SW_IPU2_RST
};

static int imx_src_reset_module(struct reset_controller_dev *rcdev,
		unsigned long sw_reset_idx)
{
	unsigned long timeout;
	unsigned long flags;
	int bit;
	u32 val;

	if (!src_base)
		return -ENODEV;

	if (sw_reset_idx >= ARRAY_SIZE(sw_reset_bits))
		return -EINVAL;

	bit = 1 << sw_reset_bits[sw_reset_idx];

	spin_lock_irqsave(&scr_lock, flags);
	val = readl_relaxed(src_base + SRC_SCR);
	val |= bit;
	writel_relaxed(val, src_base + SRC_SCR);
	spin_unlock_irqrestore(&scr_lock, flags);

	timeout = jiffies + msecs_to_jiffies(1000);
	while (readl(src_base + SRC_SCR) & bit) {
		if (time_after(jiffies, timeout))
			return -ETIME;
		cpu_relax();
	}

	return 0;
}

static const struct reset_control_ops imx_src_ops = {
	.reset = imx_src_reset_module,
};

static struct reset_controller_dev imx_reset_controller = {
	.ops = &imx_src_ops,
	.nr_resets = ARRAY_SIZE(sw_reset_bits),
};

void imx_enable_cpu(int cpu, bool enable)
{
	u32 mask, val;

	cpu = cpu_logical_map(cpu);

	if (imx_src_devtype == IMX_SRC_DEVTYPE_SAC58R) {
		mask = 1 << SAC58R_SRC_SCR_CORE1_ENABLE;
		spin_lock(&scr_lock);
		val = readl_relaxed(src_base + SRC_SCR);
		val = enable ? val | mask : val & ~mask;
		writel_relaxed(val, src_base + SRC_SCR);
		spin_unlock(&scr_lock);
	}
	else {
		mask = 1 << (BP_SRC_SCR_CORE1_ENABLE + cpu - 1);
		spin_lock(&scr_lock);
		val = readl_relaxed(src_base + SRC_SCR);
		val = enable ? val | mask : val & ~mask;
		val |= 1 << (BP_SRC_SCR_CORE1_RST + cpu - 1);
		writel_relaxed(val, src_base + SRC_SCR);
		spin_unlock(&scr_lock);
	}
}

void imx_set_cpu_jump(int cpu, void *jump_addr)
{
	int src_base_address = SRC_GPR1;

	if (imx_src_devtype == IMX_SRC_DEVTYPE_SAC58R) {
		src_base_address = SAC58R_SRC_GPR2;
	}

	printk(KERN_DEBUG "%s: cpu(%d), jump_addr(%x), base @ = %x\n",
		__func__, cpu, (int)virt_to_phys(jump_addr),
		(int)(src_base + src_base_address + cpu * 8));


	cpu = cpu_logical_map(cpu);
	writel_relaxed(__pa_symbol(jump_addr),
		       src_base + SRC_GPR1 + cpu * 8);
}

u32 imx_get_cpu_arg(int cpu)
{
	int src_base_address = SRC_GPR1;

	if (imx_src_devtype == IMX_SRC_DEVTYPE_SAC58R) {
		src_base_address = SAC58R_SRC_GPR2;
	}

	cpu = cpu_logical_map(cpu);
	return readl_relaxed(src_base + src_base_address + cpu * 8 + 4);
}

void imx_set_cpu_arg(int cpu, u32 arg)
{
	int src_base_address = SRC_GPR1;

	if (imx_src_devtype == IMX_SRC_DEVTYPE_SAC58R) {
		src_base_address = SAC58R_SRC_GPR2;
	}

	pr_debug("%s: cpu(%d), arg(%x), base @ = %x\n",
		__func__, cpu, arg, (int)(src_base + src_base_address + cpu * 8 + 4));

	cpu = cpu_logical_map(cpu);
	writel_relaxed(arg, src_base + src_base_address + cpu * 8 + 4);
}

void __init imx_src_init(void)
{
	struct device_node *np;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx51-src");
	if (np) {
		imx_src_devtype = IMX_SRC_DEVTYPE_IMX6;
	}
	else {
		np = of_find_compatible_node(NULL, NULL, "fsl,sac58r-src");
		if (np) {
			imx_src_devtype = IMX_SRC_DEVTYPE_SAC58R;
		}
		else {
			return;
		}
	}

	src_base = of_iomap(np, 0);
	WARN_ON(!src_base);

	if (IMX_SRC_DEVTYPE_IMX6 == imx_src_devtype) {
		imx_reset_controller.of_node = np;
		if (IS_ENABLED(CONFIG_RESET_CONTROLLER))
			reset_controller_register(&imx_reset_controller);

		/*
		 * force warm reset sources to generate cold reset
		 * for a more reliable restart
		 */
		spin_lock(&scr_lock);
		val = readl_relaxed(src_base + SRC_SCR);
		val &= ~(1 << BP_SRC_SCR_WARM_RESET_ENABLE);
		writel_relaxed(val, src_base + SRC_SCR);
		spin_unlock(&scr_lock);
	}
}
