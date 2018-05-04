/*
 * Copyright 2004-2007, 2010-2015 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#include <linux/types.h>

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define MXC_CPU_MX1		1
#define MXC_CPU_MX21		21
#define MXC_CPU_MX25		25
#define MXC_CPU_MX27		27
#define MXC_CPU_MX31		31
#define MXC_CPU_MX35		35
#define MXC_CPU_MX51		51
#define MXC_CPU_MX53		53
#define MXC_CPU_IMX6SL		0x60
#define MXC_CPU_IMX6DL		0x61
#define MXC_CPU_IMX6SX		0x62
#define MXC_CPU_IMX6Q		0x63
#define MXC_CPU_IMX6UL		0x64
#define MXC_CPU_IMX6ULL		0x65
#define MXC_CPU_IMX7D		0x72
#define MXC_CPU_SAC58R		58


#define IMX_CHIP_REVISION_1_0		0x10
#define IMX_CHIP_REVISION_1_1		0x11
#define IMX_CHIP_REVISION_1_2		0x12
#define IMX_CHIP_REVISION_1_3		0x13
#define IMX_CHIP_REVISION_1_4		0x14
#define IMX_CHIP_REVISION_1_5		0x15
#define IMX_CHIP_REVISION_2_0		0x20
#define IMX_CHIP_REVISION_2_1		0x21
#define IMX_CHIP_REVISION_2_2		0x22
#define IMX_CHIP_REVISION_2_3		0x23
#define IMX_CHIP_REVISION_3_0		0x30
#define IMX_CHIP_REVISION_3_1		0x31
#define IMX_CHIP_REVISION_3_2		0x32
#define IMX_CHIP_REVISION_3_3		0x33
#define IMX_CHIP_REVISION_UNKNOWN	0xff

#define IMX_DDR_TYPE_LPDDR2		1

#ifndef __ASSEMBLY__
extern unsigned int __mxc_cpu_type;

#ifdef CONFIG_SOC_IMX53
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_MX53
# endif
# define cpu_is_mx53()		(mxc_cpu_type == MXC_CPU_MX53)
#else
# define cpu_is_mx53()		(0)
#endif

#ifdef CONFIG_SOC_SAC58R
# ifdef mxc_cpu_type
#  undef mxc_cpu_type
#  define mxc_cpu_type __mxc_cpu_type
# else
#  define mxc_cpu_type MXC_CPU_SAC58R
# endif
# define cpu_is_sac58r()		(mxc_cpu_type == MXC_CPU_SAC58R)
#else
# define cpu_is_sac58r()		(0)

#endif

#ifndef __ASSEMBLY__
#ifdef CONFIG_SOC_IMX6SL
static inline bool cpu_is_imx6sl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SL;
}
#else
static inline bool cpu_is_imx6sl(void)
{
	return false;
}
#endif

static inline bool cpu_is_imx6dl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6DL;
}

static inline bool cpu_is_imx6sx(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SX;
}

static inline bool cpu_is_imx6ul(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6UL;
}

static inline bool cpu_is_imx6ull(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULL;
}

static inline bool cpu_is_imx6q(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6Q;
}

static inline bool cpu_is_imx7d(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7D;
}

struct cpu_op {
	u32 cpu_rate;
};

int tzic_enable_wake(void);

extern struct cpu_op *(*get_cpu_op)(int *op);
#endif

#define imx_readl	readl_relaxed
#define imx_readw	readw_relaxed
#define imx_writel	writel_relaxed
#define imx_writew	writew_relaxed

#endif /*  __ASM_ARCH_MXC_H__ */
