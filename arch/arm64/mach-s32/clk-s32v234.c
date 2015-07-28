/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/of_address.h>
#include <linux/clk.h>
#include <dt-bindings/clock/s32v234-clock.h>

#include "clk.h"

/* Source Reset Control: General Purpose Register 1 */
#define SRC_GPR1			(src_base + 0x100)
#define SRC_GPR1_ARMPLL_SRC_SEL_OFFSET	(27)
#define SRC_GPR1_ENETPLL_SRC_SEL_OFFSET	(28)
#define SRC_GPR1_DDRPLL_SRC_SEL_OFFSET	(29)
#define SRC_GPR1_PERIPHPLL_SRC_SEL_OFFSET	(30)
#define SRC_GPR1_VIDEOPLL_SRC_SEL_OFFSET	(31)
#define SRC_GPR1_XPLL_SRC_SEL_SIZE		(1)

/* Clock Generation Module 0 (CGM0) */
#define ARMPLL_PLLDIG(mc_cgm)			(mc_cgm)
#define ARMPLL_PLLDIG_DFS(mc_cgm)		((mc_cgm) + 0x40)
/* TODO: Update these values when SOC is available */
#define ARMPLL_PLLDIG_PLLDV_MFD		(50)
#define ARMPLL_PLLDIG_PLLDV_MFN		(0)
#define ARMPLL_PLLDIG_PLLDV_RFDPHI0	(1)
#define ARMPLL_PLLDIG_PLLDV_RFDPHI1	(1)
#define ARMPLL_PLLDIG_DFS0_MFN		(194)
#define ARMPLL_PLLDIG_DFS1_MFN		(170)
#define ARMPLL_PLLDIG_DFS2_MFN		(170)
#define PERIPHPLL_PLLDIG(mc_cgm)	((mc_cgm) + 0x80)
#define PERIPHPLL_PLLDIG_PLLDV_MFD	(30)
#define PERIPHPLL_PLLDIG_PLLDV_MFN	(0)
#define PERIPHPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define PERIPHPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)
#define ENETPLL_PLLDIG(mc_cgm)		((mc_cgm) + 0x100)
#define ENETPLL_PLLDIG_DFS(mc_cgm)	((mc_cgm) + 0x100 + 0x40)
#define ENETPLL_PLLDIG_PLLDV_MFD	(50)
#define ENETPLL_PLLDIG_PLLDV_MFN	(0)
#define ENETPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define ENETPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)
#define ENETPLL_PLLDIG_DFS0_MFN		(219)
#define ENETPLL_PLLDIG_DFS1_MFN		(219)
#define ENETPLL_PLLDIG_DFS2_MFN		(32)
#define ENETPLL_PLLDIG_DFS3_MFN		(0)
#define DDRPLL_PLLDIG(mc_cgm)		((mc_cgm) + 0x180)
#define DDRPLL_PLLDIG_DFS(mc_cgm)	((mc_cgm) + 0x180 + 0x40)
#define DDRPLL_PLLDIG_PLLDV_MFD		(53)
#define DDRPLL_PLLDIG_PLLDV_MFN		(6144)
#define DDRPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define DDRPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)
#define DDRPLL_PLLDIG_DFS0_MFN		(33)
#define DDRPLL_PLLDIG_DFS1_MFN		(33)
#define DDRPLL_PLLDIG_DFS2_MFN		(11)
#define VIDEOPLL_PLLDIG(mc_cgm)		((mc_cgm)+ 0x200)
#define VIDEOPLL_PLLDIG_PLLDV_MFD	(30)
#define VIDEOPLL_PLLDIG_PLLDV_MFN	(0)
#define VIDEOPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define VIDEOPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)

/* MC_CGM_SC_SS */
#define CGM_SC_SS(mc_cgm)		(((mc_cgm) + 0x7E4))

/* MC_CGM_SC_DCn */
#define CGM_SC_DCn(mc_cgm,dc)		(((mc_cgm) + 0x7E8) + ((dc) * 0x4))

#define MC_CGM_SC_DCn_PREDIV_OFFSET	(16)
#define MC_CGM_SC_DCn_PREDIV_SIZE	(3)
#define MC_CGM_SC_DCn_DE		(1 << 31)
#define MC_CGM_SC_SEL_OFFSET		(24)
#define MC_CGM_SC_SEL_SIZE		(4)

/* MC_CGM_ACn_DCm */
#define CGM_ACn_DCm(mc_cgm,ac,dc)	( ((mc_cgm) + 0x808) + ((ac) * 0x20)\
					+ ((dc) * 0x4) )

#define MC_CGM_ACn_DCm_PREDIV(val)		(MC_CGM_ACn_DCm_PREDIV_MASK & ((val) << MC_CGM_ACn_DCm_PREDIV_OFFSET))
#define MC_CGM_ACn_DCm_PREDIV_MASK	(0x001F0000)
#define MC_CGM_ACn_DCm_PREDIV_OFFSET	(16)
#define MC_CGM_ACn_DCm_PREDIV_SIZE	(5)
#define MC_CGM_ACn_DCm_DE		(1 << 31)

 /*
  * MC_CGM_ACn_SC/MC_CGM_ACn_SS
  */
 #define CGM_ACn_SC(mc_cgm,ac)		(((mc_cgm) + 0x800) + ((ac) * 0x20))
 #define CGM_ACn_SS(mc_cgm,ac)		(((mc_cgm) + 0x804) + ((ac) * 0x24))
 #define MC_CGM_ACn_SEL_MASK		(0x07000000)
 #define MC_CGM_ACn_SEL_SET(source)	(MC_CGM_ACn_SEL_MASK & (((source) & 0x7) \
					<< MC_CGM_ACn_SEL_OFFSET))
 #define MC_CGM_ACn_SEL_OFFSET		(24)
 #define MC_CGM_ACn_SEL_SIZE		(4)

/* TODO: Implement a specific driver only for MC_ME register when
 * current clock framework is stable.
 */

/* MC_ME registers definitions */
/* MC_ME_GS */
 #define MC_ME_GS(mc_me)		((mc_me) + 0x00000000)

/* MC_ME_MCTL */
#define MC_ME_MCTL(mc_me)		((mc_me) + 0x00000004)
#define MC_ME_MCTL_RESET		(0x0 << 28)
#define MC_ME_MCTL_TEST			(0x1 << 28)
#define MC_ME_MCTL_DRUN			(0x3 << 28)
#define MC_ME_MCTL_RUN0			(0x4 << 28)
#define MC_ME_MCTL_RUN1			(0x5 << 28)
#define MC_ME_MCTL_RUN2			(0x6 << 28)
#define MC_ME_MCTL_RUN3			(0x7 << 28)

#define MC_ME_GS_S_MTRANS		(1 << 27)

#define MC_ME_MCTL_KEY			(0x00005AF0)
#define MC_ME_MCTL_INVERTEDKEY		(0x0000A50F)

/*
 * MC_ME_RESET_MC/MC_ME_TEST_MC
 * MC_ME_DRUN_MC
 * MC_ME_RUNn_MC
 */
#define MC_ME_RESET_MC(mc_me)		((mc_me) + 0x00000020)
#define MC_ME_TEST_MC(mc_me)		((mc_me) + 0x00000024)
#define MC_ME_DRUN_MC(mc_me)		((mc_me) + 0x0000002C)
#define MC_ME_RUNn_MC(mc_me,n)		((mc_me) + 0x00000030 + 0x4 * (n))
#define MC_ME_MODE_MC_SYSCLK_OFFSET	(0)
#define MC_ME_MODE_MC_SYSCLK_SIZE	(0x3)
#define MC_ME_MODE_MC_SYSCLK(val)	(MC_ME_MODE_MC_SYSCLK_MASK& (val))
#define MC_ME_MODE_MC_SYSCLK_MASK	(0x0000000F)
#define MC_ME_MODE_MC_FIRCON		(1 << 4)
#define MC_ME_MODE_MC_XOSCON		(1 << 5)
#define MC_ME_MODE_MC_ARMPLL		(1 << 6)
#define MC_ME_MODE_MC_PERIPHPLL		(1 << 7)
#define MC_ME_MODE_MC_ENETPLL		(1 << 8)
#define MC_ME_MODE_MC_DDRPLL		(1 << 9)
#define MC_ME_MODE_MC_VIDEOPLL		(1 << 10)
#define MC_ME_MODE_MC_MVRON		(1 << 20)

/* MC_ME_DRUN_SEC_CC_I */
#define MC_ME_DRUN_SEC_CC_I(mc_me)		((mc_me) + 0x260)
/* MC_ME_RUNn_SEC_CC_I */
#define MC_ME_RUNn_SEC_CC_I(mc_me,n)		((mc_me) + 0x270 + (n) * 0x10)
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_OFFSET	(4)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_OFFSET	(8)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_OFFSET	(12)
/* Consider only the defined clocks */
#define MC_ME_MODE_SEC_CC_I_SYSCLK1_SIZE	(0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK2_SIZE	(0x3)
#define MC_ME_MODE_SEC_CC_I_SYSCLK3_SIZE	(0x3)

/* MC_ME_RUN_PCn */
#define MC_ME_RUN_PCn(mc_me,n)		(mc_me + 0x00000080 + 0x4 * (n))

#define MC_ME_RUN_PCn_MIN_IDX		(0)
#define MC_ME_RUN_PCn_MAX_IDX		(7)
#define MC_ME_RUN_PCn_RESET		(1 << 0)
#define MC_ME_RUN_PCn_TEST		(1 << 1)
#define MC_ME_RUN_PCn_DRUN		(1 << 3)
#define MC_ME_RUN_PCn_RUN0		(1 << 4)
#define MC_ME_RUN_PCn_RUN1		(1 << 5)
#define MC_ME_RUN_PCn_RUN2		(1 << 6)
#define MC_ME_RUN_PCn_RUN3		(1 << 7)

static void entry_to_target_mode( void __iomem * mc_me, u32 mode )
{
	writel_relaxed( mode | MC_ME_MCTL_KEY, MC_ME_MCTL(mc_me) );
	writel_relaxed( mode | MC_ME_MCTL_INVERTEDKEY, MC_ME_MCTL(mc_me) );
	while( (readl_relaxed(MC_ME_GS(mc_me)) & MC_ME_GS_S_MTRANS) != 0x00000000 );
}

static void enable_cpumodes_onperipheralconfig(void __iomem * mc_me, u32 modes, u32 run_pc_idx)
{
	BUG_ON( run_pc_idx < MC_ME_RUN_PCn_MIN_IDX ||
		run_pc_idx >= MC_ME_RUN_PCn_MAX_IDX );

	writel_relaxed( modes , MC_ME_RUN_PCn(mc_me,run_pc_idx) );
}

static void enable_clocks_sources( u32 flags, u32 clks, void __iomem * xrun_mc_addr )
{
	writel_relaxed( readl_relaxed(xrun_mc_addr) | flags | clks,
			xrun_mc_addr );
}

static void disable_clocks_sources( u32 clks, void __iomem * xrun_mc_addr )
{
	writel_relaxed( readl_relaxed(xrun_mc_addr) & ~clks,
			xrun_mc_addr );
}

static void enable_sysclock( u32 clk, void __iomem * xrun_mc_addr )
{
	writel_relaxed( readl_relaxed(xrun_mc_addr) & clk,
			xrun_mc_addr );
}

static void __iomem * mc_cgm0_base;
static void __iomem * mc_cgm1_base;
static void __iomem * mc_cgm2_base;
static void __iomem * mc_cgm3_base;
static void __iomem * mc_me_base;
static void __iomem * src_base;

/* sources for multiplexer clocks, this is used multiple times */
static const char const * osc_sels[]	= { "firc", "fxosc", };

static const char const * cores_sels[]	= { "firc", "fxosc", "armpll_phi0", };

static const char const * sys_sels[]	= { "firc", "fxosc", "armpll_dfs0", };

static const char const * perifray_sels[] = { "firc", "fxosc", "dummy", "periphpll_phi0_div5", };

static const char const * lin_sels[]	= { "firc", "fxosc", "dummy", "periphpll_phi0_div3", "dummy", "dummy", "dummy", "dummy", "sys6",};

static const char const * sdhc_sels[]	= { "firc", "fxosc", "dummy", "dummy", "enetpll_dfs3",};

static struct clk *clk[S32V234_CLK_END];
static struct clk_onecell_data clk_data;

static void __init s32v234_clocks_init(struct device_node * mc_cgm0_node)
{
	struct device_node *np;

	clk[S32V234_CLK_DUMMY] = s32_clk_fixed("dummy", 0);
	clk[S32V234_CLK_FXOSC] = s32_obtain_fixed_clock("fxosc", 0);
	clk[S32V234_CLK_FIRC] = s32_obtain_fixed_clock("firc", 0);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_me");
	mc_me_base = of_iomap(np, 0);
	BUG_ON(!mc_me_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-src");
	src_base = of_iomap(np, 0);
	BUG_ON(!src_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm1");
	mc_cgm1_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm1_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm2");
	mc_cgm2_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm2_base);

	np = of_find_compatible_node(NULL, NULL, "fsl,s32v234-mc_cgm3");
	mc_cgm3_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm3_base);

	np = mc_cgm0_node;
	mc_cgm0_base = of_iomap(np, 0);
	BUG_ON(!mc_cgm0_base);

	enable_cpumodes_onperipheralconfig( mc_me_base, MC_ME_RUN_PCn_DRUN |
					    MC_ME_RUN_PCn_RUN0 |
					    MC_ME_RUN_PCn_RUN1 |
					    MC_ME_RUN_PCn_RUN2 |
					    MC_ME_RUN_PCn_RUN3,
					    0 );

	/* turn on XOSC and FIRC */
	enable_clocks_sources( MC_ME_MODE_MC_MVRON, MC_ME_MODE_MC_XOSCON |
			       MC_ME_MODE_MC_FIRCON, MC_ME_RUNn_MC(mc_me_base, 0) );

	/* transition the core to RUN0 mode */
	entry_to_target_mode( mc_me_base, MC_ME_MCTL_RUN0 );

	clk[S32V234_CLK_ARMPLL_SRC_SEL] = s32_clk_mux("armpll_sel",
		SRC_GPR1, SRC_GPR1_ARMPLL_SRC_SEL_OFFSET, SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_ENETPLL_SRC_SEL] = s32_clk_mux("enetpll_sel",
		SRC_GPR1, SRC_GPR1_ENETPLL_SRC_SEL_OFFSET, SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_DDRPLL_SRC_SEL] = s32_clk_mux("ddrpll_sel",
		SRC_GPR1, SRC_GPR1_DDRPLL_SRC_SEL_OFFSET, SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_PERIPHPLL_SRC_SEL] = s32_clk_mux("periphpll_sel",
		SRC_GPR1, SRC_GPR1_PERIPHPLL_SRC_SEL_OFFSET, SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	clk[S32V234_CLK_VIDEOPLL_SRC_SEL] = s32_clk_mux("videopll_sel",
		SRC_GPR1, SRC_GPR1_VIDEOPLL_SRC_SEL_OFFSET, SRC_GPR1_XPLL_SRC_SEL_SIZE,
		osc_sels, ARRAY_SIZE(osc_sels));

	/* ARM_PLL */
	clk[S32V234_CLK_ARMPLL_VCO] = s32_clk_plldig(S32_PLLDIG_ARM,
		"armpll_vco", "armpll_sel", ARMPLL_PLLDIG(mc_cgm0_base),
		ARMPLL_PLLDIG_PLLDV_MFD, ARMPLL_PLLDIG_PLLDV_MFN,
		ARMPLL_PLLDIG_PLLDV_RFDPHI0, ARMPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ARMPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi0", "armpll_vco", ARMPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ARMPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_ARM,
		"armpll_phi1", "armpll_vco", ARMPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ARMPLL_DFS0] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs0", "armpll_phi1", ARMPLL_PLLDIG_DFS(mc_cgm0_base), 0, ARMPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ARMPLL_DFS1] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs1", "armpll_phi1", ARMPLL_PLLDIG_DFS(mc_cgm0_base), 1, ARMPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ARMPLL_DFS2] = s32_clk_dfs(S32_PLLDIG_ARM,
		 "armpll_dfs2", "armpll_phi1", ARMPLL_PLLDIG_DFS(mc_cgm0_base), 2, ARMPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_CORES_SEL] = s32_clk_mux("cores_sels",
		MC_ME_RUNn_SEC_CC_I(mc_me_base,0),
		MC_ME_MODE_SEC_CC_I_SYSCLK1_OFFSET,
		MC_ME_MODE_SEC_CC_I_SYSCLK1_SIZE,
		cores_sels, ARRAY_SIZE(cores_sels));

	clk[S32V234_CLK_CORE] = s32_clk_divider("core", "cores_sels",
		CGM_SC_DCn(mc_cgm1_base,0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_CORE2] = s32_clk_divider("core2", "cores_sels",
		CGM_SC_DCn(mc_cgm1_base,1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_COREDBG] = s32_clk_divider("coredbgcoredbg", "cores_sels",
		CGM_SC_DCn(mc_cgm1_base,2), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS_SEL] = s32_clk_mux("sys_sel",
		MC_ME_RUNn_MC(mc_me_base, 0),
		MC_ME_MODE_MC_SYSCLK_OFFSET,
		MC_ME_MODE_MC_SYSCLK_SIZE,
		sys_sels, ARRAY_SIZE(sys_sels));

	clk[S32V234_CLK_SYS3] = s32_clk_divider("sys3", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 0), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS6] = s32_clk_divider("sys6", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	clk[S32V234_CLK_SYS6_DIV2] = s32_clk_divider("sys6_div2", "sys_sel",
		CGM_SC_DCn(mc_cgm0_base, 2), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_SC_DCn_PREDIV_SIZE);

	/* enable ARMPLL */
	enable_clocks_sources( 0, MC_ME_MODE_MC_ARMPLL, MC_ME_RUNn_MC(mc_me_base, 0) );

	/* PERIPH_PLL */
	clk[S32V234_CLK_PERIPHPLL_VCO] = s32_clk_plldig(S32_PLLDIG_PERIPH,
		"periphpll_vco", "periphpll_sel", PERIPHPLL_PLLDIG(mc_cgm0_base),
		PERIPHPLL_PLLDIG_PLLDV_MFD, PERIPHPLL_PLLDIG_PLLDV_MFN,
		PERIPHPLL_PLLDIG_PLLDV_RFDPHI0, PERIPHPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_PERIPHPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi0", "periphpll_vco", PERIPHPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_PERIPHPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_PERIPH,
		"periphpll_phi1", "periphpll_vco", PERIPHPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV3] = s32_clk_fixed_factor(
		"periphpll_phi0_div3", "periphpll_phi0", 1, 3);

	clk[S32V234_CLK_PERIPHPLL_PHI0_DIV5] = s32_clk_fixed_factor(
		"periphpll_phi0_div5", "periphpll_phi0", 1, 5);

	/* Peri Clock */
	clk[S32V234_CLK_PERI_FRAY_PLL_SEL] = s32_clk_mux("perifray_sel",
		CGM_ACn_SC(mc_cgm0_base,5),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		perifray_sels, ARRAY_SIZE(perifray_sels));

	clk[S32V234_CLK_PERI] = s32_clk_divider("peri", "perifray_sel",
		CGM_ACn_DCm(mc_cgm0_base,5,0), MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_PERI_FRAY_PLL] = s32_clk_divider("perifray", "perifray_sel",
		CGM_ACn_DCm(mc_cgm0_base,5,1), MC_CGM_SC_DCn_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	/* Lin Clock */
	clk[S32V234_CLK_LIN_SEL] = s32_clk_mux("lin_sel",
		CGM_ACn_SC(mc_cgm0_base,5),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		lin_sels, ARRAY_SIZE(lin_sels));

	clk[S32V234_CLK_LIN] = s32_clk_divider("lin", "lin_sel",
		CGM_ACn_DCm(mc_cgm0_base,3,0), MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	clk[S32V234_CLK_LIN_IPG] = s32_clk_fixed_factor("lin_ipg",
		"lin", 1, 2);

	/* enable PERIPHPLL */
	enable_clocks_sources( 0, MC_ME_MODE_MC_PERIPHPLL, MC_ME_RUNn_MC(mc_me_base, 0) );

	/* ENET_PLL */
	clk[S32V234_CLK_ENETPLL_VCO] = s32_clk_plldig(S32_PLLDIG_ENET,
		"enetpll_vco", "enetpll_sel", ENETPLL_PLLDIG(mc_cgm0_base),
		ENETPLL_PLLDIG_PLLDV_MFD, ENETPLL_PLLDIG_PLLDV_MFN,
		ENETPLL_PLLDIG_PLLDV_RFDPHI0, ENETPLL_PLLDIG_PLLDV_RFDPHI1);

	clk[S32V234_CLK_ENETPLL_PHI0] = s32_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi0", "enetphpll_vco", ENETPLL_PLLDIG(mc_cgm0_base), 0);

	clk[S32V234_CLK_ENETPLL_PHI1] = s32_clk_plldig_phi(S32_PLLDIG_ENET,
		"enetpll_phi1", "enetpll_vco", ENETPLL_PLLDIG(mc_cgm0_base), 1);

	clk[S32V234_CLK_ENETPLL_DFS0] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs0", "enetpll_phi1", ENETPLL_PLLDIG_DFS(mc_cgm0_base), 0, ENETPLL_PLLDIG_DFS0_MFN);

	clk[S32V234_CLK_ENETPLL_DFS1] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs1", "enetpll_phi1", ENETPLL_PLLDIG_DFS(mc_cgm0_base), 1, ENETPLL_PLLDIG_DFS1_MFN);

	clk[S32V234_CLK_ENETPLL_DFS2] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs2", "enetpll_phi1", ENETPLL_PLLDIG_DFS(mc_cgm0_base), 2, ENETPLL_PLLDIG_DFS2_MFN);

	clk[S32V234_CLK_ENETPLL_DFS3] = s32_clk_dfs(S32_PLLDIG_ENET,
		 "enetpll_dfs3", "enetpll_phi1", ENETPLL_PLLDIG_DFS(mc_cgm0_base), 3, ENETPLL_PLLDIG_DFS3_MFN);

	/* SDHC Clock */
	clk[S32V234_CLK_SDHC_SEL] = s32_clk_mux("sdhc_sel",
		CGM_ACn_SC(mc_cgm0_base, 15),
		MC_CGM_ACn_SEL_OFFSET,
		MC_CGM_ACn_SEL_SIZE,
		sdhc_sels, ARRAY_SIZE(sdhc_sels));

	clk[S32V234_CLK_SDHC] = s32_clk_divider("sdhc", "sdhc_sel",
		CGM_ACn_DCm(mc_cgm0_base, 15, 0), MC_CGM_ACn_DCm_PREDIV_OFFSET,
		MC_CGM_ACn_DCm_PREDIV_SIZE);

	/* enable ENETPLL */
	enable_clocks_sources( 0, MC_ME_MODE_MC_ENETPLL, MC_ME_RUNn_MC(mc_me_base, 0) );

	/* set the system clock */
	enable_sysclock( MC_ME_MODE_MC_SYSCLK(0x2), MC_ME_RUNn_MC(mc_me_base, 0) );

	/* transition the core to RUN0 mode */
	entry_to_target_mode( mc_me_base, MC_ME_MCTL_RUN0 );

	/* Add the clocks to provider list */
	clk_data.clks = clk;
	clk_data.clk_num = ARRAY_SIZE(clk);
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data);
}

CLK_OF_DECLARE(S32V234, "fsl,s32v234-mc_cgm0", s32v234_clocks_init);
