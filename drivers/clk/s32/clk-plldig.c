/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
  *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include "clk.h"

/* PLLDIG PLL Divider Register (PLLDIG_PLLDV) */
#define PLLDIG_PLLDV(base)		((base) + 0x00000028)
#define PLLDIG_PLLDV_MFD_SET(val)	(PLLDIG_PLLDV_MFD_MASK & (val))
#define PLLDIG_PLLDV_MFD_MASK		(0x000000FF)

#define PLLDIG_PLLDV_RFDPHI_SET(val)	(PLLDIG_PLLDV_RFDPHI_MASK & \
					(((val) & \
					PLLDIG_PLLDV_RFDPHI_MAXVALUE) \
					<< PLLDIG_PLLDV_RFDPHI_OFFSET))
#define PLLDIG_PLLDV_RFDPHI_MASK	(0x003F0000)
#define PLLDIG_PLLDV_RFDPHI_MAXVALUE	(0x3F)
#define PLLDIG_PLLDV_RFDPHI_MINVALUE	(0x0)

#define PLLDIG_PLLDV_RFDPHI_OFFSET	(16)

#define PLLDIG_PLLDV_RFDPHI1_SET(val)	(PLLDIG_PLLDV_RFDPHI1_MASK & \
					(((val) & \
					PLLDIG_PLLDV_RFDPHI1_MAXVALUE) \
					<< PLLDIG_PLLDV_RFDPHI1_OFFSET))
#define PLLDIG_PLLDV_RFDPHI1_MASK	(0x7E000000)
#define PLLDIG_PLLDV_RFDPHI1_MAXVALUE	(0x3F)
#define PLLDIG_PLLDV_RFDPHI1_MINVALUE	(0x0)
#define PLLDIG_PLLDV_RFDPHI1_OFFSET	(25)

#define PLLDIG_PLLDV_PREDIV_SET(val)	(PLLDIG_PLLDV_PREDIV_MASK & \
					(((val) & \
					PLLDIG_PLLDV_PREDIV_MAXVALUE) \
					 << PLLDIG_PLLDV_PREDIV_OFFSET))
#define PLLDIG_PLLDV_PREDIV_MASK	(0x00007000)
#define PLLDIG_PLLDV_PREDIV_MAXVALUE	(0x7)
#define PLLDIG_PLLDV_PREDIV_OFFSET	(12)

/* PLLDIG PLL Fractional  Divide Register (PLLDIG_PLLFD) */
#define PLLDIG_PLLFD(base)		((base) + 0x00000030)
#define PLLDIG_PLLFD_MFN_SET(val)	(PLLDIG_PLLFD_MFN_MASK & (val))
#define PLLDIG_PLLFD_MFN_MASK		(0x00007FFF)

/* PLL Calibration Register 1 (PLLDIG_PLLCAL1) */
#define PLLDIG_PLLCAL1(base)		((base) + 0x00000038)
#define PLLDIG_PLLCAL1_NDAC1_SET(val)	(PLLDIG_PLLCAL1_NDAC1_MASK & \
					((val) << PLLDIG_PLLCAL1_NDAC1_OFFSET))
#define PLLDIG_PLLCAL1_NDAC1_OFFSET	(24)
#define PLLDIG_PLLCAL1_NDAC1_MASK	(0x7F000000)

/* Naming convention for PLL:
 * ARMPLL - PLL0
 * PERIPHPLL - PLL1
 * ENETPLL - PLL2
 * DDRPLL - PLL3
 * VIDEOPLL - PLL4
 */
/* The min,max values for PLL VCO (Hz) */
#define ARMPLL_MAX_VCO_RATE		(1000000000)
#define PERIPHPLL_MAX_VCO_RATE		(1200000000)
#define ENETPLL_MAX_VCO_RATE		(1000000000)
#define DDRPLL_MAX_VCO_RATE		(1066000000)
#define VIDEOPLL_MAX_VCO_RATE		(1200000000)

/* The min,max values for PLL PHI0 and PHI1 outputs (Hz) */
#define ARMPLL_MAX_PHI0_MAX_RATE	(1000000000)
#define ARMPLL_MAX_PHI1_MAX_RATE	(1000000000)
#define PERIPHPLL_MAX_PHI0_MAX_RATE	(400000000)
#define PERIPHPLL_MAX_PHI1_MAX_RATE	(100000000)
#define ENETPLL_MAX_PHI0_MAX_RATE	(500000000)
#define ENETPLL_MAX_PHI1_MAX_RATE	(1000000000)
#define DDRPLL_MAX_PHI0_MAX_RATE	(533000000)
#define DDRPLL_MAX_PHI1_MAX_RATE	(1066000000)
#define VIDEOPLL_MAX_PHI0_MAX_RATE	(600000000)

/* The maximum value for PLL VCO according to data sheet */
#define MAX_VCO_RATE			(1300000000)
#define MIN_VCO_RATE			(650000000)

/*
 * struct clk_plldig - S32 PLLDIG clock
 * @clk_hw:	   clock source
 * @base:	   base address of PLL registers
 * @plldv_mfd:	   multiplication loop factor divider
 * @plldv_rfdphi:  PHI reduced frequency divider
 * @plldv_rfdphi1: PHI reduced frequency divider
 *
 * PLLDIG clock version 1, found on S32 series.
 */
struct clk_plldig {
	struct clk_hw	hw;
	void __iomem	*base;
	enum s32_plldig_type type;
	u32		plldv_mfd;
	u32		pllfd_mfn;
	u32		plldv_rfdphi;
	u32		plldv_rfdphi1;
};

#define to_clk_plldig(_hw) container_of(_hw, struct clk_plldig, hw)

static unsigned long get_pllx_max_vco_rate(enum s32_plldig_type plltype)
{
	switch (plltype) {
	case S32_PLLDIG_ARM:
		return ARMPLL_MAX_VCO_RATE;
	case S32_PLLDIG_PERIPH:
		return PERIPHPLL_MAX_VCO_RATE;
	case S32_PLLDIG_ENET:
		return ENETPLL_MAX_VCO_RATE;
	case S32_PLLDIG_DDR:
		return DDRPLL_MAX_VCO_RATE;
	case S32_PLLDIG_VIDEO:
		return VIDEOPLL_MAX_VCO_RATE;
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32_PLLDIG_ARM,
			S32_PLLDIG_VIDEO);
			return -EINVAL;
	}
}

static unsigned long get_pllx_phiy_max_rate(enum s32_plldig_type plltype,
					    unsigned int phino)
{
	switch (plltype) {
	case S32_PLLDIG_ARM:
		switch (phino) {
		case 0:
			return ARMPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return ARMPLL_MAX_PHI1_MAX_RATE;
		}
		break;
	case S32_PLLDIG_PERIPH:
		switch (phino) {
		case 0:
			return PERIPHPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return PERIPHPLL_MAX_PHI1_MAX_RATE;
		}
		break;
	case S32_PLLDIG_ENET:
		switch (phino) {
		case 0:
			return ENETPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return ENETPLL_MAX_PHI1_MAX_RATE;
		}
		break;
	case S32_PLLDIG_DDR:
		switch (phino) {
		case 0:
			return DDRPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return DDRPLL_MAX_PHI1_MAX_RATE;
		}
		break;
	case S32_PLLDIG_VIDEO:
		switch (phino) {
		case 0:
			return VIDEOPLL_MAX_PHI0_MAX_RATE;
		case 1:
			return -EINVAL;
		}
	default:
		pr_warn("Unsupported PLL. Use: %d or %d\n",
			S32_PLLDIG_ARM,
			S32_PLLDIG_VIDEO);
		break;
	}
	return -EINVAL;
}

#if 0 /* Disable until an pll wait locking method will be implemented */
static int clk_plldig_wait_lock(struct clk_plldig *pll)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(10);
	u32 val = readl_relaxed(pll->base) & BM_PLL_POWER;

	/* No need to wait for lock when pll is not powered up */
	if ((pll->powerup_set && !val) || (!pll->powerup_set && val))
		return 0;

	/* Wait for PLL to lock */
	do {
		if (readl_relaxed(pll->base) & BM_PLL_LOCK)
			break;
		if (time_after(jiffies, timeout))
			break;
		usleep_range(50, 500);
	} while (1);

	return readl_relaxed(pll->base) & BM_PLL_LOCK ? 0 : -ETIMEDOUT;
}
#endif

static int clk_plldig_prepare(struct clk_hw *hw)
{

#if 0 /* Currently, this feature is desactivated */
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for PLLDIG
	 * if it is possible
	 */
#endif

	return 0;
}

static void clk_plldig_unprepare(struct clk_hw *hw)
{
#if 0 /* Currently, this feature is desactivated */
	/*
	 * TODO: When SOC is available, this function
	 * should be tested and implemented for PLLDIG
	 * if it is possible
	 */
#endif
}

static unsigned long clk_plldig_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 plldv = readl_relaxed(PLLDIG_PLLDV(pll->base));
	u32 pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	u32 prediv, mfd, mfn, vco;

	prediv = (plldv & PLLDIG_PLLDV_PREDIV_MASK)
			 >> PLLDIG_PLLDV_PREDIV_OFFSET;
	mfd = (plldv & PLLDIG_PLLDV_MFD_MASK);

	mfn = (pllfd & PLLDIG_PLLFD_MFN_MASK);

	if (prediv == 0)
		prediv = 1;

	/*
	 * This formula is from platform reference manual
	 * (Rev. 1, 6/2015), PLLDIG chapter.
	 */
	vco = (parent_rate / prediv) * (mfd + mfn/20480);

	return vco;
}

static long clk_plldig_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *prate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);

	if (rate > max_allowed_rate)
		rate = max_allowed_rate;
	else if (rate < MIN_VCO_RATE)
		rate = MIN_VCO_RATE;

	return rate;
}

static int clk_plldig_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_plldig *pll = to_clk_plldig(hw);
	u32 plldv, pllfd, prediv;

	unsigned long max_allowed_rate = get_pllx_max_vco_rate(pll->type);
	unsigned long phi0_max_rate =  get_pllx_phiy_max_rate(pll->type, 0);
	unsigned long phi1_max_rate =  get_pllx_phiy_max_rate(pll->type, 1);

	if (rate < MIN_VCO_RATE || rate > max_allowed_rate)
		return -EINVAL;

	if (((rate/pll->plldv_rfdphi) > phi0_max_rate) ||
	    ((rate/pll->plldv_rfdphi) > phi1_max_rate))
		return -EINVAL;

	plldv = readl_relaxed(PLLDIG_PLLDV(pll->base));
	pllfd = readl_relaxed(PLLDIG_PLLFD(pll->base));
	prediv = (parent_rate / rate) * (pll->plldv_mfd + pll->pllfd_mfn/20480);

	writel_relaxed(PLLDIG_PLLDV_RFDPHI1_SET(pll->plldv_rfdphi1) |
			PLLDIG_PLLDV_RFDPHI_SET(pll->plldv_rfdphi) |
			PLLDIG_PLLDV_PREDIV_SET(prediv) |
			PLLDIG_PLLDV_MFD_SET(pll->plldv_mfd),
			PLLDIG_PLLDV(pll->base));

	writel_relaxed(pllfd | PLLDIG_PLLFD_MFN_SET(pll->pllfd_mfn),
			PLLDIG_PLLFD(pll->base));

	/*
	 * To be implemented the wait_lock or an equivalent state
	 * return clk_plldig_wait_lock(pll);
	 */
	return 0;
}

static const struct clk_ops clk_plldig_ops = {
	.prepare	= clk_plldig_prepare,
	.unprepare	= clk_plldig_unprepare,
	.recalc_rate	= clk_plldig_recalc_rate,
	.round_rate	= clk_plldig_round_rate,
	.set_rate	= clk_plldig_set_rate,
};

struct clk *s32_clk_plldig_phi(enum s32_plldig_type type, const char *name,
			       const char *parent, void __iomem *base,
			       u32 phi)
{
	u32 plldv, rfd_phi;

	if (!base)
		return ERR_PTR(-ENOMEM);

	plldv = readl_relaxed(PLLDIG_PLLDV(base));

	switch (phi) {
	/* PHI0 */
	case 0:
		rfd_phi = (plldv & PLLDIG_PLLDV_RFDPHI_MASK)
			  >> PLLDIG_PLLDV_RFDPHI_OFFSET;
		break;
	/* PHI1 */
	case 1:
		rfd_phi = (plldv & PLLDIG_PLLDV_RFDPHI1_MASK)
			  >> PLLDIG_PLLDV_RFDPHI1_OFFSET;

		if (rfd_phi == 0)
			rfd_phi = 1;

		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	return clk_register_fixed_factor(NULL, name, parent,
			CLK_SET_RATE_PARENT, 1, rfd_phi);

}

struct clk *s32_clk_plldig(enum s32_plldig_type type, const char *name,
			       const char *parent_name, void __iomem *base,
			       u32 plldv_mfd, u32 pllfd_mfn,
			       u32 plldv_rfdphi, u32 plldv_rfdphi1)
{
	struct clk_plldig *pll;
	const struct clk_ops *ops;
	struct clk *clk;
	struct clk_init_data init;

	if ((plldv_rfdphi > PLLDIG_PLLDV_RFDPHI_MAXVALUE) ||
	    (plldv_rfdphi < PLLDIG_PLLDV_RFDPHI_MINVALUE))
		return ERR_PTR(-EINVAL);

	if ((plldv_rfdphi1 > PLLDIG_PLLDV_RFDPHI1_MAXVALUE) ||
	    (plldv_rfdphi1 < PLLDIG_PLLDV_RFDPHI1_MINVALUE))
		return ERR_PTR(-EINVAL);

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	ops = &clk_plldig_ops;

	pll->base = base;
	pll->type = type;
	pll->plldv_mfd = plldv_mfd;
	pll->pllfd_mfn = pllfd_mfn;
	pll->plldv_rfdphi = plldv_rfdphi;
	pll->plldv_rfdphi1 = plldv_rfdphi1;

	init.name = name;
	init.ops = ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
