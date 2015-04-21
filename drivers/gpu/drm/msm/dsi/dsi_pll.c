/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include "dsi.h"
#include "dsi.xml.h"
#include "pll.xml.h"

#ifndef CONFIG_COMMON_CLK
struct msm_dsi_pll *msm_dsi_pll_init(struct platform_device *pdev,
			enum msm_dsi_phy_type type, int dsi_id) {
	return ERR_PTR(-ENODEV);
}
void msm_dsi_pll_destroy(struct msm_dsi_pll *pll)
{
}
#else

#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "pll.xml.h"

#define NUM_DSI_CLOCKS_MAX	6
#define MAX_DSI_PLL_EN_SEQS	10

struct msm_dsi_pll {
	struct platform_device *pdev;
	void __iomem	*mmio;

	enum msm_dsi_phy_type type;
	int		dsi_id;

	struct clk_hw	clk_hw;
	unsigned long	vco_cached_rate;
	bool		pll_on;
	unsigned long	ref_clk_rate;
	unsigned long	min_rate;
	unsigned long	max_rate;
	u32		pll_en_seq_cnt;

	/* Loop filter resistance: */
	const struct lpfr_cfg *lpfr_lut;
	u32		lpfr_lut_size;

	int vco_delay;

	int (*pll_enable_seqs[MAX_DSI_PLL_EN_SEQS]) (struct msm_dsi_pll *pll);

	/* private clocks: */
	struct clk	*clks[NUM_DSI_CLOCKS_MAX];
	int		num_clks;
};
#define hw_clk_to_pll(x) container_of(x, struct msm_dsi_pll, clk_hw)

struct lpfr_cfg {
	unsigned long vco_rate;
	u32 resistance;
};

static void pll_write(struct msm_dsi_pll *pll, u32 reg, u32 data)
{
	msm_writel(data, pll->mmio + reg);
}

static u32 pll_read(struct msm_dsi_pll *pll, u32 reg)
{
	return msm_readl(pll->mmio + reg);
}

/*
 * DSI PLL 28nm - clock diagram (eg: DSI0):
 *
 *         dsi0analog_postdiv_clk
 *                             |         dsi0indirect_path_div2_clk
 *                             |          |
 *                   +------+  |  +----+  |  |\   dsi0byte_mux
 *  dsi0vco_clk --o--| DIV1 |--o--| /2 |--o--| \   |
 *                |  +------+     +----+     | m|  |  +----+
 *                |                          | u|--o--| /4 |-- dsi0pllbyte
 *                |                          | x|     +----+
 *                o--------------------------| /
 *                |                          |/
 *                |          +------+
 *                o----------| DIV3 |------------------------- dsi0pll
 *                           +------+
 */

#define DSI_PLL_POLL_MAX_READS			10
#define DSI_PLL_POLL_TIMEOUT_US			50

static bool poll_for_pll_ready_status(struct msm_dsi_pll *pll,
				u32 nb_tries, u32 timeout_us)
{
	bool pll_locked = false;
	u32 val;

	while (nb_tries--) {
		val = pll_read(pll, REG_PLL_28nm_STATUS);
		pll_locked = !!(val & PLL_28nm_STATUS_PLL_RDY);

		if (pll_locked)
			break;

		udelay(timeout_us);
	}
	DBG("eDP PLL is %slocked", pll_locked ? "" : "*not* ");

	return pll_locked;
}

static void dsi_pll_software_reset(struct msm_dsi_pll *pll)
{
	/*
	 * Add HW recommended delays after toggling the software
	 * reset bit off and back on.
	 */
	pll_write(pll, REG_PLL_28nm_TEST_CFG, PLL_28nm_TEST_CFG_PLL_SW_RESET);
	udelay(1);
	pll_write(pll, REG_PLL_28nm_TEST_CFG, 0x00);
	udelay(1);
}

static int dsi_pll_enable_seq(struct msm_dsi_pll *pll)
{
	struct device *dev = &pll->pdev->dev;
	u32 max_reads = 5, timeout_us = 100;
	bool locked;
	u32 val;
	int i;

	dsi_pll_software_reset(pll);

	/*
	 * PLL power up sequence.
	 * Add necessary delays recommended by hardware.
	 */
	val = PLL_28nm_GLB_CFG_PLL_PWRDN_B;
	pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
	udelay(1);

	val |= PLL_28nm_GLB_CFG_PLL_PWRGEN_PWRDN_B;
	pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
	udelay(200);

	val |= PLL_28nm_GLB_CFG_PLL_LDO_PWRDN_B;
	pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
	udelay(500);

	val |= PLL_28nm_GLB_CFG_PLL_ENABLE;
	pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
	udelay(500);

	for (i = 0; i < 2; i++) {
		udelay(100);

		/* DSI Uniphy lock detect setting */
		pll_write(pll, REG_PLL_28nm_LKDET_CFG2, 0x0c);
		udelay(100);
		pll_write(pll, REG_PLL_28nm_LKDET_CFG2, 0x0d);

		/* poll for PLL ready status */
		locked = poll_for_pll_ready_status(pll, max_reads, timeout_us);
		if (locked)
			break;

		dsi_pll_software_reset(pll);

		/*
		 * PLL power up sequence.
		 * Add necessary delays recommended by hardware.
		 */
		val = PLL_28nm_GLB_CFG_PLL_PWRDN_B;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(1);

		val |= PLL_28nm_GLB_CFG_PLL_PWRGEN_PWRDN_B;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(200);

		val |= PLL_28nm_GLB_CFG_PLL_LDO_PWRDN_B;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(250);

		val &= ~PLL_28nm_GLB_CFG_PLL_LDO_PWRDN_B;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(200);

		val |= PLL_28nm_GLB_CFG_PLL_LDO_PWRDN_B;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(500);

		val |= PLL_28nm_GLB_CFG_PLL_ENABLE;
		pll_write(pll, REG_PLL_28nm_GLB_CFG, val);
		udelay(500);
	}

	if (unlikely(!locked))
		dev_err(dev, "DSI PLL lock failed\n");
	else
		DBG("DSI PLL Lock success");

	return locked ? 0 : -EINVAL;
}

static int dsi_pll_enable_seq_lp(struct msm_dsi_pll *pll)
{
	struct device *dev = &pll->pdev->dev;
	bool locked;
	u32 max_reads = 10, timeout_us = 50;

	dsi_pll_software_reset(pll);

	/*
	 * PLL power up sequence.
	 * Add necessary delays recommended by hardware.
	 */
	pll_write(pll, REG_PLL_28nm_CAL_CFG1, 0x34);
	ndelay(500);
	pll_write(pll, REG_PLL_28nm_GLB_CFG, 0x01);
	ndelay(500);
	pll_write(pll,REG_PLL_28nm_GLB_CFG, 0x05);
	ndelay(500);
	pll_write(pll, REG_PLL_28nm_GLB_CFG, 0x0f);
	ndelay(500);

	/* DSI PLL toggle lock detect setting */
	pll_write(pll, REG_PLL_28nm_LKDET_CFG2, 0x04);
	ndelay(500);
	pll_write(pll, REG_PLL_28nm_LKDET_CFG2, 0x05);
	udelay(512);

	locked = poll_for_pll_ready_status(pll, max_reads, timeout_us);

	if (unlikely(!locked))
		dev_err(dev, "DSI PLL lock failed\n");
	else
		DBG("DSI PLL lock success");

	return locked ? 0 : -EINVAL;
}

static int dsi_pll_enable(struct clk_hw *hw)
{
	int i, ret = 0;
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);
	struct device *dev = &pll->pdev->dev;

	VERB("DSI:%d", pll->dsi_id);

	/* Try all enable sequences until one succeeds */
	for (i = 0; i < pll->pll_en_seq_cnt; i++) {
		ret = pll->pll_enable_seqs[i](pll);
		DBG("DSI PLL %s after sequence #%d",
			ret ? "unlocked" : "locked", i + 1);
		if (!ret)
			break;
	}

	if (ret) {
		dev_err(dev, "DSI PLL failed to lock\n");
		return ret;
	}

	pll->pll_on = true;

	return 0;
}

static void dsi_pll_disable(struct clk_hw *hw)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);

	/*
	 * Certain PLLs do not allow VCO rate update when it is on.
	 * Keep track of their status to turn on/off after set rate success.
	 */
	if (unlikely(!pll->pll_on))
		return;

	pll_write(pll, REG_PLL_28nm_GLB_CFG, 0x00);

	pll->pll_on = false;

	DBG("DSI PLL disabled");
}

static int dsi_pll_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);
	struct device *dev = &pll->pdev->dev;
	s64 refclk_cfg, frac_n_mode, ref_doubler_en_b;
	s64 ref_clk_to_pll, div_fbx1000, frac_n_value;
	s64 sdm_cfg0, sdm_cfg1, sdm_cfg2, sdm_cfg3;
	s64 gen_vco_clk, cal_cfg10, cal_cfg11;
	s64 vco_clk_rate = rate;
	s32 rem;
	int i;

	VERB("rate=%lu, parent's=%lu", rate, parent_rate);

	/* Force postdiv2 to be div-4 */
	pll_write(pll, REG_PLL_28nm_POSTDIV2_CFG, 3);

	/* Configure the Loop filter resistance */
	for (i = 0; i < pll->lpfr_lut_size; i++)
		if (rate <= pll->lpfr_lut[i].vco_rate)
			break;
	if (i == pll->lpfr_lut_size) {
		dev_err(dev, "unable to get loop filter resistance. vco=%lu\n",
				rate);
		return -EINVAL;
	}
	pll_write(pll, REG_PLL_28nm_LPFR_CFG, pll->lpfr_lut[i].resistance);

	/* Loop filter capacitance values : c1 and c2 */
	pll_write(pll, REG_PLL_28nm_LPFC1_CFG, 0x70);
	pll_write(pll, REG_PLL_28nm_LPFC2_CFG, 0x15);

	div_s64_rem(vco_clk_rate, pll->ref_clk_rate, &rem);
	if (rem) {
		refclk_cfg = 0x1;
		frac_n_mode = 1;
		ref_doubler_en_b = 0;
	} else {
		refclk_cfg = 0x0;
		frac_n_mode = 0;
		ref_doubler_en_b = 1;
	}

	DBG("refclk_cfg = %lld", refclk_cfg);

	ref_clk_to_pll = ((pll->ref_clk_rate * 2 * (refclk_cfg))
			  + (ref_doubler_en_b * pll->ref_clk_rate));
	div_fbx1000 = div_s64((vco_clk_rate * 1000), ref_clk_to_pll);

	div_s64_rem(div_fbx1000, 1000, &rem);
	frac_n_value = div_s64((rem * (1 << 16)), 1000);
	gen_vco_clk = div_s64(div_fbx1000 * ref_clk_to_pll, 1000);

	DBG("ref_clk_to_pll = %lld", ref_clk_to_pll);
	DBG("div_fb = %lld", div_fbx1000);
	DBG("frac_n_value = %lld", frac_n_value);

	DBG("Generated VCO Clock: %lld", gen_vco_clk);
	rem = 0;
	if (frac_n_mode) {
		sdm_cfg0 = (0x0 << 5);
		sdm_cfg0 |= (0x0 & 0x3f);
		sdm_cfg1 = (div_s64(div_fbx1000, 1000) & 0x3f) - 1;
		sdm_cfg3 = div_s64_rem(frac_n_value, 256, &rem);
		sdm_cfg2 = rem;
	} else {
		sdm_cfg0 = (0x1 << 5);
		sdm_cfg0 |= (div_s64(div_fbx1000, 1000) & 0x3f) - 1;
		sdm_cfg1 = (0x0 & 0x3f);
		sdm_cfg2 = 0;
		sdm_cfg3 = 0;
	}

	DBG("sdm_cfg0=%lld", sdm_cfg0);
	DBG("sdm_cfg1=%lld", sdm_cfg1);
	DBG("sdm_cfg2=%lld", sdm_cfg2);
	DBG("sdm_cfg3=%lld", sdm_cfg3);

	cal_cfg11 = div_s64_rem(gen_vco_clk, 256 * 1000000, &rem);
	cal_cfg10 = rem / 1000000;
	DBG("cal_cfg10=%lld, cal_cfg11=%lld", cal_cfg10, cal_cfg11);

	pll_write(pll, REG_PLL_28nm_CHGPUMP_CFG, 0x02);
	pll_write(pll, REG_PLL_28nm_CAL_CFG3,    0x2b);
	pll_write(pll, REG_PLL_28nm_CAL_CFG4,    0x66);
	pll_write(pll, REG_PLL_28nm_LKDET_CFG2,  0x0d);

	pll_write(pll, REG_PLL_28nm_SDM_CFG1, (u32)(sdm_cfg1 & 0xff));
	pll_write(pll, REG_PLL_28nm_SDM_CFG2, (u32)(sdm_cfg2 & 0xff));
	pll_write(pll, REG_PLL_28nm_SDM_CFG3, (u32)(sdm_cfg3 & 0xff));
	pll_write(pll, REG_PLL_28nm_SDM_CFG4, 0x00);

	/* Add hardware recommended delay for correct PLL configuration */
	udelay(pll->vco_delay);

	pll_write(pll, REG_PLL_28nm_REFCLK_CFG, (u32)refclk_cfg);
	pll_write(pll, REG_PLL_28nm_PWRGEN_CFG, 0x00);
	pll_write(pll, REG_PLL_28nm_VCOLPF_CFG, 0x71);
	pll_write(pll, REG_PLL_28nm_SDM_CFG0,   (u32)sdm_cfg0);
	pll_write(pll, REG_PLL_28nm_CAL_CFG0,   0x12);
	pll_write(pll, REG_PLL_28nm_CAL_CFG6,   0x30);
	pll_write(pll, REG_PLL_28nm_CAL_CFG7,   0x00);
	pll_write(pll, REG_PLL_28nm_CAL_CFG8,   0x60);
	pll_write(pll, REG_PLL_28nm_CAL_CFG9,   0x00);
	pll_write(pll, REG_PLL_28nm_CAL_CFG10, (u32)(cal_cfg10 & 0xff));
	pll_write(pll, REG_PLL_28nm_CAL_CFG11, (u32)(cal_cfg11 & 0xff));
	pll_write(pll, REG_PLL_28nm_EFUSE_CFG,  0x20);

	return 0;
}

static long dsi_pll_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);

	if      (rate < pll->min_rate)
		return  pll->min_rate;
	else if (rate > pll->max_rate)
		return  pll->max_rate;
	else
		return rate;
}

static int dsi_pll_is_enabled(struct clk_hw *hw)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);

	return poll_for_pll_ready_status(pll, DSI_PLL_POLL_MAX_READS,
					      DSI_PLL_POLL_TIMEOUT_US);
}

static unsigned long dsi_pll_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);
	u32 sdm0, doubler, sdm_byp_div;
	u64 vco_rate;
	u32 sdm_dc_off, sdm_freq_seed, sdm2, sdm3;
	u64 ref_clk = pll->ref_clk_rate;

	VERB("parent_rate=%lu", parent_rate);

	/* Check to see if the ref clk doubler is enabled */
	doubler = pll_read(pll, REG_PLL_28nm_REFCLK_CFG) & BIT(0);
	ref_clk += (doubler * pll->ref_clk_rate);

	/* see if it is integer mode or sdm mode */
	sdm0 = pll_read(pll, REG_PLL_28nm_SDM_CFG0);
	if (sdm0 & BIT(6)) {
		/* integer mode */
		sdm_byp_div = (pll_read(pll, REG_PLL_28nm_SDM_CFG0) & 0x3f) + 1;
		vco_rate = ref_clk * sdm_byp_div;
	} else {
		/* sdm mode */
		sdm_dc_off = pll_read(pll, REG_PLL_28nm_SDM_CFG1) & 0xFF;
		DBG("sdm_dc_off = %d", sdm_dc_off);
		sdm2 = pll_read(pll, REG_PLL_28nm_SDM_CFG2) & 0xFF;
		sdm3 = pll_read(pll, REG_PLL_28nm_SDM_CFG3) & 0xFF;
		sdm_freq_seed = (sdm3 << 8) | sdm2;
		DBG("sdm_freq_seed = %d", sdm_freq_seed);

		vco_rate = (ref_clk * (sdm_dc_off + 1)) +
			mult_frac(ref_clk, sdm_freq_seed, BIT(16));
		DBG("vco rate = %lld", vco_rate);
	}

	DBG("returning vco rate = %lu", (unsigned long)vco_rate);

	return (unsigned long)vco_rate;
}

static int dsi_pll_prepare(struct clk_hw *hw)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);
	struct device *dev = &pll->pdev->dev;
	int ret;

	VERB("DSI:%d", pll->dsi_id);

	/*
	 * Certain PLLs need to update the same VCO rate after resume in
	 * suspend/resume scenario. Cache the VCO rate for such PLLs.
	 */
	if ((pll->vco_cached_rate != 0) &&
	    (pll->vco_cached_rate == __clk_get_rate(hw->clk))) {
		ret = dsi_pll_set_rate(hw, pll->vco_cached_rate, 0);
		if (ret) {
			dev_err(dev, "dsi_pll_set_rate failed. ret=%d\n", ret);
			goto error;
		}
	}

	ret = dsi_pll_enable(hw);

error:
	return ret;
}

static void dsi_pll_unprepare(struct clk_hw *hw)
{
	struct msm_dsi_pll *pll = hw_clk_to_pll(hw);

	pll->vco_cached_rate = __clk_get_rate(hw->clk);
	dsi_pll_disable(hw);

	VERB("DSI:%d", pll->dsi_id);
}

static const struct clk_ops clk_ops_dsi_vco = {
	.round_rate = dsi_pll_round_rate,
	.set_rate = dsi_pll_set_rate,
	.recalc_rate = dsi_pll_recalc_rate,
	.prepare = dsi_pll_prepare,
	.unprepare = dsi_pll_unprepare,
	.is_enabled = dsi_pll_is_enabled,
};

static struct clk_init_data dsi_init[] = {
	[0] = {
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.name = "dsi0vco_clk",
		.ops = &clk_ops_dsi_vco,
	},
	[1] = {
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.name = "dsi1vco_clk",
		.ops = &clk_ops_dsi_vco,
	},
};

static void dsi_28nm_pll_unregister(struct msm_dsi_pll *pll)
{
	of_clk_del_provider(pll->pdev->dev.of_node);

	DBG("DSI:%d", pll->dsi_id);

	if (!pll->num_clks)
		return;

	do {
		clk_unregister(pll->clks[--pll->num_clks]);
	} while (pll->num_clks);
}

static int dsi_28nm_pll_register(struct msm_dsi_pll *pll)
{
	char clk_name[32], parent1[32], parent2[32];
	struct device *dev = &pll->pdev->dev;
	struct clk **clks = pll->clks;
	struct clk *ahb_clk;
	int num = 0;
	int ret;

	DBG("DSI:%d", pll->dsi_id);

	/* grab ahb clock for initial configurations */
	ahb_clk = clk_get(dev, "iface_clk");
	if (IS_ERR(ahb_clk)) {
		dev_err(dev, "failed to get interface clock\n");
		return PTR_ERR(ahb_clk);
	}

	ret = clk_prepare_enable(ahb_clk);
	if (ret) {
		dev_err(dev, "failed to enable interface clock\n");
		goto err_enable;
	}

	pll->clk_hw.init = &dsi_init[pll->dsi_id];

	clks[num++] = clk_register(dev, &pll->clk_hw);

	sprintf(clk_name, "dsi%danalog_postdiv_clk", pll->dsi_id);
	sprintf(parent1, "dsi%dvco_clk", pll->dsi_id);
	clks[num++] = clk_register_divider(dev, clk_name,
			parent1, CLK_SET_RATE_PARENT,
			pll->mmio +
			REG_PLL_28nm_POSTDIV1_CFG,
			0, 4, 0, NULL);

	sprintf(clk_name, "dsi%dindirect_path_div2_clk", pll->dsi_id);
	sprintf(parent1, "dsi%danalog_postdiv_clk", pll->dsi_id);
	clks[num++] = clk_register_fixed_factor(dev, clk_name,
			parent1, CLK_SET_RATE_PARENT,
			1, 2);

	sprintf(clk_name, "dsi%dpll", pll->dsi_id);
	sprintf(parent1, "dsi%dvco_clk", pll->dsi_id);
	clks[num++] = clk_register_divider(dev, clk_name,
				parent1, 0, pll->mmio +
				REG_PLL_28nm_POSTDIV3_CFG,
				0, 8, 0, NULL);

	sprintf(clk_name, "dsi%dbyte_mux", pll->dsi_id);
	sprintf(parent1, "dsi%dvco_clk", pll->dsi_id);
	sprintf(parent2, "dsi%dindirect_path_div2_clk", pll->dsi_id);
	clks[num++] = clk_register_mux(dev, clk_name,
			(const char *[]){
				parent1, parent2
			}, 2, CLK_SET_RATE_PARENT, pll->mmio +
			REG_PLL_28nm_VREG_CFG, 1, 1, 0, NULL);

	sprintf(clk_name, "dsi%dpllbyte", pll->dsi_id);
	sprintf(parent1, "dsi%dbyte_mux", pll->dsi_id);
	clks[num++] = clk_register_fixed_factor(dev, clk_name,
				parent1, CLK_SET_RATE_PARENT, 1, 4);

	pll->num_clks = num;

	clk_disable_unprepare(ahb_clk);

err_enable:
	clk_put(ahb_clk);

	return ret;
}

static void dsi_28nm_pll_destroy(struct msm_dsi_pll *pll)
{
	dsi_28nm_pll_unregister(pll);
}


static int dsi_28nm_pll_init(struct msm_dsi_pll *pll)
{
	int ret;
	static const struct lpfr_cfg lpfr_lut[] = {
			{ 479500000,  8 },
			{ 480000000, 11 },
			{ 575500000,  8 },
			{ 576000000, 12 },
			{ 610500000,  8 },
			{ 659500000,  9 },
			{ 671500000, 10 },
			{ 672000000, 14 },
			{ 708500000, 10 },
			{ 750000000, 11 },
	};

	pll->ref_clk_rate = 19200000;
	pll->min_rate = 350000000;
	pll->max_rate = 750000000;

	if (pll->type == MSM_DSI_PHY_28NM) {
		pll->vco_delay = 1;

		pll->pll_en_seq_cnt = 3;
		pll->pll_enable_seqs[0] = dsi_pll_enable_seq;
		pll->pll_enable_seqs[1] = dsi_pll_enable_seq;
		pll->pll_enable_seqs[2] = dsi_pll_enable_seq;
	} else {
		pll->vco_delay = 1000;

		pll->pll_en_seq_cnt = 1;
		pll->pll_enable_seqs[0] = dsi_pll_enable_seq_lp;
	}

	pll->lpfr_lut = lpfr_lut;
	pll->lpfr_lut_size = ARRAY_SIZE(lpfr_lut);

	ret = dsi_28nm_pll_register(pll);
	if (ret) {
		dev_err(&pll->pdev->dev, "failed to register PLL: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * DSI PLL common
 */

void msm_dsi_pll_destroy(struct msm_dsi_pll *pll)
{
	switch (pll->type) {
	case MSM_DSI_PHY_28NM:
	case MSM_DSI_PHY_28NM_LP:
		dsi_28nm_pll_destroy(pll);
		break;
	default:
		dev_err(&pll->pdev->dev, "unsupported type: %d\n", pll->type);
	}
}

struct msm_dsi_pll *msm_dsi_pll_init(struct platform_device *pdev,
			enum msm_dsi_phy_type type, int dsi_id)
{
	struct device *dev = &pdev->dev;
	struct msm_dsi_pll *pll;
	int ret;

	pll = devm_kzalloc(&pdev->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return NULL;

	pll->mmio = msm_ioremap(pdev, "dsi_pll", "DSI_PLL");
	if (IS_ERR_OR_NULL(pll->mmio)) {
		dev_err(dev, "%s: failed to map pll pll_base\n", __func__);
		return NULL;
	}

	pll->pdev = pdev;
	pll->type = type;
	pll->dsi_id = dsi_id;

	switch (type) {
	case MSM_DSI_PHY_28NM:
	case MSM_DSI_PHY_28NM_LP:
		ret = dsi_28nm_pll_init(pll);
		break;
	default:
		ret = -ENXIO;
	}

	if (ret) {
		dev_err(dev, "%s: failed to init DSI PLL\n", __func__);
		return NULL;
	}

	DBG("DSI:%d PLL registered", dsi_id);

	return pll;
}

#endif /* CONFIG_COMMON_CLK */
