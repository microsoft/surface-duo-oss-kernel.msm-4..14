#include "pcie-kirin.h"
/*lint -e438 -e550 -e715 -esym(438,*) -esym(550,*) -esym(715,*) */

static void kirin_pcie_oe_ctrl(struct kirin_pcie *pcie, int en_flag)
{
	u32 val;

	regmap_read(pcie->sysctrl, SCTRL_SCPERCLKEN3, &val);//lint !e732

	/* set phy_debounce in&out time: 300ns*/
	val |= 0xF0F000;

	/*select oe_gt_mode */
	val |= 0x400;

	/*IO bypass */
	val &= ~IO_HARD_CTRL_DEBOUNCE_BYPASS;

	if (en_flag)
		val &= ~IO_OE_EN_HARD_BYPASS;
	else
		val |= IO_OE_EN_HARD_BYPASS;

	regmap_write(pcie->sysctrl, SCTRL_SCPERCLKEN3, val);
}

static void kirin_pcie_iso_ctrl(struct kirin_pcie *pcie, int en_flag)
{
	if (en_flag)
		regmap_write(pcie->sysctrl, pcie->isoen_offset, pcie->isoen_value);
	else
		regmap_write(pcie->sysctrl, pcie->isodis_offset, pcie->isodis_value);
}

static void kirin_pcie_mtcmos_ctrl(struct kirin_pcie *pcie, int enable)
{
	if (enable)/* [false alarm]:fortify */
		regmap_write(pcie->sysctrl, SCTRL_SCPWREN, MTCMOS_CTRL_BIT);
	else
		regmap_write(pcie->sysctrl, SCTRL_SCPWRDIS, MTCMOS_CTRL_BIT);
}

static void kirin_pcie_reset_ctrl(struct kirin_pcie *pcie, int reset)
{
	u32 val;

	if (reset) {
		val = pcie->core_assert_value | pcie->phy_assert_value;
		regmap_write(pcie->crgctrl, pcie->core_assert_offset, val);
	} else {
		val = pcie->core_deassert_value | pcie->phy_deassert_value;
		regmap_write(pcie->crgctrl, pcie->core_deassert_offset, val);
	}
}

static int kirin_pcie_clk_ctrl(struct kirin_pcie *pcie, int clk_on)
{
	int ret = 0;
	unsigned long ref_clk_rate = REF_CLK_FREQ;

	if (clk_on) {
		ret = clk_set_rate(pcie->phy_ref_clk, ref_clk_rate);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to set ref clk rate 100MHz ");
			return ret;
		}

		ret = clk_prepare_enable(pcie->phy_ref_clk);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to enable phy_ref_clk ");
			return ret;
		}

		ret = clk_prepare_enable(pcie->apb_sys_clk);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to enable apb_sys_clk ");
			clk_disable_unprepare(pcie->phy_ref_clk);
			return ret;
		}

		ret = clk_prepare_enable(pcie->apb_phy_clk);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to enable apb_phy_clk ");
			clk_disable_unprepare(pcie->apb_sys_clk);
			clk_disable_unprepare(pcie->phy_ref_clk);
			return ret;
		}

		ret = clk_prepare_enable(pcie->pcie_aclk);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to enable pcie_aclk ");
			clk_disable_unprepare(pcie->apb_phy_clk);
			clk_disable_unprepare(pcie->phy_ref_clk);
			clk_disable_unprepare(pcie->apb_sys_clk);
			return ret;
		}

		ret = clk_prepare_enable(pcie->pcie_aux_clk);
		if (0 != ret) {
			PCIE_PR_ERR("Failed to enable pcie_pclk ");
			clk_disable_unprepare(pcie->pcie_aclk);
			clk_disable_unprepare(pcie->apb_phy_clk);
			clk_disable_unprepare(pcie->phy_ref_clk);
			clk_disable_unprepare(pcie->apb_sys_clk);
			return ret;
		}
	} else {
		clk_disable_unprepare(pcie->apb_sys_clk);
		clk_disable_unprepare(pcie->apb_phy_clk);
		clk_disable_unprepare(pcie->pcie_aclk);
		clk_disable_unprepare(pcie->phy_ref_clk);
		clk_disable_unprepare(pcie->pcie_aux_clk);
	}

	return ret;
}

static int kirin_pcie_phy_init(struct kirin_pcie *pcie);

int kirin_pcie_power_on(struct pcie_port *pp, int on_flag)
{
	struct kirin_pcie *pcie = to_kirin_pcie(pp);//lint !e826
	int ret;
	u32 reg_val;
	struct pinctrl_state *pinctrl_def;
	struct pinctrl_state *pinctrl_idle;

	/*power on*/
	if (on_flag) {
		kirin_pcie_mtcmos_ctrl(pcie, ENABLE);
		udelay(100);//lint !e778  !e774  !e516
		PCIE_PR_DEBUG("mtcmos on Done ");

		kirin_pcie_oe_ctrl(pcie, ENABLE);
		PCIE_PR_DEBUG("OE CFG Done ");
			/*clk enable*/
		ret = kirin_pcie_clk_ctrl(pcie, ENABLE);
		udelay((unsigned long)1);//lint !e778  !e774  !e516
		if (ret)
			return -EINVAL;
		PCIE_PR_DEBUG("clock on Done ");

		/*clk disable*/
		ret = kirin_pcie_clk_ctrl(pcie, DISABLE);
		if (ret)
			return -EINVAL;
		udelay(1);//lint !e778  !e774  !e516
		PCIE_PR_DEBUG("clk off Done ");

		/*ISO disable*/
		kirin_pcie_iso_ctrl(pcie, DISABLE);
		PCIE_PR_DEBUG("iso disable Done ");

		/*unset module*/
		kirin_pcie_reset_ctrl(pcie, DISABLE);
		PCIE_PR_DEBUG("module unreset Done ");

		/*clk on*/
		ret = kirin_pcie_clk_ctrl(pcie, ENABLE);
		if (ret)
			return -EINVAL;
		PCIE_PR_DEBUG("clk on Done ");

		/*disable hardware auto-gate, enable software vote*/
		regmap_write(pcie->sysctrl, SCTRL_SCPERCLKEN2, HW_AUTO_CF_BIT);
		ret = kirin_pcie_phy_init(pcie);
		if (ret) {
			PCIE_PR_ERR("Phy Init Failed ");
			return ret;
		}
		PCIE_PR_DEBUG("Phy Init Done ");

		pcie->is_power_on = 1;

		pinctrl_def = pinctrl_lookup_state(pcie->pin,PINCTRL_STATE_DEFAULT);
		if (IS_ERR(pinctrl_def)) {
		        PCIE_PR_ERR("could not get defstate");
		        return -1;
		}

		if (pinctrl_select_state(pcie->pin, pinctrl_def)){
		        PCIE_PR_ERR("could not set reset pins to defstate");
		        return -1;
		}

		/*rst EP*/

		reg_val = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL12_ADDR);
		reg_val |= PERST_FUN_SEC;
		reg_val |= PERST_ASSERT_EN;
		kirin_elb_writel(pcie, reg_val, SOC_PCIECTRL_CTRL12_ADDR);
	} else {
		/*rst EP*/
		reg_val = kirin_elb_readl(pcie, SOC_PCIECTRL_CTRL12_ADDR);
		reg_val |= PERST_FUN_SEC;
		reg_val &= ~PERST_ASSERT_EN;
		kirin_elb_writel(pcie, reg_val, SOC_PCIECTRL_CTRL12_ADDR);
		mdelay(10);//lint !e778  !e774  !e516  !e747  !e845

		kirin_pcie_reset_ctrl(pcie, ENABLE);

		ret = kirin_pcie_clk_ctrl(pcie, DISABLE);
		if (ret)
			return -EINVAL;
		kirin_pcie_iso_ctrl(pcie, ENABLE);

		kirin_pcie_mtcmos_ctrl(pcie, DISABLE);

		kirin_pcie_oe_ctrl(pcie, DISABLE);
		PCIE_PR_DEBUG("OE CFG Done ");

		/*enable hardware auto-gate, disable software vote*/
		regmap_write(pcie->sysctrl, SCTRL_SCPERCLKDIS2, HW_AUTO_CF_BIT);

		pcie->is_power_on = 0;

		pinctrl_idle = pinctrl_lookup_state(pcie->pin,PINCTRL_STATE_IDLE);
		if (IS_ERR(pinctrl_idle)) {
		        PCIE_PR_ERR("could not get idlestate");
		        return -1;
		}

		if (pinctrl_select_state(pcie->pin,pinctrl_idle)) {
		        PCIE_PR_ERR("could not set reset pins to idle state");
		        return -1;
		}
		if (gpio_request((unsigned int)pcie->gpio_id_reset, "pcie_reset")) {
		        PCIE_PR_ERR("can't request gpio-%d", pcie->gpio_id_reset);
		        return -1;
		}

		if (gpio_direction_input((unsigned int)pcie->gpio_id_reset)) {
		        PCIE_PR_ERR("can't set reset_gpio direction input");
		        return -1;
		}
		gpio_free((unsigned int)pcie->gpio_id_reset);
	}
	return 0;
}

static void set_phy_eye_param(struct kirin_pcie *pcie)
{
	u32 eye_param;

	if (pcie->pcie_eye_param_ctrl2)
		kirin_phy_writel(pcie, pcie->pcie_eye_param_ctrl2, SOC_PCIEPHY_CTRL2_ADDR);

	if (pcie->pcie_eye_param_ctrl3)
		kirin_phy_writel(pcie, pcie->pcie_eye_param_ctrl3, SOC_PCIEPHY_CTRL3_ADDR);

	eye_param = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL2_ADDR);
	PCIE_PR_INFO("pcie_eye_param_ctrl2 is 0x%x", eye_param);
	eye_param = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL3_ADDR);
	PCIE_PR_INFO("pcie_eye_param_ctrl3 is 0x%x", eye_param);
}

static int kirin_pcie_phy_init(struct kirin_pcie *pcie)
{
	u32 reg_val;
	u32 pipe_clk_stable = 0x1 << 19;
	u32 time = 10;

	set_phy_eye_param(pcie);

	/*choose 100MHz clk src: Bit[8]==1 pll, Bit[8]==0 Board */
	reg_val = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL1_ADDR);
	reg_val &= ~(0x1 << 8);
	kirin_phy_writel(pcie, reg_val, SOC_PCIEPHY_CTRL1_ADDR);

	/*pull down phy_test_powerdown signal */
	reg_val = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL0_ADDR);
	reg_val &= ~(0x1 << 22);
	kirin_phy_writel(pcie, reg_val, SOC_PCIEPHY_CTRL0_ADDR);
	udelay((unsigned long)10);//lint !e778  !e774  !e516

	/*derst PHY */
	reg_val = kirin_phy_readl(pcie, SOC_PCIEPHY_CTRL1_ADDR);
	reg_val &= ~(0x1 << 16);
	kirin_phy_writel(pcie, reg_val, SOC_PCIEPHY_CTRL1_ADDR);


	reg_val = kirin_phy_readl(pcie, SOC_PCIEPHY_STATE0_ADDR);
	while (reg_val & pipe_clk_stable) {
		udelay((unsigned long)100);//lint !e778  !e774  !e516
		if (time == 0) {
			PCIE_PR_INFO("PIPE clk is not stable");
			return -EINVAL;
		}
		time--;
		reg_val = kirin_phy_readl(pcie, SOC_PCIEPHY_STATE0_ADDR);
	}

	return 0;
}
/*lint -e438 -e550 -e715 +esym(438,*) +esym(550,*) +esym(715,*) */

