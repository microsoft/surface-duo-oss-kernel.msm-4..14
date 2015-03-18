#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/mfd/qcom_rpm.h>
#include <dt-bindings/mfd/qcom-rpm.h>

struct rpm_cc {
	struct clk_onecell_data data;
	struct clk *clks[];
};
struct rpm_clk {
	const int rpm_clk_id;
	struct qcom_rpm *rpm;
	unsigned last_set_khz;
	bool enabled;
	bool branch; /* true: RPM only accepts 1 for ON and 0 for OFF */
	unsigned factor;
	struct clk_hw hw;
};

#define to_rpm_clk(_hw) container_of(_hw, struct rpm_clk, hw)

static int rpm_clk_prepare(struct clk_hw *hw)
{
	struct rpm_clk *r = to_rpm_clk(hw);
	uint32_t value;
	int rc = 0;
	unsigned long this_khz;

	this_khz = r->last_set_khz;
	/* Don't send requests to the RPM if the rate has not been set. */
	if (r->last_set_khz == 0)
		goto out;

	value = this_khz;
	if (r->branch)
		value = !!value;

	rc = qcom_rpm_write(r->rpm, QCOM_RPM_ACTIVE_STATE,
			    r->rpm_clk_id, &value, 1);
	if (rc)
		goto out;

out:
	if (!rc)
		r->enabled = true;
	return rc;
}

static void rpm_clk_unprepare(struct clk_hw *hw)
{
	struct rpm_clk *r = to_rpm_clk(hw);

	if (r->last_set_khz) {
		uint32_t value = 0;
		int rc;

		rc = qcom_rpm_write(r->rpm, QCOM_RPM_ACTIVE_STATE,
			    r->rpm_clk_id, &value, 1);
		if (rc)
			return;

	}
	r->enabled = false;
}

int rpm_clk_set_rate(struct clk_hw *hw,
		     unsigned long rate, unsigned long prate)
{
	struct rpm_clk *r = to_rpm_clk(hw);
	unsigned long this_khz;
	int rc = 0;

	this_khz = DIV_ROUND_UP(rate, r->factor);

	if (r->enabled) {
		uint32_t value = this_khz;

		rc = qcom_rpm_write(r->rpm, QCOM_RPM_ACTIVE_STATE,
				    r->rpm_clk_id, &value, 1);
		if (rc)
			goto out;
	}

	if (!rc)
		r->last_set_khz = this_khz;

out:
	return rc;
}

static long rpm_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static unsigned long rpm_clk_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct rpm_clk *r = to_rpm_clk(hw);
	u32 val;
	int rc;

	rc = qcom_rpm_read(r->rpm, r->rpm_clk_id, &val, 1);
	if (rc < 0)
		return 0;

	return val * r->factor;
}

static unsigned long rpm_branch_clk_recalc_rate(struct clk_hw *hw,
				unsigned long parent_rate)
{
	struct rpm_clk *r = to_rpm_clk(hw);

	return r->last_set_khz * r->factor;
}

static const struct clk_ops branch_clk_ops_rpm = {
	.prepare	= rpm_clk_prepare,
	.unprepare	= rpm_clk_unprepare,
	.recalc_rate	= rpm_branch_clk_recalc_rate,
	.round_rate	= rpm_clk_round_rate,
};

static const struct clk_ops clk_ops_rpm = {
	.prepare	= rpm_clk_prepare,
	.unprepare	= rpm_clk_unprepare,
	.set_rate	= rpm_clk_set_rate,
	.recalc_rate	= rpm_clk_recalc_rate,
	.round_rate	= rpm_clk_round_rate,
};

static struct rpm_clk pxo_clk = {
	.rpm_clk_id = QCOM_RPM_PXO_CLK,
	.branch	= true,
	.factor = 1000,
	.last_set_khz = 27000,
	.hw.init = &(struct clk_init_data){
		.name = "pxo",
		.ops = &branch_clk_ops_rpm,
	},
};

static struct rpm_clk cxo_clk = {
	.rpm_clk_id = QCOM_RPM_CXO_CLK,
	.branch	= true,
	.factor = 1000,
	.last_set_khz = 19200,
	.hw.init = &(struct clk_init_data){
		.name = "cxo",
		.ops = &branch_clk_ops_rpm,
	},
};

static struct rpm_clk afab_clk = {
	.rpm_clk_id = QCOM_RPM_APPS_FABRIC_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "afab_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk cfpb_clk = {
	.rpm_clk_id = QCOM_RPM_CFPB_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "cfpb_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk daytona_clk = {
	.rpm_clk_id = QCOM_RPM_DAYTONA_FABRIC_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "daytona_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk ebi1_clk = {
	.rpm_clk_id = QCOM_RPM_EBI1_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "ebi1_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk mmfab_clk = {
	.rpm_clk_id = QCOM_RPM_MM_FABRIC_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "mmfab_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk mmfpb_clk = {
	.rpm_clk_id = QCOM_RPM_MMFPB_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "mmfpb_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk sfab_clk = {
	.rpm_clk_id = QCOM_RPM_SYS_FABRIC_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "sfab_clk",
		.ops = &clk_ops_rpm,
	},
};

static struct rpm_clk sfpb_clk = {
	.rpm_clk_id = QCOM_RPM_SFPB_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "sfpb_clk",
		.ops = &clk_ops_rpm,
	},
};

/*
static struct rpm_clk qdss_clk = {
	.rpm_clk_id = QCOM_RPM_QDSS_CLK,
	.factor = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "qdss_clk",
		.ops = &clk_ops_rpm,
	},
};
*/

struct rpm_clk *rpm_clks[] = {
	[QCOM_RPM_PXO_CLK] = &pxo_clk,
	[QCOM_RPM_CXO_CLK] = &cxo_clk,
	[QCOM_RPM_APPS_FABRIC_CLK] = &afab_clk,
	[QCOM_RPM_CFPB_CLK] = &cfpb_clk,
	[QCOM_RPM_DAYTONA_FABRIC_CLK] = &daytona_clk,
	[QCOM_RPM_EBI1_CLK] = &ebi1_clk,
	[QCOM_RPM_MM_FABRIC_CLK] = &mmfab_clk,
	[QCOM_RPM_MMFPB_CLK] = &mmfpb_clk,
	[QCOM_RPM_SYS_FABRIC_CLK] = &sfab_clk,
	[QCOM_RPM_SFPB_CLK] = &sfpb_clk,
/**	[QCOM_RPM_QDSS_CLK] = &qdss_clk, Needs more checking here **/
};

static int rpm_clk_probe(struct platform_device *pdev)
{
	struct clk **clks;
	struct clk *clk;
	struct rpm_cc *cc;
	struct qcom_rpm *rpm;
	int num_clks = ARRAY_SIZE(rpm_clks);
	struct clk_onecell_data *data;
	int i;

	if (!pdev->dev.of_node)
		return -ENODEV;

	cc = devm_kzalloc(&pdev->dev, sizeof(*cc) + sizeof(*clks) * num_clks,
			  GFP_KERNEL);
	if (!cc)
		return -ENOMEM;

	clks = cc->clks;
	data = &cc->data;
	data->clks = clks;
	data->clk_num = num_clks;

	rpm = dev_get_drvdata(pdev->dev.parent);
	if (!rpm) {
		dev_err(&pdev->dev, "unable to retrieve handle to rpm\n");
		return -ENODEV;
	}

	for (i = 0; i < num_clks; i++) {
		if (!rpm_clks[i]) {
			clks[i] = ERR_PTR(-ENOENT);
			continue;
		}
		rpm_clks[i]->rpm = rpm;
		clk = devm_clk_register(&pdev->dev, &rpm_clks[i]->hw);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		clks[i] = clk;
	}

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				    data);
}

static int rpm_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static const struct of_device_id rpm_clk_of_match[] = {
	{ .compatible = "qcom,apq8064-rpm-clk" },
	{ },
};

static struct platform_driver rpm_clk_driver = {
	.driver = {
		.name = "qcom-rpm-clk",
		.of_match_table = rpm_clk_of_match,
	},
	.probe = rpm_clk_probe,
	.remove = rpm_clk_remove,
};

/* dummy handoff clk handling... */
static int rpm_clk_ho_probe(struct platform_device *pdev)
{
	int i, max_clks;
	struct clk *clk;
	max_clks = of_count_phandle_with_args(pdev->dev.of_node,
					      "clocks", "#clock-cells");

	for (i = 0; i < max_clks; i++)	{
		clk = of_clk_get(pdev->dev.of_node, i);

		if (IS_ERR(clk))
			break;

		clk_prepare_enable(clk);
	}
	return 0;
}

static const struct of_device_id rpm_clk_ho_of_match[] = {
	{ .compatible = "qcom,apq8064-rpmcc-handoff" },
	{ },
};

static struct platform_driver rpm_clk_ho_driver = {
	.driver = {
		.name = "qcom-rpm-clk-handoff",
		.of_match_table = rpm_clk_ho_of_match,
	},
	.probe = rpm_clk_ho_probe,
};

static int __init rpm_clk_init(void)
{
	platform_driver_register(&rpm_clk_driver);
	return platform_driver_register(&rpm_clk_ho_driver);
}
subsys_initcall(rpm_clk_init);

static void __exit rpm_clk_exit(void)
{
	platform_driver_unregister(&rpm_clk_ho_driver);
	platform_driver_unregister(&rpm_clk_driver);
}
module_exit(rpm_clk_exit)

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org>");
MODULE_DESCRIPTION("Driver for the RPM clocks");
