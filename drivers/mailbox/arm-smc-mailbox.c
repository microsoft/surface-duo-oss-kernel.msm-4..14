// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016,2017 ARM Ltd.
 * Copyright 2019 NXP
 */

#include <linux/arm-smccc.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/arm-smccc-mbox.h>
#include <linux/module.h>
#include <linux/platform_device.h>

typedef unsigned long (smc_mbox_fn)(unsigned int, unsigned long,
				    unsigned long, unsigned long,
				    unsigned long, unsigned long,
				    unsigned long);

struct arm_smc_chan_data {
	unsigned int function_id;
	smc_mbox_fn *invoke_smc_mbox_fn;
};

static int arm_smc_send_data(struct mbox_chan *link, void *data)
{
	struct arm_smc_chan_data *chan_data = link->con_priv;
	struct arm_smccc_mbox_cmd *cmd = data;
	unsigned long ret;

	if (ARM_SMCCC_IS_64(chan_data->function_id)) {
		ret = chan_data->invoke_smc_mbox_fn(chan_data->function_id,
						    cmd->args_smccc64[0],
						    cmd->args_smccc64[1],
						    cmd->args_smccc64[2],
						    cmd->args_smccc64[3],
						    cmd->args_smccc64[4],
						    cmd->args_smccc64[5]);
	} else {
		ret = chan_data->invoke_smc_mbox_fn(chan_data->function_id,
						    cmd->args_smccc32[0],
						    cmd->args_smccc32[1],
						    cmd->args_smccc32[2],
						    cmd->args_smccc32[3],
						    cmd->args_smccc32[4],
						    cmd->args_smccc32[5]);
	}

	mbox_chan_received_data(link, (void *)ret);

	return 0;
}

static unsigned long __invoke_fn_hvc(unsigned int function_id,
				     unsigned long arg0, unsigned long arg1,
				     unsigned long arg2, unsigned long arg3,
				     unsigned long arg4, unsigned long arg5)
{
	struct arm_smccc_res res;

	arm_smccc_hvc(function_id, arg0, arg1, arg2, arg3, arg4,
		      arg5, 0, &res);
	return res.a0;
}

static unsigned long __invoke_fn_smc(unsigned int function_id,
				     unsigned long arg0, unsigned long arg1,
				     unsigned long arg2, unsigned long arg3,
				     unsigned long arg4, unsigned long arg5)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function_id, arg0, arg1, arg2, arg3, arg4,
		      arg5, 0, &res);
	return res.a0;
}

static const struct mbox_chan_ops arm_smc_mbox_chan_ops = {
	.send_data	= arm_smc_send_data,
};

static struct mbox_chan *
arm_smc_mbox_of_xlate(struct mbox_controller *mbox,
		      const struct of_phandle_args *sp)
{
	return mbox->chans;
}

static int arm_smc_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mbox_controller *mbox;
	struct arm_smc_chan_data *chan_data;
	int ret;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->of_xlate = arm_smc_mbox_of_xlate;
	mbox->num_chans = 1;
	mbox->chans = devm_kzalloc(dev, sizeof(*mbox->chans), GFP_KERNEL);
	if (!mbox->chans)
		return -ENOMEM;

	chan_data = devm_kzalloc(dev, sizeof(*chan_data), GFP_KERNEL);
	if (!chan_data)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "arm,func-id",
				   &chan_data->function_id);
	if (ret)
		return ret;

	if (of_device_is_compatible(dev->of_node, "arm,smc-mbox"))
		chan_data->invoke_smc_mbox_fn = __invoke_fn_smc;
	else
		chan_data->invoke_smc_mbox_fn = __invoke_fn_hvc;


	mbox->chans->con_priv = chan_data;

	mbox->txdone_poll = false;
	mbox->txdone_irq = false;
	mbox->ops = &arm_smc_mbox_chan_ops;
	mbox->dev = dev;

	platform_set_drvdata(pdev, mbox);

	ret = devm_mbox_controller_register(dev, mbox);
	if (ret)
		return ret;

	dev_info(dev, "ARM SMC mailbox enabled.\n");

	return ret;
}

static int arm_smc_mbox_remove(struct platform_device *pdev)
{
	struct mbox_controller *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(mbox);
	return 0;
}

static const struct of_device_id arm_smc_mbox_of_match[] = {
	{ .compatible = "arm,smc-mbox", },
	{ .compatible = "arm,hvc-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, arm_smc_mbox_of_match);

static struct platform_driver arm_smc_mbox_driver = {
	.driver = {
		.name = "arm-smc-mbox",
		.of_match_table = arm_smc_mbox_of_match,
	},
	.probe		= arm_smc_mbox_probe,
	.remove		= arm_smc_mbox_remove,
};
module_platform_driver(arm_smc_mbox_driver);

MODULE_AUTHOR("Peng Fan <peng.fan@nxp.com>");
MODULE_DESCRIPTION("Generic ARM smc mailbox driver");
MODULE_LICENSE("GPL v2");
