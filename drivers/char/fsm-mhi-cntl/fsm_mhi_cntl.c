/* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/ipc_logging.h>
#include <linux/types.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/fsm_mhi_cntl.h>

#define MAX_IPC_LOG_NAME_LEN 25
#define FSM_MHI_PCIE_LOG_PAGES 20

#define FSM_MHI_CNTL_DBG(__msg, ...) \
do { \
	if (fsm_mhi_cntl_ipc_log) \
		ipc_log_string(fsm_mhi_cntl_ipc_log, \
			"[%s]: "__msg, __func__, ##__VA_ARGS__); \
} while (0)

/* globals */
void	*fsm_mhi_cntl_ipc_log;
void	*fsm_mhi_cntl;

/* debugfs related params */
static struct dentry *dent_pcie_mhi_cntl;
static struct dentry *dfile_power;

enum fsm_mhi_cntl_power_option {
	PCIE_MHI_POWER_OFF,
	PCIE_MHI_POWER_ON,
	PCIE_MHI_MAX_POWER_OPTION
};

static const char * const
	pcie_mhi_power_option_desc[PCIE_MHI_MAX_POWER_OPTION] = {
		"MHI_POWER_OFF",
		"MHI_POWER_ON",
};

static void fsm_mhi_cntl_sel_power_case(void *ctrl, u32 powercase)
{
	switch (powercase) {
	case PCIE_MHI_POWER_OFF:
		FSM_MHI_CNTL_DBG("MHI Processing turn off\n");
		mhi_arch_pcie_ops_power_off(ctrl);
		break;
	case PCIE_MHI_POWER_ON:
		FSM_MHI_CNTL_DBG("MHI Processing turn on\n");
		mhi_arch_pcie_ops_power_on(ctrl);
		break;
	default:
		FSM_MHI_CNTL_DBG("Invalid powercase: %d.\n", powercase);
		break;
	}
}

static int fsm_mhi_cntl_debugfs_parse_input(const char __user *buf,
					size_t count, unsigned int *data)
{
	unsigned long ret;
	char *str, *str_temp;

	str = kmalloc(count + 1, GFP_KERNEL);
	if (!str)
		return -ENOMEM;

	ret = copy_from_user(str, buf, count);
	if (ret) {
		kfree(str);
		FSM_MHI_CNTL_DBG("Unable to copy string from user");
		return -EFAULT;
	}

	str[count] = 0;
	str_temp = str;

	ret = get_option(&str_temp, data);
	kfree(str);
	if (ret != 1) {
		FSM_MHI_CNTL_DBG("No valid option found");
		return -EINVAL;
	}
	return 0;
}

static int fsm_mhi_cntl_power_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < PCIE_MHI_MAX_POWER_OPTION; i++)
		seq_printf(m, "\t%d:\t %s\n", i, pcie_mhi_power_option_desc[i]);

	return 0;
}

static int fsm_mhi_cntl_power_open(
	struct inode *inode,
	struct file *file)
{
	return single_open(file, fsm_mhi_cntl_power_show, NULL);
}

static ssize_t fsm_mhi_cntl_power_select(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	int i, ret;
	unsigned int powercase = 0;

	ret = fsm_mhi_cntl_debugfs_parse_input(buf, count, &powercase);
	if (ret) {
		FSM_MHI_CNTL_DBG("Unable to Parse Input");
		return ret;
	}

	FSM_MHI_CNTL_DBG("POWER: %d\n", powercase);

	fsm_mhi_cntl_sel_power_case(fsm_mhi_cntl, powercase);

	return count;
}

static const struct file_operations fsm_mhi_cntl_power_ops = {
	.open = fsm_mhi_cntl_power_open,
	.release = single_release,
	.read = seq_read,
	.write = fsm_mhi_cntl_power_select,
};

static int fsm_pcie_mhi_cntl_debugfs_init(void)
{
	dent_pcie_mhi_cntl = debugfs_create_dir("pci-mhi-cntl", NULL);
	if (IS_ERR(dent_pcie_mhi_cntl)) {
		FSM_MHI_CNTL_DBG("PCIe: fail to create the folder for debug_fs.\n");
		return -EINVAL;
	}

	dfile_power = debugfs_create_file("power", 0664,
					dent_pcie_mhi_cntl, NULL,
					&fsm_mhi_cntl_power_ops);

	if (!dfile_power || IS_ERR(dfile_power)) {
			FSM_MHI_CNTL_DBG(
				"PCIe: fail to create the file for debug_fs case.\n");
			goto power_error;
	}
	return 0;

power_error:
	debugfs_remove(dfile_power);
	return -EINVAL;
}

void mhi_pci_cntl_register(void *priv) {
	fsm_mhi_cntl = priv;
}
EXPORT_SYMBOL(mhi_pci_cntl_register);

int mhi_pcie_cntl_probe(struct platform_device *pdev)
{
	int len;
	const char *name;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	name = of_get_property(np, "compatible", &len);

	FSM_MHI_CNTL_DBG("Compatible : %s\n : Device Successfully Probed", name);
	return 0;
}

int mhi_pcie_cntl_remove(struct platform_device *pdev) {
	FSM_MHI_CNTL_DBG("MHI PCIe Control Removed");
	return 0;
}

static const struct of_device_id fsm_mhi_cntl_of_table[] = {
	{ .compatible = "qcom,mhi_pcie_cntl" },
	{},
};

MODULE_DEVICE_TABLE(of, mymatch);

static struct platform_driver fsm_mhi_cntl_driver = {
	.probe		= mhi_pcie_cntl_probe,
	.remove		= mhi_pcie_cntl_remove,
	.driver = {
		.name	= KBUILD_MODNAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(fsm_mhi_cntl_of_table),
	},
};

static int __init mhi_pcie_cntl_register(void)
{
	char ipc_log_name[MAX_IPC_LOG_NAME_LEN];

	pr_info("MHI PCIe Control Driver Register");

	/* Initialize IPC Logging */
	snprintf(ipc_log_name, MAX_IPC_LOG_NAME_LEN, "pcie_mhi_cntl-short");
	fsm_mhi_cntl_ipc_log = ipc_log_context_create(
							FSM_MHI_PCIE_LOG_PAGES,
							ipc_log_name, 0);
	if (fsm_mhi_cntl_ipc_log == NULL)
		pr_err("%s: unable to create IPC log context for %s\n",
				__func__, ipc_log_name);
	else
		FSM_MHI_CNTL_DBG("IPC logging: %s is enable", ipc_log_name);

	/* Initialize debugfs */
	fsm_pcie_mhi_cntl_debugfs_init();

	return platform_driver_register(&fsm_mhi_cntl_driver);
}
module_init(mhi_pcie_cntl_register);

static void __exit mhi_pcie_cntl_unregister(void)
{
	pr_info("MHI PCIe Control Driver Unregister");
	platform_driver_unregister(&fsm_mhi_cntl_driver);
}
module_exit(mhi_pcie_cntl_unregister);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MHI_PCIE_CNTL");

