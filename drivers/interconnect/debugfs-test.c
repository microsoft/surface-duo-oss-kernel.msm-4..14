// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017, Linaro Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>

struct platform_device *icc_pdev;
static struct icc_path *path;
static u32 src_port;
static u32 dst_port;

static u32 avg_bw;
static u32 peak_bw;

static ssize_t get_write_op(struct file *file, char const __user *buf,
			    size_t count, loff_t *ppos)
{
	path = icc_get(&icc_pdev->dev, src_port, dst_port);
	if (IS_ERR(path))
		pr_err("icc_get() error (%ld)\n", PTR_ERR(path));

	return 1;
}

static const struct file_operations get_fops = {
	.owner = THIS_MODULE,
	.write = get_write_op
};

static ssize_t commit_write_op(struct file *file, char const __user *buf,
			       size_t count, loff_t *ppos)
{
	int ret;

	ret = icc_set_bw(path, avg_bw, peak_bw);
	if (ret)
		pr_err("icc_set_bw() error (%d)\n", ret);

	return 1;
}

static const struct file_operations commit_fops = {
	.owner = THIS_MODULE,
	.write = commit_write_op
};

static int __init icc_test_init(void)
{
	struct dentry *debugfs_dir;

	icc_pdev = platform_device_alloc("icc-test", PLATFORM_DEVID_AUTO);
	platform_device_add(icc_pdev);

	debugfs_dir = debugfs_create_dir("interconnect-test", NULL);
	if (!debugfs_dir)
		pr_err("interconnect: error creating debugfs directory\n");

	debugfs_create_u32("src_port", 0600, debugfs_dir, &src_port);
	debugfs_create_u32("dst_port", 0600, debugfs_dir, &dst_port);
	debugfs_create_file("get", 0200, debugfs_dir, NULL, &get_fops);
	debugfs_create_u32("avg_bw", 0600, debugfs_dir, &avg_bw);
	debugfs_create_u32("peak_bw", 0600, debugfs_dir, &peak_bw);
	debugfs_create_file("commit", 0200, debugfs_dir, NULL, &commit_fops);

	return 0;
}
late_initcall(icc_test_init);
MODULE_LICENSE("GPL v2");

#endif
