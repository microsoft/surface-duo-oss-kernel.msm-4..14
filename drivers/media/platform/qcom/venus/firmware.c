/*
 * Copyright (C) 2017 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/slab.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/mdt_loader.h>

#define VENUS_FIRMWARE_NAME		"venus.mdt"
#define VENUS_PAS_ID			9
#define VENUS_FW_MEM_SIZE		SZ_8M

struct firmware_mem {
	struct device dev;
	void *mem_va;
	phys_addr_t mem_phys;
	size_t mem_size;
};

static struct firmware_mem fw;

static void device_release_dummy(struct device *dev)
{
}

static int firmware_alloc_mem(struct device *parent, struct firmware_mem *fw)
{
	struct device_node *np;
	struct device *dev = &fw->dev;
	int ret;

	np = of_get_child_by_name(parent->of_node, "video-firmware");
	if (!np)
		return -ENODEV;

	memset(fw, 0, sizeof(*fw));

	dev->of_node = np;
	dev->parent = parent;
	dev->release = device_release_dummy;

	ret = dev_set_name(dev, "venus-fw");
	if (ret)
		return ret;

	ret = device_register(dev);
	if (ret < 0)
		return ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		goto err_unreg_device;

	fw->mem_size = VENUS_FW_MEM_SIZE;

	fw->mem_va = dma_alloc_coherent(dev, fw->mem_size, &fw->mem_phys,
					GFP_KERNEL);
	if (!fw->mem_va) {
		ret = -ENOMEM;
		goto err_mem_device_release;
	}

	return 0;

err_mem_device_release:
	of_reserved_mem_device_release(dev);
err_unreg_device:
	device_unregister(dev);
	return ret;
}

static void firmware_free_mem(struct firmware_mem *fw)
{
	dma_free_coherent(&fw->dev, fw->mem_size, fw->mem_va, fw->mem_phys);
	of_reserved_mem_device_release(&fw->dev);
	device_unregister(&fw->dev);
	memset(fw, 0, sizeof(*fw));
}

static int firmware_load(struct firmware_mem *fw)
{
	struct device *dev = &fw->dev;
	const struct firmware *mdt;
	ssize_t fw_size;
	int ret;

	ret = request_firmware(&mdt, VENUS_FIRMWARE_NAME, dev);
	if (ret < 0)
		return ret;

	fw_size = qcom_mdt_get_size(mdt);
	if (fw_size < 0) {
		ret = fw_size;
		goto err_release_fw;
	} else if (fw_size > VENUS_FW_MEM_SIZE) {
		ret = -ENOMEM;
		goto err_release_fw;
	}

	ret = qcom_mdt_load(&fw->dev, mdt, VENUS_FIRMWARE_NAME, VENUS_PAS_ID,
			    fw->mem_va, fw->mem_phys, fw->mem_size);

err_release_fw:
	release_firmware(mdt);

	return ret;
}

int venus_boot(struct device *parent)
{
	int ret;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	ret = firmware_alloc_mem(parent, &fw);
	if (ret)
		return ret;

	ret = firmware_load(&fw);
	if (ret) {
		firmware_free_mem(&fw);
		return ret;
	}

	return qcom_scm_pas_auth_and_reset(VENUS_PAS_ID);
}

int venus_shutdown(void)
{
	int ret;

	ret = qcom_scm_pas_shutdown(VENUS_PAS_ID);
	firmware_free_mem(&fw);
	return ret;
}
