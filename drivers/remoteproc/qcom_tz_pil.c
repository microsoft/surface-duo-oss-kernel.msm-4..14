/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/qcom/smem.h>

#include "remoteproc_internal.h"

#include "../../arch/arm/mach-qcom/scm.h"

extern int scm_is_call_available(u32 svc_id, u32 cmd_id);

#define PAS_INIT_IMAGE_CMD      1
#define PAS_MEM_SETUP_CMD       2
#define PAS_AUTH_AND_RESET_CMD  5
#define PAS_SHUTDOWN_CMD        6
#define PAS_IS_SUPPORTED_CMD    7

struct qproc {
	struct device *dev;
	struct rproc *rproc;

	int pas_id;

	int wdog_irq;
	int fatal_irq;
	int ready_irq;
	int handover_irq;
	int stop_ack_irq;

	struct gpio_desc *stop_gpio;

	const char *name;
	struct regulator *pll;

	unsigned proxy_clk_count;
	struct clk *scm_core_clk;
	struct clk *scm_iface_clk;
	struct clk *scm_bus_clk;

	struct clk **proxy_clks;

	struct completion start_done;
	struct completion stop_done;

	struct qcom_smem_item crash_reason;
};

static int pas_supported(int id)
{
	u32 periph = id;
	u32 ret_val;
	int ret;

	ret = scm_is_call_available(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD);
	if (ret <= 0)
		return 0;

        ret = scm_call(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD,
		       &periph, sizeof(periph),
		       &ret_val, sizeof(ret_val));

	return ret ? : ret_val;
}

static int pas_init_image(int id, const char *metadata, size_t size)
{
	dma_addr_t mdata_phys;
	void *mdata_buf;
	u32 scm_ret;
	int ret;
	struct pas_init_image_req {
		u32 proc;
		u32 image_addr;
	} request;

	mdata_buf = dma_alloc_coherent(NULL, size, &mdata_phys, GFP_KERNEL);
	if (!mdata_buf) {
		pr_err("Allocation for metadata failed.\n");
		return -ENOMEM;
	}

	memcpy(mdata_buf, metadata, size);
	outer_flush_range(request.image_addr, request.image_addr + size);

	request.proc = id;
	request.image_addr = mdata_phys;

	ret = scm_call(SCM_SVC_PIL, PAS_INIT_IMAGE_CMD,
		       &request, sizeof(request),
		       &scm_ret, sizeof(scm_ret));

	dma_free_coherent(NULL, size, mdata_buf, mdata_phys);

	return ret ? : scm_ret;
}

static int pas_mem_setup(int id, phys_addr_t start_addr, phys_addr_t size)
{
	u32 scm_ret;
	int ret;
	struct pas_init_image_req {
		u32 proc;
		u32 start_addr;
		u32 len;
	} request;

	request.proc = id;
	request.start_addr = start_addr;
	request.len = size;

	ret = scm_call(SCM_SVC_PIL, PAS_MEM_SETUP_CMD,
		       &request, sizeof(request),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

static int pas_auth_and_reset(int id)
{
	u32 proc = id;
	u32 scm_ret;
	int ret;

	ret = scm_call(SCM_SVC_PIL, PAS_AUTH_AND_RESET_CMD,
		       &proc, sizeof(proc),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

static int pas_shutdown(int id)
{
	u32 scm_ret;
	u32 proc = id;
	int ret;

	ret = scm_call(SCM_SVC_PIL, PAS_SHUTDOWN_CMD,
		       &proc, sizeof(proc),
		       &scm_ret, sizeof(scm_ret));

	return ret ? : scm_ret;
}

static int qproc_scm_clk_enable(struct qproc *qproc)
{
	int ret;

	ret = clk_prepare_enable(qproc->scm_core_clk);
	if (ret)
		goto bail;
	ret = clk_prepare_enable(qproc->scm_iface_clk);
	if (ret)
		goto disable_core;
	ret = clk_prepare_enable(qproc->scm_bus_clk);
	if (ret)
		goto disable_iface;

	return 0;

disable_iface:
	clk_disable_unprepare(qproc->scm_iface_clk);
disable_core:
	clk_disable_unprepare(qproc->scm_core_clk);
bail:
	return ret;
}

static void qproc_scm_clk_disable(struct qproc *qproc)
{
	clk_disable_unprepare(qproc->scm_core_clk);
	clk_disable_unprepare(qproc->scm_iface_clk);
	clk_disable_unprepare(qproc->scm_bus_clk);
}

/**
 * struct pil_mdt - Representation of <name>.mdt file in memory
 * @hdr: ELF32 header
 * @phdr: ELF32 program headers
 */
struct mdt_hdr {
	struct elf32_hdr hdr;
	struct elf32_phdr phdr[];
};

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int segment_is_loadable(const struct elf32_phdr *p)
{
	return (p->p_type == PT_LOAD) &&
	       !segment_is_hash(p->p_flags) &&
	       p->p_memsz;
}

static bool segment_is_relocatable(const struct elf32_phdr *p)
{
	return !!(p->p_flags & BIT(27));
}

/**
 * rproc_mdt_sanity_check() - sanity check mdt firmware header
 * @rproc: the remote processor handle
 * @fw: the mdt header firmware image
 */
static int qproc_sanity_check(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct elf32_hdr *ehdr;
	struct mdt_hdr *mdt;

	if (!fw) {
		dev_err(&rproc->dev, "failed to load %s\n", rproc->name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(&rproc->dev, "image is too small\n");
		return -EINVAL;
	}

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(&rproc->dev, "image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(&rproc->dev, "no loadable segments\n");
		return -EINVAL;
	}

	if (sizeof(struct elf32_phdr) * ehdr->e_phnum +
	    sizeof(struct elf32_hdr) > fw->size) {
		dev_err(&rproc->dev, "firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}

static struct resource_table * qproc_find_rsc_table(struct rproc *rproc,
						    const struct firmware *fw,
						    int *tablesz)
{
	static struct resource_table table = { .ver = 1, };

	*tablesz = sizeof(table);
	return &table;
}

static int qproc_load_segment(struct rproc *rproc, const char *fw_name, const struct elf32_phdr *phdr)
{
	const struct firmware *fw;
	void *ptr;
	int ret = 0;

	ptr = ioremap(phdr->p_paddr, phdr->p_memsz);
	if (!ptr) {
		dev_err(&rproc->dev, "failed to ioremap segment area (0x%x+0x%x)\n", phdr->p_paddr, phdr->p_memsz);
		return -EBUSY;
	}

	if (phdr->p_filesz) {
		ret = request_firmware(&fw, fw_name, &rproc->dev);
		if (ret) {
			dev_err(&rproc->dev, "failed to load %s\n", fw_name);
			goto out;
		}

		memcpy(ptr, fw->data, fw->size);

		release_firmware(fw);
	}

	if (phdr->p_memsz > phdr->p_filesz)
		memset(ptr + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);

out:
	iounmap(ptr);
	return ret;
}

static int qproc_load(struct rproc *rproc, const struct firmware *fw)
{
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	const struct mdt_hdr *mdt;
	phys_addr_t min_addr = (phys_addr_t)ULLONG_MAX;
	phys_addr_t max_addr = 0;
	struct qproc *qproc = rproc->priv;
	char *fw_name;
	int ret;
	int i;

	ret = qproc_scm_clk_enable(qproc);
	if (ret)
		return ret;

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		if (segment_is_relocatable(phdr)) {
			dev_err(&rproc->dev, "relocation unsupported\n");
			return -EINVAL;
		}

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = ALIGN(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	ret = pas_init_image(qproc->pas_id, fw->data, fw->size);
	if (ret) {
		dev_err(qproc->dev, "Invalid firmware metadata\n");
		return -EINVAL;
	}

	dev_dbg(qproc->dev, "pas_mem_setup(0x%x, 0x%x)\n", min_addr, max_addr - min_addr);

	ret = pas_mem_setup(qproc->pas_id, min_addr, max_addr - min_addr);
	if (ret) {
		dev_err(qproc->dev, "unable to setup memory for image\n");
		return -EINVAL;
	}

	fw_name = kzalloc(strlen(qproc->name) + 5, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		sprintf(fw_name, "%s.b%02d", qproc->name, i);
		ret = qproc_load_segment(rproc, fw_name, phdr);
		if (ret)
			break;
	}

	kfree(fw_name);

	qproc_scm_clk_disable(qproc);

	return 0;
}

const struct rproc_fw_ops qproc_fw_ops = {
	.find_rsc_table = qproc_find_rsc_table,
	.load = qproc_load,
	.sanity_check = qproc_sanity_check,
};

static int qproc_start(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;
	int ret;

	ret = regulator_enable(qproc->pll);
	if (ret) {
		dev_err(qproc->dev, "failed to enable pll supply\n");
		return ret;
	}

	ret = qproc_scm_clk_enable(qproc);
	if (ret)
		goto disable_regulator;

	ret = pas_auth_and_reset(qproc->pas_id);
	if (ret) {
		dev_err(qproc->dev,
				"failed to authenticate image and release reset\n");
		goto unroll_clocks;
	}

	ret = wait_for_completion_timeout(&qproc->start_done, msecs_to_jiffies(10000));
	if (ret == 0) {
		dev_err(qproc->dev, "start timed out\n");

		pas_shutdown(qproc->pas_id);
		goto unroll_clocks;
	}

	return 0;

unroll_clocks:
	qproc_scm_clk_disable(qproc);

disable_regulator:
	regulator_disable(qproc->pll);

	return ret;
}

static int qproc_stop(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;
	int ret;

	gpiod_set_value(qproc->stop_gpio, 1);

	ret = wait_for_completion_timeout(&qproc->stop_done, msecs_to_jiffies(1000));
	if (ret == 0) {
		dev_err(qproc->dev, "timed out on wait\n");
		return ret;
	}

	gpiod_set_value(qproc->stop_gpio, 0);

	ret = pas_shutdown(qproc->pas_id);
	if (ret)
		dev_err(qproc->dev, "failed to shutdown: %d\n", ret);
	return ret;
}

static const struct rproc_ops qproc_ops = {
	.start = qproc_start,
	.stop = qproc_stop,
};

static irqreturn_t qproc_wdog_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	rproc_report_crash(qproc->rproc, RPROC_WATCHDOG);
	return IRQ_HANDLED;
}

static irqreturn_t qproc_fatal_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;
	size_t len;
	char *msg;
	int ret;

	ret = qcom_smem_get(&qproc->crash_reason, (void**)&msg, &len);
	if (!ret && len > 0 && msg[0])
		dev_err(qproc->dev, "fatal error received: %s\n", msg);

	rproc_report_crash(qproc->rproc, RPROC_FATAL_ERROR);

	if (!ret) {
		msg[0] = '\0';
#if 0
		qcom_smem_put(qproc->smem, msg);
#endif
	}

	return IRQ_HANDLED;
}

static irqreturn_t qproc_ready_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	complete(&qproc->start_done);

	return IRQ_HANDLED;
}

static irqreturn_t qproc_handover_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	qproc_scm_clk_disable(qproc);
	regulator_disable(qproc->pll);
	return IRQ_HANDLED;
}

static irqreturn_t qproc_stop_ack_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	complete(&qproc->stop_done);
	return IRQ_HANDLED;
}

static ssize_t qproc_boot_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct qproc *qproc = dev_get_drvdata(dev);
	int ret;

	ret = rproc_boot(qproc->rproc);
	return ret ? : size;
}

static ssize_t qproc_shutdown_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct qproc *qproc = dev_get_drvdata(dev);

	rproc_shutdown(qproc->rproc);
	return size;
}

static const struct device_attribute qproc_attrs[] = {
	__ATTR(boot, S_IWUSR, 0, qproc_boot_store),
	__ATTR(shutdown, S_IWUSR, 0, qproc_shutdown_store),
};

static int qproc_init_pas(struct qproc *qproc)
{
	char *key;
	int ret;

	key = "qcom,pas-id";
	ret = of_property_read_u32(qproc->dev->of_node, key, &qproc->pas_id);
	if (ret) {
		dev_err(qproc->dev, "Missing or incorrect %s\n", key);
		return -EINVAL;
	}

	if (!pas_supported(qproc->pas_id)) {
		dev_err(qproc->dev, "PAS is not available for %d\n", qproc->pas_id);
		return -EIO;
	}

	return 0;
}

static int qproc_init_clocks(struct qproc *qproc)
{
	long rate;
	int ret;

	qproc->scm_core_clk = devm_clk_get(qproc->dev, "scm_core_clk");
	if (IS_ERR(qproc->scm_core_clk)) {
		if (PTR_ERR(qproc->scm_core_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_core_clk\n");
		return PTR_ERR(qproc->scm_core_clk);
	}

	qproc->scm_iface_clk = devm_clk_get(qproc->dev, "scm_iface_clk");
	if (IS_ERR(qproc->scm_iface_clk)) {
		if (PTR_ERR(qproc->scm_iface_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_iface_clk\n");
		return PTR_ERR(qproc->scm_iface_clk);
	}

	qproc->scm_bus_clk = devm_clk_get(qproc->dev, "scm_bus_clk");
	if (IS_ERR(qproc->scm_bus_clk)) {
		if (PTR_ERR(qproc->scm_bus_clk) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to acquire scm_bus_clk\n");
		return PTR_ERR(qproc->scm_bus_clk);
	}

	rate = clk_round_rate(qproc->scm_core_clk, 50000000);
	ret = clk_set_rate(qproc->scm_core_clk, rate);
	if (ret) {
		dev_err(qproc->dev, "failed to set rate of scm_core_clk\n");
		return ret;
	}

	return 0;
}

static int qproc_init_regulators(struct qproc *qproc)
{
	int ret;
	u32 uA;
	u32 uV;

	qproc->pll = devm_regulator_get(qproc->dev, "qcom,pll");
	if (IS_ERR(qproc->pll)) {
		if (PTR_ERR(qproc->pll) != -EPROBE_DEFER)
			dev_err(qproc->dev, "failed to aquire regulator\n");
		return PTR_ERR(qproc->pll);
	}

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,pll-uV", &uV);
	if (ret)
		dev_warn(qproc->dev, "failed to read qcom,pll_uV, skipping\n");
	else
		regulator_set_voltage(qproc->pll, uV, uV);

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,pll-uA", &uA);
	if (ret)
		dev_warn(qproc->dev, "failed to read qcom,pll_uA, skipping\n");
	else
		regulator_set_optimum_mode(qproc->pll, uA);

	return 0;
}

static int qproc_request_irq(struct qproc *qproc, struct platform_device *pdev, const char *name, irq_handler_t thread_fn)
{
	int ret;

	ret = platform_get_irq_byname(pdev, name);
	if (ret < 0) {
		dev_err(&pdev->dev, "no %s IRQ defined\n", name);
		return ret;
	}

	ret = devm_request_threaded_irq(&pdev->dev, ret,
					NULL, thread_fn,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"qproc", qproc);
	if (ret)
		dev_err(&pdev->dev, "request %s IRQ failed\n", name);
	return ret;
}

static int qproc_probe(struct platform_device *pdev)
{
	struct qproc *qproc;
	struct rproc *rproc;
	char *fw_name;
	const char *name;
	const char *key;
	int ret;
	int i;

	key = "qcom,firmware-name";
	ret = of_property_read_string(pdev->dev.of_node, key, &name);
	if (ret) {
		dev_err(&pdev->dev, "missing or incorrect %s\n", key);
		return -EINVAL;
	}

	fw_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s.mdt", name);
	if (!fw_name)
		return -ENOMEM;

	rproc = rproc_alloc(&pdev->dev, pdev->name, &qproc_ops,
			    fw_name, sizeof(*qproc));
	if (!rproc)
		return -ENOMEM;

	rproc->fw_ops = &qproc_fw_ops;

	qproc = (struct qproc *)rproc->priv;
	qproc->dev = &pdev->dev;
	qproc->rproc = rproc;
	qproc->name = name;
	platform_set_drvdata(pdev, qproc);

	init_completion(&qproc->start_done);
	init_completion(&qproc->stop_done);

	ret = of_parse_qcom_smem_item(pdev->dev.of_node,
				      "qcom,crash-reason", 0,
				      &qproc->crash_reason);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to acquire smem handle\n");
		return ret;
	}

	ret = qproc_init_pas(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_clocks(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_regulators(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_request_irq(qproc, pdev, "wdog", qproc_wdog_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->wdog_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "fatal", qproc_fatal_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->fatal_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "ready", qproc_ready_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->ready_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "handover", qproc_handover_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->handover_irq = ret;

	ret = qproc_request_irq(qproc, pdev, "stop-ack", qproc_stop_ack_interrupt);
	if (ret < 0)
		goto free_rproc;
	qproc->stop_ack_irq = ret;

	qproc->stop_gpio = devm_gpiod_get(&pdev->dev, "qcom,stop", GPIOD_OUT_LOW);
	if (IS_ERR(qproc->stop_gpio)) {
		dev_err(&pdev->dev, "failed to acquire stop gpio\n");
		return PTR_ERR(qproc->stop_gpio);
	}

	for (i = 0; i < ARRAY_SIZE(qproc_attrs); i++) {
		ret = device_create_file(&pdev->dev, &qproc_attrs[i]);
		if (ret) {
			dev_err(&pdev->dev, "unable to create sysfs file\n");
			goto remove_device_files;
		}
	}

	ret = rproc_add(rproc);
	if (ret)
		goto remove_device_files;

	return 0;

remove_device_files:
	for (i--; i >= 0; i--)
		device_remove_file(&pdev->dev, &qproc_attrs[i]);

free_rproc:
	rproc_put(rproc);

	return ret;
}

static int qproc_remove(struct platform_device *pdev)
{
	struct qproc *qproc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(qproc_attrs); i++)
		device_remove_file(&pdev->dev, &qproc_attrs[i]);

	rproc_put(qproc->rproc);

	return 0;
}

static const struct of_device_id qproc_of_match[] = {
	{ .compatible = "qcom,tz-pil", },
	{ },
};

static struct platform_driver qproc_driver = {
	.probe = qproc_probe,
	.remove = qproc_remove,
	.driver = {
		.name = "qcom-tz-pil",
		.owner = THIS_MODULE,
		.of_match_table = qproc_of_match,
	},
};

module_platform_driver(qproc_driver);
