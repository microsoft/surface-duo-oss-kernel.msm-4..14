/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2016 Linaro Ltd.
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
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"
#include "qcom_mdt_loader.h"

#include <linux/qcom_scm.h>

#define MBA_FIRMWARE_NAME		"mba.b00"
#define MPSS_FIRMWARE_NAME		"modem.mdt"

#define MPSS_CRASH_REASON_SMEM		421

#define VDD_MSS_UV_MIN			1000000
#define VDD_MSS_UV_MAX			1150000
#define VDD_MSS_UA			100000

/* AXI Halting Registers */
#define MSS_Q6_HALT_BASE		0x180
#define MSS_MODEM_HALT_BASE		0x200
#define MSS_NC_HALT_BASE		0x280

/* RMB Status Register Values */
#define RMB_PBL_SUCCESS			0x1

#define RMB_MBA_XPU_UNLOCKED		0x1
#define RMB_MBA_XPU_UNLOCKED_SCRIBBLED	0x2
#define RMB_MBA_META_DATA_AUTH_SUCCESS	0x3
#define RMB_MBA_AUTH_COMPLETE		0x4

/* PBL/MBA interface registers */
#define RMB_MBA_IMAGE_REG		0x00
#define RMB_PBL_STATUS_REG		0x04
#define RMB_MBA_COMMAND_REG		0x08
#define RMB_MBA_STATUS_REG		0x0C
#define RMB_PMI_META_DATA_REG		0x10
#define RMB_PMI_CODE_START_REG		0x14
#define RMB_PMI_CODE_LENGTH_REG		0x18

#define RMB_CMD_META_DATA_READY		0x1
#define RMB_CMD_LOAD_READY		0x2

/* QDSP6SS Register Offsets */
#define QDSP6SS_RESET_REG		0x014
#define QDSP6SS_GFMUX_CTL_REG		0x020
#define QDSP6SS_PWR_CTL_REG		0x030

/* AXI Halt Register Offsets */
#define AXI_HALTREQ_REG			0x0
#define AXI_HALTACK_REG			0x4
#define AXI_IDLE_REG			0x8

#define HALT_ACK_TIMEOUT_MS		100

/* QDSP6SS_RESET */
#define Q6SS_STOP_CORE			BIT(0)
#define Q6SS_CORE_ARES			BIT(1)
#define Q6SS_BUS_ARES_ENABLE		BIT(2)

/* QDSP6SS_GFMUX_CTL */
#define Q6SS_CLK_ENABLE			BIT(1)

/* QDSP6SS_PWR_CTL */
#define Q6SS_L2DATA_SLP_NRET_N_0	BIT(0)
#define Q6SS_L2DATA_SLP_NRET_N_1	BIT(1)
#define Q6SS_L2DATA_SLP_NRET_N_2	BIT(2)
#define Q6SS_L2TAG_SLP_NRET_N		BIT(16)
#define Q6SS_ETB_SLP_NRET_N		BIT(17)
#define Q6SS_L2DATA_STBY_N		BIT(18)
#define Q6SS_SLP_RET_N			BIT(19)
#define Q6SS_CLAMP_IO			BIT(20)
#define QDSS_BHS_ON			BIT(21)
#define QDSS_LDO_BYP			BIT(22)

struct q6v5 {
	struct device *dev;
	struct rproc *rproc;

	void __iomem *reg_base;
	void __iomem *halt_base;
	void __iomem *rmb_base;

	struct reset_control *mss_restart;

	struct qcom_smem_state *state;
	unsigned stop_bit;

	struct regulator *vdd;

	struct clk *ahb_clk;
	struct clk *axi_clk;
	struct clk *rom_clk;

	struct completion start_done;

	phys_addr_t mba_phys;
	void *mba_region;
	size_t mba_size;

	phys_addr_t mpss_phys;
	void *mpss_region;
	size_t mpss_size;
};

static int q6v5_load(struct rproc *rproc, const struct firmware *fw)
{
	struct q6v5 *qproc = rproc->priv;

	memcpy(qproc->mba_region, fw->data, fw->size);

	return 0;
}

static const struct rproc_fw_ops q6v5_fw_ops = {
	.find_rsc_table = qcom_mdt_find_rsc_table,
	.load = q6v5_load,
};

static int q6v5_rmb_pbl_wait(struct q6v5 *qproc, int ms)
{
	unsigned long timeout;
	s32 val;

	timeout = jiffies + msecs_to_jiffies(ms);
	for (;;) {
		val = readl(qproc->rmb_base + RMB_PBL_STATUS_REG);
		if (val)
			break;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		msleep(1);
	}

	return val;
}

static int q6v5_rmb_mba_wait(struct q6v5 *qproc, u32 status, int ms)
{

	unsigned long timeout;
	s32 val;

	timeout = jiffies + msecs_to_jiffies(ms);
	for (;;) {
		val = readl(qproc->rmb_base + RMB_MBA_STATUS_REG);
		if (val < 0)
			break;

		if (!status && val)
			break;
		else if (status && val == status)
			break;

		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		msleep(1);
	}

	return val;
}

static void q6v5proc_reset(struct q6v5 *qproc)
{
	u32 val;

	/* Assert resets, stop core */
	val = readl(qproc->reg_base + QDSP6SS_RESET_REG);
	val |= (Q6SS_CORE_ARES | Q6SS_BUS_ARES_ENABLE | Q6SS_STOP_CORE);
	writel(val, qproc->reg_base + QDSP6SS_RESET_REG);

	/* Enable power block headswitch, and wait for it to stabilize */
	val = readl(qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	val |= QDSS_BHS_ON | QDSS_LDO_BYP;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	mb();
	udelay(1);

	/*
	 * Turn on memories. L2 banks should be done individually
	 * to minimize inrush current.
	 */
	val = readl(qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	val |= Q6SS_SLP_RET_N | Q6SS_L2TAG_SLP_NRET_N |
		Q6SS_ETB_SLP_NRET_N | Q6SS_L2DATA_STBY_N;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	val |= Q6SS_L2DATA_SLP_NRET_N_2;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	val |= Q6SS_L2DATA_SLP_NRET_N_1;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);
	val |= Q6SS_L2DATA_SLP_NRET_N_0;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);

	/* Remove IO clamp */
	val &= ~Q6SS_CLAMP_IO;
	writel(val, qproc->reg_base + QDSP6SS_PWR_CTL_REG);

	/* Bring core out of reset */
	val = readl(qproc->reg_base + QDSP6SS_RESET_REG);
	val &= ~Q6SS_CORE_ARES;
	writel(val, qproc->reg_base + QDSP6SS_RESET_REG);

	/* Turn on core clock */
	val = readl(qproc->reg_base + QDSP6SS_GFMUX_CTL_REG);
	val |= Q6SS_CLK_ENABLE;
	writel(val, qproc->reg_base + QDSP6SS_GFMUX_CTL_REG);

	/* Start core execution */
	val = readl(qproc->reg_base + QDSP6SS_RESET_REG);
	val &= ~Q6SS_STOP_CORE;
	writel(val, qproc->reg_base + QDSP6SS_RESET_REG);
}

static void q6v5proc_halt_axi_port(struct q6v5 *qproc, void __iomem *halt)
{
	unsigned long timeout;
	u32 val;

	/* Check if we're already idle */
	if (readl(halt + AXI_IDLE_REG))
		return;

        /* Assert halt request */
        writel(1, halt + AXI_HALTREQ_REG);

        /* Wait for halt */
	timeout = jiffies + msecs_to_jiffies(HALT_ACK_TIMEOUT_MS);
	for (;;) {
		val = readl(halt + AXI_HALTACK_REG);
		if (val || time_after(jiffies, timeout))
			break;

		msleep(1);
	}

	if (!readl(halt + AXI_IDLE_REG))
		dev_err(qproc->dev, "port %pa failed halt\n", &halt);

        /* Clear halt request (port will remain halted until reset) */
        writel(0, halt + AXI_HALTREQ_REG);
}

static int q6v5_mpss_init_image(struct q6v5 *qproc, const struct firmware *fw)
{
	int ret;

	/* Use mpss memory as scratch buffer for the mdt validation */
	memcpy(qproc->mpss_region, fw->data, fw->size);

	writel(qproc->mpss_phys, qproc->rmb_base + RMB_PMI_META_DATA_REG);
	writel(RMB_CMD_META_DATA_READY, qproc->rmb_base + RMB_MBA_COMMAND_REG);

	ret = q6v5_rmb_mba_wait(qproc, RMB_MBA_META_DATA_AUTH_SUCCESS, 1000);
	if (ret == -ETIMEDOUT)
		dev_err(qproc->dev, "MBA header authentication timed out\n");
	else if (ret < 0)
		dev_err(qproc->dev, "MBA returned error %d for MDT header\n", ret);

	return ret < 0 ? ret : 0;
}

static int q6v5_mpss_validate(struct q6v5 *qproc, const struct firmware *fw)
{
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *phdr;
	struct elf32_hdr *ehdr;
	size_t size;
	u32 val;
	int i;

	ehdr = (struct elf32_hdr *)fw->data;
	phdrs = (struct elf32_phdr *)(ehdr + 1);
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		phdr = &phdrs[i];

		if (phdr->p_type != PT_LOAD)
			continue;

		if ((phdr->p_flags & QCOM_MDT_TYPE_MASK) == QCOM_MDT_TYPE_HASH)
			continue;

		if (!phdr->p_memsz)
			continue;

		size = readl(qproc->rmb_base + RMB_PMI_CODE_LENGTH_REG);
		if (!size) {
			writel(qproc->mpss_phys, qproc->rmb_base + RMB_PMI_CODE_START_REG);
			writel(RMB_CMD_LOAD_READY, qproc->rmb_base + RMB_MBA_COMMAND_REG);
		}

		size += phdr->p_memsz;
		writel(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH_REG);
	}

	val = readl(qproc->rmb_base + RMB_MBA_STATUS_REG);
	return val < 0 ? val : 0;
}

static int q6v5_mpss_load(struct q6v5 *qproc)
{
	const struct firmware *fw;
	phys_addr_t fw_addr;
	size_t fw_size;
	bool relocate;
	int ret;

	ret = request_firmware(&fw, MPSS_FIRMWARE_NAME, qproc->dev);
	if (ret < 0) {
		dev_err(qproc->dev, "unable to load " MPSS_FIRMWARE_NAME "\n");
		return ret;
	}

	ret = qcom_mdt_parse(fw, &fw_addr, &fw_size, &relocate);
	if (ret) {
		dev_err(qproc->dev, "failed to parse mdt header\n");
		return ret;
	}

	/* Initialize the RMB validator */
	writel(0, qproc->rmb_base + RMB_PMI_CODE_LENGTH_REG);

	ret = q6v5_mpss_init_image(qproc, fw);
	if (ret)
		goto release_firmware;

	ret = qcom_mdt_load(qproc->rproc, fw, MPSS_FIRMWARE_NAME, fw_addr, qproc->mpss_region, qproc->mpss_size);
	if (ret)
		goto release_firmware;

	ret = q6v5_mpss_validate(qproc, fw);
	if (ret)
		goto release_firmware;

	ret = q6v5_rmb_mba_wait(qproc, RMB_MBA_AUTH_COMPLETE, 10000);
	if (ret == -ETIMEDOUT)
		dev_err(qproc->dev, "MBA authentication timed out\n");
	else if (ret < 0)
		dev_err(qproc->dev, "MBA returned error %d\n", ret);

release_firmware:
	release_firmware(fw);

	return ret < 0 ? ret : 0;
}

static int q6v5_start(struct rproc *rproc)
{
	struct q6v5 *qproc = (struct q6v5 *)rproc->priv;
	int ret;

	ret = regulator_enable(qproc->vdd);
	if (ret) {
		dev_err(qproc->dev, "failed to enable mss vdd\n");
		return ret;
	}

	ret = reset_control_deassert(qproc->mss_restart);
	if (ret) {
		dev_err(qproc->dev, "failed to deassert mss restart\n");
		goto disable_vdd;
	}

	ret = clk_prepare_enable(qproc->ahb_clk);
	if (ret)
		goto assert_reset;

	ret = clk_prepare_enable(qproc->axi_clk);
	if (ret)
		goto disable_ahb_clk;

	ret = clk_prepare_enable(qproc->rom_clk);
	if (ret)
		goto disable_axi_clk;

	writel(qproc->mba_phys, qproc->rmb_base + RMB_MBA_IMAGE_REG);

	q6v5proc_reset(qproc);

	ret = q6v5_rmb_pbl_wait(qproc, 1000);
	if (ret == -ETIMEDOUT) {
		dev_err(qproc->dev, "PBL boot timed out\n");
		goto halt_axi_ports;
	} else if (ret != RMB_PBL_SUCCESS) {
		dev_err(qproc->dev, "PBL returned unexpected status %d\n", ret);
		ret = -EINVAL;
		goto halt_axi_ports;
	}

	ret = q6v5_rmb_mba_wait(qproc, 0, 5000);
	if (ret == -ETIMEDOUT) {
		dev_err(qproc->dev, "MBA boot timed out\n");
		goto halt_axi_ports;
	} else if (ret != RMB_MBA_XPU_UNLOCKED && ret != RMB_MBA_XPU_UNLOCKED_SCRIBBLED) {
		dev_err(qproc->dev, "MBA returned unexpected status %d\n", ret);
		ret = -EINVAL;
		goto halt_axi_ports;
	}

	dev_info(qproc->dev, "MBA booted, loading mpss\n");

	ret = q6v5_mpss_load(qproc);
	if (ret)
		goto halt_axi_ports;

	ret = wait_for_completion_timeout(&qproc->start_done,
					  msecs_to_jiffies(5000));
	if (ret == 0) {
		dev_err(qproc->dev, "start timed out\n");
		ret = -ETIMEDOUT;
		goto halt_axi_ports;
	}

	/* All done, release the handover resources */

	return 0;

halt_axi_ports:
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_Q6_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_MODEM_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_NC_HALT_BASE);
disable_axi_clk:
	clk_disable_unprepare(qproc->axi_clk);
disable_ahb_clk:
	clk_disable_unprepare(qproc->ahb_clk);
assert_reset:
	reset_control_assert(qproc->mss_restart);
disable_vdd:
	regulator_disable(qproc->vdd);

	return ret;
}

static int q6v5_stop(struct rproc *rproc)
{
	struct q6v5 *qproc = (struct q6v5 *)rproc->priv;

	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_Q6_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_MODEM_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_NC_HALT_BASE);

	reset_control_assert(qproc->mss_restart);
	clk_disable_unprepare(qproc->axi_clk);
	clk_disable_unprepare(qproc->ahb_clk);
	regulator_disable(qproc->vdd);

	return 0;
}

static const struct rproc_ops q6v5_ops = {
	.start = q6v5_start,
	.stop = q6v5_stop,
};

static irqreturn_t q6v5_wdog_interrupt(int irq, void *dev)
{
	struct q6v5 *qproc = dev;
	size_t len;
	char *msg;

	msg = qcom_smem_get(QCOM_SMEM_HOST_ANY, MPSS_CRASH_REASON_SMEM, &len);
	if (!IS_ERR(msg) && len > 0 && msg[0])
		dev_err(qproc->dev, "watchdog received: %s\n", msg);
	else
		dev_err(qproc->dev, "watchdog without message\n");

	rproc_report_crash(qproc->rproc, RPROC_WATCHDOG);

	if (!IS_ERR(msg))
		msg[0] = '\0';

	return IRQ_HANDLED;
}

static irqreturn_t q6v5_fatal_interrupt(int irq, void *dev)
{
	struct q6v5 *qproc = dev;
	size_t len;
	char *msg;

	msg = qcom_smem_get(QCOM_SMEM_HOST_ANY, MPSS_CRASH_REASON_SMEM, &len);
	if (!IS_ERR(msg) && len > 0 && msg[0])
		dev_err(qproc->dev, "fatal error received: %s\n", msg);
	else
		dev_err(qproc->dev, "fatal error without message\n");

	rproc_report_crash(qproc->rproc, RPROC_FATAL_ERROR);

	if (!IS_ERR(msg))
		msg[0] = '\0';

	return IRQ_HANDLED;
}

static irqreturn_t q6v5_handover_interrupt(int irq, void *dev)
{
	struct q6v5 *qproc = dev;

	complete(&qproc->start_done);
	return IRQ_HANDLED;
}

static irqreturn_t q6v5_stop_ack_interrupt(int irq, void *dev)
{
	return IRQ_HANDLED;
}

static int q6v5_init_mem(struct q6v5 *qproc, struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qdsp6_base");
	qproc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->reg_base)) {
		dev_err(qproc->dev, "failed to get qdsp6_base\n");
		return PTR_ERR(qproc->reg_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "halt_base");
	qproc->halt_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->halt_base)) {
		dev_err(qproc->dev, "failed to get halt_base\n");
		return PTR_ERR(qproc->halt_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rmb_base");
	qproc->rmb_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->rmb_base)) {
		dev_err(qproc->dev, "failed to get rmb_base\n");
		return PTR_ERR(qproc->rmb_base);
	}

	return 0;
}

static int q6v5_init_clocks(struct q6v5 *qproc)
{
	qproc->ahb_clk = devm_clk_get(qproc->dev, "iface");
	if (IS_ERR(qproc->ahb_clk)) {
		dev_err(qproc->dev, "failed to get iface clock\n");
		return PTR_ERR(qproc->ahb_clk);
	}

	qproc->axi_clk = devm_clk_get(qproc->dev, "bus");
	if (IS_ERR(qproc->axi_clk)) {
		dev_err(qproc->dev, "failed to get bus clock\n");
		return PTR_ERR(qproc->axi_clk);
	}

	qproc->rom_clk = devm_clk_get(qproc->dev, "mem");
	if (IS_ERR(qproc->rom_clk)) {
		dev_err(qproc->dev, "failed to get mem clock\n");
		return PTR_ERR(qproc->rom_clk);
	}

	return 0;
}

static int q6v5_init_regulators(struct q6v5 *qproc)
{
	qproc->vdd = devm_regulator_get(qproc->dev, "vdd");
	if (IS_ERR(qproc->vdd)) {
		dev_err(qproc->dev, "failed to get vdd supply\n");
		return PTR_ERR(qproc->vdd);
	}

	regulator_set_voltage(qproc->vdd, VDD_MSS_UV_MIN, VDD_MSS_UV_MAX);
	regulator_set_load(qproc->vdd, VDD_MSS_UA);

	return 0;
}

static int q6v5_init_reset(struct q6v5 *qproc)
{
	qproc->mss_restart = devm_reset_control_get(qproc->dev, NULL);
	if (IS_ERR(qproc->mss_restart)) {
		dev_err(qproc->dev, "failed to acquire mss restart\n");
		return PTR_ERR(qproc->mss_restart);
	}

	return 0;
}

static int q6v5_request_irq(struct q6v5 *qproc,
			     struct platform_device *pdev,
			     const char *name,
			     irq_handler_t thread_fn)
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
					"q6v5", qproc);
	if (ret)
		dev_err(&pdev->dev, "request %s IRQ failed\n", name);
	return ret;
}

static int q6v5_alloc_memory_region(struct q6v5 *qproc)
{
	struct device_node *child;
	struct device_node *node;
	struct resource r;
	int ret;

	child = of_get_child_by_name(qproc->dev->of_node, "mba");
	node = of_parse_phandle(child, "memory-region", 0);
	ret = of_address_to_resource(node, 0, &r);
	if (ret) {
		dev_err(qproc->dev, "unable to resolve mba region\n");
		return ret;
	}

	qproc->mba_phys = r.start;
	qproc->mba_size = resource_size(&r);
	qproc->mba_region = devm_ioremap_wc(qproc->dev, qproc->mba_phys, qproc->mba_size);
	if (!qproc->mba_region) {
		dev_err(qproc->dev, "unable to map memory region: %pa+%zx\n",
			&r.start, qproc->mba_size);
		return -EBUSY;
	}

	child = of_get_child_by_name(qproc->dev->of_node, "mpss");
	node = of_parse_phandle(child, "memory-region", 0);
	ret = of_address_to_resource(node, 0, &r);
	if (ret) {
		dev_err(qproc->dev, "unable to resolve mpss region\n");
		return ret;
	}

	qproc->mpss_phys = r.start;
	qproc->mpss_size = resource_size(&r);
	qproc->mpss_region = devm_ioremap_wc(qproc->dev, qproc->mpss_phys, qproc->mpss_size);
	if (!qproc->mpss_region) {
		dev_err(qproc->dev, "unable to map memory region: %pa+%zx\n",
			&r.start, qproc->mpss_size);
		return -EBUSY;
	}

	return 0;
}

static int q6v5_probe(struct platform_device *pdev)
{
	struct q6v5 *qproc;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(&pdev->dev, pdev->name, &q6v5_ops,
			    MBA_FIRMWARE_NAME, sizeof(*qproc));
	if (!rproc) {
		dev_err(&pdev->dev, "failed to allocate rproc\n");
		return -ENOMEM;
	}

	rproc->fw_ops = &q6v5_fw_ops;

	qproc = (struct q6v5 *)rproc->priv;
	qproc->dev = &pdev->dev;
	qproc->rproc = rproc;
	platform_set_drvdata(pdev, qproc);

	init_completion(&qproc->start_done);

	ret = q6v5_init_mem(qproc, pdev);
	if (ret)
		goto free_rproc;

	ret = q6v5_alloc_memory_region(qproc);
	if (ret)
		goto free_rproc;

	ret = q6v5_init_clocks(qproc);
	if (ret)
		goto free_rproc;

	ret = q6v5_init_regulators(qproc);
	if (ret)
		goto free_rproc;

	ret = q6v5_init_reset(qproc);
	if (ret)
		goto free_rproc;

	ret = q6v5_request_irq(qproc, pdev, "wdog", q6v5_wdog_interrupt);
	if (ret < 0)
		goto free_rproc;

	ret = q6v5_request_irq(qproc, pdev, "fatal", q6v5_fatal_interrupt);
	if (ret < 0)
		goto free_rproc;

	ret = q6v5_request_irq(qproc, pdev, "handover", q6v5_handover_interrupt);
	if (ret < 0)
		goto free_rproc;

	ret = q6v5_request_irq(qproc, pdev, "stop-ack", q6v5_stop_ack_interrupt);
	if (ret < 0)
		goto free_rproc;

	qproc->state = qcom_smem_state_get(&pdev->dev, "stop", &qproc->stop_bit);
	if (IS_ERR(qproc->state))
		goto free_rproc;

	ret = rproc_add(rproc);
	if (ret)
		goto free_rproc;

	return 0;

free_rproc:
	rproc_put(rproc);

	return ret;
}

static int q6v5_remove(struct platform_device *pdev)
{
	struct q6v5 *qproc = platform_get_drvdata(pdev);

	rproc_del(qproc->rproc);
	rproc_put(qproc->rproc);

	return 0;
}

static const struct of_device_id q6v5_of_match[] = {
	{ .compatible = "qcom,q6v5-pil", },
	{ },
};

static struct platform_driver q6v5_driver = {
	.probe = q6v5_probe,
	.remove = q6v5_remove,
	.driver = {
		.name = "qcom-q6v5-pil",
		.of_match_table = q6v5_of_match,
	},
};

module_platform_driver(q6v5_driver);
