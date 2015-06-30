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
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/soc/qcom/smem.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

#include <linux/qcom_scm.h>

#define SCM_SVC_PIL                    0x2

struct qproc {
	struct device *dev;
	struct rproc *rproc;

	void __iomem *reg_base;
	void __iomem *halt_base;
	void __iomem *rmb_base;

	struct reset_control *mss_restart;

	int wdog_irq;
	int fatal_irq;
	int ready_irq;
	int handover_irq;
	int stop_ack_irq;

	struct gpio_desc *stop_gpio;

	struct regulator *vdd;
	struct regulator *cx;
	struct regulator *mx;
	struct regulator *pll;

	struct clk *ahb_clk;
	struct clk *axi_clk;
	struct clk *rom_clk;

	struct completion start_done;

	void			*mba_va;
	dma_addr_t		mba_da;
	size_t			mba_size;
	struct dma_attrs	mba_attrs;
};

#define VDD_MSS_UV	1000000
#define VDD_MSS_UV_MAX	1150000
#define VDD_MSS_UA	100000

/* Q6 Register Offsets */
#define QDSP6SS_RST_EVB                 0x010

/* AXI Halting Registers */
#define MSS_Q6_HALT_BASE                0x180
#define MSS_MODEM_HALT_BASE             0x200
#define MSS_NC_HALT_BASE                0x280

/* RMB Status Register Values */
#define STATUS_PBL_SUCCESS              0x1
#define STATUS_XPU_UNLOCKED             0x1
#define STATUS_XPU_UNLOCKED_SCRIBBLED   0x2

/* PBL/MBA interface registers */
#define RMB_MBA_IMAGE                   0x00
#define RMB_PBL_STATUS                  0x04
#define RMB_MBA_COMMAND                 0x08
#define RMB_MBA_STATUS                  0x0C
#define RMB_PMI_META_DATA               0x10
#define RMB_PMI_CODE_START              0x14
#define RMB_PMI_CODE_LENGTH             0x18

#define POLL_INTERVAL_US                50

#define CMD_META_DATA_READY             0x1
#define CMD_LOAD_READY                  0x2

#define STATUS_META_DATA_AUTH_SUCCESS   0x3
#define STATUS_AUTH_COMPLETE            0x4

/* External BHS */
#define EXTERNAL_BHS_ON                 BIT(0)
#define EXTERNAL_BHS_STATUS             BIT(4)
#define BHS_TIMEOUT_US                  50

#define MSS_RESTART_ID                  0xA

/* QDSP6SS Register Offsets */
#define QDSP6SS_RESET                   0x014
#define QDSP6SS_GFMUX_CTL               0x020
#define QDSP6SS_PWR_CTL                 0x030
#define QDSP6SS_STRAP_ACC               0x110

/* AXI Halt Register Offsets */
#define AXI_HALTREQ                     0x0
#define AXI_HALTACK                     0x4
#define AXI_IDLE                        0x8

#define HALT_ACK_TIMEOUT_US             100000

/* QDSP6SS_RESET */
#define Q6SS_STOP_CORE                  BIT(0)
#define Q6SS_CORE_ARES                  BIT(1)
#define Q6SS_BUS_ARES_ENA               BIT(2)

/* QDSP6SS_GFMUX_CTL */
#define Q6SS_CLK_ENA                    BIT(1)
#define Q6SS_CLK_SRC_SEL_C              BIT(3)
#define Q6SS_CLK_SRC_SEL_FIELD          0xC
#define Q6SS_CLK_SRC_SWITCH_CLK_OVR     BIT(8)

/* QDSP6SS_PWR_CTL */
#define Q6SS_L2DATA_SLP_NRET_N_0        BIT(0)
#define Q6SS_L2DATA_SLP_NRET_N_1        BIT(1)
#define Q6SS_L2DATA_SLP_NRET_N_2        BIT(2)
#define Q6SS_L2TAG_SLP_NRET_N           BIT(16)
#define Q6SS_ETB_SLP_NRET_N             BIT(17)
#define Q6SS_L2DATA_STBY_N              BIT(18)
#define Q6SS_SLP_RET_N                  BIT(19)
#define Q6SS_CLAMP_IO                   BIT(20)
#define QDSS_BHS_ON                     BIT(21)
#define QDSS_LDO_BYP                    BIT(22)

/* QDSP6v55 parameters */
#define QDSP6v55_LDO_ON                 BIT(26)
#define QDSP6v55_LDO_BYP                BIT(25)
#define QDSP6v55_BHS_ON                 BIT(24)
#define QDSP6v55_CLAMP_WL               BIT(21)
#define L1IU_SLP_NRET_N                 BIT(15)
#define L1DU_SLP_NRET_N                 BIT(14)
#define L2PLRU_SLP_NRET_N               BIT(13)

#define HALT_CHECK_MAX_LOOPS            (200)
#define QDSP6SS_XO_CBCR                 (0x0038)

#define QDSP6SS_ACC_OVERRIDE_VAL        0x20

static int qproc_sanity_check(struct rproc *rproc,
				  const struct firmware *fw)
{
	if (!fw) {
		dev_err(&rproc->dev, "failed to load %s\n", rproc->name);
		return -EINVAL;
	}

	/* XXX: ??? */

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

static int qproc_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qproc *qproc = rproc->priv;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t phys;
	dma_addr_t end;
	void *ptr;

	dma_set_mask(qproc->dev, DMA_BIT_MASK(32));
	dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &attrs);

	ptr = dma_alloc_attrs(qproc->dev, fw->size, &phys, GFP_KERNEL, &attrs);
	if (!ptr) {
		dev_err(qproc->dev, "failed to allocate mba metadata buffer\n");
		return -ENOMEM;
	}

	end = phys + fw->size;
	dev_info(qproc->dev, "loading MBA from %pa to %pa\n", &phys, &end);

	memcpy(ptr, fw->data, fw->size);

	qproc->mba_va = ptr;
	qproc->mba_da = phys;
	qproc->mba_size = fw->size;
	qproc->mba_attrs = attrs;

	return 0;
}

static const struct rproc_fw_ops qproc_fw_ops = {
	.find_rsc_table = qproc_find_rsc_table,
	.load = qproc_load,
	.sanity_check = qproc_sanity_check,
};

static void q6v5proc_reset(struct qproc *qproc)
{
	u32 val;

	/* Assert resets, stop core */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_RESET);
	val |= (Q6SS_CORE_ARES | Q6SS_BUS_ARES_ENA | Q6SS_STOP_CORE);
	writel_relaxed(val, qproc->reg_base + QDSP6SS_RESET);

	/* Enable power block headswitch, and wait for it to stabilize */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_PWR_CTL);
	val |= QDSS_BHS_ON | QDSS_LDO_BYP;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);
	mb();
	udelay(1);

	/*
	 * Turn on memories. L2 banks should be done individually
	 * to minimize inrush current.
	 */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_PWR_CTL);
	val |= Q6SS_SLP_RET_N | Q6SS_L2TAG_SLP_NRET_N |
		Q6SS_ETB_SLP_NRET_N | Q6SS_L2DATA_STBY_N;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);
	val |= Q6SS_L2DATA_SLP_NRET_N_2;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);
	val |= Q6SS_L2DATA_SLP_NRET_N_1;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);
	val |= Q6SS_L2DATA_SLP_NRET_N_0;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);

	/* Remove IO clamp */
	val &= ~Q6SS_CLAMP_IO;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_PWR_CTL);

	/* Bring core out of reset */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_RESET);
	val &= ~Q6SS_CORE_ARES;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_RESET);

	/* Turn on core clock */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_GFMUX_CTL);
	val |= Q6SS_CLK_ENA;

#if 0
	/* Need a different clock source for v5.2.0 */
	if (qproc->qdsp6v5_2_0) {
		val &= ~Q6SS_CLK_SRC_SEL_FIELD;
		val |= Q6SS_CLK_SRC_SEL_C;
	}

	/* force clock on during source switch */
	if (qproc->qdsp6v56)
		val |= Q6SS_CLK_SRC_SWITCH_CLK_OVR;
#endif

	writel_relaxed(val, qproc->reg_base + QDSP6SS_GFMUX_CTL);

	/* Start core execution */
	val = readl_relaxed(qproc->reg_base + QDSP6SS_RESET);
	val &= ~Q6SS_STOP_CORE;
	writel_relaxed(val, qproc->reg_base + QDSP6SS_RESET);
}

static void q6v5proc_halt_axi_port(struct qproc *qproc, void __iomem *halt)
{
	unsigned long timeout;

	if (readl_relaxed(halt + AXI_IDLE))
		return;

        /* Assert halt request */
        writel_relaxed(1, halt + AXI_HALTREQ);

        /* Wait for halt */
	timeout = jiffies + 10 * HZ;
	for (;;) {
		if (readl(halt + AXI_HALTACK) || time_after(jiffies, timeout))
			break;

		msleep(1);
	}

	if (!readl_relaxed(halt + AXI_IDLE))
		dev_err(qproc->dev, "port %pa failed halt\n", &halt);

        /* Clear halt request (port will remain halted until reset) */
        writel_relaxed(0, halt + AXI_HALTREQ);
}

static int qproc_mba_load_mdt(struct qproc *qproc, const struct firmware *fw)
{
	DEFINE_DMA_ATTRS(attrs);
	unsigned long timeout;
	dma_addr_t phys;
	dma_addr_t end;
	void *ptr;
	int ret;
	s32 val;

	dma_set_mask(qproc->dev, DMA_BIT_MASK(32));
	dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &attrs);

	ptr = dma_alloc_attrs(qproc->dev, fw->size, &phys, GFP_KERNEL, &attrs);
	if (!ptr) {
		dev_err(qproc->dev, "failed to allocate mba metadata buffer\n");
		return -ENOMEM;
	}

	end = phys + fw->size;
	dev_info(qproc->dev, "loading mdt header from %pa to %pa\n", &phys, &end);

	memcpy(ptr, fw->data, fw->size);

	writel_relaxed(0, qproc->rmb_base + RMB_PMI_CODE_LENGTH);

	writel_relaxed(phys, qproc->rmb_base + RMB_PMI_META_DATA);
	writel(CMD_META_DATA_READY, qproc->rmb_base + RMB_MBA_COMMAND);

	timeout = jiffies + HZ;
	for (;;) {
		msleep(1);

		val = readl(qproc->rmb_base + RMB_MBA_STATUS);
		if (val == STATUS_META_DATA_AUTH_SUCCESS || val < 0)
			break;

		if (time_after(jiffies, timeout))
			break;
	}
	if (val == 0) {
		dev_err(qproc->dev, "MBA authentication of headers timed out\n");
		ret = -ETIMEDOUT;
		goto out;
	} else if (val < 0) {
		dev_err(qproc->dev, "MBA returned error %d for headers\n", val);
		ret = -EINVAL;
		goto out;
	}

	dev_err(qproc->dev, "mdt authenticated\n");

	ret = 0;
out:
	dma_free_attrs(qproc->dev, fw->size, ptr, phys, &attrs);

	return ret;
}

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int
qproc_load_segments(struct qproc *qproc, const struct firmware *fw)
{
	struct device *dev = qproc->dev;
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;
	const struct firmware *seg_fw;
	char fw_name[20];

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		if (segment_is_hash(phdr->p_flags))
			continue;

		if (filesz == 0)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%x memsz 0x%x filesz 0x%x\n",
					phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%x memsz 0x%x\n",
							filesz, memsz);
			ret = -EINVAL;
			break;
		}

		ptr = ioremap(da, memsz);
		if (!ptr) {
			dev_err(qproc->dev, "failed to allocate mba metadata buffer\n");
			ret = -ENOMEM;
			break;
		}

		if (filesz) {
			snprintf(fw_name, sizeof(fw_name), "modem.b%02d", i);
			ret = request_firmware(&seg_fw, fw_name, qproc->dev);
			if (ret) {
				iounmap(ptr);
				break;
			}

			memcpy(ptr, seg_fw->data, filesz);

			release_firmware(seg_fw);
		}

		if (memsz > filesz)
			memset(ptr + filesz, 0, memsz - filesz);

		wmb();
		iounmap(ptr);
	}

	return ret;
}

static int qproc_verify_segments(struct qproc *qproc, const struct firmware *fw)
{
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	const u8 *elf_data = fw->data;
	unsigned long timeout;
	u32 min_addr = (phys_addr_t)ULLONG_MAX;
	u32 size = 0;
	s32 val;
	int ret;
	int i;
	u32 v;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	v = readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %pa\n", &v);

	msleep(1);

	v = readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %pa\n", &v);

#if 1
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u32 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;

		if (phdr->p_type != PT_LOAD)
			continue;

		dev_err(qproc->dev, "0x%x %d %d\n", phdr->p_paddr, segment_is_hash(phdr->p_flags), !!(phdr->p_flags & BIT(27)));

		if (segment_is_hash(phdr->p_flags))
			continue;

		if (memsz == 0)
			continue;

		if (da < min_addr)
			min_addr = da;

		size += memsz;
	}

	dev_err(qproc->dev, "verify: %pa:%pa\n", &min_addr, &size);

	writel_relaxed(min_addr, qproc->rmb_base + RMB_PMI_CODE_START);
	writel(CMD_LOAD_READY, qproc->rmb_base + RMB_MBA_COMMAND);
	writel(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
#endif

	v = readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %pa\n", &v);

#if 0
	writel_relaxed(0x08400000, qproc->rmb_base + RMB_PMI_CODE_START);
	writel_relaxed(CMD_LOAD_READY, qproc->rmb_base + RMB_MBA_COMMAND);

	size = 0;
	size += 0x162f4;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x5f7620;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x719e1c;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x14000;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x2b929;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x0d500;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x19ab8;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x16d68;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x124a98;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x103588;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0xbf99b0;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0xa07a0;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x12000;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x01500;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x792878;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x256c44;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x14fee4;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x20d13c0;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x2c4f0;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x3a2a8;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	size += 0x3ca000;
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
	writel_relaxed(size, qproc->rmb_base + RMB_PMI_CODE_LENGTH);
	dev_err(qproc->dev, "RMB_PMI_CODE_LENGTH: %x\n", readl_relaxed(qproc->rmb_base + RMB_PMI_CODE_LENGTH));
	dev_err(qproc->dev, "RMB_MBA_STATUS: 0x%x\n", readl(qproc->rmb_base + RMB_MBA_STATUS));
#endif

	timeout = jiffies + 10 * HZ;
	for (;;) {
		msleep(1);

		val = readl(qproc->rmb_base + RMB_MBA_STATUS);
		if (val == STATUS_AUTH_COMPLETE || val < 0)
			break;

		if (time_after(jiffies, timeout))
			break;
	}
	if (val == 0) {
		dev_err(qproc->dev, "MBA authentication of headers timed out\n");
		ret = -ETIMEDOUT;
		goto out;
	} else if (val < 0) {
		dev_err(qproc->dev, "MBA returned error %d for segments\n", val);
		ret = -EINVAL;
		goto out;
	}

	ret = 0;
out:
	return ret;
}

static int qproc_load_modem(struct qproc *qproc)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "modem.mdt", qproc->dev);
	if (ret < 0) {
		dev_err(qproc->dev, "unable to load modem.mdt\n");
		return ret;
	}

	ret = qproc_mba_load_mdt(qproc, fw);
	if (ret)
		goto out;

	ret = qproc_load_segments(qproc, fw);
	if (ret)
		goto out;

	ret = qproc_verify_segments(qproc, fw);
	if (ret)
		goto out;

out:
	release_firmware(fw);

	return ret;
}

static int qproc_start(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;
	unsigned long timeout;
	int ret;
	u32 val;

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

	writel_relaxed(qproc->mba_da, qproc->rmb_base + RMB_MBA_IMAGE);

	/* Ensure order of data/entry point and the following reset release */
	wmb();

	q6v5proc_reset(qproc);

	timeout = jiffies + HZ;
	for (;;) {
		msleep(1);

		val = readl(qproc->rmb_base + RMB_PBL_STATUS);
		if (val || time_after(jiffies, timeout))
			break;
	}
	if (val == 0) {
		dev_err(qproc->dev, "PBL boot timed out\n");
		ret = -ETIMEDOUT;
		goto halt_axi_ports;
	} else if (val != STATUS_PBL_SUCCESS) {
		dev_err(qproc->dev, "PBL returned unexpected status %d\n", val);
		ret = -EINVAL;
		goto halt_axi_ports;
	}

	timeout = jiffies + HZ;
	for (;;) {
		msleep(1);

		val = readl(qproc->rmb_base + RMB_MBA_STATUS);
		if (val || time_after(jiffies, timeout))
			break;
	}
	if (val == 0) {
		dev_err(qproc->dev, "MBA boot timed out\n");
		ret = -ETIMEDOUT;
		goto halt_axi_ports;
	} else if (val != STATUS_XPU_UNLOCKED && val != STATUS_XPU_UNLOCKED_SCRIBBLED) {
		dev_err(qproc->dev, "MBA returned unexpected status %d\n", val);
		ret = -EINVAL;
		goto halt_axi_ports;
	}

	dev_info(qproc->dev, "MBA boot done\n");

	ret = qproc_load_modem(qproc);
	if (ret)
		goto halt_axi_ports;

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

	dma_free_attrs(qproc->dev, qproc->mba_size, qproc->mba_va, qproc->mba_da, &qproc->mba_attrs);
	return ret;
}

static int qproc_stop(struct rproc *rproc)
{
	struct qproc *qproc = (struct qproc *)rproc->priv;

	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_Q6_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_MODEM_HALT_BASE);
	q6v5proc_halt_axi_port(qproc, qproc->halt_base + MSS_NC_HALT_BASE);

	reset_control_assert(qproc->mss_restart);
	clk_disable_unprepare(qproc->axi_clk);
	clk_disable_unprepare(qproc->ahb_clk);
	regulator_disable(qproc->vdd);

	dma_free_attrs(qproc->dev, qproc->mba_size, qproc->mba_va, qproc->mba_da, &qproc->mba_attrs);

	return 0;
}

static const struct rproc_ops qproc_ops = {
	.start = qproc_start,
	.stop = qproc_stop,
};

static irqreturn_t qproc_wdog_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	dev_err(qproc->dev, "                                                            WATCHDOG\n");

	rproc_report_crash(qproc->rproc, RPROC_WATCHDOG);
	return IRQ_HANDLED;
}

static irqreturn_t qproc_fatal_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	dev_err(qproc->dev, "                                                            FATAL\n");

	rproc_report_crash(qproc->rproc, RPROC_FATAL_ERROR);

	return IRQ_HANDLED;
}

static irqreturn_t qproc_ready_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	dev_err(qproc->dev, "                                                            READY\n");

	complete(&qproc->start_done);

	return IRQ_HANDLED;
}

static irqreturn_t qproc_handover_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	dev_err(qproc->dev, "                                                            HANDOVER\n");

	return IRQ_HANDLED;
}

static irqreturn_t qproc_stop_ack_interrupt(int irq, void *dev)
{
	struct qproc *qproc = dev;

	dev_err(qproc->dev, "                                                            STOP-ACK\n");

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

static int qproc_init_mem(struct qproc *qproc, struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qdsp6_base");
	qproc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->reg_base))
		return PTR_ERR(qproc->reg_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "halt_base");
	qproc->halt_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->halt_base))
		return PTR_ERR(qproc->halt_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rmb_base");
	qproc->rmb_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qproc->rmb_base))
		return PTR_ERR(qproc->rmb_base);

	return 0;
}

static int qproc_init_clocks(struct qproc *qproc)
{
	qproc->ahb_clk = devm_clk_get(qproc->dev, "iface");
	if (IS_ERR(qproc->ahb_clk))
		return PTR_ERR(qproc->ahb_clk);

	qproc->axi_clk = devm_clk_get(qproc->dev, "bus");
	if (IS_ERR(qproc->axi_clk))
		return PTR_ERR(qproc->axi_clk);

	qproc->rom_clk = devm_clk_get(qproc->dev, "mem");
	if (IS_ERR(qproc->rom_clk))
		return PTR_ERR(qproc->rom_clk);

	return 0;
}

static int qproc_init_regulators(struct qproc *qproc)
{
	int ret;
	u32 uV;

	qproc->vdd = devm_regulator_get(qproc->dev, "qcom,vdd");
	if (IS_ERR(qproc->vdd))
		return PTR_ERR(qproc->vdd);

	regulator_set_voltage(qproc->vdd, VDD_MSS_UV, VDD_MSS_UV_MAX);
	regulator_set_load(qproc->vdd, VDD_MSS_UA);

	qproc->cx = devm_regulator_get(qproc->dev, "qcom,cx");
	if (IS_ERR(qproc->cx))
		return PTR_ERR(qproc->cx);

	qproc->mx = devm_regulator_get(qproc->dev, "qcom,mx");
	if (IS_ERR(qproc->mx))
		return PTR_ERR(qproc->mx);

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,mx-uV", &uV);
	if (!ret)
		regulator_set_voltage(qproc->mx, uV, uV);

	qproc->pll = devm_regulator_get(qproc->dev, "qcom,pll");
	if (IS_ERR(qproc->pll))
		return PTR_ERR(qproc->pll);

	ret = of_property_read_u32(qproc->dev->of_node, "qcom,pll-uV", &uV);
	if (!ret)
		regulator_set_voltage(qproc->pll, uV, uV);

	return 0;
}

static int qproc_init_reset(struct qproc *qproc)
{
	qproc->mss_restart = devm_reset_control_get(qproc->dev, NULL);
	if (IS_ERR(qproc->mss_restart)) {
		dev_err(qproc->dev, "failed to acquire mss restart\n");
		return PTR_ERR(qproc->mss_restart);
	}

	return 0;
}

static int qproc_request_irq(struct qproc *qproc,
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
					"qproc", qproc);
	if (ret)
		dev_err(&pdev->dev, "request %s IRQ failed\n", name);
	return ret;
}

static int qproc_probe(struct platform_device *pdev)
{
	struct qproc *qproc;
	struct rproc *rproc;
	int ret;
	int i;

	rproc = rproc_alloc(&pdev->dev, pdev->name, &qproc_ops,
			    "mba.b00", sizeof(*qproc));
	if (!rproc)
		return -ENOMEM;

	rproc->fw_ops = &qproc_fw_ops;

	qproc = (struct qproc *)rproc->priv;
	qproc->dev = &pdev->dev;
	qproc->rproc = rproc;
	platform_set_drvdata(pdev, qproc);

	init_completion(&qproc->start_done);

	ret = qproc_init_mem(qproc, pdev);
	if (ret)
		goto free_rproc;

	ret = qproc_init_clocks(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_regulators(qproc);
	if (ret)
		goto free_rproc;

	ret = qproc_init_reset(qproc);
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
	{ .compatible = "qcom,q6v5-pil", },
	{ },
};

static struct platform_driver qproc_driver = {
	.probe = qproc_probe,
	.remove = qproc_remove,
	.driver = {
		.name = "qcom-q6v5-pil",
		.of_match_table = qproc_of_match,
	},
};

module_platform_driver(qproc_driver);
