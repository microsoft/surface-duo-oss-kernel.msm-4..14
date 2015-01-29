/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2014 Red Hat
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */


/* NOTE: originally based on msm_iommu non-DT driver for same hw
 * but as the structure of the driver changes considerably for DT
 * it seemed easier to not try to support old platforms with the
 * same driver.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/clk.h>

#include <asm/cacheflush.h>
#include <asm/sizes.h>

#include "msm_iommu_hw-8xxx.h"
#include "qcom_iommu_v0.h"

#define MRC(reg, processor, op1, crn, crm, op2)				\
__asm__ __volatile__ (							\
"   mrc   "   #processor "," #op1 ", %0,"  #crn "," #crm "," #op2 "\n"  \
: "=r" (reg))

#define RCP15_PRRR(reg)		MRC(reg, p15, 0, c10, c2, 0)
#define RCP15_NMRR(reg)		MRC(reg, p15, 0, c10, c2, 1)

/* bitmap of the page sizes currently supported */
#define QCOM_IOMMU_PGSIZES	(SZ_4K | SZ_64K | SZ_1M | SZ_16M)

static int qcom_iommu_tex_class[4];

// TODO any good reason for global lock vs per-iommu lock?
static DEFINE_MUTEX(qcom_iommu_lock);
static LIST_HEAD(qcom_iommu_devices);

/* Note that a single iommu_domain can, for devices sitting behind
 * more than one IOMMU (ie. one per AXI interface) will have more
 * than one iommu in the iommu_list.  But all are programmed to
 * point at the same pagetables so from client device perspective
 * they act as a single IOMMU.
 */
struct qcom_domain_priv {
	unsigned long *pgtable;
	struct iommu_domain *domain;
	struct list_head iommu_list;  /* list of attached 'struct qcom_iommu' */
};

static int __enable_clocks(struct qcom_iommu *iommu)
{
	int ret;

	ret = clk_prepare_enable(iommu->pclk);
	if (ret)
		goto fail;

	if (iommu->clk) {
		ret = clk_prepare_enable(iommu->clk);
		if (ret)
			clk_disable_unprepare(iommu->pclk);
	}
fail:
	return ret;
}

static void __disable_clocks(struct qcom_iommu *iommu)
{
	if (iommu->clk)
		clk_disable_unprepare(iommu->clk);
	clk_disable_unprepare(iommu->pclk);
}

static void __flush_range(struct qcom_iommu *iommu,
		unsigned long *start, unsigned long *end)
{
	dmac_flush_range(start, end);
}

static int __flush_iotlb_va(struct qcom_domain_priv *priv, unsigned int va)
{
	struct qcom_iommu *iommu;
	int ret = 0;

	list_for_each_entry(iommu, &priv->iommu_list, dom_node) {
		struct qcom_iommu_ctx *iommu_ctx;
		list_for_each_entry(iommu_ctx, &iommu->ctx_list, node) {
			int ctx = iommu_ctx->num;
			uint32_t asid;

			ret = __enable_clocks(iommu);
			if (ret)
				goto fail;

			asid = GET_CONTEXTIDR_ASID(iommu->base, ctx);

			SET_TLBIVA(iommu->base, ctx, asid | (va & TLBIVA_VA));
			mb();

			__disable_clocks(iommu);
		}
	}

fail:
	return ret;
}

static int __flush_iotlb(struct qcom_domain_priv *priv)
{
	struct qcom_iommu *iommu;
	int ret = 0;

#ifndef CONFIG_IOMMU_PGTABLES_L2
	unsigned long *fl_table = priv->pgtable;
	int i;

	list_for_each_entry(iommu, &priv->iommu_list, dom_node) {

		__flush_range(iommu, fl_table, fl_table + SZ_16K);

		for (i = 0; i < NUM_FL_PTE; i++) {
			if ((fl_table[i] & 0x03) == FL_TYPE_TABLE) {
				void *sl_table = __va(fl_table[i] &
						FL_BASE_MASK);
				__flush_range(iommu, sl_table, sl_table + SZ_4K);
			}
		}

		/*
		 * Only need to flush once, all iommu's attached
		 * to the domain use common set of pagetables:
		 */
		break;
	}
#endif

	list_for_each_entry(iommu, &priv->iommu_list, dom_node) {
		struct qcom_iommu_ctx *iommu_ctx;

		ret = __enable_clocks(iommu);
		if (ret)
			goto fail;

		list_for_each_entry(iommu_ctx, &iommu->ctx_list, node) {
			int ctx = iommu_ctx->num;
			uint32_t asid;

			asid = GET_CONTEXTIDR_ASID(iommu->base, ctx);

			SET_TLBIASID(iommu->base, ctx, asid);
			mb();
		}
		__disable_clocks(iommu);
	}

fail:
	return ret;
}

static void __reset_context(struct qcom_iommu *iommu, int ctx)
{
	void __iomem *base = iommu->base;

	SET_BPRCOSH(base, ctx, 0);
	SET_BPRCISH(base, ctx, 0);
	SET_BPRCNSH(base, ctx, 0);
	SET_BPSHCFG(base, ctx, 0);
	SET_BPMTCFG(base, ctx, 0);
	SET_ACTLR(base, ctx, 0);
	SET_SCTLR(base, ctx, 0);
	SET_FSRRESTORE(base, ctx, 0);
	SET_TTBR0(base, ctx, 0);
	SET_TTBR1(base, ctx, 0);
	SET_TTBCR(base, ctx, 0);
	SET_BFBCR(base, ctx, 0);
	SET_PAR(base, ctx, 0);
	SET_FAR(base, ctx, 0);
	SET_TLBFLPTER(base, ctx, 0);
	SET_TLBSLPTER(base, ctx, 0);
	SET_TLBLKCR(base, ctx, 0);
	SET_PRRR(base, ctx, 0);
	SET_NMRR(base, ctx, 0);
	SET_RESUME(base, ctx, 1);
	mb();
}

static void __reset_iommu(struct qcom_iommu *iommu)
{
	void __iomem *base = iommu->base;
	int ctx;

	SET_RPUE(base, 0);
	SET_RPUEIE(base, 0);
	SET_ESRRESTORE(base, 0);
	SET_TBE(base, 0);
	SET_CR(base, 0);
	SET_SPDMBE(base, 0);
	SET_TESTBUSCR(base, 0);
	SET_TLBRSW(base, 0);
	SET_GLOBAL_TLBIALL(base, 0);
	SET_RPU_ACR(base, 0);
	SET_TLBLKCRWE(base, 1);

	for (ctx = 0; ctx < iommu->ncb; ctx++) {
		SET_BPRCOSH(base, ctx, 0);
		SET_BPRCISH(base, ctx, 0);
		SET_BPRCNSH(base, ctx, 0);
		SET_BPSHCFG(base, ctx, 0);
		SET_BPMTCFG(base, ctx, 0);
		SET_ACTLR(base, ctx, 0);
		SET_SCTLR(base, ctx, 0);
		SET_FSRRESTORE(base, ctx, 0);
		SET_TTBR0(base, ctx, 0);
		SET_TTBR1(base, ctx, 0);
		SET_TTBCR(base, ctx, 0);
		SET_BFBCR(base, ctx, 0);
		SET_PAR(base, ctx, 0);
		SET_FAR(base, ctx, 0);
		SET_TLBFLPTER(base, ctx, 0);
		SET_TLBSLPTER(base, ctx, 0);
		SET_TLBLKCR(base, ctx, 0);
		SET_CTX_TLBIALL(base, ctx, 0);
		SET_TLBIVA(base, ctx, 0);
		SET_PRRR(base, ctx, 0);
		SET_NMRR(base, ctx, 0);
		SET_CONTEXTIDR(base, ctx, 0);
	}
	mb();
}


static void __program_context(struct qcom_domain_priv *priv,
		struct qcom_iommu *iommu, int ctx)
{
	void __iomem *base = iommu->base;
	phys_addr_t pgtable = __pa(priv->pgtable);
	unsigned int prrr, nmrr;
	bool found;
	int i, j;

	__reset_context(iommu, ctx);

	/* Set up HTW mode */
	/* TLB miss configuration: perform HTW on miss */
	SET_TLBMCFG(base, ctx, 0x3);

	/* V2P configuration: HTW for access */
	SET_V2PCFG(base, ctx, 0x3);

	SET_TTBCR(base, ctx, iommu->ttbr_split);
	SET_TTBR0_PA(base, ctx, (pgtable >> TTBR0_PA_SHIFT));
	if (iommu->ttbr_split)
		SET_TTBR1_PA(base, ctx, (pgtable >> TTBR1_PA_SHIFT));

	/* Enable context fault interrupt */
	SET_CFEIE(base, ctx, 1);

	/* Stall access on a context fault and let the handler deal with it */
	SET_CFCFG(base, ctx, 1);

	/* Redirect all cacheable requests to L2 slave port. */
	SET_RCISH(base, ctx, 1);
	SET_RCOSH(base, ctx, 1);
	SET_RCNSH(base, ctx, 1);

	/* Turn on TEX Remap */
	SET_TRE(base, ctx, 1);

	/* Set TEX remap attributes */
	RCP15_PRRR(prrr);
	RCP15_NMRR(nmrr);
	SET_PRRR(base, ctx, prrr);
	SET_NMRR(base, ctx, nmrr);

	/* Turn on BFB prefetch */
	SET_BFBDFE(base, ctx, 1);

#ifdef CONFIG_IOMMU_PGTABLES_L2
	/* Configure page tables as inner-cacheable and shareable to reduce
	 * the TLB miss penalty.
	 */
	SET_TTBR0_SH(base, ctx, 1);
	SET_TTBR1_SH(base, ctx, 1);

	SET_TTBR0_NOS(base, ctx, 1);
	SET_TTBR1_NOS(base, ctx, 1);

	SET_TTBR0_IRGNH(base, ctx, 0); /* WB, WA */
	SET_TTBR0_IRGNL(base, ctx, 1);

	SET_TTBR1_IRGNH(base, ctx, 0); /* WB, WA */
	SET_TTBR1_IRGNL(base, ctx, 1);

	SET_TTBR0_ORGN(base, ctx, 1); /* WB, WA */
	SET_TTBR1_ORGN(base, ctx, 1); /* WB, WA */
#endif

	/* Find if this page table is used elsewhere, and re-use ASID */
	found = false;
	for (i = 0; i < iommu->ncb; i++) {
		if (i == ctx)
			continue;

		if (GET_TTBR0_PA(base, i) == (pgtable >> TTBR0_PA_SHIFT)) {
			SET_CONTEXTIDR_ASID(base, ctx, GET_CONTEXTIDR_ASID(base, i));
			found = true;
			break;
		}
	}

	/* If page table is new, find an unused ASID */
	if (!found) {
		for (i = 0; i < iommu->ncb; i++) {
			found = false;
			for (j = 0; j < iommu->ncb; j++) {
				if (j != ctx)
					continue;
				if (GET_CONTEXTIDR_ASID(base, j) == i)
					found = true;
			}

			if (!found) {
				SET_CONTEXTIDR_ASID(base, ctx, i);
				break;
			}
		}
		BUG_ON(found);
	}

	/* Enable the MMU */
	SET_M(base, ctx, 1);
	mb();
}

static int qcom_iommu_domain_init(struct iommu_domain *domain)
{
	struct qcom_domain_priv *priv = kzalloc(sizeof(*priv), GFP_KERNEL);

	if (!priv)
		goto fail_nomem;

	INIT_LIST_HEAD(&priv->iommu_list);
	priv->pgtable = (unsigned long *)__get_free_pages(GFP_KERNEL,
							  get_order(SZ_16K));

	if (!priv->pgtable)
		goto fail_nomem;

	memset(priv->pgtable, 0, SZ_16K);
	domain->priv = priv;
	priv->domain = domain;

//XXX I think not needed?
	dmac_flush_range(priv->pgtable, priv->pgtable + NUM_FL_PTE);

	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end   = (1ULL << 32) - 1;
	domain->geometry.force_aperture = true;

	return 0;

fail_nomem:
	kfree(priv);
	return -ENOMEM;
}

static void qcom_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct qcom_domain_priv *priv;
	unsigned long *fl_table;
	int i;

	mutex_lock(&qcom_iommu_lock);
	priv = domain->priv;
	domain->priv = NULL;

	if (priv) {
		fl_table = priv->pgtable;

		for (i = 0; i < NUM_FL_PTE; i++)
			if ((fl_table[i] & 0x03) == FL_TYPE_TABLE)
				free_page((unsigned long) __va(((fl_table[i]) &
								FL_BASE_MASK)));

		free_pages((unsigned long)priv->pgtable, get_order(SZ_16K));
		priv->pgtable = NULL;
	}

	kfree(priv);
	mutex_unlock(&qcom_iommu_lock);
}

static int qcom_iommu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	struct qcom_domain_priv *priv = domain->priv;
	struct qcom_iommu *iommu;
	struct qcom_iommu_ctx *iommu_ctx = NULL;
	int ret = 0;
	bool found = false;

	if (!priv)
		return -EINVAL;

	mutex_lock(&qcom_iommu_lock);
	list_for_each_entry(iommu, &qcom_iommu_devices, dev_node) {
		iommu_ctx = list_first_entry(&iommu->ctx_list,
				struct qcom_iommu_ctx, node);

		if (iommu_ctx->of_node == dev->of_node) {
			found = true;

			ret = __enable_clocks(iommu);
			if (ret)
				goto fail;

			/* we found a matching device, attach all it's contexts: */
			list_for_each_entry(iommu_ctx, &iommu->ctx_list, node)
				__program_context(priv, iommu, iommu_ctx->num);

			__disable_clocks(iommu);

			// TODO check for double attaches, etc..

			list_add_tail(&iommu->dom_node, &priv->iommu_list);
			iommu->domain = domain;
		}
	}

	if (!found) {
		ret = -ENXIO;
		goto fail;
	}

	// TODO might want to get_device(iommu->dev) unless iommu framework
	// does this somewhere for us?

	ret = __flush_iotlb(priv);

fail:
	if (ret) {
		// TODO make sure we completely detach..
	}
	mutex_unlock(&qcom_iommu_lock);
	return ret;
}

static void qcom_iommu_detach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct qcom_domain_priv *priv = domain->priv;
	struct qcom_iommu *iommu;
	struct qcom_iommu_ctx *iommu_ctx;
	int ret;

	if (!priv)
		return;

	mutex_lock(&qcom_iommu_lock);

	ret = __flush_iotlb(priv);
	if (ret)
		goto fail;

	while (!list_empty(&priv->iommu_list)) {
		iommu = list_first_entry(&priv->iommu_list,
				struct qcom_iommu, dom_node);

		ret = __enable_clocks(iommu);
		if (ret)
			goto fail;

		/* reset all contexts: */
		list_for_each_entry(iommu_ctx, &iommu->ctx_list, node) {
			int ctx = iommu_ctx->num;
			uint32_t asid = GET_CONTEXTIDR_ASID(iommu->base, ctx);
			SET_TLBIASID(iommu->base, ctx, asid);
			__reset_context(iommu, ctx);
		}

		__disable_clocks(iommu);

		list_del(&iommu->dom_node);
	}

	// TODO might want to put_device(iommu->dev) unless iommu framework
	// does this somewhere for us?

fail:
	mutex_unlock(&qcom_iommu_lock);
}

static int __get_pgprot(int prot, int len)
{
	unsigned int pgprot;
	int tex;

	if (!(prot & (IOMMU_READ | IOMMU_WRITE))) {
		prot |= IOMMU_READ | IOMMU_WRITE;
		WARN_ONCE(1, "No attributes in iommu mapping; assuming RW\n");
	}

	if ((prot & IOMMU_WRITE) && !(prot & IOMMU_READ)) {
		prot |= IOMMU_READ;
		WARN_ONCE(1, "Write-only iommu mappings unsupported; falling back to RW\n");
	}

	if (prot & IOMMU_CACHE)
		tex = (pgprot_kernel >> 2) & 0x07;
	else
		tex = qcom_iommu_tex_class[QCOM_IOMMU_ATTR_NONCACHED];

	if (tex < 0 || tex > NUM_TEX_CLASS - 1)
		return 0;

	if (len == SZ_16M || len == SZ_1M) {
		pgprot = FL_SHARED;
		pgprot |= tex & 0x01 ? FL_BUFFERABLE : 0;
		pgprot |= tex & 0x02 ? FL_CACHEABLE : 0;
		pgprot |= tex & 0x04 ? FL_TEX0 : 0;
		pgprot |= FL_AP0 | FL_AP1;
		pgprot |= prot & IOMMU_WRITE ? 0 : FL_AP2;
	} else	{
		pgprot = SL_SHARED;
		pgprot |= tex & 0x01 ? SL_BUFFERABLE : 0;
		pgprot |= tex & 0x02 ? SL_CACHEABLE : 0;
		pgprot |= tex & 0x04 ? SL_TEX0 : 0;
		pgprot |= SL_AP0 | SL_AP1;
		pgprot |= prot & IOMMU_WRITE ? 0 : SL_AP2;
	}

	return pgprot;
}

static int qcom_iommu_map(struct iommu_domain *domain, unsigned long va,
			 phys_addr_t pa, size_t len, int prot)
{
	struct qcom_domain_priv *priv = domain->priv;
	struct qcom_iommu *iommu;
	unsigned long *fl_table, *fl_pte, fl_offset;
	unsigned long *sl_table, *sl_pte, sl_offset;
	unsigned int pgprot;
	int ret = 0;

	mutex_lock(&qcom_iommu_lock);

	if (!priv || list_empty(&priv->iommu_list))
		goto fail;

	/* all IOMMU's in the domain have same pgtables: */
	iommu = list_first_entry(&priv->iommu_list,
			struct qcom_iommu, dom_node);

	fl_table = priv->pgtable;

	if (len != SZ_16M && len != SZ_1M &&
	    len != SZ_64K && len != SZ_4K) {
		dev_err(iommu->dev, "Bad size: %d\n", len);
		ret = -EINVAL;
		goto fail;
	}

	if (!fl_table) {
		dev_err(iommu->dev, "Null page table\n");
		ret = -EINVAL;
		goto fail;
	}

	pgprot = __get_pgprot(prot, len);

	if (!pgprot) {
		ret = -EINVAL;
		goto fail;
	}

	fl_offset = FL_OFFSET(va);	/* Upper 12 bits */
	fl_pte = fl_table + fl_offset;	/* int pointers, 4 bytes */

	if (len == SZ_16M) {
		int i = 0;

		for (i = 0; i < 16; i++)
			if (*(fl_pte+i)) {
				ret = -EBUSY;
				goto fail;
			}

		for (i = 0; i < 16; i++)
			*(fl_pte+i) = (pa & 0xFF000000) | FL_SUPERSECTION
				  | FL_TYPE_SECT | FL_SHARED | FL_NG | pgprot;
		__flush_range(iommu, fl_pte, fl_pte + 16);
	}

	if (len == SZ_1M) {
		if (*fl_pte) {
			ret = -EBUSY;
			goto fail;
		}

		*fl_pte = (pa & 0xFFF00000) | FL_NG | FL_TYPE_SECT | FL_SHARED
					    | pgprot;
		__flush_range(iommu, fl_pte, fl_pte + 1);
	}

	/* Need a 2nd level table */
	if (len == SZ_4K || len == SZ_64K) {

		if (*fl_pte == 0) {
			unsigned long *sl;
			sl = (unsigned long *) __get_free_pages(GFP_ATOMIC,
							get_order(SZ_4K));

			if (!sl) {
				dev_err(iommu->dev, "Could not allocate second level table\n");
				ret = -ENOMEM;
				goto fail;
			}
			memset(sl, 0, SZ_4K);
			__flush_range(iommu, sl, sl + NUM_SL_PTE);

			*fl_pte = ((((int)__pa(sl)) & FL_BASE_MASK) | \
						      FL_TYPE_TABLE);

			__flush_range(iommu, fl_pte, fl_pte + 1);
		}

		if (!(*fl_pte & FL_TYPE_TABLE)) {
			ret = -EBUSY;
			goto fail;
		}
	}

	sl_table = (unsigned long *) __va(((*fl_pte) & FL_BASE_MASK));
	sl_offset = SL_OFFSET(va);
	sl_pte = sl_table + sl_offset;

	if (len == SZ_4K) {
		if (*sl_pte) {
			ret = -EBUSY;
			goto fail;
		}

		*sl_pte = (pa & SL_BASE_MASK_SMALL) | SL_NG | SL_SHARED
						    | SL_TYPE_SMALL | pgprot;
		__flush_range(iommu, sl_pte, sl_pte + 1);
	}

	if (len == SZ_64K) {
		int i;

		for (i = 0; i < 16; i++)
			if (*(sl_pte+i)) {
				ret = -EBUSY;
				goto fail;
			}

		for (i = 0; i < 16; i++)
			*(sl_pte+i) = (pa & SL_BASE_MASK_LARGE) | SL_NG
					  | SL_SHARED | SL_TYPE_LARGE | pgprot;

		__flush_range(iommu, sl_pte, sl_pte + 16);
	}

	ret = __flush_iotlb_va(priv, va);

fail:
	mutex_unlock(&qcom_iommu_lock);
	return ret;
}

static size_t qcom_iommu_unmap(struct iommu_domain *domain, unsigned long va,
			    size_t len)
{
	struct qcom_domain_priv *priv = domain->priv;
	struct qcom_iommu *iommu;
	unsigned long *fl_table, *fl_pte, fl_offset;
	unsigned long *sl_table, *sl_pte, sl_offset;
	int i, ret = 0;

	mutex_lock(&qcom_iommu_lock);

	if (!priv || list_empty(&priv->iommu_list))
		goto fail;

	/* all IOMMU's in the domain have same pgtables: */
	iommu = list_first_entry(&priv->iommu_list,
			struct qcom_iommu, dom_node);

	fl_table = priv->pgtable;

	if (len != SZ_16M && len != SZ_1M &&
	    len != SZ_64K && len != SZ_4K) {
		dev_err(iommu->dev, "Bad length: %d\n", len);
		goto fail;
	}

	if (!fl_table) {
		dev_err(iommu->dev, "Null page table\n");
		goto fail;
	}

	fl_offset = FL_OFFSET(va);	/* Upper 12 bits */
	fl_pte = fl_table + fl_offset;	/* int pointers, 4 bytes */

	if (*fl_pte == 0) {
		dev_err(iommu->dev, "First level PTE is 0\n");
		goto fail;
	}

	/* Unmap supersection */
	if (len == SZ_16M) {
		for (i = 0; i < 16; i++)
			*(fl_pte+i) = 0;

		__flush_range(iommu, fl_pte, fl_pte + 16);
	}

	if (len == SZ_1M) {
		*fl_pte = 0;

		__flush_range(iommu, fl_pte, fl_pte + 1);
	}

	sl_table = (unsigned long *) __va(((*fl_pte) & FL_BASE_MASK));
	sl_offset = SL_OFFSET(va);
	sl_pte = sl_table + sl_offset;

	if (len == SZ_64K) {
		for (i = 0; i < 16; i++)
			*(sl_pte+i) = 0;

		__flush_range(iommu, sl_pte, sl_pte + 16);
	}

	if (len == SZ_4K) {
		*sl_pte = 0;

		__flush_range(iommu, sl_pte, sl_pte + 1);
	}

	if (len == SZ_4K || len == SZ_64K) {
		int used = 0;

		for (i = 0; i < NUM_SL_PTE; i++)
			if (sl_table[i])
				used = 1;
		if (!used) {
			free_page((unsigned long)sl_table);
			*fl_pte = 0;

			__flush_range(iommu, fl_pte, fl_pte + 1);
		}
	}

	ret = __flush_iotlb_va(priv, va);

fail:
	mutex_unlock(&qcom_iommu_lock);

	/* the IOMMU API requires us to return how many bytes were unmapped */
	len = ret ? 0 : len;
	return len;
}

static phys_addr_t qcom_iommu_iova_to_phys(struct iommu_domain *domain,
					  dma_addr_t va)
{
	struct qcom_domain_priv *priv = domain->priv;
	struct qcom_iommu *iommu;
	struct qcom_iommu_ctx *iommu_ctx;
	unsigned int par;
	void __iomem *base;
	phys_addr_t ret = 0;
	int ctx;

	mutex_lock(&qcom_iommu_lock);

	if (!priv || list_empty(&priv->iommu_list))
		goto fail;

	/* all IOMMU's in the domain have same pgtables: */
	iommu = list_first_entry(&priv->iommu_list,
			struct qcom_iommu, dom_node);

	if (list_empty(&iommu->ctx_list))
		goto fail;

	iommu_ctx = list_first_entry(&iommu->ctx_list,
			struct qcom_iommu_ctx, node);

	base = iommu->base;
	ctx = iommu_ctx->num;

	ret = __enable_clocks(iommu);
	if (ret)
		goto fail;

	SET_V2PPR(base, ctx, va & V2Pxx_VA);

	mb();
	par = GET_PAR(base, ctx);

	/* We are dealing with a supersection */
	if (GET_NOFAULT_SS(base, ctx))
		ret = (par & 0xFF000000) | (va & 0x00FFFFFF);
	else	/* Upper 20 bits from PAR, lower 12 from VA */
		ret = (par & 0xFFFFF000) | (va & 0x00000FFF);

	if (GET_FAULT(base, ctx))
		ret = 0;

	__disable_clocks(iommu);

fail:
	mutex_unlock(&qcom_iommu_lock);
	return ret;
}

static void print_ctx_regs(void __iomem *base, int ctx)
{
	unsigned int fsr = GET_FSR(base, ctx);
	pr_err("FAR    = %08x    PAR    = %08x\n",
	       GET_FAR(base, ctx), GET_PAR(base, ctx));
	pr_err("FSR    = %08x [%s%s%s%s%s%s%s%s%s%s]\n", fsr,
			(fsr & 0x02) ? "TF " : "",
			(fsr & 0x04) ? "AFF " : "",
			(fsr & 0x08) ? "APF " : "",
			(fsr & 0x10) ? "TLBMF " : "",
			(fsr & 0x20) ? "HTWDEEF " : "",
			(fsr & 0x40) ? "HTWSEEF " : "",
			(fsr & 0x80) ? "MHF " : "",
			(fsr & 0x10000) ? "SL " : "",
			(fsr & 0x40000000) ? "SS " : "",
			(fsr & 0x80000000) ? "MULTI " : "");

	pr_err("FSYNR0 = %08x    FSYNR1 = %08x\n",
	       GET_FSYNR0(base, ctx), GET_FSYNR1(base, ctx));
	pr_err("TTBR0  = %08x    TTBR1  = %08x\n",
	       GET_TTBR0(base, ctx), GET_TTBR1(base, ctx));
	pr_err("SCTLR  = %08x    ACTLR  = %08x\n",
	       GET_SCTLR(base, ctx), GET_ACTLR(base, ctx));
	pr_err("PRRR   = %08x    NMRR   = %08x\n",
	       GET_PRRR(base, ctx), GET_NMRR(base, ctx));
}

static irqreturn_t __fault_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct qcom_iommu *iommu = platform_get_drvdata(pdev);
	struct qcom_iommu_ctx *iommu_ctx = NULL;
	void __iomem *base = iommu->base;
	int ret;
	bool first = true;

	mutex_lock(&qcom_iommu_lock);

	ret = __enable_clocks(iommu);
	if (ret)
		goto fail;

	list_for_each_entry(iommu_ctx, &iommu->ctx_list, node) {
		int i = iommu_ctx->num;
		unsigned int fsr = GET_FSR(base, i);
		unsigned long iova;
		int flags;

		if (!fsr)
			continue;

		iova = GET_FAR(base, i);

		/* TODO without docs, not sure about IOMMU_FAULT_* flags */
		flags = 0;

		if (!report_iommu_fault(iommu->domain, iommu->dev,
				iova, flags)) {
			ret = IRQ_HANDLED;
		} else {
			// XXX ratelimited
			if (first) {
				/* only print header for first context */
				pr_err("Unexpected IOMMU page fault!\n");
				pr_err("base = %08x\n", (unsigned int) base);
				first = false;
			}
			pr_err("Fault occurred in context %d.\n", i);
			pr_err("name    = %s\n", dev_name(&pdev->dev));
			pr_err("Interesting registers:\n");
			print_ctx_regs(base, i);
		}

		SET_FSR(base, i, fsr);
		SET_RESUME(base, i, 1);
	}
	__disable_clocks(iommu);

fail:
	mutex_unlock(&qcom_iommu_lock);
	return IRQ_HANDLED;
}

static struct iommu_ops qcom_iommu_ops = {
	.domain_init = qcom_iommu_domain_init,
	.domain_destroy = qcom_iommu_domain_destroy,
	.attach_dev = qcom_iommu_attach_dev,
	.detach_dev = qcom_iommu_detach_dev,
	.map = qcom_iommu_map,
	.unmap = qcom_iommu_unmap,
	.iova_to_phys = qcom_iommu_iova_to_phys,
	.pgsize_bitmap = QCOM_IOMMU_PGSIZES,
};

/*
 * IOMMU Platform Driver:
 */

static int __register_ctx(struct qcom_iommu *iommu,
		struct of_phandle_args *masterspec, int ctx)
{
	struct qcom_iommu_ctx *iommu_ctx;
	int i, ret;

	if (masterspec->args_count > MAX_NUM_MIDS)
		return -EINVAL;

	iommu_ctx = kzalloc(sizeof(*iommu_ctx), GFP_KERNEL);
	if (!iommu) {
		ret = -ENOMEM;
		goto fail;
	}

	iommu_ctx->of_node = masterspec->np;
	iommu_ctx->num = ctx;

	for (i = 0; i < masterspec->args_count; i++)
		iommu_ctx->mids[i] = masterspec->args[i];
	for (; i < ARRAY_SIZE(iommu_ctx->mids); i++)
		iommu_ctx->mids[i] = -1;

	list_add_tail(&iommu_ctx->node, &iommu->ctx_list);

	return 0;

fail:
	// TODO cleanup;
	return ret;
}

static int qcom_iommu_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct qcom_iommu *iommu;
	struct qcom_iommu_ctx *iommu_ctx;
	struct of_phandle_args masterspec;
	struct resource *r;
	int ctx, ret, par, i;
	u32 val;

	iommu = kzalloc(sizeof(*iommu), GFP_KERNEL);
	if (!iommu) {
		ret = -ENOMEM;
		goto fail;
	}

	iommu->dev = &pdev->dev;
	INIT_LIST_HEAD(&iommu->ctx_list);

	ret = of_property_read_u32(node, "ncb", &val);
	if (ret) {
		dev_err(iommu->dev, "could not get ncb\n");
		goto fail;
	}
	iommu->ncb = val;

	ret = of_property_read_u32(node, "ttbr-split", &val);
	if (ret)
		val = 0;
	iommu->ttbr_split = val;

	iommu->pclk = devm_clk_get(iommu->dev, "smmu_pclk");
	if (IS_ERR(iommu->pclk)) {
		dev_err(iommu->dev, "could not get smmu_pclk\n");
		ret = PTR_ERR(iommu->pclk);
		iommu->pclk = NULL;
		goto fail;
	}

	iommu->clk = devm_clk_get(iommu->dev, "iommu_clk");
	if (IS_ERR(iommu->clk)) {
		dev_err(iommu->dev, "could not get iommu_clk\n");
		ret = PTR_ERR(iommu->clk);
		iommu->clk = NULL;
		goto fail;
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "physbase");
	if (!r) {
		dev_err(iommu->dev, "could not get physbase\n");
		ret = -ENODEV;
		goto fail;
	}

	iommu->base = devm_ioremap_resource(iommu->dev, r);
	if (IS_ERR(iommu->base)) {
		ret = PTR_ERR(iommu->base);
		goto fail;
	}

	iommu->irq = platform_get_irq_byname(pdev, "nonsecure_irq");
	if (WARN_ON(iommu->irq < 0)) {
		ret = -ENODEV;
		goto fail;
	}

	/* now find our contexts: */
	i = 0;
	while (!of_parse_phandle_with_args(node, "mmu-masters",
			"#stream-id-cells", i, &masterspec)) {
		ret = __register_ctx(iommu, &masterspec, i);
		if (ret) {
			dev_err(iommu->dev, "failed to add context %s\n",
					masterspec.np->name);
			goto fail;
		}

		i++;
	}
	dev_notice(iommu->dev, "registered %d master devices\n", i);

	__enable_clocks(iommu);
	__reset_iommu(iommu);

	SET_M(iommu->base, 0, 1);
	SET_PAR(iommu->base, 0, 0);
	SET_V2PCFG(iommu->base, 0, 1);
	SET_V2PPR(iommu->base, 0, 0);
	mb();
	par = GET_PAR(iommu->base, 0);
	SET_V2PCFG(iommu->base, 0, 0);
	SET_M(iommu->base, 0, 0);
	mb();

	ctx = 0;
	list_for_each_entry(iommu_ctx, &iommu->ctx_list, node) {
		for (i = 0; iommu_ctx->mids[i] != -1; i++) {
			int mid = iommu_ctx->mids[i];

			SET_M2VCBR_N(iommu->base, mid, 0);
			SET_CBACR_N(iommu->base, ctx, 0);

			/* Set VMID = 0 */
			SET_VMID(iommu->base, mid, 0);

			/* Set the context number for that MID to this context */
			SET_CBNDX(iommu->base, mid, ctx);

			/* Set MID associated with this context bank to 0*/
			SET_CBVMID(iommu->base, ctx, 0);

			/* Set the ASID for TLB tagging for this context */
			SET_CONTEXTIDR_ASID(iommu->base, ctx, ctx);

			/* Set security bit override to be Non-secure */
			SET_NSCFG(iommu->base, mid, 3);
		}

		ctx++;
	}

	__disable_clocks(iommu);

	if (!par) {
		dev_err(iommu->dev, "Invalid PAR value detected\n");
		ret = -ENODEV;
		goto fail;
	}

	ret = devm_request_threaded_irq(iommu->dev, iommu->irq, NULL,
			__fault_handler, IRQF_ONESHOT | IRQF_SHARED,
			"iommu", pdev);
	if (ret) {
		dev_err(iommu->dev, "Request IRQ %d failed with ret=%d\n",
				iommu->irq, ret);
		goto fail;
	}

	dev_info(iommu->dev, "device mapped at %p, irq %d with %d ctx banks\n",
			iommu->base, iommu->irq, iommu->ncb);

	platform_set_drvdata(pdev, iommu);

	mutex_lock(&qcom_iommu_lock);
	list_add(&iommu->dev_node, &qcom_iommu_devices);
	mutex_unlock(&qcom_iommu_lock);

	return 0;
fail:
	// TODO cleanup..
	return ret;
}

static int qcom_iommu_remove(struct platform_device *pdev)
{
	struct qcom_iommu *priv = platform_get_drvdata(pdev);

	if (WARN_ON(!priv))
		return 0;

	if (priv->clk)
		clk_disable_unprepare(priv->clk);

	if (priv->pclk)
		clk_disable_unprepare(priv->pclk);

	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static const struct of_device_id qcom_iommu_dt_match[] = {
	{ .compatible = "qcom,iommu-v0" },
	{}
};

static struct platform_driver qcom_iommu_driver = {
	.driver = {
		.name	= "qcom-iommu-v0",
		.of_match_table = qcom_iommu_dt_match,
	},
	.probe		= qcom_iommu_probe,
	.remove		= qcom_iommu_remove,
};

static int __init get_tex_class(int icp, int ocp, int mt, int nos)
{
	int i = 0;
	unsigned int prrr = 0;
	unsigned int nmrr = 0;
	int c_icp, c_ocp, c_mt, c_nos;

	RCP15_PRRR(prrr);
	RCP15_NMRR(nmrr);

	for (i = 0; i < NUM_TEX_CLASS; i++) {
		c_nos = PRRR_NOS(prrr, i);
		c_mt = PRRR_MT(prrr, i);
		c_icp = NMRR_ICP(nmrr, i);
		c_ocp = NMRR_OCP(nmrr, i);

		if (icp == c_icp && ocp == c_ocp && c_mt == mt && c_nos == nos)
			return i;
	}

	return -ENODEV;
}

static int __init qcom_iommu_init(void)
{
	int ret;

	ret = platform_driver_register(&qcom_iommu_driver);
	if (ret) {
		pr_err("Failed to register IOMMU driver\n");
		goto error;
	}

	qcom_iommu_tex_class[QCOM_IOMMU_ATTR_NONCACHED] =
			get_tex_class(CP_NONCACHED, CP_NONCACHED, MT_NORMAL, 1);

	qcom_iommu_tex_class[QCOM_IOMMU_ATTR_CACHED_WB_WA] =
			get_tex_class(CP_WB_WA, CP_WB_WA, MT_NORMAL, 1);

	qcom_iommu_tex_class[QCOM_IOMMU_ATTR_CACHED_WB_NWA] =
			get_tex_class(CP_WB_NWA, CP_WB_NWA, MT_NORMAL, 1);

	qcom_iommu_tex_class[QCOM_IOMMU_ATTR_CACHED_WT] =
			get_tex_class(CP_WT, CP_WT, MT_NORMAL, 1);

	bus_set_iommu(&platform_bus_type, &qcom_iommu_ops);

	return 0;

error:
	return ret;
}

static void __exit qcom_iommu_driver_exit(void)
{
	platform_driver_unregister(&qcom_iommu_driver);
}

subsys_initcall(qcom_iommu_init);
module_exit(qcom_iommu_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Stepan Moskovchenko <stepanm@codeaurora.org>");
MODULE_AUTHOR("Rob Clark <robdclark@gmail.com>");
