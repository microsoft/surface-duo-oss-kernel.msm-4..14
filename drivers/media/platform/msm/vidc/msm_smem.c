/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
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

#include <asm/dma-iommu.h>
#include <linux/dma-attrs.h>
#include <linux/dma-direction.h>
#include <linux/iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "msm_smem.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_resources.h"

static int alloc_dma_mem(struct smem_client *client, size_t size, u32 align,
			 u32 flags, enum hal_buffer buffer_type,
			 struct smem *mem, int map_kernel)
{
	int ret;

	if (align > 1) {
		align = ALIGN(align, SZ_4K);
		size = ALIGN(size, align);
	}

	size = ALIGN(size, SZ_4K);

	dprintk(VIDC_DBG, "%s: type: %x, size %zu\n", __func__, buffer_type,
		size);

	mem->flags = flags;
	mem->buffer_type = buffer_type;
	mem->size = size;
	mem->kvaddr = NULL;
	mem->smem_priv = NULL;

	mem->iommu_dev = msm_iommu_get_ctx("venus_ns");
	if (IS_ERR(mem->iommu_dev))
		return PTR_ERR(mem->iommu_dev);

	init_dma_attrs(&mem->attrs);

	if (!map_kernel)
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &mem->attrs);

	mem->kvaddr = dma_alloc_attrs(mem->iommu_dev, size, &mem->da,
				      GFP_KERNEL, &mem->attrs);
	if (!mem->kvaddr)
		return -ENOMEM;

	mem->sgt = kmalloc(sizeof(*mem->sgt), GFP_KERNEL);
	if (!mem->sgt) {
		ret = -ENOMEM;
		goto error_sgt;
	}

	ret = dma_get_sgtable_attrs(mem->iommu_dev, mem->sgt, mem->kvaddr,
				    mem->da, mem->size, &mem->attrs);
	if (ret)
		goto error;

	return 0;
error:
	kfree(mem->sgt);
error_sgt:
	dma_free_attrs(mem->iommu_dev, mem->size, mem->kvaddr,
		       mem->da, &mem->attrs);
	return ret;
}

static void free_dma_mem(struct smem_client *client, struct smem *mem)
{
	dma_free_attrs(mem->iommu_dev, mem->size, mem->kvaddr,
		       mem->da, &mem->attrs);
	kfree(mem->sgt);
}

static int sync_dma_cache(struct smem *mem, enum smem_cache_ops cache_op)
{
	if (cache_op == SMEM_CACHE_CLEAN) {
		dma_sync_sg_for_device(mem->iommu_dev, mem->sgt->sgl,
				       mem->sgt->nents, DMA_TO_DEVICE);
	} else if (cache_op == SMEM_CACHE_INVALIDATE) {
		dma_sync_sg_for_cpu(mem->iommu_dev, mem->sgt->sgl,
				    mem->sgt->nents, DMA_FROM_DEVICE);
	} else {
		dma_sync_sg_for_device(mem->iommu_dev, mem->sgt->sgl,
				       mem->sgt->nents, DMA_TO_DEVICE);
		dma_sync_sg_for_cpu(mem->iommu_dev, mem->sgt->sgl,
				    mem->sgt->nents, DMA_FROM_DEVICE);
	}

	return 0;
}

int smem_cache_operations(struct smem_client *client, struct smem *mem,
			  enum smem_cache_ops cache_op)
{
	if (!client)
		return -EINVAL;

	return sync_dma_cache(mem, cache_op);
}

struct smem_client *smem_new_client(void *platform_resources)
{
	struct vidc_resources *res = platform_resources;
	struct smem_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->clnt = NULL;
	client->res = res;

	return client;
}

void smem_delete_client(struct smem_client *client)
{
	if (!client)
		return;

	kfree(client);
}

struct smem *smem_alloc(struct smem_client *client, size_t size, u32 align,
			u32 flags, enum hal_buffer buffer_type, int map_kernel)
{
	struct smem *mem;
	int ret;

	if (!client || !size)
		return ERR_PTR(-EINVAL);

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return ERR_PTR(-ENOMEM);

	ret = alloc_dma_mem(client, size, align, flags, buffer_type,
			    mem, map_kernel);
	if (ret) {
		kfree(mem);
		return ERR_PTR(ret);
	}

	return mem;
}

void smem_free(struct smem_client *client, struct smem *mem)
{
	if (!client || !mem)
		return;

	free_dma_mem(client, mem);
	kfree(mem);
};

struct context_bank_info *
smem_get_context_bank(struct smem_client *client, bool is_secure,
		      enum hal_buffer buffer_type)
{
	struct context_bank_info *cb = NULL, *match = NULL;

	if (!client) {
		dprintk(VIDC_ERR, "%s - invalid params\n", __func__);
		return NULL;
	}

	list_for_each_entry(cb, &client->res->context_banks, list) {
		if (cb->is_secure == is_secure &&
		    cb->buffer_type & buffer_type) {
			match = cb;
			break;
		}
	}

	return match;
}
