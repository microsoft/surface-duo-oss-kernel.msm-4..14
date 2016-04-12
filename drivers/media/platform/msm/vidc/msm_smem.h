/*
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Linaro Ltd.
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

#ifndef __VIDC_MSM_SMEM_H__
#define __VIDC_MSM_SMEM_H__

#include <linux/dma-attrs.h>
#include <linux/scatterlist.h>

#include "msm_vidc_resources.h"
#include "msm_vidc.h"
#include "hfi/vidc_hfi_api.h"

enum smem_prop {
	SMEM_CACHED = BIT(0),
	SMEM_SECURE = BIT(1),
};

enum smem_cache_ops {
	SMEM_CACHE_CLEAN,
	SMEM_CACHE_INVALIDATE,
	SMEM_CACHE_CLEAN_INVALIDATE,
};

struct smem {
	size_t size;
	void *kvaddr;
	dma_addr_t da;
	unsigned long flags;
	void *smem_priv;
	enum hal_buffer buffer_type;
	struct dma_attrs attrs;
	struct device *iommu_dev;
	struct sg_table *sgt;
};

struct smem_client {
	void *clnt;
	struct vidc_resources *res;
};

struct smem_client *smem_new_client(void *platform_resources);
void smem_delete_client(struct smem_client *clt);

struct smem *smem_alloc(struct smem_client *clt, size_t size, u32 align,
			u32 flags, enum hal_buffer buffer_type, int map_kernel);
void smem_free(struct smem_client *clt, struct smem *mem);

int smem_cache_operations(struct smem_client *clt, struct smem *mem,
			  enum smem_cache_ops);
struct context_bank_info *
smem_get_context_bank(struct smem_client *clt, bool is_secure,
		      enum hal_buffer buffer_type);

#endif /* __VIDC_MSM_SMEM_H__ */
