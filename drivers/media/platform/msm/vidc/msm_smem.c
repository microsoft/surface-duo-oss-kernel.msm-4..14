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
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/iommu.h>
#include <linux/msm_iommu_domains.h>
#ifdef CONFIG_ION
#include <linux/msm_ion.h>
#endif
#include <linux/slab.h>
#include <linux/types.h>
#include <media/msm_vidc.h>
#include <media/videobuf2-dma-contig.h>
#include "msm_vidc_debug.h"
#include "msm_vidc_resources.h"

struct smem_client {
	int mem_type;
	void *clnt;
	struct msm_vidc_platform_resources *res;
};

static int get_device_address(struct smem_client *smem_client,
		struct dma_buf *buf, dma_addr_t *iova,
		unsigned long *buffer_size,
		unsigned long flags, struct context_bank_info *cb,
		struct dma_mapping_info *mapping_info)
{
	int rc = 0;
	struct dma_buf_attachment *attach;
	struct sg_table *table = NULL;
	phys_addr_t phys, orig_phys;

	if (!iova || !buffer_size || !buf || !smem_client || !mapping_info || !cb) {
		dprintk(VIDC_ERR, "Invalid params: %p, %p, %p, %p, %p\n",
				smem_client, buf, iova, buffer_size, cb);
		return -EINVAL;
	}


	/* Prepare a dma buf for dma on the given device */
	attach = dma_buf_attach(buf, cb->dev);
	if (IS_ERR_OR_NULL(attach)) {
		rc = PTR_ERR(attach) ?: -ENOMEM;
		dprintk(VIDC_ERR, "Failed to attach dmabuf\n");
		goto mem_buf_attach_failed;
	}

	/* Get the scatterlist for the given attachment */
	table = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR_OR_NULL(table)) {
		rc = PTR_ERR(table) ?: -ENOMEM;
		dprintk(VIDC_ERR, "Failed to map table\n");
		goto mem_map_table_failed;
	}

	if (table->sgl) {
		dprintk(VIDC_DBG,
				"%s: DMA buf: %p, device: %p, attach: %p, table: %p, table sgl: %p, rc: %d, dma_address: %pa\n",
				__func__, buf, cb->dev, attach,
				table, table->sgl, rc,
				&table->sgl->dma_address);

		*iova = table->sgl->dma_address;
		*buffer_size = table->sgl->dma_length;
	} else {
		dprintk(VIDC_ERR, "sgl is NULL\n");
		rc = -ENOMEM;
		goto mem_map_sg_failed;
	}

	/* Translation check for debugging */
	orig_phys = sg_phys(table->sgl);

	phys = iommu_iova_to_phys(cb->mapping->domain, *iova);
	if (phys != orig_phys) {
		dprintk(VIDC_ERR,
				"%s iova_to_phys failed!!! mapped: %pa, got: %pa\n",
				__func__, &orig_phys, &phys);
		rc = -EIO;
		goto mem_iova_to_phys_failed;
	}

	mapping_info->dev = cb->dev;
	mapping_info->mapping = cb->mapping;
	mapping_info->table = table;
	mapping_info->attach = attach;
	mapping_info->buf = buf;

	dprintk(VIDC_DBG, "mapped dma buf %p to %pa\n", buf, iova);
	return 0;
mem_iova_to_phys_failed:
	dma_unmap_sg(cb->dev, table->sgl, table->nents, DMA_BIDIRECTIONAL);
mem_map_sg_failed:
	dma_buf_unmap_attachment(attach, table, DMA_BIDIRECTIONAL);
mem_map_table_failed:
	dma_buf_detach(buf, attach);
mem_buf_attach_failed:
	return rc;
}

static void put_device_address(struct smem_client *smem_client,
	u32 flags, struct dma_mapping_info *mapping_info,
	enum hal_buffer buffer_type)
{
	if (!smem_client || !mapping_info) {
		dprintk(VIDC_WARN, "Invalid params: %p, %p\n",
				smem_client, mapping_info);
		return;
	}

	if (!mapping_info->dev || !mapping_info->table ||
		!mapping_info->buf || !mapping_info->attach) {
			dprintk(VIDC_WARN, "Invalid params:\n");
			return;
	}

	if (is_iommu_present(smem_client->res)) {
		dprintk(VIDC_DBG,
			"Calling dma_unmap_sg - device: %p, address: %pa, buf: %p, table: %p, attach: %p\n",
			mapping_info->dev,
			&mapping_info->table->sgl->dma_address,
			mapping_info->buf, mapping_info->table,
			mapping_info->attach);

		dma_buf_unmap_attachment(mapping_info->attach,
			mapping_info->table, DMA_BIDIRECTIONAL);
		dma_buf_detach(mapping_info->buf, mapping_info->attach);
		dma_buf_put(mapping_info->buf);
	}
}

#ifdef CONFIG_MSM_VIDC_USES_ION
static int ion_get_device_address(struct smem_client *smem_client,
		struct ion_handle *hndl, unsigned long align,
		ion_phys_addr_t *iova, unsigned long *buffer_size,
		unsigned long flags, enum hal_buffer buffer_type,
		struct dma_mapping_info *mapping_info)
{
	int rc = 0;
	struct dma_buf *buf = NULL;
	struct context_bank_info *cb = NULL;
	struct ion_client *clnt = smem_client->clnt;
	if (!clnt) {
		dprintk(VIDC_ERR, "Invalid client\n");
		return -EINVAL;
	}
	cb = msm_smem_get_context_bank(smem_client, flags & SMEM_SECURE,
			buffer_type);
	if (!cb) {
		dprintk(VIDC_ERR,
				"%s: Failed to get context bank device\n",
				__func__);
		return -EIO;
	}
	if (is_iommu_present(smem_client->res)) {
		/* Convert an Ion handle to a dma buf */
		buf = ion_share_dma_buf(clnt, hndl);
		if (IS_ERR_OR_NULL(buf)) {
			rc = PTR_ERR(buf) ?: -ENOMEM;
			dprintk(VIDC_ERR, "Share ION buf to DMA failed\n");
			goto mem_map_failed;
		}
		rc = get_device_address(smem_client, buf, align, iova, buffer_size, flags, cb, mapping_info);
		if (rc) {
			dprintk(VIDC_ERR, "ion iommu map failed - %d\n", rc);
			goto mem_map_failed;
		}
	} else {
		dprintk(VIDC_DBG, "Using physical memory address\n");
		rc = ion_phys(clnt, hndl, iova, (size_t *)buffer_size);
		if (rc) {
			dprintk(VIDC_ERR, "ion memory map failed - %d\n", rc);
			goto mem_map_failed;
		}
	}
mem_map_failed:
	return rc;
}

static int ion_user_to_kernel(struct smem_client *client, int fd, u32 offset,
		struct msm_smem *mem, enum hal_buffer buffer_type)
{
	struct ion_handle *hndl;
	ion_phys_addr_t iova = 0;
	unsigned long buffer_size = 0;
	int rc = 0;
	unsigned long align = SZ_4K;

	hndl = ion_import_dma_buf(client->clnt, fd);
	dprintk(VIDC_DBG, "%s ion handle: %p\n", __func__, hndl);
	if (IS_ERR_OR_NULL(hndl)) {
		dprintk(VIDC_ERR, "Failed to get handle: %p, %d, %d, %p\n",
				client, fd, offset, hndl);
		rc = -ENOMEM;
		goto fail_import_fd;
	}
	mem->kvaddr = NULL;
	rc = ion_handle_get_flags(client->clnt, hndl, &mem->flags);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get ion flags: %d\n", rc);
		goto fail_device_address;
	}

	mem->buffer_type = buffer_type;
	if (mem->flags & SMEM_SECURE)
		align = ALIGN(align, SZ_1M);

	rc = ion_get_device_address(client, hndl, align, &iova, &buffer_size,
				mem->flags, buffer_type, &mem->mapping_info);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get device address: %d\n", rc);
		goto fail_device_address;
	}

	mem->mem_type = client->mem_type;
	mem->smem_priv = hndl;
	mem->device_addr = iova;
	mem->size = buffer_size;
	if ((u32)mem->device_addr != iova) {
		dprintk(VIDC_ERR, "iova(%pa) truncated to %#x",
			&iova, (u32)mem->device_addr);
		goto fail_device_address;
	}
	dprintk(VIDC_DBG,
		"%s: ion_handle = %p, fd = %d, device_addr = %pa, size = %zx, kvaddr = %p, buffer_type = %d, flags = %#lx\n",
		__func__, mem->smem_priv, fd, &mem->device_addr, mem->size,
		mem->kvaddr, mem->buffer_type, mem->flags);
	return rc;
fail_device_address:
	ion_free(client->clnt, hndl);
fail_import_fd:
	return rc;
}

static int alloc_ion_mem(struct smem_client *client, size_t size, u32 align,
	u32 flags, enum hal_buffer buffer_type, struct msm_smem *mem,
	int map_kernel)
{
	struct ion_handle *hndl;
	ion_phys_addr_t iova = 0;
	unsigned long buffer_size = 0;
	unsigned long heap_mask = 0;
	int rc = 0;

	align = ALIGN(align, SZ_4K);
	size = ALIGN(size, SZ_4K);

	if (flags & SMEM_SECURE) {
		size = ALIGN(size, SZ_1M);
		align = ALIGN(align, SZ_1M);
		flags |= ION_FLAG_ALLOW_NON_CONTIG;
	}

	if (is_iommu_present(client->res)) {
		heap_mask = ION_HEAP(ION_IOMMU_HEAP_ID);
	} else {
		dprintk(VIDC_DBG,
			"allocate shared memory from adsp heap size %zx align %d\n",
			size, align);
		heap_mask = ION_HEAP(ION_ADSP_HEAP_ID);
	}

	if (flags & SMEM_SECURE)
		heap_mask = ION_HEAP(ION_CP_MM_HEAP_ID);

	trace_msm_smem_buffer_ion_op_start("ALLOC", (u32)buffer_type,
		heap_mask, size, align, flags, map_kernel);
	hndl = ion_alloc(client->clnt, size, align, heap_mask, flags);
	if (IS_ERR_OR_NULL(hndl)) {
		dprintk(VIDC_ERR,
		"Failed to allocate shared memory = %p, %zx, %d, %#x\n",
		client, size, align, flags);
		rc = -ENOMEM;
		goto fail_shared_mem_alloc;
	}
	trace_msm_smem_buffer_ion_op_end("ALLOC", (u32)buffer_type,
		heap_mask, size, align, flags, map_kernel);
	mem->mem_type = client->mem_type;
	mem->smem_priv = hndl;
	mem->flags = flags;
	mem->buffer_type = buffer_type;
	if (map_kernel) {
		mem->kvaddr = ion_map_kernel(client->clnt, hndl);
		if (IS_ERR_OR_NULL(mem->kvaddr)) {
			dprintk(VIDC_ERR,
				"Failed to map shared mem in kernel\n");
			rc = -EIO;
			goto fail_map;
		}
	} else {
		mem->kvaddr = NULL;
	}

	rc = ion_get_device_address(client, hndl, align, &iova, &buffer_size,
				flags, buffer_type, &mem->mapping_info);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get device address: %d\n",
			rc);
		goto fail_device_address;
	}
	mem->device_addr = iova;
	if ((u32)mem->device_addr != iova) {
		dprintk(VIDC_ERR, "iova(%pa) truncated to %#x",
			&iova, (u32)mem->device_addr);
		goto fail_device_address;
	}
	mem->size = size;
	dprintk(VIDC_DBG,
		"%s: ion_handle = %p, device_addr = %pa, size = %#zx, kvaddr = %p, buffer_type = %#x, flags = %#lx\n",
		__func__, mem->smem_priv, &mem->device_addr,
		mem->size, mem->kvaddr, mem->buffer_type, mem->flags);
	return rc;
fail_device_address:
	if (mem->kvaddr)
		ion_unmap_kernel(client->clnt, hndl);
fail_map:
	ion_free(client->clnt, hndl);
fail_shared_mem_alloc:
	return rc;
}

static void free_ion_mem(struct smem_client *client, struct msm_smem *mem)
{
	dprintk(VIDC_DBG,
		"%s: ion_handle = %p, device_addr = %pa, size = %#zx, kvaddr = %p, buffer_type = %#x\n",
		__func__, mem->smem_priv, &mem->device_addr,
		mem->size, mem->kvaddr, mem->buffer_type);

	if (mem->device_addr)
		put_device_address(client, mem->flags,
			&mem->mapping_info, mem->buffer_type);

	if (mem->kvaddr)
		ion_unmap_kernel(client->clnt, mem->smem_priv);
	if (mem->smem_priv) {
		trace_msm_smem_buffer_ion_op_start("FREE",
				(u32)mem->buffer_type, -1, mem->size, -1,
				mem->flags, -1);
		dprintk(VIDC_DBG,
			"%s: Freeing handle %p, client: %p\n",
			__func__, mem->smem_priv, client->clnt);
		ion_free(client->clnt, mem->smem_priv);
		trace_msm_smem_buffer_ion_op_end("FREE", (u32)mem->buffer_type,
			-1, mem->size, -1, mem->flags, -1);
	}
}

static void *ion_new_client(void)
{
	struct ion_client *client = NULL;
	client = msm_ion_client_create("video_client");
	if (!client)
		dprintk(VIDC_ERR, "Failed to create smem client\n");
	return client;
};

static void ion_delete_client(struct smem_client *client)
{
	ion_client_destroy(client->clnt);
}

static int ion_cache_operations(struct smem_client *client,
	struct msm_smem *mem, enum smem_cache_ops cache_op)
{
	unsigned long ionflag = 0;
	int rc = 0;
	int msm_cache_ops = 0;
	if (!mem || !client) {
		dprintk(VIDC_ERR, "Invalid params: %p, %p\n",
			mem, client);
		return -EINVAL;
	}
	rc = ion_handle_get_flags(client->clnt,	mem->smem_priv,
		&ionflag);
	if (rc) {
		dprintk(VIDC_ERR,
			"ion_handle_get_flags failed: %d\n", rc);
		goto cache_op_failed;
	}
	if (ION_IS_CACHED(ionflag)) {
		switch (cache_op) {
		case SMEM_CACHE_CLEAN:
			msm_cache_ops = ION_IOC_CLEAN_CACHES;
			break;
		case SMEM_CACHE_INVALIDATE:
			msm_cache_ops = ION_IOC_INV_CACHES;
			break;
		case SMEM_CACHE_CLEAN_INVALIDATE:
			msm_cache_ops = ION_IOC_CLEAN_INV_CACHES;
			break;
		default:
			dprintk(VIDC_ERR, "cache operation not supported\n");
			rc = -EINVAL;
			goto cache_op_failed;
		}
		rc = msm_ion_do_cache_op(client->clnt,
				(struct ion_handle *)mem->smem_priv,
				0, (unsigned long)mem->size,
				msm_cache_ops);
		if (rc) {
			dprintk(VIDC_ERR,
					"cache operation failed %d\n", rc);
			goto cache_op_failed;
		}
	}
cache_op_failed:
	return rc;
}
#endif

static int alloc_dma_mem(struct smem_client *client, size_t size, u32 align,
	u32 flags, enum hal_buffer buffer_type, struct msm_smem *mem,
	int map_kernel)
{
	int rc = 0;
	enum dma_data_direction dma_dir;
	unsigned long buffer_size = 0;
	struct context_bank_info *cb = NULL;

	align = ALIGN(align, SZ_4K);
	size = ALIGN(size, SZ_4K);
	size = PAGE_ALIGN(size);

	dprintk(VIDC_DBG, "alloc_dma_mem type %d, size %zu\n", buffer_type, size);

	switch (buffer_type) {
	case HAL_BUFFER_INPUT:
		dma_dir = DMA_TO_DEVICE;
		break;
	case HAL_BUFFER_OUTPUT:
		dma_dir = DMA_FROM_DEVICE;
		break;
	default:
		dma_dir = DMA_FROM_DEVICE;
		break;
	}

	mem->mem_type = client->mem_type;
	mem->flags = flags;
	mem->buffer_type = buffer_type;
	mem->kvaddr = NULL;
	mem->size = size;

	cb = msm_smem_get_context_bank(client, flags & SMEM_SECURE,
			buffer_type);
	if (!cb) {
		dprintk(VIDC_ERR,
				"%s: Failed to get context bank device\n",
				__func__);
		return -EIO;
	}

	mem->smem_priv = vb2_dma_contig_memops.alloc(cb->alloc_ctx, size, dma_dir, 0);
	if (IS_ERR_OR_NULL(mem->smem_priv)) {
		dprintk(VIDC_ERR, "VB2 contig memops alloc failed %zu\n", size);
		rc = PTR_ERR(mem->smem_priv) ?: -ENOMEM;
		goto err_dma_alloc;
	}

	dprintk(VIDC_DBG, "alloc_dma_mem, map_kernel\n");
	mem->kvaddr = vb2_dma_contig_memops.vaddr(mem->smem_priv);
	if (!mem->kvaddr) {
		dprintk(VIDC_WARN, "Failed to map dma buf %pad\n", &mem->device_addr);
		rc = -ENOMEM;
		goto err_put;
	}

	//rc = dma_get_sgtable(&res->pdev->dev, &mem->mapping_info.table, mem->kvaddr, mem->device_addr, size);
	rc = get_device_address(client, vb2_dma_contig_memops.get_dmabuf(mem->smem_priv, O_CLOEXEC),
			&mem->device_addr, &buffer_size, flags, cb, &mem->mapping_info);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get iova for dma buf %pad\n",
				vb2_dma_contig_memops.cookie(mem->smem_priv));
		goto err_put;
	}

	dprintk(VIDC_DBG,
			"%s: dma_handle = %p, device_addr = %pad, size = %#zx, kvaddr = %p, buffer_type = %#x, flags = %#lx\n",
			__func__, mem->smem_priv, &mem->device_addr,
			mem->size, mem->kvaddr, mem->buffer_type, mem->flags);

	dprintk(VIDC_DBG,
		"%s: sg_handle = 0x%p, device_addr = 0x%x, size = %zu, kvaddr = 0x%p, buffer_type = %d\n",
		__func__, mem->smem_priv, (u32)mem->device_addr,
		mem->size, mem->kvaddr, mem->buffer_type);
	return rc;
err_put:
	vb2_dma_contig_memops.put(mem->smem_priv);
err_dma_alloc:
	mem->smem_priv = NULL;
	return rc;
}

static void free_dma_mem(struct smem_client *client, struct msm_smem *mem)
{
	dprintk(VIDC_DBG,
		"%s: mem priv = 0x%p, device_addr = 0x%pa, size = 0x%zx, kvaddr = 0x%p, buffer_type = 0x%x\n",
		__func__, mem->smem_priv, &mem->device_addr,
		mem->size, mem->kvaddr, mem->buffer_type);
	if (mem->device_addr)
		put_device_address(client, mem->flags,
			&mem->mapping_info, mem->buffer_type);
	if (mem->smem_priv) {
		vb2_dma_contig_memops.put(mem->smem_priv);
		mem->smem_priv = NULL;
	}
}

struct msm_smem *msm_smem_user_to_kernel(void *clt, int fd, u32 offset,
		enum hal_buffer buffer_type)
{
	struct smem_client *client = clt;
	int rc = 0;
	struct msm_smem *mem;
	if (fd < 0) {
		dprintk(VIDC_ERR, "Invalid fd: %d\n", fd);
		return NULL;
	}
	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		dprintk(VIDC_ERR, "Failed to allocte shared mem\n");
		return NULL;
	}
	switch (client->mem_type) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		rc = ion_user_to_kernel(clt, fd, offset, mem, buffer_type);
		break;
#endif
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		rc = -EINVAL;
		break;
	}
	if (rc) {
		dprintk(VIDC_ERR, "Failed to allocate shared memory\n");
		kfree(mem);
		mem = NULL;
	}
	return mem;
}

int msm_smem_cache_operations(void *clt, struct msm_smem *mem,
		enum smem_cache_ops cache_op)
{
	struct smem_client *client = clt;
	int rc = 0;
	if (!client) {
		dprintk(VIDC_ERR, "Invalid params: %p\n",
			client);
		return -EINVAL;
	}
	switch (client->mem_type) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		rc = ion_cache_operations(client, mem, cache_op);
		if (rc)
			dprintk(VIDC_ERR,
			"Failed cache operations: %d\n", rc);
		break;
#endif
	case SMEM_DMA:
		dprintk(VIDC_DBG,
			"Ignore dma cache operations: %d\n", rc);
		break;
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		break;
	}
	return rc;
}

void *msm_smem_new_client(enum smem_type mtype,
		void *platform_resources)
{
	struct smem_client *client = NULL;
	void *clnt = NULL;
	struct msm_vidc_platform_resources *res = platform_resources;
	switch (mtype) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		clnt = ion_new_client();
		break;
#endif
	case SMEM_DMA:
		clnt = vb2_dma_contig_init_ctx(&(res->pdev->dev));
		break;
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		break;
	}
	if (clnt) {
		client = kzalloc(sizeof(*client), GFP_KERNEL);
		if (client) {
			client->mem_type = mtype;
			client->clnt = clnt;
			client->res = res;
		}
	} else {
		dprintk(VIDC_ERR, "Failed to create new client: mtype = %d\n",
			mtype);
	}
	return client;
}

struct msm_smem *msm_smem_alloc(void *clt, size_t size, u32 align, u32 flags,
		enum hal_buffer buffer_type, int map_kernel)
{
	struct smem_client *client;
	int rc = 0;
	struct msm_smem *mem;
	client = clt;
	if (!client) {
		dprintk(VIDC_ERR, "Invalid  client passed\n");
		return NULL;
	}
	if (!size) {
		dprintk(VIDC_ERR, "No need to allocate memory of size: %zx\n",
			size);
		return NULL;
	}
	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		dprintk(VIDC_ERR, "Failed to allocate shared mem\n");
		return NULL;
	}
	switch (client->mem_type) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		rc = alloc_ion_mem(client, size, align, flags, buffer_type,
					mem, map_kernel);
		break;
#endif
	case SMEM_DMA:
		rc = alloc_dma_mem(client, size, align, flags, buffer_type,
					mem, map_kernel);
		break;
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		rc = -EINVAL;
		break;
	}
	if (rc) {
		dprintk(VIDC_ERR, "Failed to allocate shared memory\n");
		kfree(mem);
		mem = NULL;
	}
	return mem;
}

void msm_smem_free(void *clt, struct msm_smem *mem)
{
	struct smem_client *client = clt;
	if (!client || !mem) {
		dprintk(VIDC_ERR, "Invalid  client/handle passed\n");
		return;
	}
	switch (client->mem_type) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		free_ion_mem(client, mem);
		break;
#endif
	case SMEM_DMA:
		free_dma_mem(client, mem);
		break;
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		break;
	}
	kfree(mem);
};

void msm_smem_delete_client(void *clt)
{
	struct smem_client *client = clt;
	if (!client) {
		dprintk(VIDC_ERR, "Invalid  client passed\n");
		return;
	}
	switch (client->mem_type) {
#ifdef CONFIG_MSM_VIDC_USES_ION
	case SMEM_ION:
		ion_delete_client(client);
		break;
#endif
	case SMEM_DMA:
		vb2_dma_contig_cleanup_ctx(client->clnt);
		break;
	default:
		dprintk(VIDC_ERR, "Mem type not supported\n");
		break;
	}
	kfree(client);
}

struct context_bank_info *msm_smem_get_context_bank(void *clt,
			bool is_secure, enum hal_buffer buffer_type)
{
	struct smem_client *client = clt;
	struct context_bank_info *cb = NULL, *match = NULL;

	if (!clt) {
		dprintk(VIDC_ERR, "%s - invalid params\n", __func__);
		return NULL;
	}

	list_for_each_entry(cb, &client->res->context_banks, list) {
		if (cb->is_secure == is_secure &&
				cb->buffer_type & buffer_type) {
			match = cb;
			dprintk(VIDC_DBG,
				"context bank found for CB : %s, device: %p mapping: %p\n",
				match->name, match->dev, match->mapping);
			break;
		}
	}

	return match;
}

void *msm_smem_get_alloc_ctx(void *mem_client)
{
	struct smem_client *client = mem_client;
	return client->clnt;
}


struct msm_smem* msm_smem_map_dma_buf(void* smem_client, struct dma_buf* dbuf,
		enum hal_buffer buffer_type)
{
	struct smem_client *client = smem_client;
	int rc = 0;
	struct msm_smem *mem;
	unsigned long buffer_size = 0;
	u32 flags = 0;
	struct context_bank_info *cb = NULL;

	dprintk(VIDC_DBG, "msm_smem_map_dma_buf type %d\n", buffer_type);

	if (client == NULL || dbuf == NULL) {
		dprintk(VIDC_ERR, "%s: Invalid  params \n", __func__);
		return NULL;
	}

	if (is_iommu_present(client->res)) {
		cb = msm_smem_get_context_bank(smem_client, flags & SMEM_SECURE,
				buffer_type);
		if (!cb) {
			dprintk(VIDC_ERR,
					"%s: Failed to get context bank device\n",
					__func__);
			return NULL;
		}
	}

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem) {
		dprintk(VIDC_ERR, "Failed to allocte shared mem\n");
		return NULL;
	}

	rc = get_device_address(client, dbuf, &mem->device_addr, &buffer_size,
			flags, cb, &mem->mapping_info);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to get iova for dma buf %pad\n", dbuf);
		goto err_get_device_address;
	}
	mem->mem_type = client->mem_type;
	mem->smem_priv = NULL;
	mem->size = buffer_size;
	mem->kvaddr = NULL;
	mem->buffer_type = buffer_type;

	return mem;
err_get_device_address:
	kfree(mem);
	mem = NULL;
	return ERR_PTR(rc);
}
