/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "scm-pas: " fmt

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>

#include <soc/qcom/scm.h>
#include <asm/cacheflush.h>
#include "scm-pas.h"

#define PAS_INIT_IMAGE_CMD	1
#define PAS_MEM_SETUP_CMD	2
#define PAS_AUTH_AND_RESET_CMD	5
#define PAS_SHUTDOWN_CMD	6
#define PAS_IS_SUPPORTED_CMD	7


int pas_init_image(enum pas_id id, const u8 *metadata, size_t size)
{
	int ret;
	struct pas_init_image_req {
		u32	proc;
		u32	image_addr;
	} request;
	u32 scm_ret = 0;
	void *mdata_buf;
	dma_addr_t mdata_phys;
	DEFINE_DMA_ATTRS(attrs);


	dma_set_attr(DMA_ATTR_WRITE_BARRIER, &attrs);
	mdata_buf = dma_alloc_attrs(NULL, size, &mdata_phys, GFP_KERNEL,
					&attrs);
	if (!mdata_buf) {
		pr_err("Allocation for metadata failed.\n");
		return -ENOMEM;
	}

	memcpy(mdata_buf, metadata, size);

	request.proc = id;
	request.image_addr = mdata_phys;

	ret = scm_call(SCM_SVC_PIL, PAS_INIT_IMAGE_CMD, &request,
			sizeof(request), &scm_ret, sizeof(scm_ret));

	dma_free_attrs(NULL, size, mdata_buf, mdata_phys, &attrs);

	if (ret)
		return ret;
	return scm_ret;
}
EXPORT_SYMBOL(pas_init_image);

int pas_auth_and_reset(enum pas_id id)
{
	int ret;
	u32 proc = id, scm_ret = 0;
	ret = scm_call(SCM_SVC_PIL, PAS_AUTH_AND_RESET_CMD, &proc,
			sizeof(proc), &scm_ret, sizeof(scm_ret));
	if (ret)
		scm_ret = ret;

	return scm_ret;
}
EXPORT_SYMBOL(pas_auth_and_reset);

int pas_shutdown(enum pas_id id)
{
	int ret;
	u32 proc = id, scm_ret = 0;

	ret = scm_call(SCM_SVC_PIL, PAS_SHUTDOWN_CMD, &proc, sizeof(proc),
			&scm_ret, sizeof(scm_ret));
	if (ret)
		return ret;

	return scm_ret;
}
EXPORT_SYMBOL(pas_shutdown);

int pas_supported(enum pas_id id)
{
	int ret;
	u32 periph = id, ret_val = 0;

	if (scm_is_call_available(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD) <= 0)
		return 0;

	ret = scm_call(SCM_SVC_PIL, PAS_IS_SUPPORTED_CMD, &periph,
			sizeof(periph), &ret_val, sizeof(ret_val));
	if (ret)
		return ret;

	return ret_val;
}
EXPORT_SYMBOL(pas_supported);
