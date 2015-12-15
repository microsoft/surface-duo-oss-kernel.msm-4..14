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

#include "msm_smem.h"
#include "msm_internal-buffers.h"
#include "msm_vidc_debug.h"
#include "msm_hfi_interface.h"

static enum hal_buffer
scratch_buf_sufficient(struct vidc_inst *inst, enum hal_buffer buffer_type)
{
	struct hal_buffer_requirements *bufreq;
	struct vidc_internal_buf *buf;
	int count = 0;

	bufreq = get_buff_req_buffer(inst, buffer_type);
	if (!bufreq)
		goto not_sufficient;

	/* Check if current scratch buffers are sufficient */
	mutex_lock(&inst->scratchbufs.lock);
	list_for_each_entry(buf, &inst->scratchbufs.list, list) {
		if (!buf->smem) {
			dprintk(VIDC_ERR, "%s: invalid buf handle\n", __func__);
			mutex_unlock(&inst->scratchbufs.lock);
			goto not_sufficient;
		}

		if (buf->type == buffer_type &&
		    buf->smem->size >= bufreq->size)
			count++;
	}
	mutex_unlock(&inst->scratchbufs.lock);

	if (count != bufreq->count_actual)
		goto not_sufficient;

	dprintk(VIDC_DBG,
		"Existing scratch buffer is sufficient for buffer type %#x\n",
		buffer_type);

	return buffer_type;

not_sufficient:
	return HAL_BUFFER_NONE;
}

static int set_internal_buf_on_fw(struct vidc_inst *inst,
				  enum hal_buffer buffer_type,
				  struct smem *mem, bool reuse)
{
	struct hfi_device *hdev = inst->core->hfidev;
	struct vidc_buffer_addr_info bai = {0};
	int ret;

	ret = smem_cache_operations(inst->mem_client, mem, SMEM_CACHE_CLEAN);
	if (ret)
		dprintk(VIDC_WARN,
			"Failed to clean cache. May cause undefined behavior\n");

	bai.buffer_size = mem->size;
	bai.buffer_type = buffer_type;
	bai.num_buffers = 1;
	bai.device_addr = mem->da;

	ret = call_hfi_op(hdev, session_set_buffers, inst->session, &bai);
	if (ret) {
		dprintk(VIDC_ERR, "session_set_buffers failed\n");
		return ret;
	}

	return 0;
}

static int alloc_and_set(struct vidc_inst *inst,
			 struct hal_buffer_requirements *bufreq,
			 struct vidc_list *buf_list)
{
	struct smem *smem;
	struct vidc_internal_buf *buf;
	u32 smem_flags = 0;
	unsigned int i;
	int ret = 0;

	if (!bufreq->size)
		return 0;

	if (inst->flags & VIDC_SECURE)
		smem_flags |= SMEM_SECURE;

	for (i = 0; i < bufreq->count_actual; i++) {

		smem = smem_alloc(inst->mem_client, bufreq->size, 1, smem_flags,
				  bufreq->type, 0);
		if (IS_ERR(smem)) {
			ret = PTR_ERR(smem);
			goto err_no_mem;
		}

		buf = kzalloc(sizeof(*buf), GFP_KERNEL);
		if (!buf) {
			ret = -ENOMEM;
			goto fail_kzalloc;
		}

		buf->smem = smem;
		buf->type = bufreq->type;

		ret = set_internal_buf_on_fw(inst, bufreq->type, smem, false);
		if (ret)
			goto fail_set_buffers;

		mutex_lock(&buf_list->lock);
		list_add_tail(&buf->list, &buf_list->list);
		mutex_unlock(&buf_list->lock);
	}

	return ret;

fail_set_buffers:
	kfree(buf);
fail_kzalloc:
	smem_free(inst->mem_client, smem);
err_no_mem:
	return ret;
}

static bool
reuse_scratch_buffer(struct vidc_inst *inst, enum hal_buffer buffer_type)
{
	struct vidc_internal_buf *buf;
	bool reused = false;
	int ret = 0;

	mutex_lock(&inst->scratchbufs.lock);
	list_for_each_entry(buf, &inst->scratchbufs.list, list) {
		if (!buf->smem) {
			reused = false;
			break;
		}

		if (buf->type != buffer_type)
			continue;

		ret = set_internal_buf_on_fw(inst, buffer_type, buf->smem,
					     true);
		if (ret) {
			dprintk(VIDC_ERR,
				"%s: session_set_buffers failed\n", __func__);
			reused = false;
			break;
		}

		reused = true;
	}
	mutex_unlock(&inst->scratchbufs.lock);

	return reused;
}

static int set_scratch_buffer(struct vidc_inst *inst, enum hal_buffer type)
{
	struct hal_buffer_requirements *bufreq;

	bufreq = get_buff_req_buffer(inst, type);
	if (!bufreq)
		return 0;

	dprintk(VIDC_DBG, "%s: num:%d, size:%d\n", __func__,
		bufreq->count_actual, bufreq->size);

	/*
	 * Try reusing existing scratch buffers first.
	 * If it's not possible to reuse, allocate new buffers.
	 */
	if (reuse_scratch_buffer(inst, type))
		return 0;

	return alloc_and_set(inst, bufreq, &inst->scratchbufs);
}

static int set_persist_buffer(struct vidc_inst *inst, enum hal_buffer type)
{
	struct hal_buffer_requirements *bufreq;

	bufreq = get_buff_req_buffer(inst, type);
	if (!bufreq)
		return 0;

	dprintk(VIDC_DBG, "persist: num = %d, size = %d\n",
		bufreq->count_actual, bufreq->size);

	mutex_lock(&inst->persistbufs.lock);
	if (!list_empty(&inst->persistbufs.list)) {
		mutex_unlock(&inst->persistbufs.lock);
		return 0;
	}
	mutex_unlock(&inst->persistbufs.lock);

	return alloc_and_set(inst, bufreq, &inst->persistbufs);
}

int release_scratch_buffers(struct vidc_inst *inst, bool reuse)
{
	struct smem *smem;
	struct vidc_internal_buf *buf, *dummy;
	struct vidc_buffer_addr_info bai = {0};
	enum hal_buffer sufficiency = HAL_BUFFER_NONE;
	int ret = 0;

	if (reuse) {
		sufficiency |= scratch_buf_sufficient(inst,
					HAL_BUFFER_INTERNAL_SCRATCH);

		sufficiency |= scratch_buf_sufficient(inst,
					HAL_BUFFER_INTERNAL_SCRATCH_1);

		sufficiency |= scratch_buf_sufficient(inst,
					HAL_BUFFER_INTERNAL_SCRATCH_2);
	}

	mutex_lock(&inst->scratchbufs.lock);
	list_for_each_entry_safe(buf, dummy, &inst->scratchbufs.list, list) {
		if (!buf->smem) {
			ret = -EINVAL;
			goto exit;
		}

		smem = buf->smem;
		bai.buffer_size = smem->size;
		bai.buffer_type = buf->type;
		bai.num_buffers = 1;
		bai.device_addr = smem->da;
		bai.response_required = true;

		mutex_unlock(&inst->scratchbufs.lock);
		ret = hfi_session_release_buffers(inst, &bai);
		mutex_lock(&inst->scratchbufs.lock);

		/* If scratch buffers can be reused, do not free the buffers */
		if (sufficiency & buf->type)
			continue;

		list_del(&buf->list);
		mutex_unlock(&inst->scratchbufs.lock);
		smem_free(inst->mem_client, buf->smem);
		mutex_lock(&inst->scratchbufs.lock);
		kfree(buf);
	}

exit:
	mutex_unlock(&inst->scratchbufs.lock);
	return ret;
}

int release_persist_buffers(struct vidc_inst *inst)
{
	struct smem *smem;
	struct list_head *ptr, *next;
	struct vidc_internal_buf *buf;
	struct vidc_buffer_addr_info bai = {0};
	int ret = 0;

	mutex_lock(&inst->persistbufs.lock);
	list_for_each_safe(ptr, next, &inst->persistbufs.list) {
		buf = list_entry(ptr, struct vidc_internal_buf, list);

		smem = buf->smem;
		bai.buffer_size = smem->size;
		bai.buffer_type = buf->type;
		bai.num_buffers = 1;
		bai.device_addr = smem->da;
		bai.response_required = true;

		mutex_unlock(&inst->persistbufs.lock);
		ret = hfi_session_release_buffers(inst, &bai);
		mutex_lock(&inst->persistbufs.lock);

		list_del(&buf->list);
		mutex_unlock(&inst->persistbufs.lock);
		smem_free(inst->mem_client, buf->smem);
		mutex_lock(&inst->persistbufs.lock);
		kfree(buf);
	}
	mutex_unlock(&inst->persistbufs.lock);

	return ret;
}

int set_scratch_buffers(struct vidc_inst *inst)
{
	int ret;

	ret = release_scratch_buffers(inst, true);
	if (ret)
		dprintk(VIDC_WARN, "Failed to release scratch buffers\n");

	ret = set_scratch_buffer(inst, HAL_BUFFER_INTERNAL_SCRATCH);
	if (ret)
		goto error;

	ret = set_scratch_buffer(inst, HAL_BUFFER_INTERNAL_SCRATCH_1);
	if (ret)
		goto error;

	ret = set_scratch_buffer(inst, HAL_BUFFER_INTERNAL_SCRATCH_2);
	if (ret)
		goto error;

	return 0;
error:
	release_scratch_buffers(inst, false);
	return ret;
}

int set_persist_buffers(struct vidc_inst *inst)
{
	int ret;

	ret = set_persist_buffer(inst, HAL_BUFFER_INTERNAL_PERSIST);
	if (ret)
		goto error;

	ret = set_persist_buffer(inst, HAL_BUFFER_INTERNAL_PERSIST_1);
	if (ret)
		goto error;

	return 0;

error:
	release_persist_buffers(inst);
	return ret;
}
