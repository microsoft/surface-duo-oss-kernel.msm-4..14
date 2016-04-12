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

#define CREATE_TRACE_POINTS

#include "msm_vidc_debug.h"
#include "hfi/vidc_hfi_api.h"

int vidc_debug = VIDC_ERR | VIDC_WARN;
int vidc_debug_out = VIDC_OUT_PRINTK;
int vidc_fw_debug = 0x18;
int vidc_fw_debug_mode = 1;
int vidc_fw_low_power_mode = 1;
int vidc_hw_rsp_timeout = 1000;
bool vidc_fw_coverage = 0;
bool vidc_vpe_csc_601_to_709 = 0;
bool vidc_dec_dcvs_mode = 1;
bool vidc_enc_dcvs_mode = 1;
bool vidc_sys_idle_indicator = 0;
int vidc_firmware_unload_delay = 15000;
bool vidc_thermal_mitigation_disabled = 0;

#define MAX_DBG_BUF_SIZE 4096

struct debug_buffer {
	char ptr[MAX_DBG_BUF_SIZE];
	char *curr;
	u32 filled_size;
};

static struct debug_buffer dbg_buf;

#define INIT_DBG_BUF(__buf) ({ \
	__buf.curr = __buf.ptr;\
	__buf.filled_size = 0; \
})

#define DYNAMIC_BUF_OWNER(__binfo) ({ \
	atomic_read(&__binfo->ref_count) == 2 ? "video driver" : "firmware";\
})

static int core_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static u32 write_str(struct debug_buffer *buffer, const char *fmt, ...)
{
	va_list args;
	u32 size;
	va_start(args, fmt);
	size = vscnprintf(buffer->curr, MAX_DBG_BUF_SIZE - 1, fmt, args);
	va_end(args);
	buffer->curr += size;
	buffer->filled_size += size;
	return size;
}

static ssize_t core_info_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct vidc_core *core = file->private_data;
	struct hfi_device *hdev;
	int i = 0;
	if (!core || !core->hfidev) {
		dprintk(VIDC_ERR, "Invalid params, core: %p\n", core);
		return 0;
	}
	hdev = core->hfidev;
	INIT_DBG_BUF(dbg_buf);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "CORE %d: %p\n", core->id, core);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "state: %d\n", core->state);
	for (i = SYS_MSG_START; i < SYS_MSG_END; i++) {
		write_str(&dbg_buf, "completions[%d]: %s\n", i,
			completion_done(&core->done) ?
			"pending" : "done");
	}
	return simple_read_from_buffer(buf, count, ppos,
			dbg_buf.ptr, dbg_buf.filled_size);
}

static const struct file_operations core_info_fops = {
	.open = core_info_open,
	.read = core_info_read,
};

static int trigger_ssr_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t trigger_ssr_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *ppos) {
	u32 ssr_trigger_val;
	int rc;
	struct vidc_core *core = filp->private_data;
	rc = sscanf(buf, "%d", &ssr_trigger_val);
	if (rc < 0) {
		dprintk(VIDC_WARN, "returning error err %d\n", rc);
		rc = -EINVAL;
	} else {
		vidc_trigger_ssr(core, ssr_trigger_val);
		rc = count;
	}
	return rc;
}

static const struct file_operations ssr_fops = {
	.open = trigger_ssr_open,
	.write = trigger_ssr_write,
};

struct dentry *vidc_debugfs_init_drv(void)
{
	bool ok = false;
	struct dentry *dir = debugfs_create_dir("msm_vidc", NULL);
	if (IS_ERR_OR_NULL(dir)) {
		dir = NULL;
		goto failed_create_dir;
	}

#define __debugfs_create(__type, __name, __value) ({                          \
	struct dentry *f = debugfs_create_##__type(__name, S_IRUGO | S_IWUSR, \
		dir, __value);                                                \
	if (IS_ERR_OR_NULL(f)) {                                              \
		dprintk(VIDC_ERR, "Failed creating debugfs file '%pd/%s'\n",  \
			dir, __name);                                         \
		f = NULL;                                                     \
	}                                                                     \
	f;                                                                    \
})

	ok =
	__debugfs_create(x32, "debug_level", &vidc_debug) &&
	__debugfs_create(x32, "fw_level", &vidc_fw_debug) &&
	__debugfs_create(u32, "fw_debug_mode", &vidc_fw_debug_mode) &&
	__debugfs_create(bool, "fw_coverage", &vidc_fw_coverage) &&
	__debugfs_create(bool, "dcvs_dec_mode", &vidc_dec_dcvs_mode) &&
	__debugfs_create(bool, "dcvs_enc_mode", &vidc_enc_dcvs_mode) &&
	__debugfs_create(u32, "fw_low_power_mode", &vidc_fw_low_power_mode) &&
	__debugfs_create(u32, "debug_output", &vidc_debug_out) &&
	__debugfs_create(u32, "hw_rsp_timeout", &vidc_hw_rsp_timeout) &&
	__debugfs_create(bool, "enable_vpe_csc_601_709",
			&vidc_vpe_csc_601_to_709) &&
	__debugfs_create(bool, "sys_idle_indicator",
			&vidc_sys_idle_indicator) &&
	__debugfs_create(u32, "firmware_unload_delay",
			&vidc_firmware_unload_delay) &&
	__debugfs_create(bool, "disable_thermal_mitigation",
			&vidc_thermal_mitigation_disabled);

#undef __debugfs_create

	if (!ok)
		goto failed_create_dir;

	return dir;

failed_create_dir:
	if (dir)
		debugfs_remove_recursive(vidc_driver->debugfs_root);

	return NULL;
}

struct dentry *vidc_debugfs_init_core(struct vidc_core *core,
				      struct dentry *parent)
{
	struct dentry *dir = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];

	if (!core) {
		dprintk(VIDC_ERR, "Invalid params, core: %p\n", core);
		goto failed_create_dir;
	}

	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "core%d", core->id);
	dir = debugfs_create_dir(debugfs_name, parent);
	if (!dir) {
		dprintk(VIDC_ERR, "Failed to create debugfs for msm_vidc\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("info", S_IRUGO, dir, core, &core_info_fops)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("trigger_ssr", S_IWUSR,
			dir, core, &ssr_fops)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
failed_create_dir:
	return dir;
}

static int inst_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t inst_info_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct vidc_inst *inst = file->private_data;
	int i, j;
	if (!inst) {
		dprintk(VIDC_ERR, "Invalid params, core: %p\n", inst);
		return 0;
	}
	INIT_DBG_BUF(dbg_buf);
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "INSTANCE: %p (%s)\n", inst,
		inst->session_type == VIDC_ENCODER ? "Encoder" : "Decoder");
	write_str(&dbg_buf, "===============================\n");
	write_str(&dbg_buf, "core: %p\n", inst->core);
	write_str(&dbg_buf, "height: %d\n", inst->height);
	write_str(&dbg_buf, "width: %d\n", inst->width);
	write_str(&dbg_buf, "fps: %d\n", inst->fps);
	write_str(&dbg_buf, "state: %d\n", inst->state);
	write_str(&dbg_buf, "secure: %d\n", !!(inst->flags & VIDC_SECURE));
	write_str(&dbg_buf, "-----------Formats-------------\n");
	for (i = 0; i < MAX_PORT_NUM; i++) {
		const struct vidc_format *fmt;

		if (i == OUTPUT_PORT)
			fmt = inst->fmt_out;
		else
			fmt = inst->fmt_cap;

		write_str(&dbg_buf, "capability: %s\n", i == OUTPUT_PORT ?
			"Output" : "Capture");
		write_str(&dbg_buf, "pixformat : %x\n", fmt->pixfmt);
		write_str(&dbg_buf, "planes : %d\n", fmt->num_planes);
		write_str(&dbg_buf, "type: %s\n", fmt->type == OUTPUT_PORT ?
			"Output" : "Capture");
		switch (inst->buffer_mode[i]) {
		case HAL_BUFFER_MODE_STATIC:
			write_str(&dbg_buf, "buffer mode : %s\n", "static");
			break;
		case HAL_BUFFER_MODE_RING:
			write_str(&dbg_buf, "buffer mode : %s\n", "ring");
			break;
		case HAL_BUFFER_MODE_DYNAMIC:
			write_str(&dbg_buf, "buffer mode : %s\n", "dynamic");
			break;
		default:
			write_str(&dbg_buf, "buffer mode : unsupported\n");
		}

		if (i == OUTPUT_PORT)
			write_str(&dbg_buf, "count: %u\n",
				  inst->bufq_out.num_buffers);
		else
			write_str(&dbg_buf, "count: %u\n",
				  inst->bufq_cap.num_buffers);

		for (j = 0; j < fmt->num_planes; j++)
			write_str(&dbg_buf, "size for plane %d: %u\n", j,
				i == 0 ? inst->bufq_out.plane_sizes[j] :
					 inst->bufq_cap.plane_sizes[j]);

		if (i < MAX_PORT_NUM - 1)
			write_str(&dbg_buf, "\n");
	}
	write_str(&dbg_buf, "-------------------------------\n");
	for (i = SESSION_MSG_START; i < SESSION_MSG_END; i++) {
		write_str(&dbg_buf, "completions[%d]: %s\n", i,
		completion_done(&inst->done) ?
		"pending" : "done");
	}
	write_str(&dbg_buf, "ETB Count: %d\n", inst->count.etb);
	write_str(&dbg_buf, "EBD Count: %d\n", inst->count.ebd);
	write_str(&dbg_buf, "FTB Count: %d\n", inst->count.ftb);
	write_str(&dbg_buf, "FBD Count: %d\n", inst->count.fbd);

	return simple_read_from_buffer(buf, count, ppos,
		dbg_buf.ptr, dbg_buf.filled_size);
}

static const struct file_operations inst_info_fops = {
	.open = inst_info_open,
	.read = inst_info_read,
};

struct dentry *vidc_debugfs_init_inst(struct vidc_inst *inst,
				      struct dentry *parent)
{
	struct dentry *dir = NULL;
	char debugfs_name[MAX_DEBUGFS_NAME];
	if (!inst) {
		dprintk(VIDC_ERR, "Invalid params, inst: %p\n", inst);
		goto failed_create_dir;
	}
	snprintf(debugfs_name, MAX_DEBUGFS_NAME, "inst_%p", inst);
	dir = debugfs_create_dir(debugfs_name, parent);
	if (!dir) {
		dprintk(VIDC_ERR, "Failed to create debugfs for msm_vidc\n");
		goto failed_create_dir;
	}
	if (!debugfs_create_file("info", S_IRUGO, dir, inst, &inst_info_fops)) {
		dprintk(VIDC_ERR, "debugfs_create_file: fail\n");
		goto failed_create_dir;
	}
	inst->debug.pdata[FRAME_PROCESSING].sampling = true;
failed_create_dir:
	return dir;
}

void vidc_debugfs_update(struct vidc_inst *inst,
			 enum msm_vidc_debugfs_event e)
{
	struct vidc_debug *d = &inst->debug;
	char a[64] = "Frame processing";
	switch (e) {
	case MSM_VIDC_DEBUGFS_EVENT_ETB:
		mutex_lock(&inst->lock);
		inst->count.etb++;
		mutex_unlock(&inst->lock);
		if (inst->count.ebd && inst->count.ftb > inst->count.fbd) {
			d->pdata[FRAME_PROCESSING].name[0] = '\0';
			tic(inst, FRAME_PROCESSING, a);
		}
	break;
	case MSM_VIDC_DEBUGFS_EVENT_EBD:
		mutex_lock(&inst->lock);
		inst->count.ebd++;
		mutex_unlock(&inst->lock);
		if (inst->count.ebd && inst->count.ebd == inst->count.etb) {
			toc(inst, FRAME_PROCESSING);
			dprintk(VIDC_PROF, "EBD: FW needs input buffers\n");
		}
		if (inst->count.ftb == inst->count.fbd)
			dprintk(VIDC_PROF, "EBD: FW needs output buffers\n");
	break;
	case MSM_VIDC_DEBUGFS_EVENT_FTB: {
		inst->count.ftb++;
		if (inst->count.ebd && inst->count.etb > inst->count.ebd) {
			d->pdata[FRAME_PROCESSING].name[0] = '\0';
			tic(inst, FRAME_PROCESSING, a);
		}
	}
	break;
	case MSM_VIDC_DEBUGFS_EVENT_FBD:
		inst->debug.samples++;
		if (inst->count.ebd && inst->count.fbd == inst->count.ftb) {
			toc(inst, FRAME_PROCESSING);
			dprintk(VIDC_PROF, "FBD: FW needs output buffers\n");
		}
		if (inst->count.etb == inst->count.ebd)
			dprintk(VIDC_PROF, "FBD: FW needs input buffers\n");
		break;
	default:
		dprintk(VIDC_ERR, "Invalid state in debugfs: %d\n", e);
		break;
	}
}

