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

#ifndef _MSM_VIDC_INTERNAL_H_
#define _MSM_VIDC_INTERNAL_H_

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-core.h>

#include "msm_media_info.h"
#include "hfi/vidc_hfi_api.h"

#define VIDC_DRV_NAME		"vidc"
#define VIDC_VERSION		KERNEL_VERSION(1, 0, 0);
#define MAX_DEBUGFS_NAME	50
#define DEFAULT_TIMEOUT		3
#define DEFAULT_HEIGHT		1088
#define DEFAULT_WIDTH		1920
#define MIN_SUPPORTED_WIDTH	32
#define MIN_SUPPORTED_HEIGHT	32

/* Maintains the number of FTB's between each FBD over a window */
#define DCVS_FTB_WINDOW 32

#define V4L2_EVENT_VIDC_BASE  10

#define SYS_MSG_START			SYS_EVENT_CHANGE
#define SYS_MSG_END			SYS_DEBUG
#define SESSION_MSG_START		SESSION_LOAD_RESOURCE_DONE
#define SESSION_MSG_END			SESSION_PROPERTY_INFO
#define SYS_MSG_INDEX(__msg)		(__msg - SYS_MSG_START)
#define SESSION_MSG_INDEX(__msg)	(__msg - SESSION_MSG_START)

#define MAX_NAME_LENGTH			64

#define EXTRADATA_IDX(__num_planes) ((__num_planes) ? (__num_planes) - 1 : 0)

enum vidc_ports {
	OUTPUT_PORT,
	CAPTURE_PORT,
	MAX_PORT_NUM
};

/* define core states */
#define CORE_UNINIT	0
#define CORE_INIT	1
#define CORE_INVALID	2

/* define instance states */
#define INST_INVALID			1
#define INST_UNINIT			2
#define INST_OPEN			3
#define INST_LOAD_RESOURCES		4
#define INST_START			5
#define INST_STOP			6
#define INST_RELEASE_RESOURCES		7
#define INST_CLOSE			8

struct vidc_list {
	struct list_head list;
	struct mutex lock;
};

static inline void INIT_VIDC_LIST(struct vidc_list *mlist)
{
	mutex_init(&mlist->lock);
	INIT_LIST_HEAD(&mlist->list);
}

struct vidc_internal_buf {
	struct list_head list;
	enum hal_buffer type;
	struct smem *smem;
};

struct vidc_format {
	u32 pixfmt;
	int num_planes;
	u32 type;
	u32 (*get_framesize)(int plane, u32 height, u32 width);
};

struct vidc_drv {
	struct mutex lock;
	struct list_head cores;
	int num_cores;
	struct dentry *debugfs_root;
	int thermal_level;
};

enum profiling_points {
	SYS_INIT = 0,
	SESSION_INIT,
	LOAD_RESOURCES,
	FRAME_PROCESSING,
	FW_IDLE,
	MAX_PROFILING_POINTS,
};

struct buf_count {
	int etb;
	int ftb;
	int fbd;
	int ebd;
};

struct dcvs_stats {
	int num_ftb[DCVS_FTB_WINDOW];
	bool transition_turbo;
	int ftb_index;
	int ftb_counter;
	bool prev_freq_lowered;
	bool prev_freq_increased;
	int threshold_disp_buf_high;
	int threshold_disp_buf_low;
	int load;
	int load_low;
	int load_high;
	int min_threshold;
	int max_threshold;
	bool is_clock_scaled;
	int etb_counter;
	bool is_power_save_mode;
	bool is_output_buff_added;
	bool is_input_buff_added;
	bool is_additional_buff_added;
};

struct profile_data {
	int start;
	int stop;
	int cumulative;
	char name[64];
	int sampling;
	int average;
};

struct vidc_debug {
	struct profile_data pdata[MAX_PROFILING_POINTS];
	int profile;
	int samples;
};

enum vidc_modes {
	VIDC_SECURE	= 1 << 0,
	VIDC_TURBO	= 1 << 1,
	VIDC_THUMBNAIL	= 1 << 2,
	VIDC_NOMINAL	= 1 << 3,
};

enum core_id {
	VIDC_CORE_VENUS = 0,
	VIDC_CORE_Q6,
	VIDC_CORES_MAX,
};

enum session_type {
	VIDC_ENCODER = 0,
	VIDC_DECODER,
	VIDC_MAX_DEVICES,
};

struct vidc_core_capability {
	struct hal_capability_supported width;
	struct hal_capability_supported height;
	struct hal_capability_supported frame_rate;
	u32 pixelprocess_capabilities;
	struct hal_capability_supported scale_x;
	struct hal_capability_supported scale_y;
	struct hal_capability_supported hier_p;
	struct hal_capability_supported ltr_count;
	struct hal_capability_supported mbs_per_frame;
	struct hal_capability_supported secure_output2_threshold;
	u32 capability_set;
	enum hal_buffer_mode_type buffer_mode[MAX_PORT_NUM];
};

struct vidc_core {
	struct list_head list;
	struct mutex lock;
	int id;
	void *hfidev;
	struct video_device vdev_dec;
	struct video_device vdev_enc;
	struct v4l2_device v4l2_dev;
	struct list_head instances;
	struct dentry *debugfs_root;
	unsigned int state;
	struct completion done;
	unsigned int error;
	enum vidc_hfi_type hfi_type;
	struct vidc_resources res;
	u32 enc_codecs;
	u32 dec_codecs;
	struct rproc *rproc;
	bool rproc_booted;

	int (*event_notify)(struct vidc_core *core, u32 device_id, u32 event);
};

struct vidc_inst {
	struct list_head list;
	struct mutex sync_lock, lock;
	struct vidc_core *core;

	struct vidc_list scratchbufs;
	struct vidc_list persistbufs;
	struct vidc_list pending_getpropq;
	struct vidc_list registeredbufs;

	struct list_head bufqueue;
	struct mutex bufqueue_lock;

	int streamoff;
	struct buffer_requirements buff_req;
	void *mem_client;
	struct vb2_queue bufq_out;
	struct vb2_queue bufq_cap;
	void *vb2_ctx_cap;
	void *vb2_ctx_out;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl **ctrls;
	struct v4l2_ctrl **cluster;
	struct v4l2_fh fh;

	int (*empty_buf_done)(struct vidc_inst *inst, u32 addr,
			      u32 bytesused, u32 data_offset, u32 flags);
	int (*fill_buf_done)(struct vidc_inst *inst, u32 addr,
			     u32 bytesused, u32 data_offset, u32 flags,
			     struct timeval *timestamp);
	int (*event_notify)(struct vidc_inst *inst, u32 event,
			    struct vidc_cb_event *data);

	/* session fields */
	enum session_type session_type;
	unsigned int state;
	void *session;
	u32 width;
	u32 height;
	u64 fps;
	struct v4l2_fract timeperframe;
	const struct vidc_format *fmt_out;
	const struct vidc_format *fmt_cap;
	struct completion done;
	unsigned int error;
	struct vidc_core_capability capability;
	enum hal_buffer_mode_type buffer_mode[MAX_PORT_NUM];
	enum vidc_modes flags;
	struct buf_count count;
	bool in_reconfig;
	u32 reconfig_width;
	u32 reconfig_height;

	/* encoder fields */
	atomic_t seq_hdr_reqs;

	struct dentry *debugfs_root;
	struct vidc_debug debug;
};

extern struct vidc_drv *vidc_driver;

struct vidc_ctrl {
	u32 id;
	char name[MAX_NAME_LENGTH];
	enum v4l2_ctrl_type type;
	s32 minimum;
	s32 maximum;
	s32 default_value;
	u32 step;
	u32 menu_skip_mask;
	u32 flags;
	const char * const *qmenu;
};

struct buffer_info {
	struct list_head list;
	struct vidc_buffer_addr_info bai;
};

int vidc_trigger_ssr(struct vidc_core *core, enum hal_ssr_trigger_type type);
int vidc_check_session_supported(struct vidc_inst *inst);
void vidc_queue_v4l2_event(struct vidc_inst *inst, int event_type);

#endif
