/*
 * camss-vfe-4-1.h
 *
 * Qualcomm MSM Camera Subsystem - VFE (Video Front End) Module v4.1
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2017 Linaro Ltd.
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

#include "camss-vfe.h"

u16 vfe_get_ub_size(u8 vfe_id);
void vfe_global_reset(struct vfe_device *vfe);
void vfe_request_halt(struct vfe_device *vfe);
void vfe_wm_enable(struct vfe_device *vfe, u8 wm, u8 enable);
void vfe_wm_frame_based(struct vfe_device *vfe, u8 wm, u8 enable);
void vfe_wm_line_based(struct vfe_device *vfe, u32 wm,
		       struct v4l2_pix_format_mplane *pix,
		       u8 plane, u32 enable);
void vfe_wm_set_framedrop_period(struct vfe_device *vfe, u8 wm, u8 per);
void vfe_wm_set_framedrop_pattern(struct vfe_device *vfe, u8 wm, u32 pattern);
void vfe_wm_set_ub_cfg(struct vfe_device *vfe, u8 wm, u16 offset, u16 depth);
void vfe_bus_reload_wm(struct vfe_device *vfe, u8 wm);
void vfe_wm_set_ping_addr(struct vfe_device *vfe, u8 wm, u32 addr);
void vfe_wm_set_pong_addr(struct vfe_device *vfe, u8 wm, u32 addr);
int vfe_wm_get_ping_pong_status(struct vfe_device *vfe, u8 wm);
void vfe_bus_enable_wr_if(struct vfe_device *vfe, u8 enable);
void vfe_bus_connect_wm_to_rdi(struct vfe_device *vfe, u8 wm,
			       enum vfe_line_id id);
void vfe_wm_set_subsample(struct vfe_device *vfe, u8 wm);
void vfe_bus_disconnect_wm_from_rdi(struct vfe_device *vfe, u8 wm,
				    enum vfe_line_id id);
void vfe_set_xbar_cfg(struct vfe_device *vfe, struct vfe_output *output,
		      u8 enable);
void vfe_set_rdi_cid(struct vfe_device *vfe, enum vfe_line_id id, u8 cid);
void vfe_reg_update(struct vfe_device *vfe, enum vfe_line_id line_id);
void vfe_enable_irq_wm_line(struct vfe_device *vfe, u8 wm,
			    enum vfe_line_id line_id, u8 enable);
void vfe_enable_irq_pix_line(struct vfe_device *vfe, u8 comp,
			     enum vfe_line_id line_id, u8 enable);
void vfe_enable_irq_common(struct vfe_device *vfe);
void vfe_set_demux_cfg(struct vfe_device *vfe, struct vfe_line *line);
void vfe_set_scale_cfg(struct vfe_device *vfe, struct vfe_line *line);
void vfe_set_crop_cfg(struct vfe_device *vfe, struct vfe_line *line);
void vfe_set_clamp_cfg(struct vfe_device *vfe);
void vfe_set_qos(struct vfe_device *vfe);
void vfe_set_cgc_override(struct vfe_device *vfe, u8 wm, u8 enable);
void vfe_set_camif_cfg(struct vfe_device *vfe, struct vfe_line *line);
void vfe_set_camif_cmd(struct vfe_device *vfe, u32 cmd);
void vfe_set_module_cfg(struct vfe_device *vfe, u8 enable);
int vfe_camif_wait_for_stop(struct vfe_device *vfe, struct device *dev);
