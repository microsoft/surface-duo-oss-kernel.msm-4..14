/* Copyright (c) 2010-2015, The Linux Foundation. All rights reserved.
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
#ifndef __QCOM_SCM_INT_H
#define __QCOM_SCM_INT_H

#define QCOM_SCM_SVC_BOOT		0x1
#define QCOM_SCM_BOOT_ADDR		0x1
#define QCOM_SCM_BOOT_ADDR_MC		0x11

#define QCOM_SCM_FLAG_HLOS		0x01
#define QCOM_SCM_FLAG_COLDBOOT_MC	0x02
#define QCOM_SCM_FLAG_WARMBOOT_MC	0x04
extern int __qcom_scm_set_warm_boot_addr(void *entry, const cpumask_t *cpus);
extern int __qcom_scm_set_cold_boot_addr(void *entry, const cpumask_t *cpus);

#define QCOM_SCM_CMD_TERMINATE_PC	0x2
#define QCOM_SCM_FLUSH_FLAG_MASK	0x3
#define QCOM_SCM_CMD_CORE_HOTPLUGGED	0x10
extern void __qcom_scm_cpu_power_down(u32 flags);

#define QCOM_SCM_SVC_INFO		0x6
#define QCOM_IS_CALL_AVAIL_CMD		0x1
extern int __qcom_scm_is_call_available(u32 svc_id, u32 cmd_id);

#define QCOM_SCM_SVC_HDCP		0x11
#define QCOM_SCM_CMD_HDCP		0x01
extern int __qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt,
		u32 *resp);

#define QCOM_SCM_SVC_PIL		0x2
#define QCOM_SCM_PAS_INIT_IMAGE_CMD	0x1
#define QCOM_SCM_PAS_MEM_SETUP_CMD	0x2
#define QCOM_SCM_PAS_AUTH_AND_RESET_CMD	0x5
#define QCOM_SCM_PAS_SHUTDOWN_CMD	0x6
#define QCOM_SCM_PAS_IS_SUPPORTED_CMD	0x7
#define QCOM_SCM_PAS_MSS_RESET		0xa
extern bool __qcom_scm_pas_supported(u32 peripheral);
extern int  __qcom_scm_pas_init_image(u32 peripheral, dma_addr_t metadata_phys);
extern int  __qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size);
extern int  __qcom_scm_pas_auth_and_reset(u32 peripheral);
extern int  __qcom_scm_pas_shutdown(u32 peripheral);
extern int  __qcom_scm_pas_mss_reset(bool reset);

/* common error codes */
#define QCOM_SCM_ENOMEM		-5
#define QCOM_SCM_EOPNOTSUPP	-4
#define QCOM_SCM_EINVAL_ADDR	-3
#define QCOM_SCM_EINVAL_ARG	-2
#define QCOM_SCM_ERROR		-1
#define QCOM_SCM_INTERRUPTED	1

static inline int qcom_scm_remap_error(int err)
{
	switch (err) {
	case QCOM_SCM_ERROR:
		return -EIO;
	case QCOM_SCM_EINVAL_ADDR:
	case QCOM_SCM_EINVAL_ARG:
		return -EINVAL;
	case QCOM_SCM_EOPNOTSUPP:
		return -EOPNOTSUPP;
	case QCOM_SCM_ENOMEM:
		return -ENOMEM;
	}
	return -EINVAL;
}

enum scm_cmd {
	PAS_INIT_IMAGE_CMD = 1,
	PAS_MEM_SETUP_CMD,
	PAS_AUTH_AND_RESET_CMD = 5,
	PAS_SHUTDOWN_CMD,
};

#define SCM_SVC_BOOT		0x1
#define SCM_SVC_PIL		0x2
#define SCM_SVC_INFO		0x6

#define GET_FEAT_VERSION_CMD	3

extern int __qcom_scm_pil_init_image_cmd(u32 proc, u64 image_addr);
extern int __qcom_scm_pil_mem_setup_cmd(u32 proc, u64 start_addr, u32 len);
extern int __qcom_scm_pil_auth_and_reset_cmd(u32 proc);
extern int __qcom_scm_pil_shutdown_cmd(u32 proc);

extern int __qcom_scm_iommu_dump_fault_regs(u32 id, u32 context, u64 addr,
					    u32 len);
extern int __qcom_scm_iommu_set_cp_pool_size(u32 size, u32 spare);
extern int __qcom_scm_iommu_secure_ptbl_size(u32 spare, int psize[2]);
extern int __qcom_scm_iommu_secure_ptbl_init(u64 addr, u32 size, u32 spare);
extern int __qcom_scm_iommu_secure_map(u64 list, u32 list_size, u32 size,
				       u32 id, u32 ctx_id, u64 va,
				       u32 info_size, u32 flags);
extern int __qcom_scm_iommu_secure_unmap(u32 id, u32 ctx_id, u64 va,
					 u32 size, u32 flags);

extern int __qcom_scm_is_call_available(u32 svc_id, u32 cmd_id);
extern int __qcom_scm_get_feat_version(u32 feat);
extern int __qcom_scm_restore_sec_cfg(u32 device_id, u32 spare);

extern int __qcom_scm_set_video_state(u32 state, u32 spare);
extern int __qcom_scm_mem_protect_video_var(u32 start, u32 size,
					    u32 nonpixel_start,
					    u32 nonpixel_size);
extern int __qcom_scm_init(void);
#endif
