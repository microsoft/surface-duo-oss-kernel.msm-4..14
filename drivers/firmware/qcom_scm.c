/* Copyright (c) 2010,2015, The Linux Foundation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/platform_device.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/qcom_scm.h>

#include "qcom_scm.h"

/**
 * qcom_scm_set_cold_boot_addr() - Set the cold boot address for cpus
 * @entry: Entry point function for the cpus
 * @cpus: The cpumask of cpus that will use the entry point
 *
 * Set the cold boot address of the cpus. Any cpu outside the supported
 * range would be removed from the cpu present mask.
 */
int qcom_scm_set_cold_boot_addr(void *entry, const cpumask_t *cpus)
{
	return __qcom_scm_set_cold_boot_addr(entry, cpus);
}
EXPORT_SYMBOL(qcom_scm_set_cold_boot_addr);

/**
 * qcom_scm_set_warm_boot_addr() - Set the warm boot address for cpus
 * @entry: Entry point function for the cpus
 * @cpus: The cpumask of cpus that will use the entry point
 *
 * Set the Linux entry point for the SCM to transfer control to when coming
 * out of a power down. CPU power down may be executed on cpuidle or hotplug.
 */
int qcom_scm_set_warm_boot_addr(void *entry, const cpumask_t *cpus)
{
	return __qcom_scm_set_warm_boot_addr(entry, cpus);
}
EXPORT_SYMBOL(qcom_scm_set_warm_boot_addr);

/**
 * qcom_scm_cpu_power_down() - Power down the cpu
 * @flags - Flags to flush cache
 *
 * This is an end point to power down cpu. If there was a pending interrupt,
 * the control would return from this function, otherwise, the cpu jumps to the
 * warm boot entry point set for this cpu upon reset.
 */
void qcom_scm_cpu_power_down(u32 flags)
{
	__qcom_scm_cpu_power_down(flags);
}
EXPORT_SYMBOL(qcom_scm_cpu_power_down);

/**
 * qcom_scm_hdcp_available() - Check if secure environment supports HDCP.
 *
 * Return true if HDCP is supported, false if not.
 */
bool qcom_scm_hdcp_available(void)
{
	int ret;

	ret = __qcom_scm_is_call_available(QCOM_SCM_SVC_HDCP,
		QCOM_SCM_CMD_HDCP);

	return (ret > 0) ? true : false;
}
EXPORT_SYMBOL(qcom_scm_hdcp_available);

/**
 * qcom_scm_hdcp_req() - Send HDCP request.
 * @req: HDCP request array
 * @req_cnt: HDCP request array count
 * @resp: response buffer passed to SCM
 *
 * Write HDCP register(s) through SCM.
 */
int qcom_scm_hdcp_req(struct qcom_scm_hdcp_req *req, u32 req_cnt, u32 *resp)
{
	return __qcom_scm_hdcp_req(req, req_cnt, resp);
}
EXPORT_SYMBOL(qcom_scm_hdcp_req);

/**
 * qcom_scm_pas_supported() - Check if the peripheral authentication service is
 *			      available for the given peripherial
 * @peripheral:	peripheral id
 *
 * Returns true if PAS is supported for this peripheral, otherwise false.
 */
bool qcom_scm_pas_supported(u32 peripheral)
{
	int ret;

	ret = __qcom_scm_is_call_available(QCOM_SCM_SVC_PIL,
					   QCOM_SCM_PAS_IS_SUPPORTED_CMD);
	if (ret <= 0)
		return false;

	return __qcom_scm_pas_supported(peripheral);
}
EXPORT_SYMBOL(qcom_scm_pas_supported);

/**
 * qcom_scm_pas_init_image() - Initialize peripheral authentication service
 *			       state machine for a given peripheral, using the
 *			       metadata
 * @peripheral: peripheral id
 * @metadata:	pointer to memory containing ELF header, program header table
 *		and optional blob of data used for authenticating the metadata
 *		and the rest of the firmware
 * @size:	size of the metadata
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_init_image(struct device *dev, u32 peripheral, const void *metadata, size_t size)
{
	return __qcom_scm_pas_init_image(dev, peripheral, metadata, size);
}
EXPORT_SYMBOL(qcom_scm_pas_init_image);

/**
 * qcom_scm_pas_mem_setup() - Prepare the memory related to a given peripheral
 *			      for firmware loading
 * @peripheral:	peripheral id
 * @addr:	start address of memory area to prepare
 * @size:	size of the memory area to prepare
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_mem_setup(u32 peripheral, phys_addr_t addr, phys_addr_t size)
{
	return __qcom_scm_pas_mem_setup(peripheral, addr, size);
}
EXPORT_SYMBOL(qcom_scm_pas_mem_setup);

/**
 * qcom_scm_pas_auth_and_reset() - Authenticate the given peripheral firmware
 *				   and reset the remote processor
 * @peripheral:	peripheral id
 *
 * Return 0 on success.
 */
int qcom_scm_pas_auth_and_reset(u32 peripheral)
{
	return __qcom_scm_pas_auth_and_reset(peripheral);
}
EXPORT_SYMBOL(qcom_scm_pas_auth_and_reset);

/**
 * qcom_scm_pas_shutdown() - Shut down the remote processor
 * @peripheral: peripheral id
 *
 * Returns 0 on success.
 */
int qcom_scm_pas_shutdown(u32 peripheral)
{
	return __qcom_scm_pas_shutdown(peripheral);
}
EXPORT_SYMBOL(qcom_scm_pas_shutdown);

int qcom_scm_pil_init_image_cmd(u32 proc, u64 image_addr)
{
	return __qcom_scm_pil_init_image_cmd(proc, image_addr);
}
EXPORT_SYMBOL(qcom_scm_pil_init_image_cmd);

int qcom_scm_pil_mem_setup_cmd(u32 proc, u64 start_addr, u32 len)
{
	return __qcom_scm_pil_mem_setup_cmd(proc, start_addr, len);
}
EXPORT_SYMBOL(qcom_scm_pil_mem_setup_cmd);

int qcom_scm_pil_auth_and_reset_cmd(u32 proc)
{
	return __qcom_scm_pil_auth_and_reset_cmd(proc);
}
EXPORT_SYMBOL(qcom_scm_pil_auth_and_reset_cmd);

int qcom_scm_pil_shutdown_cmd(u32 proc)
{
	return __qcom_scm_pil_shutdown_cmd(proc);
}
EXPORT_SYMBOL(qcom_scm_pil_shutdown_cmd);

int qcom_scm_iommu_dump_fault_regs(u32 id, u32 context, u64 addr, u32 len)
{
	return __qcom_scm_iommu_dump_fault_regs(id, context, addr, len);
}
EXPORT_SYMBOL(qcom_scm_iommu_dump_fault_regs);

int qcom_scm_iommu_set_cp_pool_size(u32 size, u32 spare)
{
	return __qcom_scm_iommu_set_cp_pool_size(size, spare);
}
EXPORT_SYMBOL(qcom_scm_iommu_set_cp_pool_size);

int qcom_scm_iommu_secure_ptbl_size(u32 spare, int psize[2])
{
	return __qcom_scm_iommu_secure_ptbl_size(spare, psize);
}
EXPORT_SYMBOL(qcom_scm_iommu_secure_ptbl_size);

int qcom_scm_iommu_secure_ptbl_init(u64 addr, u32 size, u32 spare)
{
	return __qcom_scm_iommu_secure_ptbl_init(addr, size, spare);
}
EXPORT_SYMBOL(qcom_scm_iommu_secure_ptbl_init);

int qcom_scm_iommu_secure_map(u64 list, u32 list_size, u32 size,
			      u32 id, u32 ctx_id, u64 va, u32 info_size,
			      u32 flags)
{
	return __qcom_scm_iommu_secure_map(list, list_size, size, id,
					   ctx_id, va, info_size, flags);
}
EXPORT_SYMBOL(qcom_scm_iommu_secure_map);

int qcom_scm_iommu_secure_unmap(u32 id, u32 ctx_id, u64 va, u32 size, u32 flags)
{
	return __qcom_scm_iommu_secure_unmap(id, ctx_id, va, size, flags);
}
EXPORT_SYMBOL(qcom_scm_iommu_secure_unmap);

int qcom_scm_is_call_available(u32 svc_id, u32 cmd_id)
{
	return __qcom_scm_is_call_available(svc_id, cmd_id);
}
EXPORT_SYMBOL(qcom_scm_is_call_available);

int qcom_scm_get_feat_version(u32 feat)
{
	return __qcom_scm_get_feat_version(feat);
}
EXPORT_SYMBOL(qcom_scm_get_feat_version);

int qcom_scm_restore_sec_cfg(u32 device_id, u32 spare)
{
	return __qcom_scm_restore_sec_cfg(device_id, spare);
}
EXPORT_SYMBOL(qcom_scm_restore_sec_cfg);
