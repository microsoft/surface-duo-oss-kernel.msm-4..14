/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/slab.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/scm-boot.h>

static DEFINE_PER_CPU(void *, last_known_entry);
/*
 * Set the cold/warm boot address for one of the CPU cores.
 */
int scm_set_boot_addr(phys_addr_t addr, int flags)
{
	struct {
		unsigned int flags;
		phys_addr_t  addr;
	} cmd;

	cmd.addr = addr;
	cmd.flags = flags;
	return scm_call(SCM_SVC_BOOT, SCM_BOOT_ADDR,
			&cmd, sizeof(cmd), NULL, 0);
}
EXPORT_SYMBOL(scm_set_boot_addr);


int scm_set_warm_boot_addr(void *entry, int cpu)
{
	static int flags[NR_CPUS] = {
		SCM_FLAG_WARMBOOT_CPU0,
		SCM_FLAG_WARMBOOT_CPU1,
		SCM_FLAG_WARMBOOT_CPU2,
		SCM_FLAG_WARMBOOT_CPU3,
	};
	int ret;

	if (entry == per_cpu(last_known_entry, cpu))
		return 0;

	ret = scm_set_boot_addr(virt_to_phys(entry), flags[cpu]);
	if (!ret)
		per_cpu(last_known_entry, cpu) = entry;

	return ret;
}
