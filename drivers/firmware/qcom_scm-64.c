/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/qcom_scm.h>

#include <asm/cacheflush.h>
#include <asm/compiler.h>
#include <asm/smp_plat.h>

#include "qcom_scm.h"

#define QCOM_SCM_SIP_FNID(s, c) (((((s) & 0xFF) << 8) | ((c) & 0xFF)) | 0x02000000)

#define MAX_QCOM_SCM_ARGS 10
#define MAX_QCOM_SCM_RETS 3

#define QCOM_SCM_ARGS_IMPL(num, a, b, c, d, e, f, g, h, i, j, ...) (\
			(((a) & 0xff) << 4) | \
			(((b) & 0xff) << 6) | \
			(((c) & 0xff) << 8) | \
			(((d) & 0xff) << 10) | \
			(((e) & 0xff) << 12) | \
			(((f) & 0xff) << 14) | \
			(((g) & 0xff) << 16) | \
			(((h) & 0xff) << 18) | \
			(((i) & 0xff) << 20) | \
			(((j) & 0xff) << 22) | \
			(num & 0xffff))

#define QCOM_SCM_ARGS(...) QCOM_SCM_ARGS_IMPL(__VA_ARGS__, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

/**
 * struct qcom_scm_desc
 * @arginfo: Metadata describing the arguments in args[]
 * @args: The array of arguments for the secure syscall
 * @ret: The values returned by the secure syscall
 * @extra_arg_buf: The buffer containing extra arguments
		   (that don't fit in available registers)
 * @x5: The 4rd argument to the secure syscall or physical address of
	extra_arg_buf
 */
struct qcom_scm_desc {
	u32 arginfo;
	u64 args[MAX_QCOM_SCM_ARGS];
	u64 ret[MAX_QCOM_SCM_RETS];

	/* private */
	void *extra_arg_buf;
	u64 x5;
};


#define QCOM_SCM_ENOMEM		-5
#define QCOM_SCM_EOPNOTSUPP	-4
#define QCOM_SCM_EINVAL_ADDR	-3
#define QCOM_SCM_EINVAL_ARG	-2
#define QCOM_SCM_ERROR		-1
#define QCOM_SCM_INTERRUPTED	1
#define QCOM_SCM_EBUSY		-55
#define QCOM_SCM_V2_EBUSY	-12

static DEFINE_MUTEX(qcom_scm_lock);

#define QCOM_SCM_EBUSY_WAIT_MS 30
#define QCOM_SCM_EBUSY_MAX_RETRY 20

#define N_EXT_QCOM_SCM_ARGS 7
#define FIRST_EXT_ARG_IDX 3
#define SMC_ATOMIC_SYSCALL 31
#define N_REGISTER_ARGS (MAX_QCOM_SCM_ARGS - N_EXT_QCOM_SCM_ARGS + 1)
#define SMC64_MASK 0x40000000
#define SMC_ATOMIC_MASK 0x80000000
#define IS_CALL_AVAIL_CMD 1

#define R0_STR "x0"
#define R1_STR "x1"
#define R2_STR "x2"
#define R3_STR "x3"
#define R4_STR "x4"
#define R5_STR "x5"

static int qcom_scm_remap_error(int err)
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
	case QCOM_SCM_EBUSY:
		return QCOM_SCM_EBUSY;
	case QCOM_SCM_V2_EBUSY:
		return QCOM_SCM_V2_EBUSY;
	}
	return -EINVAL;
}

int __qcom_scm_call_armv8_64(u64 x0, u64 x1, u64 x2, u64 x3, u64 x4, u64 x5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	register u64 r0 asm("r0") = x0;
	register u64 r1 asm("r1") = x1;
	register u64 r2 asm("r2") = x2;
	register u64 r3 asm("r3") = x3;
	register u64 r4 asm("r4") = x4;
	register u64 r5 asm("r5") = x5;

	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R2_STR)
			__asmeq("%3", R3_STR)
			__asmeq("%4", R0_STR)
			__asmeq("%5", R1_STR)
			__asmeq("%6", R2_STR)
			__asmeq("%7", R3_STR)
			__asmeq("%8", R4_STR)
			__asmeq("%9", R5_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
			  "r" (r5)
			: "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
			  "x14", "x15", "x16", "x17");
	} while (r0 == QCOM_SCM_INTERRUPTED);

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}

int __qcom_scm_call_armv8_32(u32 w0, u32 w1, u32 w2, u32 w3, u32 w4, u32 w5,
				u64 *ret1, u64 *ret2, u64 *ret3)
{
	register u32 r0 asm("r0") = w0;
	register u32 r1 asm("r1") = w1;
	register u32 r2 asm("r2") = w2;
	register u32 r3 asm("r3") = w3;
	register u32 r4 asm("r4") = w4;
	register u32 r5 asm("r5") = w5;

	do {
		asm volatile(
			__asmeq("%0", R0_STR)
			__asmeq("%1", R1_STR)
			__asmeq("%2", R2_STR)
			__asmeq("%3", R3_STR)
			__asmeq("%4", R0_STR)
			__asmeq("%5", R1_STR)
			__asmeq("%6", R2_STR)
			__asmeq("%7", R3_STR)
			__asmeq("%8", R4_STR)
			__asmeq("%9", R5_STR)
#ifdef REQUIRES_SEC
			".arch_extension sec\n"
#endif
			"smc	#0\n"
			: "=r" (r0), "=r" (r1), "=r" (r2), "=r" (r3)
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4),
			  "r" (r5)
			: "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
			"x14", "x15", "x16", "x17");

	} while (r0 == QCOM_SCM_INTERRUPTED);

	if (ret1)
		*ret1 = r1;
	if (ret2)
		*ret2 = r2;
	if (ret3)
		*ret3 = r3;

	return r0;
}

struct qcom_scm_extra_arg {
	union {
		u32 args32[N_EXT_QCOM_SCM_ARGS];
		u64 args64[N_EXT_QCOM_SCM_ARGS];
	};
};

static enum qcom_scm_interface_version {
	QCOM_SCM_UNKNOWN,
	QCOM_SCM_LEGACY,
	QCOM_SCM_ARMV8_32,
	QCOM_SCM_ARMV8_64,
} qcom_scm_version = QCOM_SCM_UNKNOWN;

/* This will be set to specify SMC32 or SMC64 */
static u32 qcom_scm_version_mask;

/*
 * If there are more than N_REGISTER_ARGS, allocate a buffer and place
 * the additional arguments in it. The extra argument buffer will be
 * pointed to by X5.
 */
static int allocate_extra_arg_buffer(struct qcom_scm_desc *desc, gfp_t flags)
{
	int i, j;
	struct qcom_scm_extra_arg *argbuf;
	int arglen = desc->arginfo & 0xf;
	size_t argbuflen = PAGE_ALIGN(sizeof(struct qcom_scm_extra_arg));

	desc->x5 = desc->args[FIRST_EXT_ARG_IDX];

	if (likely(arglen <= N_REGISTER_ARGS)) {
		desc->extra_arg_buf = NULL;
		return 0;
	}

	argbuf = kzalloc(argbuflen, flags);
	if (!argbuf) {
		pr_err("qcom_scm_call: failed to alloc mem for extended argument buffer\n");
		return -ENOMEM;
	}

	desc->extra_arg_buf = argbuf;

	j = FIRST_EXT_ARG_IDX;
	if (qcom_scm_version == QCOM_SCM_ARMV8_64)
		for (i = 0; i < N_EXT_QCOM_SCM_ARGS; i++)
			argbuf->args64[i] = desc->args[j++];
	else
		for (i = 0; i < N_EXT_QCOM_SCM_ARGS; i++)
			argbuf->args32[i] = desc->args[j++];
	desc->x5 = virt_to_phys(argbuf);
	__flush_dcache_area(argbuf, argbuflen);

	return 0;
}

/**
 * qcom_scm_call() - Invoke a syscall in the secure world
 * @svc_id: service identifier
 * @cmd_id: command identifier
 * @fn_id: The function ID for this syscall
 * @desc: Descriptor structure containing arguments and return values
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 * This should *only* be called in pre-emptible context.
 *
 * A note on cache maintenance:
 * Note that any buffers that are expected to be accessed by the secure world
 * must be flushed before invoking qcom_scm_call and invalidated in the cache
 * immediately after qcom_scm_call returns. An important point that must be noted
 * is that on ARMV8 architectures, invalidation actually also causes a dirty
 * cache line to be cleaned (flushed + unset-dirty-bit). Therefore it is of
 * paramount importance that the buffer be flushed before invoking qcom_scm_call,
 * even if you don't care about the contents of that buffer.
 *
 * Note that cache maintenance on the argument buffer (desc->args) is taken care
 * of by qcom_scm_call; however, callers are responsible for any other cached
 * buffers passed over to the secure world.
*/
static int qcom_scm_call(u32 svc_id, u32 cmd_id, struct qcom_scm_desc *desc)
{
	int arglen = desc->arginfo & 0xf;
	int ret, retry_count = 0;
	u32 fn_id = QCOM_SCM_SIP_FNID(svc_id, cmd_id);
	u64 x0;

	ret = allocate_extra_arg_buffer(desc, GFP_KERNEL);
	if (ret)
		return ret;

	x0 = fn_id | qcom_scm_version_mask;

	do {
		mutex_lock(&qcom_scm_lock);

		desc->ret[0] = desc->ret[1] = desc->ret[2] = 0;

		pr_debug("qcom_scm_call: func id %#llx, args: %#x, %#llx, %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5);

		if (qcom_scm_version == QCOM_SCM_ARMV8_64)
			ret = __qcom_scm_call_armv8_64(x0, desc->arginfo,
						  desc->args[0], desc->args[1],
						  desc->args[2], desc->x5,
						  &desc->ret[0], &desc->ret[1],
						  &desc->ret[2]);
		else
			ret = __qcom_scm_call_armv8_32(x0, desc->arginfo,
						  desc->args[0], desc->args[1],
						  desc->args[2], desc->x5,
						  &desc->ret[0], &desc->ret[1],
						  &desc->ret[2]);
		mutex_unlock(&qcom_scm_lock);

		if (ret == QCOM_SCM_V2_EBUSY)
			msleep(QCOM_SCM_EBUSY_WAIT_MS);
	}  while (ret == QCOM_SCM_V2_EBUSY && (retry_count++ < QCOM_SCM_EBUSY_MAX_RETRY));

	if (ret < 0)
		pr_err("qcom_scm_call failed: func id %#llx, arginfo: %#x, args: %#llx, %#llx, %#llx, %#llx, ret: %d, syscall returns: %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5, ret, desc->ret[0],
			desc->ret[1], desc->ret[2]);

	if (arglen > N_REGISTER_ARGS)
		kfree(desc->extra_arg_buf);
	if (ret < 0)
		return qcom_scm_remap_error(ret);
	return 0;
}

/**
 * qcom_scm_call_atomic() - Invoke a syscall in the secure world
 *
 * Similar to qcom_scm_call except that this can be invoked in atomic context.
 * There is also no retry mechanism implemented. Please ensure that the
 * secure world syscall can be executed in such a context and can complete
 * in a timely manner.
 */
static int qcom_scm_call_atomic(u32 s, u32 c, struct qcom_scm_desc *desc)
{
	int arglen = desc->arginfo & 0xf;
	int ret;
	u32 fn_id = QCOM_SCM_SIP_FNID(s, c);
	u64 x0;

	ret = allocate_extra_arg_buffer(desc, GFP_ATOMIC);
	if (ret)
		return ret;

	x0 = fn_id | BIT(SMC_ATOMIC_SYSCALL) | qcom_scm_version_mask;

	pr_debug("qcom_scm_call: func id %#llx, args: %#x, %#llx, %#llx, %#llx, %#llx\n",
		x0, desc->arginfo, desc->args[0], desc->args[1],
		desc->args[2], desc->x5);

	if (qcom_scm_version == QCOM_SCM_ARMV8_64)
		ret = __qcom_scm_call_armv8_64(x0, desc->arginfo, desc->args[0],
					  desc->args[1], desc->args[2],
					  desc->x5, &desc->ret[0],
					  &desc->ret[1], &desc->ret[2]);
	else
		ret = __qcom_scm_call_armv8_32(x0, desc->arginfo, desc->args[0],
					  desc->args[1], desc->args[2],
					  desc->x5, &desc->ret[0],
					  &desc->ret[1], &desc->ret[2]);
	if (ret < 0)
		pr_err("qcom_scm_call failed: func id %#llx, arginfo: %#x, args: %#llx, %#llx, %#llx, %#llx, ret: %d, syscall returns: %#llx, %#llx, %#llx\n",
			x0, desc->arginfo, desc->args[0], desc->args[1],
			desc->args[2], desc->x5, ret, desc->ret[0],
			desc->ret[1], desc->ret[2]);

	if (arglen > N_REGISTER_ARGS)
		kfree(desc->extra_arg_buf);
	if (ret < 0)
		return qcom_scm_remap_error(ret);
	return ret;
}

static int qcom_scm_set_boot_addr(void *entry, const cpumask_t *cpus, int flags)
{
	struct qcom_scm_desc desc = {0};
	unsigned int cpu = cpumask_first(cpus);
	u64 mpidr_el1 = cpu_logical_map(cpu);

	/* For now we assume only a single cpu is set in the mask */
	WARN_ON(cpumask_weight(cpus) != 1);

	if (mpidr_el1 & ~MPIDR_HWID_BITMASK) {
		pr_err("CPU%d:Failed to set boot address\n", cpu);
		return -ENOSYS;
	}

	desc.args[0] = virt_to_phys(entry);
	desc.args[1] = BIT(MPIDR_AFFINITY_LEVEL(mpidr_el1, 0));
	desc.args[2] = BIT(MPIDR_AFFINITY_LEVEL(mpidr_el1, 1));
	desc.args[3] = BIT(MPIDR_AFFINITY_LEVEL(mpidr_el1, 2));
	desc.args[4] = ~0ULL;
	desc.args[5] = QCOM_SCM_FLAG_HLOS | flags;
	desc.arginfo = QCOM_SCM_ARGS(6);

	return qcom_scm_call(QCOM_SCM_SVC_BOOT, QCOM_SCM_BOOT_ADDR_MC, &desc);
}

int __qcom_scm_set_cold_boot_addr(void *entry, const cpumask_t *cpus)
{
	int flags = QCOM_SCM_FLAG_COLDBOOT_MC;

	return qcom_scm_set_boot_addr(entry, cpus, flags);
}

int __qcom_scm_set_warm_boot_addr(void *entry, const cpumask_t *cpus)
{
	int flags = QCOM_SCM_FLAG_WARMBOOT_MC;

	return qcom_scm_set_boot_addr(entry, cpus, flags);
}

void __qcom_scm_cpu_power_down(u32 flags)
{
	struct qcom_scm_desc desc = {0};
	desc.args[0] = QCOM_SCM_CMD_CORE_HOTPLUGGED |
		       (flags & QCOM_SCM_FLUSH_FLAG_MASK);
	desc.arginfo = QCOM_SCM_ARGS(1);

	qcom_scm_call_atomic(QCOM_SCM_SVC_BOOT, QCOM_SCM_CMD_TERMINATE_PC, &desc);
}

#define QCOM_SCM_SVC_INFO              0x6
static int __init qcom_scm_init(void)
{
	int ret;
	u64 ret1 = 0, x0;

	/* First try a SMC64 call */
	qcom_scm_version = QCOM_SCM_ARMV8_64;
	x0 = QCOM_SCM_SIP_FNID(QCOM_SCM_SVC_INFO, IS_CALL_AVAIL_CMD) | SMC_ATOMIC_MASK;
	ret = __qcom_scm_call_armv8_64(x0 | SMC64_MASK, QCOM_SCM_ARGS(1), x0, 0, 0, 0,
				  &ret1, NULL, NULL);
	if (ret || !ret1) {
		/* Try SMC32 call */
		ret1 = 0;
		ret = __qcom_scm_call_armv8_32(x0, QCOM_SCM_ARGS(1), x0, 0, 0,
						0, &ret1, NULL, NULL);
		if (ret || !ret1)
			qcom_scm_version = QCOM_SCM_LEGACY;
		else
			qcom_scm_version = QCOM_SCM_ARMV8_32;
	} else
		qcom_scm_version_mask = SMC64_MASK;

	pr_debug("qcom_scm_call: qcom_scm version is %x, mask is %x\n",
		 qcom_scm_version, qcom_scm_version_mask);

	return 0;
}
early_initcall(qcom_scm_init);
