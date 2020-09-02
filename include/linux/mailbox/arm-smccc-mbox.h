/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_ARM_SMCCC_MBOX_H_
#define _LINUX_ARM_SMCCC_MBOX_H_

#include <linux/types.h>

/**
 * struct arm_smccc_mbox_cmd - ARM SMCCC message structure
 * @args_smccc32/64:	actual usage of registers is up to the protocol
 *			(within the SMCCC limits)
 */
struct arm_smccc_mbox_cmd {
	union {
		u32 args_smccc32[6];
		u64 args_smccc64[6];
	};
};

#endif /* _LINUX_ARM_SMCCC_MBOX_H_ */
