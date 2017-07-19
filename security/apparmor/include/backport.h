/*
 * AppArmor security module
 *
 * This file contains AppArmor file mediation function definitions.
 *
 * Copyright 2014 Canonical Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2 of the
 * License.
 *
 *
 * This is a file of helper macros, defines for backporting newer versions
 * of apparmor to older kernels
 */
#ifndef __AA_BACKPORT_H
#define __AA_BACKPORT_H

#include <linux/types.h>
#include <linux/slab.h>

/* backport to 4.13 - 1751e8a6cb93 ("Rename superblock flags (MS_xyz -> SB_xyz)") */
#define SB_NOUSER MS_NOUSER

/* backport 4.13 - a481f4d91783 ("apparmor: add custom apparmorfs that will be used by policy namespace files") */
#define AAFS_MAGIC	0x5a3c69f0

/* backport 4.12 to 4.11 support __ro_after_init introduce by ca97d939db11 */
#define __lsm_ro_after_init /* nothing */

/* backport 4.12 to 4.11 support kvmalloc interface a7c3e901a46f introduced by */
void *__aa_kvmalloc(size_t size, gfp_t flags);

static inline void *kvmalloc(size_t size, gfp_t flags)
{
	return __aa_kvmalloc(size, flags );
}

static inline void *kvzalloc(size_t size, gfp_t flags)
{
	return __aa_kvmalloc(size, flags | __GFP_ZERO);
}

/* 4.10 backport support LSM name in security_add_hooks d69dece5f5b6 */
#define security_add_hooks(H, C, NAME) (security_add_hooks)(H, C)

#endif /* __AA_BACKPORT_H */
