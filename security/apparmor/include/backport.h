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

/* backport to 4.13 - 1751e8a6cb93 ("Rename superblock flags (MS_xyz -> SB_xyz)") */
#define SB_NOUSER MS_NOUSER

/* backport 4.13 - a481f4d91783 ("apparmor: add custom apparmorfs that will be used by policy namespace files") */
#define AAFS_MAGIC	0x5a3c69f0

/* backport 4.12 to 4.11 support __ro_after_init introduce by ca97d939db11 */
#define __lsm_ro_after_init /* nothing */

#endif /* __AA_BACKPORT_H */
