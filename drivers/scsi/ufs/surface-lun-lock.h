/*
 * surface-lun-lock.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef DRIVERS_SCSI_UFS_SURFACE_LUN_LOCK_H_
#define DRIVERS_SCSI_UFS_SURFACE_LUN_LOCK_H_

#include <linux/sizes.h>
#include <linux/types.h>

// user or userdebug
#define BUILD_VARIANT_SZ 25
#define BOOT_MODE_SZ 30

extern bool get_manuf_mode(void);

bool can_enable_f_poweron_wpen(void);
#endif /* DRIVERS_SCSI_UFS_SURFACE_LUN_LOCK_H_ */
