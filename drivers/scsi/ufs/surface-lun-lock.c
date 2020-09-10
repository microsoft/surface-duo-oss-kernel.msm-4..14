/*
 * surface-lun-lock.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/ctype.h>
#include <linux/genhd.h>
#include <linux/blktrace_api.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include "surface-lun-lock.h"

static char buildvariant[BUILD_VARIANT_SZ];
static int boot_mode;

static int __init read_buildvariant(char *line) {
	strlcpy(buildvariant, line, sizeof(buildvariant));
	return 1;
}

__setup("buildvariant=", read_buildvariant);

/* buildvariant userdebug is used to identify dev builds
 *
 */
static inline bool is_userdebug(void) {
	static const char typeuserdebug[] = "userdebug";

	return !strncmp(buildvariant, typeuserdebug, sizeof(typeuserdebug));
}

static int __init read_boot_mode(char *line) {
	get_option(&line, &boot_mode);
	return 1;
}

__setup("androidboot.force_normal_boot=", read_boot_mode);

/* androidboot.force_normal_boot = 1 denotes normal boot
 * else it indicates recovery mode
 */
static inline bool is_normal_boot(void) {
	return boot_mode == 1;
}


bool can_enable_f_poweron_wpen(void) {
	/* See 46819
	   Temporarily disable LUN locking until we fix
	if (get_manuf_mode() || is_userdebug() || !is_normal_boot()) {
		return false;
	}

	return true;
	*/
    return false;
}
