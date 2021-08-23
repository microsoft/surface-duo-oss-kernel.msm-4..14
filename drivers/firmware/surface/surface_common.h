/*
 * surface_common.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SURFACE_COMMON_H
#define _SURFACE_COMMON_H
#include <linux/kernel.h>

#define ATTR(x, y)							\
	static struct kobj_attribute x##_attribute =		\
		__ATTR(x, 0664, y,		\
				NULL);

#define ATTRCMP(x) (0 == strcmp(attr->attr.name, #x))

#define ATTR_LIST(x)	& x ## _attribute.attr

typedef int (*lib_initialize)(struct kobject* kobj);
typedef int (*lib_deinitialize)(struct kobject* kobj);

#endif