/*
 * mcfg_check.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _MCFG_CHECK_H
#define _MCFG_CHECK_H

#include "surface_common.h"

int mcfg_check_init(struct kobject* kobj);
int mcfg_check_deinit(struct kobject* kobj);

#endif
