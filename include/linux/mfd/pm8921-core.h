/*
 * Copyright (c) 2014, Sony Mobile Communications AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MFD_PM8921_CORE_H
#define __MFD_PM8921_CORE_H

#include <linux/err.h>

#if IS_ENABLED(CONFIG_MFD_PM8921_CORE)

int pm8xxx_read_irq_status(int irq);

#else
static inline int pm8xxx_read_irq_status(int irq)
{
	return -ENOSYS;
}

#endif

#endif
