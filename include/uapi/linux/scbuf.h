/*
 * include/linux/scbuf.h
 *
 * Copyright (c) 2018 Cog Systems Pty Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Secure Camera Buffer client driver API.
 *
 * This driver exposes a single ioctl() operation which is used to obtain a
 * file descriptor for a secure camera buffer that has been exported from
 * another VM.
 */

#ifndef _UAPI__SCBUF_H_
#define _UAPI__SCBUF_H_

#include <linux/ioctl.h>

#define IOCTL_SCBUF_LOOKUP_HANDLE _IO('4', 0x50)

#endif /* _UAPI__SCBUF_H_ */
