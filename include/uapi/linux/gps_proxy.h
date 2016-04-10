/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#ifndef __GPS_PROXY_H__
#define __GPS_PROXY_H__

#define QMI_LOC_NMEA_STRING_MAX_LENGTH_V02 201

enum QGPS_TTY_IOCTL_CMDS {
	QGPS_REGISTER_HANDLE_IOC = 0,
	QGPS_SEND_NMEA_IOC,
	QGPS_IS_ACTIVE_IOC,
};

#define QGPS_IOC_MAGIC 		'q'
#define QGPS_REGISTER_HANDLE 	_IO(QGPS_IOC_MAGIC, QGPS_REGISTER_HANDLE_IOC)
#define QGPS_SEND_NMEA 		_IO(QGPS_IOC_MAGIC, QGPS_SEND_NMEA_IOC)
#define QGPS_IS_ACTIVE 		_IO(QGPS_IOC_MAGIC, QGPS_IS_ACTIVE_IOC)

struct gps_proxy_data {
	size_t nmea_length;
	char nmea_string[QMI_LOC_NMEA_STRING_MAX_LENGTH_V02];
};

#endif /* __GPS_PROXY_H__ */
