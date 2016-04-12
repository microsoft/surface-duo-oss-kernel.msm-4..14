/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#ifndef __DT_BINDINGS_QCOM_IDS_H
#define __DT_BINDINGS_QCOM_IDS_H

/* qcom,msm-id */
#define QCOM_ID_MSM8916		206
#define QCOM_ID_APQ8016		247
#define QCOM_ID_MSM8216		248
#define QCOM_ID_MSM8116		249
#define QCOM_ID_MSM8616		250

/* qcom,board-id */
#define QCOM_BRD_ID(a, major, minor) \
	(((major & 0xff) << 16) | ((minor & 0xff) << 8) | QCOM_BRD_ID_##a)

#define QCOM_BRD_ID_MTP		8
#define QCOM_BRD_ID_DRAGONBRD	10
#define QCOM_BRD_ID_SBC		24

#define QCOM_BRD_SUBTYPE_DEFAULT		0
#define QCOM_BRD_SUBTYPE_MTP8916_SMB1360	1

#endif
