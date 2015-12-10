/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/slab.h>

#include "msm_vidc_debug.h"
#include "vidc_hfi_api.h"
#include "hfi/venus/venus_hfi.h"

void *vidc_hfi_init(enum vidc_hfi_type hfi_type, u32 device_id,
		    struct vidc_resources *res)
{
	struct hfi_device *hdev;

	switch (hfi_type) {
	case VIDC_HFI_VENUS:
		hdev = venus_hfi_initialize(device_id, res);
		break;
	default:
		dprintk(VIDC_ERR, "Unsupported host-firmware interface\n");
		return ERR_PTR(-ENOTSUPP);
	}

	return hdev;
}

void vidc_hfi_deinit(enum vidc_hfi_type hfi_type, struct hfi_device *hdev)
{
	switch (hfi_type) {
	case VIDC_HFI_VENUS:
		venus_hfi_deinitialize(hdev);
		break;
	default:
		dprintk(VIDC_ERR, "Unsupported host-firmware interface\n");
		return;
	}
}
