/*
 * surface_utils.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _SURFACE_UTILS_H
#define _SURFACE_UTILS_H

#include <linux/types.h>
#include <linux/regulator/driver.h>

extern struct kobject *telemetry_kobj;

#pragma pack(1)
#define MCFG_MODE 0
#define CUST_MODE 1
#define MAX_VERSION_LEN 16

typedef struct XBL_HLOS_HOB
{
    uint8_t BoardID;                          // (00)
    uint8_t BatteryPresent;                   // (01) Indicates battery presence: 0 - battery absent, 1 - battery present
    uint8_t HwInitRetries;                    // (02) Indicates retries attempted to initialize descrete hardware circuit
    uint8_t IsCustomerMode;                   // (03) Indicates whether the device is in Manufacturing Mode: 0 - in manuf mode, 1 - in Customer mode
    uint8_t IsActMode;                        // (04) Indicates whether device has act mode enabled. 0 - disabled 1 - enabled
    uint8_t PmicResetReason;                  // (05) PmicResetReason: 9 - battery driver triggered
    char    TouchFWVersion[MAX_VERSION_LEN];  // (06) Current Touch Firmware version number
    uint16_t OCPErrorLocation;                // (07) Identify which power rail has the OCP Error
											  //      Bit(s)     Meaning
											  //      15         More than one OCP error occurred
											  //      14-12      PMIC
											  //      11-7       SMPS
											  //      6-2        LDO
											  //      1-0        BOB
} *PXBL_HLOS_HOB;

#pragma pack()

bool get_manuf_mode(void);
uint16_t get_ocp_error_info(void);
int xbl_hlos_hob_init(void);
int xbl_hlos_hob_deinit(void);
int telemetry_init(void);
int get_act_mode(void);
int get_pmic_reset_reason(void);
int get_touch_fw_version(char *);

#endif
