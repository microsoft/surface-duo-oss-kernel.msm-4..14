/*
 * BQ27xxx battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 * Copyright (C) 2017 Liam Breck <kernel@networkimprov.net>
 * Copyright (c) 2020 Microsoft Corporation
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Datasheets:
 * http://www.ti.com/product/bq27000
 * http://www.ti.com/product/bq27200
 * http://www.ti.com/product/bq27010
 * http://www.ti.com/product/bq27210
 * http://www.ti.com/product/bq27500
 * http://www.ti.com/product/bq27510-g1
 * http://www.ti.com/product/bq27510-g2
 * http://www.ti.com/product/bq27510-g3
 * http://www.ti.com/product/bq27520-g4
 * http://www.ti.com/product/bq27520-g1
 * http://www.ti.com/product/bq27520-g2
 * http://www.ti.com/product/bq27520-g3
 * http://www.ti.com/product/bq27520-g4
 * http://www.ti.com/product/bq27530-g1
 * http://www.ti.com/product/bq27531-g1
 * http://www.ti.com/product/bq27541-g1
 * http://www.ti.com/product/bq27542-g1
 * http://www.ti.com/product/bq27546-g1
 * http://www.ti.com/product/bq27742-g1
 * http://www.ti.com/product/bq27545-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27621-g1
 */


#include <linux/debugfs.h>  // MSCHANGE adding debugfs node to fg driver
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <linux/power/bq27xxx_battery.h>

#define BQ27XXX_MANUFACTURER	"Texas Instruments"

/* BQ27XXX Flags */
#define BQ27XXX_FLAG_DSC	BIT(0)
#define BQ27XXX_FLAG_SOCF	BIT(1) /* State-of-Charge threshold final */
#define BQ27XXX_FLAG_SOC1	BIT(2) /* State-of-Charge threshold 1 */
#define BQ27XXX_FLAG_CFGUP	BIT(4)
#define BQ27XXX_FLAG_FC		BIT(9)
#define BQ27XXX_FLAG_OTD	BIT(14)
#define BQ27XXX_FLAG_OTC	BIT(15)
#define BQ27XXX_FLAG_UT		BIT(14)
#define BQ27XXX_FLAG_OT		BIT(15)

/* BQ27000 has different layout for Flags register */
#define BQ27000_FLAG_EDVF	BIT(0) /* Final End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_EDV1	BIT(1) /* First End-of-Discharge-Voltage flag */
#define BQ27000_FLAG_CI		BIT(4) /* Capacity Inaccurate flag */
#define BQ27000_FLAG_FC		BIT(5)
#define BQ27000_FLAG_CHGS	BIT(7) /* Charge state flag */

/* control register params */
#define BQ27XXX_SEALED			0x20
#define BQ27XXX_SET_CFGUPDATE		0x13
#define BQ27XXX_SOFT_RESET		0x42
#define BQ27XXX_RESET			0x41

#define BQ27XXX_RS			(20) /* Resistor sense mOhm */
#define BQ27XXX_POWER_CONSTANT		(29200) /* 29.2 µV^2 * 1000 */
#define BQ27XXX_CURRENT_CONSTANT	(3570) /* 3.57 µV * 1000 */

#define INVALID_REG_ADDR	0xff

// MSCHANGE adding debugfs node to fg driver
#define TEMPERATURE_OVERRIDE(OverrideTemperature)	(OverrideTemperature <= MAX_ALLOWED_TEMPERATURE ? 1:0) // allow user to override the pack temperature
#define RSOC_OVERRIDE(OverrideRSOC)					(OverrideRSOC <= MAX_ALLOWED_RSOC ? 1:0) // allow user to override the pack rsoc

/*
 * bq27xxx_reg_index - Register names
 *
 * These are indexes into a device's register mapping array.
 */

enum bq27xxx_reg_index {
	BQ27XXX_REG_CTRL = 0,	/* Control */
	BQ27XXX_REG_TEMP,	/* Temperature */
	BQ27XXX_REG_INT_TEMP,	/* Internal Temperature */
	BQ27XXX_REG_VOLT,	/* Voltage */
	BQ27XXX_REG_AI,		/* Average Current */
	BQ27XXX_REG_FLAGS,	/* Flags */
	BQ27XXX_REG_TTE,	/* Time-to-Empty */
	BQ27XXX_REG_TTF,	/* Time-to-Full */
	BQ27XXX_REG_TTES,	/* Time-to-Empty Standby */
	BQ27XXX_REG_TTECP,	/* Time-to-Empty at Constant Power */
	BQ27XXX_REG_NAC,	/* Nominal Available Capacity */
	BQ27XXX_REG_FCC,	/* Full Charge Capacity */
	BQ27XXX_REG_CYCT,	/* Cycle Count */
	BQ27XXX_REG_AE,		/* Available Energy */
	BQ27XXX_REG_SOC,	/* State-of-Charge */
	BQ27XXX_REG_DCAP,	/* Design Capacity */
	BQ27XXX_REG_AP,		/* Average Power */
	BQ27XXX_REG_PROTECTOR_STATUS,  /* Protector Status */    // MSCHANGE enabling faultstatus query
	BQ27XXX_REG_PROTECTOR_STATE,   /* Protector State */     // MSCHANGE
	BQ27XXX_REG_STATE_OF_HEALTH,   /* State Of Health */	 // MSCHANGE enabling fg telemetry query
	BQ27XXX_REG_SAFETY_STATUS, 	   /* Safety Status */       // MSCHANGE adding debugfs
	BQ27XXX_REG_UNFILTERED_FCC,    /* Unfiltered FCC */      // MSCHANGE adding debugfs
	BQ27XXX_REG_UNFILTERED_RM, 	   /* Unfiltered RM */       // MSCHANGE adding debugfs
	BQ27XXX_REG_PASSED_CHARGE, 	   /* Passed Charge */       // MSCHANGE adding debugfs
	BQ27XXX_REG_DOD0, 			   /* DOD0 */                // MSCHANGE adding debugfs
	BQ27XXX_REG_DODatEOC, 		   /* DODatEOC */            // MSCHANGE adding debugfs
	BQ27XXX_REG_QSTART, 		   /* QSTART */              // MSCHANGE adding debugfs
	BQ27XXX_REG_FAST_QMAX, 		   /* FAST QMAX */           // MSCHANGE adding debugfs
	BQ27XXX_DM_CTRL,	/* Block Data Control */
	BQ27XXX_DM_CLASS,	/* Data Class */
	BQ27XXX_DM_BLOCK,	/* Data Block */
	BQ27XXX_DM_DATA,	/* Block Data */
	BQ27XXX_DM_CKSUM,	/* Block Data Checksum */
	BQ27XXX_REG_MAX,	/* sentinel */
};

#define BQ27XXX_DM_REG_ROWS \
	[BQ27XXX_DM_CTRL] = 0x61,  \
	[BQ27XXX_DM_CLASS] = 0x3e, \
	[BQ27XXX_DM_BLOCK] = 0x3f, \
	[BQ27XXX_DM_DATA] = 0x40,  \
	[BQ27XXX_DM_CKSUM] = 0x60

/* Register mappings */
static u8
	bq27000_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = 0x18,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = 0x22,
		[BQ27XXX_REG_SOC] = 0x0b,
		[BQ27XXX_REG_DCAP] = 0x76,
		[BQ27XXX_REG_AP] = 0x24,
		[BQ27XXX_DM_CTRL] = INVALID_REG_ADDR,
		[BQ27XXX_DM_CLASS] = INVALID_REG_ADDR,
		[BQ27XXX_DM_BLOCK] = INVALID_REG_ADDR,
		[BQ27XXX_DM_DATA] = INVALID_REG_ADDR,
		[BQ27XXX_DM_CKSUM] = INVALID_REG_ADDR,
	},
	bq27010_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = 0x18,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x0b,
		[BQ27XXX_REG_DCAP] = 0x76,
		[BQ27XXX_REG_AP] = INVALID_REG_ADDR,
		[BQ27XXX_DM_CTRL] = INVALID_REG_ADDR,
		[BQ27XXX_DM_CLASS] = INVALID_REG_ADDR,
		[BQ27XXX_DM_BLOCK] = INVALID_REG_ADDR,
		[BQ27XXX_DM_DATA] = INVALID_REG_ADDR,
		[BQ27XXX_DM_CKSUM] = INVALID_REG_ADDR,
	},
	bq2750x_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x28,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = 0x1a,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = INVALID_REG_ADDR,
		BQ27XXX_DM_REG_ROWS,
	},
#define bq2751x_regs bq27510g3_regs
#define bq2752x_regs bq27510g3_regs
	bq27500_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = 0x18,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = 0x22,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
#define bq27510g1_regs bq27500_regs
#define bq27510g2_regs bq27500_regs
	bq27510g3_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x28,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = 0x1a,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x1e,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x20,
		[BQ27XXX_REG_DCAP] = 0x2e,
		[BQ27XXX_REG_AP] = INVALID_REG_ADDR,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27520g1_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = 0x18,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = INVALID_REG_ADDR,
		[BQ27XXX_REG_AE] = 0x22,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27520g2_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x36,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = 0x18,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = 0x22,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27520g3_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x36,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = 0x26,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = 0x22,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27520g4_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x28,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = 0x1c,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x1e,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x20,
		[BQ27XXX_REG_DCAP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_AP] = INVALID_REG_ADDR,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27530_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x32,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
#define bq27531_regs bq27530_regs
	bq27541_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x28,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x10,  // MSCHANGE we don't want to read the NAC value so changing this register to RemainingCapacity(): 0x10
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x24,
		[BQ27XXX_REG_PROTECTOR_STATUS] = 0x6d,   // MSCHANGE
		[BQ27XXX_REG_PROTECTOR_STATE] = 0x78,   // MSCHANGE
		[BQ27XXX_REG_STATE_OF_HEALTH] = 0x2e,	// MSCHANGE
		[BQ27XXX_REG_SAFETY_STATUS] = 0x1a, 	// MSCHANGE adding debugfs
		[BQ27XXX_REG_UNFILTERED_FCC] = 0x1c,    // MSCHANGE adding debugfs
		[BQ27XXX_REG_UNFILTERED_RM] = 0x20, 	// MSCHANGE adding debugfs
		[BQ27XXX_REG_PASSED_CHARGE] = 0x34, 	// MSCHANGE adding debugfs
		[BQ27XXX_REG_DOD0] = 0x36, 			    // MSCHANGE adding debugfs
		[BQ27XXX_REG_DODatEOC] = 0x62, 		    // MSCHANGE adding debugfs
		[BQ27XXX_REG_QSTART] = 0x64,            // MSCHANGE adding debugfs
		[BQ27XXX_REG_FAST_QMAX] = 0x66, 		// MSCHANGE adding debugfs
		BQ27XXX_DM_REG_ROWS,
	},
#define bq27542_regs bq27541_regs
#define bq27546_regs bq27541_regs
#define bq27742_regs bq27541_regs
	bq27545_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x06,
		[BQ27XXX_REG_INT_TEMP] = 0x28,
		[BQ27XXX_REG_VOLT] = 0x08,
		[BQ27XXX_REG_AI] = 0x14,
		[BQ27XXX_REG_FLAGS] = 0x0a,
		[BQ27XXX_REG_TTE] = 0x16,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x0c,
		[BQ27XXX_REG_FCC] = 0x12,
		[BQ27XXX_REG_CYCT] = 0x2a,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x2c,
		[BQ27XXX_REG_DCAP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_AP] = 0x24,
		BQ27XXX_DM_REG_ROWS,
	},
	bq27421_regs[BQ27XXX_REG_MAX] = {
		[BQ27XXX_REG_CTRL] = 0x00,
		[BQ27XXX_REG_TEMP] = 0x02,
		[BQ27XXX_REG_INT_TEMP] = 0x1e,
		[BQ27XXX_REG_VOLT] = 0x04,
		[BQ27XXX_REG_AI] = 0x10,
		[BQ27XXX_REG_FLAGS] = 0x06,
		[BQ27XXX_REG_TTE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTF] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTES] = INVALID_REG_ADDR,
		[BQ27XXX_REG_TTECP] = INVALID_REG_ADDR,
		[BQ27XXX_REG_NAC] = 0x08,
		[BQ27XXX_REG_FCC] = 0x0e,
		[BQ27XXX_REG_CYCT] = INVALID_REG_ADDR,
		[BQ27XXX_REG_AE] = INVALID_REG_ADDR,
		[BQ27XXX_REG_SOC] = 0x1c,
		[BQ27XXX_REG_DCAP] = 0x3c,
		[BQ27XXX_REG_AP] = 0x18,
		BQ27XXX_DM_REG_ROWS,
	};
#define bq27425_regs bq27421_regs
#define bq27441_regs bq27421_regs
#define bq27621_regs bq27421_regs

static enum power_supply_property bq27000_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27010_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

#define bq2750x_props bq27510g3_props
#define bq2751x_props bq27510g3_props
#define bq2752x_props bq27510g3_props

static enum power_supply_property bq27500_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#define bq27510g1_props bq27500_props
#define bq27510g2_props bq27500_props

static enum power_supply_property bq27510g3_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27520g1_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

#define bq27520g2_props bq27500_props

static enum power_supply_property bq27520g3_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27520g4_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27530_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#define bq27531_props bq27530_props

static enum power_supply_property bq27541_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#define bq27542_props bq27541_props
#define bq27546_props bq27541_props
#define bq27742_props bq27541_props

static enum power_supply_property bq27545_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static enum power_supply_property bq27421_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MANUFACTURER,
};
#define bq27425_props bq27421_props
#define bq27441_props bq27421_props
#define bq27621_props bq27421_props

struct bq27xxx_dm_reg {
	u8 subclass_id;
	u8 offset;
	u8 bytes;
	u16 min, max;
};

// MSCHANGE adding data flash registers for FG telemetry queries
enum bq27xxx_dm_reg_id {
	BQ27XXX_DM_DESIGN_CAPACITY = 0,
	BQ27XXX_DM_DESIGN_ENERGY,
	BQ27XXX_DM_TERMINATE_VOLTAGE,
	BQ27XXX_DM_CHARGING_VOLTAGE,
	BQ27XXX_DM_LT_MAX_TEMP,
	BQ27XXX_DM_LT_MIN_TEMP,
	BQ27XXX_DM_LT_MAX_PACK_VOLTAGE,
	BQ27XXX_DM_LT_MIN_PACK_VOLTAGE,
	BQ27XXX_DM_LT_MAX_CHG_CURRENT,
	BQ27XXX_DM_LT_MAX_DSG_CURRENT,
	BQ27XXX_DM_LT_FLASH_COUNT,
	BQ27XXX_DM_LT_AFE_STATUS,
	BQ27XXX_DM_QMAX_CELL0,
	BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM,
	BQ27XXX_DM_AVG_I_LAST_RUN,
	BQ27XXX_DM_AVG_P_LAST_RUN,
	BQ27XXX_DM_DELTA_VOLTAGE,
};

#define bq27000_dm_regs 0
#define bq27010_dm_regs 0
#define bq2750x_dm_regs 0
#define bq2751x_dm_regs 0
#define bq2752x_dm_regs 0

#if 0 /* not yet tested */
static struct bq27xxx_dm_reg bq27500_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   = { 48, 10, 2,    0, 65535 },
	[BQ27XXX_DM_DESIGN_ENERGY]     = { }, /* missing on chip */
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = { 80, 48, 2, 1000, 32767 },
};
#else
#define bq27500_dm_regs 0
#endif

/* todo create data memory definitions from datasheets and test on chips */
#define bq27510g1_dm_regs 0
#define bq27510g2_dm_regs 0
#define bq27510g3_dm_regs 0
#define bq27520g1_dm_regs 0
#define bq27520g2_dm_regs 0
#define bq27520g3_dm_regs 0
#define bq27520g4_dm_regs 0
#define bq27530_dm_regs 0
#define bq27531_dm_regs 0
#define bq27541_dm_regs 0
#define bq27542_dm_regs 0
#define bq27546_dm_regs 0
//#define bq27742_dm_regs 0  // MSCHANGE defined these for FG telemetry queries

// MSCHANGE adding data flash register reads for FG telemetry queries
static struct bq27xxx_dm_reg bq27742_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   		= { 82, 10, 2,     0,  8000  },
	[BQ27XXX_DM_DESIGN_ENERGY]     		= { 82, 12, 2,     0,  32767 },
	[BQ27XXX_DM_TERMINATE_VOLTAGE] 		= { 82, 16, 2,  2500,  3700  },
	[BQ27XXX_DM_CHARGING_VOLTAGE]  		= { 34, 0,  2,  4000,  5000  },
	[BQ27XXX_DM_LT_MAX_TEMP]  			= { 59, 0,  2,  -600,  1400  },
	[BQ27XXX_DM_LT_MIN_TEMP]  			= { 59, 2,  2,  -600,  1400  },
	[BQ27XXX_DM_LT_MAX_PACK_VOLTAGE]  	= { 59, 4,  2,     0,  32767 },
	[BQ27XXX_DM_LT_MIN_PACK_VOLTAGE]  	= { 59, 6,  2,     0,  32767 },
	[BQ27XXX_DM_LT_MAX_CHG_CURRENT]  	= { 59, 8,  2,-32767,  32767 },
	[BQ27XXX_DM_LT_MAX_DSG_CURRENT]  	= { 59, 10, 2,-32767,  32767 },
	[BQ27XXX_DM_LT_FLASH_COUNT]			= { 60, 0,  2,     0,  32767 },
	[BQ27XXX_DM_LT_AFE_STATUS]			= { 60, 2,  1,     0,  255   },
	[BQ27XXX_DM_QMAX_CELL0]				= { 82, 0,  2,     0,  14500 },
	[BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM]	= { 82, 3,  2,     0,  5000  },
	[BQ27XXX_DM_AVG_I_LAST_RUN]			= { 82, 5,  2,-32768,  0     },
	[BQ27XXX_DM_AVG_P_LAST_RUN]			= { 82, 7,  2,-32768,  0     },
	[BQ27XXX_DM_DELTA_VOLTAGE]			= { 82, 9,  2,     0,  32767 },
};

#if 0 /* not yet tested */
static struct bq27xxx_dm_reg bq27545_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   = { 48, 23, 2,    0, 32767 },
	[BQ27XXX_DM_DESIGN_ENERGY]     = { 48, 25, 2,    0, 32767 },
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = { 80, 67, 2, 2800,  3700 },
};
#else
#define bq27545_dm_regs 0
#endif

static struct bq27xxx_dm_reg bq27421_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   = { 82, 10, 2,    0,  8000 },
	[BQ27XXX_DM_DESIGN_ENERGY]     = { 82, 12, 2,    0, 32767 },
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = { 82, 16, 2, 2500,  3700 },
};

static struct bq27xxx_dm_reg bq27425_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   = { 82, 12, 2,    0, 32767 },
	[BQ27XXX_DM_DESIGN_ENERGY]     = { 82, 14, 2,    0, 32767 },
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = { 82, 18, 2, 2800,  3700 },
};

#if 0 /* not yet tested */
#define bq27441_dm_regs bq27421_dm_regs
#else
#define bq27441_dm_regs 0
#endif

#if 0 /* not yet tested */
static struct bq27xxx_dm_reg bq27621_dm_regs[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY]   = { 82, 3, 2,    0,  8000 },
	[BQ27XXX_DM_DESIGN_ENERGY]     = { 82, 5, 2,    0, 32767 },
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = { 82, 9, 2, 2500,  3700 },
};
#else
#define bq27621_dm_regs 0
#endif

#define BQ27XXX_O_ZERO	0x00000001
#define BQ27XXX_O_OTDC	0x00000002
#define BQ27XXX_O_UTOT  0x00000004
#define BQ27XXX_O_CFGUP	0x00000008
#define BQ27XXX_O_RAM	0x00000010

#define BQ27XXX_DATA(ref, key, opt) {		\
	.opts = (opt),				\
	.unseal_key = key,			\
	.regs  = ref##_regs,			\
	.dm_regs = ref##_dm_regs,		\
	.props = ref##_props,			\
	.props_size = ARRAY_SIZE(ref##_props) }

static struct {
	u32 opts;
	u32 unseal_key;
	u8 *regs;
	struct bq27xxx_dm_reg *dm_regs;
	enum power_supply_property *props;
	size_t props_size;
} bq27xxx_chip_data[] = {
	[BQ27000]   = BQ27XXX_DATA(bq27000,   0         , BQ27XXX_O_ZERO),
	[BQ27010]   = BQ27XXX_DATA(bq27010,   0         , BQ27XXX_O_ZERO),
	[BQ2750X]   = BQ27XXX_DATA(bq2750x,   0         , BQ27XXX_O_OTDC),
	[BQ2751X]   = BQ27XXX_DATA(bq2751x,   0         , BQ27XXX_O_OTDC),
	[BQ2752X]   = BQ27XXX_DATA(bq2752x,   0         , BQ27XXX_O_OTDC),
	[BQ27500]   = BQ27XXX_DATA(bq27500,   0x04143672, BQ27XXX_O_OTDC),
	[BQ27510G1] = BQ27XXX_DATA(bq27510g1, 0         , BQ27XXX_O_OTDC),
	[BQ27510G2] = BQ27XXX_DATA(bq27510g2, 0         , BQ27XXX_O_OTDC),
	[BQ27510G3] = BQ27XXX_DATA(bq27510g3, 0         , BQ27XXX_O_OTDC),
	[BQ27520G1] = BQ27XXX_DATA(bq27520g1, 0         , BQ27XXX_O_OTDC),
	[BQ27520G2] = BQ27XXX_DATA(bq27520g2, 0         , BQ27XXX_O_OTDC),
	[BQ27520G3] = BQ27XXX_DATA(bq27520g3, 0         , BQ27XXX_O_OTDC),
	[BQ27520G4] = BQ27XXX_DATA(bq27520g4, 0         , BQ27XXX_O_OTDC),
	[BQ27530]   = BQ27XXX_DATA(bq27530,   0         , BQ27XXX_O_UTOT),
	[BQ27531]   = BQ27XXX_DATA(bq27531,   0         , BQ27XXX_O_UTOT),
	[BQ27541]   = BQ27XXX_DATA(bq27541,   0         , BQ27XXX_O_OTDC),
	[BQ27542]   = BQ27XXX_DATA(bq27542,   0         , BQ27XXX_O_OTDC),
	[BQ27546]   = BQ27XXX_DATA(bq27546,   0         , BQ27XXX_O_OTDC),
	[BQ27742]   = BQ27XXX_DATA(bq27742,   0			, BQ27XXX_O_OTDC),
	[BQ27545]   = BQ27XXX_DATA(bq27545,   0x04143672, BQ27XXX_O_OTDC),
	[BQ27421]   = BQ27XXX_DATA(bq27421,   0x80008000, BQ27XXX_O_UTOT | BQ27XXX_O_CFGUP | BQ27XXX_O_RAM),
	[BQ27425]   = BQ27XXX_DATA(bq27425,   0x04143672, BQ27XXX_O_UTOT | BQ27XXX_O_CFGUP),
	[BQ27441]   = BQ27XXX_DATA(bq27441,   0x80008000, BQ27XXX_O_UTOT | BQ27XXX_O_CFGUP | BQ27XXX_O_RAM),
	[BQ27621]   = BQ27XXX_DATA(bq27621,   0x80008000, BQ27XXX_O_UTOT | BQ27XXX_O_CFGUP | BQ27XXX_O_RAM),
};

static DEFINE_MUTEX(bq27xxx_list_lock);
static LIST_HEAD(bq27xxx_battery_devices);

#define BQ27XXX_MSLEEP(i) usleep_range((i)*1000, (i)*1000+500)

#define BQ27XXX_DM_SZ	32

/**
 * struct bq27xxx_dm_buf - chip data memory buffer
 * @class: data memory subclass_id
 * @block: data memory block number
 * @data: data from/for the block
 * @has_data: true if data has been filled by read
 * @dirty: true if data has changed since last read/write
 *
 * Encapsulates info required to manage chip data memory blocks.
 */
struct bq27xxx_dm_buf {
	u8 class;
	u8 block;
	u8 data[BQ27XXX_DM_SZ];
	bool has_data, dirty;
};

#define BQ27XXX_DM_BUF(di, i) { \
	.class = (di)->dm_regs[i].subclass_id, \
	.block = (di)->dm_regs[i].offset / BQ27XXX_DM_SZ, \
}

static inline u16 *bq27xxx_dm_reg_ptr(struct bq27xxx_dm_buf *buf,
				      struct bq27xxx_dm_reg *reg)
{
	if (buf->class == reg->subclass_id &&
	    buf->block == reg->offset / BQ27XXX_DM_SZ)
		return (u16 *) (buf->data + reg->offset % BQ27XXX_DM_SZ);

	return NULL;
}

static const char * const bq27xxx_dm_reg_name[] = {
	[BQ27XXX_DM_DESIGN_CAPACITY] = "design-capacity",
	[BQ27XXX_DM_DESIGN_ENERGY] = "design-energy",
	[BQ27XXX_DM_TERMINATE_VOLTAGE] = "terminate-voltage",
};


static bool bq27xxx_dt_to_nvm = true;
module_param_named(dt_monitored_battery_updates_nvm, bq27xxx_dt_to_nvm, bool, 0444);
MODULE_PARM_DESC(dt_monitored_battery_updates_nvm,
	"Devicetree monitored-battery config updates data memory on NVM/flash chips.\n"
	"Users must set this =0 when installing a different type of battery!\n"
	"Default is =1."
#ifndef CONFIG_BATTERY_BQ27XXX_DT_UPDATES_NVM
	"\nSetting this affects future kernel updates, not the current configuration."
#endif
);

static int poll_interval_param_set(const char *val, const struct kernel_param *kp)
{
	struct bq27xxx_device_info *di;
	unsigned int prev_val = *(unsigned int *) kp->arg;
	int ret;

	ret = param_set_uint(val, kp);
	if (ret < 0 || prev_val == *(unsigned int *) kp->arg)
		return ret;

	mutex_lock(&bq27xxx_list_lock);
	list_for_each_entry(di, &bq27xxx_battery_devices, list) {
		cancel_delayed_work_sync(&di->work);
		schedule_delayed_work(&di->work, 0);
	}
	mutex_unlock(&bq27xxx_list_lock);

	return ret;
}

static const struct kernel_param_ops param_ops_poll_interval = {
	.get = param_get_uint,
	.set = poll_interval_param_set,
};

static unsigned int poll_interval = 360;
module_param_cb(poll_interval, &param_ops_poll_interval, &poll_interval, 0644);
MODULE_PARM_DESC(poll_interval,
		 "battery poll interval in seconds - 0 disables polling");

/*
 * Common code for BQ27xxx devices
 */
// MSCHANGE adding retries for i2c reads
#define MAX_FG_I2C_RETRIES 3
static inline int bq27xxx_read(struct bq27xxx_device_info *di, int reg_index,
			       bool single)
{
	int ret, retries = 0;

	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -EINVAL;

	do {
		ret = di->bus.read(di, di->regs[reg_index], single);
	} while (ret < 0 && retries++ < MAX_FG_I2C_RETRIES);

	if (ret < 0)
		dev_dbg(di->dev, "failed to read register 0x%02x (index %d)\n",
			di->regs[reg_index], reg_index);

	return ret;
}

// MSCHANGE adding retries for i2c writes
static inline int bq27xxx_write(struct bq27xxx_device_info *di, int reg_index,
				u16 value, bool single)
{
	int ret, retries = 0;

	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -EINVAL;

	if (!di->bus.write)
		return -EPERM;

	do {
		ret = di->bus.write(di, di->regs[reg_index], value, single);
	} while (ret < 0 && retries++ < MAX_FG_I2C_RETRIES);

	if (ret < 0)
		dev_dbg(di->dev, "failed to write register 0x%02x (index %d)\n",
			di->regs[reg_index], reg_index);

	return ret;
}

// MSCHANGE adding retries for i2c reads
static inline int bq27xxx_read_block(struct bq27xxx_device_info *di, int reg_index,
				     u8 *data, int len)
{
	int ret, retries = 0;

	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -EINVAL;

	if (!di->bus.read_bulk)
		return -EPERM;

	do {
		ret = di->bus.read_bulk(di, di->regs[reg_index], data, len);
	} while (ret < 0 && retries++ < MAX_FG_I2C_RETRIES);
	if (ret < 0)
		dev_dbg(di->dev, "failed to read_bulk register 0x%02x (index %d)\n",
			di->regs[reg_index], reg_index);

	return ret;
}

// MSCHANGE adding retries for i2c writes
static inline int bq27xxx_write_block(struct bq27xxx_device_info *di, int reg_index,
				      u8 *data, int len)
{
	int ret, retries = 0;

	if (!di || di->regs[reg_index] == INVALID_REG_ADDR)
		return -EINVAL;

	if (!di->bus.write_bulk)
		return -EPERM;

	do {
		ret = di->bus.write_bulk(di, di->regs[reg_index], data, len);
	} while (ret < 0 && retries++ < MAX_FG_I2C_RETRIES);

	if (ret < 0)
		dev_dbg(di->dev, "failed to write_bulk register 0x%02x (index %d)\n",
			di->regs[reg_index], reg_index);

	return ret;
}

static int bq27xxx_battery_seal(struct bq27xxx_device_info *di)
{
	int ret;

	ret = bq27xxx_write(di, BQ27XXX_REG_CTRL, BQ27XXX_SEALED, false);
	if (ret < 0) {
		dev_err(di->dev, "bus error on seal: %d\n", ret);
		return ret;
	}

	return 0;
}

static int bq27xxx_battery_unseal(struct bq27xxx_device_info *di)
{
	int ret;

	if (di->unseal_key == 0) {
		dev_err(di->dev, "unseal failed due to missing key\n");
		return -EINVAL;
	}

	ret = bq27xxx_write(di, BQ27XXX_REG_CTRL, (u16)(di->unseal_key >> 16), false);
	if (ret < 0)
		goto out;

	ret = bq27xxx_write(di, BQ27XXX_REG_CTRL, (u16)di->unseal_key, false);
	if (ret < 0)
		goto out;

	return 0;

out:
	dev_err(di->dev, "bus error on unseal: %d\n", ret);
	return ret;
}

static u8 bq27xxx_battery_checksum_dm_block(struct bq27xxx_dm_buf *buf)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < BQ27XXX_DM_SZ; i++)
		sum += buf->data[i];
	sum &= 0xff;

	return 0xff - sum;
}

static int bq27xxx_battery_read_dm_block(struct bq27xxx_device_info *di,
					 struct bq27xxx_dm_buf *buf)
{
	int ret;

	buf->has_data = false;

	ret = bq27xxx_write(di, BQ27XXX_DM_CLASS, buf->class, true);
	if (ret < 0)
		goto out;

	ret = bq27xxx_write(di, BQ27XXX_DM_BLOCK, buf->block, true);
	if (ret < 0)
		goto out;

	BQ27XXX_MSLEEP(1);

	ret = bq27xxx_read_block(di, BQ27XXX_DM_DATA, buf->data, BQ27XXX_DM_SZ);
	if (ret < 0)
		goto out;

	ret = bq27xxx_read(di, BQ27XXX_DM_CKSUM, true);
	if (ret < 0)
		goto out;

	if ((u8)ret != bq27xxx_battery_checksum_dm_block(buf)) {
		ret = -EINVAL;
		goto out;
	}

	buf->has_data = true;
	buf->dirty = false;

	return 0;

out:
	dev_err(di->dev, "bus error reading chip memory: %d\n", ret);
	return ret;
}

static void bq27xxx_battery_update_dm_block(struct bq27xxx_device_info *di,
					    struct bq27xxx_dm_buf *buf,
					    enum bq27xxx_dm_reg_id reg_id,
					    unsigned int val)
{
	struct bq27xxx_dm_reg *reg = &di->dm_regs[reg_id];
	const char *str = bq27xxx_dm_reg_name[reg_id];
	u16 *prev = bq27xxx_dm_reg_ptr(buf, reg);

	if (prev == NULL) {
		dev_warn(di->dev, "buffer does not match %s dm spec\n", str);
		return;
	}

	if (reg->bytes != 2) {
		dev_warn(di->dev, "%s dm spec has unsupported byte size\n", str);
		return;
	}

	if (!buf->has_data)
		return;

	if (be16_to_cpup(prev) == val) {
		dev_info(di->dev, "%s has %u\n", str, val);
		return;
	}

#ifdef CONFIG_BATTERY_BQ27XXX_DT_UPDATES_NVM
	if (!(di->opts & BQ27XXX_O_RAM) && !bq27xxx_dt_to_nvm) {
#else
	if (!(di->opts & BQ27XXX_O_RAM)) {
#endif
		/* devicetree and NVM differ; defer to NVM */
		dev_warn(di->dev, "%s has %u; update to %u disallowed "
#ifdef CONFIG_BATTERY_BQ27XXX_DT_UPDATES_NVM
			 "by dt_monitored_battery_updates_nvm=0"
#else
			 "for flash/NVM data memory"
#endif
			 "\n", str, be16_to_cpup(prev), val);
		return;
	}

	dev_info(di->dev, "update %s to %u\n", str, val);

	*prev = cpu_to_be16(val);
	buf->dirty = true;
}

static int bq27xxx_battery_cfgupdate_priv(struct bq27xxx_device_info *di, bool active)
{
	const int limit = 100;
	u16 cmd = active ? BQ27XXX_SET_CFGUPDATE : BQ27XXX_SOFT_RESET;
	int ret, try = limit;

	ret = bq27xxx_write(di, BQ27XXX_REG_CTRL, cmd, false);
	if (ret < 0)
		return ret;

	do {
		BQ27XXX_MSLEEP(25);
		ret = bq27xxx_read(di, BQ27XXX_REG_FLAGS, false);
		if (ret < 0)
			return ret;
	} while (!!(ret & BQ27XXX_FLAG_CFGUP) != active && --try);

	if (!try && di->chip != BQ27425) { // 425 has a bug
		dev_err(di->dev, "timed out waiting for cfgupdate flag %d\n", active);
		return -EINVAL;
	}

	if (limit - try > 3)
		dev_warn(di->dev, "cfgupdate %d, retries %d\n", active, limit - try);

	return 0;
}

static inline int bq27xxx_battery_set_cfgupdate(struct bq27xxx_device_info *di)
{
	int ret = bq27xxx_battery_cfgupdate_priv(di, true);
	if (ret < 0 && ret != -EINVAL)
		dev_err(di->dev, "bus error on set_cfgupdate: %d\n", ret);

	return ret;
}

static inline int bq27xxx_battery_soft_reset(struct bq27xxx_device_info *di)
{
	int ret = bq27xxx_battery_cfgupdate_priv(di, false);
	if (ret < 0 && ret != -EINVAL)
		dev_err(di->dev, "bus error on soft_reset: %d\n", ret);

	return ret;
}

static int bq27xxx_battery_write_dm_block(struct bq27xxx_device_info *di,
					  struct bq27xxx_dm_buf *buf)
{
	bool cfgup = di->opts & BQ27XXX_O_CFGUP;
	int ret;

	if (!buf->dirty)
		return 0;

	if (cfgup) {
		ret = bq27xxx_battery_set_cfgupdate(di);
		if (ret < 0)
			return ret;
	}

	ret = bq27xxx_write(di, BQ27XXX_DM_CTRL, 0, true);
	if (ret < 0)
		goto out;

	ret = bq27xxx_write(di, BQ27XXX_DM_CLASS, buf->class, true);
	if (ret < 0)
		goto out;

	ret = bq27xxx_write(di, BQ27XXX_DM_BLOCK, buf->block, true);
	if (ret < 0)
		goto out;

	BQ27XXX_MSLEEP(1);

	ret = bq27xxx_write_block(di, BQ27XXX_DM_DATA, buf->data, BQ27XXX_DM_SZ);
	if (ret < 0)
		goto out;

	ret = bq27xxx_write(di, BQ27XXX_DM_CKSUM,
			    bq27xxx_battery_checksum_dm_block(buf), true);
	if (ret < 0)
		goto out;

	/* DO NOT read BQ27XXX_DM_CKSUM here to verify it! That may cause NVM
	 * corruption on the '425 chip (and perhaps others), which can damage
	 * the chip.
	 */

	if (cfgup) {
		BQ27XXX_MSLEEP(1);
		ret = bq27xxx_battery_soft_reset(di);
		if (ret < 0)
			return ret;
	} else {
		BQ27XXX_MSLEEP(100); /* flash DM updates in <100ms */
	}

	buf->dirty = false;

	return 0;

out:
	if (cfgup)
		bq27xxx_battery_soft_reset(di);

	dev_err(di->dev, "bus error writing chip memory: %d\n", ret);
	return ret;
}

static void bq27xxx_battery_set_config(struct bq27xxx_device_info *di,
				       struct power_supply_battery_info *info)
{
	struct bq27xxx_dm_buf bd = BQ27XXX_DM_BUF(di, BQ27XXX_DM_DESIGN_CAPACITY);
	struct bq27xxx_dm_buf bt = BQ27XXX_DM_BUF(di, BQ27XXX_DM_TERMINATE_VOLTAGE);
	bool updated;

	if (bq27xxx_battery_unseal(di) < 0)
		return;

	if (info->charge_full_design_uah != -EINVAL &&
	    info->energy_full_design_uwh != -EINVAL) {
		bq27xxx_battery_read_dm_block(di, &bd);
		/* assume design energy & capacity are in same block */
		bq27xxx_battery_update_dm_block(di, &bd,
					BQ27XXX_DM_DESIGN_CAPACITY,
					info->charge_full_design_uah / 1000);
		bq27xxx_battery_update_dm_block(di, &bd,
					BQ27XXX_DM_DESIGN_ENERGY,
					info->energy_full_design_uwh / 1000);
	}

	if (info->voltage_min_design_uv != -EINVAL) {
		bool same = bd.class == bt.class && bd.block == bt.block;
		if (!same)
			bq27xxx_battery_read_dm_block(di, &bt);
		bq27xxx_battery_update_dm_block(di, same ? &bd : &bt,
					BQ27XXX_DM_TERMINATE_VOLTAGE,
					info->voltage_min_design_uv / 1000);
	}

	updated = bd.dirty || bt.dirty;

	bq27xxx_battery_write_dm_block(di, &bd);
	bq27xxx_battery_write_dm_block(di, &bt);

	bq27xxx_battery_seal(di);

	if (updated && !(di->opts & BQ27XXX_O_CFGUP)) {
		bq27xxx_write(di, BQ27XXX_REG_CTRL, BQ27XXX_RESET, false);
		BQ27XXX_MSLEEP(300); /* reset time is not documented */
	}
	/* assume bq27xxx_battery_update() is called hereafter */
}

static void bq27xxx_battery_settings(struct bq27xxx_device_info *di)
{
	struct power_supply_battery_info info = {};
	unsigned int min, max;

	if (power_supply_get_battery_info(di->bat, &info) < 0)
		return;

	if (!di->dm_regs) {
		dev_warn(di->dev, "data memory update not supported for chip\n");
		return;
	}

	if (info.energy_full_design_uwh != info.charge_full_design_uah) {
		if (info.energy_full_design_uwh == -EINVAL)
			dev_warn(di->dev, "missing battery:energy-full-design-microwatt-hours\n");
		else if (info.charge_full_design_uah == -EINVAL)
			dev_warn(di->dev, "missing battery:charge-full-design-microamp-hours\n");
	}

	/* assume min == 0 */
	max = di->dm_regs[BQ27XXX_DM_DESIGN_ENERGY].max;
	if (info.energy_full_design_uwh > max * 1000) {
		dev_err(di->dev, "invalid battery:energy-full-design-microwatt-hours %d\n",
			info.energy_full_design_uwh);
		info.energy_full_design_uwh = -EINVAL;
	}

	/* assume min == 0 */
	max = di->dm_regs[BQ27XXX_DM_DESIGN_CAPACITY].max;
	if (info.charge_full_design_uah > max * 1000) {
		dev_err(di->dev, "invalid battery:charge-full-design-microamp-hours %d\n",
			info.charge_full_design_uah);
		info.charge_full_design_uah = -EINVAL;
	}

	min = di->dm_regs[BQ27XXX_DM_TERMINATE_VOLTAGE].min;
	max = di->dm_regs[BQ27XXX_DM_TERMINATE_VOLTAGE].max;
	if ((info.voltage_min_design_uv < min * 1000 ||
	     info.voltage_min_design_uv > max * 1000) &&
	     info.voltage_min_design_uv != -EINVAL) {
		dev_err(di->dev, "invalid battery:voltage-min-design-microvolt %d\n",
			info.voltage_min_design_uv);
		info.voltage_min_design_uv = -EINVAL;
	}

	if ((info.energy_full_design_uwh != -EINVAL &&
	     info.charge_full_design_uah != -EINVAL) ||
	     info.voltage_min_design_uv  != -EINVAL)
		bq27xxx_battery_set_config(di, &info);
}

/*
 * Return the battery State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_soc(struct bq27xxx_device_info *di)
{
	int soc;

	if (di->opts & BQ27XXX_O_ZERO)
		soc = bq27xxx_read(di, BQ27XXX_REG_SOC, true);
	else
		soc = bq27xxx_read(di, BQ27XXX_REG_SOC, false);

	if (soc < 0)
		dev_dbg(di->dev, "error reading State-of-Charge\n");

	return soc;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_charge(struct bq27xxx_device_info *di, u8 reg)
{
	int charge;

	charge = bq27xxx_read(di, reg, false);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	if (di->opts & BQ27XXX_O_ZERO)
		charge *= BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	else
		charge *= 1000;

	return charge;
}

/*
 * Return the battery Nominal available capacity in µAh
 * Or < 0 if something fails.
 */
static inline int bq27xxx_battery_read_nac(struct bq27xxx_device_info *di)
{
	int flags;

	if (di->opts & BQ27XXX_O_ZERO) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, true);
		if (flags >= 0 && (flags & BQ27000_FLAG_CI))
			return -ENODATA;
	}

	return bq27xxx_battery_read_charge(di, BQ27XXX_REG_NAC);
}

/*
 * Return the battery Full Charge Capacity in µAh
 * Or < 0 if something fails.
 */
static inline int bq27xxx_battery_read_fcc(struct bq27xxx_device_info *di)
{
	return bq27xxx_battery_read_charge(di, BQ27XXX_REG_FCC);
}

/*
 * Return the Design Capacity in µAh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_dcap(struct bq27xxx_device_info *di)
{
	int dcap;

	if (di->opts & BQ27XXX_O_ZERO)
		dcap = bq27xxx_read(di, BQ27XXX_REG_DCAP, true);
	else
		dcap = bq27xxx_read(di, BQ27XXX_REG_DCAP, false);

	if (dcap < 0) {
		dev_dbg(di->dev, "error reading initial last measured discharge\n");
		return dcap;
	}

	if (di->opts & BQ27XXX_O_ZERO)
		dcap = (dcap << 8) * BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	else
		dcap *= 1000;

	return dcap;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_energy(struct bq27xxx_device_info *di)
{
	int ae;

	ae = bq27xxx_read(di, BQ27XXX_REG_AE, false);
	if (ae < 0) {
		dev_dbg(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->opts & BQ27XXX_O_ZERO)
		ae *= BQ27XXX_POWER_CONSTANT / BQ27XXX_RS;
	else
		ae *= 1000;

	return ae;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_temperature(struct bq27xxx_device_info *di)
{
	int temp;

	temp = bq27xxx_read(di, BQ27XXX_REG_TEMP, false);
	if (temp < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return temp;
	}

	if (di->opts & BQ27XXX_O_ZERO)
		temp = 5 * temp / 2;

	return temp;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_read_cyct(struct bq27xxx_device_info *di)
{
	int cyct;

	cyct = bq27xxx_read(di, BQ27XXX_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_time(struct bq27xxx_device_info *di, u8 reg)
{
	int tval;

	tval = bq27xxx_read(di, reg, false);
	if (tval < 0) {
		dev_dbg(di->dev, "error reading time register %02x: %d\n",
			reg, tval);
		return tval;
	}

	if (tval == 65535)
		return -ENODATA;

	return tval * 60;
}

/*
 * Read an average power register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_pwr_avg(struct bq27xxx_device_info *di)
{
	int tval;

	tval = bq27xxx_read(di, BQ27XXX_REG_AP, false);
	if (tval < 0) {
		dev_err(di->dev, "error reading average power register  %02x: %d\n",
			BQ27XXX_REG_AP, tval);
		return tval;
	}

	if (di->opts & BQ27XXX_O_ZERO)
		return (tval * BQ27XXX_POWER_CONSTANT) / BQ27XXX_RS;
	else
		return tval;
}

/*
 * Returns true if a battery over temperature condition is detected
 */
static bool bq27xxx_battery_overtemp(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->opts & BQ27XXX_O_OTDC)
		return flags & (BQ27XXX_FLAG_OTC | BQ27XXX_FLAG_OTD);
        if (di->opts & BQ27XXX_O_UTOT)
		return flags & BQ27XXX_FLAG_OT;

	return false;
}

/*
 * Returns true if a battery under temperature condition is detected
 */
static bool bq27xxx_battery_undertemp(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->opts & BQ27XXX_O_UTOT)
		return flags & BQ27XXX_FLAG_UT;

	return false;
}

/*
 * Returns true if a low state of charge condition is detected
 */
static bool bq27xxx_battery_dead(struct bq27xxx_device_info *di, u16 flags)
{
	if (di->opts & BQ27XXX_O_ZERO)
		return flags & (BQ27000_FLAG_EDV1 | BQ27000_FLAG_EDVF);
	else
		return flags & (BQ27XXX_FLAG_SOC1 | BQ27XXX_FLAG_SOCF);
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27xxx_battery_read_health(struct bq27xxx_device_info *di)
{
	int flags;
	bool has_singe_flag = di->opts & BQ27XXX_O_ZERO;

	flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, has_singe_flag);
	if (flags < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", flags);
		return flags;
	}

	/* Unlikely but important to return first */
	if (unlikely(bq27xxx_battery_overtemp(di, flags)))
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	if (unlikely(bq27xxx_battery_undertemp(di, flags)))
		return POWER_SUPPLY_HEALTH_COLD;
	if (unlikely(bq27xxx_battery_dead(di, flags)))
		return POWER_SUPPLY_HEALTH_DEAD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

void bq27xxx_battery_update(struct bq27xxx_device_info *di)
{
	struct bq27xxx_reg_cache cache = {0, };
	bool has_ci_flag = di->opts & BQ27XXX_O_ZERO;
	bool has_singe_flag = di->opts & BQ27XXX_O_ZERO;

	cache.flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, has_singe_flag);
	if ((cache.flags & 0xff) == 0xff)
		cache.flags = -1; /* read error */
	if (cache.flags >= 0) {
		cache.temperature = bq27xxx_battery_read_temperature(di);
		if (has_ci_flag && (cache.flags & BQ27000_FLAG_CI)) {
			dev_info_once(di->dev, "battery is not calibrated! ignoring capacity values\n");
			cache.capacity = -ENODATA;
			cache.energy = -ENODATA;
			cache.time_to_empty = -ENODATA;
			cache.time_to_empty_avg = -ENODATA;
			cache.time_to_full = -ENODATA;
			cache.charge_full = -ENODATA;
			cache.health = -ENODATA;
		} else {
			if (di->regs[BQ27XXX_REG_TTE] != INVALID_REG_ADDR)
				cache.time_to_empty = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTE);
			if (di->regs[BQ27XXX_REG_TTECP] != INVALID_REG_ADDR)
				cache.time_to_empty_avg = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTECP);
			if (di->regs[BQ27XXX_REG_TTF] != INVALID_REG_ADDR)
				cache.time_to_full = bq27xxx_battery_read_time(di, BQ27XXX_REG_TTF);
			cache.charge_full = bq27xxx_battery_read_fcc(di);
			cache.capacity = bq27xxx_battery_read_soc(di);
			if (di->regs[BQ27XXX_REG_AE] != INVALID_REG_ADDR)
				cache.energy = bq27xxx_battery_read_energy(di);
			cache.health = bq27xxx_battery_read_health(di);
		}
		if (di->regs[BQ27XXX_REG_CYCT] != INVALID_REG_ADDR)
			cache.cycle_count = bq27xxx_battery_read_cyct(di);
		if (di->regs[BQ27XXX_REG_AP] != INVALID_REG_ADDR)
			cache.power_avg = bq27xxx_battery_read_pwr_avg(di);

		/* We only have to read charge design full once */
		if (di->charge_design_full <= 0)
			di->charge_design_full = bq27xxx_battery_read_dcap(di);
	}

	if (di->cache.capacity != cache.capacity)
		power_supply_changed(di->bat);

	if (memcmp(&di->cache, &cache, sizeof(cache)) != 0)
		di->cache = cache;

	di->last_update = jiffies;
}
EXPORT_SYMBOL_GPL(bq27xxx_battery_update);

static void bq27xxx_battery_poll(struct work_struct *work)
{
	struct bq27xxx_device_info *di =
			container_of(work, struct bq27xxx_device_info,
				     work.work);

	bq27xxx_battery_update(di);

	if (poll_interval > 0)
		schedule_delayed_work(&di->work, poll_interval * HZ);
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27xxx_battery_current(struct bq27xxx_device_info *di,
				   union power_supply_propval *val)
{
	int curr;
	int flags;

	curr = bq27xxx_read(di, BQ27XXX_REG_AI, false);
	if (curr < 0) {
		dev_err(di->dev, "error reading current\n");
		return curr;
	}

	if (di->opts & BQ27XXX_O_ZERO) {
		flags = bq27xxx_read(di, BQ27XXX_REG_FLAGS, true);
		if (flags & BQ27000_FLAG_CHGS) {
			dev_dbg(di->dev, "negative current!\n");
			curr = -curr;
		}

		val->intval = curr * BQ27XXX_CURRENT_CONSTANT / BQ27XXX_RS;
	} else {
		/* Other gauges return signed value */
		val->intval = (int)((s16)curr) * 1000;
	}

	return 0;
}

static int bq27xxx_battery_status(struct bq27xxx_device_info *di,
				  union power_supply_propval *val)
{
	int status;

	if (di->opts & BQ27XXX_O_ZERO) {
		if (di->cache.flags & BQ27000_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else if (power_supply_am_i_supplied(di->bat))
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	}

	val->intval = status;

	return 0;
}

static int bq27xxx_battery_capacity_level(struct bq27xxx_device_info *di,
					  union power_supply_propval *val)
{
	int level;

	if (di->opts & BQ27XXX_O_ZERO) {
		if (di->cache.flags & BQ27000_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27000_FLAG_EDV1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27000_FLAG_EDVF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	} else {
		if (di->cache.flags & BQ27XXX_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27XXX_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27XXX_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Return the battery Voltage in millivolts
 * Or < 0 if something fails.
 */
static int bq27xxx_battery_voltage(struct bq27xxx_device_info *di,
				   union power_supply_propval *val)
{
	int volt;

	volt = bq27xxx_read(di, BQ27XXX_REG_VOLT, false);
	if (volt < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return volt;
	}

	val->intval = volt * 1000;

	return 0;
}

static int bq27xxx_simple_value(int value,
				union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

static int bq27xxx_battery_read_telemetry(struct bq27xxx_device_info *di, enum power_supply_property psp, union power_supply_propval *val)
{
	int size = 0;
	int telemetry_value = 0;
	int offset = 0;
	int ret,i = 0;
	struct bq27xxx_dm_buf dm_telemetry = BQ27XXX_DM_BUF(di, BQ27XXX_DM_CHARGING_VOLTAGE);

	val->intval = 0;
	return 0;

	switch(psp) {
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
			size = di->dm_regs[BQ27XXX_DM_CHARGING_VOLTAGE].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_CHARGING_VOLTAGE].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_CHARGING_VOLTAGE].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_CHARGING_VOLTAGE].offset;
			break;
		case POWER_SUPPLY_PROP_TEMP_MAX:
			size = di->dm_regs[BQ27XXX_DM_LT_MAX_TEMP].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MAX_TEMP].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MAX_TEMP].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MAX_TEMP].offset;
			break;
		case POWER_SUPPLY_PROP_TEMP_MIN:
			size = di->dm_regs[BQ27XXX_DM_LT_MIN_TEMP].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MIN_TEMP].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MIN_TEMP].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MIN_TEMP].offset;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			size = di->dm_regs[BQ27XXX_DM_LT_MAX_PACK_VOLTAGE].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MAX_PACK_VOLTAGE].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MAX_PACK_VOLTAGE].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MAX_PACK_VOLTAGE].offset;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			size = di->dm_regs[BQ27XXX_DM_LT_MIN_PACK_VOLTAGE].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MIN_PACK_VOLTAGE].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MIN_PACK_VOLTAGE].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MIN_PACK_VOLTAGE].offset;
			break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
			size = di->dm_regs[BQ27XXX_DM_LT_MAX_CHG_CURRENT].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MAX_CHG_CURRENT].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MAX_CHG_CURRENT].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MAX_CHG_CURRENT].offset;
			break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
			size = di->dm_regs[BQ27XXX_DM_LT_MAX_DSG_CURRENT].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_MAX_DSG_CURRENT].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_MAX_DSG_CURRENT].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_MAX_DSG_CURRENT].offset;
			break;
		case POWER_SUPPLY_PROP_UPDATE_NOW:
			size = di->dm_regs[BQ27XXX_DM_LT_FLASH_COUNT].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_FLASH_COUNT].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_FLASH_COUNT].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_FLASH_COUNT].offset;
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNTS:
			size = di->dm_regs[BQ27XXX_DM_LT_AFE_STATUS].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_LT_AFE_STATUS].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_LT_AFE_STATUS].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_LT_AFE_STATUS].offset;
			break;
		case POWER_SUPPLY_PROP_CAPACITY_RAW:
			size = di->dm_regs[BQ27XXX_DM_QMAX_CELL0].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_QMAX_CELL0].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_QMAX_CELL0].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_QMAX_CELL0].offset;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX_LIMIT:
			size = di->dm_regs[BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_VOLTAGE_AT_CHARGE_TERM].offset;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			size = di->dm_regs[BQ27XXX_DM_AVG_I_LAST_RUN].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_AVG_I_LAST_RUN].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_AVG_I_LAST_RUN].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_AVG_I_LAST_RUN].offset;
			break;
		case POWER_SUPPLY_PROP_POWER_NOW:
			size = di->dm_regs[BQ27XXX_DM_AVG_P_LAST_RUN].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_AVG_P_LAST_RUN].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_AVG_P_LAST_RUN].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_AVG_P_LAST_RUN].offset;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
			size = di->dm_regs[BQ27XXX_DM_DELTA_VOLTAGE].bytes;
			dm_telemetry.class = di->dm_regs[BQ27XXX_DM_DELTA_VOLTAGE].subclass_id;
			dm_telemetry.block = di->dm_regs[BQ27XXX_DM_DELTA_VOLTAGE].offset / BQ27XXX_DM_SZ;
			offset = di->dm_regs[BQ27XXX_DM_DELTA_VOLTAGE].offset;
			break;
		default:
			return -EINVAL;
	}

	ret = bq27xxx_battery_read_dm_block(di, &dm_telemetry);
	if (ret < 0)
		return -EINVAL;

	for(i = 0; i < size; i++)
		telemetry_value = telemetry_value | (dm_telemetry.data[offset+i] << (size-1-i)*8);

	val->intval = telemetry_value;
	return 0;
}

// MSCHANGE adding state of health query
static int bq27xxx_battery_read_state_of_health(struct bq27xxx_device_info *di)
{
	int state_of_health;

	state_of_health = bq27xxx_read(di, BQ27XXX_REG_STATE_OF_HEALTH, false);
	if (state_of_health < 0) {
		dev_err(di->dev, "error reading state_of_health\n");
		return state_of_health;
	}

	return state_of_health;
}

// MSCHANGE adding protector status query
static int bq27xxx_battery_read_protector_status(struct bq27xxx_device_info *di)
{
	int protector_status;

	protector_status = bq27xxx_read(di, BQ27XXX_REG_PROTECTOR_STATUS, false);
	if (protector_status < 0) {
		dev_err(di->dev, "error reading protector_status\n");
		return protector_status;
	}

	return protector_status;
}

// MSCHANGE adding protector state query
static int bq27xxx_battery_read_protector_state(struct bq27xxx_device_info *di)
{
	int protector_state;

	protector_state = bq27xxx_read(di, BQ27XXX_REG_PROTECTOR_STATE, false);
	if (protector_state < 0) {
		dev_err(di->dev, "error reading protector_state\n");
		return protector_state;
	}

	return protector_state;
}

// MSCHANGE adding safety_status query
static int bq27xxx_battery_read_safety_status(struct bq27xxx_device_info *di)
{
	int safety_status;

	safety_status = bq27xxx_read(di, BQ27XXX_REG_SAFETY_STATUS, false);
	if (safety_status < 0) {
		dev_err(di->dev, "error reading BQ27XXX_REG_SAFETY_STATUS\n");
		return safety_status;
	}

	return safety_status;
}

// MSCHANGE Manufacturer Info Blocks functions
#define BQ27742_MANUF_INFO_BLOCK_A					 0x01
#define BQ27742_MANUF_INFO_BLOCK_B                   0x02
#define BQ27742_MANUF_INFO_BLOCK_B_NUM_HVT_BYTES     0x04

// MSCHANGE checksum calculation for Manuf Block B write
static u8 bq27xxx_checksum_manufacturer_info_B(u8 *data)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < BQ27XXX_DM_SZ; i++)
	{
		sum += data[i];
	}

	sum &= 0xff;

	return 0xff - sum;
}

// MSCHANGE Manuf Name read from Manufacturer Info Block A
static int bq27xxx_read_manufacturer_name(struct bq27xxx_device_info *di, union power_supply_propval *val)
{
	int ret;
	u8 data[BQ27XXX_DM_SZ];
	static struct BQ27742_MANUF_INFO_TYPE ManufInfoblkA = {0};

	ret = bq27xxx_write(di, BQ27XXX_DM_BLOCK, BQ27742_MANUF_INFO_BLOCK_A, true);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_BLOCK write failed: %d\n", ret);
		goto out;
	}

	BQ27XXX_MSLEEP(1);

	ret = bq27xxx_read_block(di, BQ27XXX_DM_DATA, data, BQ27XXX_DM_SZ);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_DATA read failed: %d\n", ret);
		goto out;
	}
	memcpy(&ManufInfoblkA, data, sizeof(ManufInfoblkA));

	val->strval = ManufInfoblkA.batt_manufacture_name;
	return 0;

out:
	dev_err(di->dev, "bus error reading chip memory: %d\n", ret);
	return ret;
}

// MSCHANGE Manufacturer Info Block B HVTCOUNT read
static int bq27xxx_read_hvtcount(struct bq27xxx_device_info *di, union power_supply_propval *val)
{
	int ret,i;
	u8 data[BQ27XXX_DM_SZ];
	uint32_t HvtCount = 0;
	uint32_t ManufacturerInfoBlockB_Bytes[BQ27742_MANUF_INFO_BLOCK_B_NUM_HVT_BYTES] = {1, 2, 7, 31};

	ret = bq27xxx_write(di, BQ27XXX_DM_BLOCK, BQ27742_MANUF_INFO_BLOCK_B, true);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_BLOCK write failed: %d\n", ret);
		goto out;
	}

	BQ27XXX_MSLEEP(1);

	ret = bq27xxx_read_block(di, BQ27XXX_DM_DATA, data, BQ27XXX_DM_SZ);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_DATA read failed: %d\n", ret);
		goto out;
	}

	for (i = 0; i < BQ27742_MANUF_INFO_BLOCK_B_NUM_HVT_BYTES; i++)
		HvtCount = HvtCount | (data[ManufacturerInfoBlockB_Bytes[i]] << (i*8));
	val->intval = HvtCount;

	return 0;

out:
	dev_err(di->dev, "bus error reading chip memory: %d\n", ret);
	return ret;
}

// MSCHANGE Manufacturer Info Block B HVTCOUNT write
static int bq27xxx_write_hvtcount(struct bq27xxx_device_info *di, const union power_supply_propval *val)
{
	int ret = 0, i;
	u8 data[BQ27XXX_DM_SZ];
	uint32_t ManufacturerInfoBlockB_Bytes[BQ27742_MANUF_INFO_BLOCK_B_NUM_HVT_BYTES] = {1, 2, 7, 31};

	uint32_t HvtCount = (uint32_t)val->intval;

	ret = bq27xxx_write(di, BQ27XXX_DM_BLOCK, BQ27742_MANUF_INFO_BLOCK_B, true);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_BLOCK write failed: %d\n", ret);
		goto out;
	}

	BQ27XXX_MSLEEP(1);

	ret = bq27xxx_read_block(di, BQ27XXX_DM_DATA, data, BQ27XXX_DM_SZ);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_DATA read failed: %d\n", ret);
		goto out;
	}

	for (i = 0; i < BQ27742_MANUF_INFO_BLOCK_B_NUM_HVT_BYTES; i++)
	{
		data[ManufacturerInfoBlockB_Bytes[i]] = (HvtCount >> (i*8)) & 0xFF;
	}

	ret = bq27xxx_write_block(di, BQ27XXX_DM_DATA, data, BQ27XXX_DM_SZ);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_DATA write failed: %d\n", ret);
		goto out;
	}

	ret = bq27xxx_write(di, BQ27XXX_DM_CKSUM,
			    bq27xxx_checksum_manufacturer_info_B(data), true);
	if (ret < 0)
	{
		dev_err(di->dev, "BQ27XXX_DM_CKSUM write failed: %d\n", ret);
		goto out;
	}

	BQ27XXX_MSLEEP(100); // flash DM updates in <100ms

	return 0;

out:
	dev_err(di->dev, "bus error writing chip memory: %d\n", ret);
	return ret;
}

static int bq27xxx_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27xxx_device_info *di = power_supply_get_drvdata(psy);

	mutex_lock(&di->lock);
	if (time_is_before_jiffies(di->last_update + 5 * HZ)) {
		cancel_delayed_work_sync(&di->work);
		bq27xxx_battery_poll(&di->work.work);
	}
	//mutex_unlock(&di->lock);  // MSCHANGE serializing requests to the FG

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
	{
		ret = -ENODEV;
		goto bq27xxx_battery_get_property_exit;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27xxx_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = bq27xxx_battery_voltage(di, val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = di->cache.flags < 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = bq27xxx_battery_current(di, val);
		if (ret == 0)
			val->intval = (~val->intval + 1);  //  MSCHANGE report -ve current charging, +ve for discharging
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq27xxx_simple_value(di->cache.capacity, val);
		if(RSOC_OVERRIDE(di->override_capacity)) // MSCHANGE adding debugfs node to fg driver
			val->intval = di->override_capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27xxx_battery_capacity_level(di, val);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq27xxx_simple_value(di->cache.temperature, val);
		if (ret == 0)
			val->intval -= 2731; /* convert decidegree k to c */
		if(TEMPERATURE_OVERRIDE(di->override_temperature))  // MSCHANGE adding debugfs node to fg driver
			val->intval = di->override_temperature;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27xxx_simple_value(di->cache.time_to_empty, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27xxx_simple_value(di->cache.time_to_empty_avg, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27xxx_simple_value(di->cache.time_to_full, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_nac(di), val);
		if(RSOC_OVERRIDE(di->override_capacity)) // MSCHANGE override REMCAM when override flag is set
			val->intval = ((di->override_capacity * di->cache.charge_full) + 50)/100;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_fcc(di), val); //  MSCHANGE query hardware instead of cached value to avoid SOC jumps
		if(RSOC_OVERRIDE(di->override_capacity)) // MSCHANGE override FCC when override flag is set
			val->intval = di->cache.charge_full;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27xxx_simple_value(di->charge_design_full, val);
		val->intval /= 1000; // MSCHANGE converting to mAh for telemetry since this will be stored in a unit16_t
		break;
	/*
	 * TODO: Implement these to make registers set from
	 * power_supply_battery_info visible in sysfs.
	 */
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
	//case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:  // MSCHANGE we are repurposing this for the FG telemetry query and it has been redefined below
	//	return -EINVAL;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = bq27xxx_simple_value(di->cache.cycle_count, val);
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27xxx_simple_value(di->cache.energy, val);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = bq27xxx_simple_value(di->cache.power_avg, val);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = bq27xxx_simple_value(di->cache.health, val);
		break;
	// MSCHANGE adding extfg fault status and safety status query
	case POWER_SUPPLY_PROP_PROTECTOR_STATUS:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_protector_status(di), val);
		break;
	case POWER_SUPPLY_PROP_PROTECTOR_STATE:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_protector_state(di), val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_safety_status(di), val);
		break;
	// MSCHANGE get HvtCount from Manufacturer Info Block B
	case POWER_SUPPLY_PROP_CHARGE_COUNTER: // Note: repurposing CHARGE_COUNTER prop for HvtCount
		ret = bq27xxx_read_hvtcount(di, val);
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		// MSCHANGE we will be getting the MANUF NAME from Manufacturer Block A
		//val->strval = BQ27XXX_MANUFACTURER;
		ret = bq27xxx_read_manufacturer_name(di, val);
		break;
	// MSCHANGE adding FG telemetry queries
	case POWER_SUPPLY_PROP_SOH:
		ret = bq27xxx_simple_value(bq27xxx_battery_read_state_of_health(di), val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_TEMP_MAX:
	case POWER_SUPPLY_PROP_TEMP_MIN:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:  // NOTE: repurposing CONSTANT_CHARGE_CURRENT for lifetime max dsg current for telemetry
	case POWER_SUPPLY_PROP_UPDATE_NOW: // NOTE: repurposing UPDATE_NOW for LTFlashCount for telemetry
	case POWER_SUPPLY_PROP_CYCLE_COUNTS: // NOTE: repurposing CYCLE_COUNTS for LFAFEStatus for telemetry
	case POWER_SUPPLY_PROP_CAPACITY_RAW:  // NOTE: repurposing CAPACITY_RAW for QmaxCell0 for telemetry
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_LIMIT: // NOTE: repurposing VOLTAGE_MAX_LIMIT for VoltageAtChargeTerm for telemetry
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_POWER_NOW:  // NOTE: repurposing POWER_NOW for AveragePLastRun for telemetry
	case POWER_SUPPLY_PROP_VOLTAGE_BOOT: // NOTE: repurposing VOLTAGE_BOOT for DeltaVoltage for telemetry
		ret = bq27xxx_battery_read_telemetry(di, psp, val);
		break;
	default:
		ret = -EINVAL;  // MSCHANGE serializing requests to the FG
		break;
	}

// MSCHANGE serializing requests to the FG
bq27xxx_battery_get_property_exit:
	mutex_unlock(&di->lock);
	return ret;
}

// MSCHANGE adding set_property for Manufacturer Info Blocks
static int bq27xxx_battery_set_property(struct power_supply *psy,
						enum power_supply_property psp,
						const union power_supply_propval *val)
{
	int ret = 0;
	struct bq27xxx_device_info *di = power_supply_get_drvdata(psy);

	mutex_lock(&di->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_COUNTER: // Note: repurposing CHARGE_COUNTER prop for HvtCount
		ret = bq27xxx_write_hvtcount(di, val);
		break;
	default:
		pr_err("set prop %d is not supported\n", psp);
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&di->lock);
	return ret;
}

static void bq27xxx_external_power_changed(struct power_supply *psy)
{
	struct bq27xxx_device_info *di = power_supply_get_drvdata(psy);

	cancel_delayed_work_sync(&di->work);
	schedule_delayed_work(&di->work, 0);
}


// MSCHANGE adding debugfs node to fg driver
#if defined(CONFIG_DEBUG_FS)

static int force_temperature_read(void *data, u64 *val)
{
	struct bq27xxx_device_info *di = data;
    *val = di->override_temperature;
    return 0;
}

static int force_temperature_write(void *data, u64 val)
{
	struct bq27xxx_device_info *di = data;
	di->override_temperature = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_temperature_ops, force_temperature_read,
			force_temperature_write, "%d\n");

static int force_rsoc_read(void *data, u64 *val)
{
	struct bq27xxx_device_info *di = data;
    *val = di->override_capacity;
    return 0;
}

static int force_rsoc_write(void *data, u64 val)
{
	struct bq27xxx_device_info *di = data;
	di->override_capacity = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_rsoc_ops, force_rsoc_read,
			force_rsoc_write, "%d\n");

static int fg_register_read_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t fg_register_read_read(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct bq27xxx_device_info *di = file->private_data;
	char buf[512];
	int flags, safety_status, unfiltered_fcc, unfiltered_rm, dod_zero, dod_at_eoc, qstart, fast_qmax, protector_status;
	int16_t passed_charge = 0;
	int reg_value = 0;
	int length = 0;

	pr_err("fg_register_read: Querying register for pack %s", di->name);
	flags = di->cache.flags;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_SAFETY_STATUS, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_SAFETY_STATUS\n");
	else
		safety_status = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_UNFILTERED_FCC, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_UNFILTERED_FCC\n");
	else
		unfiltered_fcc = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_UNFILTERED_RM, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_UNFILTERED_RM\n");
	else
		unfiltered_rm = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_PASSED_CHARGE, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_PASSED_CHARGE\n");
	else
		passed_charge = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_DOD0, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_DOD0\n");
	else
		dod_zero = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_DODatEOC, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_DODatEOC\n");
	else
		dod_at_eoc = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_QSTART, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_QSTART\n");
	else
		qstart = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_FAST_QMAX, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_FAST_QMAX\n");
	else
		fast_qmax = reg_value;

	reg_value = bq27xxx_read(di, BQ27XXX_REG_PROTECTOR_STATUS, false);
	if (reg_value < 0)
		dev_err(di->dev, "error reading BQ27XXX_REG_PROTECTOR_STATUS\n");
	else
		protector_status = reg_value;

	length = sprintf(buf, "FLAGS=%d\nSAFETY_STATUS=%d\nUNFILTERED_FCC=%d\nUNFILTERED_RM=%d\nPASSED_CHARGE=%hd\nDOD0=%d\nDOD_AT_EOC=%d\nQSTART=%d\nFAST_QMAX=%d\nPROTECTOR_STATUS=%d\n",
						flags, safety_status, unfiltered_fcc, unfiltered_rm, passed_charge,
						dod_zero, dod_at_eoc, qstart, fast_qmax, protector_status);

	return simple_read_from_buffer(userbuf, count, ppos, buf, length);
}

static const struct file_operations fg_register_read_ops = {
	.open = fg_register_read_open,
	.read = fg_register_read_read,
};

static void bq27xxx_create_debugfs(struct bq27xxx_device_info *di)
{
	struct dentry *file;

	di->dfs_root = debugfs_create_dir(di->name, NULL);
	if (IS_ERR_OR_NULL(di->dfs_root)) {
		pr_err("Couldn't create BQ27742 debugfs rc=%ld\n",
			(long)di->dfs_root);
		return;
	}

	file = debugfs_create_file("force_temperature", 0600,
			    di->dfs_root, di, &force_temperature_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create force_temperature file rc=%ld\n",
			(long)file);

	file = debugfs_create_file("force_rsoc", 0600,
			    di->dfs_root, di, &force_rsoc_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create force_rsoc file rc=%ld\n",
			(long)file);

	file = debugfs_create_file("fg_register_read", 0600,
			    di->dfs_root, di, &fg_register_read_ops);
	if (IS_ERR_OR_NULL(file))
		pr_err("Couldn't create fg_register_read file rc=%ld\n",
			(long)file);
}

#else

static void bq27xxx_create_debugfs(struct bq27xxx_device_info *di)
{}

#endif

// MSCHANGE start: Add battery discharge sensor for fg-pack thermal zones
static int bq27xxx_thermal_get_temp(void *data, int *temp)
{

	struct bq27xxx_device_info *di;
	union power_supply_propval val;
	int ret;

        if(WARN_ON(data == NULL))
                return -ENOMEM;

        di = data;
        ret = bq27xxx_battery_current(di, &val);
        if (ret == 0)
            val.intval = (~val.intval + 1);  //  report -ve current charging, +ve for discharging

	*temp = val.intval;

        return 0;
}

static const struct thermal_zone_of_device_ops bq27xxx_thermal_ops = {
        .get_temp = bq27xxx_thermal_get_temp,
};

static int bq27xxx_battery_register_thermal(struct bq27xxx_device_info *di)
{

	di->tzd = devm_thermal_zone_of_sensor_register(di->dev, 0, di,
		&bq27xxx_thermal_ops);
	if (!di->tzd)
		pr_err("Error registering thermal zone %s\n", di->name);

	return 0;
}
// MSCHANGE end: Add battery discharge sensor for fg-pack thermal zones

// MSCHANGE adding debugfs node to fg driver
#define DISABLE_TEMP_OVERRIDE_VALUE (MAX_ALLOWED_TEMPERATURE + 50)
#define DISABLE_RSOC_OVERRIDE_VALUE (MAX_ALLOWED_RSOC + 50)

int bq27xxx_battery_setup(struct bq27xxx_device_info *di)
{
	int val;       // MSCHANGE i2c probe to ensure FGs are reachable
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {
		.of_node = di->dev->of_node,
		.drv_data = di,
	};

	INIT_DELAYED_WORK(&di->work, bq27xxx_battery_poll);
	mutex_init(&di->lock);

	di->regs       = bq27xxx_chip_data[di->chip].regs;
	di->unseal_key = bq27xxx_chip_data[di->chip].unseal_key;
	di->dm_regs    = bq27xxx_chip_data[di->chip].dm_regs;
	di->opts       = bq27xxx_chip_data[di->chip].opts;

	psy_desc = devm_kzalloc(di->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_desc->name = di->name;
	psy_desc->type = POWER_SUPPLY_TYPE_BMS;   // MSCHANGE the external FGs shouldn't be reported to power_supply framework as batteries; changing this to BMS
	psy_desc->properties = bq27xxx_chip_data[di->chip].props;
	psy_desc->num_properties = bq27xxx_chip_data[di->chip].props_size;
	psy_desc->get_property = bq27xxx_battery_get_property;
	psy_desc->set_property = bq27xxx_battery_set_property;  // MSCHANGE adding set_property for Manufacturer Info Blocks
	psy_desc->external_power_changed = bq27xxx_external_power_changed;

	// MSCHANGE i2c poke in driver probe to ensure FGs are reachable
	val = bq27xxx_read(di, BQ27XXX_REG_NAC, false);
	if (val < 0)
	{
		pr_err("i2c poke failed in the EXTFG probe function, driver load is going to fail");
		return -ENODATA;
	}

	// MSCHANGE Add battery discharge sensor for fg-pack thermal zones
	bq27xxx_battery_register_thermal(di);

	di->bat = power_supply_register_no_ws(di->dev, psy_desc, &psy_cfg);
	if (IS_ERR(di->bat)) {
		if (PTR_ERR(di->bat) == -EPROBE_DEFER)
			dev_dbg(di->dev, "failed to register battery, deferring probe\n");
		else
			dev_err(di->dev, "failed to register battery\n");
		return PTR_ERR(di->bat);
	}

	bq27xxx_battery_settings(di);
	bq27xxx_battery_update(di);

	// MSCHANGE adding debugfs node to fg driver
	bq27xxx_create_debugfs(di);
	di->override_temperature = DISABLE_TEMP_OVERRIDE_VALUE;  // initially disabling user temperature override
	di->override_capacity = DISABLE_RSOC_OVERRIDE_VALUE; // initially disabling user RSOC override

	mutex_lock(&bq27xxx_list_lock);
	list_add(&di->list, &bq27xxx_battery_devices);
	mutex_unlock(&bq27xxx_list_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(bq27xxx_battery_setup);

void bq27xxx_battery_teardown(struct bq27xxx_device_info *di)
{
	/*
	 * power_supply_unregister call bq27xxx_battery_get_property which
	 * call bq27xxx_battery_poll.
	 * Make sure that bq27xxx_battery_poll will not call
	 * schedule_delayed_work again after unregister (which cause OOPS).
	 */
	poll_interval = 0;

	cancel_delayed_work_sync(&di->work);

	// MSCHANGE end: Add battery discharge sensor for fg-pack thermal zones
	if (di->tzd)
		devm_thermal_zone_of_sensor_unregister(di->dev,
			di->tzd);

	power_supply_unregister(di->bat);

	mutex_lock(&bq27xxx_list_lock);
	list_del(&di->list);
	mutex_unlock(&bq27xxx_list_lock);

	mutex_destroy(&di->lock);
}
EXPORT_SYMBOL_GPL(bq27xxx_battery_teardown);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27xxx battery monitor driver");
MODULE_LICENSE("GPL");
