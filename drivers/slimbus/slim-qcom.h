/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
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

#ifndef _SLIM_QCOM_H
#define _SLIM_QCOM_H

#include <linux/irq.h>
#include <linux/workqueue.h>

#define QC_MFGID_LSB	0x2
#define QC_MFGID_MSB	0x17

#define SLIM_MSG_ASM_FIRST_WORD(l, mt, mc, dt, ad) \
		((l) | ((mt) << 5) | ((mc) << 8) | ((dt) << 15) | ((ad) << 16))

#define SLIM_ROOT_FREQ 24576000

/* MAX message size over control channel */
#define SLIM_MSGQ_BUF_LEN	40
#define MSM_TX_MSGS 2
#define MSM_RX_MSGS	8

#define CFG_PORT(r, v) ((v) ? CFG_PORT_V2(r) : CFG_PORT_V1(r))

/* V2 Component registers */
#define CFG_PORT_V2(r) ((r ## _V2))
#define	COMP_CFG_V2		4
#define	COMP_TRUST_CFG_V2	0x3000

/* V1 Component registers */
#define CFG_PORT_V1(r) ((r ## _V1))
#define	COMP_CFG_V1		0
#define	COMP_TRUST_CFG_V1	0x14

/* Resource group info for manager, and non-ported generic device-components */
#define EE_MGR_RSC_GRP	(1 << 10)
#define EE_NGD_2	(2 << 6)
#define EE_NGD_1	0

struct msm_slim_ctrl {
	struct slim_controller  ctrl;
	struct slim_framer	framer;
	struct device		*dev;
	void __iomem		*base;
	struct resource		*slew_mem;
	int			irq;
	struct workqueue_struct *rxwq;
	struct work_struct	wd;
	struct clk		*rclk;
	struct clk		*hclk;
	u32			ver;
};

#endif
