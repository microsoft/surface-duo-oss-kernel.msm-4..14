/*
 * Copyright 2015 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#ifndef __MACH_IOMUX_H__
#define __MACH_IOMUX_H__

/*
 *	build IOMUX_PAD structure
 *
 * This iomux scheme is based around pads, which are the physical balls
 * on the processor.
 *
 * - Each pad has a pad configuration register (SIUL2_MSCRn) which
 *   controls the settings for a PAD and is used for selecting ALT modes using MUX_MODE bits.
 * - Each pad can have but not necessarily does have an input routing register SIUL2_IMSCRn
 * - For GPIO pins, GPDO bit has to be set before MSCRn
 *
 * The naming convention for the pad modes is S32V234_PAD_<padname>__<padmode>
 * If <padname> or <padmode> refers to a GPIO, it is named
 * GPIO_<unit>_<num>
 *
 * PAD Bit field definitions:
 *
 * MUX_CTRL_OFS:			0..11  (12) - MSCR register offset
 * PAD_CTRL + MUX_MODE:		12..43 (32) - value of MUX_MODE field from MSCR
 * SEL_PAD_DATA_OUTPUT_OFS:	44..55 (12) - GPDO register offset
 * DATA_OUTPUT_CFG:			56..63 (8)  - GPDO register value
*/

typedef u64 siul2_cfg_t;

#define SIUL2_MSCRn_OFFSET					(0x240)
#define SIUL2_GPDOn_OFFSET					(0x1300)

#define MUX_CTRL_OFS_SHIFT					(0)
#define MUX_CTRL_OFS_MASK					((siul2_cfg_t)0xfff << MUX_CTRL_OFS_SHIFT)

#define MUX_PAD_CFG_SHIFT					(12)
#define MUX_PAD_CFG_MASK					((siul2_cfg_t)0xffffffff << MUX_PAD_CFG_SHIFT)

#define MUX_PAD_DATA_OUTPUT_OFS_SHIFT		(44)
#define MUX_PAD_DATA_OUTPUT_OFS_MASK		((siul2_cfg_t)0xfff << MUX_PAD_DATA_OUTPUT_OFS_SHIFT)

#define MUX_DATA_OUTPUT_VAL_SHIFT			(56)
#define MUX_DATA_OUTPUT_VAL_MASK			((siul2_cfg_t)0x1ff << MUX_DATA_OUTPUT_VAL_SHIFT)

#define IOMUX_PAD(_mux_ctrl_ofs, _mux_cfg, _pad_data_output_ofs, _pad_data_output_val)	\
		(((siul2_cfg_t)(_mux_ctrl_ofs) << MUX_CTRL_OFS_SHIFT) |	\
		((siul2_cfg_t)(_mux_cfg) << MUX_PAD_CFG_SHIFT) |	\
		((siul2_cfg_t)(_pad_data_output_ofs) << MUX_PAD_DATA_OUTPUT_OFS_SHIFT) | \
		((siul2_cfg_t)(_pad_data_output_val) << MUX_DATA_OUTPUT_VAL_SHIF))

/*
 * Use to set PAD control
 */
#define PAD_CTL_OBE			(1 << 21)
#define PAD_CTL_ODE			(1 << 20)
#define PAD_CTL_IBE			(1 << 19)
#define PAD_CTL_HYS			(1 << 18)
#define PAD_CTL_INV			(1 << 17)
#define PAD_CTL_PKE			(1 << 16)

#define PAD_CTL_SRE_OFS				(14)
#define PAD_CTL_SRE_LOW_50HZ		(0 << PAD_CTL_SRE)
#define PAD_CTL_SRE_MED_100MHZ		(1 << PAD_CTL_SRE)
/* The manual reports the same value for SRE = 01 and SRE = 10
 * #define PAD_CTL_SRE_MED_100MHZ		(2) */
#define PAD_CTL_SRE_HIGH_200MHZ		(3 << PAD_CTL_SRE)

#define PAD_CTL_PUE				(1 << 13)

#define PAD_CTL_PUS_OFS			(11)
#define PAD_CTL_PUS_100K_DOWN	(0 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_50K_UP		(1 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_100K_UP		(2 << PAD_CTL_PUS_OFS)
#define PAD_CTL_PUS_33K_UP		(3 << PAD_CTL_PUS_OFS)

#define PAD_CTL_DSE_OFS			(8)
#define PAD_CTL_DSE_OUT_DISABLE	(0 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_240			(1 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_120			(2 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_80			(3 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_60			(4 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_48			(5 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_40			(6 << PAD_CTL_DSE_OFS)
#define PAD_CTL_DSE_34			(7 << PAD_CTL_DSE_OFS)

#define PAD_CTL_CRPOINT_TRIM	(3 << 6)

#define PAD_CTL_SMC				(1 << 5)

#define PAD_CTL_MUX_MODE_ALT0	(0)
#define PAD_CTL_MUX_MODE_ALT1	(1)
#define PAD_CTL_MUX_MODE_ALT2	(2)
#define PAD_CTL_MUX_MODE_ALT3	(3)
#define PAD_CTL_MUX_MODE_ALT4	(4)
#define PAD_CTL_MUX_MODE_ALT5	(5)
#define PAD_CTL_MUX_MODE_ALT6	(6)
#define PAD_CTL_MUX_MODE_ALT7	(7)


/*
 * setups a single pad in the iomuxer
 */
int siul2_setup_pad(siul2_cfg_t pad);

/*
 * setups mutliple pads
 * convenient way to call the above function with tables
 */
int siul2_setup_multiple_pads(siul2_cfg_t *pad_list, unsigned count);

/*
 * Initialise the iomux controller
 */
void siul2_init(void __iomem *siul2_base);

#endif /* __MACH_siul2_H__*/

