/*
 *  linux/drivers/mmc/host/mmci.h - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define MMCIPOWER		0x000
#define MCI_PWR_OFF		0x00
#define MCI_PWR_UP		0x02
#define MCI_PWR_ON		0x03
#define MCI_OD			BIT(6)
#define MCI_ROD			BIT(7)
/*
 * The ST Micro version does not have ROD and reuse the voltage registers for
 * direction settings.
 */
#define MCI_ST_DATA2DIREN	BIT(2)
#define MCI_ST_CMDDIREN		BIT(3)
#define MCI_ST_DATA0DIREN	BIT(4)
#define MCI_ST_DATA31DIREN	BIT(5)
#define MCI_ST_FBCLKEN		BIT(7)
#define MCI_ST_DATA74DIREN	BIT(8)

#define MMCICLOCK		0x004
#define MCI_CLK_ENABLE		BIT(8)
#define MCI_CLK_PWRSAVE		BIT(9)
#define MCI_CLK_BYPASS		BIT(10)
#define MCI_4BIT_BUS		BIT(11)
/*
 * 8bit wide buses, hardware flow contronl, negative edges and clock inversion
 * supported in ST Micro U300 and Ux500 versions
 */
#define MCI_ST_8BIT_BUS		BIT(12)
#define MCI_ST_U300_HWFCEN	BIT(13)
#define MCI_ST_UX500_NEG_EDGE	BIT(13)
#define MCI_ST_UX500_HWFCEN	BIT(14)
#define MCI_ST_UX500_CLK_INV	BIT(15)
/* Modified PL180 on Versatile Express platform */
#define MCI_ARM_HWFCEN		BIT(12)

#define MMCIARGUMENT		0x008
#define MMCICOMMAND		0x00c
#define MCI_CPSM_RESPONSE	BIT(6)
#define MCI_CPSM_LONGRSP	BIT(7)
#define MCI_CPSM_INTERRUPT	BIT(8)
#define MCI_CPSM_PENDING	BIT(9)
#define MCI_CPSM_ENABLE		BIT(10)
/* Argument flag extenstions in the ST Micro versions */
#define MCI_ST_SDIO_SUSP	BIT(11)
#define MCI_ST_ENCMD_COMPL	BIT(12)
#define MCI_ST_NIEN		BIT(13)
#define MCI_ST_CE_ATACMD	BIT(14)

#define MMCIRESPCMD		0x010
#define MMCIRESPONSE0		0x014
#define MMCIRESPONSE1		0x018
#define MMCIRESPONSE2		0x01c
#define MMCIRESPONSE3		0x020
#define MMCIDATATIMER		0x024
#define MMCIDATALENGTH		0x028
#define MMCIDATACTRL		0x02c
#define MCI_DPSM_ENABLE		BIT(0)
#define MCI_DPSM_DIRECTION	BIT(1)
#define MCI_DPSM_MODE		BIT(2)
#define MCI_DPSM_DMAENABLE	BIT(3)
#define MCI_DPSM_BLOCKSIZE	BIT(4)
/* Control register extensions in the ST Micro U300 and Ux500 versions */
#define MCI_ST_DPSM_RWSTART	BIT(8)
#define MCI_ST_DPSM_RWSTOP	BIT(9)
#define MCI_ST_DPSM_RWMOD	BIT(10)
#define MCI_ST_DPSM_SDIOEN	BIT(11)
/* Control register extensions in the ST Micro Ux500 versions */
#define MCI_ST_DPSM_DMAREQCTL	BIT(12)
#define MCI_ST_DPSM_DBOOTMODEEN	BIT(13)
#define MCI_ST_DPSM_BUSYMODE	BIT(14)
#define MCI_ST_DPSM_DDRMODE	BIT(15)

#define MMCIDATACNT		0x030
#define MMCISTATUS		0x034
#define MCI_CMDCRCFAIL		BIT(0)
#define MCI_DATACRCFAIL		BIT(1)
#define MCI_CMDTIMEOUT		BIT(2)
#define MCI_DATATIMEOUT		BIT(3)
#define MCI_TXUNDERRUN		BIT(4)
#define MCI_RXOVERRUN		BIT(5)
#define MCI_CMDRESPEND		BIT(6)
#define MCI_CMDSENT		BIT(7)
#define MCI_DATAEND		BIT(8)
#define MCI_STARTBITERR		BIT(9)
#define MCI_DATABLOCKEND	BIT(10)
#define MCI_CMDACTIVE		BIT(11)
#define MCI_TXACTIVE		BIT(12)
#define MCI_RXACTIVE		BIT(13)
#define MCI_TXFIFOHALFEMPTY	BIT(14)
#define MCI_RXFIFOHALFFULL	BIT(15)
#define MCI_TXFIFOFULL		BIT(16)
#define MCI_RXFIFOFULL		BIT(17)
#define MCI_TXFIFOEMPTY		BIT(18)
#define MCI_RXFIFOEMPTY		BIT(19)
#define MCI_TXDATAAVLBL		BIT(20)
#define MCI_RXDATAAVLBL		BIT(21)
/* Extended status bits for the ST Micro variants */
#define MCI_ST_SDIOIT		BIT(22)
#define MCI_ST_CEATAEND		BIT(23)
#define MCI_ST_CARDBUSY		BIT(24)

#define MMCICLEAR		0x038
#define MCI_CMDCRCFAILCLR	BIT(0)
#define MCI_DATACRCFAILCLR	BIT(1)
#define MCI_CMDTIMEOUTCLR	BIT(2)
#define MCI_DATATIMEOUTCLR	BIT(3)
#define MCI_TXUNDERRUNCLR	BIT(4)
#define MCI_RXOVERRUNCLR	BIT(5)
#define MCI_CMDRESPENDCLR	BIT(6)
#define MCI_CMDSENTCLR		BIT(7)
#define MCI_DATAENDCLR		BIT(8)
#define MCI_STARTBITERRCLR	BIT(9)
#define MCI_DATABLOCKENDCLR	BIT(10)
/* Extended status bits for the ST Micro variants */
#define MCI_ST_SDIOITC		BIT(22)
#define MCI_ST_CEATAENDC	BIT(23)
#define MCI_ST_BUSYENDC		BIT(24)

#define MMCIMASK0		0x03c
#define MCI_CMDCRCFAILMASK	BIT(0)
#define MCI_DATACRCFAILMASK	BIT(1)
#define MCI_CMDTIMEOUTMASK	BIT(2)
#define MCI_DATATIMEOUTMASK	BIT(3)
#define MCI_TXUNDERRUNMASK	BIT(4)
#define MCI_RXOVERRUNMASK	BIT(5)
#define MCI_CMDRESPENDMASK	BIT(6)
#define MCI_CMDSENTMASK		BIT(7)
#define MCI_DATAENDMASK		BIT(8)
#define MCI_STARTBITERRMASK	BIT(9)
#define MCI_DATABLOCKENDMASK	BIT(10)
#define MCI_CMDACTIVEMASK	BIT(11)
#define MCI_TXACTIVEMASK	BIT(12)
#define MCI_RXACTIVEMASK	BIT(13)
#define MCI_TXFIFOHALFEMPTYMASK	BIT(14)
#define MCI_RXFIFOHALFFULLMASK	BIT(15)
#define MCI_TXFIFOFULLMASK	BIT(16)
#define MCI_RXFIFOFULLMASK	BIT(17)
#define MCI_TXFIFOEMPTYMASK	BIT(18)
#define MCI_RXFIFOEMPTYMASK	BIT(19)
#define MCI_TXDATAAVLBLMASK	BIT(20)
#define MCI_RXDATAAVLBLMASK	BIT(21)
/* Extended status bits for the ST Micro variants */
#define MCI_ST_SDIOITMASK	BIT(22)
#define MCI_ST_CEATAENDMASK	BIT(23)
#define MCI_ST_BUSYEND		BIT(24)

#define MMCIMASK1		0x040
#define MMCIFIFOCNT		0x048
#define MMCIFIFO		0x080 /* to 0x0bc */

#define MCI_IRQENABLE	\
	(MCI_CMDCRCFAILMASK|MCI_DATACRCFAILMASK|MCI_CMDTIMEOUTMASK|	\
	MCI_DATATIMEOUTMASK|MCI_TXUNDERRUNMASK|MCI_RXOVERRUNMASK|	\
	MCI_CMDRESPENDMASK|MCI_CMDSENTMASK|MCI_STARTBITERRMASK)

/* These interrupts are directed to IRQ1 when two IRQ lines are available */
#define MCI_IRQ1MASK \
	(MCI_RXFIFOHALFFULLMASK | MCI_RXDATAAVLBLMASK | \
	 MCI_TXFIFOHALFEMPTYMASK)

#define NR_SG		128

struct clk;
struct variant_data;
struct dma_chan;

struct mmci_host_next {
	struct dma_async_tx_descriptor	*dma_desc;
	struct dma_chan			*dma_chan;
	s32				cookie;
};

struct mmci_host {
	phys_addr_t		phybase;
	void __iomem		*base;
	struct mmc_request	*mrq;
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_host		*mmc;
	struct clk		*clk;
	bool			singleirq;

	spinlock_t		lock;

	unsigned int		mclk;
	unsigned int		cclk;
	u32			pwr_reg;
	u32			pwr_reg_add;
	u32			clk_reg;
	u32			datactrl_reg;
	u32			busy_status;
	bool			vqmmc_enabled;
	struct mmci_platform_data *plat;
	struct variant_data	*variant;

	u8			hw_designer;
	u8			hw_revision:4;

	struct timer_list	timer;
	unsigned int		oldstat;

	/* pio stuff */
	struct sg_mapping_iter	sg_miter;
	unsigned int		size;

#ifdef CONFIG_DMA_ENGINE
	/* DMA stuff */
	struct dma_chan		*dma_current;
	struct dma_chan		*dma_rx_channel;
	struct dma_chan		*dma_tx_channel;
	struct dma_async_tx_descriptor	*dma_desc_current;
	struct mmci_host_next	next_data;

#define dma_inprogress(host)	((host)->dma_current)
#else
#define dma_inprogress(host)	(0)
#endif
};

