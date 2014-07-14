#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include "mmci.h"

/* DML config register defination */
#define DML_CONFIG			0x00
#define PRODUCER_CRCI_DIS		0x00
#define PRODUCER_CRCI_X_SEL		0x01
#define PRODUCER_CRCI_Y_SEL		0x02
#define PRODUCER_CRCI_MSK		0x3
#define CONSUMER_CRCI_DIS		(0x00 << 2)
#define CONSUMER_CRCI_X_SEL		(0x01 << 2)
#define CONSUMER_CRCI_Y_SEL		(0x02 << 2)
#define CONSUMER_CRCI_MSK		(0x3 << 2)
#define PRODUCER_TRANS_END_EN		(1 << 4)
#define BYPASS				(1 << 16)
#define DIRECT_MODE			(1 << 17)
#define INFINITE_CONS_TRANS		(1 << 18)

#define DML_SW_RESET			0x08
#define DML_PRODUCER_START		0x0C
#define DML_CONSUMER_START		0x10
#define DML_PRODUCER_PIPE_LOGICAL_SIZE	0x14
#define DML_CONSUMER_PIPE_LOGICAL_SIZE	0x18
#define DML_PIPE_ID			0x1C
#define DML_PRODUCER_BAM_BLOCK_SIZE	0x24
#define DML_PRODUCER_BAM_TRANS_SIZE	0x28
#define PRODUCER_PIPE_ID_SHFT		0
#define PRODUCER_PIPE_ID_MSK		0x1f
#define CONSUMER_PIPE_ID_SHFT		16
#define CONSUMER_PIPE_ID_MSK		(0x1f << 16)
/* other definations */
#define PRODUCER_PIPE_LOGICAL_SIZE	4096
#define CONSUMER_PIPE_LOGICAL_SIZE	4096

#define DML_OFFSET			0x800

void dml_start_xfer(struct mmci_host *host, struct mmc_data *data)
{
	u32 config;
	void __iomem *dml_base;
	dml_base = host->base + DML_OFFSET;

	if (data->flags & MMC_DATA_READ) {
		/* Read operation: configure DML for producer operation */
		/* Set producer CRCI-x and disable consumer CRCI */
		config = readl(dml_base + DML_CONFIG);
		config = (config & ~PRODUCER_CRCI_MSK) | PRODUCER_CRCI_X_SEL;
		config = (config & ~CONSUMER_CRCI_MSK) | CONSUMER_CRCI_DIS;
		writel(config, (dml_base + DML_CONFIG));

		/* Set the Producer BAM block size */
		writel(data->blksz, (dml_base +
					DML_PRODUCER_BAM_BLOCK_SIZE));

		/* Set Producer BAM Transaction size */
		writel(data->blocks * data->blksz,
			(dml_base + DML_PRODUCER_BAM_TRANS_SIZE));
		/* Set Producer Transaction End bit */
		writel((readl_relaxed(dml_base + DML_CONFIG)
			| PRODUCER_TRANS_END_EN),
			(dml_base + DML_CONFIG));
		/* Trigger producer */
		writel(1, (dml_base + DML_PRODUCER_START));
	} else {
		/* Write operation: configure DML for consumer operation */
		/* Set consumer CRCI-x and disable producer CRCI*/
		config = readl(dml_base + DML_CONFIG);
		config = (config & ~CONSUMER_CRCI_MSK) | CONSUMER_CRCI_X_SEL;
		config = (config & ~PRODUCER_CRCI_MSK) | PRODUCER_CRCI_DIS;

		config = 0x4;
		writel(config, (dml_base + DML_CONFIG));
		/* Clear Producer Transaction End bit */
		writel((readl_relaxed(dml_base + DML_CONFIG)
			& ~PRODUCER_TRANS_END_EN),
			(dml_base + DML_CONFIG));
		/* Trigger consumer */
		writel(1, (dml_base + DML_CONSUMER_START));
	}
}

static int of_get_dml_pipe_index(struct device_node *np, const char *name)
{
	int count, i;
	const char *s;
	struct of_phandle_args	dma_spec;

	if (!np || !name)
		return -ENODEV;

	count = of_property_count_strings(np, "dma-names");
	if (count < 0)
		return -ENODEV;

	for (i = 0; i < count; i++) {

		if (of_property_read_string_index(np, "dma-names", i, &s))
			continue;

		if (strcmp(name, s))
			continue;

		if (of_parse_phandle_with_args(np, "dmas", "#dma-cells", i,
				       &dma_spec))
			continue;

		if (dma_spec.args_count)
			return dma_spec.args[0];
	}

	return -ENODEV;
}

/* Initialize the dml hardware connectd to SD Card controller */
int dml_hw_init(struct mmci_host *host, struct device_node *np)
{
	u32 config = 0;
	void __iomem *dml_base;
	u32 consumer_id = 0, producer_id = 0;

	consumer_id = of_get_dml_pipe_index(np, "tx");
	producer_id = of_get_dml_pipe_index(np, "rx");

	if (IS_ERR_VALUE(producer_id) || IS_ERR_VALUE(consumer_id))
		return -ENODEV;

	dml_base = host->base + DML_OFFSET;

	/* Reset the DML block */
	writel(1, (dml_base + DML_SW_RESET));

	/* Disable the producer and consumer CRCI */
	config = (PRODUCER_CRCI_DIS | CONSUMER_CRCI_DIS);
	/*
	 * Disable the bypass mode. Bypass mode will only be used
	 * if data transfer is to happen in PIO mode and don't
	 * want the BAM interface to connect with SDCC-DML.
	 */
	config &= ~BYPASS;
	/*
	 * Disable direct mode as we don't DML to MASTER the AHB bus.
	 * BAM connected with DML should MASTER the AHB bus.
	 */
	config &= ~DIRECT_MODE;
	/*
	 * Disable infinite mode transfer as we won't be doing any
	 * infinite size data transfers. All data transfer will be
	 * of finite data size.
	 */
	config &= ~INFINITE_CONS_TRANS;
	writel(config, (dml_base + DML_CONFIG));

	/*
	 * Initialize the logical BAM pipe size for producer
	 * and consumer.
	 */
	writel(PRODUCER_PIPE_LOGICAL_SIZE,
		(dml_base + DML_PRODUCER_PIPE_LOGICAL_SIZE));
	writel(CONSUMER_PIPE_LOGICAL_SIZE,
		(dml_base + DML_CONSUMER_PIPE_LOGICAL_SIZE));

	/* Initialize Producer/consumer pipe id */
	writel(producer_id | (consumer_id << CONSUMER_PIPE_ID_SHFT),
		(dml_base + DML_PIPE_ID));

	return 0;
}
