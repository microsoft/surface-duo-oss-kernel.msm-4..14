#ifndef _QCOM_BAM_DMA_H_
#define _QCOM_BAM_DMA_H_
/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/dmaengine.h>

void qcom_bam_set_desc_cmd(struct dma_async_tx_descriptor *);
void qcom_bam_set_desc_eot(struct dma_async_tx_descriptor *);
void qcom_bam_set_desc_nwd(struct dma_async_tx_descriptor *);

#endif
