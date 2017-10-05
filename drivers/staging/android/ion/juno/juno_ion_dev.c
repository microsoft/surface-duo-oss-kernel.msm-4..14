/*
 * drivers/gpu/ion/juno/juno_ion_dev.c
 *
 * Copyright (C) 2014 ARM, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include "../ion.h"

u64 juno_dmamask = DMA_BIT_MASK(64);

struct platform_device juno_device_ion = {
        .name           = "ion-juno",
        .id             = -1,
};

static struct ion_cpa_platform_data cpa_config = {
	.lowmark = 8,
	.highmark = 128,
	.fillmark = 64,
	.align_order = 0,
	.order = 9,
};

struct ion_platform_heap juno_heaps[] = {
		{
			.id	= ION_HEAP_TYPE_SYSTEM,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= "system",
		},
		{
			.id	= ION_HEAP_TYPE_SYSTEM_CONTIG,
			.type	= ION_HEAP_TYPE_SYSTEM_CONTIG,
			.name	= "system contig",
		},
		{
			.id	= ION_HEAP_TYPE_DMA,
			.type 	= ION_HEAP_TYPE_DMA,
			.name	= "ion_dma_heap-3",
			.priv	= &juno_device_ion.dev,
		},
		{
			.id = ION_HEAP_TYPE_COMPOUND_PAGE,
			.type = ION_HEAP_TYPE_COMPOUND_PAGE,
			.name = "compound_page",
			.priv = &cpa_config,
		}

};

struct ion_platform_data juno_ion_pdata = {
	.nr = ARRAY_SIZE(juno_heaps),
	.heaps = juno_heaps,
};

static int __init juno_ion_dev_init(void)
{
	int ret;

	ret = dma_declare_coherent_memory(&juno_device_ion.dev,
		0x60000000, 0x60000000, 0x08000000,
		DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
	if (ret < 0)
	{
		pr_err("%s: dma_declare_contiguous failed %d\n", __func__, ret);
		return ret;
	}

	juno_device_ion.dev.platform_data = &juno_ion_pdata;
	juno_device_ion.dev.coherent_dma_mask = juno_dmamask;
	juno_device_ion.dev.dma_mask = &juno_dmamask;

	return platform_device_register(&juno_device_ion);
}

static void __exit juno_ion_dev_exit(void)
{
	platform_device_unregister(&juno_device_ion);
}

module_init(juno_ion_dev_init);
module_exit(juno_ion_dev_exit);

