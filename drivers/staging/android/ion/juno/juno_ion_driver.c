/*
 * drivers/gpu/ion/juno/juno_ion_driver.c
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
 * Some parts based on drivers/staging/android/ion/tegra/tegra_ion.c
 * which is:
 * Copyright (C) 2011 Google, Inc.
 */

#include <linux/err.h>
#include "../ion.h"
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../ion_priv.h"

struct ion_device *idev;
int num_heaps;
struct ion_heap **heaps;

int juno_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	int err;
	int i;

	num_heaps = pdata->nr;

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	idev = ion_device_create(NULL);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

int juno_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}

static struct platform_driver juno_ion_driver = {
	.probe = juno_ion_probe,
	.remove = juno_ion_remove,
	.driver = { .name = "ion-juno" }
};

static int __init juno_ion_init(void)
{
	return platform_driver_register(&juno_ion_driver);
}

static void __exit juno_ion_exit(void)
{
	platform_driver_unregister(&juno_ion_driver);
}

module_init(juno_ion_init);
module_exit(juno_ion_exit);

