/*
 *    Copyright (C) 2006 Benjamin Herrenschmidt, IBM Corp.
 *			 <benh@kernel.crashing.org>
 *    and		 Arnd Bergmann, IBM Corp.
 *    Merged from powerpc/kernel/of_platform.c and
 *    sparc{,64}/kernel/of_device.c by Stephen Rothwell
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/**
 * of_find_device_by_node - Find the platform_device associated with a node
 * @np: Pointer to device tree node
 *
 * Returns platform_device pointer, or NULL if not found
 */
struct platform_device *of_find_device_by_node(struct device_node *np)
{
	struct device *dev;

	dev = bus_find_device(&platform_bus_type, NULL, np, of_dev_node_match);
	return dev ? to_platform_device(dev) : NULL;
}
EXPORT_SYMBOL(of_find_device_by_node);

static int platform_driver_probe_shim(struct platform_device *pdev)
{
	struct platform_driver *pdrv;
	struct of_platform_driver *ofpdrv;
	const struct of_device_id *match;

	pdrv = container_of(pdev->dev.driver, struct platform_driver, driver);
	ofpdrv = container_of(pdrv, struct of_platform_driver, platform_driver);

	/* There is an unlikely chance that an of_platform driver might match
	 * on a non-OF platform device.  If so, then of_match_device() will
	 * come up empty.  Return -EINVAL in this case so other drivers get
	 * the chance to bind. */
	match = of_match_device(pdev->dev.driver->of_match_table, &pdev->dev);
	return match ? ofpdrv->probe(pdev, match) : -EINVAL;
}

static void platform_driver_shutdown_shim(struct platform_device *pdev)
{
	struct platform_driver *pdrv;
	struct of_platform_driver *ofpdrv;

	pdrv = container_of(pdev->dev.driver, struct platform_driver, driver);
	ofpdrv = container_of(pdrv, struct of_platform_driver, platform_driver);
	ofpdrv->shutdown(pdev);
}

/**
 * of_register_platform_driver
 */
int of_register_platform_driver(struct of_platform_driver *drv)
{
	char *of_name;

	/* setup of_platform_driver to platform_driver adaptors */
	drv->platform_driver.driver = drv->driver;

	/* Prefix the driver name with 'of:' to avoid namespace collisions
	 * and bogus matches.  There are some drivers in the tree that
	 * register both an of_platform_driver and a platform_driver with
	 * the same name.  This is a temporary measure until they are all
	 * cleaned up --gcl July 29, 2010 */
	of_name = kmalloc(strlen(drv->driver.name) + 5, GFP_KERNEL);
	if (!of_name)
		return -ENOMEM;
	sprintf(of_name, "of:%s", drv->driver.name);
	drv->platform_driver.driver.name = of_name;

	if (drv->probe)
		drv->platform_driver.probe = platform_driver_probe_shim;
	drv->platform_driver.remove = drv->remove;
	if (drv->shutdown)
		drv->platform_driver.shutdown = platform_driver_shutdown_shim;
	drv->platform_driver.suspend = drv->suspend;
	drv->platform_driver.resume = drv->resume;

	return platform_driver_register(&drv->platform_driver);
}
EXPORT_SYMBOL(of_register_platform_driver);

void of_unregister_platform_driver(struct of_platform_driver *drv)
{
	platform_driver_unregister(&drv->platform_driver);
	kfree(drv->platform_driver.driver.name);
	drv->platform_driver.driver.name = NULL;
}
EXPORT_SYMBOL(of_unregister_platform_driver);

#if defined(CONFIG_PPC_DCR)
#include <asm/dcr.h>
#endif

extern struct device_attribute of_platform_device_attrs[];

static int of_platform_bus_match(struct device *dev, struct device_driver *drv)
{
	const struct of_device_id *matches = drv->of_match_table;

	if (!matches)
		return 0;

	return of_match_device(matches, dev) != NULL;
}

static int of_platform_device_probe(struct device *dev)
{
	int error = -ENODEV;
	struct of_platform_driver *drv;
	struct platform_device *of_dev;
	const struct of_device_id *match;

	drv = to_of_platform_driver(dev->driver);
	of_dev = to_platform_device(dev);

	if (!drv->probe)
		return error;

	of_dev_get(of_dev);

	match = of_match_device(drv->driver.of_match_table, dev);
	if (match)
		error = drv->probe(of_dev, match);
	if (error)
		of_dev_put(of_dev);

	return error;
}

static int of_platform_device_remove(struct device *dev)
{
	struct platform_device *of_dev = to_platform_device(dev);
	struct of_platform_driver *drv = to_of_platform_driver(dev->driver);

	if (dev->driver && drv->remove)
		drv->remove(of_dev);
	return 0;
}

static void of_platform_device_shutdown(struct device *dev)
{
	struct platform_device *of_dev = to_platform_device(dev);
	struct of_platform_driver *drv = to_of_platform_driver(dev->driver);

	if (dev->driver && drv->shutdown)
		drv->shutdown(of_dev);
}

#ifdef CONFIG_PM_SLEEP

static int of_platform_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct platform_device *of_dev = to_platform_device(dev);
	struct of_platform_driver *drv = to_of_platform_driver(dev->driver);
	int ret = 0;

	if (dev->driver && drv->suspend)
		ret = drv->suspend(of_dev, mesg);
	return ret;
}

static int of_platform_legacy_resume(struct device *dev)
{
	struct platform_device *of_dev = to_platform_device(dev);
	struct of_platform_driver *drv = to_of_platform_driver(dev->driver);
	int ret = 0;

	if (dev->driver && drv->resume)
		ret = drv->resume(of_dev);
	return ret;
}

static int of_platform_pm_prepare(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (drv && drv->pm && drv->pm->prepare)
		ret = drv->pm->prepare(dev);

	return ret;
}

static void of_platform_pm_complete(struct device *dev)
{
	struct device_driver *drv = dev->driver;

	if (drv && drv->pm && drv->pm->complete)
		drv->pm->complete(dev);
}

#ifdef CONFIG_SUSPEND

static int of_platform_pm_suspend(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->suspend)
			ret = drv->pm->suspend(dev);
	} else {
		ret = of_platform_legacy_suspend(dev, PMSG_SUSPEND);
	}

	return ret;
}

static int of_platform_pm_suspend_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->suspend_noirq)
			ret = drv->pm->suspend_noirq(dev);
	}

	return ret;
}

static int of_platform_pm_resume(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->resume)
			ret = drv->pm->resume(dev);
	} else {
		ret = of_platform_legacy_resume(dev);
	}

	return ret;
}

static int of_platform_pm_resume_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->resume_noirq)
			ret = drv->pm->resume_noirq(dev);
	}

	return ret;
}

#else /* !CONFIG_SUSPEND */

#define of_platform_pm_suspend		NULL
#define of_platform_pm_resume		NULL
#define of_platform_pm_suspend_noirq	NULL
#define of_platform_pm_resume_noirq	NULL

#endif /* !CONFIG_SUSPEND */

#ifdef CONFIG_HIBERNATION

static int of_platform_pm_freeze(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->freeze)
			ret = drv->pm->freeze(dev);
	} else {
		ret = of_platform_legacy_suspend(dev, PMSG_FREEZE);
	}

	return ret;
}

static int of_platform_pm_freeze_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->freeze_noirq)
			ret = drv->pm->freeze_noirq(dev);
	}

	return ret;
}

static int of_platform_pm_thaw(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->thaw)
			ret = drv->pm->thaw(dev);
	} else {
		ret = of_platform_legacy_resume(dev);
	}

	return ret;
}

static int of_platform_pm_thaw_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->thaw_noirq)
			ret = drv->pm->thaw_noirq(dev);
	}

	return ret;
}

static int of_platform_pm_poweroff(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->poweroff)
			ret = drv->pm->poweroff(dev);
	} else {
		ret = of_platform_legacy_suspend(dev, PMSG_HIBERNATE);
	}

	return ret;
}

static int of_platform_pm_poweroff_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->poweroff_noirq)
			ret = drv->pm->poweroff_noirq(dev);
	}

	return ret;
}

static int of_platform_pm_restore(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->restore)
			ret = drv->pm->restore(dev);
	} else {
		ret = of_platform_legacy_resume(dev);
	}

	return ret;
}

static int of_platform_pm_restore_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->restore_noirq)
			ret = drv->pm->restore_noirq(dev);
	}

	return ret;
}

#else /* !CONFIG_HIBERNATION */

#define of_platform_pm_freeze		NULL
#define of_platform_pm_thaw		NULL
#define of_platform_pm_poweroff		NULL
#define of_platform_pm_restore		NULL
#define of_platform_pm_freeze_noirq	NULL
#define of_platform_pm_thaw_noirq		NULL
#define of_platform_pm_poweroff_noirq	NULL
#define of_platform_pm_restore_noirq	NULL

#endif /* !CONFIG_HIBERNATION */

static struct dev_pm_ops of_platform_dev_pm_ops = {
	.prepare = of_platform_pm_prepare,
	.complete = of_platform_pm_complete,
	.suspend = of_platform_pm_suspend,
	.resume = of_platform_pm_resume,
	.freeze = of_platform_pm_freeze,
	.thaw = of_platform_pm_thaw,
	.poweroff = of_platform_pm_poweroff,
	.restore = of_platform_pm_restore,
	.suspend_noirq = of_platform_pm_suspend_noirq,
	.resume_noirq = of_platform_pm_resume_noirq,
	.freeze_noirq = of_platform_pm_freeze_noirq,
	.thaw_noirq = of_platform_pm_thaw_noirq,
	.poweroff_noirq = of_platform_pm_poweroff_noirq,
	.restore_noirq = of_platform_pm_restore_noirq,
};

#define OF_PLATFORM_PM_OPS_PTR	(&of_platform_dev_pm_ops)

#else /* !CONFIG_PM_SLEEP */

#define OF_PLATFORM_PM_OPS_PTR	NULL

#endif /* !CONFIG_PM_SLEEP */

int of_bus_type_init(struct bus_type *bus, const char *name)
{
	bus->name = name;
	bus->match = of_platform_bus_match;
	bus->probe = of_platform_device_probe;
	bus->remove = of_platform_device_remove;
	bus->shutdown = of_platform_device_shutdown;
	bus->dev_attrs = of_platform_device_attrs;
	bus->pm = OF_PLATFORM_PM_OPS_PTR;
	return bus_register(bus);
}

int of_register_driver(struct of_platform_driver *drv, struct bus_type *bus)
{
	/*
	 * Temporary: of_platform_bus used to be distinct from the platform
	 * bus.  It isn't anymore, and so drivers on the platform bus need
	 * to be registered in a special way.
	 *
	 * After all of_platform_bus_type drivers are converted to
	 * platform_drivers, this exception can be removed.
	 */
	if (bus == &platform_bus_type)
		return of_register_platform_driver(drv);

	/* register with core */
	drv->driver.bus = bus;
	return driver_register(&drv->driver);
}
EXPORT_SYMBOL(of_register_driver);

void of_unregister_driver(struct of_platform_driver *drv)
{
	if (drv->driver.bus == &platform_bus_type)
		of_unregister_platform_driver(drv);
	else
		driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(of_unregister_driver);

#if !defined(CONFIG_SPARC)
/*
 * The following routines scan a subtree and registers a device for
 * each applicable node.
 *
 * Note: sparc doesn't use these routines because it has a different
 * mechanism for creating devices from device tree nodes.
 */

/**
 * of_device_make_bus_id - Use the device node data to assign a unique name
 * @dev: pointer to device structure that is linked to a device tree node
 *
 * This routine will first try using either the dcr-reg or the reg property
 * value to derive a unique name.  As a last resort it will use the node
 * name followed by a unique number.
 */
void of_device_make_bus_id(struct device *dev)
{
	static atomic_t bus_no_reg_magic;
	struct device_node *node = dev->of_node;
	const u32 *reg;
	u64 addr;
	int magic;

#ifdef CONFIG_PPC_DCR
	/*
	 * If it's a DCR based device, use 'd' for native DCRs
	 * and 'D' for MMIO DCRs.
	 */
	reg = of_get_property(node, "dcr-reg", NULL);
	if (reg) {
#ifdef CONFIG_PPC_DCR_NATIVE
		dev_set_name(dev, "d%x.%s", *reg, node->name);
#else /* CONFIG_PPC_DCR_NATIVE */
		u64 addr = of_translate_dcr_address(node, *reg, NULL);
		if (addr != OF_BAD_ADDR) {
			dev_set_name(dev, "D%llx.%s",
				     (unsigned long long)addr, node->name);
			return;
		}
#endif /* !CONFIG_PPC_DCR_NATIVE */
	}
#endif /* CONFIG_PPC_DCR */

	/*
	 * For MMIO, get the physical address
	 */
	reg = of_get_property(node, "reg", NULL);
	if (reg) {
		addr = of_translate_address(node, reg);
		if (addr != OF_BAD_ADDR) {
			dev_set_name(dev, "%llx.%s",
				     (unsigned long long)addr, node->name);
			return;
		}
	}

	/*
	 * No BusID, use the node name and add a globally incremented
	 * counter (and pray...)
	 */
	magic = atomic_add_return(1, &bus_no_reg_magic);
	dev_set_name(dev, "%s.%d", node->name, magic - 1);
}

/**
 * of_device_alloc - Allocate and initialize an of_device
 * @np: device node to assign to device
 * @bus_id: Name to assign to the device.  May be null to use default name.
 * @parent: Parent device.
 */
struct platform_device *of_device_alloc(struct device_node *np,
				  const char *bus_id,
				  struct device *parent)
{
	struct platform_device *dev;
	int rc, i, num_reg = 0, num_irq;
	struct resource *res, temp_res;

	dev = platform_device_alloc("", -1);
	if (!dev)
		return NULL;

	/* count the io and irq resources */
	while (of_address_to_resource(np, num_reg, &temp_res) == 0)
		num_reg++;
	num_irq = of_irq_count(np);

	/* Populate the resource table */
	if (num_irq || num_reg) {
		res = kzalloc(sizeof(*res) * (num_irq + num_reg), GFP_KERNEL);
		if (!res) {
			platform_device_put(dev);
			return NULL;
		}

		dev->num_resources = num_reg + num_irq;
		dev->resource = res;
		for (i = 0; i < num_reg; i++, res++) {
			rc = of_address_to_resource(np, i, res);
			WARN_ON(rc);
		}
		WARN_ON(of_irq_to_resource_table(np, res, num_irq) != num_irq);
	}

	dev->dev.of_node = of_node_get(np);
#if defined(CONFIG_PPC) || defined(CONFIG_MICROBLAZE)
	dev->dev.dma_mask = &dev->archdata.dma_mask;
#endif
	dev->dev.parent = parent;

	if (bus_id)
		dev_set_name(&dev->dev, "%s", bus_id);
	else
		of_device_make_bus_id(&dev->dev);

	return dev;
}
EXPORT_SYMBOL(of_device_alloc);

/**
 * of_platform_device_create - Alloc, initialize and register an of_device
 * @np: pointer to node to create device for
 * @bus_id: name to assign device
 * @parent: Linux device model parent device.
 *
 * Returns pointer to created platform device, or NULL if a device was not
 * registered.  Unavailable devices will not get registered.
 */
struct platform_device *of_platform_device_create(struct device_node *np,
					    const char *bus_id,
					    struct device *parent)
{
	struct platform_device *dev;

	if (!of_device_is_available(np))
		return NULL;

	dev = of_device_alloc(np, bus_id, parent);
	if (!dev)
		return NULL;

#if defined(CONFIG_PPC) || defined(CONFIG_MICROBLAZE)
	dev->archdata.dma_mask = 0xffffffffUL;
#endif
	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	dev->dev.bus = &platform_bus_type;

	/* We do not fill the DMA ops for platform devices by default.
	 * This is currently the responsibility of the platform code
	 * to do such, possibly using a device notifier
	 */

	if (of_device_add(dev) != 0) {
		platform_device_put(dev);
		return NULL;
	}

	return dev;
}
EXPORT_SYMBOL(of_platform_device_create);

struct of_platform_prepare_data {
	struct list_head list;
	struct device_node *node;
	struct device *dev;		/* assigned device */

	int num_resources;
	struct resource resource[0];
};

static LIST_HEAD(of_platform_prepare_list);
static struct notifier_block of_platform_nb;

static struct of_platform_prepare_data *of_platform_find_prepare_data(
						struct device_node *node)
{
	struct of_platform_prepare_data *prep;
	list_for_each_entry(prep, &of_platform_prepare_list, list)
		if (prep->node == node)
			return prep;
	return NULL;
}

static bool of_pdev_match_resources(struct platform_device *pdev,
			struct of_platform_prepare_data *prep)
{
	struct resource *node_res = prep->resource;
	struct resource *pdev_res;
	int i, j;

	if (prep->num_resources == 0 || pdev->num_resources == 0)
		return false;

	dev_dbg(&pdev->dev, "compare dt node %s\n", prep->node->full_name);

	/* Compare both resource tables and make sure every node resource
	 * is represented by the platform device.  Here we check that each
	 * resource has corresponding entry with the same type and start
	 * values, and the end value falls inside the range specified
	 * in the device tree node. */
	for (i = 0; i < prep->num_resources; i++, node_res++) {
		pr_debug("        node res %2i:%.8x..%.8x[%lx]...\n", i,
			node_res->start, node_res->end, node_res->flags);
		pdev_res = pdev->resource;
		for (j = 0; j < pdev->num_resources; j++, pdev_res++) {
			pr_debug("        pdev res %2i:%.8x..%.8x[%lx]\n", j,
				pdev_res->start, pdev_res->end, pdev_res->flags);
			if ((pdev_res->start == node_res->start) &&
			    (pdev_res->end >= node_res->start) &&
			    (pdev_res->end <= node_res->end) &&
			    (pdev_res->flags == node_res->flags)) {
				pr_debug("    ...MATCH!  :-)\n");
				break;
			}
		}
		if (j >= pdev->num_resources)
			return false;
	}
	return true;
}

static int of_platform_device_notifier_call(struct notifier_block *nb,
					unsigned long event, void *_dev)
{
	struct platform_device *pdev = to_platform_device(_dev);
	struct of_platform_prepare_data *prep;

	switch (event) {
	case BUS_NOTIFY_ADD_DEVICE:
		if (pdev->dev.of_node)
			return NOTIFY_DONE;

		list_for_each_entry(prep, &of_platform_prepare_list, list) {
			if (prep->dev)
				continue;

			if (!of_pdev_match_resources(pdev, prep))
				continue;

			/* If disabled, don't let the device bind */
			if (!of_device_is_available(prep->node)) {
				char buf[strlen(pdev->name) + 12];
				dev_info(&pdev->dev, "disabled by dt node %s\n",
					prep->node->full_name);
				sprintf(buf, "%s-disabled", pdev->name);
				pdev->name = kstrdup(buf, GFP_KERNEL);
				continue;
			}

			dev_info(&pdev->dev, "attaching dt node %s\n",
				prep->node->full_name);
			prep->dev = get_device(&pdev->dev);
			pdev->dev.of_node = of_node_get(prep->node);
			return NOTIFY_OK;
		}
		break;

	case BUS_NOTIFY_DEL_DEVICE:
		list_for_each_entry(prep, &of_platform_prepare_list, list) {
			if (prep->dev == &pdev->dev) {
				dev_info(&pdev->dev, "detaching dt node %s\n",
					 prep->node->full_name);
				of_node_put(pdev->dev.of_node);
				put_device(prep->dev);
				pdev->dev.of_node = NULL;
				prep->dev = NULL;
				return NOTIFY_OK;
			}
		}
		break;
	}

	return NOTIFY_DONE;
}

/**
 * of_platform_prepare - Flag nodes to be used for creating devices
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @bus_match: match table for child bus nodes, or NULL
 *
 * This function sets up 'snooping' of device tree registrations and
 * when a device registration is found that matches a node in the
 * device tree, it populates the platform_device with a pointer to the
 * matching node.
 *
 * A bus notifier is used to implement this behaviour.  When this
 * function is called, it will parse all the child nodes of @root and
 * create a lookup table of eligible device nodes.  A device node is
 * considered eligible if it:
 *    a) has a compatible property,
 *    b) has memory mapped registers, and
 *    c) has a mappable interrupt.
 *
 * It will also recursively parse child buses providing
 *    a) the child bus node has a ranges property (children have
 *       memory-mapped registers), and
 *    b) it is compatible with the @matches list.
 *
 * The lookup table will be used as data for a platform bus notifier
 * that will compare each new device registration with the table
 * before a device driver is bound to it.  If there is a match, then
 * the of_node pointer will be added to the device.  Therefore it is
 * important to call this function *before* any platform devices get
 * registered.
 */
void of_platform_prepare(struct device_node *root,
			 const struct of_device_id *matches)
{
	struct device_node *child;
	struct of_platform_prepare_data *prep;

	/* register the notifier if it isn't already */
	if (!of_platform_nb.notifier_call) {
		of_platform_nb.notifier_call = of_platform_device_notifier_call;
		bus_register_notifier(&platform_bus_type, &of_platform_nb);
	}

	/* If root is null, then start at the root of the tree */
	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return;

	pr_debug("of_platform_prepare()\n");
	pr_debug(" starting at: %s\n", root->full_name);

	/* Loop over children and record the details */
	for_each_child_of_node(root, child) {
		struct resource *res;
		int num_irq, num_reg, i;

		/* If this is a bus node, recursively inspect the children,
		 * but *don't* prepare it.  Prepare only concerns
		 * itself with leaf-nodes.  */
		if (of_match_node(matches, child)) {
			of_platform_prepare(child, matches);
			continue;
		}

		/* Is it already in the list? */
		if (of_platform_find_prepare_data(child))
			continue;

		/* Make sure it has a compatible property */
		if (!of_get_property(child, "compatible", NULL))
			continue;

		/*
		 * Count the resources.  If the device doesn't have any
		 * register ranges, then it gets skipped because there is no
		 * way to match such a device against static registration
		 */
		num_irq = of_irq_count(child);
		num_reg = of_address_count(child);
		if (!num_reg)
			continue;

		/* Device node looks valid; record the details */
		prep = kzalloc(sizeof(*prep) +
			(sizeof(prep->resource[0]) * (num_irq + num_reg)),
			GFP_KERNEL);
		if (!prep)
			return; /* We're screwed if malloc doesn't work. */

		INIT_LIST_HEAD(&prep->list);

		res = &prep->resource[0];
		for (i = 0; i < num_reg; i++, res++)
			WARN_ON(of_address_to_resource(child, i, res));
		WARN_ON(of_irq_to_resource_table(child, res, num_irq) != num_irq);
		prep->num_resources = num_reg + num_irq;
		prep->node = of_node_get(child);

		list_add_tail(&prep->list, &of_platform_prepare_list);

		pr_debug("%s() - %s prepared (%i regs, %i irqs)\n",
			__func__, prep->node->full_name, num_reg, num_irq);
	}

}

/**
 * of_platform_bus_create() - Create a device for a node and its children.
 * @bus: device node of the bus to instantiate
 * @matches: match table for bus nodes
 * disallow recursive creation of child buses
 * @parent: parent for new device, or NULL for top level.
 *
 * Creates a platform_device for the provided device_node, and optionally
 * recursively create devices for all the child nodes.
 */
static int of_platform_bus_create(struct device_node *bus,
				  const struct of_device_id *matches,
				  struct device *parent, bool strict)
{
	struct of_platform_prepare_data *prep;
	struct device_node *child;
	struct platform_device *dev;
	int rc = 0;

	/* Make sure it has a compatible property */
	if (strict && (!of_get_property(bus, "compatible", NULL))) {
		pr_debug("%s() - skipping %s, no compatible prop\n",
			 __func__, bus->full_name);
		return 0;
	}

	/* Has the device already been registered manually? */
	prep = of_platform_find_prepare_data(bus);
	if (prep && prep->dev) {
		pr_debug("%s() - skipping %s, already registered\n",
			 __func__, bus->full_name);
		return 0;
	}

	dev = of_platform_device_create(bus, NULL, parent);
	if (!dev || !of_match_node(matches, bus))
		return 0;

	for_each_child_of_node(bus, child) {
		pr_debug("   create child: %s\n", child->full_name);
		rc = of_platform_bus_create(child, matches, &dev->dev, strict);
		if (rc) {
			of_node_put(child);
			break;
		}
	}
	return rc;
}

/**
 * of_platform_bus_probe() - Probe the device-tree for platform buses
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @matches: match table for bus nodes
 * @parent: parent to hook devices from, NULL for toplevel
 *
 * Note that children of the provided root are not instantiated as devices
 * unless the specified root itself matches the bus list and is not NULL.
 */
int of_platform_bus_probe(struct device_node *root,
			  const struct of_device_id *matches,
			  struct device *parent)
{
	struct device_node *child;
	int rc = 0;

	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	pr_debug("of_platform_bus_probe()\n");
	pr_debug(" starting at: %s\n", root->full_name);

	/* Do a self check of bus type, if there's a match, create children */
	if (of_match_node(matches, root)) {
		rc = of_platform_bus_create(root, matches, parent, false);
	} else for_each_child_of_node(root, child) {
		if (!of_match_node(matches, child))
			continue;
		rc = of_platform_bus_create(child, matches, parent, false);
		if (rc)
			break;
	}

	of_node_put(root);
	return rc;
}
EXPORT_SYMBOL(of_platform_bus_probe);

/**
 * of_platform_populate() - Populate platform_devices from device tree data
 * @root: parent of the first level to probe or NULL for the root of the tree
 * @matches: match table, NULL to use the default
 * @parent: parent to hook devices from, NULL for toplevel
 *
 * Similar to of_platform_bus_probe(), this function walks the device tree
 * and creates devices from nodes.  It differs in that it follows the modern
 * convention of requiring all device nodes to have a 'compatible' property,
 * and it is suitable for creating devices which are children of the root
 * node (of_platform_bus_probe will only create children of the root which
 * are selected by the @matches argument).
 *
 * New board support should be using this function instead of
 * of_platform_bus_probe().
 *
 * Returns 0 on success, < 0 on failure.
 */
int of_platform_populate(struct device_node *root,
			const struct of_device_id *matches,
			struct device *parent)
{
	struct device_node *child;
	int rc = 0;

	root = root ? of_node_get(root) : of_find_node_by_path("/");
	if (!root)
		return -EINVAL;

	for_each_child_of_node(root, child) {
		rc = of_platform_bus_create(child, matches, parent, true);
		if (rc)
			break;
	}

	of_node_put(root);
	return rc;
}
#endif /* !CONFIG_SPARC */
