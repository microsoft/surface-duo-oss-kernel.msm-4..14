/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/slimbus.h>
#include <linux/of.h>

static DEFINE_MUTEX(slim_lock);
static DEFINE_IDR(ctrl_idr);

static bool slim_eaddr_equal(struct slim_eaddr *a, struct slim_eaddr *b)
{
	pr_err("MATCH A: %d: %d: %d: \n", a->manf_id, a->prod_code, a->dev_index)
	pr_err("MATCH B:: %d: %d: %d: \n", b->manf_id, b->prod_code, b->dev_index)
	return (a->manf_id == b->manf_id &&
		a->prod_code == b->prod_code &&
		a->dev_index == b->dev_index &&
		a->instance == b->instance);
}

static const struct slim_device_id *
slim_match(const struct slim_device_id *id, const struct slim_device *slim_dev)
{
	while (id->manf_id != 0 || id->prod_code != 0) {

		pr_err("MATCH:: %d: %d: %d: \n", id->manf_id, id->prod_code, id->dev_index)
		if (id->manf_id == slim_dev->e_addr.manf_id &&
		    id->prod_code == slim_dev->e_addr.prod_code &&
		    id->dev_index == slim_dev->e_addr.dev_index)
			return id;
		id++;
	}
	return NULL;
}

static int slim_device_match(struct device *dev, struct device_driver *driver)
{
	struct slim_device *slim_dev;
	struct slim_driver *drv = to_slim_driver(driver);

	slim_dev = to_slim_device(dev);
	if (drv->id_table)
		return slim_match(drv->id_table, slim_dev) != NULL;
	return 0;
}

struct sb_report_wd {
	struct work_struct wd;
	struct slim_device *sb;
	bool report;
};

static void slim_report(struct work_struct *work)
{
	struct slim_driver *sbdrv;
	struct sb_report_wd *sbw = container_of(work, struct sb_report_wd, wd);
	struct slim_device *sbdev = sbw->sb;

	mutex_lock(&sbdev->report_lock);
	if (!sbdev->dev.driver)
		goto report_exit;

	/* check if device-up or down needs to be called */
	if ((!sbdev->reported && !sbdev->notified) ||
	    (sbdev->reported && sbdev->notified))
		goto report_exit;

	sbdrv = to_slim_driver(sbdev->dev.driver);

	/**
	 * address no longer valid, means device reported absent, whereas
	 * address valid, means device reported present
	 */
	if (sbdev->notified && !sbdev->reported) {
		sbdev->notified = false;
		if (sbdrv->device_down)
			sbdrv->device_down(sbdev);
	} else if (!sbdev->notified && sbdev->reported) {
		sbdev->notified = true;
		if (sbdrv->device_up)
			sbdrv->device_up(sbdev);
	}
report_exit:
	mutex_unlock(&sbdev->report_lock);
	kfree(sbw);
}

/**
 * Report callbacks(device_up, device_down) are implemented by slimbus-devices.
 * The calls are scheduled into a workqueue to avoid holding up controller
 * intialization/tear-down.
 */
static void schedule_slim_report(struct slim_controller *ctrl,
				 struct slim_device *sb, bool report)
{
	struct sb_report_wd *sbw;

	dev_dbg(&ctrl->dev, "report:%d for slave:%s\n", report, sb->name);

	sbw = kmalloc(sizeof(*sbw), GFP_KERNEL);
	if (!sbw)
		return;

	INIT_WORK(&sbw->wd, slim_report);
	sbw->sb = sb;
	sbw->report = report;
	if (!queue_work(ctrl->wq, &sbw->wd)) {
		dev_err(&ctrl->dev, "failed to queue report:%d slave:%s\n",
				    report, sb->name);
		kfree(sbw);
	}
}

static int slim_device_probe(struct device *dev)
{
	struct slim_device	*slim_dev;
	struct slim_driver	*driver;
	struct slim_controller	*ctrl;
	int status = 0;

	slim_dev = to_slim_device(dev);
	driver = to_slim_driver(dev->driver);
	if (!driver->id_table)
		return -ENODEV;

	slim_dev->driver = driver;

	if (driver->probe)
		status = driver->probe(slim_dev);
	if (status) {
		slim_dev->driver = NULL;
	} else if (driver->device_up) {
		ctrl = slim_dev->ctrl;
		schedule_slim_report(ctrl, slim_dev, true);
	}
	return status;
}

static int slim_device_remove(struct device *dev)
{
	struct slim_device *slim_dev;
	struct slim_driver *driver;
	int status = 0;

	slim_dev = to_slim_device(dev);
	if (!dev->driver)
		return 0;

	driver = to_slim_driver(dev->driver);
	if (driver->remove)
		status = driver->remove(slim_dev);

	mutex_lock(&slim_dev->report_lock);
	slim_dev->notified = false;
	if (status == 0)
		slim_dev->driver = NULL;
	mutex_unlock(&slim_dev->report_lock);
	return status;
}

struct bus_type slimbus_type = {
	.name		= "slimbus",
	.match		= slim_device_match,
	.probe		= slim_device_probe,
	.remove		= slim_device_remove,
};
EXPORT_SYMBOL_GPL(slimbus_type);

/**
 * slim_driver_register: Client driver registration with slimbus
 * @drv:Client driver to be associated with client-device.
 * This API will register the client driver with the slimbus
 * It is called from the driver's module-init function.
 */
int slim_driver_register(struct slim_driver *drv)
{
	drv->driver.bus = &slimbus_type;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(slim_driver_register);

/**
 * slim_driver_unregister: Undo effect of slim_driver_register
 * @drv: Client driver to be unregistered
 */
void slim_driver_unregister(struct slim_driver *drv)
{
	if (drv)
		driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(slim_driver_unregister);

static struct slim_controller *slim_ctrl_get(struct slim_controller *ctrl)
{
	if (!ctrl || !get_device(&ctrl->dev))
		return NULL;

	return ctrl;
}

static void slim_ctrl_put(struct slim_controller *ctrl)
{
	if (ctrl)
		put_device(&ctrl->dev);
}

static void slim_dev_release(struct device *dev)
{
	struct slim_device *sbdev = to_slim_device(dev);

	slim_ctrl_put(sbdev->ctrl);
	kfree(sbdev->name);
	kfree(sbdev);
}

static void slim_ctrl_release(struct device *dev)
{
	struct slim_controller *ctrl = to_slim_controller(dev);

	dma_free_coherent(dev->parent, (ctrl->rx.sl_sz * ctrl->rx.n),
			  ctrl->rx.base, ctrl->rx.phy);
	dma_free_coherent(dev->parent, (ctrl->tx.sl_sz * ctrl->tx.n),
			  ctrl->tx.base, ctrl->tx.phy);
}

/**
 * slim_add_device: Add a new device without register board info.
 * @ctrl: Controller to which this device is to be added to.
 * Called when device doesn't have an explicit client-driver to be probed, or
 * the client-driver is a module installed dynamically.
 */
int slim_add_device(struct slim_controller *ctrl, struct slim_device *sbdev)
{
	sbdev->dev.bus = &slimbus_type;
	sbdev->dev.parent = &ctrl->dev;
	sbdev->dev.release = slim_dev_release;
	sbdev->dev.driver = NULL;
	sbdev->ctrl = ctrl;

	slim_ctrl_get(ctrl);
	if (!sbdev->name) {
		sbdev->name = kasprintf(GFP_KERNEL, "%x:%x:%x:%x",
					sbdev->e_addr.manf_id,
					sbdev->e_addr.prod_code,
					sbdev->e_addr.dev_index,
					sbdev->e_addr.instance);
		if (!sbdev->name)
			return -ENOMEM;
	}
	dev_set_name(&sbdev->dev, "%s", sbdev->name);
	mutex_init(&sbdev->report_lock);

	/* probe slave on this controller */
	return device_register(&sbdev->dev);
}
EXPORT_SYMBOL_GPL(slim_add_device);

struct sbi_boardinfo {
	struct list_head	list;
	struct slim_boardinfo	board_info;
};

static LIST_HEAD(board_list);
static LIST_HEAD(slim_ctrl_list);
static DEFINE_MUTEX(board_lock);

#if IS_ENABLED(CONFIG_OF)
/* OF helpers for SLIMbus */
static void of_register_slim_devices(struct slim_controller *ctrl)
{
	struct device_node *node;

	if (!ctrl->dev.of_node)
		return;

	for_each_child_of_node(ctrl->dev.of_node, node) {
		int ret;
		u32 ea[4];
		struct slim_device *slim;
		char *name;

		ret = of_property_read_u32_array(node, "reg", ea, 4);
		if (ret) {
			dev_err(&ctrl->dev, "of_slim: E-addr err:%d\n", ret);
			continue;
		}
		name = kcalloc(SLIMBUS_NAME_SIZE, sizeof(char), GFP_KERNEL);
		if (!name)
			return;

		slim = kzalloc(sizeof(struct slim_device), GFP_KERNEL);
		if (!slim) {
			kfree(name);
			return;
		}
		slim->dev.of_node = of_node_get(node);

		slim->e_addr.manf_id = (u16)ea[0];
		slim->e_addr.prod_code = (u16)ea[1];
		slim->e_addr.dev_index = (u8)ea[2];
		slim->e_addr.instance = (u8)ea[3];

		ret = of_modalias_node(node, name, SLIMBUS_NAME_SIZE);
		if (ret < 0) {
			/* use device address for name, if not specified */
			snprintf(name, SLIMBUS_NAME_SIZE, "%x:%x:%x:%x",
				 slim->e_addr.manf_id, slim->e_addr.prod_code,
				 slim->e_addr.dev_index, slim->e_addr.instance);
		}
		slim->name = name;

		ret = slim_add_device(ctrl, slim);
		if (ret)
			dev_err(&ctrl->dev, "of_slim device register err:%d\n",
				ret);
	}
}
#else
static void of_register_slim_devices(struct slim_controller *ctrl) { }
#endif

/* If controller is not present, only add to boards list */
static void slim_match_ctrl_to_boardinfo(struct slim_controller *ctrl,
					 struct slim_boardinfo *bi)
{
	int ret;

	if (ctrl->nr != bi->bus_num)
		return;

	ret = slim_add_device(ctrl, bi->slim_slave);
	if (ret != 0)
		dev_err(ctrl->dev.parent, "can't create new device %s, ret:%d\n",
			bi->slim_slave->name, ret);
}

/**
 * slim_register_board_info: Board-initialization routine.
 * @info: List of all devices on all controllers present on the board.
 * @n: number of entries.
 * API enumerates respective devices on corresponding controller.
 * Called from board-init function.
 */
int slim_register_board_info(struct slim_boardinfo const *info, unsigned n)
{
	struct sbi_boardinfo *bi;
	int i;

	bi = kcalloc(n, sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	for (i = 0; i < n; i++, bi++, info++) {
		struct slim_controller *ctrl;

		memcpy(&bi->board_info, info, sizeof(*info));
		mutex_lock(&board_lock);
		list_add_tail(&bi->list, &board_list);
		list_for_each_entry(ctrl, &slim_ctrl_list, list)
			slim_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
		mutex_unlock(&board_lock);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(slim_register_board_info);

static void slim_ctrl_add_boarddevs(struct slim_controller *ctrl)
{
	struct sbi_boardinfo *bi;

	of_register_slim_devices(ctrl);
	mutex_lock(&board_lock);
	list_add_tail(&ctrl->list, &slim_ctrl_list);
	list_for_each_entry(bi, &board_list, list)
		slim_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
	mutex_unlock(&board_lock);
}

/**
 * slim_register_controller: Controller bring-up and registration.
 * @ctrl: Controller to be registered.
 * A controller is registered with the framework using this API.
 * If devices on a controller were registered before controller,
 * this will make sure that they get probed when controller is up
 */
int slim_register_controller(struct slim_controller *ctrl)
{
	int id, ret = 0;

	mutex_lock(&slim_lock);
	id = idr_alloc(&ctrl_idr, ctrl, ctrl->nr, -1, GFP_KERNEL);
	mutex_unlock(&slim_lock);

	if (id < 0)
		return id;

	ctrl->nr = id;

	ctrl->dev.release = slim_ctrl_release;
	dev_set_name(&ctrl->dev, "sb-%d", ctrl->nr);
	ctrl->num_dev = 0;

	if (!ctrl->min_cg)
		ctrl->min_cg = SLIM_MIN_CLK_GEAR;
	if (!ctrl->max_cg)
		ctrl->max_cg = SLIM_MAX_CLK_GEAR;

	mutex_init(&ctrl->m_ctrl);
	spin_lock_init(&ctrl->tx.lock);
	spin_lock_init(&ctrl->rx.lock);
	mutex_init(&ctrl->sched.m_reconf);
	init_completion(&ctrl->sched.pause_comp);

	ctrl->pending_wr = kcalloc((ctrl->tx.n - 1),
				   sizeof(struct slim_pending),
				   GFP_KERNEL);
	if (!ctrl->pending_wr) {
		ret = -ENOMEM;
		goto wr_alloc_failed;
	}

	sema_init(&ctrl->tx_sem, (ctrl->tx.n - 1));

	ctrl->tx.base = dma_alloc_coherent(ctrl->dev.parent,
					   (ctrl->tx.sl_sz * ctrl->tx.n),
					   &ctrl->tx.phy, GFP_KERNEL);
	if (!ctrl->tx.base) {
		ret = -ENOMEM;
		goto tx_alloc_failed;
	}

	ctrl->rx.base = dma_alloc_coherent(ctrl->dev.parent,
					   (ctrl->rx.sl_sz * ctrl->rx.n),
					   &ctrl->rx.phy, GFP_KERNEL);
	if (!ctrl->rx.base) {
		ret = -ENOMEM;
		goto rx_alloc_failed;
	}

	ret = device_register(&ctrl->dev);
	if (ret)
		goto dev_reg_failed;

	dev_dbg(&ctrl->dev, "Bus [%s] registered:dev:%p\n",
		ctrl->name, &ctrl->dev);

	ctrl->wq = create_singlethread_workqueue(dev_name(&ctrl->dev));
	if (!ctrl->wq)
		goto err_workq_failed;

	slim_ctrl_add_boarddevs(ctrl);
	return 0;

err_workq_failed:
	device_unregister(&ctrl->dev);
dev_reg_failed:
	dma_free_coherent(ctrl->dev.parent, (ctrl->rx.sl_sz * ctrl->rx.n),
			  ctrl->rx.base, ctrl->rx.phy);
rx_alloc_failed:
	dma_free_coherent(ctrl->dev.parent, (ctrl->tx.sl_sz * ctrl->tx.n),
			  ctrl->tx.base, ctrl->tx.phy);
tx_alloc_failed:
	kfree(ctrl->pending_wr);
wr_alloc_failed:
	mutex_lock(&slim_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);
	dev_err(&ctrl->dev, "slimbus controller registration failed:%d", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_register_controller);

/* slim_remove_device: Remove the effect of slim_add_device() */
void slim_remove_device(struct slim_device *sbdev)
{
	device_unregister(&sbdev->dev);
}
EXPORT_SYMBOL_GPL(slim_remove_device);

static int slim_ctrl_remove_device(struct device *dev, void *null)
{
	slim_remove_device(to_slim_device(dev));
	return 0;
}

/**
 * slim_del_controller: Controller tear-down.
 * @ctrl: Controller to tear-down.
 */
int slim_del_controller(struct slim_controller *ctrl)
{
	struct slim_controller *found;

	/* First make sure that this bus was added */
	mutex_lock(&slim_lock);
	found = idr_find(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);
	if (found != ctrl)
		return -EINVAL;

	/* Remove all clients */
	device_for_each_child(&ctrl->dev, NULL, slim_ctrl_remove_device);

	/* Enter clock pause */
	slim_ctrl_clk_pause(ctrl, false, 0);

	list_del(&ctrl->list);
	destroy_workqueue(ctrl->wq);

	/* free bus id */
	mutex_lock(&slim_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);

	device_unregister(&ctrl->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_del_controller);

/**
 * slim_report_absent: Controller calls this function when a device
 *	reports absent, OR when the device cannot be communicated with
 * @sbdev: Device that cannot be reached, or sent report absent
 */
void slim_report_absent(struct slim_device *sbdev)
{
	struct slim_controller *ctrl;
	int i;

	if (!sbdev)
		return;
	ctrl = sbdev->ctrl;
	if (!ctrl)
		return;

	/* invalidate logical addresses */
	mutex_lock(&ctrl->m_ctrl);
	for (i = 0; i < ctrl->num_dev; i++) {
		if (sbdev->laddr == ctrl->addrt[i].laddr)
			ctrl->addrt[i].valid = false;
	}
	mutex_unlock(&ctrl->m_ctrl);

	mutex_lock(&sbdev->report_lock);
	sbdev->reported = false;
	schedule_slim_report(ctrl, sbdev, false);
	mutex_unlock(&sbdev->report_lock);
}
EXPORT_SYMBOL_GPL(slim_report_absent);

static int slim_boot_child(struct device *dev, void *unused)
{
	struct slim_driver *sbdrv;
	struct slim_device *sbdev = to_slim_device(dev);

	if (sbdev && sbdev->dev.driver) {
		sbdrv = to_slim_driver(sbdev->dev.driver);
		if (sbdrv->boot_device)
			sbdrv->boot_device(sbdev);
	}
	return 0;
}

static int slim_match_dev(struct device *dev, void *data)
{
	struct slim_eaddr *e_addr = data;
	struct slim_device *slim = to_slim_device(dev);

	return slim_eaddr_equal(&slim->e_addr, e_addr);
}

/**
 * slim_framer_booted: This function is called by controller after the active
 * framer has booted (using Bus Reset sequence, or after it has shutdown and has
 * come back up).
 * @ctrl: Controller associated with this framer
 * Components, devices on the bus may be in undefined state,
 * and this function triggers their drivers to do the needful
 * to bring them back in Reset state so that they can acquire sync, report
 * present and be operational again.
 */
void slim_framer_booted(struct slim_controller *ctrl)
{
	if (!ctrl)
		return;

	device_for_each_child(&ctrl->dev, NULL, slim_boot_child);
}
EXPORT_SYMBOL_GPL(slim_framer_booted);

/**
 * slim_query_device: Query and get handle to a device.
 * @ctrl: Controller on which this device will be added/queried
 * @e_addr: Enumeration address of the device to be queried
 * Returns pointer to a device if it has already reported. Creates a new
 * device and returns pointer to it if the device has not yet enumerated.
 */
struct slim_device *slim_query_device(struct slim_controller *ctrl,
				      struct slim_eaddr *e_addr)
{
	struct device *dev;
	struct slim_device *slim = NULL;

	dev = device_find_child(&ctrl->dev, e_addr, slim_match_dev);
	if (dev) {
		slim = to_slim_device(dev);
		return slim;
	}

	slim = kzalloc(sizeof(struct slim_device), GFP_KERNEL);
	if (IS_ERR(slim))
		return NULL;

	slim->e_addr = *e_addr;
	if (slim_add_device(ctrl, slim) != 0) {
		kfree(slim);
		return NULL;
	}
	return slim;
}
EXPORT_SYMBOL_GPL(slim_query_device);

static int ctrl_getaddr_entry(struct slim_controller *ctrl,
			      struct slim_eaddr *eaddr, u8 *entry)
{
	int i;

	for (i = 0; i < ctrl->num_dev; i++) {
		if (ctrl->addrt[i].valid &&
		    slim_eaddr_equal(&ctrl->addrt[i].eaddr, eaddr)) {
			*entry = i;
			return 0;
		}
	}
	return -ENXIO;
}

/**
 * slim_assign_laddr: Assign logical address to a device enumerated.
 * @ctrl: Controller with which device is enumerated.
 * @e_addr: Enumeration address of the device.
 * @laddr: Return logical address (if valid flag is false)
 * @valid: true if laddr holds a valid address that controller wants to
 *	set for this enumeration address. Otherwise framework sets index into
 *	address table as logical address.
 * Called by controller in response to REPORT_PRESENT. Framework will assign
 * a logical address to this enumeration address.
 * Function returns -EXFULL to indicate that all logical addresses are already
 * taken.
 */
int slim_assign_laddr(struct slim_controller *ctrl, struct slim_eaddr *e_addr,
		      u8 *laddr, bool valid)
{
	int ret;
	u8 i = 0;
	bool exists = false;
	struct slim_device *slim;
	struct slim_addrt *temp;

	ret = pm_runtime_get_sync(ctrl->dev.parent);

	if (ctrl->sched.clk_state != SLIM_CLK_ACTIVE) {
		dev_err(&ctrl->dev, "slim ctrl not active,state:%d, ret:%d\n",
				    ctrl->sched.clk_state, ret);
		goto slimbus_not_active;
	}

	mutex_lock(&ctrl->m_ctrl);
	/* already assigned */
	if (ctrl_getaddr_entry(ctrl, e_addr, &i) == 0) {
		*laddr = ctrl->addrt[i].laddr;
		exists = true;
	} else {
		if (ctrl->num_dev >= (SLIM_LA_MANAGER - 1)) {
			ret = -EXFULL;
			goto ret_assigned_laddr;
		}
		for (i = 0; i < ctrl->num_dev; i++) {
			if (ctrl->addrt[i].valid == false)
				break;
		}
		if (i == ctrl->num_dev) {
			temp = krealloc(ctrl->addrt,
					(ctrl->num_dev + 1) *
					sizeof(struct slim_addrt),
					GFP_KERNEL);
			if (!temp) {
				ret = -ENOMEM;
				goto ret_assigned_laddr;
			}
			ctrl->addrt = temp;
			ctrl->num_dev++;
		}
		ctrl->addrt[i].eaddr = *e_addr;
		ctrl->addrt[i].valid = true;

		/* Preferred address is index into table */
		if (!valid)
			*laddr = i;
	}

	ret = ctrl->set_laddr(ctrl, &ctrl->addrt[i].eaddr, *laddr);
	if (ret) {
		ctrl->addrt[i].valid = false;
		goto ret_assigned_laddr;
	}
	ctrl->addrt[i].laddr = *laddr;

ret_assigned_laddr:
	mutex_unlock(&ctrl->m_ctrl);
	if (exists || ret)
		return ret;

	dev_info(&ctrl->dev, "setting slimbus l-addr:%x, ea:%x,%x,%x,%x\n",
		*laddr, e_addr->manf_id, e_addr->prod_code,
		e_addr->dev_index, e_addr->instance);

	/**
	 * Add this device to list of devices on this controller if it's
	 * not already present
	 */
	slim = slim_query_device(ctrl, e_addr);
	if (!slim) {
		ret = -ENODEV;
	} else {
		struct slim_driver *sbdrv;

		slim->laddr = *laddr;
		mutex_lock(&slim->report_lock);
		slim->reported = true;
		if (slim->dev.driver) {
			sbdrv = to_slim_driver(slim->dev.driver);
			if (sbdrv->device_up)
				schedule_slim_report(ctrl, slim, true);
		}
		mutex_unlock(&slim->report_lock);
	}
slimbus_not_active:
	pm_runtime_mark_last_busy(ctrl->dev.parent);
	pm_runtime_put_autosuspend(ctrl->dev.parent);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_assign_laddr);

/**
 * slim_get_logical_addr: Return the logical address of a slimbus device.
 * @sb: client handle requesting the adddress.
 * @e_addr: Enumeration address of the device.
 * @laddr: output buffer to store the address
 * context: can sleep
 * -EINVAL is returned in case of invalid parameters, and -ENXIO is returned if
 *  the device with this enumeration address is not found.
 */
int slim_get_logical_addr(struct slim_device *sb, struct slim_eaddr *e_addr,
			  u8 *laddr)
{
	int ret;
	u8 entry;
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl || !laddr || !e_addr)
		return -EINVAL;

	mutex_lock(&ctrl->m_ctrl);
	ret = ctrl_getaddr_entry(ctrl, e_addr, &entry);
	if (!ret)
		*laddr = ctrl->addrt[entry].laddr;
	mutex_unlock(&ctrl->m_ctrl);

	if (ret == -ENXIO && ctrl->get_laddr) {
		ret = ctrl->get_laddr(ctrl, e_addr, laddr);
		if (!ret)
			ret = slim_assign_laddr(ctrl, e_addr, laddr, true);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(slim_get_logical_addr);

static void __exit slimbus_exit(void)
{
	bus_unregister(&slimbus_type);
}
module_exit(slimbus_exit);

static int __init slimbus_init(void)
{
	return bus_register(&slimbus_type);
}
postcore_initcall(slimbus_init);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("Slimbus module");
