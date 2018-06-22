/*
 * drivers/char/scbuf-client.c
 *
 * Copyright (c) 2018 Cog Systems Pty Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Secure Camera Buffer client driver.
 *
 * This driver communicates with a resource manager VM through an OKL4 Pipe,
 * to request the physical address ranges for Secure Camera buffers identified
 * by integer handles that have been obtained by userspace out-of-band
 * (typically through a direct IPC link to the VM that exported the buffers).
 * When a physical address range for a buffer is successfully obtained, the
 * buffer is exported to userspace as an FD that supports mmap().
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/scbuf.h>
#include <linux/anon_inodes.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/file.h>
#include <linux/poll.h>

#include <microvisor/microvisor.h>

#include "scbuf-client.h"

#define DRIVER_NAME "scbuf-client"
#define DEVICE_PREFIX "scbuf"
#define MAX_DEVICES 4

#define MAX_REGIONS 32


static struct class *scbuf_class;
static struct ida minor_ida;
static dev_t chr_dev;


struct scbuf_dev {
	atomic_t open;
	struct file *filp;

	okl4_kcap_t tx_pipe_kcap;
	int tx_irq, tx_okl4_irq;
	okl4_kcap_t rx_pipe_kcap;
	int rx_irq, rx_okl4_irq;
	size_t max_msg_size;

	bool notify_start;
	wait_queue_head_t notify_wq;

	bool tx_avail;
	wait_queue_head_t tx_wq;

	/* List of struct scbuf_req, for expected replies */
	spinlock_t req_lock;
	struct list_head req_queue;
	struct work_struct rx_work;
	size_t rx_buf_size;
	struct res_mgr_msg *rx_buf;

	struct platform_device *plat;
	struct cdev cdev;
	int id;
	struct kref kref;
};

static void
scbuf_dev_free(struct kref *kref)
{
	struct scbuf_dev *d = container_of(kref, struct scbuf_dev, kref);

	if (d->rx_buf)
		kfree(d->rx_buf);

	kfree(d);
}

static inline void
scbuf_dev_put(struct scbuf_dev *d)
{
	kref_put(&d->kref, scbuf_dev_free);
}

struct scbuf_region {
	phys_addr_t base;
	size_t size;
};

struct scbuf {
	struct kref kref;
	struct scbuf_dev *dev;

	unsigned int region_count;
	struct scbuf_region regions[];
};

static void
scbuf_free(struct kref *kref)
{
	struct scbuf *b = container_of(kref, struct scbuf, kref);

	scbuf_dev_put(b->dev);
	kfree(b);
}

static inline void
scbuf_put(struct scbuf *b)
{
	kref_put(&b->kref, scbuf_free);
}

struct scbuf_req {
	struct scbuf_dev *dev;
	int ret;
	struct scbuf *buffer;
	struct completion complete;
	struct list_head list;
	struct kref kref;
};

static void
scbuf_req_free(struct kref *kref)
{
	struct scbuf_req *r = container_of(kref, struct scbuf_req, kref);

	if (r->buffer)
		scbuf_put(r->buffer);
	scbuf_dev_put(r->dev);
	kfree(r);
}

static inline void
scbuf_req_put(struct scbuf_req *r)
{
	kref_put(&r->kref, scbuf_req_free);
}

static okl4_error_t
okl4_pipe_control(okl4_kcap_t kcap, uint8_t control)
{
	okl4_pipe_control_t x = 0;

	okl4_pipe_control_setdoop(&x, true);
	okl4_pipe_control_setoperation(&x, control);
	return _okl4_sys_pipe_control(kcap, x);
}

static int
scbuf_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct scbuf *b = (struct scbuf *)filp->private_data;
	size_t size = vma->vm_end - vma->vm_start;
	size_t mapped, remaining;
	unsigned long first_pgoff;
	unsigned long pgoff = vma->vm_pgoff;
	unsigned int first_region, last_region;
	unsigned int i;

	/* Look for the region containing the first page */
	for (i = 0; i < b->region_count; i++) {
		unsigned long region_pages = b->regions[i].size >> PAGE_SHIFT;

		if (pgoff < region_pages)
			break;
		pgoff -= region_pages;
	}
	first_region = i;
	first_pgoff = pgoff;

	/* Offset out of range? */
	if (first_region >= b->region_count)
		return -EINVAL;

	/* Look for the region containing the last page */
	remaining = size;
	pgoff = first_pgoff;
	for (i = first_region; i < b->region_count; i++) {
		if ((pgoff << PAGE_SHIFT) + remaining <= b->regions[i].size)
			break;
		remaining -= b->regions[i].size - (pgoff << PAGE_SHIFT);
		pgoff = 0;
	}
	last_region = i;

	/* End out of range? */
	if (last_region >= b->region_count)
		return -EINVAL;

	/* Now set up the mappings */
	pgoff = first_pgoff;
	mapped = 0;
	remaining = size;
	for (i = first_region; i <= last_region; i++) {
		unsigned long pfn = (b->regions[i].base >> PAGE_SHIFT) + pgoff;
		size_t this_size = min(remaining,
				b->regions[i].size - (pgoff << PAGE_SHIFT));

		if (remap_pfn_range(vma, vma->vm_start + mapped, pfn,
					this_size, vma->vm_page_prot))
			return -EAGAIN;
		remaining -= this_size;
		mapped += this_size;
		pgoff = 0;
	}

	return 0;
}

static int
scbuf_release(struct inode *inode, struct file *filp)
{
	struct scbuf *b = (struct scbuf *)filp->private_data;

	/* Allow the device file to close */
	fput(b->dev->filp);

	filp->private_data = NULL;
	scbuf_put(b);

	return 0;
}

static const struct file_operations scbuf_fops = {
	.owner = THIS_MODULE,
	.mmap = scbuf_mmap,
	.release = scbuf_release,
};

static int
scbuf_lookup_handle(struct scbuf_dev *d, u32 handle)
{
	struct res_mgr_msg msg;
	struct scbuf_req *req;
	int ret;
	okl4_error_t err;

	memset(&msg, 0, sizeof(msg));

	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	kref_init(&req->kref);
	init_completion(&req->complete);

	kref_get(&d->kref);
	req->dev = d;

	msg.msg_id = RES_MGR_LOOKUP_HANDLE;
	msg.securecam.handle = handle;

	do {
		spin_lock_irq(&d->req_lock);
		err = _okl4_sys_pipe_send(d->tx_pipe_kcap, sizeof(msg),
				(const uint8_t *)&msg);
		if (err == OKL4_OK) {
			kref_get(&req->kref);
			list_add_tail(&req->list, &d->req_queue);
		}
		if (err == OKL4_ERROR_PIPE_FULL)
			d->tx_avail = false;
		spin_unlock_irq(&d->req_lock);

		if (err != OKL4_ERROR_PIPE_FULL)
			break;
		if (d->filp->f_flags & O_NONBLOCK)
			break;

		ret = wait_event_interruptible(d->tx_wq, d->tx_avail);
		if (ret < 0)
			goto fail_tx;
	} while (true);

	if (err == OKL4_ERROR_PIPE_FULL) {
		ret = -EAGAIN;
		goto fail_tx;
	} else if (err != OKL4_OK) {
		ret = -EIO;
		goto fail_tx;
	}

	ret = wait_for_completion_interruptible(&req->complete);
	if (ret < 0)
		goto fail_rx;

	ret = req->ret;
	if (ret < 0)
		goto fail_rx;

	if (WARN_ON(req->buffer == NULL)) {
		ret = -EIO;
		goto fail_rx;
	}

	/* Obtain an fd for the buffer */
	ret = anon_inode_getfd("[scbuf]", &scbuf_fops, req->buffer, O_RDWR);
	if (ret < 0)
		goto fail_fd;
	req->buffer = NULL;

	/*
	 * Hold a reference to the device file so it can't be released while
	 * the buffer fd is still open / mapped
	 */
	get_file(d->filp);

fail_fd:
fail_rx:
fail_tx:
	scbuf_req_put(req);
	return ret;
}

static void
scbuf_lookup_handle_reply(struct scbuf_dev *d, struct res_mgr_msg *msg,
		size_t msg_size)
{
	struct scbuf_req *req;
	struct scbuf *b;
	const unsigned int region_count = msg->securecam.sglist.region_count;
	const size_t expected_size = offsetof(struct res_mgr_msg,
			securecam.sglist.regions)
		+ (region_count * sizeof(msg->securecam.sglist.regions[0]));
	unsigned int i;

	spin_lock(&d->req_lock);
	req = list_first_entry(&d->req_queue, struct scbuf_req, list);
	list_del(&req->list);
	spin_unlock(&d->req_lock);

	if (region_count == 0) {
		req->ret = -ENXIO;
		goto out;
	} else if (region_count > MAX_REGIONS) {
		dev_err(&d->plat->dev, "too many regions in sglist: %u\n",
				region_count);
		req->ret = -EIO;
		goto out;
	}

	if (msg_size < expected_size) {
		dev_err(&d->plat->dev, "short reply: want %zd bytes, got %zd\n",
				expected_size, msg_size);
		req->ret = -EIO;
		goto out;
	}

	b = kzalloc(sizeof(*b) + (region_count * sizeof(b->regions[0])),
			GFP_KERNEL);
	if (!b) {
		req->ret = -ENOMEM;
		goto out;
	}
	kref_init(&b->kref);
	b->dev = req->dev;
	kref_get(&b->dev->kref);

	/* Copy the regions into the buffer object */
	b->region_count = region_count;
	for (i = 0; i < region_count; i++) {
		phys_addr_t base = msg->securecam.sglist.regions[i].address_ipa;
		phys_addr_t size = msg->securecam.sglist.regions[i].size;

		if (!IS_ALIGNED(base, PAGE_SIZE) ||
				!IS_ALIGNED(size, PAGE_SIZE)) {
			dev_err(&d->plat->dev, "misaligned region %#zx + %#zx\n",
					(size_t)base, (size_t)size);
			req->ret = -EIO;
			scbuf_put(b);
			goto out;
		}

		b->regions[i].base = base;
		b->regions[i].size = size;
	}

	req->buffer = b;
	req->ret = 0;

out:
	complete(&req->complete);
	scbuf_req_put(req);
}

static void
scbuf_notify_start(struct scbuf_dev *d)
{
	const uint32_t msg_id = RES_MGR_SECURECAM_ACK_START;
	_okl4_sys_pipe_send(d->tx_pipe_kcap, sizeof(msg_id),
			(const uint8_t *)&msg_id);
	d->notify_start = true;
	wake_up_all(&d->notify_wq);
}

static void
scbuf_rx_work(struct work_struct *work)
{
	struct scbuf_dev *d = container_of(work, struct scbuf_dev, rx_work);
	struct _okl4_sys_pipe_recv_return recv_ret;
	struct res_mgr_msg *msg = d->rx_buf;

	/*
	 * Note that there's no need to take the lock before the receive call,
	 * because we only read in this work item which is serialised.
	 */
	recv_ret = _okl4_sys_pipe_recv(d->rx_pipe_kcap, d->rx_buf_size,
			(uint8_t *)msg);

	/* If the pipe is empty, nothing to do */
	if (recv_ret.error == OKL4_ERROR_PIPE_EMPTY)
		return;

	if (recv_ret.error != OKL4_OK) {
		dev_err(&d->plat->dev, "pipe receive error: %d\n",
				recv_ret.error);
		return;
	}
	if (recv_ret.size < sizeof(u32)) {
		dev_err(&d->plat->dev, "pipe short message, ignored\n");
		return;
	}

	if (msg->msg_id == RES_MGR_LOOKUP_HANDLE_REPLY) {
		scbuf_lookup_handle_reply(d, msg, recv_ret.size);
	} else if (msg->msg_id == RES_MGR_SECURECAM_NOTIFY_START) {
		scbuf_notify_start(d);
	} else {
		dev_err(&d->plat->dev, "pipe unknown message id: %#x\n",
				(unsigned int)msg->msg_id);
	}

	/* Reschedule the work in case there's another message waiting */
	schedule_work(work);
}

static long
scbuf_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	int ret;
	struct scbuf_dev *d = (struct scbuf_dev *)filp->private_data;

	switch (cmd) {
	case IOCTL_SCBUF_LOOKUP_HANDLE:
		ret = scbuf_lookup_handle(d, (u32)data);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int
scbuf_open(struct inode *inode, struct file *filp)
{
	int ret;
	okl4_error_t okl4_err;
	struct scbuf_dev *d = container_of(inode->i_cdev,
			struct scbuf_dev, cdev);

	if (atomic_inc_return(&d->open) > 1) {
		atomic_dec(&d->open);
		return -EBUSY;
	}

	kref_get(&d->kref);
	filp->private_data = d;
	d->filp = filp;

	okl4_err = okl4_pipe_control(d->rx_pipe_kcap,
			OKL4_PIPE_CONTROL_OP_SET_RX_READY);
	if (okl4_err != OKL4_OK) {
		ret = -EIO;
		goto fail_rx_ready;
	}

	okl4_err = okl4_pipe_control(d->tx_pipe_kcap,
			OKL4_PIPE_CONTROL_OP_SET_TX_READY);
	if (okl4_err != OKL4_OK) {
		ret = -EIO;
		goto fail_tx_ready;
	}

	d->tx_avail = true;
	schedule_work(&d->rx_work);

	return 0;

fail_tx_ready:
	okl4_pipe_control(d->rx_pipe_kcap, OKL4_PIPE_CONTROL_OP_RESET);
fail_rx_ready:
	d->filp = NULL;
	filp->private_data = NULL;
	scbuf_dev_put(d);
	atomic_dec(&d->open);
	return ret;
}

static unsigned int
scbuf_poll(struct file *filp, poll_table *wait)
{
	struct scbuf_dev *d = (struct scbuf_dev *)filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &d->notify_wq, wait);
	poll_wait(filp, &d->tx_wq, wait);

	if (d->notify_start)
		mask |= POLLPRI;
	if (d->tx_avail)
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static int
scbuf_close(struct inode *inode, struct file *filp)
{
	struct scbuf_dev *d = container_of(inode->i_cdev,
			struct scbuf_dev, cdev);

	/* Prevent any buffer creation or notifications while closed */
	okl4_pipe_control(d->rx_pipe_kcap, OKL4_PIPE_CONTROL_OP_RESET);
	okl4_pipe_control(d->tx_pipe_kcap, OKL4_PIPE_CONTROL_OP_RESET);

	/* Ensure that interrupt handlers have completed */
	disable_irq(d->rx_irq);
	disable_irq(d->tx_irq);

	/* Abort any still-pending buffer lookups */
	spin_lock(&d->req_lock);
	while (!list_empty(&d->req_queue)) {
		struct scbuf_req *req = list_first_entry(
				&d->req_queue, struct scbuf_req, list);
		req->ret = -ECANCELED;
		complete(&req->complete);
		list_del(&req->list);
		scbuf_req_put(req);
	}
	spin_unlock(&d->req_lock);

	/* Clean up the open device's state */
	d->notify_start = false;
	d->filp = NULL;
	filp->private_data = NULL;
	scbuf_dev_put(d);
	atomic_dec(&d->open);

	/* Re-enable interrupts */
	enable_irq(d->rx_irq);
	enable_irq(d->tx_irq);

	return 0;
}

static const struct file_operations scbuf_dev_fops = {
	.owner = THIS_MODULE,
	.open = scbuf_open,
	.release = scbuf_close,
	.compat_ioctl = scbuf_ioctl,
	.unlocked_ioctl = scbuf_ioctl,
	.poll = scbuf_poll,
};

static irqreturn_t
scbuf_tx_irq(int irq, void *dev)
{
	struct scbuf_dev *d = dev;
	struct _okl4_sys_interrupt_get_payload_return ret;
	okl4_pipe_state_t payload;

	ret = _okl4_sys_interrupt_get_payload(d->tx_okl4_irq);
	if (ret.error != OKL4_OK)
		return IRQ_NONE;

	payload = ret.payload;
	if (okl4_pipe_state_gettxavailable(&payload)) {
		d->tx_avail = true;
		wake_up(&d->tx_wq);
	}

	return IRQ_HANDLED;
}

static irqreturn_t
scbuf_rx_irq(int irq, void *dev)
{
	struct scbuf_dev *d = dev;
	struct _okl4_sys_interrupt_get_payload_return ret;
	okl4_pipe_state_t payload;

	ret = _okl4_sys_interrupt_get_payload(d->rx_okl4_irq);
	if (ret.error != OKL4_OK)
		return IRQ_NONE;

	payload = ret.payload;
	if (okl4_pipe_state_getrxavailable(&payload))
		schedule_work(&d->rx_work);

	return IRQ_HANDLED;
}

static int
scbuf_probe(struct platform_device *plat)
{
	int ret;
	dev_t new_chr_dev;
	struct device *device;
	u32 reg[2];
	u32 size_prop;
	struct resource *irq;

	struct scbuf_dev *priv = kzalloc(sizeof(struct scbuf_dev),
			GFP_KERNEL);
	if (priv == NULL) {
		ret = -ENOMEM;
		goto fail_alloc_priv;
	}
	priv->plat = plat;
	kref_init(&priv->kref);
	dev_set_drvdata(&plat->dev, priv);

	INIT_WORK(&priv->rx_work, scbuf_rx_work);
	INIT_LIST_HEAD(&priv->req_queue);
	atomic_set(&priv->open, 0);
	init_waitqueue_head(&priv->notify_wq);
	init_waitqueue_head(&priv->tx_wq);

	ret = ida_simple_get(&minor_ida, 0, MAX_DEVICES, GFP_KERNEL);
	if (ret < 0)
		goto fail_alloc_minor;
	priv->id = ret;

	if (of_property_read_u32_array(plat->dev.of_node, "reg", reg, 2)) {
		dev_err(&plat->dev, "two OKL4 Pipe capabilities required\n");
		ret = -ENODEV;
		goto fail_pipe_kcaps;
	}
	priv->tx_pipe_kcap = reg[0];
	priv->rx_pipe_kcap = reg[1];

	if (of_property_read_u32(plat->dev.of_node, "okl,message-size",
				&size_prop)) {
		dev_warn(&plat->dev, "message size unknown, assuming 128\n");
		size_prop = 128;
	}
	priv->rx_buf_size = (size_t)size_prop;
	priv->rx_buf = kmalloc(priv->rx_buf_size, GFP_KERNEL);
	if (!priv->rx_buf) {
		ret = -ENOMEM;
		goto fail_alloc_rx_buf;
	}

	irq = platform_get_resource(plat, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&plat->dev, "no TX interrupt found");
		ret = -ENODEV;
		goto fail_tx_interrupt;
	}
	priv->tx_irq = irq->start;
	ret = devm_request_irq(&plat->dev, priv->tx_irq, scbuf_tx_irq, 0,
			dev_name(&plat->dev), priv);
	if (ret) {
		dev_err(&plat->dev, "can't register TX interrupt %d: %d\n",
				priv->tx_irq, ret);
		goto fail_tx_interrupt;
	}
	priv->tx_okl4_irq = irqd_to_hwirq(irq_get_irq_data(priv->tx_irq));

	irq = platform_get_resource(plat, IORESOURCE_IRQ, 1);
	if (!irq) {
		dev_err(&plat->dev, "no RX interrupt found");
		ret = -ENODEV;
		goto fail_rx_interrupt;
	}
	priv->rx_irq = irq->start;
	ret = devm_request_irq(&plat->dev, priv->rx_irq, scbuf_rx_irq, 0,
			dev_name(&plat->dev), priv);
	if (ret) {
		dev_err(&plat->dev, "can't register RX interrupt %d: %d\n",
				priv->rx_irq, ret);
		goto fail_rx_interrupt;
	}
	priv->rx_okl4_irq = irqd_to_hwirq(irq_get_irq_data(priv->rx_irq));

	new_chr_dev = MKDEV(MAJOR(chr_dev), MINOR(chr_dev) + priv->id);
	cdev_init(&priv->cdev, &scbuf_dev_fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, new_chr_dev, 1);
	if (ret)
		goto fail_cdev_add;

	device = device_create(scbuf_class, &plat->dev, new_chr_dev,
			NULL, DEVICE_PREFIX "%d", priv->id);
	if (IS_ERR(device)) {
		ret = PTR_ERR(scbuf_class);
		goto fail_device_create;
	}

	return 0;

	device_destroy(scbuf_class, priv->cdev.dev);
fail_device_create:
	cdev_del(&priv->cdev);
fail_cdev_add:
	devm_free_irq(&plat->dev, priv->rx_irq, priv);
fail_rx_interrupt:
	devm_free_irq(&plat->dev, priv->tx_irq, priv);
fail_tx_interrupt:
fail_alloc_rx_buf:
fail_pipe_kcaps:
	ida_simple_remove(&minor_ida, priv->id);
fail_alloc_minor:
	priv->plat = NULL;

	dev_set_drvdata(&plat->dev, NULL);
	kref_put(&priv->kref, scbuf_free);
fail_alloc_priv:
	return ret;
}

static int
scbuf_remove(struct platform_device *plat)
{
	struct scbuf_dev *priv = dev_get_drvdata(&plat->dev);

	device_destroy(scbuf_class, priv->cdev.dev);
	cdev_del(&priv->cdev);

	devm_free_irq(&plat->dev, priv->rx_irq, priv);
	devm_free_irq(&plat->dev, priv->tx_irq, priv);

	ida_simple_remove(&minor_ida, priv->id);

	priv->plat = NULL;
	synchronize_rcu();

	dev_set_drvdata(&plat->dev, NULL);
	kref_put(&priv->kref, scbuf_free);

	return 0;
}

static const struct of_device_id scbuf_match[] = {
	{ .compatible = "qcom,resource-manager-scbuf" },
	{},
};

static struct platform_driver scbuf_driver = {
	.probe = scbuf_probe,
	.remove = scbuf_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = scbuf_match,
	},
};

static int __init
scbuf_init(void)
{
	int ret;

	ida_init(&minor_ida);

	ret = alloc_chrdev_region(&chr_dev, 0, MAX_DEVICES, DEVICE_PREFIX);
	if (ret < 0)
		goto fail_alloc_chrdev_region;

	scbuf_class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(scbuf_class)) {
		ret = PTR_ERR(scbuf_class);
		goto fail_class_create;
	}

	ret = platform_driver_register(&scbuf_driver);
	if (ret < 0)
		goto fail_driver_register;

	return 0;

fail_driver_register:
	class_destroy(scbuf_class);
fail_class_create:
	unregister_chrdev_region(chr_dev, MAX_DEVICES);
fail_alloc_chrdev_region:
	return ret;
}

static void __exit
scbuf_exit(void)
{
	platform_driver_unregister(&scbuf_driver);
	class_destroy(scbuf_class);
	unregister_chrdev_region(chr_dev, MAX_DEVICES);
}

module_init(scbuf_init);
module_exit(scbuf_exit);

MODULE_DESCRIPTION("Secure Camera Buffer client");
