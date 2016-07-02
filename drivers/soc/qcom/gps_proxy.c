/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
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

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <uapi/linux/gps_proxy.h>

/* Module information */
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL v2");

#define LOC_SERVICE_SVC_ID 	0x00000010
#define LOC_SERVICE_V1 		2
#define LOC_SERVICE_INS_ID 	0

#define DRIVER_VERSION 		"v1.0"
#define DRIVER_DESC 		"GPS TTY driver"
#define MODULE_NAME 		"gps_proxy"
#define TTY_DRIVER_NAME 	"gps_serial"
#define TTY_DEV_NAME 		"ttyGPS"
#define MAX_TTY_BUFFER_SZ 	0x10000
#define GPS_TTY_MAJOR		100	/* experimental range */
#define DEVICE_NAME 		"gps_proxy_ch"
#define CLASS_NAME 		"gps_proxy_class"

static struct tty_driver *gps_proxy_tty_driver;
static struct tty_port gps_proxy_tty_port;
static bool g_port_open = false;
static struct semaphore g_port_sem;
static int gps_proxy_ch_driver_major = 0;
static struct class* gps_proxy_ch_class = 0;
static struct device* gps_proxy_ch_dev = 0;

static void serial_port_shutdown(struct tty_port *tport)
{
}

static int serial_port_activate(struct tty_port *tport, struct tty_struct *tty)
{
	return 0;
}

static struct tty_port_operations serial_port_ops = {
	.activate = serial_port_activate,
	.shutdown = serial_port_shutdown
};

static int gps_proxy_open(struct tty_struct *tty, struct file *file)
{
	int rc; 
	rc = tty_port_open(&gps_proxy_tty_port, tty, file);
	g_port_open = true;
	up(&g_port_sem);
	return rc;
}

static void gps_proxy_close(struct tty_struct *tty, struct file *file)
{
	tty_port_close(tty->port, tty, file);
	g_port_open = false;
	down(&g_port_sem);
}

static void gps_proxy_tty_hangup(struct tty_struct *tty)
{
	tty_port_hangup(tty->port);
}

static int gps_proxy_write(struct tty_struct *tty, 
		      const unsigned char *buffer, int count)
{
	return count;
}

static int gps_proxy_write_room(struct tty_struct *tty) 
{
	return MAX_TTY_BUFFER_SZ;
}

static int gps_proxy_tty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}


static int gps_proxy_tiocmget(struct tty_struct *tty)
{
	return 0;
}

static int gps_proxy_tiocmset(struct tty_struct *tty,
                         unsigned int set, unsigned int clear)
{
	return 0;
}

static int gps_proxy_ioctl(struct tty_struct *tty,
                      unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct tty_operations serial_ops = {
	.open = gps_proxy_open,
	.close = gps_proxy_close,
	.hangup = gps_proxy_tty_hangup,
	.write = gps_proxy_write,
	.write_room = gps_proxy_write_room,
	.chars_in_buffer = gps_proxy_tty_chars_in_buffer,
	.tiocmget = gps_proxy_tiocmget,
	.tiocmset = gps_proxy_tiocmset,
	.ioctl = gps_proxy_ioctl,
};

int gps_proxy_ch_driver_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int gps_proxy_ch_driver_close(struct inode *inode, struct file *filp)
{
	return 0;
}

long gps_proxy_chdev_ioctl(struct file *filp, unsigned int opt, unsigned long arg)
{
	int rc = 0;
	struct gps_proxy_data buff;
	switch (opt)
	{
		case QGPS_REGISTER_HANDLE:
			/* DOWN is necessary to make client wait till port is open */
			if (!down_killable(&g_port_sem)) {
			/* UP to semaphore is necessary here for close or
			next register handle (for parity) */
				up(&g_port_sem);
				rc = 0;
			}
			else {
				rc = -EFAULT;
			}
			break;
		case QGPS_SEND_NMEA:
			pr_debug(KERN_INFO "Received string: %s\n", 
				((struct gps_proxy_data*)arg)->nmea_string);
			rc = access_ok(struct gps_proxy_data, (struct gps_proxy_data*)arg,
					sizeof(struct gps_proxy_data));
			if (!rc) {
				pr_err(KERN_ERR "Invalid argument was received\n");
				return rc;
			}
			rc = copy_from_user((void*)&buff, (void*)arg, sizeof(buff));
			if (rc) {
				pr_err(KERN_ERR "Number of bytes that \
						 couldn't be copied: %d", rc);
				return -EFAULT;
			}
			if (buff.nmea_length < QMI_LOC_NMEA_STRING_MAX_LENGTH_V02 + 1) {
				pr_debug(KERN_INFO "Received string: %s\n",
					buff.nmea_string);
				rc = tty_insert_flip_string(&gps_proxy_tty_port, 
						buff.nmea_string,
						strnlen(buff.nmea_string,
						QMI_LOC_NMEA_STRING_MAX_LENGTH_V02) + 1);
				if (rc < 0) {
					pr_err(KERN_ERR "Error flipping string");
					return rc;
				}
				tty_flip_buffer_push(&gps_proxy_tty_port);
			}
			else {
				pr_err(KERN_ERR "Illegal message size");
				rc = -EFAULT;
			}
			break;
		case QGPS_IS_ACTIVE:
			if (g_port_open)
				rc = 0;
			else
				rc = -EFAULT;
			break;
		default:
			rc = -EFAULT;
			break;
	}
	return rc;
}

struct file_operations gps_proxy_ch_driver_ops = {
	open: gps_proxy_ch_driver_open,
	unlocked_ioctl: gps_proxy_chdev_ioctl,
	release: gps_proxy_ch_driver_close
};

static int __init gps_proxy_init(void)
{
	int rc;
	struct device *ttydev;

	sema_init(&g_port_sem,0);

	gps_proxy_ch_driver_major = register_chrdev(0, "gps_proxy_ch_dev",
							&gps_proxy_ch_driver_ops);
	if (gps_proxy_ch_driver_major < 0) {
		pr_err(KERN_ERR "Failed to register char device\n");
		return -EFAULT;
	}
	else {
		pr_debug(KERN_INFO "char device registered with major %d\n",
			gps_proxy_ch_driver_major);
	}

	/* Register the device class */
	gps_proxy_ch_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gps_proxy_ch_class)){
		unregister_chrdev(gps_proxy_ch_driver_major, DEVICE_NAME);
		pr_debug(KERN_ALERT "Failed to register device class\n");
		return -EFAULT;
	}
	pr_debug(KERN_INFO "EBBChar: device class registered correctly\n");

	/* Register the device driver */
	gps_proxy_ch_dev = device_create(gps_proxy_ch_class, NULL,
				MKDEV(gps_proxy_ch_driver_major, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gps_proxy_ch_dev)){
		class_destroy(gps_proxy_ch_class);
		unregister_chrdev(gps_proxy_ch_driver_major, DEVICE_NAME);
		pr_debug(KERN_ALERT "Failed to create the device\n");
		return -EFAULT;
	}

	/* allocate the tty driver */
	gps_proxy_tty_driver = alloc_tty_driver(1);
	if (!gps_proxy_tty_driver)
		return -ENOMEM;

	tty_port_init(&gps_proxy_tty_port);
	gps_proxy_tty_port.ops = &serial_port_ops;

	/* initialize the tty driver */
	gps_proxy_tty_driver->driver_name = TTY_DRIVER_NAME;
	gps_proxy_tty_driver->name = TTY_DEV_NAME;
	gps_proxy_tty_driver->major = GPS_TTY_MAJOR;
	gps_proxy_tty_driver->minor_start = 0;
	gps_proxy_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gps_proxy_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	gps_proxy_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	gps_proxy_tty_driver->init_termios = tty_std_termios;

	tty_set_operations(gps_proxy_tty_driver, &serial_ops);
 
	/* register the tty driver */
	rc = tty_register_driver(gps_proxy_tty_driver);
	if (rc) {
		pr_err("Failed to register gps_proxy tty driver\n");
		put_tty_driver(gps_proxy_tty_driver);
		return rc;
	}

	ttydev = tty_port_register_device(&gps_proxy_tty_port, gps_proxy_tty_driver, 0, 0);
	if (IS_ERR(ttydev)) {
		rc = PTR_ERR(ttydev);
		pr_err("Failed to register gps_proxy tty proxy\n");
		return rc;
	}

	pr_debug(KERN_INFO DRIVER_DESC " \n" DRIVER_VERSION);
	return rc;
}

static void __exit gps_proxy_exit(void)
{
	tty_unregister_device(gps_proxy_tty_driver, 0);
	tty_unregister_driver(gps_proxy_tty_driver);
 	unregister_chrdev(gps_proxy_ch_driver_major, "gps_proxy_ch_dev");
	device_destroy(gps_proxy_ch_class, MKDEV(gps_proxy_ch_driver_major, 0));
	class_unregister(gps_proxy_ch_class);
	class_destroy(gps_proxy_ch_class);
}

late_initcall(gps_proxy_init);
module_exit(gps_proxy_exit);
