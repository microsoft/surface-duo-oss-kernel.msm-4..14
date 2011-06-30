#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/usb/android_composite.h>

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_all[] = {
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= GOOGLE_VENDOR_ID,
	.product_id	= GOOGLE_PRODUCT_ID,
	.functions	= usb_functions_all,
	.products	= usb_products,
	.version	= 0x0100,
	.product_name	= "generic gadget",
	.manufacturer_name	= "generic",
	.serial_number	= "10101010",
	.num_functions	= ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init android_generic_adb_gadget_init(void)
{
	platform_device_register(&androidusb_device);
	return 0;
}
late_initcall(android_generic_adb_gadget_init);
