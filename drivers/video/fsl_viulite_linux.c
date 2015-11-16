/*
 * Copyright 2012-2014 Freescale Semiconductor, Inc.
 *
 * Freescale fsl-VIUlite device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/pm_runtime.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_net.h>
#include <linux/cdev.h>
#include <asm/current.h>
#include <asm/segment.h>
#include <linux/uaccess.h>

#include <uapi/video/fsl_viulite_ioctl.h>

/**********************************************************
 * DCU regbase, found in agnostic DCU driver layer
 **********************************************************/
/* number of DCU units */
#define VIULITE_DEV_COUNT    1

/* list of DCU base addr */
uint64_t *viulite_base;

/**********************************************************
 * DCU defines
 **********************************************************/
#define VIULite_INFO    KERN_INFO
#define MAJOR_NUM       100
#define DEVICE_NAME     "fsl_viulite0"
#define DRIVER_NAME     "fsl_viulite1"

/**********************************************************
 * Macros for tracing
 **********************************************************/
/*#define __LOG_TRACE__ 1*/

#ifdef __LOG_TRACE__
	#define __TRACE__() printk(VIULite_INFO "[ fsl-VIU ] %s\n", __func__)
	#define __MSG_TRACE__(string, args...) printk(VIULite_INFO \
		"[ fsl-VIU ]  %s : %d : " string, __func__, __LINE__, ##args)
#else
	#define __TRACE__()
	#define __MSG_TRACE__(string, args...)
	#define __HERE__ printk(VIULite_INFO " HERE %s\n", __func__)
#endif

/**********************************************************
 * GLOBAL DCU configuration registers
 **********************************************************/
void __iomem *viulite_reg_base;
struct clk   *viulite_clk;
uint32_t      viulite_clk_val;
/*
struct viulite_data {
	struct fb_info **fsl_dcu_info;
	struct device *dev;
	void __iomem *reg_base;
	unsigned int irq;
	struct clk *clk;
};

struct viulite_data  *viulite_data;
*/
struct platform_device *viulite_pdev;
struct cdev            *viulite_cdev;
struct class           *viulite_class;
dev_t  viulite_devno; /*__u32 */

#define DCU_INIT_TRUE       0
#define DCU_INIT_ERR_PROBE  1
#define DCU_INIT_ERR_CFG    2
int viulite_init_status = DCU_INIT_ERR_PROBE;

/**********************************************************
 * FUNCTION: fsl_viulite_get_pdev
 **********************************************************/
struct platform_device *fsl_viulite_get_pdev(void)
{
	__TRACE__();
	return viulite_pdev;
}

/**********************************************************
 * FUNCTION: fsl_viulite_getinitstatus
 **********************************************************/
int fsl_viulite_getinitstatus(void)
{
	__TRACE__();
	return viulite_init_status;
}

/**********************************************************
 * FUNCTION: fsl_viulite_regs
 **********************************************************/
void fsl_viulite_regs(void)
{
	printk(VIULite_INFO "-----------VIULite REGS ----------\n");
	printk(VIULite_INFO "[REG : VIULite_SCR]\t : %02x => %08x\n",
		0x0,
		readl(viulite_base));
	printk(VIULite_INFO "[REG : VIULite_INTR  ]\t : %02x => %08x\n",
		0x04,
		readl(viulite_base + 0x04));
	printk(VIULite_INFO "[REG : VIULite_DINVSZ]\t : %02x => %08x\n",
		0x08,
		readl(viulite_base + 0x08));
	printk(VIULite_INFO "[REG : VIULite_DINVFL]\t : %02x => %08x\n",
		0x0C,
		readl(viulite_base + 0x0C));
	printk(VIULite_INFO "[REG : VIULite_DMA_SIZE]\t : %02x => %08x\n",
		0x10,
		readl(viulite_base + 0x10));
	printk(VIULite_INFO "[REG : VIULite_DMA_ADDR]\t : %02x => %08x\n",
		0x14,
		readl(viulite_base + 0x14));
	printk(VIULite_INFO "[REG : VIULite_DMA_INC]\t : %02x => %08x\n",
		0x18,
		readl(viulite_base + 0x18));
	printk(VIULite_INFO "[REG : VIULite_INVSZ]\t : %02x => %08x\n",
		0x1C,
		readl(viulite_base + 0x1C));
	printk(VIULite_INFO "[REG : VIULite_ALPHA]\t : %02x => %08x\n",
		0x24,
		readl(viulite_base + 0x24));
	printk(VIULite_INFO "[REG : VIULite_ACT_ORG]\t : %02x => %08x\n",
		0x28,
		readl(viulite_base + 0x28));
	printk(VIULite_INFO "[REG : VIULite_ACT_SIZE]\t : %02x => %08x\n",
		0x2C,
		readl(viulite_base + 0x2C));
	printk(VIULite_INFO "-------------------------------\n");
}

/**********************************************************
 * FUNCTION: fsl_viulite_irq
 **********************************************************/
irqreturn_t fsl_viulite_irq(int irq, void *dev_id)
{
	__TRACE__();
	return IRQ_HANDLED;
}

/**********************************************************
 * FUNCTION: fsl_viulite_remove
 **********************************************************/
int fsl_viulite_remove(struct platform_device *pdev)
{
	__TRACE__();

	cdev_del(viulite_cdev);
	device_destroy(viulite_class, viulite_devno);
	class_destroy(viulite_class);
	unregister_chrdev_region(viulite_devno, 1);

	/*DCU_Disable(0);*/

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

/**********************************************************
 * FUNCTIONS: fsl_viulite_ioctl set of functions
 **********************************************************/

void viulite_set_videoinputformat(uint64_t *reg_base,
			VIU_INPUT_FORMAT *in_format)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_ITU_MODE_MASK;
	reg_value &= ~SCR_CPP_MASK;
	reg_value &= ~SCR_INWIDTH_MASK;

	reg_value |= ((((uint32_t)(in_format->mode)) << SCR_ITUMODE_OFFSET)
			& SCR_ITU_MODE_MASK);
	reg_value |= ((((uint32_t)(in_format->width)) << SCR_INWIDTH_OFFSET)
			& SCR_INWIDTH_MASK);
	reg_value |= ((((uint32_t)(in_format->clocks_per_pixell))
			<< SCR_CPP_OFFSET) & SCR_CPP_MASK);

	writel(reg_value, reg_address);
}

void viulite_get_videoinputformat(uint64_t *reg_base,
			VIU_INPUT_FORMAT *in_format)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	in_format->mode = (in_mode_t)((reg_value & SCR_ITU_MODE_MASK) >>
			SCR_ITUMODE_OFFSET);
	in_format->width = (in_width_t)((reg_value & SCR_INWIDTH_MASK) >>
			SCR_INWIDTH_OFFSET);
	in_format->clocks_per_pixell = (uint8_t)((reg_value & SCR_CPP_MASK) >>
			SCR_CPP_OFFSET);
}

void viulite_set_datainterface(uint64_t *reg_base,
			VIU_DATA_INTERFACE *data_interface)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_PCLK_POL_MASK;
	reg_value &= ~SCR_VSYNC_POL_MASK;
	reg_value &= ~SCR_HSYNC_POL_MASK;
	reg_value &= ~SCR_ENDIANNESS_MASK;

	reg_value |= ((((uint32_t)(data_interface->pclk_pol))  <<
			SCR_PCLK_POL_OFFSET)  & SCR_PCLK_POL_MASK);
	reg_value |= ((((uint32_t)(data_interface->vsync_pol)) <<
			SCR_VSYNC_POL_OFFSET) & SCR_VSYNC_POL_MASK);
	reg_value |= ((((uint32_t)(data_interface->hsync_pol)) <<
			SCR_HSYNC_POL_OFFSET) & SCR_HSYNC_POL_MASK);
	reg_value |= ((((uint32_t)(data_interface->endianness)) <<
			SCR_ENDIANNESS_OFFSET) & SCR_ENDIANNESS_MASK);

	writel(reg_value, reg_address);
}

void viulite_get_datainterface(uint64_t *reg_base,
			VIU_DATA_INTERFACE *data_interface)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(
	reg_address);
	data_interface->pclk_pol  = (uint8_t)((reg_value & SCR_PCLK_POL_MASK)
			>> SCR_PCLK_POL_OFFSET);
	data_interface->vsync_pol = (uint8_t)((reg_value & SCR_VSYNC_POL_MASK)
			>> SCR_VSYNC_POL_OFFSET);
	data_interface->hsync_pol = (uint8_t)((reg_value & SCR_HSYNC_POL_MASK)
			>> SCR_HSYNC_POL_OFFSET);
	data_interface->endianness = (uint8_t)((reg_value & SCR_ENDIANNESS_MASK)
			>> SCR_ENDIANNESS_OFFSET);
}

void viulite_dma_config(uint64_t *reg_base, DMA_CONFIG *dmaconfig)
{
	uint64_t *reg_address;
	uint32_t  reg_value;

	reg_address = reg_base + DMA_ADDR_OFFSET;
	writel(dmaconfig->buff_addr, reg_address);

	reg_address = reg_base + DMA_SIZE_OFFSET;
	writel(dmaconfig->tx_size, reg_address);

	reg_address = reg_base + INVSZ_OFFSET;
	reg_value = (uint32_t) (((uint32_t)(dmaconfig->nmb_lines)) << 16)
			| (uint32_t)(dmaconfig->nmb_pixells);
	writel(reg_value, reg_address);

	reg_address = reg_base + DMA_INC_OFFSET;
	writel(dmaconfig->frame_inc, reg_address);

	if (MAX_ALPHA_VAL > dmaconfig->alpha_val) {
		reg_address = reg_base + ALPHA_OFFSET;
		writel(dmaconfig->alpha_val, reg_address);
	}
}

void viulite_dma_start(uint64_t *reg_base)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	if (0 == (reg_value & SCR_DMA_ACT_MASK)) {
		reg_value |= SCR_DMA_ACT_MASK;
		writel(reg_value, reg_address);
	}
}

void viulite_dma_getstatus(uint64_t *reg_base, VIU_BOOL *dmastatus)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	if (0 == (reg_value & SCR_DMA_ACT_MASK))
		*dmastatus = VIU_OFF;
	else
		*dmastatus = VIU_ON;

}

void viulite_sw_reset(uint64_t *reg_base, VIU_BOOL res_status)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_SWRESET_MASK;

	if (VIU_ON == res_status)
		reg_value |= SCR_SWRESET_MASK;

	writel(reg_value, reg_address);
}

void viulite_set_ituerror(uint64_t *reg_base, VIU_BOOL itu_errset)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_ECCEN_MASK;

	if (VIU_ON == itu_errset)
		reg_value |= SCR_ECCEN_MASK;

	writel(reg_value, reg_address);
}

void viulite_get_ituerror(uint64_t *reg_base, VIU_ITU_ERR *itu_errcode)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);
	*itu_errcode = (VIU_ITU_ERR)((reg_value & INTR_ITUERR_MASK)
			>> INTR_ITUERR_OFFSET);
}

void viulite_get_irqstatus(uint64_t *reg_base, uint8_t *intr_status)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);
	*intr_status = (uint8_t)((reg_value & INTR_STATUS_MASK)
			>> INTR_STATUS_OFFSET);
}

void viulite_reset_irqstatus(uint64_t *reg_base, uint8_t intr_status)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);

	reg_value &= ~INTR_STATUS_MASK;
	reg_value |= ((uint32_t)intr_status << INTR_STATUS_OFFSET);

	writel(reg_value, reg_address);
}

void viulite_get_syncsignals(uint64_t *reg_base, VIU_SYNC_STATUS *sync_status)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	sync_status->vsync = (uint8_t)((reg_value & SCR_VSYNC_MASK)
			>> SCR_VSYNC_OFFSET);
	sync_status->hsync = (uint8_t)((reg_value & SCR_HSYNC_MASK)
			>> SCR_HSYNC_OFFSET);
}

void viulite_get_fieldnum(uint64_t *reg_base, uint8_t *field_n)
{
	uint32_t  reg_value;
	uint64_t *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	*field_n = (uint8_t)((reg_value & SCR_FIELDNO_MASK)
			>> SCR_FIELDNO_OFFSET);
}

void viulite_get_framesize(uint64_t *reg_base, VIU_FRAME_SIZE *frame_params)
{

}

void viulite_set_clippingdata(uint64_t *reg_base,
		VIU_IMAGE_PARAMS *image_params)
{
	uint32_t  reg_value;
	uint64_t *reg_address;

	reg_address = reg_base + ACTORG_OFFSET;
	reg_value = ((uint32_t)(image_params->x_origin) & CLIPP_LOW16_MASK) |
			((uint32_t)(image_params->y_origin)<<16);
	writel(reg_value, reg_address);

	reg_address = reg_base + ACTSIZE_OFFSET;
	reg_value = ((uint32_t)(image_params->x_size) & CLIPP_LOW16_MASK) |
			((uint32_t)(image_params->y_size)<<16);
	writel(reg_value, reg_address);
}

void viulite_get_clippingdata(uint64_t *reg_base,
		VIU_IMAGE_PARAMS *image_params)
{
	uint32_t  reg_value;
	uint64_t *reg_address;

	reg_address = reg_base + ACTORG_OFFSET;
	reg_value = readl(reg_address);
	image_params->x_origin = (uint16_t)(reg_value & CLIPP_LOW16_MASK);
	image_params->y_origin = (uint16_t)((reg_value & CLIPP_HIGH16_MASK)
			>> 16);

	reg_address = reg_base + ACTSIZE_OFFSET;
	reg_value = readl(reg_address);
	image_params->x_size = (uint16_t)(reg_value & CLIPP_LOW16_MASK);
	image_params->y_size = (uint16_t)((reg_value & CLIPP_HIGH16_MASK)
			>> 16);
}

/**********************************************************
 * FUNCTION: fsl_viulite_ioctl
 * VIULite Linux IOCTL operations
 **********************************************************/
long fsl_viulite_ioctl(struct file *filp, unsigned int ioctl_cmd,
		unsigned long arg)
{
	int ret;

	__TRACE__();

	switch (ioctl_cmd) {
	case VIULITE_IOCTL_DMA_CONFIG:
	{
		DMA_CONFIG dma_configuration;

		/* copy from user space */
		ret = copy_from_user(&dma_configuration, (DMA_CONFIG *)arg,
				sizeof(dma_configuration));

		viulite_dma_config(viulite_base, &dma_configuration);
	}
	break;

	case VIULITE_IOCTL_DMA_START:
	{
		viulite_dma_start(viulite_base);
	}
	break;

	case VIULITE_IOCTL_DMA_GET_STATUS:
	{
		VIU_BOOL dma_st;

		viulite_dma_getstatus(viulite_base, &dma_st);

		/* copy back to user space */
		ret = copy_to_user((VIU_BOOL *)arg, &dma_st, sizeof(dma_st));
	}
	break;

	case VIULITE_IOCTL_SW_RESET:
	{
		VIU_BOOL res_status;

		/* copy from user space */
		ret = copy_from_user(&res_status, (VIU_BOOL *)arg,
				sizeof(res_status));

		viulite_sw_reset(viulite_base, res_status);
	}
	break;

	case VIULITE_IOCTL_SET_VIDEOIN_FORMAT:
	{
		VIU_INPUT_FORMAT in_format;

		/* copy from user space */
		ret = copy_from_user(&in_format, (VIU_INPUT_FORMAT *)arg,
				sizeof(in_format));

		viulite_set_videoinputformat(viulite_base, &in_format);

	}
	break;

	case VIULITE_IOCTL_GET_VIDEOIN_FORMAT:
	{
		VIU_INPUT_FORMAT in_format;

		viulite_get_videoinputformat(viulite_base, &in_format);

		/* copy back to user space */
		ret = copy_to_user((VIU_INPUT_FORMAT *)arg, &in_format,
				sizeof(in_format));
	}
	break;

	case VIULITE_IOCTL_SET_DATA_INTERFACE:
	{
		VIU_DATA_INTERFACE data_interface;

		/* copy from user space */
		ret = copy_from_user(&data_interface, (VIU_DATA_INTERFACE *)arg,
				sizeof(data_interface));

		viulite_set_datainterface(viulite_base, &data_interface);

	}
	break;

	case VIULITE_IOCTL_GET_DATA_INTERFACE:
	{
		VIU_DATA_INTERFACE data_interface;

		viulite_get_datainterface(viulite_base, &data_interface);

		/* copy back to user space */
		ret = copy_to_user((VIU_DATA_INTERFACE *)arg, &data_interface,
				sizeof(data_interface));
	}
	break;

	case VIULITE_IOCTL_SET_ITU_ERRCODE:
	{
		VIU_BOOL itu_errset;

		/* copy from user space */
		ret = copy_from_user(&itu_errset, (VIU_BOOL *)arg,
				sizeof(itu_errset));

		viulite_set_ituerror(viulite_base, itu_errset);

	}
	break;

	case VIULITE_IOCTL_RESET_IRQSTATUS:
	{
		uint8_t irq_status;

		/* copy from user space */
		ret = copy_from_user(&irq_status, (uint8_t *)arg,
				sizeof(irq_status));

		viulite_reset_irqstatus(viulite_base, irq_status);

	}
	break;

	case VIULITE_IOCTL_GET_IRQSTATUS:
	{
		uint8_t irq_status;

		viulite_get_irqstatus(viulite_base, &irq_status);

		/* copy back to user space */
		ret = copy_to_user((uint8_t *)arg, &irq_status,
				sizeof(irq_status));
	}
	break;

	case VIULITE_IOCTL_GET_ITU_ERRCODE:
	{
		VIU_ITU_ERR itu_err;

		viulite_get_ituerror(viulite_base, &itu_err);

		/* copy back to user space */
		ret = copy_to_user((VIU_ITU_ERR *)arg, &itu_err,
				sizeof(itu_err));
	}
	break;

	case VIULITE_IOCTL_GET_FIELDNUM:
	{
		uint8_t field_num;

		viulite_get_fieldnum(viulite_base, &field_num);

		/* copy back to user space */
		ret = copy_to_user((uint8_t *)arg, &field_num,
				sizeof(field_num));
	}
	break;

	case VIULITE_IOCTL_GET_SYNC:
	{
		VIU_SYNC_STATUS sync_st;

		viulite_get_syncsignals(viulite_base, &sync_st);

		/* copy back to user space */
		ret = copy_to_user((VIU_SYNC_STATUS *)arg, &sync_st,
				sizeof(sync_st));
	}
	break;

	case VIULITE_IOCTL_GET_FRAME_SIZE:
	{
		VIU_FRAME_SIZE frame_size;

		viulite_get_framesize(viulite_base, &frame_size);

		/* copy to user space */
		ret = copy_to_user(&frame_size, (VIU_FRAME_SIZE *)arg,
				sizeof(frame_size));
	}
	break;

	case VIULITE_IOCTL_SET_CLIPPING:
	{
		VIU_IMAGE_PARAMS image_params;

		/* copy from user space */
		ret = copy_from_user(&image_params, (VIU_IMAGE_PARAMS *)arg,
				sizeof(image_params));

		viulite_set_clippingdata(viulite_base, &image_params);

	}
	break;

	case VIULITE_IOCTL_GET_CLIPPING:
	{
		VIU_IMAGE_PARAMS image_params;

		viulite_get_clippingdata(viulite_base, &image_params);

		/* copy back to user space */
		ret = copy_to_user((VIU_IMAGE_PARAMS *)arg, &image_params,
				sizeof(image_params));
	}
	break;
	}
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_viulite_open
 **********************************************************/
static int fsl_viulite_open(struct inode *inod, struct file *fil)
{
	__TRACE__();
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_viulite_close
 **********************************************************/
static int fsl_viulite_close(struct inode *inode, struct file *filp)
{
	__TRACE__();
	return 0;
}

/**********************************************************
 * STRUCT operations
 **********************************************************/
const struct file_operations viulite_fops = {
	.owner          = THIS_MODULE,
	.open           = fsl_viulite_open,
	.release        = fsl_viulite_close,
	.unlocked_ioctl = fsl_viulite_ioctl,
};

/**********************************************************
 * FUNCTION: fsl_viulite_dev_create
 **********************************************************/
int fsl_viulite_dev_create(struct platform_device *pdev)
{
	int ret;
	viulite_pdev = pdev;

	__TRACE__();

	/* Alloc MAJOR number for the character and
	 * the 1st minor device in dev
	 */
	ret = alloc_chrdev_region(&viulite_devno, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		printk(VIULite_INFO "alloc_chrdev_region failed: %d\n", ret);
		return ret;
	}

	viulite_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (!viulite_class) {
		printk(VIULite_INFO "class_create failed with %d\n", ret);
		return -1;
	}

	if (!(device_create(viulite_class, NULL,
			viulite_devno, NULL, DEVICE_NAME))) {
		printk(VIULite_INFO "device_create failed with %d\n", ret);
		return -1;
	}

	/* setup file operations */
	viulite_cdev = cdev_alloc();
	viulite_cdev->ops = &viulite_fops;

	/* add file operations */
	ret = cdev_add(viulite_cdev, viulite_devno, 1);
	if (ret < 0) {
		printk(VIULite_INFO "cdev_add failed with %d\n", ret);
		return ret;
	}

	return 0;
}


/**********************************************************
 * FUNCTION: fsl_viulite_probe
 **********************************************************/
int fsl_viulite_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	__TRACE__();

	viulite_pdev = pdev;
	ret = 0;

	/* create device and register it in /dev through sysfs */
	fsl_viulite_dev_create(pdev);

	/* map register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		viulite_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could not get memory IO resource\n");
		return -ENODEV;
	}

/*	viulite_data = devm_kzalloc(&pdev->dev,
			sizeof(struct dcu_fb_data), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, dcu_fb_data);
*/
	viulite_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(viulite_reg_base)) {
		viulite_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could not ioremap resource\n");
/*		return PTR_ERR(dcu_fb_data->reg_base);
*/
		goto failed_alloc_base;
	}
/*	viulite_data->reg_base = viulite_reg_base;
*/
	/* allocate memory for base reg */
	viulite_base = kmalloc(sizeof(uint64_t) * VIULITE_DEV_COUNT,
			GFP_KERNEL);
	if (!viulite_base) {
		viulite_init_status = DCU_INIT_ERR_CFG;
		dev_err(&pdev->dev, "could allocate memory for reg_base\n");
		goto failed_alloc_base;
	}

	/* save DCU0 register map to global variable for DCU agnostic layer */
	viulite_base = (uint64_t *)viulite_reg_base;

	/*KK enable clocks for DCU */
	viulite_clk = devm_clk_get(&pdev->dev, "dcu");
	if (IS_ERR(viulite_clk)) {
		viulite_init_status = DCU_INIT_ERR_CFG;
		ret = PTR_ERR(viulite_clk);
		dev_err(&pdev->dev, "could not get clock\n");
		goto failed_getclock;
	}
	clk_prepare_enable(viulite_clk);

	/* get DCU clock in Hz*/
	viulite_clk_val = clk_get_rate(viulite_clk);
/*	viulite_data->clk = viulite_clk;*/
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	/* init has finalized */
	viulite_init_status = DCU_INIT_TRUE;

	__TRACE__();

	return 0;

failed_alloc_base:
failed_getclock:
	return ret;
}

/**********************************************************
 * DCU Linux DTB query related operations
 **********************************************************/
static struct of_device_id fsl_viulite_dt_ids[] = {
	{
		.compatible = "fsl,s32v234-viulite0",
	},
	{
		.compatible = "fsl,s32v234-viulite1",
	},
	{}
};

static int fsl_viulite_runtime_suspend(struct device *dev)
{
/*	struct dcu_fb_data *viufb = dev_get_drvdata(dev);
	clk_disable_unprepare(viufb->clk);
*/
	clk_disable_unprepare(viulite_clk);
	return 0;
}
static int fsl_viulite_runtime_resume(struct device *dev)
{
/*	struct dcu_fb_data *viufb = dev_get_drvdata(dev);
	clk_prepare_enable(viufb->clk);
*/
	clk_prepare_enable(viulite_clk);
	return 0;
}

static const struct dev_pm_ops fsl_viulite_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_viulite_runtime_suspend,
			fsl_viulite_runtime_resume, NULL)
};


static struct platform_driver fsl_viulite_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = fsl_viulite_dt_ids,
		.pm = &fsl_viulite_pm_ops,
	},
	.probe = fsl_viulite_probe,
	.remove = fsl_viulite_remove,
};

module_platform_driver(fsl_viulite_driver);

MODULE_AUTHOR("Tomescu Cristian Corneliu");
MODULE_DESCRIPTION("Freescale fsl-VIULite driver");
MODULE_LICENSE("GPL");
