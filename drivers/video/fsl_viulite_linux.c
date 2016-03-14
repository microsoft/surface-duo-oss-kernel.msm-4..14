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
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/cdev.h>
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
 * VIULite defines
 **********************************************************/

#define DRIVER_NAME	"fsl_viulite"

/* number of VIULite units */
#define VIULITE_DEV_COUNT	2

/**********************************************************
 * Macros for tracing
 **********************************************************/
#define VIULite_INFO	KERN_INFO
/*#define __LOG_TRACE__	1
*/
#if __LOG_TRACE__
	#define __TRACE__() printk(VIULite_INFO "[ fsl-VIU ] %s\n", __func__)
	#define __MSG_TRACE__(string, args...) printk(VIULite_INFO \
		"[ fsl-VIU ]  %s : %d : " string, __func__, __LINE__, ##args)
#else
	#define __TRACE__()
	#define __MSG_TRACE__(string, args...)
	#define __HERE__ printk(VIULite_INFO " HERE %s\n", __func__)
#endif

/**********************************************************
 * GLOBAL configuration registers
 **********************************************************/
struct viulite_data {
	struct cdev	chdev;
	void __iomem	*reg_base;
	unsigned int	irq;
	struct clk	*clk;
	atomic_t	access;
};

const char *device_name[VIULITE_DEV_COUNT] = {
	"fsl_viulite0",
	"fsl_viulite1"
};

static struct class	*viulite_class;
static dev_t		viulite_devn; /*__u32 */
static struct cdev	*viulite_cdev;

struct viulite_data *vdata;
/**********************************************************
 * FUNCTION: fsl_viulite_getviudata
 * To be developed
 **********************************************************/
struct viulite_data *fsl_viulite_getviudata(void)
{
	__TRACE__();
	return vdata;
}


/**********************************************************
 * FUNCTION: fsl_viulite_regs
void fsl_viulite_regs(base_address)
{
	printk(VIULite_INFO "-----------VIULite REGS ----------\n");

	printk(VIULite_INFO "[REG : VIULite_SCR]\t : %02x => %08x\n",
		0x0,
		readl(base_address));
	printk(VIULite_INFO "[REG : VIULite_INTR  ]\t : %02x => %08x\n",
		0x04,
		readl(base_address + INTR_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_DINVSZ]\t : %02x => %08x\n",
		0x08,
		readl(base_address + DINVSZ_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_DINVFL]\t : %02x => %08x\n",
		0x0C,
		readl(base_address + DINVFL_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_DMA_SIZE]\t : %02x => %08x\n",
		0x10,
		readl(base_address + DMA_SIZE_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_DMA_ADDR]\t : %02x => %08x\n",
		0x14,
		readl(base_address + DMA_ADDR_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_DMA_INC]\t : %02x => %08x\n",
		0x18,
		readl(base_address + DMA_INC_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_INVSZ]\t : %02x => %08x\n",
		0x1C,
		readl(base_address + INVSZ_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_ALPHA]\t : %02x => %08x\n",
		0x24,
		readl(base_address + ALPHA_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_ACT_ORG]\t : %02x => %08x\n",
		0x28,
		readl(base_address + ACTORG_OFFSET));
	printk(VIULite_INFO "[REG : VIULite_ACT_SIZE]\t : %02x => %08x\n",
		0x2C,
		readl(base_address + ACTSIZE_OFFSET));
	printk(VIULite_INFO "-------------------------------\n");

}
 **********************************************************/

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
 * to be modified for 2 VIUs
 **********************************************************/
int fsl_viulite_remove(struct platform_device *pdev)
{
	__TRACE__();

	cdev_del(viulite_cdev);

	device_destroy(viulite_class, viulite_devn);
	class_destroy(viulite_class);
	unregister_chrdev_region(viulite_devn, 2);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

/**********************************************************
 * FUNCTIONS: fsl_viulite_ioctl set of functions
 **********************************************************/

void viulite_set_videoinputformat(void __iomem *reg_base,
				  VIU_INPUT_FORMAT *in_format)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_VIDEOIN_MASK;

	reg_value |= ((((uint32_t)(in_format->mode)) << SCR_ITUMODE_OFFSET)
			& SCR_ITUMODE_MASK);
	reg_value |= ((((uint32_t)(in_format->width)) << SCR_INWIDTH_OFFSET)
			& SCR_INWIDTH_MASK);
	reg_value |= ((((uint32_t)(in_format->clocks_per_pixell))
			<< SCR_CPP_OFFSET) & SCR_CPP_MASK);

	writel(reg_value, reg_address);
}

void viulite_get_videoinputformat(void __iomem *reg_base,
				  VIU_INPUT_FORMAT *in_format)
{
	uint32_t reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	in_format->mode = (in_mode_t)((reg_value & SCR_ITUMODE_MASK) >>
			SCR_ITUMODE_OFFSET);
	in_format->width = (in_width_t)((reg_value & SCR_INWIDTH_MASK) >>
			SCR_INWIDTH_OFFSET);
	in_format->clocks_per_pixell = (uint8_t)((reg_value & SCR_CPP_MASK) >>
			SCR_CPP_OFFSET);
}

void viulite_set_datainterface(void __iomem *reg_base,
			       VIU_DATA_INTERFACE *data_interface)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

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

void viulite_get_datainterface(void __iomem *reg_base,
			       VIU_DATA_INTERFACE *data_interface)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	data_interface->pclk_pol  = (uint8_t)((reg_value & SCR_PCLK_POL_MASK)
			>> SCR_PCLK_POL_OFFSET);
	data_interface->vsync_pol = (uint8_t)((reg_value & SCR_VSYNC_POL_MASK)
			>> SCR_VSYNC_POL_OFFSET);
	data_interface->hsync_pol = (uint8_t)((reg_value & SCR_HSYNC_POL_MASK)
			>> SCR_HSYNC_POL_OFFSET);
	data_interface->endianness = (uint8_t)((reg_value & SCR_ENDIANNESS_MASK)
			>> SCR_ENDIANNESS_OFFSET);
}

void viulite_dma_config(void __iomem *reg_base, DMA_CONFIG *dmaconfig)
{
	uint32_t  reg_value;
	void __iomem *reg_address;

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

void viulite_dma_start(void __iomem *reg_base)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	if (0 == (reg_value & SCR_DMA_ACT_MASK)) {
		reg_value |= SCR_DMA_ACT_MASK;
		writel(reg_value, reg_address);
	}
}

void viulite_dma_stop(void __iomem *reg_base)
{

	uint32_t  reg_value;
	void __iomem *reg_address;

	/* Clear pending interrupts */
	reg_address = reg_base + INTR_OFFSET;
	reg_value = readl(reg_address);
	reg_value |= INTR_STATUS_MASK;
	writel(reg_value, reg_address);

	reg_address = reg_base + SCR_OFFSET;
	reg_value = readl(reg_address);

	/* DMA already stopped ? */
	if (0 == (reg_value & SCR_DMA_ACT_MASK))
		return;

	reg_value &= ~SCR_DMA_ACT_MASK;
	writel(reg_value, reg_address);

	/* Soft reset */
	reg_value |= SCR_SWRESET_MASK;
	writel(reg_value, reg_address);

	reg_value &= ~SCR_SWRESET_MASK;
	writel(reg_value, reg_address);
}

void viulite_dma_getstatus(void __iomem *reg_base, VIU_BOOL *dmastatus)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	if (0 == (reg_value & SCR_DMA_ACT_MASK))
		*dmastatus = VIU_OFF;
	else
		*dmastatus = VIU_ON;
}

void viulite_sw_reset(void __iomem *reg_base)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_SWRESET_MASK;
	writel(reg_value, reg_address);

	reg_value |= SCR_SWRESET_MASK;
	writel(reg_value, reg_address);

	reg_value &= ~SCR_SWRESET_MASK;
	writel(reg_value, reg_address);
}

void viulite_enable_ituerror(void __iomem *reg_base, VIU_BOOL itu_errset)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~SCR_ECCEN_MASK;
	if (VIU_ON == itu_errset)
		reg_value |= SCR_ECCEN_MASK;

	writel(reg_value, reg_address);
}

void viulite_get_ituerror(void __iomem *reg_base, VIU_ITU_ERR *itu_errcode)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);

	*itu_errcode = (VIU_ITU_ERR)((reg_value & INTR_ITUERR_MASK)
			>> INTR_ITUERR_OFFSET);
}

void viulite_enable_irqs(void __iomem *reg_base, uint8_t irq_mask)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~INTR_ENABLE_MASK;
	reg_value |= ((uint32_t)irq_mask << INTR_ENABLE_OFFSET);

	writel(reg_value, reg_address);
}

void viulite_get_irqstatus(void __iomem *reg_base, uint8_t *intr_status)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);

	*intr_status = (uint8_t)((reg_value & INTR_STATUS_MASK)
			>> INTR_STATUS_OFFSET);
}

void viulite_reset_irqstatus(void __iomem *reg_base, uint8_t intr_status)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + INTR_OFFSET;

	reg_value = readl(reg_address);
	reg_value &= ~INTR_STATUS_MASK;
	reg_value |= ((uint32_t)intr_status << INTR_STATUS_OFFSET);

	writel(reg_value, reg_address);
}

void viulite_get_syncsignals(void __iomem *reg_base,
			     VIU_SYNC_STATUS *sync_status)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	sync_status->vsync = (uint8_t)((reg_value & SCR_VSYNC_MASK)
			>> SCR_VSYNC_OFFSET);
	sync_status->hsync = (uint8_t)((reg_value & SCR_HSYNC_MASK)
			>> SCR_HSYNC_OFFSET);
}

void viulite_get_fieldnum(void __iomem *reg_base, uint8_t *field_n)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + SCR_OFFSET;

	reg_value = readl(reg_address);

	*field_n = (uint8_t)((reg_value & SCR_FIELDNO_MASK)
			>> SCR_FIELDNO_OFFSET);
}

void viulite_get_framesize(void __iomem *reg_base,
			   VIU_FRAME_SIZE *frame_params)
{
	uint32_t  reg_value;
	void __iomem *reg_address = reg_base + DINVSZ_OFFSET;

	reg_value = readl(reg_address);

	frame_params->nmb_pixells = (uint16_t)(reg_value & VIU_LOW16_MASK);
	frame_params->nmb_lines = (uint16_t)((reg_value & VIU_HIGH16_MASK)
			>> 16);
}

void viulite_set_clippingdata(void __iomem *reg_base,
			      VIU_IMAGE_PARAMS *image_params)
{
	uint32_t  reg_value;
	void __iomem *reg_address;

	reg_address = reg_base + ACTORG_OFFSET;
	reg_value = ((uint32_t)(image_params->x_origin) & VIU_LOW16_MASK) |
			((uint32_t)(image_params->y_origin) << 16);
	writel(reg_value, reg_address);

	reg_address = reg_base + ACTSIZE_OFFSET;
	reg_value = ((uint32_t)(image_params->x_size) & VIU_LOW16_MASK) |
			((uint32_t)(image_params->y_size) << 16);
	writel(reg_value, reg_address);
}

void viulite_get_clippingdata(void __iomem *reg_base,
			      VIU_IMAGE_PARAMS *image_params)
{
	uint32_t  reg_value;
	void __iomem *reg_address;

	reg_address = reg_base + ACTORG_OFFSET;
	reg_value = readl(reg_address);
	image_params->x_origin = (uint16_t)(reg_value & VIU_LOW16_MASK);
	image_params->y_origin = (uint16_t)((reg_value & VIU_HIGH16_MASK)
			>> 16);

	reg_address = reg_base + ACTSIZE_OFFSET;
	reg_value = readl(reg_address);
	image_params->x_size = (uint16_t)(reg_value & VIU_LOW16_MASK);
	image_params->y_size = (uint16_t)((reg_value & VIU_HIGH16_MASK)
			>> 16);
}

/**********************************************************
 * FUNCTION: fsl_viulite_ioctl
 * VIULite Linux IOCTL operations
 **********************************************************/
long fsl_viulite_ioctl(struct file *pfile, unsigned int ioctl_cmd,
		       unsigned long arg)
{
	int ret;
	struct inode *pnode = pfile->f_inode;
	struct viulite_data *viu_data =
		container_of(pnode->i_cdev, struct viulite_data, chdev);
	uint64_t *viulite_baseloc = (uint64_t *)(viu_data->reg_base);
	uint32_t irq = (uint32_t)(viu_data->irq);

	__TRACE__();

	switch (ioctl_cmd) {
	case VIULITE_IOCTL_DMA_CONFIG:
	{
		DMA_CONFIG dma_configuration;

		/* copy data from user space */
		ret = copy_from_user(&dma_configuration, (DMA_CONFIG *)arg,
				sizeof(dma_configuration));

		viulite_dma_config(viulite_baseloc, &dma_configuration);
	}
	break;

	case VIULITE_IOCTL_DMA_START:
	{
		disable_irq(irq);
		viulite_dma_start(viulite_baseloc);
		enable_irq(irq);
	}
	break;

	case VIULITE_IOCTL_DMA_STOP:
	{
		disable_irq(irq);
		viulite_dma_stop(viulite_baseloc);
	}
	break;

	case VIULITE_IOCTL_DMA_GET_STATUS:
	{
		VIU_BOOL dma_st;

		viulite_dma_getstatus(viulite_baseloc, &dma_st);

		/* copy data to user space */
		ret = copy_to_user((VIU_BOOL *)arg, &dma_st, sizeof(dma_st));
	}
	break;

	case VIULITE_IOCTL_SW_RESET:
	{
		viulite_sw_reset(viulite_baseloc);
	}
	break;

	case VIULITE_IOCTL_SET_VIDEOIN_FORMAT:
	{
		VIU_INPUT_FORMAT in_format;

		/* copy data from user space */
		ret = copy_from_user(&in_format, (VIU_INPUT_FORMAT *)arg,
				sizeof(in_format));

		viulite_set_videoinputformat(viulite_baseloc, &in_format);

	}
	break;

	case VIULITE_IOCTL_GET_VIDEOIN_FORMAT:
	{
		VIU_INPUT_FORMAT in_format;

		viulite_get_videoinputformat(viulite_baseloc, &in_format);

		/* copy data to user space */
		ret = copy_to_user((VIU_INPUT_FORMAT *)arg, &in_format,
				sizeof(in_format));
	}
	break;

	case VIULITE_IOCTL_SET_DATA_INTERFACE:
	{
		VIU_DATA_INTERFACE data_interface;

		/* copy data from user space */
		ret = copy_from_user(&data_interface, (VIU_DATA_INTERFACE *)arg,
				sizeof(data_interface));

		viulite_set_datainterface(viulite_baseloc, &data_interface);

	}
	break;

	case VIULITE_IOCTL_GET_DATA_INTERFACE:
	{
		VIU_DATA_INTERFACE data_interface;

		viulite_get_datainterface(viulite_baseloc, &data_interface);

		/* copy data to user space */
		ret = copy_to_user((VIU_DATA_INTERFACE *)arg, &data_interface,
				sizeof(data_interface));
	}
	break;

	case VIULITE_IOCTL_CONFIG_IRQS:
	{
		uint8_t irq_mask;

		/* copy data from user space */
		ret = copy_from_user(&irq_mask, (uint8_t *)arg,
				sizeof(irq_mask));

		viulite_enable_irqs(viulite_baseloc, irq_mask);
	}
	break;

	case VIULITE_IOCTL_EN_ITU_ERRCODE:
	{
		VIU_BOOL itu_errset;

		/* copy data from user space */
		ret = copy_from_user(&itu_errset, (VIU_BOOL *)arg,
				sizeof(itu_errset));

		viulite_enable_ituerror(viulite_baseloc, itu_errset);
	}
	break;

	case VIULITE_IOCTL_RESET_IRQSTATUS:
	{
		uint8_t irq_status;

		/* copy data from user space */
		ret = copy_from_user(&irq_status, (uint8_t *)arg,
				sizeof(irq_status));

		viulite_reset_irqstatus(viulite_baseloc, irq_status);
	}
	break;

	case VIULITE_IOCTL_GET_IRQSTATUS:
	{
		uint8_t irq_status;

		viulite_get_irqstatus(viulite_baseloc, &irq_status);

		/* copy data to user space */
		ret = copy_to_user((uint8_t *)arg, &irq_status,
				sizeof(irq_status));
	}
	break;

	case VIULITE_IOCTL_GET_ITU_ERRCODE:
	{
		VIU_ITU_ERR itu_err;

		viulite_get_ituerror(viulite_baseloc, &itu_err);

		/* copy data to user space */
		ret = copy_to_user((VIU_ITU_ERR *)arg, &itu_err,
				sizeof(itu_err));
	}
	break;

	case VIULITE_IOCTL_GET_SYNC:
	{
		VIU_SYNC_STATUS sync_st;

		viulite_get_syncsignals(viulite_baseloc, &sync_st);

		/* copy data to user space */
		ret = copy_to_user((VIU_SYNC_STATUS *)arg, &sync_st,
				sizeof(sync_st));
	}
	break;

	case VIULITE_IOCTL_GET_FIELDNUM:
	{
		uint8_t field_num;

		viulite_get_fieldnum(viulite_baseloc, &field_num);

		/* copy data to user space */
		ret = copy_to_user((uint8_t *)arg, &field_num,
				sizeof(field_num));
	}
	break;

	case VIULITE_IOCTL_GET_FRAME_SIZE:
	{
		VIU_FRAME_SIZE frame_size;

		viulite_get_framesize(viulite_baseloc, &frame_size);

		/* copy data to user space */
		ret = copy_to_user((VIU_FRAME_SIZE *)arg, &frame_size,
				sizeof(frame_size));
	}
	break;

	case VIULITE_IOCTL_SET_CLIPPING:
	{
		VIU_IMAGE_PARAMS image_params;

		/* copy data from user space */
		ret = copy_from_user(&image_params, (VIU_IMAGE_PARAMS *)arg,
				sizeof(image_params));

		viulite_set_clippingdata(viulite_baseloc, &image_params);

	}
	break;

	case VIULITE_IOCTL_GET_CLIPPING:
	{
		VIU_IMAGE_PARAMS image_params;

		viulite_get_clippingdata(viulite_baseloc, &image_params);

		/* copy data to user space */
		ret = copy_to_user((VIU_IMAGE_PARAMS *)arg, &image_params,
				sizeof(image_params));
	}
	break;

	default:
	break;
	}

	return 0;
}

/**********************************************************
 * FUNCTION: fsl_viulite_open
 **********************************************************/
int fsl_viulite_open(struct inode *inod, struct file *pfile)
{
	struct viulite_data *data;

	__TRACE__();

	data = container_of(inod->i_cdev, struct viulite_data, chdev);
	pfile->private_data = data;

/*	if (atomic_cmpxchg(&data->access, 1, 0) != 1)
		return -EBUSY;
*/
	return 0;
}

/**********************************************************
 * FUNCTION: fsl_viulite_close
 **********************************************************/
static int fsl_viulite_close(struct inode *inod, struct file *pfile)
{
	struct viulite_data *viu_data;
	uint32_t irq;

	__TRACE__();

	viu_data = container_of(inod->i_cdev, struct viulite_data, chdev);
	irq = (uint32_t)(viu_data->irq);
	disable_irq(irq);

/*	atomic_inc(&viu_data->access);
*/	return 0;
}

/**********************************************************
 * FUNCTION: viulite_intr
 **********************************************************/
irqreturn_t viulite_intr(int irq, void *dev_id)
{
	struct viulite_data *viu_data = (struct viulite_data *)dev_id;
	uintptr_t viulite_regbase = (uintptr_t)(viu_data->reg_base);
/*	void __iomem *viulite_regbase = (void __iomem *)(viu_data->reg_base);
*/
	uint32_t  int_status, reg_value;
	void __iomem *reg_address;
	void __iomem *int_address;

	int_address = (void __iomem *)(viulite_regbase + INTR_OFFSET);
/*	int_address = viulite_regbase + INTR_OFFSET;
*/	int_status = readl(int_address);

	if (int_status & (INTR_VSYNC_MASK | INTR_VSYNC_EN)) {
		/* start a new frame transfer */
		reg_address = (void __iomem *)(viulite_regbase + SCR_OFFSET);
/*		reg_address = viulite_regbase + SCR_OFFSET;
*/
		reg_value = readl(reg_address);
		reg_value |= SCR_DMA_ACT_MASK;
		writel(reg_value, reg_address);

		int_status |= INTR_VSYNC_MASK;
	}

	if (int_status & (INTR_DMA_END_MASK | INTR_DMA_END_EN))
		int_status |= INTR_DMA_END_MASK;

	if (int_status & (INTR_ERR_MASK | INTR_ERR_EN))
		int_status |= INTR_ERR_MASK;

	if (int_status & (INTR_HSYNC_MASK | INTR_HSYNC_EN))
		int_status |= INTR_HSYNC_MASK;

	if (int_status & (INTR_LINE_END_MASK | INTR_LINE_END_EN))
		int_status |= INTR_LINE_END_MASK;

	if (int_status & (INTR_FRAME_END_MASK | INTR_FRAME_END_EN))
		int_status |= INTR_FRAME_END_MASK;

	if (int_status & (INTR_FIELD_MASK | INTR_FIELD_EN))
		int_status |= INTR_FIELD_MASK;

	if (int_status & (INTR_VSTART_MASK | INTR_VSTART_EN))
		int_status |= INTR_VSTART_MASK;

	/* clear all pending irqs */
	writel(int_status, int_address);

	return IRQ_HANDLED;
}

/**********************************************************
 * STRUCT operations
 **********************************************************/
const struct file_operations viulite_fops = {
	.owner			= THIS_MODULE,
	.open			= fsl_viulite_open,
	.release		= fsl_viulite_close,
	.unlocked_ioctl	= fsl_viulite_ioctl,
};

/**********************************************************
 * FUNCTION: fsl_viulite_dev_create
 **********************************************************/
int fsl_viulite_dev_create(struct platform_device *pdev)
{
	int ret = 0;
	uint32_t index;

	__TRACE__();

	if (NULL == viulite_class) {
		/* Alloc MAJOR number for the character and
		* the 1st minor device in dev
		*/
		ret = alloc_chrdev_region(&viulite_devn, 0,
					VIULITE_DEV_COUNT, DRIVER_NAME);
		if (ret < 0) {
			printk(VIULite_INFO "alloc_region failed: %d\n", ret);
			return ret;
		}

		viulite_class = class_create(THIS_MODULE, DRIVER_NAME);
		if (!viulite_class) {
			printk(VIULite_INFO "class_create failed %d\n", ret);
			return -1;
		}
	}

	index = MINOR(viulite_devn);

	/* Device is created and registered in sysfs */
	if (!(device_create(viulite_class, NULL,
			viulite_devn, NULL, device_name[index]))) {
		printk(VIULite_INFO "device_create 0 failed %d\n", ret);
		return -1;
	}

	return 0;
}


/**********************************************************
 * FUNCTION: fsl_viulite_probe
 **********************************************************/
int fsl_viulite_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct viulite_data *viudata;
	void __iomem *viulite_reg_base;
	struct clk *viulite_clk;
	uint32_t viulite_irq;

	uint32_t index;
	int ret = 0;

	__TRACE__();

	/* create device and register it in /dev through sysfs */
	fsl_viulite_dev_create(pdev);
	index = MINOR(viulite_devn);

	/* map register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "could not get memory IO resource\n");
		return -ENODEV;
	}

	viudata = devm_kzalloc(&pdev->dev,
			sizeof(struct viulite_data), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, viudata);

	/* setup file operations */
	cdev_init(&(viudata->chdev), &viulite_fops);
	ret = cdev_add(&(viudata->chdev), viulite_devn, 1);
	if (ret < 0) {
		printk(VIULite_INFO "cdev_add failed with %d\n", ret);
		return ret;
	}

	viulite_reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(viulite_reg_base)) {
		dev_err(&pdev->dev, "could not ioremap resource\n");
		return PTR_ERR(viudata->reg_base);

		goto failed_alloc_base;
	}
	viudata->reg_base = viulite_reg_base;

	/* enable clocks for VIU */
	viulite_clk = devm_clk_get(&pdev->dev, device_name[index]);
	if (IS_ERR(viulite_clk)) {
		ret = PTR_ERR(viulite_clk);
		dev_err(&pdev->dev, "could not get clock\n");
		goto failed_getclock;
	}
	clk_prepare_enable(viulite_clk);

	viudata->clk = viulite_clk;
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	viulite_irq = platform_get_irq(pdev, 0);
	if (viulite_irq == 0) {
		dev_err(&pdev->dev, "Error while mapping the irq\n");
		return -EINVAL;
	}
	viudata->irq = viulite_irq;

	/* install interrupt handler */
	if (request_irq(viudata->irq, viulite_intr, 0,
			device_name[index], (void *)viudata)) {
		dev_err(&pdev->dev, "Request VIULite IRQ failed.\n");
		ret = -ENODEV;
		goto failed_reqirq;
	}

	viulite_devn++;

	__TRACE__();

	return 0;

failed_alloc_base:
failed_getclock:
failed_reqirq:
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
	struct viulite_data *viu_data = dev_get_drvdata(dev);
	clk_disable_unprepare(viu_data->clk);

	return 0;
}

static int fsl_viulite_runtime_resume(struct device *dev)
{
	struct viulite_data *viu_data = dev_get_drvdata(dev);
	clk_prepare_enable(viu_data->clk);

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
