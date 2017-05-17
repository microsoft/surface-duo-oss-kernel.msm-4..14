/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/usb.h>
#include "rsi_usb.h"
#include "rsi_hal.h"

/* Default operating mode is Wi-Fi alone */
#ifdef CONFIG_CARACALLA_BOARD
#if defined (CONFIG_VEN_RSI_COEX) || defined(CONFIG_VEN_RSI_BT_ALONE)
u16 dev_oper_mode = DEV_OPMODE_STA_BT_DUAL;
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
#else
u16 dev_oper_mode = DEV_OPMODE_WIFI_ALONE;
#endif
module_param(dev_oper_mode, ushort, S_IRUGO);
MODULE_PARM_DESC(dev_oper_mode,
		 "1 -	Wi-Fi Alone \
		  4 -	BT Alone \
		  8 -	BT LE Alone \
		  5 -	Wi-Fi STA + BT classic \
		  9 -	Wi-Fi STA + BT LE \
		  13 -	Wi-Fi STA + BT classic + BT LE \
		  6 -	AP + BT classic \
		  14 -	AP + BT classic + BT LE");

static struct rsi_host_intf_ops usb_host_intf_ops = {
	.write_pkt		= rsi_usb_host_intf_write_pkt,
	.master_reg_read	= rsi_usb_master_reg_read,
	.master_reg_write	= rsi_usb_master_reg_write,
	.read_reg_multiple	= rsi_usb_read_register_multiple,
	.write_reg_multiple	= rsi_usb_write_register_multiple,
	.load_data_master_write	= rsi_usb_load_data_master_write,
	.check_hw_queue_status	= rsi_usb_check_queue_status,
};

/**
 * rsi_usb_card_write() - This function writes data to the USB Card.
 * @adapter: Pointer to the adapter structure.
 * @buf: Pointer to the buffer from where the data has to be taken.
 * @len: Length to be written.
 * @endpoint: Type of endpoint.
 *
 * Return: status: 0 on success, a negative error code on failure.
 */
static int rsi_usb_card_write(struct rsi_hw *adapter,
			      u8 *buf,
			      u16 len,
			      u32 endpoint)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	int status = 0;
	u8 *seg = dev->tx_buffer;
	int transfer = 0;
	int ep = dev->bulkout_endpoint_addr[endpoint - 1];

	memset(seg, 0, len + 128);
	memcpy(seg + 128, buf, len);
	len += 128;
	transfer = len;

	status = usb_bulk_msg(dev->usbdev,
			      usb_sndbulkpipe(dev->usbdev, ep),
			      (void *)seg,
			      (int)len,
			      &transfer,
			      HZ * 5);
	if (status < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"Card write failed with error code :%d\n", status);
		dev->write_fail = 1;
		goto fail;
	}
	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sent Message successfully\n", __func__);

fail:
	return status;
}

/**
 * rsi_write_multiple() - This function writes multiple bytes of information
 *			  to the USB card.
 * @adapter: Pointer to the adapter structure.
 * @addr: Address of the register.
 * @data: Pointer to the data that has to be written.
 * @count: Number of multiple bytes to be written.
 *
 * Return: 0 on success, a negative error code on failure.
 */
static int rsi_write_multiple(struct rsi_hw *adapter,
			      u32 addr,
			      u8 *data,
			      u32 count)
{
	struct rsi_91x_usbdev *dev =
		(struct rsi_91x_usbdev *)adapter->rsi_dev;

	if (!adapter || addr == 0) {
		ven_rsi_dbg(INFO_ZONE,
			"%s: Unable to write to card\n", __func__);
		return -1;
	}

	if (dev->write_fail)
		return -1;

	return rsi_usb_card_write(adapter, data, count, addr);
}

#define RSI_USB_REQ_OUT	(USB_TYPE_VENDOR | USB_DIR_OUT | USB_RECIP_DEVICE)
#define RSI_USB_REQ_IN	(USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_DEVICE)

/* rsi_usb_reg_read() - This function reads data from given register address.
 * @usbdev: Pointer to the usb_device structure.
 * @reg: Address of the register to be read.
 * @value: Value to be read.
 * @len: length of data to be read.
 *
 * Return: status: 0 on success, a negative error code on failure.
 */
static int rsi_usb_reg_read(struct usb_device *usbdev,
			    u32 reg,
			    u32 *value,
			    u16 len)
{
	u8 buf[4];
	int status = 0;
	u16 reg_value;
	u16 index;

	len = 2;
	reg_value = cpu_to_le16(((u16 *)&reg)[1] & 0xffff);
	index = cpu_to_le16(((u16 *)&reg)[0] & 0xffff);
	status = usb_control_msg(usbdev,
				 usb_rcvctrlpipe(usbdev, 0),
				 USB_VENDOR_REGISTER_READ,
				 RSI_USB_REQ_IN,
				 reg_value,
				 index,
				 (void *)buf,
				 len,
				 USB_CTRL_GET_TIMEOUT);

	*value = (buf[0] | (buf[1] << 8));
	if (status < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Reg read failed with error code :%d\n",
			__func__, status);
	}

	return status;
}

/**
 * rsi_usb_reg_write() - This function writes the given data into the given
 *			 register address.
 * @usbdev: Pointer to the usb_device structure.
 * @reg: Address of the register.
 * @value: Value to write.
 * @len: Length of data to be written.
 *
 * Return: status: 0 on success, a negative error code on failure.
 */
static int rsi_usb_reg_write(struct usb_device *usbdev,
			     unsigned long reg,
			     unsigned long value,
			     u16 len)
{
	u8 usb_reg_buf[4];
	int status = 0;
	u16 reg_value, index;

	usb_reg_buf[0] = (value & 0x00ff);
	usb_reg_buf[1] = (value & 0xff00) >> 8;
	usb_reg_buf[2] = 0x0;
	usb_reg_buf[3] = 0x0;

	reg_value = ((u16 *)&reg)[1] & 0xffff;
	index = ((u16 *)&reg)[0] & 0xffff;
	status = usb_control_msg(usbdev,
				 usb_sndctrlpipe(usbdev, 0),
				 USB_VENDOR_REGISTER_WRITE,
				 RSI_USB_REQ_OUT,
				 reg_value,
				 index,
				 (void *)usb_reg_buf,
				 len,
				 USB_CTRL_SET_TIMEOUT);
	if (status < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: Reg write failed with error code :%d\n",
			__func__, status);
	}

	return status;
}

/**
 * rsi_usb_read_register_multiple() - This function reads multiple
 *					bytes of data from the address.
 * @adapter:	Pointer to the adapter structure.
 * @addr:	Address of the register.
 * @data:	Read data.
 * @len:	Number of bytes to read.
 *
 * Return: status: 0 on success, a negative error code on failure.
 */
int rsi_usb_read_register_multiple(struct rsi_hw *adapter,
				   u32 addr,
				   u8 *data,
				   u16 count)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	u8 *buf;
	u16 transfer;
	int status = 0;
	u16 reg_val, index;

	if (addr == 0)
		return -EINVAL;

	buf = kzalloc(4096, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	reg_val = ((u16 *)&addr)[1] & 0xffff;
	index = ((u16 *)&addr)[0] & 0xffff;
	while (count) {
		transfer = min_t(u16, count, 4096);
		status = usb_control_msg(dev->usbdev,
					 usb_rcvctrlpipe(dev->usbdev, 0),
					 USB_VENDOR_REGISTER_READ,
					 RSI_USB_REQ_IN,
					 reg_val,
					 index,
					 (void *)buf,
					 transfer,
					 USB_CTRL_GET_TIMEOUT);
		if (status < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"Reg read failed with error code :%d\n",
				 status);
			kfree(buf);
			return status;

		} else {
			memcpy(data, buf, transfer);
			count -= transfer;
			data += transfer;
			addr += transfer;
		}
	}
	kfree(buf);
	return status;
}

/**
 * rsi_usb_write_register_multiple() - This function writes multiple bytes of
 *				       information to the given address.
 * @adapter:	Pointer to the adapter structure.
 * @addr:	Address of the register.
 * @data:	Pointer to the data that has to be written.
 * @count:	Number of multiple bytes to be written on to the registers.
 *
 * Return: status: 0 on success, a negative error code on failure.
 */
int rsi_usb_write_register_multiple(struct rsi_hw *adapter,
				    u32 addr,
				    u8 *data,
				    u16 count)
{
	struct rsi_91x_usbdev *dev =
		(struct rsi_91x_usbdev *)adapter->rsi_dev;
	u8 *buf;
	u16 transfer;
	int status = 0;
	u16 reg_val, index;

	buf = kzalloc(4096, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	reg_val = ((u16 *)&addr)[1] & 0xffff;
	index = ((u16 *)&addr)[0] & 0xffff;
	while (count) {
		transfer = min_t(u16, count, 4096);
		memcpy(buf, data, transfer);
		status = usb_control_msg(dev->usbdev,
					 usb_sndctrlpipe(dev->usbdev, 0),
					 USB_VENDOR_REGISTER_WRITE,
					 RSI_USB_REQ_OUT,
					 reg_val,
					 index,
					 (void *)buf,
					 transfer,
					 USB_CTRL_SET_TIMEOUT);
		if (status < 0) {
			ven_rsi_dbg(ERR_ZONE,
				"Reg write failed with error code :%d\n",
				status);
			kfree(buf);
			return status;
		}
		count -= transfer;
		data += transfer;
		addr += transfer;
	}

	kfree(buf);
	return status;
}

/**
 * rsi_rx_done_handler() - This function is called when a packet is received
 *			   from USB stack. This is callback to receive done.
 * @urb: Received URB.
 *
 * Return: None.
 */
static void rsi_rx_done_handler(struct urb *urb)
{
	struct rx_usb_ctrl_block *rx_cb = urb->context;
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)rx_cb->data;

	if (urb->status)
		return;

	if (urb->actual_length <= 0) {
		ven_rsi_dbg(INFO_ZONE, "%s: Zero length packet\n", __func__);
		return;
	}
	rx_cb->pend = 1;

	rsi_set_event(&dev->rx_thread.event);
}

/**
 * rsi_rx_urb_submit() - This function submits the given URB to the USB stack.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: 0 on success, a negative error code on failure.
 */
static int rsi_rx_urb_submit(struct rsi_hw *adapter, u8 ep_num)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	struct rx_usb_ctrl_block *rx_cb = &dev->rx_cb[ep_num - 1];
	struct urb *urb = rx_cb->rx_urb;
	int status;

	usb_fill_bulk_urb(urb,
			dev->usbdev,
			usb_rcvbulkpipe(dev->usbdev,
					dev->bulkin_endpoint_addr[ep_num - 1]),
			urb->transfer_buffer,
			3000,
			rsi_rx_done_handler,
			rx_cb);

	status = usb_submit_urb(urb, GFP_KERNEL);
	if (status)
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in urb submission\n", __func__);

	return status;
}

/**
 *rsi_usb_host_intf_write_pkt() - This function writes the packet to the
 *				   USB card.
 * @adapter: Pointer to the adapter structure.
 * @pkt: Pointer to the data to be written on to the card.
 * @len: Length of the data to be written on to the card.
 *
 * Return: 0 on success, a negative error code on failure.
 */
int rsi_usb_host_intf_write_pkt(struct rsi_hw *adapter,
				u8 *pkt,
				u32 len)
{
	u32 queueno = ((pkt[1] >> 4) & 0x7);
	u8 endpoint;

	ven_rsi_dbg(DATA_TX_ZONE, "%s: queueno=%d\n", __func__, queueno);
	endpoint = ((queueno == RSI_WIFI_MGMT_Q || queueno == RSI_COEX_Q ||
		     queueno == RSI_WIFI_DATA_Q) ?
		    MGMT_EP : DATA_EP);

	return rsi_write_multiple(adapter,
				  endpoint,
				  pkt,
				  len);
}

int rsi_usb_master_reg_read(struct rsi_hw *adapter,
			    u32 reg,
			    u32 *value,
			    u16 len)
{
	struct usb_device *usbdev =
		((struct rsi_91x_usbdev *)adapter->rsi_dev)->usbdev;

	return rsi_usb_reg_read(usbdev, reg, value, len);
}

int rsi_usb_master_reg_write(struct rsi_hw *adapter,
			     unsigned long reg,
			     unsigned long value,
			     u16 len)
{
	struct usb_device *usbdev =
		((struct rsi_91x_usbdev *)adapter->rsi_dev)->usbdev;

	return rsi_usb_reg_write(usbdev, reg, value, len);
}

int rsi_usb_load_data_master_write(struct rsi_hw *adapter,
				   u32 base_address,
				   u32 instructions_sz,
				   u16 block_size,
				   u8 *ta_firmware)
{
	u16 num_blocks;
	u32 cur_indx, ii;
	u8 temp_buf[256];

	num_blocks = instructions_sz / block_size;
	ven_rsi_dbg(INFO_ZONE, "num_blocks: %d\n", num_blocks);

	for (cur_indx = 0, ii = 0;
	     ii < num_blocks;
	     ii++, cur_indx += block_size) {
		memset(temp_buf, 0, block_size);
		memcpy(temp_buf, ta_firmware + cur_indx, block_size);
		if ((rsi_usb_write_register_multiple(adapter,
						     base_address,
						     (u8 *)(temp_buf),
						     block_size)) < 0)
			return -EIO;

		ven_rsi_dbg(INFO_ZONE, "%s: loading block: %d\n", __func__, ii);
		base_address += block_size;
	}

	if (instructions_sz % block_size) {
		memset(temp_buf, 0, block_size);
		memcpy(temp_buf, ta_firmware + cur_indx,
		       instructions_sz % block_size);
		if ((rsi_usb_write_register_multiple(adapter,
					     base_address,
					     (u8 *)temp_buf,
					     instructions_sz % block_size)) < 0)
			return -EIO;
		ven_rsi_dbg(INFO_ZONE,
			"Written Last Block in Address 0x%x Successfully\n",
			cur_indx);
	}
	return 0;
}

int rsi_usb_check_queue_status(struct rsi_hw *adapter, u8 q_num)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	int status;
	u32 buf_status = 0;

	return QUEUE_NOT_FULL;

	if (adapter->priv->fsm_state != FSM_MAC_INIT_DONE)
		return QUEUE_NOT_FULL;

	status = rsi_usb_reg_read(dev->usbdev, adapter->usb_buffer_status_reg,
				  &buf_status, 2);
	if (status < 0)
		return status;

	printk("buffer_status = %x\n", buf_status);
	if (buf_status & (BIT(PKT_MGMT_BUFF_FULL))) {
		if (!dev->rx_info.mgmt_buffer_full)
			dev->rx_info.mgmt_buf_full_counter++;
		dev->rx_info.mgmt_buffer_full = true;
	} else {
		dev->rx_info.mgmt_buffer_full = false;
	}

	if (buf_status & (BIT(PKT_BUFF_FULL))) {
		if (!dev->rx_info.buffer_full)
			dev->rx_info.buf_full_counter++;
		dev->rx_info.buffer_full = true;
	} else {
		dev->rx_info.buffer_full = false;
	}

	if (buf_status & (BIT(PKT_BUFF_SEMI_FULL))) {
		if (!dev->rx_info.semi_buffer_full)
			dev->rx_info.buf_semi_full_counter++;
		dev->rx_info.semi_buffer_full = true;
	} else {
		dev->rx_info.semi_buffer_full = false;
	}

	if ((q_num == MGMT_SOFT_Q) && (dev->rx_info.mgmt_buffer_full))
		return QUEUE_FULL;

	if ((q_num < MGMT_SOFT_Q) && (dev->rx_info.buffer_full))
		return QUEUE_FULL;

	return QUEUE_NOT_FULL;
}

/**
 * rsi_deinit_usb_interface() - This function deinitializes the usb interface.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: None.
 */
static void rsi_deinit_usb_interface(struct rsi_hw *adapter)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;

	ven_rsi_dbg(INFO_ZONE, "Deinitializing USB interface...\n");

	rsi_kill_thread(&dev->rx_thread);
	kfree(dev->rx_cb[0].rx_buffer);
	usb_free_urb(dev->rx_cb[0].rx_urb);
#if defined (CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	kfree(dev->rx_cb[1].rx_buffer);
	usb_free_urb(dev->rx_cb[1].rx_urb);
#endif
	kfree(dev->saved_tx_buffer);
}

/**
 * rsi_find_bulk_in_and_out_endpoints() - This function initializes the bulk
 *					  endpoints to the device.
 * @interface: Pointer to the USB interface structure.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: ret_val: 0 on success, -ENOMEM on failure.
 */
static int rsi_find_bulk_in_and_out_endpoints(struct usb_interface *interface,
					      struct rsi_hw *adapter)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	__le16 buffer_size;
	int ii, bin_found = 0, bout_found = 0;

	iface_desc = &interface->altsetting[0];

	for (ii = 0; ii < iface_desc->desc.bNumEndpoints; ++ii) {
		endpoint = &(iface_desc->endpoint[ii].desc);

		if ((!dev->bulkin_endpoint_addr[bin_found]) &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		     USB_ENDPOINT_XFER_BULK)) {
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulkin_size[bin_found] = buffer_size;
			dev->bulkin_endpoint_addr[bin_found] =
					endpoint->bEndpointAddress;
			printk("bulkin addr[%d] = %d\n", bin_found, dev->bulkin_endpoint_addr[bin_found]);
			bin_found++;
		}

		if (!dev->bulkout_endpoint_addr[bout_found] &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		     USB_ENDPOINT_XFER_BULK)) {
			printk("%s:%d\n", __func__, __LINE__);
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulkout_endpoint_addr[bout_found] =
					endpoint->bEndpointAddress;
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulkout_size[bout_found] = buffer_size;
			printk("bulkout addr[%d] = %d\n", bout_found, dev->bulkout_endpoint_addr[bout_found]);
			bout_found++;
		}

		if ((bin_found >= MAX_BULK_EP) || (bout_found >= MAX_BULK_EP))
			break;
	}

	if (!(dev->bulkin_endpoint_addr[0]) &&
	    (dev->bulkout_endpoint_addr[0]))
		return -EINVAL;

	return 0;
}

static int rsi_usb_init_rx(struct rsi_hw *adapter)
{
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	struct rx_usb_ctrl_block *rx_cb;
	u8 dword_align_bytes = 0, idx;

	for (idx = 0; idx < MAX_RX_URBS; idx++) {
		rx_cb = &dev->rx_cb[idx];

		rx_cb->rx_buffer = kzalloc(2000 * 4, GFP_KERNEL | GFP_DMA);
		if (!rx_cb->rx_buffer)
			return -ENOMEM
				;
		rx_cb->orig_rx_buffer = rx_cb->rx_buffer;
		dword_align_bytes = (unsigned long)rx_cb->rx_buffer & 0x3f;
		if (dword_align_bytes)
			rx_cb->rx_buffer = rx_cb->rx_buffer +
					   (64 - dword_align_bytes);

		rx_cb->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!rx_cb->rx_urb) {
			ven_rsi_dbg(ERR_ZONE, "Failed alloc rx urb[%d]\n", idx);
			goto err;
		}
		rx_cb->rx_urb->transfer_buffer = rx_cb->rx_buffer;
		rx_cb->ep_num = idx + 1;
		rx_cb->data = (void *)dev;
	}
	return 0;

err:
	kfree(rx_cb[0].rx_buffer);
	kfree(rx_cb[0].rx_urb);
	kfree(rx_cb[1].rx_buffer);
	kfree(rx_cb[1].rx_urb);
	return -1;
}

/**
 * rsi_init_usb_interface() - This function initializes the usb interface.
 * @adapter: Pointer to the adapter structure.
 * @pfunction: Pointer to USB interface structure.
 *
 * Return: 0 on success, a negative error code on failure.
 */
static int rsi_init_usb_interface(struct rsi_hw *adapter,
				  struct usb_interface *pfunction)
{
	struct rsi_91x_usbdev *rsi_dev;
	struct rsi_common *common = adapter->priv;
	int status = 0;
	u8 dword_align_bytes = 0;

	rsi_dev = kzalloc(sizeof(*rsi_dev), GFP_KERNEL);
	if (!rsi_dev)
		return -ENOMEM;

	adapter->rsi_dev = rsi_dev;
	rsi_dev->usbdev = interface_to_usbdev(pfunction);

	if (rsi_find_bulk_in_and_out_endpoints(pfunction, adapter))
		return -EINVAL;

	adapter->device = &pfunction->dev;
	usb_set_intfdata(pfunction, adapter);

	rsi_dev->tx_buffer = kmalloc(2048, GFP_KERNEL);
	if (!rsi_dev->tx_buffer) {
		status = -ENOMEM;
		goto fail_1;
	}
	rsi_dev->saved_tx_buffer = rsi_dev->tx_buffer;
	dword_align_bytes = (unsigned long)rsi_dev->tx_buffer & 0x3f;
	if (dword_align_bytes)
		rsi_dev->tx_buffer = rsi_dev->tx_buffer +
				     (64 - dword_align_bytes);

	/* Initialize RX handle */
	if (rsi_usb_init_rx(adapter)) {
		ven_rsi_dbg(ERR_ZONE, "Failed to init RX handle\n");
		goto fail_1;
	}

	rsi_dev->tx_blk_size = 252;
	adapter->tx_blk_size = rsi_dev->tx_blk_size;

	/* Initializing function callbacks */
	adapter->rx_urb_submit = rsi_rx_urb_submit;
	adapter->check_hw_queue_status = rsi_usb_check_queue_status;
	adapter->determine_event_timeout = rsi_usb_event_timeout;
	adapter->host_intf_ops = &usb_host_intf_ops;
	rsi_dev->priv = (void *)adapter;

	rsi_init_event(&rsi_dev->rx_thread.event);
	status = rsi_create_kthread(common, &rsi_dev->rx_thread,
				    rsi_usb_rx_thread, "RX-Thread");
	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init rx thrd\n", __func__);
		goto fail_2;
	}

#ifdef CONFIG_VEN_RSI_DEBUGFS
	/* In USB, one less than the MAX_DEBUGFS_ENTRIES entries
	 * is required */
	adapter->num_debugfs_entries = MAX_DEBUGFS_ENTRIES - 1;
#endif

	ven_rsi_dbg(INIT_ZONE, "%s: Enabled the interface\n", __func__);
	return 0;

fail_2:
	kfree(rsi_dev->saved_tx_buffer);
	rsi_kill_thread(&rsi_dev->rx_thread);
fail_1:
	return status;
}

static int rsi_usb_gspi_init(struct rsi_hw *adapter)
{
	u32 gspi_ctrl_reg0_val;

	/**
	 * Programming gspi frequency = soc_frequency / 2
	 * Warning : ULP seemed to be not working
	 * well at high frequencies. Modify accordingly
	 */
	gspi_ctrl_reg0_val = 0x4;
	gspi_ctrl_reg0_val |= 0x10;
	gspi_ctrl_reg0_val |= 0x40;
	gspi_ctrl_reg0_val |= 0x100;
	gspi_ctrl_reg0_val |= 0x000;
	gspi_ctrl_reg0_val |= 0x000;

	/* Initializing GSPI for ULP read/writes */
	return rsi_usb_master_reg_write(adapter, GSPI_CTRL_REG0,
			gspi_ctrl_reg0_val, 2);
}

static int usb_ulp_read_write(struct rsi_hw *adapter,
			      u16 addr,
			      u16 *data,
			      u16 len_in_bits)
{
	if ((rsi_usb_master_reg_write(adapter,
				      GSPI_DATA_REG1,
				      ((addr << 6) | (data[1] & 0x3f)),
				      2) < 0))
		goto fail;

	if ((rsi_usb_master_reg_write(adapter,
				      GSPI_DATA_REG0,
				      (*(u16 *)&data[0]),
				      2)) < 0)
		goto fail;

	if ((rsi_usb_gspi_init(adapter)) < 0)
		goto fail;

	if ((rsi_usb_master_reg_write(adapter, GSPI_CTRL_REG1,
				      ((len_in_bits - 1) | GSPI_TRIG),
				      2)) < 0)
		goto fail;

	msleep(10);

	return 0;

fail:
	return -1;
}

static int rsi_reset_card(struct rsi_hw *adapter)
{
	u16 temp[4] = {0};

	ven_rsi_dbg(INFO_ZONE, "Resetting Card...\n");

#define TA_HOLD_REG 0x22000844
	rsi_usb_master_reg_write(adapter, TA_HOLD_REG, 0xE, 4);
	msleep(100);
	*(u32 *)temp = 2;
	if ((usb_ulp_read_write(adapter,
				WATCH_DOG_TIMER_1,
				&temp[0], 32)) < 0) {
		goto fail;
	}

	*(u32 *)temp = 0;
	if ((usb_ulp_read_write(adapter,
				WATCH_DOG_TIMER_2,
				temp, 32)) < 0) {
		goto fail;
	}

	*(u32 *)temp = 50;
	if ((usb_ulp_read_write(adapter,
				WATCH_DOG_DELAY_TIMER_1,
				temp, 32)) < 0) {
		goto fail;
	}

	*(u32 *)temp = 0;
	if ((usb_ulp_read_write(adapter,
				WATCH_DOG_DELAY_TIMER_2,
				temp, 32)) < 0) {
		goto fail;
	}

	*(u32 *)temp = ((0xaa000) | RESTART_WDT | BYPASS_ULP_ON_WDT);
	if ((usb_ulp_read_write(adapter,
				WATCH_DOG_TIMER_ENABLE,
				temp, 32)) < 0) {
		goto fail;
	}
	ven_rsi_dbg(INFO_ZONE, "Card Reset Done\n");
	return 0;

fail:
	ven_rsi_dbg(ERR_ZONE, "Reset card Failed\n");
	return -1;
}

/**
 * rsi_probe() - This function is called by kernel when the driver provided
 *		 Vendor and device IDs are matched. All the initialization
 *		 work is done here.
 * @pfunction: Pointer to the USB interface structure.
 * @id: Pointer to the usb_device_id structure.
 *
 * Return: 0 on success, a negative error code on failure.
 */
static int rsi_probe(struct usb_interface *pfunction,
		     const struct usb_device_id *id)
{
	struct rsi_hw *adapter;
	struct rsi_91x_usbdev *dev;
	u32 fw_status = 0;
	int status = 0;

	ven_rsi_dbg(INIT_ZONE, "%s: Init function called\n", __func__);

	adapter = ven_rsi_91x_init();
	if (!adapter) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to init os intf ops\n",
			__func__);
		return -ENOMEM;
	}
	adapter->rsi_host_intf = RSI_HOST_INTF_USB;
	adapter->priv->oper_mode = dev_oper_mode;

	status = rsi_init_usb_interface(adapter, pfunction);
	if (status) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to init usb interface\n",
			__func__);
		goto err;
	}

	ven_rsi_dbg(ERR_ZONE, "%s: Initialized os intf ops\n", __func__);

	dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;

	status = rsi_usb_reg_read(dev->usbdev, FW_STATUS_REG, &fw_status, 2);
	if (status < 0)
		goto err1;
	else
		fw_status &= 1;

	if (!fw_status) {
		ven_rsi_dbg(INIT_ZONE, "Loading firmware...\n");
		status = rsi_hal_device_init(adapter);
		if (status) {
			ven_rsi_dbg(ERR_ZONE, "%s: Failed in device init\n",
				__func__);
			goto err1;
		}
		ven_rsi_dbg(INIT_ZONE, "%s: Device Init Done\n", __func__);
	}

	status = rsi_rx_urb_submit(adapter, 1 /* RX_WLAN_EP */);  
	if (status)
		goto err1;

#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	status = rsi_rx_urb_submit(adapter, 2 /* RX_BT_EP */);
	if (status)
		goto err1;
#endif

	return 0;
err1:
	rsi_deinit_usb_interface(adapter);
err:
	ven_rsi_91x_deinit(adapter);
	ven_rsi_dbg(ERR_ZONE, "%s: Failed in probe...Exiting\n", __func__);
	return status;
}

/**
 * rsi_disconnect() - This function performs the reverse of the probe function,
 *		      it deintialize the driver structure.
 * @pfunction: Pointer to the USB interface structure.
 *
 * Return: None.
 */
static void rsi_disconnect(struct usb_interface *pfunction)
{
	struct rsi_hw *adapter = usb_get_intfdata(pfunction);

	if (!adapter)
		return;

	ven_rsi_mac80211_detach(adapter);
	ven_rsi_dbg(INFO_ZONE, "mac80211 detach done\n");

#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
	rsi_hci_detach(adapter->priv);
	ven_rsi_dbg(INFO_ZONE, "HCI Detach Done\n");
#endif

	rsi_reset_card(adapter);

	rsi_deinit_usb_interface(adapter);
	ven_rsi_dbg(INFO_ZONE, "USB interface down\n");

	ven_rsi_91x_deinit(adapter);

	ven_rsi_dbg(INFO_ZONE, "%s: Deinitialization completed\n", __func__);
}

#ifdef CONFIG_PM
static int rsi_suspend(struct usb_interface *intf, pm_message_t message)
{
	/* Not yet implemented */
	return -ENOSYS;
}

static int rsi_resume(struct usb_interface *intf)
{
	/* Not yet implemented */
	return -ENOSYS;
}
#endif

static const struct usb_device_id rsi_dev_table[] = {
#if 0
	{ USB_DEVICE(0x0303, 0x0100) },
	{ USB_DEVICE(0x041B, 0x0301) },
	{ USB_DEVICE(0x041B, 0x0201) },
	{ USB_DEVICE(0x041B, 0x9330) },
#endif
	{ USB_DEVICE(0x1618, 0x9113) },
	{ /* Blank */},
};

static struct usb_driver rsi_driver = {
	.name       = "RSI-USB WLAN",
	.probe      = rsi_probe,
	.disconnect = rsi_disconnect,
	.id_table   = rsi_dev_table,
#ifdef CONFIG_PM
	.suspend    = rsi_suspend,
	.resume     = rsi_resume,
#endif
};

static int __init rsi_usb_module_init(void)
{
	ven_rsi_dbg(INIT_ZONE,
		"=====> RSI USB Module Initialize <=====\n");
	return usb_register(&rsi_driver);
}

static void __exit rsi_usb_module_exit(void)
{
	usb_deregister(&rsi_driver);
}

module_init(rsi_usb_module_init);
module_exit(rsi_usb_module_exit);

MODULE_AUTHOR("Redpine Signals Inc");
MODULE_DESCRIPTION("Common USB layer for RSI drivers");
MODULE_SUPPORTED_DEVICE("RSI-91x");
MODULE_DEVICE_TABLE(usb, rsi_dev_table);
MODULE_FIRMWARE(FIRMWARE_RSI9113);
MODULE_VERSION(DRV_VER);
MODULE_LICENSE("Dual BSD/GPL");
