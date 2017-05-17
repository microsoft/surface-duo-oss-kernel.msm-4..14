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

#include <linux/firmware.h>
#include "rsi_usb.h"

/**
 * rsi_usb_rx_thread() - This is a kernel thread to receive the packets from
 *			 the USB device.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
void rsi_usb_rx_thread(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;
	struct rsi_91x_usbdev *dev = (struct rsi_91x_usbdev *)adapter->rsi_dev;
	struct rx_usb_ctrl_block *rx_cb;
	int status, idx;

	do {
		rsi_wait_event(&dev->rx_thread.event, EVENT_WAIT_FOREVER);

		if (atomic_read(&dev->rx_thread.thread_done))
			break;

		for (idx = 0; idx < MAX_RX_URBS; idx++) {
			rx_cb = &dev->rx_cb[idx];
			if (!rx_cb->pend)
				continue;
			
			mutex_lock(&common->rx_lock);
			status = ven_rsi_read_pkt(common, rx_cb->rx_buffer, 0);
			if (status) {
				ven_rsi_dbg(ERR_ZONE, "%s: Failed To read data",
					__func__);
				mutex_unlock(&common->rx_lock);
				break;
			}
			rx_cb->pend = 0;
			mutex_unlock(&common->rx_lock);
			
			if (adapter->rx_urb_submit(adapter, rx_cb->ep_num)) {
				ven_rsi_dbg(ERR_ZONE,
					"%s: Failed in urb submission", __func__);
				break;
			}
			/* Update TX buffer status */
			//rsi_usb_check_queue_status(adapter, 0);
		}
		rsi_reset_event(&dev->rx_thread.event);
	} while (1);

	ven_rsi_dbg(INFO_ZONE, "%s: Terminated USB RX thread\n", __func__);
	atomic_inc(&dev->rx_thread.thread_done);
	complete_and_exit(&dev->rx_thread.completion, 0);
}

