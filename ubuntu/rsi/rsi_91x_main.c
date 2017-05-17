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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/firmware.h>
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_hal.h"
#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
#include "rsi_hci.h"
#endif
#ifdef CONFIG_VEN_RSI_COEX
#include "rsi_coex.h"
#endif

u32 ven_rsi_zone_enabled =	//INFO_ZONE |
				//INIT_ZONE |
				//MGMT_TX_ZONE |
				//MGMT_RX_ZONE |
				//DATA_TX_ZONE |
				//DATA_RX_ZONE |
				//FSM_ZONE |
				//ISR_ZONE |
				ERR_ZONE |
				0;
EXPORT_SYMBOL_GPL(ven_rsi_zone_enabled);

/**
 * ven_rsi_dbg() - This function outputs informational messages.
 * @zone: Zone of interest for output message.
 * @fmt: printf-style format for output message.
 *
 * Return: none
 */
void ven_rsi_dbg(u32 zone, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	if (zone & ven_rsi_zone_enabled)
		pr_info("%pV", &vaf);
	va_end(args);
}
EXPORT_SYMBOL_GPL(ven_rsi_dbg);

/**
 * rsi_hex_dump() - This function prints the packet (/msg) in hex bytes.
 * @zone: Zone of interest for output message.
 * @msg_str: Message to be printed with packet
 * @msg: Packet to be printed
 * @len: Length of the packet
 *
 * Return: none
 */
void rsi_hex_dump(u32 zone, char *msg_str, const u8 *msg, u32 len)
{
	int ii;

	if (!(zone & ven_rsi_zone_enabled))
		return;
	printk("%s: (length = %d)\n", msg_str, len);
	for (ii = 0; ii < len; ii++) {
		if (ii && !(ii % 16))
			printk("\n");
		printk("%02x ", msg[ii]);
	}
	printk("\n");
}
EXPORT_SYMBOL_GPL(rsi_hex_dump);

char *opmode_str(int oper_mode)
{
	switch (oper_mode) {
	case DEV_OPMODE_WIFI_ALONE:
	       return "Wi-Fi alone";
	case DEV_OPMODE_BT_ALONE:
	       return "BT EDR alone";
	case DEV_OPMODE_BT_LE_ALONE:
	       return "BT LE alone";
	case DEV_OPMODE_BT_DUAL:
	       return "BT Dual";
	case DEV_OPMODE_STA_BT:
	       return "Wi-Fi STA + BT EDR";
	case DEV_OPMODE_STA_BT_LE:
	       return "Wi-Fi STA + BT LE";
	case DEV_OPMODE_STA_BT_DUAL:
	       return "Wi-Fi STA + BT DUAL";
	case DEV_OPMODE_AP_BT:
	       return "Wi-Fi AP + BT EDR";
	case DEV_OPMODE_AP_BT_DUAL:
	       return "Wi-Fi AP + BT DUAL";
	}
	return "Unknown";
}

void rsi_print_version(struct rsi_common *common)
{
	memcpy(common->driver_ver, DRV_VER, ARRAY_SIZE(DRV_VER));
	common->driver_ver[ARRAY_SIZE(DRV_VER)] = '\0';

	ven_rsi_dbg(ERR_ZONE, "================================================\n");
	ven_rsi_dbg(ERR_ZONE, "================ RSI Version Info ==============\n");
	ven_rsi_dbg(ERR_ZONE, "================================================\n");
	ven_rsi_dbg(ERR_ZONE, "FW Version\t: %d.%d.%d\n",
		common->lmac_ver.major, common->lmac_ver.minor,
		common->lmac_ver.release_num);
	ven_rsi_dbg(ERR_ZONE, "Driver Version\t: %s", common->driver_ver);
	ven_rsi_dbg(ERR_ZONE, "Operating mode\t: %d [%s]",
		common->oper_mode, opmode_str(common->oper_mode));
	ven_rsi_dbg(ERR_ZONE, "Firmware file\t: %s", common->priv->fw_file_name);
	ven_rsi_dbg(ERR_ZONE, "================================================\n");
}

/**
 * rsi_prepare_skb() - This function prepares the skb.
 * @common: Pointer to the driver private structure.
 * @buffer: Pointer to the packet data.
 * @pkt_len: Length of the packet.
 * @extended_desc: Extended descriptor.
 *
 * Return: Successfully skb.
 */
static struct sk_buff *rsi_prepare_skb(struct rsi_common *common,
				       u8 *buffer,
				       u32 pkt_len,
				       u8 extended_desc)
{
	struct ieee80211_tx_info *info;
	struct skb_info *rx_params;
	struct sk_buff *skb = NULL;
	u8 payload_offset;

	if (WARN(!pkt_len, "%s: Dummy pkt received", __func__))
		return NULL;

	if (pkt_len > (RSI_RCV_BUFFER_LEN * 4)) {
		ven_rsi_dbg(ERR_ZONE, "%s: Pkt size > max rx buf size %d\n",
			__func__, pkt_len);
		pkt_len = RSI_RCV_BUFFER_LEN * 4;
	}

	pkt_len -= extended_desc;
	skb = dev_alloc_skb(pkt_len + FRAME_DESC_SZ);
	if (!skb)
		return NULL;

	payload_offset = (extended_desc + FRAME_DESC_SZ);
	skb_put(skb, pkt_len);
	memcpy((skb->data), (buffer + payload_offset), skb->len);

	info = IEEE80211_SKB_CB(skb);
	rx_params = (struct skb_info *)info->driver_data;
	rx_params->rssi = rsi_get_rssi(buffer);

	rx_params->channel = rsi_get_connected_channel(common->priv);

	return skb;
}

/**
 * ven_rsi_read_pkt() - This function reads frames from the card.
 * @common: Pointer to the driver private structure.
 * @rcv_pkt_len: Received pkt length. In case of USB it is 0.
 *
 * Return: 0 on success, -1 on failure.
 */
int ven_rsi_read_pkt(struct rsi_common *common, u8 *rx_pkt, s32 rcv_pkt_len)
{
	u8 *frame_desc = NULL, extended_desc = 0;
	u32 index = 0, length = 0, queueno = 0;
	u16 actual_length = 0, offset;
	struct sk_buff *skb = NULL;

	do {
		frame_desc = &rx_pkt[index];
		actual_length = *(u16 *)&frame_desc[0];
		offset = *(u16 *)&frame_desc[2];

		if ((actual_length < (4 + FRAME_DESC_SZ)) || (offset < 4)) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: actual_length (%d) is less than 20 or"
				" offset(%d) is less than 4\n",
				__func__, actual_length, offset);
			break;
		}
		queueno = rsi_get_queueno(frame_desc, offset);
		length = rsi_get_length(frame_desc, offset);
		if (queueno == RSI_WIFI_DATA_Q || queueno == RSI_WIFI_MGMT_Q)
			extended_desc = rsi_get_extended_desc(frame_desc,
							      offset);

		switch (queueno) {
		case RSI_COEX_Q:
			rsi_hex_dump(MGMT_RX_ZONE,
				     "RX Command co ex packet",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
#ifdef CONFIG_VEN_RSI_COEX
			rsi_coex_recv_pkt(common, (frame_desc + offset));
#else
			rsi_mgmt_pkt_recv(common, (frame_desc + offset));
#endif
			break;
		case RSI_WIFI_DATA_Q:
			rsi_hex_dump(DATA_RX_ZONE,
				     "RX Data pkt",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
			skb = rsi_prepare_skb(common,
					      (frame_desc + offset),
					      length,
					      extended_desc);
			if (!skb)
				goto fail;

			rsi_indicate_pkt_to_os(common, skb);
			break;

		case RSI_WIFI_MGMT_Q:
			rsi_mgmt_pkt_recv(common, (frame_desc + offset));
			break;
#if defined(CONFIG_VEN_RSI_BT_ALONE) || defined(CONFIG_VEN_RSI_COEX)
		case RSI_BT_MGMT_Q:
		case RSI_BT_DATA_Q:
			rsi_hex_dump(DATA_RX_ZONE,
				     "RX BT Pkt",
				     frame_desc + offset,
				     FRAME_DESC_SZ + length);
			rsi_hci_recv_pkt(common, frame_desc + offset);
			break;
#endif

		default:
			ven_rsi_dbg(ERR_ZONE, "%s: pkt from invalid queue: %d\n",
				__func__,   queueno);
			goto fail;
		}

		index  += actual_length;
		rcv_pkt_len -= actual_length;
	} while (rcv_pkt_len > 0);

	return 0;
fail:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(ven_rsi_read_pkt);

/**
 * rsi_tx_scheduler_thread() - This function is a kernel thread to send the
 *			       packets to the device.
 * @common: Pointer to the driver private structure.
 *
 * Return: None.
 */
static void rsi_tx_scheduler_thread(struct rsi_common *common)
{
	struct rsi_hw *adapter = common->priv;
	u32 timeout = EVENT_WAIT_FOREVER;

	do {
		if (adapter->determine_event_timeout)
			timeout = adapter->determine_event_timeout(adapter);
		rsi_wait_event(&common->tx_thread.event, timeout);
		rsi_reset_event(&common->tx_thread.event);

		if (common->init_done)
			rsi_core_qos_processor(common);
	} while (atomic_read(&common->tx_thread.thread_done) == 0);
	complete_and_exit(&common->tx_thread.completion, 0);
}

#ifdef CONFIG_SDIO_INTR_POLL
void rsi_sdio_intr_poll_scheduler_thread(struct rsi_common *common)
{
        struct rsi_hw *adapter = common->priv;
        int status = 0;

        do {
                status = adapter->check_intr_status_reg(adapter);
                if (adapter->isr_pending)
                        adapter->isr_pending = 0;
                msleep(20);

        } while (atomic_read(&common->sdio_intr_poll_thread.thread_done) == 0);
        complete_and_exit(&common->sdio_intr_poll_thread.completion, 0);
}

void init_sdio_intr_status_poll_thread(struct rsi_common *common)
{
	rsi_init_event(&common->sdio_intr_poll_thread.event);
	if (rsi_create_kthread(common,
			       &common->sdio_intr_poll_thread,
			       rsi_sdio_intr_poll_scheduler_thread,
			       "Sdio Intr poll-Thread")) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init sdio intr poll thrd\n",
				__func__);
	}
}
EXPORT_SYMBOL_GPL(init_sdio_intr_status_poll_thread);
#endif

/**
 * ven_rsi_91x_init() - This function initializes os interface operations.
 * @void: Void.
 *
 * Return: Pointer to the adapter structure on success, NULL on failure .
 */
struct rsi_hw *ven_rsi_91x_init(void)
{
	struct rsi_hw *adapter = NULL;
	struct rsi_common *common = NULL;
	u8 ii = 0;

	adapter = kzalloc(sizeof(*adapter), GFP_KERNEL);
	if (!adapter)
		return NULL;

	adapter->priv = kzalloc(sizeof(*common), GFP_KERNEL);
	if (!adapter->priv) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed in allocation of priv\n",
			__func__);
		kfree(adapter);
		return NULL;
	}
	common = adapter->priv;
	common->priv = adapter;

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_head_init(&common->tx_queue[ii]);

	rsi_init_event(&common->tx_thread.event);
	mutex_init(&common->mutex);
	mutex_init(&common->tx_lock);
	mutex_init(&common->rx_lock);
	sema_init(&common->tx_bus_lock, 1);

#ifdef CONFIG_HW_SCAN_OFFLOAD
	rsi_init_event(&common->chan_set_event);
	rsi_init_event(&common->probe_cfm_event);
	rsi_init_event(&common->chan_change_event);
	rsi_init_event(&common->cancel_hw_scan_event);
	common->scan_workqueue = 
		create_singlethread_workqueue("rsi_scan_worker");
	INIT_WORK(&common->scan_work, rsi_scan_start);
#endif

	if (rsi_create_kthread(common,
			       &common->tx_thread,
			       rsi_tx_scheduler_thread,
			       "Tx-Thread")) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init tx thrd\n", __func__);
		goto err;
	}

#ifdef CONFIG_VEN_RSI_COEX
	if (rsi_coex_init(common)) {
		ven_rsi_dbg(ERR_ZONE, "Failed to init COEX module\n");
		goto err;
	}
#endif
	/* Power save related */
	rsi_default_ps_params(adapter);
	spin_lock_init(&adapter->ps_lock);
	common->uapsd_bitmap = 0;

	/* BGScan related */
	init_bgscan_params(common);

	/* Wi-Fi direct related */
	common->roc_timer.data = (unsigned long)common;
	common->roc_timer.function = (void *)&rsi_roc_timeout;
	init_timer(&common->roc_timer);

	common->init_done = true;
	return adapter;

err:
	kfree(common);
	kfree(adapter);
	return NULL;
}
EXPORT_SYMBOL_GPL(ven_rsi_91x_init);

/**
 * ven_rsi_91x_deinit() - This function de-intializes os intf operations.
 * @adapter: Pointer to the adapter structure.
 *
 * Return: None.
 */
void ven_rsi_91x_deinit(struct rsi_hw *adapter)
{
	struct rsi_common *common = adapter->priv;
	u8 ii;

	ven_rsi_dbg(INFO_ZONE, "%s: Deinit core module...\n", __func__);

#ifdef CONFIG_HW_SCAN_OFFLOAD
	flush_workqueue(common->scan_workqueue);
	destroy_workqueue(common->scan_workqueue);
#endif

	rsi_kill_thread(&common->tx_thread);

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_purge(&common->tx_queue[ii]);

#ifdef CONFIG_VEN_RSI_COEX
	rsi_coex_deinit(common);
#endif
	common->init_done = false;

	kfree(common);
	kfree(adapter->rsi_dev);
	kfree(adapter);
}
EXPORT_SYMBOL_GPL(ven_rsi_91x_deinit);

/**
 * rsi_91x_hal_module_init() - This function is invoked when the module is
 *			       loaded into the kernel.
 *			       It registers the client driver.
 * @void: Void.
 *
 * Return: 0 on success, -1 on failure.
 */
static int rsi_91x_hal_module_init(void)
{
	ven_rsi_dbg(INIT_ZONE, "%s: Module init called\n", __func__);
	return 0;
}

/**
 * rsi_91x_hal_module_exit() - This function is called at the time of
 *			       removing/unloading the module.
 *			       It unregisters the client driver.
 * @void: Void.
 *
 * Return: None.
 */
static void rsi_91x_hal_module_exit(void)
{
	ven_rsi_dbg(INIT_ZONE, "%s: Module exit called\n", __func__);
}

module_init(rsi_91x_hal_module_init);
module_exit(rsi_91x_hal_module_exit);
MODULE_AUTHOR("Redpine Signals Inc");
MODULE_DESCRIPTION("Station driver for RSI 91x devices");
MODULE_SUPPORTED_DEVICE("RSI-91x");
MODULE_VERSION(DRV_VER);
MODULE_LICENSE("Dual BSD/GPL");
