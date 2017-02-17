/**
 * Copyright (c) 2014 Redpine Signals Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/firmware.h>
#include "rsi_mgmt.h"
#include "rsi_common.h"
#include "rsi_hal.h"
#if defined(CONFIG_VEN_RSI_HCI) || defined(CONFIG_VEN_RSI_COEX)
#include "rsi_hci.h"
#endif
#ifdef CONFIG_VEN_RSI_COEX
#include "rsi_coex.h"
#endif

u32 ven_rsi_zone_enabled =	//INFO_ZONE |
			INIT_ZONE |
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
		if (!(ii % 16))
			printk("\n");
		printk("%02x ", msg[ii]);
	}
	printk("\n");
}
EXPORT_SYMBOL_GPL(rsi_hex_dump);

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

//	if (vif->type == NL80211_IFTYPE_STATION)
		rx_params->channel = rsi_get_connected_channel(common->priv);
//	else
//		rx_params->channel = common->ap_channel->hw_value;

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
#if defined(CONFIG_VEN_RSI_HCI) || defined(CONFIG_VEN_RSI_COEX)
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

#ifdef CONFIG_CARACALLA_BOARD
static void rsi_bcn_sched(unsigned long data)
{
	struct rsi_common *common = (struct rsi_common *)data;

	rsi_set_event(&common->bcn_thread.event);

	common->bcn_timer.expires =
		msecs_to_jiffies(common->beacon_interval - 5) + jiffies;
	add_timer(&common->bcn_timer);
}

void rsi_init_bcn_timer(struct rsi_common *common)
{
	init_timer(&common->bcn_timer);

	common->bcn_timer.data = (unsigned long)common;
	common->bcn_timer.expires =
		msecs_to_jiffies(common->beacon_interval - 5) + jiffies;
	common->bcn_timer.function = (void *)rsi_bcn_sched;

	add_timer(&common->bcn_timer);
}

void rsi_del_bcn_timer(struct rsi_common *common)
{
	del_timer(&common->bcn_timer);
}

void rsi_bcn_scheduler_thread(struct rsi_common *common)
{
	do {
		rsi_wait_event(&common->bcn_thread.event,
			       msecs_to_jiffies(common->beacon_interval));
		rsi_reset_event(&common->bcn_thread.event);

		if (!common->beacon_enabled)
			continue;
		if (!common->init_done)
			continue;
		if (common->iface_down)
			continue;
		rsi_send_beacon(common);
	} while (atomic_read(&common->bcn_thread.thread_done) == 0);
	complete_and_exit(&common->bcn_thread.completion, 0);
}
#endif

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
		rsi_dbg(ERR_ZONE, "%s: Unable to init sdio intr poll thrd\n",
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

	common->beacon_frame = kzalloc(512, GFP_KERNEL);
	if (!common->beacon_frame)
		goto err;
	common->beacon_frame_len = 0;

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_head_init(&common->tx_queue[ii]);

	rsi_init_event(&common->tx_thread.event);
	rsi_init_event(&common->bcn_thread.event);
	mutex_init(&common->mutex);
	mutex_init(&common->tx_lock);
	mutex_init(&common->rx_lock);

	if (rsi_create_kthread(common,
			       &common->tx_thread,
			       rsi_tx_scheduler_thread,
			       "Tx-Thread")) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init tx thrd\n", __func__);
		goto err;
	}

#ifdef CONFIG_CARACALLA_BOARD
	if (rsi_create_kthread(common,
			       &common->bcn_thread,
			       rsi_bcn_scheduler_thread,
			       "Beacon-Thread")) {
		ven_rsi_dbg(ERR_ZONE, "%s: Unable to init bcn thrd\n", __func__);
		goto err;
	}
#endif

#ifdef CONFIG_VEN_RSI_COEX
	if (rsi_coex_init(common)) {
		ven_rsi_dbg(ERR_ZONE, "Failed to init COEX module\n");
		goto err;
	}
#endif
	rsi_default_ps_params(adapter);
	spin_lock_init(&adapter->ps_lock);
	common->uapsd_bitmap = 0;
	init_bgscan_params(common);

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

	rsi_kill_thread(&common->tx_thread);
#ifdef CONFIG_CARACALLA_BOARD
	rsi_kill_thread(&common->bcn_thread);
#endif

	for (ii = 0; ii < NUM_SOFT_QUEUES; ii++)
		skb_queue_purge(&common->tx_queue[ii]);

#ifdef CONFIG_VEN_RSI_COEX
	rsi_coex_deinit(common);
#endif
	common->init_done = false;

	kfree(common->beacon_frame);
	common->beacon_frame = NULL;
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
MODULE_VERSION("0.1");
MODULE_LICENSE("Dual BSD/GPL");
