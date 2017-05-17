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

#include "rsi_hci.h"
#include "rsi_mgmt.h"
#include "rsi_coex.h"
#include "rsi_hal.h"

#define RSI_BT_GENL_FAMILY "RSI-BTgenl"
#define RSI_USER_A_MAX	(__RSI_USER_A_MAX - 1)
#define RSI_VERSION_NR	1

static struct nla_policy bt_genl_policy[RSI_USER_A_MAX + 1] = {
	[RSI_USER_A_MSG] = { .type = NLA_NUL_STRING },
};

static struct genl_family bt_genl_family = {
	.id      = 0,
	.hdrsize = 0,
	.name    = RSI_BT_GENL_FAMILY,
	.version = RSI_VERSION_NR,
	.maxattr = RSI_USER_A_MAX,
};

static struct genl_ops bt_genl_ops = {
	.cmd    = RSI_USER_C_CMD,
	.flags  = 0,
	.policy = bt_genl_policy,
	.doit   = rsi_genl_recv,
	.dumpit = NULL,
};

/* Global GCB */
static struct genl_cb *global_gcb;

/**
 * rsi_hci_open() - This function is called when HCI device is
 * 						opened 
 * 
 * @hdev - pointer to HCI device
 * @return - 0 on success
 */
static int rsi_hci_open(struct hci_dev *hdev)
{
	ven_rsi_dbg(ERR_ZONE, "RSI HCI DEVICE \"%s\" open\n", hdev->name);

	if (test_and_set_bit(HCI_RUNNING, &hdev->flags))
		ven_rsi_dbg(ERR_ZONE, "%s: device `%s' already running\n", 
				__func__, hdev->name);

	return 0;
}

/**
 * rsi_hci_close() - This function is called when HCI device is
 * 						closed 
 * 
 * @hdev - pointer to HCI device
 * @return - 0 on success
 */
static int rsi_hci_close(struct hci_dev *hdev)
{
	ven_rsi_dbg(ERR_ZONE, "RSI HCI DEVICE \"%s\" closed\n", hdev->name);

	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		ven_rsi_dbg(ERR_ZONE, "%s: device `%s' not running\n",
				 __func__, hdev->name);

	return 0;
}

/**
 * rsi_hci_flush() - This function is called when HCI device is
 * 						flushed 
 * 
 * @hdev - pointer to HCI device
 * @return - 0 on success; negative error code on failure
 */
static int rsi_hci_flush(struct hci_dev *hdev)
{
	struct rsi_hci_adapter *h_adapter;

	if (!(h_adapter = hci_get_drvdata(hdev)))
		return -EFAULT;

	ven_rsi_dbg(ERR_ZONE, "RSI `%s' flush\n", hdev->name);

	return 0;
}

/**
 * rsi_hci_send_pkt() - This function is used send the packet received 
 * 						from HCI layer to co-ex module
 *
 * @hdev - pointer to HCI device
 * @skb - Received packet from HCI
 * @return - 0 on success; negative error code on failure
 *  
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION (3, 13, 0)
static int rsi_hci_send_pkt(struct sk_buff *skb)
#else
static int rsi_hci_send_pkt(struct hci_dev *hdev, struct sk_buff *skb)
#endif
{
	struct rsi_hci_adapter *h_adapter;
	struct sk_buff *new_skb = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION (3, 13, 0)
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
#endif
	int status = 0;

	if (skb->len <= 0) {
		ven_rsi_dbg(ERR_ZONE, "Zero length packet\n");
		//hdev->sta.err_tx++;
		status = -EINVAL;
		goto fail;
	}

	if (!(h_adapter = hci_get_drvdata(hdev))) {
		//hdev->sta.err_tx++;
		status = -EFAULT;
		goto fail;
	}

#ifdef CONFIG_VEN_RSI_WOW
	/* Stop here when in suspend */
	if (h_adapter->priv->wow_flags & RSI_WOW_ENABLED) {
		ven_rsi_dbg(INFO_ZONE, "In suspend: Dropping the pkt\n");
		status = -ENETDOWN;
		goto fail;
	}
#endif

	if (h_adapter->priv->bt_fsm_state != BT_DEVICE_READY) {
		ven_rsi_dbg(ERR_ZONE, "BT Device not ready\n");
		status = -ENODEV;
		goto fail;
	}

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		status = -EBUSY;
		goto fail;
	}

	switch (bt_cb(skb)->pkt_type) {
		case HCI_COMMAND_PKT:
			hdev->stat.cmd_tx++;
			break;

		case HCI_ACLDATA_PKT:
			hdev->stat.acl_tx++;
			break;

		case HCI_SCODATA_PKT:
			hdev->stat.sco_tx++;
			break;

		default:
			dev_kfree_skb(skb);
			status = -EILSEQ;
			goto fail;
	}

	if (skb_headroom(skb) < REQUIRED_HEADROOM_FOR_BT_HAL) {
               /* Re-allocate one more skb with sufficent headroom 
		 * make copy of input-skb to new one */
		u16 new_len = skb->len + REQUIRED_HEADROOM_FOR_BT_HAL;

		new_skb = dev_alloc_skb(new_len);
		if (!new_skb) {
			ven_rsi_dbg(ERR_ZONE, "%s: Failed to alloc skb\n",
				__func__);
			dev_kfree_skb(skb);
			return -ENOMEM;
		}
		skb_reserve(new_skb, REQUIRED_HEADROOM_FOR_BT_HAL);
                skb_put(new_skb, skb->len);
		memcpy(new_skb->data, skb->data, skb->len);
		bt_cb(new_skb)->pkt_type = bt_cb(skb)->pkt_type;
                dev_kfree_skb(skb);
                skb = new_skb;
	}

        rsi_hex_dump(DATA_RX_ZONE, "TX BT Pkt", skb->data, skb->len); 

#ifdef CONFIG_VEN_RSI_COEX
	rsi_coex_send_pkt(h_adapter->priv, skb, BT_Q);
#else
        rsi_send_bt_pkt(h_adapter->priv, skb);
#endif
	return 0;

fail:
	dev_kfree_skb(skb);
	return status;
}

int rsi_send_rfmode_frame(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_bt_rfmode_frame *cmd_frame;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending BT RF mode frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_bt_rfmode_frame));
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, sizeof(struct rsi_bt_rfmode_frame));
	cmd_frame = (struct rsi_bt_rfmode_frame *)skb->data;

	/* Length is 0 */
	cmd_frame->desc.q_no = RSI_BT_MGMT_Q;
	cmd_frame->desc.pkt_type = RSI_BT_PKT_TYPE_RFMODE;
	cmd_frame->bt_rf_tx_power_mode = 0;
	cmd_frame->bt_rf_tx_power_mode = 0;

	skb_put(skb, sizeof(struct rsi_bt_rfmode_frame));

//	return rsi_coex_send_pkt(common, skb, RSI_BT_Q);
	return common->priv->host_intf_ops->write_pkt(common->priv, skb->data, skb->len);
}
EXPORT_SYMBOL_GPL(rsi_send_rfmode_frame);

int rsi_deregister_bt(struct rsi_common *common)
{
	struct sk_buff *skb;
	struct rsi_bt_cmd_frame *cmd_frame;

	ven_rsi_dbg(MGMT_TX_ZONE, "%s: Sending BT register frame\n", __func__);

	skb = dev_alloc_skb(sizeof(struct rsi_bt_cmd_frame));
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0, sizeof(struct rsi_bt_cmd_frame));
	cmd_frame = (struct rsi_bt_cmd_frame *)skb->data;

	/* Length is 0 */
	cmd_frame->q_no = RSI_BT_MGMT_Q;
	cmd_frame->pkt_type = RSI_BT_PKT_TYPE_DEREGISTR;

	skb_put(skb, sizeof(struct rsi_bt_cmd_frame));

	//return rsi_coex_send_pkt(common, skb, RSI_BT_Q);
	return common->priv->host_intf_ops->write_pkt(common->priv, skb->data, skb->len);
}
EXPORT_SYMBOL_GPL(rsi_deregister_bt);

int rsi_hci_recv_pkt(struct rsi_common *common, u8 *pkt)
{
	struct rsi_hci_adapter *h_adapter =
		(struct rsi_hci_adapter *)common->hci_adapter;
	struct sk_buff *skb = NULL;
	struct hci_dev *hdev = NULL;
	int pkt_len = rsi_get_length(pkt, 0);
	u8 queue_no = rsi_get_queueno(pkt, 0);

	if ((common->bt_fsm_state == BT_DEVICE_NOT_READY) &&
	    (pkt[14] == BT_CARD_READY_IND)) {
		ven_rsi_dbg(INIT_ZONE, "%s: ===> BT Card Ready Received <===\n",
			__func__);

		if (common->suspend_in_prog) {
			ven_rsi_dbg(INFO_ZONE,
				"Suspend is in prog; Do not process\n");
			return 0;
		}

		rsi_send_rfmode_frame(common);

		ven_rsi_dbg(INFO_ZONE, "Attaching HCI module\n");

		if (rsi_hci_attach(common)) {
			ven_rsi_dbg(ERR_ZONE, "Failed to attach HCI module\n");
			return 0;
		}

		return 0;
	}
 
	if (common->bt_fsm_state != BT_DEVICE_READY) {
		ven_rsi_dbg(INFO_ZONE, "BT Device not ready\n");
		return 0;
	}
	
	if (queue_no == RSI_BT_MGMT_Q) {
		u8 msg_type = pkt[14] & 0xFF;
	
		switch (msg_type) {
		case RESULT_CONFIRM:
			ven_rsi_dbg(MGMT_RX_ZONE, "BT Result Confirm\n");
			return 0;
		case BT_BER:
			ven_rsi_dbg(MGMT_RX_ZONE, "BT Ber\n");
			return 0;
		case BT_CW:
			ven_rsi_dbg(MGMT_RX_ZONE, "BT CW\n");
			return 0;
		default:
			break;
		}
	}

	skb = dev_alloc_skb(pkt_len);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to alloc skb\n", __func__);
		return -ENOMEM;
	}
        hdev = h_adapter->hdev;
	memcpy(skb->data, pkt + FRAME_DESC_SZ, pkt_len);
	skb_put(skb, pkt_len);
	h_adapter->hdev->stat.byte_rx += skb->len;

	skb->dev = (void *)hdev;
	bt_cb(skb)->pkt_type = pkt[14];

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	return hci_recv_frame(skb);
#else
	return hci_recv_frame(hdev, skb);
#endif
}
EXPORT_SYMBOL_GPL(rsi_hci_recv_pkt);

/**
 * rsi_genl_recv() - This function gets the command request from
 * 					 user space over netlink socket
 *             
 * @skb		pointer to sk_buff structure
 * @info	read command info pointer
 *
 * @return	0 on success, negative error code on failure
 */
int rsi_genl_recv(struct sk_buff *skb, struct genl_info *info)
{
	struct rsi_hci_adapter *h_adapter = NULL;
	struct genl_cb *gcb;
	struct nlattr *na;
	u8 *data;
	int rc = -1, len, pkttype;
	u8 dword_align_req_bytes = 0;

	if (!(gcb = global_gcb))
		return -1;

	if (!(h_adapter = global_gcb->gc_drvpriv))
		return -1;

	gcb->gc_pid = get_portid(info);
	gcb->gc_seq = info->snd_seq;

	na = info->attrs[RSI_USER_A_MSG];
	if (na) {
		data = (u8 *)nla_data(na);
		if (!data) {
			ven_rsi_dbg(ERR_ZONE,
				"%s: no data recevied on family `%s'\n",
				__func__, gcb->gc_name);
			goto err;
		}
	} else {
		ven_rsi_dbg(ERR_ZONE,
			"%s: netlink attr is NULL on family `%s'\n",
			 __func__, gcb->gc_name);
		goto err;
	}
	gcb->gc_info = NULL;
	gcb->gc_skb = NULL;

	pkttype = *(u16 *)&data[0];
	len = *(u16 *)&data[2];

	data += 16;

	ven_rsi_dbg(ERR_ZONE, "%s: len %x pkt_type %x\n", 
			__func__, len, pkttype);

	rsi_hex_dump (DATA_RX_ZONE, "BT TX data", data, len);

	skb = dev_alloc_skb(len + REQUIRED_HEADROOM_FOR_BT_HAL);
	if (!skb) {
		ven_rsi_dbg(ERR_ZONE, "%s: Failed to alloc skb\n",
				__func__);
		return -ENOMEM;
	}
	skb_reserve(skb, REQUIRED_HEADROOM_FOR_BT_HAL);
	dword_align_req_bytes = ((unsigned long)skb->data) & 0x3f;
	if (dword_align_req_bytes)
		skb_push(skb, dword_align_req_bytes);
	memcpy(skb->data, data, len);
	bt_cb(skb)->pkt_type = pkttype;

#ifdef CONFIG_VEN_RSI_COEX
	return rsi_coex_send_pkt(h_adapter->priv, skb, RSI_BT_Q);
#else
        return rsi_send_bt_pkt(h_adapter->priv, skb);
#endif

err:
	ven_rsi_dbg(ERR_ZONE, "%s: error(%d) occured\n", __func__, rc);
	return rc;
}

/**
 * rsi_hci_attach () - This function initializes HCI interface
 *				      
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, negative error code on failure
 */
int rsi_hci_attach(struct rsi_common *common)
{
	struct rsi_hci_adapter *h_adapter = NULL;
	struct genl_cb *gcb = NULL;
	struct hci_dev *hdev;
	int status = 0;

	/* Allocate HCI adapter */
	/* TODO: Check GFP_ATOMIC */
	h_adapter = kzalloc(sizeof (*h_adapter), GFP_KERNEL);
	if (!h_adapter) {
		ven_rsi_dbg (ERR_ZONE, "Failed to alloc HCI adapter\n");
		return -ENOMEM;
	}
	h_adapter->priv = common;
	
	/* Create HCI Interface */
	hdev = hci_alloc_dev();
	if (!hdev) {
		ven_rsi_dbg (ERR_ZONE, "Failed to alloc HCI device\n");
		goto err;
	}
	h_adapter->hdev = hdev;

	if (common->priv->rsi_host_intf == RSI_HOST_INTF_SDIO)
		hdev->bus = HCI_SDIO;
	else
		hdev->bus = HCI_USB;

	hci_set_drvdata(hdev, h_adapter);
#if LINUX_VERSION_CODE >= KERNEL_VERSION (4, 8, 0)
  hdev->dev_type = HCI_PRIMARY;
#else
	hdev->dev_type = HCI_BREDR;
#endif

	hdev->open = rsi_hci_open;
	hdev->close = rsi_hci_close;
	hdev->flush = rsi_hci_flush;
	hdev->send = rsi_hci_send_pkt;
#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 3, 8)
	hdev->destruct = rsi_hci_destruct;
	hdev->owner = THIS_MODULE;
#endif

        /* Initialize TX queue */
	skb_queue_head_init(&h_adapter->hci_tx_queue);
	common->hci_adapter = (void *)h_adapter;
	
	status = hci_register_dev(hdev);
	if (status < 0) {
		ven_rsi_dbg(ERR_ZONE,
			"%s: HCI registration failed with errcode %d\n",
			__func__, status);
		goto err;
	}
	ven_rsi_dbg(INIT_ZONE, "HCI Interface Created with name \'%s\'\n",
		hdev->name);

	/* Register for general netlink operations */
	/* TODO: Check GFP_ATOMIC */
	gcb = kzalloc(sizeof(*gcb), GFP_KERNEL);
	if (!gcb) {
		ven_rsi_dbg (ERR_ZONE, "%s: Failed to alloc genl control block\n",
				__func__); 
		goto err;
	}
	h_adapter->gcb = gcb;
	global_gcb = gcb;

	gcb->gc_drvpriv = h_adapter;
	gcb->gc_family = &bt_genl_family;
	gcb->gc_policy = &bt_genl_policy[0];
	gcb->gc_ops = &bt_genl_ops;
	gcb->gc_n_ops = 1;
	gcb->gc_name = RSI_BT_GENL_FAMILY;
	gcb->gc_pid = gcb->gc_done = 0;

	ven_rsi_dbg(INIT_ZONE, "genl-register: nl_family `%s'\n", gcb->gc_name);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	gcb->gc_family->ops = gcb->gc_ops;
	gcb->gc_family->n_ops = gcb->gc_n_ops;
#endif

	if (genl_register_family(gcb->gc_family)) {
		ven_rsi_dbg(ERR_ZONE, "%s: genl_register_family failed\n",
			__func__);
		goto err;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 12, 34)
	if (genl_register_ops(gcb->gc_family, gcb->gc_ops)) {
		ven_rsi_dbg(ERR_ZONE, "%s: genl_register_ops failed\n", __func__);
		genl_unregister_family(family);
		goto err;
	}
#endif
	gcb->gc_done = 1;
	common->bt_fsm_state = BT_DEVICE_READY;
	ven_rsi_dbg(ERR_ZONE, " HCI module init done...\n");

	return 0;

err:
	if (hdev) {
		hci_unregister_dev(hdev);
		hci_free_dev(hdev);
		h_adapter->hdev = NULL;
	}
	if (gcb) {
		genl_unregister_family(gcb->gc_family);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
		genl_unregister_ops(gcb->gc_family, gcb->gc_ops);
#endif
		kfree(gcb);
	}
	h_adapter->gcb = NULL;
	kfree(h_adapter);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(rsi_hci_attach);

/**
 * rsi_hci_attach () - This function initializes HCI interface
 *				      
 * @common: Pointer to the driver private structure.
 *
 * Return: 0 on success, negative error code on failure
 */
void rsi_hci_detach(struct rsi_common *common)
{
	struct rsi_hci_adapter *h_adapter = 
		(struct rsi_hci_adapter *)common->hci_adapter;
	struct hci_dev *hdev;
	struct genl_cb *gcb;

	ven_rsi_dbg(INFO_ZONE, "Detaching HCI...\n");

	if (!h_adapter)
		return;

	if (common->suspend_in_prog)
		rsi_deregister_bt(common);

	hdev = h_adapter->hdev;
	if (hdev) {
                //hci_dev_hold(hdev);
		hci_unregister_dev(hdev);
                //hci_dev_put(hdev);
		hci_free_dev(hdev);
		h_adapter->hdev = NULL;
	}

	gcb = h_adapter->gcb;
	if (gcb) {
		genl_unregister_family(gcb->gc_family);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 12, 34)
		genl_unregister_ops(gcb->gc_family, gcb->gc_ops);
#endif
		h_adapter->gcb = NULL;
		kfree(gcb);
	}
	kfree(h_adapter);
	common->bt_fsm_state = BT_DEVICE_NOT_READY;

	return;
}
EXPORT_SYMBOL_GPL(rsi_hci_detach);

