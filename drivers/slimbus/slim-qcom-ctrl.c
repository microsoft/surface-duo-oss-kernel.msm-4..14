/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
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
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/slimbus.h>
#include "slim-qcom.h"

#define MSM_SLIM_NAME	"msm_slim_ctrl"

/* Manager registers */
#define	MGR_CFG		0x200
#define	MGR_STATUS	0x204
#define	MGR_INT_EN	0x210
#define	MGR_INT_STAT	0x214
#define	MGR_INT_CLR	0x218
#define	MGR_TX_MSG	0x230
#define	MGR_RX_MSG	0x270
#define	MGR_IE_STAT	0x2F0
#define	MGR_VE_STAT	0x300
#define	MGR_CFG_ENABLE	1

/* Framer registers */
#define	FRM_CFG		0x400
#define	FRM_STAT	0x404
#define	FRM_INT_EN	0x410
#define	FRM_INT_STAT	0x414
#define	FRM_INT_CLR	0x418
#define	FRM_WAKEUP	0x41C
#define	FRM_CLKCTL_DONE	0x420
#define	FRM_IE_STAT	0x430
#define	FRM_VE_STAT	0x440

/* Interface registers */
#define	INTF_CFG	0x600
#define	INTF_STAT	0x604
#define	INTF_INT_EN	0x610
#define	INTF_INT_STAT	0x614
#define	INTF_INT_CLR	0x618
#define	INTF_IE_STAT	0x630
#define	INTF_VE_STAT	0x640

/* Interrupt status bits */
#define	MGR_INT_TX_NACKED_2	BIT(25)
#define	MGR_INT_MSG_BUF_CONTE	BIT(26)
#define	MGR_INT_RX_MSG_RCVD	BIT(30)
#define	MGR_INT_TX_MSG_SENT	BIT(31)

/* Framer config register settings */
#define	FRM_ACTIVE	1
#define	CLK_GEAR	7
#define	ROOT_FREQ	11
#define	REF_CLK_GEAR	15
#define	INTR_WAKE	19

static int msm_slim_queue_tx(struct msm_slim_ctrl *dev, u32 *buf, u8 len,
			     u32 tx_reg)
{
	int i;

	for (i = 0; i < (len + 3) >> 2; i++) {
		dev_dbg(dev->dev, "AHB TX data:0x%x\n", buf[i]);
		writel_relaxed(buf[i], dev->base + tx_reg + (i * 4));
	}
	/* Guarantee that message is sent before returning */
	mb();
	return 0;
}

static irqreturn_t msm_slim_interrupt(int irq, void *d)
{
	struct msm_slim_ctrl *dev = d;
	u32 stat = readl_relaxed(dev->base + MGR_INT_STAT);
	int err = 0, ret = IRQ_NONE;

	if (stat & MGR_INT_TX_MSG_SENT || stat & MGR_INT_TX_NACKED_2) {
		if (stat & MGR_INT_TX_MSG_SENT)
			writel_relaxed(MGR_INT_TX_MSG_SENT,
				       dev->base + MGR_INT_CLR);
		if (stat & MGR_INT_TX_NACKED_2) {
			u32 mgr_stat = readl_relaxed(dev->base + MGR_STATUS);
			u32 mgr_ie_stat = readl_relaxed(dev->base +
							MGR_IE_STAT);
			u32 frm_stat = readl_relaxed(dev->base + FRM_STAT);
			u32 frm_cfg = readl_relaxed(dev->base + FRM_CFG);
			u32 frm_intr_stat = readl_relaxed(dev->base +
							  FRM_INT_STAT);
			u32 frm_ie_stat = readl_relaxed(dev->base +
							FRM_IE_STAT);
			u32 intf_stat = readl_relaxed(dev->base + INTF_STAT);
			u32 intf_intr_stat = readl_relaxed(dev->base +
							   INTF_INT_STAT);
			u32 intf_ie_stat = readl_relaxed(dev->base +
							 INTF_IE_STAT);

			writel_relaxed(MGR_INT_TX_NACKED_2, dev->base +
				       MGR_INT_CLR);
			dev_err(dev->dev, "TX Nack MGR:int:0x%x, stat:0x%x\n",
				stat, mgr_stat);
			dev_err(dev->dev, "TX Nack MGR:ie:0x%x\n", mgr_ie_stat);
			dev_err(dev->dev, "TX Nack FRM:int:0x%x, stat:0x%x\n",
				frm_intr_stat, frm_stat);
			dev_err(dev->dev, "TX Nack FRM:cfg:0x%x, ie:0x%x\n",
				frm_cfg, frm_ie_stat);
			dev_err(dev->dev, "TX Nack INTF:intr:0x%x, stat:0x%x\n",
				intf_intr_stat, intf_stat);
			dev_err(dev->dev, "TX Nack INTF:ie:0x%x\n",
				intf_ie_stat);
			err = -ENOTCONN;
		}
		/**
		 * Guarantee that interrupt clear bit write goes through before
		 * signalling completion/exiting ISR
		 */
		mb();
		slim_return_tx(&dev->ctrl, err);
		ret = IRQ_HANDLED;
	}
	if (stat & MGR_INT_RX_MSG_RCVD) {
		u8 mc, mt;
		u8 len, i;
		u32 *rx_buf, pkt[10];
		bool q_rx = false;

		pkt[0] = readl_relaxed(dev->base + MGR_RX_MSG);
		mt = (pkt[0] >> 5) & 0x7;
		mc = (pkt[0] >> 8) & 0xff;
		len = pkt[0] & 0x1F;
		dev_dbg(dev->dev, "RX-IRQ: MC: %x, MT: %x\n", mc, mt);

		/**
		 * this message cannot be handled by ISR, so
		 * let work-queue handle it
		 */
		if (mt == SLIM_MSG_MT_CORE &&
			mc == SLIM_MSG_MC_REPORT_PRESENT)
			rx_buf = (u32 *)slim_get_rx(&dev->ctrl);
		else
			rx_buf = pkt;

		if (rx_buf == NULL) {
			dev_err(dev->dev, "dropping RX:0x%x due to RX full\n",
						pkt[0]);
			goto rx_ret_irq;
		}

		rx_buf[0] = pkt[0];
		for (i = 1; i < ((len + 3) >> 2); i++) {
			rx_buf[i] = readl_relaxed(dev->base + MGR_RX_MSG +
						(4 * i));
			dev_dbg(dev->dev, "reading data: %x\n", rx_buf[i]);
		}

		switch (mc) {
			u8 *buf, la;
			u16 ele;

		case SLIM_MSG_MC_REPORT_PRESENT:
			q_rx = true;
			break;
		case SLIM_MSG_MC_REPLY_INFORMATION:
		case SLIM_MSG_MC_REPLY_VALUE:
			slim_msg_response(&dev->ctrl, (u8 *)(rx_buf + 1),
					  (u8)(*rx_buf >> 24), (len - 4));
			break;
		case SLIM_MSG_MC_REPORT_INFORMATION:
			buf = (u8 *)rx_buf;
			la = buf[2];
			ele = (u16)buf[4] << 4;

			ele |= ((buf[3] & 0xf0) >> 4);
			/**
			 * report information is most likely loss of
			 * sync or collision detected in data slots
			 */
			dev_err(dev->dev, "LA:%d report inf ele:0x%x\n",
				la, ele);
			for (i = 0; i < len - 5; i++)
				dev_err(dev->dev, "bit-mask:%x\n",
					buf[i+5]);
			break;
		default:
			dev_err(dev->dev, "unsupported MC,%x MT:%x\n",
				mc, mt);
			break;
		}
rx_ret_irq:
		writel_relaxed(MGR_INT_RX_MSG_RCVD, dev->base +
			       MGR_INT_CLR);
		/**
		 * Guarantee that CLR bit write goes through
		 * before exiting
		 */
		mb();
		if (q_rx)
			queue_work(dev->rxwq, &dev->wd);

		ret = IRQ_HANDLED;
	}
	return ret;
}

static int msm_xfer_msg(struct slim_controller *ctrl, struct slim_msg_txn *txn,
			void *pbuf)
{
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	DECLARE_COMPLETION_ONSTACK(done);
	u32 *head = (u32 *)pbuf;
	u8 *puc = (u8 *)pbuf;
	u8 la = txn->la;

	/* HW expects length field to be excluded */
	txn->rl--;

	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
		*head = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt, txn->mc, 0,
						la);
	else
		*head = SLIM_MSG_ASM_FIRST_WORD(txn->rl, txn->mt, txn->mc, 1,
						la);

	if (txn->dt == SLIM_MSG_DEST_LOGICALADDR)
		puc += 3;
	else
		puc += 2;

	if (txn->mt == SLIM_MSG_MT_CORE && slim_tid_txn(txn->mt, txn->mc))
		*(puc++) = txn->tid;

	if ((txn->mt == SLIM_MSG_MT_CORE) &&
		((txn->mc >= SLIM_MSG_MC_REQUEST_INFORMATION &&
		txn->mc <= SLIM_MSG_MC_REPORT_INFORMATION) ||
		(txn->mc >= SLIM_MSG_MC_REQUEST_VALUE &&
		 txn->mc <= SLIM_MSG_MC_CHANGE_VALUE))) {
		*(puc++) = (txn->ec & 0xFF);
		*(puc++) = (txn->ec >> 8) & 0xFF;
	}

	if (txn->msg && txn->msg->wbuf)
		memcpy(puc, txn->msg->wbuf, txn->msg->num_bytes);

	return msm_slim_queue_tx(dev, head, txn->rl, MGR_TX_MSG);
}

static int msm_set_laddr(struct slim_controller *ctrl,
				struct slim_eaddr *ead, u8 laddr)
{
	struct msm_slim_ctrl *dev = slim_get_ctrldata(ctrl);
	u8 buf[7];
	int ret;
	struct slim_val_inf msg = {0};

	DEFINE_SLIM_EDEST_TXN(txn, SLIM_MSG_MC_ASSIGN_LOGICAL_ADDRESS,
			      10, laddr, &msg);

	/* Enumeration address */
	buf[0] = (u8)(ead->manf_id >> 8);
	buf[1] = (u8)(ead->manf_id & 0xFF);
	buf[2] = (u8) (ead->prod_code >> 8);
	buf[3] = (u8) (ead->prod_code & 0xFF);
	buf[4] = ead->dev_index;
	buf[5] = ead->instance;

	/* Logical address for this EA */
	buf[6] = laddr;

	/**
	 * Retries are needed since bus may lose sync when multiple devices
	 * are coming up and reporting present
	 */
	msg.wbuf = buf;
	msg.num_bytes = 7;

	ret = slim_processtxn(&dev->ctrl, &txn);

	if (ret)
		dev_err(dev->dev, "set LA:0x%x failed:ret:%d\n",
				  laddr, ret);
	return ret;
}

static void msm_slim_rxwq(struct work_struct *work)
{
	u8 buf[40];
	u8 mc, mt, len;
	int i, ret;
	struct msm_slim_ctrl *dev = container_of(work, struct msm_slim_ctrl,
						 wd);

	while ((slim_return_rx(&dev->ctrl, buf)) != -ENODATA) {
		len = buf[0] & 0x1F;
		mt = (buf[0] >> 5) & 0x7;
		mc = buf[1];
		if (mt == SLIM_MSG_MT_CORE &&
			mc == SLIM_MSG_MC_REPORT_PRESENT) {
			u8 laddr;
			struct slim_eaddr ea;
			u8 e_addr[6];

			for (i = 0; i < 6; i++)
				e_addr[i] = buf[7-i];

			ea.manf_id = (u16)(e_addr[5] << 8) | e_addr[4];
			ea.prod_code = (u16)(e_addr[3] << 8) | e_addr[2];
			ea.dev_index = e_addr[1];
			ea.instance = e_addr[0];
			ret = slim_assign_laddr(&dev->ctrl, &ea, &laddr, false);
			if (ret)
				dev_err(dev->dev, "assign laddr failed:%d\n",
					ret);
		} else {
			dev_err(dev->dev, "unexpected message:mc:%x, mt:%x\n",
				mc, mt);

		}

	}
}

static void msm_slim_prg_slew(struct platform_device *pdev,
				struct msm_slim_ctrl *dev)
{
	void __iomem *slew_reg;

	/* SLEW RATE register for this slimbus */
	dev->slew_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"slimbus_slew_reg");
	if (!dev->slew_mem) {
		dev_warn(&pdev->dev, "no slimbus slew resource\n");
		return;
	}

	slew_reg = devm_ioremap(&pdev->dev, dev->slew_mem->start,
				resource_size(dev->slew_mem));
	if (!slew_reg) {
		dev_err(dev->dev, "slew register mapping failed");
		release_mem_region(dev->slew_mem->start,
					resource_size(dev->slew_mem));
		dev->slew_mem = NULL;
		return;
	}
	writel_relaxed(1, slew_reg);
	/* Make sure slimbus-slew rate enabling goes through */
	wmb();
}

static int msm_slim_probe(struct platform_device *pdev)
{
	struct msm_slim_ctrl *dev;
	struct resource *slim_mem;
	struct resource *irq;
	struct clk *hclk, *rclk;
	int ret;

	hclk = devm_clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(hclk))
		return PTR_ERR(hclk);

	rclk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(rclk)) {
		/* unlikely that this is probe-defer */
		dev_err(&pdev->dev, "rclk get failed:%ld\n", PTR_ERR(rclk));
		return PTR_ERR(rclk);
	}

	ret = clk_set_rate(rclk, SLIM_ROOT_FREQ);
	if (ret) {
		dev_err(&pdev->dev, "ref-clock set-rate failed:%d\n", ret);
		return ret;
	}

	slim_mem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"slimbus_physical");
	if (!slim_mem) {
		dev_err(&pdev->dev, "no slimbus physical memory resource\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no slimbus IRQ resource\n");
		return -ENODEV;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->hclk = hclk;
	dev->rclk = rclk;

	dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, dev);
	slim_set_ctrldata(&dev->ctrl, dev);
	dev->base = devm_ioremap(dev->dev, slim_mem->start,
				 resource_size(slim_mem));
	if (!dev->base) {
		dev_err(&pdev->dev, "IOremap failed\n");
		return -ENOMEM;
	}

	dev->ctrl.set_laddr = msm_set_laddr;
	dev->ctrl.xfer_msg = msm_xfer_msg;
	dev->ctrl.tx.n = MSM_TX_MSGS;
	dev->ctrl.rx.n = MSM_RX_MSGS;
	dev->ctrl.tx.sl_sz = SLIM_MSGQ_BUF_LEN;
	dev->ctrl.rx.sl_sz = SLIM_MSGQ_BUF_LEN;

	dev->irq = irq->start;

	INIT_WORK(&dev->wd, msm_slim_rxwq);
	dev->rxwq = create_singlethread_workqueue("msm_slim_rx");
	if (!dev->rxwq) {
		dev_err(dev->dev, "Failed to start Rx WQ\n");
		return -ENOMEM;
	}

	dev->framer.rootfreq = SLIM_ROOT_FREQ >> 3;
	dev->framer.superfreq =
		dev->framer.rootfreq / SLIM_CL_PER_SUPERFRAME_DIV8;
	dev->ctrl.a_framer = &dev->framer;
	dev->ctrl.clkgear = SLIM_MAX_CLK_GEAR;
	dev->ctrl.dev.parent = &pdev->dev;
	dev->ctrl.dev.of_node = pdev->dev.of_node;

	msm_slim_prg_slew(pdev, dev);

	ret = devm_request_irq(&pdev->dev, dev->irq, msm_slim_interrupt,
				IRQF_TRIGGER_HIGH, "msm_slim_irq", dev);
	if (ret) {
		dev_err(&pdev->dev, "request IRQ failed\n");
		goto err_request_irq_failed;
	}

	ret = clk_prepare_enable(hclk);
	if (ret)
		goto err_hclk_enable_failed;

	ret = clk_prepare_enable(rclk);
	if (ret)
		goto err_rclk_enable_failed;

	/* Register with framework before enabling frame, clock */
	ret = slim_register_controller(&dev->ctrl);
	if (ret) {
		dev_err(dev->dev, "error adding controller\n");
		goto err_ctrl_failed;
	}

	dev->ver = readl_relaxed(dev->base);
	/* Version info in 16 MSbits */
	dev->ver >>= 16;
	/* Component register initialization */
	writel_relaxed(1, dev->base + CFG_PORT(COMP_CFG, dev->ver));
	writel_relaxed((EE_MGR_RSC_GRP | EE_NGD_2 | EE_NGD_1),
				dev->base + CFG_PORT(COMP_TRUST_CFG, dev->ver));

	writel_relaxed((MGR_INT_TX_NACKED_2 |
			MGR_INT_MSG_BUF_CONTE | MGR_INT_RX_MSG_RCVD |
			MGR_INT_TX_MSG_SENT), dev->base + MGR_INT_EN);
	writel_relaxed(1, dev->base + MGR_CFG);
	/*
	 * Framer registers are beyond 1K memory region after Manager and/or
	 * component registers. Make sure those writes are ordered
	 * before framer register writes
	 */
	wmb();

	/* Framer register initialization */
	writel_relaxed((1 << INTR_WAKE) | (0xA << REF_CLK_GEAR) |
		(0xA << CLK_GEAR) | (1 << ROOT_FREQ) | (1 << FRM_ACTIVE) | 1,
		dev->base + FRM_CFG);
	/*
	 * Make sure that framer wake-up and enabling writes go through
	 * before any other component is enabled. Framer is responsible for
	 * clocking the bus and enabling framer first will ensure that other
	 * devices can report presence when they are enabled
	 */
	mb();

	writel_relaxed(MGR_CFG_ENABLE, dev->base + MGR_CFG);
	/*
	 * Make sure that manager-enable is written through before interface
	 * device is enabled
	 */
	mb();
	writel_relaxed(1, dev->base + INTF_CFG);
	/*
	 * Make sure that interface-enable is written through before enabling
	 * ported generic device inside MSM manager
	 */
	mb();

	writel_relaxed(1, dev->base + CFG_PORT(COMP_CFG, dev->ver));
	/*
	 * Make sure that all writes have gone through before exiting this
	 * function
	 */
	mb();

	dev_dbg(dev->dev, "MSM SB controller is up:ver:0x%x!\n", dev->ver);
	return 0;

err_ctrl_failed:
	clk_disable_unprepare(dev->rclk);
err_rclk_enable_failed:
	clk_disable_unprepare(dev->hclk);
err_hclk_enable_failed:
err_request_irq_failed:
	destroy_workqueue(dev->rxwq);
	return ret;
}

static int msm_slim_remove(struct platform_device *pdev)
{
	struct msm_slim_ctrl *dev = platform_get_drvdata(pdev);

	disable_irq(dev->irq);
	clk_disable_unprepare(dev->rclk);
	clk_disable_unprepare(dev->hclk);
	slim_del_controller(&dev->ctrl);
	destroy_workqueue(dev->rxwq);
	return 0;
}

static const struct of_device_id msm_slim_dt_match[] = {
	{
		.compatible = "qcom,slim-msm",
	},
	{}
};

static struct platform_driver msm_slim_driver = {
	.probe = msm_slim_probe,
	.remove = msm_slim_remove,
	.driver	= {
		.name = MSM_SLIM_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_slim_dt_match,
	},
};
module_platform_driver(msm_slim_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("Qualcomm Slimbus controller");
MODULE_ALIAS("platform:qcom-slim");
