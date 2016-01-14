/*
 * Copyright (c) 2013 Eugene Krasnikov <k.eugene.e@gmail.com>
 * Copyright (c) 2013 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/remoteproc.h>
#include <linux/soc/qcom/smd.h>
#include "wcn36xx.h"
#include "wcnss_core.h"

#define MAC_ADDR_0 "wlan/macaddr0"

struct smd_packet_item {
	struct list_head list;
	void *buf;
	size_t count;
};

static int wcn36xx_msm_smsm_change_state(u32 clear_mask, u32 set_mask)
{
	return 0;
}

static int wcn36xx_msm_get_hw_mac(struct wcn36xx *wcn, u8 *addr)
{
	const struct firmware *addr_file = NULL;
	int status;
	u8 tmp[18];
	static const u8 qcom_oui[3] = {0x00, 0x0A, 0xF5};
	static const char *files = {MAC_ADDR_0};
	struct wcn36xx_platform_data *pdata = wcn->wcn36xx_data;

	status = request_firmware(&addr_file, files, &pdata->core->dev);

	if (status < 0) {
		/* Assign a random mac with Qualcomm oui */
		dev_err(&pdata->core->dev, "Failed (%d) to read macaddress"
			"file %s, using a random address instead", status, files);
		memcpy(addr, qcom_oui, 3);
		get_random_bytes(addr + 3, 3);
	} else {
		memset(tmp, 0, sizeof(tmp));
		memcpy(tmp, addr_file->data, sizeof(tmp) - 1);
		sscanf(tmp, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &addr[0],
		       &addr[1],
		       &addr[2],
		       &addr[3],
		       &addr[4],
		       &addr[5]);

		release_firmware(addr_file);
	}

	return 0;
}

static int wcn36xx_msm_smd_send_and_wait(struct wcn36xx *wcn, char *buf, size_t len)
{
	int ret = 0;
	struct wcn36xx_platform_data *pdata = wcn->wcn36xx_data;

	mutex_lock(&pdata->wlan_ctrl_lock);
	ret = qcom_smd_send(pdata->wlan_ctrl_channel, buf, len);
	if (ret) {
		dev_err(wcn->dev, "wlan ctrl channel tx failed\n");
	}
	mutex_unlock(&pdata->wlan_ctrl_lock);

	return ret;
}

static int wcn36xx_msm_smd_open(struct wcn36xx *wcn, void *rsp_cb)
{
	struct wcn36xx_platform_data *pdata = wcn->wcn36xx_data;

	pdata->cb = rsp_cb;
	return 0;
}

static void wcn36xx_msm_smd_close(struct wcn36xx *wcn)
{
	return;
}

static int wcn36xx_msm_get_chip_type(struct wcn36xx *wcn)
{
	struct wcn36xx_platform_data *pdata = wcn->wcn36xx_data;
	return pdata->chip_type;
}

static struct wcn36xx_platform_data wcn36xx_data = {
	.ctrl_ops = {
		.open = wcn36xx_msm_smd_open,
		.close = wcn36xx_msm_smd_close,
		.tx = wcn36xx_msm_smd_send_and_wait,
		.get_hw_mac = wcn36xx_msm_get_hw_mac,
		.smsm_change_state = wcn36xx_msm_smsm_change_state,
		.get_chip_type = wcn36xx_msm_get_chip_type,
	},
};

static void wlan_ctrl_smd_process(struct work_struct *worker)
{
	unsigned long flags;
	struct wcn36xx_platform_data *pdata =
		container_of(worker,
			struct wcn36xx_platform_data, packet_process_work);

	spin_lock_irqsave(&pdata->packet_lock, flags);
	while (!list_empty(&pdata->packet_list)) {
		struct smd_packet_item *packet;

		packet = list_first_entry(&pdata->packet_list,
				struct smd_packet_item, list);
		list_del(&packet->list);
		spin_unlock_irqrestore(&pdata->packet_lock, flags);
		pdata->cb(pdata->wcn, packet->buf, packet->count);
		kfree(packet->buf);
		spin_lock_irqsave(&pdata->packet_lock, flags);
	}
	spin_unlock_irqrestore(&pdata->packet_lock, flags);
}

static int qcom_smd_wlan_ctrl_probe(struct qcom_smd_device *sdev)
{
	pr_info("%s: enter\n", __func__);
        mutex_init(&wcn36xx_data.wlan_ctrl_lock);
        init_completion(&wcn36xx_data.wlan_ctrl_ack);

	wcn36xx_data.sdev = sdev;
	spin_lock_init(&wcn36xx_data.packet_lock);
	INIT_LIST_HEAD(&wcn36xx_data.packet_list);
	INIT_WORK(&wcn36xx_data.packet_process_work, wlan_ctrl_smd_process);

        dev_set_drvdata(&sdev->dev, &wcn36xx_data);
	wcn36xx_data.wlan_ctrl_channel = sdev->channel;

	of_platform_populate(sdev->dev.of_node, NULL, NULL, &sdev->dev);

        return 0;
}

static void qcom_smd_wlan_ctrl_remove(struct qcom_smd_device *sdev)
{
        of_platform_depopulate(&sdev->dev);
}

static int qcom_smd_wlan_ctrl_callback(struct qcom_smd_device *qsdev,
                                 const void *data,
                                 size_t count)
{
	unsigned long flags;
	struct smd_packet_item *packet = NULL;
	struct wcn36xx_platform_data *pdata = dev_get_drvdata(&qsdev->dev);
	void *buf = kzalloc(count + sizeof(struct smd_packet_item),
				GFP_ATOMIC);
	if (!buf) {
		dev_err(&pdata->core->dev, "can't allocate buffer\n");
		return -ENOMEM;
	}

	memcpy_fromio(buf, data, count);
	packet = buf + count;
	packet->buf = buf;
	packet->count = count;

	spin_lock_irqsave(&pdata->packet_lock, flags);
	list_add_tail(&packet->list, &pdata->packet_list);
	spin_unlock_irqrestore(&pdata->packet_lock, flags);
	schedule_work(&pdata->packet_process_work);

	/* buf will be freed in workqueue */

	return 0;
}

static const struct qcom_smd_id qcom_smd_wlan_ctrl_match[] = {
	{ .name = "WLAN_CTRL" },
	{}
};

static struct qcom_smd_driver qcom_smd_wlan_ctrl_driver = {
	.probe = qcom_smd_wlan_ctrl_probe,
	.remove = qcom_smd_wlan_ctrl_remove,
	.callback = qcom_smd_wlan_ctrl_callback,
	.smd_match_table = qcom_smd_wlan_ctrl_match,
	.driver  = {
		.name  = "qcom_smd_wlan_ctrl",
		.owner = THIS_MODULE,
	},
};

static const struct of_device_id wcn36xx_msm_match_table[] = {
	{ .compatible = "qcom,wcn3660", .data = (void *)WCN36XX_CHIP_3660 },
	{ .compatible = "qcom,wcn3680", .data = (void *)WCN36XX_CHIP_3680 },
	{ .compatible = "qcom,wcn3620", .data = (void *)WCN36XX_CHIP_3620 },
	{ }
};
MODULE_DEVICE_TABLE(of, wcn36xx_msm_match_table);

static int wcn36xx_msm_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *of_id;
	struct resource *r;
	struct resource res[3];
	static const char const *rnames[] = {
		"wcnss_mmio", "wcnss_wlantx_irq", "wcnss_wlanrx_irq" };
	static const int rtype[] = {
		IORESOURCE_MEM, IORESOURCE_IRQ, IORESOURCE_IRQ };
	struct device_node *dn;
	int n;

	wcnss_core_prepare(pdev);

	dn = of_parse_phandle(pdev->dev.of_node, "rproc", 0);
	if (!dn) {
		dev_err(&pdev->dev, "No rproc specified in DT\n");
	} else {
		struct rproc *rproc= rproc_get_by_phandle(dn->phandle);
		if (rproc)
			rproc_boot(rproc);
		else {
			dev_err(&pdev->dev, "No rproc registered\n");
		}
	}

	qcom_smd_driver_register(&qcom_smd_wlan_ctrl_driver);
	wcnss_core_init();

	of_id = of_match_node(wcn36xx_msm_match_table, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;

	wcn36xx_data.chip_type = (enum wcn36xx_chip_type)of_id->data;

	wcn36xx_data.core = platform_device_alloc("wcn36xx", -1);

	for (n = 0; n < ARRAY_SIZE(rnames); n++) {
		r = platform_get_resource_byname(pdev, rtype[n], rnames[n]);
		if (!r) {
			dev_err(&wcn36xx_data.core->dev,
				"Missing resource %s'\n", rnames[n]);
			ret = -ENOMEM;
			return ret;
		}
		res[n] = *r;
	}

	platform_device_add_resources(wcn36xx_data.core, res, n);
	wcn36xx_data.core->dev.platform_data = &wcn36xx_data;

	platform_device_add(wcn36xx_data.core);

	dev_info(&pdev->dev, "%s initialized\n", __func__);

	return 0;
}

static int wcn36xx_msm_remove(struct platform_device *pdev)
{
	platform_device_del(wcn36xx_data.core);
	platform_device_put(wcn36xx_data.core);
	return 0;
}

static struct platform_driver wcn36xx_msm_driver = {
	.probe		= wcn36xx_msm_probe,
	.remove		= wcn36xx_msm_remove,
	.driver		= {
		.name	= "wcn36xx-msm",
		.owner	= THIS_MODULE,
		.of_match_table = wcn36xx_msm_match_table,
	},
};

static int __init wcn36xx_msm_init(void)
{
	return platform_driver_register(&wcn36xx_msm_driver);
}
module_init(wcn36xx_msm_init);

static void __exit wcn36xx_msm_exit(void)
{
	platform_driver_unregister(&wcn36xx_msm_driver);
}
module_exit(wcn36xx_msm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eugene Krasnikov k.eugene.e@gmail.com");
MODULE_FIRMWARE(MAC_ADDR_0);

