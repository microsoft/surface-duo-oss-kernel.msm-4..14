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
#include <linux/workqueue.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/smsm.h>
#include "wcn36xx.h"

#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>

#define MAC_ADDR_0 "wlan/macaddr0"

static void *pil;

struct wcn36xx_msm {
	struct wcn36xx_platform_ctrl_ops ctrl_ops;
	struct platform_device *core;
	void *drv_priv;
	void (*rsp_cb)(void *drv_priv, void *buf, size_t len);
	/* SMD related */
	struct workqueue_struct	*wq;
	struct work_struct	smd_work;
	struct completion	smd_compl;
	smd_channel_t		*smd_ch;
	struct pinctrl *pinctrl;
	enum wcn36xx_chip_type chip_type;
};

static struct wcn36xx_msm wmsm;

static int wcn36xx_msm_smsm_change_state(u32 clear_mask, u32 set_mask)
{
	 return smsm_change_state(SMSM_APPS_STATE, clear_mask, set_mask);
}

static int wcn36xx_msm_get_hw_mac(u8 *addr)
{
	const struct firmware *addr_file = NULL;
	int status;
	u8 tmp[18];
	static const u8 qcom_oui[3] = {0x00, 0x0A, 0xF5};
	static const char *files = {MAC_ADDR_0};

	status = request_firmware(&addr_file, files, &wmsm.core->dev);

	if (status < 0) {
		/* Assign a random mac with Qualcomm oui */
		dev_err(&wmsm.core->dev, "Failed (%d) to read macaddress file %s, using a random address instead", status,
			     files);
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

static int wcn36xx_msm_smd_send_and_wait(char *buf, size_t len)
{
	int avail;
	int ret = 0;

	avail = smd_write_avail(wmsm.smd_ch);

	if (avail >= len) {
		avail = smd_write(wmsm.smd_ch, buf, len);
		if (avail != len) {
			dev_err(&wmsm.core->dev,
				"Cannot write to SMD channel\n");
			ret = -EAGAIN;
			goto out;
		}
	} else {
		dev_err(&wmsm.core->dev,
			"SMD channel can accept only %d bytes\n", avail);
		ret = -ENOMEM;
		goto out;
	}

out:
	return ret;
}

static void wcn36xx_msm_smd_notify(void *data, unsigned event)
{
	struct wcn36xx_msm *wmsm_priv = (struct wcn36xx_msm *)data;

	switch (event) {
	case SMD_EVENT_OPEN:
		complete(&wmsm_priv->smd_compl);
		break;
	case SMD_EVENT_DATA:
		queue_work(wmsm_priv->wq, &wmsm_priv->smd_work);
		break;
	case SMD_EVENT_CLOSE:
		break;
	case SMD_EVENT_STATUS:
		break;
	case SMD_EVENT_REOPEN_READY:
		break;
	default:
		dev_err(&wmsm_priv->core->dev,
			"%s: SMD_EVENT (%d) not supported\n", __func__, event);
		break;
	}
}

static void wcn36xx_msm_smd_work(struct work_struct *work)
{
	int avail;
	int msg_len;
	void *msg;
	int ret;
	struct wcn36xx_msm *wmsm_priv =
		container_of(work, struct wcn36xx_msm, smd_work);

	while (1) {
		msg_len = smd_cur_packet_size(wmsm_priv->smd_ch);
		if (0 == msg_len) {
			return;
		}
		avail = smd_read_avail(wmsm_priv->smd_ch);
		if (avail < msg_len) {
			return;
		}
		msg = kmalloc(msg_len, GFP_KERNEL);
		if (NULL == msg) {
			return;
		}

		ret = smd_read(wmsm_priv->smd_ch, msg, msg_len);
		if (ret != msg_len) {
			return;
		}
		wmsm_priv->rsp_cb(wmsm_priv->drv_priv, msg, msg_len);
		kfree(msg);
	}
}

int wcn36xx_msm_smd_open(void *drv_priv, void *rsp_cb)
{
	int ret, left;
	wmsm.drv_priv = drv_priv;
	wmsm.rsp_cb = rsp_cb;
	INIT_WORK(&wmsm.smd_work, wcn36xx_msm_smd_work);
	init_completion(&wmsm.smd_compl);

	wmsm.wq = create_workqueue("wcn36xx_msm_smd_wq");
	if (!wmsm.wq) {
		dev_err(&wmsm.core->dev, "failed to allocate wq");
		ret = -ENOMEM;
		return ret;
	}

	ret = smd_named_open_on_edge("WLAN_CTRL", SMD_APPS_WCNSS,
		&wmsm.smd_ch, &wmsm, wcn36xx_msm_smd_notify);
	if (ret) {
		dev_err(&wmsm.core->dev,
			"smd_named_open_on_edge failed: %d\n", ret);
		return ret;
	}

	left = wait_for_completion_interruptible_timeout(&wmsm.smd_compl,
		msecs_to_jiffies(HAL_MSG_TIMEOUT));
	if (left <= 0) {
		dev_err(&wmsm.core->dev,
			"timeout waiting for smd open: %d\n", ret);
		return left;
	}

	/* Not to receive INT until the whole buf from SMD is read */
	smd_disable_read_intr(wmsm.smd_ch);

	return 0;
}

void wcn36xx_msm_smd_close(void)
{
	smd_close(wmsm.smd_ch);
	flush_workqueue(wmsm.wq);
	destroy_workqueue(wmsm.wq);
}

int wcn36xx_msm_shutdown(const struct subsys_desc *desc, bool force_stop)
{
	return 0;
}
int wcn36xx_msm_powerup(const struct subsys_desc *desc)
{
	return 0;
}

static const struct of_device_id wcn36xx_msm_match_table[] = {
	{ .compatible = "qcom,wcn3660", .data = (void *)WCN36XX_CHIP_3660 },
	{ .compatible = "qcom,wcn3680", .data = (void *)WCN36XX_CHIP_3680 },
	{ .compatible = "qcom,wcn3620", .data = (void *)WCN36XX_CHIP_3620 },
	{ }
};

static int wcn36xx_msm_get_chip_type(void)
{
	return wmsm.chip_type;
}

static struct wcn36xx_msm wmsm = {
	.ctrl_ops = {
		.open = wcn36xx_msm_smd_open,
		.close = wcn36xx_msm_smd_close,
		.tx = wcn36xx_msm_smd_send_and_wait,
		.get_hw_mac = wcn36xx_msm_get_hw_mac,
		.smsm_change_state = wcn36xx_msm_smsm_change_state,
		.get_chip_type = wcn36xx_msm_get_chip_type,
	},
};

static int wcn36xx_msm_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *of_id;
	struct resource *r;
	struct resource res[3];
	struct pinctrl_state *ps;
	static const char const *rnames[] = {
		"wcnss_mmio", "wcnss_wlantx_irq", "wcnss_wlanrx_irq" };
	static const int rtype[] = {
		IORESOURCE_MEM, IORESOURCE_IRQ, IORESOURCE_IRQ };
	int n;

	of_id = of_match_node(wcn36xx_msm_match_table, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;

	wmsm.chip_type = (enum wcn36xx_chip_type)of_id->data;

	wmsm.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(wmsm.pinctrl))
		return PTR_ERR(wmsm.pinctrl);

	ps = pinctrl_lookup_state(wmsm.pinctrl, "wcnss_default");
	if (IS_ERR_OR_NULL(ps))
			return PTR_ERR(ps);

	ret = pinctrl_select_state(wmsm.pinctrl, ps);
	if (ret)
		return ret;

	if (IS_ERR_OR_NULL(pil))
		pil = subsystem_get("wcnss");
	if (IS_ERR_OR_NULL(pil))
		return PTR_ERR(pil);

	wmsm.core = platform_device_alloc("wcn36xx", -1);

	for (n = 0; n < ARRAY_SIZE(rnames); n++) {
		r = platform_get_resource_byname(pdev, rtype[n], rnames[n]);
		if (!r) {
			dev_err(&wmsm.core->dev,
				"Missing resource %s'\n", rnames[n]);
			ret = -ENOMEM;
			return ret;
		}
		res[n] = *r;
	}

	platform_device_add_resources(wmsm.core, res, n);

	ret = platform_device_add_data(wmsm.core, &wmsm.ctrl_ops,
				       sizeof(wmsm.ctrl_ops));
	if (ret) {
		dev_err(&wmsm.core->dev, "Can't add platform data\n");
		ret = -ENOMEM;
		return ret;
	}

	platform_device_add(wmsm.core);

	dev_info(&pdev->dev, "%s initialized\n", __func__);

	return 0;
}
static int wcn36xx_msm_remove(struct platform_device *pdev)
{
        struct pinctrl_state *ps;

	platform_device_del(wmsm.core);
	platform_device_put(wmsm.core);

	if (wmsm.pinctrl) {
		ps = pinctrl_lookup_state(wmsm.pinctrl, "wcnss_sleep");
		if (IS_ERR_OR_NULL(ps))
			return PTR_ERR(ps);

		pinctrl_select_state(wmsm.pinctrl, ps);
	}

	return 0;
}

MODULE_DEVICE_TABLE(of, wcn36xx_msm_match_table);

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
	if (pil)
		subsystem_put(pil);


}
module_exit(wcn36xx_msm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eugene Krasnikov k.eugene.e@gmail.com");
MODULE_FIRMWARE(MAC_ADDR_0);

