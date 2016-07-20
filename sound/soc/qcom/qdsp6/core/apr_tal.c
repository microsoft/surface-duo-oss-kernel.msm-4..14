/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <sound/qdsp6v2/apr_tal.h>
#include <linux/soc/qcom/smd.h>
#include <linux/io.h>

struct apr_svc_ch_dev apr_svc_ch[APR_DL_MAX][APR_DEST_MAX][APR_CLIENT_MAX];

int apr_tal_write(struct apr_svc_ch_dev *apr_ch, void *data, int len)
{
	int ret;
	ret = qcom_smd_send(apr_ch->ch, data, len);
	if (ret) { 
		pr_err("apr_tal: Error in write %d\n", ret);
		return ret;;
	}
	return len;
}

struct apr_svc_ch_dev *apr_tal_open(uint32_t svc, uint32_t dest,
				uint32_t dl, apr_svc_cb_fn func, void *priv)
{
	int rc;
	pr_err("apr_tal:open\n");
	if ((svc >= APR_CLIENT_MAX) || (dest >= APR_DEST_MAX) ||
						(dl >= APR_DL_MAX)) {
		pr_err("apr_tal: Invalid params\n");
		return NULL;
	}

	if (apr_svc_ch[dl][dest][svc].ch) {
		pr_err("apr_tal: This channel alreday openend\n");
		return NULL;
	}

	if (!apr_svc_ch[dl][dest][svc].dest_state) {
		rc = wait_event_timeout(apr_svc_ch[dl][dest][svc].dest,
			apr_svc_ch[dl][dest][svc].dest_state,
				msecs_to_jiffies(APR_OPEN_TIMEOUT_MS));
		if (rc == 0) {
			pr_err("apr_tal:open timeout\n");
			return NULL;
		}
		pr_info("apr_tal:Wakeup done\n");
	}
	apr_svc_ch[dl][dest][svc].func = func;
	apr_svc_ch[dl][dest][svc].priv = priv;

	pr_info("apr_tal:apr svc init done\n");

	return &apr_svc_ch[dl][dest][svc];
}

int apr_tal_close(struct apr_svc_ch_dev *apr_ch)
{
	if (!apr_ch->ch)
		return -EINVAL;

	apr_ch->ch = NULL;
	apr_ch->func = NULL;
	apr_ch->priv = NULL;
	return 0;
}


static int qcom_smd_q6_callback(struct qcom_smd_channel *channel,
				 const void *data,
				 size_t count)
{
	struct apr_svc_ch_dev *apr_ch = qcom_smd_get_drvdata(channel);

	memcpy(apr_ch->data, data, count);

	if (apr_ch->func)
		apr_ch->func(apr_ch->data, count, apr_ch->priv);

	return 0;
}

static int qcom_smd_q6_probe(struct qcom_smd_device *sdev)
{
	struct apr_svc_ch_dev *apr = &apr_svc_ch[APR_DL_SMD][APR_DEST_QDSP6][APR_CLIENT_AUDIO];

	pr_info("apr_tal:Q6 Is Up\n");

	qcom_smd_set_drvdata(sdev->channel, apr);

	apr->ch = sdev->channel;
	apr->dest_state = 1;
	wake_up(&apr->dest);

	return 0;
}

static void qcom_smd_q6_remove(struct qcom_smd_device *sdev)
{
	struct apr_svc_ch_dev *apr = &apr_svc_ch[APR_DL_SMD][APR_DEST_QDSP6][APR_CLIENT_AUDIO];

	apr->ch = NULL;
	apr->dest_state = 0;
}


static const struct of_device_id qcom_smd_q6_of_match[] = {
	{ .compatible = "qcom,apr" },
	{}
};

static struct qcom_smd_driver qcom_smd_q6_driver = {
	.probe = qcom_smd_q6_probe,
	.remove = qcom_smd_q6_remove,
	.callback = qcom_smd_q6_callback,
	.driver  = {
		.name  = "qcom_smd_q6",
		.owner = THIS_MODULE,
		.of_match_table = qcom_smd_q6_of_match,
	},
};

static void __exit qcom_smd_q6_exit(void)
{
	qcom_smd_driver_unregister(&qcom_smd_q6_driver);
}
module_exit(qcom_smd_q6_exit);

static int __init apr_tal_init(void)
{

	int i, j, k;

	for (i = 0; i < APR_DL_MAX; i++)
		for (j = 0; j < APR_DEST_MAX; j++)
			for (k = 0; k < APR_CLIENT_MAX; k++) {
				init_waitqueue_head(&apr_svc_ch[i][j][k].wait);
				init_waitqueue_head(&apr_svc_ch[i][j][k].dest);
			}
	qcom_smd_driver_register(&qcom_smd_q6_driver);
	return 0;
}
device_initcall(apr_tal_init);

MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("Qualcomm SMD backed apr driver");
MODULE_LICENSE("GPL v2");
