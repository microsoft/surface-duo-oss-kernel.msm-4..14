// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2011-2017, The Linux Foundation
 * Copyright (c) 2018, Linaro Limited
 */

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/soc/qcom/apr.h>
#include "q6dsp-errno.h"

#define ADSP_STATE_READY_TIMEOUT_MS    3000
#define Q6_READY_TIMEOUT_MS 100
#define AVCS_CMD_ADSP_EVENT_GET_STATE		0x0001290C
#define AVCS_CMDRSP_ADSP_EVENT_GET_STATE	0x0001290D
#define AVCS_GET_VERSIONS       0x00012905
#define AVCS_GET_VERSIONS_RSP   0x00012906
#define AVCS_CMD_GET_FWK_VERSION	0x001292c
#define AVCS_CMDRSP_GET_FWK_VERSION	0x001292d


struct avcs_svc_info {
	uint32_t service_id;
	uint32_t version;
} __packed;

struct avcs_cmdrsp_get_version {
	uint32_t build_id;
	uint32_t num_services;
	struct avcs_svc_info svc_api_info[];
} __packed;

/* for ADSP2.8 and above */
struct avcs_svc_api_info {
	uint32_t service_id;
	uint32_t api_version;
	uint32_t api_branch_version;
} __packed;

struct avcs_cmdrsp_get_fwk_version {
	uint32_t build_major_version;
	uint32_t build_minor_version;
	uint32_t build_branch_version;
	uint32_t build_subbranch_version;
	uint32_t num_services;
	struct avcs_svc_api_info svc_api_info[];
} __packed;

struct q6core {
	struct apr_device *adev;
	wait_queue_head_t wait;
	uint32_t avcs_state;
	bool resp_received;
	uint32_t num_services;
	struct avcs_cmdrsp_get_fwk_version *fwk_version;
	struct avcs_cmdrsp_get_version *svc_version;
	bool fwk_version_supported;
	bool get_state_supported;
	bool get_version_supported;
};

struct q6core *g_core;

static int q6core_callback(struct apr_device *adev,
			 struct apr_client_message *data)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);
	struct aprv2_ibasic_rsp_result_t *result;

	result = data->payload;
	switch (data->opcode) {
	case APR_BASIC_RSP_RESULT:{
		result = data->payload;
		switch (result->opcode) {
		case AVCS_GET_VERSIONS:
			if (result->status == ADSP_EUNSUPPORTED)
				core->get_version_supported = false;
			core->resp_received = true;
			break;
		case AVCS_CMD_GET_FWK_VERSION:
			if (result->status == ADSP_EUNSUPPORTED)
				core->fwk_version_supported = false;
			core->resp_received = true;
			break;
		case AVCS_CMD_ADSP_EVENT_GET_STATE:
			if (result->status == ADSP_EUNSUPPORTED)
				core->get_state_supported = false;
			core->resp_received = true;
			break;
		}
		break;
	}
	case AVCS_CMDRSP_GET_FWK_VERSION: {
		struct avcs_cmdrsp_get_fwk_version *fwk;
		int bytes;
		fwk = data->payload;

		core->fwk_version_supported = true;
		bytes = sizeof(*fwk) + fwk->num_services *
				sizeof(fwk->svc_api_info[0]);

		core->fwk_version = kzalloc(bytes, GFP_ATOMIC);
		if (!core->fwk_version)
			return -ENOMEM;

		memcpy(core->fwk_version, data->payload, bytes);

		core->resp_received = true;

		break;
	}
	case AVCS_GET_VERSIONS_RSP: {
		struct avcs_cmdrsp_get_version *v;
		int len;
		v = data->payload;
		core->get_version_supported = true;

		len = sizeof(*v) + v->num_services * sizeof(v->svc_api_info[0]);

		core->svc_version = kzalloc(len, GFP_ATOMIC);
		if (!core->svc_version)
			return -ENOMEM;

		memcpy(core->svc_version, data->payload, len);

		core->resp_received = true;

		break;
	}
	case AVCS_CMDRSP_ADSP_EVENT_GET_STATE:
		core->get_state_supported = true;
		core->avcs_state = result->opcode;

		core->resp_received = true;
		break;
	default:
		dev_err(&adev->dev, "Message id from adsp core svc: 0x%x\n",
			data->opcode);
		break;
	}

	if (core->resp_received)
		wake_up(&core->wait);

	return 0;
}

static int q6core_get_fwk_versions(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_hdr hdr = {0};
	int rc;

	core->fwk_version_supported = true;
	hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr.pkt_size = APR_HDR_SIZE;
	hdr.opcode = AVCS_CMD_GET_FWK_VERSION;

	rc = apr_send_pkt(adev, &hdr);
	if (rc < 0)
		return rc;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;
		return 0;
	}


	return rc;
}

static int q6core_get_svc_versions(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_hdr hdr = {0};
	int rc;

	core->get_version_supported = true;
	hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr.pkt_size = APR_HDR_SIZE;
	hdr.opcode = AVCS_GET_VERSIONS;

	rc = apr_send_pkt(adev, &hdr);
	if (rc < 0)
		return rc;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;
		return 0;
	}

	return rc;
}

static bool __q6core_is_adsp_ready(struct q6core *core)
{
	struct apr_device *adev = core->adev;
	struct apr_hdr hdr = {0};
	int rc;

	core->get_state_supported = false;

	hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr.pkt_size = APR_HDR_SIZE;
	hdr.opcode = AVCS_CMD_ADSP_EVENT_GET_STATE;

	rc = apr_send_pkt(adev, &hdr);
	if (rc < 0)
		return false;

	rc = wait_event_timeout(core->wait, (core->resp_received),
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS));
	if (rc > 0 && core->resp_received) {
		core->resp_received = false;

		if (core->avcs_state == 0x1)
			return true;
	}

	/* assume that the adsp is up if we not support this command */
	if (!core->get_state_supported)
		return true;

	return false;
}

/**
 * q6core_get_svc_version() - Get version number of a service.
 *
 * @svc_id: service id of the service.
 *
 * Return: Will be a valid version number on success and zero on failure.
 * version number returned contains bits 0 to 15 as Minor version number
 * Bits 16 to 31 as Major version number
 */
uint32_t q6core_get_svc_version(int svc_id)
{
	int i;

	if (!g_core)
		return 0;

	if (g_core->fwk_version_supported) {
		for (i = 0; i < g_core->fwk_version->num_services; i++) {
			struct avcs_svc_api_info *info;

			info = &g_core->fwk_version->svc_api_info[i];
			if (svc_id == info->service_id)
				return info->api_version;
		}
	} else if (g_core->get_version_supported) {
		for (i = 0; i < g_core->svc_version->num_services; i++) {
			struct avcs_svc_info *info;

			info = &g_core->svc_version->svc_api_info[i];
			pr_err("SVC API Info SVC-ID: %x vs %x API-VER: %x\n",
					info->service_id, svc_id,  info->version);

			if (svc_id == info->service_id)
				return info->version;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(q6core_get_svc_version);

/**
 * q6core_is_adsp_ready() - Get status of adsp
 *
 * Return: Will be an true if adsp is ready and false if not.
 */
bool q6core_is_adsp_ready(void)
{
	unsigned long  timeout;

	if (!g_core)
		return false;

	timeout = jiffies + msecs_to_jiffies(ADSP_STATE_READY_TIMEOUT_MS);
	for (;;) {
		if (__q6core_is_adsp_ready(g_core))
			return true;

		if (!time_after(timeout, jiffies))
			return false;
	}

	return false;
}
EXPORT_SYMBOL_GPL(q6core_is_adsp_ready);

static int q6core_probe(struct apr_device *adev)
{
	g_core = kzalloc(sizeof(*g_core), GFP_KERNEL);
	if (!g_core)
		return -ENOMEM;

	dev_set_drvdata(&adev->dev, g_core);

	g_core->adev = adev;
	init_waitqueue_head(&g_core->wait);

	/* get adsp state */
	if (q6core_is_adsp_ready()) {
		q6core_get_fwk_versions(g_core);
		if (!g_core->fwk_version_supported)
			q6core_get_svc_versions(g_core);
	}

	return 0;
}

static int q6core_exit(struct apr_device *adev)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);

	if (core->fwk_version_supported)
		kfree(core->fwk_version);
	if (core->get_version_supported)
		kfree(core->svc_version);

	kfree(core);
	g_core = NULL;

	return 0;
}

static const struct of_device_id q6core_device_id[]  = {
	{ .compatible = "qcom,q6core" },
	{},
};
MODULE_DEVICE_TABLE(of, q6core_device_id);

static struct apr_driver qcom_q6core_driver = {
	.probe = q6core_probe,
	.remove = q6core_exit,
	.callback = q6core_callback,
	.driver = {
		.name = "qcom-q6core",
		.of_match_table = of_match_ptr(q6core_device_id),
	},
};

module_apr_driver(qcom_q6core_driver);
MODULE_DESCRIPTION("q6 core");
MODULE_LICENSE("GPL v2");
