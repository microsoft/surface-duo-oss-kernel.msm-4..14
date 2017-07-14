// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2011-2017, The Linux Foundation
// Copyright (c) 2018, Linaro Limited

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/kref.h>
#include <linux/wait.h>
#include <linux/soc/qcom/apr.h>
#include <linux/platform_device.h>
#include <sound/asound.h>
#include "q6adm.h"
#include "q6afe.h"
#include "q6core.h"
#include "q6dsp-errno.h"
#include "q6dsp-common.h"

#define ADM_CMD_DEVICE_OPEN_V5		0x00010326
#define ADM_CMDRSP_DEVICE_OPEN_V5	0x00010329
#define ADM_CMD_DEVICE_CLOSE_V5		0x00010327
#define ADM_CMD_MATRIX_MAP_ROUTINGS_V5	0x00010325

#define TIMEOUT_MS 1000
#define RESET_COPP_ID 99
#define INVALID_COPP_ID 0xFF
/* Definition for a legacy device session. */
#define ADM_LEGACY_DEVICE_SESSION	0
#define ADM_MATRIX_ID_AUDIO_RX		0
#define ADM_MATRIX_ID_AUDIO_TX		1

struct copp {
	int afe_port;
	int copp_idx;
	int id;
	int topology;
	int mode;
	int rate;
	int bit_width;
	int channels;
	int app_type;
	int acdb_id;

	struct aprv2_ibasic_rsp_result_t result;
	struct kref refcount;
	wait_queue_head_t wait;
	struct list_head node;
	struct q6adm *adm;
};

struct q6adm {
	struct apr_device *apr;
	struct device *dev;
	struct q6core_svc_api_info ainfo;
	unsigned long copp_bitmap[AFE_MAX_PORTS];
	struct list_head copps_list;
	spinlock_t copps_list_lock;
	struct aprv2_ibasic_rsp_result_t result;
	struct mutex lock;
	wait_queue_head_t matrix_map_wait;
	void *routing_data;
	struct platform_device *pdev_routing;
};

struct adm_cmd_device_open_v5 {
	struct apr_hdr hdr;
	u16 flags;
	u16 mode_of_operation;
	u16 endpoint_id_1;
	u16 endpoint_id_2;
	u32 topology_id;
	u16 dev_num_channel;
	u16 bit_width;
	u32 sample_rate;
	u8 dev_channel_mapping[8];
} __packed;

struct adm_cmd_matrix_map_routings_v5 {
	struct apr_hdr hdr;
	u32 matrix_id;
	u32 num_sessions;
} __packed;

struct adm_session_map_node_v5 {
	u16 session_id;
	u16 num_copps;
} __packed;

static struct copp *adm_find_copp(struct q6adm *adm, int port_idx,
				  int copp_idx)
{
	struct copp *c = NULL;
	unsigned long flags;

	spin_lock_irqsave(&adm->copps_list_lock, flags);
	list_for_each_entry(c, &adm->copps_list, node) {
		if ((port_idx == c->afe_port) && (copp_idx == c->copp_idx))
			break;
	}

	spin_unlock_irqrestore(&adm->copps_list_lock, flags);

	return c;

}

static int q6adm_callback(struct apr_device *adev,
			struct apr_client_message *data)
{
	struct aprv2_ibasic_rsp_result_t *result = data->payload;
	int port_idx, copp_idx;
	struct copp *copp;
	struct q6adm *adm = dev_get_drvdata(&adev->dev);

	if (!data->payload_size)
		return 0;

	copp_idx = (data->token) & 0XFF;
	port_idx = ((data->token) >> 16) & 0xFF;
	if (port_idx < 0 || port_idx >= AFE_MAX_PORTS) {
		dev_err(&adev->dev, "Invalid port idx %d token %d\n",
		       port_idx, data->token);
		return 0;
	}
	if (copp_idx < 0 || copp_idx >= MAX_COPPS_PER_PORT) {
		dev_err(&adev->dev, "Invalid copp idx %d token %d\n",
			copp_idx, data->token);
		return 0;
	}

	switch (data->opcode) {
	case APR_BASIC_RSP_RESULT: {
		if (result->status != 0) {
			dev_err(&adev->dev, "cmd = 0x%x return error = 0x%x\n",
				result->opcode, result->status);
		}
		switch (result->opcode) {
		case ADM_CMD_DEVICE_OPEN_V5:
		case ADM_CMD_DEVICE_CLOSE_V5:
			copp = adm_find_copp(adm, port_idx, copp_idx);
			if (!copp)
				return 0;

			copp->result = *result;
			wake_up(&copp->wait);
			break;
		case ADM_CMD_MATRIX_MAP_ROUTINGS_V5:
			adm->result = *result;
			wake_up(&adm->matrix_map_wait);
			break;

		default:
			dev_err(&adev->dev, "Unknown Cmd: 0x%x\n",
				result->opcode);
			break;
		}
		return 0;
	}
	case ADM_CMDRSP_DEVICE_OPEN_V5: {
		struct adm_cmd_rsp_device_open_v5 {
			u32 status;
			u16 copp_id;
			u16 reserved;
		} __packed * open = data->payload;

		open = data->payload;
		copp = adm_find_copp(adm, port_idx, copp_idx);
		if (!copp)
			return 0;

		if (open->copp_id == INVALID_COPP_ID) {
			dev_err(&adev->dev, "Invalid coppid rxed %d\n",
				open->copp_id);
			copp->result.status = ADSP_EBADPARAM;
			wake_up(&copp->wait);
			break;
		}
		copp->result.opcode = data->opcode;
		copp->id = open->copp_id;
		wake_up(&copp->wait);
	}
	break;
	default:
		dev_err(&adev->dev, "Unknown cmd:0x%x\n",
		       data->opcode);
		break;
	}

	return 0;
}

static struct copp *adm_alloc_copp(struct q6adm *adm, int port_idx)
{
	struct copp *c;
	int idx;

	idx = find_first_zero_bit(&adm->copp_bitmap[port_idx],
				  MAX_COPPS_PER_PORT);

	if (idx > MAX_COPPS_PER_PORT)
		return ERR_PTR(-EBUSY);

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return ERR_PTR(-ENOMEM);

	set_bit(idx, &adm->copp_bitmap[port_idx]);
	c->copp_idx = idx;
	c->afe_port = port_idx;
	c->adm = adm;

	init_waitqueue_head(&c->wait);

	return c;
}

static int q6adm_apr_send_copp_pkt(struct q6adm *adm, struct copp *copp,
				   void *data, uint32_t rsp_opcode)
{
	struct device *dev = adm->dev;
	struct apr_hdr *hdr = data;
	uint32_t opcode = hdr->opcode;
	int ret;

	mutex_lock(&adm->lock);
	copp->result.opcode = 0;
	copp->result.status = 0;
	ret = apr_send_pkt(adm->apr, data);
	if (ret < 0) {
		dev_err(dev, "Failed to send APR packet\n");
		ret = -EINVAL;
		goto err;
	}

	/* Wait for the callback with copp id */
	if (rsp_opcode)
		ret = wait_event_timeout(copp->wait,
					 (copp->result.opcode == opcode) ||
					 (copp->result.opcode == rsp_opcode),
					 msecs_to_jiffies(TIMEOUT_MS));
	else
		ret = wait_event_timeout(copp->wait,
					 (copp->result.opcode == opcode),
					 msecs_to_jiffies(TIMEOUT_MS));

	if (!ret) {
		dev_err(dev, "ADM copp cmd timedout\n");
		ret = -EINVAL;
	} else if (copp->result.status > 0) {
		dev_err(dev, "DSP returned error[%d]\n",
			copp->result.status);
		ret = -EINVAL;
	}

err:
	mutex_unlock(&adm->lock);
	return ret;
}

static int q6adm_device_close(struct q6adm *adm, struct copp *copp,
			      int port_id, int copp_idx)
{
	struct apr_hdr close = {0};

	close.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					APR_HDR_LEN(APR_HDR_SIZE),
					APR_PKT_VER);
	close.pkt_size = sizeof(close);
	close.src_svc = APR_SVC_ADM;
	close.src_domain = APR_DOMAIN_APPS;
	close.src_port = port_id;
	close.dest_svc = APR_SVC_ADM;
	close.dest_domain = APR_DOMAIN_ADSP;
	close.dest_port = copp->id;
	close.token = port_id << 16 | copp_idx;
	close.opcode = ADM_CMD_DEVICE_CLOSE_V5;

	return q6adm_apr_send_copp_pkt(adm, copp, &close, 0);
}

static void adm_free_copp(struct kref *ref)
{
	struct copp *c = container_of(ref, struct copp, refcount);
	struct q6adm *adm = c->adm;
	int port_idx = c->afe_port;
	int copp_idx = c->copp_idx;
	unsigned long flags;
	int ret;

	ret = q6adm_device_close(adm, c, port_idx, copp_idx);
	if (ret < 0)
		dev_err(adm->dev, "Failed to close copp %d\n", ret);

	clear_bit(c->copp_idx, &adm->copp_bitmap[port_idx]);
	spin_lock_irqsave(&adm->copps_list_lock, flags);
	list_del(&c->node);
	spin_unlock_irqrestore(&adm->copps_list_lock, flags);
	kfree(c);
}

static struct copp *adm_find_matching_copp(struct q6adm *adm,
					   int port_id, int topology,
					   int mode, int rate, int channel_mode,
					   int bit_width, int app_type)
{
	unsigned long flags;
	struct copp *c;

	spin_lock_irqsave(&adm->copps_list_lock, flags);

	list_for_each_entry(c, &adm->copps_list, node) {
		if ((port_id == c->afe_port) && (topology == c->topology) &&
		    (mode == c->mode) && (rate == c->rate) &&
		    (bit_width == c->bit_width) && (app_type == c->app_type)) {
			spin_unlock_irqrestore(&adm->copps_list_lock, flags);
			return c;
		}
	}
	spin_unlock_irqrestore(&adm->copps_list_lock, flags);

	c = adm_alloc_copp(adm, port_id);
	if (IS_ERR_OR_NULL(c))
		return ERR_CAST(c);

	kref_init(&c->refcount);
	c->topology = topology;
	c->mode = mode;
	c->rate = rate;
	c->channels = channel_mode;
	c->bit_width = bit_width;
	c->app_type = app_type;

	spin_lock_irqsave(&adm->copps_list_lock, flags);
	list_add_tail(&c->node, &adm->copps_list);
	spin_unlock_irqrestore(&adm->copps_list_lock, flags);

	return c;

}

static int q6adm_device_open(struct q6adm *adm, struct copp *copp, int port_id,
			     int path, int topology, int channel_mode,
			     int bit_width, int rate)
{
	struct adm_cmd_device_open_v5 open = {0,};
	int afe_port = q6afe_get_port_id(port_id);
	int ret;

	open.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					   APR_HDR_LEN(APR_HDR_SIZE),
					   APR_PKT_VER);
	open.hdr.pkt_size = sizeof(open);
	open.hdr.src_svc = APR_SVC_ADM;
	open.hdr.src_domain = APR_DOMAIN_APPS;
	open.hdr.src_port = afe_port;
	open.hdr.dest_svc = APR_SVC_ADM;
	open.hdr.dest_domain = APR_DOMAIN_ADSP;
	open.hdr.dest_port = afe_port;
	open.hdr.token = port_id << 16 | copp->copp_idx;
	open.hdr.opcode = ADM_CMD_DEVICE_OPEN_V5;
	open.flags = ADM_LEGACY_DEVICE_SESSION;
	open.mode_of_operation = path;
	open.endpoint_id_1 = afe_port;
	open.topology_id = topology;
	open.dev_num_channel = channel_mode & 0x00FF;
	open.bit_width = bit_width;
	open.sample_rate = rate;

	ret = q6dsp_map_channels(&open.dev_channel_mapping[0],
				 channel_mode);
	if (ret)
		return ret;

	return q6adm_apr_send_copp_pkt(adm, copp, &open,
				       ADM_CMDRSP_DEVICE_OPEN_V5);
}

/**
 * q6adm_open() - open adm and grab a free copp
 *
 * @dev: Pointer to adm child device.
 * @port_id: port id
 * @path: playback or capture path.
 * @rate: rate at which copp is required.
 * @channel_mode: channel mode
 * @topology: adm topology id
 * @perf_mode: performace mode.
 * @bit_width: audio sample bit width
 * @app_type: Application type.
 * @acdb_id: ACDB id
 *
 * Return: Will be an negative on error or a valid copp index on success.
 */
int q6adm_open(struct device *dev, int port_id, int path, int rate,
	       int channel_mode, int topology, int perf_mode,
	       uint16_t bit_width, int app_type, int acdb_id)
{
	struct q6adm *adm = dev_get_drvdata(dev->parent);
	struct copp *copp;
	int ret = 0;

	if (port_id < 0) {
		dev_err(dev, "Invalid port_id 0x%x\n", port_id);
		return -EINVAL;
	}

	copp = adm_find_matching_copp(adm, port_id, topology, perf_mode,
				      rate, channel_mode, bit_width, app_type);
	if (kref_read(&copp->refcount) == 1) {
		/* not initialized yet */
		ret = q6adm_device_open(adm, copp, port_id, path, topology,
				  channel_mode, bit_width, rate);
		if (ret < 0)
			return ret;
	}

	kref_get(&copp->refcount);

	return copp->copp_idx;
}
EXPORT_SYMBOL_GPL(q6adm_open);

/**
 * q6adm_set_routing_data() - set routing private data
 *
 * @dev: Pointer to adm device.
 * @data: routing private data
 *
 */
void q6adm_set_routing_data(struct device *dev, void *data)
{
	struct q6adm *adm = dev_get_drvdata(dev->parent);

	adm->routing_data = data;
}
EXPORT_SYMBOL_GPL(q6adm_set_routing_data);

/**
 * q6adm_get_routing_data() - get routing private data
 *
 * @dev: Pointer to adm device.
 *
 * Return: pointer to routing private data
 */
void *q6adm_get_routing_data(struct device *dev)
{
	struct q6adm *adm = dev_get_drvdata(dev->parent);

	return adm->routing_data;
}
EXPORT_SYMBOL_GPL(q6adm_get_routing_data);

/**
 * q6adm_matrix_map() - Map asm streams and afe ports using payload
 *
 * @dev: Pointer to adm child device.
 * @path: playback or capture path.
 * @payload_map: map between session id and afe ports.
 * @perf_mode: Performace mode.
 *
 * Return: Will be an negative on error or a zero on success.
 */
int q6adm_matrix_map(struct device *dev, int path,
		     struct route_payload payload_map, int perf_mode)
{
	struct q6adm *adm = dev_get_drvdata(dev->parent);
	struct adm_cmd_matrix_map_routings_v5 *route;
	struct adm_session_map_node_v5 *node;
	uint16_t *copps_list;
	int cmd_size, ret, i, copp_idx;
	void *matrix_map = NULL;
	struct copp *copp;

	/* Assumes port_ids have already been validated during adm_open */
	cmd_size = (sizeof(*route) +
		    sizeof(*node) +
		    (sizeof(uint32_t) * payload_map.num_copps));
	matrix_map = kzalloc(cmd_size, GFP_KERNEL);
	if (!matrix_map)
		return -ENOMEM;

	route = matrix_map;
	route->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
					     APR_HDR_LEN(APR_HDR_SIZE),
					     APR_PKT_VER);
	route->hdr.pkt_size = cmd_size;
	route->hdr.src_svc = 0;
	route->hdr.src_domain = APR_DOMAIN_APPS;
	route->hdr.dest_svc = APR_SVC_ADM;
	route->hdr.dest_domain = APR_DOMAIN_ADSP;
	route->hdr.token = 0;
	route->hdr.opcode = ADM_CMD_MATRIX_MAP_ROUTINGS_V5;
	route->num_sessions = 1;

	switch (path) {
	case ADM_PATH_PLAYBACK:
		route->matrix_id = ADM_MATRIX_ID_AUDIO_RX;
		break;
	case ADM_PATH_LIVE_REC:
		route->matrix_id = ADM_MATRIX_ID_AUDIO_TX;
		break;
	default:
		dev_err(dev, "Wrong path set[%d]\n", path);

		break;
	}

	node = matrix_map + sizeof(*route);
	node->session_id = payload_map.session_id;
	node->num_copps = payload_map.num_copps;
	copps_list = matrix_map + sizeof(*route) + sizeof(*node);

	for (i = 0; i < payload_map.num_copps; i++) {
		int port_idx = payload_map.port_id[i];

		if (port_idx < 0) {
			dev_err(dev, "Invalid port_id 0x%x\n",
				payload_map.port_id[i]);
			ret = -EINVAL;
			goto fail_cmd;
		}
		copp_idx = payload_map.copp_idx[i];

		copp = adm_find_copp(adm, port_idx, copp_idx);
		if (!copp) {
			ret = -EINVAL;
			goto fail_cmd;
		}

		copps_list[i] = copp->id;
	}

	mutex_lock(&adm->lock);
	adm->result.status = 0;
	adm->result.opcode = 0;

	ret = apr_send_pkt(adm->apr, matrix_map);
	if (ret < 0) {
		dev_err(dev, "routing for syream %d failed ret %d\n",
		       payload_map.session_id, ret);
		goto fail_cmd;
	}
	ret = wait_event_timeout(adm->matrix_map_wait,
				 adm->result.opcode == route->hdr.opcode,
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		dev_err(dev, "routing for syream %d failed\n",
		       payload_map.session_id);
		ret = -ETIMEDOUT;
		goto fail_cmd;
	} else if (adm->result.status > 0) {
		dev_err(dev, "DSP returned error[%d]\n",
		        adm->result.status);
		ret = -EINVAL;
		goto fail_cmd;
	}

fail_cmd:
	mutex_unlock(&adm->lock);
	kfree(matrix_map);
	return ret;
}
EXPORT_SYMBOL_GPL(q6adm_matrix_map);

/**
 * q6adm_close() - Close adm copp
 *
 * @dev: Pointer to adm child device.
 * @port_id: afe port id.
 * @perf_mode: perf_mode mode
 * @copp_idx: copp index to close
 *
 * Return: Will be an negative on error or a zero on success.
 */
int q6adm_close(struct device *dev, int port_id, int perf_mode, int copp_idx)
{
	struct q6adm *adm = dev_get_drvdata(dev->parent);
	struct copp *copp;

	if (port_id < 0) {
		dev_err(dev, "Invalid port_id 0x%x\n", port_id);
		return -EINVAL;
	}

	if ((copp_idx < 0) || (copp_idx >= MAX_COPPS_PER_PORT)) {
		dev_err(dev, "Invalid copp idx: %d\n", copp_idx);
		return -EINVAL;
	}

	copp = adm_find_copp(adm, port_id, copp_idx);
	if (!copp)
		return -EINVAL;

	kref_put(&copp->refcount, adm_free_copp);

	return 0;
}
EXPORT_SYMBOL_GPL(q6adm_close);

static int q6adm_probe(struct apr_device *adev)
{
	struct device *dev = &adev->dev;
	struct device_node *dais_np;
	struct q6adm *adm;

	adm = devm_kzalloc(&adev->dev, sizeof(*adm), GFP_KERNEL);
	if (!adm)
		return -ENOMEM;

	adm->apr = adev;
	dev_set_drvdata(&adev->dev, adm);
	adm->dev = dev;
	q6core_get_svc_api_info(adev->svc_id, &adm->ainfo);
	mutex_init(&adm->lock);
	init_waitqueue_head(&adm->matrix_map_wait);

	INIT_LIST_HEAD(&adm->copps_list);
	spin_lock_init(&adm->copps_list_lock);

	dais_np = of_get_child_by_name(dev->of_node, "routing");
	if (dais_np) {
		adm->pdev_routing = of_platform_device_create(dais_np,
							   "q6routing", dev);
		of_node_put(dais_np);
	}

	return 0;
}

static int q6adm_remove(struct apr_device *adev)
{
	struct q6adm *adm = dev_get_drvdata(&adev->dev);

	if (adm->pdev_routing)
		of_platform_device_destroy(&adm->pdev_routing->dev, NULL);

	return 0;
}

static const struct of_device_id q6adm_device_id[]  = {
	{ .compatible = "qcom,q6adm" },
	{},
};
MODULE_DEVICE_TABLE(of, q6adm_device_id);

static struct apr_driver qcom_q6adm_driver = {
	.probe = q6adm_probe,
	.remove = q6adm_remove,
	.callback = q6adm_callback,
	.driver = {
		.name = "qcom-q6adm",
		.of_match_table = of_match_ptr(q6adm_device_id),
	},
};

module_apr_driver(qcom_q6adm_driver);
MODULE_DESCRIPTION("Q6 Audio Device Manager");
MODULE_LICENSE("GPL v2");
