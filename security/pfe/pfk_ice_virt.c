/* Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/habmm.h>
#include "pfk_ice.h"
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/completion.h>

/**********************************/
/** global definitions		 **/
/**********************************/

#define AES256_KEY_SIZE        (32)
#define RESERVE_SIZE           (36*sizeof(uint16_t))
#define HAB_TIMEOUT_MS         (3000)

/* FBE request command ids */
#define	FDE_CMD_SET_KEY           (0)
#define	FDE_CMD_UPDATE_PASSWORD   (1)
#define	FDE_CMD_CLEAR_KEY         (2)
#define	FDE_CMD_WIPE_KEY          (3)
#define	FDE_SET_ICE               (4)
#define	FBE_SET_KEY               (5)
#define	FBE_CLEAR_KEY             (6)
#define	FBE_GET_MAX_SLOTS         (7)

struct fbe_request_t {
	uint8_t  reserve[RESERVE_SIZE]; // for compatibility
	uint32_t cmd;
	uint8_t  key[AES256_KEY_SIZE];
	uint8_t  salt[AES256_KEY_SIZE];
	uint32_t virt_slot;
};

struct fbe_req_args {
	struct fbe_request_t req;
	int32_t status;
	int32_t ret;
};

static struct completion send_fbe_req_done;

static int32_t send_fbe_req_hab(void *arg)
{
	int ret = 0;
	uint32_t status_size;
	uint32_t handle;
	struct fbe_req_args *req_args = (struct fbe_req_args*)arg;

	do {
		if (!req_args) {
			pr_err("%s Null input\n", __func__);
			ret = -EINVAL;
			break;
		}

		ret = habmm_socket_open(&handle, MM_FDE_1, 0, 0);
		if (ret) {
			pr_err("habmm_socket_open failed with ret = %d\n", ret);
			break;
		}

		ret = habmm_socket_send(handle, &req_args->req, sizeof(struct fbe_request_t), 0);
		if (ret) {
			pr_err("habmm_socket_send failed, ret= 0x%x\n", ret);
			break;
		}

		do {
			status_size = sizeof(int32_t);
			ret = habmm_socket_recv(handle, &req_args->status, &status_size, 0,
					HABMM_SOCKET_RECV_FLAGS_UNINTERRUPTIBLE);
		} while (-EINTR == ret);

		if (ret) {
			pr_err("habmm_socket_recv failed, ret= 0x%x\n", ret);
			break;
		}

		if (status_size != sizeof(int32_t)) {
			pr_err("habmm_socket_recv expected size: %lu, actual=%u\n",
					sizeof(int32_t),
					status_size);
			ret = -E2BIG;
			break;
		}

		ret = habmm_socket_close(handle);
		if (ret) {
			pr_err("habmm_socket_close failed with ret = %d\n", ret);
			break;
		}
	} while (0);

	req_args->ret = ret;

	complete(&send_fbe_req_done);

	return 0;
}

static void send_fbe_req(struct fbe_req_args *arg)
{
	struct task_struct *thread;

	init_completion(&send_fbe_req_done);
	arg->status  = 0;

	thread = kthread_run(send_fbe_req_hab, arg, "send_fbe_req");
	if (IS_ERR(thread)){
		arg->ret = -1;
		return;
	}

	if (wait_for_completion_interruptible_timeout(
		&send_fbe_req_done, msecs_to_jiffies(HAB_TIMEOUT_MS)) <= 0) {
		pr_err("%s: timeout hit", __func__);
		kthread_stop(thread);
		arg->ret = -ETIME;
		return;
	}
}


int qti_pfk_ice_set_key(uint32_t index, uint8_t *key, uint8_t *salt,
			char *storage_type, unsigned int data_unit)
{
	struct fbe_req_args arg;

	if (!key || !salt) {
		pr_err("%s Invalid key/salt\n", __func__);
		return -EINVAL;
	}

	arg.req.cmd = FBE_SET_KEY;
	arg.req.virt_slot = index;
	memcpy(&(arg.req.key[0]), key, AES256_KEY_SIZE);
	memcpy(&(arg.req.salt[0]), salt, AES256_KEY_SIZE);

	send_fbe_req(&arg);

	if (arg.ret || arg.status) {
		pr_err("%s: send_fbe_req failed with ret = %d, status = %d\n",
		__func__, arg.ret, arg.status);
		return -ECOMM;
	}

	return 0;
}

int qti_pfk_ice_invalidate_key(uint32_t index, char *storage_type)
{
	struct fbe_req_args arg;

	arg.req.cmd = FBE_CLEAR_KEY;
	arg.req.virt_slot = index;

	send_fbe_req(&arg);

	if (arg.ret || arg.status) {
		pr_err("%s: send_fbe_req failed with ret = %d, status = %d\n",
		__func__, arg.ret, arg.status);
		return -ECOMM;
	}

	return 0;
}

int qti_pfk_ice_get_info(uint32_t *min_slot_index, uint32_t *total_num_slots,
		bool async)
{
	struct fbe_req_args arg;

	if (!min_slot_index || !total_num_slots) {
		pr_err("%s Null input\n", __func__);
		return -EINVAL;
	}

	if (async)
		return -EAGAIN;

	arg.req.cmd = FBE_GET_MAX_SLOTS;
	send_fbe_req(&arg);

	if (arg.ret || arg.status < 0) {
		pr_err("%s: send_fbe_req failed with ret = %d, max_slots = %d\n",
				__func__, arg.ret, arg.status);
		return -ECOMM;
	}

	*min_slot_index = 0;
	*total_num_slots = (uint32_t) arg.status;

	return 0;
}
