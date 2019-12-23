/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - Core
 *
 * This file defines the driver core interface for the HSE cryptographic engine.
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_CORE_H
#define HSE_CORE_H

#include <crypto/aes.h>

#define HSE_NUM_CHANNELS    16u /* number of available service channels */
#define HSE_STREAM_COUNT    2u /* number of usable streams per MU instance */

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

#define HSE_CHANNEL_ANY    0xACu /* use any channel, no request ordering */
#define HSE_CHANNEL_INV    0xFFu /* invalid acquired service channel index */

/**
 * enum hse_ch_type - channel type
 * @HSE_CHANNEL_SHARED: shared channel
 * @HSE_CHANNEL_STREAM: stream
 */
enum hse_ch_type {
	HSE_CH_TYPE_SHARED = 0u,
	HSE_CH_TYPE_STREAM = 1u,
};

/**
 * struct hse_key - HSE key slot
 * @entry: list position
 * @handle: key handle
 * @type: key type
 */
struct hse_key {
	struct list_head entry;
	u32 handle;
	enum hse_key_type type;
};

/**
 * struct hse_drvdata - HSE driver private data
 * @ahash_algs: registered hash and hash-based MAC algorithms
 * @skcipher_algs: registered symmetric key cipher algorithms
 * @aead_algs: registered AEAD algorithms
 * @mu: MU instance handle
 * @channel_busy[n]: internally cached status of MU channel n
 * @refcnt: acquired service channel reference count
 * @rx_cbk[n].fn: upper layer RX callback for channel n
 * @rx_cbk[n].ctx: context passed to the RX callback on channel n
 * @sync[n].done: completion for synchronous requests on channel n
 * @sync[n].reply: decoded service response location for channel n
 * @stream_lock: lock used for stream channel reservation
 * @tx_lock: lock used for service request transmission
 * @hmac_key_ring: HMAC key slots currently available
 * @aes_key_ring: AES key slots currently available
 * @key_ring_lock: lock used for key slot acquisition
 */
struct hse_drvdata {
	struct list_head ahash_algs;
	struct list_head skcipher_algs;
	struct list_head aead_algs;
	void *mu;
	bool channel_busy[HSE_NUM_CHANNELS];
	atomic_t refcnt[HSE_NUM_CHANNELS];
	struct {
		void (*fn)(int err, void *ctx);
		void *ctx;
	} rx_cbk[HSE_NUM_CHANNELS];
	struct {
		struct completion *done;
		int *reply;
	} sync[HSE_NUM_CHANNELS];
	spinlock_t stream_lock; /* covers stream reservation */
	spinlock_t tx_lock; /* covers request transmission */
	struct list_head hmac_key_ring;
	struct list_head aes_key_ring;
	spinlock_t key_ring_lock; /* covers key slot acquisition */
};

/**
 * check_aes_keylen - validate key length for AES algorithms
 * @keylen: AES key length
 *
 * Return: 0 on success, -EINVAL otherwise
 */
static inline int hse_check_aes_keylen(u32 keylen)
{
	switch (keylen) {
	case AES_KEYSIZE_128:
	case AES_KEYSIZE_192:
	case AES_KEYSIZE_256:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct hse_key *hse_key_slot_acquire(struct device *dev,
				     enum hse_key_type type);
void hse_key_slot_release(struct device *dev, struct hse_key *slot);

int hse_channel_acquire(struct device *dev, enum hse_ch_type type, u8 *channel);
int hse_channel_release(struct device *dev, u8 channel);

int hse_srv_req_async(struct device *dev, u8 channel, dma_addr_t srv_desc,
		      void *ctx, void (*rx_cbk)(int err, void *ctx));
int hse_srv_req_sync(struct device *dev, u8 channel, dma_addr_t srv_desc);

void hse_ahash_register(struct device *dev);
void hse_ahash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);
void hse_skcipher_unregister(void);

void hse_aead_register(struct device *dev);
void hse_aead_unregister(void);

void hse_hwrng_register(struct device *dev);

#endif /* HSE_CORE_H */
