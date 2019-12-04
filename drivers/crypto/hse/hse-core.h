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

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

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
 * @mu_inst: MU instance
 * @ahash_algs: registered hash and hash-based MAC algorithms
 * @hmac_key_ring: HMAC key slots currently available
 * @aes_key_ring: AES key slots currently available
 * @key_ring_lock: lock for acquiring key slot
 */
struct hse_drvdata {
	void *mu_inst;
	struct list_head ahash_algs;
	struct list_head hmac_key_ring;
	struct list_head aes_key_ring;
	spinlock_t key_ring_lock; /* lock for acquiring key slot */
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

void hse_ahash_register(struct device *dev);
void hse_ahash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);
void hse_skcipher_unregister(void);

void hse_aead_register(struct device *dev);
void hse_aead_unregister(void);

void hse_hwrng_register(struct device *dev);

#endif /* HSE_CORE_H */
