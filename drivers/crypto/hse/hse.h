/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * NXP HSE Driver - HSE Common
 *
 * Copyright 2019 NXP
 */

#ifndef HSE_H
#define HSE_H

#include <crypto/aes.h>

#define HSE_CRA_PRIORITY    2000u /* HSE crypto algorithm priority */

/**
 * struct hse_key - HSE key
 * @entry: list position
 * @handle: key handle
 */
struct hse_key {
	struct list_head entry;
	u32 handle;
};

/**
 * struct hse_drvdata - HSE driver private data
 * @mu_inst: MU instance
 * @hash_algs: supported hash algorithms
 * @hmac_keys: available HMAC key slots
 * @hmac_keys: available AES key slots
 * @key_lock: lock for acquiring key handle
 */
struct hse_drvdata {
	void *mu_inst;
	struct list_head ahash_algs;
	struct list_head hmac_keys;
	struct list_head aes_keys;
	spinlock_t key_lock; /* lock for acquiring key handle */
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

void hse_ahash_register(struct device *dev);

void hse_ahash_unregister(struct device *dev);

void hse_skcipher_register(struct device *dev);

void hse_skcipher_unregister(void);

void hse_aead_register(struct device *dev);

void hse_aead_unregister(void);

void hse_hwrng_register(struct device *dev);

#endif /* HSE_H */
