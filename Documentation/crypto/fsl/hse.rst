.. SPDX-License-Identifier: BSD-3-Clause

=============================
HSE crypto accelerator driver
=============================

:Copyright: 2019 NXP

Overview
========
This file provides documentation for NXP HSE (Hardware Security Engine) crypto
accelerator driver.

Supported Platforms
===================
This driver provides crypto acceleration support for NXP S32G274A processor.

Features & Offloads
===================
Currently the driver supports offloading the following crypto operations:

- Hashing: SHA1, MD5
- Symmetric Key Ciphering: AES-CBC
- Message Authentication: HMAC(MD5), HMAC(SHA1)
- Authenticated Encryption with Associated Data: AES-GCM
- True Random Number Generator

Configuration
=============
The following Kconfig options are available:

- Messaging Unit (MU) Instance (CONFIG_CRYPTO_DEV_NXP_HSE_MU_ID):
  There are 4 Messaging Units available for interfacing processor subsystems 
  with HSE and user can configure which one is used by Linux driver for sending
  and receiving crypto requests.

- HSE hardware True RNG support (CONFIG_CRYPTO_DEV_NXP_HSE_HWRNG):
  Enable/disable HSE True RNG support. Default value is yes.

- AES Key Group ID within HSE Key Catalog (CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GID):
  There can be up to 256 key groups within HSE RAM Key Catalog, each one storing
  a different key type. This option specifies which key group is used by driver
  for programming AES 256-bit keys into HSE, depending on how catalog was
  initialized by firmware. Default value is 1.

- Number of AES Key Slots in Key Group (CONFIG_CRYPTO_DEV_NXP_HSE_AES_KEY_GSIZE):
  Each Key Group can store up to 256 keys and this option configures the number
  of keys that can be stored in the AES 256-bit key group.

- HMAC Key Group ID within HSE Key Catalog (CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GID):
  There can be up to 256 key groups within HSE RAM Key Catalog, each one storing
  a different key type. This option specifies which key group is used by driver
  for programming HMAC keys into HSE, depending on how catalog was initialized
  by firmware. Default value is 2.

- Number of HMAC Key Slots in Key Group (CONFIG_CRYPTO_DEV_NXP_HSE_HMAC_KEY_GSIZE):
  Each Key Group can store up to 256 keys and this option configures the number
  of keys that can be stored in the HMAC key group.
