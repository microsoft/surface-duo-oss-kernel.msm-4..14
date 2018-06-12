/*
 * User-space CSE3 crypto requests sample file
 * Check Usage: `./cse_test h` for the supported crypto requests
 * and their expected command line arguments
 *
 * Copyright (c) 2015 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * later as publishhed by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "cse3_ioctl.h"
#include "cse_test.h"

#define DEVICE_PATH		"/dev/cse3"
#define BUFFER_SIZE		256

/**
 * Converts the source data represented as a hex string to its binary form
 * @src: Hex representation of the source data
 * @dst: array of bytes
 * src example: 6bc1bee22e409f96e93d7e117393172a
 * NOTE: The function expects that each byte is represented by two characters
 * in @src. E.g.: 0xa will be "0a" in the source string.
 */
static void hexStrToStr(unsigned char *dst, const char *src, int size)
{
	int i;
	for (i = 0; i < (size / 2); i++) {
		sscanf(src + 2*i, "%2hhx", &dst[i]);
	}
}

static void print_hex(const unsigned char *s, int size)
{
	int i;
	for (i = 0; i < size; i++)
		printf("%02x", s[i]);
	printf("\n");
}

static inline void error(const char *message)
{
	perror(message);
	exit(EXIT_FAILURE);
}

/**
 * cse_test comamnd line arguments
 */
static void usage(const char *argv0)
{
    printf("Usage: %s <options>\n options:\n"
			"\tr                           - generate a random 128bit value\n"
			"\te plainText key <iv>        - encrypt with AES-128\n"
			"\td cipherText key <iv>       - decrypt with AES-128\n"
			"\tc inputText key             - generate CMAC\n"
			"\tv inputText key expectedMac - verify CMAC\n"
			"\tl m1 m2 m3                  - load custom encrypted key\n"
			"\ty plainText keyID <iv>      - encrypt with AES-128 using\n"
			"\tpreviously loaded encrypted key\n"
			"\tz cipherText keyID <iv>     - decrypt with AES-128 using\n"
			"\tpreviously loaded encrypted key\n", argv0);
    exit(EXIT_FAILURE);
}

static void cse_load_key(int fd, char *m1, char *m2, char *m3)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char buffer1[BUFFER_SIZE];
	unsigned char buffer2[BUFFER_SIZE];
	struct ioctl_ldkey ld;

	memset(buffer, 0, BUFFER_SIZE);
	memset(buffer1, 0, BUFFER_SIZE);
	memset(buffer2, 0, BUFFER_SIZE);
	ld.addr_m1 = &buffer;
	ld.addr_m2 = &buffer1;
	ld.addr_m3 = &buffer2;
	/* NOTE: this test program only loads keys for the first key bank */
	ld.kbs_key = 0;
	hexStrToStr(buffer, m1, M1SIZE*2);
	hexStrToStr(buffer1, m2, M2SIZE*2);
	hexStrToStr(buffer2, m3, M3SIZE*2);

	if (ioctl(fd, CSE3_IOCTL_LOAD_KEY, &ld) < 0)
		error("ioctl");
}

static void cse_compress_mp(int fd, char *mpin, int input_len)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char result[BUFFER_SIZE];
	struct ioctl_mp mp;

	memset(buffer, 0, BUFFER_SIZE);
	memset(result, 0, BUFFER_SIZE);
	mp.addr_in = &buffer;
	mp.addr_out = &result;
	mp.len = input_len/2; //string to bytes
	hexStrToStr(buffer, mpin, input_len);

	if (ioctl(fd, CSE3_IOCTL_COMPRESS_MP, &mp) < 0)
		error("ioctl");

	result[BUFFER_SIZE-1] = 0;
	printf("MP value:\n");
	print_hex(result, MP_SIZE);
}

static void cse_gen_mac(int fd, char *macText, int input_len)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char result[BUFFER_SIZE];
	struct ioctl_cmac cmac;

	memset(buffer, 0, BUFFER_SIZE);
	memset(result, 0, BUFFER_SIZE);
	cmac.addr_in = &buffer;
	cmac.addr_out = &result;
	cmac.len = input_len/2;
	hexStrToStr(buffer, macText, input_len);

	if (ioctl(fd, CSE3_IOCTL_GEN_MAC, &cmac) < 0)
		error("ioctl");

	result[BUFFER_SIZE-1] = 0;
	printf("CMAC value:\n");
	print_hex(result, MAC_SIZE);
}

static void cse_check_mac(int fd, char *macText, char *macVal, int input_len)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char result[BUFFER_SIZE];
	struct ioctl_cmac cmac;
	int ret;

	memset(buffer, 0, BUFFER_SIZE);
	memset(result, 0, BUFFER_SIZE);
	cmac.addr_in = &buffer;
	cmac.addr_out = &result;
	cmac.len = input_len/2;
	hexStrToStr(buffer, macText, input_len);
	hexStrToStr(result, macVal, MAC_SIZE*2);

	ret = ioctl(fd, CSE3_IOCTL_CHECK_MAC, &cmac);
	printf("CMAC Verification %s", ret == 0 ? "PASSED\n" : "FAILED\n");
	if (ret < 0)
		error("ioctl");
}

/**
 * Sets the plaintext key associated to
 * the current file descriptor (fd) context
 */
static void cse_set_key(int fd, char *key)
{
	unsigned char buffer[BUFFER_SIZE];

	memset(buffer, 0, BUFFER_SIZE);
	hexStrToStr(buffer, key, KEY_SIZE*2);
	if (ioctl(fd, CSE3_IOCTL_SET_KEY, buffer) < 0)
		error("ioctl");
}

/**
 * Sets the AES-CBC IV associated to
 * the current file descriptor (fd) context
 */
static void cse_set_iv(int fd, char *iv)
{
	unsigned char buffer[BUFFER_SIZE];

	memset(buffer, 0, BUFFER_SIZE);
	hexStrToStr(buffer, iv, KEY_SIZE*2);
	if (ioctl(fd, CSE3_IOCTL_SET_IV, buffer) < 0)
		error("ioctl");
}

static void cse_encrypt(int fd, int hasKey, int keyId, int hasIV,
		char *plain, int input_len)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char result[BUFFER_SIZE];
	struct ioctl_crypt crypt;

	memset(buffer, 0, BUFFER_SIZE);
	memset(result, 0, BUFFER_SIZE);
	crypt.addr_in = &buffer;
	crypt.addr_out = &result;
	crypt.len = input_len/2;
	if (hasKey) {
		crypt.key_id = keyId;
		/* NOTE: this test program only loads keys for the first key bank */
		crypt.kbs_key = 0;
	}
	hexStrToStr(buffer, plain, input_len);

	if (hasKey) {
		if (ioctl(fd, hasIV ?
					CSE3_IOCTL_ENC_CBC_WK : CSE3_IOCTL_ENC_ECB_WK, &crypt) < 0)
			error("ioctl");
	} else {
		if (ioctl(fd, hasIV ?
					CSE3_IOCTL_ENC_CBC : CSE3_IOCTL_ENC_ECB, &crypt) < 0)
			error("ioctl");
	}
	result[BUFFER_SIZE-1] = 0;
	printf("Encrypted value:\n");
	print_hex(result, input_len/2);
}

static void cse_decrypt(int fd, int hasKey, int keyId, int hasIV,
		char *cipher, int input_len)
{
	unsigned char buffer[BUFFER_SIZE];
	unsigned char result[BUFFER_SIZE];
	struct ioctl_crypt crypt;

	memset(buffer, 0, BUFFER_SIZE);
	memset(result, 0, BUFFER_SIZE);
	crypt.addr_in =  buffer;
	crypt.addr_out = result;
	crypt.len = input_len/2;
	if (hasKey) {
		crypt.key_id = keyId;
		/* NOTE: this test program only loads keys for the first key bank */
		crypt.kbs_key = 0;
	}
	hexStrToStr(buffer, cipher, input_len);

	if (hasKey) {
		if (ioctl(fd, hasIV ?
					CSE3_IOCTL_DEC_CBC_WK : CSE3_IOCTL_DEC_ECB_WK, &crypt) < 0)
			error("ioctl");
	} else {
		if (ioctl(fd, hasIV ?
					CSE3_IOCTL_DEC_CBC : CSE3_IOCTL_DEC_ECB, &crypt) < 0)
			error("ioctl");
	}

	result[BUFFER_SIZE-1] = 0;
	printf("Decrypted value:\n");
	print_hex(result, input_len/2);
}

static void cse_gen_rnd(int fd)
{
	unsigned char buffer[BUFFER_SIZE];

	memset(buffer, 0, BUFFER_SIZE);
	if (ioctl(fd, CSE3_IOCTL_RND, buffer) < 0)
		error("ioctl");

	buffer[BUFFER_SIZE-1] = 0;
	printf("Random value:\n");
	print_hex(buffer, RND_SIZE);
}

/*
 * Sample run:
 *  ./cse_test r						; generate random value
 *  ./cse_test e plainText key <iv>		; AES-123 encryption
 */
int main(int argc, char **argv)
{
	/* Buffer sizes are multiplied by 2 since the program
	 * expects input buffers represented by hex string values,
	 * where each byte is represented by 2 charactes:
	 * e.g.: 0xa will be "0a" in the input string
	 */
	char key[KEY_SIZE*2], iv[KEY_SIZE*2], macVal[MAC_SIZE*2];
	char m1[M1SIZE*2], m2[M2SIZE*2], m3[M3SIZE*2];
	char *plain, *cipher, *macText, *mpin;
	int input_len;
	int hasIV = 0;
	int keyId = 0;
	int fd;

	if (argc < 2)
		usage(argv[0]);
	else if (argv[1][0] == 'h')
		usage(argv[0]);

	fd = open(DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		perror("open");
		exit(EXIT_FAILURE);
	}

	if (argv[1][0] == CSE_COMPRESS_MP) {
		mpin = strdup(argv[2]);
		input_len = strlen(argv[2]);
	}

	if (argv[1][0] == CSE_GEN_MAC || argv[1][0] == CSE_CHECK_MAC) {
		macText = strdup(argv[2]);
		input_len = strlen(argv[2]);
		strncpy(key, argv[3], KEY_SIZE*2);
		if (argv[1][0] == CSE_CHECK_MAC)
			strncpy(macVal, argv[4], MAC_SIZE*2);
	}

	if (argv[1][0] == CSE_ENCRYPT || argv[1][0] == CSE_DECRYPT) {
		if (argv[1][0] == CSE_ENCRYPT)
			plain = strdup(argv[2]);
		else
			cipher = strdup(argv[2]);
		input_len = strlen(argv[2]);
		strncpy(key, argv[3], KEY_SIZE*2);
		if (argc == 5) {
			strncpy(iv, argv[4], KEY_SIZE*2);
			hasIV = 1;
		}
	}

	if (argv[1][0] == CSE_ENCRYPT_WK || argv[1][0] == CSE_DECRYPT_WK) {
		if (argv[1][0] == CSE_ENCRYPT_WK)
			plain = strdup(argv[2]);
		else
			cipher = strdup(argv[2]);
		input_len = strlen(argv[2]);
		keyId = atoi(argv[3]);
		if (argc == 5) {
			strncpy(iv, argv[4], KEY_SIZE*2);
			hasIV = 1;
		}
	}

	if (argv[1][0] == CSE_LOAD_KEY) {
		strncpy(m1, argv[2], M1SIZE*2);
		strncpy(m2, argv[3], M2SIZE*2);
		strncpy(m3, argv[4], M3SIZE*2);
	}

	switch (argv[1][0]) {
	case CSE_ENCRYPT: {
		cse_set_key(fd, key);
		if (hasIV)
			cse_set_iv(fd, iv);
		cse_encrypt(fd, 0, 0, hasIV, plain, input_len);
		break;
	}

	case CSE_DECRYPT: {
		cse_set_key(fd, key);
		if (hasIV)
			cse_set_iv(fd, iv);
		cse_decrypt(fd, 0, 0, hasIV, cipher, input_len);
		break;
	}

	case CSE_ENCRYPT_WK: {
		if (hasIV)
			cse_set_iv(fd, iv);
		cse_encrypt(fd, 1, keyId, hasIV, plain, input_len);
		break;
	}

	case CSE_DECRYPT_WK: {
		if (hasIV)
			cse_set_iv(fd, iv);
		cse_decrypt(fd, 1, keyId, hasIV, cipher, input_len);
		break;
	}

	case CSE_GEN_MAC: {
		cse_set_key(fd, key);
		cse_gen_mac(fd, macText, input_len);
		break;
	}

	case CSE_CHECK_MAC: {
		cse_set_key(fd, key);
		cse_check_mac(fd, macText, macVal, input_len);
		break;
	}

	case CSE_LOAD_KEY:
		cse_load_key(fd, m1, m2, m3);
		break;
	case CSE_COMPRESS_MP:
		cse_compress_mp(fd, mpin, input_len);
		break;
	case CSE_GEN_RND:
		cse_gen_rnd(fd);
		break;
	default:
		printf("Unkowm request\n");
		break;
	}

	close(fd);

	return 0;
}
