#!/bin/bash

##
# Sample script for calling CSE3 crypto requests from userspace
# Requires cse_test sample program and the CSE3 device driver cse3.ko.
#
# Copyright (c) 2015 Freescale Semiconductor, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 or
# later as publishhed by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
##

if [ ! -f cse3.ko ]; then
	echo "Couldn't find cse3.ko module"
	exit
fi

if [ ! -f cse_test ]; then
	echo "Couldn't find cse_test program"
	exit
fi

# Create CSE3 device file, with its registered MAJOR (42) and MINOR (0)
mknod /dev/cse3 c 42 0
# Insert the module
insmod cse3.ko

echo 'testing AES-128 CBC encryption'
read -p $'Press... \n' -n1 -s
./cse_test e 6bc1bee22e409f96e93d7e117393172a 2b7e151628aed2a6abf7158809cf4f3c 000102030405060708090a0b0c0d0e0f

echo 'testing AES-128 CBC decryption'
read -p $'Press... \n' -n1 -s
./cse_test d 7649abac8119b246cee98e9b12e9197d 2b7e151628aed2a6abf7158809cf4f3c 000102030405060708090a0b0c0d0e0f

echo 'testing CMAC generation'
read -p $'Press... \n' -n1 -s
./cse_test c 6bc1bee22e409f96e93d7e117393172a 2b7e151628aed2a6abf7158809cf4f3c

echo 'testing good CMAC value'
read -p $'Press... \n' -n1 -s
./cse_test v 6bc1bee22e409f96e93d7e117393172a 2b7e151628aed2a6abf7158809cf4f3c 070a16b46b4d4144f79bdd9dd04a287c

echo 'testing bad CMAC value'
read -p $'Press... \n' -n1 -s
./cse_test v 6bc1bee22e409f96e93d7e117393172a 2b7e151628aed2a6abf7158809cf4f3c 070a16b46b4d4144f79bdd9dd04a2777

echo 'testing Miyaguchi-Preneel compression'
read -p $'Press... \n' -n1 -s
./cse_test p 6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e51

echo "testing custom encrypted key generation"
read -p $'Press... \n' -n1 -s
echo "gen k1 with MP compression"
./cse_test p 000102030405060708090a0b0c0d0e0f010153484500
echo "gen k2 with MP compression"
./cse_test p 000102030405060708090a0b0c0d0e0f010253484500
echo "generate M2"
./cse_test e 000000100000000000000000000000000f0e0d0c0b0a09080706050403020100 118a46447a770d87828a69c222e2d17e 00000000000000000000000000000000
echo "generate M3"
./cse_test c 000000000000000000000000000001412b111e2d93f486566bcbba1d7f7a9797c94643b050fc5d4d7de14cff682203c3 2ebb2a3da62dbd64b18ba6493e9fbe22

echo "testing custom encrypted key load and encryption"
read -p $'Press... \n' -n1 -s
./cse_test l 00000000000000000000000000000044 ff8b75f73e6ad5a1729423c6e9311f1a7b152023f03fa356a33f101c3e8195fe 3ef0722bbea6ddb98f555d1e6f0f617e
./cse_test y 6bc1bee22e409f96e93d7e117393172a 4 000102030405060708090a0b0c0d0e0f

