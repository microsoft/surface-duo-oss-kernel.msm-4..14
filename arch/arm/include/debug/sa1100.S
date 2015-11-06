/*
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define UTDR            0x14
#define UTSR1           0x20
#define UTSR1_TBY       0x00000001      /* Transmitter BusY (read)         */
#define UTSR1_TNF       0x00000004      /* Transmit FIFO Not Full (read)   */

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =CONFIG_DEBUG_UART_PHYS
		ldr	\rv, =CONFIG_DEBUG_UART_VIRT
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #UTDR]
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #UTSR1]
		tst	\rd, #UTSR1_TNF
		beq	1001b
		.endm

		.macro	busyuart,rd,rx
1001:		ldr	\rd, [\rx, #UTSR1]
		tst	\rd, #UTSR1_TBY
		bne	1001b
		.endm
