/*
 *  linux/arch/arm/mm/rodata.c
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  Author: Colin Cross <ccross@android.com>
 *
 *  Based on x86 implementation in arch/x86/mm/init_32.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>

#include <asm/cache.h>
#include <asm/pgtable.h>
#include <asm/rodata.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>

#include "mm.h"

static int kernel_set_to_readonly __read_mostly;

#ifdef CONFIG_DEBUG_RODATA_TEST
static const int rodata_test_data = 0xC3;

static noinline void rodata_test(void)
{
	int result;

	pr_info("%s: attempting to write to read-only section:\n", __func__);

	if (*(volatile int *)&rodata_test_data != 0xC3) {
		pr_err("read only data changed before test\n");
		return;
	}

	/*
	 * Attempt to to write to rodata_test_data, trapping the expected
	 * data abort.  If the trap executed, result will be 1.  If it didn't,
	 * result will be 0xFF.
	 */
	asm volatile(
		"0:	str	%[zero], [%[rodata_test_data]]\n"
		"	mov	%[result], #0xFF\n"
		"	b	2f\n"
		"1:	mov	%[result], #1\n"
		"2:\n"

		/* Exception fixup - if store at label 0 faults, jumps to 1 */
		".pushsection __ex_table, \"a\"\n"
		"	.long	0b, 1b\n"
		".popsection\n"

		: [result] "=r" (result)
		: [rodata_test_data] "r" (&rodata_test_data), [zero] "r" (0)
		: "memory"
	);

	if (result == 1)
		pr_info("write to read-only section trapped, success\n");
	else
		pr_err("write to read-only section NOT trapped, test failed\n");

	if (*(volatile int *)&rodata_test_data != 0xC3)
		pr_err("read only data changed during write\n");
}
#else
static inline void rodata_test(void) { }
#endif

void set_kernel_text_rw(void)
{
	unsigned long start = PAGE_ALIGN((unsigned long)_text);
	unsigned long size = PAGE_ALIGN((unsigned long)__end_rodata) - start;

	if (!kernel_set_to_readonly)
		return;

	pr_debug("Set kernel text: %lx - %lx to read-write\n",
		 start, start + size);

	set_memory_rw(start, size >> PAGE_SHIFT);
}

void set_kernel_text_ro(void)
{
	unsigned long start = PAGE_ALIGN((unsigned long)_text);
	unsigned long size = PAGE_ALIGN((unsigned long)__end_rodata) - start;

	if (!kernel_set_to_readonly)
		return;

	pr_info_once("Write protecting the kernel text section %lx - %lx\n",
		start, start + size);

	pr_debug("Set kernel text: %lx - %lx to read only\n",
		 start, start + size);

	set_memory_ro(start, size >> PAGE_SHIFT);
}

void mark_rodata_ro(void)
{
	kernel_set_to_readonly = 1;

	set_kernel_text_ro();

	rodata_test();
}
