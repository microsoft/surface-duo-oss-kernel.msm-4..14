#ifndef _ASM_X86_SPEC_CTRL_H
#define _ASM_X86_SPEC_CTRL_H

#include <linux/stringify.h>
#include <asm/msr-index.h>
#include <asm/cpufeatures.h>
#include <asm/alternative-asm.h>

#ifdef __ASSEMBLY__

.extern use_ibrs
.extern use_ibpb

#define __ASM_ENABLE_IBRS			\
	pushq %rax;				\
	pushq %rcx;				\
	pushq %rdx;				\
	movl $MSR_IA32_SPEC_CTRL, %ecx;		\
	movl $0, %edx;				\
	movl $FEATURE_ENABLE_IBRS, %eax;	\
	wrmsr;					\
	popq %rdx;				\
	popq %rcx;				\
	popq %rax
#define __ASM_ENABLE_IBRS_CLOBBER		\
	movl $MSR_IA32_SPEC_CTRL, %ecx;		\
	movl $0, %edx;				\
	movl $FEATURE_ENABLE_IBRS, %eax;	\
	wrmsr;
#define __ASM_DISABLE_IBRS			\
	pushq %rax;				\
	pushq %rcx;				\
	pushq %rdx;				\
	movl $MSR_IA32_SPEC_CTRL, %ecx;		\
	movl $0, %edx;				\
	movl $0, %eax;				\
	wrmsr;					\
	popq %rdx;				\
	popq %rcx;				\
	popq %rax

.macro ENABLE_IBRS
	testl	$1, use_ibrs
	jz	10f
	__ASM_ENABLE_IBRS
	jmp 20f
10:
	lfence
20:
.endm

.macro ENABLE_IBRS_CLOBBER
	testl	$1, use_ibrs
	jz	11f
	__ASM_ENABLE_IBRS_CLOBBER
	jmp 21f
11:
	lfence
21:
.endm

.macro DISABLE_IBRS
	testl	$1, use_ibrs
	jz	9f
	__ASM_DISABLE_IBRS
9:
.endm

#endif /* __ASSEMBLY__ */
#endif /* _ASM_X86_SPEC_CTRL_H */
