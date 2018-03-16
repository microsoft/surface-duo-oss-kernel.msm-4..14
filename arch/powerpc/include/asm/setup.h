#ifndef _ASM_POWERPC_SETUP_H
#define _ASM_POWERPC_SETUP_H

#include <uapi/asm/setup.h>

#ifndef __ASSEMBLY__
extern void ppc_printk_progress(char *s, unsigned short hex);

extern unsigned int rtas_data;
extern unsigned long long memory_limit;
extern unsigned long klimit;
extern void *zalloc_maybe_bootmem(size_t size, gfp_t mask);

struct device_node;
extern void note_scsi_host(struct device_node *, void *);

/* Used in very early kernel initialization. */
extern unsigned long reloc_offset(void);
extern unsigned long add_reloc_offset(unsigned long);
extern void reloc_got2(unsigned long);

#define PTRRELOC(x)	((typeof(x)) add_reloc_offset((unsigned long)(x)))

void check_for_initrd(void);
void initmem_init(void);
#define ARCH_PANIC_TIMEOUT 180

void rfi_flush_enable(bool enable);

enum l1d_flush_type {
	L1D_FLUSH_NONE,
	L1D_FLUSH_FALLBACK,
	L1D_FLUSH_ORI,
	L1D_FLUSH_MTTRIG,
};

void __init setup_rfi_flush(enum l1d_flush_type, bool enable);

#endif /* !__ASSEMBLY__ */

#endif	/* _ASM_POWERPC_SETUP_H */

