/*
 * Copyright (C) 2005-2010 Mathieu Desnoyers (mathieu.desnoyers@efficios.com)
 *
 * This contains the core definitions for the Linux Trace Toolkit.
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#ifndef LTT_CORE_H
#define LTT_CORE_H

/* Keep track of trap nesting inside LTT */
DECLARE_PER_CPU(unsigned int, ltt_nesting);

#ifndef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS

/*
 * Calculate the offset needed to align the type.
 * size_of_type must be non-zero.
 */
static inline unsigned int ltt_align(size_t align_drift, size_t size_of_type)
{
	return offset_align(align_drift, min(sizeof(void *), size_of_type));
}
/* Default arch alignment */
#define LTT_ALIGN

static inline int ltt_get_alignment(void)
{
	return sizeof(void *);
}

extern unsigned int ltt_fmt_largest_align(size_t align_drift, const char *fmt);

#else

static inline unsigned int ltt_align(size_t align_drift,
		 size_t size_of_type)
{
	return 0;
}

#define LTT_ALIGN __attribute__((packed))

static inline int ltt_get_alignment(void)
{
	return 0;
}

static inline unsigned int ltt_fmt_largest_align(size_t align_drift,
		const char *fmt)
{
	return 0;
}

#endif /* HAVE_EFFICIENT_UNALIGNED_ACCESS */

#endif /* LTT_CORE_H */
