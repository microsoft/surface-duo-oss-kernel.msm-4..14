/*
 * drivers/staging/android/ion/ion_compound_page.c
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2015 ARM Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/atomic.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/log2.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/shrinker.h>
#include <linux/slab.h>
#include <linux/string_helpers.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/wait.h>
#include "ion_priv.h"

#define CPP_ORDER CONFIG_ION_COMPOUND_PAGE_SIZE
#ifdef CONFIG_ION_COMPOUND_PAGE_STATS
	#define CPP_STATS
#endif
/* # histogram bins */
#define CPP_HISTOGRAM_BINS 16
/* size in bytes of a compound page */
#define CPP_PAGE_SIZE (PAGE_SIZE << CPP_ORDER)
/* number of PAGE_SIZE pages in the compound page */
#define CPP_SUB_PAGES (CPP_PAGE_SIZE >> PAGE_SHIFT)

static int pool_page_lowmark = CONFIG_ION_COMPOUND_PAGE_LOWMARK;
static int pool_page_highmark = CONFIG_ION_COMPOUND_PAGE_HIGHMARK;
static int pool_page_fillmark = CONFIG_ION_COMPOUND_PAGE_FILLMARK;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
static const gfp_t gfp_flags = (GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN |
		__GFP_NORETRY | __GFP_COMP) & ~__GFP_WAIT;
#else
static const gfp_t gfp_flags = (GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN |
		__GFP_NORETRY | __GFP_COMP) & ~__GFP_RECLAIM;
#endif

#ifdef CPP_STATS
/** struct cpp_stats - usage tracking.
 * @bytes_requested: accumulated bytes request
 * @bytes_committed: accumulated bytes actually committed
 * @num_allocs: Accumulated number of allocations
 * @live_alliocs: Number of current allocations
 * @live_requested: bytes request for current allocations
 * @live_committed: bytes committed for current allocations
 */
struct cpp_stats {
	atomic64_t bytes_requested;
	atomic64_t bytes_committed;
	atomic_t num_allocs;
	atomic_t live_allocs;
	atomic_t live_requested;
	atomic_t live_committed;
};
#endif

/**
 * struct cpp_sub_alloc - tracks usage of a sub-allocate compound page.
 * @link: to be tracked on struct @cpp_alloc's @partials.
 * @page: The compound page which has been sub-allocated
 * @sub_pages: Bitmap to track use of PAGE_SIZE chunks of the compound page
 */
struct cpp_sub_alloc {
	struct list_head link;
	struct page *page;
	DECLARE_BITMAP(sub_pages, CPP_SUB_PAGES);
};

/**
 * struct cpp_pool - the main compound page pool structure
 * @heap:                    Embedded @ion_heap as requested by the ion API.
 * @lock:                    Lock protecting access to:
 *                             - @pages
 *                             - @partials
 *                             - @free_queue
 * @lowmark:                 Refill if count of free pages drop below this
 * @fillmark:                Refill to this
 * @highmark:                Free directly to kernel above this
 * @count:                   Number of free pages on the @pages list
 * @pages:                   List of free pages
 * @num_partials:            Number of partials in use
 * @free_bytes_in_partials:  Number of bytes available in the partials
 * @partials:                List of partials
 * @worker:                  Thread to perform refill and drain
 * @worker_wait:             Wait object for worker thread
 * @free_queue:              List of pages to free (drain)
 * @shrinker:                Allow Linux to reclaim memory on our free list
 * @overall:                 Overall @cpp_stats object
 * @histogram:               @cpp_stats per number of compound pages
 * @max_alloc_time:          Max time to allocate for a buffer
 * @max_cp_alloc_time:       Max time to allocate a single page from the kernel
 * @num_soft_alloc_failures: Number of times a kernel allocation has failed
 * @num_hard_alloc_failures: Number of times the kernel allocation failure was
 *                           fatal
 * @depleted                 Number of times the pool was fully exhausted
 * @shrinks                  Number of times the pool's shrinker has been
 *                           activated
 * @pages_shrunk             Number of compound page reclaimed via shrinker
 *
 * Main structure representing a compound page pool.
 * Embeds an @ion_heap per the ion API.
 * Uses a mutex to synchronize access to the list_heads used to track the
 * free list and partial in-use.
 * Atomics is used when possible/needed to allow debugfs without locks.
 * Optionally tracks usage statistics.
 */
struct cpp_pool {
	struct ion_heap heap;

	/* lock protecting the pool */
	struct mutex lock;

	unsigned int lowmark;
	unsigned int fillmark;
	unsigned int  highmark;
	atomic_t count;
	struct list_head pages;

	struct list_head partials;

	/* kthread to asynchronously refill and zero freed pages */
	struct task_struct *worker;
	wait_queue_head_t worker_wait;
	struct list_head free_queue;

	struct shrinker shrinker;

	/* statistics */
#ifdef CPP_STATS
	atomic_t num_partials;
	atomic_t free_bytes_in_partials;
	struct cpp_stats overall;
	struct cpp_stats histogram[CPP_HISTOGRAM_BINS];
	atomic_t max_alloc_time;
	atomic_t max_cp_alloc_time;
	atomic_t num_soft_alloc_failures;
	atomic_t num_hard_alloc_failures;
	atomic_t depleted;
	atomic_t shrinks;
	atomic_t pages_shrunk;
#endif
};

#ifdef CPP_STATS
/** cpp_stats_update() - Helper to update a @cpp_stats object.
 * @stats: The @cpp_stats object to update.
 * @requested: The number of bytes requested.
 * @committed: The number of bytes committed.
 */
static void cpp_stats_update(struct cpp_stats *stats, unsigned long requested,
			     unsigned long committed)
{
	atomic_inc(&stats->num_allocs);
	atomic64_add(requested, &stats->bytes_requested);
	atomic64_add(committed, &stats->bytes_committed);
}

/** cpp_histogram_index() - Find the histogram index to use.
 * @committed: Bytes to find the histogram index for.
 *
 * Returns the histogram index to use based on the number of whole compound
 * pages.
 */
static unsigned int cpp_histogram_index(unsigned long committed)
{
	int pages = committed >> (PAGE_SHIFT + CPP_ORDER);

	if (pages >= CPP_HISTOGRAM_BINS)
		return CPP_HISTOGRAM_BINS - 1;
	else
		return pages;
}

static void cpp_log_shrink(struct cpp_pool *pool, unsigned long count)
{
	atomic_inc(&pool->shrinks);
	atomic_add(count, &pool->pages_shrunk);
}

/** cpp_log_alloc() - Track allocation.
 * @pool: The pool to update the statistics for.
 * @requested: bytes requested.
 * @committed: bytes actually committed.
 *
 * Updates the histogram and overall stats.
 */
static void cpp_log_alloc(struct cpp_pool *pool, unsigned long requested,
			  unsigned long committed)
{
	unsigned int hidx = cpp_histogram_index(committed);

	cpp_stats_update(&pool->overall, requested, committed);
	cpp_stats_update(&pool->histogram[hidx], requested, committed);

	atomic_inc(&pool->overall.live_allocs);
	atomic_add(committed, &pool->overall.live_committed);
	atomic_add(requested, &pool->overall.live_requested);

	atomic_inc(&pool->histogram[hidx].live_allocs);
	atomic_add(committed, &pool->histogram[hidx].live_committed);
	atomic_add(requested, &pool->histogram[hidx].live_requested);
}

/** cpp_log_dealloc() - Track allocation free.
 * @pool: The pool to update the statistics for.
 * @requested: bytes requested.
 * @committed: bytes actually committed.
 *
 * Updates the histogram and overall stats.
 */
static void cpp_log_dealloc(struct cpp_pool *pool, unsigned long requested,
			    unsigned long committed)
{
	unsigned int hidx = cpp_histogram_index(committed);

	atomic_sub(committed, &pool->overall.live_committed);
	atomic_sub(requested, &pool->overall.live_requested);
	atomic_dec(&pool->overall.live_allocs);

	atomic_sub(committed, &pool->histogram[hidx].live_committed);
	atomic_sub(requested, &pool->histogram[hidx].live_requested);
	atomic_dec(&pool->histogram[hidx].live_allocs);
}

/** cpp_stats_debug_print_helper() - Pretty-print bytes.
 * @s: Sequence file to output to.
 * @val: Value to pretty-print.
 *
 * Pretty-prints the value @val with base2 units to the sequence file.
 */
static void cpp_stats_debug_print_helper(struct seq_file *s, u64 val)
{
	char cap_str[24];

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	string_get_size(val, STRING_UNITS_2, cap_str, sizeof(cap_str));
#else
	string_get_size(val, 1, STRING_UNITS_2, cap_str, sizeof(cap_str));
#endif
	seq_printf(s, "%s (%llu)\n", cap_str, val);
}

/**
 * cpp_stats_debug_show() - Display a @cpp_stats object.
 * @stats: The @cpp_stats object to display stats for.
 * @s: The sequence file to use for output.
 */
static void cpp_stats_debug_show(struct cpp_stats *stats, struct seq_file *s)
{
	seq_printf(s, "\t\tTotal number of allocs seen: %u\n",
		   atomic_read(&stats->num_allocs));
	seq_printf(s, "\t\tLive allocations: %u\n",
		   atomic_read(&stats->live_allocs));

	seq_puts(s, "\t\tAccumulated bytes requested: ");
	cpp_stats_debug_print_helper(s, atomic64_read(&stats->bytes_requested));

	seq_puts(s, "\t\tAccumulated bytes committed: ");
	cpp_stats_debug_print_helper(s, atomic64_read(&stats->bytes_committed));

	seq_puts(s, "\t\tLive bytes requested: ");
	cpp_stats_debug_print_helper(s, atomic_read(&stats->live_requested));

	seq_puts(s, "\t\tLive bytes committed: ");
	cpp_stats_debug_print_helper(s, atomic_read(&stats->live_committed));
}

/**
 * cpp_debug_show() - Display pool statistics.
 * @heap: Pointer to the embedded @ion_heap in the @cpp_pool.
 * @s: sequence file to use for output.
 * @unused: Optional payload, not used.
 *
 * Called by the ion core to display pool statistics.
 * Always returns 0.
 */
static int cpp_debug_show(struct ion_heap *heap, struct seq_file *s,
			  void *unused)
{
	struct cpp_pool *pool = container_of(heap,
					     struct cpp_pool,
					     heap);
	int i;
	int count;

	count = atomic_read(&pool->count);

	seq_puts(s, "Free pool:\n");
	seq_printf(s, "\t%u times depleted\n",
		   atomic_read(&pool->depleted));
	seq_printf(s, "\t%u page(s) in pool - ", count);
	cpp_stats_debug_print_helper(s, count << (PAGE_SHIFT + CPP_ORDER));
	seq_printf(s, "\t%u partial(s) in use\n",
		   atomic_read(&pool->num_partials));
	seq_puts(s, "\tUnused in partials - ");
	cpp_stats_debug_print_helper(s,
				     atomic_read(&pool->free_bytes_in_partials));
	seq_puts(s, "Shrink info:\n");
	seq_printf(s, "\tShrunk performed %u time(s)\n",
		   atomic_read(&pool->shrinks));
	seq_printf(s, "\t%u page(s) shrunk in total\n",
		   atomic_read(&pool->pages_shrunk));

	seq_puts(s, "Usage stats:\n");
	seq_printf(s, "\tMax time spent to perform an allocation: %u ns\n",
		   atomic_read(&pool->max_alloc_time));
	seq_printf(s, "\tMax time spent to allocate a single page from kernel: %u ns\n",
		   atomic_read(&pool->max_cp_alloc_time));
	seq_printf(s, "\tSoft alloc failures: %u\n",
		   atomic_read(&pool->num_soft_alloc_failures));
	seq_printf(s, "\tHard alloc failures: %u\n",
		   atomic_read(&pool->num_hard_alloc_failures));

	seq_puts(s, "\tAllocations:\n");
	cpp_stats_debug_show(&pool->overall, s);

	seq_puts(s, "\tDistribution:\n");

	for (i = 0; i < CPP_HISTOGRAM_BINS; i++) {
		if (!atomic_read(&pool->histogram[i].num_allocs))
			continue;
		seq_printf(s, "\t%d page(s):\n", i);
		cpp_stats_debug_show(&pool->histogram[i], s);
	}

	return 0;
}

#else /* CPP_STATS */
#define cpp_log_alloc(a, b, c)
#define cpp_log_dealloc(a, b, c)
#define cpp_log_shrink(a, b)
#endif

/** cpp_drain_pages() - Drain pages
 * @pool: The pool to drain into.
 * @pages: list head for the pages to drain.
 *
 * Until the high-mark is reached pages are zeroed and put into the free pool.
 * Once above the high-mark the pages are freed directly to the kernel.
 *
 * The pool lock is held on entry, but is dropped while clearing the pages.
 * The lock is retaken before the just-cleared pages are exposed for reuse.
 * The lock is held on exit.
 */
static void cpp_drain_pages(struct cpp_pool *pool, struct list_head *pages)
{
	struct page *page, *tmp_page;

	/* as the highmarks are soft, we can calculate the max here */
	ssize_t max = pool->highmark - atomic_read(&pool->count);
	size_t count = 0;
	size_t count_for_pool = 0;

	lockdep_assert_held(&pool->lock);

	/* drop the lock while zeroing and freeing */
	mutex_unlock(&pool->lock);

	if (max < 0)
		max = 0;

	list_for_each_entry_safe(page, tmp_page, pages, lru) {
		int i;

		if (++count > max) {
			list_del(&page->lru);
			__free_pages(page, CPP_ORDER);
			continue;
		}

		for (i = 0; i < CPP_SUB_PAGES; i++)
			clear_highpage(page + i);

		ion_pages_sync_for_device(NULL, page, CPP_PAGE_SIZE,
					  DMA_BIDIRECTIONAL);

		count_for_pool++;
	}

	/* expose the now-clean pages for re-use */
	mutex_lock(&pool->lock);
	list_splice(pages, &pool->pages);
	atomic_add(count_for_pool, &pool->count);
	/* keep the pool locked on exit */
}

/** cpp_alloc_page() - Allocate a compound page from the kernel.
 * @pool: The pool to allocate for.
 *
 * Allocates a compound page and uses the ion API to prepare it for use.
 * If requested the time spent is recorded.
 */
static struct page *cpp_alloc_page(struct cpp_pool *pool)
{
	struct page *page;
#ifdef CPP_STATS
	unsigned long nv;
	unsigned long long start_cycles = sched_clock();
#endif

	page = alloc_pages(gfp_flags, CPP_ORDER);
	if (!page) {
#ifdef CPP_STATS
		atomic_inc(&pool->num_soft_alloc_failures);
#endif
		return NULL;
	}

	INIT_LIST_HEAD(&page->lru);
	ion_pages_sync_for_device(NULL, page, CPP_PAGE_SIZE, DMA_BIDIRECTIONAL);

#ifdef CPP_STATS
	nv = (unsigned long)(sched_clock() - start_cycles);
	while (true) {
		unsigned long ov;
		unsigned long rc;

		ov = atomic_read(&pool->max_cp_alloc_time);
		rc = ov;
		if (nv > ov)
			rc = atomic_cmpxchg(&pool->max_cp_alloc_time, ov, nv);
		if (rc == ov)
			break;
	}
#endif

	return page;
}

/** cpp_refill_pool() - Refill a @cpp_pool.
 * @pool: The pool to refill.
 *
 * Adds new pages to the pool to reach the fill-mark.
 * Called with the lock held, but drops the lock while allocating physical
 * pages.  Retakes the lock again before exposing the pages and returning.
 */
static void cpp_refill_pool(struct cpp_pool *pool)
{
	ssize_t pages_wanted = pool->fillmark - atomic_read(&pool->count);
	size_t i;
	LIST_HEAD(pages);

	lockdep_assert_held(&pool->lock);

	/* drop the lock while allocating */
	mutex_unlock(&pool->lock);

#ifdef CPP_STATS
	if (pool->fillmark == pages_wanted)
		atomic_inc(&pool->depleted);
#endif

	for (i = 0; i < pages_wanted; i++) {
		struct page *page;

		page = cpp_alloc_page(pool);
		if (!page)
			break; /* non-fatal */
		list_add_tail(&page->lru, &pages);
	}
	/* update to reflect how much we could allocate */
	pages_wanted = i;

	/* expose the new memory */
	mutex_lock(&pool->lock);

	atomic_add(pages_wanted, &pool->count);
	list_splice(&pages, &pool->pages);
	/* keep the pool locked on exit */
}

/** cpp_worker() - Worker thread to refill the pool and drain the free queue.
 * @data: The @cpp_pool to operate on.
 *
 * Thread which drives @cpp_refill_pool and @cpp_drain_pages.
 */
static int cpp_worker(void *data)
{
	struct cpp_pool *pool = (struct cpp_pool *)data;
	DECLARE_WAITQUEUE(wait, current);

	mutex_lock(&pool->lock);

	while (1) {
		if (!list_empty(&pool->free_queue)) {
			LIST_HEAD(pages);

			list_splice_init(&pool->free_queue, &pages);
			cpp_drain_pages(pool, &pages);
		}

		if (atomic_read(&pool->count) < pool->lowmark)
			cpp_refill_pool(pool);

		__set_current_state(TASK_INTERRUPTIBLE);
		__add_wait_queue(&pool->worker_wait, &wait);

		mutex_unlock(&pool->lock);

		if (unlikely(kthread_should_stop())) {
			set_task_state(current, TASK_RUNNING);
			remove_wait_queue(&pool->worker_wait, &wait);
			break;
		}

		schedule();

		set_task_state(current, TASK_RUNNING);
		mutex_lock(&pool->lock);
		__remove_wait_queue(&pool->worker_wait, &wait);
	}

	return 0;
}

/**
 * cpp_align_size() - Align requested size.
 * @size: Requested size.
 *
 * If the requested size is less than the compound page size the return value
 * is aligned up to the next page size.
 * For other requests the size is rounded up to be a multiple of the compound
 * page size.
 */
static unsigned long cpp_align_size(unsigned long size)
{
	if (size < CPP_PAGE_SIZE)
		return ALIGN(size, PAGE_SIZE);
	else
		return ALIGN(size, CPP_PAGE_SIZE);
}

/** cpp_get_page_from_pool() - get a page from a @cpp_pool
 * @pool: The @cpp_pool to request a page from.
 *
 * Helper to get the first page in the free pool.
 * Caller must verify that the pool is not empty before calling.
 *
 * Returns the page taken from the pool.
 */
static struct page *cpp_get_page_from_pool(struct cpp_pool *pool)
{
	struct page *page;

	lockdep_assert_held(&pool->lock);
	page = list_first_entry(&pool->pages, struct page, lru);
	list_del_init(&page->lru);
	return page;
}

/**
 * cpp_partial_alloc() - Allocate a partial compound page
 * @pool: the pool to allocated from
 * @bytes: Number of bytes. Must be aligned on a power-of-two boundary
 * @ppage: Where to store a pointer to the compound page allocated
 *
 * Free a partial allocation within a compound page.
 * The arguments must match what was passed to @cpp_partial_alloc.
 *
 * Return:
 * The byte offset within the returned compound page, zero or greater or
 * -errno on failure.
 */
static int cpp_partial_alloc(struct cpp_pool *pool, unsigned long bytes,
			     struct page **ppage)
{
	int order;
	struct page *new_page = NULL;
	struct cpp_sub_alloc *sub_alloc;

	lockdep_assert_held(&pool->lock);

	order = ilog2(bytes) - PAGE_SHIFT;

	list_for_each_entry(sub_alloc, &pool->partials, link) {
		int pos = bitmap_find_free_region(sub_alloc->sub_pages,
						  CPP_SUB_PAGES, order);

		if (pos >= 0) {
#ifdef CPP_STATS
			atomic_sub(bytes, &pool->free_bytes_in_partials);
#endif
			*ppage = sub_alloc->page;
			return pos << PAGE_SHIFT;
		}
	}

	/* no existing partial found, try to allocate a new one */
	if (!list_empty(&pool->pages)) {
		new_page = cpp_get_page_from_pool(pool);
		if (atomic_dec_return(&pool->count) < pool->lowmark)
			wake_up(&pool->worker_wait);
	}

	if (!new_page) {
		new_page = cpp_alloc_page(pool);
		wake_up(&pool->worker_wait);
	}

	if (!new_page)
		return -ENOMEM;

	sub_alloc = kzalloc(sizeof(*sub_alloc), GFP_KERNEL);
	if (!sub_alloc)
		goto no_sub_alloc;

	bitmap_allocate_region(sub_alloc->sub_pages, 0, order);

	INIT_LIST_HEAD(&sub_alloc->link);
	sub_alloc->page = new_page;
	set_page_private(new_page, (unsigned long)sub_alloc);

#ifdef CPP_STATS
	atomic_add(CPP_PAGE_SIZE - bytes, &pool->free_bytes_in_partials);
	atomic_inc(&pool->num_partials);
#endif

	list_add(&sub_alloc->link, &pool->partials);

	*ppage = new_page;

	return 0;

no_sub_alloc:
	/* as we failed to allocate memory, let's free directly
	 * back to the kernel
	 */
	__free_pages(new_page, CPP_ORDER);
	return -ENOMEM;
}

/**
 * cpp_partial_free() - free a partial compound page allocation
 * @pool: the pool the partial was allocated from
 * @page: compound page previously returned from @cpp_partial_alloc
 * @bytes: Number of bytes. Must match the argument to @cpp_partial_alloc
 * @offset: Offset in bytes. Must match the argument to @cpp_partial_alloc
 *
 * Free a partial allocation within a compound page.
 * The arguments must match what was passed to @cpp_partial_alloc.
 */
static void cpp_partial_free(struct cpp_pool *pool, struct page *page,
			     unsigned long bytes, int offset)
{
	struct cpp_sub_alloc *sub_alloc;
	int order;

	lockdep_assert_held(&pool->lock);

	sub_alloc = (struct cpp_sub_alloc *)page_private(page);
	order = ilog2(bytes) - PAGE_SHIFT;
	bitmap_release_region(sub_alloc->sub_pages, offset >> PAGE_SHIFT,
			      order);
	if (bitmap_empty(sub_alloc->sub_pages, CPP_SUB_PAGES)) {
		/* partial has no clients, freeing */
#ifdef CPP_STATS
		atomic_dec(&pool->num_partials);
		atomic_sub(CPP_PAGE_SIZE - bytes,
			   &pool->free_bytes_in_partials);
#endif
		list_del(&sub_alloc->link);
		list_add(&sub_alloc->page->lru, &pool->free_queue);
		wake_up(&pool->worker_wait);
		kfree(sub_alloc);
	} else {
		/* need to zero pages before */
		int i;
		int range_start = offset >> PAGE_SHIFT;
		int range_end = range_start + (bytes >> PAGE_SHIFT);

		for (i = range_start; i < range_end; i++)
			clear_highpage(sub_alloc->page + i);
#ifdef CPP_STATS
		atomic_add(bytes, &pool->free_bytes_in_partials);
#endif
	}
}

/** cpp_alloc() - ion API entry-point to allocate for a new buffer.
 * @heap: Pointer to the embedded @ion_heap in the @cpp_pool.
 * @buffer: The buffer to allocate for
 * @size: Requested length
 * @align: Requested alignment
 * @flags: Flags to control the allocation
 *
 * Rounds up the requested size using @cpp_align_size to better fit the pool.
 * Flags is not used. Alignment is always set to the compound page size, so the
 * requested alignment is not honored.
 *
 * Pages are taken from the pool. If the pool is exhausted then direct kernel
 * allocations is attempted. If @cpp_align_size made the buffer not a multiple
 * of the compound page size, a partial compound page is added as the last page.
 *
 * If the free pool is detected to go below the low-mark then the worker thread
 * is requested to re-fill the pool.
 *
 * Returns 0 on success, -errno on failure.
 */
static int cpp_alloc(struct ion_heap *heap,
		     struct ion_buffer *buffer, unsigned long size,
		     unsigned long align, unsigned long flags)
{
	struct sg_table *table;
	struct scatterlist *sg;
	LIST_HEAD(pages);
	struct page *page, *tmp_page;
	unsigned long committed_size = cpp_align_size(size);
	unsigned long size_remaining = committed_size;
	unsigned int nents = ALIGN(size_remaining, CPP_PAGE_SIZE) >>
		(CPP_ORDER + PAGE_SHIFT);
	struct cpp_pool *pool = container_of(heap,
					     struct cpp_pool,
					     heap);

	int err;
#ifdef CPP_STATS
	unsigned long long start_time;
	unsigned long nv;

	start_time = sched_clock();
#endif

	if (align > PAGE_SIZE)
		return -EINVAL;

	if (size / PAGE_SIZE > totalram_pages / 2)
		return -ENOMEM;

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table)
		goto no_sg_table;

	err = sg_alloc_table(table, nents, GFP_KERNEL);
	if (err)
		goto no_sg_alloc;

	sg = table->sgl;

	mutex_lock(&pool->lock);

	/* pull from our pool first */
	while ((roundup_pow_of_two(size_remaining) >= CPP_PAGE_SIZE) &&
	       !list_empty(&pool->pages)) {
		struct page *page;

		page = cpp_get_page_from_pool(pool);
		list_add_tail(&page->lru, &pages);
		size_remaining -= min(size_remaining, CPP_PAGE_SIZE);
		atomic_dec(&pool->count);

		sg_set_page(sg, page, CPP_PAGE_SIZE, 0);
		sg = sg_next(sg);
	}

	mutex_unlock(&pool->lock);

	/* do we need to pull in from the kernel? */
	while (roundup_pow_of_two(size_remaining) >= CPP_PAGE_SIZE) {
		struct page *page = cpp_alloc_page(pool);

		if (!page)
			goto rollback; /* fatal error */

		list_add_tail(&page->lru, &pages);

		sg_set_page(sg, page, min(CPP_PAGE_SIZE, size_remaining), 0);
		sg = sg_next(sg);
		size_remaining -= min(CPP_PAGE_SIZE, size_remaining);
	}

	/* add a partial if anything left */
	if (size_remaining) {
		struct page *page;
		int offset;
		int tail_size = roundup_pow_of_two(size_remaining);

		mutex_lock(&pool->lock);
		offset = cpp_partial_alloc(pool, tail_size, &page);
		mutex_unlock(&pool->lock);
		if (offset >= 0) {
			sg_set_page(sg, page, size_remaining, offset);
			/* account for the pow2 alignment */
			committed_size += (tail_size - size_remaining);
			size_remaining = 0;
		}
	}

	if (size_remaining)
		goto rollback;
	else if (atomic_read(&pool->count) < pool->lowmark)
		wake_up(&pool->worker_wait);

	list_del(&pages);

	buffer->priv_virt = table;
#ifdef CPP_STATS
	nv = sched_clock() - start_time;
	while (true) {
		unsigned long ov = atomic_read(&pool->max_alloc_time);
		unsigned long rc = ov;

		if (ov < nv)
			rc = atomic_cmpxchg(&pool->max_alloc_time, ov, nv);
		if (rc == ov)
			break;
	}
#endif

	cpp_log_alloc(pool, size, committed_size);

	return 0;

rollback:
#ifdef CPP_STATS
	atomic_inc(&pool->num_hard_alloc_failures);
#endif
	/* as we failed to allocate memory, let's free directly
	 * back to the kernel
	 */
	list_for_each_entry_safe(page, tmp_page, &pages, lru) {
		__free_pages(page, CPP_ORDER);
	}
	sg_free_table(table);
no_sg_alloc:
	kfree(table);
no_sg_table:
	return -ENOMEM;
}

/** cpp_free() - ion API entry-point to free a buffer.
 * @buffer: The buffer to free
 */
static void cpp_free(struct ion_buffer *buffer)
{
	struct sg_table *table = buffer->priv_virt;
	struct cpp_pool *pool = container_of(buffer->heap,
			struct cpp_pool,
			heap);
	struct scatterlist *sg;
	struct page *head;
	unsigned int tail_extra = 0;

	head = sg_page(table->sgl);
	sg = sg_last(table->sgl, table->nents);

	mutex_lock(&pool->lock);

	/* have a chain only if buffer is backed with >= CPP_PAGE_SIZE.
	 * As each sg is max CPP_PAGE_SIZE we can do a simple check.
	 */
	if (table->sgl->length == CPP_PAGE_SIZE) {
		/* move 1..N */
		list_splice(&head->lru, &pool->free_queue);
		/* move 0 */
		list_add(&head->lru, &pool->free_queue);
		wake_up(&pool->worker_wait);
	}

	/* any tail sub-allocated page? */
	if (sg->length & (CPP_PAGE_SIZE - 1)) {
		unsigned long len = roundup_pow_of_two(sg->length);

		tail_extra = len - sg->length;
		cpp_partial_free(pool, sg_page(sg), len, sg->offset);
	}

	cpp_log_dealloc(pool, buffer->size, cpp_align_size(buffer->size) +
			tail_extra);

	mutex_unlock(&pool->lock);

	sg_free_table(table);
	kfree(table);
}

/** cpp_map_dma() - ion API entry-point for DMA map.
 * @heap: The ion heap the buffer belongs to.
 * @buffer: The buffer to map.
 *
 * Just returns the already mapped sg_table stored in the buffer.
 */
static struct sg_table *cpp_map_dma(struct ion_heap *heap,
				    struct ion_buffer *buffer)
{
	return buffer->priv_virt;
}

/** cpp_unmap_dma() - ion API entry-point for DMA unmap.
 * @heap: The ion heap the buffer belongs to
 * @buffer: The buffer to unmap.
 *
 * A no-op for us.
 */
static void cpp_unmap_dma(struct ion_heap *heap, struct ion_buffer *buffer)
{
}

static struct ion_heap_ops cpp_ops = {
	.allocate = cpp_alloc,
	.free = cpp_free,
	.map_dma = cpp_map_dma,
	.unmap_dma = cpp_unmap_dma,

	/* use default heap functions for the kernel/user map functions */
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
	.map_user = ion_heap_map_user,
};

/** cpp_shrink_count() - Query the number of objects we can free.
 * @shrinker: The @shrinker object embedded in a @cpp_pool.
 * @sc: Not used.
 *
 * Returns the number of elements in the free list.
 */
static unsigned long cpp_shrink_count(struct shrinker *shrinker,
				      struct shrink_control *sc)
{
	struct cpp_pool *pool = container_of(shrinker,
			struct cpp_pool, shrinker);

	/* return the number of compound pages we have */
	return atomic_read(&pool->count);
}

/** cpp_shrink_scan() - Free objects on the free list.
 * @shrinker: The @shrinker object embedded in a @cpp_pool.
 * @sc: Information about how many objects to try to free.
 *
 * Returns how many objects we could free, or @SHRINK_STOP
 * if none could be freed.
 */
static unsigned long cpp_shrink_scan(struct shrinker *shrinker,
				     struct shrink_control *sc)
{
	struct cpp_pool *pool = container_of(shrinker,
			struct cpp_pool, shrinker);

	unsigned long freed = 0;
	unsigned long to_scan = sc->nr_to_scan;

	if (!mutex_trylock(&pool->lock))
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
		return -1;
#else
		return SHRINK_STOP;
#endif

	while (to_scan && !list_empty(&pool->pages)) {
		struct page *page;

		page = cpp_get_page_from_pool(pool);
		__free_pages(page, CPP_ORDER);
		freed++;
		to_scan--;
	}

	atomic_sub(freed, &pool->count);

	mutex_unlock(&pool->lock);
	if (freed) {
		cpp_log_shrink(pool, freed);
		return freed;
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
	return -1;
#else
	return SHRINK_STOP;
#endif
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
/**
 * cpp_shrink_wrapper() - Shrinker API wrapper for pre 3.12 kernels.
 * @shrinker: The shrinker object embedded in a @cpp_pool.
 * @sc: Describes the operation to perform.
 *
 * Wraps the pre 3.12 API which had a single entry-point for both
 * count and scan.
 * Returns what the respective back-end functions return.
 */
static int cpp_shrink_wrapper(struct shrinker *shrinker,
			      struct shrink_control *sc)
{
	if (sc->nr_to_scan == 0)
		return cpp_shrink_count(shrinker, sc);
	else
		return cpp_shrink_scan(shrinker, sc);
}
#endif

/**
 * ion_compound_page_pool_create() - Create a new compound page pool.
 * @unused: Pointer to platform data for the heap, not used.
 *
 * Initialize a new heap. Fills the pool with pages asynchronously.
 * On success a pointer to the embedded @ion_heap object is returned,
 * -errno on failure.
 */
struct ion_heap *ion_compound_page_pool_create(struct ion_platform_heap *unused)
{
	struct cpp_pool *pool;
	static int pools;
	int err = 0;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);

	if (!pool)
		return ERR_PTR(-ENOMEM);

	mutex_init(&pool->lock);

	pool->lowmark = pool_page_lowmark;
	pool->highmark = pool_page_highmark;
	pool->fillmark = pool_page_fillmark;

	INIT_LIST_HEAD(&pool->pages);
	INIT_LIST_HEAD(&pool->free_queue);
	INIT_LIST_HEAD(&pool->partials);

	pool->heap.ops = &cpp_ops;
	pool->heap.type = ION_HEAP_TYPE_COMPOUND_PAGE;
#ifdef CPP_STATS
	pool->heap.debug_show = cpp_debug_show;
#endif

	/* our shrinker */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
	pool->shrinker.shrink = cpp_shrink_wrapper;
#else
	pool->shrinker.count_objects = cpp_shrink_count;
	pool->shrinker.scan_objects = cpp_shrink_scan;
#endif
	pool->shrinker.seeks = DEFAULT_SEEKS;
	pool->shrinker.batch = 2;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 12, 0)
	register_shrinker(&pool->shrinker);
#else
	err = register_shrinker(&pool->shrinker);
	if (err)
		goto no_shrinker;
#endif

	init_waitqueue_head(&pool->worker_wait);

	pool->worker = kthread_run(cpp_worker, pool,
			"ion-compound-pool-worker-%d", pools++);
	if (IS_ERR(pool->worker)) {
		err = PTR_ERR(pool->worker);
		goto no_worker;
	}

	/* trigger async refill */
	wake_up(&pool->worker_wait);

	return &pool->heap;

no_worker:
	unregister_shrinker(&pool->shrinker);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
no_shrinker:
#endif
	kfree(pool);
	return ERR_PTR(err);
}

/**
 * ion_compound_page_pool_destroy() - Destroy a compound page pool.
 * @heap: The heap to destroy
 */
void ion_compound_page_pool_destroy(struct ion_heap *heap)
{
	struct cpp_pool *pool = container_of(heap,
			struct cpp_pool,
			heap);
	struct page *page, *tmp_page;
	struct cpp_sub_alloc *sub_alloc, *tmp_sub_alloc;

	/* signal worker to stop */
	kthread_stop(pool->worker);

	unregister_shrinker(&pool->shrinker);

	/* cleanup */
	list_for_each_entry_safe(page, tmp_page, &pool->pages, lru) {
		list_del(&page->lru);
		__free_pages(page, CPP_ORDER);
	}

	list_for_each_entry_safe(sub_alloc, tmp_sub_alloc, &pool->partials,
				 link) {
		list_del(&sub_alloc->link);
		__free_pages(sub_alloc->page, CPP_ORDER);
		kfree(sub_alloc);
	}

	kfree(pool);
}
