/*
 * Contiguous Memory Allocator for DMA mapping framework
 * Copyright (c) 2010-2011 by Samsung Electronics.
 * Written by:
 *	Marek Szyprowski <m.szyprowski@samsung.com>
 *	Michal Nazarewicz <mina86@mina86.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License or (at your optional) any later version of the license.
 */

#define pr_fmt(fmt) "cma: " fmt

#ifdef CONFIG_CMA_DEBUG
#ifndef DEBUG
#  define DEBUG
#endif
#endif

#include <asm/page.h>
#include <asm/dma-contiguous.h>

#include <linux/memblock.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/page-isolation.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/mm_types.h>
#include <linux/dma-contiguous.h>

#ifndef CONFIG_MTK_SVP

struct cma {
	unsigned long	base_pfn;
	unsigned long	count;
	unsigned long	*bitmap;
};

struct cma *dma_contiguous_default_area;

#ifdef CONFIG_CMA_SIZE_MBYTES
#define CMA_SIZE_MBYTES CONFIG_CMA_SIZE_MBYTES
#else
#define CMA_SIZE_MBYTES 0
#endif

/*
 * Default global CMA area size can be defined in kernel's .config.
 * This is usefull mainly for distro maintainers to create a kernel
 * that works correctly for most supported systems.
 * The size can be set in bytes or as a percentage of the total memory
 * in the system.
 *
 * Users, who want to set the size of global CMA area for their system
 * should use cma= kernel parameter.
 */
static const phys_addr_t size_bytes = CMA_SIZE_MBYTES * SZ_1M;
static phys_addr_t size_cmdline = -1;

static int __init early_cma(char *p)
{
	pr_debug("%s(%s)\n", __func__, p);
	size_cmdline = memparse(p, &p);
	return 0;
}
early_param("cma", early_cma);

#ifdef CONFIG_CMA_SIZE_PERCENTAGE

static phys_addr_t __init __maybe_unused cma_early_percent_memory(void)
{
	struct memblock_region *reg;
	unsigned long total_pages = 0;

	/*
	 * We cannot use memblock_phys_mem_size() here, because
	 * memblock_analyze() has not been called yet.
	 */
	for_each_memblock(memory, reg)
		total_pages += memblock_region_memory_end_pfn(reg) -
			       memblock_region_memory_base_pfn(reg);

	return (total_pages * CONFIG_CMA_SIZE_PERCENTAGE / 100) << PAGE_SHIFT;
}

#else

static inline __maybe_unused phys_addr_t cma_early_percent_memory(void)
{
	return 0;
}

#endif

/**
 * dma_contiguous_reserve() - reserve area for contiguous memory handling
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory from early allocator. It should be
 * called by arch specific code once the early allocator (memblock or bootmem)
 * has been activated and all other subsystems have already allocated/reserved
 * memory.
 */
void __init dma_contiguous_reserve(phys_addr_t limit)
{
	phys_addr_t selected_size = 0;

	pr_debug("%s(limit %08lx)\n", __func__, (unsigned long)limit);

	if (size_cmdline != -1) {
		selected_size = size_cmdline;
	} else {
#ifdef CONFIG_CMA_SIZE_SEL_MBYTES
		selected_size = size_bytes;
#elif defined(CONFIG_CMA_SIZE_SEL_PERCENTAGE)
		selected_size = cma_early_percent_memory();
#elif defined(CONFIG_CMA_SIZE_SEL_MIN)
		selected_size = min(size_bytes, cma_early_percent_memory());
#elif defined(CONFIG_CMA_SIZE_SEL_MAX)
		selected_size = max(size_bytes, cma_early_percent_memory());
#endif
	}

	if (selected_size) {
		pr_debug("%s: reserving %ld MiB for global area\n", __func__,
			 (unsigned long)selected_size / SZ_1M);

		dma_declare_contiguous(NULL, selected_size, 0, limit);
	}
};

static DEFINE_MUTEX(cma_mutex);

static __init int cma_activate_area(unsigned long base_pfn, unsigned long count)
{
	unsigned long pfn = base_pfn;
	unsigned i = count >> pageblock_order;
	struct zone *zone;

	WARN_ON_ONCE(!pfn_valid(pfn));
	zone = page_zone(pfn_to_page(pfn));

	do {
		unsigned j;
		base_pfn = pfn;
		for (j = pageblock_nr_pages; j; --j, pfn++) {
			WARN_ON_ONCE(!pfn_valid(pfn));
			if (page_zone(pfn_to_page(pfn)) != zone)
				return -EINVAL;
		}
		init_cma_reserved_pageblock(pfn_to_page(base_pfn));
	} while (--i);
	return 0;
}

static __init struct cma *cma_create_area(unsigned long base_pfn,
				     unsigned long count)
{
	int bitmap_size = BITS_TO_LONGS(count) * sizeof(long);
	struct cma *cma;
	int ret = -ENOMEM;

	pr_debug("%s(base %08lx, count %lx)\n", __func__, base_pfn, count);

	cma = kmalloc(sizeof *cma, GFP_KERNEL);
	if (!cma)
		return ERR_PTR(-ENOMEM);

	cma->base_pfn = base_pfn;
	cma->count = count;
	cma->bitmap = kzalloc(bitmap_size, GFP_KERNEL);

	if (!cma->bitmap)
		goto no_mem;

	ret = cma_activate_area(base_pfn, count);
	if (ret)
		goto error;

	pr_debug("%s: returned %p\n", __func__, (void *)cma);
	return cma;

error:
	kfree(cma->bitmap);
no_mem:
	kfree(cma);
	return ERR_PTR(ret);
}

static struct cma_reserved {
	phys_addr_t start;
	unsigned long size;
	struct device *dev;
} cma_reserved[MAX_CMA_AREAS] __initdata;
static unsigned cma_reserved_count __initdata;

static int __init cma_init_reserved_areas(void)
{
	struct cma_reserved *r = cma_reserved;
	unsigned i = cma_reserved_count;

	pr_debug("%s()\n", __func__);

	for (; i; --i, ++r) {
		struct cma *cma;
		cma = cma_create_area(PFN_DOWN(r->start),
				      r->size >> PAGE_SHIFT);
		if (!IS_ERR(cma))
			dev_set_cma_area(r->dev, cma);
	}
	return 0;
}
core_initcall(cma_init_reserved_areas);

/**
 * dma_declare_contiguous() - reserve area for contiguous memory handling
 *			      for particular device
 * @dev:   Pointer to device structure.
 * @size:  Size of the reserved memory.
 * @base:  Start address of the reserved memory (optional, 0 for any).
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory for specified device. It should be
 * called by board specific code when early allocator (memblock or bootmem)
 * is still activate.
 */
int __init dma_declare_contiguous(struct device *dev, phys_addr_t size,
				  phys_addr_t base, phys_addr_t limit)
{
	struct cma_reserved *r = &cma_reserved[cma_reserved_count];
	phys_addr_t alignment;

	pr_debug("%s(size %lx, base %08lx, limit %08lx)\n", __func__,
		 (unsigned long)size, (unsigned long)base,
		 (unsigned long)limit);

	/* Sanity checks */
	if (cma_reserved_count == ARRAY_SIZE(cma_reserved)) {
		pr_err("Not enough slots for CMA reserved regions!\n");
		return -ENOSPC;
	}

	if (!size)
		return -EINVAL;

	/* Sanitise input arguments */
	alignment = PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	base = ALIGN(base, alignment);
	size = ALIGN(size, alignment);
	limit &= ~(alignment - 1);

	/* Reserve memory */
	if (base) {
		if (memblock_is_region_reserved(base, size) ||
		    memblock_reserve(base, size) < 0) {
			base = -EBUSY;
			goto err;
		}
	} else {
		/*
		 * Use __memblock_alloc_base() since
		 * memblock_alloc_base() panic()s.
		 */
		phys_addr_t addr = __memblock_alloc_base(size, alignment, limit);
		if (!addr) {
			base = -ENOMEM;
			goto err;
		} else {
			base = addr;
		}
	}

	/*
	 * Each reserved area must be initialised later, when more kernel
	 * subsystems (like slab allocator) are available.
	 */
	r->start = base;
	r->size = size;
	r->dev = dev;
	cma_reserved_count++;
	pr_info("CMA: reserved %ld MiB at %08lx\n", (unsigned long)size / SZ_1M,
		(unsigned long)base);

	/* Architecture specific contiguous memory fixup. */
	dma_contiguous_early_fixup(base, size);
	return 0;
err:
	pr_err("CMA: failed to reserve %ld MiB\n", (unsigned long)size / SZ_1M);
	return base;
}

/**
 * dma_alloc_from_contiguous() - allocate pages from contiguous area
 * @dev:   Pointer to device for which the allocation is performed.
 * @count: Requested number of pages.
 * @align: Requested alignment of pages (in PAGE_SIZE order).
 *
 * This function allocates memory buffer for specified device. It uses
 * device specific contiguous memory area if available or the default
 * global one. Requires architecture specific get_dev_cma_area() helper
 * function.
 */
struct page *dma_alloc_from_contiguous(struct device *dev, int count,
				       unsigned int align)
{
	unsigned long mask, pfn, pageno, start = 0;
	struct cma *cma = dev_get_cma_area(dev);
	struct page *page = NULL;
	int ret;

	if (!cma || !cma->count)
		return NULL;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	pr_debug("%s(cma %p, count %d, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
		return NULL;

	mask = (1 << align) - 1;

	mutex_lock(&cma_mutex);

	for (;;) {
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    start, count, mask);
		if (pageno >= cma->count)
			break;

		pfn = cma->base_pfn + pageno;
		ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA);
		if (ret == 0) {
			bitmap_set(cma->bitmap, pageno, count);
			page = pfn_to_page(pfn);
			break;
		} else if (ret != -EBUSY) {
			break;
		}
		pr_debug("%s(): memory range at %p is busy, retrying\n",
			 __func__, pfn_to_page(pfn));
		/* try again with a bit different memory target */
		start = pageno + mask + 1;
	}

	mutex_unlock(&cma_mutex);
	pr_debug("%s(): returned %p\n", __func__, page);
	return page;
}

/**
 * dma_release_from_contiguous() - release allocated pages
 * @dev:   Pointer to device for which the pages were allocated.
 * @pages: Allocated pages.
 * @count: Number of allocated pages.
 *
 * This function releases memory allocated by dma_alloc_from_contiguous().
 * It returns false when provided pages do not belong to contiguous area and
 * true otherwise.
 */
bool dma_release_from_contiguous(struct device *dev, struct page *pages,
				 int count)
{
	struct cma *cma = dev_get_cma_area(dev);
	unsigned long pfn;

	if (!cma || !pages)
		return false;

	pr_debug("%s(page %p)\n", __func__, (void *)pages);

	pfn = page_to_pfn(pages);

	if (pfn < cma->base_pfn || pfn >= cma->base_pfn + cma->count)
		return false;

	VM_BUG_ON(pfn + count > cma->base_pfn + cma->count);

	mutex_lock(&cma_mutex);
	bitmap_clear(cma->bitmap, pfn - cma->base_pfn, count);
	free_contig_range(pfn, count);
	mutex_unlock(&cma_mutex);

	return true;
}

#else

struct cma {
	unsigned long	base_pfn;
	unsigned long	count;
	unsigned long	*bitmap;
	unsigned long	end_pfn;
};

struct cma *dma_contiguous_default_area;

#ifdef CONFIG_CMA_SIZE_MBYTES
#define CMA_SIZE_MBYTES CONFIG_CMA_SIZE_MBYTES
#else
#define CMA_SIZE_MBYTES 0
#endif

/*
 * Default global CMA area size can be defined in kernel's .config.
 * This is useful mainly for distro maintainers to create a kernel
 * that works correctly for most supported systems.
 * The size can be set in bytes or as a percentage of the total memory
 * in the system.
 *
 * Users, who want to set the size of global CMA area for their system
 * should use cma= kernel parameter.
 */
static const phys_addr_t size_bytes = CMA_SIZE_MBYTES * SZ_1M;
static phys_addr_t size_cmdline = -1;

static int __init early_cma(char *p)
{
	pr_debug("%s(%s)\n", __func__, p);
	size_cmdline = memparse(p, &p);
	return 0;
}
early_param("cma", early_cma);

#ifdef CONFIG_CMA_SIZE_PERCENTAGE

static phys_addr_t __init __maybe_unused cma_early_percent_memory(void)
{
	struct memblock_region *reg;
	unsigned long total_pages = 0;

	/*
	 * We cannot use memblock_phys_mem_size() here, because
	 * memblock_analyze() has not been called yet.
	 */
	for_each_memblock(memory, reg)
		total_pages += memblock_region_memory_end_pfn(reg) -
			       memblock_region_memory_base_pfn(reg);

	return (total_pages * CONFIG_CMA_SIZE_PERCENTAGE / 100) << PAGE_SHIFT;
}

#else

static inline __maybe_unused phys_addr_t cma_early_percent_memory(void)
{
	return 0;
}

#endif

#ifdef CONFIG_DMA_CMA
/**
 * dma_contiguous_reserve() - reserve area(s) for contiguous memory handling
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory from early allocator. It should be
 * called by arch specific code once the early allocator (memblock or bootmem)
 * has been activated and all other subsystems have already allocated/reserved
 * memory.
 */
void __init dma_contiguous_reserve(phys_addr_t limit)
{
	phys_addr_t selected_size = 0;

	pr_debug("%s(limit %08lx)\n", __func__, (unsigned long)limit);

	if (size_cmdline != -1) {
		selected_size = size_cmdline;
	} else {
#ifdef CONFIG_CMA_SIZE_SEL_MBYTES
		selected_size = size_bytes;
#elif defined(CONFIG_CMA_SIZE_SEL_PERCENTAGE)
		selected_size = cma_early_percent_memory();
#elif defined(CONFIG_CMA_SIZE_SEL_MIN)
		selected_size = min(size_bytes, cma_early_percent_memory());
#elif defined(CONFIG_CMA_SIZE_SEL_MAX)
		selected_size = max(size_bytes, cma_early_percent_memory());
#endif
	}

	if (selected_size && !dma_contiguous_default_area) {
		pr_debug("%s: reserving %ld MiB for global area\n", __func__,
			 (unsigned long)selected_size / SZ_1M);

		dma_contiguous_reserve_area(selected_size, 0, limit,
					    &dma_contiguous_default_area);
	}
};
#endif

static DEFINE_MUTEX(cma_mutex);

// SVP 14
static unsigned long __initdata stealed_pages[MAX_NUMNODES][MAX_NR_ZONES];

static int __init cma_activate_area(struct cma *cma)
{
	int bitmap_size = BITS_TO_LONGS(cma->count) * sizeof(long);
	unsigned long base_pfn = cma->base_pfn, pfn = base_pfn;
	unsigned i = cma->count >> pageblock_order;
// SVP 13
//	struct zone *zone;
	int nid;
// SVP 14
	int zone_index;

	cma->bitmap = kzalloc(bitmap_size, GFP_KERNEL);

	if (!cma->bitmap)
		return -ENOMEM;

	WARN_ON_ONCE(!pfn_valid(pfn));
// SVP 13
//	zone = page_zone(pfn_to_page(pfn));
	nid = page_to_nid(pfn_to_page(pfn));

	do {
		unsigned j;
		base_pfn = pfn;
		for (j = pageblock_nr_pages; j; --j, pfn++) {
			WARN_ON_ONCE(!pfn_valid(pfn));
			/*
			 * alloc_contig_range requires the pfn range
// SVP 13
//			 * specified to be in the same zone. Make this
//			 * simple by forcing the entire CMA resv range
//			 * to be in the same zone.
			 * specified to be in the same zone. We will
			 * achieve this goal by stealing pages from
			 * oridinary zone to ZONE_CMA. But, we need
			 * to make sure that entire CMA resv range to
			 * be in the same node. Otherwise, they could
			 * be on ZONE_CMA of different node.
			 */
// SVP 13
//			if (page_zone(pfn_to_page(pfn)) != zone)
			if (page_to_nid(pfn_to_page(pfn)) != nid)
				goto err;
		}
// SVP 14
		zone_index = zone_idx(page_zone(pfn_to_page(base_pfn)));
		stealed_pages[nid][zone_index] += pageblock_nr_pages;
// SVP 13
//		init_cma_reserved_pageblock(pfn_to_page(base_pfn));
		init_cma_reserved_pageblock(base_pfn);
	} while (--i);

// SVP 13
	/*
	 * ZONE_CMA steals some managed pages from other zones,
	 * so we need to re-calculate pcp count for all zones.
	 */
	recalc_per_cpu_pageset();

	return 0;

err:
	kfree(cma->bitmap);
	return -EINVAL;
}

static struct cma cma_areas[MAX_CMA_AREAS];
static unsigned cma_area_count;

static int __init cma_init_reserved_areas(void)
{
// SVP 14
//	int i;
	int i, j;
	pg_data_t *pgdat;
	struct zone *zone;

	for (i = 0; i < cma_area_count; i++) {
		int ret = cma_activate_area(&cma_areas[i]);
		if (ret)
			return ret;
	}

// SVP 14
	for (i = 0; i < MAX_NUMNODES; i++) {
		for (j = 0; j < MAX_NR_ZONES; j++) {
			if (stealed_pages[i][j])
				goto print;
		}
		continue;

print:
		pgdat = NODE_DATA(i);
		for (j = 0; j < MAX_NR_ZONES; j++) {
			if (!stealed_pages[i][j])
				continue;

			zone = pgdat->node_zones + j;
			pr_alert("Steal %lu pages from %s\n",
				stealed_pages[i][j], zone->name);
		}
	}

	return 0;
}
core_initcall(cma_init_reserved_areas);

/**
 * dma_contiguous_reserve_area() - reserve custom contiguous area
 * @size: Size of the reserved area (in bytes),
 * @base: Base address of the reserved area optional, use 0 for any
 * @limit: End address of the reserved memory (optional, 0 for any).
 * @res_cma: Pointer to store the created cma region.
 *
 * This function reserves memory from early allocator. It should be
 * called by arch specific code once the early allocator (memblock or bootmem)
 * has been activated and all other subsystems have already allocated/reserved
 * memory. This function allows to create custom reserved areas for specific
 * devices.
 */
int __init dma_contiguous_reserve_area(phys_addr_t size, phys_addr_t base,
				       phys_addr_t limit, struct cma **res_cma)
{
	struct cma *cma = &cma_areas[cma_area_count];
	phys_addr_t alignment;
	int ret = 0;

	pr_alert("%s(size %lx, base %08lx, limit %08lx)\n", __func__,
		 (unsigned long)size, (unsigned long)base,
		 (unsigned long)limit);

	/* Sanity checks */
	if (cma_area_count == ARRAY_SIZE(cma_areas)) {
		pr_err("Not enough slots for CMA reserved regions!\n");
		return -ENOSPC;
	}

	if (!size)
		return -EINVAL;

	/* Sanitise input arguments */
	alignment = PAGE_SIZE << max(MAX_ORDER - 1, pageblock_order);
	base = ALIGN(base, alignment);
	size = ALIGN(size, alignment);
	limit &= ~(alignment - 1);

	/* Reserve memory */
	if (base) {
		if (memblock_is_region_reserved(base, size) ||
		    memblock_reserve(base, size) < 0) {
			ret = -EBUSY;
			goto err;
		}
	} else {
		/*
		 * Use __memblock_alloc_base() since
		 * memblock_alloc_base() panic()s.
		 */
		phys_addr_t addr = __memblock_alloc_base(size, alignment, limit);
		if (!addr) {
			ret = -ENOMEM;
			goto err;
		} else {
			base = addr;
		}
	}

	/*
	 * Each reserved area must be initialised later, when more kernel
	 * subsystems (like slab allocator) are available.
	 */
	cma->base_pfn = PFN_DOWN(base);
	cma->count = size >> PAGE_SHIFT;
	cma->end_pfn = cma->base_pfn + cma->count;
	*res_cma = cma;
	cma_area_count++;

	pr_alert("CMA: reserved %ld MiB at %08lx\n", (unsigned long)size / SZ_1M,
		(unsigned long)base);

	/* Architecture specific contiguous memory fixup. */
	dma_contiguous_early_fixup(base, size);
	return 0;
err:
	pr_err("CMA: failed to reserve %ld MiB\n", (unsigned long)size / SZ_1M);
	return ret;
}

/**
 * dma_alloc_from_contiguous() - allocate pages from contiguous area
 * @dev:   Pointer to device for which the allocation is performed.
 * @count: Requested number of pages.
 * @align: Requested alignment of pages (in PAGE_SIZE order).
 *
 * This function allocates memory buffer for specified device. It uses
 * device specific contiguous memory area if available or the default
 * global one. Requires architecture specific get_dev_cma_area() helper
 * function.
 */
struct page *dma_alloc_from_contiguous_org(struct device *dev, int count,
				       unsigned int align)
{
	unsigned long mask, pfn, pageno, start = 0;
	struct cma *cma = dev_get_cma_area(dev);
	struct page *page = NULL;
	int ret;

	if (!cma || !cma->count)
		return NULL;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	pr_debug("%s(cma %p, count %d, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
		return NULL;

	mask = (1 << align) - 1;

	mutex_lock(&cma_mutex);

	for (;;) {
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    start, count, mask);
		if (pageno >= cma->count) { pr_alert("%s %d: %lu %lu\n", __func__, __LINE__, mask, pageno);
			break; }

		pfn = cma->base_pfn + pageno;
// SVP 16
//		ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA, 0);
		ret = alloc_contig_range(pfn, pfn + count, MIGRATE_MOVABLE, 0);
		if (ret == 0) {
			bitmap_set(cma->bitmap, pageno, count);
			page = pfn_to_page(pfn);

			break;
		} else if (ret != -EBUSY) {
			pr_alert("%s %d: %d\n", __func__, __LINE__, ret);
			break;
		}
		pr_debug("%s(): memory range at %p is busy, retrying\n",
			 __func__, pfn_to_page(pfn));
		/* try again with a bit different memory target */
		start = pageno + mask + 1;
	}

	mutex_unlock(&cma_mutex);
	pr_debug("%s(): returned %p\n", __func__, page);
	return page;
}

struct page *dma_alloc_from_contiguous(struct device *dev, int count,
				       unsigned int align)
{
	unsigned long mask, pfn, pageno, start = 0;
	struct cma *cma = dev_get_cma_area(dev);
	struct page *page = NULL;
	int ret;

	if (!cma || !cma->count)
		return NULL;

	if (align > CONFIG_CMA_ALIGNMENT)
		align = CONFIG_CMA_ALIGNMENT;

	pr_debug("%s(cma %p, count %d, align %d)\n", __func__, (void *)cma,
		 count, align);

	if (!count)
		return NULL;

	mask = (1 << align) - 1;

	mutex_lock(&cma_mutex);

	for (;;) {
		pageno = bitmap_find_next_zero_area(cma->bitmap, cma->count,
						    start, count, mask);
		if (pageno >= cma->count) { pr_alert("%s %d: %lu %lu\n", __func__, __LINE__, mask, pageno);
			break; }

		pfn = cma->base_pfn + pageno;

		if (count == cma->count)
			adjust_forbid_cma_alloc_flag(1);

// SVP 16
//		ret = alloc_contig_range(pfn, pfn + count, MIGRATE_CMA, 0);
		ret = alloc_contig_range(pfn, pfn + count, MIGRATE_MOVABLE, 0);

		if (ret == 0) {
			bitmap_set(cma->bitmap, pageno, count);
			page = pfn_to_page(pfn);

			if (count == cma->count)
				adjust_forbid_cma_alloc_flag(0);

			break;
		} else if (ret != -EBUSY) {

			if (count == cma->count)
				adjust_forbid_cma_alloc_flag(0);

			pr_alert("%s %d: %d\n", __func__, __LINE__, ret);
			break;
		}

		if (count == cma->count)
			adjust_forbid_cma_alloc_flag(0);

		pr_debug("%s(): memory range at %p is busy, retrying\n",
			 __func__, pfn_to_page(pfn));
		/* try again with a bit different memory target */
		start = pageno + mask + 1;
	}

	mutex_unlock(&cma_mutex);
	pr_debug("%s(): returned %p\n", __func__, page);
	return page;
}

/**
 * dma_release_from_contiguous() - release allocated pages
 * @dev:   Pointer to device for which the pages were allocated.
 * @pages: Allocated pages.
 * @count: Number of allocated pages.
 *
 * This function releases memory allocated by dma_alloc_from_contiguous().
 * It returns false when provided pages do not belong to contiguous area and
 * true otherwise.
 */
bool dma_release_from_contiguous(struct device *dev, struct page *pages,
				 int count)
{
	struct cma *cma = dev_get_cma_area(dev);
	unsigned long pfn;

	if (!cma || !pages)
		return false;

	pr_debug("%s(page %p)\n", __func__, (void *)pages);

	pfn = page_to_pfn(pages);

	if (pfn < cma->base_pfn || pfn >= cma->base_pfn + cma->count)
		return false;

	VM_BUG_ON(pfn + count > cma->base_pfn + cma->count);

	mutex_lock(&cma_mutex);
	bitmap_clear(cma->bitmap, pfn - cma->base_pfn, count);
	free_contig_range(pfn, count);

	mutex_unlock(&cma_mutex);

	return true;
}


/* 
 * kernel module helper for SVP CMA 
 * 
 * Licensed under GPLv2 or later. 
 */  
  
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/uaccess.h>
#include <linux/sh_svp.h>

#define SVP_STATE_NULL		0x00
#define SVP_STATE_ONING_WAIT	0x01
#define SVP_STATE_ONING		0x02
#define SVP_STATE_ON		0x03
#define SVP_STATE_OFFING	0x04
#define SVP_STATE_OFF		0x05

#define _SVP_MBSIZE_ CONFIG_MTK_SVP_RAM_SIZE

static int ires;
static struct device *cma_dev;
static char _svp_state;
static struct page *_page;
static int _svp_onlinewait_tries;
static struct task_struct *_svp_online_task = NULL;

struct cma *svp_contiguous_default_area;

static int __svp_region_online(void)
{
	int retval = 0;
	bool retb;

	//_dev_info(cma_dev, "%s %d: svp to online enter state: %d\n", __func__, __LINE__, _svp_state);
	pr_alert("%s %d: svp to online enter state: %d\n", __func__, __LINE__, _svp_state);

	if (_svp_state == SVP_STATE_ONING_WAIT) {
		_svp_state = SVP_STATE_ONING;

		retb = dma_release_from_contiguous(cma_dev, _page, svp_contiguous_default_area->count);

		if (retb == true) {
			_page = 0;
			_svp_state = SVP_STATE_ON;
		}
	} else {
		retval = -EBUSY;
	}

	//_dev_info(cma_dev, "%s %d: svp to online leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);
	pr_alert("%s %d: svp to online leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);

	return retval;
}

static int _svp_online_kthread_func(void *data)
{
	int secusage = 0;
	int secdisable = 0;

	pr_alert("%s %d: start\n", __func__, __LINE__);

	_svp_onlinewait_tries = 0;

	while (_svp_onlinewait_tries < 10) {
		msleep(100);
		secusage = 0;

		// call secmem_query

		_svp_onlinewait_tries++;
		pr_alert("%s %d: _svp_onlinewait_tries: %d, secusage: %d\n", __func__, __LINE__, _svp_onlinewait_tries, secusage);

		// for no TEE test
		if (_svp_onlinewait_tries < 1) {
			secusage = 10;
		}

		if (!secusage) {
			break;
		}
	}

	if (_svp_onlinewait_tries >= 10) {
		// to do, show error
		pr_alert("%s %d: _svp_onlinewait_tries: %d, secusage: %d\n", __func__, __LINE__, _svp_onlinewait_tries, secusage);

		_svp_state = SVP_STATE_OFF;
		goto out;
	}

	// call secmem_disable

	pr_alert("%s %d: _svp_onlinewait_tries: %d, secusage: %d, secdisable %d\n", __func__, __LINE__, _svp_onlinewait_tries, secusage, secdisable);

	if (!secdisable) {
		__svp_region_online();

		_svp_state = SVP_STATE_ON;
	} else { // to do, error handle

		_svp_state = SVP_STATE_OFF;
	}

out:
	_svp_online_task = NULL;

	pr_alert("%s %d: end, _svp_onlinewait_tries: %d\n", __func__, __LINE__, _svp_onlinewait_tries);

	return 0;
}

static int __svp_migrate_range(unsigned long start, unsigned long end)
{
	int ret = 0;

	mutex_lock(&cma_mutex);

// SVP 16
//	ret = alloc_contig_range(start, end, MIGRATE_CMA, 0);
	ret = alloc_contig_range(start, end, MIGRATE_MOVABLE, 0);

	if (ret == 0) {
		free_contig_range(start, end - start);
	} else {
		// error handle
		pr_alert("%s %d: alloc failed: [%lu %lu) at [%lu - %lu)\n", __func__, __LINE__, start, end, get_svp_cma_basepfn(), get_svp_cma_basepfn() + get_svp_cma_count());
	}

	mutex_unlock(&cma_mutex);

	return ret;
}

/**
 * svp_contiguous_reserve() - reserve area(s) for contiguous memory handling
 * @limit: End address of the reserved memory (optional, 0 for any).
 *
 * This function reserves memory from early allocator. It should be
 * called by arch specific code once the early allocator (memblock or bootmem)
 * has been activated and all other subsystems have already allocated/reserved
 * memory.
 */
void __init svp_contiguous_reserve(phys_addr_t limit)
{
	phys_addr_t selected_size = _SVP_MBSIZE_ * 1024 * 1024;

	pr_alert("%s(limit %08lx)\n", __func__, (unsigned long)limit);

	if (selected_size && !svp_contiguous_default_area) {
		pr_debug("%s: reserving %ld MiB for svp area\n", __func__,
			 (unsigned long)selected_size / SZ_1M);

		ires = dma_contiguous_reserve_area(selected_size, 0, limit,
					    &svp_contiguous_default_area);
	}
};

int svp_migrate_range(unsigned long pfn)
{
	int ret = 0;

	if (!svp_is_in_range(pfn)) {
		return 1;
	}

	ret = __svp_migrate_range(pfn, pfn + 1);

	if (ret == 0) {
		// success

	} else {
		// error handle
		pr_alert("%s %d: migrate failed: %lu [%lu - %lu)\n", __func__, __LINE__, pfn, get_svp_cma_basepfn(), get_svp_cma_basepfn() + get_svp_cma_count());
	}

	return ret;
}

unsigned long get_svp_cma_basepfn(void)
{
	if (svp_contiguous_default_area) {
		return svp_contiguous_default_area->base_pfn;
	}

	return 0;
}

unsigned long get_svp_cma_count(void)
{
	if (svp_contiguous_default_area) {
		return svp_contiguous_default_area->count;
	}

	return 0;
}

int svp_is_in_range(unsigned long pfn)
{
	if (svp_contiguous_default_area) {
		if (pfn >= svp_contiguous_default_area->base_pfn &&
		    pfn < svp_contiguous_default_area->end_pfn)
			return 1;
	}

	return 0;
}

int svp_region_offline(void)
{
	struct page *page;
	int retval = 0;

	//_dev_info(cma_dev, "%s %d: svp to offline enter state: %d\n", __func__, __LINE__, _svp_state);
	pr_alert("%s %d: svp to offline enter state: %d\n", __func__, __LINE__, _svp_state);

	if (_svp_state == SVP_STATE_ON) {
		_svp_state = SVP_STATE_OFFING;

		page = dma_alloc_from_contiguous(cma_dev, svp_contiguous_default_area->count, 0);

		if (page) {
			_page = page;
			_svp_state = SVP_STATE_OFF;

			// call secmem_enable
		} else {
			_svp_state = SVP_STATE_ON;
			retval = -EAGAIN;
		}
	} else {
		retval = -EBUSY;
	}

	//_dev_info(cma_dev, "%s %d: svp to offline leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);
	pr_alert("%s %d: svp to offline leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);

	return retval;
}

int svp_region_online(void)
{
	int retval = 0;

	//_dev_info(cma_dev, "%s %d: svp to online enter state: %d\n", __func__, __LINE__, _svp_state);
	pr_alert("%s %d: svp to online enter state: %d\n", __func__, __LINE__, _svp_state);

	if (_svp_state == SVP_STATE_OFF) {
		_svp_state = SVP_STATE_ONING_WAIT;

		_svp_online_task = kthread_create(_svp_online_kthread_func, NULL, "svp_online_kthread");
		wake_up_process(_svp_online_task);
	} else {
		retval = -EBUSY;
	}

	//_dev_info(cma_dev, "%s %d: svp to online leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);
	pr_alert("%s %d: svp to online leave state: %d, retval: %d\n", __func__, __LINE__, _svp_state, retval);

	return retval;
}

/* any read request will free coherent memory, eg.
 * cat /dev/svp_region
 */
static ssize_t
svp_cma_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	//_dev_info(cma_dev, "svp cma state: %d\n", _svp_state);
	pr_alert("%s %d:svp state: %d ires: %d\n", __func__, __LINE__, _svp_state, ires);

	if (svp_contiguous_default_area)
	{
		//_dev_info(cma_dev, "svp cma base_pfn: %ld, count %ld\n", svp_contiguous_default_area->base_pfn, svp_contiguous_default_area->count);
		pr_alert("%s %d: svp cma base_pfn: %ld, count %ld\n", __func__, __LINE__, svp_contiguous_default_area->base_pfn, svp_contiguous_default_area->count);
	}
	if (dma_contiguous_default_area)
	{
		//_dev_info(cma_dev, "dma cma base_pfn: %ld, count %ld\n", dma_contiguous_default_area->base_pfn, dma_contiguous_default_area->count);
		pr_alert("%s %d: dma cma base_pfn: %ld, count %ld\n", __func__, __LINE__, dma_contiguous_default_area->base_pfn, dma_contiguous_default_area->count);
	}

	return 0;
}  
  
/*
 * any write request will alloc coherent memory, eg.
 * echo 0 > /dev/cma_test
 */
static ssize_t
svp_cma_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	unsigned char val;
	int retval = count;

	pr_alert("%s %d: count: %zu\n", __func__, __LINE__, count);

	if (count >= 2) {
		if (copy_from_user(&val, &buf[0], 1)) {
			retval = -EFAULT;
			goto out;
		}

		if (count == 2 && val == '0') {
			svp_region_offline();
		}
		else
		{
			svp_region_online();
		}
	}

out:
	return retval;
}  
  
static const struct file_operations svp_cma_fops = {  
    .owner =    THIS_MODULE,  
    .read  =    svp_cma_read,
    .write =    svp_cma_write,  
};  
  
static struct miscdevice svp_cma_misc = {  
    .name = "svp_region",
    .fops = &svp_cma_fops,
};  
  
static int __init svp_cma_init(void)
{  
	int ret = 0;

	ret = misc_register(&svp_cma_misc);
	if (unlikely(ret)) {
		pr_err("failed to register svp cma misc device!\n");
		return ret;
	}
	cma_dev = svp_cma_misc.this_device;
	cma_dev->coherent_dma_mask = ~0;

	_svp_state = SVP_STATE_ON;
	dev_set_cma_area(cma_dev, svp_contiguous_default_area);

	_dev_info(cma_dev, "registered.\n");  

	return ret;  
}  
module_init(svp_cma_init);
 
static void __exit svp_cma_exit(void)
{  
	misc_deregister(&svp_cma_misc);
}  
module_exit(svp_cma_exit);

// SVP
unsigned long cma_total_pages(unsigned long node_start_pfn,
				unsigned long node_end_pfn)
{
	int i;
	unsigned long total_pages = 0;

	for (i = 0; i < cma_area_count; i++) {
		struct cma *cma = &cma_areas[i];

		if (node_start_pfn <= cma->base_pfn &&
			cma->base_pfn < node_end_pfn)
			total_pages += cma->count;
	}

	return total_pages;
}

#endif
