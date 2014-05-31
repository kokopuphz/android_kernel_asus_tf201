/*
 * arch/arm/mach-tegra/eprj_memory.c
 *
 * Low level CPU cache management functions for nVidia Tegra SoC
 * for use with special external drivers (Android, practically).
 *
 * Copyright (C) 2013, EternityProject Development.
 * Angelo G. Del Regno <kholk11@gmail.com>
 *
 */

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
//#include <linux/memory_alloc.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <mach/eternityproject.h>

/* These cache related routines make the assumption (if outer cache is
 * available) that the associated physical memory is contiguous.
 * They will operate on all (L1 and L2 if present) caches.
 */
void clean_and_invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	dmac_flush_range((void *)vstart, (void *) (vstart + length));
	//v7_dma_flush_range((void *)vstart, (void *) (vstart + length));
	outer_flush_range(pstart, pstart + length);
}

void clean_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	dmac_clean_range((void *)vstart, (void *) (vstart + length));
	outer_clean_range(pstart, pstart + length);
}

void invalidate_caches(unsigned long vstart,
	unsigned long length, unsigned long pstart)
{
	dmac_inv_range((void *)vstart, (void *) (vstart + length));
	outer_inv_range(pstart, pstart + length);
}
