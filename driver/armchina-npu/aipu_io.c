// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2025 Arm Technology (China) Co. Ltd. */

#include <linux/ioport.h>
#include "aipu_io.h"

/**
 * @init_aipu_ioregion() - initialize struct io_region using physical base address
 * @region:    pointer to struct io_region to be initialized
 * @phys_base: base address of this region
 * @size:      size of this region
 *
 * Return: 0 on success and error code otherwise.
 */
int init_aipu_ioregion(struct io_region *region, u64 phys_base, u32 size)
{
	int ret = 0;

	if (!region || !size)
		return -EINVAL;

	if (!request_mem_region(phys_base, size, "aipu"))
		return -ENOMEM;

	region->kern = ioremap(phys_base, size);
	if (!region->kern) {
		release_mem_region(phys_base, size);
		return -ENOMEM;
	}

	region->phys = phys_base;
	region->size = size;
	return ret;
}

/**
 * @deinit_aipu_ioregion() - destroy an AIPU IO region
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 */
void deinit_aipu_ioregion(struct io_region *region)
{
	if (region && region->kern) {
		iounmap(region->kern);
		release_mem_region(region->phys, region->size);
		region->kern = NULL;
		region->phys = 0;
		region->size = 0;
	}
}

/**
 * @aipu_read32() - read AIPU register in word
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 * @offset: AIPU register offset
 * @val:    pointer receiving the value read back, written on success only
 *
 * Return: 0 on success and error code otherwise. The value travels through
 * @val rather than the return so a register reading 0xFFFFFFEA is
 * distinguishable from -EINVAL; the bound requires the whole word to fit
 * (offset < size accepted reads up to 3 bytes past the mapping).
 */
int aipu_read32(struct io_region *region, int offset, u32 *val)
{
	if (!region || !region->kern || !val)
		return -EINVAL;

	if (offset < 0 || (u64)offset + 4 > region->size)
		return -EINVAL;

	*val = readl((u8 __iomem *)region->kern + offset);
	pr_debug("[Read AIPU Register]: reg addr: 0x%llx + 0x%x, read back value: 0x%x\n",
		 region->phys, offset, *val);
	return 0;
}

/**
 * @aipu_write32() - write AIPU register in word
 * @region: pointer to struct io_region initialized in init_aipu_ioregion()
 * @offset: AIPU register offset
 * @data:   u32 data to be writen
 *
 * Return: 0 on success and error code otherwise. A rejected write is no
 * longer logged as if it happened (upstream 4.3.0).
 */
int aipu_write32(struct io_region *region, int offset, unsigned int data)
{
	if (!region || !region->kern)
		return -EINVAL;

	if (offset < 0 || (u64)offset + 4 > region->size)
		return -EINVAL;

	writel((u32)data, (u8 __iomem *)region->kern + offset);
	pr_debug("[Write AIPU Register]: reg addr: 0x%llx + 0x%x, write value: 0x%x\n",
		 region->phys, offset, (u32)data);
	return 0;
}
