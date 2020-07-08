// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 */

#include <common.h>
#include <dm.h>
#include <ram.h>
#include <asm/cache.h>

DECLARE_GLOBAL_DATA_PTR;

static void set_mmu_dcache(u32 addr, u32 size)
{
	int	i;
	u32 start, end;

	start = addr >> MMU_SECTION_SHIFT;
	end = ((u64)addr + (u64)size) >> MMU_SECTION_SHIFT;

	for (i = start; i < end; i++) {
#if defined(CONFIG_SYS_ARM_CACHE_WRITETHROUGH)
		set_section_dcache(i, DCACHE_WRITETHROUGH);
#elif defined(CONFIG_SYS_ARM_CACHE_WRITEALLOC)
		set_section_dcache(i, DCACHE_WRITEALLOC);
#else
		set_section_dcache(i, DCACHE_WRITEBACK);
#endif
	}
}

int dram_init(void)
{
	struct ram_info ram;
	struct udevice *dev;
	int ret;

	ret = uclass_get_device(UCLASS_RAM, 0, &dev);
	if (ret) {
		debug("RAM init failed: %d\n", ret);
		return ret;
	}
	ret = ram_get_info(dev, &ram);
	if (ret) {
		debug("Cannot get RAM size: %d\n", ret);
		return ret;
	}
	debug("RAM init base=%lx, size=%x\n", ram.base, ram.size);

	gd->ram_size = ram.size;

	return 0;
}

void dram_bank_mmu_setup(int bank)
{
	if (gd->flags & GD_FLG_RELOC) {
		debug("%s: bank: %d\n", __func__, bank);
		set_mmu_dcache(gd->bd->bi_dram[bank].start,
			       gd->bd->bi_dram[bank].size);
	} else {
#if defined(CONFIG_SPL_BUILD)
		set_mmu_dcache(STM32_SYSRAM_BASE, STM32_SYSRAM_SIZE);
#else
		set_mmu_dcache(STM32_DDR_BASE, STM32_DDR_SIZE);
#endif
	}
}
