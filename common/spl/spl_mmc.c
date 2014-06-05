/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <mmc.h>
#include <version.h>
#include <image.h>

DECLARE_GLOBAL_DATA_PTR;

static int mmc_load_image_raw(struct mmc *mmc, unsigned long sector)
{
	unsigned long err;
	u32 image_size_sectors;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0, sector, 1, header);
	if (err == 0)
		goto end;

	if (image_get_magic(header) != IH_MAGIC)
		return -1;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + mmc->read_bl_len - 1) /
				mmc->read_bl_len;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0, sector, image_size_sectors,
					(void *)spl_image.load_addr);

end:
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
	if (err == 0)
		printf("spl: mmc blk read err - %lu\n", err);
#endif

	return (err == 0);
}

#ifdef CONFIG_SPL_OS_BOOT
static int mmc_load_image_raw_os(struct mmc *mmc)
{
	if (!mmc->block_dev.block_read(0,
				       CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR,
				       CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS,
				       (void *)CONFIG_SYS_SPL_ARGS_ADDR)) {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
		printf("mmc args blk read error\n");
#endif
		return -1;
	}

	return mmc_load_image_raw(mmc, CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR);
}
#endif

#ifndef CONFIG_SYS_EXOR_USOM
void spl_mmc_load_image(void)
{
	struct mmc *mmc;
	int err;
	u32 boot_mode;

	mmc_initialize(gd->bd);
	/* We register only one device. So, the dev id is always 0 */
	mmc = find_mmc_device(0);
	if (!mmc) {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
		puts("spl: mmc device not found!!\n");
#endif
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
		printf("spl: mmc init failed: err - %d\n", err);
#endif
		hang();
	}

	boot_mode = spl_boot_mode();
	if (boot_mode == MMCSD_MODE_RAW) {
		debug("boot mode - RAW\n");
#ifdef CONFIG_SPL_OS_BOOT
		if (spl_start_uboot() || mmc_load_image_raw_os(mmc))
#endif
		err = mmc_load_image_raw(mmc,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR);
#ifdef CONFIG_SPL_FAT_SUPPORT
	} else if (boot_mode == MMCSD_MODE_FAT) {
		debug("boot mode - FAT\n");
#ifdef CONFIG_SPL_OS_BOOT
		if (spl_start_uboot() || spl_load_image_fat_os(&mmc->block_dev,
								CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION))
#endif
		err = spl_load_image_fat(&mmc->block_dev,
					CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION,
					CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME);
#endif
#ifdef CONFIG_SUPPORT_EMMC_BOOT
	} else if (boot_mode == MMCSD_MODE_EMMCBOOT) {
		/*
		 * We need to check what the partition is configured to.
		 * 1 and 2 match up to boot0 / boot1 and 7 is user data
		 * which is the first physical partition (0).
		 */
		int part = (mmc->part_config >> 3) & PART_ACCESS_MASK;

		if (part == 7)
			part = 0;

		if (mmc_switch_part(0, part)) {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
			puts("MMC partition switch failed\n");
#endif
			hang();
		}
#ifdef CONFIG_SPL_OS_BOOT
		if (spl_start_uboot() || mmc_load_image_raw_os(mmc))
#endif
		err = mmc_load_image_raw(mmc,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR);
#endif
	} else {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
		puts("spl: wrong MMC boot mode\n");
#endif
		hang();
	}

	if (err)
		hang();
}
#else
/* For Exor microsoms we use a dedicated sequence for loading u-boot, in order to have a recovery solution in case the system (SPL) booted
 * from EMMC but the u-boot is not valid/present on the EMMC itself.
 * For this reason, loading u-boot from the FAT formatted partition of a removable mmc card takes the precedence; if not found, we will proceed
 * loading the u-boot in raw mode from the BOOT1 partition of the EMMC
 */

extern int spl_mmc_initialize(void);
extern int spl_emmc_initialize(void);

void spl_mmc_load_image(void)
{
  struct mmc *mmc;
  int err;
  int emmc_dev = 0;
  u32 boot_mode;
  
  /* 1: Try loading u-boot from the FAT partition of a removable SD-card */
  err = spl_mmc_initialize();
  if(!err)
  {
    puts("spl: Trying loading u-boot from FAT partition of SD-Card ...\n");
    emmc_dev ++;
  }
  
  if(!err)
  {
    /* In SPL mode we always see 1 only mmc device */
    mmc = find_mmc_device(0);
    if (!mmc) 
    {
      puts("spl: SD-card device not found!!\n");
      err = 1;
    }
  }
  
  if(!err)
  {
    err = mmc_init(mmc);
    if (err) 
    {
      printf("spl: SD-card init failed: err - %d\n", err);
    }
  }
  
  if(!err)
  {
    err = spl_load_image_fat(&mmc->block_dev, CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION, CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME); 
    if(!err)
      return;
    
    printf("spl: File: %s not found on SD-card ... loading RAW u-boot image from EMMC.\n",CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME);
  }
  
  /* 2: Try loading RAW u-boot image from EMMC BOOT1 partition */
  err = spl_emmc_initialize();
  if(!err)
  {
    puts("spl: Proceed loading RAW u-boot image from EMMC ...\n");
  }
  else
  {
    printf("spl: EMMC init failed: err: %d\n", err);
    hang();
  }
  /* In SPL mode we always see 1 only mmc device */
  mmc = find_mmc_device(0);
  if (!mmc) 
  {
    puts("spl: EMMC device not found!!\n");
    hang();
  }
  
  err = mmc_init(mmc);
  if (err) 
  {
    printf("spl: EMMC init failed: err - %d\n", err);
    hang();
  }
  
  printf("spl: Switching to EMMC partition n. %d ...\n", CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_PARTITION);
  if (mmc_switch_part(0, CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_PARTITION)) 
  {
    puts("spl: EMMC partition switch failed\n");
    hang();
  }
  
  puts("spl: Loading raw u-boot image...\n");
  err = mmc_load_image_raw(mmc, CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR);
  
  if (err)
  {
    puts("spl: EMMC error\n");
    hang();
  }
}
#endif