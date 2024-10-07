/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/*
 * Copyright (C) 2018, STMicroelectronics - All Rights Reserved
 *
 * Configuration settings for the STM32MP15x CPU
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include <linux/sizes.h>
#include <asm/arch/stm32.h>

/*
 * Number of clock ticks in 1 sec
 */
#define CONFIG_SYS_HZ				1000

#ifndef CONFIG_STM32MP1_TRUSTED
/* PSCI support */
#define CONFIG_ARMV7_PSCI_1_0
#define CONFIG_ARMV7_SECURE_BASE		STM32_SYSRAM_BASE
#define CONFIG_ARMV7_SECURE_MAX_SIZE		STM32_SYSRAM_SIZE
#endif

/*
 * malloc() pool size
 */
#define CONFIG_SYS_MALLOC_LEN			SZ_32M

/*
 * Configuration of the external SRAM memory used by U-Boot
 */
#define CONFIG_SYS_SDRAM_BASE			STM32_DDR_BASE
#define CONFIG_SYS_INIT_SP_ADDR			CONFIG_SYS_TEXT_BASE

#define CONFIG_DISABLE_CONSOLE

/*
 * Console I/O buffer size
 */
#define CONFIG_SYS_CBSIZE			SZ_1K

/*
 * Needed by "loadb"
 */
#define CONFIG_SYS_LOAD_ADDR			STM32_DDR_BASE

/*
 * Env parameters
 */
#ifdef CONFIG_NSXX_TARGET
#define CONFIG_ENV_SIZE				SZ_8K
#else
#define CONFIG_ENV_SIZE				SZ_4K
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET              (0)
#define CONFIG_SYS_MMC_ENV_DEV         1 /* Environment in EMMC, BOOT2 partition */
#define CONFIG_SYS_MMC_ENV_PART        2 
#endif

/* ATAGs */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Extend size of kernel image for uncompression */
#define CONFIG_SYS_BOOTM_LEN			SZ_32M

/* SPL support */
#ifdef CONFIG_SPL
/* BOOTROM load address */
#define CONFIG_SPL_TEXT_BASE		0x2FFC2500
/* SPL use DDR */
#define CONFIG_SPL_BSS_START_ADDR	0xC0200000
#define CONFIG_SPL_BSS_MAX_SIZE		0x00100000
#define CONFIG_SYS_SPL_MALLOC_START	0xC0300000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x00100000

/* limit SYSRAM usage to first 128 KB */
#define CONFIG_SPL_MAX_SIZE		0x00020000
#define CONFIG_SPL_STACK		0x2FFFC000
#endif /* #ifdef CONFIG_SPL */

#define CONFIG_SYS_MEMTEST_START	STM32_DDR_BASE
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + SZ_64M)
#define CONFIG_SYS_MEMTEST_SCRATCH	(CONFIG_SYS_MEMTEST_END + 4)

/*MMC SD*/
#define CONFIG_SYS_MMC_MAX_DEVICE	3
#define CONFIG_SUPPORT_EMMC_BOOT

/*****************************************************************************/
#ifdef CONFIG_DISTRO_DEFAULTS
/*****************************************************************************/

/* NAND support */
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_MAX_NAND_DEVICE	1

/* SPI nand */
#define CONFIG_SYS_MAX_NAND_DEVICE	1

/* SPI FLASH support */
#if defined(CONFIG_SPL_BUILD)
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x80000
#endif

/* FILE SYSTEM */

#if defined(CONFIG_STM32_QSPI) || defined(CONFIG_NAND_STM32_FMC2)
/* Dynamic MTD partition support */
#define CONFIG_SYS_MTDPARTS_RUNTIME
#endif

/* Ethernet need */
#ifdef CONFIG_DWC_ETH_QOS
#define CONFIG_SYS_NONCACHED_MEMORY	(1 * SZ_1M)	/* 1M */
#define CONFIG_SERVERIP			192.168.1.1
#define CONFIG_BOOTP_SERVERIP
#define CONFIG_SYS_AUTOLOAD		"no"
#endif

#ifdef CONFIG_DM_VIDEO
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_BMP_16BPP
#define CONFIG_BMP_24BPP
#define CONFIG_BMP_32BPP
#endif

#if !defined(CONFIG_SPL_BUILD)

/* default order is eMMC (SDMMC 1)/ NAND / SDCARD (SDMMC 0) / SDMMC2 */
#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 1) \
	func(UBIFS, ubifs, 0) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 2) \
	func(PXE, pxe, na)

/*
 * bootcmd for stm32mp1:
 * for serial/usb: execute the stm32prog command
 * for mmc boot (eMMC, SD card), boot only on the same device
 * for nand boot, boot with on ubifs partition on nand
 * for nor boot, use the default order
 */
#define CONFIG_PREBOOT
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND "echo \"Boot over ${boot_device}${boot_instance}!\";run bootcmd_stm32mp"

#define STM32MP_BOOTCMD "bootcmd_stm32mp=" \
	"if test ${boot_device} = serial || test ${boot_device} = usb; then " \
		"stm32prog ${boot_device} ${boot_instance}; " \
	"else " \
		"if test ${boot_device} = mmc; then env set boot_targets \"mmc${boot_instance}\"; fi;" \
		"if test ${boot_device} = nand; then env set boot_targets ubifs0; fi;" \
		"run distro_bootcmd;" \
	"fi;\0"

#include <config_distro_bootcmd.h>

#ifdef CONFIG_STM32MP1_OPTEE
#define CONFIG_SYS_MEM_TOP_HIDE			SZ_32M
/* with OPTEE: define specific MTD partitions = teeh, teed, teex */
#define STM32MP_MTDPARTS \
	"mtdparts_nor0=256k(fsbl1),256k(fsbl2),2m(ssbl),256k(logo),256k(teeh),256k(teed),256k(teex),-(nor_user)\0" \
	"mtdparts_nand0=2m(fsbl),2m(ssbl1),2m(ssbl2),512k(teeh),512k(teed),512k(teex),-(UBI);\0"

#else /* CONFIG_STM32MP1_OPTEE */

#define STM32MP_MTDPARTS \
	"mtdparts_nor0=256k(fsbl1),256k(fsbl2),2m(ssbl),256k(logo),-(nor_user)\0" \
	"mtdparts_nand0=2m(fsbl),2m(ssbl1),2m(ssbl2),-(UBI)\0"

#endif /* CONFIG_STM32MP1_OPTEE */

/*
 * memory layout for 32M uncompressed/compressed kernel,
 * 1M fdt, 1M script, 1M pxe and 1M for splashimage
 * and the ramdisk at the end.
 */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"stdin=serial\0" \
	"stdout=serial\0" \
	"stderr=serial\0" \
	"bootdelay=1\0" \
	"kernel_addr_r=0xc2000000\0" \
	"fdt_addr_r=0xc4000000\0" \
	"scriptaddr=0xc4100000\0" \
	"pxefile_addr_r=0xc4200000\0" \
	"splashimage=0xc4300000\0"  \
	"ramdisk_addr_r=0xc4400000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"bootlimit=0\0" \
	"altbootcmd=run bootcmd\0" \
	"usb_pgood_delay=2000\0" \
	STM32MP_BOOTCMD \
	STM32MP_MTDPARTS \
	BOOTENV \
	"boot_net_usb_start=true\0"

#endif /* ifndef CONFIG_SPL_BUILD */
#endif /* ifdef CONFIG_DISTRO_DEFAULTS*/

/*****************************************************************************/
/*****************************************************************************/
/* NSXX specific configuration setup                                         */
/*****************************************************************************/
/*****************************************************************************/
#ifdef CONFIG_NSXX_TARGET

/*
 * Bootcounter support
 */
#define CONFIG_SYS_I2C_RTC_ADDR 0x68

/* Support for factory section in I2C SEEPROM */
#define CONFIG_SYS_I2C_SPEED 100000
#define CONFIG_CMD_I2CHWCFG
#define CONFIG_SYS_I2C_EEPROM_ADDR 0x54
#define DEF_SYS_I2C_ADPADD 0x56


#define CONFIG_EXTRA_ENV_SETTINGS \
	"altbootcmd="CFG_SYS_ALT_BOOTCOMMAND"\0"\
	"fastbootcmd="CFG_SYS_FAST_BOOTCOMMAND"\0"\
	"silent=1\0" \
	"bootlimit=3\0" \
	"fdtaddr=0xc4000000\0" \
	"loadaddr=0xc2000000\0" \
	"fdt_high=0xffffffff\0" \
	"boot_fdt=yes\0" \
	"skipbsp1=0\0" \
	"bootpart=0:1\0" \
	"bootdir=/boot\0" \
	"bootfile=zImage\0" \
	"fdtfile=usom_undefined.dtb\0" \
	"fastboot=n\0" \
	"console=/dev/null\0" \
	"rs232_txen=0\0" \
	"optargs=\0" \
	"mmcdev=0\0" \
	"mmcroot=/dev/mmcblk0p2 rw\0" \
	"mmcrootfstype=ext4 rootwait\0" \
	"rootpath=/export/rootfs\0" \
	"nfsopts=nolock\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"hw_dispid=${hw_dispid} " \
		"hw_code=${hw_code} " \
		"fastboot=${fastboot} " \
		"board_name=${board_name} " \
		"touch_type=${touch_type} " \
		"root=${mmcroot} " \
		"rootfstype=${mmcrootfstype}\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
	"env import -t $loadaddr $filesize\0" \
	"loadimage=load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}\0" \
	"loadfdt=load mmc ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile}\0" \
	"mmcloados=run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdtaddr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"mmcboot=mmc dev ${mmcdev}; " \
		"if mmc rescan; then " \
			"echo SD/MMC found on device ${mmcdev};" \
			"if run loadbootenv; then " \
				"echo Loaded environment from ${bootenv};" \
				"run importbootenv;" \
			"fi;" \
			"if test -n $uenvcmd; then " \
				"echo Running uenvcmd ...;" \
				"run uenvcmd;" \
			"fi;" \
			"if run loadimage; then " \
				"run mmcloados;" \
			"fi;" \
		"fi;\0" \
	"usbargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"hw_dispid=${hw_dispid} " \
		"hw_code=${hw_code} " \
		"board_name=${board_name} " \
		"touch_type=${touch_type} " \
		"root=${usbroot} " \
		"rootfstype=${usbrootfstype}\0" \
	"usbroot=/dev/sda2 rw\0" \
	"usbrootfstype=ext4 rootwait\0" \
	"usbloados=run usbargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run usbloadfdt; then " \
				"bootz ${loadaddr} - ${fdtaddr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"usbloadimage=load usb 0 ${loadaddr} ${bootdir}/${bootfile}\0" \
	"usbloadfdt=load usb 0 ${fdtaddr} ${bootdir}/${fdtfile}\0" \
	"usbboot=mmc dev ${mmcdev}; " \
		"if usb reset; then " \
			"if run usbloadimage; then " \
				"run usbloados;" \
			"fi;" \
			"usb stop;" \
		"fi;\0" \
	"findfdt="\
		"if test $board_name = ns02_ek435; then " \
			"setenv fdtfile ns02_ek435.dtb; fi; " \
		"if test $board_name = ns02_wu20; then " \
			"setenv fdtfile ns02_wu20.dtb; fi; " \
		"if test $board_name = ns02_wu07; then " \
			"setenv fdtfile ns02_wu07.dtb; fi; " \
		"if test $board_name = usom_undefined; then " \
			"setenv fdtfile usom_undefined.dtb; fi; \0" 

#ifdef CONFIG_BOOTCOMMAND
#undef CONFIG_BOOTCOMMAND
#endif
			
#define CONFIG_BOOTCOMMAND \
	"setenv mmcdev 0; " \
	"run findfdt; " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"if test $skipbsp1 = 0; then " \
	"echo Try booting Linux from EMMC, main BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:3; " \
	"setenv mmcroot /dev/mmcblk1p3 ro; " \
	"run mmcboot;" \
	"fi; " \
	"echo Try booting Linux from USB stick...;" \
	"run usbboot;" \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv fastboot n; " \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 

#define CFG_SYS_FAST_BOOTCOMMAND \
	"setenv verify n; " \
	"setenv mmcdev 1; " \
	"setenv mmcroot /dev/mmcblk1p3 ro; " \
	"run mmcargs; " \
	"load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}; " \
	"load mmc ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile}; " \
	"bootz ${loadaddr} - ${fdtaddr}; "

#define CFG_SYS_ALT_BOOTCOMMAND \
	"bootcount reset; " \
	"setenv mmcdev 0; " \
	"run findfdt; " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"echo Try booting Linux from USB stick...;" \
	"run usbboot;" \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv fastboot n; " \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 

#endif /* #ifdef CONFIG_NSXX_TARGET */
#endif /* __CONFIG_H */
