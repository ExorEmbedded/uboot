/*
 * am335x_usom_wu16.h
 *
 * Copyright (C) 2016 Exor S.p.a.
 *
 * Based on:
 * u-boot:/include/configs/am335x_evm.h
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CONFIG_AM335X_USOM_H
#define __CONFIG_AM335X_USOM_H

#include <configs/ti_am335x_common.h>
/* Super silent u-boot */
#define CONFIG_SILENT_CONSOLE               1
#define CONFIG_SILENT_U_BOOT_ONLY           1

#define MACH_TYPE_TIAM335EVM		3589	/* Until the next sync */
#define CONFIG_MACH_TYPE		MACH_TYPE_TIAM335EVM
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_SYS_EXOR_USOM
#undef  CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 	0
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_PARTITION	1

/* memory test */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START 0x80000000
#define CONFIG_SYS_MEMTEST_END   0xA0000000

/* Clock Defines */
#define V_OSCK				24000000  /* Clock output from T2 */
#define V_SCLK				(V_OSCK)

/* Custom script for NOR */
#define CONFIG_SYS_LDSCRIPT		"board/exor/am335x_usom_wu16/u-boot.lds"

/* Always 128 KiB env size */
#define CONFIG_ENV_SIZE			(128 << 10)
/* Enhance our eMMC support / experience. */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION
#define CONFIG_PARTITION_UUIDS
#define CONFIG_CMD_PART

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

#ifndef CONFIG_SPL_BUILD
#define CONFIG_EXTRA_ENV_SETTINGS \
	"altbootcmd="CONFIG_SYS_ALT_BOOTCOMMAND"\0"\
	"bootlimit=3\0" \
	"loadaddr=0x80200000\0" \
	"fdtaddr=0x80F80000\0" \
	"fdt_high=0xffffffff\0" \
	"boot_fdt=yes\0" \
	"skipbsp1=0\0" \
	"rdaddr=0x81000000\0" \
	"bootpart=0:1\0" \
	"bootdir=/boot\0" \
	"bootfile=zImage\0" \
	"fdtfile=usom_undefined.dtb\0" \
	"fastboot=n\0" \
	"silent=1\0" \
	"console=/dev/null\0" \
	"rs232_txen=0\0" \
	"optargs=\0" \
	"mmcdev=0\0" \
	"mmcroot=/dev/mmcblk0p2 rw\0" \
	"mmcrootfstype=ext4 rootwait\0" \
	"rootpath=/export/rootfs\0" \
	"nfsopts=nolock\0" \
	"ramroot=/dev/ram0 rw ramdisk_size=65536 initrd=${rdaddr},64M\0" \
	"ramrootfstype=ext2\0" \
	"mmcargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"hw_dispid=${hw_dispid} " \
		"hw_code=${hw_code} " \
		"fastboot=${fastboot} " \
		"board_name=${board_name} " \
		"touch_type=${touch_type} " \
		"ethaddr=${ethaddr} " \
		"root=${mmcroot} " \
		"rootfstype=${mmcrootfstype}\0" \
	"netargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"hw_dispid=${hw_dispid} " \
		"hw_code=${hw_code} " \
		"board_name=${board_name} " \
		"touch_type=${touch_type} " \
		"ethaddr=${ethaddr} " \
		"root=/dev/nfs " \
		"nfsroot=${serverip}:${rootpath},${nfsopts} rw " \
		"ip=dhcp\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize\0" \
	"ramargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"hw_dispid=${hw_dispid} " \
		"hw_code=${hw_code} " \
		"board_name=${board_name} " \
		"touch_type=${touch_type} " \
		"ethaddr=${ethaddr} " \
		"root=${ramroot} " \
		"rootfstype=${ramrootfstype}\0" \
	"loadramdisk=load mmc ${mmcdev} ${rdaddr} ramdisk.gz\0" \
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
		"ethaddr=${ethaddr} " \
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
	"netboot=echo Booting from network ...; " \
		"setenv autoload no; " \
		"dhcp; " \
		"tftp ${loadaddr} ${bootfile}; " \
		"tftp ${fdtaddr} ${fdtfile}; " \
		"run netargs; " \
		"bootz ${loadaddr} - ${fdtaddr}\0" \
	"ramboot=echo Booting from ramdisk ...; " \
		"run ramargs; " \
		"bootz ${loadaddr} ${rdaddr} ${fdtaddr}\0" \
	"findfdt="\
		"if test $board_name = usom_etop5xx; then " \
			"setenv fdtfile usom_etop5xx.dtb; fi; " \
		"if test $board_name = usom_eco; then " \
			"setenv fdtfile usom_eco.dtb; fi; " \
		"if test $board_name = usom_plcm07; then " \
			"setenv fdtfile usom_plcm07.dtb; fi; " \
		"if test $board_name = usom_wu16; then " \
			"setenv fdtfile usom_wu16.dtb; fi; " \
		"if test $board_name = usom_undefined; then " \
			"setenv fdtfile usom_undefined.dtb; fi; \0" 
#endif

#undef  CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY 0
#define CONFIG_ZERO_BOOTDELAY_CHECK

#define CONFIG_BOOTCOMMAND_FAST \
       "run findfdt; " \
       "echo Try booting Linux from EMMC, main BSP...;" \
       "setenv mmcdev 1; " \
       "setenv bootpart 1:3; " \
       "setenv mmcroot /dev/mmcblk1p3 ro; " \
       "run mmcboot;" \
       "echo Try booting Linux from USB stick...;" \
       "run usbboot;" \
       "echo Try booting Linux from EMMC, recovery BSP...;" \
       "setenv mmcdev 1; " \
       "setenv bootpart 1:2; " \
       "setenv mmcroot /dev/mmcblk1p2 ro; " \
       "run mmcboot;" 

#define CONFIG_BOOTCOMMAND \
	"run findfdt; " \
	"echo Try booting Linux from USB stick...;" \
	"run usbboot;" \
	"setenv mmcdev 0; " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"if test $skipbsp1 = 0; then " \
	"echo Try booting Linux from EMMC, main BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:3; " \
	"setenv mmcroot /dev/mmcblk1p3 ro; " \
	"run mmcboot;" \
	"fi; " \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 

#define CONFIG_SYS_ALT_BOOTCOMMAND \
	"i2c mw 68 19 0; " \
	"setenv mmcdev 0; " \
	"run findfdt; " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"echo Try booting Linux from USB stick...;" \
	"run usbboot;" \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 

#define CONFIG_ANDROID_BOOTCOMMAND \
	"setenv mmcdev 0; " \
	"run findfdt; " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"if test $skipbsp1 = 0; then " \
	"echo Try booting Android from EMMC, main BSP...;" \
	"setenv bootargs console=ttyO0,115200n8 androidboot.console=ttyO0 rootwait ro; " \
	"mmc dev 1; " \
	"mmc rescan; " \
	"ext4load mmc 1:8 82000000 uImage ; " \
	"ext4load mmc 1:8 83000000 uRamdisk; " \
	"bootm 82000000 83000000; " \
	"fi; " \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 
	
/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1		0x44e09000	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		0x48022000	/* UART1 */
#define CONFIG_BAUDRATE			115200

#define CONFIG_HAVEPRGUART	/* We have a programmable Uart physical */

#define CONFIG_CMD_EEPROM
#define CONFIG_ENV_EEPROM_IS_ON_I2C
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x54	/* Main EEPROM */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
#define CONFIG_SYS_I2C_MULTI_EEPROMS

/* PMIC support */
#define CONFIG_POWER_TPS65217
#define CONFIG_POWER_TPS65910

/* SPL */
#define CONFIG_SPL_POWER_SUPPORT
#define CONFIG_SPL_YMODEM_SUPPORT

/* Bootcount using the M41T83 I2C RTC NVRAM */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_I2C
#define CONFIG_BOOTCOUNT_ALEN 1
#define CONFIG_SYS_I2C_RTC_ADDR 0x68
#undef  CONFIG_SYS_BOOTCOUNT_ADDR
#define CONFIG_SYS_BOOTCOUNT_ADDR 0x19
#define CONFIG_SYS_BOOTCOUNT_WU16 1

#define CONFIG_SPL_ENV_SUPPORT
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/am33xx/u-boot-spl.lds"


/*
 * USB configuration.  We enable MUSB support.
 * USB0 and USB1 are set for HOST mode.
 */
#define CONFIG_USB_MUSB_DSPS
#define CONFIG_ARCH_MISC_INIT
#define CONFIG_MUSB_PIO_ONLY
#define CONFIG_MUSB_DISABLE_BULK_COMBINE_SPLIT
#define CONFIG_MUSB_HOST
#define CONFIG_AM335X_USB0
#define CONFIG_AM335X_USB0_MODE	MUSB_HOST

#ifdef CONFIG_MUSB_HOST
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_USBETH_SUPPORT)
/* disable host part of MUSB in SPL */
#undef CONFIG_MUSB_HOST
/* disable EFI partitions and partition UUID support */
#undef CONFIG_PARTITION_UUIDS
#undef CONFIG_EFI_PARTITION
/*
 * Disable CPSW SPL support so we fit within the 101KiB limit.
 */
#undef CONFIG_SPL_ETH_SUPPORT
#endif

#if defined(CONFIG_EMMC_BOOT)
#undef CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		1
#define CONFIG_SYS_MMC_ENV_PART		2
#define CONFIG_ENV_OFFSET		0x0
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#endif

/* Network. */
#define CONFIG_PHY_GIGE
#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC

/* Exor custom features */
#define CONFIG_CMD_I2CHWCFG
#define CONFIG_SYS_I2C_ADPADD 0x56

#endif	/* ! __CONFIG_AM335X_EVM_H */
