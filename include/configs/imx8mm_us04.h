/*
 * Copyright 2018 NXP
 * Copyright 2019 Exor International S.p.a.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IMX8MM_US04_H
#define __IMX8MM_US04_H

#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>

#include "imx_env.h"

#ifdef CONFIG_SECURE_BOOT
#define CONFIG_CSF_SIZE                0x2000 /* 8K region */
#endif

#define CONFIG_SPL_MAX_SIZE            (148 * 1024)
#define CONFIG_SYS_MONITOR_LEN         (512 * 1024)
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_USE_SECTOR
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x300
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SYS_UBOOT_BASE          (QSPI0_AMBA_BASE + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)

#ifdef CONFIG_SPL_BUILD
/*#define CONFIG_ENABLE_DDR_TRAINING_DEBUG*/
#define CONFIG_SPL_WATCHDOG_SUPPORT
#define CONFIG_SPL_POWER_SUPPORT
#define CONFIG_SPL_DRIVERS_MISC_SUPPORT
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_LDSCRIPT		"arch/arm/cpu/armv8/u-boot-spl.lds"
#define CONFIG_SPL_STACK		0x91fff0
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_BSS_START_ADDR      0x00910000
#define CONFIG_SPL_BSS_MAX_SIZE        0x2000	/* 8 KB */
#define CONFIG_SYS_SPL_MALLOC_START    0x42200000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x80000	/* 512 KB */
#define CONFIG_SYS_ICACHE_OFF
#define CONFIG_SYS_DCACHE_OFF

#define CONFIG_MALLOC_F_ADDR           0x912000 /* malloc f used before GD_FLG_FULL_MALLOC_INIT set */

/* Differnt OCRAM layout for NS05/imx8mn due to different size */
#ifdef CONFIG_TARGET_IMX8MN_NS05
#undef CONFIG_SPL_STACK
#undef CONFIG_SPL_BSS_START_ADDR
#undef CONFIG_SYS_SPL_MALLOC_START
#undef CONFIG_SYS_SPL_MALLOC_SIZE
#undef CONFIG_MALLOC_F_ADDR
#define CONFIG_SPL_STACK		0x95fff0
#define CONFIG_SPL_BSS_START_ADDR      0x00950000
#define CONFIG_SYS_SPL_MALLOC_START    0x00940000
#define CONFIG_SYS_SPL_MALLOC_SIZE     0x10000	/* 64 KB */
#define CONFIG_MALLOC_F_ADDR		0x00940000
#endif

#define CONFIG_SPL_ABORT_ON_RAW_IMAGE /* For RAW image gives a error info not panic */

#undef CONFIG_DM_MMC
#undef CONFIG_DM_PMIC
#undef CONFIG_DM_PMIC_PFUZE100

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_BD71837

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC_I2C1		   /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		   /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		   /* enable I2C bus 3 */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define CONFIG_REMAKE_ELF

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_POSTCLK_INIT
#define CONFIG_BOARD_LATE_INIT

/* Flat Device Tree Definitions */
#define CONFIG_OF_BOARD_SETUP

#undef CONFIG_CMD_EXPORTENV
#undef CONFIG_CMD_IMPORTENV
#undef CONFIG_CMD_IMLS

#undef CONFIG_CMD_CRC32
#undef CONFIG_BOOTM_NETBSD


/* Initial environment variables */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"altbootcmd="CFG_SYS_ALT_BOOTCOMMAND"\0"\
	"bootlimit=3\0" \
	"mmcautodetect=yes\0" \
	"fdtaddr=0x43000000\0" \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_addr=0x43800000\0"		\
	"initrd_high=0xffffffffffffffff\0" \
	"boot_fdt=yes\0" \
	"skipbsp1=0\0" \
	"bootpart=0:1\0" \
	"bootdir=/boot\0" \
	"bootfile=Image\0" \
	"fdtfile=usom_undefined.dtb\0" \
	"fastboot=n\0" \
	"rs232_txen=0\0" \
	"optargs=\0" \
	"mmcdev=0\0" \
	"mmcroot=/dev/mmcblk0p2 rw\0" \
	"mmcrootfstype=ext4 rootwait\0" \
	"rootpath=/export/rootfs\0" \
	"mmcargs=setenv bootargs ${jh_clk} console=${console} " \
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
	"mmcloados=run mmcargs; run findisolcpus; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"booti ${loadaddr} - ${fdtaddr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"booti; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"booti; " \
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
	"usbargs=setenv bootargs ${jh_clk} console=${console} " \
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
				"booti ${loadaddr} - ${fdtaddr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"booti; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"booti; " \
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
	"findisolcpus="\
		"if test \"${isolcpus}\" != \"\"; then " \
			"setenv bootargs ${bootargs} isolcpus=${isolcpus}; fi; \0" \
	"findfdt="\
		"if test $board_name = us04_jsmart; then " \
			"setenv fdtfile us04_jsmart.dtb; fi; " \
		"if test $board_name = us04_etopxx; then " \
			"setenv fdtfile us04_etopxx.dtb; fi; " \
		"if test $board_name = us04_wu10; then " \
			"setenv fdtfile us04_wu10.dtb; fi; " \
		"if test $board_name = us04_ex705m; then " \
			"setenv fdtfile us04_ex705m.dtb; fi; " \
		"if test $board_name = ns04_eco2xx; then " \
			"setenv fdtfile ns04_eco2xx.dtb; fi; " \
		"if test $board_name = us04_x07; then " \
			"setenv fdtfile us04_x07.dtb; fi; " \
		"if test $board_name = ns04_dah21; then " \
			"setenv fdtfile ns04_dah21.dtb; fi; " \
		"if test $board_name = ns04_x07bs; then " \
			"setenv fdtfile ns04_x07bs.dtb; fi; " \
		"if test $board_name = usom_undefined; then " \
			"setenv fdtfile usom_undefined.dtb; fi; \0" 
			
#define CONFIG_BOOTCOMMAND \
	"setenv mmcdev 0; " \
	"setenv mmcroot /dev/mmcblk0p2 ro; " \
	"run findfdt; " \
	"if test $sd_detected = 1; then " \
	"echo Try booting Linux from SD-card...;" \
	"run mmcboot;" \
	"fi; " \
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

#define CFG_SYS_ALT_BOOTCOMMAND \
	"i2c mw 68 19 0; " \
	"setenv mmcdev 0; " \
	"setenv mmcroot /dev/mmcblk0p2 ro; " \
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
       
/* Link Definitions */
#define CONFIG_LOADADDR                0x40480000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_SYS_INIT_RAM_ADDR       0x40000000
#define CONFIG_SYS_INIT_RAM_SIZE       0x80000
#define CONFIG_SYS_INIT_SP_OFFSET \
        (CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
        (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_OFFSET              (0)
#define CONFIG_ENV_SIZE                0x2000
#define CONFIG_SYS_MMC_ENV_DEV         1 /* Environment in EMMC, BOOT2 partition */
#define CONFIG_SYS_MMC_ENV_PART        2 
#define CONFIG_MMCROOT                 "/dev/mmcblk1p2"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN          ((CONFIG_ENV_SIZE + (2*1024) + (16*1024)) * 1024)

#define CONFIG_SYS_SDRAM_BASE          0x40000000
#define PHYS_SDRAM                     0x40000000
#define PHYS_SDRAM_SIZE                0x80000000 /* 2GB DDR */
#define CONFIG_NR_DRAM_BANKS           1

#ifdef CONFIG_TARGET_IMX8MM_NS04
#undef PHYS_SDRAM_SIZE
#define PHYS_SDRAM_SIZE                0x40000000 /* 1GB DDR for NS04 */
#endif

#ifdef CONFIG_TARGET_IMX8MN_NS05
#undef PHYS_SDRAM_SIZE
#define PHYS_SDRAM_SIZE                0x40000000 /* 1GB DDR for NS05 */
#endif

#define CONFIG_SYS_MEMTEST_START       PHYS_SDRAM
#define CONFIG_SYS_MEMTEST_END         (CONFIG_SYS_MEMTEST_START + (PHYS_SDRAM_SIZE >> 1))

#define CONFIG_BAUDRATE                115200

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE           UART1_BASE_ADDR

/* Use UART3 for NS04 */
#ifdef CONFIG_TARGET_IMX8MM_NS04
#undef CONFIG_MXC_UART_BASE
#define CONFIG_MXC_UART_BASE           UART3_BASE_ADDR
#endif

/* Use UART3 for NS05 */
#ifdef CONFIG_TARGET_IMX8MN_NS05
#undef CONFIG_MXC_UART_BASE
#define CONFIG_MXC_UART_BASE           UART3_BASE_ADDR
#endif

/* Monitor Command Prompt */
#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT              "uS04=> "
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_SYS_CBSIZE              2048
#define CONFIG_SYS_MAXARGS             64
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE              (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_IMX_BOOTAUX

/* USDHC */
#define CONFIG_CMD_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC

#define CONFIG_SYS_FSL_USDHC_NUM       2
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_SUPPORT_EMMC_BOOT	   /* eMMC specific */
#define CONFIG_SYS_MMC_IMG_LOAD_PART   1

#define CONFIG_MXC_GPIO

#define CONFIG_MXC_OCOTP
#define CONFIG_CMD_FUSE

#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#define CONFIG_SYS_I2C_MXC_I2C1        /* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2        /* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3        /* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED           100000

/* USB configs */
#ifndef CONFIG_SPL_BUILD
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#define CONFIG_USBD_HS

#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE

/* Support for factory section in I2C SEEPROM */
#define CONFIG_CMD_I2CHWCFG
#define CONFIG_SYS_I2C_EEPROM_ADDR 0x54
#define DEF_SYS_I2C_ADPADD 0x56

#endif

#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW    2

#define CONFIG_CI_UDC

#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#define CONFIG_OF_SYSTEM_SETUP

/* Bootcounter using the M41T83 I2C RTC NVRAM */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_I2C
#define CONFIG_BOOTCOUNT_ALEN           1
#define CONFIG_SYS_I2C_RTC_ADDR         0x68
#undef  CONFIG_SYS_BOOTCOUNT_ADDR
#define CONFIG_SYS_BOOTCOUNT_ADDR       0x19

#endif
