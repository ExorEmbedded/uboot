/*
 * Global settings for the iMX6 USOM (US03) boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6USOMUS03_COMMON_CONFIG_H
#define __MX6USOMUS03_COMMON_CONFIG_H

#define CONFIG_MX6

#ifdef CONFIG_MX6SOLO
#define CONFIG_MX6DL
#endif

/* uncomment for PLUGIN mode support */
/* #define CONFIG_USE_PLUGIN */

/* uncomment for SECURE mode support */
/* #define CONFIG_SECURE_BOOT */

#include "mx6_common.h"
#include <linux/sizes.h>

#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

/* We do not want a dedicated boot sequence if booting from USB
 */
#undef is_boot_from_usb

#define CONFIG_CMD_I2CHWCFG
#define CONFIG_HAVEPRGUART
#define CONFIG_SYS_I2C_EEPROM_ADDR 0x54
#define CONFIG_SYS_I2C_ADPADD 0x56

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define CONFIG_CMD_FS_GENERIC

#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1

//Front RED LED
//#define CONFIG_RGB_RED_LED

//Reduce boot time 500ms
#undef CONFIG_PHYLIB
#undef CONFIG_PHY_ATHEROS


/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1
#define CONFIG_BAUDRATE                        115200

/* Command definition */
#include <config_cmd_default.h>

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY               1

#define CONFIG_LOADADDR                0x12000000
#define CONFIG_SYS_TEXT_BASE           0x17800000
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_MFG_NAND_PARTITION ""

#define CONFIG_EXTRA_ENV_SETTINGS \
	"altbootcmd="CONFIG_SYS_ALT_BOOTCOMMAND"\0"\
	"bootlimit=3\0" \
	"silent=1\0" \
	"loadaddr=0x12000000\0" \
	"fdtaddr=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"boot_fdt=yes\0" \
	"smp=" CONFIG_SYS_NOSMP "\0"\
	"skipbsp1=0\0" \
	"enable_gigabit=0\0" \
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
		"ethaddr=${ethaddr} " \
		"root=${mmcroot} " \
		"${smp} " \
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
		"${smp} " \
		"ip=dhcp\0" \
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc ${mmcdev} ${loadaddr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize\0" \
	"loadimage=load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}\0" \
	"loadfdt=load mmc ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile}; \0" \
	"checkgigabit="\
		"if test $enable_gigabit = 1; then " \
			"fdt addr ${fdtaddr} ; " \
				"if fdt rm eth0_7xx/mdio/ethernet-phy1  fsl,disable-gigabit ; then " \
					"echo Gigabit enabled ;" \
				"else " \
					"echo Gigabit not enabled ;" \
				"fi; " \
			"fi; \0 " \
	"mmcloados=run mmcargs; run findisolcpus; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"run checkgigabit; " \
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
		"${smp} " \
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
	"findisolcpus="\
		"if test \"${isolcpus}\" != \"\"; then " \
			"setenv bootargs ${bootargs} isolcpus=${isolcpus}; fi; \0" \
	"findfdt="\
		"if test $board_name = usom_etop7xx; then " \
			"setenv fdtfile usom_etop7xx.dtb; fi; " \
		"if test $board_name = usom_evkitq; then " \
			"setenv fdtfile usom_evkitq.dtb; fi; " \
		"if test $board_name = usom_etop7xxq; then " \
			"setenv fdtfile usom_etop7xxq.dtb; fi; " \
		"if test $board_name = usom_be15a; then " \
			"setenv fdtfile usom_be15a.dtb; fi; " \
		"if test $board_name = usom_wu16; then " \
			"setenv fdtfile usom_wu16.dtb; fi; " \
		"if test $board_name = usom_jsmart; then " \
			"setenv fdtfile usom_jsmart.dtb; fi; " \
		"if test $board_name = usom_jsmartq; then " \
			"setenv fdtfile usom_jsmartq.dtb; fi; " \
		"if test $board_name = usom_jsmartttl; then " \
			"setenv fdtfile usom_jsmartttl.dtb; fi; " \
		"if test $board_name = usom_ta19; then " \
			"setenv fdtfile usom_ta19.dtb; fi; " \
		"if test $board_name = usom_be15b; then " \
			"setenv fdtfile usom_be15b.dtb; fi; " \
		"if test $board_name = usom_ta19_dl; then " \
			"setenv fdtfile usom_ta19_dl.dtb; fi; " \
		"if test $board_name = usom_undefined; then " \
			"setenv fdtfile usom_undefined.dtb; fi; \0" 
	
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

#define CFG_BOOTCMD_FACTORY_MODE \
	"run findfdt; " \
	"if test $skipbsp1 = 0; then " \
	"echo Try booting Linux from EMMC, main BSP...;" \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:3; " \
	"setenv mmcroot /dev/mmcblk1p3 ro; " \
	"run mmcboot;" \
	"fi; " \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv fastboot n; " \
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
	"setenv fastboot n; " \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 

#define CFG_ALTBOOTCMD_FACTORY_MODE \
	"i2c mw 68 19 0; " \
	"run findfdt; " \
	"echo Try booting Linux from EMMC, recovery BSP...;" \
	"setenv fastboot n; " \
	"setenv mmcdev 1; " \
	"setenv bootpart 1:2; " \
	"setenv mmcroot /dev/mmcblk1p2 ro; " \
	"run mmcboot;" 
	
#define CONFIG_ANDROID_BOOTCOMMAND "echo Android boot not yet available;\0" 
	
#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE              1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS             256
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

#define CONFIG_SYS_LOAD_ADDR           CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE               (128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#ifndef CONFIG_SYS_NOSMP
#define CONFIG_SYS_NOSMP
#endif

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(0 * 1024)

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8

#define CONFIG_OF_LIBFDT
#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

/*
 * I2C configs
 */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

#ifdef CONFIG_MX6DL
#define CONFIG_IPUV3_CLK 198000000
#else
#define CONFIG_IPUV3_CLK 264000000
#endif

#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6sabreandroid_common.h"
#endif

/* Bootcounter using the M41T83 I2C RTC NVRAM */
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_I2C
#define CONFIG_BOOTCOUNT_ALEN 1
#define CONFIG_SYS_I2C_RTC_ADDR 0x68
#undef  CONFIG_SYS_BOOTCOUNT_ADDR
#define CONFIG_SYS_BOOTCOUNT_ADDR 0x19

#define CONFIG_DISABLE_CONSOLE
#endif                         /* __MX6USOMUS03_COMMON_CONFIG_H */
