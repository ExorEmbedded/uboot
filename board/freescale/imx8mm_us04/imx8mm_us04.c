/*
 * Copyright 2018 NXP
 * Copyright 2019 Exor International S.p.a.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#if defined(CONFIG_TARGET_IMX8MN_NS05)
#include <asm/arch/imx8mn_pins.h>
#else
#include <asm/arch/imx8mm_pins.h>
#endif
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <usb.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

#define US04JSMART_VAL    139
#define US04ETOPXX_VAL    145
#define US04WU10_VAL      147
#define US04EX705M_VAL    152
#define US04EXW705M_VAL   153
#define NS04ECO2XX_VAL    154

#if defined(CONFIG_TARGET_IMX8MM_US04)
/* Specific code for the US04 target */
#warning "Building for target: US04"
static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_SAI2_RXC_UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_SAI2_RXFS_UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};


#define US04_RST_OUT_GPIO IMX_GPIO_NR(4, 8)
#define US04_DXEN0_GPIO   IMX_GPIO_NR(1, 1)
#define US04_RXEN0_GPIO   IMX_GPIO_NR(1, 3)
#define US04_SDCD_GPIO    IMX_GPIO_NR(2, 12)
#define US04_RST_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const us04_rst_pads[] = {
    IMX8MM_PAD_SAI1_RXD6_GPIO4_IO8 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
    IMX8MM_PAD_GPIO1_IO01_GPIO1_IO1 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
    IMX8MM_PAD_GPIO1_IO03_GPIO1_IO3 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
};

#ifdef CONFIG_CMD_I2CHWCFG
void ena_rs232phy(void)
{
  gpio_request(US04_DXEN0_GPIO, "us04_dxen0_out");
  gpio_direction_output(US04_DXEN0_GPIO, 1);
  udelay(1000);
}
#else
void ena_rs232phy(void){}
#endif
#elif defined(CONFIG_TARGET_IMX8MM_NS04)
/* Specific code for the NS04 target */
#warning "Building for target: NS04"
static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART3_RXD_UART3_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART3_TXD_UART3_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#define US04_RST_OUT_GPIO IMX_GPIO_NR(4, 25)
#define US04_SDCD_GPIO    IMX_GPIO_NR(2, 12)
#define US04_RST_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const us04_rst_pads[] = {
    IMX8MM_PAD_SAI2_TXC_GPIO4_IO25 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
};

void ena_rs232phy(void){}
#elif defined(CONFIG_TARGET_IMX8MN_NS05)
/* Specific code for the NS05 imx8mn target */
#warning "Building for target: NS05 imx8mn"
static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MN_PAD_UART3_RXD__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MN_PAD_UART3_TXD__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MN_PAD_GPIO1_IO02__WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

#define US04_RST_OUT_GPIO IMX_GPIO_NR(4, 25)
#define US04_SDCD_GPIO    IMX_GPIO_NR(2, 12)
#define NS05_HUB_RST_GPIO IMX_GPIO_NR(1, 14)
#define NS05_USB_ID_GPIO  IMX_GPIO_NR(1, 10)
#define US04_RST_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const us04_rst_pads[] = {
    IMX8MN_PAD_SAI2_TXC__GPIO4_IO25 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
    IMX8MN_PAD_GPIO1_IO14__GPIO1_IO14 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
    IMX8MN_PAD_GPIO1_IO10__GPIO1_IO10 | MUX_PAD_CTRL(US04_RST_GPIO_PAD_CTRL),
};

void ena_rs232phy(void){}
#else
#error "Unknown CONFIG_TARGET_xxx: exiting"
#endif /* CONFIG_TARGET_xxx*/
/*
 * Read I2C SEEPROM infos and set env. variables accordingly
 */
static int read_eeprom(void)
{
#if (defined(CONFIG_CMD_I2CHWCFG))  
  extern int i2cgethwcfg (void);
  return i2cgethwcfg();
#endif
  return 0;
}

/*
 * Reads the hwcfg.txt file from USB stick (root of FATFS partition) if any, parses it
 * and updates the environment variable accordingly.
 * 
 * NOTE: This function is used in case the I2C SEEPROM contents are not valid, in order to get
 *       a temporary and volatile HW configuration from USB to boot properly Linux (even if the I2C SEEPROM is not programmed) 
 */
static int USBgethwcfg(void)
{
#if (defined(CONFIG_CMD_I2CHWCFG))  
  printf("Trying to get the HW cfg from USB stick...\n");
  
  run_command("usb stop", 0);
  run_command("usb reset", 0);
  run_command("setenv filesize 0", 0);
  run_command("fatload usb 0 ${loadaddr} hwcfg.txt", 0);
  run_command("env import -t ${loadaddr} ${filesize}", 0);
  run_command("usb stop", 0);
#endif  
  return 0;
}

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));
	set_wdog_reset(wdog);
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	
	/* Set the US04 RESET_OUT signal to high */
	imx_iomux_v3_setup_multiple_pads(us04_rst_pads, ARRAY_SIZE(us04_rst_pads));
	gpio_request(US04_RST_OUT_GPIO, "us04_reset_out");
	gpio_direction_output(US04_RST_OUT_GPIO, 1);

	return 0;
}

#ifdef CONFIG_BOARD_POSTCLK_INIT
int board_postclk_init(void)
{
	/* TODO */
	return 0;
}
#endif

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	if (rom_pointer[1])
		gd->ram_size = PHYS_SDRAM_SIZE - rom_pointer[1];
	else
		gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, bd_t *bd)
{
	return 0;
}
#endif


#ifdef CONFIG_CI_UDC
int board_usb_init(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, true);

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	imx8m_usb_power(index, false);

	return 0;
}
#endif

int board_init(void)
{
	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return CONFIG_SYS_MMC_ENV_DEV;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

int board_late_init(void)
{
#if (defined(CONFIG_CMD_I2CHWCFG))  
    char* tmp;
    unsigned long hwcode = 0;
#if defined(CONFIG_TARGET_IMX8MM_US04)
    gpio_request(US04_RXEN0_GPIO, "us04_rxen0_out");
    gpio_direction_output(US04_RXEN0_GPIO, 1);
#endif	
#if defined(CONFIG_TARGET_IMX8MN_NS05)
    gpio_request(NS05_HUB_RST_GPIO, "ns05_hub_rst_gpio");
    gpio_direction_output(NS05_HUB_RST_GPIO, 1);
    gpio_request(NS05_USB_ID_GPIO, "ns05_usb_id_gpio");
    gpio_direction_output(NS05_USB_ID_GPIO, 0);
#endif	
#endif
	
#if (defined(CONFIG_CMD_I2CHWCFG))  
    /* Get the system configuration from the I2C SEEPROM */
    if(read_eeprom())
    {
        ena_rs232phy();
        printf("Failed to read the HW cfg from the I2C SEEPROM: trying to load it from USB ...\n");
        USBgethwcfg();
    }

    /* Set the "board_name" env. variable according with the "hw_code" */
    tmp = env_get("hw_code");
    if(!tmp)
    {
        puts ("WARNING: 'hw_code' environment var not found!\n");
    }
    else
        hwcode = (simple_strtoul (tmp, NULL, 10))&0xff;

    /* For eTOP7xx panels... 
	 * 1: pass the mac addresses for the additional PCIe eth ports via cmdline 
	 * 2: unbind the UART4 core from the M4 CPU domain */
    if((hwcode==US04ETOPXX_VAL))
    {
        if(env_get("eth1addr"))
            if(env_get("eth2addr"))
                run_command("setenv optargs pcie_tse1addr=${eth1addr} pcie_tse2addr=${eth2addr}", 0);
			
		run_command("mw.l 0x303d0518 0xff", 0);
    }
#if defined(CONFIG_TARGET_IMX8MM_US04)
	else //On US04 targets, other than eTOP7xx, restore cpu default pinmux settings for the RGMII_RXD3 pin
		run_command("mw.l 0x30330304 0x116", 0);
#endif	

    if((hwcode==US04EX705M_VAL) || (hwcode==US04EXW705M_VAL))
    {
        if(env_get("eth1addr"))
			run_command("setenv optargs pcie_tse1addr=${eth1addr}", 0);
			
		run_command("mw.l 0x303d0518 0xff", 0);
    }

    if(hwcode==US04JSMART_VAL)
    {
        env_set("board_name", "us04_jsmart");
    }
    else if(hwcode==US04ETOPXX_VAL)
    {
        env_set("board_name", "us04_etopxx"); 
    }
    else if(hwcode==US04WU10_VAL)
    {
        env_set("board_name", "us04_wu10"); 
    }
    else if((hwcode==US04EX705M_VAL) || (hwcode==US04EXW705M_VAL))
	{
		env_set("board_name", "us04_ex705m");
	}
    else if(hwcode==NS04ECO2XX_VAL)
    {
        env_set("board_name", "ns04_eco2xx"); 
    }
    else
    {
        ena_rs232phy();
        puts ("WARNING: unknowm carrier hw code; using 'usom_undefined' board name. \n");
        env_set("board_name", "usom_undefined");
    }
    /* Check if file $0030d8$.bin exists on the 1st partition of the SD-card and, if so, skips booting the mainOS */
    run_command("setenv skipbsp1 0", 0);
	
	/* Check the SD_CD status; the SD-card will be accessed during boot sequence only if SD-card detected */
	gpio_request(US04_SDCD_GPIO , "us04_sdcd_gpio");
	if(gpio_get_value(US04_SDCD_GPIO))
	{ /* SD-card not present */
		run_command("setenv sd_detected 0", 0);
	}
	else
	{
		run_command("setenv sd_detected 1", 0);
		run_command("mmc dev 0", 0);
		run_command("mmc rescan", 0);
		run_command("if test -e mmc 0:1 /$0030d8$.bin; then setenv skipbsp1 1; fi", 0);
	}
#endif    
	return 0;
}
