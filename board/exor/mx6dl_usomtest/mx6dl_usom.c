/*
 * Board functions for US03 imx6 test system
 * Copyright (C) 2016 Exor S.p.a.
 * 
 * Author: Giovanni Pavoni Exor S.p.a.
 *
 * Based on:
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/crm_regs.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mxc_hdmi.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>

#include <i2c.h>

#ifdef CONFIG_CMD_SATA
#include <asm/imx-common/sata.h>
#endif
#ifdef CONFIG_FASTBOOT
#include <fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define I2C_EXP_RST IMX_GPIO_NR(1, 15)
#define I2C3_STEER  IMX_GPIO_NR(5, 4)
#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

/*Need more drive strength for SD1 slot on base board*/
#define USDHC1_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |               \
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)
	
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define WEIM_NOR_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)


#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define BE15A_VAL     114
#define ETOP7XX_VAL   115
#define ETOP7XXQ_VAL  118
#define RXEN0_GPIO IMX_GPIO_NR(6, 31)
#define DXEN0_GPIO IMX_GPIO_NR(3, 14)
#define MODE0_GPIO IMX_GPIO_NR(3, 15)

#define WDOG1WRSRREG (0x20bc004)
#define SNVSLPCRREG  (0x20cc038)

#define SPI1_CLK  (IMX_GPIO_NR(3, 16))
#define SPI1_SOMI (IMX_GPIO_NR(3, 17))
#define SPI1_SIMO (IMX_GPIO_NR(3, 18))
#define SPI1_CSFR (IMX_GPIO_NR(4, 12))
#define SPI1_CSAD (IMX_GPIO_NR(4, 11))

/*
 * Perform FRAM Test via SPI bitbang.
 * Test result is given by setting the framok env. variable: framok=ok.
 */
static void FRAM_test(void)
{
  unsigned char in = 0x9f;
  unsigned char out=0;
  int i;

  gpio_direction_output(SPI1_CLK, 0);
  gpio_direction_output(SPI1_SIMO, 0);
  gpio_direction_output(SPI1_CSFR, 1);
  gpio_direction_input(SPI1_SOMI);
  mdelay(20);
  
  //1) Send WREN command (0x06)
  in = 0x06;
  run_command("gpio clear 108", 0);
  mdelay(1);

   //Write cmd
  for(i=0; i< 8; i++)
  {
    if(in & 0x80)
      gpio_set_value(SPI1_SIMO, 1);
    else
     gpio_set_value(SPI1_SIMO, 0);
    
    mdelay(1);
    gpio_set_value(SPI1_CLK, 1);
    mdelay(1);
    gpio_set_value(SPI1_CLK, 0);
    in = in << 1;
  }

  mdelay(1);
  run_command("gpio set 108", 0);
  mdelay(10);
  
  //RDSR command (read status register)
  in = 0x05;
  run_command("gpio clear 108", 0);
  mdelay(1);
  
  //Write data
  for(i=0; i< 8; i++)
  {
    if(in & 0x80)
      gpio_set_value(SPI1_SIMO, 1);
    else
     gpio_set_value(SPI1_SIMO, 0);
    
    mdelay(1);
    gpio_set_value(SPI1_CLK, 1);
    mdelay(1);
    gpio_set_value(SPI1_CLK, 0);
    in = in << 1;
  }
  
  //Read data
  for(i=0; i< 8; i++)
  {
    out = out << 1;
    mdelay(1);
    gpio_set_value(SPI1_CLK, 1);
    if(gpio_get_value(SPI1_SOMI))
      out |= 0x01;
    mdelay(1);
    gpio_set_value(SPI1_CLK, 0);
  } 
  
  mdelay(1);
  run_command("gpio set 108", 0);
  mdelay(1);
  
  //Set the framok env. var occording with the test result
  if((out != 0xff) && (out & 0x02))
    run_command("setenv framok ok", 0);
  else
    run_command("setenv framok", 0);
}


/*
 * Perform MCP3204 Test via SPI bitbang (just check the chip presence).
 * Test result is given by setting the mcp3204ok env. variable: mcp3204ok=ok.
 */
static void mcp3204_test(void)
{
  unsigned char in;
  unsigned short out=0;
  int i;

  gpio_direction_output(SPI1_CLK, 0);
  gpio_direction_output(SPI1_SIMO, 0);
  gpio_direction_output(SPI1_CSFR, 1);
  gpio_direction_output(SPI1_CSAD, 1);
  gpio_direction_input(SPI1_SOMI);
  mdelay(20);
  
  //1) Send Read command on channel 0
  in = 0x06;
  run_command("gpio clear 107", 0);
  mdelay(1);

   //Write cmd
  for(i=0; i< 8; i++)
  {
    if(in & 0x80)
      gpio_set_value(SPI1_SIMO, 1);
    else
     gpio_set_value(SPI1_SIMO, 0);
    
    mdelay(1);
    gpio_set_value(SPI1_CLK, 1);
    mdelay(1);
    gpio_set_value(SPI1_CLK, 0);
    in = in << 1;
  }

  //Read data
  for(i=0; i< 16; i++)
  {
    out = out << 1;
    mdelay(1);
    gpio_set_value(SPI1_CLK, 1);
    if(gpio_get_value(SPI1_SOMI))
      out |= 0x01;
    mdelay(1);
    gpio_set_value(SPI1_CLK, 0);
  } 
  
  mdelay(1);
  run_command("gpio set 107", 0);
  mdelay(1);
  
  printf("mcp3204 out=%d\n",out);
  
  //Set the mcp3204ok env. var occording with the test result
  out = out & 0xfff;
  if((out != 0) && (out != 0xfff))
    run_command("setenv mcp3204ok ok", 0);
  else
    run_command("setenv mcp3204ok", 0);
}


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
  
  printf("Trying to get the HW cfg from USB stick...\n");
  
  run_command("usb stop", 0);
  run_command("usb reset", 0);
  run_command("setenv filesize 0", 0);
  run_command("fatload usb 0 ${loadaddr} hwcfg.txt", 0);
  run_command("env import -t ${loadaddr} ${filesize}", 0);
  run_command("usb stop", 0);
  
  return 0;
}

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

iomux_v3_cfg_t const uart4_pads[] = {
	MX6_PAD_KEY_COL0__UART4_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_KEY_ROW0__UART4_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_KEY_COL1__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL2__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

/*
 * I2C bus mapped to I2C3 port
 */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO_3__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO_3__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO_6__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO_6__GPIO1_IO06 | PC,
		.gp = IMX_GPIO_NR(1, 6)
	}
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
}


/* SD-card pads on SD2 port */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC1_PAD_CTRL),
	MX6_PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL), /* CD */
};

/* EMMC pads on SD4 port */
iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/* GPIO pads */
iomux_v3_cfg_t const gpio_pads_iomux[] = {
  MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), //T15
  MX6_PAD_GPIO_18__GPIO7_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL), //T18
  MX6_PAD_SD3_DAT4__GPIO7_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL), //T26
  MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), //T27
  MX6_PAD_SD3_DAT5__GPIO7_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), //T28
  MX6_PAD_DI0_PIN15__GPIO4_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), //T29
  MX6_PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL), //T30
  MX6_PAD_DI0_PIN3__GPIO4_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), //T31
  MX6_PAD_DI0_DISP_CLK__GPIO4_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), //T32
  MX6_PAD_DISP0_DAT16__GPIO5_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL), //T34
  MX6_PAD_DISP0_DAT17__GPIO5_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), //T35
  MX6_PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), //T36
  MX6_PAD_DISP0_DAT19__GPIO5_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL), //T37
  MX6_PAD_DISP0_DAT20__GPIO5_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), //T39
  MX6_PAD_DISP0_DAT21__GPIO5_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL), //T40
  MX6_PAD_DISP0_DAT22__GPIO5_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), //T41
  MX6_PAD_DISP0_DAT23__GPIO5_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), //T42
  MX6_PAD_DISP0_DAT0__GPIO4_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), //T44
  MX6_PAD_DISP0_DAT1__GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL), //T45
  MX6_PAD_DISP0_DAT2__GPIO4_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), //T46
  MX6_PAD_DISP0_DAT3__GPIO4_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), //T47
  MX6_PAD_DISP0_DAT4__GPIO4_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), //T49
  MX6_PAD_DISP0_DAT5__GPIO4_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), //T50
  MX6_PAD_DISP0_DAT6__GPIO4_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), //T51
  MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), //T52
  MX6_PAD_DISP0_DAT8__GPIO4_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), //T54
  MX6_PAD_DISP0_DAT9__GPIO4_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL), //T55
  MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), //T56
  MX6_PAD_DISP0_DAT11__GPIO5_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), //T57
  MX6_PAD_DISP0_DAT12__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), //T59
  MX6_PAD_DISP0_DAT13__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL), //T60
  MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), //T61
  MX6_PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), //T62
 
  MX6_PAD_EIM_D20__GPIO3_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL), //R4
  MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), //R5
  MX6_PAD_EIM_D27__GPIO3_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), //R7
  MX6_PAD_EIM_D29__GPIO3_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), //R8
  MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), //R9
  MX6_PAD_EIM_D26__GPIO3_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), //R10
  MX6_PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), //R12
  MX6_PAD_EIM_D31__GPIO3_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), //R13
  MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), //R14
  MX6_PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), //R15
  MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), //R16
  MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL), //R17
  MX6_PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL), //R18
  MX6_PAD_KEY_COL4__GPIO4_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), //R19
  MX6_PAD_ENET_RXD0__GPIO1_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), //R41
  MX6_PAD_ENET_RXD1__GPIO1_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), //R42
  MX6_PAD_CSI0_DAT10__GPIO5_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), //R43
  MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), //R44
  MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), //R46
 
  MX6_PAD_EIM_DA10__GPIO3_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL), //B3
  MX6_PAD_EIM_DA13__GPIO3_IO13 | MUX_PAD_CTRL(NO_PAD_CTRL), //B4
  MX6_PAD_EIM_DA12__GPIO3_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), //B5
  MX6_PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), //B6
  MX6_PAD_EIM_DA4__GPIO3_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), //B8
  MX6_PAD_EIM_DA8__GPIO3_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), //B9
  MX6_PAD_EIM_DA3__GPIO3_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL), //B10
  MX6_PAD_EIM_EB1__GPIO2_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), //B11
  MX6_PAD_EIM_DA6__GPIO3_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), //B12
  MX6_PAD_SD3_CLK__GPIO7_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL), //B13
  MX6_PAD_SD3_CMD__GPIO7_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), //B14
  MX6_PAD_SD3_DAT0__GPIO7_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), //B15
  MX6_PAD_SD3_DAT1__GPIO7_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), //B17
  MX6_PAD_SD3_DAT2__GPIO7_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), //B18
  MX6_PAD_SD3_DAT3__GPIO7_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL), //B19
  MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), //B20
  MX6_PAD_EIM_DA14__GPIO3_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL), //B21
  MX6_PAD_EIM_DA15__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL), //B22
  MX6_PAD_EIM_DA1__GPIO3_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL), //B23
  MX6_PAD_EIM_DA2__GPIO3_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), //B24
  MX6_PAD_DI0_PIN4__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL), //B25
  MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), //B26
  MX6_PAD_CSI0_VSYNC__GPIO5_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), //B27
  MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL), //B28
  MX6_PAD_CSI0_MCLK__GPIO5_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL), //B29
  MX6_PAD_CSI0_DAT12__GPIO5_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL), //B31
  MX6_PAD_CSI0_DAT13__GPIO5_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), //B32
  MX6_PAD_CSI0_DAT14__GPIO6_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), //B33
  MX6_PAD_CSI0_DAT15__GPIO6_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL), //B34
  MX6_PAD_CSI0_DAT16__GPIO6_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), //B36
  MX6_PAD_CSI0_DAT17__GPIO6_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL), //B37
  MX6_PAD_CSI0_DAT18__GPIO6_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), //B38
  MX6_PAD_CSI0_DAT19__GPIO6_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), //B39
  MX6_PAD_CSI0_PIXCLK__GPIO5_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL), //B40
  MX6_PAD_EIM_DA0__GPIO3_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), //B42
  MX6_PAD_EIM_CS1__GPIO2_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), //B43
  MX6_PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), //B44
  MX6_PAD_EIM_LBA__GPIO2_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), //B45
  MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), //B46
  MX6_PAD_EIM_CS0__GPIO2_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), //B47
  MX6_PAD_KEY_COL0__GPIO4_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), //B49
  MX6_PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), //B50
  MX6_PAD_KEY_COL1__GPIO4_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), //B51
  MX6_PAD_KEY_ROW0__GPIO4_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL), //B52
  MX6_PAD_NANDF_D0__GPIO2_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), //B54
  MX6_PAD_ENET_TX_EN__GPIO1_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), //B55
  MX6_PAD_ENET_RX_ER__GPIO1_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), //B56
  MX6_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), //B58
  MX6_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), //B59
  MX6_PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL), //B60
  MX6_PAD_ENET_TXD0__GPIO1_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL), //B61
  MX6_PAD_ENET_CRS_DV__GPIO1_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), //B63
 
  MX6_PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL), //L15
  MX6_PAD_NANDF_D5__GPIO2_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), //L16
  MX6_PAD_EIM_D16__GPIO3_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), //L18
  MX6_PAD_CSI0_DAT7__GPIO5_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), //L19
  MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL), //L20
  MX6_PAD_EIM_D17__GPIO3_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), //L21
  MX6_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL), //L22
  MX6_PAD_GPIO_0__GPIO1_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL), //L23
  MX6_PAD_EIM_DA9__GPIO3_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), //L35
  
  MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL), //SPI FRAM CS
  MX6_PAD_KEY_ROW2__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), //SPI AD MCP3204 CS
};

#ifdef CONFIG_SYS_I2C_MXC
/* set all switches APS in normal and PFM mode in standby */
static int setup_pmic_mode(int chip)
{
	unsigned char offset, i, switch_num, value;

	if (!chip) {
		/* pfuze100 */
		switch_num = 6;
		offset = 0x31;
	} else {
		/* pfuze200 */
		switch_num = 4;
		offset = 0x38;
	}

	value = 0xc;
	if (i2c_write(0x8, 0x23, 1, &value, 1)) {
		printf("Set SW1AB mode error!\n");
		return -1;
	}

	for (i = 0; i < switch_num - 1; i++) {
		if (i2c_write(0x8, offset + i * 7, 1, &value, 1)) {
			printf("Set switch%x mode error!\n", offset);
			return -1;
		}
	}

	return 0;
}

static int setup_pmic_voltages(void)
{
	unsigned char value, rev_id = 0 ;
	i2c_set_bus_num(2);
	if (!i2c_probe(0x8)) {
		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("Found PFUZE100! deviceid=%x,revid=%x\n", value, rev_id);

		if (setup_pmic_mode(value & 0xf)) {
			printf("setup pmic mode error!\n");
			return -1;
		}
		/* set SW1AB staby volatage 0.975V*/
		if (i2c_read(0x8, 0x21, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x21, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x24, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x24, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}

		/* set SW1C staby volatage 0.975V*/
		if (i2c_read(0x8, 0x2f, 1, &value, 1)) {
			printf("Read SW1CSTBY error!\n");
			return -1;
		}
		value &= ~0x3f;
		value |= 0x1b;
		if (i2c_write(0x8, 0x2f, 1, &value, 1)) {
			printf("Set SW1CSTBY error!\n");
			return -1;
		}

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		if (i2c_read(0x8, 0x32, 1, &value, 1)) {
			printf("Read SW1ABSTBY error!\n");
			return -1;
		}
		value &= ~0xc0;
		value |= 0x40;
		if (i2c_write(0x8, 0x32, 1, &value, 1)) {
			printf("Set SW1ABSTBY error!\n");
			return -1;
		}
	}

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned char value;
	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		/* increase VDDARM to 1.425V */
		if (i2c_read(0x8, 0x20, 1, &value, 1)) {
			printf("Read SW1AB error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x20, 1, &value, 1)) {
			printf("Set SW1AB error!\n");
			return;
		}
		/* increase VDDSOC to 1.425V */
		if (i2c_read(0x8, 0x2e, 1, &value, 1)) {
			printf("Read SW1C error!\n");
			return;
		}
		value &= ~0x3f;
		value |= 0x2d;
		if (i2c_write(0x8, 0x2e, 1, &value, 1)) {
			printf("Set SW1C error!\n");
			return;
		}
	}
}
#endif
#endif /* CONFIG_SYS_I2C_MXC */

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_gpio(void)
{
	imx_iomux_v3_setup_multiple_pads(gpio_pads_iomux, ARRAY_SIZE(gpio_pads_iomux));
}

#ifdef CONFIG_FSL_ESDHC

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR, 0, 4},
	{USDHC4_BASE_ADDR, 0, 8},
};

int mmc_map_to_kernel_blk(int dev_no)
{
	if (1 == dev_no)
		dev_no = 2;

	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i;

	/*
	* According to the board_mmc_init() the following map is done:
	* (U-boot device node)    (Physical Port)
	* mmc0                    USDHC2 (SD-card)
	* mmc1                    USDHC4 (EMMC)
	*/
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;

		case 1:
			imx_iomux_v3_setup_multiple_pads(usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;

		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return 0;
			}

		if (fsl_esdhc_initialize(bis, &usdhc_cfg[i]))
			printf("Warning: failed to initialize mmc dev %d\n", i);
	}

	return 0;
}

#endif /* CONFIG_FSL_ESDHC */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	gd->flags |= GD_FLG_SILENT;
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0", MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	{NULL,   0},
};
#endif

int board_late_init(void)
{
  int ret;
  char* tmp;
  unsigned long hwcode;

#ifdef CONFIG_CMD_BMODE
  add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_SYS_I2C_MXC
  setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
  ret = setup_pmic_voltages();
  if (ret)
    return -1;
#endif

#if 0
  /* Get the system configuration from the I2C SEEPROM */
  if(read_eeprom())
  {
    printf("Failed to read the HW cfg from the I2C SEEPROM: trying to load it from USB ...\n");
    USBgethwcfg();
  }
#endif
  /* Set the "board_name" env. variable according with the "hw_code" */
  tmp = getenv("hw_code");
  if(!tmp)
  {
    puts ("WARNING: 'hw_code' environment var not found!\n");
    // return 1;
  }
  hwcode = (simple_strtoul (tmp, NULL, 10))&0xff;
  
  if(hwcode==ETOP7XX_VAL)
    setenv("board_name", "usom_etop7xx"); 
  else if(hwcode==BE15A_VAL)
    setenv("board_name", "usom_be15a"); 
  else if(hwcode==ETOP7XXQ_VAL)
    setenv("board_name", "usom_etop7xxq"); 
  else
  {
    puts ("WARNING: unknowm carrier hw code; using 'usom_undefined' board name. \n");
    setenv("board_name", "usom_undefined");
  }
  
  setenv("bootcmd", "");
  
  setup_iomux_gpio();
  FRAM_test();
  mcp3204_test();
  
  return 0;
}


int checkboard(void)
{
	int rev = get_cpu_rev();
	printf("CPU rev.= 0x%x\n", rev);
	return 0;
}

#ifdef CONFIG_IMX_UDC
iomux_v3_cfg_t const otg_udc_pads[] = {
	(MX6_PAD_ENET_RX_ER__GPIO1_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
void udc_pins_setting(void)
{
	imx_iomux_v3_setup_multiple_pads(otg_udc_pads,
		ARRAY_SIZE(otg_udc_pads));

	/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
    mxc_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif /*CONFIG_IMX_UDC*/

#ifdef CONFIG_USB_EHCI_MX6
iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_ENET_RX_ER__GPIO1_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const usb_pads[] = {
	MX6_PAD_EIM_WAIT__GPIO5_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
			ARRAY_SIZE(usb_otg_pads));

		/*set daisy chain for otg_pin_id on 6q. for 6dl, this bit is reserved*/
		mxc_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		printf("Resetting MXC USB port %d ...\n", port);
		imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));
		/* Reset USB hub */
		gpio_direction_output(IMX_GPIO_NR(5, 0), 0);
		mdelay(20);
		gpio_set_value(IMX_GPIO_NR(5, 0), 1);
		mdelay(20);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif

