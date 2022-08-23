/*
 * Board functions for US03 imx6
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
#define US03KITQ_VAL  119
#define JSMART_VAL    124
#define JSMARTQ_VAL   125
#define JSMARTTTL_VAL 126
#define TA19_VAL      141
#define BE15B_VAL     142
#define TA19DL_VAL    148

#define RXEN0_GPIO IMX_GPIO_NR(6, 31)
#define DXEN0_GPIO IMX_GPIO_NR(3, 14)
#define MODE0_GPIO IMX_GPIO_NR(3, 15)
#define RST_OUT_GPIO IMX_GPIO_NR(7, 12)

#define DVIPLUGIN_I2CSA (0x38)

#define WDOG1WRSRREG (0x20bc004)
#define SNVSLPCRREG  (0x20cc038)

#define TSC2004_RSTGPIO IMX_GPIO_NR(2, 3)
iomux_v3_cfg_t const tsc2004_rst_pads[] = {
	MX6_PAD_NANDF_D3__GPIO2_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),  /* Touch RST for BE15 platform */
};

static void tsc2004_reset(void)
{
  imx_iomux_v3_setup_multiple_pads(tsc2004_rst_pads, ARRAY_SIZE(tsc2004_rst_pads));
  gpio_request(TSC2004_RSTGPIO,"");
  gpio_direction_output(TSC2004_RSTGPIO,0);
  mdelay(1);
}

void ena_rs232phy(void);
static void check_wdog1_or_sw_reset(void);
static int etop7xx_qspi_flash_reset(void);

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
 * Check if the system has restarted due to watchdog or sw reset; in such a case toggle the PMIC_ON_REQ in order to perfrom
 * a clean power cycle (POR).
 * This is the only way to be sure the CPU and PMIC are in a well known reset state.
 */
static void check_wdog1_or_sw_reset(void)
{
  u16 cause;
  cause = __raw_readw(WDOG1WRSRREG);
  
  if((cause & 0x03) !=0 )
  {
    //WE are here due to a SW or WDT reset ... need a clean POR sequence through the PMIC_ON_REQ pin. 
    u32 val;
    if(etop7xx_qspi_flash_reset())
      return;
    printf("SW or WDT reset detected, performing POR...\n");
    mdelay (10);
    val = __raw_readl(SNVSLPCRREG);
    val |= 0x60;
    __raw_writel(val, SNVSLPCRREG);
    while(1); //Wait for POR to occur
  }
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
	MX6_PAD_EIM_DA14__GPIO3_IO14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_DA15__GPIO3_IO15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const rst_out_pads[] = {
	MX6_PAD_GPIO_17__GPIO7_IO12  | MUX_PAD_CTRL(NO_PAD_CTRL),  /* RST_OUT line GPIO7,12 */
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

/* SPI pads, used to reset the QSPI Flash on etop7xx panels
 * in case of SW restart
 */
iomux_v3_cfg_t const spi_pads[] = {
    MX6_PAD_CSI0_VSYNC__GPIO5_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), //SEL_MUX
    MX6_PAD_CSI0_MCLK__GPIO5_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),  //QSPI_CS
    MX6_PAD_EIM_D16__GPIO3_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL),    //SCLK
    MX6_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL),    //SPI_MOSI
};

#define SPI1_CLK  (IMX_GPIO_NR(3, 16))
#define SPI1_SIMO (IMX_GPIO_NR(3, 18))
#define SPI1_CSF  (IMX_GPIO_NR(5, 19))
#define SEL_MUX   (IMX_GPIO_NR(5, 21))

/* Resets the qspi flash on etop7xxq systems only */
static int etop7xx_qspi_flash_reset(void)
{
  unsigned char in = 0x9f;
  int i;
  char* tmp;
  unsigned long hwcode = 0;
  
  /* Check if we are an etop7xxq board and exit if no */
  setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
  i2c_set_bus_num(2);
  read_eeprom();
  tmp = getenv("hw_code");
  if(tmp)
    hwcode = (simple_strtoul (tmp, NULL, 10))&0xff;
  
  if((hwcode!=ETOP7XXQ_VAL) && (hwcode!=ETOP7XX_VAL))
    return 0;

  /* We are an etop7xx board: do qspi flash reset */
  imx_iomux_v3_setup_multiple_pads(spi_pads, ARRAY_SIZE(spi_pads));
  gpio_direction_output(SPI1_CLK, 0);
  gpio_direction_output(SPI1_SIMO, 0);
  gpio_direction_output(SPI1_CSF, 1);
  gpio_direction_output(SEL_MUX, 1);
  mdelay(2);
  
  //1) Send RESET enable cmd
  in = 0x66;
  gpio_set_value(SPI1_CSF, 0);
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
  gpio_set_value(SPI1_CSF, 1);
  mdelay(1);
  
  //1) Send RESET cmd
  in = 0x99;
  gpio_set_value(SPI1_CSF, 0);
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
  gpio_set_value(SPI1_CSF, 1);
  mdelay(1);
  
  //Done: set SEL_MUX to low
  gpio_set_value(SEL_MUX, 0);
  mdelay(1);
  return 1;
}




#ifdef CONFIG_SYS_I2C_MXC

#ifdef CONFIG_RGB_RED_LED
#define PCA9633_I2C_ADDR	0x62
#define PCA9633_MODE1		0x00
#define PCA9633_MODE2		0x01
#define PCA9633_PWM_BASE	0x02
#define PCA9633_LEDOUT		0x08

#define PCA9633_RED_ON	    0x01
#define PCA9633_GREEN_ON	0x04
#define PCA9633_BLUE_ON	    0x10
#define PCA9633_UNUSED_ON	0x40

#define PCA9633_RED_OFF	    0x00
#define PCA9633_GREEN_OFF	0x00
#define PCA9633_BLUE_OFF	0x00
#define PCA9633_UNUSED_OFF	0x00

static void setup_led(void)
{
    unsigned char value = 0xFF;
    i2c_set_bus_num(2);
    if (!i2c_probe(PCA9633_I2C_ADDR)) {

	value = 0x00;
	i2c_write(PCA9633_I2C_ADDR, PCA9633_MODE1, 1, &value, 1);

	value = 0x01;
	i2c_write(PCA9633_I2C_ADDR, PCA9633_MODE2, 1, &value, 1);

	value = (unsigned char) PCA9633_RED_ON | PCA9633_GREEN_OFF | PCA9633_BLUE_OFF | PCA9633_UNUSED_OFF;
	i2c_write(PCA9633_I2C_ADDR, PCA9633_LEDOUT, 1, &value, 1);
    }
}
#endif

/* Detection sequence for the DVI plugin module 
 */
static void DVIpluginDetect(void)
{
  /* Set RST_OUT to HIGH 
   * NOTE: On final design this should be not required and should be removed
   *       to reduce the boot time.
   */ 
  imx_iomux_v3_setup_multiple_pads(rst_out_pads, ARRAY_SIZE(rst_out_pads));
  gpio_request(RST_OUT_GPIO,"");
  gpio_direction_output(RST_OUT_GPIO,0);
  mdelay(5);
  gpio_set_value(RST_OUT_GPIO, 1);
  mdelay(10);
  
  /* Probe the TFP410 DVI out chip for DVI plugin detection */
  i2c_set_bus_num(2);
  if (!i2c_probe(DVIPLUGIN_I2CSA)) 
  {
    printf("DVI plugin module found\n");
    
    /* DVI plugin module found, append the dvi_dispid parameter to the optargs */
    if(!getenv("dvi_dispid"))
      run_command("setenv dvi_dispid 0", 0);
    
    run_command("setenv optargs $optargs dvi_dispid=${dvi_dispid}", 0);
    
    /* When the ADP is not present and we have the DVI plugin installed, the hw_dispid var needs to be invalidated (255) */
    if(i2c_probe(CONFIG_SYS_I2C_ADPADD))
      run_command("setenv hw_dispid 255", 0);
  }
}

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
		
		/* Set LPDDR2 voltage to 1.275V for DualLite, 1.3V for QuadCore */
		value = 0x23;
		if(CONFIG_DDR_MB == 2048)value = 0x24;
		if (i2c_write(0x8, 0x3c, 1, &value, 1)) {
			printf("Set LPDDR2 voltage error!\n");
			return -1;
		}
		value = 0x23;
		if(CONFIG_DDR_MB == 2048)value = 0x24;
		if (i2c_write(0x8, 0x43, 1, &value, 1)) {
			printf("Set LPDDR2 voltage error!\n");
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



int mx6_rgmii_rework(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
	phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

	val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}


/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	return cpu_eth_init(bis);
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
  unsigned long hwcode = 0;
  unsigned long rs232phyena = 0;
  unsigned long jumperflagsl = 0;
  unsigned char value = 0;
  
  setenv("silent", "1"); 
  gd->flags |= GD_FLG_SILENT;

#ifdef CONFIG_CMD_BMODE
  add_board_boot_modes(board_boot_modes);
#endif

  /* Enable the rs232 phy based on "rs232_txen" environment variable */
  tmp = getenv("rs232_txen");
  if(tmp)
  {
    rs232phyena = (simple_strtoul (tmp, NULL, 10))&0xff;
    if(rs232phyena != 0)
    {
      ena_rs232phy();
    }
  }
  
  /* Check the reset cause and perform required actions */
  check_wdog1_or_sw_reset();

#ifdef CONFIG_BE15PLAT
  /* On BE15 target, keep the tsc2004 controller in reset */
  tsc2004_reset();
#else
  /* If i2c bus not responding, try to put the tsc2004 controller in reset */
  setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
  i2c_set_bus_num(2);
  if (i2c_read(0x8, 0, 1, &value, 1))
  {
    mdelay(2000);
    tsc2004_reset();
  }
#endif
  
#ifdef CONFIG_SYS_I2C_MXC
  setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
  ret = setup_pmic_voltages();
  if (ret)
    return -1;
#endif

  /* BSP-1611: Set CAN_CLK_ROOT clock rate to 12Mhz to get a precise baud rate also at 800Kbit*/
  {
    struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
    int reg;
    reg = readl(&mxc_ccm->cscmr2);
    reg &= ~(MXC_CCM_CSCMR2_CAN_CLK_SEL_MASK);
    reg |= (4 << MXC_CCM_CSCMR2_CAN_CLK_SEL_OFFSET);
    writel(reg, &mxc_ccm->cscmr2);
  }
  
  /* Get the system configuration from the I2C SEEPROM */
  if(read_eeprom())
  {
    ena_rs232phy();
    printf("Failed to read the HW cfg from the I2C SEEPROM: trying to load it from USB ...\n");
    USBgethwcfg();
  }
  
  /* Set the "board_name" env. variable according with the "hw_code" */
  tmp = getenv("hw_code");
  if(!tmp)
  {
    puts ("WARNING: 'hw_code' environment var not found!\n");
    // return 1;
  }
  else
    hwcode = (simple_strtoul (tmp, NULL, 10))&0xff;
  
  /* For eTOP7xxL panels, pass the mac addresses for the additional PCIe eth ports via cmdline */
  if((hwcode==ETOP7XX_VAL) || (hwcode==ETOP7XXQ_VAL))
  {
    if(getenv("eth1addr"))
      if(getenv("eth2addr"))
	run_command("setenv optargs pcie_tse1addr=${eth1addr} pcie_tse2addr=${eth2addr}", 0);
    
    /* Detection of the DVI plugin module */
    DVIpluginDetect();
  }
  
  /* For some newer BE15Ax boards we need to pass the eth1 address via cmdline and set rst_out line to high */
  if((hwcode==BE15A_VAL) || (hwcode==BE15B_VAL))
  {
    if(getenv("eth1addr"))
	  run_command("setenv optargs pcie_tse1addr=${eth1addr}", 0);
    
    imx_iomux_v3_setup_multiple_pads(rst_out_pads, ARRAY_SIZE(rst_out_pads));
    gpio_request(RST_OUT_GPIO,"");
    gpio_direction_output(RST_OUT_GPIO, 1);
  }
    
  /* For the TA19 target, the number of cores is forced to 2 */
  if(hwcode==TA19_VAL)
    run_command("setenv optargs maxcpus=2", 0);
  
  if(hwcode==ETOP7XX_VAL)
    setenv("board_name", "usom_etop7xx"); 
  else if(hwcode==BE15A_VAL)
    setenv("board_name", "usom_be15a"); 
  else if(hwcode==ETOP7XXQ_VAL)
    setenv("board_name", "usom_etop7xxq"); 
  else if(hwcode==US03KITQ_VAL)
    setenv("board_name", "usom_evkitq"); 
  else if(hwcode==JSMART_VAL)
    setenv("board_name", "usom_jsmart"); 
  else if(hwcode==JSMARTQ_VAL)
    setenv("board_name", "usom_jsmartq"); 
  else if(hwcode==JSMARTTTL_VAL)
    setenv("board_name", "usom_jsmartttl"); 
  else if(hwcode==TA19_VAL)
    setenv("board_name", "usom_ta19");
  else if(hwcode==TA19DL_VAL)
    setenv("board_name", "usom_ta19_dl");
  else if(hwcode==BE15B_VAL)
    setenv("board_name", "usom_be15b");
  else
  {
    puts ("WARNING: unknowm carrier hw code; using 'usom_undefined' board name. \n");
    setenv("board_name", "usom_undefined");
  }
  
#if defined (CONFIG_SYS_I2C_MXC) && defined (CONFIG_RGB_RED_LED)
    if(hwcode==JSMART_VAL || hwcode==JSMARTQ_VAL || hwcode==JSMARTTTL_VAL)
	setup_led();
#endif

  /* Check if file $0030d8$.bin exists on the 1st partition of the SD-card and, if so, skips booting the mainOS */
  run_command("setenv skipbsp1 0", 0);
  run_command("mmc dev 0", 0);
  run_command("mmc rescan", 0);
  run_command("if test -e mmc 0:1 /$0030d8$.bin; then setenv skipbsp1 1; fi", 0);
    
  /* Determine which mainOS has to be booted (Android vs Linux) based on the swflag_android env. variable, taken from SEEPROM */
  /* Override the swflag_android env. variable if $0030d8android$.bin or $0030d8linux$.bin files are found into the Linux data partition */
  run_command("mmc dev 1", 0);
  run_command("mmc rescan", 0);
  run_command("if test -e mmc 1:6 /$0030d8android$.bin; then setenv swflag_android 1; fi", 0);
  run_command("if test -e mmc 1:6 /$0030d8linux$.bin; then setenv swflag_android 0; fi", 0);
  run_command("if test -e mmc 1:6 /$0030d8gigabit.bin; then setenv enable_gigabit 1; fi", 0);
 
  tmp = getenv("swflag_android");
  if((tmp) && (tmp[0] == '1'))
  {
    puts ("mainOS: Android\n");
    setenv("bootcmd", CONFIG_ANDROID_BOOTCOMMAND);
  }
  else
    setenv("bootcmd", CONFIG_BOOTCOMMAND);
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
	(MX6_PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
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
	(MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
	(MX6_PAD_EIM_D21__USB_OTG_OC | MUX_PAD_CTRL(NO_PAD_CTRL)),
	(MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(UART_PAD_CTRL)),
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

#ifdef CONFIG_HAVEPRGUART
void ena_rs232phy(void)
{
  gpio_request(RXEN0_GPIO,"");
  gpio_direction_output(RXEN0_GPIO,1);
  
  gpio_request(DXEN0_GPIO,"");
  gpio_direction_output(DXEN0_GPIO,1);
  
  gpio_request(MODE0_GPIO,"");
  gpio_direction_output(MODE0_GPIO,0);
  
  run_command("setenv silent", 0);
  gd->flags &= ~GD_FLG_SILENT;
  udelay(1000);
}
#else
void ena_rs232phy(void){}
#endif
