/*
 * board.c
 *
 * Board functions for uS01 AM335X test system
 *
 * Copyright (C) 2014 Exor S.p.a.
 *
 * Based on:
 * u-boot:/board/ti/am335x/board.c
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

void ena_rs232phy(void);
static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;
static struct cpsw_slave_data cpsw_slaves[];

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
 * Reads the $hwcfg$.txt file from USB stick (root of FATFS partition) if any, parses it
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


#if defined(CONFIG_SPL_BUILD)
static const struct ddr_data ddr3_data = {
  .datardsratio0 = MT41K256M16HA125E_RD_DQS,
  .datawdsratio0 = MT41K256M16HA125E_WR_DQS,
  .datafwsratio0 = MT41K256M16HA125E_PHY_FIFO_WE,
  .datawrsratio0 = MT41K256M16HA125E_PHY_WR_DATA,
};

static const struct cmd_control ddr3_cmd_ctrl_data = {
  .cmd0csratio = MT41K256M16HA125E_RATIO,
  .cmd0iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

  .cmd1csratio = MT41K256M16HA125E_RATIO,
  .cmd1iclkout = MT41K256M16HA125E_INVERT_CLKOUT,

  .cmd2csratio = MT41K256M16HA125E_RATIO,
  .cmd2iclkout = MT41K256M16HA125E_INVERT_CLKOUT,
};

static struct emif_regs ddr3_emif_reg_data = {
  .sdram_config = MT41K256M16HA125E_EMIF_SDCFG,
  .ref_ctrl = MT41K256M16HA125E_EMIF_SDREF,
  .sdram_tim1 = MT41K256M16HA125E_EMIF_TIM1,
  .sdram_tim2 = MT41K256M16HA125E_EMIF_TIM2,
  .sdram_tim3 = MT41K256M16HA125E_EMIF_TIM3,
  .zq_config = MT41K256M16HA125E_ZQ_CFG,
  .emif_ddr_phy_ctlr_1 = MT41K256M16HA125E_EMIF_READ_LATENCY,
};


#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
  /* break into full u-boot on 'c' */
  return (serial_tstc() && serial_getc() == 'c');
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr = {400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
  int mpu_vdd;
  int sil_rev;

  printf("am33xx_spl_board_init++\n");
	
  /* Get the frequency */
  dpll_mpu_opp100.m = am335x_get_efuse_mpu_max_freq(cdev);
  printf("dpll_mpu_opp100.m = %d\n",dpll_mpu_opp100.m);

  /*
  * The uS01 systems use a TPS65910 PMIC.  For all
  * MPU frequencies we support we use a CORE voltage of
  * 1.1375V.  For MPU voltage we need to switch based on
  * the frequency we are running at.
  */
  printf("Probing TPS65910...\n");

  if (i2c_probe(TPS65910_CTRL_I2C_ADDR))
    goto spl_setpll;
  printf("Probing TPS65910...:OK\n");
	
  /*
  * Depending on MPU clock and PG we will need a different
  * VDD to drive at that speed.
  */
  sil_rev = readl(&cdev->deviceid) >> 28;
  mpu_vdd = am335x_get_tps65910_mpu_vdd(sil_rev, dpll_mpu_opp100.m);
  printf("mpu_vdd...%d\n",mpu_vdd);

  /* Tell the TPS65910 to use i2c */
  tps65910_set_i2c_control();
  
  /* First update MPU voltage. */
  if (tps65910_voltage_update(MPU, mpu_vdd))
    return;
  printf("mpu_vdd...SET OK\n");

  /* Second, update the CORE voltage. */
  if (tps65910_voltage_update(CORE, TPS65910_OP_REG_SEL_1_1_3))
    return;
  printf("CORE voltage...SET OK\n");
spl_setpll:
  /* Set CORE Frequencies to OPP100 */
  do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);

  /* Set MPU Frequency to what we detected now that voltages are set */
  do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
  printf("am33xx_spl_board_init--\n");
}

const struct dpll_params *get_dpll_ddr_params(void)
{
  enable_i2c0_pin_mux();
  i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

  return &dpll_ddr;
}

void set_uart_mux_conf(void)
{
#ifdef CONFIG_SERIAL1
  enable_uart0_pin_mux();
#endif /* CONFIG_SERIAL1 */
#ifdef CONFIG_SERIAL2
  enable_uart1_pin_mux();
#endif /* CONFIG_SERIAL2 */
}

void set_mux_conf_regs(void)
{
  enable_board_pin_mux();
  
  gpio_request(ESDBY_GPIO,"");
  gpio_direction_output(ESDBY_GPIO,1);
  
  gpio_request(E3W3S_GPIO,"");
  gpio_direction_output(E3W3S_GPIO,1);
}

const struct ctrl_ioregs ioregs_uS01 = {
  .cm0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
  .cm1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
  .cm2ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
  .dt0ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
  .dt1ioctl		= MT41K256M16HA125E_IOCTRL_VALUE,
};

void sdram_init(void)
{
  config_ddr(400, &ioregs_uS01,
    &ddr3_data,
    &ddr3_cmd_ctrl_data,
    &ddr3_emif_reg_data, 0);
}

/*
 * Init the mmc (mmc0) for the dedicated/custom SPL u-boot loading sequence. 
 */
int spl_board_mmc_initialize(void)
{
  return omap_mmc_init(0, 0, 0, -1, -1);
}

/*
 * Init the EMMC (mmc1) for the dedicated/custom SPL u-boot loading sequence. 
 */
int spl_board_emmc_initialize(void)
{
  return omap_mmc_init(1, 0, 0, -1, -1);
}
#endif

void ena_rs232phy(void)
{
}

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
  printf("board_init++\n");
#if defined(CONFIG_HW_WATCHDOG)
  hw_watchdog_init();
#endif
  gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
  printf("board_init--\n");
  return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
#define US01_256MB_300MHZ 0x02
int board_late_init(void)
{
  char* tmp;
  unsigned long hwcode;
  unsigned char buf;
  
  /* Get the system configuration from USB stick */
  ena_rs232phy();
  printf("Reading the system configuration from USB stick ...\n");
  USBgethwcfg();
  
  /* Initialize the MCP23016 I/O expander @ 0x20 to all outs to 0
   */
  buf=0;
  i2c_write (0x20, 6, 1, &buf, 1);
  udelay(2000);
	
  buf=0;
  i2c_write (0x20, 7, 1, &buf, 1);
  udelay(2000);

  buf=0;
  i2c_write (0x20, 0, 1, &buf, 1);
  udelay(2000);

  buf=0;
  i2c_write (0x20, 1, 1, &buf, 1);
  udelay(2000);
 
  /* Initialize the PCA9536 I/O expander @ 0x41 to out0=out1=out2=0
   */
  buf=0xf8;
  i2c_write (0x41, 3, 1, &buf, 1);
  udelay(2000);

  buf=0xf8;
  i2c_write (0x41, 1, 1, &buf, 1);
  udelay(2000);
    
  /* Set the "board_name" env. variable according with the "hw_code" */
  tmp = getenv("hw_code");
  if(!tmp)
  {
    puts ("WARNING: 'hw_code' environment var not found!\n");
    return 1;
  }
  hwcode = (simple_strtoul (tmp, NULL, 10))&0xff;
  
  if((hwcode==ETOP507_VAL)||(hwcode==ETOP507G_VAL))
    setenv("board_name", "usom_etop5xx"); 
  else if(hwcode==ECO_VAL)
    setenv("board_name", "usom_eco"); 
  else if(hwcode==PLCM07_VAL)
    setenv("board_name", "usom_plcm07"); 
  else
  {
    puts ("WARNING: unknowm carrier hw code; using 'usom_undefined' board name. \n");
    setenv("board_name", "usom_undefined");
  }
  
  /* Platform related ethernet configuration */
  if((hwcode==ETOP507_VAL)||(hwcode==ETOP507G_VAL))
    cpsw_slaves[0].phy_addr = 2;
  
  return 0;
}
#endif

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) 
static void cpsw_control(int enabled)
{
  /* VTP can be added here */
  return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
  {
    .slave_reg_ofs	= 0x208,
    .sliver_reg_ofs	= 0xd80,
    .phy_addr 		= 1,  //NOTE: This value will be set to 2 in case of etop5xx (we have a switch in this case)
  },
  {
  .slave_reg_ofs 	= 0x308,
  .sliver_reg_ofs	= 0xdc0,
  .phy_addr		= 0,
  },
};

static struct cpsw_platform_data cpsw_data = {
  .mdio_base		= CPSW_MDIO_BASE,
  .cpsw_base		= CPSW_BASE,
  .mdio_div		= 0xff,
  .channels		= 8,
  .cpdma_reg_ofs	= 0x800,
  .slaves		= 1,
  .slave_data		= cpsw_slaves,
  .ale_reg_ofs		= 0xd00,
  .ale_entries		= 1024,
  .host_port_reg_ofs	= 0x108,
  .hw_stats_reg_ofs	= 0x900,
  .bd_ram_ofs		= 0x2000,
  .mac_control		= (1 << 5),
  .control		= cpsw_control,
  .host_port_num	= 0,
  .version		= CPSW_CTRL_VERSION_2,
};
#endif

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD))
int board_eth_init(bd_t *bis)
{
  int rv, n = 0;
  uint8_t mac_addr[6];
  uint32_t mac_hi, mac_lo;

  /* try reading mac address from efuse */
  mac_lo = readl(&cdev->macid0l);
  mac_hi = readl(&cdev->macid0h);
  mac_addr[0] = mac_hi & 0xFF;
  mac_addr[1] = (mac_hi & 0xFF00) >> 8;
  mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
  mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
  mac_addr[4] = mac_lo & 0xFF;
  mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD))
  if (!getenv("ethaddr")) {
    printf("<ethaddr> not set. Validating first E-fuse MAC\n");

  if (is_valid_ether_addr(mac_addr))
    eth_setenv_enetaddr("ethaddr", mac_addr);
  }

  mac_lo = readl(&cdev->macid1l);
  mac_hi = readl(&cdev->macid1h);
  mac_addr[0] = mac_hi & 0xFF;
  mac_addr[1] = (mac_hi & 0xFF00) >> 8;
  mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
  mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
  mac_addr[4] = mac_lo & 0xFF;
  mac_addr[5] = (mac_lo & 0xFF00) >> 8;

  if (!getenv("eth1addr")) {
    if (is_valid_ether_addr(mac_addr))
      eth_setenv_enetaddr("eth1addr", mac_addr);
  }

  writel(RMII_MODE_ENABLE|RMII_CHIPCKL_ENABLE, &cdev->miisel);	// RMII interface, RMII clock as input
  cpsw_slaves[0].phy_if = cpsw_slaves[1].phy_if = PHY_INTERFACE_MODE_RMII;

  rv = cpsw_register(&cpsw_data);
  if (rv < 0)
    printf("Error %d registering CPSW switch\n", rv);
  else
    n += rv;

#endif
  return n;
}
#endif
