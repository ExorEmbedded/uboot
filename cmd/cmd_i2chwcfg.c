// SPDX-License-Identifier: GPL-2.0+
 /* This file gives support for I2C SEEPROM HW parameters handling.
  * Factory section v3 supported.
  * DM_I2C model supported.
  *
  * Author: Giovanni Pavoni Exor Int.
  *
  */
 
#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <stdlib.h>    /* env_get, atoi */
#include <i2c.h>
#include <net.h>
#include <dm.h>

#ifdef CONFIG_CMD_I2CHWCFG

/*=======================================================================
 * DM_I2C adaption layer
 *======================================================================= */
#if CONFIG_IS_ENABLED(DM_I2C)
static struct udevice *i2c_cur_bus;

static int cmd_i2c_set_bus_num(unsigned int busnum)
{
	struct udevice *bus;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, busnum, &bus);
	if (ret) {
		debug("%s: No bus %d\n", __func__, busnum);
		return ret;
	}
	i2c_cur_bus = bus;

	return 0;
}

static int i2c_get_cur_bus(struct udevice **busp)
{
#ifdef CONFIG_I2C_SET_DEFAULT_BUS_NUM
	if (!i2c_cur_bus) {
		if (cmd_i2c_set_bus_num(CONFIG_I2C_DEFAULT_BUS_NUMBER)) {
			printf("Default I2C bus %d not found\n",
			       CONFIG_I2C_DEFAULT_BUS_NUMBER);
			return -ENODEV;
		}
	}
#endif

	if (!i2c_cur_bus) {
		puts("No I2C bus selected\n");
		return -ENODEV;
	}
	*busp = i2c_cur_bus;

	return 0;
}

static int i2c_get_cur_bus_chip(uint chip_addr, struct udevice **devp)
{
	struct udevice *bus;
	int ret;

	ret = i2c_get_cur_bus(&bus);
	if (ret)
		return ret;

	return i2c_get_chip(bus, chip_addr, 1, devp);
}
#endif

/*
 * Read/Write interface:
 *   chip:    I2C chip address, range 0..127
 *   addr:    Memory (register) address within the chip
 *   alen:    Number of bytes to use for addr (typically 1, 2 for larger
 *              memories, 0 for register type devices with only one
 *              register)
 *   buffer:  Where to read/write the data
 *   len:     How many bytes to read/write
 *
 *   Returns: 0 on success, not 0 on failure
 */
static int i2c_seep_read(uint8_t chip, unsigned int addr, int alen, uint8_t *buffer, int len);
static int i2c_seep_write(uint8_t chip, unsigned int addr, int alen, uint8_t *buffer, int len);

static int i2c_seep_read(uint8_t chip, unsigned int addr, int alen, uint8_t *buffer, int len)
{
	int ret;
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *dev;

	ret = i2c_get_cur_bus_chip(chip, &dev);
	if (!ret && alen != -1)
		ret = i2c_set_chip_offset_len(dev, alen);
	if (!ret)
		ret = dm_i2c_read(dev, addr, buffer, len);
#else
	ret = i2c_read(chip, addr, alen, buffer, len);
#endif
    return ret;
}

static int i2c_seep_write(uint8_t chip, unsigned int addr, int alen, uint8_t *buffer, int len)
{
	int ret;
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *dev;

	ret = i2c_get_cur_bus_chip(chip, &dev);
	if (!ret && alen != -1)
		ret = i2c_set_chip_offset_len(dev, alen);
	if (ret != 0){
		puts("Error i2c_seep_write\n");
		return ret;
	}
	ret = dm_i2c_write(dev, addr, buffer, len);
#else
	ret = i2c_write(chip, addr, alen, buffer, len);
#endif
    return ret;
}


#define ENET_ADDR_LENGTH 6
/*=======================================================================
 * Layout of I2C memory
 *======================================================================= */
#define FACTORY_SECTION_SIZE_V2	128	/* First part of I2C = factory section for V2 and V1*/

/* Header */
#define SIGNATURE1_POS	0
#define SIGNATURE2_POS	1
#define VERSION_POS		2
#define CKSUM_POS		3

/* Data */
#define DISPID_POS		5
#define JUMPERFLAGSL_POS	14
#define TCHTYPE_POS		21

#define MACID0_POS		24
#define MACID1_POS		25
#define MACID2_POS		26
#define MACID3_POS		27
#define MACID4_POS		28
#define MACID5_POS		29

/* --------------------- Format 3 specific ------------------------------ */
#define FACTORY_SECTION_SIZE_V3	64	/* First part of I2C = factory section for V3*/
#define BL_TIME_CNT		130
#define BL_TIME_CHK		134
#define SYS_TIME_CNT		136
#define SYS_TIME_CHK		140
#define HWPICKPANELCODE_POS     4

#define ZEROCKSUM		0x57
#define SAFE_FACTORY_AREA_POS	184

#define AUXMACID0_POS                   55
#define AUXMACID1_POS                   56
#define AUXMACID2_POS                   57
#define AUXMACID3_POS                   58
#define AUXMACID4_POS                   59
#define AUXMACID5_POS                   60

#define THIRDMACID_POS                  49

/*=======================================================================
 * I2C pre-defined / fixed values
 *======================================================================= */
#define SIGNATURE1_VAL	0xaa
#define SIGNATURE2_VAL	0x55
#define VERSION_VAL		3
#define RFU_VAL			0xff

#ifdef CONFIG_CMD_DISPLAYCONFIGXML
  extern int getdisplay(unsigned int lcdid);
#endif

/*=======================================================================
 * Accessor method for reading the SWmoduleFlag area
 *======================================================================= */
#define SWMODULEFLAG_AREA_OFFSET 248
#define SWMODULEFLAG_AREA_LEN    8
#define ANDROID_BIT_POS          5

/* Return the value of the specified bit (bitIndex) in the SWModuleFlag are.
 * 
 * Return: false (the bit is not set, or we have an error)
 *         true  (the bit is set)
 */
bool getBitInSWModuleFlagArea(unsigned int bitIndex)
{
  unsigned char   SWModuleArea[SWMODULEFLAG_AREA_LEN];
  unsigned char   i, chksum = 1;
  
  if(bitIndex/8 >= SWMODULEFLAG_AREA_LEN) 
    return false;
 
  if (i2c_seep_read(CONFIG_SYS_I2C_EEPROM_ADDR, SWMODULEFLAG_AREA_OFFSET, 1, SWModuleArea, SWMODULEFLAG_AREA_LEN) != 0)
    return false;
  
  for(i = 0; i < SWMODULEFLAG_AREA_LEN-1 ; i++)
    chksum += SWModuleArea[i]; 
  chksum -= 0xAA;
  if(chksum != SWModuleArea[SWMODULEFLAG_AREA_LEN-1])
  {
    return false;    //checksum error
  }
  return (SWModuleArea[bitIndex/8] & (1 << (bitIndex & 7)))? true : false;
}

unsigned char read_factory_mode_flag(void)
{
  unsigned char factory_mode = 0;

  i2c_seep_read(0x50, 0xa3, 1, &factory_mode, 1);
  return factory_mode;
}

/* ======================================================================
 * Helper function indicating if the buffer contents define a valid 
 * factory section, format >= 3. Returns 1 if valid, 0 otherwise.
 * ====================================================================== */
int isvalidfactory3(unsigned char* buf)
{
  unsigned char cksum;
  int i;
  
  /* Version check; failure if version < 3 */
  if(buf[VERSION_POS] < 3)     
    return 0;
    
  /* Checksum check */  
  cksum = 1;
  for(i = CKSUM_POS + 1; i < FACTORY_SECTION_SIZE_V3; i ++)
    cksum += buf[i];
  cksum -= 0xaa;
  
  if(buf[CKSUM_POS] != cksum)
    return 0;
  
  /* Signature check */
  if( (buf[SIGNATURE1_POS] != SIGNATURE1_VAL) || (buf[SIGNATURE2_POS] != SIGNATURE2_VAL) )
    return 0;
  
  /* It is a valid factory version 3 */
  return 1;
}


/* ======================================================================
 * Helper function returning the factory section area of the selected i2c
 * seeprom device.
 * In case of seeprom format >=3, the proper alignment/handling of the
 * safe factory section area is performed transparently.
 * Returns 1 in case of i2c error.
 * ====================================================================== */
int getfactorysection(unsigned char i2cslaveaddr, unsigned char* buf)
{
  unsigned char safefactory_buf[FACTORY_SECTION_SIZE_V3];
  int i;
  
#if CONFIG_IS_ENABLED(DM_I2C)
#else
  i2c_init(CONFIG_SYS_I2C_SPEED, i2cslaveaddr);
#endif
  /* Reads the factory section from seeprom */
  if (i2c_seep_read(i2cslaveaddr, 0, 1, buf, FACTORY_SECTION_SIZE_V3) != 0)
  {
    puts ("I2C error reading the factory section\n");
    return 1;
  }
  
  /* Reads the safe factory section from seeprom */
  if (i2c_seep_read(i2cslaveaddr, SAFE_FACTORY_AREA_POS, 1, safefactory_buf, FACTORY_SECTION_SIZE_V3) != 0)
  {
    puts ("I2C error reading the factory section\n");
    return 1;
  }
  
  /* Performs synchronization/handling of the safe factory section if we detect a format >= 3 */
  if(isvalidfactory3(buf))   //If we have a valid v3 factory section area...
  {
    /* ...copy factory section to safe factory section, wherever any difference is found  */
    for(i = 0; i < FACTORY_SECTION_SIZE_V3; i ++)
      if(buf[i] != safefactory_buf[i])
      {
        i2c_seep_write (i2cslaveaddr, SAFE_FACTORY_AREA_POS + i, 1, &(buf[i]), 1);
        udelay(20000);
      }
	
  }
  else                       // else: The factory section area is not a valid version 3 one ...
  {
    if(isvalidfactory3(safefactory_buf)) //...if the safe factory section is a valid version 3 one
    {                                    //...copy the safe area to the factory area (recovery)
      for(i = 0; i < FACTORY_SECTION_SIZE_V3; i ++)
	if(buf[i] != safefactory_buf[i])
	{
	  buf[i]= safefactory_buf[i];
	  if(i2c_seep_write (i2cslaveaddr, i, 1, &(buf[i]), 1))
	  {
	    puts ("I2C error writing the factory section\n");
	    return 1;
	  }
	  udelay(20000);
	}
    }
    else // In this case neither the factory section nor the safe factory section areas are valid format 3... assume an old version 2 is 
    {    // used. Just read the factory section assuming a version 2 size and leave the upper levels to validate its contents.
      if (i2c_seep_read(i2cslaveaddr, 0, 1, buf, FACTORY_SECTION_SIZE_V2) != 0)
      {
	puts ("I2C error reading the factory section:2\n");
	return 1;
      }
    }
  }
  
  return 0;
}


/* ======================================================================
 * Perform HW cfg load from I2C SEEPROM to env vars
 * ====================================================================== */
int i2cgethwcfg (void)
{
  unsigned char buf[FACTORY_SECTION_SIZE_V2];
  unsigned char adpbuf[FACTORY_SECTION_SIZE_V2];
  unsigned char cksum;
  int i, j;
  char label[40];
  
  /* Reads the I2C contents */
  if(getfactorysection(CONFIG_SYS_I2C_EEPROM_ADDR, buf))
    return 1;
  
  /* Checksum check */
  if(buf[VERSION_POS] < 3) 
	j = FACTORY_SECTION_SIZE_V2;
  else
	j = FACTORY_SECTION_SIZE_V3;
  
  cksum = 1;
  for(i = CKSUM_POS + 1; i < j; i ++)
	cksum += buf[i];
  cksum -= 0xaa;
  
  if(buf[CKSUM_POS] != cksum)
  {
	puts ("I2C SEEPROM checksum error: HW cfg not valid\n");
	return 1;
  }
  
  /* Signature check */
  if( (buf[SIGNATURE1_POS] != SIGNATURE1_VAL) || (buf[SIGNATURE2_POS] != SIGNATURE2_VAL) )
  {
	puts ("I2C SEEPROM invalid signature: HW cfg not valid\n");
	return 1;
  }
	
  /* Get display number and touch type */
#ifdef DEF_SYS_I2C_ADPADD
  /* If display ID=0xff, get them from the ADP board */
  if(buf[DISPID_POS] == 0xff)
  {
    if(getfactorysection(DEF_SYS_I2C_ADPADD, adpbuf))
      return 1;
    
    buf[DISPID_POS] = adpbuf[DISPID_POS];
    buf[TCHTYPE_POS] = adpbuf[TCHTYPE_POS];
  }
#endif
  sprintf(label, "%u", buf[DISPID_POS]);
  env_set("hw_dispid", label); 
  sprintf(label, "%u", buf[TCHTYPE_POS]);
  env_set("touch_type", label); 
  #ifdef CONFIG_CMD_DISPLAYCONFIGXML
  getdisplay(buf[DISPID_POS]);
#endif
  
  /* *** Now gets datas for version >= 2 of the SEEPROM *** */
  if(buf[VERSION_POS] < 2) 
  {
	puts ("I2C SEEPROM WARNING: old version 1 found\n");
	return 1;
  }

  /* get eth mac ID */
  eth_env_set_enetaddr("ethaddr", &(buf[MACID0_POS]));
  
  /* get boardhwcode */
  sprintf(label, "%u", buf[HWPICKPANELCODE_POS]);
  env_set("hw_code", label); 
  
  /* get jumperflagsl */
  sprintf(label, "%u", buf[JUMPERFLAGSL_POS]);
  env_set("jumperflagsl", label); 

  /* get 2nd eth mac ID */
  eth_env_set_enetaddr("eth1addr", &(buf[AUXMACID0_POS]));

  /* get 3rd eth mac ID */
  eth_env_set_enetaddr("eth2addr", &(buf[THIRDMACID_POS]));
  
  /* get the swflag_android from the SWModuleFlagArea */
  if(getBitInSWModuleFlagArea(ANDROID_BIT_POS))
  {
    env_set("swflag_android", "1"); 
  }
  else
  {
    env_set("swflag_android", "0"); 
  }
 
  return 0;
}
  

/* ======================================================================
 * Performs the HW cfg store to I2C SEEPROM
 * ====================================================================== */
int do_i2csavehw (struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
  unsigned char buf[FACTORY_SECTION_SIZE_V3];
  unsigned long i;
  char* tmp;
  unsigned char dispid;
  unsigned char hwcode;
  unsigned char tchtype;
  u8 hw_addr[ENET_ADDR_LENGTH];
  unsigned char cksum;
  unsigned char n;

  if (argc>1) 
  {
	puts ("ERROR: Too many input parameters!\n");
	return 1;
  }
  
  /* Initializes the buffer ... uses still existing SEEPROM datas, if valid */

  /* Reads the I2C contents ...*/
  i2c_seep_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, 1, buf, FACTORY_SECTION_SIZE_V3);
  /* ...and verifies chksum for format 3*/
  cksum = 1;
  for(i = CKSUM_POS + 1; i < FACTORY_SECTION_SIZE_V3; i ++)
	cksum += buf[i];
  cksum -= 0xaa;
  
  if(buf[CKSUM_POS] != cksum)
  {
	puts ("WARNING: No SEEPROM format 3 found ... initializing  ...\n");
	for(i=0; i < FACTORY_SECTION_SIZE_V3; i++)
	  buf[i] = RFU_VAL;
  }
  
  /* Sets the pre-defined fields of buffer */
  buf[SIGNATURE1_POS] = SIGNATURE1_VAL;
  buf[SIGNATURE2_POS] = SIGNATURE2_VAL;
  buf[VERSION_POS] = VERSION_VAL;
  
  /* Now copy into buffer the datas from env vars */ 
  
  /* Display ID handling */
  tmp = env_get("hw_dispid");
  if(!tmp)
  {
	puts ("ERROR: 'hw_dispid' environment var not found!\n");
	return 1;
  }
  i = simple_strtoul (tmp, NULL, 10);
  dispid = (unsigned char)(i & 0xff);
  buf[DISPID_POS] = dispid;
  
  /* Eth mac address handling */
  if (eth_env_get_enetaddr("ethaddr", hw_addr))
  {
	buf[MACID0_POS] = hw_addr[0];
	buf[MACID1_POS] = hw_addr[1];
	buf[MACID2_POS] = hw_addr[2];
	buf[MACID3_POS] = hw_addr[3];
	buf[MACID4_POS] = hw_addr[4];
	buf[MACID5_POS] = hw_addr[5];
  }
  else
  {
	puts ("ERROR: 'ethaddr' environment var not found!\n");
	return 1;
  }

  /* 2nd Eth mac address handling */
  if (eth_env_get_enetaddr("eth1addr", hw_addr))
  {
	buf[AUXMACID0_POS] = hw_addr[0];
	buf[AUXMACID1_POS] = hw_addr[1];
	buf[AUXMACID2_POS] = hw_addr[2];
	buf[AUXMACID3_POS] = hw_addr[3];
	buf[AUXMACID4_POS] = hw_addr[4];
	buf[AUXMACID5_POS] = hw_addr[5];
  }
  else
  {
	puts ("WARNING: 'eth1addr' environment var not found ... skipped\n");
  }

  /* 3rd Eth mac address handling */
  if (eth_env_get_enetaddr("eth2addr", hw_addr))
  {
	buf[THIRDMACID_POS + 0] = hw_addr[0];
	buf[THIRDMACID_POS + 1] = hw_addr[1];
	buf[THIRDMACID_POS + 2] = hw_addr[2];
	buf[THIRDMACID_POS + 3] = hw_addr[3];
	buf[THIRDMACID_POS + 4] = hw_addr[4];
	buf[THIRDMACID_POS + 5] = hw_addr[5];
  }
  
  /* hw_code handling */
  tmp = env_get("hw_code");
  if(!tmp)
  {
    puts ("WARNING: 'hw_code' environment var not found!\n");
  }
  else
  {
    i = simple_strtoul (tmp, NULL, 10);
    hwcode = (unsigned char)(i & 0xff);
    buf[HWPICKPANELCODE_POS] = hwcode;
  }
  
  /* touch_type handling */
  tmp = env_get("touch_type");
  if(!tmp)
  {
    puts ("WARNING: 'touch_type' environment var not found!\n");
  }
  else
  {
    i = simple_strtoul (tmp, NULL, 10);
    tchtype = (unsigned char)(i & 0xff);
    buf[TCHTYPE_POS] = tchtype;
  }

  /* Calculate and copies checksum into buffer */
  cksum = 1;
  for(i = CKSUM_POS + 1; i < FACTORY_SECTION_SIZE_V3; i ++)
	cksum += buf[i];
  cksum -= 0xaa;
  
  buf[CKSUM_POS] = cksum;
  
  /* Now write buffer into SEEPROM */
  puts ("Writing HW configuration to I2C SEEPROM ...\n");
  for(n=0; n < FACTORY_SECTION_SIZE_V3; n++)
  {
      i2c_seep_write(CONFIG_SYS_I2C_EEPROM_ADDR, n, 1, &(buf[n]), 1);
      udelay(20000);
  }
  
  return 0;
}


/* ====================================================================== */

U_BOOT_CMD(
	i2csavehw,      2,      0,      do_i2csavehw,
	" Stores display id and MAC id into I2C SEEPROM (format v3) keeping other params unchanged.\n",
	" If no valid SEEPROM 3 datas are found, SEEPROM is initialized with defaults\n"
);


#endif	/* CONFIG_CMD_I2CHWCFG */

