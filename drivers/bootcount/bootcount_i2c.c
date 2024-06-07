// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2013
 * Heiko Schocher, DENX Software Engineering, hs@denx.de.
 */

#include <bootcount.h>
#include <linux/compiler.h>
#include <i2c.h>
#include <dm.h>

#define BC_MAGIC	0xbc

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


void bootcount_store(ulong a)
{
	unsigned char buf[3];
	int ret;
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *dev;

	ret = i2c_get_cur_bus_chip(CFG_SYS_I2C_RTC_ADDR, &dev);
	if (!ret && CONFIG_BOOTCOUNT_ALEN != -1)
		ret = i2c_set_chip_offset_len(dev, CONFIG_BOOTCOUNT_ALEN);
	if (ret != 0){
		puts("Error writing bootcount: i2c_set_chip_offset_len\n");
		return;
	}
#endif

	buf[0] = BC_MAGIC;
	buf[1] = (a & 0xff);

#if CONFIG_IS_ENABLED(DM_I2C)
	ret = dm_i2c_write(dev, CONFIG_SYS_BOOTCOUNT_ADDR, buf, 2);

#else
	ret = i2c_write(CFG_SYS_I2C_RTC_ADDR, CONFIG_SYS_BOOTCOUNT_ADDR,
		  CONFIG_BOOTCOUNT_ALEN, buf, 2);
#endif
	if (ret != 0)
		puts("Error writing bootcount\n");
}

ulong bootcount_load(void)
{
	unsigned char buf[3];
	int ret;
#if CONFIG_IS_ENABLED(DM_I2C)
	struct udevice *dev;

	ret = i2c_get_cur_bus_chip(CFG_SYS_I2C_RTC_ADDR, &dev);
	if (!ret && CONFIG_BOOTCOUNT_ALEN != -1)
		ret = i2c_set_chip_offset_len(dev, CONFIG_BOOTCOUNT_ALEN);
	if (!ret)
		ret = dm_i2c_read(dev, CONFIG_SYS_BOOTCOUNT_ADDR, buf, 2);
#else
	ret = i2c_read(CFG_SYS_I2C_RTC_ADDR, CONFIG_SYS_BOOTCOUNT_ADDR,
		       CONFIG_BOOTCOUNT_ALEN, buf, 2);
#endif
	if (ret != 0) {
		puts("Error loading bootcount\n");
		return 0;
	}
	if (buf[0] == BC_MAGIC)
		return buf[1];

	bootcount_store(0);

	return 0;
}
