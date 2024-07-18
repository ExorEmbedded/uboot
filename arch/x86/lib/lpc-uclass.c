// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <common.h>
#include <dm.h>

DECLARE_GLOBAL_DATA_PTR;

UCLASS_DRIVER(lpc) = {
	.id		= UCLASS_LPC,
	.name		= "lpc",
	.post_bind	= dm_scan_fdt_dev,
};
