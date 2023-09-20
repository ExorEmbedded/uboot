#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mm_us04.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
ifdef CONFIG_TARGET_IMX8MM_NS06
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing_ns06.o
else
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif
ifdef CONFIG_TARGET_IMX8MM_NS04
obj-y += ddr/helper.o
obj-y += ddr/ddr3l/ddr_init.o ddr/ddr3l/ddrphy_train.o
endif
ifdef CONFIG_TARGET_IMX8MN_NS05
obj-y += ddr/helper.o
obj-y += ddr/ddr3l_imx8mn_ns05/ddr_init.o ddr/ddr3l_imx8mn_ns05/ddrphy_train.o
endif

endif
