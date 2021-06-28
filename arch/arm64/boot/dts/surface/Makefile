# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2020 Microsoft Corporation
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 2 as published by
# the Free Software Foundation.
#
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-a-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-b-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-c-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-5-a-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-5-b-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-a-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-b-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-a-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-b-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-c-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-dv-a-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-dv-b-overlay.dtbo
dtbo-$(CONFIG_ARCH_SM8150) += surface-duo-dv-c-overlay.dtbo
surface-duo-ev1-a-overlay.dtbo-base := surface-duo-ev1-a.dtb
surface-duo-ev1-b-overlay.dtbo-base := surface-duo-ev1-b.dtb
surface-duo-ev1-c-overlay.dtbo-base := surface-duo-ev1-c.dtb
surface-duo-ev1-5-a-overlay.dtbo-base := surface-duo-ev1-5-a.dtb
surface-duo-ev1-5-b-overlay.dtbo-base := surface-duo-ev1-5-b.dtb
surface-duo-ev2-a-overlay.dtbo-base := surface-duo-ev2-a.dtb
surface-duo-ev2-b-overlay.dtbo-base := surface-duo-ev2-b.dtb
surface-duo-ev2-1-a-overlay.dtbo-base := surface-duo-ev2-1-a.dtb
surface-duo-ev2-1-b-overlay.dtbo-base := surface-duo-ev2-1-b.dtb
surface-duo-ev2-1-c-overlay.dtbo-base := surface-duo-ev2-1-c.dtb
surface-duo-dv-a-overlay.dtbo-base := surface-duo-dv-a.dtb
surface-duo-dv-b-overlay.dtbo-base := surface-duo-dv-b.dtb
surface-duo-dv-c-overlay.dtbo-base := surface-duo-dv-c.dtb
else
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-a.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-b.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-c.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-5-a.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev1-5-b.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-a.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-b.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-a.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-b.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-ev2-1-c.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-dv-a.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-dv-b.dtb
dtb-$(CONFIG_ARCH_SM8150) += surface-duo-dv-c.dtb
endif

ifeq ($(CONFIG_ARM64),y)
always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
else
targets += dtbs
targets += $(addprefix ../, $(dtb-y))

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files	:= *.dtb *.dtbo
