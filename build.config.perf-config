#
# Copyright (c) 2020 Microsoft Corporation
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 2 as published by
# the Free Software Foundation.
#
KERNEL_DIR=kernel/msm-4.14
. ${ROOT_DIR}/${KERNEL_DIR}/build.config.common
DEFCONFIG=vendor/surfaceduo-perf_defconfig
POST_DEFCONFIG_CMDS="update_perf_config"

function update_perf_config() {
  # Disable clang-specific options
  ${KERNEL_DIR}/scripts/config --file ${OUT_DIR}/.config \
    -d CONFIG_QCOM_CAMERA_TEMPERATURE
  (cd ${OUT_DIR} && \
    make O=${OUT_DIR} $archsubarch CROSS_COMPILE=${CROSS_COMPILE} olddefconfig)
}
