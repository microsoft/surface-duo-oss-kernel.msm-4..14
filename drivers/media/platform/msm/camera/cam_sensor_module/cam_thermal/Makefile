#
# Copyright (c) 2020 Microsoft Corporation
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 2 as published by
# the Free Software Foundation.
#
ccflags-y += -Idrivers/media/platform/msm/camera/cam_core
ccflags-y += -Idrivers/media/platform/msm/camera/cam_cpas/include
ccflags-y += -Idrivers/media/platform/msm/camera/cam_req_mgr
ccflags-y += -Idrivers/media/platform/msm/camera/cam_utils
ccflags-y += -Idrivers/media/platform/msm/camera/cam_smmu
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_cci
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_io
ccflags-y += -Idrivers/media/platform/msm/camera/cam_sensor_module/cam_sensor_utils

ifeq ($(CONFIG_QCOM_CAMERA_TEMPERATURE),y)
obj-$(CONFIG_SPECTRA_CAMERA) += cam_thermal_reg.o cam_thermal_sensor.o
else
obj-$(CONFIG_SPECTRA_CAMERA) += cam_thermal_reg_dummy.o
endif