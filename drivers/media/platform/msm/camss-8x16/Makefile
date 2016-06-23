# Makefile for Qualcomm camss driver

ccflags-y += -Idrivers/media/platform/msm/camss
msm-camss-objs += \
		camss.o \
		csid.o \
		csiphy.o \
		ispif.o \
		vfe.o \
		video.o \

obj-$(CONFIG_MSM_CAMSS_V4L2) += msm-camss.o
