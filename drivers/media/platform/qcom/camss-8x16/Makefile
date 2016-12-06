# Makefile for Qualcomm CAMSS driver

ccflags-y += -Idrivers/media/platform/qcom/camss
qcom-camss-objs += \
		camss.o \
		csid.o \
		csiphy.o \
		ispif.o \
		vfe.o \
		video.o \

obj-$(CONFIG_VIDEO_QCOM_CAMSS) += qcom-camss.o
