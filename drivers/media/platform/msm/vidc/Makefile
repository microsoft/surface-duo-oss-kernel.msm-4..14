# Makefile for Qualcomm vidc driver

ccflags-y += -Idrivers/media/platform/msm/vidc
vidc-objs += \
		msm_v4l2_vidc.o \
		msm_internal-buffers.o \
		msm_vidc_common.o \
		msm_vdec.o \
		msm_vdec-ctrls.o \
		msm_venc.o \
		msm_venc-ctrls.o \
		msm_smem.o \
		msm_vidc_debug.o \
		msm_vidc_resources.o \
		msm_vidc_load.o \
		hfi/venus/venus_hfi.o \
		hfi/hfi_response_handler.o \
		hfi/hfi_packetization.o \
		hfi/vidc_hfi.o \

obj-$(CONFIG_MSM_VIDC_V4L2) += vidc.o
