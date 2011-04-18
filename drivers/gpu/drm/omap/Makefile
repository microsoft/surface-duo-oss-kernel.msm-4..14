#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-y := -Iinclude/drm
omapgpu-y := omap_gpu.o omap_crtc.o omap_encoder.o omap_connector.o omap_fb.o omap_fbdev.o

obj-$(CONFIG_DRM_OMAP)	+= omapgpu.o
