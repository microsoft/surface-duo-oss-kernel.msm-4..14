#
# Makefile for OpenNSL drivers
#

SDK=OpenNSL/sdk-6.5.10-gpl-modules
OP=ubuntu/opennsl/$(SDK)

ccflags-y += \
	-I$(OP)/include \
	-I$(OP)/systems/bde/linux/include \
	-I$(OP)/systems/bde/shared/include \
	-I$(OP)/systems/linux/kernel/modules/include \
	-DLONGS_ARE_64BITS \
	-DPTRS_ARE_64BITS \
	-DSAL_SPL_LOCK_ON_IRQ \
	-DSYS_BE_PIO=0 -DSYS_BE_PACKET=0 -DSYS_BE_OTHER=0 \
	-DLE_HOST=1 -DSAL_BDE_DMA_MEM_DEFAULT=16

obj-$(CONFIG_OPENNSL_BCM_KNET) += linux-bcm-knet.o

linux-bcm-knet-objs := \
	$(SDK)/systems/linux/kernel/modules/bcm-knet/bcm-knet.o \
	$(SDK)/systems/linux/kernel/modules/shared/gmodule.o \

obj-$(CONFIG_OPENNSL_KERNEL_BDE) += linux-kernel-bde.o

linux-kernel-bde-objs := \
	$(SDK)/systems/bde/linux/kernel/linux_dma.o \
	$(SDK)/systems/bde/linux/kernel/linux-kernel-bde.o \
	$(SDK)/systems/bde/linux/kernel/linux_shbde.o \
	$(SDK)/systems/bde/linux/shared/mpool.o \
	$(SDK)/systems/bde/shared/shbde_iproc.o \
	$(SDK)/systems/bde/shared/shbde_mdio.o \
	$(SDK)/systems/bde/shared/shbde_pci.o \
	$(SDK)/systems/linux/kernel/modules/shared/gmodule.o \

obj-$(CONFIG_OPENNSL_USER_BDE) += linux-user-bde.o

linux-user-bde-objs := \
	$(SDK)/systems/bde/linux/user/kernel/linux-user-bde.o \
	$(SDK)/systems/linux/kernel/modules/shared/ksal.o \
	$(SDK)/systems/linux/kernel/modules/shared/gmodule.o \

