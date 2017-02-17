EXTRA_CFLAGS += -DCONFIG_CARACALLA_BOARD -DCONFIG_VEN_RSI_COEX -DLINUX -Wimplicit -Wstrict-prototypes -DCONFIG_VEN_RSI_DEBUGFS -DPLATFORM_X86 -DCONFIG_RSI_WOW

ven_rsi_91x-y			+= rsi_91x_main.o
ven_rsi_91x-y			+= rsi_91x_core.o
ven_rsi_91x-y			+= rsi_91x_mac80211.o
ven_rsi_91x-y			+= rsi_91x_mgmt.o
ven_rsi_91x-y			+= rsi_91x_hal.o
ven_rsi_91x-y			+= rsi_91x_ps.o
ven_rsi_91x-y	+= rsi_91x_debugfs.o
ven_rsi_91x-$(CONFIG_VEN_RSI_HCI)	+= rsi_91x_hci.o
ven_rsi_91x-y	+= rsi_91x_hci.o rsi_91x_coex.o

ven_rsi_usb-y			+= rsi_91x_usb.o rsi_91x_usb_ops.o
ven_rsi_sdio-y			+= rsi_91x_sdio.o rsi_91x_sdio_ops.o
obj-$(CONFIG_VEN_RSI_91X) 		+= ven_rsi_91x.o
obj-$(CONFIG_VEN_RSI_SDIO)		+= ven_rsi_sdio.o
obj-$(CONFIG_VEN_RSI_USB)		+= ven_rsi_usb.o
