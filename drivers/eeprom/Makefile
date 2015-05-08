#
# Makefile for eeprom drivers.
#

obj-$(CONFIG_EEPROM)		+= eeprom_core.o
eeprom_core-y			:= core.o

# Devices
obj-$(CONFIG_EEPROM_SUNXI_SID)	+= eeprom-sunxi-sid.o
eeprom-sunxi-sid-y		:= sunxi-sid.o
obj-$(CONFIG_QCOM_QFPROM)	+= eeprom_qfprom.o
eeprom_qfprom-y			:= qfprom.o
