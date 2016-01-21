TOUCH CONTROLLERS SUPPORTED
---------------------------

The DSx driver supports both the DS4 and DS5 families of touch controllers,
including the following.
   S21xx
   S22xx
   S23xx
   S27xx
   S32xx
   S33xx
   S34xx
   S35xx
   S36xx
   S37xx
   S70xx
   S73xx
   S75xx
   S78xx

Touch controllers based on the following communication protocols are supported.
   RMI over I2C
   RMI over SPI
   HID over I2C



DRIVER SOURCE
-------------

The source code of the driver and reference hardware configuration can be found
in the \kernel directory inside the driver tarball. The following are the files
within this directory.

drivers\input\touchscreen\synaptics_dsx\

   synaptics_dsx_core.[ch]
      Source code of core driver containing support for 2D touch and 0D buttons

   synaptics_dsx_i2c.c
      Source code of I2C module used for supporting touch controllers based on
      RMI over I2C protocol

   synaptics_dsx_spi.c
      Source code of SPI module used for supporting touch controllers based on
      RMI over SPI protocol

   synaptics_dsx_rmi_hid_i2c.c
      Source code of HID over I2C (vendor mode) module used for supporting
      touch controllers based on HID over I2C protocol

   synaptics_dsx_rmi_dev.c
      Source code of RMI device module used to provide direct RMI register
      access from user space via character device node or sysfs interface

   synaptics_dsx_fw_update.c
      Source code of firmware update module used to provide reflash
      functionality

   synaptics_dsx_active_pen.c
      Source code of active pen module used to provide active pen functionality

   synaptics_dsx_proximity.c
      Source code of proximity module used to provide proximity functionality

   synaptics_dsx_test_reporting.c
      Source code of test reporting module used for retrieving production test
      reports

   synaptics_dsx_gesture.c
      Source code of gesture module used to provide user defined gesture
      functionality

   synaptics_dsx_video.c
      Source code of video module used for sending display DCS commands via RMI

   Kconfig
      Kconfig for setting up build configuration of DSx driver

   Makefile
      Makefile for building DSx driver

firmware\

   Makefile
      Example Makefile for including firmware image in kernel build for doing
      firmware update during system power-on

include\linux\input\

   synaptics_dsx.h
      Header file shared between hardware configuration and driver

arch\arm\configs\

   panda_defconfig
      Example defconfig for PandaBoard to include DSx driver

   msm8974_defconfig
      Example defconfig for DragonBoard to include DSx driver

arch\arm\mach-omap2\

   board-omap4panda.c
      Example board file PandaBoard to include support Synaptics touch
      controllers

   board-omap4panda_original.c
      Original board file PandaBoard not including support for Synaptics touch
      controllers

arch\arm\boot\dts\

   apq8074-dragonboard.dtsi
      Example Device Tree dtsi file for DragonBoard to include support for
      Synaptics touch controllers

   apq8074-dragonboard_original.dtsi
      Original Device Tree dtsi file for DragonBoard not including support for
      Synaptics touch controllers

   synaptics-dsx-i2c.dtsi
      Example Device Tree dtsi file for describing hardware setup of Synaptics
      touch controllers based on RMI over I2C protocol

   synaptics-dsx-rmi-hid-i2c.dtsi
      Example Device Tree dtsi file for describing hardware setup of Synaptics
      touch controllers based on HID over I2C protocol



DRIVER PORTING
--------------

The procedures listed below describe the general process involved in porting the
driver to a target platform. Depending on the actual kernel BSP used for the
target platform, adjustments in the procedures and additional customization may
be needed.

1) Copy the synaptics_dsx folder in the kernel\driver\input\touchscreen
   directory of the driver tarball to the equivalent directory in the kernel
   source tree of the target platform.

2) Copy synaptics_dsx.h in the kernel\include\linux\input directory of the
   driver tarball to the equivalent directory in the kernel source tree of the
   target platform.

3) Add the line below inside Makefile in the drivers\input\touchscreen
   directory in the kernel source tree of the target platform.
         obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX) += synaptics_dsx/

4) Add the line below inside Kconfig in the drivers\input\touchscreen directory
   in the kernel source tree of the target platform.
         source "drivers/input/touchscreen/synaptics_dsx/Kconfig"

5) Update the defconfig file in the kernel source tree of the target platform by
   referring to the defconfig files in the driver tarball as examples.
   * The following configuration options need to be enabled in the defconfig
     file.
         ONFIG_TOUCHSCREEN_SYNAPTICS_DSX
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_CORE
   * One of the following configuration options needs to be enabled in the
     defconfig file based on the communication protocol of the touch controller
     used on the target platform.
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SPI
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_HID_I2C
   * The other configuration options can be enabled as needed. The ones below
     are recommended.
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE
         CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING

6) If the kernel source tree of the target platform uses the Device Tree model
   for hardware description, update the Device Tree by referring to the dtsi
   files in the kernel\arch\arm\boot\dts directory of the driver tarball as
   examples. The changes made in the dtsi file for the DragonBoard platform to
   include support for Synaptics touch controllers can be found by diffing the
   files below.
         apq8074-dragonboard.dtsi
         apq8074-dragonboard_original.dtsi

7) If the kernel source tree of the target platform uses board files for
   hardware description, update the board file by referring to the board files
   in the kernel\arch\arm\mach-omap2 directory of the driver tarball as
   examples. The changes made in the board file for the PandaBoard platform to
   include support for Synaptics touch controllers can be found by diffing the
   files below.
         board-omap4panda.c
         board-omap4panda_original.c



FIRMWARE UPDATE DURING SYSTEM POWER-ON
--------------------------------------

If the DO_STARTUP_FW_UPDATE macro in the synaptics_dsx_fw_update.c firmware
update module is enabled, the driver uses the kernel's request_firmware()
feature to obtain a default firmware image to do firmware update during system
power-on if necessary. The default firmware image is expected to live in a file
named startup_fw_update.img.ihex in the firmware\synaptics directory in the
kernel source tree during the kernel build.

To convert the .img firmware image provided by Synaptics to the .ihex format,
use the command below.
   objcopy -I binary -O ihex <firmware_name>.img startup_fw_update.img.ihex

To inlcude the firmware image in the kernel build so that it can be used for
doing firmware update during system power-on, place the .ihex file obtained
using the command above in the firmware\synaptics directory in the kernel source
tree and make sure the line below is added inside Makefile in the firmware
directory in the kernel source tree. Note that the line below is commented out
by default in the example Makefile in the driver tarball.
   fw-shipped-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) += synaptics/startup_fw_update.img



VIRTUAL BUTTONS
---------------

The example board file (board-omap4panda.c) and dtsi file (synaptics-dsx.dtsi)
contain support for virtual buttons. In the example board file, the virtual key
map is defined in the vir_button_codes data structure. In the example dtsi file,
the virtual key map is defined with the synaptics,vir-button-codes property.

Detailed information on the virtual key map can be found in the link below.
   http://source.android.com/devices/tech/input/touch-devices.html#virtual-key-map-files

In addition to the virtual key map file, a key layout file also needs to be
created and placed in the system\usr\keylayout directory in the Android file
system. The key layout file to match the virtual key map defined in the example
board file and dtsi file needs to be named synaptics_dsx.kl and contain the
following information.
   key 102 HOME VIRTUAL
   key 158 BACK VIRTUAL

Detailed information on the key layout file can be found in the link below.
   http://source.android.com/devices/tech/input/key-layout-files.html#virtual-soft-keys

When there are virtual buttons, the maximum sensor Y coordinate may extend
beyond the area covered by the display. In this case, the maximum Y value for
the display (i.e. 2D touch) area is specified with the DSX_MAX_Y_FOR_2D macro in
the example board file and with the synaptics,max-y-for-2d property in the
example dtsi file. Note that this maximum Y value for the display area needs to
be specified in sensor resolution units, not in display resolution units (unless
the two match).



MISCELLANEOUS OPTIONS
---------------------

Wakeup Gestures
** To enable wakeup gesture support, set the WAKEUP_GESTURE macro defined in
   synaptics_dsx_core.c to true.

Microbootloader Mode I2C Slave Address
** TDDI solutions implement an on-chip microbootloder due to the use of an
   external flash where the bootloader, UI firmware, and various configuration
   areas reside. The microbootloader may have a unique I2C slave address and is
   used during recovery when the external flash is blank or corrupt. The unique
   I2C slave address of the microbootloader may be defined with the
   DSX_UB_I2C_ADDR macro in the board file or with the synaptics,ub-i2c-addr
   property in the dtsi file.
