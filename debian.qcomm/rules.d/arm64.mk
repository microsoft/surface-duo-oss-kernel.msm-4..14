human_arch	= ARMv8
build_arch	= arm64
header_arch	= arm64
defconfig	= defconfig
flavours	= qcomm
compiler    = gcc
compiler_qcomm = clang
build_image	= Image-dtb
kernel_file	= arch/$(build_arch)/boot/Image-dtb
install_file	= vmlinuz
no_dumpfile	= true
android_kernel_signed = false

loader		= grub

do_linux_tools	= true
do_tools_usbip  = true
do_tools_cpupower = true
do_tools_perf	= true

do_common_headers_indep=false

do_dtbs		= true
disable_d_i = true
do_libc_dev_package=true
do_doc_package	= false
do_source_package= false
do_dkms_wireguard = true
do_zfs = false

skipretpoline	= true
do_use_ext_dtc	= true
