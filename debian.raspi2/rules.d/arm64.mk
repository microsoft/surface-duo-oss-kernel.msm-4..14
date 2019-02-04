human_arch	= ARMv8
build_arch	= arm64
header_arch	= arm64
defconfig	= defconfig
flavours	= raspi2
build_image	= Image
kernel_file	= arch/$(build_arch)/boot/Image
install_file	= vmlinuz
no_dumpfile	= true

loader		= grub

do_linux_tools	= true
do_tools_usbip  = true
do_tools_cpupower = true
do_tools_perf	= true

do_common_headers_indep=false

do_dtbs		= true
disable_d_i = true
do_libc_dev_package=false
do_doc_package	= false
do_source_package= false
