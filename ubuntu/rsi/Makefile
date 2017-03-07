#/*
# Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 	1. Redistributions of source code must retain the above copyright
# 	   notice, this list of conditions and the following disclaimer.
#
# 	2. Redistributions in binary form must reproduce the above copyright
# 	   notice, this list of conditions and the following disclaimer in the
# 	   documentation and/or other materials provided with the distribution.
#
# 	3. Neither the name of the copyright holder nor the names of its
# 	   contributors may be used to endorse or promote products derived from
# 	   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#*/

ven_rsi_91x-y	+= rsi_91x_main.o
ven_rsi_91x-y	+= rsi_91x_core.o
ven_rsi_91x-y	+= rsi_91x_mac80211.o
ven_rsi_91x-y	+= rsi_91x_mgmt.o
ven_rsi_91x-y	+= rsi_91x_hal.o
ven_rsi_91x-y	+= rsi_91x_ps.o
ven_rsi_91x-$(CONFIG_VEN_RSI_DEBUGFS)	+= rsi_91x_debugfs.o
ven_rsi_91x-$(CONFIG_VEN_RSI_BT_ALONE)	+= rsi_91x_hci.o
ven_rsi_91x-$(CONFIG_VEN_RSI_COEX)	+= rsi_91x_coex.o
ven_rsi_91x-$(CONFIG_VEN_RSI_COEX)	+= rsi_91x_hci.o

ven_rsi_usb-y	+= rsi_91x_usb.o rsi_91x_usb_ops.o
ven_rsi_sdio-y	+= rsi_91x_sdio.o rsi_91x_sdio_ops.o
obj-$(CONFIG_VEN_RSI_91X) 	+= ven_rsi_91x.o
obj-$(CONFIG_VEN_RSI_SDIO)	+= ven_rsi_sdio.o
obj-$(CONFIG_VEN_RSI_USB)	+= ven_rsi_usb.o
