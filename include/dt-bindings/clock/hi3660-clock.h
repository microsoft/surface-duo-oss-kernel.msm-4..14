/*
 * Copyright (c) 2016, Hisilicon Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DT_BINDINGS_CLOCK_HI3660_H
#define __DT_BINDINGS_CLOCK_HI3660_H

/* clk in Hi3660 CRG controller */
#define HI3660_NONE_CLOCK	0

/* fixed rate clocks */
#define HI3660_REF32K		1
#define HI3660_CLK_TCXO		2

/* gate clocks */
#define HI3660_UART5_PCLK	3

#define HI3660_CRG_NR_CLKS	4

#endif /* __DT_BINDINGS_CLOCK_HI3660_H */
