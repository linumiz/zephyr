/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_IFX_CLOCK_CONTROL
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_IFX_CLOCK_CONTROL

#include <stdint.h>

#define IFX_CLK_PLL		0
#define IFX_CLK_HF		1
#define IFX_CLK_LF		2

struct ifx_clk {
	uint8_t clk;
	uint8_t clk_id;
};

struct ifx_clk_peri {
	uint8_t rootclk_id;
	uint8_t divider_type;
	uint8_t divider_inst;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_IFX_CLOCK_CONTROL */
