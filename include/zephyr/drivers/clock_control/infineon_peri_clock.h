/*
 * Copyright (c) 2026 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_INFINEON_PERI_CLOCK_H
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_INFINEON_PERI_CLOCK_H

#include <zephyr/dt-bindings/clock/infineon_clock.h>

struct infineon_sys_clock {
	uint8_t root_clk_id;
	uint8_t divider_type; /* 8/16/16.5/24.5 dividers */
	uint8_t divider_inst;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_CONTROL_INFINEON_PERI_CLOCK_H */
