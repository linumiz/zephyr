/*
 * Copyright (c) 2021 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon PSOC 6 SOC.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <cy_sysint.h>

// /* Dummy symbols, requres for cy_sysint.c module.
//  * NOTE: in this PSOC 6 integration, PSOC 6 Zephyr drivers (uart, spi, gpio)
//  * do not use cy_sysint.c implementation to handle interrupt routine.
//  * Instead this they use IRQ_CONNECT to define ISR.
//  */
 cy_israddress __ramVectors[];
 const cy_israddress __Vectors[];

static int init_cycfg_platform_wraper(void)
{

	/* Initializes the system */
	SystemInit();
	return 0;
}

SYS_INIT(init_cycfg_platform_wraper, PRE_KERNEL_1, 0);
