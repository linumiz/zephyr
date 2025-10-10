/*
 * Copyright (c) 2021 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon CAT1C SOC.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <cy_sysint.h>
#include <cy_wdt.h>
#include <cy_sysclk.h>

void soc_prep_hook(void)
{
#define CM7_0_FLASH_BASE_ADDRESS	0x10080000
#define CM7_1_FLASH_BASE_ADDRESS	0x10358000

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	SystemCoreClockUpdate();

	Cy_SysEnableCM7(CORE_CM7_0, CM7_0_FLASH_BASE_ADDRESS);
	Cy_SysEnableCM7(CORE_CM7_1, CM7_1_FLASH_BASE_ADDRESS);
}

void enable_sys_int(uint32_t int_num, uint32_t priority, void (*isr)(const void *), const void *arg)
{
	/* Interrupts are not supported on cm0p */
	k_fatal_halt(K_ERR_CPU_EXCEPTION);
}
