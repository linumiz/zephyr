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

struct _isr_table_entry sys_int_table[CPUSS_SYSTEM_INT_NR];

void enable_sys_int(uint32_t int_num, uint32_t priority, void (*isr)(const void *), const void *arg)
{
	/* IRQ_PRIO_LOWEST = 6 */
	if (priority <= IRQ_PRIO_LOWEST) {
		Cy_SysInt_SetInterruptSource(priority, int_num);
	} else {
		Cy_SysInt_SetInterruptSource(IRQ_PRIO_LOWEST + 1, int_num);
	}

	if (int_num < CPUSS_SYSTEM_INT_NR) {
		sys_int_table[int_num].arg = arg;
		sys_int_table[int_num].isr = isr;
	} else {
		k_fatal_halt(K_ERR_CPU_EXCEPTION);
	}

	NVIC_ClearPendingIRQ(int_num);
	NVIC_EnableIRQ(priority);
}

void sys_int_handler(uint32_t intrNum)
{
	uint32_t system_int_idx;
	if (_FLD2VAL(CPUSS_CM0_INT0_STATUS_SYSTEM_INT_VALID, CPUSS_CM0_INT_STATUS_BASE[intrNum]))
	{
		system_int_idx = _FLD2VAL(CPUSS_CM0_INT0_STATUS_SYSTEM_INT_IDX, CPUSS_CM0_INT_STATUS_BASE[intrNum]);
		struct _isr_table_entry *entry = &sys_int_table[system_int_idx];
		(entry->isr)(entry->arg);
	}
	else
	{
		// Triggered by software or because of software cleared a peripheral interrupt flag but did not clear the pending flag at NVIC
	}
	NVIC_ClearPendingIRQ((IRQn_Type)intrNum);
}

void system_irq_init(void)
{
	/* Set system interrupt table defaults */
	for (uint32_t index = 0; index < CPUSS_SYSTEM_INT_NR; index++) {
		sys_int_table[index].arg = (const void *)0x0;
		sys_int_table[index].isr = z_irq_spurious;
	}

	/* Connect System Interrupts (IRQ0-IRQ7) to handler */
	/*          irq  priority  handler          arg  flags */
	IRQ_CONNECT(0, 0, sys_int_handler, 0, 0);
	IRQ_CONNECT(1, 0, sys_int_handler, 1, 0);
	IRQ_CONNECT(2, 1, sys_int_handler, 2, 0);
	IRQ_CONNECT(3, 1, sys_int_handler, 3, 0);
	IRQ_CONNECT(4, 2, sys_int_handler, 4, 0);
	IRQ_CONNECT(5, 2, sys_int_handler, 5, 0);
	IRQ_CONNECT(6, 3, sys_int_handler, 6, 0);
	/* Priority 0 is reserved for processor faults.  So, the priority number here
	 * is incremented by 1 in the code associated with IRQ_CONNECT.  Which means that
	 * can not select priority 7, because that gets converted to 8, and doesn't fit
	 * in the 3-bit priority encoding.
	 *
	 * We will use this for additional interrupts that have any priority lower than
	 * the lowest level.
	 */
	IRQ_CONNECT(7, IRQ_PRIO_LOWEST, sys_int_handler, 7, 0);

	/* Enable System Interrupts (IRQ0-IRQ7) */
	irq_enable(0);
	irq_enable(1);
	irq_enable(2);
	irq_enable(3);
	irq_enable(4);
	irq_enable(5);
	irq_enable(6);
	irq_enable(7);
}

void soc_prep_hook(void)
{
#define CM7_0_FLASH_BASE_ADDRESS	0x10080000
#define CM7_1_FLASH_BASE_ADDRESS	0x10080000

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	SystemCoreClockUpdate();

	system_irq_init();

	Cy_SysEnableCM7(CORE_CM7_0, CM7_0_FLASH_BASE_ADDRESS);
	Cy_SysEnableCM7(CORE_CM7_1, CM7_1_FLASH_BASE_ADDRESS);
}
