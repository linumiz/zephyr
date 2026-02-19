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
#include <zephyr/cache.h>
#include <cy_sysclk.h>

#define SYS_INT_IDX_Msk 0X1FF



// /* Dummy symbols, requres for cy_sysint.c module.
//  * NOTE: in this PSOC 6 integration, PSOC 6 Zephyr drivers (uart, spi, gpio)
//  * do not use cy_sysint.c implementation to handle interrupt routine.
//  * Instead this they use IRQ_CONNECT to define ISR.
//  */
 cy_israddress __ramVectors[];
 const cy_israddress __Vectors[];

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
}

void sys_int_handler(uint32_t intrNum)
{
	uint32_t system_int_idx;

	if ((_FLD2VAL(CPUSS_V2_CM4_SYSTEM_INT_CTL_CPU_INT_VALID, CPUSS_CM4_INT_STATUS[intrNum]))) {
		system_int_idx =  CPUSS_CM4_INT_STATUS[intrNum] & SYS_INT_IDX_Msk;
		struct _isr_table_entry *entry = &sys_int_table[system_int_idx];
		(entry->isr)(entry->arg);
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
	IRQ_CONNECT(1, 1, sys_int_handler, 1, 0);
	IRQ_CONNECT(2, 2, sys_int_handler, 2, 0);
	IRQ_CONNECT(3, 3, sys_int_handler, 3, 0);
	IRQ_CONNECT(4, 4, sys_int_handler, 4, 0);
	IRQ_CONNECT(5, 5, sys_int_handler, 5, 0);
	IRQ_CONNECT(6, 6, sys_int_handler, 6, 0);
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

void soc_early_init_hook(void)
{
	sys_cache_instr_enable();
	sys_cache_data_enable();

	system_irq_init();
}

static int init_cycfg_platform_wraper(void)
{

   /*Initializes the system */
	SystemInit();
	return 0;
}


SYS_INIT(init_cycfg_platform_wraper, PRE_KERNEL_1, 0);