/*
 * Copyright (c) 2023, 2024 Arm Limited (or its affiliates).
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel/thread_stack.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/kernel.h>
#include <ksched.h>
#include "zephyr/cache.h"
#include "zephyr/kernel/thread_stack.h"
#include "zephyr/toolchain/gcc.h"
#include <zephyr/platform/hooks.h>
#include <zephyr/sys/barrier.h>

#include <cy_sysclk.h>
#include <cy_wdt.h>
#include "boot.h"

#define sev()	__asm__ volatile("sev" : : : "memory")
#define wfe()	__asm__ volatile("wfe" : : : "memory")

K_KERNEL_PINNED_STACK_ARRAY_DECLARE(z_interrupt_stacks,
                                    CONFIG_MP_MAX_NUM_CPUS,
                                    CONFIG_ISR_STACK_SIZE);

struct boot_params {
	char *sp;
	arch_cpustart_t fn;
	void *arg;
	int cpu_num;
};

volatile struct boot_params arm_m_cpu_boot_params = {
	.fn = NULL,
};

const uint64_t cpu_node_list[] = {
	DT_FOREACH_CHILD_STATUS_OKAY_SEP(DT_PATH(cpus), DT_REG_ADDR, (,))
};

BUILD_ASSERT(ARRAY_SIZE(cpu_node_list) == DT_CHILD_NUM_STATUS_OKAY(DT_PATH(cpus)));

static void secondary_core_setup(void)
{
	/* disable global interrupt */
	__disable_irq();

	CPUSS->CM7_1_CTL &= ~(0xB);
	__DSB();
	__ISB();

	/* Enable ITCM and DTCM */
	SCB->ITCMCR = SCB->ITCMCR | 0x7; /* Set ITCMCR.EN, .RMW and .RETEN fields */
	SCB->DTCMCR = SCB->DTCMCR | 0x7; /* Set DTCMCR.EN, .RMW and .RETEN fields */

	CPUSS_CM7_1_CTL |= (0x1 << CPUSS_CM7_1_CTL_INIT_TCM_EN_Pos);
	CPUSS_CM7_1_CTL |= (0x2 << CPUSS_CM7_1_CTL_INIT_TCM_EN_Pos);
	CPUSS_CM7_1_CTL |= (0x1 << CPUSS_CM7_1_CTL_INIT_RMW_EN_Pos);
	CPUSS_CM7_1_CTL |= (0x2 << CPUSS_CM7_1_CTL_INIT_RMW_EN_Pos);

	/* ITCMCR EN/RMW/RETEN enabled to access ITCM */
	__UNALIGNED_UINT32_WRITE(((void const *)0xE000EF90), 0x2F);
	/* DTCMCR EN/RMW/RETEN enabled to access DTCM */
	__UNALIGNED_UINT32_WRITE(((void const *)0xE000EF94), 0x2F);

	__DSB();
	__ISB();

	(void)Cy_SysClk_PeriGroupSetSlaveCtl(1, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(2, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(3, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(4, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(5, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(6, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(8, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU); /* Suppress a compiler warning about unused return value */
	(void)Cy_SysClk_PeriGroupSetSlaveCtl(9, CY_SYSCLK_PERI_GROUP_SL_CTL, 0xFFFFU);

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	Cy_SystemInit();
	SystemCoreClockUpdate();

	return;
}

uint32_t z_ifx_core_id()
{
	volatile uint32_t *cpuss_identity = (volatile uint32_t *)0x40200000;
	uint32_t ms_field = (*cpuss_identity >> 8) & 0xF;

	switch (ms_field) {
		case 0xE:
			return 0;  /* M7_0 */
		case 0xD:
			return 1;  /* M7_1 */
		default:
			return -1; /* Error */
    	}
}

/* Called from Zephyr initialization */
void arch_cpu_start(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	int cpu_count;

	/* Now it is on primary core */
	__ASSERT(arch_curr_cpu()->id == 0, "");

	cpu_count = ARRAY_SIZE(cpu_node_list);

	arm_m_cpu_boot_params.sp = K_KERNEL_STACK_BUFFER(stack) + sz;
	arm_m_cpu_boot_params.arg = arg;
	arm_m_cpu_boot_params.cpu_num = cpu_num;
	arm_m_cpu_boot_params.fn = fn;

	barrier_dsync_fence_full();
	sev();

	return;
}

void z_secondary_init(void)
{
	arch_cpustart_t fn;
	void *arg;

	secondary_core_setup();
	wfe();

	while (arm_m_cpu_boot_params.fn == NULL) {
		barrier_dsync_fence_full();
	}

	fn = arm_m_cpu_boot_params.fn;
	arg = arm_m_cpu_boot_params.arg;
	barrier_dsync_fence_full();

	/* Calls smp_init_top which calls z_swap_unlocked (scheduler)*/
	fn(arg);
}

int arch_smp_init(void)
{
	/* TODO: Add support for IPIs and setup them here */
	return 0;
}
