/*
 * Copyright (c) 2025 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "cy_device.h"
#include "zephyr/arch/arm/cortex_m/exception.h"
#include <stdint.h>
#include <zephyr/fatal.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/toolchain.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/arch/arm/irq.h>

#include <cy_sysint.h>

static void enable_cpu_int(uint32_t priority)
{
#if (CONFIG_INFINEON_CAT1A_M0PLUS)
	/* Upper IRQs for priority mapping */
	priority = MAX(priority, IRQ_PRIO_LOWEST);
	NVIC_SetPriority(NvicMux0_IRQn + priority, priority);
#else
	NVIC_SetPriority(NvicMux0_IRQn + priority, MAX(priority, IRQ_PRIO_LOWEST));
#endif
	NVIC_EnableIRQ(NvicMux0_IRQn + priority);
}

void enable_sys_int(uint32_t int_num, uint32_t priority, void (*isr)(const void *), const void *arg)
{
	irq_connect_dynamic(int_num, priority, isr, arg, 0);
	irq_enable(int_num);
	enable_cpu_int(priority);
}

/* Cy_SysInt_Init wrapper for Zephyr IRQ integration */
cy_en_sysint_status_t Cy_SysInt_Init(const cy_stc_sysint_t *config, cy_israddress userIsr)
{
#if CONFIG_DYNAMIC_INTERRUPTS
	irq_connect_dynamic(config->intrSrc, config->intrPriority, (void (*)(const void *))userIsr,
				NULL, 0);
	return CY_SYSINT_SUCCESS;
#else
	/* Interrupts are not supported on this configuration */
	k_fatal_halt(K_ERR_CPU_EXCEPTION);
	return CY_SYSINT_BAD_PARAM;
#endif
}

void Cy_SysInt_SetSystemIrqVector(cy_en_intr_t sysIntSrc, cy_israddress userIsr)
{
#if CONFIG_SRAM_VECTOR_TABLE
	_sw_isr_table[sysIntSrc].isr = userIsr;
#else
	k_fatal_halt(K_ERR_CPU_EXCEPTION);
#endif
}

cy_israddress Cy_SysInt_GetSystemIrqVector(cy_en_intr_t sysIntSrc)
{
	return (cy_israddress)_sw_isr_table[sysIntSrc].isr;
}

/* Custom interrupt controller */
void z_soc_irq_init()
{
	return;
}

void z_soc_irq_enable(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
#if IS_ENABLED(CONFIG_INFINEON_CAT1A_M0PLUS)
		CPUSS_CM0_SYSTEM_INT_CTL[irq] |= CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#else
		CPUSS_CM4_SYSTEM_INT_CTL[irq] |= CPUSS_CM4_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#endif
	}
}

void z_soc_irq_disable(unsigned int irq)
{
#ifdef CONFIG_INFINEON_CAT1A_M0PLUS
	CPUSS_CM0_SYSTEM_INT_CTL[irq] &= ~CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#else
	CPUSS_CM4_SYSTEM_INT_CTL[irq] &= ~CPUSS_CM4_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#endif
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (irq > CPUSS_SYSTEM_INT_NR) {
		return 0;
	}

#ifdef CONFIG_INFINEON_CAT1A_M0PLUS
	return (CPUSS_CM0_SYSTEM_INT_CTL[irq] & CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#else
	return (CPUSS_CM4_SYSTEM_INT_CTL[irq] & CPUSS_CM4_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#endif
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio, unsigned int flags)
{
#if (CONFIG_INFINEON_CAT1A_M0PLUS)
	/* NvicMux0-2 are reserved for SROM API on M0+ (direct ARM vectors in _irq_vector_table).
	 * Routing any system interrupt to NvicMux0-2 via z_soc_irq_priority_set() would bypass
	 * _isr_wrapper and call the SROM handlers (cat1a_srom_syscall_isr / ROM handlers) directly.
	 * Clamp prio to IRQ_PRIO_LOWEST (=3 for M0+ with 2 priority bits) to keep NvicMux0-2
	 * exclusively for SROM use. */
	prio = MAX(prio, IRQ_PRIO_LOWEST);
	CPUSS_CM0_SYSTEM_INT_CTL[irq] =
		_VAL2FLD(CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_IDX, NvicMux0_IRQn + prio) |
		CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#else
	CPUSS_CM4_SYSTEM_INT_CTL[irq] =
		_VAL2FLD(CPUSS_CM4_SYSTEM_INT_CTL_CPU_INT_IDX, NvicMux0_IRQn + prio) |
		CPUSS_CM4_SYSTEM_INT_CTL_CPU_INT_VALID_Msk;
#endif
}

void z_soc_irq_eoi(unsigned int irq)
{
	NVIC_ClearPendingIRQ(__get_IPSR() - 16);
}

unsigned int z_soc_irq_get_active(void)
{
	const volatile uint32 *const int_state =
#if IS_ENABLED(CONFIG_INFINEON_CAT1A_M0PLUS)
		CPUSS_CM0_INT_STATUS;
#else
		CPUSS_CM4_INT_STATUS;
#endif
	IRQn_Type actirqn = ((int32_t)__get_IPSR()) - 16;

	if (actirqn <= NvicMux7_IRQn &&
		(int_state[actirqn] & CPUSS_CM0_INT0_STATUS_SYSTEM_INT_VALID_Msk)) {
		return (int_state[actirqn] & CPUSS_CM0_INT0_STATUS_SYSTEM_INT_IDX_Msk) + 16;
	}

#ifndef CONFIG_INFINEON_CAT1A_M0PLUS
	/* Cortex-M7 has support for 8 software IRQn. These are appended to the sw_irq_table
	 * after the system interrupt sources. */
	if (actirqn >= Internal0_IRQn && actirqn <= Internal7_IRQn) {
		return ((CONFIG_NUM_IRQS - 1) - 8 + (actirqn - Internal0_IRQn)) + 16;
	}
#endif

	return (CONFIG_NUM_IRQS - 1) + 16;
}
