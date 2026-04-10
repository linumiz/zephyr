/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_cat1_intc

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/arch/arm/arch.h>
#include <zephyr/drivers/interrupt_controller/intc_cat1.h>
#include <cy_device.h>
#include <cy_sysint.h>

void cat1_intc_irq_enable(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
		Cy_SysInt_EnableSystemInt(irq);
	}
}

void cat1_intc_irq_disable(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
		Cy_SysInt_DisableSystemInt(irq);
	}
}

int cat1_intc_irq_is_enabled(unsigned int irq)
{
	if (irq <= CPUSS_SYSTEM_INT_NR) {
#if (CY_CPU_CORTEX_M0P)
		return (CPUSS_CM0_SYSTEM_INT_CTL[irq] &
			CPUSS_CM0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
#elif (CY_CPU_CORTEX_M7)
		if (CY_IS_CM7_CORE_0)
		{
			return (CPUSS_CM7_0_SYSTEM_INT_CTL[irq] &
				CPUSS_CM7_0_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
		} else {
			return (CPUSS_CM7_1_SYSTEM_INT_CTL[irq] &
				CPUSS_CM7_1_SYSTEM_INT_CTL_CPU_INT_VALID_Msk) != 0;
		}
#endif
	}
	return 0;
}

void cat1_intc_irq_priority_set(unsigned int irq, unsigned int prio,
				unsigned int flags)
{
	NVIC_SetPriority(NvicMux0_IRQn + prio, MIN(IRQ_PRIO_LOWEST, prio));
	NVIC_EnableIRQ(NvicMux0_IRQn + prio);
	Cy_SysInt_SetInterruptSource(prio, irq);
}

void cat1_intc_irq_eoi(unsigned int irq)
{
	NVIC_ClearPendingIRQ(__get_IPSR() - 16);
}

unsigned int cat1_intc_irq_get_active(void)
{
	IRQn_Type actirqn = ((int32_t)__get_IPSR()) - 16;
	cy_en_intr_t sys_int = Cy_SysInt_GetInterruptActive(actirqn);

	if (sys_int != CY_CPUSS_NOT_CONNECTED_IRQN) {
		return (unsigned int)sys_int + 16;
	}

#if (CY_CPU_CORTEX_M7)
	/* Cortex-M7 has support for 8 software IRQn. These are appended to the sw_irq_table
	 * after the system interrupt sources. */
	if (actirqn >= Internal0_IRQn && actirqn <= Internal7_IRQn) {
		return ((CONFIG_NUM_IRQS - 1) - 8 +
			(actirqn - Internal0_IRQn)) + 16;
	}
#endif
	return (CONFIG_NUM_IRQS - 1) + 16;
}

static int cat1_intc_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

DEVICE_DT_INST_DEFINE(0, cat1_intc_init, NULL, NULL, NULL,
		      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);
