/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/irq.h>
#include <zephyr/drivers/interrupt_controller/intc_cat1.h>
#include <cy_sysint.h>

#if defined(CONFIG_DYNAMIC_INTERRUPTS)
void enable_sys_int(uint32_t sys_int, uint32_t cpu_int,
		    void (*isr)(const void *), const void *arg)
{
	irq_connect_dynamic(sys_int, cpu_int, isr, arg, 0);
	irq_enable(sys_int);
}
#endif

void z_soc_irq_init(void)
{
	/* Nothing to initialize */
}

void z_soc_irq_enable(unsigned int irq)
{
	cat1_intc_irq_enable(irq);
}

void z_soc_irq_disable(unsigned int irq)
{
	cat1_intc_irq_disable(irq);
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	return cat1_intc_irq_is_enabled(irq);
}

void z_soc_irq_priority_set(unsigned int irq, unsigned int prio,
			     unsigned int flags)
{
	cat1_intc_irq_priority_set(irq, prio, flags);
}

void z_soc_irq_eoi(unsigned int irq)
{
	cat1_intc_irq_eoi(irq);
}

unsigned int z_soc_irq_get_active(void)
{
	return cat1_intc_irq_get_active();
}
