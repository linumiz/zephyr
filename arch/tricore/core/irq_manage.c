/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/interrupt_controller/intc_aurix_ir.h>

void arch_irq_enable(unsigned int irq)
{
	intc_aurix_ir_irq_enable(irq);
}

void arch_irq_disable(unsigned int irq)
{
	intc_aurix_ir_irq_disable(irq);
}

int arch_irq_is_enabled(unsigned int irq)
{
	return intc_aurix_ir_irq_is_enabled(irq);
}

void z_tricore_irq_config(unsigned int irq, unsigned int prio, unsigned int flags)
{
	intc_aurix_ir_irq_config(irq, prio, flags);
}

/*
 * Copyright (c) 2016 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq_multilevel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/pm/pm.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

FUNC_NORETURN void z_irq_spurious(const void *unused)
{
#ifdef CONFIG_EMPTY_IRQ_SPURIOUS
	while (1) {
	}

	CODE_UNREACHABLE;
#else
	unsigned long irq;

#if IS_ENABLED(CONFIG_SOC_SERIES_TC3XX)

#elif IS_ENABLED(CONFIG_SOC_SERIES_TC4XX)

#endif

	LOG_ERR("Spurious interrupt detected! IRQ: %ld", irq);
	//LOG_ERR("PLIC interrupt line causing the IRQ: %d (%p)", save_irq, save_dev);
	//z_riscv_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
	while (1) {
	}
#endif /* CONFIG_EMPTY_IRQ_SPURIOUS */
}

#ifdef CONFIG_DYNAMIC_INTERRUPTS
int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
			     void (*routine)(const void *parameter),
			     const void *parameter, uint32_t flags)
{
	z_isr_install(irq + CONFIG_RISCV_RESERVED_IRQ_ISR_TABLES_OFFSET, routine, parameter);

#if defined(CONFIG_RISCV_HAS_PLIC) || defined(CONFIG_RISCV_HAS_CLIC)
	z_riscv_irq_priority_set(irq, priority, flags);
#else
	ARG_UNUSED(flags);
	ARG_UNUSED(priority);
#endif
	return irq;
}

#ifdef CONFIG_SHARED_INTERRUPTS
int arch_irq_disconnect_dynamic(unsigned int irq, unsigned int priority,
				void (*routine)(const void *parameter), const void *parameter,
				uint32_t flags)
{
	ARG_UNUSED(priority);
	ARG_UNUSED(flags);

	return z_isr_uninstall(irq + CONFIG_RISCV_RESERVED_IRQ_ISR_TABLES_OFFSET, routine,
			       parameter);
}
#endif /* CONFIG_SHARED_INTERRUPTS */
#endif /* CONFIG_DYNAMIC_INTERRUPTS */

#ifdef CONFIG_PM
void arch_isr_direct_pm(void)
{
	unsigned int key;

	key = irq_lock();

	if (_kernel.idle) {
		_kernel.idle = 0;
		pm_system_resume();
	}

	irq_unlock(key);
}
#endif
