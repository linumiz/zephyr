/*
 * Copyright (c) 2026 Linumiz
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_INTC_CAT1_H_
#define ZEPHYR_DRIVERS_INTC_CAT1_H_

#include <stdint.h>

void cat1_intc_irq_enable(unsigned int irq);
void cat1_intc_irq_disable(unsigned int irq);
int cat1_intc_irq_is_enabled(unsigned int irq);
void cat1_intc_irq_priority_set(unsigned int irq, unsigned int prio,
				unsigned int flags);
void cat1_intc_irq_eoi(unsigned int irq);
unsigned int cat1_intc_irq_get_active(void);

#endif
