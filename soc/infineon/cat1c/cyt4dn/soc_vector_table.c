/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>

#if CONFIG_INFINEON_CAT1C_M0PLUS /* For M0 */
const uintptr_t __irq_vector_table _irq_vector_table[8] = {
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
};
#else /* For M7 */
const uintptr_t __irq_vector_table _irq_vector_table[16] = {
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
#endif
