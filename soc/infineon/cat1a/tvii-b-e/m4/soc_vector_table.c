/*
 * Copyright (c) 2026 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/kernel.h>

const uintptr_t __irq_vector_table _irq_vector_table[16] = {
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
