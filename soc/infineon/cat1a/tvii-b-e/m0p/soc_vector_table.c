/*
 * Copyright (c) 2026 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/kernel.h>

extern void cat1a_srom_syscall_isr(void);
#if CONFIG_SOC_DIE_CYT2BL
const uintptr_t __irq_vector_table _irq_vector_table[16] = {
	((uintptr_t)0x49),         ((uintptr_t)0x22d),        ((uintptr_t)cat1a_srom_syscall_isr),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
#elif CONFIG_SOC_DIE_CYT2B7
const uintptr_t __irq_vector_table _irq_vector_table[16] = {
	((uintptr_t)0x49),         ((uintptr_t)0x22d),        ((uintptr_t)cat1a_srom_syscall_isr),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
#else
#error "Unsupported Infineon CAT1A SoC"
#endif
