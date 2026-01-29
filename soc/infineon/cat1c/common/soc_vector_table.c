/*
 * Copyright (c) 2026 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <zephyr/kernel.h>

#if CONFIG_INFINEON_CAT1C_M0PLUS
extern void cat1c_srom_syscall_isr(void);
#if CONFIG_SOC_DIE_CYT4DN
const uintptr_t __irq_vector_table _irq_vector_table[8] = {
	((uintptr_t)0x49),         ((uintptr_t)0x281),        ((uintptr_t)cat1c_srom_syscall_isr),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
};
#elif CONFIG_SOC_DIE_CYT4BF
const uintptr_t __irq_vector_table _irq_vector_table[8] = {
	((uintptr_t)0x49),         ((uintptr_t)0x2A5),        ((uintptr_t)cat1c_srom_syscall_isr),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
};
#else
#error "Unsupported Infineon CAT1C M0+ SOC"
#endif
#else
const uintptr_t __irq_vector_table _irq_vector_table[16] = {
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper),
};
#endif
