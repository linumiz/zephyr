
#include <stdint.h>
#include <zephyr/kernel.h>

#if CONFIG_INFINEON_CAT1C_M0PLUS
extern void cat1c_srom_syscall_isr(void);
const uintptr_t __irq_vector_table _irq_vector_table[8] = {
	((uintptr_t)0x49),         ((uintptr_t)0x281),        ((uintptr_t)cat1c_srom_syscall_isr),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
	((uintptr_t)_isr_wrapper), ((uintptr_t)_isr_wrapper),
};
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
