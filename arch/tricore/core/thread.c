/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kernel_arch_data.h"
#include "zephyr/arch/tricore/thread.h"
#include "zephyr/sys/util_macro.h"
#include <zephyr/kernel.h>
#include <ksched.h>
#include <sys/cdefs.h>

union z_tricore_context __kstackmem __aligned(4 * 16) z_tricore_csa[CONFIG_TRICORE_CSA_COUNT];

#ifdef CONFIG_USERSPACE
/*
 * Per-thread (TLS) variable indicating whether execution is in user mode.
 */
//__thread uint8_t is_user_mode;
#endif

unsigned int z_tricore_create_context(struct k_thread *thread, k_thread_entry_t entry, void *p1,
				      void *p2, void *p3, char *stack_ptr)
{
	uint32_t icr = cr_read(TRICORE_ICR);
	__asm volatile("disable" ::: "memory");
	uint32_t fcx = cr_read(TRICORE_FCX);
	z_tricore_lower_context_t *lower =
		(z_tricore_lower_context_t *)(((fcx & 0xF0000) << 12) | ((fcx & 0xFFFF) << 6));
	z_tricore_upper_context_t *upper =
		(z_tricore_upper_context_t *)(((lower->pcxi & 0xF0000) << 12) |
					      ((lower->pcxi & 0xFFFF) << 6));
	cr_write(TRICORE_FCX, upper->pcxi);
	if (icr & 0x8000) {
		__asm volatile("enable" ::: "memory");
	}

	lower->a4 = (uint32_t)entry;
	lower->a5 = (uint32_t)p1;
	lower->a6 = (uint32_t)p2;
	lower->a7 = (uint32_t)p3;
	lower->a11 = (uint32_t)z_thread_entry;
	lower->pcxi |= (1 << 21) | (1 << 20); /* Set PIE and UL bits */

	upper->a10 = (uint32_t)stack_ptr;
	upper->psw = (1 << 7); /* Set CDE bit*/
	upper->pcxi = 0;

	if (thread->base.user_options & K_USER) {
		upper->psw |= (1 << 10); /* User-1 mode */
	} else {
		upper->psw |= (1 << 8) | (2 << 10); /* GW & Privileged mode */
	}

	return fcx;
}

uint32_t z_tricore_create_context_from_mem(struct k_thread *thread, uint32_t *mem)
{
	return z_tricore_create_context(thread, (k_thread_entry_t)mem[0], (void *)mem[1],
					(void *)mem[2], (void *)mem[3], (char *)mem[4]);
}

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack, char *stack_ptr,
		     k_thread_entry_t entry, void *p1, void *p2, void *p3)
{
#if !defined(CONFIG_SMP)
	thread->callee_saved.pcxi = z_tricore_create_context(thread, entry, p1, p2, p3, stack_ptr);
#else
	thread->callee_saved.pcxi = 0;
	thread->arch.arg_mem = stack;
	thread->arch.arg_mem[0] = (uint32_t)entry;
	thread->arch.arg_mem[1] = (uint32_t)p1;
	thread->arch.arg_mem[2] = (uint32_t)p2;
	thread->arch.arg_mem[3] = (uint32_t)p3;
	thread->arch.arg_mem[4] = (uint32_t)stack_ptr;
#endif

	/* Set protection set value, if MPU is enabled, the PRS 0 is reseverd for ISR, SYSCALL */
	thread->arch.prs =
		CONFIG_TRICORE_MPU
			? MAX(1, FIELD_GET(K_PROTECTION_SET_MASK, thread->base.user_options))
			: FIELD_GET(K_PROTECTION_SET_MASK, thread->base.user_options);
#if CONFIG_CPU_TC18
	thread->callee_saved.pprs = thread->arch.prs;
#endif

	/* our switch handle is the thread pointer itself */
	thread->switch_handle = thread;
}

#ifdef CONFIG_USERSPACE
/*
 * User space entry function
 *
 * This function is the entry point to user mode from privileged execution.
 * The conversion is one way, and threads which transition to user mode do
 * not transition back later, unless they are doing system calls.
 */
FUNC_NORETURN void arch_user_mode_enter(k_thread_entry_t user_entry, void *p1, void *p2, void *p3)
{
}

#endif /* CONFIG_USERSPACE */

#ifndef CONFIG_MULTITHREADING

FUNC_NORETURN void z_riscv_switch_to_main_no_multithreading(k_thread_entry_t main_entry, void *p1,
							    void *p2, void *p3)
{
}
#endif /* !CONFIG_MULTITHREADING */
