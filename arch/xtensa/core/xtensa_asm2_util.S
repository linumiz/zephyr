/*
 * Copyright (c) 2017, Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <xtensa_asm2_s.h>
#include <zephyr/offsets.h>
#include <zephyr/zsr.h>

#if defined(CONFIG_SIMULATOR_XTENSA) || defined(XT_SIMULATOR)
#include <xtensa/simcall.h>
#endif

/*
 * xtensa_spill_reg_windows
 *
 * Spill all register windows.  Not a C function, enter this via CALL0
 * (so you have to save off A0, but no other registers need to be
 * spilled).  On return, all registers not part of the current
 * function will be spilled to memory.  The WINDOWSTART SR will have a
 * single 1 bit corresponding to the current frame at WINDOWBASE.
 */
.global xtensa_spill_reg_windows
.align 4
xtensa_spill_reg_windows:
        SPILL_ALL_WINDOWS
        ret

/*
 * xtensa_save_high_regs
 *
 * Call with CALL0, with A2/A3 available as scratch.  Pushes the high
 * A4-A15 GPRs to the stack if needed (i.e. if those registers are not
 * part of wrapped-around frames higher up the call stack), returning
 * to the caller with the stack pointer HAVING BEEN MODIFIED to
 * contain them.
 */
.global xtensa_save_high_regs
.align 4
xtensa_save_high_regs:
	/* Generate a rotated (modulo NREGS/4 bits!) WINDOWSTART in A2
	 * by duplicating the bits twice and shifting down by WINDOWBASE
	 * bits.  Now the LSB is the register quad at WINDOWBASE.
	 */
	rsr a2, WINDOWSTART
	slli a3, a2, (XCHAL_NUM_AREGS / 4)
	or a2, a2, a3
	rsr a3, WINDOWBASE
	ssr a3
	srl a2, a2

	mov a3, a1 /* Stash our original stack pointer */

	/* For the next three bits in WINDOWSTART (which correspond to
	 * the A4-A7, A8-A11 and A12-A15 quads), if we find a one,
	 * that means that the quad is owned by a wrapped-around call
	 * in the registers, so we don't need to spill it or any
	 * further registers from the GPRs and can skip to the end.
	 */
	bbsi a2, 1, _high_gpr_spill_done
	addi a1, a1, -16
	s32i a4, a1, 0
	s32i a5, a1, 4
	s32i a6, a1, 8
	s32i a7, a1, 12

	bbsi a2, 2, _high_gpr_spill_done
	addi a1, a1, -16
	s32i a8, a1, 0
	s32i a9, a1, 4
	s32i a10, a1, 8
	s32i a11, a1, 12

	bbsi a2, 3, _high_gpr_spill_done
	addi a1, a1, -16
	s32i a12, a1, 0
	s32i a13, a1, 4
	s32i a14, a1, 8
	s32i a15, a1, 12

_high_gpr_spill_done:
	/* Push the original stack pointer so we know at restore
	 * time how many registers were spilled, then return, leaving the
	 * modified SP in A1.
	 */
	addi a1, a1, -4
	s32i a3, a1, 0

	ret

/*
 * xtensa_restore_high_regs
 *
 * Does the inverse of xtensa_save_high_regs, taking a stack pointer
 * in A1 that resulted and restoring the A4-A15 state (and the stack
 * pointer) to the state they had at the earlier call.  Call with
 * CALL0, leaving A2/A3 available as scratch.
 */
.global xtensa_restore_high_regs
.align 4
xtensa_restore_high_regs:
	/* pop our "original" stack pointer into a2, stash in a3 also */
	l32i a2, a1, 0
	addi a1, a1, 4
	mov a3, a2

	beq a1, a2, _high_restore_done
	addi a2, a2, -16
	l32i a4, a2, 0
	l32i a5, a2, 4
	l32i a6, a2, 8
	l32i a7, a2, 12

	beq a1, a2, _high_restore_done
	addi a2, a2, -16
	l32i a8, a2, 0
	l32i a9, a2, 4
	l32i a10, a2, 8
	l32i a11, a2, 12

	beq a1, a2, _high_restore_done
	addi a2, a2, -16
	l32i a12, a2, 0
	l32i a13, a2, 4
	l32i a14, a2, 8
	l32i a15, a2, 12

_high_restore_done:
	mov a1, a3 /* Original stack */
	ret

/*
 * _restore_context
 *
 * Arrive here via a jump.  Enters into the restored context and does
 * not return.  A1 should have a context pointer in it as received
 * from switch or an interrupt exit.  Interrupts must be disabled,
 * and register windows should have been spilled.
 *
 * Note that exit from the restore is done with the RFI instruction,
 * using the EPCn/EPSn registers.  Those will have been saved already
 * by any interrupt entry so they are save to use.  Note that EPC1 and
 * RFE are NOT usable (they can't preserve PS).  Per the ISA spec, all
 * RFI levels do the same thing and differ only in the special
 * registers used to hold PC/PS, but Qemu has been observed to behave
 * strangely when RFI doesn't "return" to a INTLEVEL strictly lower
 * than it started from.  So we leverage the zsr.h framework to pick
 * the highest level available for our specific platform.
 */
.global _restore_context
_restore_context:
	call0 xtensa_restore_high_regs

	l32i a0, a1, ___xtensa_irq_bsa_t_pc_OFFSET
	wsr a0, ZSR_EPC

#ifdef CONFIG_USERSPACE
	/* When restoring context via xtensa_switch and
	 * returning from non-nested interrupts, we use
	 * the stashed PS value in the thread struct
	 * instead of the one in the thread stack.
	 * Both scenarios will have nested value of 0.
	 */
	rsr.ZSR_CPU a2
	l32i a0, a2, ___cpu_t_nested_OFFSET
	bnez a0, _restore_ps_from_stack

	l32i a0, a2, ___cpu_t_current_OFFSET
	l32i a0, a0, _thread_offset_to_return_ps
	wsr a0, ZSR_EPS

	j _restore_ps_after

_restore_ps_from_stack:
#endif

	l32i a0, a1, ___xtensa_irq_bsa_t_ps_OFFSET
	wsr a0, ZSR_EPS

#ifdef CONFIG_USERSPACE
_restore_ps_after:
#endif

#if XCHAL_HAVE_FP && defined(CONFIG_CPU_HAS_FPU) && defined(CONFIG_FPU_SHARING)
	FPU_REG_RESTORE
#endif

#if defined(CONFIG_XTENSA_EAGER_HIFI_SHARING)
.extern _xtensa_hifi_load
	call0 _xtensa_hifi_load
#endif

	ODD_REG_RESTORE a0, a1

	rsync

	l32i a0, a1, ___xtensa_irq_bsa_t_a0_OFFSET
	l32i a2, a1, ___xtensa_irq_bsa_t_a2_OFFSET
	l32i a3, a1, ___xtensa_irq_bsa_t_a3_OFFSET
	addi a1, a1, ___xtensa_irq_bsa_t_SIZEOF

	rfi ZSR_RFI_LEVEL

/*
 * void xtensa_arch_except(int reason_p);
 *
 * Implements hardware exception for Xtensa ARCH_EXCEPT to save
 * interrupted stack frame and reason_p for use in exception handler
 * and coredump
 */
.global xtensa_arch_except
.global xtensa_arch_except_epc
.align 4
xtensa_arch_except:
	entry a1, 16
xtensa_arch_except_epc:
	ill
	retw

/*
 * void xtensa_arch_kernel_oops(int reason_p, void *ssf);
 *
 * Simply to raise hardware exception for Kernel OOPS.
 */
.global xtensa_arch_kernel_oops
.global xtensa_arch_kernel_oops_epc
.align 4
xtensa_arch_kernel_oops:
	entry a1, 16
xtensa_arch_kernel_oops_epc:
	ill
	retw

/*
 * void xtensa_switch(void *new, void **old_return);
 *
 * Context switches into the previously-saved "new" handle, placing
 * the saved "old" handle into the address provided by old_return.
 */
.global xtensa_switch
.align 4
xtensa_switch:
#ifdef CONFIG_USERSPACE
	entry a1, 32

	s32i a4, a1, 0
	s32i a5, a1, 4
	s32i a6, a1, 8
	s32i a7, a1, 12

	rsr a6, ZSR_CPU
	l32i a6, a6, ___cpu_t_current_OFFSET
#ifdef CONFIG_XTENSA_MMU
#ifdef CONFIG_XTENSA_MMU_FLUSH_AUTOREFILL_DTLBS_ON_SWAP
	call4 xtensa_swap_update_page_tables
#else
	SWAP_PAGE_TABLE a6, a4, a7
#endif
#endif
#ifdef CONFIG_XTENSA_MPU
	call4 xtensa_mpu_map_write
#endif

	l32i a7, a1, 12
	l32i a6, a1, 8
	l32i a5, a1, 4
	l32i a4, a1, 0
#else
	entry a1, 16
#endif

	SPILL_ALL_WINDOWS
	addi a1, a1, -___xtensa_irq_bsa_t_SIZEOF

	/* Stash our A0/2/3 and the shift/loop registers into the base
	 * save area so they get restored as they are now.  A2/A3
	 * don't actually get used post-restore, but they need to be
	 * stashed across the xtensa_save_high_regs call and this is a
	 * convenient place.
	 */
	s32i a0, a1, ___xtensa_irq_bsa_t_a0_OFFSET
	s32i a2, a1, ___xtensa_irq_bsa_t_a2_OFFSET
	s32i a3, a1, ___xtensa_irq_bsa_t_a3_OFFSET
	ODD_REG_SAVE a0, a1

#if XCHAL_HAVE_FP && defined(CONFIG_CPU_HAS_FPU) && defined(CONFIG_FPU_SHARING)
	FPU_REG_SAVE
#endif

	/* Stash our PS register contents and a "restore" PC. */
	rsr a0, PS
	s32i a0, a1, ___xtensa_irq_bsa_t_ps_OFFSET

#ifdef CONFIG_USERSPACE
	/* Backtrack to the head of thread struct and
	 * then store the PS value to be restored in
	 * the architecture specific section.
	 * This will be used to restore PS instead of
	 * the one stashed inside stack.
	 */
	addi a3, a3, -___thread_t_switch_handle_OFFSET
	s32i a0, a3, _thread_offset_to_return_ps
#endif

	movi a0, _switch_restore_pc
	s32i a0, a1, ___xtensa_irq_bsa_t_pc_OFFSET

#if defined(CONFIG_XTENSA_EAGER_HIFI_SHARING)
	call0 _xtensa_hifi_save
#elif defined(CONFIG_XTENSA_LAZY_HIFI_SHARING)
	/* Disable HiFi sharing */
	rsr a6, CPENABLE
	movi a7, ~(1 << XCHAL_CP_ID_AUDIOENGINELX)
	and a6, a6, a7
	wsr a6, CPENABLE
#endif

	/* Now the high registers */
	call0 xtensa_save_high_regs

#if defined(CONFIG_KERNEL_COHERENCE) && !defined(CONFIG_SCHED_CPU_MASK_PIN_ONLY)
	/* Flush the stack.  The top of stack was stored for us by
	 * arch_cohere_stacks().  It can be NULL for a dummy thread.
	 */
	rsync
	rsr a0, ZSR_FLUSH
	beqz a0, noflush
	mov a3, a1
flushloop:
	dhwb a3, 0
	addi a3, a3, XCHAL_DCACHE_LINESIZE
	blt a3, a0, flushloop
noflush:
#endif

	/* Restore the A3 argument we spilled earlier (via the base
	 * save pointer pushed at the bottom of the stack) and set the
	 * stack to the "new" context out of the A2 spill slot.
	 */
	l32i a2, a1, 0
	l32i a3, a2, ___xtensa_irq_bsa_t_a3_OFFSET
	s32i a1, a3, 0

	/* Switch stack pointer and restore.  The jump to
	 * _restore_context does not return as such, but we arrange
	 * for the restored "next" address to be immediately after for
	 * sanity.
	 */
	 l32i a1, a2, ___xtensa_irq_bsa_t_a2_OFFSET

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
	call4 z_thread_mark_switched_in
#endif
	j _restore_context
_switch_restore_pc:
	retw

/* Define our entry handler to load the struct kernel_t from the
 * MISC0 special register, and to find the nest and irq_stack values
 * at the precomputed offsets.
 */
.align 4
_handle_excint:
	EXCINT_HANDLER ___cpu_t_nested_OFFSET, ___cpu_t_irq_stack_OFFSET

/* Define the actual vectors for the hardware-defined levels with
 * DEF_EXCINT.  These load a C handler address and jump to our handler
 * above.
 */

DEF_EXCINT 1, _handle_excint, xtensa_excint1_c

/* In code below we are using XCHAL_NMILEVEL and XCHAL_NUM_INTLEVELS
 * (whichever is higher), since not all Xtensa configurations support
 * NMI. In such case we will use XCHAL_NUM_INTLEVELS.
 */
#if XCHAL_HAVE_NMI
#define MAX_INTR_LEVEL XCHAL_NMILEVEL
#elif XCHAL_HAVE_INTERRUPTS
#define MAX_INTR_LEVEL XCHAL_NUM_INTLEVELS
#else
#error Xtensa core with no interrupt support is used
#define MAX_INTR_LEVEL 0
#endif

#if MAX_INTR_LEVEL >= 2
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 2))
DEF_EXCINT 2, _handle_excint, xtensa_int2_c
#endif
#endif

#if MAX_INTR_LEVEL >= 3
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 3))
DEF_EXCINT 3, _handle_excint, xtensa_int3_c
#endif
#endif

#if MAX_INTR_LEVEL >= 4
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 4))
DEF_EXCINT 4, _handle_excint, xtensa_int4_c
#endif
#endif

#if MAX_INTR_LEVEL >= 5
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 5))
DEF_EXCINT 5, _handle_excint, xtensa_int5_c
#endif
#endif

#if MAX_INTR_LEVEL >= 6
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 6))
DEF_EXCINT 6, _handle_excint, xtensa_int6_c
#endif
#endif

#if MAX_INTR_LEVEL >= 7
#if !(defined(CONFIG_GDBSTUB) && (XCHAL_DEBUGLEVEL == 7))
DEF_EXCINT 7, _handle_excint, xtensa_int7_c
#endif
#endif

#if defined(CONFIG_GDBSTUB)
DEF_EXCINT XCHAL_DEBUGLEVEL, _handle_excint, xtensa_debugint_c
#endif

/* The user exception vector is defined here, as we need to handle
 * MOVSP exceptions in assembly (the result has to be to unspill the
 * caller function of the code that took the exception, and that can't
 * be done in C).  A prototype exists which mucks with the stack frame
 * from the C handler instead, but that would add a LARGE overhead to
 * some alloca() calls (those whent he caller has been spilled) just
 * to save these five cycles during other exceptions and L1
 * interrupts.  Maybe revisit at some point, with better benchmarking.
 * Note that _xt_alloca_exc is Xtensa-authored code which expects A0
 * to have been saved to EXCSAVE1, we've modified it to use the zsr.h
 * API to get assigned a scratch register.
 */
.pushsection .UserExceptionVector.text, "ax"
.global _Level1RealVector
_Level1RealVector:
	wsr a0, ZSR_A0SAVE
	rsync
	rsr.exccause a0
#ifdef CONFIG_XTENSA_MMU
	beqi a0, EXCCAUSE_ITLB_MISS, _handle_tlb_miss_user
#endif /* CONFIG_XTENSA_MMU */
#ifdef CONFIG_USERSPACE
	beqi a0, EXCCAUSE_SYSCALL, _syscall
#endif /* CONFIG_USERSPACE */
#ifdef CONFIG_XTENSA_MMU
	addi a0, a0, -EXCCAUSE_DTLB_MISS
	beqz a0, _handle_tlb_miss_user
	rsr.exccause a0
#endif /* CONFIG_XTENSA_MMU */
	bnei a0, EXCCAUSE_ALLOCA, _not_alloca

	j _xt_alloca_exc
_not_alloca:
	rsr a0, ZSR_A0SAVE
	j _Level1Vector
#ifdef CONFIG_XTENSA_MMU
_handle_tlb_miss_user:
	/**
	 * Handle TLB miss by loading the PTE page:
	 * The way it works is, when we try to access an address that is not
	 * mapped, we will have a miss. The HW then will try to get the
	 * correspondent memory in the page table. As the page table is not
	 * mapped in memory we will have a second miss, which will trigger
	 * an exception. In the exception (here) what we do is to exploit
	 * this hardware capability just trying to load the page table
	 * (not mapped address), which will cause a miss, but then the hardware
	 * will automatically map it again from the page table. This time
	 * it will work since the page necessary to map the page table itself
	 * are wired map.
	 */
	rsr.ptevaddr a0
	l32i a0, a0, 0
	rsr a0, ZSR_A0SAVE
	rfe
#endif /* CONFIG_XTENSA_MMU */
#ifdef CONFIG_USERSPACE
_syscall:
	rsr a0, ZSR_A0SAVE
	j xtensa_do_syscall
#endif /* CONFIG_USERSPACE */
.popsection

/* In theory you can have levels up to 15, but known hardware only uses 7. */
#if XCHAL_NMILEVEL > 7
#error More interrupts than expected.
#endif

/* We don't actually use "kernel mode" currently.  Populate the vector
 * out of simple caution in case app code clears the UM bit by mistake.
 */
.pushsection .KernelExceptionVector.text, "ax"
.global _KernelExceptionVector
_KernelExceptionVector:
#ifdef CONFIG_XTENSA_MMU
	wsr a0, ZSR_A0SAVE
	rsr.exccause a0
	beqi a0, EXCCAUSE_ITLB_MISS, _handle_tlb_miss_kernel
	addi a0, a0, -EXCCAUSE_DTLB_MISS
	beqz a0, _handle_tlb_miss_kernel
	rsr a0, ZSR_A0SAVE
#endif
	j _Level1Vector
#ifdef CONFIG_XTENSA_MMU
_handle_tlb_miss_kernel:
	/* The TLB miss handling is used only during xtensa_mmu_init()
	 * where vecbase is at a different address, as the offset used
	 * in the jump ('j') instruction will not jump to correct
	 * address (... remember the vecbase is moved).
	 * So we handle TLB misses in a very simple way here until
	 * we move back to using UserExceptionVector above.
	 */
	rsr.ptevaddr a0
	l32i a0, a0, 0
	rsr a0, ZSR_A0SAVE
	rfe
#endif
.popsection

#ifdef XCHAL_DOUBLEEXC_VECTOR_VADDR
.pushsection .DoubleExceptionVector.text, "ax"
.global _DoubleExceptionVector
_DoubleExceptionVector:
#ifdef CONFIG_XTENSA_MMU
	wsr a0, ZSR_DBLEXC
	rsync

	rsr.exccause a0
	addi a0, a0, -EXCCAUSE_DTLB_MISS
	beqz a0, _handle_tlb_miss_dblexc

	/* Need to stash the DEPC for used by the C handler.
	 * If we encounter any DTLB misses when PS.EXCM is set,
	 * this vector will be used and the DEPC register will
	 * have the new address instead of the one resulted in
	 * double exception.
	 */
	rsr.depc a0
	wsr a0, ZSR_DEPC_SAVE

	rsr a0, ZSR_DBLEXC

	j _Level1Vector

_TripleFault:
#endif /* CONFIG_XTENSA_MMU */

#if XCHAL_HAVE_DEBUG && defined(CONFIG_XTENSA_BREAK_ON_UNRECOVERABLE_EXCEPTIONS)
	/* Signals an unhandled double exception, and unrecoverable exceptions.
	 * Definitely needs debugger to be attached to the hardware or simulator
	 * to catch this.
	 */
	break	1, 4
#elif defined(CONFIG_SIMULATOR_XTENSA) || defined(XT_SIMULATOR)
/* Tell simulator to stop executing here, instead of trying to do
 * an infinite loop (see below). Greatly help with using tracing in
 * simulator so that traces will not have infinite iterations of
 * jumps.
 */
	movi a3, 1
	movi a2, SYS_exit
	simcall
#endif
1:
	j	1b

#ifdef CONFIG_XTENSA_MMU
_handle_tlb_miss_dblexc:
	/* Handle all data TLB misses here.
	 * These data TLB misses are mostly caused by preloading
	 * page table entries in the level 1 exception handler.
	 * Failure to load the PTE will result in another exception
	 * with different failure (exccause), which can be handled
	 * when the CPU re-enters the double exception handler.
	 */
	rsr.ptevaddr a0
	l32i a0, a0, 0

	rsr a0, ZSR_DBLEXC
	rfde
#endif
.popsection

#endif
