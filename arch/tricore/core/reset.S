#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include "vector_table.h"

CORE_ID = 0xfe1c
_ASM_FILE_PROLOGUE

GDATA(_vector_table)
GDATA(z_interrupt_stacks)
GDATA(z_main_stack)

//0xa0000000.
SECTION_FUNC(_reset_section, __start)

	mfcr	%d0, CORE_ID
	and	%d0, %d0, [TRICORE_CORE_ID]

	sh	%d2, %d0, 3
	sh	%d1, %d0, 2
	add	%d2, %d2, %d1

	/* unlock & clear ENDINIT */
	movh.a 	%a2, hi:TRICORE_SCU_WDTCPU0CON0
	lea	%a2, [%a2]lo:TRICORE_SCU_WDTCPU0CON0
	mov.a	%a7, %d2
	add.a	%a7, %a2, %a7
	jl unlock_clear_endinit_wdt

	/* disable safety WDT in CPU0 */
	jnz	%d0, skip_safety_wdt_clear

	movh.a 	%a7, hi:TRICORE_SCU_WDTSCON0
	lea	%a7, [%a7]lo:TRICORE_SCU_WDTSCON0
	jl unlock_clear_endinit_wdt

skip_safety_wdt_clear:
	/* enable PCACHE */
	movh.a 	%a2, hi:TRICORE_PMI_CPU_PCON0
	lea	%a2, [%a2]lo:TRICORE_PMI_CPU_PCON0
	ld.w	%d4, [%a2]0
	andn	%d4, %d4, 2
	st.w	[%a2]0, %d4

	/* enable DCACHE */
	movh.a 	%a2, hi:TRICORE_PMI_CPU_DCON0
	lea	%a2, [%a2]lo:TRICORE_PMI_CPU_DCON0
	ld.w	%d4, [%a2]0
	andn	%d4, %d4, 2
	st.w	[%a2]0, %d4

	/* initialize stack pointer */
	madd	%d0, lo:z_main_stack, %d0, CONFIG_MAIN_STACK_SIZE
	movh.a	%sp, hi:z_main_stack
	lea	%sp, [%sp]lo:%d0

	/* initialize interrupt stack pointer */
	madd	%d0, lo:z_interrupt_stacks, %d0, CONFIG_ISR_STACK_SIZE
	movh	%d0, hi:z_interrupt_stacks
	addi	%d0, %d0, lo:z_interrupt_stacks
	mtcr	$isp, %d0
	isync

	/* initialize vector table */
	movh	%d0, hi:_vector_table
	addi	%d0, %d0, lo:_vector_table
	mtcr	$btv, %d0
	isync

	/* TODO Interrupt table BIV */

	/* disable call depth counter */
	mfcr	%d0, $psw
	or	%d0, %d0, [TRICORE_PSW_CDC_MSK]
	andn	%d0, %d0, [TRICORE_PSW_CDE]
	mtcr	$psw, %d0
	isync

	/* enable system global registers */
	mfcr	%d0, $psw
	or	%d0, %d0, [TRICORE_PSW_GW]
	mtcr	$psw, %d0
	isync

	/* Maybe TODO SDA - small area init */

	/* disable system global registers */
	mfcr	%d0, $psw
	andn	%d0, %d0, [TRICORE_PSW_GW]
	mtcr	$psw, %d0
	isync

	/* lock & set ENDINIT */
	movh.a 	%a2, hi:TRICORE_SCU_WDTCPU0CON0
	lea	%a2, [%a2]lo:TRICORE_SCU_WDTCPU0CON0
	mov.a	%a7, %d2
	add.a	%a7, %a2, %a7
	jl lock_set_endinit_wdt

	/* disable safety WDT in CPU0 */
	jnz	%d0, skip_safety_wdt_set

	movh.a 	%a7, hi:TRICORE_SCU_WDTSCON0
	lea	%a7, [%a7]lo:TRICORE_SCU_WDTSCON0
	jl lock_set_endinit_wdt

skip_safety_wdt_set:
	mfcr	%d4, CORE_ID
	and	%d4, %d4, [TRICORE_CORE_ID]
	jl	z_tricore_prep_c

/* lock & set ENDINIT */
lock_set_endinit_wdt:
	ld.w	%d1, [%a7]0
        or      %d4,%d1,1
	ld.w	%d2, [%a7]4
	or      %d2,%d2,8
	st.w    [%a7]4,%d2
	jz.t	%d1, 1, wdt_skip_lock
        ld.w    %d15,[%a4]0
        andn    %d15,%d15,14
        or      %d15,%d15,241
        st.w    [%a4]0,%d15
        dsync
wdt_skip_lock:
        andn    %d4,%d4,12
        or      %d4,%d4,242
        st.w    [%a4]0,%d4
        ld.w    %d15,[%a4]0
        ji      %a11

/* unlock & clear ENDINIT */
unlock_clear_endinit_wdt:
	ld.w	%d1, [%a7]0
	andn	%d4, %d1, 1
	jz.t	%d1, 1, wdt_skip_unlock
	andn	%d1, %d1, 14
	or	%d1, %d1, 241
	st.w	[%a7]0, %d1
	dsync
wdt_skip_unlock:
	andn	%d4, %d4, 12
	or	%d4, %d4, 242
	st.w	[%a7]0, %d4
	ld.w	%d1, [%a7]0
	ji	%a11
