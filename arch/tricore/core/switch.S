/*
 * Copyright (c) 2024 Infineon Technologies AG
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>

GTEXT(z_tricore_switch)
GTEXT(z_thread_mark_switched_in)
	
/* void z_tricore_switch(k_thread_t *switch_to, k_thread_t *switch_from) */
SECTION_FUNC(TEXT, z_tricore_switch)
	/* Save old context */
	mfcr	%d14, 0xFE00
	mfcr	%d15, 0xFE34
	st.d	[%a5]+_thread_offset_to_saved_pcxi, %e14
	mov.aa	%a15, %a4

	/* Load new context */
	ld.d 	%e14, [%a4]+_thread_offset_to_saved_pcxi

#if CONFIG_TRICORE_MPU
	/* Configure MPU for new thread */
	call	z_tricore_mpu_configure_thread
#endif

#if defined(CONFIG_SMP)
	/* In SMP, we might be switching to a thread that has not run yet
	 * and thus has no valid context saved. Context is created on first run
	 * to use the CSA from the current core.
	 */
	jnz	%d14, has_context
	mov.aa	%a4, %a15
	call	z_tricore_create_context
	mov	%d14, %d2
#endif

has_context:
#if CONFIG_INSTRUMENT_THREAD_SWITCHING
	mov.aa	%a4, %a15
	call	z_thread_mark_switched_in
#endif
	/* Set context and prs values for new thread */
	mtcr	0xFE34, %d15
	mtcr	0xFE00, %d14
	isync
	
	fret
