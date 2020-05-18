#include <kernel_structs.h>
#include <offsets_short.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <thread.h>
#include <swap_macros.h>

#define CPU_FCX		0xFE38
#define CPU_LCX		0xFE3C
#define CPU_PC		0xFE08

_ASM_FILE_PROLOGUE
GTEXT(arch_switch)

SECTION_FUNC(TEXT, arch_switch)
/*
	void arch_switch(void *switch_to, void **switched_from)

	- from->csa_head = FCX
	- from->csa_tail = LCX
	- STLCX(from->lower)
	- STUCX(from->upper)
	- from->pc = PC

	- FCX = to->csa_head
	- LCX = to->csa_tail
	- LDLCX(to->lower)
	- LDUCX(to->upper)
	- PC = to->pc
*/
	/* switched_from */
	ld.a %a12, [%a5]
	mfcr %d0, CPU_FCX
	st.w [%a12]_tricore_context_t_csa_head_OFFSET, %d0

	mfcr %d0, CPU_LCX
	st.w [%a12]_tricore_context_t_csa_tail_OFFSET, %d0

	stlcx [%a12]_tricore_context_t_lower_OFFSET
	stucx [%a12]_tricore_context_t_upper_OFFSET

	/* switch_to */
	ld.w %d0, [%a4]_tricore_context_t_csa_head_OFFSET
	mtcr CPU_FCX, %d0

	ld.w %d0, [%a4]_tricore_context_t_csa_tail_OFFSET
	mtcr CPU_LCX, %d0

	ldlcx [%a4]_tricore_context_t_lower_OFFSET
	lducx [%a4]_tricore_context_t_upper_OFFSET

	isync
	ret
