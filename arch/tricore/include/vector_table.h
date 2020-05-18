#ifndef ZEPHYR_ARCH_TRICORE_INCLUDE_VECTOR_TABLE_H_
#define ZEPHYR_ARCH_TRICORE_INCLUDE_VECTOR_TABLE_H_

#ifdef _ASMLANGUAGE

.macro _tricore_trap_ tclass tclass_entry
	.global __trap_ \tclass
	__trap_\tclass:
	svlcx
	mov %d4, %d15
	calla \tclass_entry
	rslcx
	rfe
	.align 5
.endm

#if 0
.macro _tricore_ivt_ tivt tivt_entry
	.global __ivt_ \tivt
	__ivt_\tivt:
	bisr \tivt
	mov %d4, \tivt
	calla \tivt_entry
	rslcx
	rfe
	.align 5
.endm
#endif

#endif
