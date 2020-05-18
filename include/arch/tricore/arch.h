#ifndef ZEPHYR_INCLUDE_ARCH_TRICORE_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_TRICORE_ARCH_H_

#include <arch/tricore/regs.h>

#define ARCH_TRICORE_CSA_SIZE	(128 * CONFIG_TRICORE_CALL_DEPTH)

#define ARCH_THREAD_STACK_DEFINE(sym, size) \
		struct z_thread_stack_element __noinit \
		__aligned(POW2_CEIL(size)) \
		sym[POW2_CEIL(size + ARCH_TRICORE_CSA_SIZE)]
#endif
