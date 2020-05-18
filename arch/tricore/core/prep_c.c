#include <zephyr/types.h>
#include <toolchain.h>
#include <linker/linker-defs.h>
#include <kernel_structs.h>
#include <kernel_internal.h>

#define CSA_ALIGNMENT	64
int __attribute__((aligned(CSA_ALIGNMENT))) csabase[CONFIG_MP_NUM_CPUS][CONFIG_MAIN_STACK_SIZE / 2 + CSA_ALIGNMENT];

extern FUNC_NORETURN void z_cstart(void);

volatile struct {
	arch_cpustart_t fn;
	void *arg;
} tricore_cpu_init[CONFIG_MP_NUM_CPUS];

#define CPUX_PC_BASE		0xF883FE08
#define CPUX_PC_OFFSET		0x20000
#define CPUX_SYSCON_BASE	0xF883FE14
#define CPUX_SYSCON_OFFSET	0x20000

void arch_start_cpu(int coreid, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	volatile unsigned int *pc;
	volatile unsigned int *halt;

	arc_cpu_init[coreid].fn = fn;
	arc_cpu_init[coreid].arg = arg;

	pc = CPUX_PC_BASE + ((coreid - 1) * CPUX_PC_OFFSET);
	*pc = (unsigned int )__start;

	halt = CPUX_SYSCON_BASE + ((coreid - 1) * CPUX_SYSCON_OFFSET);
	*halt |= BIT(24);
}

void z_tricore_prep_c(int coreid)
{
	int csasize = CONFIG_MAIN_STACK_SIZE / 2;

	IfxCpu_initCSA(csabase[coreid], csasize);

	if (coreid == 0) {
		z_bss_zero();
		z_data_copy();
	}

	if (coreid != 0) {
		arc_cpu_init[coreid].fn(arc_cpu_init[coreid].arg);
	}

	z_cstart();
	CODE_UNREACHABLE;
}
