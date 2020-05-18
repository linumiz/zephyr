#include <zephyr.h>
#include <sys/printk.h>

void z_tricore_reset(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_internal_protection(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_instruction_error(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_context_mgmt(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_bus_error(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_assertion(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_syscall(int tin)
{
	printk("%s:%d\n", __func__, tin);
}

void z_tricore_nmi(int tin)
{
	printk("%s:%d\n", __func__, tin);
}
