#include "kernel_internal.h"
#include "zephyr/arch/tricore/arch.h"
#include "zephyr/arch/tricore/arch_inlines.h"
#include "zephyr/arch/tricore/cr.h"
#include "zephyr/linker/linker-defs.h"
#include "zephyr/sys/dlist.h"
#include "zephyr/sys/util.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/arch_interface.h>

#define TRICORE_DPR_L(n) _CONCAT(TRICORE_DPR, _CONCAT(n, _L))
#define TRICORE_DPR_U(n) _CONCAT(TRICORE_DPR, _CONCAT(n, _U))
#define TRICORE_CPR_L(n) _CONCAT(TRICORE_CPR, _CONCAT(n, _L))
#define TRICORE_CPR_U(n) _CONCAT(TRICORE_CPR, _CONCAT(n, _U))
#define TRICORE_DPRE(n)  _CONCAT(TRICORE_DPRE_, n)
#define TRICORE_DPWE(n)  _CONCAT(TRICORE_DPWE_, n)
#define TRICORE_CPXE(n)  _CONCAT(TRICORE_CPXE_, n)

static void _set_dpr(uint8_t region, uintptr_t start_addr, uintptr_t end_addr)
{
#define MPU_DPR_SET(n, ...)                                                                        \
	case n:                                                                                    \
		__asm("mtcr " STRINGIFY(TRICORE_DPR_L(n)) ", %0\n\t" : : "r"(start_addr));         \
		__asm("mtcr " STRINGIFY(TRICORE_DPR_U(n)) ", %0\n\t" : : "r"(end_addr));           \
		break;
	switch (region) {
		LISTIFY(CONFIG_TRICORE_MPU_DATA_REGIONS, MPU_DPR_SET, ( ))
	}
}

static void _set_cpr(uint8_t region, uintptr_t start_addr, uintptr_t end_addr)
{
#define MPU_CPR_SET(n, ...)                                                                        \
	case n:                                                                                    \
		__asm("mtcr " STRINGIFY(TRICORE_CPR_L(n)) ", %0\n\t" : : "r"(start_addr));         \
		__asm("mtcr " STRINGIFY(TRICORE_CPR_U(n)) ", %0\n\t" : : "r"(end_addr));           \
		break;
	switch (region) {
		LISTIFY(CONFIG_TRICORE_MPU_CODE_REGIONS, MPU_CPR_SET, ( ))
	}
}

static void _set_dpre(uint8_t prs, uint32_t re)
{
#define MPU_DPRE_SET(n, ...)                                                                       \
	case n:                                                                                    \
		__asm("mtcr " STRINGIFY(TRICORE_DPRE(n)) ", %0\n\t" : : "r"(re));                  \
		break;
	switch (prs) {
		LISTIFY(CONFIG_TRICORE_MPU_PROTECTION_SETS, MPU_DPRE_SET, ( ))
	}
}

static void _set_dpwe(uint8_t prs, uint32_t we)
{
#define MPU_DPWE_SET(n, ...)                                                                       \
	case n:                                                                                    \
		__asm("mtcr " STRINGIFY(TRICORE_DPWE(n)) ", %0\n\t" : : "r"(we));                  \
		break;
	switch (prs) {
		LISTIFY(CONFIG_TRICORE_MPU_PROTECTION_SETS, MPU_DPWE_SET, ( ))
	}
}

static void _set_cpxe(uint8_t prs, uint32_t xe)
{
#define MPU_CPXE_SET(n, ...)                                                                       \
	case n:                                                                                    \
		__asm("mtcr " STRINGIFY(TRICORE_CPXE(n)) ", %0\n\t" : : "r"(xe));                  \
		break;
	switch (prs) {
		LISTIFY(CONFIG_TRICORE_MPU_PROTECTION_SETS, MPU_CPXE_SET, ( ))
	}
}

/* Range Definitions*/
#define MPU_TEXT_CPR 0
#if CONFIG_MPU_STACK_GUARD
#define MPU_RO_DPR        0
#define MPU_PERI_DPR      1
#define MPU_RW_ISR_DPR    2
#define MPU_RW_DPR        3
#define MPU_RW_KERNEL_DPR 4
#define MPU_STACK_DPR     5
#else
#define MPU_RO_DPR    0
#define MPU_PERI_DPR  1
#define MPU_RW_DPR    2
#define MPU_STACK_DPR 3
#endif

/* Code MPU Settings */
#define MPU_DEFAULT_CPXE (BIT(MPU_TEXT_CPR))

/* ISR MPU Settings */
#if CONFIG_MPU_STACK_GUARD
#define MPU_ISR_DPWE (BIT(MPU_PERI_DPR) | BIT(MPU_RW_ISR_DPR) | BIT(MPU_RW_DPR))
#define MPU_ISR_DPRE (BIT(MPU_RO_DPR) | BIT(MPU_PERI_DPR) | BIT(MPU_RW_ISR_DPR) | BIT(MPU_RW_DPR))
#else
#define MPU_ISR_DPWE (BIT(MPU_PERI_DPR) | BIT(MPU_RW_DPR))
#define MPU_ISR_DPRE (BIT(MPU_RO_DPR) | BIT(MPU_PERI_DPR) | BIT(MPU_RW_DPR))
#endif

/* Kernel MPU Settings */
#if CONFIG_MPU_STACK_GUARD
#define MPU_KERNEL_DPWE (BIT(MPU_PERI_DPR) | BIT(MPU_RW_KERNEL_DPR) | BIT(MPU_STACK_DPR))
#define MPU_KERNEL_DPRE                                                                            \
	(BIT(MPU_RO_DPR) | BIT(MPU_PERI_DPR) | BIT(MPU_RW_KERNEL_DPR) | BIT(MPU_STACK_DPR))
#else
#define MPU_KERNEL_DPWE (BIT(MPU_PERI_DPR) | BIT(MPU_RW_DPR))
#define MPU_KERNEL_DPRE (BIT(MPU_RO_DPR) | BIT(MPU_PERI_DPR) | BIT(MPU_RW_DPR))
#endif

/* Userspace MPU Settings*/
#define MPU_USER_DPWE (BIT(MPU_STACK_DPR))
#define MPU_USER_DPRE (BIT(MPU_RO_DPR) | BIT(MPU_STACK_DPR))

static uint8_t cpr_free = ~((1 << MPU_TEXT_CPR));
static uint32_t dpr_free = ~(GENMASK(MPU_STACK_DPR, 0));
static sys_dlist_t loaded_mem_domains = SYS_DLIST_STATIC_INIT(&loaded_mem_domains);

void z_tricore_mpu_enable(void)
{
	uint32_t corecon = cr_read(TRICORE_CORECON);
	corecon |= (1 << 1); /* Enable MPU */
	cr_write(TRICORE_CORECON, corecon);
}

void z_tricore_mpu_disable(void)
{
	uint32_t corecon = cr_read(TRICORE_CORECON);
	corecon &= ~(1 << 1); /* Disable MPU */
	cr_write(TRICORE_CORECON, corecon);
}

#if CONFIG_MPU_STACK_GUARD
void z_tricore_mpu_stackguard_disable(struct k_thread *thread)
{
	if (thread == NULL) {
		_set_dpr(MPU_RW_ISR_DPR, (uintptr_t)_image_ram_start,
			 (uintptr_t)&z_interrupt_stacks[arch_proc_id()]);
		_set_dpr(MPU_RW_DPR, (uintptr_t)&z_interrupt_stacks[arch_proc_id()],
			 (uintptr_t)_image_ram_end);
	} else {
		_set_dpr(MPU_RW_KERNEL_DPR, (uintptr_t)_image_ram_start,
			 (uintptr_t)thread->stack_info.start);
		_set_dpr(MPU_RW_DPR, (uintptr_t)thread->stack_info.start,
			 (uintptr_t)_image_ram_end);
	}
}

void z_tricore_mpu_stackguard_enable(struct k_thread *thread)
{
	if (thread == NULL) {
		_set_dpr(MPU_RW_ISR_DPR, (uintptr_t)_image_ram_start,
			 (uintptr_t)&z_interrupt_stacks[arch_proc_id()]);
		_set_dpr(MPU_RW_DPR,
			 (uintptr_t)&z_interrupt_stacks[arch_proc_id()] +
				 Z_TRICORE_STACK_GUARD_SIZE,
			 (uintptr_t)_image_ram_end);
	} else {
		_set_dpr(MPU_RW_KERNEL_DPR, (uintptr_t)_image_ram_start,
			 (uintptr_t)thread->stack_info.start);
		_set_dpr(MPU_STACK_DPR,
			 (uintptr_t)thread->stack_info.start + Z_TRICORE_STACK_GUARD_SIZE,
			 (uintptr_t)_image_ram_end);
	}
}
#endif

void z_tricore_mpu_configure_kernel_thread(struct k_thread *thread)
{
	__ASSERT((thread->base.user_options & K_USER) == 0, "Kernel thread expected");

#if CONFIG_MPU_STACK_GUARD
	/* Update stack guard region for kernel threads */
	_set_dpr(MPU_RW_KERNEL_DPR, (uintptr_t)_image_ram_start,
		 (uintptr_t)thread->stack_info.start);
	_set_dpr(MPU_STACK_DPR, (uintptr_t)thread->stack_info.start + Z_TRICORE_STACK_GUARD_SIZE,
		 (uintptr_t)_image_ram_end);
#endif
	/* Set region configuration for the thread prs value */
	_set_dpre(thread->arch.prs, MPU_KERNEL_DPRE);
	_set_dpwe(thread->arch.prs, MPU_KERNEL_DPWE);
	_set_cpxe(thread->arch.prs, MPU_DEFAULT_CPXE);
}

void z_tricore_mpu_configure_user_thread(struct k_thread *thread)
{
	struct k_mem_domain *mem_domain = thread->mem_domain_info.mem_domain;
	uint8_t i;

	__ASSERT((thread->base.user_options & K_USER) != 0, "User thread expected");

	/* Set stack pointer protection range */
	_set_dpr(MPU_STACK_DPR, thread->stack_info.start,
		 thread->stack_info.start + thread->stack_info.size);

	/* Mem domain is already loaded into MPU ranges. Just set the correct values
	 * for the thread PRS */
	if (sys_dnode_is_linked(&mem_domain->arch.loaded_node)) {
		_set_dpre(thread->arch.prs, mem_domain->arch.dpre);
		_set_dpwe(thread->arch.prs, mem_domain->arch.dpwe);
		_set_cpxe(thread->arch.prs, mem_domain->arch.cpxe);
		return;
	}

	/* Set default values for enable ranges */
	mem_domain->arch.dpwe = MPU_USER_DPWE;
	mem_domain->arch.dpre = MPU_USER_DPRE;
	mem_domain->arch.cpxe = MPU_DEFAULT_CPXE;

	for (i = 0; i < mem_domain->num_partitions; i++) {
		struct k_mem_partition *partition = &mem_domain->partitions[i];
		/* Skip empty partitions */
		if (partition->size == 0) {
			continue;
		}

		if (partition->attr.access_rights &
		    (TRICORE_MPU_ACCESS_U_R | TRICORE_MPU_ACCESS_U_W)) {
			/* Fetch a free dprs from the list of loaded */
			if (dpr_free == 0) {
				sys_dnode_t *node = sys_dlist_get(&loaded_mem_domains);
				struct k_mem_domain *empty_domain =
					SYS_DLIST_CONTAINER(node, empty_domain, arch.loaded_node);
				dpr_free = (empty_domain->arch.dpre | empty_domain->arch.dpwe) &
					   ~(GENMASK(MPU_STACK_DPR, 0));
			}
			uint8_t dpr_offset = __builtin_ctz(dpr_free);
			dpr_free &= ~(1 << dpr_offset);

			_set_dpr(dpr_offset, (uintptr_t)partition->start,
				 (uintptr_t)(partition->start + partition->size));

			if (partition->attr.access_rights & TRICORE_MPU_ACCESS_U_W) {
				mem_domain->arch.dpwe |= (1 << dpr_offset);
			}
			if (partition->attr.access_rights & TRICORE_MPU_ACCESS_U_R) {
				mem_domain->arch.dpre |= (1 << dpr_offset);
			}
		}
		if (partition->attr.access_rights & (TRICORE_MPU_ACCESS_U_X)) {
			/* Fetch a free cpr from the list of loaded */
			if (cpr_free == 0) {
				sys_dnode_t *node = sys_dlist_get(&loaded_mem_domains);
				struct k_mem_domain *empty_domain =
					SYS_DLIST_CONTAINER(node, empty_domain, arch.loaded_node);
				cpr_free = empty_domain->arch.cpxe & ~(GENMASK(MPU_TEXT_CPR, 0));
			}
			uint8_t cpr_offset = __builtin_ctz(cpr_free);
			cpr_free &= ~(1 << cpr_offset);

			_set_cpr(cpr_offset, (uintptr_t)partition->start,
				 (uintptr_t)(partition->start + partition->size));

			mem_domain->arch.cpxe |= (1 << cpr_offset);
		}
	}

	sys_dlist_append(&loaded_mem_domains, &mem_domain->arch.loaded_node);
}

void z_tricore_mpu_configure_thread(struct k_thread *thread)
{
#if CONFIG_USERSPACE
	if ((thread->base.user_options & K_USER) != 0) {
		z_tricore_mpu_configure_user_thread(thread);
	} else {
		z_tricore_mpu_configure_kernel_thread(thread);
	}
#else
	z_tricore_mpu_configure_kernel_thread(thread);
#endif
}

void z_tricore_mpu_init(void)
{
	/* Configure text section as code protection range */
	_set_cpr(MPU_TEXT_CPR, (uintptr_t)__text_region_start, (uintptr_t)__text_region_end);

	/* Configure read only data sections as data protection range */
	_set_dpr(MPU_RO_DPR, (uintptr_t)__rodata_region_start, (uintptr_t)__rodata_region_end);

#if CONFIG_MPU_STACK_GUARD
	_set_dpr(MPU_RW_ISR_DPR, (uintptr_t)_image_ram_start,
		 (uintptr_t)&z_interrupt_stacks[arch_proc_id()]);
	_set_dpr(MPU_RW_DPR,
		 (uintptr_t)&z_interrupt_stacks[arch_proc_id()] + Z_TRICORE_STACK_GUARD_SIZE,
		 (uintptr_t)_image_ram_end);
#else
	_set_dpr(MPU_RW_DPR, (uintptr_t)_image_ram_start, (uintptr_t)_image_ram_end);
#endif
	/* Configure access to peripheral space */
	_set_dpr(MPU_PERI_DPR, 0xF0000000, 0xFFFFFFFF);

	/* Set regions for the default PRS value */
	_set_dpre(0, MPU_ISR_DPRE);
	_set_dpwe(0, MPU_ISR_DPWE);
	_set_cpxe(0, MPU_DEFAULT_CPXE);

	z_tricore_mpu_enable();
}

int arch_mem_domain_init(struct k_mem_domain *domain)
{
	domain->arch.cpxe = 0;
	domain->arch.dpre = 0;
	domain->arch.dpwe = 0;
	sys_dnode_init(&domain->arch.loaded_node);

	return 0;
}

int arch_mem_domain_max_partitions_get()
{
	return 32 - MPU_STACK_DPR;
}

int arch_buffer_validate(const void *addr, size_t size, int write)
{
	uint32_t lower_pcxi = _current->callee_saved.pcxi;
	uint32_t upper_pcxi =
		*((uint32_t *)(((lower_pcxi & 0xF0000) << 12) | ((lower_pcxi & 0xFFFF) << 6)));
	uint32_t psw = *(
		(uint32_t *)((((upper_pcxi & 0xF0000) << 12) | ((upper_pcxi & 0xFFFF) << 6)) + 1));

	return 0;
}
