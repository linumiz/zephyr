/*
 * Copyright (c) 2024 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_INFINEON_AURIX_TC3XX_SOC_H_
#define ZEPHYR_SOC_INFINEON_AURIX_TC3XX_SOC_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "soc_tagid.h"
#include "soc_prot.h"

#if !defined(_ASMLANGUAGE)
#include "IfxCpu_reg.h"

static inline int aurix_enable_clock(uintptr_t clc, uint32_t timeout)
{
	if ((sys_read32(clc) & 0x2) == 0) {
		return 1;
	}

	sys_write32(sys_read32(clc) & ~BIT(0), clc);
	return WAIT_FOR((sys_read32(clc) & 0x2) == 0, timeout, k_busy_wait(1));
}

static inline int aurix_disable_clock(uintptr_t clc, uint32_t timeout)
{
	sys_write32(sys_read32(clc) | BIT(0), clc);
	return WAIT_FOR((sys_read32(clc) & 0x2) == 0x2, timeout, k_busy_wait(1));
}

static inline int aurix_kernel_reset(mem_addr_t krst0, uint32_t timeout)
{
	sys_write32(1, krst0 + 8);
	sys_write32(1, krst0);
	sys_write32(1, krst0 + 4);
	return WAIT_FOR((sys_read32(krst0) & 0x2) == 0x2, timeout, k_busy_wait(1));
}

static inline void aurix_start_core(uint8_t core_id, uint32_t pc)
{
	Ifx_CPU *cpu = &MODULE_CPU0 + core_id;

	cpu->HRA_PC.U = pc;
	cpu->HRA_BOOTCON.U = 0;
}

#endif
#endif