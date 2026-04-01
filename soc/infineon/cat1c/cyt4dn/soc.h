/*
 * Copyright (c) 2026 Linumiz
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon CAT1C SOC.
 */

#ifndef _SOC__H_
#define _SOC__H_

#ifndef _ASMLANGUAGE
#include <cy_device_headers.h>
#include <cy_sysint.h>

#if defined(CONFIG_DYNAMIC_INTERRUPTS)
/* Used to pull values from the device tree array */
#define SYS_INT 0
#define CPU_INT 1

#define ENABLE_SYS_INT(n, isr_handler)                                         \
	enable_sys_int(DT_INST_PROP_BY_IDX(n, system_interrupts, SYS_INT),     \
		       DT_INST_PROP_BY_IDX(n, system_interrupts, CPU_INT),     \
		       (void (*)(const void *))(void *)isr_handler,            \
		       DEVICE_DT_INST_GET(n));

void enable_sys_int(uint32_t sys_int, uint32_t cpu_int, void(*isr)(const void *),
		    const void *arg);
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
