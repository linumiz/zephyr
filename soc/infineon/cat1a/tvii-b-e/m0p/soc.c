/*
 * Copyright (c) 2026 Infineon Technologies AG
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon CAT1A M0+ SoC.
 */

#include "cy_device.h"
#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <cy_sysint.h>
#include <cy_wdt.h>
#include <cy_sysclk.h>

#if (CONFIG_SOC_DIE_CYT2B7)
     #include "tviibe1m_config.h"
#elif (CONFIG_SOC_DIE_CYT2BL)
	 #include "tviibe4m_config.h"
#endif
#define IFX_FAST_CLOCK_DOMAIN_FREQ	160 /* 160 MHz */

void cat1a_srom_syscall_isr(void *arg)
{
	/* Trigger IRQ0 in Software by writing to ISPR register */
	NVIC_SetPendingIRQ(NvicMux0_IRQn);
	/* Read back the register to ensure that the write has happened */
	NVIC->ISPR[0U];
	/* Clear the NVIC Pending bit of IRQ0. This is done as a fallback in case the system call
	was suppressed (e.g., by disabled interrupts) */
	NVIC_ClearPendingIRQ(NvicMux0_IRQn);
	/* Read back the register to ensure that the write has happened */
	NVIC->ICPR[0U];

}

static void cat1a_m0p_srom_init()
{
#if defined(CONFIG_SRAM_VECTOR_TABLE)
	volatile uint32_t *const volatile vectorTable = (uint32_t *)_sram_vector_start;
#else
	volatile const uint32_t *const volatile vectorTable = (uint32_t *)_vector_start;
#endif
	/* The array syntax is necessary to avoid out-of-bounds warnings in some compilers. */
	volatile uint32_t (*const volatile sromTable)[32] = (uint32_t (*)[32])0x00000000;

#if defined(CONFIG_SRAM_VECTOR_TABLE)
	/* Use IRQ0 and IRQ1 handlers from SROM vector table */
	vectorTable[16] = (*sromTable)[16];
	vectorTable[17] = (*sromTable)[17];
#else
	/* If the vector table can't be changed, check the entries. */
	if (vectorTable[16] != (*sromTable)[16] || vectorTable[17] != (*sromTable)[17]) {
		k_fatal_halt(K_ERR_CPU_EXCEPTION);
	}
#endif

	/* Setup SROM API IRQs */
	/* IRQ0 & IRQ1 are used as entry to the SROM APIs.
	 * IRQ2 is used as trampoline into handler mode as
	 * required by the SROM API. Priority are setup as
	 * described in the TRM. */
	NVIC_SetPriority(NvicMux0_IRQn, 0);
	NVIC_SetPriority(NvicMux1_IRQn, 0);
	NVIC_SetPriority(NvicMux2_IRQn, 1);
	NVIC_EnableIRQ(NvicMux0_IRQn);
	NVIC_EnableIRQ(NvicMux1_IRQn);
	NVIC_EnableIRQ(NvicMux2_IRQn);

	/* Move IPC SROM API trigger to IRQ2 for the trampoline function. */
	Cy_PDL_Init(CY_DEVICE_CFG);
	Cy_SysInt_SetInterruptSource(NvicMux2_IRQn, cpuss_interrupts_ipc_0_IRQn);
}

static void EnableEcc(void)
{
	/* Enable ECC checking in SRAM controllers again (had been switched off by assembly startup
	 * code) */
	CPUSS->RAM0_CTL0 &= ~(0x80000); /* set bit 19 to 0 */
#if (CPUSS_RAMC1_PRESENT == 1u)
	CPUSS->RAM1_CTL0 &= ~(0x80000); /* set bit 19 to 0 */
#endif
#if (CPUSS_RAMC2_PRESENT == 1u)
	CPUSS->RAM2_CTL0 &= ~(0x80000); /* set bit 19 to 0 */
#endif
}

void soc_prep_hook(void)
{
	EnableEcc();

	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	//SystemCoreClockUpdate();
	cat1a_m0p_srom_init();
	Cy_SysLib_SetWaitStates(false, IFX_FAST_CLOCK_DOMAIN_FREQ);

}

static int early_init() {
	Cy_PDL_Init(CY_DEVICE_CFG);
	return 0;
}
SYS_INIT(early_init, PRE_KERNEL_1, 0);

static int soc_start_cm4()
{
#if CONFIG_INFINEON_CAT1A_START_M4
	Cy_SysEnableCM4(DT_REG_ADDR(DT_NODELABEL(code_flash0)) +
					    DT_REG_ADDR(DT_NODELABEL(m4_partition)));
#endif
	return 0;
}
SYS_INIT(soc_start_cm4, PRE_KERNEL_2, 0);
