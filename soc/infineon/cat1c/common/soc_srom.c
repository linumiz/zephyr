/*
 * Copyright (c) 2025 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/linker/linker-defs.h>

#include <cy_flash_srom.h>
#include <cy_sysint.h>

static void (*gp_srom_resp_handler)(void) = NULL;

/* Cy_Srom_SetResponseHandler wrapper for Zephyr IRQ integration */
void Cy_Srom_SetResponseHandler(cy_srom_handler handler)
{
	gp_srom_resp_handler = handler;
}

static void cat1c_srom_responseip_isr(void *arg)
{
	uint32_t masked = 0;

	IPC_INTR_STRUCT_Type *sromRespIntrStr =
		Cy_IPC_Drv_GetIntrBaseAddr(CY_SROM_DR_IPC_INTR_STRUCT);
	masked = Cy_IPC_Drv_GetInterruptStatusMasked(sromRespIntrStr);

	//    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 10.1','Checked manually. Intentional Non
	//    boolean type is interpreted as boolean.');
	if ((uint32_t)(masked & (uint32_t)(1UL << (uint32_t)CY_IPC_CHAN_SYSCALL)) != 0UL) {
		if (gp_srom_resp_handler != NULL) {
			gp_srom_resp_handler();
		}
	}

	Cy_IPC_Drv_ClearInterrupt(sromRespIntrStr, (masked & 0x0000FFFFUL),
				  (masked & 0xFFFF0000UL) >> 16UL);
}

static int cat1c_srom_init()
{
	/* Initialize SROM response interrupt*/
	IRQ_CONNECT(CY_SROM_DR_IPC_INTR_NO, 2, cat1c_srom_responseip_isr, NULL, 0);
	irq_enable(CY_SROM_DR_IPC_INTR_NO);

	/*  Set IPC interrupt mask    */
	IPC_INTR_STRUCT_Type *sromRespIntrStr =
		Cy_IPC_Drv_GetIntrBaseAddr(CY_SROM_DR_IPC_INTR_STRUCT);

	CY_MISRA_DEVIATE_LINE(
		'MISRA C-2012 Rule 10.1',
		'Checked manually. Intentional Non boolean type is interpreted as boolean.');
	Cy_IPC_Drv_SetInterruptMask(sromRespIntrStr,
				    (uint32_t)(1UL << (uint32_t)CY_IPC_CHAN_SYSCALL), 0UL);

	return 0;
}
SYS_INIT(cat1c_srom_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
