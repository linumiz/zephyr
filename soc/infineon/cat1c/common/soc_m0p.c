/*
 * Copyright (c) 2021 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Infineon CAT1C SOC.
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <cy_sysint.h>
#include <cy_wdt.h>
#include <cy_sysclk.h>

#define CY_SYS_CM7_PWR_CTL_KEY_OPEN  (0x05FAUL)
#define CY_SYS_CM7_PWR_CTL_KEY_CLOSE (0xFA05UL)

uint32_t Cy_SysGetCM7Status(uint8_t core)
{
    uint32_t regValue = 0u;

    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        /* Get current power mode */
        regValue = _FLD2VAL(CPUSS_CM7_0_PWR_CTL_PWR_MODE, CPUSS->CM7_0_PWR_CTL);
    }
    else if(core == CORE_CM7_1)
    {
        /* Get current power mode */
        regValue = _FLD2VAL(CPUSS_CM7_1_PWR_CTL_PWR_MODE, CPUSS->CM7_1_PWR_CTL);
    }
    else
    {
        /* */
    }

    return (regValue);
}


void Cy_SysEnableCM7(uint8_t core, uint32_t vectorTableOffset)
{
    uint32_t cmStatus;
    uint32_t interruptState;
    uint32_t regValue;

    CY_ASSERT(core < CORE_MAX);

    interruptState = Cy_SaveIRQ();

    cmStatus = Cy_SysGetCM7Status(core);
    if(cmStatus == CY_SYS_CM7_STATUS_ENABLED)
    {
        // Set core into reset first, so that new settings can get effective
        // This branch is e.g. entered if a debugger is connected that would power-up the CM7,
        // but let it run in ROM boot or pause its execution by keeping CPU_WAIT bit set.
        Cy_SysResetCM7(core);
    }

    // CLK_HF1, by default is disabled for use by CM7_0/1, hence enable
    SRSS->CLK_ROOT_SELECT[1] |= SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    if(core == CORE_CM7_0)
    {
        /* Adjust the vector address */
        CPUSS->CM7_0_VECTOR_TABLE_BASE = vectorTableOffset;

        /* Enable the Power Control Key */
        regValue = CPUSS->CM7_0_PWR_CTL & ~(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_0_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_ENABLED;
        CPUSS->CM7_0_PWR_CTL = regValue;

        while((CPUSS->CM7_0_STATUS & CPUSS_CM7_0_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }

        CPUSS->CM7_0_CTL &= ~(0x1 << CPUSS_CM7_0_CTL_CPU_WAIT_Pos);
    }
    else if(core == CORE_CM7_1)
    {
        /* Adjust the vector address */
        CPUSS->CM7_1_VECTOR_TABLE_BASE = vectorTableOffset;

        /* Enable the Power Control Key */
        regValue = CPUSS->CM7_1_PWR_CTL & ~(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_1_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_ENABLED;
        CPUSS->CM7_1_PWR_CTL = regValue;

        while((CPUSS->CM7_1_STATUS & CPUSS_CM7_1_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }

        CPUSS->CM7_1_CTL &= ~(0x1 << CPUSS_CM7_1_CTL_CPU_WAIT_Pos);
    }

    Cy_RestoreIRQ(interruptState);

}

void Cy_SysDisableCM7(uint8_t core)
{
    uint32_t regValue;

    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        regValue = CPUSS->CM7_0_PWR_CTL & ~(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_0_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_DISABLED;
        CPUSS->CM7_0_PWR_CTL = regValue;

        while((CPUSS->CM7_0_STATUS & CPUSS_CM7_0_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }

    }
    else if(core == CORE_CM7_1)
    {
        regValue = CPUSS->CM7_1_PWR_CTL & ~(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_1_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_DISABLED;
        CPUSS->CM7_1_PWR_CTL = regValue;

        while((CPUSS->CM7_1_STATUS & CPUSS_CM7_0_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }
    }
}

void Cy_SysRetainCM7(uint8_t core)
{
    uint32_t cmStatus;
    uint32_t  interruptState;
    uint32_t regValue;

    interruptState = Cy_SaveIRQ();

    cmStatus = Cy_SysGetCM7Status(core);
    if(cmStatus == CY_SYS_CM7_STATUS_ENABLED)
    {
        if(core == CORE_CM7_0)
        {
            regValue = CPUSS->CM7_0_PWR_CTL & ~(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_0_PWR_CTL_PWR_MODE_Msk);
            regValue |= _VAL2FLD(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
            regValue |= CY_SYS_CM7_STATUS_RETAINED;
            CPUSS->CM7_0_PWR_CTL = regValue;
        }
        else if(core == CORE_CM7_1)
        {
            regValue = CPUSS->CM7_1_PWR_CTL & ~(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_1_PWR_CTL_PWR_MODE_Msk);
            regValue |= _VAL2FLD(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
            regValue |= CY_SYS_CM7_STATUS_RETAINED;
            CPUSS->CM7_1_PWR_CTL = regValue;
        }
    }

    Cy_RestoreIRQ(interruptState);
}

void Cy_SysResetCM7(uint8_t core)
{
    uint32_t regValue;

    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        regValue = CPUSS->CM7_0_PWR_CTL & ~(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_0_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_0_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_RESET;
        CPUSS->CM7_0_PWR_CTL = regValue;

        while((CPUSS->CM7_0_STATUS & CPUSS_CM7_0_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }
    }
    else if(core == CORE_CM7_1)
    {
        regValue = CPUSS->CM7_1_PWR_CTL & ~(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT_Msk | CPUSS_CM7_1_PWR_CTL_PWR_MODE_Msk);
        regValue |= _VAL2FLD(CPUSS_CM7_1_PWR_CTL_VECTKEYSTAT, CY_SYS_CM7_PWR_CTL_KEY_OPEN);
        regValue |= CY_SYS_CM7_STATUS_RESET;
        CPUSS->CM7_1_PWR_CTL = regValue;

        while((CPUSS->CM7_1_STATUS & CPUSS_CM7_1_STATUS_PWR_DONE_Msk) == 0UL)
        {
            /* Wait for the power mode to take effect */
        }
    }
}

void soc_prep_hook(void)
{
	Cy_WDT_Unlock();
	Cy_WDT_Disable();
	SystemCoreClockUpdate();

	Cy_SysEnableCM7(CORE_CM7_0, 0x40200200);
	Cy_SysEnableCM7(CORE_CM7_1, 0x40200600);
}

void enable_sys_int(uint32_t int_num, uint32_t priority, void (*isr)(const void *), const void *arg)
{
	/* Interrupts are not supported on cm0p */
	k_fatal_halt(K_ERR_CPU_EXCEPTION);
}
