# Copyright (c) 2024 Cypress Semiconductor Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_CPU_CORTEX_M0PLUS soc_m0p.c)
zephyr_sources_ifdef(CONFIG_CPU_CORTEX_M7 soc_m7.c)
zephyr_sources_ifdef(CONFIG_SOC_CYT4DNJBZS_M7_SMP smp.c)

#zephyr_sources_ifdef(CONFIG_BOARD_LATE_INIT_HOOK init_system.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

# CAT1C family defines
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1 CY_USING_HAL)
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1C COMPONENT_CAT1C)
