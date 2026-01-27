# Copyright (c) 2024 Cypress Semiconductor Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

zephyr_sources_ifdef(CONFIG_SOC_CYT4DNJBZS_M0PLUS soc_m0p.c)

if (CONFIG_SOC_CYT4DNJBZS_M7_0 OR CONFIG_SOC_CYT4DNJBZS_M7_1)
zephyr_sources(soc_m7.c)
endif()

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

# CAT1C family defines
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1 CY_USING_HAL)
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1C COMPONENT_CAT1C)
