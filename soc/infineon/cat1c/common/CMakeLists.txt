# Copyright (c) 2024 Cypress Semiconductor Corporation.
# Copyright (c) 2025 Infineon Technologies AG
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if (CONFIG_CPU_CORTEX_M0PLUS)
add_subdirectory(m0p)
endif()

if(CONFIG_CPU_CORTEX_M7)
add_subdirectory(m7)
endif()

zephyr_sources(soc_irq.c soc_srom.c soc_vector_table.c)

# Add linker files for ROM vectors, pdl lib section and ram offset
zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors ld/vector_table.ld)
zephyr_linker_sources(NOINIT ld/noinit.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")

# CAT1C family defines
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1 CY_USING_HAL)
zephyr_compile_definitions_ifdef(CONFIG_SOC_FAMILY_INFINEON_CAT1C COMPONENT_CAT1C)
zephyr_compile_definitions_ifdef(CONFIG_INFINEON_CAT1C_M7_0 CORE_NAME_CM7_0)
zephyr_compile_definitions_ifdef(CONFIG_INFINEON_CAT1C_M7_1 CORE_NAME_CM7_1)
