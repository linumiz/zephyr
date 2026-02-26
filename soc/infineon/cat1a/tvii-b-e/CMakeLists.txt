# Copyright (c) 2026 Infineon Technologies AG
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_INFINEON_CAT1A_M0PLUS)
    add_subdirectory(m0p)
endif()

if(CONFIG_INFINEON_CAT1A_M4)
    add_subdirectory(m4)
endif()

zephyr_include_directories(.)
zephyr_library_sources(soc_irq.c soc_srom_client.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors vector_table.ld)