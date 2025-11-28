# Copyright (c) 2024 Cypress Semiconductor Corporation.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

# Add section for cm0p image ROM
zephyr_linker_sources(ROM_START SORT_KEY 0x0cm0p common/rom_cm0image.ld)

# Add section for cm0p image RAM
zephyr_linker_sources(RAM_SECTIONS SORT_KEY 0 common/ram_cm0image.ld)