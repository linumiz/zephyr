# Copyright (c) 2025 Linumiz GmbH 
# SPDX-License-Identifier: Apache-2.0

zephyr_compile_definitions_ifdef(CONFIG_SOC_CYT4DNJBZS_M7_0 CORE_NAME_CM7_0)
zephyr_compile_definitions_ifdef(CONFIG_SOC_CYT4DNJBZS_M7_1 CORE_NAME_CM7_1)

zephyr_include_directories(../common)

# Add sections
zephyr_linker_sources(NOINIT ../common/noinit.ld)

zephyr_include_directories(.)
