# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MSPM0C1104)
    zephyr_compile_definitions(-D__MSPM0C1104__)
endif()
zephyr_sources(soc.c)
zephyr_include_directories(.)
