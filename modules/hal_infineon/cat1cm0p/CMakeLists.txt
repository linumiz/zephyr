# Copyright (c) 2022 Cypress Semiconductor Corporation.
#
# SPDX-License-Identifier: Apache-2.0

set(hal_dir               ${ZEPHYR_HAL_INFINEON_MODULE_DIR})
set(hal_cat1cm0p_dir      ${hal_dir}/cat1cm0p)
set(blob_gen_dir          ${ZEPHYR_BINARY_DIR}/include/generated)

if(SOC_FAMILY_INFINEON_CAT1A)
    set(hal_blobs_dir         ${hal_dir}/zephyr/blobs/img/cat1cm0p/COMPONENT_CM0P_SLEEP)
    set(blob_gen_inc_file     ${blob_gen_dir}/psoc6_cm0p_sleep.bin.inc)

    if(CONFIG_SOC_DIE_PSOC6_01)
        set(blob_bin_file ${hal_blobs_dir}/psoc6_01_cm0p_sleep.bin)
    endif()

    if(CONFIG_SOC_DIE_PSOC6_02)
        set(blob_bin_file ${hal_blobs_dir}/psoc6_02_cm0p_sleep.bin)
    endif()

    if(CONFIG_SOC_DIE_PSOC6_03)
        set(blob_bin_file ${hal_blobs_dir}/psoc6_03_cm0p_sleep.bin)
    endif()

    if(CONFIG_SOC_DIE_PSOC6_04)
        set(blob_bin_file ${hal_blobs_dir}/psoc6_04_cm0p_sleep.bin)
    endif()
endif()

if(CONFIG_SOC_SERIES_CYT4DN)
    set(hal_blobs_dir       ${hal_dir}/zephyr/blobs/img/cat1cm0p/COMPONENT_CAT1C/COMPONENT_CM0P_SLEEP)
    set(blob_bin_file       ${hal_blobs_dir}/tviic2d6m_cm0p_sleep.bin)
    set(blob_gen_inc_file   ${blob_gen_dir}/tviic2d6m_cm0p_sleep.bin.inc)
endif()

# Generate Include binaries and add source
if(EXISTS ${blob_bin_file})
    # Generate Include binaries
    generate_inc_file_for_target(app ${blob_bin_file} ${blob_gen_inc_file})

    if(SOC_FAMILY_INFINEON_CAT1A)
        # psoc6_cm0p_sleep.c - template c-array file, which include generated
        # blob include file.
        zephyr_library_sources(${hal_cat1cm0p_dir}/COMPONENT_CM0P_SLEEP/psoc6_cm0p_sleep.c)
    endif()

    if(CONFIG_SOC_SERIES_CYT4DN)
        zephyr_library_sources(${hal_cat1cm0p_dir}/COMPONENT_CAT1C/COMPONENT_CM0P_SLEEP/tviic2d6m_cm0p_sleep.c)
    endif()
else()
    message(WARNING "Can't find CM0+ binaries, please execute: \'west blobs fetch hal_infineon\' ")
endif()
