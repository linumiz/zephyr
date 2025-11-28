# Copyright (c) 2025 Infineon Technologies AG,
# or an affiliate of Infineon Technologies AG.
#
# SPDX-License-Identifier: Apache-2.0

set(hal_dir ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/mtb-hal-t2gc2d6m/release-v1.1.2)
# set(pdl_dir ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/mtb-dsl-pse8xxgp/pdl)

zephyr_include_directories(${hal_dir}/include)
zephyr_library_sources(${hal_dir}/source/mtb_hal_clock.c)

# set(pdl_drv_dir ${pdl_dir}/drivers)
# zephyr_include_directories(${pdl_drv_dir}/include)

# zephyr_library_sources_ifdef(CONFIG_USE_INFINEON_SMIF ${pdl_drv_dir}/source/cy_smif_memnum.c)
# zephyr_code_relocate(FILES ${pdl_drv_dir}/source/cy_smif_memnum.c LOCATION RAM)
