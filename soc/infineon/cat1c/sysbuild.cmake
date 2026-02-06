# Copyright (c) Infineon Technologies AG
# SPDX-License-Identifier: Apache-2.0

if(SB_CONFIG_M0P_LAUNCHER)
  set(launcher_core "m0p")
  string(REPLACE "/" ";" launcher_quals ${BOARD_QUALIFIERS})
  list(LENGTH launcher_quals launcher_quals_len)
  list(GET launcher_quals 1 launcher_soc)
  list(GET launcher_quals 2 launcher_main)

  if(launcher_quals_len EQUAL 4)
    list(GET launcher_quals 3 launcher_variant)
    set(launcher_main ${launcher_main}-${launcher_variant})
  endif()

  string(CONCAT launcher_board ${BOARD} "/" ${launcher_soc} "/" ${launcher_core})

  set(image "m0p_launcher")

  ExternalZephyrProject_Add(
    APPLICATION ${image}
    SOURCE_DIR ${ZEPHYR_BASE}/samples/basic/minimal
    BOARD ${launcher_board}
  )

  #string(CONCAT launcher_snippet "infineon-" ${launcher_main})

  set_config_bool(${image} CONFIG_INFINEON_CAT1C_START_M7_0 1)
endif()
