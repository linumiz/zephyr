# Copyright (c) 2025 Infineon Technologies AG,
# or an affiliate of Infineon Technologies AG.
#
# SPDX-License-Identifier: Apache-2.0

# Connect to CM0P core
board_runner_args(openocd "--target-handle=cat1c.cpu.cm0")
board_runner_args(jlink "--device=CYT4BFCCHE" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
