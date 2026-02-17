
# Copyright (c) 2023 David Ullmann
# SPDX-License-Identifier: Apache-2.0
board_runner_args(openocd "--target-handle=traveo2.cpu.cm4")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
