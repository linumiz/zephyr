# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  thread.c
  trap.c
  switch.S
  prep_c.c
  reset.S
  vector_table.S
  offsets/offsets.c
  )

zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors vector_table.ld)
