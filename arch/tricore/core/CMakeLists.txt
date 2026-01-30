# Copyright (c) 2024 Infineon Technologies AG
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
    reset.S
    switch.S
    isr_wrapper.S
    cache.c
    fatal.c
    irq_manage.c
    prep_c.c
    thread.c
    vectors.c
)
zephyr_library_sources_ifdef(CONFIG_TRICORE_MPU mpu.c)
zephyr_library_sources_ifdef(CONFIG_USERSPACE syscall_wrapper.S userspace.S)
zephyr_library_sources_ifdef(CONFIG_CPU_TC16 tc16/lib.S)
zephyr_library_sources_ifdef(CONFIG_CPU_TC18 tc18/lib.S)

zephyr_linker_sources(ROM_START SORT_KEY 0x0reset reset.ld)