# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(ais2dw12.c)
zephyr_library_sources_ifdef(CONFIG_AIS2DW12_TRIGGER	ais2dw12_trigger.c)
zephyr_library_include_directories(../stmemsc)
