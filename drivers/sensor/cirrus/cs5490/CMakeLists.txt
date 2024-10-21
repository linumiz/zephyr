# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 Linumiz

zephyr_library()

zephyr_library_sources(cs5490.c)
zephyr_library_sources_ifdef(CONFIG_CS5490_TRIGGER cs5490_trigger.c)
