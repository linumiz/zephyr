# Copyright (c) 2025 Linumiz GmbH 
# SPDX-License-Identifier: Apache-2.0
zephyr_sources_ifdef(CONFIG_PM power.c)
zephyr_sources_ifdef(CONFIG_CORTEX_M_SYSTICK_LPM_TIMER_HOOKS wakeup_timer.c)
