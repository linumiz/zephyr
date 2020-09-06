# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(modbus-unit)

# target_sources(app PRIVATE src/unit-test-client-tcp.c)
target_sources(app PRIVATE src/unit-test-server-tcp.c)
# target_sources(app PRIVATE src/unit-test-client-rtu.c)
# target_sources(app PRIVATE src/unit-test-server-rtu.c)
