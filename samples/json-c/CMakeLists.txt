# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(json-c)

#target_sources(app PRIVATE src/json_parse.c)
target_sources(app PRIVATE src/json_print.c)
