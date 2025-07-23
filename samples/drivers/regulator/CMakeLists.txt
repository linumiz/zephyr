cmake_minimum_required(VERSION 4.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(regulator_mp5416)

target_sources(app PRIVATE src/main.c)
