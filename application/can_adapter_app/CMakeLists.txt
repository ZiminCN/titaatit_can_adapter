# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Core/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/User/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/SoftwareVersion)

# Bootloader Class API
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/User/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/Core/inc)
file(GLOB BOOT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/Core/src/flash.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/Core/src/boot_can.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/Core/src/dev_info.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/../../bootloader/can_adapter_bootloader/User/src/boot.cpp)

file(GLOB USER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/User/src/*.c*)
file(GLOB CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Core/src/*.c*)

include(${CMAKE_CURRENT_LIST_DIR}/SoftwareVersion/version.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can_adapter)

target_compile_definitions(app PRIVATE _GNU_SOURCE=1)
target_sources(app PRIVATE ${USER_SOURCES} ${CORE_SOURCES} ${BOOT_SOURCES})
