# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE} $ENV{ZEPHYR_MODULES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Core/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/User/inc)
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/hal/stm32/stm32cube/stm32g4xx/drivers/include/)
include_directories(${ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32g4xx/drivers/include/)


file(GLOB USER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/User/src/*.c*)
file(GLOB CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/Core/src/*.c*)
# file(GLOB HAL_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/hal/stm32/stm32cube/stm32g4xx/drivers/src/stm32g4xx_hal_flash_ramfunc.c ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/hal/stm32/stm32cube/stm32g4xx/drivers/src/stm32g4xx_hal_flash.c)
file(GLOB HAL_SOURCES ${ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32g4xx/drivers/src/stm32g4xx_hal_flash_ramfunc.c ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/hal/stm32/stm32cube/stm32g4xx/drivers/src/stm32g4xx_hal_flash.c)

message(STATUS "ZEPHYR_BASE = ${ZEPHYR_BASE}")


project(can_adapter)

target_compile_definitions(app PRIVATE _GNU_SOURCE=1)
target_sources(app PRIVATE ${USER_SOURCES} ${CORE_SOURCES} ${HAL_SOURCES})
