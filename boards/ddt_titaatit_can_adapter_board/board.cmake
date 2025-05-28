# SPDX-License-Identifier: Apache-2.0

# board_runner_args(pyocd "--target=stm32g474retx")
# board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")

# include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
# include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
# include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
# board_runner_args(jlink "--device=STM32G474CB" "--speed=4000" "--reset-after-load" "-i=69611444")
board_runner_args(jlink "--device=STM32G473RC" "--speed=4000" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)