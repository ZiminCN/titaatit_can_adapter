board_runner_args(jlink "--device=STM32G473RC" "--speed=4000" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)