// Copyright (c) Direct Drive Technology Co., Ltd. All rights reserved.
// Author: Zi Min <jianming.zeng@directdrivetech.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "boot.hpp"

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/logging/log.h>

#include <cmsis_core.h>

LOG_MODULE_REGISTER(boot, LOG_LEVEL_INF);

std::unique_ptr<BOOT> BOOT::Instance = std::make_unique<BOOT>();

std::unique_ptr<BOOT> BOOT::getInstance()
{
	return std::move(Instance);
}

struct arm_vector_table {
	uint32_t msp;
	uint32_t reset;
};

void BOOT::cleanup_arm_nvic(void)
{
	/* Allow any pending interrupts to be recognized */
	__ISB();
	__disable_irq();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;
	// /* Disable NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
	}
	/* Clear pending NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICPR); i++) {
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
}

void BOOT::boot2app()
{
	struct arm_vector_table *vt;

	/*application base address = bootloader base address + bootloader size + bootloader arg
	 * size*/
	vt = (struct arm_vector_table *)(CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_SIZE +
					 CONFIG_CAN_ADAPTER_BOOT_ARG_PARTITION_LOAD_SIZE);

	LOG_DBG("vt->msp:%p value:0x%08x", (void *)&vt->msp, vt->msp);
	LOG_DBG("vt->reset:%p value:0x%08x", (void *)&vt->reset, vt->reset);

	if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
		sys_clock_disable();
	}

	cleanup_arm_nvic(); /* cleanup NVIC registers */
	__set_MSP(vt->msp);

	__set_CONTROL(0x00); /* application will configures core on its own */
	__ISB();

	((void (*)(void))vt->reset)();
}
