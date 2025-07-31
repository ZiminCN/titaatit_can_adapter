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

#include <zephyr/cache.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/kernel.h>

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

void BOOT::boot2app(void)
{
	struct arm_vector_table *vt;

	/*application base address = flash base address + bootloader size + bootloader arg
	 * size*/
	vt = (struct arm_vector_table *)(CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_SIZE +
					 CONFIG_CAN_ADAPTER_BOOT_ARG_PARTITION_LOAD_SIZE);

	if ((vt->msp & 0x2FFE0000) == 0x20000000) {
		if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
			sys_clock_disable();
		}

		cleanup_arm_nvic(); /* cleanup NVIC registers */
		__set_MSP(vt->msp);

		__set_CONTROL(0x00); /* application will configures core on its own */
		__ISB();

		((void (*)(void))vt->reset)();
	}
}

void BOOT::boot2boot(void)
{
	struct arm_vector_table *vt;

	/*boot base address = flash base address
	 * size*/
	vt = (struct arm_vector_table *)(CONFIG_FLASH_BASE_ADDRESS);

	if ((vt->msp & 0x2FFE0000) == 0x20000000) {
		if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
			sys_clock_disable();
		}

		cleanup_arm_nvic(); /* cleanup NVIC registers */
		__set_MSP(vt->msp);

		__set_CONTROL(0x00); /* application will configures core on its own */
		__ISB();

		((void (*)(void))vt->reset)();
	}
}

// refer to "Docs/RM0440.pdf" 1.5(Page: 75/2140) about Product category definition

//! refer to "Docs/RM0440.pdf" 3.7.8(Page: 138/2140) about  Flash option register

void BOOT::set_ota_signal_timeout_flag(bool flag)
{
	this->ota_signal_timeout_flag = flag;
}

bool BOOT::get_ota_signal_timeout_flag(void)
{
	return this->ota_signal_timeout_flag;
}

void BOOT::set_deadloop_flag(bool flag)
{
	this->deadloop_flag = flag;
}

bool BOOT::get_deadloop_flag(void)
{
	return this->deadloop_flag;
}

void BOOT::init(void)
{
	this->ota_signal_timeout_flag = true;
	this->deadloop_flag = false;
	this->can_driver->init();
}

void BOOT::ota_process(const device *dev, can_frame *frame)
{
}

void BOOT::robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data)
{
	BOOT *boot_driver = static_cast<BOOT *>(user_data);

	switch (frame->id) {
	case CANFD_ID_AS_OTA_SIGNAL: {
		boot_driver->set_ota_signal_timeout_flag(false);
		break;
	};
	case CANFD_ID_AS_OTA_PACKAGE: {
		boot_driver->ota_process(dev, frame);
		break;
	};
	default: {
		break;
	}
	}
}

void BOOT::adapter2adapter_ota_process(const device *dev, can_frame *frame, void *user_data)
{
}

void BOOT::register_ota_canfd_data_signal()
{
	struct can_filter robot2adapter_filter = {
		.id = 0x382,
		.mask = 0x7FF,
		.flags = 0,
	};

	struct can_filter adapter2adapter_filter = {
		.id = 0x200,
		.mask = 0x7FF,
		.flags = 0,
	};

	this->can_driver->add_can_filter(this->can_driver->get_canfd_2_dev(), &robot2adapter_filter,
					 this->robot2adapter_ota_process, this);
	this->can_driver->add_can_filter(this->can_driver->get_canfd_3_dev(),
					 &adapter2adapter_filter, this->adapter2adapter_ota_process,
					 this);
}
