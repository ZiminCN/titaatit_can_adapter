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
std::unique_ptr<OTA_UPGRADE_INFO_T> BOOT::ota_upgrade_info = std::make_unique<OTA_UPGRADE_INFO_T>();
std::unique_ptr<RETURN_ACK_T> BOOT::return_ack = std::make_unique<RETURN_ACK_T>();

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

	/*application base address = flash base address + bootloader size + bootloader
	 * arg size*/
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

// refer to "Docs/RM0440.pdf" 1.5(Page: 75/2140) about Product category
// definition

//! refer to "Docs/RM0440.pdf" 3.7.8(Page: 138/2140) about  Flash option
//! register

void BOOT::set_ota_signal_timeout_flag(bool flag)
{
	this->ota_signal_timeout_flag = flag;
}

bool BOOT::get_ota_signal_timeout_flag(void)
{
	return this->ota_signal_timeout_flag;
}

void BOOT::set_deadloop_cnt(int cnt)
{
	this->deadloop_cnt = cnt;
}

int BOOT::get_deadloop_cnt(void)
{
	return this->deadloop_cnt;
}

void BOOT::init(void)
{
	this->ota_signal_timeout_flag = true;
	this->deadloop_cnt = 0;
	this->can_driver->init();
}

static struct can_frame A2R_ota_ack_frame = {
	.id = CANFD_ID_AS_A2R_OTA_ACK,
	.dlc = can_bytes_to_dlc(64),
	.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
};
static struct can_frame A2A_ota_ack_frame = {
	.id = CANFD_ID_AS_A2A_OTA_ACK,
	.dlc = can_bytes_to_dlc(64),
	.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
};

void BOOT::return_adapter2robot_ota_ack(can_frame *frame)
{
	this->can_driver->send_can_msg(can_driver->get_canfd_2_dev(), frame);
}

void BOOT::return_adapter2adapter_ota_ack(can_frame *frame)
{
	this->can_driver->send_can_msg(can_driver->get_canfd_3_dev(), frame);
}

void BOOT::ota_info_verification(const device *dev, can_frame *frame)
{
	OTA_ORDER_E ota_order = static_cast<OTA_ORDER_E>(frame->data[0]);

	switch (ota_order) {
	case OTA_ORDER_AS_UPGRADE_MODE: {
		this->ota_upgrade_info->ota_order = OTA_ORDER_AS_UPGRADE_MODE;
		this->ota_upgrade_info->ota_order_as_upgrade_mode =
			static_cast<OTA_ORDER_AS_UPGRADE_MODE_E>(frame->data[1]);

		// if upgrade mode is one2one, return ack immediately
		if (this->ota_upgrade_info->ota_order_as_upgrade_mode ==
		    OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE) {
			// return myself device id and software version
			// TODO:
			// A2R_ota_ack_frame.data[0] = 0x00U;
			this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
		} else if (this->ota_upgrade_info->ota_order_as_upgrade_mode ==
			   OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO) {
			// send OTA signal to another adapter
		}
		break;
	}
	case OTA_ORDER_AS_FIRMWARE_INFO: {

		break;
	}
	default: {
		break;
	}
	}
}

void BOOT::ota_upgrade_app_firmware_one2one(const device *dev, can_frame *frame)
{
}

void BOOT::ota_upgrade_app_firmware_one2two(const device *dev, can_frame *frame)
{
}

void BOOT::robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data)
{
	BOOT *boot_driver = static_cast<BOOT *>(user_data);

	switch (frame->id) {
	case CANFD_ID_AS_R2A_OTA_SIGNAL: {
		// make sure canfd data is 0xdeadc0de
		uint32_t ota_signal_data = (frame->data[0] << 24) | (frame->data[1] << 16) |
					   (frame->data[2] << 8) | (frame->data[3]);
		if (ota_signal_data != static_cast<uint32_t>(ACK_DEADC0DE)) {
			break;
		}

		boot_driver->set_ota_signal_timeout_flag(false);
		// first ack needs return 0xdeadc0de
		boot_driver->return_ack->return_ack_ota_signal.ota_ack_info =
			static_cast<uint32_t>(ACK_DEADC0DE);
		memcpy(A2R_ota_ack_frame.data, &(boot_driver->return_ack->return_ack_ota_signal),
		       sizeof(boot_driver->return_ack->return_ack_ota_signal));
		boot_driver->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
		break;
	};
	case CANFD_ID_AS_R2A_OTA_UPGRADE: {
		boot_driver->ota_info_verification(dev, frame);
		break;
	};
	case CANFD_ID_AS_R2A_OTA_PACKAGE: {
		// make sure ota process is complete
		if (boot_driver->ota_upgrade_info->ota_order_as_firmware_info_order ==
		    OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_DEFAULT) {
			A2R_ota_ack_frame.data[0] = ACK_ERROR;
			boot_driver->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
			break;
		}

		if (boot_driver->ota_upgrade_info->ota_order_as_upgrade_mode ==
		    OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE) {
			boot_driver->ota_upgrade_app_firmware_one2one(dev, frame);
		} else {
			boot_driver->ota_upgrade_app_firmware_one2two(dev, frame);
		}
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
	// get canfd id: 0x380U ~ 0x385U
	struct can_filter robot2adapter_filter = {
		.id = 0x380,
		.mask = 0x7FA,
		.flags = 0,
	};

	// get canfd id: 0x200U ~ 0x205U
	struct can_filter adapter2adapter_filter = {
		.id = 0x200,
		.mask = 0x7FA,
		.flags = 0,
	};

	this->can_driver->add_can_filter(this->can_driver->get_canfd_2_dev(), &robot2adapter_filter,
					 this->robot2adapter_ota_process, this);
	this->can_driver->add_can_filter(this->can_driver->get_canfd_3_dev(),
					 &adapter2adapter_filter, this->adapter2adapter_ota_process,
					 this);
}
