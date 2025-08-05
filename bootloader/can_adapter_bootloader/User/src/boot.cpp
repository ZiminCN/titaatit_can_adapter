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
#include <zephyr/sys/reboot.h>

std::unique_ptr<BOOT> BOOT::Instance = std::make_unique<BOOT>();
std::unique_ptr<RING_BUF> BOOT::ring_buf_driver = std::make_unique<RING_BUF>();
std::unique_ptr<OTA_UPGRADE_INFO_T> BOOT::ota_upgrade_info = std::make_unique<OTA_UPGRADE_INFO_T>();
std::unique_ptr<RETURN_ACK_T> BOOT::return_ack = std::make_unique<RETURN_ACK_T>();

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
	// struct arm_vector_table *vt;

	// /*boot base address = flash base address
	//  * size*/
	// vt = (struct arm_vector_table *)(CONFIG_FLASH_BASE_ADDRESS);

	// if ((vt->msp & 0x2FFE0000) == 0x20000000) {
	// 	if (IS_ENABLED(CONFIG_SYSTEM_TIMER_HAS_DISABLE_SUPPORT)) {
	// 		sys_clock_disable();
	// 	}

	// 	cleanup_arm_nvic(); /* cleanup NVIC registers */
	// 	__set_MSP(vt->msp);

	// 	__set_CONTROL(0x00); /* application will configures core on its own */
	// 	__ISB();

	// 	((void (*)(void))vt->reset)();
	// }
	sys_reboot(SYS_REBOOT_WARM);
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

void BOOT::factory_arg_self_check(void)
{
	bool ret_bool;

	this->flash_manager_driver->read_factory_arg_data();
	ret_bool = this->flash_manager_driver->check_factory_arg_data_is_void();

	if (likely(ret_bool != false)) {
		// no factory arg data
		this->flash_manager_driver->init_new_factory_arg_data();
		this->flash_manager_driver->write_factory_arg_data();
	} else {
		// check checkpoint is exist
		this->flash_manager_driver->read_factory_arg_data();
		FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();

		// Under normal circumstances, both the boot checkpoint and the app
		// checkpoint should be same. If they are not consistent, it indicates that
		// the APP is lost
		if (temp_factory_arg.boot_checkpoint_flag != temp_factory_arg.app_checkpoint_flag) {
			temp_factory_arg.arg_status = FACTORY_ARG_STATUS_ERROR;
		} else {
			temp_factory_arg.boot_checkpoint_flag = false;
			temp_factory_arg.app_checkpoint_flag = false;
		}

		this->flash_manager_driver->set_factory_arg(temp_factory_arg);
		this->flash_manager_driver->write_factory_arg_data();
	}
}

void BOOT::factory_arg_error(void)
{
	this->flash_manager_driver->read_factory_arg_data();
	FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();
	temp_factory_arg.arg_status = FACTORY_ARG_STATUS_ERROR;
	this->flash_manager_driver->set_factory_arg(temp_factory_arg);
	this->flash_manager_driver->write_factory_arg_data();
}

void BOOT::init(void)
{
	this->ota_signal_timeout_flag = true;
	this->deadloop_cnt = 0;

	this->ota_upgrade_info->ota_package.total_package_cnt = 0;
	this->ota_upgrade_info->ota_package.current_package_index = 0;

	this->ring_buf_driver->ring_buf_init(false, false, WRITE_FLASH_PAGE_SIZE);

	this->can_driver->init();
	this->dev_info_driver->init();
}

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
			HARDWARE_DEVICE_INFO_T dev_info;
			dev_info = this->dev_info_driver->get_hardware_device_uuid();

			this->return_ack->return_ack_ota_upgrade.ota_order =
				OTA_ORDER_AS_UPGRADE_MODE;
			memcpy(this->return_ack->return_ack_ota_upgrade.local_device_id,
			       &(dev_info.uuid),
			       sizeof(this->return_ack->return_ack_ota_upgrade.local_device_id));
			this->return_ack->return_ack_ota_upgrade.ota_ack_info = ACK_OK;
			memcpy(A2R_ota_ack_frame.data, &(this->return_ack->return_ack_ota_upgrade),
			       sizeof(RETURN_ACK_OTA_UPGRADE_T));

			this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
		} else if (this->ota_upgrade_info->ota_order_as_upgrade_mode ==
			   OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO) {
			// send OTA signal to another adapter
			// TODO
		}
		break;
	}
	case OTA_ORDER_AS_FIRMWARE_INFO: {
		this->ota_upgrade_info->ota_order = OTA_ORDER_AS_FIRMWARE_INFO;
		this->ota_upgrade_info->ota_order_as_firmware_info_order =
			static_cast<OTA_ORDER_AS_FIRMWARE_INFO_ORDER_E>(frame->data[1]);
		this->ota_upgrade_info->ota_firmware_info.firmware_size =
			static_cast<uint32_t>((frame->data[2] << 24) | (frame->data[3] << 16) |
					      (frame->data[4] << 8) | (frame->data[5]));
		this->ota_upgrade_info->ota_firmware_info.firmware_version =
			static_cast<uint32_t>((frame->data[6] << 24) | (frame->data[7] << 16) |
					      (frame->data[8] << 8) | (frame->data[9]));
		this->ota_upgrade_info->ota_firmware_info.firmware_build_timestamp =
			static_cast<uint32_t>((frame->data[10] << 24) | (frame->data[11] << 16) |
					      (frame->data[12] << 8) | (frame->data[13]));
		this->ota_upgrade_info->ota_firmware_info.firmware_crc32 =
			static_cast<uint32_t>((frame->data[14] << 24) | (frame->data[15] << 16) |
					      (frame->data[16] << 8) | (frame->data[17]));
		this->ota_upgrade_info->ota_package.total_package_cnt =
			static_cast<uint32_t>((frame->data[18] << 24) | (frame->data[19] << 16) |
					      (frame->data[20] << 8) | (frame->data[21]));

		// if upgrade mode is one2one, return ack immediately
		if (this->ota_upgrade_info->ota_order_as_upgrade_mode ==
		    OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE) {
			if (this->ota_upgrade_info->ota_order_as_firmware_info_order ==
			    OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_UPWARD_UPGRADE) {
				// get current app firmware info
				this->flash_manager_driver->read_factory_arg_data();
				FACTORY_ARG_T temp_factory_arg =
					this->flash_manager_driver->get_factory_arg();

				// make sure the ota app firmware version and timestamp is upper
				// than current firmware
				if (this->ota_upgrade_info->ota_firmware_info.firmware_version <
				    temp_factory_arg.app_version) {

					// return ack error @RETURN_ACK_OTA_FIRMWARE_INFO_T
					this->return_ack->return_ack_ota_firmware_info.ota_order =
						OTA_ORDER_AS_FIRMWARE_INFO;
					this->return_ack->return_ack_ota_firmware_info
						.ota_ack_info = ACK_ERROR;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_version =
						temp_factory_arg.app_version;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_build_timestamp =
						temp_factory_arg.app_build_timestamp;
					memcpy(A2R_ota_ack_frame.data,
					       &(this->return_ack->return_ack_ota_firmware_info),
					       sizeof(RETURN_ACK_OTA_FIRMWARE_INFO_T));
					this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);

					break;
				}

				if (this->ota_upgrade_info->ota_firmware_info
					    .firmware_build_timestamp <
				    temp_factory_arg.app_build_timestamp) {

					// return ack error @RETURN_ACK_OTA_FIRMWARE_INFO_T
					this->return_ack->return_ack_ota_firmware_info.ota_order =
						OTA_ORDER_AS_FIRMWARE_INFO;
					this->return_ack->return_ack_ota_firmware_info
						.ota_ack_info = ACK_ERROR;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_version =
						temp_factory_arg.app_version;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_build_timestamp =
						temp_factory_arg.app_build_timestamp;
					memcpy(A2R_ota_ack_frame.data,
					       &(this->return_ack->return_ack_ota_firmware_info),
					       sizeof(RETURN_ACK_OTA_FIRMWARE_INFO_T));
					this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);

					break;
				}

				// make sure the ota app firmware size is smaller than the storage
				// area
				if (this->ota_upgrade_info->ota_firmware_info.firmware_size <
				    APP_AREA_SIZE) {

					// return ack error @RETURN_ACK_OTA_FIRMWARE_INFO_T
					this->return_ack->return_ack_ota_firmware_info.ota_order =
						OTA_ORDER_AS_FIRMWARE_INFO;
					this->return_ack->return_ack_ota_firmware_info
						.ota_ack_info = ACK_ERROR;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_version =
						temp_factory_arg.app_version;
					this->return_ack->return_ack_ota_firmware_info
						.local_software_build_timestamp =
						temp_factory_arg.app_build_timestamp;
					memcpy(A2R_ota_ack_frame.data,
					       &(this->return_ack->return_ack_ota_firmware_info),
					       sizeof(RETURN_ACK_OTA_FIRMWARE_INFO_T));
					this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);

					break;
				}

				// erase app flash
				this->flash_manager_driver->erase_all_app_flash();

				// return ack ok
				this->return_ack->return_ack_ota_firmware_info.ota_order =
					OTA_ORDER_AS_FIRMWARE_INFO;
				this->return_ack->return_ack_ota_firmware_info.ota_ack_info =
					ACK_OK;
				this->return_ack->return_ack_ota_firmware_info
					.local_software_version = temp_factory_arg.app_version;
				this->return_ack->return_ack_ota_firmware_info
					.local_software_build_timestamp =
					temp_factory_arg.app_build_timestamp;
				memcpy(A2R_ota_ack_frame.data,
				       &(this->return_ack->return_ack_ota_firmware_info),
				       sizeof(RETURN_ACK_OTA_FIRMWARE_INFO_T));
				this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);

				break;
			} else if (this->ota_upgrade_info->ota_order_as_firmware_info_order ==
				   OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_FORCED_UPGRADE) {

				// get current app firmware info
				this->flash_manager_driver->read_factory_arg_data();
				FACTORY_ARG_T temp_factory_arg =
					this->flash_manager_driver->get_factory_arg();

				// erase app flash
				this->flash_manager_driver->erase_all_app_flash();

				// return ack ok
				this->return_ack->return_ack_ota_firmware_info.ota_order =
					OTA_ORDER_AS_FIRMWARE_INFO;
				this->return_ack->return_ack_ota_firmware_info.ota_ack_info =
					ACK_OK;
				this->return_ack->return_ack_ota_firmware_info
					.local_software_version = temp_factory_arg.app_version;
				this->return_ack->return_ack_ota_firmware_info
					.local_software_build_timestamp =
					temp_factory_arg.app_build_timestamp;
				memcpy(A2R_ota_ack_frame.data,
				       &(this->return_ack->return_ack_ota_firmware_info),
				       sizeof(RETURN_ACK_OTA_FIRMWARE_INFO_T));
				this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);

				break;
			}
		} else if (this->ota_upgrade_info->ota_order_as_upgrade_mode ==
			   OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO) {
			// send OTA signal to another adapter
			if (this->ota_upgrade_info->ota_order_as_firmware_info_order ==
			    OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_UPWARD_UPGRADE) {
				// TODO:
				break;
			} else if (this->ota_upgrade_info->ota_order_as_firmware_info_order ==
				   OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_FORCED_UPGRADE) {
				// TODO:
				break;
			}
		}
		break;
	}
	default: {
		break;
	}
	}
}

void BOOT::ota_upgrade_app_firmware_one2one(const device *dev, can_frame *frame)
{
	uint32_t temp_current_package_index = frame->data[1];
	// make sure every package index is increasing...
	if (((temp_current_package_index) -
	     (this->ota_upgrade_info->ota_package.current_package_index)) != 1) {
		// return ack error
		this->return_ack->return_ack_ota_package.total_package_cnt =
			this->ota_upgrade_info->ota_package.total_package_cnt;
		this->return_ack->return_ack_ota_package.current_package_index =
			temp_current_package_index;
		this->return_ack->return_ack_ota_package.ota_ack_info = ACK_ERROR;
		memcpy(A2R_ota_ack_frame.data, &(this->return_ack->return_ack_ota_package),
		       sizeof(RETURN_ACK_OTA_PACKAGE_T));
		this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
	}

	// if ringbuffer is full, read all data and write to flash.
	if (this->ring_buf_driver->get_ring_buf_state() == RING_BUFFER_FULL) {
		uint8_t temp_buf[WRITE_FLASH_PAGE_SIZE];
		this->ring_buf_driver->read_data(temp_buf,
						 static_cast<uint16_t>(WRITE_FLASH_PAGE_SIZE));
		this->flash_manager_driver->write_app_flash_page(temp_buf, WRITE_FLASH_PAGE_SIZE,
								 this->firmware_flash_package_cnt);
		this->firmware_flash_package_cnt += 1;
	}

	// write firmware to ringbuffer.
	this->ring_buf_driver->write_data(frame->data, can_dlc_to_bytes(frame->dlc));

	// return ack ok
	this->return_ack->return_ack_ota_package.total_package_cnt =
		this->ota_upgrade_info->ota_package.total_package_cnt;
	this->return_ack->return_ack_ota_package.current_package_index = temp_current_package_index;
	this->return_ack->return_ack_ota_package.ota_ack_info = ACK_OK;
	memcpy(A2R_ota_ack_frame.data, &(this->return_ack->return_ack_ota_package),
	       sizeof(RETURN_ACK_OTA_PACKAGE_T));
	this->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
}

void BOOT::ota_upgrade_app_firmware_one2two(const device *dev, can_frame *frame)
{
}

void BOOT::robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data)
{
	BOOT *boot_driver = static_cast<BOOT *>(user_data);

	switch (frame->id) {
	case CANFD_ID_AS_R2A_OTA_SIGNAL: {
		boot_driver->set_deadloop_cnt(0);

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
		memcpy(A2R_ota_ack_frame.data, &(boot_driver->return_ack),
		       sizeof(RETURN_ACK_OTA_SIGNAL_T));
		boot_driver->return_adapter2robot_ota_ack(&A2R_ota_ack_frame);
		break;
	};
	case CANFD_ID_AS_R2A_OTA_UPGRADE: {
		boot_driver->set_deadloop_cnt(0);

		boot_driver->ota_info_verification(dev, frame);
		break;
	};
	case CANFD_ID_AS_R2A_OTA_PACKAGE: {
		boot_driver->set_deadloop_cnt(0);

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

void BOOT::set_boot_checkpoint_flag_active()
{
	this->flash_manager_driver->read_factory_arg_data();
	FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();
	temp_factory_arg.boot_checkpoint_flag = true;
	this->flash_manager_driver->set_factory_arg(temp_factory_arg);
	this->flash_manager_driver->write_factory_arg_data();
}

void BOOT::set_app_checkpoint_flag_active()
{
	this->flash_manager_driver->read_factory_arg_data();
	FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();
	temp_factory_arg.app_checkpoint_flag = true;
	this->flash_manager_driver->set_factory_arg(temp_factory_arg);
	this->flash_manager_driver->write_factory_arg_data();
}

uint8_t BOOT::get_boot_upgrade_flag()
{
	this->flash_manager_driver->read_factory_arg_data();
	FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();
	return temp_factory_arg.is_boot_update_flag;
}

void BOOT::set_app_upgrade_flag_active()
{
	this->flash_manager_driver->read_factory_arg_data();
	FACTORY_ARG_T temp_factory_arg = this->flash_manager_driver->get_factory_arg();
	temp_factory_arg.is_app_update_flag = true;
	this->flash_manager_driver->set_factory_arg(temp_factory_arg);
	this->flash_manager_driver->write_factory_arg_data();
}

uint32_t BOOT::test_return_app_crc32(uint8_t *data, uint32_t app_size)
{
	return this->flash_manager_driver->calculate_app_firmware_crc32(data, app_size);
}
