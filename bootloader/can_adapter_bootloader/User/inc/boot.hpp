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

#ifndef __BOOT_HPP__
#define __BOOT_HPP__

#include "can.hpp"
#include <memory>

/**
 * @brief Bootloader OTA Process
 *
 * 1. Bootloader/Application will wait for  OTA signal to enter OTA process.
 * OTA signal: canfd => ID: 0x382, Data: 0xDEADC0DE, (This is the canfd bus from the robot to the
 * adapter). canfd => ID: 0x202, Data: 0xDEADC0DE, (This is the canfd bus from the adapter to the
 * adapter). Bootloader will return ACK!
 *
 * 2. Bootloader will wait for OTA Order as upgrade mode, Support upgrade
 * methods of one to one and one to two one to one: Robot -> Adapter. one to two:
 * Robot -> Adapter -> Adapter. Return ACK!
 *
 * Need to send the MCU Device ID and Software Version to the Robot.
 *
 * 3. Bootloader will wait for OTA Order as firmware info, The app version
 * allows upward upgrades and forced upgrade. Return ACK!
 *
 * 4. Bootloader will wait for OTA Order as firmware package, Only perform
 * verification for the entire firmware package. Return ACK for every package!
 */

typedef enum {
	OTA_ORDER_AS_DEFAULT = 0x00,
	OTA_ORDER_AS_UPGRADE_MODE = 0x01,
	OTA_ORDER_AS_FIRMWARE_INFO = 0x02,
	OTA_ORDER_AS_PACKAGE = 0x03,
} OTA_ORDER_E;

typedef enum {
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_DEFAULT = 0X00,
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_UPWARD_UPGRADE = 0X01,
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_FORCED_UPGRADE = 0X02,
} OTA_ORDER_AS_FIRMWARE_INFO_ORDER_E;

typedef struct {
	uint32_t firmware_size;
	uint32_t firmware_crc;
	uint32_t firmware_version;
} OTA_FIRMWARE_INFO_T;

typedef struct {
	uint32_t total_package_cnt;
	uint32_t current_package_index;
	uint32_t firmware_package[6];
} OTA_PACKAGE_T;

typedef struct {
	OTA_ORDER_E ota_order;
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_E ota_order_as_firmware_info_order;
	OTA_FIRMWARE_INFO_T ota_firmware_info;
	OTA_PACKAGE_T ota_package;
} OTA_UPGRADE_INFO_T;

typedef enum {
	BOOT_PROCESS_NO_UPDATE_SIGNAL = 0x00,
	BOOT_PROCESS_WAIT_FOR_UPDATE_SIGNAL = 0x01,
	BOOT_PROCESS_GET_FIRMWARE_INFO = 0x02,
	BOOT_PROCESS_GET_FIRMWARE_PACKAGE = 0x03,
	BOOT_PROCESS_VIRIFY_FIRMWARE = 0x04,
	BOOT_PROCESS_BOOT_TO_APP = 0x05,
} BOOT_PROCESS_E;

class BOOT
{
      public:
	BOOT() = default;
	~BOOT() = default;
	BOOT(const BOOT &) = delete;
	BOOT &operator=(const BOOT &) = delete;
	static std::unique_ptr<BOOT> getInstance();
	void boot2app(void);
	void boot2boot(void);
	void init(void);
	void register_ota_canfd_data_signal();
	void set_ota_signal_timeout_flag(bool flag);
	bool get_ota_signal_timeout_flag(void);
	void set_deadloop_cnt(int cnt);
	int get_deadloop_cnt(void);

      private:
	static std::unique_ptr<BOOT> Instance;
	static std::unique_ptr<OTA_UPGRADE_INFO_T> ota_upgrade_info;
	std::shared_ptr<CAN> can_driver = CAN::getInstance();
	inline static bool ota_signal_timeout_flag = true;
	inline static int deadloop_cnt = 0;
	static void robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data);
	static void adapter2adapter_ota_process(const device *dev, can_frame *frame,
						void *user_data);
	void cleanup_arm_nvic(void);
	void return_adapter2robot_ota_ack(can_frame *frame);
	void return_adapter2adapter_ota_ack(can_frame *frame);
	void ota_upgrade_app_firmware(const device *dev, can_frame *frame);
	void ota_process(const device *dev, can_frame *frame);
};

#endif // __BOOT_HPP__
