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
 * 1. Bootloader/Application will wait for OTA signal to enter OTA process.
 * 	OTA signal: canfd => 	ID: 0x382,
 * 				Data: 0xDEADC0DE, (This is the canfd bus from the robot to the
 * adapter). canfd   =>    ID: 0x202, Data: 0xDEADC0DE, (This is the canfd bus from the adapter to
 * the adapter). No Return ACK!
 * 2. Bootloader will wait for OTA Order as upgrade mode, Support upgrade methods of one to one and
 * one to two one to one: Robot -> Adapter one to two: Robot -> Adapter -> Adapter Return ACK!
 * 3. Bootloader will wait for OTA Order as firmware info, The app version only allows upward
 * upgrades. Return ACK!
 * 4. Bootloader will wait for OTA Order as firmware package, Only perform verification for the
 * entire firmware package. Return ACK for every package!
 */

typedef enum {
	OTA_ORDER_AS_UPGRADE_MODE = 0x00,
	OTA_ORDER_AS_FIRMWARE_INFO = 0x01,
	OTA_ORDER_AS_PACKAGE = 0x02,
} OTA_ORDER_E;

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
	void ota_process(const device *dev, can_frame *frame);
	void set_ota_signal_timeout_flag(bool flag);
	bool get_ota_signal_timeout_flag(void);
	void set_deadloop_flag(bool flag);
	bool get_deadloop_flag(void);

      private:
	static std::unique_ptr<BOOT> Instance;
	std::shared_ptr<CAN> can_driver = CAN::getInstance();
	inline static bool ota_signal_timeout_flag = true;
	inline static bool deadloop_flag = false; // true: deadloop, false: not deadloop
	static void robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data);
	static void adapter2adapter_ota_process(const device *dev, can_frame *frame,
						void *user_data);
	void cleanup_arm_nvic(void);
};

#endif // __BOOT_HPP__
