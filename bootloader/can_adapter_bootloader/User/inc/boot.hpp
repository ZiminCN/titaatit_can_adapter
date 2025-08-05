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

#include "boot_can.hpp"
#include "dev_info.hpp"
#include "flash.hpp"
#include "ring_buf.hpp"
#include <memory>

/**
 * @brief Bootloader OTA Process
 *
 * 1. Bootloader/Application will wait for  OTA signal to enter OTA process.
 * OTA signal: canfd => ID: 0x382, Data: 0xDEADC0DE, (This is the canfd bus from
 * the robot to the adapter). canfd => ID: 0x202, Data: 0xDEADC0DE, (This is the
 * canfd bus from the adapter to the adapter). Bootloader will return ACK!
 *
 * 2. Bootloader will wait for OTA Order as upgrade mode, Support upgrade
 * methods of one to one and one to two one to one: Robot -> Adapter. one to
 * two: Robot -> Adapter -> Adapter. Return ACK! Please refer to the parameters
 * @arg OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE and @arg
 * OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO Need to send the MCU Device ID and Software
 * Version as ACK to the Robot.
 *
 * 3. Bootloader will wait for OTA Order as firmware info, The app version
 * allows upward upgrades and forced upgrade. Please refer to the parameters
 * @arg OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE and @arg
 * OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO. Return ACK!
 *
 *
 * 4. Bootloader will wait for OTA Order as firmware package, Only perform
 * verification for the entire firmware package. Return ACK for every package!
 *
 * 5. Bootloader will try jump to App, and The APP will accept
 * can data with 0x383 and order being OTA_ORDER_AS_CHECK_APP.
 * The host computer can determine whether the upgrade was successful through this ACK
 */

#define ACK_DEADC0DE 0xDEADC0DEU

typedef enum {
	ACK_DEFAULT = 0x00U,
	ACK_OK = 0x01U,
	ACK_ERROR = 0x02U,
} RETURN_ACK_INFO_E;

typedef enum {
	OTA_ORDER_AS_DEFAULT = 0x00U,
	OTA_ORDER_AS_UPGRADE_MODE = 0x01U,
	OTA_ORDER_AS_FIRMWARE_INFO = 0x02U,
	OTA_ORDER_AS_CHECK_APP = 0x03U,
} OTA_ORDER_E;

typedef enum {
	OTA_ORDER_AS_UPGRADE_MODE_ONE2ONE = 0x00U,
	OTA_ORDER_AS_UPGRADE_MODE_ONE2TWO = 0x01U,
} OTA_ORDER_AS_UPGRADE_MODE_E;

typedef enum {
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_DEFAULT = 0X00U,
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_UPWARD_UPGRADE = 0X01U,
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_AS_FORCED_UPGRADE = 0X02U,
} OTA_ORDER_AS_FIRMWARE_INFO_ORDER_E;

typedef struct {
	uint32_t firmware_size;
	uint32_t firmware_version;
	uint32_t firmware_build_timestamp;
	uint32_t firmware_crc32;
} OTA_FIRMWARE_INFO_T;

typedef struct {
	uint32_t total_package_cnt;
	uint32_t current_package_index; // first package is 1, start from 1
	uint32_t firmware_package[6];
} OTA_PACKAGE_T;

typedef struct {
	OTA_ORDER_E ota_order;
	OTA_ORDER_AS_UPGRADE_MODE_E ota_order_as_upgrade_mode;
	OTA_ORDER_AS_FIRMWARE_INFO_ORDER_E ota_order_as_firmware_info_order;
	OTA_FIRMWARE_INFO_T ota_firmware_info;
	OTA_PACKAGE_T ota_package;
} OTA_UPGRADE_INFO_T;

/**
 * receive canfd: ID: 0x382,
 * 		  Data: 0xDEADC0DE
 */
typedef struct {
	uint32_t ota_ack_info; // refer to RETURN_ACK_INFO_E
} RETURN_ACK_OTA_SIGNAL_T;

/**
 * receive canfd: ID: 0x383,
 * 		  Data: uint8_t ota_order
 * 			uint8_t ota_order_as_upgrade_mode
 */
typedef struct {
	OTA_ORDER_E ota_order;		// refer to OTA_ORDER_E
	RETURN_ACK_INFO_E ota_ack_info; // refer to RETURN_ACK_INFO_E
	uint32_t local_device_id[3];
	uint32_t remote_device_id[3]; // if one2two
} RETURN_ACK_OTA_UPGRADE_T;

/**
 * refer to OTA_FIRMWARE_INFO_T
 * receive canfd: ID: 0x383,
 * 		  Data: uint8_t ota_order
 * 			uint8_t ota_order_as_firmware_info_order
 * 			uint32_t firmware_size (Bytes)
 * 			uint32_t firmware_version
 * 			uint32_t firmware_build_timestamp
 * 			uint32_t firmware_crc3232
 * 	                uint32_t firmawre_all_package_cnt;
 */
typedef struct {
	OTA_ORDER_E ota_order;		// refer to OTA_ORDER_E
	RETURN_ACK_INFO_E ota_ack_info; // refer to RETURN_ACK_INFO_E
	uint32_t local_software_version;
	uint32_t local_software_build_timestamp;
	uint32_t remote_software_version;	  // if one2two
	uint32_t remote_software_build_timestamp; // if one2two
} RETURN_ACK_OTA_FIRMWARE_INFO_T;

/**
 * refer to OTA_PACKAGE_T
 * receive canfd: ID: 0x384,
 * 		  Data: uint32_t total_package_cnt
 * 			uint32_t current_package_cnt
 * 			uint32_t firmware_package[6]
 *
 */
typedef struct {
	uint32_t total_package_cnt;
	uint32_t current_package_index;
	RETURN_ACK_INFO_E ota_ack_info; // refer to RETURN_ACK_INFO_E
} RETURN_ACK_OTA_PACKAGE_T;

/**
 * receive canfd: ID: 0x383,
 * 		  Data: uint8_t ota_order
 *
 */
typedef struct {
	OTA_ORDER_E ota_order;		// refer to OTA_ORDER_E
	RETURN_ACK_INFO_E ota_ack_info; // refer to RETURN_ACK_INFO_E
} RETURN_ACK_OTA_CHECK_APP_T;

typedef struct {
	RETURN_ACK_OTA_SIGNAL_T return_ack_ota_signal;
	RETURN_ACK_OTA_UPGRADE_T return_ack_ota_upgrade;
	RETURN_ACK_OTA_FIRMWARE_INFO_T return_ack_ota_firmware_info;
	RETURN_ACK_OTA_PACKAGE_T return_ack_ota_package;
	RETURN_ACK_OTA_CHECK_APP_T return_ack_ota_check_app_crc;
} RETURN_ACK_T;

typedef enum {
	BOOT_PROCESS_NO_UPDATE_SIGNAL = 0x00U,
	BOOT_PROCESS_WAIT_FOR_UPDATE_SIGNAL = 0x01U,
	BOOT_PROCESS_GET_FIRMWARE_INFO = 0x02U,
	BOOT_PROCESS_GET_FIRMWARE_PACKAGE = 0x03U,
	BOOT_PROCESS_VIRIFY_FIRMWARE = 0x04U,
	BOOT_PROCESS_BOOT_TO_APP = 0x05U,
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
	void factory_arg_self_check(void);
	void factory_arg_error(void);
	void init(void);
	void register_ota_canfd_data_signal();
	void set_ota_signal_timeout_flag(bool flag);
	bool get_ota_signal_timeout_flag(void);
	void set_deadloop_cnt(int cnt);
	int get_deadloop_cnt(void);

	void set_boot_checkpoint_flag_active();
	void set_app_checkpoint_flag_active();
	uint8_t get_boot_upgrade_flag();
	void set_app_upgrade_flag_active();
	uint32_t test_return_app_crc32(uint8_t *data, uint32_t app_size);
	static std::unique_ptr<RETURN_ACK_T> return_ack;

      private:
	static std::unique_ptr<BOOT> Instance;
	static std::unique_ptr<RING_BUF> ring_buf_driver;
	static std::unique_ptr<OTA_UPGRADE_INFO_T> ota_upgrade_info;

	std::shared_ptr<BOOT_CAN> can_driver = BOOT_CAN::getInstance();
	std::shared_ptr<DEV_INFO> dev_info_driver = DEV_INFO::getInstance();
	std::unique_ptr<FLASH_MANAGER> flash_manager_driver = FLASH_MANAGER::getInstance();

	inline static bool ota_signal_timeout_flag = true;
	inline static int deadloop_cnt = 0;
	inline static int firmware_flash_package_cnt = 1;
	static void robot2adapter_ota_process(const device *dev, can_frame *frame, void *user_data);
	static void adapter2adapter_ota_process(const device *dev, can_frame *frame,
						void *user_data);
	void cleanup_arm_nvic(void);
	void return_adapter2robot_ota_ack(can_frame *frame);
	void return_adapter2adapter_ota_ack(can_frame *frame);
	void ota_info_verification(const device *dev, can_frame *frame);
	void ota_upgrade_app_firmware_one2one(const device *dev, can_frame *frame);
	void ota_upgrade_app_firmware_one2two(const device *dev, can_frame *frame);
};

#endif // __BOOT_HPP__
