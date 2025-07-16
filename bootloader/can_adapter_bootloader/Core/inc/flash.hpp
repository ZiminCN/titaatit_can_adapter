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

#ifndef __FLASH_HPP__
#define __FLASH_HPP__

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

#include <memory>

#define MAGIC_NUMBER 0xDEADC0DE

#define BOOT_AREA    FIXED_PARTITION_ID(boot_partition)
#define APP_AREA     FIXED_PARTITION_ID(slot0_partition)
#define FACTORY_AREA FIXED_PARTITION_ID(factory_partition)

typedef enum {
	BOOT_OK = 0x00,
	BOOT_ENTER_UPDATE = 0x01,
	BOOT_ERROR = 0xFF,
} BOOT_STATUS_E;

typedef struct {
	uint32_t magic_number;
	uint8_t is_boot_update_flag;
	uint8_t is_app_update_flag;

	BOOT_STATUS_E boot_status;

	// based from 1970-01-01 00:00:00 UTC
	uint32_t boot_build_timestamp;
	uint32_t app_build_timestamp;

	uint8_t boot_version;
	uint8_t boot_version_major;
	uint8_t boot_version_minor;
	uint8_t app_version;
	uint8_t app_version_major;
	uint8_t app_version_minor;
	uint8_t hw_version;

	uint32_t boot_crc_value;
	uint32_t app_crc_value;
} FACTORY_ARG_T;

class FLASH_MANAGER
{
      public:
	FLASH_MANAGER() = default;
	~FLASH_MANAGER() = default;
	FLASH_MANAGER(const FLASH_MANAGER &) = delete;
	FLASH_MANAGER &operator=(const FLASH_MANAGER &) = delete;
	static std::unique_ptr<FLASH_MANAGER> getInstance();
	bool init();
	bool erase_app_flash_page(uint8_t page_num);
	void read_factory_arg_data(FACTORY_ARG_T *ouput_factory_arg_data);
	bool write_factory_arg_data(FACTORY_ARG_T *factory_arg_data);

      private:
	static std::unique_ptr<FLASH_MANAGER> Instance;
	static std::unique_ptr<FACTORY_ARG_T> factory_arg;
	int get_factory_arg_free_cnt();
};

#endif // __FLASH_HPP__