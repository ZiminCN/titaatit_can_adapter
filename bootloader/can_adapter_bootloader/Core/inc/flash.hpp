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

#define MAGIC_NUMBER	      0xDEADC0DE
#define WRITE_FLASH_PAGE_SIZE 2048

#define BOOT_AREA    FIXED_PARTITION_ID(boot_partition)
#define APP_AREA     FIXED_PARTITION_ID(slot0_partition)
#define FACTORY_AREA FIXED_PARTITION_ID(factory_partition)

#define BOOT_AREA_SIZE	  FIXED_PARTITION_SIZE(boot_partition)
#define APP_AREA_SIZE	  FIXED_PARTITION_SIZE(slot0_partition)
#define FACTORY_AREA_SIZE FIXED_PARTITION_SIZE(factory_partition)

typedef enum {
	FACTORY_ARG_STATUS_OK,
	FACTORY_ARG_NOT_READY,
	FACTORY_ARG_STATUS_ERROR,
} FACTORY_ARG_STATUS_E;

typedef struct {
	uint32_t magic_number;
	uint8_t is_boot_update_flag;
	uint8_t is_app_update_flag;
	uint8_t boot_checkpoint_flag;
	uint8_t app_checkpoint_flag;

	FACTORY_ARG_STATUS_E arg_status;

	// based from 1970-01-01 00:00:00 UTC
	uint32_t boot_build_timestamp;
	uint32_t app_build_timestamp;

	uint32_t boot_version;
	uint32_t app_version;

	uint32_t reserved_data_1;
} FACTORY_ARG_T;

class FLASH_MANAGER
{
      public:
	static FACTORY_ARG_T factory_arg;
	FLASH_MANAGER() = default;
	~FLASH_MANAGER() = default;
	FLASH_MANAGER(const FLASH_MANAGER &) = delete;
	FLASH_MANAGER &operator=(const FLASH_MANAGER &) = delete;
	static std::unique_ptr<FLASH_MANAGER> getInstance();
	bool init();
	bool erase_app_flash_page(uint8_t page_num);
	bool write_app_flash_page(uint8_t *data, uint16_t data_len, uint8_t page_num);
	bool erase_all_app_flash();
	void init_new_factory_arg_data();
	bool check_factory_arg_data_is_void();
	void read_factory_arg_data();
	bool write_factory_arg_data();
	uint32_t calculate_app_firmware_crc32(uint32_t firmware_size);
	FACTORY_ARG_T get_factory_arg();
	void set_factory_arg(FACTORY_ARG_T arg);

      private:
	static std::unique_ptr<FLASH_MANAGER> Instance;
	int get_factory_arg_free_cnt();
};

#endif // __FLASH_HPP__