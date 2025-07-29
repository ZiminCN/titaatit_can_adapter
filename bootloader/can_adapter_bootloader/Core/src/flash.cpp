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

#include "flash.hpp"

// #include <zephyr/logging/log.h>

// LOG_MODULE_REGISTER(flash_manager, LOG_LEVEL_INF);

std::unique_ptr<FLASH_MANAGER> FLASH_MANAGER::Instance = std::make_unique<FLASH_MANAGER>();
std::unique_ptr<FACTORY_ARG_T> FLASH_MANAGER::factory_arg = std::make_unique<FACTORY_ARG_T>();

std::unique_ptr<FLASH_MANAGER> FLASH_MANAGER::getInstance()
{
	return std::move(Instance);
}

bool FLASH_MANAGER::init()
{
	return true;
}

int FLASH_MANAGER::get_factory_arg_free_cnt()
{
	const struct flash_area *temp_fa;
	struct flash_sector boot_arg_sector;
	FACTORY_ARG_T temp_factory_arg_data;
	uint32_t sec_cnt = 1;
	int free_cnt = 0;

	flash_area_open(FACTORY_AREA, &temp_fa);

	if (!flash_area_device_is_ready(temp_fa)) {
		// LOG_ERR("Flash area device is not ready!");
		flash_area_close(temp_fa);
		return false;
	}

	flash_area_get_sectors(FACTORY_AREA, &sec_cnt, &boot_arg_sector);

	for (uint32_t i = 0; i < boot_arg_sector.fs_size; i += sizeof(FACTORY_ARG_T)) {
		flash_area_read(temp_fa, i, &temp_factory_arg_data, sizeof(FACTORY_ARG_T));
		if (temp_factory_arg_data.magic_number == MAGIC_NUMBER) {
			free_cnt = i / sizeof(FACTORY_ARG_T);
			break;
		} else if (temp_factory_arg_data.magic_number != 0xFFFFFFFF) {
			// error
			flash_area_close(temp_fa);
			return false;
		}
	}

	flash_area_close(temp_fa);
	return free_cnt;
}

void FLASH_MANAGER::read_factory_arg_data(FACTORY_ARG_T *ouput_factory_arg_data)
{
	const struct flash_area *temp_fa;
	struct flash_sector boot_arg_sector;
	uint32_t sec_cnt = 1;

	flash_area_open(FACTORY_AREA, &temp_fa);

	if (!flash_area_device_is_ready(temp_fa)) {
		// LOG_ERR("Flash area device is not ready!");
		flash_area_close(temp_fa);
	}

	flash_area_get_sectors(FACTORY_AREA, &sec_cnt, &boot_arg_sector);

	flash_area_read(temp_fa, 0, ouput_factory_arg_data, sizeof(FACTORY_ARG_T));

	flash_area_close(temp_fa);
}

bool FLASH_MANAGER::write_factory_arg_data(FACTORY_ARG_T *factory_arg_data)
{
	const struct flash_area *temp_fa;
	struct flash_sector boot_arg_sector;
	uint32_t sec_cnt = 2;
	int ret = 0;
	int retry = 3;

	flash_area_open(FACTORY_AREA, &temp_fa);

	flash_area_get_sectors(FACTORY_AREA, &sec_cnt, &boot_arg_sector);

	if (!flash_area_device_is_ready(temp_fa)) {
		// LOG_ERR("Flash area device is not ready!");
		flash_area_close(temp_fa);
		return false;
	}

	do {
		// ret = flash_area_erase(temp_fa, 0, boot_arg_sector.fs_size);
		ret = flash_area_erase(temp_fa, 0, 1024);
		retry--;
	} while ((ret != 0) && (retry >= 0));

	flash_area_write(temp_fa, 0, factory_arg_data, sizeof(FACTORY_ARG_T));

	flash_area_close(temp_fa);

	return true;
}

bool FLASH_MANAGER::erase_app_flash_page(uint8_t page_num)
{
	const struct flash_area *temp_fa;
	struct flash_sector app_sector;
	uint32_t sec_cnt = 1;
	int ret = 0;
	int retry = 3;

	flash_area_open(APP_AREA, &temp_fa);

	if (!flash_area_device_is_ready(temp_fa)) {
		// LOG_ERR("Flash area device is not ready!");
		flash_area_close(temp_fa);
		return false;
	}

	flash_area_get_sectors(APP_AREA, &sec_cnt, &app_sector);

	do {
		ret = flash_area_erase(temp_fa, page_num * app_sector.fs_size,
				       (app_sector.fs_size));
		retry--;
	} while ((ret != 0) && (retry >= 0));

	flash_area_close(temp_fa);

	return true;
}
