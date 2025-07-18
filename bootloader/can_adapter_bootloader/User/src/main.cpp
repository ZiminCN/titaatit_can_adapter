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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "boot.hpp"
#include "can.hpp"
#include "flash.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("Hello World! I am %s", CONFIG_BOARD);
	std::unique_ptr<BOOT> boot_manager_driver = BOOT::getInstance();

	k_sleep(K_SECONDS(2));

	boot_manager_driver->boot2app();
	// boot_manager_driver->do_boot();

	// std::unique_ptr<FLASH_MANAGER> flash_manager_driver = FLASH_MANAGER::getInstance();

	// FACTORY_ARG_T test_arg = {
	// 	.magic_number = MAGIC_NUMBER,
	// 	.is_boot_update_flag = 0xA,
	// 	.is_app_update_flag = 0xB,
	// 	.boot_status = BOOT_OK,
	// 	.boot_build_timestamp = 0x12345678,
	// 	.app_build_timestamp = 0x87654321,
	// 	.boot_version = 0xC,
	// 	.boot_version_major = 0xD,
	// 	.boot_version_minor = 0xE,
	// 	.app_version = 0xF,
	// 	.app_version_major = 0xAB,
	// 	.app_version_minor = 0xCD,
	// 	.hw_version = 0xEF,
	// };

	// flash_manager_driver->write_factory_arg_data(&test_arg);

	// LOG_INF("Now try to erase app flash area...[0x08010000 - 0x0803F800]");
	// flash_manager_driver->erase_app_flash_page(0);
	// LOG_INF("Erase app flash area down!");

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
