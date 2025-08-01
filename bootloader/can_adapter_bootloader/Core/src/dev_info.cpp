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

#include "dev_info.hpp"

#include <zephyr/drivers/hwinfo.h>

std::shared_ptr<DEV_INFO> DEV_INFO::Instance = std::make_shared<DEV_INFO>();
std::unique_ptr<HARDWARE_DEVICE_INFO_T> DEV_INFO::hardware_device_info =
	std::make_unique<HARDWARE_DEVICE_INFO_T>(HARDWARE_DEVICE_INFO_T{{0x00, 0x00, 0x00}});

std::shared_ptr<DEV_INFO> DEV_INFO::getInstance()
{
	return Instance;
}

void DEV_INFO::init()
{
	this->set_hardware_device_uuid();
}

HARDWARE_DEVICE_INFO_T &DEV_INFO::get_hardware_device_uuid()
{
	return *(this->hardware_device_info); // 解引用返回
}

void DEV_INFO::set_hardware_device_uuid()
{
	int size_ret = 0;
	size_ret = hwinfo_get_device_id(this->hardware_device_info->uuid,
					sizeof(this->hardware_device_info->uuid));
}