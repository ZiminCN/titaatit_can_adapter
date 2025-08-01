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

#ifndef __DEV_INFO_HPP__
#define __DEV_INFO_HPP__

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <memory>

typedef struct {
	uint8_t uuid[12]; // 96 bits
} HARDWARE_DEVICE_INFO_T;

class DEV_INFO
{
      public:
	DEV_INFO() = default;
	~DEV_INFO() = default;
	DEV_INFO(const DEV_INFO &) = delete;
	DEV_INFO &operator=(const DEV_INFO &) = delete;
	static std::shared_ptr<DEV_INFO> getInstance();
	void init();
	HARDWARE_DEVICE_INFO_T &get_hardware_device_uuid();

      private:
	static std::shared_ptr<DEV_INFO> Instance;
	static std::unique_ptr<HARDWARE_DEVICE_INFO_T> hardware_device_info;
	void set_hardware_device_uuid(void);
};

#endif // __DEV_INFO_HPP__