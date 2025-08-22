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

#ifndef __GPIO_CONTROL_HPP__
#define __GPIO_CONTROL_HPP__

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "gpio.hpp"
#include <memory>

class GPIO_CONTROL
{
      public:
	GPIO_CONTROL() = default;
	~GPIO_CONTROL() = default;
	GPIO_CONTROL(const GPIO_CONTROL &) = delete;
	GPIO_CONTROL &operator=(const GPIO_CONTROL &) = delete;
	static std::unique_ptr<GPIO_CONTROL> getInstance();

	void init();
	void set_48v_mosfet_state(gpio_flags_t extra_flags);
	void set_heartbeat_led_state(gpio_flags_t extra_flags);
	void set_system_led_state(gpio_flags_t extra_flags);

      private:
	static std::unique_ptr<GPIO_CONTROL> Instance;
	std::unique_ptr<GPIO> gpio_handle = GPIO::getInstance();
};

#endif // __GPIO_CONTROL_HPP__
