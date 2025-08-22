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

#ifndef __GPIO_HPP__
#define __GPIO_HPP__

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <functional>
#include <memory>

class GPIO
{
      public:
	GPIO() = default;
	~GPIO() = default;
	GPIO(const GPIO &) = delete;
	GPIO &operator=(const GPIO &) = delete;
	static std::unique_ptr<GPIO> getInstance();
	void init();
	void set_48v_gpio_state(gpio_flags_t extra_flags);
	void set_heartbeat_gpio_state(gpio_flags_t extra_flags);
	void set_system_gpio_state(gpio_flags_t extra_flags);

      private:
	static std::unique_ptr<GPIO> Instance;
	void gpio_callback(std::function<void()> callback);
	void set_gpio_state(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags);
};

#endif // __GPIO_HPP__