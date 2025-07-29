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

#include "gpio.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio, LOG_LEVEL_INF);

static const struct gpio_dt_spec pwr48v_mosfet_spec =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwr_48v_mosfet_gpios);

std::unique_ptr<GPIO> GPIO::Instance = std::make_unique<GPIO>();

std::unique_ptr<GPIO> GPIO::getInstance()
{
	return std::move(GPIO::Instance);
}

void GPIO::init()
{
	bool ret;

	ret = gpio_is_ready_dt(&pwr48v_mosfet_spec);
	if (!ret) {
		LOG_ERR("PWR 48V MOSFET GPIO is not ready!");
		return;
	}

	LOG_INF("PWR 48V MOSFET GPIO is ready!");
}

void GPIO::gpio_callback(std::function<void()> callback)
{
	callback();
}

void GPIO::set_gpio_state(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags)
{
	gpio_pin_configure_dt(spec, extra_flags);
}

void GPIO::set_48v_gpio_state(gpio_flags_t extra_flags)
{
	this->gpio_callback(
		std::bind(&GPIO::set_gpio_state, this, &pwr48v_mosfet_spec, extra_flags));
}
