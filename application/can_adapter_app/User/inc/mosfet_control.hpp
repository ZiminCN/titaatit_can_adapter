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

#ifndef __MOSFET_CONTROL_HPP__
#define __MOSFET_CONTROL_HPP__

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "gpio.hpp"
#include <memory>

class MOSFET_CONTROL
{
      public:
	MOSFET_CONTROL() = default;
	~MOSFET_CONTROL() = default;
	MOSFET_CONTROL(const MOSFET_CONTROL &) = delete;
	MOSFET_CONTROL &operator=(const MOSFET_CONTROL &) = delete;
	static std::unique_ptr<MOSFET_CONTROL> getInstance();

	void init();
	void set_48v_mosfet_state(gpio_flags_t extra_flags);

      private:
	static std::unique_ptr<MOSFET_CONTROL> Instance;
	std::unique_ptr<GPIO> gpio_handle = GPIO::getInstance();
};

#endif // __MOSFET_CONTROL_HPP__
