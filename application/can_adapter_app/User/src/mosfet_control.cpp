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

#include "mosfet_control.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mosfet_control, LOG_LEVEL_INF);

std::unique_ptr<MOSFET_CONTROL> MOSFET_CONTROL::Instance = std::make_unique<MOSFET_CONTROL>();

std::unique_ptr<MOSFET_CONTROL> MOSFET_CONTROL::getInstance()
{
        return std::move(MOSFET_CONTROL::Instance);
}

void MOSFET_CONTROL::init(){
        this->gpio_handle->init();
}

void MOSFET_CONTROL::set_48v_mosfet_state(gpio_flags_t extra_flags)
{
        this->gpio_handle->set_48v_gpio_state(extra_flags);
}
