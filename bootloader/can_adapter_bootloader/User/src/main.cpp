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
#include <zephyr/settings/settings.h>

#include "boot.hpp"
#include "flash.hpp"
#include "timer.hpp"

static FACTORY_ARG_T factory_arg;

std::unique_ptr<FLASH_MANAGER> flash_manager_driver = FLASH_MANAGER::getInstance();
std::unique_ptr<BOOT> boot_driver = BOOT::getInstance();
std::unique_ptr<TIMER> timer_driver = TIMER::getInstance();

static int wait_for_ota_timeout_cnt = 0;
static bool timer_timeout_flag = false;
void wait_for_ota_signal_expiry_callback(struct k_timer *timer)
{
	if (boot_driver->get_ota_signal_timeout_flag() == true) {
		wait_for_ota_timeout_cnt += 1;
	} else {
		timer_driver->timer_stop(timer_driver->get_ota_signal_timer());
		return;
	}

	if (wait_for_ota_timeout_cnt > 30) {
		timer_driver->timer_stop(timer_driver->get_ota_signal_timer());
		return;
	}
}
void wait_for_ota_signal_stop_callback(struct k_timer *timer)
{
	wait_for_ota_timeout_cnt = 0;
	timer_timeout_flag = true;
}

static int deadloop_cnt = 0;
void avoid_deadloop_expiry_callback(struct k_timer *timer)
{
	deadloop_cnt += 1;

	// 200ms * 50 = 10s
	if (deadloop_cnt > 50) {
		// reboot
		timer_driver->timer_stop(timer_driver->get_deadloop_timer());
		return;
	}
}

void avoid_deadloop_stop_callback(struct k_timer *timer)
{
	// reboot
	deadloop_cnt = 0;
	timer_timeout_flag = true;
	flash_manager_driver->read_factory_arg_data(&factory_arg);
	factory_arg.arg_status = FACTORY_ARG_STATUS_ERROR;
	flash_manager_driver->write_factory_arg_data(&factory_arg);
	boot_driver->boot2boot();
}

int main(void)
{
	bool ret_bool;

	timer_driver->timer_init(timer_driver->get_ota_signal_timer(),
				 wait_for_ota_signal_expiry_callback,
				 wait_for_ota_signal_stop_callback);
	timer_driver->timer_init(timer_driver->get_deadloop_timer(), avoid_deadloop_expiry_callback,
				 avoid_deadloop_stop_callback);
	boot_driver->init();

	// self-check
	flash_manager_driver->read_factory_arg_data(&factory_arg);
	ret_bool = flash_manager_driver->check_factory_arg_data_is_void(&factory_arg);

	if (likely(ret_bool != false)) {
		// no factory arg data
		flash_manager_driver->init_new_factory_arg_data(&factory_arg);
		flash_manager_driver->write_factory_arg_data(&factory_arg);
	}

	// wait for updata can message
	boot_driver->register_ota_canfd_data_signal();
	timer_timeout_flag = false;
	timer_driver->timer_start(timer_driver->get_ota_signal_timer(), K_NO_WAIT, K_MSEC(100));
	// if get ota signal, cancel timer

	while (!timer_timeout_flag) {
		k_sleep(K_MSEC(200));
	}

	// if get ota signal timeout, boot to app
	if (boot_driver->get_ota_signal_timeout_flag() != false) {
		boot_driver->boot2app();
	}

	// enter ota mode, add a 10 sec timer to avoid dead loop
	timer_timeout_flag = false;
	timer_driver->timer_start(timer_driver->get_deadloop_timer(), K_NO_WAIT, K_MSEC(200));

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
