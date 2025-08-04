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

void avoid_deadloop_expiry_callback(struct k_timer *timer)
{
	boot_driver->set_deadloop_cnt(boot_driver->get_deadloop_cnt() + 1);

	// 200ms * 25 = 5s, timeout 5 sec
	if (boot_driver->get_deadloop_cnt() > 25) {
		// reboot
		timer_driver->timer_stop(timer_driver->get_deadloop_timer());
		return;
	}
}

void avoid_deadloop_stop_callback(struct k_timer *timer)
{
	// reboot
	boot_driver->set_deadloop_cnt(0);
	timer_timeout_flag = true;
	boot_driver->factory_arg_error();
	boot_driver->boot2boot();
}

int main(void)
{
	timer_driver->timer_init(timer_driver->get_ota_signal_timer(),
				 wait_for_ota_signal_expiry_callback,
				 wait_for_ota_signal_stop_callback);
	timer_driver->timer_init(timer_driver->get_deadloop_timer(), avoid_deadloop_expiry_callback,
				 avoid_deadloop_stop_callback);
	boot_driver->init();

	// self-check
	boot_driver->factory_arg_self_check();

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
		// boot_driver->boot2app();
		boot_driver->boot2boot();
	}

	// enter ota mode, add a 10 sec timer to avoid dead loop
	timer_timeout_flag = false;
	boot_driver->set_deadloop_cnt(0);
	timer_driver->timer_start(timer_driver->get_deadloop_timer(), K_NO_WAIT, K_MSEC(200));

	while (1) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
