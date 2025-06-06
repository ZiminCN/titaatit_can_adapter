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

#include "can.hpp"
#include "fsm.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
	LOG_INF("Hello World! I am %s", CONFIG_BOARD);

	// std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();

	// fsm_driver_handle->fsm_init(FSM_INIT_STATE);

	std::shared_ptr<CAN> can_driver_handle = CAN::getInstance();
	const struct device *canfd_1_dev = can_driver_handle->get_canfd_1_dev();
	const struct device *canfd_2_dev = can_driver_handle->get_canfd_2_dev();
	const struct device *canfd_3_dev = can_driver_handle->get_canfd_3_dev();

	can_driver_handle->init();

	struct can_frame test_frame;
	test_frame.id = 0x123U;
	test_frame.dlc = 8U;
	test_frame.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;
	// test_frame.flags =  0;
	test_frame.data[0] = 0x11U;

	int ret = 0;

	while (1) {
		// ret = can_driver_handle->send_can_msg(canfd_1_dev, &test_frame);
		// if(ret!=0){
		// 	LOG_INF("send can1 msg failed");
		// }
		// ret = can_driver_handle->send_can_msg(canfd_2_dev, &test_frame);
		// if(ret!=0){
		// 	LOG_INF("send can2 msg failed");
		// }
		ret = can_driver_handle->send_can_msg(canfd_3_dev, &test_frame);
		if (ret != 0) {
			LOG_INF("send can3 msg failed");
		}
		LOG_INF("Idle...");
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
