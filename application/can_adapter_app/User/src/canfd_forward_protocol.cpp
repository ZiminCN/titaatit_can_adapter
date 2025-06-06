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

#include "canfd_forward_protocol.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(canfd_forward_protocol, LOG_LEVEL_INF);

std::unique_ptr<CANFD_FORWARD_PROTOCOL> CANFD_FORWARD_PROTOCOL::Instance =
	std::make_unique<CANFD_FORWARD_PROTOCOL>();

std::unique_ptr<CANFD_FORWARD_PROTOCOL> CANFD_FORWARD_PROTOCOL::getInstance()
{
	return std::move(CANFD_FORWARD_PROTOCOL::Instance);
}

std::unique_ptr<AdapterDataT> CANFD_FORWARD_PROTOCOL::adapter_data2master =
	std::make_unique<AdapterDataT>(AdapterDataT{
		.is_enable = true,
		.extern_port_dev_can_id_min = 0x10A,
		.extern_port_dev_can_id_max = 0x10B,
		.is_tx2master = true,
		.is_tx2slave = false,
		.is_tx2peripheral = false,
		.forward_bus_can_id = 0x1U,
		.filter =
			{
				.id = 0x10A,
				.mask = 0x7FE, // 0x10A~0x10B
				.flags = CAN_FILTER_IDE,
			},
		.callback = [](const struct device *dev, struct can_frame *frame, void *user_data) {
			ARG_UNUSED(dev);
			ARG_UNUSED(user_data);
			ARG_UNUSED(frame);
			LOG_INF("CANFD_FORWARD_PROTOCOL::adapter_data2master callback");
		}});

std::unique_ptr<AdapterDataT> CANFD_FORWARD_PROTOCOL::adapter_data2slave =
	std::make_unique<AdapterDataT>(AdapterDataT{
		.is_enable = true,
		.extern_port_dev_can_id_min = 0x10A,
		.extern_port_dev_can_id_max = 0x10B,
		.is_tx2master = true,
		.is_tx2slave = false,
		.is_tx2peripheral = false,
		.forward_bus_can_id = 0x2U,
		.filter =
			{
				.id = 0x124,
				.mask = 0x7FC, // 0x124~0x127
				.flags = CAN_FILTER_IDE,
			},
		.callback = [](const struct device *dev, struct can_frame *frame, void *user_data) {
			ARG_UNUSED(dev);
			ARG_UNUSED(user_data);
			ARG_UNUSED(frame);
			LOG_INF("CANFD_FORWARD_PROTOCOL::adapter_data2slave callback");
		}});

std::unique_ptr<AdapterHeartBeatT> CANFD_FORWARD_PROTOCOL::adapter_heart_beat =
	std::make_unique<AdapterHeartBeatT>(AdapterHeartBeatT{
		.is_enable = true,
		.is_received_heartbeat = false,
		.is_master_dev = false,
		.is_slave_dev = false,
	});

bool CANFD_FORWARD_PROTOCOL::forward_protocol_init()
{
	int i_ret;

	const struct device *canfd_1_dev = this->can_driver_handle->get_canfd_1_dev();
	// const struct device *canfd_2_dev = this->can_driver_handle->get_canfd_2_dev();
	// const struct device *canfd_3_dev = this->can_driver_handle->get_canfd_3_dev();

	i_ret = this->can_driver_handle->add_can_filter(canfd_1_dev, &adapter_data2master->filter,
							adapter_data2master->callback);

	return true;
}

int CANFD_FORWARD_PROTOCOL::test_canfd_send()
{
	int ret;
	const struct device *canfd_1_dev = this->can_driver_handle->get_canfd_1_dev();
	const struct device *canfd_2_dev = this->can_driver_handle->get_canfd_2_dev();
	const struct device *canfd_3_dev = this->can_driver_handle->get_canfd_3_dev();

	uint16_t now = sys_clock_cycle_get_32() & 0xFFFF; // transform 32-bit to 16-bit

	struct can_frame canfd_1_msg = {
		.id = 0x101U,
		.dlc = can_bytes_to_dlc(8),
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
		.timestamp = now,
		.data = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01},
	};
	struct can_frame canfd_2_msg = {
		.id = 0x102U,
		.dlc = can_bytes_to_dlc(8),
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
		.timestamp = now,
		.data = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02},
	};
	struct can_frame canfd_3_msg = {
		.id = 0x103U,
		.dlc = can_bytes_to_dlc(8),
		.flags = CAN_FRAME_FDF | CAN_FRAME_BRS,
		.timestamp = now,
		.data = {0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03},
	};

	ret = this->can_driver_handle->send_can_msg(canfd_1_dev, &canfd_1_msg);
	if (ret != 0) {
		LOG_ERR("send canfd_1_msg failed, err code:[%d]", ret);
		return false;
	}

	ret = this->can_driver_handle->send_can_msg(canfd_2_dev, &canfd_2_msg);
	if (ret != 0) {
		LOG_ERR("send canfd_2_msg failed, err code:[%d]", ret);
		return false;
	}

	ret = this->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_3_msg);
	if (ret != 0) {
		LOG_ERR("send canfd_3_msg failed, err code:[%d]", ret);
		return false;
	}

	return true;
}