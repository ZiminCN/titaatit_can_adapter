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

// CAN2 PORT receive robot device data and forward send data to another forward
// adapter device
static struct can_frame canfd_data2adapter;
std::unique_ptr<AdapterDataT> CANFD_FORWARD_PROTOCOL::adapter_data2adapter =
	std::make_unique<AdapterDataT>(AdapterDataT{
		.is_enable = true,
		.forward_data_cnt = 0x0U,
		.loop_back_forward_data_cnt = 0x0U,
		.master_data_forward_bus_can_id = 0x10U,	       // 0x10U ~ 0x17U
		.slave_data_forward_bus_can_id = 0x18U,		       // 0x18U ~ 0x1FU
		.bus_order_lock_order_data_forward_bus_can_id = 0x20U, // 0x20U ~ 0x27U
		.bus_order_data_forward_bus_can_id = 0x28U,	       // 0x28U ~ 0x2FU
		.forward_bus_can_id_offset_max = 0x08U,
		// filter robot lock order data
		.bus_order_lock_order_filter =
			{
				.id = 0x89,
				.mask = 0x7FF, // 0x89U
				.flags = 0,
			},
		.bus_order_lock_order_callback = data2adapter_bus_order_lock_order_data_callback,
		// filter robot bus order device data
		.bus_order_filter =
			{
				.id = 0x170,
				.mask = 0x7FF, // 0x170U
				.flags = 0,
			},
		.bus_order_callback = data2adapter_bus_order_data_callback,
		// filter master robot device data
		.master_filter =
			{
				.id = 0x10A,
				.mask = 0x7FE, // 0x10A~0x10B
				.flags = 0,
			},
		.master_callback = data2adapter_master_data_callback,
		// filter slave robot device data
		.slave_filter =
			{
				.id = 0x124,
				.mask = 0x7FC, // 0x124~0x127
				.flags = 0,
			},
		.slave_callback = data2adapter_slave_data_callback,
		// filter forward bus data
		.forward_bus_filter = {},
		.forward_bus_callback = NULL,
		// filter forward bus order data
		.forward_bus_order_filter = {},
		.forward_bus_order_callback = NULL,
		// filter forward bus heartbeat data
		.heartbeat_filter = {},
		.heartbeat_callback = NULL,
	});

// CAN3 PORT receive another forward adapter device and forward send data to
// robot device
static struct can_frame canfd_data2robot;
std::unique_ptr<AdapterDataT> CANFD_FORWARD_PROTOCOL::adapter_data2robot =
	std::make_unique<AdapterDataT>(AdapterDataT{
		.is_enable = true,
		.forward_data_cnt = 0x0U,
		.loop_back_forward_data_cnt = 0x0U,
		.master_data_forward_bus_can_id = 0x10AU,
		.slave_data_forward_bus_can_id = 0x124U,
		.bus_order_lock_order_data_forward_bus_can_id = 0x89U, // 0x89U
		.bus_order_data_forward_bus_can_id = 0x170U,
		.forward_bus_can_id_offset_max = 0x08U,
		// filter robot lock order data
		.bus_order_lock_order_filter = {},
		.bus_order_lock_order_callback = NULL,
		// filter robot bus order device data
		.bus_order_filter = {},
		.bus_order_callback = NULL,
		// filter master robot device data
		.master_filter = {},
		.master_callback = NULL,
		// filter slave robot device data
		.slave_filter = {},
		.slave_callback = NULL,
		// filter forward bus data
		.forward_bus_filter =
			{
				.id = 0x10,
				.mask = 0x7F0, // 0x10 ~ 0x1F
				.flags = 0,
			},
		.forward_bus_callback = data2robot_forward_data_callback,
		// filter forward bus order data
		.forward_bus_order_filter =
			{
				.id = 0x20,
				.mask = 0x7F0, // 0x20 ~ 0x2F
				.flags = 0,
			},
		.forward_bus_order_callback = data2robot_forward_bus_order_data_callback,
		// filter forward bus heartbeat data
		.heartbeat_filter =
			{
				.id = 0x100,
				.mask = 0x7FE, // master: 0x100, slave: 0x101
				.flags = 0,
			},
		.heartbeat_callback = data2robot_heartbeat_data_callback,
	});

std::unique_ptr<AdapterHeartBeatT> CANFD_FORWARD_PROTOCOL::adapter_heart_beat =
	std::make_unique<AdapterHeartBeatT>(AdapterHeartBeatT{
		.is_enable = true,
		.is_received_heartbeat = false,
		.is_master_dev = false,
		.is_slave_dev = false,
		.timeout_cnt = 0x0U,
	});

bool CANFD_FORWARD_PROTOCOL::forward_protocol_init()
{
	int i_ret;

	// can port to robot device
	const struct device *canfd_2_dev = this->can_driver_handle->get_canfd_2_dev();
	// can port to other forward device
	const struct device *canfd_3_dev = this->can_driver_handle->get_canfd_3_dev();

	// add filter for order lock data
	// 0x89
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_2_dev, &adapter_data2adapter->bus_order_lock_order_filter,
		adapter_data2adapter->bus_order_lock_order_callback);
	// add filter for master robot device, forward data to adapter
	// 0x10A~0x10B
	i_ret = this->can_driver_handle->add_can_filter(canfd_2_dev,
							&adapter_data2adapter->master_filter,
							adapter_data2adapter->master_callback);
	// add filter for slave robot device, forward data to adapter
	// 0x124~0x127
	i_ret = this->can_driver_handle->add_can_filter(canfd_2_dev,
							&adapter_data2adapter->slave_filter,
							adapter_data2adapter->slave_callback);
	// add filter for order data
	// 0x170
	i_ret = this->can_driver_handle->add_can_filter(canfd_2_dev,
							&adapter_data2adapter->bus_order_filter,
							adapter_data2adapter->bus_order_callback);

	// add filter for forward bus
	// 0x10~0x1F
	i_ret = this->can_driver_handle->add_can_filter(canfd_3_dev,
							&adapter_data2robot->forward_bus_filter,
							adapter_data2robot->forward_bus_callback);
	// add filter for forward order bus
	// 0x20~0x2F
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_3_dev, &adapter_data2robot->forward_bus_order_filter,
		adapter_data2robot->forward_bus_order_callback);
	// add filter for heartbeat bus
	// 0x100
	i_ret = this->can_driver_handle->add_can_filter(canfd_3_dev,
							&adapter_data2robot->heartbeat_filter,
							adapter_data2robot->heartbeat_callback);

	return true;
}

void CANFD_FORWARD_PROTOCOL::data2adapter_bus_order_lock_order_data_callback(const device *dev,
									     can_frame *frame,
									     void *user_data)
{
	ARG_UNUSED(user_data);

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_3_dev =
		forward_driver_handle->can_driver_handle->get_canfd_3_dev();

	adapter_data2adapter->forward_data_cnt += 1;
	if (adapter_data2adapter->forward_data_cnt >= UINT64_MAX) {
		adapter_data2adapter->forward_data_cnt = 0;
		adapter_data2adapter->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2adapter->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2adapter->loop_back_forward_data_cnt = 0;
		}
	}

	// forward_can_id = base_forward_can_id
	canfd_data2adapter.id = adapter_data2adapter->bus_order_lock_order_data_forward_bus_can_id;
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2adapter_bus_order_data_callback(const device *dev,
								  can_frame *frame, void *user_data)
{
	ARG_UNUSED(user_data);

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_3_dev =
		forward_driver_handle->can_driver_handle->get_canfd_3_dev();

	adapter_data2adapter->forward_data_cnt += 1;
	if (adapter_data2adapter->forward_data_cnt >= UINT64_MAX) {
		adapter_data2adapter->forward_data_cnt = 0;
		adapter_data2adapter->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2adapter->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2adapter->loop_back_forward_data_cnt = 0;
		}
	}

	// forward_can_id = base_forward_can_id
	canfd_data2adapter.id = adapter_data2adapter->bus_order_data_forward_bus_can_id;
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2adapter_master_data_callback(const device *dev, can_frame *frame,
							       void *user_data)
{
	ARG_UNUSED(user_data);

	// if ((adapter_heart_beat->is_master_dev != false) ||
	//     (adapter_heart_beat->is_slave_dev != false)) {

	// 	if (!adapter_heart_beat->is_received_heartbeat) {
	// 		return;
	// 	}
	// } else {
	// 	// make sure reset
	// 	adapter_heart_beat->is_received_heartbeat = false;
	// 	adapter_heart_beat->is_master_dev = false;
	// 	adapter_heart_beat->is_slave_dev = false;
	// 	adapter_heart_beat->timeout_cnt = 0;
	// }

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_3_dev =
		forward_driver_handle->can_driver_handle->get_canfd_3_dev();

	adapter_heart_beat->is_master_dev = true;
	adapter_data2adapter->forward_data_cnt += 1;
	if (adapter_data2adapter->forward_data_cnt >= UINT64_MAX) {
		adapter_data2adapter->forward_data_cnt = 0;
		adapter_data2adapter->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2adapter->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2adapter->loop_back_forward_data_cnt = 0;
		}
	}

	// forward_can_id = base_forward_can_id + offset
	canfd_data2adapter.id = adapter_data2adapter->master_data_forward_bus_can_id +
				(frame->id - adapter_data2adapter->master_filter.id);
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2adapter_slave_data_callback(const device *dev, can_frame *frame,
							      void *user_data)
{
	ARG_UNUSED(user_data);

	// if ((adapter_heart_beat->is_master_dev != false) ||
	//     (adapter_heart_beat->is_slave_dev != false)) {
	// 	if (!adapter_heart_beat->is_received_heartbeat) {
	// 		return;
	// 	}
	// } else {
	// 	// make sure reset
	// 	adapter_heart_beat->is_received_heartbeat = false;
	// 	adapter_heart_beat->is_master_dev = false;
	// 	adapter_heart_beat->is_slave_dev = false;
	// 	adapter_heart_beat->timeout_cnt = 0;
	// }

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_3_dev =
		forward_driver_handle->can_driver_handle->get_canfd_3_dev();

	adapter_heart_beat->is_slave_dev = true;
	adapter_data2adapter->forward_data_cnt += 1;
	if (adapter_data2adapter->forward_data_cnt >= UINT64_MAX) {
		adapter_data2adapter->forward_data_cnt = 0;
		adapter_data2adapter->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2adapter->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2adapter->loop_back_forward_data_cnt = 0;
		}
	}

	// forward_can_id = base_forward_can_id + offset
	canfd_data2adapter.id = adapter_data2adapter->slave_data_forward_bus_can_id +
				(frame->id - adapter_data2adapter->slave_filter.id);
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2robot_forward_data_callback(const device *dev, can_frame *frame,
							      void *user_data)
{
	ARG_UNUSED(user_data);

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_2_dev =
		forward_driver_handle->can_driver_handle->get_canfd_2_dev();

	// use if-else instead of switch-case
	if ((frame->id & (0x4F8U)) == (adapter_data2adapter->master_data_forward_bus_can_id)) {
		canfd_data2robot.id = adapter_data2robot->master_data_forward_bus_can_id +
				      (frame->id - adapter_data2robot->forward_bus_filter.id);
	} else if ((frame->id & (0x4F8U)) ==
		   (adapter_data2adapter->slave_data_forward_bus_can_id)) {
		canfd_data2robot.id = adapter_data2robot->slave_data_forward_bus_can_id +
				      (frame->id - adapter_data2robot->forward_bus_filter.id -
				       adapter_data2robot->forward_bus_can_id_offset_max);
	}

	adapter_data2robot->forward_data_cnt += 1;
	if (adapter_data2robot->forward_data_cnt >= UINT64_MAX) {
		adapter_data2robot->forward_data_cnt = 0;
		adapter_data2robot->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2robot->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2robot->loop_back_forward_data_cnt = 0;
		}
	}

	canfd_data2robot.dlc = frame->dlc;
	canfd_data2robot.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2robot.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_2_dev, &canfd_data2robot);
}

void CANFD_FORWARD_PROTOCOL::data2robot_forward_bus_order_data_callback(const device *dev,
									can_frame *frame,
									void *user_data)
{
	ARG_UNUSED(user_data);

	std::unique_ptr<CANFD_FORWARD_PROTOCOL> forward_driver_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();
	const struct device *canfd_2_dev =
		forward_driver_handle->can_driver_handle->get_canfd_2_dev();

	// use if-else instead of switch-case
	if ((frame->id & (0x4F8U)) ==
	    (adapter_data2adapter->bus_order_lock_order_data_forward_bus_can_id)) {
		canfd_data2robot.id =
			adapter_data2robot->bus_order_lock_order_data_forward_bus_can_id;
	} else if ((frame->id & (0x4F8U)) ==
		   (adapter_data2adapter->bus_order_data_forward_bus_can_id)) {
		canfd_data2robot.id = adapter_data2robot->bus_order_data_forward_bus_can_id;
	}

	adapter_data2robot->forward_data_cnt += 1;
	if (adapter_data2robot->forward_data_cnt >= UINT64_MAX) {
		adapter_data2robot->forward_data_cnt = 0;
		adapter_data2robot->loop_back_forward_data_cnt += 1;

		// reset loop back
		if (adapter_data2robot->loop_back_forward_data_cnt >= UINT64_MAX) {
			adapter_data2robot->loop_back_forward_data_cnt = 0;
		}
	}

	canfd_data2robot.dlc = frame->dlc;
	canfd_data2robot.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2robot.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_2_dev, &canfd_data2robot);
}

void CANFD_FORWARD_PROTOCOL::data2robot_heartbeat_data_callback(const device *dev, can_frame *frame,
								void *user_data)
{
	ARG_UNUSED(user_data);
	adapter_heart_beat->is_received_heartbeat = true;
	adapter_heart_beat->timeout_cnt = 0x00U;

	// can frame heartbeat data[2] is the is_master_dev, data[3] is the
	// is_slave_dev another adapter is master dev
	if ((frame->data[2] == 0x01U) && (frame->data[3] == 0x00U)) {
		// this adapter is slave dev
		adapter_heart_beat->is_master_dev = false;
		adapter_heart_beat->is_slave_dev = true;
	} else if ((frame->data[2] == 0x00U) && (frame->data[3] == 0x01U)) {
		// otherwise, this adapter is master dev
		adapter_heart_beat->is_master_dev = true;
		adapter_heart_beat->is_slave_dev = false;
	}
}

// just a test function
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

static struct can_frame canfd_data2heartbeat;
void CANFD_FORWARD_PROTOCOL::heartbeat_pong_tong()
{
	if (adapter_heart_beat->is_received_heartbeat == true) {
		adapter_heart_beat->timeout_cnt += 1;
		if (adapter_heart_beat->timeout_cnt >= 10) {
			LOG_INF("Loss Heart Beat...");
			adapter_heart_beat->is_received_heartbeat = false;
			adapter_heart_beat->is_master_dev = false;
			adapter_heart_beat->is_slave_dev = false;
			adapter_heart_beat->timeout_cnt = 0;
		}
	}

	// wait for robot master can data to send heartbeat

	// Assign heartbeat ID
	if ((adapter_heart_beat->is_master_dev == true) &&
	    (adapter_heart_beat->is_slave_dev == false)) {
		// master heartbeat ID: 0x100
		adapter_heart_beat->is_received_heartbeat = true;
		canfd_data2heartbeat.id = 0x100U;
	} else if ((adapter_heart_beat->is_slave_dev == true) &&
		   (adapter_heart_beat->is_master_dev == false)) {
		// master heartbeat ID: 0x101
		adapter_heart_beat->is_received_heartbeat = true;
		canfd_data2heartbeat.id = 0x101U;
	} else {
		return;
	}

	const struct device *canfd_3_dev = this->can_driver_handle->get_canfd_3_dev();

	canfd_data2heartbeat.dlc = can_bytes_to_dlc(5);
	canfd_data2heartbeat.data[0] = ((adapter_heart_beat->is_enable == true) ? 0x01U : 0x00U);
	canfd_data2heartbeat.data[1] =
		((adapter_heart_beat->is_received_heartbeat == true) ? 0x01U : 0x00U);
	canfd_data2heartbeat.data[2] =
		((adapter_heart_beat->is_master_dev == true) ? 0x01U : 0x00U);
	canfd_data2heartbeat.data[3] = ((adapter_heart_beat->is_slave_dev == true) ? 0x01U : 0x00U);
	canfd_data2heartbeat.data[4] = adapter_heart_beat->timeout_cnt;

	this->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2heartbeat);
}