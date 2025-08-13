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

#define DATA2ADAPTER_TX_TASK_PRIORITY 0
#define DATA2ADAPTER_TX_STACK_SIZE    2048
static struct k_thread data2adapter_tx_task_thread;
K_THREAD_STACK_DEFINE(data2adapter_tx_task_stack, DATA2ADAPTER_TX_STACK_SIZE);

#define DATA2ROBOT_TX_TASK_PRIORITY 0
#define DATA2ROBOT_TX_STACK_SIZE    2048
static struct k_thread data2robot_tx_task_thread;
K_THREAD_STACK_DEFINE(data2robot_tx_task_stack, DATA2ROBOT_TX_STACK_SIZE);

// double message buffer
CAN_MSGQ_DEFINE(data2adapter_dev_msgq_buffer_A, 1024);
CAN_MSGQ_DEFINE(data2adapter_dev_msgq_buffer_B, 1024);
CAN_MSGQ_DEFINE(data2robot_dev_msgq_buffer_A, 1024);
CAN_MSGQ_DEFINE(data2robot_dev_msgq_buffer_B, 1024);

std::unique_ptr<data2adapter_msgq_task_info_t> data2adapter_msgq_task_info =
	std::make_unique<data2adapter_msgq_task_info_t>();
std::unique_ptr<data2robot_msgq_task_info_t> data2robot_msgq_task_info =
	std::make_unique<data2robot_msgq_task_info_t>();

std::unique_ptr<CANFD_FORWARD_PROTOCOL> CANFD_FORWARD_PROTOCOL::Instance =
	std::make_unique<CANFD_FORWARD_PROTOCOL>();

std::unique_ptr<CANFD_FORWARD_PROTOCOL> CANFD_FORWARD_PROTOCOL::getInstance()
{
	return std::move(CANFD_FORWARD_PROTOCOL::Instance);
}

static bool stable_connection_flag = false;
static int stable_heartbeat_cnt = 0;
static int lost_heartbeat_cnt = 0;

//! CAN2 port: robot   <->  adapter
//! CAN3 port: adapter <->  adapter
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
		// filter slave robot device data
		.master_filter =
			{
				.id = 0x124,
				.mask = 0x7FC, // 0x124~0x127
				.flags = 0,
			},
		.master_callback = data2adapter_master_data_callback,
		// filter master robot device data
		.slave_filter =
			{
				.id = 0x10A,
				.mask = 0x7FE, // 0x10A~0x10B
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
		// filter about bootloader ota data(robot data to adapter)
		.robot2adapter_boot_ota_filter =
			{
				.id = CANFD_ID_AS_R2A_OTA_SIGNAL, // 0x382, 0x383
				.mask = 0x7FA,
				.flags = 0,
			},
		.robot2adapter_boot_ota_callback = robot2adapter_bootloader_ota_callback,
		// filter about bootloader ota data(adapter data to adapter)
		.adapter2adapter_boot_ota_filter = {},
		.adapter2adapter_boot_ota_callback = NULL,
	});

// CAN3 PORT receive another forward adapter device and forward send data to
// robot device
static struct can_frame canfd_data2robot;
std::unique_ptr<AdapterDataT> CANFD_FORWARD_PROTOCOL::adapter_data2robot =
	std::make_unique<AdapterDataT>(AdapterDataT{
		.is_enable = true,
		.forward_data_cnt = 0x0U,
		.loop_back_forward_data_cnt = 0x0U,
		.master_data_forward_bus_can_id = 0x124U,
		.slave_data_forward_bus_can_id = 0x10AU,
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
		// filter about bootloader ota data(robot data to adapter)
		.robot2adapter_boot_ota_filter = {},
		.robot2adapter_boot_ota_callback = NULL,
		// filter about bootloader ota data(adapter data to adapter)
		.adapter2adapter_boot_ota_filter =
			{
				.id = CANFD_ID_AS_A2A_OTA_SIGNAL,
				.mask = 0x7FA,
				.flags = 0,
			},
		.adapter2adapter_boot_ota_callback = adapter2adapter_bootloader_ota_callback,
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
		adapter_data2adapter->bus_order_lock_order_callback, this);
	// add filter for master robot device, forward data to adapter
	// 0x10A~0x10B
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_2_dev, &adapter_data2adapter->master_filter,
		adapter_data2adapter->master_callback, this);
	// add filter for slave robot device, forward data to adapter
	// 0x124~0x127
	i_ret = this->can_driver_handle->add_can_filter(canfd_2_dev,
							&adapter_data2adapter->slave_filter,
							adapter_data2adapter->slave_callback, this);
	// add filter for order data
	// 0x170
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_2_dev, &adapter_data2adapter->bus_order_filter,
		adapter_data2adapter->bus_order_callback, this);
	// add filter for bootloader data
	// 0x382, 0x383, 0x384, 0x385
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_2_dev, &adapter_data2adapter->robot2adapter_boot_ota_filter,
		adapter_data2adapter->robot2adapter_boot_ota_callback, this);

	// add filter for forward bus
	// 0x10~0x1F
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_3_dev, &adapter_data2robot->forward_bus_filter,
		adapter_data2robot->forward_bus_callback, this);
	// add filter for forward order bus
	// 0x20~0x2F
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_3_dev, &adapter_data2robot->forward_bus_order_filter,
		adapter_data2robot->forward_bus_order_callback, this);
	// add filter for heartbeat bus
	// 0x100
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_3_dev, &adapter_data2robot->heartbeat_filter,
		adapter_data2robot->heartbeat_callback, this);
	// add filter for bootloader data
	// 0x202, 0x203, 0x204, 0x205
	i_ret = this->can_driver_handle->add_can_filter(
		canfd_3_dev, &adapter_data2robot->adapter2adapter_boot_ota_filter,
		adapter_data2robot->adapter2adapter_boot_ota_callback, this);
	return true;
}

void CANFD_FORWARD_PROTOCOL::data2adapter_msgq_callback(const device *dev, can_frame *frame,
							void *user_data)
{
	ARG_UNUSED(dev);
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

	// The two buffers can only be switched between being full and empty. If data cannot be
	// written, it will be discarded.

	// Determine which message queue is currently being read.
	if (!forward_driver_handle->data2adapter_current_get_msgq_switch) {
		if (!k_msgq_num_free_get(&data2adapter_dev_msgq_buffer_A)) {
			// data2adapter_dev_msgq_buffer_B queue used index is zero or not
			if (!k_msgq_num_used_get(&data2adapter_dev_msgq_buffer_B)) {
				k_msgq_put(&data2adapter_dev_msgq_buffer_B, frame, K_MSEC(2));
			}
		} else {
			k_msgq_put(&data2adapter_dev_msgq_buffer_A, frame, K_MSEC(2));
		}
	} else if (forward_driver_handle->data2adapter_current_get_msgq_switch) {
		if (!k_msgq_num_free_get(&data2adapter_dev_msgq_buffer_B)) {
			// data2adapter_dev_msgq_buffer_A queue used index is zero or not
			if (!k_msgq_num_used_get(&data2adapter_dev_msgq_buffer_A)) {
				k_msgq_put(&data2adapter_dev_msgq_buffer_A, frame, K_MSEC(2));
			}
		} else {
			k_msgq_put(&data2adapter_dev_msgq_buffer_B, frame, K_MSEC(2));
		}
	}
}

void CANFD_FORWARD_PROTOCOL::data2robot_msgq_callback(const device *dev, can_frame *frame,
						      void *user_data)
{
	ARG_UNUSED(dev);
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

	// The two buffers can only be switched between being full and empty. If data cannot be
	// written, it will be discarded.

	// Determine which message queue is currently being read.
	if (!forward_driver_handle->data2robot_current_get_msgq_switch) {
		if (!k_msgq_num_free_get(&data2robot_dev_msgq_buffer_A)) {
			// data2robot_dev_msgq_buffer_B queue used index is zero or not
			if (!k_msgq_num_used_get(&data2robot_dev_msgq_buffer_B)) {
				k_msgq_put(&data2robot_dev_msgq_buffer_B, frame, K_MSEC(2));
			}
		} else {
			k_msgq_put(&data2adapter_dev_msgq_buffer_A, frame, K_MSEC(2));
		}
	} else if (forward_driver_handle->data2robot_current_get_msgq_switch) {
		if (!k_msgq_num_free_get(&data2robot_dev_msgq_buffer_B)) {
			// data2robot_dev_msgq_buffer_A queue used index is zero or not
			if (!k_msgq_num_used_get(&data2robot_dev_msgq_buffer_A)) {
				k_msgq_put(&data2robot_dev_msgq_buffer_A, frame, K_MSEC(2));
			}
		} else {
			k_msgq_put(&data2robot_dev_msgq_buffer_B, frame, K_MSEC(2));
		}
	}
}

void CANFD_FORWARD_PROTOCOL::data2adapter_bus_order_lock_order_data_callback(const device *dev,
									     can_frame *frame,
									     void *user_data)
{
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

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
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

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

void CANFD_FORWARD_PROTOCOL::data2adapter_slave_data_callback(const device *dev, can_frame *frame,
							      void *user_data)
{
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

	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

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

	// make sure connect stable
	if (stable_connection_flag != true) {
		//	return;
	}

	// forward_can_id = base_forward_can_id + offset
	canfd_data2adapter.id = adapter_data2adapter->slave_data_forward_bus_can_id +
				(frame->id - adapter_data2adapter->slave_filter.id);
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2adapter_master_data_callback(const device *dev, can_frame *frame,
							       void *user_data)
{

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

	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

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

	// make sure connect stable
	if (stable_connection_flag != true) {
		//	return;
	}

	// forward_can_id = base_forward_can_id + offset
	canfd_data2adapter.id = adapter_data2adapter->master_data_forward_bus_can_id +
				(frame->id - adapter_data2adapter->master_filter.id);
	canfd_data2adapter.dlc = frame->dlc;
	canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

	memcpy(canfd_data2adapter.data, frame->data, can_dlc_to_bytes(frame->dlc));
	forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev, &canfd_data2adapter);
}

void CANFD_FORWARD_PROTOCOL::data2robot_forward_data_callback(const device *dev, can_frame *frame,
							      void *user_data)
{
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

	const struct device *canfd_2_dev =
		forward_driver_handle->can_driver_handle->get_canfd_2_dev();

	// use if-else instead of switch-case
	if ((frame->id & (0x4F8U)) == (adapter_data2adapter->slave_data_forward_bus_can_id)) {

		canfd_data2robot.id = adapter_data2robot->slave_data_forward_bus_can_id +
				      (frame->id - adapter_data2robot->forward_bus_filter.id -
				       adapter_data2robot->forward_bus_can_id_offset_max);
	} else if ((frame->id & (0x4F8U)) ==
		   (adapter_data2adapter->master_data_forward_bus_can_id)) {
		canfd_data2robot.id = adapter_data2robot->master_data_forward_bus_can_id +
				      (frame->id - adapter_data2robot->forward_bus_filter.id);
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
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

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

void CANFD_FORWARD_PROTOCOL::robot2adapter_bootloader_ota_callback(const device *dev,
								   can_frame *frame,
								   void *user_data)
{
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

	const struct device *canfd_2_dev =
		forward_driver_handle->can_driver_handle->get_canfd_2_dev();

	switch (frame->id) {
	case CANFD_ID_AS_R2A_OTA_SIGNAL: {
		// jump2boot
		forward_driver_handle->boot_driver_handle->boot2boot();
		break;
	}
	case CANFD_ID_AS_R2A_OTA_UPGRADE: {
		OTA_ORDER_E ota_order = static_cast<OTA_ORDER_E>(frame->data[0]);
		if (ota_order == OTA_ORDER_AS_CHECK_APP) {
			// return ack ok
			forward_driver_handle->boot_driver_handle->return_ack
				->return_ack_ota_check_app.ota_order = OTA_ORDER_AS_CHECK_APP;
			forward_driver_handle->boot_driver_handle->return_ack
				->return_ack_ota_check_app.ota_ack_info = ACK_OK;

			// set ack frame
			canfd_data2robot.id = CANFD_ID_AS_A2R_OTA_ACK;
			canfd_data2robot.dlc = can_bytes_to_dlc(64);
			canfd_data2robot.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

			memcpy(canfd_data2robot.data,
			       &(forward_driver_handle->boot_driver_handle->return_ack
					 ->return_ack_ota_check_app),
			       sizeof(RETURN_ACK_OTA_CHECK_APP_T));
			forward_driver_handle->can_driver_handle->send_can_msg(canfd_2_dev,
									       &canfd_data2robot);
		}
		break;
	}
	default: {
		break;
	}
	}
}

void CANFD_FORWARD_PROTOCOL::adapter2adapter_bootloader_ota_callback(const device *dev,
								     can_frame *frame,
								     void *user_data)
{
	CANFD_FORWARD_PROTOCOL *forward_driver_handle =
		static_cast<CANFD_FORWARD_PROTOCOL *>(user_data);

	const struct device *canfd_3_dev =
		forward_driver_handle->can_driver_handle->get_canfd_3_dev();

	switch (frame->id) {
	case CANFD_ID_AS_A2A_OTA_SIGNAL: {
		// jump2boot
		forward_driver_handle->boot_driver_handle->boot2boot();
		break;
	}
	case CANFD_ID_AS_A2A_OTA_UPGRADE: {
		OTA_ORDER_E ota_order = static_cast<OTA_ORDER_E>(frame->data[0]);
		if (ota_order == OTA_ORDER_AS_CHECK_APP) {
			// return ack ok
			forward_driver_handle->boot_driver_handle->return_ack
				->return_ack_ota_check_app.ota_order = OTA_ORDER_AS_CHECK_APP;
			forward_driver_handle->boot_driver_handle->return_ack
				->return_ack_ota_check_app.ota_ack_info = ACK_OK;

			// set ack frame
			canfd_data2adapter.id = CANFD_ID_AS_A2A_OTA_ACK;
			canfd_data2adapter.dlc = can_bytes_to_dlc(64);
			canfd_data2adapter.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;

			memcpy(canfd_data2adapter.data,
			       &(forward_driver_handle->boot_driver_handle->return_ack
					 ->return_ack_ota_check_app),
			       sizeof(RETURN_ACK_OTA_CHECK_APP_T));
			forward_driver_handle->can_driver_handle->send_can_msg(canfd_3_dev,
									       &canfd_data2adapter);
		}
		break;
	}
	default: {
		break;
	}
	}
}

void CANFD_FORWARD_PROTOCOL::data2robot_heartbeat_data_callback(const device *dev, can_frame *frame,
								void *user_data)
{
	ARG_UNUSED(user_data);
	adapter_heart_beat->is_received_heartbeat = true;
	stable_heartbeat_cnt += 1;
	adapter_heart_beat->timeout_cnt = 0x00U;

	// can frame heartbeat data[2] is the is_master_dev, data[3] is the
	// is_slave_dev another adapter is master dev

	if (stable_heartbeat_cnt >= 20) {
		stable_connection_flag = true;
		stable_heartbeat_cnt = 0x05U;
	}

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

		if (stable_connection_flag == false) {
			lost_heartbeat_cnt += 1;

			// it means lost some heartbeat at the very beginning of the docking
			// process
			if (((lost_heartbeat_cnt - stable_heartbeat_cnt) >= 5)) {
				lost_heartbeat_cnt = 0;
				stable_heartbeat_cnt = 0;
			}
		}

		if (adapter_heart_beat->timeout_cnt >= 10) {
			LOG_INF("Loss Heart Beat...");
			adapter_heart_beat->is_received_heartbeat = false;
			adapter_heart_beat->is_master_dev = false;
			adapter_heart_beat->is_slave_dev = false;
			adapter_heart_beat->timeout_cnt = 0;
			lost_heartbeat_cnt = 0;
			stable_heartbeat_cnt = 0;
			stable_connection_flag = false;
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
		// slave heartbeat ID: 0x101
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

static bool last_heartbeat_status = false;
HeartbeatDetectedStatusE CANFD_FORWARD_PROTOCOL::is_detected_heartbeat()
{
	if (last_heartbeat_status == adapter_heart_beat->is_received_heartbeat) {
		return HeartbeatDetectedStatusE::HEART_BEAT_NO_CHANGE; // No change in
								       // heartbeat status
	} else {
		last_heartbeat_status = adapter_heart_beat->is_received_heartbeat;
		if (last_heartbeat_status == true) {
			return HeartbeatDetectedStatusE::HEART_BEAT_DETECTED; // Heartbeat
									      // detected
		} else {
			return HEART_BEAT_LOST; // Heartbeat lost
		}
	}

	return HeartbeatDetectedStatusE::HEART_BEAT_NO_CHANGE; // Default return value
							       // if no condition is
							       // met
}

void CANFD_FORWARD_PROTOCOL::data2adapter_msgq_transmit_task(void *arg1, void *arg2, void *arg3)
{
	CANFD_FORWARD_PROTOCOL *forward_driver_handle = static_cast<CANFD_FORWARD_PROTOCOL *>(arg1);

	while (1) {
		// Determine which buffer is currently in use
		if (!forward_driver_handle->data2adapter_current_get_msgq_switch) {
			if (!k_msgq_num_free_get(&data2adapter_dev_msgq_buffer_A)) {
				forward_driver_handle->data2adapter_current_get_msgq_switch = true;
			}
		} else {
			if (!k_msgq_num_free_get(&data2adapter_dev_msgq_buffer_B)) {
				forward_driver_handle->data2adapter_current_get_msgq_switch = false;
			}
		}

		if (!forward_driver_handle->data2adapter_current_get_msgq_switch) {

		} else {
		}
	}
}

int CANFD_FORWARD_PROTOCOL::data2adapter_msgq_transmit_create_task()
{
	// create task
	this->data2adapter_msgq_task_info->loop_flag = true;

	this->data2adapter_msgq_task_info->tid =
		k_thread_create(&data2adapter_tx_task_thread, data2adapter_tx_task_stack,
				K_THREAD_STACK_SIZEOF(data2adapter_tx_task_stack),
				this->data2adapter_msgq_transmit_task, this, NULL, NULL,
				K_PRIO_COOP(DATA2ADAPTER_TX_TASK_PRIORITY), 0, K_NO_WAIT);

	if (!this->data2adapter_msgq_task_info->tid) {
		return -ENOMEM;
	}
	k_thread_name_set(this->data2adapter_msgq_task_info->tid, "Data2AdapterTxTask");
	k_thread_start(&data2adapter_tx_task_thread);

	return 0;
}

void CANFD_FORWARD_PROTOCOL::data2robot_msgq_transmit_task(void *arg1, void *arg2, void *arg3)
{
	// CANFD_FORWARD_PROTOCOL *forward_driver_handle = static_cast<CANFD_FORWARD_PROTOCOL
	// *>(arg1);
}

int CANFD_FORWARD_PROTOCOL::data2robot_msgq_transmit_create_task()
{
	// create task
	this->data2robot_msgq_task_info->loop_flag = true;

	this->data2robot_msgq_task_info->tid =
		k_thread_create(&data2robot_tx_task_thread, data2robot_tx_task_stack,
				K_THREAD_STACK_SIZEOF(data2robot_tx_task_stack),
				this->data2robot_msgq_transmit_task, this, NULL, NULL,
				K_PRIO_COOP(DATA2ROBOT_TX_TASK_PRIORITY), 0, K_NO_WAIT);

	if (!this->data2robot_msgq_task_info->tid) {
		return -ENOMEM;
	}
	k_thread_name_set(this->data2robot_msgq_task_info->tid, "Data2RobotTxTask");
	k_thread_start(&data2robot_tx_task_thread);

	return 0;
}
