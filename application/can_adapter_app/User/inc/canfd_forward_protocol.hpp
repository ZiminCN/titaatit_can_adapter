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

#ifndef __CANFD_PROTOCOL_HPP__
#define __CANFD_PROTOCOL_HPP__

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "Common.hpp"
#include "boot.hpp"
#include "can.hpp"
#include "timer.hpp"
#include <memory>

typedef struct {
	bool is_enable;
	uint64_t forward_data_cnt;
	uint64_t loop_back_forward_data_cnt;

	// canfd protocol
	uint32_t master_data_forward_bus_can_id;
	uint32_t slave_data_forward_bus_can_id;
	uint32_t bus_order_lock_order_data_forward_bus_can_id;
	uint32_t bus_order_data_forward_bus_can_id;
	uint8_t forward_bus_can_id_offset_max;
	struct can_filter bus_order_lock_order_filter;
	can_rx_callback_t bus_order_lock_order_callback;
	struct can_filter bus_order_filter;
	can_rx_callback_t bus_order_callback;
	struct can_filter master_filter;
	can_rx_callback_t master_callback;
	struct can_filter slave_filter;
	can_rx_callback_t slave_callback;
	struct can_filter forward_bus_filter;
	can_rx_callback_t forward_bus_callback;
	struct can_filter forward_bus_order_filter;
	can_rx_callback_t forward_bus_order_callback;
	struct can_filter heartbeat_filter;
	can_rx_callback_t heartbeat_callback;
	struct can_filter robot2adapter_boot_ota_filter;
	can_rx_callback_t robot2adapter_boot_ota_callback;
	struct can_filter adapter2adapter_boot_ota_filter;
	can_rx_callback_t adapter2adapter_boot_ota_callback;
} AdapterDataT;

typedef struct {
	bool is_enable;
	bool is_received_heartbeat;
	bool is_master_dev;
	bool is_slave_dev;
	uint8_t timeout_cnt;
} AdapterHeartBeatT;

typedef enum {
	HEART_BEAT_NO_CHANGE = 0x00,
	HEART_BEAT_DETECTED = 0x01,
	HEART_BEAT_LOST = 0x02,
} HeartbeatDetectedStatusE;

class CANFD_FORWARD_PROTOCOL
{
      public:
	CANFD_FORWARD_PROTOCOL() = default;
	~CANFD_FORWARD_PROTOCOL() = default;
	CANFD_FORWARD_PROTOCOL(const CANFD_FORWARD_PROTOCOL &) = delete;
	CANFD_FORWARD_PROTOCOL &operator=(const CANFD_FORWARD_PROTOCOL &) = delete;
	static std::unique_ptr<CANFD_FORWARD_PROTOCOL> getInstance();
	bool forward_protocol_init();
	int test_canfd_send();
	void heartbeat_pong_tong();
	HeartbeatDetectedStatusE is_detected_heartbeat();

	std::unique_ptr<BOOT> boot_driver_handle = BOOT::getInstance();

      private:
	static std::unique_ptr<CANFD_FORWARD_PROTOCOL> Instance;
	std::shared_ptr<CAN> can_driver_handle = CAN::getInstance();
	std::unique_ptr<TIMER> timer_driver_handle = TIMER::getInstance();

	static std::unique_ptr<AdapterDataT> adapter_data2robot;
	static std::unique_ptr<AdapterDataT> adapter_data2adapter;
	static std::unique_ptr<AdapterHeartBeatT> adapter_heart_beat;
	static void data2adapter_bus_order_lock_order_data_speedup_callback(const device *dev,
									    can_frame *frame,
									    void *user_data);
	static void data2adapter_bus_order_lock_order_data_callback(const device *dev,
								    can_frame *frame,
								    void *user_data);
	static void data2adapter_bus_order_data_speedup_callback(const device *dev,
								 can_frame *frame, void *user_data);
	static void data2adapter_bus_order_data_callback(const device *dev, can_frame *frame,
							 void *user_data);
	static void data2adapter_master_data_speedup_callback(const device *dev, can_frame *frame,
							      void *user_data);
	static void data2adapter_master_data_callback(const device *dev, can_frame *frame,
						      void *user_data);
	static void data2adapter_slave_data_speedup_callback(const device *dev, can_frame *frame,
							     void *user_data);
	static void data2adapter_slave_data_callback(const device *dev, can_frame *frame,
						     void *user_data);
	static void data2robot_forward_data_speedup_callback(const device *dev, can_frame *frame,
							     void *user_data);
	static void data2robot_forward_data_callback(const device *dev, can_frame *frame,
						     void *user_data);
	static void data2robot_forward_bus_order_data_speedup_callback(const device *dev,
								       can_frame *frame,
								       void *user_data);
	static void data2robot_forward_bus_order_data_callback(const device *dev, can_frame *frame,
							       void *user_data);
	static void data2robot_heartbeat_data_callback(const device *dev, can_frame *frame,
						       void *user_data);
	static void robot2adapter_bootloader_ota_callback(const device *dev, can_frame *frame,
							  void *user_data);
	static void adapter2adapter_bootloader_ota_callback(const device *dev, can_frame *frame,
							    void *user_data);
};

#endif // __CANFD_PROTOCOL_HPP__
