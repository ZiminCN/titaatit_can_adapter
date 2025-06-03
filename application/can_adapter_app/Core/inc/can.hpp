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

#ifndef __CAN_HPP__
#define __CAN_HPP__

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include "Common.hpp"
#include <memory>

#define FILTER_ID_ARRAY_SIZE 10

typedef struct {
	enum can_state state;
	uint32_t bus_tx_count;
	uint32_t bus_rx_count;
	uint32_t bus_tx_error;
	uint32_t bus_rx_error;

	int filter_id_array[FILTER_ID_ARRAY_SIZE];
	int filter_id_array_count;
} can_bus_status;

class CAN
{
      public:
	CAN() {

	};
	~CAN() = default;
	static std::unique_ptr<CAN> getInstance();
	bool init();
	static void any_tx_callback(const struct device *dev, int error, void *user_data);
	int send_can_msg(const struct device *dev, const struct can_frame *frame);
	int add_can_filter(const struct device *dev, k_msgq *msgq, const struct can_filter *filter);
	int add_can_filter(const struct device *dev, const struct can_filter *filter,
			   can_rx_callback_t callback);
	std::unique_ptr<can_bus_status> get_can_bus_status(const struct device *dev);
	static const struct device *const get_canfd_1_dev();
	static const struct device *const get_canfd_2_dev();
	static const struct device *const get_canfd_3_dev();
	bool reset_can_filter(const struct device *dev);

      private:
	static std::unique_ptr<CAN> Instance;
	CAN(const CAN &) = delete;
	CAN &operator=(const CAN &) = delete;
	static std::unique_ptr<can_bus_status> canfd_1_dev_bus_status;
	static std::unique_ptr<can_bus_status> canfd_2_dev_bus_status;
	static std::unique_ptr<can_bus_status> canfd_3_dev_bus_status;
};

#endif // __CAN_HPP__