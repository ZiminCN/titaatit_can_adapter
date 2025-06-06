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

#include "can.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can, LOG_LEVEL_INF);

static const struct device *canfd_1_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd1));
static const struct device *canfd_2_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd2));
static const struct device *canfd_3_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canfd3));

std::shared_ptr<CAN> CAN::Instance = std::make_unique<CAN>();

std::shared_ptr<CAN> CAN::getInstance()
{
	return std::move(CAN::Instance);
}

std::shared_ptr<can_bus_status> CAN::canfd_1_dev_bus_status =
	std::make_unique<can_bus_status>(can_bus_status{
		.state = CAN_STATE_BUS_OFF,
		.bus_tx_count = 0,
		.bus_rx_count = 0,
		.bus_tx_error = 0,
		.bus_rx_error = 0,
		.filter_id_array = {0},
		.filter_id_array_count = 0,
	});

std::shared_ptr<can_bus_status> CAN::canfd_2_dev_bus_status =
	std::make_unique<can_bus_status>(can_bus_status{
		.state = CAN_STATE_BUS_OFF,
		.bus_tx_count = 0,
		.bus_rx_count = 0,
		.bus_tx_error = 0,
		.bus_rx_error = 0,
		.filter_id_array = {0},
		.filter_id_array_count = 0,
	});

std::shared_ptr<can_bus_status> CAN::canfd_3_dev_bus_status =
	std::make_unique<can_bus_status>(can_bus_status{
		.state = CAN_STATE_BUS_OFF,
		.bus_tx_count = 0,
		.bus_rx_count = 0,
		.bus_tx_error = 0,
		.bus_rx_error = 0,
		.filter_id_array = {0},
		.filter_id_array_count = 0,
	});

bool CAN::init()
{
	int ret = 0;

	if (!device_is_ready(canfd_1_dev)) {
		LOG_INF("CANFD device 1 is not ready");
	}

	if (!device_is_ready(canfd_2_dev)) {
		LOG_INF("CANFD device 2 is not ready");
	}

	if (!device_is_ready(canfd_3_dev)) {
		LOG_INF("CANFD device 3 is not ready");
	}

	can_set_mode(canfd_1_dev, CAN_MODE_FD);
	can_set_mode(canfd_2_dev, CAN_MODE_FD);
	can_set_mode(canfd_3_dev, CAN_MODE_FD);

	ret = can_start(canfd_1_dev);
	if (ret < 0) {
		LOG_ERR("Failed to start CANFD device 1: %d", ret);
		return false;
	}

	ret = can_start(canfd_2_dev);
	if (ret < 0) {
		LOG_ERR("Failed to start CANFD device 2: %d", ret);
		return false;
	}

	ret = can_start(canfd_3_dev);
	if (ret < 0) {
		LOG_ERR("Failed to start CANFD device 3: %d", ret);
		return false;
	}

	LOG_INF("CANFD Driver Init Success!");
	return true;
}

const struct device *const CAN::get_canfd_1_dev()
{
	return canfd_1_dev;
}

const struct device *const CAN::get_canfd_2_dev()
{
	return canfd_2_dev;
}

const struct device *const CAN::get_canfd_3_dev()
{
	return canfd_3_dev;
}

std::shared_ptr<can_bus_status> CAN::get_can_bus_status(const struct device *dev)
{
	if (dev == canfd_1_dev) {
		return std::move(canfd_1_dev_bus_status);
	} else if (dev == canfd_2_dev) {
		return std::move(canfd_2_dev_bus_status);
	} else if (dev == canfd_3_dev) {
		return std::move(canfd_3_dev_bus_status);
	}

	LOG_ERR("Unknown CAN device!");
	return nullptr;
}

void CAN::any_tx_callback(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);

	std::shared_ptr<CAN> can_api = CAN::getInstance();

	std::shared_ptr<can_bus_status> can_bus_status = can_api->get_can_bus_status(dev);

	if (error == 0) {
		can_bus_status->bus_tx_count += 1;
	} else {
		can_bus_status->bus_tx_error += 1;
	}
}

int CAN::send_can_msg(const struct device *dev, const struct can_frame *frame)
{
	return can_send(dev, frame, K_MSEC(20), any_tx_callback, nullptr);
}

int CAN::add_can_filter(const struct device *dev, k_msgq *msgq, const struct can_filter *filter)
{
	uint8_t filter_id = 0x00;
	filter_id = can_add_rx_filter_msgq(dev, msgq, filter);

	if (filter_id < 0) {
		LOG_ERR("Add CAN filter ID:[%x] failed!", filter_id);
		return filter_id;
	}

	std::shared_ptr<can_bus_status> can_bus_status = this->get_can_bus_status(dev);

	can_bus_status->filter_id_array[can_bus_status->filter_id_array_count] = filter_id;
	can_bus_status->filter_id_array_count += 1;

	return filter_id;
}

int CAN::add_can_filter(const struct device *dev, const struct can_filter *filter,
			can_rx_callback_t callback)
{
	uint8_t filter_id = 0x00;
	filter_id = can_add_rx_filter(dev, callback, (void *)dev, filter);

	if (filter_id < 0) {
		LOG_ERR("Add CAN filter ID:[%x] failed!", filter_id);
		return filter_id;
	}

	std::shared_ptr<can_bus_status> can_bus_status = this->get_can_bus_status(dev);
	can_bus_status->filter_id_array[can_bus_status->filter_id_array_count] = filter_id;
	can_bus_status->filter_id_array_count += 1;
	return filter_id;
}

bool CAN::reset_can_filter(const struct device *dev)
{
	std::shared_ptr<can_bus_status> can_bus_status = this->get_can_bus_status(dev);

	uint8_t filter_id = 0x00;
	bool flag = true;
	can_bus_status->bus_tx_count = 0;
	can_bus_status->bus_rx_count = 0;
	can_bus_status->bus_tx_error = 0;
	can_bus_status->bus_rx_error = 0;

	if (can_bus_status->filter_id_array_count <= 0) {
		return true;
	}

	do {
		filter_id =
			can_bus_status->filter_id_array[can_bus_status->filter_id_array_count - 1];
		can_remove_rx_filter(dev, filter_id);
		can_bus_status->filter_id_array_count -= 1;

		if (can_bus_status->filter_id_array_count <= 0) {
			flag = false;
		}
	} while (flag);

	return true;
}
