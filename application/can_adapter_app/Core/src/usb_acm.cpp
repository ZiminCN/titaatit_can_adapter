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

#include "usb_acm.hpp"

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(usb_acm, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer_array[RING_BUF_SIZE];
struct ring_buf usb_cdc_acm_ringbuf;

const struct device *const usb_cdc_acm_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

std::unique_ptr<USB_ACM> USB_ACM::Instance = std::make_unique<USB_ACM>();

std::unique_ptr<USB_ACM> USB_ACM::getInstance()
{
	return std::move(USB_ACM::Instance);
}

bool USB_ACM::usb_cdc_acm_init()
{

	int ret = 0;

	if (!device_is_ready(usb_cdc_acm_dev)) {
		LOG_ERR("USB CDC ACM device not ready");
		return false;
	}

	ret = usb_enable(NULL);

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return false;
	}

	ring_buf_init(&usb_cdc_acm_ringbuf, sizeof(ring_buffer_array), ring_buffer_array);

	LOG_INF("USB CDC ACM initialized success");

	return true;
}
