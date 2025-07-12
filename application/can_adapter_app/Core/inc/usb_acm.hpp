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

#ifndef __USB_ACM_HPP__
#define __USB_ACM_HPP__

#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>

#include "timer.hpp"
#include <memory>

class USB_ACM
{
      public:
	USB_ACM() = default;
	~USB_ACM() = default;
	USB_ACM(const USB_ACM &) = delete;
	USB_ACM &operator=(const USB_ACM &) = delete;
	static std::unique_ptr<USB_ACM> getInstance();
	static USB_ACM &getInstancePtr();
	int usb_cdc_acm_init();

      private:
	static std::unique_ptr<USB_ACM> Instance;
	bool rx_throttled;
	static void print_baudrate(const struct device *dev);
	static void usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg);
	int enable_usb_device_next(void);
	static void interrupt_handler(const struct device *dev, void *user_data);
	struct usbd_context *usbd_init_device(usbd_msg_cb_t msg_cb);
	struct usbd_context *usbd_setup_device(usbd_msg_cb_t msg_cb);
	static void fix_code_triple(struct usbd_context *uds_ctx, const enum usbd_speed speed);
};

#endif // __USB_ACM_HPP__
