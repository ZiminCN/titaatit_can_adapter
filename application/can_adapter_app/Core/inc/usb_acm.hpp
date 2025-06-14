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

#include <memory>
class USB_ACM
{
      public:
	USB_ACM() = default;
	~USB_ACM() = default;
	USB_ACM(const USB_ACM &) = delete;
	USB_ACM &operator=(const USB_ACM &) = delete;
	static std::unique_ptr<USB_ACM> getInstance();
	bool usb_cdc_acm_init();

      private:
	static std::unique_ptr<USB_ACM> Instance;
};

#endif // __USB_ACM_HPP__
