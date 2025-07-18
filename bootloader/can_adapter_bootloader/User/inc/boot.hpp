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

#ifndef __BOOT_HPP__
#define __BOOT_HPP__

#include <memory>

typedef enum {
	BOOT_PROCESS_NO_UPDATE_SIGNAL = 0x00,
	BOOT_PROCESS_WAIT_FOR_UPDATE_SIGNAL = 0x01,
	BOOT_PROCESS_GET_FIRMWARE_INFO = 0x02,
	BOOT_PROCESS_GET_FIRMWARE_PACKAGE = 0x03,
	BOOT_PROCESS_VIRIFY_FIRMWARE = 0x04,
	BOOT_PROCESS_BOOT_TO_APP = 0x05,
} BOOT_PROCESS_E;

class BOOT
{
      public:
	BOOT() = default;
	~BOOT() = default;
	BOOT(const BOOT &) = delete;
	BOOT &operator=(const BOOT &) = delete;
	static std::unique_ptr<BOOT> getInstance();
	void cleanup_arm_nvic(void);
	void boot2app(void);
	void do_boot();

      private:
	static std::unique_ptr<BOOT> Instance;
};

#endif // __BOOT_HPP__
