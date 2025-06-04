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
#include <memory>

typedef struct {
	bool is_enable;
	uint32_t extern_port_dev_can_id_min;
	uint32_t extern_port_dev_can_id_max;
	bool is_tx2master;
	bool is_tx2slave;
	bool is_tx2peripheral;
} AdapterDataT;

class CANFD_FORWARD_PROTOCOL
{
      public:
	CANFD_FORWARD_PROTOCOL() = default;
	~CANFD_FORWARD_PROTOCOL() = default;
	CANFD_FORWARD_PROTOCOL(const CANFD_FORWARD_PROTOCOL &) = delete;
	CANFD_FORWARD_PROTOCOL &operator=(const CANFD_FORWARD_PROTOCOL &) = delete;

      private:
	static std::unique_ptr<CANFD_FORWARD_PROTOCOL> Instance;
	AdapterDataT adapter_data2master;
	AdapterDataT adapter_data2slave;
};

#endif // __CANFD_PROTOCOL_HPP__
