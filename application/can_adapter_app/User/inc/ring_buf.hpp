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

#ifndef __RING_BUF_HPP__
#define __RING_BUF_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <cstring>

#define RING_BUF_SIZE 10

class RING_BUF
{
      public:
	void ring_buf_init(bool overwrite_flag, bool multi_thread_flag);
	int get_push_data_count();
	int get_free_data_count();
	int get_max_data_count();
	int push_data(uint8_t *data, int data_len);
	int pop_data(uint8_t *data, int data_len);

      private:
	typedef struct {
		bool is_allow_overwrite; // if true, it will overwrite the old data
		bool is_multi_thread; // if true, it will use mutex/flags to protect the ring buffer

		uint8_t buf_array[RING_BUF_SIZE];
		uint8_t head_ptr;
		uint8_t tail_ptr;
		bool headptr_overloop_flag;

		uint8_t push_data_count;
		uint8_t free_data_count;
		uint8_t max_data_count;
		bool mutex_flags;
	} RING_BUF_T;
	RING_BUF_T ring_buf;
};

#ifdef __cplusplus
}
#endif

#endif // __RING_BUF_HPP__