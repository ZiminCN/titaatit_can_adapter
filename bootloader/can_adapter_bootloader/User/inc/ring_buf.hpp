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

typedef enum {
	RING_BUFFER_NORMAL = 0,
	RING_BUFFER_EMPTY = 1,
	RING_BUFFER_FULL = 2,
} RING_BUF_STATE_E;

class RING_BUF
{
      public:
	bool ring_buf_init(bool overwrite_flag, bool multi_thread_flag, uint16_t buf_size);
	int get_max_data_count();
	RING_BUF_STATE_E get_ring_buf_state();
	int write_data(uint8_t *data, uint16_t data_len);
	int read_data(uint8_t *data, uint16_t data_len);
	void output_ring_buf_data();
	void debug_LOG();

      private:
	typedef struct {
		uint8_t *buffer_ptr;
		uint16_t write_index;
		uint16_t read_index;
		uint16_t buffer_size;
	} RING_BUF_CORE_T;

	typedef struct {
		bool mutex_flag;
		uint16_t write_data_count;
		uint16_t free_data_count;
	} RING_BUF_PARAM_T;

	typedef struct {
		bool is_allow_overwrite;
	} RING_BUF_CONFIG_T;

	typedef struct {
		RING_BUF_STATE_E state;
		RING_BUF_CORE_T core;
		RING_BUF_PARAM_T param;
		RING_BUF_CONFIG_T config;
	} RING_BUF_T;

	RING_BUF_STATE_E is_ring_buffer_empty(void);
	RING_BUF_STATE_E is_ring_buffer_full(void);

	RING_BUF_T ring_buf;
};

#ifdef __cplusplus
}
#endif

#endif // __RING_BUF_HPP__