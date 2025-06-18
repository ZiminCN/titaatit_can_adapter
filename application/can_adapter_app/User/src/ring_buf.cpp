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

#include "ring_buf.hpp"

#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ring_buf, LOG_LEVEL_INF);

bool RING_BUF::ring_buf_init(bool overwrite_flag, bool multi_thread_flag, uint16_t buf_size)
{
	this->ring_buf.state = RING_BUFFER_NORMAL;
	this->ring_buf.config.is_allow_overwrite = overwrite_flag;
	this->ring_buf.core.write_index = 0;
	this->ring_buf.core.read_index = 0;
	this->ring_buf.core.buffer_size = buf_size;
	this->ring_buf.param.mutex_flag = 0;
	this->ring_buf.param.write_data_count = 0;
	this->ring_buf.param.free_data_count = buf_size;
	void *new_ptr = realloc(this->ring_buf.core.buffer_ptr, this->ring_buf.core.buffer_size);
	if (new_ptr == nullptr) {
		LOG_ERR("ring_buf.core.buffer_ptr is null");
		return false;
	}
	this->ring_buf.core.buffer_ptr = static_cast<uint8_t *>(new_ptr);
	memset(this->ring_buf.core.buffer_ptr, 0x00, this->ring_buf.core.buffer_size);

	this->output_ring_buf_data();
	return true;
}

int RING_BUF::get_max_data_count()
{
	return (this->ring_buf.core.buffer_size);
}

RING_BUF_STATE_E RING_BUF::get_ring_buf_state()
{
	this->is_ring_buffer_full();
	this->is_ring_buffer_empty();
	return this->ring_buf.state;
}

RING_BUF_STATE_E RING_BUF::is_ring_buffer_empty(void)
{

	// if(this->ring_buf.core.write_index == this->ring_buf.core.read_index){
	// 	LOG_INF("ring buffer is empty");
	// 	return RING_BUFFER_EMPTY;
	// }

	if (this->ring_buf.param.free_data_count == this->ring_buf.core.buffer_size) {
		LOG_INF("ring buffer is empty");
		return RING_BUFFER_EMPTY;
	}

	return RING_BUFFER_NORMAL;
}

RING_BUF_STATE_E RING_BUF::is_ring_buffer_full(void)
{

	// if((this->ring_buf.core.write_index + 1)%(this->ring_buf.core.buffer_size) ==
	// this->ring_buf.core.read_index){ 	LOG_INF("ring buffer is full"); 	return
	// RING_BUFFER_FULL;
	// }

	if (this->ring_buf.param.write_data_count == this->ring_buf.core.buffer_size) {
		LOG_INF("ring buffer is full");
		return RING_BUFFER_FULL;
	}

	return RING_BUFFER_NORMAL;
}

int RING_BUF::write_data(uint8_t *data, uint16_t data_len)
{
	// mutex flag
	if (this->ring_buf.param.mutex_flag == 1) {
		return -1;
	}

	this->ring_buf.param.mutex_flag = 1;

	this->ring_buf.state = this->is_ring_buffer_full();

	if (this->ring_buf.param.free_data_count < data_len) {
		LOG_INF("[Error] No Enough Buffer Space to write data");
		this->ring_buf.param.mutex_flag = 0;
		return -1;
	}

	if (this->ring_buf.state == RING_BUFFER_FULL) {
		this->ring_buf.param.mutex_flag = 0;
		return -1;
	} else {
		uint16_t remaining_elements =
			this->ring_buf.core.buffer_size - this->ring_buf.core.write_index;
		if ((remaining_elements) < (data_len)) {
			memcpy((this->ring_buf.core.buffer_ptr + this->ring_buf.core.write_index),
			       data, remaining_elements);
			memcpy((this->ring_buf.core.buffer_ptr), data + remaining_elements,
			       data_len - remaining_elements);
		} else {
			memcpy((this->ring_buf.core.buffer_ptr + this->ring_buf.core.write_index),
			       data, data_len);
		}

		this->ring_buf.core.write_index += data_len;
		this->ring_buf.core.write_index =
			(this->ring_buf.core.write_index) % (this->ring_buf.core.buffer_size);

		this->ring_buf.param.write_data_count += data_len;
		this->ring_buf.param.free_data_count =
			this->ring_buf.core.buffer_size - this->ring_buf.param.write_data_count;

		this->ring_buf.param.mutex_flag = 0;
		return data_len;
	}

	this->ring_buf.param.mutex_flag = 0;
	return -1;
}

int RING_BUF::read_data(uint8_t *data, uint16_t data_len)
{
	// mutex flag
	if (this->ring_buf.param.mutex_flag == 1) {
		return -1;
	}

	this->ring_buf.param.mutex_flag = 1;

	this->ring_buf.state = this->is_ring_buffer_empty();

	if (this->ring_buf.param.write_data_count < data_len) {
		LOG_INF("[Error] No Enough Buffer Data to read");
		this->ring_buf.param.mutex_flag = 0;
		return -1;
	}

	if (this->ring_buf.state == RING_BUFFER_EMPTY) {
		this->ring_buf.param.mutex_flag = 0;
		return -1;
	} else {
		uint16_t remaining_elements =
			this->ring_buf.core.buffer_size - this->ring_buf.core.read_index;
		if ((remaining_elements) < (data_len)) {
			memcpy(data,
			       this->ring_buf.core.buffer_ptr + this->ring_buf.core.read_index,
			       remaining_elements);
			memcpy(data + remaining_elements, this->ring_buf.core.buffer_ptr,
			       data_len - remaining_elements);
		} else {
			memcpy(data,
			       this->ring_buf.core.buffer_ptr + this->ring_buf.core.read_index,
			       data_len);
		}

		// this->ring_buf.core.read_index += (this->ring_buf.core.read_index ==
		// 0)?(data_len):(data_len+1);
		this->ring_buf.core.read_index = (this->ring_buf.core.read_index + data_len) %
						 (this->ring_buf.core.buffer_size);

		this->ring_buf.param.free_data_count += data_len;
		this->ring_buf.param.write_data_count =
			this->ring_buf.core.buffer_size - this->ring_buf.param.free_data_count;

		this->ring_buf.param.mutex_flag = 0;
		return data_len;
	}

	this->ring_buf.param.mutex_flag = 0;
	return data_len;
}

void RING_BUF::output_ring_buf_data()
{

	for (int i = 0; i < this->ring_buf.core.buffer_size; i++) {
		LOG_INF("ring buffer[%d]: [%d]", i, this->ring_buf.core.buffer_ptr[i]);
	}
}

void RING_BUF::debug_LOG()
{
	LOG_INF("write index is [%d]", this->ring_buf.core.write_index);
	LOG_INF("read index is [%d]", this->ring_buf.core.read_index);

	LOG_INF("write data count is [%d]", this->ring_buf.param.write_data_count);
	LOG_INF("free data count is [%d]", this->ring_buf.param.free_data_count);
}
