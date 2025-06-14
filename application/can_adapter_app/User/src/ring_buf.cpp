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

void RING_BUF::ring_buf_init(bool overwrite_flag, bool multi_thread_flag)
{
	this->ring_buf.is_allow_overwrite = overwrite_flag;
	this->ring_buf.is_multi_thread = multi_thread_flag;
	memset(this->ring_buf.buf_array, 0x00, sizeof(this->ring_buf.buf_array));
	this->ring_buf.head_ptr = 0;
	this->ring_buf.tail_ptr = 0;
	this->ring_buf.headptr_overloop_flag = false;
	this->ring_buf.push_data_count = 0;
	this->ring_buf.free_data_count = RING_BUF_SIZE;
	this->ring_buf.max_data_count = RING_BUF_SIZE;
	this->ring_buf.mutex_flags = false;
}

int RING_BUF::get_push_data_count()
{
	return this->ring_buf.push_data_count;
}

int RING_BUF::get_free_data_count()
{
	return this->ring_buf.free_data_count;
}

int RING_BUF::get_max_data_count()
{
	return this->ring_buf.max_data_count;
}

int RING_BUF::push_data(uint8_t *data, int data_len)
{

	if (data_len > this->ring_buf.max_data_count) {
		return -1;
	}

	if (this->ring_buf.headptr_overloop_flag == true) {
		return -1;
	}

	// check ring buffer is full
	if (this->ring_buf.is_allow_overwrite == false) {

		if (this->ring_buf.free_data_count < data_len) {
			return -1;
		}

		// push data in ring buffer
		memcpy((this->ring_buf.buf_array + this->ring_buf.head_ptr), &data, data_len);

		this->ring_buf.head_ptr += data_len;
		if (this->ring_buf.head_ptr >= this->ring_buf.max_data_count) {
			this->ring_buf.headptr_overloop_flag = true;
			this->ring_buf.head_ptr -= this->ring_buf.max_data_count;
		}

		if (this->ring_buf.headptr_overloop_flag) {
			this->ring_buf.push_data_count = this->ring_buf.head_ptr +
							 this->ring_buf.max_data_count -
							 this->ring_buf.tail_ptr;
			this->ring_buf.free_data_count =
				this->ring_buf.max_data_count - this->ring_buf.push_data_count;
			this->ring_buf.headptr_overloop_flag = false;
		} else {
			this->ring_buf.push_data_count =
				this->ring_buf.head_ptr - this->ring_buf.tail_ptr;
			this->ring_buf.free_data_count =
				this->ring_buf.max_data_count - this->ring_buf.push_data_count;
		}

		return data_len;
	} else if (this->ring_buf.is_allow_overwrite == true) {

		if (this->ring_buf.free_data_count < data_len) {
			int remain_data = data_len - (this->ring_buf.free_data_count);

			// push data in ring buffer
			memcpy((this->ring_buf.buf_array + this->ring_buf.head_ptr), &data,
			       this->ring_buf.free_data_count);

			this->ring_buf.head_ptr += this->ring_buf.free_data_count;

			memcpy((this->ring_buf.buf_array + this->ring_buf.head_ptr),
			       (&data + this->ring_buf.free_data_count), remain_data);

			this->ring_buf.head_ptr += remain_data;
			if (this->ring_buf.head_ptr >= this->ring_buf.max_data_count) {
				this->ring_buf.head_ptr -= this->ring_buf.max_data_count;
			}

			this->ring_buf.tail_ptr += remain_data;
			if (this->ring_buf.tail_ptr >= this->ring_buf.max_data_count) {
				this->ring_buf.tail_ptr -= this->ring_buf.max_data_count;
			}

			this->ring_buf.push_data_count = this->ring_buf.max_data_count;
			this->ring_buf.free_data_count = 0;

			return data_len;
		} else {
			memcpy((this->ring_buf.buf_array + this->ring_buf.head_ptr), &data,
			       data_len);

			this->ring_buf.head_ptr += data_len;
			if (this->ring_buf.head_ptr >= this->ring_buf.max_data_count) {
				this->ring_buf.headptr_overloop_flag = true;
				this->ring_buf.head_ptr -= this->ring_buf.max_data_count;
			}

			if (this->ring_buf.headptr_overloop_flag) {
				this->ring_buf.push_data_count = this->ring_buf.head_ptr +
								 this->ring_buf.max_data_count -
								 this->ring_buf.tail_ptr;
				this->ring_buf.free_data_count = this->ring_buf.max_data_count -
								 this->ring_buf.push_data_count;
				this->ring_buf.headptr_overloop_flag = false;
			} else {
				this->ring_buf.push_data_count =
					this->ring_buf.head_ptr - this->ring_buf.tail_ptr;
				this->ring_buf.free_data_count = this->ring_buf.max_data_count -
								 this->ring_buf.push_data_count;
			}

			return data_len;
		}
	}

	return -1;
}

int RING_BUF::pop_data(uint8_t *data, int data_len)
{
	if (data_len > this->ring_buf.push_data_count) {
		return -1;
	}

	if (this->ring_buf.headptr_overloop_flag == true) {
		return -1;
	}

	memcpy(&data, (this->ring_buf.buf_array + this->ring_buf.tail_ptr), data_len);

	this->ring_buf.tail_ptr += data_len;
	if (this->ring_buf.tail_ptr >= this->ring_buf.max_data_count) {
		this->ring_buf.tail_ptr -= this->ring_buf.max_data_count;
	}

	this->ring_buf.push_data_count = this->ring_buf.head_ptr - this->ring_buf.tail_ptr;
	this->ring_buf.free_data_count =
		this->ring_buf.max_data_count - this->ring_buf.push_data_count;

	return data_len;
}
