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

#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

#include <memory>

typedef struct {
	float dt;	    // 25us - 100ms
	uint32_t frequency; // 10Hz - 40kHz
	uint32_t timestamp; // accuracy 1us

	// uint64_t cyclestamp
	uint32_t last_hw_cycle, interval_hw_cycle, overflow_count;
	uint32_t cycle_count;
} TIMER_FREQ_T;

typedef struct {
	struct k_timer can_adapter_heartbeat_timer;
} TIMER_MANGEMENT_GPOUP_T;

class TIMER
{
      public:
	TIMER() = default;
	~TIMER() = default;
	TIMER(const TIMER &) = delete;
	TIMER &operator=(const TIMER &) = delete;
	static std::unique_ptr<TIMER> getInstance();
	void timer_init(struct k_timer *timer, k_timer_expiry_t expiry_fn, k_timer_stop_t stop_fn);
	void timer_start(struct k_timer *timer, k_timeout_t duration, k_timeout_t period);
	void timer_stop(struct k_timer *timer);
	void set_timer_freq_cfg_dt(float dt);
	float get_timer_freq_cfg_dt();
	void set_timer_freq_cfg_frequency(float frequency);
	float get_timer_freq_cfg_frequency();
	void set_timer_freq_cfg_timestamp(uint32_t timestamp);
	uint32_t get_timer_freq_cfg_timestamp();
	void set_timer_freq_cfg_last_hw_cycle(uint32_t last_hw_cycle);
	uint32_t get_timer_freq_cfg_last_hw_cycle();
	void set_timer_freq_cfg_interval_hw_cycle(uint32_t hw_cycle);
	uint32_t get_timer_freq_cfg_interval_hw_cycle();
	void set_timer_freq_cfg_overflow_count(uint32_t overflow_count);
	uint32_t get_timer_freq_cfg_overflow_count();
	void set_timer_freq_cfg_cycle_count(uint32_t cycle_count);
	uint32_t get_timer_freq_cfg_cycle_count();
	struct k_timer *get_can_adapter_heartbeat_timer();

      private:
	static std::unique_ptr<TIMER> Instance;
	static std::unique_ptr<TIMER_FREQ_T> timer_freq_cfg;
	static std::unique_ptr<TIMER_MANGEMENT_GPOUP_T> timer_management_group;
};

#endif // __TIMER_HPP__