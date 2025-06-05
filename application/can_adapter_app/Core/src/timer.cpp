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

#include "timer.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(timer, LOG_LEVEL_INF);

std::unique_ptr<TIMER> TIMER::Instance = std::make_unique<TIMER>();
std::unique_ptr<TIMER_FREQ_T> TIMER::timer_freq_cfg = std::make_unique<TIMER_FREQ_T>();
std::unique_ptr<TIMER_MANGEMENT_GPOUP_T> TIMER::timer_management_group =
	std::make_unique<TIMER_MANGEMENT_GPOUP_T>();

std::unique_ptr<TIMER> TIMER::getInstance()
{
	return std::move(TIMER::Instance);
}

void TIMER::timer_init(struct k_timer *timer, k_timer_expiry_t expiry_fn, k_timer_stop_t stop_fn)
{
	return k_timer_init(timer, expiry_fn, stop_fn);
}

void TIMER::timer_start(struct k_timer *timer, k_timeout_t duration, k_timeout_t period)
{
	return k_timer_start(timer, duration, period);
}

void TIMER::set_timer_freq_cfg_dt(float dt)
{
	this->timer_freq_cfg->dt = dt;
}

float TIMER::get_timer_freq_cfg_dt()
{
	return this->timer_freq_cfg->dt;
}

void TIMER::set_timer_freq_cfg_frequency(float frequency)
{
	this->timer_freq_cfg->frequency = frequency;
}

float TIMER::get_timer_freq_cfg_frequency()
{
	return this->timer_freq_cfg->frequency;
}

void TIMER::set_timer_freq_cfg_timestamp(uint32_t timestamp)
{
	this->timer_freq_cfg->timestamp = timestamp;
}

uint32_t TIMER::get_timer_freq_cfg_timestamp()
{
	return this->timer_freq_cfg->timestamp;
}

void TIMER::set_timer_freq_cfg_last_hw_cycle(uint32_t last_hw_cycle)
{
	this->timer_freq_cfg->last_hw_cycle = last_hw_cycle;
}

uint32_t TIMER::get_timer_freq_cfg_last_hw_cycle()
{
	return this->timer_freq_cfg->last_hw_cycle;
}

void TIMER::set_timer_freq_cfg_interval_hw_cycle(uint32_t interval_hw_cycle)
{
	this->timer_freq_cfg->interval_hw_cycle = interval_hw_cycle;
}

uint32_t TIMER::get_timer_freq_cfg_interval_hw_cycle()
{
	return this->timer_freq_cfg->interval_hw_cycle;
}

void TIMER::set_timer_freq_cfg_overflow_count(uint32_t overflow_count)
{
	this->timer_freq_cfg->overflow_count = overflow_count;
}

uint32_t TIMER::get_timer_freq_cfg_overflow_count()
{
	return this->timer_freq_cfg->overflow_count;
}

void TIMER::set_timer_freq_cfg_cycle_count(uint32_t cycle_count)
{
	this->timer_freq_cfg->cycle_count = cycle_count;
}

uint32_t TIMER::get_timer_freq_cfg_cycle_count()
{
	return this->timer_freq_cfg->cycle_count;
}

struct k_timer *TIMER::get_can_adapter_heartbeat_timer()
{
	return &(this->timer_management_group->can_adapter_heartbeat_timer);
}
