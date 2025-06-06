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

#ifndef __FSM_HPP__
#define __FSM_HPP__

#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/smf.h>

#include "can.hpp"
#include "canfd_forward_protocol.hpp"
#include "timer.hpp"
#include <memory>

#define DEFAULT_RATE  10
#define DIRECT_RATE   500
#define WORK_RATE     1000
#define SELFTEST_RATE 100
#define READY_RATE    10
#define FAULT_RATE    5
#define SLEEP_RATE    1

enum fsm_state_t {
	FSM_INIT_STATE = 0x00,
	FSM_DATA_FORWARD_PROCESS_STATE,
	FSM_SLEEP_STATE,
};

typedef struct {
	struct smf_ctx ctx;
} fsm_todo_list_t;

typedef struct {
	float dt;	    // 25us - 100ms
	uint32_t frequency; // 10Hz - 40kHz
	uint32_t timestamp; // accuracy 1us

	// uint64_t cyclestamp
	uint32_t last_hw_cycle, interval_hw_cycle, overflow_count;
	uint32_t cycle_count;
} FSM_FREQ_T;

typedef struct {
	struct k_timer fsm_timer;
	FSM_FREQ_T fsm_freq;
} fsm_device_t;

struct fsm_work_t {
	struct k_work work;
	fsm_todo_list_t fsm_todo_list;
	fsm_device_t fsm_device;
};

class FSM
{
      public:
	FSM() = default;
	~FSM() = default;
	FSM(const FSM &) = delete;
	FSM &operator=(const FSM &) = delete;
	static std::shared_ptr<FSM> getInstance();
	enum fsm_state_t fsm_get_state(fsm_todo_list_t *fsm_todo_list);
	void fsm_init(const enum fsm_state_t state);
	void fsm_set_frequency(uint32_t frequency);

	static void fsm_init_entry(void *obj);
	static void fsm_init_run(void *obj);
	static void fsm_init_exit(void *obj);
	static void fsm_data_forward_process_entry(void *obj);
	static void fsm_data_forward_process_run(void *obj);
	static void fsm_sleep_entry(void *obj);
	static void fsm_sleep_run(void *obj);
	static void fsm_sleep_exit(void *obj);

      private:
	static std::shared_ptr<FSM> Instance;
	static std::shared_ptr<fsm_work_t> fsm_work;

	std::unique_ptr<TIMER> timer_driver_handle = TIMER::getInstance();
	std::shared_ptr<CAN> can_driver_handle = CAN::getInstance();
	std::unique_ptr<CANFD_FORWARD_PROTOCOL> canfd_forward_protocol_handle =
		CANFD_FORWARD_PROTOCOL::getInstance();

	void device_timing_freq_process(std::shared_ptr<FSM> fsm_handle,
					struct fsm_work_t *fsm_work);
	bool hardware_init();
	bool pre_init();
	static void fsm_timer_callback(struct k_timer *timer_id);
	void set_fsm_state(std::shared_ptr<fsm_work_t> fsm_work, const enum fsm_state_t state);
	static void fsm_handle(struct k_work *work);
};

#endif // __FSM_HPP__