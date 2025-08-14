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

#include "fsm.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fsm, LOG_LEVEL_INF);

std::shared_ptr<FSM> FSM::Instance = std::make_unique<FSM>();
std::shared_ptr<fsm_work_t> FSM::fsm_work = std::make_unique<fsm_work_t>();
std::shared_ptr<FSM> FSM::getInstance()
{
	return FSM::Instance;
}

#define FSM_STACK_SIZE	   4096
#define FSM_WORKQ_PRIORITY 0
#define CYCLES_PER_SEC	   (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) // XXX HZ per second

struct k_work_q fsm_work_q;

K_THREAD_STACK_DEFINE(fsm_stack_area, FSM_STACK_SIZE);

void FSM::fsm_init_entry(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("FSM init entry");

	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();
	// fsm_driver_handle->mosfet_control_handle->set_48v_mosfet_state(GPIO_OUTPUT_ACTIVE);
	// timer init

	// timer start

	fsm_driver_handle->fsm_set_frequency(DEFAULT_RATE);
}

enum smf_state_result FSM::fsm_init_run(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("FSM init run");

	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();
	ARG_UNUSED(fsm_driver_handle);
	fsm_driver_handle->set_fsm_state(fsm_work, FSM_DATA_FORWARD_PROCESS_STATE);

	return SMF_EVENT_HANDLED;
}

void FSM::fsm_init_exit(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("FSM init exit");
}

void FSM::fsm_data_forward_process_entry(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("FSM data forward process entry");
	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();
	fsm_driver_handle->canfd_forward_protocol_handle->data2adapter_msgq_transmit_create_task();
	fsm_driver_handle->canfd_forward_protocol_handle->data2robot_msgq_transmit_create_task();
}

enum smf_state_result FSM::fsm_data_forward_process_run(void *obj)
{
	ARG_UNUSED(obj);
	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();

	fsm_driver_handle->canfd_forward_protocol_handle->heartbeat_pong_tong();

	int ret = fsm_driver_handle->canfd_forward_protocol_handle->is_detected_heartbeat();
	// control 48v mosfet on/off
	if (ret == HeartbeatDetectedStatusE::HEART_BEAT_DETECTED) {
		// fsm_driver_handle->mosfet_control_handle->set_48v_mosfet_state(GPIO_OUTPUT_ACTIVE);
	} else if (ret == HeartbeatDetectedStatusE::HEART_BEAT_LOST) {
		// fsm_driver_handle->mosfet_control_handle->set_48v_mosfet_state(GPIO_OUTPUT_INACTIVE);
	}

	return SMF_EVENT_HANDLED;
}

void FSM::fsm_data_forward_process_exit(void *obj)
{
	ARG_UNUSED(obj);
	LOG_INF("FSM data forward process exit");
	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();
	fsm_driver_handle->canfd_forward_protocol_handle->data2adapter_msgq_transmit_destory_task();
	fsm_driver_handle->canfd_forward_protocol_handle->data2robot_msgq_transmit_destory_task();
}

void FSM::fsm_sleep_entry(void *obj)
{
	ARG_UNUSED(obj);
}

enum smf_state_result FSM::fsm_sleep_run(void *obj)
{
	ARG_UNUSED(obj);

	return SMF_EVENT_HANDLED;
}

void FSM::fsm_sleep_exit(void *obj)
{
	ARG_UNUSED(obj);
}

static const struct smf_state fsm_states[] = {
	[FSM_INIT_STATE] = SMF_CREATE_STATE(FSM::fsm_init_entry, FSM::fsm_init_run,
					    FSM::fsm_init_exit, NULL, NULL),
	[FSM_DATA_FORWARD_PROCESS_STATE] = SMF_CREATE_STATE(
		FSM::fsm_data_forward_process_entry, FSM::fsm_data_forward_process_run,
		FSM::fsm_data_forward_process_exit, NULL, NULL),
	[FSM_SLEEP_STATE] = SMF_CREATE_STATE(FSM::fsm_sleep_entry, FSM::fsm_sleep_run,
					     FSM::fsm_sleep_exit, NULL, NULL)};

enum fsm_state_t FSM::fsm_get_state(fsm_todo_list_t *fsm_todo_list)
{
	return static_cast<fsm_state_t>(fsm_todo_list->ctx.current - &fsm_states[0]);
}

void FSM::device_timing_freq_process(std::shared_ptr<FSM> fsm_handle, struct fsm_work_t *fsm_work)
{

	uint32_t now, interval;
	uint64_t cycle64;
	float jitter;

	now = sys_clock_cycle_get_32();

	if (this->timer_driver_handle->get_timer_freq_cfg_last_hw_cycle() == 0) {
		// init state
		interval = static_cast<float>(CYCLES_PER_SEC);
	} else if (this->timer_driver_handle->get_timer_freq_cfg_last_hw_cycle() > now) {
		// overflow
		interval = (UINT32_MAX -
			    this->timer_driver_handle->get_timer_freq_cfg_last_hw_cycle()) +
			   now;
		this->timer_driver_handle->set_timer_freq_cfg_overflow_count(
			this->timer_driver_handle->get_timer_freq_cfg_overflow_count() + 1);
		fsm_work->fsm_device.fsm_freq.overflow_count =
			this->timer_driver_handle->get_timer_freq_cfg_overflow_count();
	} else {
		// normal
		interval = now - this->timer_driver_handle->get_timer_freq_cfg_last_hw_cycle();
	}

	this->timer_driver_handle->set_timer_freq_cfg_last_hw_cycle(now);
	fsm_work->fsm_device.fsm_freq.last_hw_cycle =
		this->timer_driver_handle->get_timer_freq_cfg_last_hw_cycle();

	fsm_work->fsm_device.fsm_freq.dt = interval / static_cast<float>(CYCLES_PER_SEC);
	this->timer_driver_handle->set_timer_freq_cfg_dt(fsm_work->fsm_device.fsm_freq.dt);
	fsm_work->fsm_device.fsm_freq.dt = this->timer_driver_handle->get_timer_freq_cfg_dt();

	int real_cycle_gap = static_cast<int>(
		interval -
		(CYCLES_PER_SEC / this->timer_driver_handle->get_timer_freq_cfg_frequency()));
	float theoretical_per_sec_cycle = static_cast<float>(
		CYCLES_PER_SEC / this->timer_driver_handle->get_timer_freq_cfg_frequency());
	jitter = 100 * (real_cycle_gap / theoretical_per_sec_cycle);

#if 0
    if (update_count % 500 == 1) {
        LOG_INF("dt: %+0.6f jitter: %+0.6f%%", static_cast<double>(this->timer_driver_handle->get_timer_freq_cfg_dt()), static_cast<double>(jitter));
    }
#else
	if (jitter > 10.0f) { // jitter > 10%, warning
		LOG_ERR("dt: %+0.6f jitter: %+0.6f%%",
			static_cast<double>(this->timer_driver_handle->get_timer_freq_cfg_dt()),
			static_cast<double>(jitter));
	}
#endif

	cycle64 = this->timer_driver_handle->get_timer_freq_cfg_overflow_count();
	cycle64 = cycle64 << 32 | now;
	this->timer_driver_handle->set_timer_freq_cfg_timestamp(cycle64 /
								(CYCLES_PER_SEC / 1000 / 1000));
	fsm_work->fsm_device.fsm_freq.timestamp =
		this->timer_driver_handle->get_timer_freq_cfg_timestamp();
}

bool FSM::hardware_init()
{
	bool ret;
	ret = this->can_driver_handle->init();
	if (!ret) {
		LOG_ERR("CAN driver init failed");
		return false;
	}

	return true;
}

bool FSM::pre_init()
{
	this->canfd_forward_protocol_handle->forward_protocol_init();
	// this->usb_acm_handle->usb_cdc_acm_init();
	// this->ring_buf_handle.ring_buf_init(false, false, 15);
	// this->mosfet_control_handle->init();
	return true;
}

void FSM::fsm_timer_callback(struct k_timer *timer_id)
{
	void *data = k_timer_user_data_get(timer_id);
	struct fsm_work_t *fsm_work = (struct fsm_work_t *)data;
	k_work_submit_to_queue(&fsm_work_q, &fsm_work->work);
}

void FSM::fsm_handle(struct k_work *work)
{
	struct fsm_work_t *fsm_work = CONTAINER_OF(work, struct fsm_work_t, work);

	std::shared_ptr<FSM> fsm_driver_handle = FSM::getInstance();

	fsm_driver_handle->device_timing_freq_process(fsm_driver_handle, fsm_work);

	smf_run_state(&fsm_driver_handle->fsm_work->fsm_todo_list.ctx);

	fsm_driver_handle->timer_driver_handle->set_timer_freq_cfg_cycle_count(
		fsm_driver_handle->timer_driver_handle->get_timer_freq_cfg_cycle_count() + 1);
	fsm_driver_handle->fsm_work->fsm_device.fsm_freq.cycle_count =
		fsm_driver_handle->timer_driver_handle->get_timer_freq_cfg_cycle_count();
}

void FSM::set_fsm_state(std::shared_ptr<fsm_work_t> fsm_work, const enum fsm_state_t state)
{
	smf_set_state(&this->fsm_work->fsm_todo_list.ctx, &fsm_states[state]);
}

void FSM::fsm_init(const enum fsm_state_t state)
{
	bool ret;
	const struct k_work_queue_config workq_cfg = {
		.name = "workq",
		.no_yield = false,
	};

	k_work_queue_init(&fsm_work_q);

	k_work_queue_start(&fsm_work_q, fsm_stack_area, K_THREAD_STACK_SIZEOF(fsm_stack_area),
			   FSM_WORKQ_PRIORITY, &workq_cfg);

	ret = this->hardware_init();
	if (!ret) {
		LOG_ERR("hardware_init failed");
	}

	ret = this->pre_init();
	if (!ret) {
		LOG_ERR("pre_init failed");
	}

	k_work_init(&this->fsm_work->work, fsm_handle);

	k_timer_init(&this->fsm_work->fsm_device.fsm_timer, FSM::fsm_timer_callback, NULL);

	k_timer_user_data_set(&this->fsm_work->fsm_device.fsm_timer, (void *)this->fsm_work.get());

	this->fsm_set_frequency(DEFAULT_RATE);

	smf_set_initial(&(this->fsm_work->fsm_todo_list.ctx), &fsm_states[state]);
}

void FSM::fsm_set_frequency(uint32_t frequency)
{
	this->timer_driver_handle->set_timer_freq_cfg_frequency(frequency);
	this->fsm_work->fsm_device.fsm_freq.frequency =
		this->timer_driver_handle->get_timer_freq_cfg_frequency();

	this->timer_driver_handle->set_timer_freq_cfg_cycle_count(0);
	this->fsm_work->fsm_device.fsm_freq.cycle_count =
		this->timer_driver_handle->get_timer_freq_cfg_cycle_count();

	LOG_DBG("ts: %08u %s: %d", this->timer_driver_handle->get_timer_freq_cfg_timestamp(),
		__func__, frequency);

	k_timer_start(
		&this->fsm_work->fsm_device.fsm_timer, K_NO_WAIT,
		K_USEC(1000000 / (this->timer_driver_handle->get_timer_freq_cfg_frequency())));
}
