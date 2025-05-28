
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(can, CONFIG_TITA_ADAPTER_LOG_LEVEL);

#include "boot.h"
#include "can.h"
#include "protocol.h"

K_KERNEL_STACK_DEFINE(m_master_tx_work_stack, 2048);
K_KERNEL_STACK_DEFINE(m_slave_tx_work_stack, 2048);
K_KERNEL_STACK_DEFINE(m_peripheral_tx_work_stack, 2048);

static const struct device *const m_master_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_master_canfd));
static const struct device *const m_slave_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_slave_canfd));
static const struct device *const m_peripheral_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_peripheral_canfd));

const char *state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

static inline struct canfd_data_t *get_inst_with_ptr(struct adapter_data_t *adapter,
						     const struct device *dev)
{
	struct canfd_data_t *inst = NULL;

	if (dev == adapter->master_canfd.dev) {
		inst = &adapter->master_canfd;
	} else if (dev == adapter->slave_canfd.dev) {
		inst = &adapter->slave_canfd;
	} else if (dev == adapter->peripheral_canfd.dev) {
		inst = &adapter->peripheral_canfd;
	}

	return inst;
}

static void state_change_callback(const struct device *dev, enum can_state state,
				  struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct adapter_data_t *adapter = (struct adapter_data_t *)user_data;
	struct canfd_data_t *inst = get_inst_with_ptr(adapter, dev);

	if (inst) {
		inst->info.state = state;
		memcpy((void *)&inst->info.state_err_cnt, (void *)&err_cnt,
		       sizeof(struct can_bus_err_cnt));
	}

	if (state == CAN_STATE_BUS_OFF) {

		if (can_recover(dev, K_MSEC(100)) != 0) {
			LOG_INF("Recovery timed out\n");
		}
	}
}

static inline void rx2other(struct canfd_data_t *src, struct canfd_data_t *dest,
			    struct can_frame *frame)
{
	uint8_t *buffer;
	uint32_t rb_len;

	rb_len = ring_buf_put_claim(&dest->tx_ringbuf, &buffer, sizeof(struct can_frame));
	if (rb_len == 0) {
		ring_buf_put_finish(&dest->tx_ringbuf, 0);
		if (dest->dev == m_master_dev) {
			src->info.rx2master_drop_cnt++;
		} else if (dest->dev == m_slave_dev) {
			src->info.rx2slave_drop_cnt++;
		} else if (dest->dev == m_peripheral_dev) {
			src->info.rx2peripheral_drop_cnt++;
		}
	} else {
		if (dest->dev == m_master_dev) {
			src->info.rx2master_cnt++;
		} else if (dest->dev == m_slave_dev) {
			src->info.rx2slave_cnt++;
		} else if (dest->dev == m_peripheral_dev) {
			src->info.rx2peripheral_cnt++;
		}
		memcpy((void *)buffer, (void *)frame, sizeof(struct can_frame));
		ring_buf_put_finish(&dest->tx_ringbuf, sizeof(struct can_frame));
		k_work_submit_to_queue(&dest->tx_work_q, &dest->tx_work);
	}
}

static void canfd_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct adapter_data_t *adapter = (struct adapter_data_t *)user_data;
	struct canfd_data_t *inst = get_inst_with_ptr(adapter, dev);

	if (!inst) {
		return;
	}
	if (inst == &adapter->master_canfd && (frame->flags & CAN_FRAME_IDE)) {

		if (frame->id == 0xB149) {
			jump_to_boot();
		}
	}

	inst->info.rx_cnt++;
#define IS_RANGE(v, min, max) ((v) >= (min) && (v) <= (max)) ? 1 : 0

	for (int i = 0; i < sizeof(inst->router_tb) / sizeof(inst->router_tb[0]); i++) {
		if (inst->router_tb[i].is_enable == true &&
		    IS_RANGE(frame->id, inst->router_tb[i].rang_canid_min,
			     inst->router_tb[i].rang_canid_max)) {
			if (inst->router_tb[i].is_tx2master) {
				frame->id += inst->router_tb[i].offset_master;
				rx2other(inst, &adapter->master_canfd, frame);
			}
			if (inst->router_tb[i].is_tx2slave) {
				frame->id += inst->router_tb[i].offset_slave;
				rx2other(inst, &adapter->slave_canfd, frame);
			}
			if (inst->router_tb[i].is_tx2peripheral) {
				frame->id += inst->router_tb[i].offset_peripheral;
				rx2other(inst, &adapter->peripheral_canfd, frame);
			}
			inst->info.rx_router_cnt++;
			break;
		}
	}
}

static inline void heartbeat(struct canfd_data_t *dest, int mode, int heartbeat_cnt)
{
	struct can_frame *frame;
	uint32_t rb_len;

	if (dest->info.state != CAN_STATE_ERROR_ACTIVE) {
		// LOG_INF("dev:%s %s", dest->dev->name, state_to_str(dest->info.state));
		ring_buf_reset(&dest->tx_ringbuf);
	}
	if (dest->info.state == CAN_STATE_ERROR_PASSIVE) {
		// LOG_INF("dev:%s %s", dest->dev->name, state_to_str(dest->info.state));
		can_stop(dest->dev);
		can_start(dest->dev);
	}

	dest->info.ring_buf_space = ring_buf_space_get(&dest->tx_ringbuf);

	rb_len =
		ring_buf_put_claim(&dest->tx_ringbuf, (uint8_t **)&frame, sizeof(struct can_frame));
	if (rb_len == 0) {
		ring_buf_put_finish(&dest->tx_ringbuf, 0);
		dest->info.heartbeat_drop_cnt++;
	} else {
		// frame->id = 0x381;
		frame->id = 0x9F;
		frame->flags = CAN_FRAME_FDF;
		frame->dlc = can_bytes_to_dlc(12);
		frame->data_32[0] = 0;
		frame->data_32[1] = mode;
		frame->data_32[2] = heartbeat_cnt;
		ring_buf_put_finish(&dest->tx_ringbuf, sizeof(struct can_frame));
		k_work_submit_to_queue(&dest->tx_work_q, &dest->tx_work);
	}
}

static void heartbeat_timer_cb(struct k_timer *timer_id)
{
	struct adapter_data_t *adapter =
		CONTAINER_OF(timer_id, struct adapter_data_t, heartbeat_timer);

	adapter->heartbeat_cnt++;

#define MODE_MASTER	1
#define MODE_SLAVE	2
#define MODE_PERIPHERAL 3

	heartbeat(&adapter->master_canfd, MODE_MASTER, adapter->heartbeat_cnt);
	heartbeat(&adapter->slave_canfd, MODE_SLAVE, adapter->heartbeat_cnt);
	heartbeat(&adapter->peripheral_canfd, MODE_PERIPHERAL, adapter->heartbeat_cnt);

	// encodec_pack(adapter, 0x1234, 0, 0);
	// encodec_pack(adapter, 0x2234, 0, 0);
	// encodec_pack(adapter, 0x3234, 0, 0);
	encodec_pack(adapter, 0x1, (uint8_t *)&adapter->master_canfd.info,
		     sizeof(struct canfd_info_t));
	encodec_pack(adapter, 0x2, (uint8_t *)&adapter->slave_canfd.info,
		     sizeof(struct canfd_info_t));
	encodec_pack(adapter, 0x3, (uint8_t *)&adapter->peripheral_canfd.info,
		     sizeof(struct canfd_info_t));
	encodec_pack(adapter, 0x4, (uint8_t *)&adapter->heartbeat_cnt,
		     sizeof(adapter->heartbeat_cnt));
}

static void tx_work_handler(struct k_work *work)
{
	struct can_frame *frame;
	struct canfd_data_t *inst = CONTAINER_OF(work, struct canfd_data_t, tx_work);

	while (0 != ring_buf_get_claim(&inst->tx_ringbuf, (uint8_t **)&frame,
				       sizeof(struct can_frame))) {

		inst->info.tx_cnt++;
		if (0 != can_send(inst->dev, frame, K_NO_WAIT, NULL, NULL)) {
			inst->info.tx_err_cnt++;
		}

		ring_buf_get_finish(&inst->tx_ringbuf, sizeof(struct can_frame));
	}
}

static inline void __can_init(struct canfd_data_t *inst, void *user_data)
{
	can_mode_t mode = CAN_MODE_NORMAL | CAN_MODE_FD;
	uint32_t rate;
	char thread_name[50];
	struct can_filter filter_stdid_fd = {
		.id = 0x000, .mask = 0x000, .flags = CAN_FILTER_DATA | CAN_FILTER_FDF};
	struct can_filter filter_extid_fd = {.id = 0x000,
					     .mask = 0x000,
					     .flags = CAN_FILTER_DATA | CAN_FILTER_FDF |
						      CAN_FILTER_IDE};
	k_sem_init(&inst->tx_sem, 0, 1);
	ring_buf_init(&inst->tx_ringbuf, sizeof(inst->tx_buf), inst->tx_buf);

	inst->info.ring_buf_size = ring_buf_capacity_get(&inst->tx_ringbuf);

	k_work_queue_init(&inst->tx_work_q);
	k_work_queue_start(&inst->tx_work_q, inst->tx_work_stack, inst->tx_work_stack_size, 0,
			   NULL);
	snprintf(thread_name, sizeof(thread_name), "%s tx_work_q", inst->dev->name);
	k_thread_name_set(&inst->tx_work_q.thread, thread_name);
	k_work_init(&inst->tx_work, tx_work_handler);

	can_get_core_clock(inst->dev, &rate);
	LOG_INF("%s clock: %u", inst->dev->name, rate);

	can_set_mode(inst->dev, mode);
	can_set_state_change_callback(inst->dev, state_change_callback, user_data);
	can_start(inst->dev);

	can_add_rx_filter(inst->dev, canfd_rx_callback, user_data, &filter_stdid_fd);
	can_add_rx_filter(inst->dev, canfd_rx_callback, user_data, &filter_extid_fd);
}

void can_init(struct adapter_data_t *adapter_data)
{

	adapter_data->master_canfd.dev = m_master_dev;
	adapter_data->slave_canfd.dev = m_slave_dev;
	adapter_data->peripheral_canfd.dev = m_peripheral_dev;

	adapter_data->master_canfd.tx_work_stack = m_master_tx_work_stack;
	adapter_data->slave_canfd.tx_work_stack = m_slave_tx_work_stack;
	adapter_data->peripheral_canfd.tx_work_stack = m_peripheral_tx_work_stack;

	adapter_data->master_canfd.tx_work_stack_size =
		K_THREAD_STACK_SIZEOF(m_master_tx_work_stack);
	adapter_data->slave_canfd.tx_work_stack_size = K_THREAD_STACK_SIZEOF(m_slave_tx_work_stack);
	adapter_data->peripheral_canfd.tx_work_stack_size =
		K_THREAD_STACK_SIZEOF(m_peripheral_tx_work_stack);

	__can_init(&adapter_data->master_canfd, (void *)adapter_data);
	__can_init(&adapter_data->slave_canfd, (void *)adapter_data);
	__can_init(&adapter_data->peripheral_canfd, (void *)adapter_data);

	k_timer_init(&adapter_data->heartbeat_timer, heartbeat_timer_cb, NULL);
	k_timer_start(&adapter_data->heartbeat_timer, K_SECONDS(1), K_SECONDS(1));
}