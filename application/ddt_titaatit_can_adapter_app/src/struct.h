#pragma once

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>

struct ddt_protocol {
	int RxPackIndex;
	int RxPackEnd;
	uint16_t RxPackCheck;
	uint8_t RxPackBuffer[CONFIG_USB_CDC_ACM_RINGBUF_SIZE];

	bool RxPackHand;
	bool RxPackHandMaybe;
	uint32_t RcvOkPack;
	uint32_t RcvErrPack;
};

struct canfd_router_t {
	bool is_enable;
	uint16_t rang_canid_min;
	uint16_t rang_canid_max;
	bool is_tx2master;
	bool is_tx2slave;
	bool is_tx2peripheral;
	int16_t offset_master;
	int16_t offset_slave;
	int16_t offset_peripheral;
};

struct canfd_info_t {
	uint32_t rx_cnt;
	uint32_t rx_router_cnt;
	uint32_t rx2master_cnt;
	uint32_t rx2slave_cnt;
	uint32_t rx2peripheral_cnt;
	uint32_t rx2master_drop_cnt;
	uint32_t rx2slave_drop_cnt;
	uint32_t rx2peripheral_drop_cnt;
	uint32_t tx_cnt;
	uint32_t tx_err_cnt;
	uint32_t heartbeat_drop_cnt;
	uint32_t ring_buf_size;
	uint32_t ring_buf_space;
	enum can_state state;
	struct can_bus_err_cnt state_err_cnt;
};

struct canfd_data_t {
	const struct device *dev;

	uint8_t tx_buf[sizeof(struct can_frame) * 100];
	struct ring_buf tx_ringbuf;
	struct k_sem tx_sem;
	struct k_work_q tx_work_q;
	struct k_work tx_work;
	k_thread_stack_t *tx_work_stack;
	int tx_work_stack_size;
	struct canfd_router_t router_tb[20];

	struct canfd_info_t info;
};

struct acm_data_t {
	const struct device *dev;
	uint8_t rx_buf[CONFIG_USB_CDC_ACM_RINGBUF_SIZE];
	uint8_t tx_buf[CONFIG_USB_CDC_ACM_RINGBUF_SIZE];
	struct ring_buf rx_ringbuf;
	struct ring_buf tx_ringbuf;
	struct k_work_q deal_work_q;
	struct k_work deal_work;
	struct ddt_protocol decodec_data;
	uint32_t pkg_tx_err_count;

	uint32_t ring_buf_size;
	uint32_t ring_buf_space;
	uint32_t dbg_recv_bytes;
};
struct adapter_data_t {
	struct acm_data_t user_acm;
	struct canfd_data_t master_canfd;
	struct canfd_data_t slave_canfd;
	struct canfd_data_t peripheral_canfd;

	struct k_timer heartbeat_timer;
	uint32_t heartbeat_cnt;
};