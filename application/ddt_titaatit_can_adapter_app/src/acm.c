#include "acm.h"
#include "protocol.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

const struct device *const acm_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
LOG_MODULE_REGISTER(acm, CONFIG_TITA_ADAPTER_LOG_LEVEL);

K_KERNEL_STACK_DEFINE(deal_work_q_stack, 2048);

static void interrupt_handler(const struct device *dev, void *user_data)
{
	struct adapter_data_t *data = user_data;
	struct acm_data_t *acm_data = &data->user_acm;
	uint32_t recv_len = 0, send_len, rb_len;
	int ret;
	uint8_t *buffer;
	uint8_t drop_data;

	/* get all of the data off UART as fast as we can */
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {

			if (ring_buf_is_empty(&acm_data->rx_ringbuf)) {
				ring_buf_reset(&acm_data->rx_ringbuf);
			}

			rb_len = ring_buf_put_claim(&acm_data->rx_ringbuf, &buffer,
						    CONFIG_USB_CDC_ACM_RINGBUF_SIZE);

			if (rb_len == 0) {
				LOG_ERR("acm rx ringbuf is full, drop data");
				ret = ring_buf_put_finish(&acm_data->rx_ringbuf, recv_len);
				if (ret != 0) {
					// put finish error
				}

				// drop all fifo data
				while (uart_fifo_read(dev, &drop_data, sizeof(drop_data)) == 0)
					;

				continue;
			}
			recv_len = uart_fifo_read(dev, buffer, sizeof(recv_len));
			acm_data->dbg_recv_bytes += recv_len;

			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			if (recv_len == rb_len) {
				// try again read fifo
			}

			ret = ring_buf_put_finish(&acm_data->rx_ringbuf, recv_len);
			if (ret != 0) {
				// put finish error
			}

			if (recv_len) {
				// to send can work queue
				k_work_submit_to_queue(&acm_data->deal_work_q,
						       &acm_data->deal_work);
				// k_work_schedule_for_queue(&acm_data->deal_work_q,
				// 			  &acm_data->deal_work, K_NO_WAIT);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			rb_len = ring_buf_get_claim(&acm_data->tx_ringbuf, &buffer,
						    CONFIG_USB_CDC_ACM_RINGBUF_SIZE);
			if (!rb_len) {
				ring_buf_get_finish(&acm_data->tx_ringbuf, 0);
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			ring_buf_get_finish(&acm_data->tx_ringbuf, rb_len);

			acm_data->ring_buf_space = ring_buf_space_get(&acm_data->tx_ringbuf);
			LOG_DBG("ringbuf -> acm fifo %d bytes", send_len);
		}
	}
}

static void acm_data_deal_work_handler(struct k_work *work)
{
	struct adapter_data_t *adapter;
	struct acm_data_t *acm_data;
	acm_data = CONTAINER_OF(work, struct acm_data_t, deal_work);
	adapter = CONTAINER_OF(acm_data, struct adapter_data_t, user_acm);

	uint8_t *rb_buf = 0;
	uint32_t rb_len;
	static int temp = 0;
	rb_len =
		ring_buf_get_claim(&acm_data->rx_ringbuf, &rb_buf, CONFIG_USB_CDC_ACM_RINGBUF_SIZE);
	for (int i = 0; i < rb_len; i++) {
		if (decodec_pack(&acm_data->decodec_data, rb_buf[i]) == true) {
			LOG_INF("cmd:%d", ((uint16_t)(acm_data->decodec_data.RxPackBuffer[4])
					   << 8) | acm_data->decodec_data.RxPackBuffer[3]);
			encodec_pack(adapter, 0x55, (uint8_t *)&temp, sizeof(temp));
			temp++;
		}
	}

	ring_buf_get_finish(&acm_data->rx_ringbuf, rb_len);
}

void acm_init(struct adapter_data_t *adapter_data)
{
	adapter_data->user_acm.dev = acm_dev;

	decodec_init(&adapter_data->user_acm.decodec_data);

	k_work_queue_init(&adapter_data->user_acm.deal_work_q);
	k_work_queue_start(&adapter_data->user_acm.deal_work_q, deal_work_q_stack,
			   K_KERNEL_STACK_SIZEOF(deal_work_q_stack), 0, NULL);
	k_thread_name_set(&adapter_data->user_acm.deal_work_q.thread, "deal_work_q_stack");
	k_work_init(&adapter_data->user_acm.deal_work, acm_data_deal_work_handler);

	ring_buf_init(&adapter_data->user_acm.rx_ringbuf, CONFIG_USB_CDC_ACM_RINGBUF_SIZE,
		      adapter_data->user_acm.rx_buf);
	ring_buf_init(&adapter_data->user_acm.tx_ringbuf, CONFIG_USB_CDC_ACM_RINGBUF_SIZE,
		      adapter_data->user_acm.tx_buf);

	adapter_data->user_acm.ring_buf_size =
		ring_buf_capacity_get(&adapter_data->user_acm.tx_ringbuf);

	int ret = usb_enable(0);
	if (ret != 0) {
		LOG_ERR("failed to enable usb");
		return;
	}

	uart_irq_callback_user_data_set(acm_dev, interrupt_handler, (void *)adapter_data);

	uart_irq_rx_enable(acm_dev);
}