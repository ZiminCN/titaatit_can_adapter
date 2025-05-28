
#include "protocol.h"

#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(tita_protocol, CONFIG_TITA_ADAPTER_LOG_LEVEL);

bool decodec_pack(struct ddt_protocol *decodec_data, unsigned char new_data)
{
	unsigned short packCheck;
	bool ret;
	if (decodec_data->RxPackHand) {
		if (2 == decodec_data->RxPackIndex) {
			decodec_data->RxPackEnd = new_data;
		}

		if (decodec_data->RxPackIndex < sizeof(decodec_data->RxPackBuffer)) {
			decodec_data->RxPackBuffer[decodec_data->RxPackIndex] = new_data;
			decodec_data->RxPackCheck += new_data;
			decodec_data->RxPackIndex++;

			if (decodec_data->RxPackIndex >= decodec_data->RxPackEnd &&
			    decodec_data->RxPackEnd > 4) {
				packCheck =
					*((unsigned short *)&decodec_data
						  ->RxPackBuffer[decodec_data->RxPackIndex - 2]);

				decodec_data->RxPackCheck -=
					decodec_data->RxPackBuffer[decodec_data->RxPackIndex - 2];
				decodec_data->RxPackCheck -=
					decodec_data->RxPackBuffer[decodec_data->RxPackIndex - 1];
				decodec_data->RxPackCheck ^= 0xFFFF;
				LOG_INF("RxPackCheck:%04x packCheck:%04x",
					decodec_data->RxPackCheck, packCheck);
				if (decodec_data->RxPackCheck == packCheck) {
					decodec_data->RcvOkPack++;
					ret = true;
				} else {
					decodec_data->RcvErrPack++;
					ret = false;
				}

				decodec_data->RxPackIndex = 0;
				decodec_data->RxPackHand = false;
				decodec_data->RxPackHandMaybe = false;
				return ret;
			}
		} else {
			decodec_data->RxPackIndex = 0;
			decodec_data->RxPackHand = false;
			decodec_data->RxPackHandMaybe = false;
		}
	} else {
		if (new_data == 0x55) {
			decodec_data->RxPackHandMaybe = true;
		} else {
			if ((new_data == 0xAA) && (decodec_data->RxPackHandMaybe)) {
				decodec_data->RxPackBuffer[0] = 0x55;
				decodec_data->RxPackBuffer[1] = 0xAA;
				decodec_data->RxPackHand = true;
				decodec_data->RxPackIndex = 2;
				decodec_data->RxPackCheck = 0;
			}
			decodec_data->RxPackHandMaybe = false;
		}
	}

	return false;
}

void decodec_init(struct ddt_protocol *decodec_data)
{
	memset((void *)decodec_data, 0, sizeof(struct ddt_protocol));
}

uint32_t encodec_pack(struct adapter_data_t *adapter, uint16_t cmd, uint8_t *data, int len)
{

	// struct canfd_data_t *master_canfd = &adapter->master_canfd;
	// struct canfd_data_t *slave_canfd = &adapter->slave_canfd;
	// struct canfd_data_t *peripheral_canfd = &adapter->peripheral_canfd;
	struct acm_data_t *acm = &adapter->user_acm;
	uint32_t rb_len, ret;
	uint8_t *buffer;
	int pack_len = 2 + 1 + 2 + len + 2;
	uint16_t check_xor;

	rb_len = ring_buf_put_claim(&acm->tx_ringbuf, &buffer, pack_len);
	acm->ring_buf_space = ring_buf_space_get(&acm->tx_ringbuf);

	if (rb_len == 0 || pack_len > rb_len) {
		ring_buf_put_finish(&acm->tx_ringbuf, 0);
		ring_buf_reset(&acm->tx_ringbuf);

		acm->pkg_tx_err_count++;
		return -1;
	}

	buffer[0] = 0x55;
	buffer[1] = 0xaa;
	buffer[2] = pack_len;
	buffer[3] = cmd;
	buffer[4] = cmd >> 8;
	check_xor = buffer[2] + buffer[3] + buffer[4];
	for (rb_len = 0; rb_len < len; rb_len++) {
		buffer[5 + rb_len] = data[rb_len];
		check_xor += data[rb_len];
	}
	check_xor ^= 0xffff;
	buffer[5 + rb_len++] = check_xor;
	buffer[5 + rb_len] = check_xor >> 8;

	ret = ring_buf_put_finish(&acm->tx_ringbuf, pack_len);

	uart_irq_tx_enable(acm->dev);
	// k_sem_give(&acm->tx_sem);
	return ret;
}
