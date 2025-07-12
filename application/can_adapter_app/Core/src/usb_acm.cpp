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

#include "usb_acm.hpp"

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/usb/bos.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <string>

LOG_MODULE_REGISTER(usb_acm, LOG_LEVEL_INF);

K_SEM_DEFINE(dtr_sem, 0, 1);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer_array[RING_BUF_SIZE];

/* By default, do not register the USB DFU class DFU mode instance. */
static const char *const blocklist[] = {
	"dfu_dfu",
	NULL,
};

struct ring_buf usb_cdc_acm_ringbuf;

const struct device *const usb_cdc_acm_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

#define ZEPHYR_PROJECT_USB_VID 0x2fe3
#define USBD_PID	       0x000b
#define USBD_MANUFACTURER      "Direct Drive Technology Co., Ltd."
#define USBD_PRODUCT	       "DDT Four-wheel-legged robot CANFD relay equipment"
// USBD_MAX_POWER: range 0 250(mA), default 125mA
#define USBD_MAX_POWER	       125
#define USBD_SELF_POWERED      true
#define USBD_REMOTE_WAKEUP     true

USBD_DEVICE_DEFINE(usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), ZEPHYR_PROJECT_USB_VID,
		   USBD_PID);
USBD_DESC_LANG_DEFINE(usbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_mfr, USBD_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(usbd_product, USBD_PRODUCT);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(usbd_sn)));

/* doc configuration instantiation start */
static const uint8_t attributes = (IS_ENABLED(USBD_SELF_POWERED) ? USB_SCD_SELF_POWERED : 0) |
				  (IS_ENABLED(USBD_REMOTE_WAKEUP) ? USB_SCD_REMOTE_WAKEUP : 0);

/*
 * This does not yet provide valuable information, but rather serves as an
 * example, and will be improved in the future.
 */
static const struct usb_bos_capability_lpm bos_cap_lpm = {
	.bLength = sizeof(struct usb_bos_capability_lpm),
	.bDescriptorType = USB_DESC_DEVICE_CAPABILITY,
	.bDevCapabilityType = USB_BOS_CAPABILITY_EXTENSION,
	.bmAttributes = 0UL,
};
USBD_DESC_BOS_DEFINE(usbext, sizeof(bos_cap_lpm), &bos_cap_lpm);

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(hs_cfg_desc, "HS Configuration");
/* Full speed configuration */
USBD_CONFIGURATION_DEFINE(usbd_fs_config, attributes, USBD_MAX_POWER, &fs_cfg_desc);

/* High speed configuration */
USBD_CONFIGURATION_DEFINE(usbd_hs_config, attributes, USBD_MAX_POWER, &hs_cfg_desc);

std::unique_ptr<USB_ACM> USB_ACM::Instance = std::make_unique<USB_ACM>();

std::unique_ptr<USB_ACM> USB_ACM::getInstance()
{
	return std::move(USB_ACM::Instance);
}

USB_ACM &USB_ACM::getInstancePtr()
{
	if (!USB_ACM::Instance) {
		USB_ACM::Instance = std::make_unique<USB_ACM>();
	}

	return *USB_ACM::Instance;
}

int USB_ACM::usb_cdc_acm_init()
{
	int ret;

	if (!device_is_ready(usb_cdc_acm_dev)) {
		LOG_ERR("USB CDC ACM device not ready");
		return 0;
	}

	ret = enable_usb_device_next();

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	ring_buf_init(&usb_cdc_acm_ringbuf, sizeof(ring_buffer_array), ring_buffer_array);

	LOG_INF("Wait for DTR");

	k_sem_take(&dtr_sem, K_FOREVER);

	LOG_INF("DTR set");

	ret = uart_line_ctrl_set(usb_cdc_acm_dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(usb_cdc_acm_dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	k_sleep(K_MSEC(100));

	print_baudrate(usb_cdc_acm_dev);

	uart_irq_callback_set(usb_cdc_acm_dev, this->interrupt_handler);

	uart_irq_rx_enable(usb_cdc_acm_dev);

	return 0;
}

inline void USB_ACM::print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate: %d", baudrate);
	}
}

void USB_ACM::usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_CONTROL_LINE_STATE) {
		uint32_t dtr = 0U;

		uart_line_ctrl_get(msg->dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			k_sem_give(&dtr_sem);
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		print_baudrate(msg->dev);
	}
}

void USB_ACM::fix_code_triple(struct usbd_context *uds_ctx, const enum usbd_speed speed)
{
	/* Always use class code information from Interface Descriptors */
	if (IS_ENABLED(CONFIG_USBD_CDC_ACM_CLASS) || IS_ENABLED(CONFIG_USBD_CDC_ECM_CLASS) ||
	    IS_ENABLED(CONFIG_USBD_CDC_NCM_CLASS) || IS_ENABLED(CONFIG_USBD_MIDI2_CLASS) ||
	    IS_ENABLED(CONFIG_USBD_AUDIO2_CLASS)) {
		/*
		 * Class with multiple interfaces have an Interface
		 * Association Descriptor available, use an appropriate triple
		 * to indicate it.
		 */
		usbd_device_set_code_triple(uds_ctx, speed, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	} else {
		usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
	}
}

struct usbd_context *USB_ACM::usbd_setup_device(usbd_msg_cb_t msg_cb)
{

	int err;

	/* doc add string descriptor start */
	err = usbd_add_descriptor(&usbd, &usbd_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return NULL;
	}

	err = usbd_add_descriptor(&usbd, &usbd_mfr);
	if (err) {
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return NULL;
	}

	err = usbd_add_descriptor(&usbd, &usbd_product);
	if (err) {
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return NULL;
	}

	IF_ENABLED(CONFIG_HWINFO, (err = usbd_add_descriptor(&usbd, &usbd_sn);))
	if (err) {
		LOG_ERR("Failed to initialize SN descriptor (%d)", err);
		return NULL;
	}
	/* doc add string descriptor end */

	if (USBD_SUPPORTS_HIGH_SPEED && usbd_caps_speed(&usbd) == USBD_SPEED_HS) {
		err = usbd_add_configuration(&usbd, USBD_SPEED_HS, &usbd_hs_config);
		if (err) {
			LOG_ERR("Failed to add High-Speed configuration");
			return NULL;
		}

		err = usbd_register_all_classes(&usbd, USBD_SPEED_HS, 1, blocklist);
		if (err) {
			LOG_ERR("Failed to add register classes");
			return NULL;
		}

		fix_code_triple(&usbd, USBD_SPEED_HS);
	}

	/* doc configuration register start */
	err = usbd_add_configuration(&usbd, USBD_SPEED_FS, &usbd_fs_config);
	if (err) {
		LOG_ERR("Failed to add Full-Speed configuration");
		return NULL;
	}
	/* doc configuration register end */

	/* doc functions register start */
	err = usbd_register_all_classes(&usbd, USBD_SPEED_FS, 1, blocklist);
	if (err) {
		LOG_ERR("Failed to add register classes");
		return NULL;
	}
	/* doc functions register end */

	fix_code_triple(&usbd, USBD_SPEED_FS);
	usbd_self_powered(&usbd, attributes & USB_SCD_SELF_POWERED);

	if (msg_cb != NULL) {
		/* doc device init-and-msg start */
		err = usbd_msg_register_cb(&usbd, msg_cb);
		if (err) {
			LOG_ERR("Failed to register message callback");
			return NULL;
		}
		/* doc device init-and-msg end */
	}

	if (IS_ENABLED(CONFIG_SAMPLE_USBD_20_EXTENSION_DESC)) {
		(void)usbd_device_set_bcd_usb(&usbd, USBD_SPEED_FS, 0x0201);
		(void)usbd_device_set_bcd_usb(&usbd, USBD_SPEED_HS, 0x0201);

		err = usbd_add_descriptor(&usbd, &usbext);
		if (err) {
			LOG_ERR("Failed to add USB 2.0 Extension Descriptor");
			return NULL;
		}
	}

	return &usbd;
}

struct usbd_context *USB_ACM::usbd_init_device(usbd_msg_cb_t msg_cb)
{
	int err;

	if (usbd_setup_device(msg_cb) == NULL) {
		return NULL;
	}

	err = usbd_init(&usbd);
	if (err) {
		LOG_ERR("Failed to initialize USB device support.");
		return NULL;
	}

	return &usbd;
}

int USB_ACM::enable_usb_device_next(void)
{
	int err;
	usbd_context *usbd_impl = usbd_init_device(usbd_msg_cb);
	if (!usbd_impl) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(usbd_impl)) {
		err = usbd_enable(usbd_impl);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");

	return 0;
}

void USB_ACM::interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	USB_ACM &usb_acm_handle = USB_ACM::getInstancePtr();

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!usb_acm_handle.rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&usb_cdc_acm_ringbuf), sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				usb_acm_handle.rx_throttled = true;
				continue;
			}

			recv_len = uart_fifo_read(dev, ring_buffer_array, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&usb_cdc_acm_ringbuf, ring_buffer_array, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&usb_cdc_acm_ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			if (usb_acm_handle.rx_throttled) {
				uart_irq_rx_enable(dev);
				usb_acm_handle.rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}
