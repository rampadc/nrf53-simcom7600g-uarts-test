/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

int read_uart_most_recent(uint8_t *msg, int msg_size);

LOG_MODULE_REGISTER(nrf53_uart_async, LOG_LEVEL_DBG);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MSG_SIZE	32
/* queue to store up to 10 messages */
K_MSGQ_DEFINE(modem_msgq, MSG_SIZE, 10, 4);

// receive buffer for modem's UART
static char rx_buf[MSG_SIZE] = {0};
static int rx_buf_index = 0;

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Use UART1 to communicate with modem
static const struct device *modem = DEVICE_DT_GET(DT_NODELABEL(uart1));
const struct uart_config uart_cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

static void modem_uart_cb(const struct device *dev, struct uart_event *event, void *user_data) {
	switch (event->type) {
		case UART_RX_RDY:
			// received data is ready for prcessing
			LOG_INF("===== Bytes received: %d\r\n", event->data.rx.len);
			LOG_INF("offset: %d\r\n", event->data.rx.offset);
			LOG_INF("rx_buf: %s\r\n", (char *)event->data.rx_buf.buf);

			char received_msg[MSG_SIZE] = {0};

			int characters_read = 0;
			int result_index = 0;
			for (int i = event->data.rx.offset; i < MSG_SIZE; i++) {
				if (characters_read >= event->data.rx.len) {
					break;
				}

				char current_char = event->data.rx.buf[i];

				if (current_char == '\n' || current_char == '\r') {
					received_msg[result_index] = '\0';
					// reset index
					result_index = 0;
					break;
				}

				received_msg[result_index++] = current_char;
			}
			LOG_INF("msg: %s\r\n", (char *)received_msg);
			break;

		// Continuous reception is not enabled by default, which means once the receive buffer is full, 
		// you must manually re-enable UART to enable reception
		case UART_RX_DISABLED:
			uart_rx_enable(dev, rx_buf, sizeof(rx_buf), 100);
			break;
		
		default:
			break;
	}
}

void print_uart(const struct device *dev, char *buf) {
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_tx(dev, buf, msg_len, SYS_FOREVER_US);
	}
}


int main(void)
{
	int ret;
	int err;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	if (!device_is_ready(modem)) {
		return 0;
	}

	LOG_INF("Configuring UART\r\n");
	err = uart_configure(modem, &uart_cfg);
	if (err) {
		LOG_ERR("uart_configure() failed. Error: %d\r\n", err);
		return err;
	}

	err = uart_callback_set(modem, modem_uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot register uart callback. Error: %d\r\n", err);
		return err;
	}
	LOG_INF("UART callback registered\r\n");

	err = uart_rx_enable(modem, rx_buf, sizeof(rx_buf), 100);
	if (err) {
		LOG_ERR("UART RX failed to enable. Error: %d\r\n", err);
		return err;
	}
	LOG_INF("UART RX enabled\r\n");

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	print_uart(modem, "Device started.\r\n");

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
