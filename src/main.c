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
			// LOG_INF("rx_buf: %s\r\n", (char *)event->data.rx_buf.buf);

			size_t length = event->data.rx.len;
			size_t offset = event->data.rx.offset;
			char s[MSG_SIZE];
			memcpy(s, event->data.rx_buf.buf + offset, length);
			s[event->data.rx.len] = '\0';

			// remove new lines
			for (size_t i = 0; i < event->data.rx.len; i++) {
				if (s[i] == '\n' || s[i] == '\r') {
					s[i] = '\0';
					break;
				}
			}

			k_msgq_put(&modem_msgq, &s, K_NO_WAIT);

			LOG_INF("msg: %s\r\n", s);
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

void modem_read(char *buf) {
	
}

void modem_write(const struct device *dev, char *buf) {
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

	modem_write(modem, "Device started.\r\n");

	char received_modem_buf[MSG_SIZE];
	while (k_msgq_get(&modem_msgq, &received_modem_buf, K_FOREVER) == 0) {
		LOG_INF("q: %s\r\n", received_modem_buf);
	}
	return 0;
}
