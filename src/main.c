/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

int read_uart_most_recent(uint8_t *msg, int msg_size);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MSG_SIZE	32
/* queue to store up to 10 messages */
K_MSGQ_DEFINE(modem_msgq, MSG_SIZE, 10, 4);

// receive buffer for modem's UART
static char uart1_rx_buf[MSG_SIZE] = {0};
static char uart0_rx_buf[MSG_SIZE] = {0};
static char printout[MSG_SIZE] = {0};

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Use UART1 to communicate with modem
static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
static const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));

const struct uart_config uart_cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

void uart_write(const struct device *dev, char *buf);

static void uart1_cb(const struct device *dev, struct uart_event *event, void *user_data) {
	switch (event->type) {
		case UART_RX_RDY:
			// received data is ready for prcessing
			printk("uart1: %s", (char *)event->data.rx_buf.buf);
			break;

		// Continuous reception is not enabled by default, which means once the receive buffer is full, 
		// you must manually re-enable UART to enable reception
		case UART_RX_DISABLED:
			uart_rx_enable(dev, uart1_rx_buf, sizeof(uart1_rx_buf), 100);
			break;
		
		default:
			break;
	}
}

static void uart0_cb(const struct device *dev, struct uart_event *event, void *user_data) {
	switch (event->type) {
		case UART_RX_RDY:
			uart_tx(uart1, event->data.rx.buf + event->data.rx.offset, event->data.rx.len, 100);
			// snprintk(printout, event->data.rx.len, "%s", event->data.rx.buf);
			// uart_write(uart1, printout);
			break;

		// Continuous reception is not enabled by default, which means once the receive buffer is full, 
		// you must manually re-enable UART to enable reception
		case UART_RX_DISABLED:
			uart_rx_enable(dev, uart1_rx_buf, sizeof(uart1_rx_buf), 100);
			break;
		
		default:
			break;
	}
}

void uart_write(const struct device *dev, char *buf) {
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

	if (!device_is_ready(uart1)) {
		return 0;
	}

	printk("Configuring UART\r\n");
	err = uart_configure(uart1, &uart_cfg);
	if (err) {
		printk("uart_configure() failed. Error: %d\r\n", err);
		return err;
	}

	err = uart_callback_set(uart1, uart1_cb, NULL);
	if (err) {
		printk("Cannot register uart callback. Error: %d\r\n", err);
		return err;
	}
	printk("UART1 callback registered\r\n");

	err = uart_callback_set(uart0, uart0_cb, NULL);
	if (err) {
		printk("Cannot register uart0 callback. Error: %d\r\n", err);
	}

	err = uart_rx_enable(uart1, uart1_rx_buf, sizeof(uart1_rx_buf), 100);
	if (err) {
		printk("UART1 RX failed to enable. Error: %d\r\n", err);
		return err;
	}
	printk("UART1 RX enabled\r\n");

	err = uart_rx_enable(uart0, uart0_rx_buf, sizeof(uart0_rx_buf), 100);
	if (err) {
		printk("UART0 RX failed to enable. Error: %d\r\n", err);
		return err;
	}
	printk("UART0 RX enabled\r\n");

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}
	printk("UART0 (Zephyr Shell) says hello\r\n");
	uart_write(uart1, "UART1 says hello\r\n");

	k_yield();
	return 0;
}
