/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/sys/ring_buffer.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define MSG_SIZE	64

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// Use UART1 to communicate with modem
static const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
static const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));

// receive buffer for modem's UART
static K_MEM_SLAB_DEFINE(uart1_rx_buf, MSG_SIZE, 3, 4);
static K_MEM_SLAB_DEFINE(uart0_rx_buf, MSG_SIZE, 3, 4);

static char uart1_rx_msg[MSG_SIZE];
static int uart1_rx_msg_pos;
static char uart0_rx_msg[MSG_SIZE];
static int uart0_rx_msg_pos;

K_MSGQ_DEFINE(uart0_tx_msgq, MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(uart1_tx_msgq, MSG_SIZE, 10, 4);

void uart_write(const struct device *dev, char *buf);

const struct uart_config uart_cfg = {
	.baudrate = 115200,
	.parity = UART_CFG_PARITY_NONE,
	.stop_bits = UART_CFG_STOP_BITS_1,
	.data_bits = UART_CFG_DATA_BITS_8,
	.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

void uart0_tx_thread(void) {
	char tx_msg[MSG_SIZE] = {0};
	while (1) {
		if (k_msgq_get(&uart0_tx_msgq, &tx_msg, K_NO_WAIT) == 0) {
			uart_write(uart0, tx_msg);
			k_yield();
		}
	}
}

void uart1_tx_thread(void) {
	char tx_msg[MSG_SIZE] = {0};
	while (1) {
		if (k_msgq_get(&uart1_tx_msgq, &tx_msg, K_NO_WAIT) == 0) {
			uart_write(uart1, tx_msg);
			k_yield();
		}
	}
}

K_THREAD_DEFINE(uart0_tx, 1024, uart0_tx_thread, NULL, NULL, NULL, 1, 0, 0);
K_THREAD_DEFINE(uart1_tx, 1024, uart1_tx_thread, NULL, NULL, NULL, 1, 0, 0);


static void uart1_cb(const struct device *dev, struct uart_event *event, void *user_data) {
	char rx_msg[MSG_SIZE] = {0};
	int err;

	switch (event->type) {
		case UART_TX_DONE:
			printk("UART1 TX: %s. Length: %d\r\n", event->data.tx.buf, event->data.tx.len);
			break;

		case UART_RX_RDY:
			snprintk(rx_msg, event->data.rx.len + 1, "%s\r\n", event->data.rx.buf + event->data.rx.offset);
			while (k_msgq_put(&uart0_tx_msgq, rx_msg, K_NO_WAIT) != 0) {
				k_msgq_purge(&uart0_tx_msgq);
			}
			
			break;

		case UART_RX_BUF_REQUEST:
		{
			uint8_t *buf;

			err = k_mem_slab_alloc(&uart1_rx_buf, (void **)&buf, K_NO_WAIT);
			if (err) {
				printk("Failed to allocate slab");
				return;
			}

			err = uart_rx_buf_rsp(uart1, buf, MSG_SIZE);
			if (err) {
				printk("Failed to provide new buffer");
				return;
			}
			break;
		}

		case UART_RX_BUF_RELEASED:
			k_mem_slab_free(&uart1_rx_buf, (void **)&event->data.rx_buf.buf);
			break;
		
		default:
			break;
	}
}

static void uart0_cb(const struct device *dev, struct uart_event *event, void *user_data) {
	char rx_msg[MSG_SIZE] = {0};
	int err;

	switch (event->type) {
		case UART_RX_RDY:
			snprintk(rx_msg, event->data.rx.len + 1, "%s\r\n", event->data.rx.buf + event->data.rx.offset);
			while (k_msgq_put(&uart1_tx_msgq, rx_msg, K_NO_WAIT) != 0) {
				k_msgq_purge(&uart1_tx_msgq);
			}
			break;

		case UART_RX_BUF_REQUEST:
		{
			uint8_t *buf;

			err = k_mem_slab_alloc(&uart0_rx_buf, (void **)&buf, K_NO_WAIT);
			if (err) {
				printk("Failed to allocate slab");
				return;
			}

			err = uart_rx_buf_rsp(uart0, buf, MSG_SIZE);
			if (err) {
				printk("Failed to provide new buffer");
				return;
			}
			break;
		}

		case UART_RX_BUF_RELEASED:
			k_mem_slab_free(&uart1_rx_buf, (void **)&event->data.rx_buf.buf);
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

	uint8_t *buf;
	err = k_mem_slab_alloc(&uart1_rx_buf, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");

	err = uart_rx_enable(uart1, buf, sizeof(uart1_rx_buf), 100);
	if (err) {
		printk("UART1 RX failed to enable. Error: %d\r\n", err);
		return err;
	}
	// uart_write(uart0, "UART1 RX enabled\r\n");

	err = k_mem_slab_alloc(&uart0_rx_buf, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");
	err = uart_rx_enable(uart0, buf, sizeof(uart0_rx_buf), 100);
	if (err) {
		printk("UART0 RX failed to enable. Error: %d\r\n", err);
		return err;
	}
	printk("UART0 RX enabled\r\n");

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	uart_write(uart0, "UART0 says hello\r\n");
	uart_write(uart1, "AT+CFUN?\r\n");
	
	k_yield();
	return 0;
}
