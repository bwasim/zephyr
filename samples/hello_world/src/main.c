/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <assert.h>
#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <drivers/hwinfo.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>
#include <kernel.h>

static const struct device   *uart_dev;
static struct k_sem    uart_sem;
static struct ring_buf uart_rx_rb;

/* Buffer to receive data from STM32. */
#define UART_COMM_BUFF_SIZE        1024
static uint8_t          uart_isr_buff[UART_COMM_BUFF_SIZE];
static uint8_t          uart_ring_buff[UART_COMM_BUFF_SIZE];

/* UART thread to process received data. */
K_THREAD_STACK_DEFINE(stack_area_uart, 2048);
struct k_thread thread_data_uart;

/* Func: uart_callback_handler
 * Desc: This function is called whenever an interrupt (and data) is
 *       received from the Modem. */
static void uart_callback_handler(const struct device *uart_dev, void *user_data)
{
	int rx = 0, ret;

	/* get all of the data off UART as fast as we can */
	while (uart_irq_update(uart_dev) &&
	       uart_irq_rx_ready(uart_dev))
	{
		rx = uart_fifo_read(uart_dev,
				uart_isr_buff, UART_COMM_BUFF_SIZE);
		if (rx <= 0)
			continue;

		ret = ring_buf_put(&uart_rx_rb, uart_isr_buff, rx);
		if (ret != rx)
		{
			printk("Rx buffer doesn't have enough space. "
				"Bytes pending: %d, written: %d",
				rx, ret);
			/* modem_iface_uart_flush(&ctx->iface); */
			k_sem_give(&uart_sem);
			break;
		}

		k_sem_give(&uart_sem);
	}
}

/* Func: uart_processor
 * Desc: This function processes data received from the STM32. */
static void uart_processor(void)
{
	while (true)
	{
		char c;

		/* Wait for incoming data */
		k_sem_take(&uart_sem, K_FOREVER);

		/* Read only 1 byte from the ring buffer. */
		while (ring_buf_get(&uart_rx_rb, &c, 1))
		{
			printk("%c", c);
		}

		/* give up time if we have a solid stream of data */
		k_yield();
	}
}

/* This function will enable the modem. */
static void enable_modem(void)
{
	printk("Setting Modem Pins");

	/* NOTE: Per the BG95 document, the Reset pin is internally connected to the
	 * Power key pin. */

	k_sleep(K_SECONDS(2));

	/* MDM_POWER -> 1 for 500-1000 msec. */
	gpio_pin_set(device_get_binding("GPIO_1"), 5, 1);
	k_sleep(K_MSEC(750));

	/* MDM_POWER -> 0 and wait for ~2secs as UART remains in "inactive" state
	 * for some time after the power signal is enabled. */
	gpio_pin_set(device_get_binding("GPIO_1"), 5, 0);
	k_sleep(K_SECONDS(2));

	printk("... Done!");
}

/* Func: setup_uart
 * Desc: This function will setup the UART so it can communicate with Modem.
 *       We will make UART work in interrupt mode so it can detect data from
 *       Modem async (save CPU cycles) and tell the Zephyr kernel to invoke a
 *       callback whenever we receive something. When the callback is invoked,
 *       we process the data and spit it out on the screen.
 */
static int setup_uart(void)
{
#define UART_DEV_NAME "UART_1"

	char c;

	/* Get UART device */
	uart_dev = device_get_binding(UART_DEV_NAME);
	if (uart_dev == NULL)
	{
		return -ENODEV;
	}

	/* Disable TX / RX interrupts to allow setup. */
	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	/* Flush all data from the interface. */
	while (uart_fifo_read(uart_dev, &c, 1) > 0)
	{
		continue;
	}

	/* Setup custom callback and Enable RX interrupts. */
	uart_irq_callback_set(uart_dev, uart_callback_handler);
	uart_irq_rx_enable(uart_dev);

	/* Stuff necessary to make things work. */
	k_sem_init(&uart_sem, 0, 1);
	ring_buf_init(&uart_rx_rb, 1024, uart_ring_buff);

	/* Success. */
	return 0;
}

void main(void)
{
	enable_modem();
	setup_uart();

	/* UART processing thread. */
	(void) k_thread_create(&thread_data_uart, stack_area_uart, K_THREAD_STACK_SIZEOF(stack_area_uart),
							(k_thread_entry_t) (uart_processor), NULL, NULL, NULL, 3, 0, K_NO_WAIT);

	/* Get data from "UART-0" and send it to UART1. */
	while (1)
	{
		uart_poll_out(uart_dev, getchar());
	}
}
