/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>

#define I2C_DEV "I2C_0"

void main(void)
{
	const struct device *i2c_dev;
	uint8_t cnt = 0;

	printk("Starting i2c scanner...\n");

	i2c_dev = device_get_binding(I2C_DEV);
	if (i2c_dev == NULL)
	{
		printk("I2C: Device driver %s not found.\n", I2C_DEV);
		return;
	}

	for (uint8_t i = 4; i <= 0x77; i++)
	{
		struct i2c_msg msgs[1];
		uint8_t dst;

		/* Send the address to read from */
		msgs[0].buf   = &dst;
		msgs[0].len   = 0U;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

		if (i2c_transfer(i2c_dev, &msgs[0], 1, i) == 0)
		{
			printk("0x%2x FOUND\n", i);
			++cnt;
		}
	}

	printk("%u devices found on %s\n", cnt, I2C_DEV);
}
