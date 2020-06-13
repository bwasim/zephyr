/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
	volatile float   a, b, c;
	volatile double  x, y, z;

	while (1)
	{
		printk("Hello World! %s\n", CONFIG_BOARD);
		k_sleep(K_SECONDS(1));

		a = sys_rand32_get() / 1.0, b = sys_rand32_get() / 2.0;
		c = a + b;

		x = 2.0, y = 5.0;
		z = x + y;
	}
}
