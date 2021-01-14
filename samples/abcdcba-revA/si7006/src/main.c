/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/sensor.h>

const static struct device *temp_dev;

int temp_sensor_init(void)
{
	temp_dev = device_get_binding("SI7006");
	if (!temp_dev)
	{
		printf("error: no temp device\n");
		return -1;
	}

	return 0;
}

double sensor_value_get(enum sensor_channel val)
{
	int r;
	struct sensor_value temp_value;

	if (temp_dev == NULL)
	{
		printk("temp_dev is NULL, Exiting..\n");
		return -1;
	}

	r = sensor_sample_fetch(temp_dev);
	if (r)
	{
		printk("sensor_sample_fetch failed return: %d\n", r);
		return -1;
	}

	r = sensor_channel_get(temp_dev, val, &temp_value);
	if (r)
	{
		printk("sensor_channel_get failed return: %d\n", r);
		return -1;
	}

	return sensor_value_to_double(&temp_value);
}

void main(void)
{
	if (temp_sensor_init() != 0)
	{
		printk("%s: Unable to setup temp sensor, Exiting !! \r\n", __func__);
		return;
	}

	while (true)
	{
		printf("%s: Temp Value: %f \r\n", __func__, sensor_value_get(SENSOR_CHAN_AMBIENT_TEMP));
		printf("%s: Humidity Value: %f \r\n", __func__, sensor_value_get(SENSOR_CHAN_HUMIDITY));
		k_sleep(K_SECONDS(1));
	}
}
