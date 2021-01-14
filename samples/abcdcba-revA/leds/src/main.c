/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

typedef struct _gpio_struct {
	const char   *led;
	gpio_pin_t    pin;
	gpio_flags_t  flags;
} gpio_info;

#define SLEEP_TIME_MS   1000
#define TOTAL_LEDS 		8

#define LED0_NODE		DT_ALIAS(led0)
#define LED1_NODE		DT_ALIAS(led1)
#define LED2_NODE		DT_ALIAS(led2)
#define LED3_NODE		DT_ALIAS(led3)
#define LED4_NODE		DT_ALIAS(led4)
#define LED5_NODE		DT_ALIAS(led5)
#define LED6_NODE		DT_ALIAS(led6)
#define LED7_NODE		DT_ALIAS(led7)

static gpio_info info[TOTAL_LEDS] = {
		{.led = DT_GPIO_LABEL(LED0_NODE, gpios), .pin = DT_GPIO_PIN(LED0_NODE, gpios), .flags = DT_GPIO_FLAGS(LED0_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED1_NODE, gpios), .pin = DT_GPIO_PIN(LED1_NODE, gpios), .flags = DT_GPIO_FLAGS(LED1_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED2_NODE, gpios), .pin = DT_GPIO_PIN(LED2_NODE, gpios), .flags = DT_GPIO_FLAGS(LED2_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED3_NODE, gpios), .pin = DT_GPIO_PIN(LED3_NODE, gpios), .flags = DT_GPIO_FLAGS(LED3_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED4_NODE, gpios), .pin = DT_GPIO_PIN(LED4_NODE, gpios), .flags = DT_GPIO_FLAGS(LED4_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED5_NODE, gpios), .pin = DT_GPIO_PIN(LED5_NODE, gpios), .flags = DT_GPIO_FLAGS(LED5_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED6_NODE, gpios), .pin = DT_GPIO_PIN(LED6_NODE, gpios), .flags = DT_GPIO_FLAGS(LED6_NODE, gpios)},
		{.led = DT_GPIO_LABEL(LED7_NODE, gpios), .pin = DT_GPIO_PIN(LED7_NODE, gpios), .flags = DT_GPIO_FLAGS(LED7_NODE, gpios)},
};

void main(void)
{
	const struct device *dev[TOTAL_LEDS];
	bool led_is_on = true;
	int ret, i;

	for (i = 0; i < TOTAL_LEDS; i ++)
	{
		dev[i] = device_get_binding(info[i].led);
		if (dev[i] == NULL)
			return;

		ret = gpio_pin_configure(dev[i], info[i].pin, GPIO_OUTPUT_ACTIVE | info[i].flags);
		if (ret < 0)
			return;
	}

	while (1)
	{
		for (i = 0; i < TOTAL_LEDS; i ++)
		{
			gpio_pin_set(dev[i], info[i].pin, (int)led_is_on);
			led_is_on = !led_is_on;
		}

		k_msleep(SLEEP_TIME_MS);
	}
}
