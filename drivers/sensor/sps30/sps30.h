/*
 * Copyright (c) 2020 Bilal Wasim
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SPS30_SPS30_H_
#define ZEPHYR_DRIVERS_SENSOR_SPS30_SPS30_H_

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

struct sps30 {
	struct device *i2c_dev;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_SPS30_SPS30_H_ */
