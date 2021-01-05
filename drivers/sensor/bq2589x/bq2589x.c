/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_bq2589x

#include <drivers/i2c.h>
#include <init.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <string.h>
#include <sys/byteorder.h>

#include "bq2589x.h"

#if 0
static inline int bq2589x_read_byte(uint8_t reg, uint8_t *data)
{
	return i2c_reg_read_byte(i2c_dev, DT_INST_REG_ADDR(0), reg, data);
}

static inline int bq2589x_write_byte(uint8_t reg, uint8_t data)
{
	return i2c_reg_write_byte(i2c_dev, DT_INST_REG_ADDR(0), reg, data);
}

static inline int bq2589x_update_bits(uint8_t reg, uint8_t mask, uint8_t data)
{
	int ret;
	uint8_t tmp;

	ret = bq2589x_read_byte(reg, &tmp);
	if (ret != 0)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(reg, tmp);
}
#endif

/**
 * @brief Sensor value get
 *
 * @return -ENOTSUP for unsupported channels
 */
static int bq2589x_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	return 0;
}

static int bq2589x_sample_fetch(const struct device *dev,
					enum sensor_channel chan)
{
	return 0;
}

/**
 * @brief Initialize the Battery Controller
 *
 * @return 0 for success
 */
static int bq2589x_init(const struct device *dev)
{
	return 0;
}

static const struct sensor_driver_api bq2589x_battery_driver_api = {
	.sample_fetch = bq2589x_sample_fetch,
	.channel_get  = bq2589x_channel_get,
};

#define BQ2589X_INIT(index)                                                     \
	static struct bq2589x_data bq2589x_driver_##index;                          \
									                                            \
	static const struct bq2589x_config bq2589x_config_##index = {               \
		.bus_name       = DT_INST_BUS_LABEL(index),                             \
		.charge_voltage = DT_INST_PROP(index, charge_voltage),                  \
		.charge_current = DT_INST_PROP(index, charge_current),                  \
		.term_current   = DT_INST_PROP(index, term_current),                    \
	};                                                                          \
									                                            \
	DEVICE_DT_INST_DEFINE(index, &bq2589x_init, device_pm_control_nop,          \
			    &bq2589x_driver_##index,                                        \
			    &bq2589x_config_##index, POST_KERNEL,                           \
			    CONFIG_SENSOR_INIT_PRIORITY,                                    \
			    &bq2589x_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ2589X_INIT)
