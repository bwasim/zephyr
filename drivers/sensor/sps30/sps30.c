/*
 * Copyright (c) 2020 Bilal Wasim
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensirion_sps30

#include "sps30.h"

LOG_MODULE_REGISTER(sps30, CONFIG_SENSOR_LOG_LEVEL);

static const struct sensor_driver_api sps30_api = {
	.sample_fetch = /* &si7060_sample_fetch */ NULL,
};

static int sps30_init(struct device *dev)
{
	struct sps30 *drv_data = dev->data;

	drv_data->i2c_dev = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!drv_data->i2c_dev) {
		LOG_ERR("Failed to get pointer to %s device!",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static struct sps30 sps30_data;

DEVICE_AND_API_INIT(sps30, DT_INST_LABEL(0), sps30_init,
		&sps30_data, NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &sps30_api);
