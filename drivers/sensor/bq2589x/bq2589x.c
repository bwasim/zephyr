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

static inline int bq2589x_read_byte(const struct device *i2c_dev,
		uint8_t reg, uint8_t *data)
{
	return i2c_reg_read_byte(i2c_dev, DT_INST_REG_ADDR(0), reg, data);
}

static inline int bq2589x_write_byte(const struct device *i2c_dev,
		uint8_t reg, uint8_t data)
{
	return i2c_reg_write_byte(i2c_dev, DT_INST_REG_ADDR(0), reg, data);
}

static inline int bq2589x_update_bits(const struct device *i2c_dev,
		uint8_t reg, uint8_t mask, uint8_t data)
{
	int ret;
	uint8_t tmp;

	ret = bq2589x_read_byte(i2c_dev, reg, &tmp);
	if (ret != 0)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(i2c_dev, reg, tmp);
}

static int bq2589x_detect_device(const struct device *i2c,
		enum bq2589x_part_no *part_no,
		int *revision)
{
	uint8_t data;

	/* Read REG14 to get the part number / revision. */
	if (bq2589x_read_byte(i2c, BQ2589X_REG_14, &data) != 0)
		return -1;

	/* Filter part number / revision. */
	*part_no  = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
	*revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	return 0;
}

static inline int bq2589x_disable_watchdog_timer(const struct device *i2c_dev)
{
	uint8_t val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}

static inline int bq2589x_enable_auto_dpdm(const struct device *i2c_dev, bool enable)
{
	uint8_t val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	if (enable) {
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	}

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);
}

static inline int bq2589x_enable_term(const struct device *i2c_dev, bool enable)
{
	uint8_t val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	if (enable) {
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	}

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);
}

static inline int bq2589x_enable_ico(const struct device *i2c_dev, bool enable)
{
	uint8_t val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	if (enable) {
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	}

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
}

static inline int bq2589x_use_absolute_vindpm(const struct device *i2c_dev, bool enable)
{
	uint8_t val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	if (enable) {
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	}

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);
}

static inline int bq2589x_set_vindpm_offset(const struct device *i2c_dev, int offset)
{
	uint8_t val = (offset - BQ2589X_VINDPMOS_BASE) / BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_01,
			BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}

static inline int bq2589x_adc_start(const struct device *i2c_dev, bool oneshot)
{
	uint8_t val;
	int     ret;

	ret = bq2589x_read_byte(i2c_dev, BQ2589X_REG_02, &val);
	if (ret < 0) {
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT)
			== BQ2589X_ADC_CONTINUE_ENABLE) {
		return 0;
	}

	if (oneshot) {
		return bq2589x_update_bits(i2c_dev, BQ2589X_REG_02, BQ2589X_CONV_START_MASK,
				BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	}

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
			BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
}

static inline int bq2589x_set_term_current(const struct device *i2c_dev, int curr)
{
	uint8_t iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_05, BQ2589X_ITERM_MASK,
			iterm << BQ2589X_ITERM_SHIFT);
}

static inline int bq2589x_set_chargevoltage(const struct device *i2c_dev, int volt)
{
	uint8_t val = (volt - BQ2589X_VREG_BASE) / BQ2589X_VREG_LSB;

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_06, BQ2589X_VREG_MASK,
			val << BQ2589X_VREG_SHIFT);
}

static inline int bq2589x_set_chargecurrent(const struct device *i2c_dev, int curr)
{
	uint8_t ichg = (curr - BQ2589X_ICHG_BASE) / BQ2589X_ICHG_LSB;

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_04, BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);
}

static inline int bq2589x_enable_charger(const struct device *i2c_dev)
{
	uint8_t val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	return bq2589x_update_bits(i2c_dev, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
}

static int bq2589x_init_device(struct bq2589x_data *bq2589x,
		const struct bq2589x_config *const config)
{
	int ret;

	bq2589x_disable_watchdog_timer(bq2589x->i2c);
	bq2589x_enable_auto_dpdm(bq2589x->i2c, config->enable_auto_dpdm);
	bq2589x_enable_term(bq2589x->i2c, config->enable_term);
	bq2589x_enable_ico(bq2589x->i2c, config->enable_ico);

#if 0
	/* Force use absolute vindpm if auto_dpdm not enabled */
	if (config->enable_auto_dpdm == false) {
		config->use_absolute_vindpm = true;
	}

	bq2589x_use_absolute_vindpm(bq2589x->i2c, config->use_absolute_vindpm);
#endif

	ret = bq2589x_set_vindpm_offset(bq2589x->i2c, 600);
	if (ret < 0) {
		LOG_ERR("Failed to set vindpm offset (ret: %d)", ret);
		return ret;
	}

	ret = bq2589x_set_term_current(bq2589x->i2c, config->term_current);
	if (ret < 0) {
		LOG_ERR("Failed to set termination current (ret: %d)", ret);
		return ret;
	}

	ret = bq2589x_set_chargevoltage(bq2589x->i2c, config->charge_voltage);
	if (ret < 0) {
		LOG_ERR("Failed to set charge voltage (ret: %d)", ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(bq2589x->i2c, config->charge_current);
	if (ret < 0) {
		LOG_ERR("Failed to set charge current (ret: %d)", ret);
		return ret;
	}

	ret = bq2589x_enable_charger(bq2589x->i2c);
	if (ret < 0) {
		LOG_ERR("Failed to enable charger (ret: %d)", ret);
		return ret;
	}

#if 0
	/* Setup CE to be low all the time, and let CHG_CONFIG decide if
	 * battery will be charged or not.. */
	ret = bq2589x_setup_ce();
	if (ret < 0)
	{
		printk("%s:Failed to setup CE:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_setup_irq();
	if (ret < 0)
	{
		printk("%s:Failed to setup IRQ:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_setup_otg();
	if (ret < 0)
	{
		printk("%s:Failed to setup OTG:%d\n", __func__, ret);
		return ret;
	}

#endif

	/* Enable PumpX? */
	bq2589x_adc_start(bq2589x->i2c, false);

	/* Enabling Watchdog would mean that we need to handle its expiry - Should
	 * work without it I guess ? */
	/* bq2589x_disable_watchdog_timer(bq2589x->i2c); */

	return ret;
}

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
	struct bq2589x_data *bq2589x = dev->data;
	const struct bq2589x_config *const config = dev->config;
	enum bq2589x_part_no part_no;
	int revision;
	int ret;

	bq2589x->i2c = device_get_binding(config->bus_name);
	if (bq2589x->i2c == NULL)
	{
		LOG_ERR("Could not get pointer to %s device.", config->bus_name);
		return -EINVAL;
	}

	/* Detect the device (Only support BQ25890 for now). */
	ret = bq2589x_detect_device(bq2589x->i2c, &part_no, &revision);
	if (ret != 0 || part_no != BQ25890)
	{
		LOG_ERR("Unable to find the TI BQ2589x Battery at %s (%d).",
				config->bus_name, DT_INST_REG_ADDR(0));
		return -EIO;
	}

	ret = bq2589x_init_device(bq2589x, config);
	if (ret != 0)
	{
		LOG_ERR("Failed to initialize the BQ2589x device (ret = %d).", ret);
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api bq2589x_battery_driver_api = {
	.sample_fetch = bq2589x_sample_fetch,
	.channel_get  = bq2589x_channel_get,
};

#define BQ2589X_INIT(index)                                                          \
	static struct bq2589x_data bq2589x_driver_##index;                               \
									                                                 \
	static const struct bq2589x_config bq2589x_config_##index = {                    \
		.bus_name            = DT_INST_BUS_LABEL(index),                             \
		.charge_voltage      = DT_INST_PROP(index, charge_voltage),                  \
		.charge_current      = DT_INST_PROP(index, charge_current),                  \
		.term_current        = DT_INST_PROP(index, term_current),                    \
		.enable_auto_dpdm    = IS_ENABLED(DT_INST_PROP(index, enable_auto_dpdm)),    \
		.enable_term         = IS_ENABLED(DT_INST_PROP(index, enable_termination)),  \
		.enable_ico          = IS_ENABLED(DT_INST_PROP(index, enable_ico)),          \
		.use_absolute_vindpm = IS_ENABLED(DT_INST_PROP(index, use_absolute_vindpm)), \
	};                                                                               \
									                                                 \
	DEVICE_DT_INST_DEFINE(index, &bq2589x_init, device_pm_control_nop,               \
			    &bq2589x_driver_##index,                                             \
			    &bq2589x_config_##index, POST_KERNEL,                                \
			    CONFIG_SENSOR_INIT_PRIORITY,                                         \
			    &bq2589x_battery_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ2589X_INIT)
