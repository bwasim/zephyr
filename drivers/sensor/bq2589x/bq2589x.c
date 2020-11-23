/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bq2589x.h"

/* I2C device to which the charger is connected. */
static const struct device* i2c_dev;

/* BQ2589 data. */
static enum bq2589x_part_no part_no;
static int revision;

/* #define PULSE_CONTROL_EN */
#define OTG_CONTROL_EN 1

static inline int bq2589x_read_byte(uint8_t *data, uint8_t reg)
{
	return i2c_reg_read_byte(i2c_dev, BQ2589X_I2C_ADDRESS, reg, data);
}

static inline int bq2589x_write_byte(uint8_t reg, uint8_t data)
{
	return i2c_reg_write_byte(i2c_dev, BQ2589X_I2C_ADDRESS, reg, data);
}

static inline int bq2589x_update_bits(uint8_t reg, uint8_t mask, uint8_t data)
{
	int ret; uint8_t tmp;

	ret = bq2589x_read_byte(&tmp, reg);
	if (ret != 0)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2589x_write_byte(reg, tmp);
}

enum bq2589x_vbus_type bq2589x_get_vbus_type(void)
{
	uint8_t val = 0; int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;

	/* Apply the correct mask. */
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	/* Return the vbus type. */
	return val;
}

#if (OTG_CONTROL_EN == 1)
int bq2589x_enable_otg(void)
{
	uint8_t val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	/* Update the bits. */
	return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
}

int bq2589x_disable_otg(void)
{
	uint8_t val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	/* Update the bits. */
	return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

}

int bq2589x_set_otg_volt(int volt)
{
	uint8_t val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_BOOSTV_MASK, val);

}

int bq2589x_set_otg_current(int curr)
{
	uint8_t temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 700)
		temp = BQ2589X_BOOST_LIM_700MA;
	else if (curr == 1100)
		temp = BQ2589X_BOOST_LIM_1100MA;
	else if (curr == 1600)
		temp = BQ2589X_BOOST_LIM_1600MA;
	else if (curr == 1800)
		temp = BQ2589X_BOOST_LIM_1800MA;
	else if (curr == 2100)
		temp = BQ2589X_BOOST_LIM_2100MA;
	else if (curr == 2400)
		temp = BQ2589X_BOOST_LIM_2400MA;
	else
		temp = BQ2589X_BOOST_LIM_1300MA;

	return bq2589x_update_bits(BQ2589X_REG_0A, BQ2589X_BOOST_LIM_MASK, temp << BQ2589X_BOOST_LIM_SHIFT);
}

void bq2589x_set_otg(int enable)
{
	int ret;

	if (enable) {
		ret = bq2589x_enable_otg();
		if (ret < 0) {
			return;
		}
	} else{
		ret = bq2589x_disable_otg();
		if (ret < 0)
			printk("%s:Failed to disable otg-%d\n", __func__, ret);
	}
}

#endif /* #if (OTG_CONTROL_EN == 1) */

static void bq2589_irq(const struct device *gpio,
		       struct gpio_callback *cb, uint32_t pins)
{

	printk("BQ2589 Interrupt Received..\n");
}

int bq2589x_setup_ce(void)
{
	const struct device *dev;

	dev = device_get_binding("GPIO_1");
	if (dev == NULL)
		return -1;

	return gpio_pin_configure(dev, GPIO_CE_PIN, GPIO_OUTPUT_LOW);
}

int bq2589x_setup_irq(void)
{
	const struct device *dev;
	int err;
	static struct gpio_callback gpio_ctx;

	dev = device_get_binding("GPIO_1");
	if (dev == NULL)
		return -1;

	err = gpio_pin_configure(dev, GPIO_INT_PIN, GPIO_INPUT);
	if (err < 0)
		return err;

	err = gpio_pin_interrupt_configure(dev, GPIO_INT_PIN, GPIO_INT_EDGE_RISING);
	if (err)
		return err;

	gpio_init_callback(&gpio_ctx, bq2589_irq, BIT(GPIO_INT_PIN));
	err = gpio_add_callback(dev, &gpio_ctx);
	if (err)
		return err;

	return 0;
}

int bq2589x_setup_otg(void)
{
	/* Set OTG pin HIGH forever to enable PMID pin (P1.08). */

	const struct device *dev;

	dev = device_get_binding("GPIO_1");
	if (dev == NULL)
		return -1;

	return gpio_pin_configure(dev, GPIO_OTG_PIN, GPIO_OUTPUT_HIGH);
}

int bq2589x_enable_charger(void)
{
	int ret;
	uint8_t val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

int bq2589x_disable_charger(void)
{
	int ret;
	uint8_t val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

/* interfaces that can be called by other module */
int bq2589x_adc_start(bool oneshot)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_02);
	if (ret < 0)
		return ret;

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,  BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}

int bq2589x_adc_stop(void)
{
	return bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
							   BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}

int bq2589x_adc_read_battery_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0E);
	if (ret < 0)
		return ret;

	volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) * BQ2589X_BATV_LSB);
	return volt;
}

int bq2589x_adc_read_sys_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0F);
	if (ret < 0)
		return ret;

	volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) * BQ2589X_SYSV_LSB);
	return volt;
}

int bq2589x_adc_read_vbus_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_11);
	if (ret < 0)
		return ret;

	volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) * BQ2589X_VBUSV_LSB);
	return volt;
}

int bq2589x_adc_read_temperature(void)
{
	uint8_t val;
	int temp, ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_10);
	if (ret < 0)
		return ret;

	temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
	return temp;
}

int bq2589x_adc_read_temperature_2(void)
{
    uint8_t val, data;
    int     ret, temp;

    bq2589x_read_byte(&data, 0x02);
    data |= 0x80;
    data &= 0xbf;
    bq2589x_write_byte(0x02, data);
    k_msleep(200);

	ret = bq2589x_read_byte(&val, BQ2589X_REG_10);

	bq2589x_read_byte(&data, 0x02);
    data&=0x3f;
    bq2589x_write_byte(0x02, data);

    if(ret < 0){
        printk("read temperature failed :%d\n",ret);
        return ret;
    }
    else{
        temp = BQ2589X_TSPCT_BASE + val * 0.465;

        return temp;
    }
}

int bq2589x_adc_read_charge_current(void)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(&val, BQ2589X_REG_12);
	if (ret < 0)
		return ret;

	volt = BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) * BQ2589X_ICHGR_LSB);
	return volt;
}

int bq2589x_set_chargecurrent(int curr)
{
	uint8_t ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(BQ2589X_REG_04, BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);
}

int bq2589x_set_term_current(int curr)
{
	uint8_t iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}

int bq2589x_set_prechg_current(int curr)
{
	uint8_t iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}

int bq2589x_set_chargevoltage(int volt)
{
	uint8_t val;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(BQ2589X_REG_06, BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}

int bq2589x_set_input_volt_limit(int volt)
{
	uint8_t val;

	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}

int bq2589x_set_input_current_limit(int curr)
{
	uint8_t val;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}

int bq2589x_set_vindpm_offset(int offset)
{
	uint8_t val;

	val = (offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}

int bq2589x_get_charging_status(void)
{
	uint8_t val = 0;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
	if (ret < 0)
		return ret;

	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}

int bq2589x_set_watchdog_timer(uint8_t timeout)
{
	return bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, (uint8_t)((timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB) << BQ2589X_WDT_SHIFT);
}

int bq2589x_disable_watchdog_timer(void)
{
	uint8_t val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}

int bq2589x_reset_watchdog_timer(void)
{
	uint8_t val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

int bq2589x_force_dpdm(void)
{
	int ret;
	uint8_t val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if (ret)
		return ret;

	k_msleep(20);/*TODO: how much time needed to finish dpdm detect?*/
	return 0;

}

int bq2589x_reset_chip(void)
{
	int ret;
	uint8_t val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	return ret;
}

int bq2589x_enter_ship_mode(void)
{
	int ret;
	uint8_t val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	return ret;
}

int bq2589x_enter_hiz_mode(void)
{
	uint8_t val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}

int bq2589x_exit_hiz_mode(void)
{
	uint8_t val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);
}

int bq2589x_get_hiz_mode(uint8_t *state)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}

#if (PULSE_CONTROL_EN == 1)
int bq2589x_pumpx_enable(int enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}

int bq2589x_pumpx_increase_volt(void)
{
	uint8_t val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}

int bq2589x_pumpx_increase_volt_done(void)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/
}

int bq2589x_pumpx_decrease_volt(void)
{
	uint8_t val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;
}

int bq2589x_pumpx_decrease_volt_done(void)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
#endif /* #if (PULSE_CONTROL_EN == 1) */

/* Commented out code to supress warnings - Unused for now. */
#if 1
static int bq2589x_force_ico(void)
{
	uint8_t val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}

static int bq2589x_check_force_ico_done(void)
{
	uint8_t val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}

static int bq2589x_enable_term(bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_EN_TERM_MASK, val);

	return ret;
}

static int bq2589x_enable_auto_dpdm(bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;
}

static int bq2589x_use_absolute_vindpm(bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;
}

static int bq2589x_enable_ico(bool enable)
{
	uint8_t val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;
}

static int bq2589x_read_idpm_limit(void)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_13);
	if (ret < 0)
		return ret;

	curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
	return curr;
}

bool bq2589x_is_charge_done(void)
{
	int ret;
	uint8_t val;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
	if (ret < 0)
		return false;

	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}

int bq2589x_charge_status(void)
{
	uint8_t val = 0;

	bq2589x_read_byte(&val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static void bq2589x_adjust_absolute_vindpm(void)
{
	uint16_t vbus_volt;
	uint16_t vindpm_volt;
	int ret;

	ret = bq2589x_disable_charger();
	if (ret < 0) {
		printk("%s:failed to disable charger\n",__func__);
		/*return;*/
	}
	/* wait for new adc data */
	k_msleep(1000);
	vbus_volt = bq2589x_adc_read_vbus_volt();
	ret = bq2589x_enable_charger();
	if (ret < 0) {
		printk("%s:failed to enable charger\n",__func__);
		return;
	}

	if (vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else
		vindpm_volt = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(vindpm_volt);
	if (ret < 0)
		printk("%s:Set absolute vindpm threshold %d Failed:%d\n", __func__, vindpm_volt, ret);
	else
		printk("%s:Set absolute vindpm threshold %d successfully\n", __func__, vindpm_volt);

}
#endif

/* Try to see if the "bq2589" device is connected with */
static int bq2589x_detect_device(void)
{
	uint8_t data;

	/* Read REG14 to get the part number / revision. */
	if (bq2589x_read_byte(&data, BQ2589X_REG_14) != 0)
		return -1;

	/* Filter part number / revision. */
	part_no  = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
	revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	return 0;
}

/* Try to init the "bq2589" device */
static int bq2589x_init_device(void)
{
	int ret;

	/* These numbers should ideally come from DT file, but this is not
	 * a proper Zephyr driver.. */
#define CHARGE_VOLTAGE 4000
#define CHARGE_CURRENT 500
#define ITERM_CURRENT  64

	bq2589x_disable_watchdog_timer();
	bq2589x_enable_auto_dpdm(true);
	bq2589x_enable_term(true);
	bq2589x_enable_ico(true);

	ret = bq2589x_set_term_current(ITERM_CURRENT);
	if (ret < 0)
	{
		printk("%s:Failed to set termination current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargevoltage(CHARGE_VOLTAGE);
	if (ret < 0)
	{
		printk("%s:Failed to set charge voltage:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_set_chargecurrent(CHARGE_CURRENT);
	if (ret < 0)
	{
		printk("%s:Failed to set charge current:%d\n", __func__, ret);
		return ret;
	}

	ret = bq2589x_enable_charger();
	if (ret < 0)
	{
		printk("%s:Failed to enable charger:%d\n", __func__, ret);
		return ret;
	}

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

	bq2589x_adc_start(false);
	bq2589x_disable_watchdog_timer();

	return ret;
}

bool charger_init(void)
{
	int     ret;

	/* Get the I2C bus. */
	i2c_dev = device_get_binding(BQ2589X_I2C_DEVICE);
	if (i2c_dev == NULL)
	{
		printk("charger_init > device_get_binding > failed\r\n");
		return false;
	}

	/* Try to detect the device. */
	ret = bq2589x_detect_device();
	if (ret != 0 || part_no != BQ25895)
	{
		printk("charger_init > bq2589x_detect_device > failed\r\n");
		return false;
	}

	ret = bq2589x_init_device();
	if (ret != 0)
	{
		printk("charger_init > bq2589x_init_device > failed\r\n");
		return false;
	}

	return true;
}

/* This function reads the PG_STAT bit of REG0B to determine
 * if the Input Power is Good or Not. Required by the MPPT
 * algorithm. */
static int bq2589x_is_power_good(void)
{
	uint8_t val = 0;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0B);
	if (ret < 0)
		return ret;

	return (val & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT;
}

/* This function finds the maximum vbus by enabling HIZ mode, where
 * all load is removed so that the measure vbus is max. Note that this
 * function expects the BQ25895 ADC to be enabled and running. Otherwise
 * it will fail to read VBus Voltage. */
static int bq2589x_find_oc_vbus(void)
{
	int vbus;

	if (bq2589x_enter_hiz_mode())
		return -1;

	vbus = bq2589x_adc_read_vbus_volt();

	if (bq2589x_exit_hiz_mode())
		return -1;

	return vbus;
}

/* This function forces the use the provided vindpm. */
static int bq2589x_force_vindpm(uint8_t volt)
{
	return bq2589x_write_byte(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK | volt);
}

/* This function finds the MPPT to use for a given input device. */
int bq2589x_find_mppt(void)
{
	/* Tries to find the MPPT between 65-95% of "oc_vbus". Can be changed. */
#define VOC_LOW         65
#define VOC_HIGH        95
#define VBUS_MIN        0x0D
#define VBUSV_OFFSET    0x1A

	int oc_vbus, count = 0;

	unsigned int voc_low, voc_high, icharge, i;
    unsigned int ichg_max = 0x0, vindpm_max = 0x0;

	if (bq2589x_is_power_good() == 0)
		return -1;

	oc_vbus = bq2589x_find_oc_vbus();
	if (oc_vbus == -1)
		return -1;

    voc_low = (VOC_LOW * (oc_vbus / 100)) - ((100 - VOC_LOW) * (VBUSV_OFFSET / 100));
    if (voc_low < VBUS_MIN)
    	voc_low = VBUS_MIN;

    voc_high = (VOC_HIGH * (oc_vbus / 100)) - ((100 - VOC_HIGH) * VBUSV_OFFSET / 100);

    for (i = voc_low; i < voc_high; i ++)
    {
    	if (bq2589x_force_vindpm(i) != 0)
    		return -1;

    	icharge = bq2589x_adc_read_charge_current();

        if (icharge > ichg_max)
        {
            count      = 0;
            ichg_max   = icharge;
            vindpm_max = i;
        }
        else if (icharge == ichg_max)
        {
            if (i > vindpm_max)
            {
                vindpm_max = i;
                count ++;
            }
        }
    }

    vindpm_max -= (count / 2) * 0x01;
    bq2589x_force_vindpm(vindpm_max);
    return vindpm_max;
}
