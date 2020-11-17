
#ifndef BQ2589X_H
#define BQ2589X_H

#include <device.h>
#include <zephyr.h>
#include <math.h>
#include <time.h>

#include <drivers/gpio.h>
#include <drivers/i2c.h>

/* Get temperature of the device. */
int  bq2589x_adc_read_temperature(void);
int  bq2589x_adc_read_temperature_2(void);

/* Get / Set charge current. */
int  bq2589x_adc_read_charge_current(void);
int  bq2589x_set_chargecurrent(int curr);
int  bq2589x_get_charging_status(void);
int  bq2589x_set_input_current_limit(int curr);
int  bq2589x_adc_read_vbus_volt(void);
int  bq2589x_adc_read_sys_volt(void);
int  bq2589x_adc_read_battery_volt(void);

/* Enable / Disable charger. */
int  bq2589x_disable_charger(void);
int  bq2589x_enable_charger(void);

/* Start / Stop ADC. */
int  bq2589x_adc_start(bool oneshot);
int  bq2589x_adc_stop(void);

/* General routines. */
int  bq2589x_reset_chip(void);
bool charger_init(void);


#endif /* BQ2589X_H */