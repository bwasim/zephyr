/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr.h>
#include <drivers/adc.h>
#include <hal/nrf_saadc.h>

#define ADC_DEVICE_NAME			DT_LABEL(DT_INST(0, nordic_nrf_saadc))
#define ADC_RESOLUTION			10
#define ADC_GAIN				ADC_GAIN_1_6
#define ADC_REFERENCE			ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID		0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN1
#define INVALID_ADC_VALUE 		SHRT_MIN

#define BUFFER_SIZE  1
static  int16_t      m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};

static void check_samples(void)
{
	int i;

	printk("Samples read: ");
	for (i = 0; i < BUFFER_SIZE; i++)
	{
		int16_t sample_value = m_sample_buffer[i];

		printk("0x%04x ", sample_value);
	}

	printk("\r\n");
}

static const struct device *init_adc(void)
{
	int i, ret;
	const struct device *adc_dev;

	adc_dev = device_get_binding(ADC_DEVICE_NAME);
	if (adc_dev == NULL)
	{
		printk("%s: Cannot get ADC device \r\n", __func__);
		return NULL;
	}

	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (ret != 0)
	{
		printk("%s: Setting up of the first channel failed with code %d \r\n", __func__, ret);
		return NULL;
	}

	for (i = 0; i < BUFFER_SIZE; ++i)
		m_sample_buffer[i] = INVALID_ADC_VALUE;

	return adc_dev;
}

/* Single Channel Test. */
int adc_one_channel(void)
{
	int ret;
	const struct device       *adc_dev;
	const struct adc_sequence sequence =
	{
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	adc_dev = init_adc();
	if (!adc_dev)
		return -1;

	while (1)
	{
		ret = adc_read(adc_dev, &sequence);
		if (ret != 0)
		{
			printk("%s: adc_read() failed with code %d \r\n", __func__, ret);
			return ret;
		}

		check_samples();
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

void main(void)
{
	adc_one_channel();
}
