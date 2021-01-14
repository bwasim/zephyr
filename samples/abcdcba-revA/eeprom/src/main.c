/*
 * Copyright (c) 2020 Analog Life LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/eeprom.h>
#include <device.h>
#include <string.h>

static void write_and_verify(const struct device *eeprom)
{
	const uint8_t wr_buf1[4] = { 0xFF, 0xEE, 0xDD, 0xCC };
	const uint8_t wr_buf2[sizeof(wr_buf1)] = { 0xAA, 0xBB, 0xCC, 0xDD };
	uint8_t rd_buf[sizeof(wr_buf1)];
	int rc, i;

	printk("%s: Writing at address %x, Contents: \r\n", __func__, 0);
	for (i = 0; i < sizeof(wr_buf1); i ++)
		printk("wr_buf1[%d]=%x\r\n", i, wr_buf1[i]);
	rc = eeprom_write(eeprom, 0, wr_buf1, sizeof(wr_buf1));
	if (rc != 0)
	{
		printk("%s: Unable to perform eeprom_write (rc = %d) \r\n", __func__, rc);
		return;
	}

	printk("%s: Reading from address %x \r\n", __func__, 0);
	rc = eeprom_read(eeprom, 0, rd_buf, sizeof(rd_buf));
	if (rc != 0)
	{
		printk("%s: Unable to perform eeprom_read (rc = %d)\r\n", __func__, rc);
		return;
	}
	printk("Contents: \r\n");
	for (i = 0; i < sizeof(rd_buf); i ++)
		printk("rd_buf[%d]=%x\r\n", i, rd_buf[i]);

	rc = memcmp(wr_buf1, rd_buf, sizeof(wr_buf1));
	if (rc != 0)
	{
		printk("%s: memcmp failed (rc = %d)\r\n", __func__, rc);
		return;
	}
	printk("%s: wr_buf1 matched with rd_buf: \r\n", __func__);

	printk("%s: Writing at address %x, Contents: \r\n", __func__, 0);
	for (i = 0; i < sizeof(wr_buf2); i ++)
		printk("wr_buf2[%d]=%x\r\n", i, wr_buf2[i]);
	rc = eeprom_write(eeprom, 0, wr_buf2, sizeof(wr_buf2));
	if (rc != 0)
	{
		printk("%s: Unable to perform eeprom_write (rc = %d) \r\n", __func__, rc);
		return;
	}

	printk("%s: Reading from address %x \r\n", __func__, 0);
	rc = eeprom_read(eeprom, 0, rd_buf, sizeof(rd_buf));
	if (rc != 0)
	{
		printk("%s: Unable to perform eeprom_read (rc = %d)\r\n", __func__, rc);
		return;
	}
	printk("Contents: \r\n");
	for (i = 0; i < sizeof(rd_buf); i ++)
		printk("rd_buf[%d]=%x\r\n", i, rd_buf[i]);

	rc = memcmp(wr_buf2, rd_buf, sizeof(wr_buf2));
	if (rc != 0)
	{
		printk("%s: memcmp failed (rc = %d)\r\n", __func__, rc);
		return;
	}

	printk("%s: wr_buf2 matched with rd_buf \r\n", __func__);
	printk("%s: Application completed successfully \r\n", __func__);
}

void main(void)
{
	const struct device *eeprom;

	eeprom = device_get_binding("EEPROM_0");
	if (eeprom == NULL)
	{
		printk("%s: Unable to get EEPROM device \r\n", __func__);
		return;
	}

	write_and_verify(eeprom);
}
