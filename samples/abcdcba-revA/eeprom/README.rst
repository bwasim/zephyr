.. _eeprom-sample:

EEPROM
######

Overview
********

A simple application which communicates with the I2C based AT24 EEPROM.
The source code writes 4 bytes at address 0x0, and reads them back to
verify that the data was written successfully. This read-write-verify
cycle is done twice in this demo, whilst the reader is free to modify
this application to serve their needs.

.. _eeprom-sample-requirements:

Requirements
************

This sample will work on any platform with an I2C based AT24 EEPROM.
Actually the application is free of any EEPROM specific code, and so
it should work on every eeprom device.

Building and Running
********************

Build and flash EEPROM sample as follows

.. zephyr-app-commands::
   :zephyr-app: samples/abcdcba-samples/eeprom
   :board: contextualelectronics_abc
   :goals: build flash
   :compact:

After flashing, read-write-verify cycle will be done on the EEPROM.
Messages will be printed on the console to indicate the status of
various operations. 
