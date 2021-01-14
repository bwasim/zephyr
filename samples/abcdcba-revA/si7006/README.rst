.. _si7006:

SI7006 Temp / Humidity Sensor
#############################

Overview
********

A simple sample that can be used with the ABC board coupled with the
ABCDCBA daughter card. It will read Temperature / Humidity values
from the sensor and print them out on the serial console.

Building and Running
********************

This application can be built and executed on ABC board as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/abcdcba-samples/si7006
   :host-os: unix
   :board: contextualelectronics_abc
   :goals: run
   :compact:

This application will work with all platforms having SI7006.

Sample Output
=============

.. code-block:: console

    main: Humidity Value: 22.000000 
    main: Temp Value: 23.900000 
    main: Humidity Value: 22.000000 
    main: Temp Value: 23.860000 
    main: Humidity Value: 22.000000 
    main: Temp Value: 23.880000 
    main: Humidity Value: 22.000000 
    main: Temp Value: 23.870000 
    main: Humidity Value: 22.000000 
