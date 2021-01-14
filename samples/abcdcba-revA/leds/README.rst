.. _leds-sample:

LEDs
####

Overview
********

A simple application which blinks all LEDs on the ABCDCBA daughter card.
The source code shows how to configure GPIO pins as outputs, then turn
them on and off.

.. _leds-sample-requirements:

Requirements
************

This sample uses all of the LEDs (8) on the ABCDCBA board, and won't
work on any platform that doesn't have all 8 LEDs.

Building and Running
********************

Build and flash LEDs as follows

.. zephyr-app-commands::
   :zephyr-app: samples/abcdcba-samples/leds
   :board: contextualelectronics_abc
   :goals: build flash
   :compact:

After flashing, the LED starts to blink. This application does not print
to the console.
