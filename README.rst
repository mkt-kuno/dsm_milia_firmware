.. zephyr:code-sample:: milia
   :name: Modbus RTU for DigitShowModbus
   :relevant-api: modbus_api

   A Modbus RTU program for DigitShowModbus device.

Overview
********

This sample implements a Modbus RTU slave for the DigitShowModbus device, demonstrating Modbus communication in userspace.

Building and Running
********************

Build for Raspberry Pi Pico RP2040:

.. code-block:: console

   west build -p always -b rpi_pico .\samples\userspace\milia

Build for Blackpill STM32F411CE:

.. code-block:: console

   west build -p always -b blackpill_f411ce .\samples\userspace\milia

Build for Bluepill STM32F103 (WIP):

.. code-block:: console

   west build -p always -b stm32_min_dev .\samples\userspace\milia

After building, flash the board and connect via serial to communicate with the Modbus RTU device.
