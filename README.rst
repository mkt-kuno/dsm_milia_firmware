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
   west flash -r uf2

Build for Blackpill STM32F411CE:

.. code-block:: console

   west build -p always -b blackpill_f411ce .\samples\userspace\milia
   west flash

Build for Bluepill STM32F103:

.. code-block:: console

   west build -p always -b stm32_min_dev .\samples\userspace\milia
   west flash

After building, flash the board and connect via serial to communicate with the Modbus RTU device.
