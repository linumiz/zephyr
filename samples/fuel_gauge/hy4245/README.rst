.. zephyr:code-sample:: hy4245
   :name: HY4245 Li-Ion battery fuel gauge

   Read battery percentage and power status using HY4245 fuel gauge.

Overview
********

This sample shows how to use the Zephyr :ref:`fuel_gauge_api` API driver for the HY4245 fuel gauge.

The sample periodically reads battery percentage and power status

Building and Running
********************

The sample can be configured to support HY4245 fuel gauge connected via either I2C. It only needs
an I2C pin configuration

Features
********
By using this fuel gauge you can get the following information:
  * Battery charge status as percentage
  * Total time until battery is fully charged or discharged
  * Battery voltage
  * Charging state: if charging or discharging


Sample output
*************

```
*** Booting Zephyr OS build 16043f62a40a ***
Found device "hy4542@55", getting fuel gauge data
Time to empty 1911
Time to full 0
Charge 72%
Voltage 3968
Time to empty 1911
Time to full 0
Charge 72%
Voltage 3968
```
