.. zephyr:board:: cytvii-b-e-100-cpu

Overview
********

The CYTVII-B-E-100-CPU evaluation board (CPU board) is an evaluation platform for the CYT2B75CABES Traveo™ II device. It can be used standalone for basic validation or in combination with the CYTVII-B-E-BB Traveo II baseboard (available separately). This document assumes usage with both CPU and baseboard, providing guidance for the full evaluation platform.

**Precautions and Warnings**
The board is a delicate PCB; handle with care and ensure only qualified personnel operate it. Use only the supplied 12V DC power adapter. Follow all safety instructions to avoid electrical hazards.

Hardware
********

Main components of the CPU board:
- Traveo II device mounted on a socket (U3)
- Power supply circuit (3.3V or 5V selectable via J23 jumper)
- Programming interfaces: Arm® Standard JTAG, Cortex® Debug, Cortex Debug + ETM, Arm ETM Mictor (compatible with IAR I-jet, Green Hills GHS, MiniProg)
- USB-UART interface for terminal logging (J12)
- One user switch (SW3) and one user LED (LED5) for standalone operation
- Manual reset switch (SW2) and voltage supervision
- Current measurement jumpers (J6, J8, J10 for VDDA, VDDD, VDDIO)
- Samtec connector interface (J21, J22) for baseboard connection

Main components of the Traveo II baseboard (CYTVII-B-E-BB):
- Six CAN-FD transceivers (TJA1057GT)
- Four CAN-FD transceivers (TJA1145T, SPI-configurable)
- Six LIN transceivers (TJA1021T)
- Two FlexRay transceivers (TJA1081TS)
- One CXPI transceiver (S6BT112A01)
- One SPI EEPROM (25LC320A)
- Five user switches, ten user LEDs, one potentiometer
- Pin headers for all I/Os
- Samtec connector interface for CPU board connection

Operation
*********

To operate the board:
1. Insert the Traveo II device into the socket (U3) with correct orientation and secure with screws.
2. Connect the supplied 12V DC adapter to the CPU board and mains.
3. Insert all required jumpers (J5, J6, J8, J10, J23).
4. Power on the board (SW1). The PWR LED should light up.
5. Connect a programming tool (GHS Trace, IAR I-jet, MiniProg3) to the appropriate interface.
6. Use the programming IDE to detect the device and load firmware (.srec).
7. For terminal logging, connect USB-mini cable to J12 and set up a terminal application (baud rate 115200, 8N1).
8. Insert jumpers J11 and J13 for terminal operation.

Connections and Settings
***********************

Insert the following jumpers on the baseboard for each transceiver and feature:
- CAN0.0: J70, J71, J72
- CAN0.1: J66, J67, J68
- CAN0.2: J81, J82, J83
- CAN1.0: J76, J77, J78
- CAN1.1: J91, J92, J93
- CAN1.2: J86, J87, J88
- LIN0: J58, J59, J60, J63
- LIN1: J51, J52, J53, J56
- LIN2: J37, J39, J40, J43
- LIN3: J30, J31, J32, J35
- LIN4: J22, J23, J24, J27
- LIN5: J10, J16, J17, J20
- EEPROM: J47, J48, J49
- User switch: J102
- Potentiometer: J89
- Power: J80 (connect to 5V select pin)

Ensure correct firmware is loaded for the selected functionality to avoid hardware damage.

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Building
========

Example for building the :zephyr:code-sample:`blinky` sample application for the M0P core:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/blinky
   :board: cytvii_b_e_100_cpu/cyt2b75cae/m0p
   :goals: build

Example for building the :zephyr:code-sample:`button` sample application for the M4 core:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/button
   :board: cytvii_b_e_100_cpu/cyt2b75cae/m4
   :goals: build

Flash M0P core first to enable M4 core, then flash M4 core.

SYS-Build
========

Example for building m0p and m4 cores together using SYS-Build:

.. zephyr-app-commands::
   :zephyr-app: samples/basic/button
   :board: cytvii_b_e_100_cpu/cyt2b75cae/m4
   :goals: build

    west build -b cytvii_b_e_1m_100_cpu/cyt2b75cae/m4  -s  .\samples\drivers\flash_driver_test_t2g\ --sysbuild
Flashing
========

The board does not include an onboard programmer/debugger. Use Cypress MiniProg3, Arm Standard JTAG, or any compatible programming tool.

References
**********

.. target-notes::

.. _CYTVII-B-E-100-CPU Board Website:
   https://www.infineon.com/evaluation-board/cytvii-b-e-100-cpu

.. _Traveo II CYT2B75CAES SoC Website:
   https://www.infineon.com/products/microcontroller/32-bit-traveo-t2g-arm-cortex/for-body/t2g-cyt2b75caes

.. _MiniProg3:
   https://www.infineon.com/cms/en/product/promopages/psocprogrammers/