.. zephyr:code-sample:: canopen
   :name: CANopen node
   :relevant-api: canopen

   Run a CANopen node with PDOs and access it from a CANopen master.

Overview
********

This sample runs a CANopen node (CiA 301) on the CAN controller selected by the
``zephyr,canbus`` chosen node. The node provides these application objects:

* ``2000h``: UNSIGNED32 counter, read only, incremented every 100 ms and
  transmitted in TPDO1 (CAN-ID 180h + node-ID) every second.
* ``2001h``: UNSIGNED8 output, read and write, received with RPDO1 (CAN-ID
  200h + node-ID).
* ``5600h``: DOMAIN of up to 256 bytes, read and write over SDO.

The node-ID (default 16), the bitrate (default 500 kbit/s), the counter period
and the TPDO1 event timer are set with the ``CONFIG_SAMPLE_*`` options. The
heartbeat producer time (``1017h``) is 1000 ms.

The node consumes TIME messages. Once it has received one and is made TIME producer by setting
bit 30 of ``1012h``, it sends its clock once per second.

If the board defines the ``canopen-run-led`` and ``canopen-err-led`` aliases to
``gpio-leds`` nodes and :kconfig:option:`CONFIG_LED` is enabled, the node drives
them as the CiA 303-3 RUN and ERR LEDs.

Requirements
************

* A board with a CAN controller and transceiver, and a CAN bus terminated with
  120 ohm at both ends.
* A CANopen master on the same bus, for example a Linux host with a SocketCAN
  interface and the ``can-utils`` tools.

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/canbus/canopen
   :board: <board>
   :goals: build flash
   :compact:

On a Linux host, bring up the CAN interface with the same bitrate, watch the bus and start the
node with an NMT command:

.. code-block:: console

   sudo ip link set can0 up type can bitrate 500000
   candump can0 &
   cansend can0 000#0110

The node sends its heartbeat on CAN-ID 710h, ``05`` once it is operational, and TPDO1 with the
counter on CAN-ID 190h. An SDO upload of ``1017h`` returns the heartbeat producer time:

.. code-block:: console

   cansend can0 610#4017100000000000
     can0  590   [8]  4B 17 10 00 E8 03 00 00

The build also writes the electronic data sheet of the node to ``build/zephyr/canopen.eds``
(:kconfig:option:`CONFIG_CANOPEN_EDS`), which CANopen tools use to access the objects of the
node.

Shell
=====

Build with ``-DEXTRA_CONF_FILE=shell.conf`` to add the ``canopen`` shell command
(:kconfig:option:`CONFIG_CANOPEN_SHELL`) on the console, for example:

.. code-block:: console

   uart:~$ canopen list
   0x10 PRE-OPERATIONAL error register 0x00 can@40520200
   uart:~$ canopen read 16 0x1017 0
   1000 (0x03e8)
   uart:~$ canopen nmt 16 start
   uart:~$ canopen sdo read 16 0x20 0x1008 0

The overlay also enables the master functions, so ``canopen sdo`` and ``canopen nmt`` with a
target node-ID reach other nodes on the bus.

Master mode
===========

With :kconfig:option:`CONFIG_SAMPLE_MASTER` the node also acts as CANopen master. After
start-up it configures the slave with node-ID ``CONFIG_SAMPLE_SLAVE_ID`` (default 32) over SDO
(``1017h`` = 1000 ms, ``2000h`` = 11223344h), starts it with an NMT command, reads its device name
and monitors its heartbeat.

Sample Output
=============

.. code-block:: console

   [00:00:00.010,000] <inf> canopen_sample: NMT state 0x7f
   [00:00:00.010,000] <inf> canopen_sample: CANopen node 0x10 started at 500000 bit/s
   [00:00:05.120,000] <inf> canopen_sample: NMT state 0x05
   [00:00:08.230,000] <inf> canopen_sample: RPDO1: output 0xa5
