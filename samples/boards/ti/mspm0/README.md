## SysTick Issue Reproduction Guide – TI MSPM0 (Zephyr RTOS)

## 1. Overview
This document provides step-by-step instructions to reproduce a SysTick timer issue observed on the TI MSPM0 SoC running Zephyr RTOS.
The issue is related to inconsistent SysTick interrupt timing under certain operating conditions.

---

## 2. Hardware Requirements
- TI MSPM0L2228 LaunchPad
- Any Board with I2C controller  

---

## 3. Software Requirements
- Host PC: **Ubuntu 22.04 LTS** (tested environment)
- [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) **version 0.17.0**
- Python 3.8+ with pip
- Git
- West tool for Zephyr
- OpenOCD for flashing

---

## 4. Environment Setup

### 4.1 Install Required Packages

1. Create and activate a new virtual environment
```bash
python3 -m venv ~/zephyrproject/.venv

source ~/zephyrproject/.venv/bin/activate
```

2. Install west
```bash
pip install west
```

3. Get the source code
```bash

mkdir ~/systick_test && cd ~/systick_test

west init -m https://github.com/linumiz/zephyr.git --mr dev/ti/mspm0-systick

west update

west packages pip --install
```

4. Build the sample i2c target ( lp\_mspm0l2228 )
```bash
cd ~/systick_test/zephyr

west build -p always -b lp_mspm0l2228 -s samples/boards/ti/mspm0/target_eeprom
```

5. Flashing using openocd runner
```bash
west flash --openocd /opt/openocd-mspm0/bin/openocd --openocd-search /opt/openocd-mspm0/share/openocd/scripts/
```

6. Build sample for i2c master ( any board with i2c )
```bash
cd ~/systick_test/zephyr

west build -p always -b <board for i2c scan> -s samples/boards/ti/mspm0/i2c_scan

west flash <i2c scan board runner for flashing>
```

---

## 5. Hardware Setup

### 5.1 Tested with STM32 and MSPM0L2228 Launchpad

| Test Case | Board           | Connection            		| Build Target   |
| --------- | --------------- | ------------------------------- | -------------- |
| I2C Scan  | STM32 nucleo    | SCL → PA1, SDA → PA0, GND → GND | i2c\_scan      |
|	    |		      |			      		|		 |
| Target    | MSPM0 LaunchPad | PA1 → SCL, PA0 → SDA, GND → GND | target\_eeprom |
| EEPROM    |                 |             	      		|                |

---

## 6. Test results & Observation

### Issue Summary
On `lp_mspm0l2228`, the SysTick timer continues to run and raise interrupts while the CPU is in Standby mode.
This behavior is incorrect because, in Standby mode, SysTick should be not available in STANDBY STOP or SHUTDOWN mode,
allowing peripherals like I2C to operate in low-power mode without waking the CPU.

---

### Test Description
- The MSPM0 I2C controller is configured to operate in low-power mode (should not require CPU wakeup for transactions).
- An external STM32 Nucleo board runs `i2c_scan` to detect the MSPM0 at address `0x54` (configured as target EEPROM).
- The `i2c_scan` will skip the `0x54` address so that MSPM0 is not disturbed for testing

---

### Expected Behavior
- SysTick **should not be available** in Standby mode → CPU remains asleep.
- I2C transactions should complete without CPU wakeup, as MSPM0’s I2C can work autonomously in low-power mode.

---

### Observed Behavior

| Step                                  | Expected                          | Observed                                  | Status|
|---------------------------------------|-----------------------------------|-------------------------------------------|-------|
| CPU enters Standby during `k_sleep()` | SysTick stops, no CPU wakeups     | SysTick continues firing, ISR executes    |   ❌  |
| I2C scan from STM32 skips `0x54`      | Device not found in scan          | Device skipped, no start signal observed  |   ✅  |

---
