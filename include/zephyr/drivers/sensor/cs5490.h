/*
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_CS5490_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_CS5490_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_channel_cs5490 {
	/** Instantaneous Current in Amps */
	SENSOR_CHAN_INST_CURRENT = SENSOR_CHAN_PRIV_START,
	/** Instantaneous Voltage in Volts */
	SENSOR_CHAN_INST_VOLTAGE,
	/** Instantaneous Active Power in Watts */
	SENSOR_CHAN_INST_POWER,
	/** RMS Current in Amps */
	SENSOR_CHAN_RMS_CURRNT,
	/** RMS Voltage in Amps */
	SENSOR_CHAN_RMS_VOLTAGE,
	/** Active Power in Watts */
	SENSOR_CHAN_ACTIVE_POWER,
	/** Appparent Power in Volt-Ampere(VA) */
	SENSOR_CHAN_APPARENT_POWER,
	/** Reactive Power in Volt-ampere reactive(VAR) */
	SENSOR_CHAN_REACTIVE_POWER,
	/** Power Factor */
	SENSOR_CHAN_POWER_FACTOR,
	/** Peak Current in Amps */
	SENSOR_CHAN_PEAK_CURRENT,
	/** Peak Voltage in Volts */
	SENSOR_CHAN_PEAK_VOLTAGE,
	/** Frequency in Hz */
	SENSOR_CHAN_FREQUENCY,
};

enum sensor_trigger_type_cs5490 {
	/** Trigger fires when Overcurrent condition is detected */
	SENSOR_TRIG_OVERCURRENT = SENSOR_TRIG_PRIV_START,
	/** Trigger fires when Voltage is out of Range */
	SENSOR_TRIG_OUT_OF_RANGE_VOLTAGE,
	/** Trigger fires when Current is out of Range */
	SENSOR_TRIG_OUT_OF_RANGE_CURRENT,
	/** Trigger fires when Power is out of Range */
	SENSOR_TRIG_OUT_OF_RANGE_POWER,
};

enum sensor_attribute_cs5490 {
	/** Baudrate runtime change attribute */
	SENSOR_ATTR_BAUDRATE= SENSOR_ATTR_PRIV_START,
	/** Overcurrent Threshold attribute */
	SENSOR_ATTR_OVERCURRENT_THRESHOLD,
	/** Overcurrent Threshold Duration attribute */
	SENSOR_ATTR_OVERCURRENT_DURATION,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_CS5490_H_ */
