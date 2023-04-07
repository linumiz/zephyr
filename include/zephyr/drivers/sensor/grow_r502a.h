/*
 * Copyright (c) 2022 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_GROW_R502A_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_GROW_R502A_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

/*LED glow control code*/
#define LED_CTRL_BREATHING 0x01
#define LED_CTRL_FLASHING 0x02
#define LED_CTRL_ON_ALWAYS 0x03
#define LED_CTRL_OFF_ALWAYS 0x04
#define LED_CTRL_ON_GRADUALLY 0x05
#define LED_CTRL_OFF_GRADUALLY 0x06

/*LED glow speed code*/
#define LED_SPEED_MAX 0x00
#define LED_SPEED_HALF 0x50
#define LED_SPEED_MIN 0xFF

/*LED color code*/
#define LED_COLOR_RED 0x01
#define LED_COLOR_BLUE 0x02
#define LED_COLOR_PURPLE 0x03

struct r502a_led_params {
	uint8_t ctrl_code;
	uint8_t color_idx;
	uint8_t speed; /* Speed 0x00-0xff */
	uint8_t cycle; /* Number of cycles | 0-infinite, 1-255 */
};

enum sensor_channel_grow_r502a {
	/** Fingerprint template count, ID number for enrolling and searching*/
	SENSOR_CHAN_FINGERPRINT = SENSOR_CHAN_PRIV_START,
};

enum sensor_trigger_type_grow_r502a {
	/** Trigger fires when a touch is detected. */
	SENSOR_TRIG_TOUCH = SENSOR_TRIG_PRIV_START,
};

enum sensor_attribute_grow_r502a {
	/** To capture finger data */
	SENSOR_ATTR_R502A_CAPTURE = SENSOR_ATTR_PRIV_START,
	/** To convert captured data to image and store in RAM buffer*/
	SENSOR_ATTR_R502A_CONVERT,
	/** To create template from RAM buffers during capture*/
	SENSOR_ATTR_R502A_RECORD_CREATE,
	/** Add template to the sensor record storage */
	SENSOR_ATTR_R502A_RECORD_ADD,
	/** To find requested data in record storage */
	SENSOR_ATTR_R502A_RECORD_FIND,
	/** To delete mentioned data from record storage */
	SENSOR_ATTR_R502A_RECORD_DEL,
	/** To empty the storage record*/
	SENSOR_ATTR_R502A_RECORD_EMPTY,
	/** To get available position to store data on record storage */
	SENSOR_ATTR_R502A_RECORD_FREE_IDX,
	/** To load template from storage to RAM buffer of sensor*/
	SENSOR_ATTR_R502A_RECORD_LOAD,
	/** To template data stored in sensor's RAM buffer*/
	SENSOR_ATTR_R502A_COMPARE,
	/** To control device LED */
	SENSOR_ATTR_R502A_DEVICE_LED,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_GROW_R502A_H_ */
