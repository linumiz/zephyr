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

#define R502A_BAUD_9600 1
#define R502A_BAUD_19200 2
#define R502A_BAUD_38400 4
#define R502A_BAUD_57600 6
#define R502A_BAUD_115200 12

/*LED glow control code*/
enum r502a_led_ctrl_code {
	R502A_LED_CTRL_BREATHING = 0x01,
	R502A_LED_CTRL_FLASHING,
	R502A_LED_CTRL_ON_ALWAYS,
	R502A_LED_CTRL_OFF_ALWAYS,
	R502A_LED_CTRL_ON_GRADUALLY,
	R502A_LED_CTRL_OFF_GRADUALLY,
};

/* LED glow speed code
 * if needed, use desired speed between 0-255
 */
enum r502a_led_speed {
	R502A_LED_SPEED_MAX = 0x00,
	R502A_LED_SPEED_HALF = 0x50,
	R502A_LED_SPEED_MIN = 0xFF,
};

/* LED glowing cycle
 * if needed, use desired cycle count between 1-255
 */
enum r502a_led_cycle {
	R502A_LED_CYCLE_INFINITE = 0x00,
	R502A_LED_CYCLE_1,
	R502A_LED_CYCLE_2,
	R502A_LED_CYCLE_3,
	R502A_LED_CYCLE_4,
	R502A_LED_CYCLE_5,
	R502A_LED_CYCLE_255 = 0xFF,
};

/*LED color code*/
enum r502a_led_color_idx {
	R502A_LED_COLOR_RED = 0x01,
	R502A_LED_COLOR_BLUE,
	R502A_LED_COLOR_PURPLE,
};

#define R502A_LED_CYCLE(x) (x & 0xFF)
#define R502A_LED_SPEED(x) ((x & 0xFF) << 8)
#define R502A_LED_COLOR(x) ((x & 0xFF) << 16)
#define R502A_LED_CTRL(x) ((x & 0xFF) << 24)

/* @param ctrl - LED glowing control code, refer enum r502a_led_ctrl_code
 * @param colour - LED glowing color, refer enum r502a_led_color_idx
 * @param speed - LED glowing speed, refer enum r502a_led_speed
 * @param cycle - LED glowing cycle, refer enum r502a_led_cycle
 */
#define R502A_LED_PARAMS(ctrl, color, speed, cycle) \
				(R502A_LED_CTRL(ctrl) | R502A_LED_COLOR(color) | \
					R502A_LED_SPEED(speed) | R502A_LED_CYCLE(cycle))

enum r502a_sec_level {
	R502A_SEC_LEVEL_1 = 1,
	R502A_SEC_LEVEL_2,
	R502A_SEC_LEVEL_3,
	R502A_SEC_LEVEL_4,
	R502A_SEC_LEVEL_5
};

enum r502a_data_len {
	R502A_PKG_LEN_32,
	R502A_PKG_LEN_64,
	R502A_PKG_LEN_128,
	R502A_PKG_LEN_256
};

enum r502a_sys_param_set {
	R502A_BAUD_RATE = 4,
	R502A_SECURITY_LEVEL,
	R502A_DATA_PKG_LEN
};

struct r502a_sys_param {
	uint16_t status_reg;
	uint16_t system_id;
	uint16_t lib_size;
	uint16_t sec_level;
	uint32_t addr;
	uint16_t data_pkt_size;
	uint32_t baud;
} __packed;

enum sensor_channel_grow_r502a {
	/** Fingerprint template count, ID number for enrolling and searching*/
	SENSOR_CHAN_FINGERPRINT = SENSOR_CHAN_PRIV_START,
};

enum sensor_trigger_type_grow_r502a {
	/** Trigger fires when a touch is detected. */
	SENSOR_TRIG_TOUCH = SENSOR_TRIG_PRIV_START,
};

enum sensor_attribute_grow_r502a {
	/** To capture finger data in 2 RAM buffers
	  * create template from those and store it in
	  * RAM buffer 1.
	  */
	SENSOR_ATTR_R502A_CAPTURE = SENSOR_ATTR_PRIV_START,
	/** Add template to the sensor record storage */
	/**
	 * @param val->val1	record index for template to be
	 * 			stored in the sensor device's flash
	 *			library.
	 */
	SENSOR_ATTR_R502A_RECORD_ADD,
	/** To find requested data in record storage */
	/**
	 * @result val->val1	matched record index.
	 *	   val->val2	matching score.
	 */
	SENSOR_ATTR_R502A_RECORD_FIND,
	/** To delete mentioned data from record storage */
	/**
	 * @param val->val1	record start index to be deleted.
	 * @param val->val2	number of records to be deleted.
	 */
	SENSOR_ATTR_R502A_RECORD_DEL,
	/** To empty the storage record*/
	SENSOR_ATTR_R502A_RECORD_EMPTY,
	/** To get available position to store data on record storage */
	SENSOR_ATTR_R502A_RECORD_FREE_IDX,
	/** To load template from storage to RAM buffer of sensor*/
	/**
	 * @param val->val1	record start index to be loaded in
	 *			device internal RAM buffer.
	 */
	SENSOR_ATTR_R502A_RECORD_LOAD,
	/** To template data stored in sensor's RAM buffer*/
	/**
	 * @result val->val1	match result.
	 *			[R502A_FINGER_MATCH_FOUND or 
	 *			R502A_FINGER_MATCH_NOT_FOUND]
	 *	   val->val2	matching score.	
	 */
	SENSOR_ATTR_R502A_COMPARE,
	/** To control device LED */
	/** 
	 * @param val->val1	use R502A_LED_PARAMS macro with desired
	 *			LED configurations.
	 */
	SENSOR_ATTR_R502A_DEVICE_LED,
	/** To read and write device's system parameters */
	/** sensor_attr_set
	 * @param val->val1 parameter number from enum r502a_sys_param_set.
	 * @param val->val2 content to be written for the respective parameter.
	 */
	/** sensor_attr_get
	 * @result val->ex.data buffer holds the system parameter values.
	 */
	SENSOR_ATTR_R502A_SYS_PARAM,
	/** To upload template data out of sensor */
	/**
	 *
	 * @result val->ex.data	buffer having collected template data.
	 *	   val->ex.len	length of the collected template data.
	 */
	SENSOR_ATTR_R502A_UPLOAD,
	/** To download template data into the sensor */
	/* @param val->val1	RAM buffer index either 1 or 2
	 *			index values other than these
	 *			considered as 2.
	 * @param val->ex	buffer holding template data.
	 */
	SENSOR_ATTR_R502A_DOWNLOAD,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_GROW_R502A_H_ */
