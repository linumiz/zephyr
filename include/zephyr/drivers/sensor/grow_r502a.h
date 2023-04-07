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
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ZEPHYR_DRIVERS_SENSOR_GROW_R502A_H_ */
