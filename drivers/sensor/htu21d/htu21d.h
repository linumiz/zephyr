/* HTU21D temperature & humidity sensor with i2c
 *
 * Copyright (c) 2022 Linumiz
 * Author: Arunmani <arunmani@linumiz.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_HTU21D_H_
#define ZEPHYR_DRIVERS_SENSOR_HTU21D_H_

#define HTU21D_READ_TEMP_H	0xE3
#define HTU21D_READ_HUMID_H	0xE5
#define HTU21D_READ_TEMP_NH	0xF3
#define HTU21D_READ_HUMID_NH	0xF5
#define HTU21D_SOFT_RESET	0xFE
#define MEASURE_WAIT_MS		50


#endif /* ZEPHYR_DRIVERS_SENSOR_HTU21D_H_ */
