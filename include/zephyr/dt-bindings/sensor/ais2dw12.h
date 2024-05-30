/*
 * Copyright (c) 2024 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_AIS2DW12_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_AIS2DW12_H_

#include <zephyr/dt-bindings/dt-util.h>

/* Operating Mode */
#define AIS2DW12_POWER_MODE_12BIT		0
#define AIS2DW12_POWER_MODE_2			1
#define AIS2DW12_POWER_MODE_3			2
#define AIS2DW12_POWER_MODE_4			3
#define AIS2DW12_SINGLE_POWER_MODE_12BIT	5
#define AIS2DW12_SINGLE_POWER_MODE_2		6
#define AIS2DW12_SINGLE_POWER_MODE_3		7
#define AIS2DW12_SINGLE_POWER_MODE_4		8

/* Data rate */
#define AIS2DW12_DT_ODR_OFF	0
#define AIS2DW12_DT_ODR_1Hz6	1
#define AIS2DW12_DT_ODR_12Hz5	2
#define AIS2DW12_DT_ODR_25Hz	3
#define AIS2DW12_DT_ODR_50Hz	4
#define AIS2DW12_DT_ODR_100Hz	5

/* Accelerometer Full-scale */
#define AIS2DW12_DT_FS_2G  0 /* 2g (0.061 mg/LSB)  */
#define AIS2DW12_DT_FS_4G  1 /* 4g (0.122 mg/LSB)  */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_AIS2DW12_H_ */
