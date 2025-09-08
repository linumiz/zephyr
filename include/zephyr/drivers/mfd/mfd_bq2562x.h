/*
 * Copyright 2025 Linumiz GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_BQ2562X_H_
#define ZEPHYR_DRIVERS_MFD_BQ2562X_H_

#include <zephyr/sys/slist.h>

#define BQ2562X_RESERVED_LSB     0x00
#define BQ2562X_RESERVED_MSB     0x01
#define BQ2562X_CHRG_CTRL_1      0x16
#define BQ2562X_CHRG_STAT_1      0x1e
#define BQ2562X_FAULT_STAT_0     0x1f
#define BQ2562X_CHRG_FLAG_0      0x20
#define BQ2562X_FAULT_FLAG_0     0x22
#define BQ2562X_FN_DISABE_0      0x27
#define BQ2562X_ADC_IBUS_LSB     0x28
#define BQ2562X_ADC_IBUS_MSB     0x29
#define BQ2562X_ADC_VBUS_LSB     0x2c
#define BQ2562X_ADC_VBUS_MSB     0x2d

/* REG0x16_Charger_Control_1 */
#define BQ2562X_CHRG_EN		BIT(5)
#define BQ2562X_CHRG_HIZ	BIT(4)
#define BQ2562X_WATCHDOG_MASK 	GENMASK(1, 0)
#define BQ2562X_WATCHDOG_DIS  	0

/* REG0x1E_Charger_Status_1 */
#define BQ2562X_CHG_STAT_MSK  GENMASK(4, 3)
#define BQ2562X_NOT_CHRGING   0
#define BQ2562X_TRICKLE_CHRG  1
#define BQ2562X_TAPER_CHRG    2
#define BQ2562X_TOP_OFF_CHRG  3
#define BQ2562X_PRECHG_MAX_UA 620000

#define BQ2562X_VBUS_STAT_MSK GENMASK(2, 0)
#define BQ2562X_USB_SDP	      BIT(0)
#define BQ2562X_USB_CDP       BIT(1)
#define BQ2562X_USB_DCP       (BIT(1) | BIT(0))
#define BQ2562X_UNKNOWN_500MA BIT(2)
#define BQ2562X_NON_STANDARD  (BIT(2) | BIT(0))
#define BQ2562X_HVDCP         (BIT(2) | BIT(1))
#define BQ2562X_OTG_MODE      (BIT(2) | BIT(1) | BIT(0))

/* REG0x1F_FAULT_Status_0 */
#define BQ2562X_TEMP_TS_NORMAL          0x00
#define BQ2562X_TEMP_COLD               BIT(0)
#define BQ2562X_TEMP_HOT                BIT(1)
#define BQ2562X_TEMP_COOL               (BIT(1) | BIT(0))
#define BQ2562X_TEMP_WARM               BIT(2)
#define BQ2562X_TEMP_PRECOOL            (BIT(2) | BIT(0))
#define BQ2562X_TEMP_PREWARM            (BIT(2) | BIT(1))
#define BQ2562X_TEMP_PIN_BIAS_REF_FAULT (BIT(2) | BIT(1) | BIT(0))
#define BQ2562X_TEMP_MASK               GENMASK(2, 0)
#define BQ2562X_TSHUT_STAT              BIT(3)
#define BQ2562X_OTG_FAULT_STAT          BIT(4)
#define BQ2562X_SYS_FAULT_STAT          BIT(5)
#define BQ2562X_BAT_FAULT_STAT          BIT(6)
#define BQ2562X_VBUS_FAULT_STAT         BIT(7)

/* REG0x28_IBUS_ADC */
#define BQ2562X_ADC_IBUS_SHIFT   1
#define BQ2562X_ADC_CURR_STEP_UA 2000

/* REG0x2C_VBUS_ADC */
#define BQ2562X_ADC_VBUS_STEP_UV 3970
#define BQ2562X_ADC_VBUS_SHIFT   2

/* Define the BQ2562X MFD interrupt callback function handler */
typedef void (*bq2562x_callback_handler_t)(const struct device *dev);

struct bq2562x_mfd_callback {
	sys_snode_t node;
	bq2562x_callback_handler_t cb;
	const struct device *dev;
};

/* Register the interrupt of BQ2562X MFD callback function */
void mfd_bq2562x_register_interrupt_callback(const struct device *dev,
					     struct bq2562x_mfd_callback *callback);

int mfd_bq2562x_reg_update_byte_dt(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				   uint8_t value);

int mfd_bq2562x_reg_read_byte_dt(const struct device *dev, uint8_t reg_addr, uint8_t *value);

int mfd_bq2562x_burst_read_dt(const struct device *dev, uint8_t start_addr, uint8_t *buf,
			      uint32_t num_bytes);

int mfd_bq2562x_burst_write_dt(const struct device *dev, uint8_t start_addr, uint8_t *buf,
			       uint32_t num_bytes);

int mfd_bq2562x_enable_interrupt_pin(const struct device *dev, bool enabled);

#endif /* ZEPHYR_DRIVERS_MFD_BQ2562X_H_ */
