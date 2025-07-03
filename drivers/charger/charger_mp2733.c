/*              
 * Copyright 2025 Linumiz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mp_mp2733

#include "charger_mp2733.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(mp_mp2733, CONFIG_CHARGER_LOG_LEVEL);

/**
 *  TODO i2c_reg_update_byte_dt is not working anywhere in my code so i need to figure out why 
 *  TODO now the update is used and tehre is no error becuase of the value that is passed to the api is
 *  32 bit long , and now it is changed to 8 bit and this works fitered , but the problem is
 *  wheen i use the get property to get the value that i have written in the reg , it will raise an error
 *  saying wrong general input output. i don't what i am going to do now and how to slve this ??
 *  */

struct mp2733_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec ce_gpio;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec otg_gpio;
};

struct mp2733_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	charger_status_notifier_t charger_status_notifier;
	charger_online_notifier_t charger_online_notifier;
	struct k_work int_routine_work;
	uint32_t constant_charge_current_max_ua;
	uint32_t constant_charge_voltage_max_uv;
	uint32_t precharge_current_ua;
	uint32_t charge_term_current_ua;

	/* Chip specific */
	uint32_t min_sys_voltage_uv;
	uint32_t input_voltage_min_uv;
	uint32_t input_current_max_ua;
	uint32_t otg_dschg_voltage_uv;
	uint32_t otg_dschg_current_ua;
	uint32_t thermal_regulation_threshold;
	//	enum charger_status state;
	//	enum charger_online online;
};

#if 1 /* valid Vin */
static int mp2733_is_valid_input(const struct device *dev)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t valid_inp;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_STATUS_IND,
			&valid_inp);
	if(ret < 0)
	{
		return ret;
	}

	if(!(valid_inp & MP2733_STATUS_IND_VIN_MASK))
	{
		/* not a valid input source */
		return -1;
	}
	return 0;
}
#endif /* end of valid Vin*/

#if 1 /* charge enable */
static bool mp2733_get_charge_enable(const struct device *dev)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_ctrl_0;
	int charger_enable;
	int ce_pin = 0;
	int ret;

	ret = mp2733_is_valid_input(dev);
	if(ret < 0)
	{
		return ret;
	}

	if (config->ce_gpio.port != NULL) {
		ce_pin = gpio_pin_get_dt(&config->ce_gpio);
	}

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_CHRG_CTRL_VSYS_CONF,
			&chrg_ctrl_0);
	if (ret) {
		return ret;
	}

	charger_enable = chrg_ctrl_0 & MP2733_CHRG_CONFIG_MASK;
	if (charger_enable != 0) {
		if (config->ce_gpio.port && ce_pin) {
			return true;
		}
	}
	return false;
}

static int mp2733_set_charge_enable(const struct device *dev, 
		const bool enable)
{
	const struct mp2733_config *const config = dev->config;
	int ret;

	ret = mp2733_is_valid_input(dev);
	if(ret < 0)
	{
		return ret;
	}
	if (config->ce_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&config->ce_gpio, enable);
		if (ret) {
			return ret;
		}
	}

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_CHRG_CTRL_VSYS_CONF,
			MP2733_CHRG_CONFIG_MASK, 
			(enable ? MP2733_CHRG_ENABLE : 0) 
			<< MP2733_CHRG_CONFIG_SHIFT);
}
#endif  /* charge enable end */

#if 1 /*  Const Charge Current Lim */
static int mp2733_get_ichrg_curr(const struct device *dev, 
		uint32_t *current_ua)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_curr;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_CCHRG_I_CONF, 
			&chrg_curr);
	if(ret < 0)
	{
		return ret;
	}
	*current_ua = (((chrg_curr & MP2733_CCHRG_I_CONFIG_MASK)*
				MP2733_CCHRG_I_STEP_UA )+ MP2733_CCHRG_I_OFFSET);
	return 0;
}

static int mp2733_set_ichrg_curr(const struct device *dev, int chrg_curr)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data * data = dev->data;
	uint8_t chrg_i;
	int ret=0;

	chrg_curr = CLAMP(chrg_curr, MP2733_CCHRG_I_MIN_UA, 
			data->constant_charge_current_max_ua);

	ret = mp2733_set_charge_enable(dev, 0);
	if(ret){
		return ret;
	}

	chrg_i = ((chrg_curr - MP2733_CCHRG_I_OFFSET )/MP2733_CCHRG_I_STEP_UA );

	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_CCHRG_I_CONF, 
			MP2733_CCHRG_I_CONFIG_MASK, chrg_i);
	if(ret)
	{
		return ret;
	}	
	ret = mp2733_set_charge_enable(dev, 1);
	if(ret){
		return ret;
	}
	return ret;
}
#endif

#if 1 /* Const Charge Voltage Limit */
static int mp2733_get_chrg_volt(const struct device *dev, 
		uint32_t *voltage_uv)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_volt;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_CCHRG_V_REGUL, 
			&chrg_volt);
	if(ret < 0)
	{
		return ret;
	}

	*voltage_uv = (((chrg_volt & MP2733_VBATT_V_CONFIG_MASK) *
			       	MP2733_VBATT_V_STEP_UV) + MP2733_VBATT_V_OFFSET );
	return 0;
}

static int mp2733_set_chrg_volt(const struct device *dev,
		uint32_t chrg_volt)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	uint8_t chrg_v;

	chrg_volt = CLAMP(chrg_volt, MP2733_VBATT_V_MIN_UV, data->constant_charge_voltage_max_uv);

	chrg_v = ((chrg_volt - MP2733_VBATT_V_OFFSET) / 
			MP2733_VBATT_V_STEP_UV);
	return i2c_reg_update_byte_dt(&config->i2c, MP2733_CCHRG_V_REGUL ,
			MP2733_VBATT_V_CONFIG_MASK, chrg_v << 1);
}
#endif

#if 1 /* Input Current */
static int mp2733_get_input_curr_lim(const struct device *dev, 
		uint32_t *current_ua)
{
	printk("input curr lim\n");
	const struct mp2733_config *const config = dev->config;
	uint8_t inp_curr;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_INP_I_LIM,
			&inp_curr);
	if(ret < 0)
	{
		printk("get prop read err : %d", ret);
		return ret;
	}

	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_INP_I_LIM, 
			MP2733_INP_I_LIM_CONFIG_MASK, 
			MP2733_INP_I_LIM_CONFIG_MASK);
	if(ret)
	{
		printk("get prop udpdate err : %d", ret);
	}

	*current_ua =(((inp_curr & MP2733_INP_I_LIM_CONFIG_MASK) * 
			MP2733_INP_I_LIM_STEP_UA) + MP2733_INP_I_LIM_OFFSET);
	return 0;
}

static int mp2733_set_input_curr_lim(const struct device *dev, int iindpm)
{
	printk("input curr set %d\n", iindpm);
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data * data = dev->data;
	uint8_t inp_val;

	iindpm = CLAMP(iindpm, MP2733_INP_I_LIM_MIN_UA, data->input_current_max_ua);

	inp_val = ((iindpm - MP2733_INP_I_LIM_OFFSET)/
			MP2733_INP_I_LIM_STEP_UA); 

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_INP_I_LIM, 
			MP2733_INP_I_LIM_CONFIG_MASK, inp_val);

}
#endif

#if 1 /* Input voltage */
static int mp2733_get_input_volt_lim(const struct device *dev,
		uint32_t *voltage_uv)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t inp_volt;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, 
			MP2733_INP_V_LIM, &inp_volt);
	if(ret < 0)
	{
		return ret;
	}

	*voltage_uv = (((inp_volt & MP2733_INP_V_LIM_CONFIG_MASK) * 
				MP2733_INP_V_LIM_STEP_UV ) + MP2733_INP_V_LIM_OFFSET);
	return 0;
}

static int mp2733_set_input_volt_lim(const struct device *dev, int vindpm)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data * data = dev->data;
	uint8_t inp_v;

	vindpm = CLAMP(vindpm, MP2733_INP_V_LIM_MIN_UV, data->input_voltage_min_uv);
	inp_v = ((vindpm - MP2733_INP_V_LIM_OFFSET)/
			MP2733_INP_V_LIM_STEP_UV);

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_INP_V_LIM,
			MP2733_INP_V_LIM_CONFIG_MASK, inp_v);

}
#endif

#if 1 /* precharge */
static int mp2733_get_prechrg_curr(const struct device *dev,
		uint32_t *current_ua)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t prechrg_curr;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_PRE_CHRG_TERM_I,
			&prechrg_curr);
	if(ret < 0)
	{
		return ret;
	}
	*current_ua = (((prechrg_curr & MP2733_PRECHRG_CONFIG_MASK)
				* MP2733_PRECHRG_I_STEP_UA ) + MP2733_PRECHRG_I_OFFSET); 
	return 0;
}

static int mp2733_set_prechrg_curr(const struct device *dev, int pre_current)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t pre_i;

	pre_current = CLAMP(pre_current, MP2733_PRECHRG_I_MIN_UA,
			MP2733_PRECHRG_I_MAX_UA);

	pre_i = ((pre_current - MP2733_PRECHRG_I_OFFSET)/ 
			MP2733_PRECHRG_I_STEP_UA);

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_PRE_CHRG_TERM_I, 
			MP2733_PRECHRG_CONFIG_MASK, (pre_i << 4));

}
#endif 

#if 1 /* termcharge */
static int mp2733_get_term_curr(const struct device *dev, uint32_t *current_ua)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t termchrg_curr;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_PRE_CHRG_TERM_I,
			&termchrg_curr);
	if(ret < 0)
	{
		return ret;
	}
	*current_ua = (((termchrg_curr & MP2733_TERMCHRG_CONFIG_MASK)
				* MP2733_TERMCHRG_I_STEP_UA ) + MP2733_TERMCHRG_I_OFFSET ); 
	return 0;
}

static int mp2733_set_term_curr(const struct device *dev, int term_current)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	uint8_t term_i;

	term_current = CLAMP(term_current, MP2733_TERMCHRG_I_MIN_UA,
			MP2733_TERMCHRG_I_MAX_UA);

	term_i = ((term_current - MP2733_TERMCHRG_I_OFFSET)/ 
			MP2733_TERMCHRG_I_STEP_UA);

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_PRE_CHRG_TERM_I,
		       	MP2733_TERMCHRG_CONFIG_MASK, term_i);
}
#endif

#if 1 /* otg volt dischg */
static int mp2733_get_otg_dschg_volt(const struct device *dev, 
		uint32_t *dschg_volt_uv)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t otg_dschg_volt;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, 
			MP2733_ADC_CTRL_OTG_CONF, &otg_dschg_volt);
	if(ret < 0)
	{
		return ret;
	}

	*dschg_volt_uv = (((otg_dschg_volt & MP2733_VIN_DSCHG_V_CONFIG_MASK)
				* MP2733_VIN_DSCHG_V_STEP_UV ) + MP2733_VIN_DSCHG_V_OFFSET);
	return 0;
}
static int mp2733_set_otg_dschg_volt(const struct device *dev, int vin_dschg)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	uint8_t dschg_v;

	vin_dschg = CLAMP(vin_dschg, MP2733_VIN_DSCHG_V_MIN_UV,
			data->otg_dschg_voltage_uv);

	dschg_v = ((vin_dschg - MP2733_VIN_DSCHG_V_OFFSET)/ 
			MP2733_VIN_DSCHG_V_STEP_UV);

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_ADC_CTRL_OTG_CONF,
			MP2733_VIN_DSCHG_V_CONFIG_MASK, dschg_v << 3);
}
#endif 

#if 1 /* otg current dischg */ 
static int mp2733_get_otg_dschg_curr(const struct device *dev,
		uint32_t *dschg_curr_ua)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t dschg_curr;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, 
			MP2733_ADC_CTRL_OTG_CONF, &dschg_curr);
	if(ret < 0)
	{
		return ret;
	}
	/*TODO find a way to retrieve the values for the bits enabled to get 
	 * the curr value
	 * */
	*dschg_curr_ua = (dschg_curr & MP2733_IIN_DSCHG_I_CONFIG_MASK);
	return 0;

}

static int mp2733_set_otg_dschg_curr(const struct device *dev, int dschg_i)
{
	const struct mp2733_config *const config = dev->config;
	
	return i2c_reg_update_byte_dt(&config->i2c, MP2733_ADC_CTRL_OTG_CONF, 
			MP2733_IIN_DSCHG_I_CONFIG_MASK, MP2733_IIN_DSCHG_I_DEF_MASK);
}
#endif 

#if 1 /* sys min */
static int mp2733_set_min_sys_volt(const struct device *dev, int vsysmin)
{
	const struct mp2733_config *const config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_CHRG_CTRL_VSYS_CONF, 
			MP2733_VSYS_CONFIG_MASK, MP2733_VSYS_DEF_MASK << 1);
}
#endif 

#if 1 /* Input Voltage ADC */
static int mp2733_get_vin_adc(const struct device * dev, uint32_t *volt)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t vinp;
	int ret; 

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_INPUT_V_ADC,
			&vinp);
	if(ret < 0)
	{
		return ret;
	}

	*volt = vinp * MP2733_VINP_V_ADC_STEP_UV;
	return 0;
}
#endif
#if 1 /* Input Current ADC */
static int mp2733_get_iin_adc(const struct device * dev, uint32_t *curr)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t curr_inp;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_INPUT_I_ADC,
			&curr_inp);
	if(ret < 0)
	{
		return ret;
	}

	*curr = curr_inp * MP2733_CHRG_I_ADC_STEP_UA;
	return 0;
}
#endif
#if 1 /* Battery Voltage ADC */                                                  
static int mp2733_get_vbatt_adc(const struct device * dev, uint32_t *volt)       
{                                                                              
	const struct mp2733_config *const config = dev->config;                
	uint8_t vinp;                                                          
	int ret;                                                               

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_BATT_V_ADC,           
			&vinp);                                                
	if(ret < 0)                                                            
	{                                                                      
		return ret;                                                    
	}                                                                      

	*volt = vinp * MP2733_BATT_V_ADC_STEP_UV;                              
	return 0;                                                              
}                                                                              
#endif 

#if 1 /* Batt Current ADC */
static int mp2733_get_ichrg_adc(const struct device * dev, uint32_t *curr)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t curr_inp;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_CHRG_I_ADC,
			&curr_inp);
	if(ret < 0)
	{
		return ret;
	}

	*curr = curr_inp * MP2733_CHRG_I_ADC_STEP_UA;
	return 0;
}
#endif

#if 1 /* System Voltage ADC */                                                  
static int mp2733_get_vsys_adc(const struct device * dev, uint32_t *volt)       
{                                                                              
	const struct mp2733_config *const config = dev->config;                
	uint8_t vinp;                                                          
	int ret;                                                               

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_SYS_V_ADC,           
			&vinp);                                                
	if(ret < 0)                                                            
	{                                                                      
		return ret;                                                    
	}                                                                      

	*volt = vinp * MP2733_SYS_V_ADC_STEP_UV;                              
	return 0;                                                              
}                                                                              
#endif 

static int mp2733_get_online_status(const struct device *dev, enum charger_online *online)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_stat;
	int online_stat;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_STATUS_IND, &chrg_stat);
	if(ret)
	{
		return ret;
	}

	online_stat = chrg_stat & MP2733_STATUS_IND_VIN_MASK;
	if(!online_stat || online_stat == MP2733_OTG_MODE) {
		*online = CHARGER_ONLINE_OFFLINE;
	} else {
		*online = CHARGER_ONLINE_FIXED;
	}
	return 0;
}

/* REG0x0D fault indicator */
static int mp2733_get_health(const struct device *dev, enum charger_health *health)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t fault;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_FAULT_IND, &fault);
	if(ret)
	{
		return ret;
	}

	*health = CHARGER_HEALTH_UNKNOWN;
	switch (fault & MP2733_NTC_FAULT) {
		case MP2733_TEMP_NORMAL:
			*health = CHARGER_HEALTH_GOOD;
			break;
		case MP2733_TEMP_COLD:
			*health = CHARGER_HEALTH_COLD;
			break;
		case MP2733_TEMP_HOT:
			*health = CHARGER_HEALTH_HOT;
			break;
		case MP2733_TEMP_COOL:
			*health = CHARGER_HEALTH_COOL;
			break;
		case MP2733_TEMP_WARM:
			*health = CHARGER_HEALTH_WARM;
			break;
	}

	if (fault & MP2733_THERMSHUT_FAULT) {
		*health = CHARGER_HEALTH_OVERHEAT;
	} else if (fault & (MP2733_OTG_FAULT |
				MP2733_BAT_FAULT | MP2733_INPUT_FAULT)) {
		*health = CHARGER_HEALTH_OVERVOLTAGE;
	} else if(fault & MP2733_WATCHDOG_FAULT){
		*health = CHARGER_HEALTH_WATCHDOG_TIMER_EXPIRE;
	}
	return 0;
}

static int mp2733_get_charger_type(const struct device *dev, enum charger_charge_type *type)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_stat, chrg_ctl;
	int32_t ibat, itrickle_max;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_STATUS_IND, &chrg_stat);
	if (ret) {
		return ret;
	}

	if (mp2733_get_charge_enable(dev)) {
		chrg_stat = FIELD_GET(MP2733_CHRG_STATUS, chrg_stat);
		switch (chrg_stat) {
			case MP2733_CHRG_DONE:
				__fallthrough;
			case MP2733_NOT_CHRG:
				*type = CHARGER_CHARGE_TYPE_NONE;
				break;
			case MP2733_CONST_CHRG :
				*type = CHARGER_CHARGE_TYPE_STANDARD;
				break;
			case MP2733_TRICKLE_CHRG:
				*type = CHARGER_CHARGE_TYPE_TRICKLE;
		}
	} else {
		*type = CHARGER_CHARGE_TYPE_UNKNOWN;
	}

	return 0;
}

static int mp2733_get_charger_status(const struct device *dev,
	       	enum charger_status *charge_status)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_stat;
	uint8_t type, status;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_STATUS_IND, &chrg_stat);
	if (ret) {
		return ret;
	}

	if (mp2733_get_charge_enable(dev)) {
		status = FIELD_GET(MP2733_CHRG_STATUS, chrg_stat);
	} else {
		status = MP2733_NOT_CHRG;
	}

	type = FIELD_GET(MP2733_STATUS_IND_VIN_MASK, chrg_stat);

	if (!type || (type == MP2733_OTG_MODE)) {
		*charge_status = CHARGER_STATUS_DISCHARGING;
	} else if (!status) {
		*charge_status = CHARGER_STATUS_NOT_CHARGING;
	} else {
		*charge_status = CHARGER_STATUS_CHARGING;
	}

	return 0;
}


static int mp2733_get_usb_type(const struct device *dev, enum charger_usb_type *type)
{
	const struct mp2733_config *const config = dev->config;
	uint8_t chrg_stat;
	int ret;

	ret = i2c_reg_read_byte_dt(&config->i2c, MP2733_STATUS_IND, &chrg_stat);
	if (ret) {
		return ret;
	}

	switch (chrg_stat & MP2733_STATUS_IND_VIN_MASK) {
		case MP2733_USB_SDP:
			*type = CHARGER_USB_TYPE_SDP;
			break;
		case MP2733_USB_CDP:
			*type = CHARGER_USB_TYPE_CDP;
			break;
		case MP2733_USB_DCP:
			*type = CHARGER_USB_TYPE_DCP;
			break;
		case MP2733_USB_APPLE_1A:
			__fallthrough;
		case MP2733_USB_APPLE_2_1A:
			__fallthrough;
		case MP2733_USB_APPLE_2_4A:
			*type = CHARGER_USB_TYPE_APPLE_BRICK_ID;	
		case MP2733_USB_UNKNOWN:
			__fallthrough;
		default:
			*type = CHARGER_USB_TYPE_UNKNOWN;
	}

	return 0;
}


static int mp2733_get_prop(const struct device *dev, charger_prop_t prop,
		union charger_propval *val)
{
	switch (prop) {
	case CHARGER_PROP_ONLINE:
		return mp2733_get_online_status(dev, &val->online);
	case CHARGER_PROP_CHARGE_TYPE:
		return mp2733_get_charger_type(dev, &val->charge_type);
	case CHARGER_PROP_HEALTH:
		return mp2733_get_health(dev, &val->health);
	case CHARGER_PROP_STATUS:
		return mp2733_get_charger_status(dev, &val->status);
	case CHARGER_PROP_USB_TYPE:
		return mp2733_get_usb_type(dev, &val->usb_type);
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return mp2733_get_ichrg_curr(dev, &val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return mp2733_get_chrg_volt(dev, &val->const_charge_voltage_uv);
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
		return mp2733_get_prechrg_curr(dev, &val->precharge_current_ua);
	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
		return mp2733_get_term_curr(dev, &val->charge_term_current_ua);
	case CHARGER_PROP_BATTERY_VOLTAGE_NOW:
		return mp2733_get_vbatt_adc(dev, &val->battery_voltage_now_uv);
	case CHARGER_PROP_BATTERY_CURRENT_NOW:
		return mp2733_get_ichrg_adc(dev, &val->battery_current_now_ua);
	case CHARGER_PROP_INPUT_VOLTAGE_NOW:
		return mp2733_get_vin_adc(dev, &val->input_voltage_now_uv);
	case CHARGER_PROP_INPUT_CURRENT_NOW:
		return mp2733_get_iin_adc(dev, &val->input_current_now_ua);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return mp2733_get_input_curr_lim(dev, 
				&val->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return mp2733_get_input_volt_lim(dev, 
				&val->input_voltage_regulation_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static int mp2733_set_prop(const struct device *dev, charger_prop_t prop,
		const union charger_propval *val)
{
	struct mp2733_data *data = dev->data;
	switch (prop) {
	case CHARGER_PROP_CONSTANT_CHARGE_CURRENT_UA:
		return mp2733_set_ichrg_curr(dev, val->const_charge_current_ua);
	case CHARGER_PROP_CONSTANT_CHARGE_VOLTAGE_UV:
		return mp2733_set_chrg_volt(dev, val->const_charge_voltage_uv);
	case CHARGER_PROP_PRECHARGE_CURRENT_UA:
		return mp2733_set_prechrg_curr(dev, val->precharge_current_ua);
	case CHARGER_PROP_CHARGE_TERM_CURRENT_UA:
		return mp2733_set_term_curr(dev, val->charge_term_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_CURRENT_UA:
		return mp2733_set_input_curr_lim(dev, val->input_current_regulation_current_ua);
	case CHARGER_PROP_INPUT_REGULATION_VOLTAGE_UV:
		return mp2733_set_input_volt_lim(dev, val->input_voltage_regulation_voltage_uv);
		// TODO
	case CHARGER_PROP_STATUS_NOTIFICATION:
		data->charger_status_notifier = val->status_notification;
		break;
	case CHARGER_PROP_ONLINE_NOTIFICATION:
		data->charger_online_notifier = val->online_notification;
		break;
		// TODO
	default:
		return -EINVAL;
	}

	return 0;
}

static int mp2733_validate_dt(struct mp2733_data *data)
{
	if(!IN_RANGE(data->min_sys_voltage_uv, MP2733_VSYS_MIN_V_MIN_UV, 
				MP2733_VSYS_MIN_V_MAX_UV )){
		data->min_sys_voltage_uv = MP2733_VSYS_MIN_V_DEF_UV;
	}
	if(!IN_RANGE(data->input_voltage_min_uv, MP2733_INP_V_LIM_MIN_UV,
				MP2733_INP_V_LIM_MAX_UV )){
		data->input_voltage_min_uv = MP2733_INP_V_LIM_DEF_UV;
	}
	if(!IN_RANGE(data->input_current_max_ua, MP2733_INP_I_LIM_MIN_UA ,
				MP2733_INP_I_LIM_MAX_UA )){
		data->input_current_max_ua = MP2733_INP_I_LIM_DEF_UA;
	}
	if(!IN_RANGE(data->otg_dschg_voltage_uv, MP2733_VIN_DSCHG_V_MIN_UV,
				MP2733_VIN_DSCHG_V_MAX_UV )){
		data->otg_dschg_voltage_uv = MP2733_VIN_DSCHG_V_DEF_UV;
	}
	return 0;
}

/* otg and disc should be called in the fault handler */
#if 1 /* otg mode */
static int mp2733_otg_mode_enable(const struct device * dev, const bool enable)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	int ret;

	if(config->otg_gpio.port != NULL) {
		ret = gpio_pin_set_dt(&config->otg_gpio, enable);
		if(ret) {
			return ret;
		}
	}
	/* enabling ntc for otg mode */
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_NTC_CONF_THERM_REGUL,
			MP2733_NTC_OTG_EN, MP2733_NTC_OTG_EN);
	if(ret < 0)
	{
		return ret;
	}
	ret =mp2733_set_otg_dschg_volt(dev, data->otg_dschg_voltage_uv);
	if(ret < 0)
	{
		return ret;
	}

	ret = mp2733_set_otg_dschg_curr(dev, data->otg_dschg_current_ua);
	if(ret < 0)
	{
		return ret;
	}

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_CHRG_CTRL_VSYS_CONF,
			MP2733_CHRG_CONFIG_OTG_MODE, 
			(enable ? MP2733_CHRG_CONFIG_OTG_MODE :0));
}

/* disc mode */
static int mp2733_disc_mode_enable(const struct device * dev, const bool enable)
{
	const struct mp2733_config *const config = dev->config;

	return i2c_reg_update_byte_dt(&config->i2c, MP2733_BATTFET_CONF, 
			MP2733_BATTFET_DIS,(enable ? MP2733_BATTFET_DIS : 0));
}

#endif
static int mp2733_set_heat_mgmt(const struct device *dev)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	int ret;

	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_NTC_CONF_THERM_REGUL, 
			MP2733_NTC_OTG_EN, MP2733_NTC_OTG_EN);
	if(ret){
		return ret;
	}
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_NTC_CONF_THERM_REGUL, 
			MP2733_AICO_EN, MP2733_AICO_EN);
	if(ret){
		return ret;
	}
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_NTC_CONF_THERM_REGUL, 
			MP2733_TEMP_REG, data->thermal_regulation_threshold << 2);
	if(ret){
		return ret;
	}
	return mp2733_set_min_sys_volt(dev, data->min_sys_voltage_uv);	
}

#if 1 /* hw init */
static int mp2733_hw_init(const struct device *dev)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	int ret;

	// charge control reg 2 is clear by reset
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_INP_V_LIM, MP2733_REG_RESET,
			MP2733_REG_RESET);
	if (ret) {
		return ret;
	}

	/* watch dog disable */
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_TIMER_CONF, 
			MP2733_WATCHDOG_TIMER_MASK, MP2733_WATCHDOG_TIMER_DIS);
	if (ret) {
		return ret;
	}

	// const charge current set
	ret = mp2733_set_ichrg_curr(dev, data->constant_charge_current_max_ua);
	if (ret) {
		return ret;
	}

	// const charge voltage set
	ret = mp2733_set_chrg_volt(dev, data->constant_charge_voltage_max_uv);
	if (ret) {
		return ret;
	}

	// pre current prop set
	ret = mp2733_set_prechrg_curr(dev, data->precharge_current_ua);
	if (ret) {
		return ret;
	}

	// term current prop set
	ret = mp2733_set_term_curr(dev, data->charge_term_current_ua);
	if (ret) {
		return ret;
	}

	// input volt lim set
	ret = mp2733_set_input_volt_lim(dev, data->input_voltage_min_uv);
	if (ret) {
		return ret;
	}

	// input current lim set
	ret = mp2733_set_input_curr_lim(dev, data->input_current_max_ua);
	if (ret) {
		return ret;
	}

	// otg volt lim set
	ret = mp2733_set_otg_dschg_volt(dev, data->otg_dschg_voltage_uv);
	if(ret)
	{
		return ret;
	}

	// otg curr lim set	
	ret = mp2733_set_otg_dschg_curr(dev, data->otg_dschg_current_ua);
	if(ret)
	{
		return ret;
	}
	
	// heat mgmt
	ret = mp2733_set_heat_mgmt(dev);
	if(ret)
	{
		return ret;
	}

	// adc setup
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_ADC_CTRL_OTG_CONF, MP2733_ADC_RATE, MP2733_ADC_RATE);
	if(ret) {
		return ret;
	}
	ret = i2c_reg_update_byte_dt(&config->i2c, MP2733_ADC_CTRL_OTG_CONF, MP2733_ADC_START, MP2733_ADC_START);
	if(ret)
	{
		return ret;
	}
	return 0;
}
#endif

static int mp2733_init(const struct device *dev)
{
	const struct mp2733_config *const config = dev->config;
	struct mp2733_data *data = dev->data;
	uint8_t val;
	int ret;

	data->dev = dev;

	/* charge enable pin config */
	if (config->ce_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->ce_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->ce_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_DBG("Assuming charge enable pin is pulled low");
	}

	/* otg pin config */
	if (config->otg_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->otg_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->otg_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	} else {
		LOG_DBG("Assuming otg pin is pulled high");
	}

	ret = mp2733_validate_dt(data);	
	if(ret) 
	{
		return ret;
	}

	ret = mp2733_hw_init(dev);
	if(ret)
	{
		return ret;
	}
	/*TODO*/
#if 0 
	if (config->int_gpio.port != NULL) {
                ret = bq2562x_configure_interrupt(dev);
                if (ret) {
                        return ret;
                }
        }
#endif
	return ret;

}


static DEVICE_API(charger, mp2733_driver_api) = {
	.get_property = mp2733_get_prop,
	.set_property = mp2733_set_prop,
	.charge_enable = mp2733_set_charge_enable,
};

#define MP2733_INIT(inst)                                                                         \
	\
	static const struct mp2733_config mp2733_config_##inst = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.ce_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, ce_gpios, {}),                           \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}),                         \
		.otg_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, otg_gpios, {}),			\
	};                                                                                         \
	\
	static struct mp2733_data mp2733_data_##inst = {                                         \
		.constant_charge_current_max_ua =                                                  \
		DT_INST_PROP(inst, constant_charge_current_max_microamp),                  \
		.constant_charge_voltage_max_uv =                                                  \
		DT_INST_PROP(inst, constant_charge_voltage_max_microvolt),                 \
		.precharge_current_ua = DT_INST_PROP(inst, precharge_current_microamp),            \
		.charge_term_current_ua = DT_INST_PROP(inst, termcharge_current_microamp),        \
		.min_sys_voltage_uv = DT_INST_PROP(inst, min_sys_voltage_microvolt),            \
		.input_voltage_min_uv = DT_INST_PROP(inst, input_voltage_limit_microvolt),      \
		.input_current_max_ua = DT_INST_PROP(inst, input_current_limit_microamp),       \
		.otg_dschg_voltage_uv = DT_INST_PROP(inst, input_voltage_dschg_otg_microvolt),	\
		.otg_dschg_current_ua = DT_INST_PROP(inst, input_current_dschg_otg_microamp),	\
		.thermal_regulation_threshold = DT_INST_PROP(inst, thermal_regulation_threshold) \
	};                                                                                         \
	\
	DEVICE_DT_INST_DEFINE(inst, mp2733_init, NULL, &mp2733_data_##inst,                      \
			&mp2733_config_##inst, POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY,   \
			&mp2733_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MP2733_INIT)
