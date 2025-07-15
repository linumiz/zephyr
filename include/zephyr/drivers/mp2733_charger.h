
#ifndef CHARGER_MP2733_H_
#define CHARGER_MP2733_H_

#include <zephyr/drivers/charger.h>

int set_property(enum charger_property prop, union charger_propval *val);
int get_property(enum charger_property prop, union charger_propval *Val);
int charge_enable(const bool enable);

#endif 
