#ifndef __TUYA_APP_ADC_H__
#define __TUYA_APP_ADC_H__

#include "gpio.h"

unsigned int get_adc_value(GPIO_PinTypeDef gpio_pin);

unsigned int get_battery_adc_value(void);
unsigned char get_battery_value(void);

unsigned int get_motor_adc_value(void);

void update_battery_value_task(void);

#endif /* __TUYA_APP_ADC_H__ */

