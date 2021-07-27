#include "adc.h"
#include "drivers.h"

#include "tuya_ble_log.h"
#include "tuya_dp_process.h"
#include "tuya_app_adc.h"

#define BATTERY_ADC_PORT GPIO_PB6
#define MOTOR_ADC_PORT GPIO_PB5

#define BATTERY_ADC_UPPER_LIMIT 4119
#define BATTERY_ADC_LOWER_LIMIT	3800

#define TIME_MS	1000
#define BARRATY_UPDATE_TIME	30*60

unsigned long update_battery_time = 0;
unsigned int time_count = 0;

unsigned int get_adc_value(GPIO_PinTypeDef gpio_pin)
{
	adc_init();
	adc_base_init(gpio_pin);
	adc_power_on_sar_adc(1);
	return adc_sample_and_get_result();
}

unsigned int get_battery_adc_value(void)
{
	unsigned int adc_value;

	adc_value = get_adc_value(BATTERY_ADC_PORT);

	return adc_value;
}

unsigned char get_battery_value(void)
{
	unsigned int battery_adc_value;
	unsigned char battery_value;

	battery_adc_value = get_battery_adc_value() * 2;

	if (battery_adc_value >= BATTERY_ADC_UPPER_LIMIT) {
		battery_value = 100;
	} else {
		battery_value = ((battery_adc_value-BATTERY_ADC_LOWER_LIMIT)*100) / (BATTERY_ADC_UPPER_LIMIT-BATTERY_ADC_LOWER_LIMIT);
	}
	//TUYA_APP_LOG_DEBUG("battery_value : %d", battery_value);
	return battery_value;
}

unsigned int get_motor_adc_value(void)
{
	unsigned int adc_value;

	adc_value = get_adc_value(MOTOR_ADC_PORT);

	return adc_value;
}

void update_battery_value_task(void)
{
	if (clock_time_exceed(update_battery_time, 1000 * TIME_MS)) {
		update_battery_time = clock_time();
		time_count++;
		if (time_count >= BARRATY_UPDATE_TIME) {
			time_count = 0;
			*curtain_robot.dp_battery_percentage.dp_data = get_battery_value();
			dp_update_single(curtain_robot.dp_battery_percentage);
		}

	}
}
