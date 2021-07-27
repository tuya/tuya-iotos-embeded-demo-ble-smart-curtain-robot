#include "gpio.h"
#include "tuya_app_led.h"

void led_init(void)
{
	gpio_set_func(LED_PORT, AS_GPIO);
    gpio_set_output_en(LED_PORT, 1);
    gpio_set_input_en(LED_PORT, 0);

    gpio_write(LED_PORT, LED_OFF_LEVEL);
}

void led_on(void)
{
	gpio_write(LED_PORT, LED_ON_LEVEL);
}

void led_off(void)
{
	gpio_write(LED_PORT, LED_OFF_LEVEL);
}
