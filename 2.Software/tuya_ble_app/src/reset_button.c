#include "tuya_ble_log.h"
#include "tuya_data_save.h"

#include "multi_button.h"

#define RESET_BUTTON_PORT	GPIO_PC3
#define TIME_MS	1000

unsigned long button_attach_time = 0;

struct Button reset_button;

static uint8_t read_reset_button_gpio()
{
	if (gpio_read(RESET_BUTTON_PORT) == 0) {
		return 0;
	} else {
		return 1;
	}
}

static void reset_button_callback(void *button)
{
	tuya_ble_status_t ret;

	ret = tuya_ble_device_factory_reset();
	if (ret == TUYA_BLE_SUCCESS) {
		TUYA_APP_LOG_DEBUG("device reset success");
		/* erase flash data */
		erase_device_data();
	} else {
		TUYA_APP_LOG_DEBUG("device reset error");
	}
}

void reset_button_init(void)
{
    gpio_set_func(RESET_BUTTON_PORT ,AS_GPIO);
    gpio_set_output_en(RESET_BUTTON_PORT, 0);
    gpio_set_input_en(RESET_BUTTON_PORT ,1);
    gpio_setup_up_down_resistor(RESET_BUTTON_PORT, PM_PIN_PULLUP_10K);

    button_init(&reset_button, read_reset_button_gpio, 0);

    button_attach(&reset_button, DOUBLE_CLICK, reset_button_callback);

    button_start(&reset_button);
}

void reset_button_task(void)
{
	if (clock_time_exceed(button_attach_time, 5 * TIME_MS)) {
		button_attach_time = clock_time();
		button_ticks();
	}
}
