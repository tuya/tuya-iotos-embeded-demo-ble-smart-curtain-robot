#include "tuya_ble_log.h"
#include "drivers.h"

#include "reset_button.h"
#include "tuya_app_led.h"
#include "tuya_app_motor.h"
#include "tuya_curtain_control.h"
#include "tuya_dp_process.h"
#include "tuya_app_lis2dw12.h"
#include "tuya_app_opt3004.h"
#include "tuya_app_adc.h"
#include "tuya_data_save.h"
#include "tuya_curtain_robot_func_logic.h"

void device_init(void)
{
	reset_button_init();
    led_init();
    motor_init();
    dp_init();

    if (!lis2dw12_init()) {
		*curtain_robot.dp_fault.dp_data |= 0x02;
		TUYA_APP_LOG_ERROR("lis2dw12 init error");
    }

    if (!opt3004_init()) {
    	*curtain_robot.dp_fault.dp_data |= 0x04;
    	TUYA_APP_LOG_ERROR("opt3004 init error");
    }

	/* update fault */
    dp_update_single(curtain_robot.dp_fault);

	read_device_data();
	led_on();
}

void device_task(void)
{
	reset_button_task();
	curtain_percent_control_stop_task();
	calculate_total_time_task();
	auto_power_task();
	illumination_task();
	update_battery_value_task();
	update_current_illumination_task();
	curtain_control_stop_task();

	motion_detection_task();
}
