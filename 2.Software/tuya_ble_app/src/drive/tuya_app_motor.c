#include "gpio.h"
#include "drivers.h"

#include "tuya_ble_log.h"

#include "tuya_dp_process.h"
#include "tuya_app_motor.h"

MOTOR_RUN_STATE motor_state;

void motor_init(void)
{
    gpio_set_func(REV_PORT, AS_GPIO);
    gpio_set_output_en(REV_PORT, 1);
    gpio_set_input_en(REV_PORT, 0);

    gpio_set_func(FWD_PORT, AS_GPIO);
    gpio_set_output_en(FWD_PORT, 1);
    gpio_set_input_en(FWD_PORT, 0);

    motor_sleep();
}

void motor_forward(void)
{
    gpio_write(FWD_PORT, 1);
    gpio_write(REV_PORT, 0);

    motor_state = MOTOR_FORWAED;
}

void motor_reverse(void)
{
    gpio_write(FWD_PORT, 0);
    gpio_write(REV_PORT, 1);

    motor_state = MOTOR_REVERSE;
}

void motor_stop(void)
{
	gpio_write(FWD_PORT, 1);
	gpio_write(REV_PORT, 1);

    motor_state = MOTOR_STOP;
}

void motor_sleep(void)
{
	gpio_write(FWD_PORT, 0);
	gpio_write(REV_PORT, 0);

    motor_state = MOTOR_SLEEP;
}

MOTOR_RUN_STATE get_motor_state(void)
{
    return motor_state;
}

void curtain_open(void)
{
	if (*(curtain_robot.dp_control_back.dp_data) == CTRL_BACK_BACK_E) {
		motor_forward();
	} else {
		motor_reverse();
	}

	*curtain_robot.dp_control.dp_data = CTRL_OPEN_E;
	if (curtain_robot.motor_run_mode == RUN_MODE_CONTROL) {
		dp_update_single(curtain_robot.dp_control);
	}
}

void curtain_close(void)
{
	if (*(curtain_robot.dp_control_back.dp_data) == CTRL_BACK_BACK_E) {
		motor_reverse();
	} else {
		motor_forward();
	}

	*curtain_robot.dp_control.dp_data = CTRL_CLOSE_E;
	if (curtain_robot.motor_run_mode == RUN_MODE_CONTROL) {
		dp_update_single(curtain_robot.dp_control);
	}
}

void curtain_pause(void)
{
	motor_sleep();

	*curtain_robot.dp_control.dp_data = CTRL_STOP_E;
	if (curtain_robot.motor_run_mode == RUN_MODE_CONTROL) {
		dp_update_single(curtain_robot.dp_control);
	}
}

