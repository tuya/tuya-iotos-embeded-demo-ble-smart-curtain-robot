#include "drivers.h"
#include "timer.h"
#include "tuya_ble_log.h"

#include "tuya_app_adc.h"
#include "tuya_data_save.h"
#include "tuya_dp_process.h"
#include "tuya_app_motor.h"
#include "tuya_curtain_control.h"

#define TIME_MS	1000

#define CURTAIN_ALL_ON	100
#define CURTAIN_ALL_OFF	0

unsigned char percent_target_value = 0;

unsigned char judge_block_flag = 1; /* Whether to start judging motor blocking */
unsigned long ADC_delay_time = 0;

unsigned long calc_start_time[2] = {0};
unsigned long calc_end_time[2] = {0};
unsigned long calculate_total_time_value[2] = {0, 0};

CALC_TIME_STATE calculate_time_state = CALC_TIME_IDLE;

unsigned long get_total_time(void)
{
	unsigned long total_time;

	total_time = curtain_robot.dp_time_total.dp_data[0];
	total_time = (total_time << 8) | curtain_robot.dp_time_total.dp_data[1];

	return total_time;
}

void curtain_control_open(void)
{
	if (curtain_robot.motor_run_mode == RUN_MODE_IDLE) {
		curtain_robot.motor_run_mode = RUN_MODE_CONTROL;
		curtain_robot.control_start_time = (clock_time() / sys_tick_per_us) / TIME_MS; //ms
		curtain_robot.run_end_time = (*(curtain_robot.dp_percent_state.dp_data) * (get_total_time() * 10)); //*10=*1000/100,ms->us
		curtain_robot.run_start_time = clock_time();
		curtain_open();
	}
}

void curtain_control_pause(void)
{
	unsigned long motor_run_time = 0;
	unsigned char motor_move_distance = 0;

	if (curtain_robot.motor_run_mode == RUN_MODE_CONTROL) {
		curtain_robot.control_end_time = (clock_time() / sys_tick_per_us) / TIME_MS; //ms

		/* calculate run time */
		motor_run_time = curtain_robot.control_end_time - curtain_robot.control_start_time;
		/* calculate motor current position */
		motor_move_distance = motor_run_time * CURTAIN_ALL_ON / get_total_time();
		if (*curtain_robot.dp_control.dp_data == CTRL_OPEN_E) {
			if (*curtain_robot.dp_percent_state.dp_data - motor_move_distance < CURTAIN_ALL_OFF) {
				*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_OFF;
			} else {
				*curtain_robot.dp_percent_state.dp_data -= motor_move_distance;
			}
		} else {
			if (*curtain_robot.dp_percent_state.dp_data + motor_move_distance > CURTAIN_ALL_ON) {
				*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_ON;
			} else {
				*curtain_robot.dp_percent_state.dp_data += motor_move_distance;
			}
		}
		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;

		*curtain_robot.dp_percent_control.dp_data = *curtain_robot.dp_percent_state.dp_data;
		save_device_data();

		/* update current position */
		dp_update_single(curtain_robot.dp_percent_state);
		/* percent control */
		dp_update_single(curtain_robot.dp_percent_control);

	} else if (curtain_robot.motor_run_mode == RUN_MODE_PERCENT_CONTROL) {
		curtain_robot.run_end_time = (clock_time() / sys_tick_per_us) / TIME_MS; //ms
		curtain_robot.run_start_time =  curtain_robot.run_start_time / sys_tick_per_us / 1000;

		motor_run_time = curtain_robot.run_end_time - curtain_robot.run_start_time;
		motor_move_distance = motor_run_time * CURTAIN_ALL_ON / get_total_time();
		if (*curtain_robot.dp_control.dp_data == CTRL_OPEN_E) {
			if (*curtain_robot.dp_percent_state.dp_data - motor_move_distance < CURTAIN_ALL_OFF) {
				*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_OFF;
			} else {
				*curtain_robot.dp_percent_state.dp_data -= motor_move_distance;
			}
		} else {
			if (*curtain_robot.dp_percent_state.dp_data + motor_move_distance > CURTAIN_ALL_ON) {
				*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_ON;
			} else {
				*curtain_robot.dp_percent_state.dp_data += motor_move_distance;
			}
		}
		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;

		/* update current position */
		dp_update_single(curtain_robot.dp_percent_state);
		*curtain_robot.dp_percent_control.dp_data = *curtain_robot.dp_percent_state.dp_data;
		dp_update_single(curtain_robot.dp_percent_control);
		save_device_data();
	} else if (curtain_robot.motor_run_mode == RUN_MODE_CALCULATE_TIME) {
		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;

		*curtain_robot.dp_calculate_time.dp_data = FALSE;
		dp_update_single(curtain_robot.dp_calculate_time);

	} else {
		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;
	}
}

void curtain_control_close(void)
{
	if (curtain_robot.motor_run_mode == RUN_MODE_IDLE) {
		curtain_robot.motor_run_mode = RUN_MODE_CONTROL;
		curtain_robot.control_start_time = (clock_time() / sys_tick_per_us) / TIME_MS; //ms

		curtain_robot.run_end_time = ((CURTAIN_ALL_ON - *curtain_robot.dp_percent_state.dp_data) * (get_total_time() * 10)); //*10=*1000/100,ms->us
		curtain_robot.run_start_time = clock_time();
		curtain_close();
	}
}

void curtain_control_stop_task(void)
{
	if (((curtain_robot.motor_run_mode == RUN_MODE_CONTROL)) && \
		(clock_time_exceed(curtain_robot.run_start_time, curtain_robot.run_end_time))) {

		if (*curtain_robot.dp_control.dp_data == CTRL_OPEN_E) {
			*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_OFF;
		} else {
			*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_ON;
		}

		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;
		save_device_data();

		/* percent state */
		dp_update_single(curtain_robot.dp_percent_state);

		/* percent control */
		*curtain_robot.dp_percent_control.dp_data = *curtain_robot.dp_percent_state.dp_data;
		dp_update_single(curtain_robot.dp_percent_control);
	}
}

void curtain_percent_control(unsigned char current_position, unsigned char target_position)
{
	unsigned long total_time;

	if ((current_position == target_position) || \
		(current_position < CURTAIN_ALL_OFF) || (current_position > CURTAIN_ALL_ON) || \
		(target_position < CURTAIN_ALL_OFF) || (target_position > CURTAIN_ALL_ON)) {
		TUYA_APP_LOG_ERROR("input error");
		*curtain_robot.dp_percent_state.dp_data = current_position;
		return;
	}

	if (curtain_robot.motor_run_mode == RUN_MODE_IDLE) {
		curtain_robot.motor_run_mode = RUN_MODE_PERCENT_CONTROL;
	} else {
		TUYA_APP_LOG_ERROR("motor_run_mode != RUN_MODE_IDLE");
		*curtain_robot.dp_percent_state.dp_data = current_position;
		return;
	}

	total_time = curtain_robot.dp_time_total.dp_data[0];
	total_time = (total_time << 8) | curtain_robot.dp_time_total.dp_data[1];

	if (total_time == 0) {
		*curtain_robot.dp_percent_state.dp_data = current_position;
		return;
	}

	if (current_position < target_position) {
		curtain_robot.run_end_time = ((target_position - current_position) * (total_time * 10)); //*10=*1000/100,ms->us
		curtain_robot.run_start_time = clock_time();
		curtain_close();
	} else {
		curtain_robot.run_end_time = (current_position - target_position) * (total_time * 10);
		curtain_robot.run_start_time = clock_time();
		curtain_open();
	}

	/* percent control state target position */
	percent_target_value = target_position;

	/* percent control */
	*curtain_robot.dp_percent_control.dp_data = target_position;
	dp_update_single(curtain_robot.dp_percent_control);
}

void curtain_percent_control_stop_task(void)
{
	if (((curtain_robot.motor_run_mode == RUN_MODE_PERCENT_CONTROL)) && \
		(clock_time_exceed(curtain_robot.run_start_time, curtain_robot.run_end_time))) {
		curtain_pause();
		curtain_robot.motor_run_mode = RUN_MODE_IDLE;

		save_device_data();

		/* percent state */
		*curtain_robot.dp_percent_state.dp_data = percent_target_value;
		dp_update_single(curtain_robot.dp_percent_state);
	}
}

void calculate_total_time_init(void)
{
	if (curtain_robot.motor_run_mode == RUN_MODE_IDLE) {
		curtain_robot.motor_run_mode = RUN_MODE_CALCULATE_TIME;
		TUYA_APP_LOG_ERROR("calculate_total_time_init run");
	} else {
		TUYA_APP_LOG_ERROR("calculate_total_time_init error:%d", curtain_robot.motor_run_mode);

		*curtain_robot.dp_calculate_time.dp_data = FALSE;
		dp_update_single(curtain_robot.dp_calculate_time);
		return;
	}

	dp_update_single(curtain_robot.dp_calculate_time);

	calculate_time_state = CALC_TIME_READY;

	judge_block_flag = 0;
	ADC_delay_time = clock_time();
}

void calculate_total_time_stop(void)
{
	dp_update_single(curtain_robot.dp_calculate_time);
	curtain_pause();
	curtain_robot.motor_run_mode = RUN_MODE_IDLE;
}


void calculate_total_time_task(void)
{
	unsigned int motor_adc_value = 0;
	unsigned short total_time; //ms

	if (*curtain_robot.dp_calculate_time.dp_data == TRUE) {
			if ((judge_block_flag == 0)&&(clock_time_exceed(ADC_delay_time, 500 * TIME_MS))) {
				judge_block_flag = 1;
			}

			if (judge_block_flag) {
				if((*curtain_robot.dp_control.dp_data == CTRL_OPEN_E) ||\
					(*curtain_robot.dp_control.dp_data == CTRL_CLOSE_E)) {
					motor_adc_value = get_motor_adc_value();
					if (motor_adc_value >= 15) {
						curtain_pause();

						judge_block_flag = 0;//Pause ADC acquisition for 500ms
						ADC_delay_time = clock_time();
					}
				}
			}

			switch(calculate_time_state) {
				case CALC_TIME_READY:
					if (*curtain_robot.dp_control.dp_data == CTRL_STOP_E) {
						curtain_open();
						calculate_time_state = CALC_TIME_CLOSE;
					}
				break;
				case CALC_TIME_CLOSE:
					if (*curtain_robot.dp_control.dp_data == CTRL_STOP_E) {
						curtain_close();
						calc_start_time[0] = clock_time();
						calculate_time_state = CALC_TIME_OPEN;
					}
				break;
				case CALC_TIME_OPEN:
					if (*curtain_robot.dp_control.dp_data == CTRL_STOP_E) {
						calc_end_time[0] = clock_time();
						curtain_open();
						calc_start_time[1] = clock_time();
						calculate_time_state = CALC_TIME_END;
					}
				break;
				case CALC_TIME_END:
					if (*curtain_robot.dp_control.dp_data == CTRL_STOP_E) {
						calc_end_time[1] = clock_time();
						calculate_total_time_value[0] = (calc_end_time[0] - calc_start_time[0])/sys_tick_per_us;
						calculate_total_time_value[1] = (calc_end_time[1] - calc_start_time[1])/sys_tick_per_us;
						if (calculate_total_time_value[0] < calculate_total_time_value[1]) {
							total_time = (unsigned short)(calculate_total_time_value[0] / TIME_MS);
						} else {
							total_time = (unsigned short)(calculate_total_time_value[1] / TIME_MS);
						}

						*curtain_robot.dp_calculate_time.dp_data = FALSE;

						dp_update_single(curtain_robot.dp_calculate_time);

						/* update total time */
						curtain_robot.dp_time_total.dp_data[0] = total_time>>8;
						curtain_robot.dp_time_total.dp_data[1] = total_time&0x00FF;
						dp_update_single(curtain_robot.dp_time_total);

						/* update percent control value */
						*curtain_robot.dp_percent_control.dp_data = CURTAIN_ALL_OFF;
						dp_update_single(curtain_robot.dp_percent_control);

						/* update */
						*curtain_robot.dp_percent_state.dp_data = CURTAIN_ALL_OFF;
						dp_update_single(curtain_robot.dp_percent_state);

						save_device_data();
						curtain_robot.motor_run_mode = RUN_MODE_IDLE;
					}
				break;
				default: break;
			}
	}
}
