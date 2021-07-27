#include "tuya_ble_log.h"
#include "tuya_ble_api.h"

#include "tuya_app_motor.h"
#include "tuya_data_save.h"
#include "tuya_app_adc.h"
#include "tuya_curtain_control.h"
#include "tuya_dp_process.h"
#include "tuya_app_opt3004.h"

CURTAIN_ROBOT curtain_robot;

/* curtain robot state data */
unsigned char dp_data_control = CTRL_STOP_E;
unsigned char dp_data_percent_control = 0;
unsigned char dp_data_percent_state = 0;
unsigned char dp_data_control_back = CTRL_BACK_FORWARD_E;
unsigned char dp_data_auto_power = FALSE;
unsigned char dp_data_time_total[2] = {0x00, 0x00};//10 000ms:{0x27, 0x10}
unsigned char dp_data_fault = 0x00;
unsigned char dp_data_battery_percentage = 0;
unsigned char dp_data_position_best = 0;
unsigned char dp_data_calculate_time = FALSE;
unsigned char dp_data_illumination_set = 0;
unsigned char dp_data_illumination_switch = FALSE;
unsigned char dp_data_current_illumination[2] = {0x00, 0x00};

void dp_init(void)
{
	curtain_robot.get_lis2dw12_data_time = 0;
	curtain_robot.auto_power_state = AUTO_POWER_IDLE;

	/* Initial run timing */
	curtain_robot.run_start_time = 0;
	curtain_robot.run_end_time = 0;

	/* get compare voltage */
	curtain_robot.motor_compare_voltage = get_motor_adc_value();

	/* 3 axis data init */
	curtain_robot.lis2dw12_x_static_value = 0;
	curtain_robot.lis2dw12_y_static_value = 0;
	curtain_robot.lis2dw12_z_static_value = 0;

	/* motor run mode init */
	curtain_robot.motor_run_mode = RUN_MODE_IDLE;
	curtain_robot.robot_move_state = ROBOT_STATIC;

	/* control */
	curtain_robot.dp_control.dp_id = DPID_CONTROL;
	curtain_robot.dp_control.dp_type = TUYA_DP_TYPE_ENUM;
	curtain_robot.dp_control.dp_data_len = 1;
	curtain_robot.dp_control.dp_data = &dp_data_control;

	/* percent control */
	curtain_robot.dp_percent_control.dp_id = DPID_PERCENT_CONTROL;
	curtain_robot.dp_percent_control.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_percent_control.dp_data_len = 1;
	curtain_robot.dp_percent_control.dp_data = &dp_data_percent_control;

	/* percent state  */
	curtain_robot.dp_percent_state.dp_id = DPID_PERCENT_STATE;
	curtain_robot.dp_percent_state.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_percent_state.dp_data_len = 1;
	curtain_robot.dp_percent_state.dp_data = &dp_data_percent_state;

	/* motor control back */
	curtain_robot.dp_control_back.dp_id = DPID_CONTROL_BACK;
	curtain_robot.dp_control_back.dp_type = TUYA_DP_TYPE_ENUM;
	curtain_robot.dp_control_back.dp_data_len = 1;
	curtain_robot.dp_control_back.dp_data = &dp_data_control_back;

	/* auto power */
	curtain_robot.dp_auto_power.dp_id = DPID_AUTO_POWER;
	curtain_robot.dp_auto_power.dp_type = TUYA_DP_TYPE_BOOL;
	curtain_robot.dp_auto_power.dp_data_len = 1;
	curtain_robot.dp_auto_power.dp_data = &dp_data_auto_power;

	/* time total */
	curtain_robot.dp_time_total.dp_id = DPID_TIME_TOTAL;
	curtain_robot.dp_time_total.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_time_total.dp_data_len = 2;
	curtain_robot.dp_time_total.dp_data = dp_data_time_total;

	/* fault */
	curtain_robot.dp_fault.dp_id = DPID_FAULT;
	curtain_robot.dp_fault.dp_type = TUYA_DP_TYPE_BITMAP;
	curtain_robot.dp_fault.dp_data_len = 1;
	curtain_robot.dp_fault.dp_data = &dp_data_fault;

	/* battery percent value */
	dp_data_battery_percentage = get_battery_value();
	curtain_robot.dp_battery_percentage.dp_id = DPID_BATTERY_PERCENTAGE;
	curtain_robot.dp_battery_percentage.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_battery_percentage.dp_data_len = 1;
	curtain_robot.dp_battery_percentage.dp_data = &dp_data_battery_percentage;

	/* calculate time switch */
	curtain_robot.dp_calculate_time.dp_id = DPID_CALCULATE_TIME;
	curtain_robot.dp_calculate_time.dp_type = TUYA_DP_TYPE_BOOL;
	curtain_robot.dp_calculate_time.dp_data_len = 1;
	curtain_robot.dp_calculate_time.dp_data = &dp_data_calculate_time;

	/* illumination set */
	curtain_robot.dp_illumination_set.dp_id = DPID_ILLUMINATION_SET;
	curtain_robot.dp_illumination_set.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_illumination_set.dp_data_len = 1;
	curtain_robot.dp_illumination_set.dp_data = &dp_data_illumination_set;

	/* illumination switch */
	curtain_robot.dp_illumination_switch.dp_id = DPID_ILLUMINATION_SWITCH;
	curtain_robot.dp_illumination_switch.dp_type = TUYA_DP_TYPE_BOOL;
	curtain_robot.dp_illumination_switch.dp_data_len = 1;
	curtain_robot.dp_illumination_switch.dp_data = &dp_data_illumination_switch;

	/* current illumination value */
	curtain_robot.dp_current_illumination.dp_id = DPID_CURRENT_ILLUMINATION;
	curtain_robot.dp_current_illumination.dp_type = TUYA_DP_TYPE_VALUE;
	curtain_robot.dp_current_illumination.dp_data_len = 2;
	curtain_robot.dp_current_illumination.dp_data = dp_data_current_illumination;
}

void dp_update(unsigned char dp_id, unsigned char dp_type, unsigned char *dp_date, unsigned short dp_data_len)
{
	uint16_t i = 0;
	unsigned char dp_data_array[16];

	dp_data_array[0] = dp_id;
	dp_data_array[1] = dp_type;
	dp_data_array[2] = dp_data_len;
	for (i = 0; i < dp_data_array[2]; i++) {
		dp_data_array[i + 3] = *(dp_date + i);
	}
	tuya_ble_dp_data_report(dp_data_array, (dp_data_array[2] + 3));
}

void dp_update_single(DataPoint dp)
{
	dp_update(dp.dp_id,	dp.dp_type, dp.dp_data, dp.dp_data_len);
}

void dp_update_all(void)
{
	/* control  */
	dp_update_single(curtain_robot.dp_control);

	/* percent control */
	dp_update_single(curtain_robot.dp_control);

	/* percent state */
	dp_update_single(curtain_robot.dp_percent_state);

	/* control back */
	dp_update_single(curtain_robot.dp_control_back);

	/* auto power */
	dp_update_single(curtain_robot.dp_auto_power);

	/* time total */
	dp_update_single(curtain_robot.dp_time_total);

	/* fault */
	dp_update_single(curtain_robot.dp_fault);

	/* battery percentage */
	*curtain_robot.dp_battery_percentage.dp_data = get_battery_value();
	dp_update_single(curtain_robot.dp_battery_percentage);

	/* calculate time */
	dp_update_single(curtain_robot.dp_calculate_time);

	/* illumination set */
	dp_update_single(curtain_robot.dp_illumination_set);

	/* illumination switch */
	dp_update_single(curtain_robot.dp_illumination_switch);

	/* current illumination value */
	update_current_illumination();
}

void dp_process(unsigned char *dp_date, unsigned short dp_len)
{
	if (dp_len <= 0) {
		return;
	}

	switch (dp_date[0]) {
	case DPID_CONTROL:
		if (dp_date[3] == CTRL_OPEN_E) {
			curtain_control_open();
		} else if (dp_date[3] == CTRL_STOP_E) {
			curtain_control_pause();
		} else if (dp_date[3] == CTRL_CLOSE_E) {
			curtain_control_close();
		} else {
			TUYA_APP_LOG_ERROR("CONTINUE");
		}
		break;
	case DPID_PERCENT_CONTROL:
		curtain_percent_control(*curtain_robot.dp_percent_state.dp_data, dp_date[dp_len - 1]);
		break;
	case DPID_CONTROL_BACK:
		if (dp_date[3] == CTRL_BACK_BACK_E) {
			*(curtain_robot.dp_control_back.dp_data) = CTRL_BACK_BACK_E;
		} else {
			*(curtain_robot.dp_control_back.dp_data) = CTRL_BACK_FORWARD_E;
		}
		dp_update_single(curtain_robot.dp_control_back);
		save_device_data();
		break;
	case DPID_AUTO_POWER:
		*(curtain_robot.dp_auto_power.dp_data) = dp_date[dp_len - 1];
		dp_update_single(curtain_robot.dp_auto_power);
		save_device_data();
		break;
	case DPID_CALCULATE_TIME:
		*(curtain_robot.dp_calculate_time.dp_data) = dp_date[dp_len - 1];
		if (*(curtain_robot.dp_calculate_time.dp_data) == TRUE) {
			calculate_total_time_init();
		} else {
			calculate_total_time_stop();
		}
		break;
	case DPID_ILLUMINATION_SET:
		*(curtain_robot.dp_illumination_set.dp_data) = dp_date[dp_len - 1];
		dp_update_single(curtain_robot.dp_illumination_set);
		save_device_data();
		break;
	case DPID_ILLUMINATION_SWITCH:
		*(curtain_robot.dp_illumination_switch.dp_data) = dp_date[dp_len - 1];
		dp_update_single(curtain_robot.dp_illumination_switch);
		save_device_data();
		break;
	default:
		TUYA_APP_LOG_ERROR("No this DP");
		break;
	}
}
