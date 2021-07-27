#ifndef __TUYA_DP_PROCESS_H__
#define __TUYA_DP_PROCESS_H__

/* tuya data point type */
#define TUYA_DP_TYPE_BOOL		1
#define TUYA_DP_TYPE_VALUE		2
#define TUYA_DP_TYPE_STRING		3
#define TUYA_DP_TYPE_ENUM		4
#define TUYA_DP_TYPE_BITMAP		5

/* control data point enum value */
#define CTRL_OPEN_E		0
#define CTRL_STOP_E		1
#define CTRL_CLOSE_E	2
#define CTRL_CONTINUE_E	3

typedef unsigned char AUTO_POWER_STATE;
#define AUTO_POWER_IDLE		0
#define AUTO_POWER_OPEN		1
#define AUTO_POWER_CLOSE	2

/* control back data point enum value */
#define CTRL_BACK_FORWARD_E	0
#define CTRL_BACK_BACK_E	1

/* motor run mode */
typedef unsigned char RUN_MODE_T;
#define RUN_MODE_IDLE				0
#define RUN_MODE_CONTROL			1
#define RUN_MODE_PERCENT_CONTROL	2
#define RUN_MODE_CALCULATE_TIME		3

typedef unsigned char MOVEMENT_STATE;
#define ROBOT_STATIC	0
#define ROBOT_MOTION	1

/* curtain robot data point ID */
#define	DPID_CONTROL				1
#define DPID_PERCENT_CONTROL		2
#define DPID_PERCENT_STATE			3
#define DPID_CONTROL_BACK			5
#define DPID_AUTO_POWER				6
#define DPID_TIME_TOTAL				10
#define DPID_FAULT					12
#define DPID_BATTERY_PERCENTAGE		13
#define DPID_CALCULATE_TIME			101
#define DPID_ILLUMINATION_SET		102
#define DPID_ILLUMINATION_SWITCH	103
#define DPID_CURRENT_ILLUMINATION	104

typedef struct {
	unsigned char dp_id;
	unsigned char dp_type;
	unsigned char dp_data_len;
	unsigned char *dp_data;
}DataPoint;

typedef struct {
	DataPoint dp_control;
	DataPoint dp_percent_control;
	DataPoint dp_percent_state;
	DataPoint dp_control_back;
	DataPoint dp_auto_power;
	DataPoint dp_time_total;
	DataPoint dp_fault;
	DataPoint dp_battery_percentage;
	DataPoint dp_calculate_time;
	DataPoint dp_illumination_set;
	DataPoint dp_illumination_switch;
	DataPoint dp_current_illumination;

	unsigned long control_start_time;
	unsigned long control_end_time;

	unsigned long run_start_time;
	unsigned long run_end_time;

	unsigned long get_lis2dw12_data_time;
	AUTO_POWER_STATE auto_power_state;

	unsigned int motor_compare_voltage;

	short lis2dw12_x_static_value;
	short lis2dw12_y_static_value;
	short lis2dw12_z_static_value;

	RUN_MODE_T motor_run_mode;

	MOVEMENT_STATE robot_move_state;

}CURTAIN_ROBOT;

extern CURTAIN_ROBOT curtain_robot;

void dp_init(void);
void dp_update_all(void);
void dp_update_single(DataPoint dp);
void dp_update(unsigned char dp_id, unsigned char dp_type, unsigned char *dp_date, unsigned short dp_data_len);
void dp_process(unsigned char *dp_date, unsigned short dp_len);
#endif /* __TUYA_DP_PROCESS_H__ */

