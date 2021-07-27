#include "drivers.h"
#include "app_config.h"
#include "drivers/8258/i2c.h"

#include "tuya_ble_log.h"
#include "tuya_dp_process.h"
#include "tuya_app_motor.h"
#include "tuya_app_lis2dw12.h"
#include "tuya_curtain_control.h"

#define I2C_CLK_SPEED 200000

#define TIME_MS	1000

#define VIBRATION_HIGH	100
#define VIBRATION_LOW	-100

#define CURTAIN_ALL_ON	100
#define CURTAIN_ALL_OFF	0

short x_data_buf[100] = {0};
unsigned int clean_x_buf_count = 0;
unsigned char x_data_index = 0;

unsigned long motion_get_data_time = 0;
unsigned long motion_get_state_time = 0;
unsigned char motion_count = 0;
unsigned char clean_motion_count = 0;

unsigned char lis2dw12_init(void)
{
    i2c_gpio_set(I2C_GPIO_GROUP_C0C1);
    i2c_master_init(LIS2DW12_I2C_ADD_H, (unsigned char)(CLOCK_SYS_CLOCK_HZ / (4 * I2C_CLK_SPEED)));

    /* who am i */
    if (LIS2DW12_ID != i2c_read_byte(LIS2DW12_WHO_AM_I, 1)) {
        /* error */
        return 0;
    }

    i2c_write_byte(LIS2DW12_CTRL1, 1, 0x53);//0x53 : low power mode4
    i2c_write_byte(LIS2DW12_CTRL2, 1, 0x84);//
    i2c_write_byte(LIS2DW12_CTRL6, 1, 0x34);//

    return 1;
}

short get_lis2dw12_x_value(void)
{
	unsigned char x_axis_low;
	unsigned char x_axis_high;

    i2c_master_init(LIS2DW12_I2C_ADD_H, (unsigned char)(CLOCK_SYS_CLOCK_HZ / (4 * I2C_CLK_SPEED)));

	x_axis_low = i2c_read_byte(LIS2DW12_OUT_X_L, 1);
	x_axis_high = i2c_read_byte(LIS2DW12_OUT_X_H, 1);

	return ((x_axis_high<<8)|x_axis_low);
}

short get_lis2dw12_y_value(void)
{
	unsigned char y_axis_low;
	unsigned char y_axis_high;

    i2c_master_init(LIS2DW12_I2C_ADD_H, (unsigned char)(CLOCK_SYS_CLOCK_HZ / (4 * I2C_CLK_SPEED)));

	y_axis_low = i2c_read_byte(LIS2DW12_OUT_Y_L, 1);
	y_axis_high = i2c_read_byte(LIS2DW12_OUT_Y_H, 1);

	return ((y_axis_high<<8)|y_axis_low);
}

short get_lis2dw12_z_value(void)
{
	unsigned char z_axis_low;
	unsigned char z_axis_high;

    i2c_master_init(LIS2DW12_I2C_ADD_H, (unsigned char)(CLOCK_SYS_CLOCK_HZ / (4 * I2C_CLK_SPEED)));

	z_axis_low = i2c_read_byte(LIS2DW12_OUT_Z_L, 1);
	z_axis_high = i2c_read_byte(LIS2DW12_OUT_Z_H, 1);

	return ((z_axis_high<<8)|z_axis_low);
}

void auto_power_task(void)
{
	short x_axis_data = 0;
	unsigned char i = 0;
	unsigned char open_count = 0, close_count = 0;

	if (*(curtain_robot.dp_auto_power.dp_data) == TRUE) {
		if (clock_time_exceed(curtain_robot.get_lis2dw12_data_time, 10 * TIME_MS)) {
			curtain_robot.get_lis2dw12_data_time = clock_time();

			if (curtain_robot.motor_run_mode != RUN_MODE_IDLE) {
				return;
			}

			x_axis_data = get_lis2dw12_x_value();

			if (x_axis_data > VIBRATION_HIGH || x_axis_data < VIBRATION_LOW) {
				x_data_buf[x_data_index] = x_axis_data;
				x_data_index++;

				clean_x_buf_count = 0;
			} else {
				clean_x_buf_count++;
			}

			if ((clean_x_buf_count > 20) && (x_data_index >= 20)) { //At this point it has levelled off
				for (i=0; i < 7; i++) {
					if (x_data_buf[i] >= 0) {
						open_count++;
					} else {
						close_count++;
					}
				}

				if (*curtain_robot.dp_percent_state.dp_data == CURTAIN_ALL_OFF) {
					curtain_robot.auto_power_state = AUTO_POWER_CLOSE;
				} else if (*curtain_robot.dp_percent_state.dp_data == CURTAIN_ALL_ON) {
					curtain_robot.auto_power_state = AUTO_POWER_OPEN;
				} else if (open_count < close_count) {
					curtain_robot.auto_power_state = AUTO_POWER_CLOSE;
				} else {
					curtain_robot.auto_power_state = AUTO_POWER_OPEN;
				}

				//clean flag
				clean_x_buf_count= 0;
				x_data_index=0;
			}

			if (curtain_robot.auto_power_state != AUTO_POWER_IDLE && clean_x_buf_count >= 100) {
				if (curtain_robot.auto_power_state == AUTO_POWER_OPEN) {
					curtain_percent_control(*curtain_robot.dp_percent_state.dp_data, 0);
				} else if (curtain_robot.auto_power_state == AUTO_POWER_CLOSE) {
					curtain_percent_control(*curtain_robot.dp_percent_state.dp_data, 100);
				}
				curtain_robot.auto_power_state=AUTO_POWER_IDLE;
				clean_x_buf_count = 0;
			}

			if (clean_x_buf_count > 500) {
				clean_x_buf_count = 0;
				x_data_index=0;
			}
		}
	}
}

void motion_detection_task(void)
{
	short x_axis_data = 0;
	if (curtain_robot.motor_run_mode == RUN_MODE_CONTROL || \
		curtain_robot.motor_run_mode ==  RUN_MODE_PERCENT_CONTROL) {
		if (clock_time_exceed(motion_get_data_time, 10 * TIME_MS)) {
			motion_get_data_time = clock_time();
			x_axis_data = get_lis2dw12_x_value();

			if (x_axis_data > VIBRATION_HIGH || x_axis_data < VIBRATION_LOW) {
				motion_count++;
				clean_motion_count = 0;
			} else {
				clean_motion_count++;
			}

			if (clean_motion_count >= 20) {
				motion_count = 0;
				curtain_robot.robot_move_state = ROBOT_STATIC;
			}

			if (motion_count >= 10) { //motion
				curtain_robot.robot_move_state = ROBOT_MOTION;
			}
		}

		if (clock_time_exceed(motion_get_state_time, 300 * TIME_MS)) {
			motion_get_state_time = clock_time();
			if (curtain_robot.robot_move_state != ROBOT_MOTION) {
				motor_sleep();
				if (*curtain_robot.dp_control.dp_data == CTRL_OPEN_E) {
					if (*(curtain_robot.dp_control_back.dp_data) == CTRL_BACK_BACK_E) {
						motor_forward();
					} else {
						motor_reverse();
					}
				} else {
					if (*(curtain_robot.dp_control_back.dp_data) == CTRL_BACK_BACK_E) {
						motor_reverse();
					} else {
						motor_forward();
					}
				}
				curtain_robot.run_start_time = clock_time();
			}
		}
	}
}
