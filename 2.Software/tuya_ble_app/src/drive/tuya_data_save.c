#include "flash.h"
#include "tuya_utils.h"
#include "tuya_ble_log.h"

#include "tuya_data_save.h"
#include "tuya_dp_process.h"

#define DEBUG	0

#define FLASH_ADDR              0x040000

/* data_head(0xff)+dp_control_back+dp_percent_state+dp_auto_power+dp_time_total*2+dp_humidity_set+dp_humidity_switch+check sum */
#define SAVE_DATA_LEN	9

volatile unsigned char curtain_robot_data[SAVE_DATA_LEN] = {0};

void save_device_data(void)
{
#if DEBUG
	int i = 0;
#endif
	curtain_robot_data[0] = 0xFF;
	curtain_robot_data[1] = *curtain_robot.dp_control_back.dp_data;
	curtain_robot_data[2] = *curtain_robot.dp_percent_state.dp_data;
	curtain_robot_data[3] = *curtain_robot.dp_auto_power.dp_data;
	curtain_robot_data[4] = curtain_robot.dp_time_total.dp_data[0];
	curtain_robot_data[5] = curtain_robot.dp_time_total.dp_data[1];
	curtain_robot_data[6] = *curtain_robot.dp_illumination_set.dp_data;
	curtain_robot_data[7] = *curtain_robot.dp_illumination_switch.dp_data;
	curtain_robot_data[8] = check_sum(curtain_robot_data, SAVE_DATA_LEN - 1);

	flash_erase_sector(FLASH_ADDR);
	flash_write_page(FLASH_ADDR, SAVE_DATA_LEN, (unsigned char *)curtain_robot_data);
#if DEBUG
	for (i=0; i<SAVE_DATA_LEN; i++) {
		TUYA_APP_LOG_DEBUG("curtain_robot_data[%d]: %d", i, curtain_robot_data[i]);
	}
#endif
}

unsigned char read_device_data(void)
{
#if DEBUG
	int i = 0;
#endif
	flash_read_page(FLASH_ADDR, SAVE_DATA_LEN, (unsigned char *)curtain_robot_data);
#if DEBUG
	for (i=0; i<SAVE_DATA_LEN; i++) {
		TUYA_APP_LOG_DEBUG("curtain_robot_data[%d]: %d", i, curtain_robot_data[i]);
	}
#endif
	if ((curtain_robot_data[0] != 0xFF) || \
		(curtain_robot_data[SAVE_DATA_LEN-1] != check_sum(curtain_robot_data, SAVE_DATA_LEN - 1))) {
		return FALSE;
	}

	*curtain_robot.dp_control_back.dp_data = curtain_robot_data[1];
	*curtain_robot.dp_percent_state.dp_data = curtain_robot_data[2];
	*curtain_robot.dp_auto_power.dp_data = curtain_robot_data[3];
	curtain_robot.dp_time_total.dp_data[0] = curtain_robot_data[4];
	curtain_robot.dp_time_total.dp_data[1] = curtain_robot_data[5];
	*curtain_robot.dp_illumination_set.dp_data = curtain_robot_data[6];
	*curtain_robot.dp_illumination_switch.dp_data = curtain_robot_data[7];

	return TRUE;
}

void erase_device_data(void)
{
	curtain_robot_data[0] = 0xFF;
	curtain_robot_data[1] = 0x00;
	curtain_robot_data[2] = 0x00;
	curtain_robot_data[3] = 0x00;
	curtain_robot_data[4] = 0x00;
	curtain_robot_data[5] = 0x00;
	curtain_robot_data[6] = 0x00;
	curtain_robot_data[7] = 0x00;
	curtain_robot_data[8] = check_sum(curtain_robot_data, SAVE_DATA_LEN - 1);

	flash_erase_sector(FLASH_ADDR);
	flash_write_page(FLASH_ADDR, SAVE_DATA_LEN, (unsigned char *)curtain_robot_data);

	read_device_data();
}
