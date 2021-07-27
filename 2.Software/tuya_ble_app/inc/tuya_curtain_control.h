#ifndef __TUYA_CURTAIN_CONTROL_H__
#define __TUYA_CURTAIN_CONTROL_H__

typedef unsigned char CALC_TIME_STATE;
#define CALC_TIME_IDLE  0
#define CALC_TIME_READY 1
#define CALC_TIME_CLOSE 2
#define CALC_TIME_OPEN  3
#define CALC_TIME_END   4

void curtain_percent_control(unsigned char current_position, unsigned char target_position);
void curtain_percent_control_stop_task(void);


void calculate_total_time_init(void);
void calculate_total_time_task(void);
void calculate_total_time_stop(void);

void curtain_control_open(void);
void curtain_control_pause(void);
void curtain_control_close(void);
void curtain_control_stop_task(void);

#endif /* __TUYA_CURTAIN_CONTROL_H__ */
