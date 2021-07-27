#ifndef __TUYA_APP_MOTOR_H__
#define __TUYA_APP_MOTOR_H__

#define REV_PORT	GPIO_PD2
#define FWD_PORT	GPIO_PB4

typedef unsigned char MOTOR_RUN_STATE;
#define MOTOR_STOP      0
#define MOTOR_SLEEP     1
#define MOTOR_FORWAED   2
#define MOTOR_REVERSE   3

void motor_init(void);
void motor_forward(void);
void motor_reverse(void);
void motor_stop(void);
void motor_sleep(void);

void curtain_open(void);
void curtain_close(void);
void curtain_pause(void);

MOTOR_RUN_STATE get_motor_state(void);

#endif /* __TUYA_APP_MOTOR */
