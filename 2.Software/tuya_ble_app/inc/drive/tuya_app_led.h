#ifndef __TUYA_APP_LED_H__
#define __TUYA_APP_LED_H__

#define LED_PORT	GPIO_PC4

#define LED_ON_LEVEL	1
#define LED_OFF_LEVEL	0

void led_init(void);
void led_on(void);
void led_off(void);

#endif /* __TUYA_APP_LED_H__ */
