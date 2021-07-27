#ifndef __TUYA_APP_OPT3004_H__
#define __TUYA_APP_OPT3004_H__

#define OPT3004_I2C_ADDR_W                      0x88
#define OPT3004_I2C_ADDR_R                      0x89
#define DEVICE_ID                               0x3001
#define MANUFACTURER_ID                         0x5449
#define OPT3004_RESULT_REGISTER_ADDR            0x00
#define OPT3004_CONFIG_REGISTER_ADDR            0x01
#define OPT3004_LOW_LIMIT_REGISTER_ADDR         0x02
#define OPT3004_HIGH_LIMIT_REGISTER_ADDR        0x03
#define OPT3004_MANUFACTURER_ID_REGISTER_ADDR   0x7E
#define OPT3004_DEVICE_ID_REGISTER_ADDR         0x7F

unsigned char opt3004_init(void);
short get_opt3004_value(void);

void illumination_task(void);
void update_current_illumination(void);
void update_current_illumination_task(void);

#endif /* __TUYA_APP_OPT3004_H */
