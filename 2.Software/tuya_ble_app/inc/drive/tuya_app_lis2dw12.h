#ifndef __TUYA_APP_LIS2DW12_H__
#define __TUYA_APP_LIS2DW12_H__

/** I2C Device Address 8 bit format  if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS2DW12_I2C_ADD_L   0x31U
#define LIS2DW12_I2C_ADD_H   0x33U

/** Device Identification (Who am I) **/
#define LIS2DW12_ID			0x44U

#define LIS2DW12_WHO_AM_I	0x0FU

#define LIS2DW12_CTRL1		0x20U
#define LIS2DW12_CTRL2		0x21U
#define LIS2DW12_CTRL4		0x23U
#define LIS2DW12_CTRL6		0x25U
#define LIS2DW12_CTRL7		0x3FU

#define LIS2DW12_OUT_X_L	0x28U
#define LIS2DW12_OUT_X_H	0x29U
#define LIS2DW12_OUT_Y_L	0x2AU
#define LIS2DW12_OUT_Y_H	0x2BU
#define LIS2DW12_OUT_Z_L	0x2CU
#define LIS2DW12_OUT_Z_H	0x2DU

#define ALL_INT_SRC			0x3BU


unsigned char lis2dw12_init(void);
short get_lis2dw12_x_value(void);
short get_lis2dw12_y_value(void);
short get_lis2dw12_z_value(void);

void auto_power_task(void);
void motion_detection_task(void);

#endif /* __TUYA_APP_LIS2DW12_H */
