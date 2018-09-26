/**
 * @author  Gregorio Aiello
 * @email   gregorio.aiello91@gmail.com
 * @version v1.0
 * @license GNU GPL v3
 * @brief   Library for BNO055 I2C on LPC
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>

// BNO055 Defines
//#define I2C_DEVICE_ADD (uint16_t)0x50
#define I2C_DEVICE_ADD (uint16_t)0x28
#define SYS_TRIGGER_ADDRESS 0x3F
#define SYS_TRIGGER_RESET_VALUE 0x20
#define SYS_TRIGGER_EXT_XTAL_VALUE 0x80
#define OPR_MODE_ADDRESS 0x3D
#define OPR_MODE 0xC
#define CALIBRATION_REG 0x35
#define ST_ADDRESS 0x36
#define ST_VALUE 0xF
#define UNIT_SEL_ADDRESS 0x3B
#define UNIT_SEL_VALUE 0x4 // BUT I AM NOT SETTING IT BC DEGREES IS FINE (0x4 IS RADIANS)
#define EUL_DATA_X_LSB_ADDRESS 0x1A // REMEMBER 1 DEGREE = 16 LSB
#define START_DATA_ADDRESS 0x08
#define QUAT_DATA_X_LSB_ADDRESS 0x20
#define GYRO_DATA_X_LSB_ADDRESS 0x14
#define ACC_DATA_X_LSB_ADDRESS 0x08
#define MAG_DATA_X_LSB_ADDRESS 0x0E
#define PAGE_ID_ADDR 0x07

// Exported functions
int init_BNO055();
int get_Euler(uint8_t *Euler);
int get_Quaternion(uint8_t *Quaternion);

#endif
