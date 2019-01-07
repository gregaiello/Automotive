/**
  ******************************************************************************
  * @file    biss.h
  * @author  Lucas M. Angarola
  * @version V1.0.0
  * @date    19-Apr-2018
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MHL200_DRIVER_H
#define MHL200_DRIVER_H

#include "stm32f405xx.h"
#include "biss.h"
#include "hardwareSetup.h"

/* Exported constants --------------------------------------------------------*/
/* MHL200 encoder IC Register addresses */
#define HALL_SIGNAL_REG0			0x00
#define HALL_SIGNAL_REG1			0x01
#define HALL_SIGNAL_REG2			0x02
#define HALL_SIGNAL_REG3			0x03
#define HALL_SIGNAL_REG4			0x04
#define RS422_DRIVER_REG			0x05
#define SINE_DIGITAL_CONV_REG0		0x06
#define SINE_DIGITAL_CONV_REG1		0x07
#define SINE_DIGITAL_CONV_REG2		0x08
#define SINE_DIGITAL_CONV_REG3		0x09
#define SINE_DIGITAL_CONV_REG4		0x0A
#define TEST_SETTINGS_REG0			0x0E
#define TEST_SETTINGS_REG1			0x0F
#define PROFILE_ID_REG0				0x42
#define PROFILE_ID_REG1				0x43
#define GAIN_STATUS					0x76
#define MAIN_STATUS					0x77
#define DEVICE_ID_REG0				0x78
#define DEVICE_ID_REG1				0x79

/* Function status definition */
#define MHL200_ERROR				1		/* General error */
#define MHL200_OK					0		/* OK status */

#define BISS_MA_GPIO_MODER_MASK			0x30000
#define BISS_MA_GPIO_MODER 				0x10000 // PB8 as output
#define BISS_SLO_GPIO_MODER_MASK		0xC0000
#define BISS_SLO_GPIO_MODER 			0x00000 // PB9 as input

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t 	mhl200_init(void);

#endif /* __MHL200_DRIVER_H */

/*****************************END OF FILE****/
