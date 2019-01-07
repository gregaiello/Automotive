/**
  ******************************************************************************
  * @file    calibration_hall_sensors.h
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   Calibration of the rotor angle from Hall sensors data and saving of 
  				the calibration values in the internal FLASH memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Gregorio Aiello</center></h2>
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to deal
  * in the Software without restriction, including without limitation the rights
  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  * copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all
  * copies or substantial portions of the Software.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALIBRATION_HALL_SENSORS_H
#define __CALIBRATION_HALL_SENSORS_H

#include "stm32f405xx.h"
#include "peripherals_STM32F405.h"
#include "struct_BLDC.h"

/* Exported constants --------------------------------------------------------*/
#define FLAG_FLASH 				 (uint16_t) 0xABBA
#define CAL_OK  				 0
#define CAL_ERR					 1
#define CALIBRATION_ENCODER		 227.75
#define PI 						 (double)3.141592

/* Global variables --------------------------------------------------------*/
volatile uint8_t flag_hall_CAL;
volatile uint8_t flag_hall_DMA;
uint16_t max_Hall[3];
uint16_t min_Hall[3];
uint16_t hall_tmp[3];
uint16_t hall_tmp_prev[3];
double angle_rotor;

/* External variables --------------------------------------------------------*/
extern volatile uint32_t hall[3];
extern volatile uint16_t duty_cycle_BLDC;

/* Exported functions --------------------------------------------------------*/
void syncronization_enc_hall();
void calibration(uint16_t* max, uint16_t* min);
uint8_t cal_update_flash(uint16_t* max, uint16_t* min);
uint8_t cal_write_flash(uint16_t* max, uint16_t* min);
double cal_get_angle_rotor(uint16_t* max, uint16_t* min);
void cal_angle_from_amplitude(double *angle_guess, double *val_norm);
void cal_set_encoder(double angle_rotor);

#endif /* __CALIBRATION_HALL_SENSORS_H */

/*****************************END OF FILE****/