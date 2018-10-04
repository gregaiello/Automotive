/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @author  Aiello Gregorio
  * @version V0.0.1
  * @date    March 2018 
  * @brief   Interrupt service routine declarations
  *  
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f405xx.h"
#include "struct_BLDC.h"

// Macro used in park and clark transforms
#define TWOTHIRD      0.66666
#define ONETHIRD      0.33333
#define TWOSQRTTHREE  1.154700538
#define INVSQRT3      0.577350269
#define SQRT3         1.73205
#define SQRT3DIV2     0.866025404

// Value of the shunt resistor to calculate current
#define R_SHUNT 1

volatile uint16_t duty_cycle_BLDC;
volatile uint8_t flag_wall;

// ISR 
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void ADC_IRQHandler(void);
void OTG_FS_IRQHandler(void);
void SysTick_Handler(void);

void update_PWM(void);
void closed_loop(void);

#endif /* __STM32F4xx_IT_H */
