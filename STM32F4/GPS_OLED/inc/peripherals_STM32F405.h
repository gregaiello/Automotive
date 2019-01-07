/**
  ******************************************************************************
  * @file    peripherals_STM32F405.h
  * @author  Aiello Gregorio
  * @version V0.0.1
  * @date March 2018
  * @brief   BLDC motor control macro definitions
  *  
  ******************************************************************************
  */ 

#ifndef __PERIPHERALS_STM32F405_H
#define __PERIPHERALS_STM32F405_H

#include "stm32f405xx.h"
#include "struct_BLDC.h"

#define NVIC_PRIORITYGROUP_4         0x00000003U

#define FREQ_PWM (NUM_SAMPLES_LUT>>1) // defined in struct.h: 0x1068 --> autoreload register = 4200, clock to the peripheral = 168MHz, the timer counts 4200 up + 4200 down = 8400 counts --> 168Mhz/8400 = 20kHz

// LED macro:
#define LEDS_REG ((uint32_t)0x540)
#define LEDS_REG_MASK ((uint32_t)0xFC0)
#define LEDS_ON ((uint32_t)0x38)
#define LEDS_OFF ((uint32_t)0x380000)
#define LED_BLUE_ON ((uint32_t)0x00000020)
#define LED_RED_ON ((uint32_t)0x00000010)
#define LED_GREEN_ON ((uint32_t)0x0000008)
#define LED_GREEN_OFF ((uint32_t)0x00080000)

// BUTTON macro:
#define BUTTON_REG_MASK ((uint32_t)0x30)

// ADC1 shunt macro
#define ADC1_REG_MASK ((uint32_t)0xF)
#define ADC1_NOPULL_MASK ((uint32_t)0xF)
#define ADC1_SMPR2 ((uint32_t)0x1) // sampling time FOR CHANNEL 2,3,4: 15 CYCLES
#define ADC1_SQR1 ((uint32_t)0x100000) //2 CONVERSIONS
#define ADC1_SQR3 ((uint32_t)0x128) //2 CONVERSIONS, CHANNEL 8(0b01000),9(0b01001) -> 01001 01000 //CHECK TOROUGHLY THIS VALUE, DO NOT CONFUSE THE GPIO AND THE CHANNEL

// ADC2 hall macro
#define ADC2_REG_MASK ((uint32_t)0x3F)
#define ADC2_NOPULL_MASK ((uint32_t)0x3F)
#define ADC2_SMPR1 ((uint32_t)0x1FF) // sampling time FOR CHANNEL 10,11,12: 480 CYCLES
#define ADC2_SQR1 ((uint32_t)0x200000) //3 CONVERSIONS
#define ADC2_SQR3 ((uint32_t)0x316A) //3 CONVERSIONS, CHANNEL 10-11-12

// TIM1 macro
#define TIM1_CCMR1_PWM1_OC1 ((uint16_t) 0x60) // selection PWM mode 1 channel 1
#define TIM1_CCMR1_PWM1_OC2 ((uint16_t) 0x6000) // selection PWM mode 1 channel 2
#define TIM1_CCMR2_PWM1_OC3 ((uint16_t) 0x60) // selection PWM mode 1 channel 3
#define TIM1_CCER_PWM1_COMPLEMENTARY_ACTIVE_HIGH ((uint16_t) 0x555) // COMPLEMENTARY ACTIE HIGH
#define TIM1_CCER_PWM1_COMPLEMENTARY_ACTIVE_LOW ((uint16_t) 0xDDD) // COMPLEMENTARY ACTIVE LOW
#define TIM1_CCER_PWM1_NORMAL ((uint16_t) 0x111) // NO COMPLEMENTARY
#define TIM1_CCER_DIS_CH1N ((uint16_t) 0x551) // enable CH1,CH2,CH2N,CH3,CH3N
#define TIM1_CCER_DIS_CH2N ((uint16_t) 0x515) // enable CH1,CH1N,CH2,CH3,CH3N
#define TIM1_CCER_DIS_CH3N ((uint16_t) 0x155) // enable CH1,CH1N,CH2,CH2N,CH3
#define TIM1_CCER_DIS_CH1N_CH2N ((uint16_t) 0x511) // enable CH1,CH2,CH3,CH3N
#define TIM1_CCER_DIS_CH2N_CH3N ((uint16_t) 0x115) // enable CH1,CH1N,CH2,CH3
#define TIM1_CCER_DIS_CH3N_CH1N ((uint16_t) 0x151) // enable CH1,CH2,CH2N,CH3
#define TIM1_ARR FREQ_PWM //autoreload register 4200x2 (20kHz)
#define TIM1_CCR_CH1 ((uint16_t)0x0) //0% duty cycle
#define TIM1_CCR_CH2 ((uint16_t)0x0) //0% duty cycle
#define TIM1_CCR_CH3 ((uint16_t)0x0) //0% duty cycle
//#define TIM1_DT ((uint16_t)0x1F4) // 3us dead time
//#define TIM1_DT ((uint16_t)0x64) // 600ns dead time
//#define TIM1_DT ((uint16_t)0xA) // 60ns dead time
#define TIM1_DT ((uint16_t)0x0) // 0ns dead time
#define PWM_B_REG_MASK ((uint32_t) 0xFC000000)
#define PWM_A_REG_MASK ((uint32_t) 0x3F0000)
#define PWM_B_MODER ((uint32_t) 0xA8000000)
#define PWM_A_MODER ((uint32_t) 0x2A0000)
#define GPIOB_AF1_PWM ((uint32_t) 0x11100000)
#define GPIOA_AF1_PWM ((uint32_t) 0x00000111)
#define GPIOB_AF1_PWM_SEL_MASK ((uint32_t) 0xFFF00000)
#define GPIOA_AF1_PWM_SEL_MASK ((uint32_t) 0x00000FFF)

// DRV8302 macro
#define DRV8302_EN_REG_MASK ((uint32_t) 0x300000)
#define DRV8302_EN ((uint32_t) 0x100000)
#define DRV8302_ON ((uint16_t) 0x400)
#define DRV8302_OFF ((uint16_t) 0x4000000)
#define DRV8302_EN_PULL_DOWN_REG_MASK ((uint32_t) 0x300000)
#define DRV8302_EN_PULL_DOWN ((uint32_t) 0x200000)

// Relay macro
#define RELAY_EN_REG_MASK ((uint32_t) 0xC000000)
#define RELAY_EN ((uint32_t) 0x4000000)
#define RELAY_ON ((uint32_t) 0x2000)
#define RELAY_OFF ((uint32_t) 0x20000000)

// ENCODER macro
#define ENCODER_A_REG_MASK ((uint32_t)0xF)
#define ENCODER_A_MODER ((uint32_t)0xA) // PA0-PA1
#define ENCODER_A_PUPDR ((uint32_t)0xA)
#define GPIOA_AF2_ENC_SEL_MASK ((uint32_t)0xFF) 
#define GPIOA_AF2_ENCODER ((uint32_t)0x22)

#define TIM5_SMCR_MASK ((uint32_t)0x7)
#define TIM5_SMCR ((uint32_t)0x3)
#define TIM5_CCMR1_MASK ((uint32_t)0x303)
#define TIM5_CCMR1 ((uint32_t)0x101)
#define TIM5_CCER_MASK ((uint32_t)0x11)
#define TIM5_CCER ((uint32_t)0x11)

// DAC macro
#define DAC_REG_MASK ((uint32_t)0xC00)

// Functions declarations
void initNVIC();
void initLED();
void initButton();
void initRelay();
void initADC_Shunt();
void initADC_Hall();
void initDMA();
void initTIM1();
void initDAC();
void startADC_Hall();
void stopADC_Hall();
void startPWM_ADC_Shunt();
void initENCODER();
void startENCODER();
void initDRV8302();
void en_DRV8302();


#endif
