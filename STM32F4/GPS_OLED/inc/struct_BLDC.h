#ifndef __STRUCT_BLDC_H
#define __STRUCT_BLDC_H

#include "stm32f405xx.h"

//#define NUM_SAMPLES_LUT ((uint16_t)0x20D0) // 8400 counts 
//#define NUM_SAMPLES_LUT ((uint16_t)0x1FF) // 511
#define NUM_SAMPLES_LUT ((uint16_t)0x1FF8) // 8184 counts as the encoder
#define NUM_SAMPLES_SINE NUM_SAMPLES_LUT
#define SHIFT (NUM_SAMPLES_SINE/3)

#define NUM_SAMPLES_ADC_CURRENT_DMA 2
#define NUM_SAMPLES_ADC_HALL_DMA 3

#define BUTTON_MASK        (uint16_t) 0x0004

#define LPF_FAST(value, sample, filter_constant) ( value -= ( filter_constant ) * ( value - ( sample ) ) )

// Global variables for currents
volatile uint32_t current[NUM_SAMPLES_ADC_CURRENT_DMA];
volatile uint32_t hall[NUM_SAMPLES_ADC_HALL_DMA];

uint32_t Min_enc;
uint8_t flag_calib;

typedef struct{
  int16_t trig_sine_table_CCR1[NUM_SAMPLES_SINE];
  int16_t trig_sine_table_CCR2[NUM_SAMPLES_SINE];
  int16_t trig_sine_table_CCR3[NUM_SAMPLES_SINE];
} sine_waves;

typedef struct{
  uint16_t header;
  uint32_t position;
  uint32_t speed;
  uint8_t  status;
  uint8_t checksum;
}__attribute__((__packed__))  comm_typedef_struct_tx_packet;

typedef struct{
  uint16_t header;
  uint32_t torque;
  uint8_t  control;
  uint8_t checksum;
}__attribute__((__packed__))  comm_typedef_struct_rx_packet;

volatile uint8_t flag_comm_tx;
volatile uint8_t flag_comm_rx;

// Torque of the motor
volatile uint32_t gain_torque;

// Angle of the rotor
volatile float angle_rotor_enc;

#endif
