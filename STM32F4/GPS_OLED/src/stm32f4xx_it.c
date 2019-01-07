/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Aiello Gregorio
  * @version V0.0.1
  * @date March 2018
  * @brief   Interrupt service routine definitions
  *  
  ******************************************************************************
  */ 
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "struct_BLDC.h"
#include "math.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern volatile uint8_t flag_hall_DMA;
extern volatile uint8_t flag_comm_tx;
extern volatile uint32_t current[NUM_SAMPLES_ADC_CURRENT_DMA];
extern sine_waves CCR_registers;

void DMA2_Stream0_IRQHandler(void){
  /* DMA ISR for current sensors */
  if((DMA2->LISR & DMA_LISR_TCIF0) && (DMA2_Stream0->CR & DMA_SxCR_TCIE)){ // trasmission complete interrupt
    update_PWM();
    DMA2->LIFCR = DMA_LIFCR_CTCIF0;  // clear interrupt
  }
  if((DMA2->LISR & DMA_LISR_TEIF0) && (DMA2_Stream0->CR & DMA_SxCR_TEIE)){ // error interrupt
    // TODO: check and resolve errors
    DMA2->LIFCR = DMA_LIFCR_CTEIF0;  // clear interrupt
  }
}

void DMA2_Stream2_IRQHandler(void){

  flag_hall_DMA = 1; // Flag used by the calibration of Hall sensors
  
  /* DMA ISR for current sensors */
  if((DMA2->LISR & DMA_LISR_TCIF2) && (DMA2_Stream2->CR & DMA_SxCR_TCIE)){ // trasmission complete interrupt
    DMA2->LIFCR = DMA_LIFCR_CTCIF2;  // clear interrupt
  }
  if((DMA2->LISR & DMA_LISR_TEIF2) && (DMA2_Stream2->CR & DMA_SxCR_TEIE)){ // error interrupt
    // TODO: check and resolve errors
    DMA2->LIFCR = DMA_LIFCR_CTEIF2;  // clear interrupt
  }
}

void TIM1_CC_IRQHandler(void){
  /* I am not checking if the isr is the one for the compare bc I activated just one ISR (and bc it takes like 50us)
     Since the PWMs are center aligned I can sample the three currents when the 3 low side FETs are on */

  ADC1->CR2 |= ADC_CR2_SWSTART; // start sampling ADC and automatically store the data in current[0:1] using circular DMA

  TIM1->SR &= (~TIM_SR_CC4IF);  // clear interrupt
}

void OTG_FS_IRQHandler(void){
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void SysTick_Handler(void){
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void HAL_SYSTICK_Callback(void){
  static uint16_t cnt_systick = 0;
  cnt_systick++;
  if(cnt_systick >= 250){
    flag_comm_tx = 1;
    cnt_systick = 0;
  }
}

/* User defined functions */

void update_PWM(){
  GPIOC->BSRR |= LED_GREEN_ON;
  closed_loop();
  GPIOC->BSRR |= LEDS_OFF;
  DAC->DHR12R1 = (current[0]);
  //static volatile uint32_t cnt = 0;
  //TIM1->CCR1 = ((uint16_t)((CCR_registers.trig_sine_table_CCR1[cnt%NUM_SAMPLES_SINE])))>>2;
  //TIM1->CCR2 = ((uint16_t)((CCR_registers.trig_sine_table_CCR2[cnt%NUM_SAMPLES_SINE])))>>2;
  //TIM1->CCR3 = ((uint16_t)((CCR_registers.trig_sine_table_CCR3[cnt%NUM_SAMPLES_SINE])))>>2;
  //cnt++;
}

void closed_loop(){

  /* Conversion to current values */
  static volatile float ia = 0.0, ib = 0.0, ic = 0.0;
  ia = (2035.0 - (float)current[0]) * 0.00403; // Ampere THIS CURRENT IS CENTERED IN THE MIDDLE OF THE FSR, ((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) = 0.00403
  ib = (2035.0 - (float)current[1]) * 0.00403; // Ampere THIS CURRENT IS CENTERED IN THE MIDDLE OF THE FSR, ((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) = 0.00403
  ic = - (ia + ib); // Ampere

  // I need to add here a LPF on the current ABSOLUTELY

  /*static volatile float ia = 0.0, ib = 0.0;
  ia = (2035.0 - (float)current[0]) * 0.00403; // the amplifier in the DRV8302 is an inverting stage centered in 1.64V
  ib = (2035.0 - (float)current[1]) * 0.00403; // the amplifier in the DRV8302 is an inverting stage centered in 1.64V*/

  /* Clarke Transform */
  static volatile float i_salpha = 0.0, i_sbeta = 0.0;
  i_salpha = 1.5 * ia;
  i_sbeta  = 0.5 * SQRT3 * (ib - ic);

  /*static volatile float i_salpha = 0.0, i_sbeta = 0.0;
  i_salpha = ia; 
  i_sbeta  = (INVSQRT3 * i_salpha) + (TWOSQRTTHREE * ib);*/

  /* Park Transform */
  static volatile float i_sd = 0.0, i_sq = 0.0;
  static volatile float theta = 0.0;
  theta = ( ( (float) ( (int64_t)(TIM5->CNT) - 0x80000000)) * 0.000309 ) * 2.0; // 2 is the number of pole pairs
  i_sd  = i_salpha * cosf(theta) + i_sbeta * sinf(theta);
  i_sq  = -i_salpha * sinf(theta) + i_sbeta * cosf(theta);


  /* PI controllers */
  static volatile float error_q = 0.0, error_d = 0.0, integral_q = 0.0, integral_d = 0.0;
  static const float set_flux = 0.0, set_torque = 1.0; // set_torque needs to be updated
  static volatile float v_q = 0.0, v_d = 0.0;
  static const float Kp_q = 0.377, Ki_q = 4166.666, Kp_d = 0.377, Ki_d = 4166.666; // Ki = R/L = 0.25Ohm / 0.00006H = 4166.666 ~~ Kp = L * BW  = 0.00006H * (2* pi * 20kHz / 20) = 0.377
  
  error_q = set_torque - i_sq;
  error_d = set_flux - i_sd; // set_flux = 0 we want torques not longitudinal forces

  integral_q += error_q * Kp_q * Ki_q;
  integral_d += error_d * Kp_d * Ki_d;

  if(integral_q > 3000.0) integral_q = 3000.0;
  else if(integral_q < 0.0) integral_q = 0.0;

  if(integral_d > 3000.0) integral_d = 3000.0;
  else if(integral_d < 0.0) integral_d = 0.0;

  v_q = Kp_q * error_q + integral_q;
  v_d = Kp_d * error_d + integral_d;

  /* Inverse Park Transform */
  static volatile float v_salpha_PI = 0.0, v_sbeta_PI = 0.0;
  v_salpha_PI = - v_q * sinf(theta) + v_d * cosf(theta);
  v_sbeta_PI  = v_q * cosf(theta) + v_d * sinf(theta);

  /* Inverse Clarke Transform */
  static volatile float v_sa_PI = 0.0, v_sb_PI = 0.0, v_sc_PI = 0.0;  
  v_sa_PI =   0.66666 * v_salpha_PI;
  v_sb_PI =   INVSQRT3 * v_sbeta_PI - 0.33333 * v_salpha_PI;
  v_sc_PI = - INVSQRT3 * v_sbeta_PI - 0.33333 * v_salpha_PI;

  /*static volatile int16_t v_sa_PI = 0, v_sb_PI = 0, v_sc_PI = 0;  
  v_sa_PI = v_salpha_PI;
  v_sb_PI = - (v_sa_PI >> 1) + SQRT3DIV2 * v_sbeta_PI;
  v_sc_PI = - (v_sa_PI >> 1) - SQRT3DIV2 * v_sbeta_PI;*/

  TIM1->CCR1 = (v_sa_PI);
  TIM1->CCR2 = (v_sb_PI);
  TIM1->CCR3 = (v_sc_PI);
}
