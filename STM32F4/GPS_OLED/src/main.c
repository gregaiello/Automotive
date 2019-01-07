/**
  ******************************************************************************
  * @file    main.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   main file for Simedis BLDC control
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 Gregorio Aiello </center></h2>
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

// When I want to load a new FW with DFU USB I just have to call DFUBootMode(); and load the new FW from the host machine

#include "main.h"
#include "usb_device.h"
#include <stdio.h>
#include <math.h>

extern volatile comm_typedef_struct_tx_packet buf_tx_struct;
extern volatile comm_typedef_struct_rx_packet buf_rx_struct;
extern volatile uint32_t gain_torque;
extern volatile float angle_rotor_enc;

int main(void) {
    //HAL_Init();
    SystemClock_Config();

    gain_torque = 5;

    /* Initialization of the peripherals: NVIC, LED, ADC, DMA, DAC, UART, TIMERS */
    init();    

    // Wait the user to type the configuration
    while(flag_comm_rx == 0);
    GPIOC->BSRR |= (uint32_t)LED_BLUE_ON;

    /* start ENCODER readings, this has to be called before the start of the PWM, an homing procedure is neede but so far is not implemented */
    startENCODER();    
    
    /* I fill the sine_waves structure that will hold the look up table for the sinusoidal control, 
        I can decide the maximum amplitude, the minimum, the number of samples. In previous version 
        of the control I used 3/4 of sine wave thus I can also select the number of samples before 
        the cut, now I am using full sine control thus the cut happens at the end of the period */    
    get_trig_LUT(&CCR_registers, 2000, 2000, NUM_SAMPLES_SINE, NUM_SAMPLES_SINE, SHIFT);

    /* Sample ADC for Hall sensors and set the cnt register for the timer that handles the encoder */
    syncronization_enc_hall();
    HAL_Delay(1000);

    /* Set PC10, enable the DRV8302 and activate the FETs */
    en_DRV8302();

    /* start PWM, if you want to change DC write in TIM1->CCR1, TIM1->CCR2, TIM1->CCR3, the N channels are set automatically.
       start also ADC, it uses timer 1 and set the pwm in center aligned mode */
    startPWM_ADC_Shunt();  

    initSSD1306();

    HAL_Delay(1000);


    buf_tx_struct.header = 0xFFFF;
    buf_tx_struct.position = 0xF0F0F0F0;
    buf_tx_struct.speed = 0xF0F0F0F0;
    buf_tx_struct.status = 0xFF;
    buf_tx_struct.checksum = (uint8_t)(buf_tx_struct.header + buf_tx_struct.position + buf_tx_struct.speed + buf_tx_struct.status);

    char angle_buf[30];
    do {
      angle_rotor_enc = ( (float) ( (int64_t)(TIM5->CNT) - 0x80000000) / 56.5);
      sprintf(angle_buf,"%.5f", angle_rotor_enc);  
      display_string_generic(angle_buf, 0);  
      display_param_generic(buf_rx_struct.control, 3);
    } while (1);
}

void init() {
  initComm();
  initENCODER();
  initNVIC();
  initLED(); 
  initDRV8302();
  initADC_Hall();
  initADC_Shunt();
  initDMA();
  initTIM1();
  initDAC();
  initButton();
  initRelay();
  
  /* start setup magnetic encoder, if the init function returns MHL200_OK the encoder is correctly initialized and it is ready to operate */ 
  if(mhl200_init() == MHL200_OK){
    GPIOC->BSRR |= (uint32_t)LED_RED_ON;
  }
}

void get_trig_LUT(sine_waves *sine_vectors, int16_t min_v, int16_t max_v, int16_t samples_period, int16_t samples_cut, int16_t shift){
  int16_t x, i;
  for(i = 0; i<samples_cut; i++){
    x = (max_v)*sin(((2*PI)/((float)samples_period))*((float)i));
    x += min_v;
    //if(x<100) {x = 100;}
    sine_vectors->trig_sine_table_CCR1[i] = x;
  }
  for(i=0; i<samples_cut; i++){
    sine_vectors->trig_sine_table_CCR2[i] = ((int16_t) sine_vectors->trig_sine_table_CCR1[((i + shift)%(samples_cut))]);
    sine_vectors->trig_sine_table_CCR3[i] = ((int16_t) sine_vectors->trig_sine_table_CCR1[((i + 2*shift)%(samples_cut))]);    
  }
}

void _Error_Handler(char *file, int line){
  while(1){
  }
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}