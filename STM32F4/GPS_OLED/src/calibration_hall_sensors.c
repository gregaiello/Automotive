/**
  ******************************************************************************
  * @file    calibration_hall_sensors.c
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

#include "calibration_hall_sensors.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "flash.h"
#include <string.h>
#include <math.h>

static char buf[100];
static char buf2[100];

/**
  * @brief  Main function to start the syncronization of encoder and Hall sensors
  * This function initializes the variables and perform the syncronization of the encoder
    with the Hall sensors to extract the absolute position of the shaft (rotor)
  * @param  none
  * @retval none
   */
void syncronization_enc_hall(){
	flag_hall_CAL = 0;
  flag_hall_DMA = 0;

  if(cal_update_flash(max_Hall, min_Hall) == CAL_ERR){
    CDC_Transmit_FS((uint8_t*)" FLASH NEW \n", 12);
    //GPIOC->BSRR |= (uint32_t)LED_RED_ON; 
    calibration(max_Hall, min_Hall);
  }
  else{
    CDC_Transmit_FS((uint8_t*)" FLASH OLD \n", 12);
    //GPIOC->BSRR |= (uint32_t)LED_GREEN_ON;
  }
  
  HAL_Delay(1000);

  angle_rotor = cal_get_angle_rotor(max_Hall, min_Hall);

  cal_set_encoder(angle_rotor);
}

/**
  * @brief  Calibration usign the Hall sensors
  * This function performs the calibration of the Hall sensors if the
    calibration parameters are not stored in the internal FLASH memory.
    It gets the max and min of the three hal sensors in order to extract 
    a meaningful angle out of them. it performs also a 1 pole LPF in
    order to reduce the HF noise.  Once the user button is pressed the 
    values are saved into the FLASH memory. 
    Until you erase the FLASH those values are gonna stay there.
  * @param  none
  * @retval none
   */
void calibration(uint16_t* max, uint16_t* min){
	uint8_t i = 0; 
  uint8_t flag_filter = 0;

	for(i = 0; i < 3; i++){
		max[i] = 0x0000;
		min[i] = 0xFFFF;
	} 

  while(flag_hall_CAL == 0){
    startADC_Hall();
    HAL_Delay(10);
    while(flag_hall_DMA == 0){ 
    }
    flag_hall_DMA = 0;

		// The values of the Hall sensors are stored in hall[0] - hall[1] - hall[2]
    if(flag_filter == 0){
      flag_filter = 1;
      hall_tmp_prev[0] = (uint16_t)hall[0];
      hall_tmp_prev[1] = (uint16_t)hall[1];
      hall_tmp_prev[2] = (uint16_t)hall[2];
    }

 		for(i = 0; i < 3; i++){ 

      //LPF: y[i] = α * x[i] + (1-α) * y[i-1]

      hall_tmp[i] = (uint16_t)((250 * (uint16_t)hall[i] + 5 * (uint16_t)hall_tmp_prev[i]) >> 8);
      hall_tmp_prev[i] = (int16_t)hall_tmp[i];

 			max[i] = ((uint16_t)hall_tmp[i] > max[i])?(uint16_t)hall_tmp[i]:max[i];
 			min[i] = ((uint16_t)hall_tmp[i] < min[i])?(uint16_t)hall_tmp[i]:min[i];
 		}
    
    sprintf(buf,"Hall: %d - %d - %d Max: %d - %d - %d Min: %d - %d - %d \n\r", (uint16_t)hall[0], (uint16_t)hall[1], (uint16_t)hall[2], max[0], max[1], max[2], min[0], min[1], min[2]);    
    if(CDC_Transmit_FS((uint8_t*)buf, sizeof(buf)) != USBD_OK){}

 		if((GPIOD->IDR & BUTTON_MASK) == BUTTON_MASK){
      //GPIOC->BSRR |= (uint32_t)LED_BLUE_ON;     
      flag_hall_CAL = 1; // this is the end of the calibration
      cal_write_flash(max, min);
      sprintf(buf,"Amplitude calibration --> Max: %d - %d - %d Min: %d - %d - %d \n\r", max[0], max[1], max[2], min[0], min[1], min[2]);    
      if(CDC_Transmit_FS((uint8_t*)buf, sizeof(buf)) != USBD_OK){}
    }
  }
}

/**
  * @brief  Routine to write variables into the internal FLASH memory
  * This function writes the 3 max and the 3 min from the Hall sensors into the internal FLASH memory.
  * @param  uint16_t* max: vector max[3] with the max values of the 3 Hall sensors
            uint16_t* min: vector min[3] with the min values of the 3 Hall sensors
  * @retval CAL_ERR in case of error, CAL_OK in case of correct writing
   */
uint8_t cal_write_flash(uint16_t* max, uint16_t* min){
  uint64_t value = (uint64_t)FLAG_FLASH;
  uint64_t value2 = 0;
  value  |= (((uint64_t)max[0]) << 16) | (((uint64_t)max[1]) << 32) | (((uint64_t)max[2]) << 48);
  value2 |= ((uint64_t)min[0]) | (((uint64_t)min[1]) << 16) | (((uint64_t)min[2]) << 32);

  if(flash_erase(FLASH_NUMBER_SECTOR_6, 1) == FLASH_ERR){
    CDC_Transmit_FS((uint8_t*)"FLASH ERROR \n", 13);
    return CAL_ERR;
  }

  HAL_Delay(250);

  if(flash_program_word( FLASH_ADDRESS_SECTOR_6, (uint32_t)value) == FLASH_ERR){
    CDC_Transmit_FS((uint8_t*)"FLASH ERROR \n", 13);
    return CAL_ERR;  
  }

  HAL_Delay(250);

  if(flash_program_word( FLASH_ADDRESS_SECTOR_6+4, (uint32_t)(value>>32)) == FLASH_ERR){
    CDC_Transmit_FS((uint8_t*)"FLASH ERROR \n", 13);
    return CAL_ERR;  
  }
  
  HAL_Delay(250);

  if(flash_program_word( FLASH_ADDRESS_SECTOR_6+8, (uint32_t)(value2)) == FLASH_ERR){
    CDC_Transmit_FS((uint8_t*)"FLASH ERROR \n", 13);
    return CAL_ERR;  
  }
  
  HAL_Delay(250);

  if(flash_program_word( FLASH_ADDRESS_SECTOR_6+12, (uint32_t)(value2>>32)) == FLASH_ERR){
    CDC_Transmit_FS((uint8_t*)"FLASH ERROR \n", 13);
    return CAL_ERR;  
  }
  
  HAL_Delay(250);
  CDC_Transmit_FS((uint8_t*)"FLASH OK \n", 10);
  return CAL_OK;  
}


/**
  * @brief  Check the calibration parameters
  * This function checks whether the calibration parameters are already stored
    in the internal FLASH. If the calibration parameters are stored it picks them 
    up and use them for the exraction of the angle of the shaft, if the FLASH section
    reserved for calibration parameters is empty it performs the calibration using 
    the Hall sensors,
  * @param  uint16_t* max: vector max[3] with the max values of the 3 Hall sensors
            uint16_t* min: vector min[3] with the min values of the 3 Hall sensors
  * @retval CAL_ERR in case of no calibration found, CAL_OK in case of correct reading of calibration parameters from FLASH
   */
uint8_t cal_update_flash(uint16_t* max, uint16_t* min){

  int64_t value_read_1 = flash_read_doubleword(FLASH_ADDRESS_SECTOR_6);
  HAL_Delay(100);
  int64_t value_read_2 = flash_read_doubleword(FLASH_ADDRESS_SECTOR_6+8);
  HAL_Delay(100);

  if(((uint16_t)value_read_1) != FLAG_FLASH){

    return CAL_ERR;
  }
  else{
    max[0] = (uint16_t)(value_read_1 >> 16); 
    max[1] = (uint16_t)(value_read_1 >> 32); 
    max[2] = (uint16_t)(value_read_1 >> 48); 
    min[0] = (uint16_t)(value_read_2); 
    min[1] = (uint16_t)(value_read_2 >> 16); 
    min[2] = (uint16_t)(value_read_2 >> 32);

    snprintf(buf,sizeof(buf),"Amplitude reading --> Max: %d - %d - %d Min: %d - %d - %d \n\r", max[0], max[1], max[2], min[0], min[1], min[2]);    
    if(CDC_Transmit_FS((uint8_t*)buf, sizeof(buf)) != USBD_OK){}

    return CAL_OK;
  }
}

/**
  * @brief  Extract the angle of the rotor
  * This function extracts the angle of the rotor from the Hall sensor signal and
    the calibration parameters saved in FLASH.
  * @param  uint16_t* max: vector max[3] with the max values of the 3 Hall sensors
            uint16_t* min: vector min[3] with the min values of the 3 Hall sensors
  * @retval ((angle_guess*180.0)/PI) angle in degrees
   */
double cal_get_angle_rotor(uint16_t* max, uint16_t* min){
  double angle_guess, angle_guess_deg;
  double val_norm[3];
  uint8_t i = 0; 
  while((GPIOD->IDR & BUTTON_MASK) != BUTTON_MASK){
    CDC_Transmit_FS((uint8_t*)"Start Sampling \n", 16);
    startADC_Hall();
    HAL_Delay(10);
    while(flag_hall_DMA == 0){ 
      CDC_Transmit_FS((uint8_t*)"wait ADC \n", 10);
      HAL_Delay(10);
    }
    flag_hall_DMA = 0;

    for(i = 0; i < 3; i++){
      val_norm[i] = (((hall[i] - ((max[i] + min[i])/2.0)) / (max[i] - min[i])) * 2.0);
      if(val_norm[i] >=  1.0){
        val_norm[i] =  1.0;
      }
      if(val_norm[i] <= -1.0){
        val_norm[i] = -1.0;
      }
    }

    cal_angle_from_amplitude(&angle_guess, val_norm);
    HAL_Delay(10);
    angle_guess_deg = 0.5*(angle_guess*180.0)/PI;
    sprintf(buf2," Angle guess:  %.5f ~ hall: %d ~ %d ~ %d ~ norm: %f ~ %f ~ %f \n", angle_guess_deg, (uint16_t)hall[0], (uint16_t)hall[1], (uint16_t)hall[2], val_norm[0], val_norm[1], val_norm[2]);    
    HAL_Delay(10);
    if(CDC_Transmit_FS((uint8_t*)buf2, sizeof(buf2)) != USBD_OK){}
    HAL_Delay(10);
  }
  stopADC_Hall();
  return ((0.5*angle_guess*180.0)/PI); // from 0 to 2pi
}

/**
  * @brief  Calculate angle of the shaft
  * This function extracts the angle of the rotor from the 3 sine waves from the Hall sensors.
  * @param  double *angle_guess: guess of the angle, the angles is gonna be stored here
            double *val_norm: vector val_norm[3] of the three signals from the Hall sensors
  * @retval none
   */
void cal_angle_from_amplitude(double *angle_guess, double *val_norm){
  if(val_norm[2] > val_norm[1]){
    *angle_guess = asin(val_norm[0]);
  }
  else{
    if(val_norm[0] > 0.0){
      *angle_guess = (PI - asin(val_norm[0]));
    }
    else{
      *angle_guess = (-PI - asin(val_norm[0]));
    }
  }
  HAL_Delay(100);
}

/**
  * @brief  Set the encoder counter based on the angle of the shaft
  * This function set the correct value in the CNT register of the encoder from the angle previously calculated
  * @param  double angle_rotor: angle of the rotor
  * @retval none
   */
void cal_set_encoder(double angle_rotor){
  duty_cycle_BLDC = (uint16_t)(((180.0 - angle_rotor)*8181.28)/180.0);

  TIM5->CNT = 0x80000000;
  TIM5->CNT += ((int64_t) ((double)angle_rotor*56.5)); // set the encoder
}