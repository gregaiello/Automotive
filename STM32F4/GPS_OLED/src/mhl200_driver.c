/**
  ******************************************************************************
  * @file    biss.c
  * @author  Lucas M. Angarola
  * @version V1.0.0
  * @date    19-Apr-2018
  * @brief   
  ******************************************************************************
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "mhl200_driver.h"

/* Local functions ------------------------------------------------------- */
void 		mhl200_gpioInit(void);
uint8_t 	mhl200_setup(uint8_t device);
uint8_t 	mhl200_readID(uint8_t device);

/**
  * @brief  
  * @param  None
  * @retval None
   */
uint8_t mhl200_init(void){
    /* GPIO initialization */
    mhl200_gpioInit();

    /* Are the encoders there? */
    if( mhl200_readID(0x00)){
        return MHL200_ERROR;
    }

    /* Device 0 setup */
    if( mhl200_setup(0x00) ){
        return MHL200_ERROR;
    }

    return MHL200_OK;
}

/**
  * @brief  
  * @param  None
  * @retval None
   */
void mhl200_gpioInit(void){
    /**
    BISS GPIO Configuration:   
    PB8     ------> MA
    PB9     ------> SLO **/

  // Enable GPIO clock
  RCC->AHB1ENR |= (uint32_t)RCC_AHB1ENR_GPIOBEN; // clock to AHB1-GPIOB enabled

  /* Set pin MA (PB8) as output */
  uint32_t tmp;
  tmp = GPIOB->MODER;
  tmp &= (~BISS_MA_GPIO_MODER_MASK); 
  tmp |= BISS_MA_GPIO_MODER; // set the correct pattern in bit
  GPIOB->MODER = (uint32_t)tmp; // write the MODER register

  // Set MA to produce its initial state //
  MA_set; 

  /* Set pin SLO (PB9) as input */
  tmp = GPIOB->MODER;
  tmp &= (~BISS_SLO_GPIO_MODER_MASK); 
  tmp |= BISS_SLO_GPIO_MODER; // set the correct pattern in bit
  GPIOB->MODER = (uint32_t)tmp; // write the MODER register
}


/**
  * @brief  
  * @param  None
  * @retval None
   */
uint8_t mhl200_setup(uint8_t device){
    /* The amplifier gain was set here */
    biss_writeRegister(HALL_SIGNAL_REG0,device,0x2F);
    if( ((uint8_t)biss_readRegister(HALL_SIGNAL_REG0,device)) != 0x2F )
        return MHL200_ERROR;

    /* The interpolator was set as 1024 (maximum resolution) */
    biss_writeRegister(SINE_DIGITAL_CONV_REG0,device,0x80);
    if( ((uint8_t)biss_readRegister(SINE_DIGITAL_CONV_REG0,device)) != 0x80 )
        return MHL200_ERROR;

    /* CFGHYS = 0x3 and CFGMTD = 1 */
    biss_writeRegister(SINE_DIGITAL_CONV_REG2,device,0xD0);
    if( ((uint8_t)biss_readRegister(SINE_DIGITAL_CONV_REG2,device)) != 0xD0 )
        return MHL200_ERROR;

    /* CFGMTD2 = 1 */
    biss_writeRegister(SINE_DIGITAL_CONV_REG4,device,0x01);
    if( ((uint8_t)biss_readRegister(SINE_DIGITAL_CONV_REG4,device)) != 0x01 )
        return MHL200_ERROR;

    return MHL200_OK;
}


/**
  * @brief  
  * @param  None
  * @retval None
   */
uint8_t mhl200_readID(uint8_t device){
    if( ((uint8_t)biss_readRegister(DEVICE_ID_REG0,device)) != 0x4D )
		  return MHL200_ERROR;

    if( ((uint8_t)biss_readRegister(DEVICE_ID_REG1,device)) != 0x48 )
		  return MHL200_ERROR;

    return MHL200_OK;
}
/*****************************END OF FILE****/
