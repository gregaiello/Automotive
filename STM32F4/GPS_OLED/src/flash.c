/**
  ******************************************************************************
  * @file    flash.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   Flash writing/erasing
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

/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

uint8_t flash_erase(uint32_t sector_n, uint8_t number_of_sectors){
  static uint32_t error = 0;

  FLASH_EraseInitTypeDef conf;
  conf.Sector       = sector_n;
  conf.NbSectors    = number_of_sectors;
  conf.TypeErase    = FLASH_TYPEERASE_SECTORS;
  conf.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  if( HAL_FLASHEx_Erase(&conf,&error) != HAL_OK ){
    return FLASH_ERR;
  }

  if( error != 0xFFFFFFFF ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint8_t flash_mass_erase(){
  static uint32_t error = 0;

  FLASH_EraseInitTypeDef conf;
  conf.TypeErase    = FLASH_TYPEERASE_MASSERASE;
  conf.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();

  if( HAL_FLASHEx_Erase(&conf,&error) != HAL_OK ){
    return FLASH_ERR;
  }

  if( error != 0xFFFFFFFF ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint8_t flash_program_byte(uint32_t address, uint8_t data){

  HAL_FLASH_Unlock();

  if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, data) != HAL_OK ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint8_t flash_read_byte(uint32_t address){
  return *((uint8_t *)address);
}

uint8_t flash_program_halfword(uint32_t address, uint16_t data){

  HAL_FLASH_Unlock();

  if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data) != HAL_OK ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint16_t flash_read_halfword(uint32_t address){
  return *((uint16_t *)address);
}

uint8_t flash_program_word(uint32_t address, uint32_t data){

  HAL_FLASH_Unlock();

  if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data) != HAL_OK ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint32_t flash_read_word(uint32_t address){
  return *((uint32_t *)address);
}

uint8_t flash_program_doubleword(uint32_t address, uint64_t data){

  HAL_FLASH_Unlock();

  if( HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) != HAL_OK ){
    return FLASH_ERR;
  }

  HAL_FLASH_Lock();

  return FLASH_OK;
}

uint64_t flash_read_doubleword(uint32_t address){
  return *((uint64_t *)address);
}
