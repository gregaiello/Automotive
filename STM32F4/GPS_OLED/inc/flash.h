/**
  ******************************************************************************
  * @file    flash.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f405xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_ERR 	 1
#define FLASH_OK	   0

/* Sector addresses for STM32F40x family */
#define FLASH_ADDRESS_SECTOR_0  (uint32_t)0x08000000 // 16  Kbytes
#define FLASH_ADDRESS_SECTOR_1  (uint32_t)0x08004000 // 16  Kbytes
#define FLASH_ADDRESS_SECTOR_2  (uint32_t)0x08008000 // 16  Kbytes
#define FLASH_ADDRESS_SECTOR_3  (uint32_t)0x0800C000 // 16  Kbytes 
#define FLASH_ADDRESS_SECTOR_4  (uint32_t)0x08010000 // 64  Kbytes
#define FLASH_ADDRESS_SECTOR_5  (uint32_t)0x08020000 // 128 Kbytes
#define FLASH_ADDRESS_SECTOR_6  (uint32_t)0x08040000 // 128 Kbytes 
#define FLASH_ADDRESS_SECTOR_7  (uint32_t)0x08060000 // 128 Kbytes 
#define FLASH_ADDRESS_SECTOR_8  (uint32_t)0x08080000 // 128 Kbytes 
#define FLASH_ADDRESS_SECTOR_9  (uint32_t)0x080A0000 // 128 Kbytes 
#define FLASH_ADDRESS_SECTOR_10 (uint32_t)0x080C0000 // 128 Kbytes 
#define FLASH_ADDRESS_SECTOR_11 (uint32_t)0x080E0000 // 128 Kbytes 

/* Sector number for STM32F40x family */
#define FLASH_NUMBER_SECTOR_0  (uint32_t)0x0U 
#define FLASH_NUMBER_SECTOR_1  (uint32_t)0x1U
#define FLASH_NUMBER_SECTOR_2  (uint32_t)0x2U
#define FLASH_NUMBER_SECTOR_3  (uint32_t)0x3U
#define FLASH_NUMBER_SECTOR_4  (uint32_t)0x4U
#define FLASH_NUMBER_SECTOR_5  (uint32_t)0x5U
#define FLASH_NUMBER_SECTOR_6  (uint32_t)0x6U
#define FLASH_NUMBER_SECTOR_7  (uint32_t)0x7U
#define FLASH_NUMBER_SECTOR_8  (uint32_t)0x8U
#define FLASH_NUMBER_SECTOR_9  (uint32_t)0x9U
#define FLASH_NUMBER_SECTOR_10 (uint32_t)0x10U
#define FLASH_NUMBER_SECTOR_11 (uint32_t)0x11U

/* Exported functions ------------------------------------------------------- */
uint8_t flash_erase(uint32_t sector_n, uint8_t number_of_sectors); // the functions expects the SECTOR NUMBER (not an address)
uint8_t flash_mass_erase(void);

uint8_t flash_program_byte(uint32_t address, uint8_t data); // the functions expects the SECTOR ADDRESS (not the number)
uint8_t flash_read_byte(uint32_t address); // the functions expects the SECTOR ADDRESS (not the number)

uint8_t flash_program_halfword(uint32_t address, uint16_t data); // the functions expects the SECTOR ADDRESS (not the number)
uint16_t flash_read_halfword(uint32_t address); // the functions expects the SECTOR ADDRESS (not the number)

uint8_t flash_program_word(uint32_t address, uint32_t data); // the functions expects the SECTOR ADDRESS (not the number)
uint32_t flash_read_word(uint32_t address); // the functions expects the SECTOR ADDRESS (not the number)

uint8_t flash_program_doubleword(uint32_t address, uint64_t data); // the functions expects the SECTOR ADDRESS (not the number)
uint64_t flash_read_doubleword(uint32_t address); // the functions expects the SECTOR ADDRESS (not the number)

#endif /* __FLASH_H */

/*****************************END OF FILE****/
