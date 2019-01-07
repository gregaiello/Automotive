/**
  ******************************************************************************
  * @file    jump_to_bootloader.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   Jump to bootloader for DFU USB
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

#include "jump_to_bootloader.h"

/**
  * @brief  Jumpo to bootloader
  * This function prepares and performs the jumpo to the bootloader memory address
  * @param  none
  * @retval none
   */
void JumpToBootloader(void){
	/**
	 * Step 0: Declare the routine. 
	 */
	void (*SysMemBootJump)(void);
	
	/**
	 * Step 1: Set system memory address. 
	 *       
	 *       For STM32F405, system memory is on 0x1FFF0000
	 *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
	 */
	volatile uint32_t addr = ADDRESS_BOOTLOADER;
	
	/**
	 * Step 2: Set jump memory location for system memory
	 *       Use address with 4 bytes offset (room for SP) which specifies jump location where program starts
	 */
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
	
	/**
	 * Step 3: Set main stack pointer.
	 *       This step must be done last otherwise local variables in this function
	 *       don't have proper value since stack pointer is located on different position
	 *
	 *       Set direct address location which specifies stack pointer in SRAM location
	 */
	__set_MSP(*(uint32_t *)addr);
	
	/**
	 * Step 4: Actually call our function to jump to set location
	 *       This will start system memory execution
	 */
	SysMemBootJump();
}

/**
  * @brief  Set up the jumpo to DFU USB
  * This function saves a pattern in RAM and calls the reset
  * @param  none
  * @retval none
   */
void DFUBootMode(void){
	/* Store flag in SRAM1 to start bootloader after reboot */
	*((uint32_t *)ADDRESS_FLAG_BOOTLOADER) = FLAG_BOOTLOADER;
	
	/* Reser MCU to get all system variables perfectly cleanup */
	NVIC_SystemReset();
}


// In SystemInit() add the following to check the way you enter the bootloader
//	
//	if(*((uint32_t *)0x2001BFF0) == 0xABCDEF00){
//		
//		/* Reset flag */
//		*((uint32_t *)0x2001BFF0) = 0;
//		
//		/* Run bootloader */
//		JumpToBootloader();
//	}
//