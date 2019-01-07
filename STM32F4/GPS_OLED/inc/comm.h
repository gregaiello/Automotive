/**
  ******************************************************************************
  * @file    comm.h
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   VCP communication over USB
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
#ifndef __COMM_H
#define __COMM_H

#include "stm32f405xx.h"
#include "stm32f4xx_hal.h"
#include "struct_BLDC.h"

/* Exported constants --------------------------------------------------------*/
#define COMM_ERR        0
#define COMM_OK         1
#define COMM_PARTIAL    2
#define LEN_PACKET_TX   12
#define LEN_PACKET_RX   8

/* Global variables --------------------------------------------------------*/
volatile comm_typedef_struct_tx_packet buf_tx_struct;
volatile comm_typedef_struct_rx_packet buf_rx_struct;
volatile uint8_t global_len;

/* Exported functions --------------------------------------------------------*/
void initComm(void);

uint8_t comm_send(uint8_t* buf, uint16_t len);

uint8_t comm_packet_send(comm_typedef_struct_tx_packet* data);

uint8_t comm_check_size(uint16_t cnt);

uint8_t comm_check_header(uint8_t* buf, uint8_t start);

uint8_t comm_check_checksum(uint8_t* buf, uint8_t start);

uint8_t comm_get_sum(uint8_t* buf, uint8_t start);

#endif /* __COMM_H */

/*****************************END OF FILE****/
