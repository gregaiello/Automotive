/**
  ******************************************************************************
  * @file    comm.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   VCP communication over USB
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

#include "comm.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

/**
  * @brief  Init USB VCP communication
  * This function initializes the hardware for USB communication
  * @param  none
  * @retval none
   */
void initComm(void){
  MX_USB_DEVICE_Init(); // Device ready: in order to send/receive data use if(CDC_Transmit_FS(&buf, len) == USBD_OK){} and if(CDC_Receive_FS(&buf, len) == USBD_OK){}
}

/**
  * @brief  Send a command over USB VCP
  * This function sends streams of bytes over the USB
  * @param  uint8_t* buf: buffer of unsigned int 8 to be sent
  *         uint16_t len: number of bytes to send
  * @retval COMM_OK when the data have been sent
   */
uint8_t comm_send(uint8_t* buf, uint16_t len){
  while(CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY){}
  return COMM_OK;
}

/**
  * @brief  Send the VESC data struct over USB VCP
  * This function sends the VESC data struct defined in struct_BLDC.h over the USB
  * @param  comm_typedef_struct_tx_packet* data:
  * @retval COMM_OK when the data have been sent
   */
uint8_t comm_packet_send(comm_typedef_struct_tx_packet* data){
  return comm_send((uint8_t*)data, LEN_PACKET_TX);
}

/**
  * @brief  Check the size of the received USB packet
  * This function checks wheter the size of the incoming USB packet is correct
  * @param  uint16_t cnt:
  * @retval COMM_OK when the the size is correct, COMM_ERR otherwise
   */
uint8_t comm_check_size(uint16_t cnt){
  if(cnt >= LEN_PACKET_RX){
    return COMM_OK; 
  }
  return COMM_ERR; 
}

/**
  * @brief  Check the header of the received USB packet
  * This function checks wheter the header of the incoming USB packet is correct
  * @param  uint8_t* buf: RX buffer
  *         uint8_t start: starting point of the message in the buffer
  * @retval COMM_OK when the the header is correct, COMM_ERR otherwise
   */
uint8_t comm_check_header(uint8_t* buf, uint8_t start){
  if((buf[start] == 0xCA) && (buf[start+1] == 0xFE)){
    return COMM_OK; 
  }
  return COMM_ERR;
}

/**
  * @brief  Check the checksum of the received USB packet
  * This function checks wheter the checksum of the incoming USB packet is correct
  * @param  uint8_t* buf: RX buffer
  *         uint8_t start: starting point of the message in the buffer
  * @retval COMM_OK when the the checksum is correct, COMM_ERR otherwise
   */
uint8_t comm_check_checksum(uint8_t* buf, uint8_t start){
  if(comm_get_sum(buf, start) == ((uint8_t)buf[start + (LEN_PACKET_RX-1)])){
    return COMM_OK;
  }
  return COMM_ERR; 
}

/**
  * @brief  Calculate the checksum of the received USB packet
  * This function calculates the checksum of the incoming USB packet
  * @param  uint8_t* buf: RX buffer
  *         uint8_t start: starting point of the message in the buffer
  * @retval sum: the sum of the bytes in the packet
   */
uint8_t comm_get_sum(uint8_t* buf, uint8_t start){
  uint8_t i = 0, sum = 0;
  for(i = 0; i < (LEN_PACKET_RX-1); i++){
    sum += buf[start + i];
  }
  return sum;
}
