/**
  ******************************************************************************
  * @file    ssd1306_HAL.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   Abstraction layer for communication with OLED SSD1306
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

#include "ssd1306_HAL.h"
#include "ssd1306_oled.h"
#include <stdio.h>
#include <string.h>

/**
  * @brief  Init  communication with SSD1306 and setup the UX
  * This function initializes the hardware for I2C and the SSD1306 OLED display
  * @param  none
  * @retval none
   */
void initSSD1306(){
  I2C1_Init();
  SSD1306_Init();
  //set_backbone_GUI_VESC();
  set_backbone_GUI_generic();
}

/**
  * @brief  Display generic parameters on OLED
  * This function shows numerical values on the OLED display (useful for debug)
  * @param  uint16_t data: value to be shown
            uint8_t pos: position of the display (0 to 8):
            
            |0|1|2|
            |3|4|5| OLED display (128-64) position reference
            |6|7|8|

  * @retval none
   */
void display_param_generic(uint16_t data, uint8_t pos){
  char x[4];
  snprintf(x,4,"%2d", data);
  if(pos < 3){
    SSD1306_GotoXY(20+38*pos,7);
    SSD1306_Puts(x, &Font_7x10, SSD1306_COLOR_WHITE);
  }
  if(pos > 2 && pos < 6){
    SSD1306_GotoXY(20+38*(pos-3),27);
    SSD1306_Puts(x, &Font_7x10, SSD1306_COLOR_WHITE);
  }
  if(pos > 5){
    SSD1306_GotoXY(20+38*(pos-6),47);
    SSD1306_Puts(x, &Font_7x10, SSD1306_COLOR_WHITE);
  }   
  SSD1306_UpdateOLED();
}

void display_string_generic(char* data, uint8_t pos){
  if(pos < 3){
    SSD1306_GotoXY(20+38*pos,7);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
  }
  if(pos > 2 && pos < 6){
    SSD1306_GotoXY(20+38*(pos-3),27);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
  }
  if(pos > 5){
    SSD1306_GotoXY(20+38*(pos-6),47);
    SSD1306_Puts(data, &Font_7x10, SSD1306_COLOR_WHITE);
  }   
  SSD1306_UpdateOLED();
}

/**
  * @brief  Display VESC data struct on OLED
  * This function shows numerical values from the VESC data struct on the OLED display (useful for debug)
  * @param  comm_typedef_struct_rx_packet* data: 
        The parameters shown are:
            
            Header:
            Torque: 
            Control:

  * @retval none
   */
void display_param_VESC(comm_typedef_struct_rx_packet* data){
  static char header[8];
  snprintf(header,8,"%7x", (int16_t)data->header);

  static char torque[8];
  snprintf(torque,8,"%7d", (int16_t)data->torque);

  static char control[8];
  snprintf(control,8,"%7d", (int16_t)data->control);

  SSD1306_GotoXY(60,7);
  SSD1306_Puts(header, &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(60,27);
  SSD1306_Puts(torque, &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(60,47);
  SSD1306_Puts(control, &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_UpdateOLED();
}

/**
  * @brief  Display error on VESC data struct on OLED
  * This function shows error messages on the OLED display (useful for debug)
  * @param  none 
        The messages shown are:
            
            Header:  error!
            Torque:  error!
            Control: error!

  * @retval none
   */
void display_error_VESC(){
  SSD1306_GotoXY(60,7);
  SSD1306_Puts(" Error!", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(60,27);
  SSD1306_Puts(" Error!", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(60,47);
  SSD1306_Puts(" Error!", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_UpdateOLED();
}

/**
  * @brief  Set up backbone of the interface for generic debug on OLED
  * This function builds up the backbone of the user interface on the OLED display
  * @param  none 
        The messages shown are:
            
          |0|1|2|
          |3|4|5| OLED display (128-64) position reference
          |6|7|8|

  * @retval none
   */
void set_backbone_GUI_generic(void){
  SSD1306_DrawLine(0, 0, 0, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(0, 0, SSD1306_WIDTH, 0, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(0, SSD1306_HEIGHT, SSD1306_WIDTH, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(SSD1306_WIDTH, 0, SSD1306_WIDTH, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(6,7);
  SSD1306_Puts("1:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(43,7);
  SSD1306_Puts("2:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(83,7);
  SSD1306_Puts("3:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(6,27);
  SSD1306_Puts("4:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(43,27);
  SSD1306_Puts("5:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(83,27);
  SSD1306_Puts("6:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(6,47);
  SSD1306_Puts("7:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(43,47);
  SSD1306_Puts("8:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(83,47);
  SSD1306_Puts("9:", &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_UpdateOLED();
}

/**
  * @brief  Set up backbone of the interface for VESC on OLED
  * This function builds up the backbone of the user interface on the OLED display
  * @param  none 
        The messages shown are:
            
            Header:  
            Torque:  
            Control:

  * @retval none
   */
void set_backbone_GUI_VESC(void){
  SSD1306_DrawLine(0, 0, 0, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(0, 0, SSD1306_WIDTH, 0, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(0, SSD1306_HEIGHT, SSD1306_WIDTH, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);
  SSD1306_DrawLine(SSD1306_WIDTH, 0, SSD1306_WIDTH, SSD1306_HEIGHT, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(6,7);
  SSD1306_Puts("Header:", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(6,27);
  SSD1306_Puts("Torque:", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_GotoXY(6,47);
  SSD1306_Puts("Control:", &Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_UpdateOLED();
}