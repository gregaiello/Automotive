/**
  ******************************************************************************
  * @file    ssd1306_oled.c
  * @author  Gregorio Aiello
  * @version V1.0.0
  * @date    July 2018
  * @brief   SSD1306 libs for stm32f405 based on the (amazing) code by Tilen Majerle
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SSD1306_OLED_H
#define __SSD1306_OLED_H

#include "stm32f4xx_hal.h"
#include "fonts.h"

/* Exported constants --------------------------------------------------------*/
#define SSD1306_COLOR_BLACK                0x00
#define SSD1306_COLOR_WHITE                0x01
#define SSD1306_WIDTH                      128
#define SSD1306_HEIGHT                     64
#define SSD1306_I2C_ADDR                   0x3C
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define I2C_DATA_LEN                       2
#define I2C_ADDRESS	                       (uint16_t)0X78
#define I2C_TIMEOUT						   10
#define SSD1306_WRITECOMMAND(command)      support_routine_I2C_Write_command(0x00, command)
#define ABS(x)   ((x) > 0 ? (x) : -(x))

/* Global variables --------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Exported functions --------------------------------------------------------*/
void I2C1_Init(void);
void support_routine_I2C_Write_command(uint8_t reg, uint8_t command);
void i2c_master_write_slave(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Len, uint32_t Timeout);
void SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateOLED(void);
void write_multiple_SSD1306(uint8_t* data, uint16_t count);
void SSD1306_ToggleInvert(void);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color);
char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c);
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c);
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c);
void SSD1306_GotoXY(uint16_t x, uint16_t y);
void SSD1306_ON(void);
void SSD1306_OFF(void);

/* SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

#endif /* __SSD1306_OLED_H */

/*****************************END OF FILE****/
