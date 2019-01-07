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

#include "ssd1306_oled.h"

/* SSD1306 data buffer */
uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
SSD1306_t SSD1306;

/**
  * @brief Init I2C communication
  * This function initializes the hardware for I2C communication
  * @param  none
  * @retval none
   */
void I2C1_Init(void)
{
  	hi2c1.Instance = I2C1;
  	hi2c1.Init.ClockSpeed = 400000;
  	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  	hi2c1.Init.OwnAddress1 = 0;
  	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  	hi2c1.Init.OwnAddress2 = 0;
  	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  	if(HAL_I2C_Init(&hi2c1) != HAL_OK);
}

/**
  * @brief Write on the I2C bus
  * This function writes a sequence of uint8_t data on the I2C bus
  * @param  I2C_HandleTypeDef *hi2c: data structure for I2C communication (part of the HAL libraries)
  			uint16_t DevAddress: I2C address of the slave
  			uint8_t *pData: pointer to the buffer of data to send
  			uint16_t Len: number of bytes to send
  			uint32_t Timeout: timeout before returning an error
  * @retval none
   */
void i2c_master_write_slave(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Len, uint32_t Timeout){
	HAL_I2C_Master_Transmit(hi2c, DevAddress, pData, Len, Timeout);
}

/**
  * @brief Support routine used to communicate with the SSD1306 OLED display
  * This function writes a command to a register
  * @param  uint8_t reg: address of the register (usually I2C registers are 1byte-long)
  			uint8_t command: command to be saved into the register 'reg'
  * @retval none
   */
void support_routine_I2C_Write_command(uint8_t reg, uint8_t command){
  	static uint8_t config[2];
  	config[0] = reg;
  	config[1] = command;
  	i2c_master_write_slave(&hi2c1, I2C_ADDRESS, config, I2C_DATA_LEN, I2C_TIMEOUT);
}

/**
  * @brief Initialize the SSD1306 OLED display
  * This function initializes the SSD1306 OLED display
  * @param  none
  * @retval none
   */
void SSD1306_Init(){
	// Datasheet with the registers available here: https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
	SSD1306_WRITECOMMAND(0xAE); //display off (sleep mode)
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0xFF);
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64) 
	SSD1306_WRITECOMMAND(0x3F); //--set multiplex ratio(1 to 64) 
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); //-not offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //
	SSD1306_WRITECOMMAND(0xAF); //--turn on SSD1306 panel

  	SSD1306_Fill(SSD1306_COLOR_BLACK);
  	SSD1306_UpdateOLED();

  	/* Set starting point in the middle*/
  	SSD1306.CurrentX = 64;
  	SSD1306.CurrentY = 32;
  	SSD1306.Initialized = 1;
}

/**
  * @brief Fill the SSD1306 OLED display with a color
  * This function fills the SSD1306 OLED display with a color and updates the OLED display
  * @param  uint8_t color: 0 (BLACK) or 1 (WHITE)
  * @retval none
   */
void SSD1306_Fill(uint8_t color){
	/* Set memory */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

/**
  * @brief Update the SSD1306 OLED display
  * This function updates the SSD1306 OLED display with pattern stored in SSD1306_Buffer
  * @param  none
  * @retval none
   */
void SSD1306_UpdateOLED(void) {
	uint8_t m;

	for (m = 0; m < 8; m++) {
		SSD1306_WRITECOMMAND(0xB0 + m);
		SSD1306_WRITECOMMAND(0x00);
		SSD1306_WRITECOMMAND(0x10);

    write_multiple_SSD1306(&SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}

/**
  * @brief Write multiple consecutive bytes to the I2C
  * This function writes multiple consecutive bytes to the I2C, 
  	the first byte sent is the address of the first register to write 
  	and then a stream of bytes is sent (storing is handled by the slave)
  * @param  uint8_t data: buffer of commands to send
  			uint8_t count: number of commands to send
   */
void write_multiple_SSD1306(uint8_t* data, uint16_t count){ 
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADDRESS, 0x40, 1, data, count, I2C_TIMEOUT);
}

/**
  * @brief Toggle color of the SSD1306 OLED display
  * This function inverts the color of the SSD1306 OLED display (Black->White, White->Black)
  * @param  none
  * @retval none
   */
void SSD1306_ToggleInvert(void) {
	uint16_t i;

	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;

	/* Do memory toggle */
	for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
		SSD1306_Buffer[i] = ~SSD1306_Buffer[i];
	}
}

/**
  * @brief Draw a pixel
  * This function draws a pixel on the OLED display
  * @param  uint16_t x: x position on the display (0-128)
  			uint16_t y: y position on the display (0-64)
  			uint8_t color: 0 (BLACK) or 1 (WHITE)
  * @retval none
   */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Error */
		return;
	}

	/* Check if pixels are inverted */
	if (SSD1306.Inverted) {
		color = (uint8_t)!color;
	}

	/* Set color */
	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	} else {
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}

/**
  * @brief Switch on the SSD1306 OLED display
  * This function switches on the SSD1306 OLED display
  * @param  none
  * @retval none
   */
void SSD1306_ON(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x14);
	SSD1306_WRITECOMMAND(0xAF);
}

/**
  * @brief Switch off the SSD1306 OLED display
  * This function switches off the SSD1306 OLED display
  * @param  none
  * @retval none
   */
void SSD1306_OFF(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x10);
	SSD1306_WRITECOMMAND(0xAE);
}

/**
  * @brief Update the position of the virtual cursor
  * This function updates the position of the virtual cursor of the SSD1306 OLED display
  * @param  uint16_t x: x position on the display (0-128)
  			uint16_t y: y position on the display (0-64)
  * @retval none
   */
void SSD1306_GotoXY(uint16_t x, uint16_t y) {
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

/**
  * @brief Display a character
  * This function displays a char at the current position of the virtual cursor on the SSD1306 OLED display
  * @param  char ch: char to display (eg 'c' or '4')
  			FontDef_t* Font: size of the character (Font_7x10, Font_11x18 or Font_16x26)
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color) {
	uint32_t i, b, j;

	/* Go through font */
	for (i = 0; i < Font->FontHeight; i++) {
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++) {
			if ((b << j) & 0x8000) {
				SSD1306_DrawPixel(((SSD1306.CurrentX + j)%SSD1306_WIDTH), ((SSD1306.CurrentY + i)%SSD1306_HEIGHT), (uint8_t) color);
			} else {
				SSD1306_DrawPixel(((SSD1306.CurrentX + j)%SSD1306_WIDTH), ((SSD1306.CurrentY + i)%SSD1306_HEIGHT), (uint8_t) !color);
			}
		}
	}

	/* Increase pointer */
	SSD1306.CurrentX += Font->FontWidth;

	/* Return character written */
	return ch;
}

/**
  * @brief Display a string
  * This function displays a string at the current position of the virtual cursor on the SSD1306 OLED display
  * @param  char* str: string to display (eg "ciao" or "pizza")
  			FontDef_t* Font: size of the character (Font_7x10, Font_11x18 or Font_16x26)
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color) {
	/* Write characters */
	while (*str) {
		/* Write character by character */
		if (SSD1306_Putc(*str, Font, color) != *str) {
			/* Return error */
			return *str;
		}

		/* Increase string pointer */
		str++;
	}

	/* Everything OK, zero should be returned */
	return *str;
}

/**
  * @brief Display a line
  * This function displays a line on the SSD1306 OLED display
  * @param  uint16_t x0: x of the origin of the line
  			uint16_t y0: y of the origin of the line
  			uint16_t x1: x of the end of the line
  			uint16_t y1: y of the end of the line
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c) {
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH) {
		x0 = SSD1306_WIDTH - 1;
	}
	if (x1 >= SSD1306_WIDTH) {
		x1 = SSD1306_WIDTH - 1;
	}
	if (y0 >= SSD1306_HEIGHT) {
		y0 = SSD1306_HEIGHT - 1;
	}
	if (y1 >= SSD1306_HEIGHT) {
		y1 = SSD1306_HEIGHT - 1;
	}

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			SSD1306_DrawPixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			SSD1306_DrawPixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		SSD1306_DrawPixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}
}

/**
  * @brief Display a triangle
  * This function displays a triangle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the first vertex of the triangle
  			uint16_t y0: y of the first vertex of the triangle
  			uint16_t x1: x of the second vertex of the triangle
  			uint16_t y1: y of the second vertex of the triangle
  			uint16_t x1: x of the third vertex of the triangle
  			uint16_t y1: y of the third vertex of the triangle 			
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color) {
	/* Draw lines */
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x3, y3, color);
	SSD1306_DrawLine(x3, y3, x1, y1, color);
}

/**
  * @brief Display a filled triangle
  * This function displays a filled triangle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the first vertex of the triangle
  			uint16_t y0: y of the first vertex of the triangle
  			uint16_t x1: x of the second vertex of the triangle
  			uint16_t y1: y of the second vertex of the triangle
  			uint16_t x1: x of the third vertex of the triangle
  			uint16_t y1: y of the third vertex of the triangle 			
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color) {
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		SSD1306_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

/**
  * @brief Display a circle
  * This function displays a circle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the center of the circle
  			uint16_t y0: y of the center of the circle
  			uint16_t r: radius of the circle
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x0 + x, y0 + y, c);
        SSD1306_DrawPixel(x0 - x, y0 + y, c);
        SSD1306_DrawPixel(x0 + x, y0 - y, c);
        SSD1306_DrawPixel(x0 - x, y0 - y, c);

        SSD1306_DrawPixel(x0 + y, y0 + x, c);
        SSD1306_DrawPixel(x0 - y, y0 + x, c);
        SSD1306_DrawPixel(x0 + y, y0 - x, c);
        SSD1306_DrawPixel(x0 - y, y0 - x, c);
    }
}

/**
  * @brief Display a filled circle
  * This function displays a filled circle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the center of the circle
  			uint16_t y0: y of the center of the circle
  			uint16_t r: radius of the circle
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SSD1306_DrawPixel(x0, y0 + r, c);
    SSD1306_DrawPixel(x0, y0 - r, c);
    SSD1306_DrawPixel(x0 + r, y0, c);
    SSD1306_DrawPixel(x0 - r, y0, c);
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, c);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, c);
      	SSD1306_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, c);

        SSD1306_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, c);
        SSD1306_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, c);
    }
}

/**
  * @brief Display a rectangle
  * This function displays a rectangle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the upper left vertex of the rectangle
  			uint16_t y0: y of the upper left vertex of the rectangle
  			uint16_t w: width of the rectangle
  			uint16_t h: height of the rectangle
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c) {
	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw 4 lines */
	SSD1306_DrawLine(x, y, x + w, y, c);         /* Top line */
	SSD1306_DrawLine(x, y + h, x + w, y + h, c); /* Bottom line */
	SSD1306_DrawLine(x, y, x, y + h, c);         /* Left line */
	SSD1306_DrawLine(x + w, y, x + w, y + h, c); /* Right line */
}

/**
  * @brief Display a filled rectangle
  * This function displays a filled rectangle on the SSD1306 OLED display
  * @param  uint16_t x0: x of the upper left vertex of the rectangle
  			uint16_t y0: y of the upper left vertex of the rectangle
  			uint16_t w: width of the rectangle
  			uint16_t h: height of the rectangle
  			uint8_t color: 0 (BLACK) or 1 (WHITE) 
  * @retval none
   */
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c) {
	uint8_t i;

	/* Check input parameters */
	if (
		x >= SSD1306_WIDTH ||
		y >= SSD1306_HEIGHT
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SSD1306_WIDTH) {
		w = SSD1306_WIDTH - x;
	}
	if ((y + h) >= SSD1306_HEIGHT) {
		h = SSD1306_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		SSD1306_DrawLine(x, y + i, x + w, y + i, c);
	}
}
