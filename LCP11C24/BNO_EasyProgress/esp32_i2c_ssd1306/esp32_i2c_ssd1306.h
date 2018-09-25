/**
 * @author  Gregorio Aiello
 * @email   gregorio.aiello91@gmail.com
 * @version v1.0
 * @license GNU GPL v3
 * @brief   Library for 128x64 SSD1306 I2C LCD on ESP32 based on the (amazing) work by Tilen Majerle
 */

#ifndef ESP32_I2C_SSD1306_H
#define ESP32_I2C_SSD1306_H

#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include <fonts.h>
// I2C driver
#include "driver/i2c.h"
// Error library
#include "esp_err.h"

#define SSD1306_COLOR_BLACK                0x00
#define SSD1306_COLOR_WHITE                0x01
#define SSD1306_WIDTH                      128
#define SSD1306_HEIGHT                     64
#define BLINK_GPIO                         (gpio_num_t)25
#define SSD1306_I2C_ADDR                   0x3C
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define I2C_DATA_LEN                       2

#define SSD1306_WRITECOMMAND(command)      support_routine_I2C_Write_command(0x00, command)

#define ABS(x)   ((x) > 0 ? (x) : -(x))


// I2C functions
void support_routine_I2C_Write_command(uint8_t reg, uint8_t command);
void configure_I2C();
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size);

void configure_SSD1306();
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateOLED(void);
void write_multiple_SSD1306(uint8_t* data, uint16_t count);
void SSD1306_ToggleInvert(void);
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
char SSD1306_Putc(char ch, TM_FontDef_t* Font, uint8_t color);
char SSD1306_Puts(char* str, TM_FontDef_t* Font, uint8_t color);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t c);
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint8_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void SSD1306_GotoXY(uint16_t x, uint16_t y);
void SSD1306_ON(void);
void SSD1306_OFF(void);

/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

/* SSD1306 data buffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static SSD1306_t SSD1306;

#endif
