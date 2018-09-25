#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>
#include "string.h"

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} TM_FontDef_t;

typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} TM_FONTS_SIZE_t;

extern TM_FontDef_t TM_Font_7x10;

extern TM_FontDef_t TM_Font_11x18;

extern TM_FontDef_t TM_Font_16x26;

/**
 * @brief  Calculates string length and height in units of pixels depending on string and font used
 * @param  *str: String to be checked for length and height
 * @param  *SizeStruct: Pointer to empty @ref TM_FONTS_SIZE_t structure where informations will be saved
 * @param  *Font: Pointer to @ref TM_FontDef_t font used for calculations
 * @retval Pointer to string used for length and height
 */
char* TM_FONTS_GetStringSize(char* str, TM_FONTS_SIZE_t* SizeStruct, TM_FontDef_t* Font);

#endif
