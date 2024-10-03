/**
  *******************************************************************************
  *
  * @file    fonts.h
  * @brief   Header for fonts.c file
  * @version v1.0
  * @date    01.09.2023
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2023 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FONTS_H_
#define __FONTS_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  const uint8_t width;
  uint8_t height;
  const uint16_t *data;
} FontDef;

/* Exported variables --------------------------------------------------------*/
extern const FontDef *Font_7x10;
extern const FontDef *Font_11x18;
extern const FontDef *Font_16x26;

#endif /* __FONTS_H_ */
