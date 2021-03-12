/*
 * yI2CprogsLCD.h
 *
 *  @brief Ensemble de fonction driver pour Grove LCD rgb backlight
 *  see datasheet : JHD1214 Y/YG 1.0
 *
 *  Modified on: 31 may 2019	(pour F401RE)
 *  Created on: 22 mars 2019
 *      Author: Jean
 */

#ifndef INC_YI2CPROGSLCD_H_
#define INC_YI2CPROGSLCD_H_

#if defined (STM32L4xx)
#include "stm32l4xx_hal.h"
#elif defined (STM32F401xE)
#include "stm32f4xx_hal.h"
#endif

/* HAL_I2C_function need 8-bits address in uint16_t data */
/* Grove LCD with RGB backlight */
#define LCD_add (0x3E << 1)
#define RGB_add (0x62 << 1)

// RGB register access
#define RED_REG         0x04
#define GREEN_REG       0x03
#define BLUE_REG        0x02
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08

// LCD
#define LCD_CLEARDISPLAY 0x01
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00

// flag for entry mode
//#define LCD_ENTRYLEFT 0x02

// flags for function set
//#define LCD_8BITMODE 0x10
#define LCD_2LINE 0x08
#define LCD_5x10DOTS 0x04

/* fonctions definitions */

int yI2Ctest(uint16_t DevAddress);

void yI2C_LCD_Affich_Txt(char *str);
void yI2C_LCD_locate(char col, char row);
void yI2C_LCD_clear();
void yI2C_LCD_init();

#endif /* INC_YI2CPROGSLCD_H_ */
