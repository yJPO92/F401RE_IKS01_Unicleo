/**
 *  @file yI2CprogLCDs.c
 *  @brief Ensemble de fonction driver pour Grove LCD rgb backlight
 *  see datasheet : JHD1214 Y/YG 1.0
 *
 *  Modified on: 31 may 2019	(pour F401RE)
 *  Created on: 22 mars 2019
 *  @author: Jean
 */

#include "yI2CprogsLCD.h"

//extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c1;		//I2C opendrain
/*
 * @brief  Test si une adresse I2C est prete
 * @param  device addresse 8-bits
 * @retval status as HAL_StatusTypeDef
 *		  HAL_OK       = 0x00
 *		  HAL_ERROR    = 0x01
 *		  HAL_BUSY     = 0x02
 *		  HAL_TIMEOUT  = 0x03
 */
int yI2Ctest(uint16_t DevAddress) {
	return (HAL_I2C_IsDeviceReady(&hi2c1, DevAddress, 1, 5000) + 5);
}

/*
 * @brief  This function is used to write to the LCD screen.
 * 		   It takes in a string of characters and writes them to the 0x40 register of the display.
 * @param  Texte a afficher
 * @retval none
 *  			Add I2C du LCD en dur via define
 */
void yI2C_LCD_Affich_Txt(char *str) {
    char data[2];
    data[0] = 0x40;     //@ At symbol
    while(*str)
    {
            data[1] = *str;
            HAL_I2C_Master_Transmit(&hi2c1, LCD_add, (uint8_t *)data, 2, 1000);
            str++;
    }
}

/*
 * @brief  This function sets where on the screen the text will be written next.
 * 			It takes in two values which indicate the row and column on the display that
 *			the cursor should be moved to
 * @param
 * @retval status
 */
void yI2C_LCD_locate(char col, char row)
{
    if(row == 0)
    {
        col = col | 0x80;
    }
    else
    {
        col = col | 0xc0;
    }
    char data[2];
    data[0] = 0x80;
    data[1] = col;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_add, (uint8_t *)data, 2, 1000);
}

/*
 * @brief  Efface l'ecran LCD
 * @param  none
 * @retval none
 */
void yI2C_LCD_clear() {
	char data[2] = {0x80, 0x01};
	HAL_I2C_Master_Transmit(&hi2c1, LCD_add, (uint8_t *)data, 2, 1000);
}

/*
 * @brief  Initialise LCD display
 * @param  none
 * @retval none
 */
void yI2C_LCD_init() {
	uint8_t _displayfunction = 0x00;
	uint8_t _displaycontrol = 0x00;
	char data[2];

	//Initialize displayfunction parameter for setting up LCD display
	_displayfunction |= LCD_2LINE;
	_displayfunction |= LCD_5x10DOTS;

	//Wait for more than 30 ms after power rises above 4.5V per the data sheet
	HAL_Delay(50);

	// Send first function set command. Wait longer that 39 us per the data sheet
	_displayfunction |= LCD_FUNCTIONSET;
	//data[2] = {0x80, _displayfunction};
	data[0] = 0x80;
	data[1] = 0x3C;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_add, (uint8_t *)data, 2, 1000);
	HAL_Delay(45);

	// turn the display on
    _displaycontrol |= LCD_DISPLAYON;
    //this->sendCommand(LCD_DISPLAYCONTROL | _displaycontrol);
    _displaycontrol |= LCD_DISPLAYCONTROL;
	data[0] = 0x80;
	data[1] = 0x0C;
	HAL_I2C_Master_Transmit(&hi2c1, LCD_add, (uint8_t *)data, 2, 1000);

	// clear the display
	yI2C_LCD_clear();

	// Initialize backlight
	data[0] = REG_MODE1;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, RGB_add, (uint8_t *)data, 2, 1000);
    // set MODE2 values
    // 0010 0000 -> 0x20  (DMBLNK to 1, ie blinky mode)
	data[0] = REG_MODE2;
	data[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, RGB_add, (uint8_t *)data, 2, 1000);
    // set LEDs controllable by both PWM and GRPPWM registers
	data[0] = REG_OUTPUT;
	data[1] = 0xAA;
	HAL_I2C_Master_Transmit(&hi2c1, RGB_add, (uint8_t *)data, 2, 1000);

	// set back color green
	data[0] = GREEN_REG;		//register add
	data[1] = 0x5F;				//register value
	HAL_I2C_Master_Transmit(&hi2c1, RGB_add, (uint8_t *)data, 2, 1000);
}

/* That's all folks! */
