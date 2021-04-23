/**
 ******************************************************************************
 * @file    yMEMs.c
 * @author  Jean
 * @brief   Mes fonction d'accès aux MEMs de  IKS01A3
 ******************************************************************************
 *
 *  Created on: 30 juin 2019
 */

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "yMEMs.h"
#include "yI2CprogsLCD.h"
#include "VT100.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
extern char aTxBuffer[1024];		    //buffer d'emission (UART & LCD)
uint8_t sensorId1,sensorId2,sensorId3;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);

/* functions */
/*
  * @brief  recolter toutes infos de conf d'un sensor environement
  * @param  Instance sensor instance to be used
  * @retval none
  * @note   les 'define' IKS01A3_env & _motion sont 1,2,3 ds les 2 cas; il faut donc 2 fonctions!!!!!!
*/
void yMEMS_Env_Sensors_Infos(uint32_t Instance) {
	uint8_t sensorId, sensorData;
	switch (Instance) {
		case IKS01A3_HTS221_0:
			IKS01A3_ENV_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_ENV_SENSOR_Read_Register(Instance, HTS221_STATUS_REG, &sensorData);
			snprintf(aTxBuffer, 1024, "HTS221\t\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		case IKS01A3_STTS751_0:
			IKS01A3_ENV_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_ENV_SENSOR_Read_Register(Instance, STTS751_STATUS, &sensorData);
			snprintf(aTxBuffer, 1024, "STTS751\t\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		case IKS01A3_LPS22HH_0:
			IKS01A3_ENV_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_ENV_SENSOR_Read_Register(Instance, LPS22HH_STATUS, &sensorData);
			snprintf(aTxBuffer, 1024, "LPS22HH\t\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		default:
			break;
	}
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
}

/*
  * @brief  recolter toutes infos de conf d'un sensor motion
  * @param  Instance sensor instance to be used
  * @retval none
  * @note   les 'define' IKS01A3_env & _motion sont 1,2,3 ds les 2 cas; il faut donc 2 fonctions!!!!!!
*/
void yMEMS_Motion_Sensors_Infos(uint32_t Instance) {
	uint8_t sensorId, sensorData;
	switch (Instance) {
		case IKS01A3_LSM6DSO_0:
			IKS01A3_MOTION_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_MOTION_SENSOR_Read_Register(Instance, LSM6DSO_STATUS_REG, &sensorData);
			snprintf(aTxBuffer, 1024, "LSM6DSO\t\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		case IKS01A3_LIS2DW12_0:
			IKS01A3_MOTION_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_MOTION_SENSOR_Read_Register(Instance, LIS2DW12_STATUS, &sensorData);
			snprintf(aTxBuffer, 1024, "LIS2DW12\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		case IKS01A3_LIS2MDL_0:
			IKS01A3_MOTION_SENSOR_ReadID(Instance, &sensorId);
			IKS01A3_MOTION_SENSOR_Read_Register(Instance, LIS2MDL_STATUS_REG, &sensorData);
			snprintf(aTxBuffer, 1024, "LIS2MDL\t\t%2.2Xh\t%2.2Xh\r\n" ERASELINE, (int)sensorId, (int)sensorData);
			break;
		default:
			break;
	}
	HAL_UART_Transmit(&huart2,(uint8_t *) aTxBuffer, strlen(aTxBuffer), 5000);
}

/*
  * @brief  my config for it
  * @param  none
  * @retval none
*/
void yMEMS_Config_STTS751(void) {
//	#define STTS751_CONFIGURATION        0x03U
//	typedef struct {
//	  uint8_t not_used_01                : 2;
//	  uint8_t tres                       : 2;
//	  uint8_t not_used_02                : 2;
//	  uint8_t stop                       : 1;
//	  uint8_t mask1                      : 1;
//	} stts751_configuration_t;
//	  stts751_configuration_t configuration;
//	  (uint8_t*)&configuration

	int32_t ret;
	uint8_t rr;	//recup valeur du registre
	//stts751_configuration_t * rr;	//valeur lue du registre

	//fixe seuils
	IKS01A3_ENV_SENSOR_Set_High_Temperature_Threshold(IKS01A3_STTS751_0, 29.5);
	IKS01A3_ENV_SENSOR_Set_Low_Temperature_Threshold(IKS01A3_STTS751_0, 28.0);
	//active pin for interrupt
	ret = IKS01A3_ENV_SENSOR_Set_Event_Pin(IKS01A3_STTS751_0, ENABLE);
	if (yTRC) printf("\nstts751_int conf event: %d", (int)ret);
	//active détection des seuils
	(void)IKS01A3_ENV_SENSOR_Read_Register(IKS01A3_STTS751_0, STTS751_CONFIGURATION, (uint8_t*)&rr);
	//*(stts751_configuration_t)rr->mask1 = 0U;
	//printf("\n--stts751 config lue: %x",rr);
	rr = rr & 0x7F;
	//printf("\n--stts751 config mod: %x",rr);
	ret = IKS01A3_ENV_SENSOR_Write_Register(IKS01A3_STTS751_0, STTS751_CONFIGURATION, rr);
	if (yTRC) printf("\nstts751_int conf conf: %d", (int)ret);
}



/*
  * @brief  Afficher une valeur d'un capteur MEMS sur LCD-RGB
  * @param  Instance sensor instance to be used
  * @param  ligne de l'afficheur LCD
  * @retval status
*/
void yMEMS_LCD(uint32_t Instance, char row) {
	float temperature;
	displayFloatToInt_t out_value;

    IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature);
    floatToInt(temperature, &out_value, 2);

    snprintf(aTxBuffer, 1024, "Temp[%d]: %c%d.%02d degC" ERASELINE,
    								(int)Instance, ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec);

	//snprintf(aTxBuffer, 9, "%02d:%02d:%02d", hh, mm, sc);
	//snprintf(aTxBuffer, 12, "MEMs values");
	yI2C_LCD_locate(0,row);
	yI2C_LCD_Affich_Txt(aTxBuffer);
}

/**
  * @brief  Splits a float into two integer values.
  * @param  in the float value as input
  * @param  out_value the pointer to the output integer structure
  * @param  dec_prec the decimal precision to be used
  * @retval None
  */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
  {
    out_value->sign = 1;
    in = -in;
  }

  in = in + (0.5f / pow(10, dec_prec));
  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/* That's all folks! */
