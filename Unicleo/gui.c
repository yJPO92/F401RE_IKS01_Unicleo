/**
 * @file	gui.c
 * @author  Jean
 * @brief   Handle the Unicleo-GUI interface
 *
 *  Created on: 24 sept. 2019
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "ycom.h"
#include "main.h"
#include "rtc.h"
#include "DemoSerial.h"
#include "gui.h"

extern UART_HandleTypeDef UartHandle;
extern volatile uint32_t SensorsEnabled; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint32_t SensorsEnabled = 0;    /*!< Enable Sensor Flag */
volatile uint32_t PreviousSensorsEnabled = 0;  	/*!< Previously Stored Enable Sensor Flag */
volatile uint8_t IntStatus = 0;
extern volatile uint8_t DataLoggerActive;		/* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint8_t DataLoggerActive;
static uint8_t NewData = 0;
static uint8_t NewDataFlags = 0;
static int RtcSynchPrediv;


/*
  * @brief
  * @param
  * @retval status
*/
void GUI_DataLog_Manage(void)
{
	if (yDBG) printf("\n---- GUI_DataLog_Manage: %d %d ---", (int)PreviousSensorsEnabled, (int)SensorsEnabled);
	/* Process incoming data (for GUI dialogue) */
	if (UART_ReceivedMSG((TMsg *)&msg_cmd) != 0U)	/* géré ds com.c */
	{
	   if (msg_cmd.Data[0] == DEV_ADDR)
	   {
		   (void)HandleMSG((TMsg *)&msg_cmd);		/* géré ds DemoSerial.c */
	   }
	}

	if (PreviousSensorsEnabled != SensorsEnabled)
	{
	  PreviousSensorsEnabled = SensorsEnabled;
	  Enable_Disable_Sensors();
	}

	/* refresh Unicleo GUI */
    RTC_Handler(&msg_dat);		//envoyer l'heure

    /* HTS221 */
    if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
    {
    	Hum_Sensor_Handler(&msg_dat, HumInstance);
    }
    /* HTS221 ou STTS751 ou LPS22HH */
    if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
    {
    	Temp_Sensor_Handler(&msg_dat, TmpInstance);
    	//if (yDBG) printf("\n--Temp sensor: %d", TmpInstance);
    }
    /* LSM6DSO ou LIS2DW12 */
    if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
    {
      Accelero_Sensor_Handler(&msg_dat, AccInstance);
    }
    /* LSM6DSO */
    if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
    {
      Gyro_Sensor_Handler(&msg_dat, GyrInstance);
    }
    /* LIS2MDL */
    if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
    {
      Magneto_Sensor_Handler(&msg_dat, MagInstance);
    }
    /* LPS22HH */
    if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
    {
      Press_Sensor_Handler(&msg_dat, PrsInstance);
    }

    Sensors_Interrupt_Handler(&msg_dat);

    /* prepare data to send to GUI */
    if (NewData != 0U)
    {
    	if (yDBG) printf("\n--- NewData?: %d", NewData);
    	INIT_STREAMING_HEADER(&msg_dat);	//ds DemoSerial.c
    	msg_dat.Data[55] = NewDataFlags;
    	msg_dat.Len = STREAMING_MSG_LENGTH;
    	UART_SendMsg(&msg_dat);
    	NewData = 0;
    	NewDataFlags = 0;
    }

}

/**
 * @brief  Build an array from the source data (LSB first)
 * @param  Dest destination
 * @param  Source source
 * @param  Len number of bytes
 * @retval None
 */
void SerializeToMsg(uint8_t Dest, void *Source, uint32_t Len)
{
  memcpy(&MsgDat.Data[Dest], Source, Len);
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  //if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  if (HAL_RTC_SetTime(&hrtc, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-3- Configure the Date #################################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year = 0x00;
  sdatestructure.Month = RTC_MONTH_JANUARY;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure the Time #################################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
void RTC_Handler(TMsg *Msg)
{
  uint8_t sub_sec;
  uint32_t ans_uint32;
  int32_t ans_int32;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

	/* envoyer data a Unicleo */
    (void)HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    (void)HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

    /* To be MISRA C-2012 compliant the original calculation:
       sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
       has been split to separate expressions */
    ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
    ans_int32 /= RtcSynchPrediv + 1;
    ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
    sub_sec = (uint8_t)ans_uint32;

    Msg->Data[3] = (uint8_t)stimestructure.Hours;
    Msg->Data[4] = (uint8_t)stimestructure.Minutes;
    Msg->Data[5] = (uint8_t)stimestructure.Seconds;
    Msg->Data[6] = sub_sec;
}

/**
 * @brief  Splits a float into two integer values
 * @param  In the float value as input
 * @param  OutValue the pointer to the output integer structure
 * @param  DecPrec the decimal precision to be used
 * @retval None
 */
void Float_To_Int(float In, displayFloatToInt_t *OutValue, int32_t DecPrec)
{
  if (In >= 0.0f)
  {
    OutValue->sign = 0;
  }
  else
  {
    OutValue->sign = 1;
    In = -In;
  }

  OutValue->out_int = (uint32_t)In;
  In = In - (float)(OutValue->out_int);
  OutValue->out_dec = (uint32_t)trunc(In * pow(10.0f, (float)DecPrec));
}


/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t acceleration;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_ACCELERO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 1U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);
    Serialize_s32(&Msg->Data[19], acceleration.x, 4);
    Serialize_s32(&Msg->Data[23], acceleration.y, 4);
    Serialize_s32(&Msg->Data[27], acceleration.z, 4);
  }
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t angular_velocity;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_GYRO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 2U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);
    Serialize_s32(&Msg->Data[31], angular_velocity.x, 4);
    Serialize_s32(&Msg->Data[35], angular_velocity.y, 4);
    Serialize_s32(&Msg->Data[39], angular_velocity.z, 4);
  }
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t magnetic_field;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_MAGNETO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 4U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field);
    Serialize_s32(&Msg->Data[43], (int32_t)magnetic_field.x, 4);
    Serialize_s32(&Msg->Data[47], (int32_t)magnetic_field.y, 4);
    Serialize_s32(&Msg->Data[51], (int32_t)magnetic_field.z, 4);
  }
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg the PRESSURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float pressure;
  uint8_t status = 0;

  if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_PRESSURE, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 8U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure);
    (void)memcpy(&Msg->Data[7], (void *)&pressure, sizeof(float));
  }
}


/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg the TEMPERATURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float temperature;
  uint8_t status = 0;
  uint8_t drdy = 0;
  static uint8_t stts751_is_busy = 0;

  if (Instance == IKS01A3_STTS751_0)
  {
    if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE)
    {
      if (status == 0)
      {
        stts751_is_busy = 1;
        drdy = 0;
      }
      else
      {
        if (stts751_is_busy == 1)
        {
          stts751_is_busy = 0;
          drdy = 1;
        }
      }
    }
  }
  else
  {
    if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE && status == 1U)
    {
      drdy = 1;
    }
    else
    {
      drdy = 0;
    }
  }

  if (drdy == 1)
  {
    NewData++;
    NewDataFlags |= 32U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature);
    (void)memcpy(&Msg->Data[11], (void *)&temperature, sizeof(float));
  }
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg the HUMIDITY part of the stream
 * @param  Instance the device instance
 * @retval None
 */
void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float humidity;
  uint8_t status = 0;

  if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_HUMIDITY, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 16U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity);
    (void)memcpy(&Msg->Data[15], (void *)&humidity, sizeof(float));
  }
}

/**
 * @brief  Handles the sensors interrupts
 * @param  Msg the INTERRUPT part of the stream
 * @retval None
 */
void Sensors_Interrupt_Handler(TMsg *Msg)
{
  static uint8_t mem_int_status = 0;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) IntStatus |= (1 << 0); else IntStatus &= ~(1 << 0);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) IntStatus |= (1 << 1); else IntStatus &= ~(1 << 1);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) IntStatus |= (1 << 2); else IntStatus &= ~(1 << 2);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) IntStatus |= (1 << 3); else IntStatus &= ~(1 << 3);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) IntStatus |= (1 << 4); else IntStatus &= ~(1 << 4);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) IntStatus |= (1 << 5); else IntStatus &= ~(1 << 5);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) IntStatus |= (1 << 6); else IntStatus &= ~(1 << 6);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) IntStatus |= (1 << 7); else IntStatus &= ~(1 << 7);

  if (mem_int_status != IntStatus)
  {
    NewData++;
    NewDataFlags |= 64U;
    Msg->Data[56] = IntStatus;
    mem_int_status = IntStatus;
  }
}

/**
 * @brief  Enable/disable desired sensors
 * @param  None
 * @retval None
 */
void Enable_Disable_Sensors(void)
{
  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(AccInstance, MOTION_ACCELERO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO);
  }

  if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(GyrInstance, MOTION_GYRO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(GyrInstance, MOTION_GYRO);
  }

  if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(MagInstance, MOTION_MAGNETO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(MagInstance, MOTION_MAGNETO);
  }

  if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(HumInstance, ENV_HUMIDITY);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(HumInstance, ENV_HUMIDITY);
  }

  if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(TmpInstance, ENV_TEMPERATURE);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(TmpInstance, ENV_TEMPERATURE);
  }

  if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(PrsInstance, ENV_PRESSURE);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(PrsInstance, ENV_PRESSURE);
  }
}

/* That's all folks! */

