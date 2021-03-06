/**
  ******************************************************************************
  * @file    app_x-cube-mems1.c
  * @brief   This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.6.2.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2019 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */
/* a ma facon */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <app_x-cube-mems1.h>
#include "main.h"
#include <stdio.h>

#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "stm32f4xx_nucleo.h"
#include "math.h"

#include "yMEMS.h"
#include "VT100.h"

#include "CubeMon.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256  

/* Private variables ---------------------------------------------------------*/
volatile uint8_t PushButtonDetected = 0;
//extern volatile uint8_t PushButtonDetected;
static uint8_t verbose = 1;  /* Verbose output to UART terminal ON/OFF. */
static IKS01A3_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A3_MOTION_INSTANCES_NBR];
static IKS01A3_ENV_SENSOR_Capabilities_t EnvCapabilities[IKS01A3_ENV_INSTANCES_NBR];
static char dataOut[MAX_BUF_SIZE];

float nr_STTS751_Temp;
float nr_HTS221_Temp;
float nr_LPS22HH_Temp;

/* Private function prototypes -----------------------------------------------*/
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
static void Accelero_Sensor_Handler(uint32_t Instance);
static void Gyro_Sensor_Handler(uint32_t Instance);
static void Magneto_Sensor_Handler(uint32_t Instance);
static void Temp_Sensor_Handler(uint32_t Instance);
static void Hum_Sensor_Handler(uint32_t Instance);
static void Press_Sensor_Handler(uint32_t Instance);
static void MX_IKS01A3_DataLogTerminal_Init(void);
static void MX_IKS01A3_DataLogTerminal_Process(void);

  /* USER CODE 0 */ 
extern UART_HandleTypeDef huart2;

  /* USER CODE END 0 */

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */
  
  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */
  MX_IKS01A3_DataLogTerminal_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */
  verbose = 0; 			/* No Verbose */

  /* JPO MEMs Inits /config (voir yMEMS folder) */
  yMEMS_Config_STTS751();

  /* USER CODE END MEMS_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */
//	snprintf(dataOut, MAX_BUF_SIZE, DECRC "\n-------------------\n");		//remet le curseur a la position ssauvegardee
//	HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A3_DataLogTerminal_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */
  //display on LCD
  yMEMS_LCD(IKS01A3_STTS751_0, 0);		//ok
  yMEMS_LCD(IKS01A3_HTS221_0, 1);		//ok!

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A3_DataLogTerminal_Init(void)
{
  displayFloatToInt_t out_value_odr;
  int i;

  /* yunicleo: suppression des init BSP_LED/BUTTON/COM/PushButtonState/... */

//  /* Check what is the Push Button State when the button is not pressed. It can change across families */
//  PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;
//
  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO);

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2DW12_0, MOTION_ACCELERO);

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO);

  for(i = 0; i < IKS01A3_MOTION_INSTANCES_NBR; i++)
  {
    IKS01A3_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nMotion Sensor Instance %d capabilities: \r\n ACCELEROMETER: %d\r\n GYROSCOPE: %d\r\n MAGNETOMETER: %d\r\n LOW POWER: %d\r\n",
             i, MotionCapabilities[i].Acc, MotionCapabilities[i].Gyro, MotionCapabilities[i].Magneto, MotionCapabilities[i].LowPower);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(MotionCapabilities[i].AccMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX ACC ODR: %d.%03d Hz, MAX ACC FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].AccMaxFS);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(MotionCapabilities[i].GyroMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX GYRO ODR: %d.%03d Hz, MAX GYRO FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].GyroMaxFS);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(MotionCapabilities[i].MagMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX MAG ODR: %d.%03d Hz, MAX MAG FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].MagMaxFS);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  }

  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE | ENV_PRESSURE);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE);

  for(i = 0; i < IKS01A3_ENV_INSTANCES_NBR; i++)
  {
    IKS01A3_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nEnvironmental Sensor Instance %d capabilities: \r\n TEMPERATURE: %d\r\n PRESSURE: %d\r\n HUMIDITY: %d\r\n LOW POWER: %d\r\n",
             i, EnvCapabilities[i].Temperature, EnvCapabilities[i].Pressure, EnvCapabilities[i].Humidity, EnvCapabilities[i].LowPower);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(EnvCapabilities[i].TempMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX TEMP ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(EnvCapabilities[i].PressMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX PRESS ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
    floatToInt(EnvCapabilities[i].HumMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX HUM ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int, (int)out_value_odr.out_dec);
    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  }
}

///**
//  * @brief  BSP Push Button callback
//  * @param  Button Specifies the pin connected EXTI line
//  * @retval None.
//  */
//void BSP_PB_Callback(Button_TypeDef Button)
//{
//  PushButtonDetected = 1;
//}

/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A3_DataLogTerminal_Process(void)
{
  int i;

  if (PushButtonDetected != 0U)
  {
	//yF4unicleo a checker
    /* Debouncing */
    HAL_Delay(50);

    /* Reset Interrupt flag */
    PushButtonDetected = 0;

    /* Do nothing */
  }

  for(i = 0; i < IKS01A3_MOTION_INSTANCES_NBR; i++)
  {
    if(MotionCapabilities[i].Acc)
    {
      Accelero_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Gyro)
    {
      Gyro_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Magneto)
    {
      Magneto_Sensor_Handler(i);
    }
  }

  for(i = 0; i < IKS01A3_ENV_INSTANCES_NBR; i++)
  {
    if(EnvCapabilities[i].Humidity)
    {
      Hum_Sensor_Handler(i);
    }
    if(EnvCapabilities[i].Temperature)
    {
      Temp_Sensor_Handler(i);
    }
    if(EnvCapabilities[i].Pressure)
    {
      Press_Sensor_Handler(i);
    }
  }

  HAL_Delay( 1000 );
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

/**
  * @brief  Handles the accelerometer axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Accelero_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A3_MOTION_SENSOR_Axes_t acceleration;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC_X[%d]: %d\t\tACC_Y[%d]: %d\t\tACC_Z[%d]: %d" ERASELINE, (int)Instance,
             (int)acceleration.x, (int)Instance, (int)acceleration.y, (int)Instance, (int)acceleration.z);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);


  if (verbose == 1)
  {
    if (IKS01A3_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_ACCELERO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetFullScale(Instance, MOTION_ACCELERO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d g\r\n", (int)Instance, (int)fullScale);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

/**
  * @brief  Handles the gyroscope axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Gyro_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A3_MOTION_SENSOR_Axes_t angular_velocity;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR_X[%d]: %d\t\tGYR_Y[%d]: %d\t\tGYR_Z[%d]: %d" ERASELINE, (int)Instance,
             (int)angular_velocity.x, (int)Instance, (int)angular_velocity.y, (int)Instance, (int)angular_velocity.z);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);

  if (verbose == 1)
  {
    if (IKS01A3_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_GYRO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetFullScale(Instance, MOTION_GYRO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d dps\r\n", (int)Instance, (int)fullScale);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

/**
  * @brief  Handles the magneto axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Magneto_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A3_MOTION_SENSOR_Axes_t magnetic_field;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nMAG[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nMAG_X[%d]: %d\t\tMAG_Y[%d]: %d\t\tMAG_Z[%d]: %d" ERASELINE, (int)Instance,
             (int)magnetic_field.x, (int)Instance, (int)magnetic_field.y, (int)Instance, (int)magnetic_field.z);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);

  if (verbose == 1)
  {
    if (IKS01A3_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_MAGNETO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_MOTION_SENSOR_GetFullScale(Instance, MOTION_MAGNETO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d gauss\r\n", (int)Instance, (int)fullScale);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

/**
  * @brief  Handles the temperature data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Temp_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float temperature;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    floatToInt(temperature, &out_value, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: %c%d.%02d degC" ERASELINE, (int)Instance, ((out_value.sign) ? '-' : '+'), (int)out_value.out_int,
             (int)out_value.out_dec);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);

  if (verbose == 1)
  {
    if (IKS01A3_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_ENV_SENSOR_GetOutputDataRate(Instance, ENV_TEMPERATURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

/**
  * @brief  Handles the pressure sensor data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Press_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float pressure;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    floatToInt(pressure, &out_value, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: %d.%02d hPa" ERASELINE, (int)Instance, (int)out_value.out_int,
             (int)out_value.out_dec);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);

  if (verbose == 1)
  {
    if (IKS01A3_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_ENV_SENSOR_GetOutputDataRate(Instance, ENV_PRESSURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

/**
  * @brief  Handles the humidity data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Hum_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float humidity;
  displayFloatToInt_t out_value;
  uint8_t whoami;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    floatToInt(humidity, &out_value, 2);
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: %d.%02d %%" ERASELINE, (int)Instance, (int)out_value.out_int,
             (int)out_value.out_dec);
  }

  HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
  //printf("%s", dataOut);

  if (verbose == 1)
  {
    if (IKS01A3_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);

    if (IKS01A3_ENV_SENSOR_GetOutputDataRate(Instance, ENV_HUMIDITY, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    HAL_UART_Transmit(&huart2,(uint8_t *) dataOut, strlen(dataOut), 5000);
	//printf("%s", dataOut);
  }
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
