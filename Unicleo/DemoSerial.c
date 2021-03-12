/**
 ******************************************************************************
 * @file    DemoSerial.c
 * @author  MEMS Software Solutions Team
 * @brief   Handle the Serial Protocol
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
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

//@TODO: import algo builder .h et .c

/* Includes ------------------------------------------------------------------*/
#include "DemoSerial.h"
#include "main.h"
#include "com.h"
#include "sensor_commands.h"
#include "serial_protocol.h"
#include "gui.h"
#include <stdio.h>

/** @addtogroup X_NUCLEO_IKS01A3_Examples X_NUCLEO_IKS01A3 Examples
 * @{
 */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
 * @{
 */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint32_t SensorsEnabled;

/* Exported variables ---------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint8_t DataLoggerActive;

/* Private variables ---------------------------------------------------------*/
#define FW_ID "101"					//a priori pour dire data logger extended(101) ; algobuilder(201)
#define FW_VERSION "6.2.1"			//version des exemlpes MEMS
#define LIB_VERSION yVER			//"1.1.4" ou plus voir main.h
#define EXPANSION_BOARD "IKS01A3"	//la carte!

//static uint8_t PresentationString[] = {"MEMS shield demo,101,6.2.0,1.0.0,IKS01A3"};
static uint8_t PresentationString[] = {"MEMS shield demo,"FW_ID","FW_VERSION","LIB_VERSION","EXPANSION_BOARD};
static volatile uint8_t DataStreamingDest = 1;		// 1|2

/*JPO: added from other sample */
volatile uint8_t SenderInterface = 0;
offline_data_t offline_data;

extern volatile uint32_t sensor_read_request;
//extern sensor_hub_data_t sensor_hub_data;

extern volatile uint32_t update_16Hz;
extern volatile uint32_t update_25Hz;
extern volatile uint32_t update_50Hz;
extern volatile uint32_t update_100Hz;
extern char Identification_String[];

/* end added */
/**
 * @brief  Build the reply header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}

/**
 * @brief  Build the nack header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
 * @brief  Initialize the streaming header
 * @param  Msg the pointer to the header to be initialized
 * @retval None
 */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}

/**
 * @brief  Initialize the streaming message
 * @param  Msg the pointer to the message to be initialized
 * @retval None
 */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint32_t i;

  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for (i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;
}

/**
 * @brief  Handle a message
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleMSG(TMsg *Msg)
/*  DestAddr | SouceAddr | CMD | SUBCMD | PAYLOAD
        1          1        1       1        N    */
{
  //if (yDBG) printf("\n\t--- HandleMsg");
  uint32_t i;
  int ret = 1;

  if (Msg->Len < 2U)
  {
    if (yDBG) printf("\n\t\t-- Msg trop court");
	return 0;
  }
  if (Msg->Data[0] != DEV_ADDR)
  {
	if (yDBG) printf("\n\t\t-- Msg bad @");
    return 0;
  }
  switch (Msg->Data[2])   /* CMD */
  {
    case CMD_Ping:
      if (Msg->Len != 3U)
      {
    	if (yDBG) printf("\n\t\t-- Msg no ping");
    	return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      break;

    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      break;

    case CMD_Read_PresString:			//ok en gui
      if (yDBG) printf("\n\t\t--presString: %u %u %u %u %u", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4]);
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      i = 0;
      while (i < (sizeof(PresentationString) - 1U))
      {
        Msg->Data[3U + i] = PresentationString[i];
        i++;
      }
      Msg->Len = 3U + i;
      UART_SendMsg(Msg);
      break;

    case CMD_CheckModeSupport:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], DATALOG_EXT_MODE, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      break;

    case CMD_Sensor:
      /* Check if the command lenght is at least 5 bytes */
      if (yDBG) printf("\n---cmdSensor: %x %x %x %x %x", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4]);
      if (Msg->Len < 5U)
      {
        return 0;
      }
      (void)Handle_Sensor_command(Msg);
      break;

    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3U)
      {
        return 0;
      }
  	  if (yDBG) printf("\n\t\t-- Msg start data stream");
      SensorsEnabled = Deserialize(&Msg->Data[3], 4);
      DataLoggerActive = 1;					//declenche log vers gui
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      break;

    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3U)
      {
        return 0;
      }
  	  if (yDBG) printf("\n\t\t-- Msg stop data stream");
      SensorsEnabled = 0;
      DataLoggerActive = 0;					//stoppe log vers gui
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      break;

    case CMD_Set_DateTime:
      if (yDBG) printf("\n\t\t-- Msg set time");
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      UART_SendMsg(Msg);
      break;

      /* JPO added from other sample */
    case CMD_PRESSURE_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER( Msg );
      Serialize_s32(&Msg->Data[3], IKS01A3_LPS22HH_0, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;

    case CMD_HUMIDITY_TEMPERATURE_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER( Msg );
      Serialize_s32(&Msg->Data[3], IKS01A3_LSM6DSO_0, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;

    case CMD_ACCELERO_GYRO_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER( Msg );
      Serialize_s32(&Msg->Data[3], IKS01A3_LSM6DSO_0, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;

    case CMD_MAGNETO_Init:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER( Msg );
      Serialize_s32(&Msg->Data[3], IKS01A3_LIS2MDL_0, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;

//    case CMD_Offline_Data:
//      if (Msg->Len != 55) return 0;
//      memcpy(&offline_data.hours, &Msg->Data[3], 1);
//      memcpy(&offline_data.minutes, &Msg->Data[4], 1);
//      memcpy(&offline_data.seconds, &Msg->Data[5], 1);
//      memcpy(&offline_data.subsec, &Msg->Data[6], 1);
//      memcpy(&offline_data.pressure, &Msg->Data[7], 4);
//      memcpy(&offline_data.temperature, &Msg->Data[11], 4);
//      memcpy(&offline_data.humidity, &Msg->Data[15], 4);
//      memcpy(&offline_data.acceleration_x_mg, &Msg->Data[19], 4);
//      memcpy(&offline_data.acceleration_y_mg, &Msg->Data[23], 4);
//      memcpy(&offline_data.acceleration_z_mg, &Msg->Data[27], 4);
//      memcpy(&offline_data.angular_rate_x_mdps, &Msg->Data[31], 4);
//      memcpy(&offline_data.angular_rate_y_mdps, &Msg->Data[35], 4);
//      memcpy(&offline_data.angular_rate_z_mdps, &Msg->Data[39], 4);
//      memcpy(&offline_data.magnetic_field_x_mgauss, &Msg->Data[43], 4);
//      memcpy(&offline_data.magnetic_field_y_mgauss, &Msg->Data[47], 4);
//      memcpy(&offline_data.magnetic_field_z_mgauss, &Msg->Data[51], 4);
//      offline_data.timestamp_ms += 1000 / sensor_hub_data.data_rate_Hz;
//      if ((offline_data.timestamp_ms - last_run_16Hz) >= 62) // 16Hz
//      {
//        update_16Hz = 1;
//        last_run_16Hz = offline_data.timestamp_ms;
//      }
//      if ((offline_data.timestamp_ms - last_run_25Hz) >= 40) // 25Hz
//      {
//        update_25Hz = 1;
//        last_run_25Hz = offline_data.timestamp_ms;
//      }
//      if ((offline_data.timestamp_ms - last_run_50Hz) >= 20) // 50Hz
//      {
//        update_50Hz = 1;
//        last_run_50Hz = offline_data.timestamp_ms;
//      }
//      if ((offline_data.timestamp_ms - last_run_100Hz) >= 10) // 100Hz
//      {
//        update_100Hz = 1;
//        last_run_100Hz = offline_data.timestamp_ms;
//      }
////      DataLoggerActive = 1;
//      sensor_read_request = 1;
//      return 1;

//    case CMD_GetFW_Info:
//      if (Msg->Len < 3) return 0;
//      BUILD_REPLY_HEADER(Msg);
//      Msg->Len = 3;
//      memcpy(&Msg->Data[3], &sensor_hub_data.data_rate_Hz, 4);
//      Msg->Len += 4;
//      i = 0;
//      while (Identification_String[i] != 0)
//      {
//				Msg->Data[7+i] = Identification_String[i];
//				i++;
//      }
//      Msg->Data[7+i] = 0;
//      i++;
//      Msg->Len += i;
//      UART_SendMsg(Msg);
//      return 1;

    default:
	  BUILD_REPLY_HEADER(Msg);
	  UART_SendMsg(Msg);
      ret = 0;
      break;
  }

  return ret;
}

/**
 * @}
 */

/**
 * @}
 */
