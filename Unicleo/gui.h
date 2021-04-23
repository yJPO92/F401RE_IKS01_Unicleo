/**
 * @file	gui.h
 * @author  Jean
 * @brief   Handle for gui.c
 *
 *  Created on: 24 sept. 2019
 */
/* test du fou, inclure un fichier .c avec functions utiles*/

#ifndef GUI_H_
#define GUI_H_

#include "stm32f4xx_hal.h"
#include "serial_protocol.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"

/* Enable sensor masks */
#define PRESSURE_SENSOR                         0x00000001U
#define TEMPERATURE_SENSOR                      0x00000002U
#define HUMIDITY_SENSOR                         0x00000004U
#define UV_SENSOR                               0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR                    0x00000010U
#define GYROSCOPE_SENSOR                        0x00000020U
#define MAGNETIC_SENSOR                         0x00000040U

#define MAX_BUF_SIZE 256

typedef struct displayFloatToInt_s
{
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;

TMsg msg_dat;		//for GUI dialog
TMsg msg_cmd;		//for GUI dialog
TMsg MsgDat;		//for GUI dialogue
RTC_HandleTypeDef RtcHandle;

extern uint32_t AccInstance;
extern uint32_t GyrInstance;
extern uint32_t MagInstance;
extern uint32_t HumInstance;
extern uint32_t TmpInstance;
extern uint32_t PrsInstance;

void GUI_DataLog_Manage(void);
void SerializeToMsg(uint8_t Dest, void *Source, uint32_t Len);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);
void RTC_Handler(TMsg *Msg);
void Enable_Disable_Sensors(void);
void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance);
void Sensors_Interrupt_Handler(TMsg *Msg);

#endif /* GUI_H_ */
