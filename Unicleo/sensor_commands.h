/**
 *******************************************************************************
 * @file    sensor_commands.h
 * @author  MEMS Software Solutions Team
 * @brief   Header for sensor_commands.h
 *******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
 ********************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef SENSOR_COMMANDS_H
#define SENSOR_COMMANDS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "serial_protocol.h"

/* Exported defines ----------------------------------------------------------*/
#define SC_ACCELEROMETER                                                0x01
#define SC_GYROSCOPE                                                    0x02
#define SC_MAGNETOMETER                                                 0x03
#define SC_TEMPERATURE                                                  0x04
#define SC_HUMIDITY                                                     0x05
#define SC_PRESSURE                                                     0x06
#define SC_UV                                                           0x07
#define SC_SEND_DEFAULTS                                                0xFF

#define SC_GET_SENSOR_NAME                                              0x01
#define SC_READ_REGISTER                                                0x02
#define SC_WRITE_REGISTER                                               0x03
#define SC_GET_FULL_SCALE_LIST                                          0x04
#define SC_SET_FULL_SCALE                                               0x05
#define SC_GET_ODR_LIST                                                 0x06
#define SC_SET_ODR                                                      0x07
#define SC_GET_CUSTOM_NAMES                                             0x08
#define SC_GET_CUSTOM_CONFIG                                            0x09
#define SC_GET_CUSTOM_NAMES2                                            0x0A
#define SC_GET_CUSTOM_NAMES3                                            0x0B
#define SC_SET_CUSTOM_VALUES                                            0x0C
#define SC_GET_SENSOR_LIST                                              0x14
#define SC_SET_SENSOR_INDEX                                             0x15
#define SC_GET_CONFIG_STRING				                            0x19
#define SC_GET_CUSTOM_VALUES                                            0x1A

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int Handle_Sensor_command(TMsg *Msg);

#endif /* SENSOR_COMMANDS_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
