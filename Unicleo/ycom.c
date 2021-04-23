/**
 ******************************************************************************
 * @file    ycom.c
 * @author  MEMS Software Solutions Team
 * @brief   This file provides communication functions
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

/** renamed in v5.3 */

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include <stdio.h>
#include "ycom.h"

/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define Uart_Msg_Max_Size TMsg_MaxLen

/* Private macro -------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
UART_HandleTypeDef huart2;
volatile uint8_t UartRxBuffer[UART_RxBufferSize];
TUart_Engine UartEngine;

/* Private variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart2_rx;
static volatile uint8_t UartTxBuffer[TMsg_MaxLen * 2];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Check if a message is received via UART
 * @param  Msg the pointer to the message to be received
 * @retval 1 if a complete message is found, 0 otherwise
 */
int UART_ReceivedMSG(TMsg *Msg)
{
  uint16_t i, j, k, j2;
  uint16_t dma_counter, length;
  uint8_t data;
  uint16_t source = 0;
  uint8_t inc;

  if (Get_DMA_Flag_Status(&hdma_usart2_rx) == (uint32_t)RESET)
  {
	  if (yDBG) printf("\n---- receiveMsg: %u %u %u %u %u %u", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4],Msg->Data[5]);
	  dma_counter = (uint16_t)UART_RxBufferSize - (uint16_t)Get_DMA_Counter(&hdma_usart2_rx);

    if (dma_counter >= UartEngine.StartOfMsg)
    {
      length = dma_counter - UartEngine.StartOfMsg;
    }
    else
    {
        length = (uint16_t)UART_RxBufferSize + dma_counter - UartEngine.StartOfMsg;
    }
    j = UartEngine.StartOfMsg;

    for (k = 0; k < length; k++)
    {
        data = UartRxBuffer[j];
      j++;
      if (j >= (uint16_t)UART_RxBufferSize)
      {
        j = 0;
      }
      if (data == (uint8_t)TMsg_EOF)
      {
        j = UartEngine.StartOfMsg;
        for (i = 0; i < k; i += inc)
        {
          uint8_t  Source0;
          uint8_t  Source1;
          uint8_t *Dest;

          j2 = (j + 1U) % (uint16_t)UART_RxBufferSize;

          Source0 = UartRxBuffer[j];
          Source1 = UartRxBuffer[j2];
          Dest    = &Msg->Data[source];

          inc = (uint8_t)ReverseByteStuffCopyByte2(Source0, Source1, Dest);
          if (inc == 0U)
          {
            UartEngine.StartOfMsg = j2;
            return 0;
          }
          j = (j + inc) % (uint16_t)UART_RxBufferSize;
          source++;
        }
        Msg->Len = source;
        j = (j + 1U) % (uint16_t)UART_RxBufferSize; /* skip TMsg_EOF */
        UartEngine.StartOfMsg = j;
//        if (CHK_CheckAndRemove(Msg) != 0) /* check message integrity */
        if (CHK_CheckAndRemove(Msg)) /* check message integrity */
        {
        	return 1;
        }
      }
    }
    if (length > (uint16_t)Uart_Msg_Max_Size)
    {
      UartEngine.StartOfMsg = dma_counter;
    }
  }
  if (yDBG) printf("\n--check error 0:%u %u %u %u %u %u", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4],Msg->Data[5]);
  return 0;
}

/**
 * @brief  Send a message via UART
 * @param  Msg the pointer to the message to be sent
 * @retval None
 */
void UART_SendMsg(TMsg *Msg)
{
	if (yDBG) printf("\n\t--- SendMsg: %u %u %u %u %u %u\n", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4],Msg->Data[5]);
  uint16_t count_out;

  CHK_ComputeAndAdd(Msg);

  /* MISRA C-2012 rule 11.8 violation for purpose */
  count_out = (uint16_t)ByteStuffCopy((uint8_t *)UartTxBuffer, Msg);

  /* MISRA C-2012 rule 11.8 violation for purpose */
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)UartTxBuffer, count_out, 5000);
}

/* end of file */
