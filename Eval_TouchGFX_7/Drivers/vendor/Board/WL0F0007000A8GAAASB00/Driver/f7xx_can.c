/**
  ******************************************************************************  
  * File Name          : f7xx_can.c
  * @author            : SI Application Team
  * Description        : This file provides code for the configuration
  *                      of the CAN instances to stm32f746.   
  * @version           : V0.0.1
  * @date              : 10-Jan-2022
  * @brief             : 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 </center></h2>
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
#include "f7xx_can.h"

#define USE_CAN

#if defined(USE_CAN)
#include <string.h>
#include <stdbool.h>

#define CANBUS_HOSTID                     0x321/*Define Receive Device ID*/
#define CANBUS_REMOTEID                   0x123/*Define Receive Device ID*/

#define CANx_INSTANCE                     CAN2

CAN_HandleTypeDef    CanHandle; /*CANBUS Handle*/
CAN_HandleData       CanHandleSt;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
/*****************************************************************************
  * @brief  Canx bus Initial
  * @param  
  * @retval None
  *BaudRate = APB1 Timer APB1 Peripheral clock / (Prescaler*(CAN_SJW_1TQ+CAN_BS1_2TQ+CAN_BS2_2TQ)
  ex: 125Kb = 54000000/(24*(1+9+8))
  ex: 500Kb = 54000000/(9*(1+5+6))
  ex: 1Mb   = 54000000/(9*(1+2+3))
******************************************************************************/

/*****************************************************************************
  * @brief  Can bus 2 De-Initial
  * @param  
  * @retval None
******************************************************************************/
void SmartDisplay_CANx_DeInit(void)
{   
  // Todo
}
/*****************************************************************************
  * @brief CAN MX USER USED Can Bus Tramsmite 
  *        This function frees the hardware resources used in this example:
  * @param hcan     : CAN handle pointer
  *        DeviceID : Remote Device ID
  *        pdata    : CAN Tramsmite Data pointer
  *        Size     : CAN Tramsmite Data Length Max:8
  *        Timeout  : CAN Tramsmite wait time out
  * @retval HAL_StatusTypeDef
******************************************************************************/  
HAL_StatusTypeDef SmartDisplay_CAN_Transmit(CAN_HandleTypeDef *hcan , uint16_t DeviceID, uint8_t *pdata , uint8_t Size)
{   
   HAL_StatusTypeDef  Status  =  HAL_ERROR;   
   
   if(Size>8) return Status;
  
  TxHeader.StdId = DeviceID;
  TxHeader.ExtId = CANBUS_HOSTID;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = Size;
  TxHeader.TransmitGlobalTime = ENABLE;
  
  memset(TxData,0,sizeof(TxData));
  memcpy(TxData,pdata,Size);
  
  if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {

  }

  return Status;
}
/*****************************************************************************
  *  * @brief CAN MX USER USED Can Bus Receive Function
  *        This function frees the hardware resources used in this example:
  * @param hcan    : CAN handle pointer
  *        pdata   : CAN Receive Data pointer
  *        Size    : CAN Receive Data Length Max:8
  *        Timeout : CAN Receive wait time out
  * @retval HAL_StatusTypeDef
******************************************************************************/

/*****************************************************************************
  *  * @brief CAN MX USER USED Can Bus Setting BaudRate Function
  *        This function frees the hardware resources used in this example:
  * @param hcan    : CAN handle pointer
  *        BaudRate   : 1M/500K/125M
  * @retval HAL_StatusTypeDef
******************************************************************************/

/*****************************************************************************
  *  * @brief CAN MX USER USED Can Bus HAL_CAN_RxFifo0MsgPendingCallback
  *        This function frees the hardware resources used in this example:
  * @param hcan    : CAN handle pointer
  * @retval HAL_StatusTypeDef
******************************************************************************/

/*****************************************************************************
  *  * @brief CAN MX USER USED Can Bus CAN_RxFifo0MsgPendingCallback
  *        This function frees the hardware resources used in this example:
  * @param hcan    : CAN handle pointer
  * @retval HAL_StatusTypeDef
******************************************************************************/

/*****************************************************************************
 CAN Bus Tramsmite IRQ Handler
*****************************************************************************/


/*****************************************************************************
 CAN Bus Receive IRQ Handler
*****************************************************************************/

/*****************************************************************************
USED BYTE CAN Bus Transmite Finlish Irq CallBack 
*****************************************************************************/


#endif /*defined(USE_CAN)*/

/************************ (C) COPYRIGHT WINSTAR *****END OF FILE****/
