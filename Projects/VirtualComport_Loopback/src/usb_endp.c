/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Endpoint routines
  ******************************************************************************
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
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_bot.h"
#include "usb_istr.h"

#include "usb_desc.h"

#define QUEUE_BUFFER_SIZE 256
typedef struct {
  uint8_t  data[QUEUE_BUFFER_SIZE];
  uint32_t head;
  uint32_t tail;
  uint32_t size;
}QUEUE;

QUEUE usrReceive ={
  .size = QUEUE_BUFFER_SIZE
};

QUEUE usrSend = {
  .head = 0,
  .size = QUEUE_BUFFER_SIZE
};
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

uint8_t usrUSBSendBufLen(void)
{
  return (usrSend.tail + usrSend.size - usrSend.head) % usrSend.size;
}

void usrUSBSendData(char *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    usrSend.data[usrSend.tail] = buf[i];
    usrSend.tail = (usrSend.tail + 1) % usrSend.size;
  }
}
uint32_t usrUSBReceiveBufLen(void)
{
  return (usrReceive.tail + usrReceive.size - usrReceive.head) % usrReceive.size;
}

uint32_t usrUSBReceiveData(char *buf, uint32_t len)
{
	uint32_t rxMaxLen = usrUSBReceiveBufLen();

  if (len > rxMaxLen) 
  {
    len = rxMaxLen;
  }

  for (uint32_t i = 0; i < len; i++) 
  {
    buf[i] = usrReceive.data[usrReceive.head];
    usrReceive.head = (usrReceive.head + 1) % usrReceive.size;
  }
  
  return len;
}

uint8_t usrUSBReceiveOneData(void)
{
  uint8_t data = usrSend.data[usrSend.tail];
  usrSend.tail = (usrSend.tail + 1) % usrSend.size;
  return data;
}
/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void VCP_IN_Callback (void)
{
  uint32_t usrTxLen = (usrSend.tail + usrSend.size - usrSend.head) % usrSend.size;
  if(usrTxLen == 0 )  return;
  
  if(usrTxLen > VIRTUAL_COM_PORT_DATA_SIZE)
    usrTxLen = VIRTUAL_COM_PORT_DATA_SIZE;  // get cur farme send size

  uint32_t *pdwVal = (uint32_t *)(ENDP3_TXADDR * 2 + PMAAddr);
  uint16_t tmp = 0;
	uint16_t val = 0;
	for (uint32_t i = 0; i < usrTxLen; i++) 
  {
		val = usrSend.data[usrSend.head];
		usrSend.head = (usrSend.head + 1) % usrSend.size;
		if (i&1) 
    {
			*pdwVal++ = tmp | (val<<8);
		}
    else 
    {
			tmp = val;
		}
	}
  if (usrTxLen & 1)
  {
    *pdwVal = tmp;
  }
  
  SetEPTxCount(ENDP3, usrTxLen);
  SetEPTxValid(ENDP3);
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void VCP_OUT_Callback(void)
{
  uint32_t usrRxLen = GetEPRxCount(ENDP3);
  uint32_t *pdwVal = (uint32_t *)(ENDP3_RXADDR * 2 + PMAAddr);
  uint16_t tmp = 0;
	uint8_t val = 0;

	for (uint32_t i = 0; i < usrRxLen; i++)
  {
		if (i & 1)
    {
			val = tmp>>8;
		}
    else
    {
			tmp = *pdwVal++;
			val = tmp & 0xFF;
		}
		usrReceive.data[usrReceive.tail] = val;
		usrReceive.tail = (usrReceive.tail + 1) % usrReceive.size;
	}
  
  SetEPRxValid(ENDP3);
}

void EP3_IN_Callback(void)
{
  VCP_IN_Callback();
}

void EP3_OUT_Callback(void)
{
  VCP_OUT_Callback();
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    : EP1 IN Callback Routine
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
  Mass_Storage_In();
}

/*******************************************************************************
* Function Name  : EP2_OUT_Callback.
* Description    : EP2 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_OUT_Callback(void)
{
  Mass_Storage_Out();
}

#include "usb_pwr.h"
void SOF_Callback(void)
{
	static uint32_t FrameCount = 0;

	if(bDeviceState == CONFIGURED)
	{
		if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
		{
			/* Reset the frame counter */
			FrameCount = 0;

			/* Check the data to be sent through IN pipe */
			VCP_IN_Callback();//通过EP1_IN_Callback函数实现TX数据发送给USB
			//Handle_USBAsynchXfer();
		}
	}  
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
