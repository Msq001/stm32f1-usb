/**
  ******************************************************************************
  * @file    mass_mal.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Medium Access Layer interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "mass_mal.h"
#include "sd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t Mass_Memory_Size;
uint32_t Mass_Block_Size;
uint32_t Mass_Block_Count;
__IO uint32_t Status = 0;

#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
SD_CardInfo mSDCardInfo;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : MAL_Init
* Description    : Initializes the Media on the STM32
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Init(uint8_t lun)
{
  uint16_t status = MAL_OK;

  switch (lun)
  {
    case 0:
      Status = SD_Init();
      break;
    default:
      return MAL_FAIL;
  }
	if (Status != 0)  return MAL_FAIL;
  return status;
}
/*******************************************************************************
* Function Name  : MAL_Write
* Description    : Write sectors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_Write(uint8_t lun, uint32_t Memory_Offset, uint32_t *Writebuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:
//    Status = SD_Write_Data((uint8_t*)Writebuff, Memory_Offset, 1);
			Status = SD_Write_Data((u8*)Writebuff, Memory_Offset>>9, Transfer_Length>>9);   
//#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
//    Status = SD_WaitWriteOperation();  
//    while(SD_GetStatus() != SD_TRANSFER_OK);
//      if ( Status != SD_OK )
//      {
//        return MAL_FAIL;
//      }      
//#endif /* USE_STM3210E_EVAL ||USE_STM32L152D_EVAL*/      
      break;
      
    default:
      return MAL_FAIL;
  }
	if (Status != 0)  return MAL_FAIL;
  return MAL_OK;
}

/*******************************************************************************
* Function Name  : MAL_Read
* Description    : Read sectors
* Input          : None
* Output         : None
* Return         : Buffer pointer
*******************************************************************************/
uint16_t MAL_Read(uint8_t lun, uint32_t Memory_Offset, uint32_t *Readbuff, uint16_t Transfer_Length)
{

  switch (lun)
  {
    case 0:

//      SD_ReadMultiBlocks((uint8_t*)Readbuff, Memory_Offset, Transfer_Length, 1);
			Status = SD_Read_Data((u8*)Readbuff, Memory_Offset>>9, Transfer_Length>>9);	 
//#if defined(USE_STM3210E_EVAL) || defined(USE_STM32L152D_EVAL)
//      Status = SD_WaitReadOperation();
//      while(SD_GetStatus() != SD_TRANSFER_OK)
//      {
//      }
//      
//      if ( Status != SD_OK )
//      {
//        return MAL_FAIL;
//      }
//#endif /* USE_STM3210E_EVAL */      
      break;
      
    default:
      return MAL_FAIL;
  }
	if (Status != 0)  return MAL_FAIL;
  return MAL_OK;
}

extern int SD_GetCardInfo(SD_CardInfo *cardinfo);
/*******************************************************************************
* Function Name  : MAL_GetStatus
* Description    : Get status
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint16_t MAL_GetStatus (uint8_t lun)
{
  uint32_t DeviceSizeMul = 0, NumberOfBlocks = 0;

  if (lun == 0)
  {
    if (SD_Init() == 0)
    {
      SD_GetCardInfo(&mSDCardInfo);
      DeviceSizeMul = (mSDCardInfo.SD_csd.DeviceSizeMul + 2);

      if(mSDCardInfo.CardType == SD_TYPE_V2HC)
      {
        Mass_Block_Count = (mSDCardInfo.SD_csd.DeviceSize + 1) * 1024;
      }
      else
      {
        NumberOfBlocks  = ((1 << (mSDCardInfo.SD_csd.RdBlockLen)) / 512);
        Mass_Block_Count = ((mSDCardInfo.SD_csd.DeviceSize + 1) * (1 << DeviceSizeMul) << (NumberOfBlocks/2));
      }
      Mass_Block_Size  = 512;

      Mass_Memory_Size= Mass_Block_Count * Mass_Block_Size;

      return MAL_OK;
    }
  }

  return MAL_FAIL;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

