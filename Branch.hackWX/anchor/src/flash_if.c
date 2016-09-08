/**
  ******************************************************************************
  * @file    STM32L1xx_IAP/src/flash_if.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    24-January-2012
  * @brief   This file provides all the memory related operation functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT FILE
  * LOCATED IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/** @addtogroup STM32L1xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 
  /* Unlock the Program memory */
  FLASH_Unlock();

  /* Clear all FLASH flags */  
  FLASH_ClearFlag(FLASH_FLAG_EOP| FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );   
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint32_t FLASH_If_Erase(uint32_t StartSector,uint32_t last_page_addr)
{
  uint32_t flashaddress;
  
  flashaddress = StartSector;
  
  while (flashaddress <= (uint32_t) last_page_addr)
  {
    if (FLASH_ErasePage(flashaddress) == FLASH_COMPLETE)
    {
      flashaddress += PAGE_SIZE;
    }
    else
    {
      /* Error occurred while page erase */
      return (1);
    }
  }
  return (0);
}
/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  buf: specifies the data to be programmed.
  * @param  iNbrToWrite: the number to read from flash
  * @retval if success return the number to write, without error
  *  
  */
uint32_t FLASH_If_Read(uint32_t FlashAddress, uint8_t* Data ,uint16_t DataLength)
{
        uint16_t i = 0;
        while(i < DataLength ) {
           *(Data + i) = *(__IO uint8_t*)FlashAddress++;
           i++;
        }
        return i;
}
/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(uint32_t FlashAddress, uint32_t* Data ,uint16_t DataLength)
{
  FLASH_Status status = FLASH_BUSY;
  uint32_t* malPointer = (uint32_t *)Data;
  //uint32_t memBuffer[32]; /* Temporary buffer holding data that will be written in a half-page space */
  //uint32_t* mempBuffer = memBuffer;
   while (malPointer < (uint32_t*)(Data + DataLength))
   {
      /* Fill with the received buffer */
    /*while (mempBuffer < (memBuffer + 32))
    {
     
      if (malPointer < ((uint32_t *)Data + DataLength))
      {
        *(uint32_t *)(mempBuffer++) = *(uint32_t *)(malPointer++);
      }
      else 
      {
        *(uint32_t *)(mempBuffer++) = 0;
      }
    }*/
     /* Program the data received into STM32F10x Flash */
    status = FLASH_ProgramWord(FlashAddress, *malPointer);
    if (status != FLASH_COMPLETE)
    {
      /* Error occurred while writing data in Flash memory */
      return 1;
    }

     if (*(uint32_t*)FlashAddress != *malPointer)
     {
       return 2;
     }
     FlashAddress += 4;
     malPointer += 1;
   }  

  return (0);
}

/**
  * @brief  Calculate the number of pages
  * @param  Size: The image size
  * @retval The number of pages
  */
uint32_t FLASH_PagesMask(__IO uint32_t Size)
{
  uint32_t pagenumber = 0x0;
  uint32_t size = Size;

  if ((size % PAGE_SIZE) != 0)
  {
    pagenumber = (size / PAGE_SIZE) + 1;
  }
  else
  {
    pagenumber = size / PAGE_SIZE;
  }
  return pagenumber;

}

/**
  * @brief  Disable the write protection of desired pages
  * @param  None
  * @retval None
  */
void FLASH_DisableWriteProtectionPages(uint32_t UserMask)
{
  uint32_t useroptionbyte = 0, WRPR = 0;
  uint16_t var1 = OB_IWDG_SW, var2 = OB_STOP_NoRST, var3 = OB_STDBY_NoRST;
  FLASH_Status status = FLASH_BUSY;

  WRPR = FLASH_GetWriteProtectionOptionByte();

  /* Test if user memory is write protected */
  if ((WRPR & UserMask) != UserMask)
  {
    useroptionbyte = FLASH_GetUserOptionByte();

    UserMask |= WRPR;

    status = FLASH_EraseOptionBytes();

    if (UserMask != 0xFFFFFFFF)
    {
      status = FLASH_EnableWriteProtection((uint32_t)~UserMask);
    }

    /* Test if user Option Bytes are programmed */
    if ((useroptionbyte & 0x07) != 0x07)
    { 
      /* Restore user Option Bytes */
      if ((useroptionbyte & 0x01) == 0x0)
      {
        var1 = OB_IWDG_HW;
      }
      if ((useroptionbyte & 0x02) == 0x0)
      {
        var2 = OB_STOP_RST;
      }
      if ((useroptionbyte & 0x04) == 0x0)
      {
        var3 = OB_STDBY_RST;
      }

      FLASH_UserOptionByteConfig(var1, var2, var3);
    }

    if (status == FLASH_COMPLETE)
    {
      //SerialPutString("Write Protection disabled...\r\n");

      //SerialPutString("...and a System Reset will be generated to re-load the new option bytes\r\n");

      /* Generate System Reset to load the new option byte values */
      NVIC_SystemReset();
    }
    else
    {
      //SerialPutString("Error: Flash write unprotection failed...\r\n");
    }
  }
  else
  {
    //SerialPutString("Flash memory not write protected\r\n");
  }
}

uint32_t FLASH_If_GetWriteProtectionStatus(void)
{
  uint32_t status = 0;
  uint32_t BlockNbr = 0, UserMemoryMask = 0;
  uint32_t FlashDestination = ApplicationAddress;
  /* Get the number of block (4 or 2 pages) from where the user program will be loaded */
  BlockNbr = (FlashDestination - 0x08000000) >> 12;

  /* Compute the mask to test if the Flash memory, where the user program will be
     loaded, is write protected */
#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
  UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
#else /* USE_STM3210E_EVAL */
  if (BlockNbr < 62)
  {
    UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
  }
  else
  {
    UserMemoryMask = ((uint32_t)0x80000000);
  }
#endif /* (STM32F10X_MD) || (STM32F10X_MD_VL) */
  status = ((FLASH_GetWriteProtectionOptionByte() & UserMemoryMask) != UserMemoryMask);
  return status;
}


/**
  * @}
  */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
