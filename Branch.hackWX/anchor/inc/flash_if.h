/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#define CONFIG_ADDRESS   (uint32_t)0x08003000
#define CONFIG_SIZE   (uint32_t)0x00000800
#define CONFIG_FLASH_LAST_PAGE_ADDRESS   (uint32_t)0x08003700

#define ApplicationAddress    0x8003800

#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x80000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif

/* Compute the FLASH upload image size */  
#define FLASH_IMAGE_SIZE                   (uint32_t) (FLASH_SIZE - (ApplicationAddress - 0x08000000))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartSector,uint32_t last_page_addr);
uint32_t FLASH_If_Read(uint32_t FlashAddress, uint8_t* Data ,uint16_t DataLength);
uint32_t FLASH_If_Write(uint32_t FlashAddress, uint32_t* Data, uint16_t DataLength);
uint32_t FLASH_PagesMask(__IO uint32_t Size);
void FLASH_DisableWriteProtectionPages(uint32_t UserMask);
uint32_t FLASH_If_GetWriteProtectionStatus(void);
#endif  /* __FLASH_IF_H */

/*******************(C)COPYRIGHT 2012 STMicroelectronics *****END OF FILE******/
