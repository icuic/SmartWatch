/**
  ******************************************************************************
  * @file    stm32f4xx_nucleo.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-June-2014
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32F4XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32f4xx_nucleo.h"
#include "stdio.h"
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F4XX_NUCLEO
  * @{
  */   
    
/** @addtogroup STM32F4XX_NUCLEO_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32F4xx-Nucleo Kit from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Defines
  * @{
  */ 

/**
  * @brief STM32F4xx NUCLEO BSP Driver version number V1.1.0
  */
#define __STM32F4xx_NUCLEO_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32F4xx_NUCLEO_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __STM32F4xx_NUCLEO_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F4xx_NUCLEO_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM32F4xx_NUCLEO_BSP_VERSION        ((__STM32F4xx_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32F4xx_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32F4xx_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32F4xx_NUCLEO_BSP_VERSION_RC))   

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF    
#define SD_NO_RESPONSE_EXPECTED  0x80

/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Variables
  * @{
  */ 


/**
 * @brief BUS variables
 */

uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
static SPI_HandleTypeDef hnucleo_Spi;
extern SPI_HandleTypeDef SpiHandle;

/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */
//static void       SPIx_Init(void);
//static void       SPIx_Write(uint8_t Value);
//static uint32_t   SPIx_Read(void);
static void       SPIx_Error(void);


/**
  * @}
  */ 

/** @defgroup STM32F4XX_NUCLEO_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32F4xx NUCLEO BSP Driver revision
  * @param  None
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32F4xx_NUCLEO_BSP_VERSION;
}



/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/
/**
  * @brief  SPI Read 4 bytes from device.
  * @param  None
  * @retval Read data
*/
uint32_t SPIx_Read(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue = 0x88888888;
  uint32_t writevalue = 1;//0xFFFFFFFF;
  
  status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 4, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }

  return readvalue;
}

/**
  * @brief  SPI Write a byte to device.
  * @param  Value: value to be written
  * @retval None
  */
void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&hnucleo_Spi, (uint8_t*) &Value, 1, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write a register.
  * @param  addr:  register address
  * @param  value: value to be written to register
  * @retval None
  */
void AFE4403_SPIx_Write(uint8_t addr, uint32_t value)
{
  uint32_t tmp = value >> 16;
  uint32_t txData = addr;
  txData += tmp << 8;

  tmp = (value & 0x00FFFF) >> 8;
  txData += tmp << 16;

  tmp = (value & 0x0000FF);
  txData += tmp << 24;

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&SpiHandle, (uint8_t *)&txData, 4, SpixTimeout);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 write rror = %d", status);
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Read a register.
  * @param  addr:  register address
  * @retval Read data
*/
uint32_t AFE4403_SPIx_Read(uint8_t addr)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  uint32_t readvalue = 0x88888888;
  uint32_t writevalue = addr;
  
  status = HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 4, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 read error = %d", status);
    /* Execute user timeout callback */
    SPIx_Error();
  }

  return readvalue;
}



/**
  * @brief  SPI error treatment function.
  * @param  None
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&hnucleo_Spi);
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/





/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
