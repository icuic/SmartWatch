/**
  ******************************************************************************
  * @file    bsp_afe4403.h 
  * @author  Li wei
  * @version V1.0.0
  * @date    1-January -2014
  * @brief   Header for bsp_afe4403.c module
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_AFE4403_H
#define __BSP_AFE4403_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "main.h"

extern SPI_HandleTypeDef SpiHandle;
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Definition for SPI_AFE4403 clock resources */
#define SPI_AFE4403                      SPI2
#define SPI_AFE4403_CLK_ENABLE()         __SPI2_CLK_ENABLE()
#define SPI_AFE4403_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define SPI_AFE4403_FORCE_RESET()        __SPI2_FORCE_RESET()
#define SPI_AFE4403_RELEASE_RESET()      __SPI2_RELEASE_RESET()

/* Definition for SPI_AFE4403 Pins */
#define SPI_AFE4403_GPIO_PORT            GPIOB

#define SPI_AFE4403_NSS_PIN              GPIO_PIN_12
#define SPI_AFE4403_CLK_PIN              GPIO_PIN_13 
#define SPI_AFE4403_MISO_PIN             GPIO_PIN_14 
#define SPI_AFE4403_MOSI_PIN             GPIO_PIN_15

#define SPI_AFE4403_AF                   GPIO_AF5_SPI2


/* Exported functions ------------------------------------------------------- */
void afe4403_init(void);

#endif /* __BSP_AFE4403_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
