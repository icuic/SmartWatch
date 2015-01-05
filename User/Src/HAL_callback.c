/**
  ******************************************************************************
  * @file    bsp_afe4403.c 
  * @author  li wei
  * @version V1.0.0
  * @date    1-January -2014
  * @brief   Initiate the peripherals would be used to control the AFE4403 .
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "HAL_callback.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Printf
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  USART_init.
  * @param  None
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8))
  {
    printf("\n\rADC_RDY interrupt!\n\r");
  }
  else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15))
  {
    printf("\n\rDIG_END interrupt!\n\r");
  }
}
