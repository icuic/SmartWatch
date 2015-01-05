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
#include "bsp_afe4403.h"

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
//SPI_HandleTypeDef SpiHandle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  USART_init.
  * @param  None
  * @retval None
  */
void GPIO_init(void)
{
  /* ADC_RDY - PA8,  DIAG_END - PA15, AFE_/PND - PB0, AFE_/RST - PB1 */

  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable GPIOA, GPIOB Clock (to be able to program the configuration registers) */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  
  /*##-2.1- Configure PA08 and PA15 IO as external interrupt with falling edge sensitivity ###*/  
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

  /*##-2.2- Configure PB0 and PB1 IO in output push-pull mode to control the AFE_/PDF and AFE_/RST pins in AFE4403 ###*/  
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*##-3 Enable EXTI */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8 | GPIO_PIN_15);
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  

  /* AFE4403 Register Initialization */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(100);

  // LED
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}



/**
  * @brief  afe4403_init.
  * @param  None
  * @retval None
  */
void afe4403_init(void)
{
  GPIO_init();

  __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_8);
  //__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_15);

  //SPI_init();
  //printf("\n\rSPI initiate complete!\n\r");
}

