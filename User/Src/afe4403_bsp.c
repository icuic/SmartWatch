/**
  ******************************************************************************
  * @file    afe4403_bsp.c
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
#include "afe4403_bsp.h"
#include "stdio.h"


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





uint32_t SpixTimeout = NUCLEO_SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

/* UART handler declaration */
UART_HandleTypeDef UartHandle;
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;



extern void Error_Handler(void);


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}



/*****************************************************************************
                          UART MSP INIT
  ****************************************************************************/
/**
  * @brief  This method returns the STM32F4xx NUCLEO BSP Driver revision
  * @param  None
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
void UART_Init(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None parity
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;
  
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void SPI_Init(void)
{
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;  
  SpiHandle.Init.Mode              = SPI_MODE_MASTER;
  
  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

}


/**
  * @brief  USART_init.
  * @param  None
  * @retval None
  */
void AFE4403_GPIO_init(void)
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
void AFE4403_Init(void)
{
  /* AFE4403 GPIO init*/
  AFE4403_GPIO_init();

  /* STM32 UART init */
  UART_Init();

  /* STM32 SPI init*/
  SPI_Init();

  /* Delay 1s */
  HAL_Delay(1000);
  
  //__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_8);

}



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
  * @brief  SPI Write a register.
  * @param  addr:  register address
  * @param  value: value to be written to register
  * @retval None
  */
void AFE4403_SPIx_Write(uint8_t addr, uint32_t value)
{
#ifndef _NO_DEBUG_
  printf("\r\n\r\nWrite register 0x%02x ( %02d ) with value 0x%06x in this order:\r\ntx: ", addr, addr, value);
#endif

  uint32_t tmp = value >> 16;
  uint32_t txData = addr;
  txData += tmp << 8;

  tmp = (value & 0x00FFFF) >> 8;
  txData += tmp << 16;

  tmp = (value & 0x0000FF);
  txData += tmp << 24;

  HAL_StatusTypeDef status = HAL_OK;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(1);

  status = HAL_SPI_Transmit(&SpiHandle, (uint8_t *)&txData, 4, SpixTimeout);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 write rror = %d", status);
    /* Execute user timeout callback */
    while(1);
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
  
  uint32_t readvalue = 0xFFFFFFFF;
  uint32_t writevalue = addr;

#ifndef _NO_DEBUG_
    printf("\r\n\r\nRead register 0x%02x ( %02d ) in this order:\r\n", addr, addr);
#endif

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(1);

  status = HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 4, SpixTimeout);

  readvalue >>= 8;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

#ifndef _NO_DEBUG_
        printf("\r\nThe return value is 0x%06x\r\n", readvalue);
#endif

  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 read error = %d", status);
    /* Execute user timeout callback */
    while(1);
  }

  return readvalue;
}


/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/



    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
