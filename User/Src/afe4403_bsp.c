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
#ifdef _SPI_DMA_
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

uint32_t readvalue = 0xFFFFFFFF;
uint32_t txData = 0;
#endif



unsigned long AFE44xx_Current_Register_Settings[36] = {
  CONTROL0_VAL,           //Reg0:CONTROL0: CONTROL REGISTER 0
  LED2STC_VAL,            //Reg1:REDSTARTCOUNT: SAMPLE RED START COUNT
  LED2ENDC_VAL,           //Reg2:REDENDCOUNT: SAMPLE RED END COUNT
  LED2LEDSTC_VAL,         //Reg3:REDLEDSTARTCOUNT: RED LED START COUNT
  LED2LEDENDC_VAL,        //Reg4:REDLEDENDCOUNT: RED LED END COUNT
  ALED2STC_VAL,           //Reg5:AMBREDSTARTCOUNT: SAMPLE AMBIENT RED START COUNT
  ALED2ENDC_VAL,          //Reg6:AMBREDENDCOUNT: SAMPLE AMBIENT RED END COUNT
  LED1STC_VAL,            //Reg7:IRSTARTCOUNT: SAMPLE IR START COUNT
  LED1ENDC_VAL,           //Reg8:IRENDCOUNT: SAMPLE IR END COUNT
  LED1LEDSTC_VAL,         //Reg9:IRLEDSTARTCOUNT: IR LED START COUNT
  LED1LEDENDC_VAL,        //Reg10:IRLEDENDCOUNT: IR LED END COUNT
  ALED1STC_VAL,           //Reg11:AMBIRSTARTCOUNT: SAMPLE AMBIENT IR START COUNT
  ALED1ENDC_VAL,          //Reg12:AMBIRENDCOUNT: SAMPLE AMBIENT IR END COUNT
  LED2CONVST_VAL,         //Reg13:REDCONVSTART: REDCONVST
  LED2CONVEND_VAL,        //Reg14:REDCONVEND: RED CONVERT END COUNT
  ALED2CONVST_VAL,        //Reg15:AMBREDCONVSTART: RED AMBIENT CONVERT START COUNT
  ALED2CONVEND_VAL,       //Reg16:AMBREDCONVEND: RED AMBIENT CONVERT END COUNT
  LED1CONVST_VAL,         //Reg17:IRCONVSTART: IR CONVERT START COUNT
  LED1CONVEND_VAL,        //Reg18:IRCONVEND: IR CONVERT END COUNT
  ALED1CONVST_VAL,        //Reg19:AMBIRCONVSTART: IR AMBIENT CONVERT START COUNT
  ALED1CONVEND_VAL,       //Reg20:AMBIRCONVEND: IR AMBIENT CONVERT END COUNT
  ADCRSTSTCT0_VAL,        //Reg21:ADCRESETSTCOUNT0: ADC RESET 0 START COUNT
  ADCRSTENDCT0_VAL,       //Reg22:ADCRESETENDCOUNT0: ADC RESET 0 END COUNT
  ADCRSTSTCT1_VAL,        //Reg23:ADCRESETSTCOUNT1: ADC RESET 1 START COUNT
  ADCRSTENDCT1_VAL,       //Reg24:ADCRESETENDCOUNT1: ADC RESET 1 END COUNT
  ADCRSTSTCT2_VAL,        //Reg25:ADCRESETENDCOUNT2: ADC RESET 2 START COUNT
  ADCRSTENDCT2_VAL,       //Reg26:ADCRESETENDCOUNT2: ADC RESET 2 END COUNT
  ADCRSTSTCT3_VAL,        //Reg27:ADCRESETENDCOUNT3: ADC RESET 3 START COUNT
  ADCRSTENDCT3_VAL,       //Reg28:ADCRESETENDCOUNT3: ADC RESET 3 END COUNT
  PRP,                    //Reg29:PRPCOUNT: PULSE REPETITION PERIOD COUNT
  CONTROL1_VAL,           //Reg30:CONTROL1: CONTROL REGISTER 1 //timer enabled, averages=3, RED and IR LED pulse ON PD_ALM AND LED_ALM pins
  0x00000,                //Reg31:?: ??          
  (ENSEPGAIN + STAGE2EN_LED1 + STG2GAIN_LED1_0DB + CF_LED1_5P + RF_LED1_1M),                      //Reg32:TIAGAIN: TRANS IMPEDANCE AMPLIFIER GAIN SETTING REGISTER
  (AMBDAC_0uA + FLTRCNRSEL_500HZ + STAGE2EN_LED2 + STG2GAIN_LED2_0DB + CF_LED2_5P + RF_LED2_1M),  //Reg33:TIA_AMB_GAIN: TRANS IMPEDANCE AAMPLIFIER AND AMBIENT CANELLATION STAGE GAIN
  (LEDCNTRL_VAL),                                                                                 //Reg34:LEDCNTRL: LED CONTROL REGISTER
#ifndef __AFE4403__
  (TX_REF_1 + RST_CLK_ON_PD_ALM_PIN_DISABLE + ADC_BYP_DISABLE + TXBRGMOD_H_BRIDGE + DIGOUT_TRISTATE_DISABLE + XTAL_ENABLE + EN_FAST_DIAG + PDN_TX_OFF + PDN_RX_OFF + PDN_AFE_OFF)                 //Reg35:CONTROL2: CONTROL REGISTER 2 //bit 9
#else
    (TX_REF_1 + TXBRGMOD_H_BRIDGE + DIGOUT_TRISTATE_DISABLE + XTAL_ENABLE + EN_FAST_DIAG + PDN_TX_OFF + PDN_RX_OFF + PDN_AFE_OFF)                 //Reg35:CONTROL2: CONTROL REGISTER 2 //bit 9  
#endif    
};

uint32_t config[52] = {
  0,        // 0
  6080,     // 1
  7998,     // 2
  6000,     // 3
  7999,     // 4
  80,       // 5
  1998,     // 6
  2080,     // 7
  3998,     // 8
  2000,     // 9
  3999,     // 10
  4080,     // 11
  5998,     // 12
  6,        // 13
  1999,     // 14
  2006,     // 15
  3999,     // 16
  4006,     // 17
  5999,     // 18
  6006,     // 19
  7999,     // 20
  0,        // 21
  5,        // 22
  2000,     // 23
  2005,     // 24
  4000,     // 25
  4005,     // 26
  6000,     // 27
  6005,     // 28
  7999,     // 29
  0x101,    // 30 - Control1
  0,        // 31 -not used
  0,        // 32 - tiagain
  2,        // 33 - tia-amb-gain
  0x11414,  // 34 - ledcntrl
  0,        // 35 - control2
  0,        // 36 - not used
  0,        // 37 - not used
  0,        // 38 - not used
  0,        // 39 - not used
  0,        // 40 - not used
  0,        // 41 - read only
  0,        // 42 - read only
  0,        // 43 - read only
  0,        // 44 - read only
  0,        // 45 - read only
  0,        // 46 - read only
  0,        // 47 - read only
  0,        // 48 - read only
  0,        // 49 - control3
  0,        // 50
  0,        // 51
};

extern void Error_Handler(void);
void AFE4403_Reset(void);


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
#if 1
#ifndef _NO_DEBUG_
    printf("SYSCLK = %d",     HAL_RCC_GetSysClockFreq());
    printf("\r\nHCLK   = %d",     HAL_RCC_GetHCLKFreq());
    printf("\r\nPCLK1  = %d",     HAL_RCC_GetPCLK1Freq());
    printf("\r\nPCLK2  = %d", HAL_RCC_GetPCLK2Freq());
#endif
#endif
}

void SPI_Init(void)
{
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;
  
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  * @brief  AFE4403_register_init.
  * @param  None
  * @retval None
  */
void AFE4403_register_init(void)
{
#if 0
  /* AFE4403 reset */
  AFE4403_Reset();


  /* Register init */
  /* Read disable */
  AFE4403_SPIx_Read_Disable();

  AFE4403_SPIx_Write(PRPCOUNT, PRP);
  
  AFE4403_SPIx_Write(LED2STC, LED2STC_VAL); 
  AFE4403_SPIx_Read_Enable();
  AFE4403_SPIx_Read(LED2STC);
  AFE4403_SPIx_Read_Disable();

  
  AFE4403_SPIx_Write(LED2ENDC, LED2ENDC_VAL);
  AFE4403_SPIx_Write(LED2LEDSTC, LED2LEDSTC_VAL);
  AFE4403_SPIx_Write(LED2LEDENDC, LED2LEDENDC_VAL);
  AFE4403_SPIx_Write(ALED2STC, ALED2STC_VAL);
  AFE4403_SPIx_Write(ALED2ENDC, ALED2ENDC_VAL);
  AFE4403_SPIx_Write(LED1STC, LED1STC_VAL);
  AFE4403_SPIx_Write(LED1ENDC, LED1ENDC_VAL);
  AFE4403_SPIx_Write(LED1LEDSTC, LED1LEDSTC_VAL);
  AFE4403_SPIx_Write(LED1LEDENDC, LED1LEDENDC_VAL);
  AFE4403_SPIx_Write(ALED1STC, ALED1STC_VAL);
  AFE4403_SPIx_Write(ALED1ENDC, ALED1ENDC_VAL);
  AFE4403_SPIx_Write(LED2CONVST, LED2CONVST_VAL);
  AFE4403_SPIx_Write(LED2CONVEND, LED2CONVEND_VAL);
  AFE4403_SPIx_Write(ALED2CONVST, ALED2CONVST_VAL);
  AFE4403_SPIx_Write(ALED2CONVEND, ALED2CONVEND_VAL);
  AFE4403_SPIx_Write(LED1CONVST, LED1CONVST_VAL);
  AFE4403_SPIx_Write(LED1CONVEND, LED1CONVEND_VAL);
  AFE4403_SPIx_Write(ALED1CONVST, ALED1CONVST_VAL);
  AFE4403_SPIx_Write(ALED1CONVEND, ALED1CONVEND_VAL);
  AFE4403_SPIx_Write(ADCRSTSTCT0, ADCRSTSTCT0_VAL);
  AFE4403_SPIx_Write(ADCRSTENDCT0, ADCRSTENDCT0_VAL);
  AFE4403_SPIx_Write(ADCRSTSTCT1, ADCRSTSTCT1_VAL);
  AFE4403_SPIx_Write(ADCRSTENDCT1, ADCRSTENDCT1_VAL);
  AFE4403_SPIx_Write(ADCRSTSTCT2, ADCRSTSTCT2_VAL);
  AFE4403_SPIx_Write(ADCRSTENDCT2, ADCRSTENDCT2_VAL);
  AFE4403_SPIx_Write(ADCRSTSTCT3, ADCRSTSTCT3_VAL);
  AFE4403_SPIx_Write(ADCRSTENDCT3, ADCRSTENDCT3_VAL);

  AFE4403_SPIx_Write(CONTROL0, AFE44xx_Current_Register_Settings[0]);            //0x00
  //AFE4403_SPIx_Write(CONTROL2, AFE44xx_Current_Register_Settings[35]);           //0x23
  //AFE4403_SPIx_Write(TIAGAIN, AFE44xx_Current_Register_Settings[32]);            //0x20
  //AFE4403_SPIx_Write(TIA_AMB_GAIN, AFE44xx_Current_Register_Settings[33]);       //0x21
  AFE4403_SPIx_Write(CONTROL2, 0);           //0x23
  AFE4403_SPIx_Write(TIAGAIN, 0);            //0x20
  AFE4403_SPIx_Write(TIA_AMB_GAIN, 2);       //0x21
  AFE4403_SPIx_Write(LEDCNTRL, AFE44xx_Current_Register_Settings[34]);           //0x22
  //AFE4403_SPIx_Write(CONTROL1, AFE44xx_Current_Register_Settings[30]);           //0x1E
  AFE4403_SPIx_Write(CONTROL1, 0x101);           //0x1E

#ifdef __AFE4403__
  AFE4403_SPIx_Write(CONTROL3, 0);           //0x31
  AFE4403_SPIx_Write(PDNCYCLESTC, 0);        //0x32
  AFE4403_SPIx_Write(PDNCYCLEENDC, 0);       //0x33
#endif

  AFE4403_SPIx_Read_Enable();


#ifndef _NO_DEBUG_
  /* Self-checking */
  if ( LED2STC_VAL == AFE4403_SPIx_Read(LED2STC))
  {
    printf("\n\rSelf-checking succeed!");
  }
  else
  {
    printf("\n\rSelf-checking failed!");
  }
#endif

  //AFE4403_SPIx_Read_Enable();
  for(uint8_t i = 1; i< 51; i++)
  {
    printf("\r\nRegister %d = %d", i, AFE4403_SPIx_Read(i));
  }
#endif

///////////////////// 500 HZ //////////////////////////////
#if 1
  /* Reset AFE4403 by hardware */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  AFE4403_SPIx_Write(CONTROL0, SW_RST);
  HAL_Delay(5);

  /* Write enable */
  AFE4403_SPIx_Read_Disable();

  AFE4403_SPIx_Write(LED2STC,6080);           // Start of sample LED2 pulse
  AFE4403_SPIx_Write(LED2ENDC,7998);          // End of sample LED2 pulse
  AFE4403_SPIx_Write(LED2LEDSTC,6000);        // Start of LED2 pulse
  AFE4403_SPIx_Write(LED2LEDENDC,7999);       // End of LED2 pulse
  AFE4403_SPIx_Write(ALED2STC,80);            // Start of sample LED2 ambient pulse
  AFE4403_SPIx_Write(ALED2ENDC,1998);         // End of sample LED2 ambient pulse
  
  AFE4403_SPIx_Write(LED1STC,2080);           // Start of sample LED1 pulse
  AFE4403_SPIx_Write(LED1ENDC,3998);          // End of sample LED1 pulse
  AFE4403_SPIx_Write(LED1LEDSTC,2000);        // Start of LED1 pulse
  AFE4403_SPIx_Write(LED1LEDENDC,3999);       // End of LED1 pulse
  AFE4403_SPIx_Write(ALED1STC,4080);          // Start of sample LED1 ambient pulse
  AFE4403_SPIx_Write(ALED1ENDC,5998);         // End of sample LED1 ambient pulse
  
  AFE4403_SPIx_Write(LED2CONVST,6);           // Start of convert LED2 pulse
  AFE4403_SPIx_Write(LED2CONVEND,1999);       // End of convert LED2 pulse
  AFE4403_SPIx_Write(ALED2CONVST,2006);       // Start of convert LED2 ambient pulse
  AFE4403_SPIx_Write(ALED2CONVEND,3999);      // End of convert LED2 ambient pulse
  AFE4403_SPIx_Write(LED1CONVST,4006);        // Start of convert LED1 pulse
  AFE4403_SPIx_Write(LED1CONVEND,5999);       // End of convert LED1 pulse
  AFE4403_SPIx_Write(ALED1CONVST,6006);       // Start of convert LED1 ambient pulse
  AFE4403_SPIx_Write(ALED1CONVEND,7999);      // End of convert LED1 ambient pulse
  
  AFE4403_SPIx_Write(ADCRSTSTCT0,0);          // Start of first ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTENDCT0,5);         // End of first ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTSTCT1,2000);       // Start of second ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTENDCT1,2005);      // End of second ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTSTCT2,4000);       // Start of third ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTENDCT2,4005);      // End of third ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTSTCT3,6000);       // Start of fourth ADC conversion reset pulse
  AFE4403_SPIx_Write(ADCRSTENDCT3,6005);      // End of fourth ADC conversion reset pulse
  
  AFE4403_SPIx_Write(PRPCOUNT,7999);          // End of pulse repetition period
  
  AFE4403_SPIx_Write(CONTROL1,0x000101);      // Timer Enable, Average two ADC samples
  AFE4403_SPIx_Write(TIAGAIN,     0x000000);  //
  AFE4403_SPIx_Write(TIA_AMB_GAIN,0x000002);  // 5pF, 100k
  AFE4403_SPIx_Write(LEDCNTRL,0x011414);      //LED current = 20/256*25 = 1.9mA
  AFE4403_SPIx_Write(CONTROL2,0x000000);
  AFE4403_SPIx_Write(CONTROL0, 0);

#ifdef __AFE4403__
  AFE4403_SPIx_Write(CONTROL3, 0);           //0x31
  AFE4403_SPIx_Write(PDNCYCLESTC, 0);        //0x32
  AFE4403_SPIx_Write(PDNCYCLEENDC, 0);       //0x33
#endif

  AFE4403_SPIx_Read_Enable();

#ifndef _NO_DEBUG_
  /* Self-checking */
  if ( 6080 == AFE4403_SPIx_Read(LED2STC))
  {
    printf("\n\rSelf-checking succeed!");
  }
  else
  {
    printf("\n\rSelf-checking failed!");
  }
#endif

#endif
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
  
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);  


  /* AFE4403 Register Initialization */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_Delay(5);

  // LED
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

#ifdef _SPI_DMA_
/** 
  * Enable DMA controller clock
  */
void DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}


/**
  * @brief  SPI Write a register.
  * @param  addr:  register address
  * @param  value: value to be written to register
  * @retval None
  */
void AFE4403_SPIx_Write(uint8_t addr, uint32_t value)
{
#ifndef _NO_DEBUG_
    printf("\r\nWrite register 0x%02x ( %02d ) with value 0x%06x (%6d)", addr, addr, value, value);
#endif
    txData = addr;
    uint32_t tmp = value >> 16;
    txData += tmp << 8;
  
    tmp = (value & 0x00FFFF) >> 8;
    txData += tmp << 16;
  
    tmp = (value & 0x0000FF);
    txData += tmp << 24;
  
    HAL_StatusTypeDef status = HAL_OK;
  
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1);
  
    status = HAL_SPI_Transmit_DMA(&SpiHandle, (uint8_t *)&txData, 4);

    printf("\r\n [7]");

        
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
  
  uint32_t writevalue = addr;

#ifndef _NO_DEBUG_
    printf("\r\nRead register 0x%02x ( %02d ),", addr, addr);
#endif

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(1);

  status = HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 4);




  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 read error = %d", status);
    /* Execute user timeout callback */
    while(1);
  }

  return readvalue;
}


#endif

/**
  * @brief  afe4403_init.
  * @param  None
  * @retval None
  */
void AFE4403_Init(void)
{
  /* AFE4403 GPIO init*/
  AFE4403_GPIO_init();

#ifdef _SPI_DMA_
  /* DMA init */
  DMA_Init();
#endif

  /* STM32 UART init */
  UART_Init();

  /* STM32 SPI init*/
  SPI_Init();

  /* Delay 5ms */
  HAL_Delay(5);
  
  /* AFE4403 register init */
  AFE4403_register_init(); 

  /* Enable AFE4403_DIAG_END interrupt */
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  
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
#ifdef _SPI_POLLING_
/**
  * @brief  SPI Write a register.
  * @param  addr:  register address
  * @param  value: value to be written to register
  * @retval None
  */
void AFE4403_SPIx_Write(uint8_t addr, uint32_t value)
{
#ifndef _NO_DEBUG_
  //printf("\r\nWrite register 0x%02x ( %02d ) with value 0x%06x (%6d) ", addr, addr, value, value);
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

  status = HAL_SPI_Transmit(&SpiHandle, (uint8_t *)&txData, 4, SpixTimeout);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    printf("\r\nafe4403 write error = %d", status);
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
    //printf("\r\nRead register 0x%02x ( %02d ),", addr, addr);
#endif

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  status = HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 4, SpixTimeout);

  readvalue = __REV(readvalue);
  readvalue &= 0x00FFFFFF;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  
#ifndef _NO_DEBUG_
  //printf(" The return value is 0x%06x (%6d)", readvalue, readvalue);
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
#endif


/**
  * @brief  Clear the SPI_READ register bit to 0.
  * @param  None
  * @retval None
*/
void AFE4403_SPIx_Read_Disable(void)
{
  AFE4403_SPIx_Write(CONTROL0, SPI_READ_DISABLE);
  //HAL_Delay(5);
}

/**
  * @brief  Set the SPI_READ register bit to 1.
  * @param  None
  * @retval None
*/
void AFE4403_SPIx_Read_Enable(void)
{
  AFE4403_SPIx_Write(CONTROL0, SPI_READ_ENABLE);
  
  //HAL_Delay(5);
}

void AFE4403_Reset(void)
{
  AFE4403_SPIx_Write(CONTROL0, SW_RST);
  HAL_Delay(10);
}

long collectIR(void)
{
  //AFE4403_SPIx_Read_Enable();
  //return AFE4403_SPIx_Read(0x2c);
#if 1
  long tmp = AFE4403_SPIx_Read(0x2c);

  uint32_t number = tmp & 0x001FFFFF;
  uint32_t flag = tmp &   0x00200000;
  long ret;

  if(flag)
  {
    number -= 1;
    number = ~number;
    number &= 0x001FFFFF;
    ret = -1 * number;
  }
  else
  {
    ret = number;
  }
  
  return ret;
#endif
}

long collectRED(void)
{
  //AFE4403_SPIx_Read_Enable();
  //return AFE4403_SPIx_Read(0x2a);

  long tmp = AFE4403_SPIx_Read(0x2a);

  uint32_t number = tmp & 0x001FFFFF;
  uint32_t flag = tmp &   0x00200000;
  long ret;

  if(flag)
  {
    number -= 1;
    number = ~number;
    number &= 0x001FFFFF;
    ret = -1 * number;
  }
  else
  {
    ret = number;
  }
  
  return ret;


}

long collectIRMinusAMBIR(void)
{
  //AFE4403_SPIx_Read_Enable();
  //return AFE4403_SPIx_Read(0x2f);

  long tmp = AFE4403_SPIx_Read(0x2f);

  uint32_t number = tmp & 0x001FFFFF;
  uint32_t flag = tmp &   0x00200000;
  long ret;

  if(flag)
  {
    number -= 1;
    number = ~number;
    number &= 0x001FFFFF;
    ret = -1 * number;
  }
  else
  {
    ret = number;
  }
  
  return ret;

}

long collectAMBIR(void)
{
  //AFE4403_SPIx_Read_Enable();
  //return AFE4403_SPIx_Read(0x2d);

  long tmp = AFE4403_SPIx_Read(0x2d);

  uint32_t number = tmp & 0x001FFFFF;
  uint32_t flag = tmp &   0x00200000;
  long ret;

  if(flag)
  {
    number -= 1;
    number = ~number;
    number &= 0x001FFFFF;
    ret = -1 * number;
  }
  else
  {
    ret = number;
  }
  
  return ret;

}


long collectAMBRED(void)
{
  //AFE4403_SPIx_Read_Enable();
  //return AFE4403_SPIx_Read(0x2b);

  long tmp = AFE4403_SPIx_Read(0x2b);

  uint32_t number = tmp & 0x001FFFFF;
  uint32_t flag = tmp &   0x00200000;
  long ret;

  if(flag)
  {
    number -= 1;
    number = ~number;
    number &= 0x001FFFFF;
    ret = -1 * number;
  }
  else
  {
    ret = number;
  }
  
  return ret;

}


/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/



    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
