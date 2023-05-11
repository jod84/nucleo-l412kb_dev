/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  const char payload[8] = "Kamin\r\n";
  //const uint8_t payload[8] = {0x4b, 0x61, 0x6d, 0x69, 0x6e, 0x0D, 0x0A};
  uint8_t i = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  RS485_TxEnable();
  while (1)
  {

      /* USER CODE END WHILE */
	  LL_USART_TransmitData8(USART1, (uint8_t)payload[i++]);
	  while(!LL_USART_IsActiveFlag_TXE(USART1)); // When the transmit shift-register has transmitted data in TDR to TX pin the TXE flag is set by HW.
	  if(i==8){
		  i= 0;
	  }

	  // LL_USART_IsActiveFlag_IDLE // Check if the USART IDLE line detected Flag is set or not.

	  // LL_USART_EnableIT_RXNE  // CR1 RXNEIE LL_USART_EnableIT_RXNE RXNEIE - RX Not Empty Interrupt Enabled
	  // LL_USART_IsEnabledIT_RXNE // CR1 RXNEIE LL_USART_IsEnabledIT_RXNE
	  // LL_USART_DisableIT_RXNE // CR1 RXNEIE LL_USART_DisableIT_RXNE
	  /*if(LL_USART_IsActiveFlag_RXNE(USART1)) { //Check if the USART Read Data Register Not Empty Flag is set or not.
		  uint8_t index = 0;
		  while ((uint8_t)(USART1->RDR)){
			  payload[index] = (uint8_t)LL_USART_ReceiveData8(USART1);
			  LL_USART_RequestRxDataFlush(USART1);
			  index++;
			  if(index > 254){
				  DBG_LED_Toggle();
				  break;
			  }
		  }
		  LL_USART_RequestRxDataFlush(USART1);

	  }*/
	  // LL_USART_EnableIT_TXE // Enable TX Empty Interrupt. CR1 TXEIE LL_USART_EnableIT_TXE
	  // LL_USART_DisableIT_TXE // CR1 TXEIE LL_USART_DisableIT_TXE
	  // LL_USART_IsActiveFlag_TC //Check if the USART Transmission Complete Flag is set or not. ISR TC LL_USART_IsActiveFlag_TC
	  // LL_USART_IsActiveFlag_TXE //Check if the USART Transmit Data Register Empty Flag is set or not. ISR TXE LL_USART_IsActiveFlag_TXE

	  //LL_USART_IsActiveFlag_PE //Check if the USART Parity Error Flag is set or not. ISR PE LL_USART_IsActiveFlag_PE
	  //LL_USART_IsActiveFlag_FE //Check if the USART Framing Error Flag is set or not. ISR FE LL_USART_IsActiveFlag_FE
	  // LL_USART_IsActiveFlag_NE //Check if the USART Noise error detected Flag is set or not. ISR NF LL_USART_IsActiveFlag_NE
	  // LL_USART_IsActiveFlag_ORE // Check if the USART OverRun Error Flag is set or not. ISR ORE LL_USART_IsActiveFlag_ORE

	  // Nothing has gotten loaded. The output buffer should be lowered
	  			// So that we are prepared for reception.
	  			// Enable the transmit complete interrupt and disable it there instead
	  			//LL_USART_ClearFlag_TC(USARTSettings[id].pxUSART);
	  			//LL_USART_EnableIT_TC(USARTSettings[id].pxUSART);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnablePLLMode();
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(4000000);

  LL_SetSystemCoreClock(4000000);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// Peripheral clock enable //
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	// USART1 GPIO Configuration
	// PA9   ------>  RS485_TX
	// PA10  ------>  RS485_RX
	// PA12  ------>  RS485_TXDE

	GPIO_InitStruct.Pin = RS485_TX_Pin | RS485_RX_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(RS485_GPIO_Port, &GPIO_InitStruct);

	// USART1 interrupt Init
	NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 8, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);

	// TX/RX direction
	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

	// 8 data bit, 1 start bit, 1 stop bit, no parity
	// Set the PS, PCE, M1, and M2 bits in USART_CR1 register and the STOP bit in USART_CR2 register
	LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1); // Not needed since default values

	// Configure transfer bit order : MSB first
	LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_LSBFIRST); // Not needed

	// Configure Clock signal format as: Phase 2 edges, Polarity Low, Last Bit Clock output enabled
	// Sets the CPHA [9], CPOL [10], and LBCL [8] bits in USART_CR2 register.
	LL_USART_ConfigClock(USART1, LL_USART_PHASE_2EDGE, LL_USART_POLARITY_LOW, LL_USART_LASTCLKPULSE_OUTPUT);

	// Configure USART BRR register for achieving expected Baud Rate value.
	const uint32_t oversampling_mode = LL_USART_OVERSAMPLING_16; // Define
	uint32_t periph_clk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE);
	const uint32_t baudrate = 115200;
	// Compute and set USARTDIV value in BRR Register (full BRR content) according to used Peripheral Clock, Oversampling
	// mode, and expected Baud Rate values
	LL_USART_SetBaudRate(USART1, periph_clk, oversampling_mode, baudrate);

	// Enable Driver Enable (DE) mode.
	// Instead of toggling the DEM bit in the USART_CR3 register (LL_USART_DisableDEMode and LL_USART_EnableDEMode),
	// the RS485 driver will enabled/disabled by the defined commands RS485_TxEnable() and RS485_TxDisable().
	// ... FIXME, can the LL_USART_EnableDEMode() command below be removed?
	LL_USART_EnableDEMode(USART1);

	// Select Driver Enable Polarity. Setting the DEP bit in the USART_CR3 register.
	LL_USART_SetDESignalPolarity(USART1, LL_USART_DE_POLARITY_HIGH);

	// Set DEAT (Driver Enable Assertion Time) bits in USART_CR1 register. Time value expressed on 5 bits ([4:0] bits).
	// This 5-bit value defines the time between the activation of the DE (Driver Enable) signal and
	// the beginning of the start bit.
	// ... FIXME, can this be removed since LL_USART_DisableDEMode and LL_USART_EnableDEMode aren't used?
	LL_USART_SetDEAssertionTime(USART1, 31u); // 31 = 0001 1111

	// Set DEDT (Driver Enable De-assertion Time) bits in USART_CR1 register. This 5-bit value defines the time between
	// the end of the last stop bit, in a transmitted message, and the de-activation of the DE (Driver Enable) signal.
	// ... FIXME, can this be removed since LL_USART_DisableDEMode and LL_USART_EnableDEMode aren't used?
	LL_USART_SetDEDeassertionTime(USART1, 31u);

	//
	LL_USART_Enable(USART1);

	// Polling USART initialization
	uint16_t nbrloops = 0;
	while (1u) {
		uint32_t teack = LL_USART_IsActiveFlag_TEACK(USART1); // Check if the USART Transmit Enable Acknowledge Flag is set or not.
		uint32_t reack = LL_USART_IsActiveFlag_REACK(USART1); // Check if the USART Receive Enable Acknowledge Flag is set or not.
		nbrloops++;
		if ((teack == 0u) || (reack == 0u)) {

		} else {
			// Both are NOT zero
			break;
		}
		if (nbrloops > 10000) {
			DBG_LED_On(); // Debug LED lit up to inform that something is wrong.
			//_NOP();
		}
	}
  /* USER CODE END USART1_Init 0 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  // Initialize TXDE pin
  RS485_TxDisable();
  GPIO_InitStruct.Pin = RS485_TXDE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RS485_TXDE_GPIO_Port, &GPIO_InitStruct);

  // Initialize DEBUG LED
  DBG_LED_Off();
  GPIO_InitStruct.Pin = DBG_LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DBG_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

