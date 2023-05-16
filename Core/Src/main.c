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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#include <stdio.h>
#include <string.h>
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buffer[75] = {0};
static uint8_t msgLen = 2;
static void uart_transmitData(void);
static void configureUartDma(const uint32_t dmaDir);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */

  //const char payload[8] = "Kamin\r\n";
  //const char payload[6] = "Kamin";

  //const uint8_t payload[8] = {0x4b, 0x61, 0x6d, 0x69, 0x6e, 0x0D, 0x0A};


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //RS485_TxEnable();
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  buffer[0] = 0x0A;
  buffer[1] = 0x0D;
  uart_transmitData();
  const char welcome[54] = "\tPlease type a string with a length less than 255.\n\n\r";
  memcpy(buffer, welcome, 53);
  msgLen = 53;
  uart_transmitData();
  buffer[0] = 0x0D;
  buffer[1] = 0x0A;
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
	//USER CODE END USART1_Init 0

	// USART1 DMA Init

	// USART1_RX Init
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMA_REQUEST_2);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5,
				LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5,
				LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5,
				LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

	// USART1_TX Init
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMA_REQUEST_2);

	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4,
				LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4,
				LL_DMA_PRIORITY_LOW);

	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4,
				LL_DMA_PERIPH_NOINCREMENT);

	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4,
				LL_DMA_MEMORY_INCREMENT);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);//*/

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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

  /**/
  LL_GPIO_ResetOutputPin(RS485_TXDE_GPIO_Port, RS485_TXDE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(DBG_LED_GPIO_Port, DBG_LED_Pin);

  /**/
  GPIO_InitStruct.Pin = RS485_TXDE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(RS485_TXDE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = DBG_LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(DBG_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void uart_rx_callback(void)
{
	uint8_t received_char;
	received_char = LL_USART_ReceiveData8(USART1);
	if (received_char == 0x0D) { // if carriage return CR character received, finalize string and send back.
		buffer[msgLen++] = 0x0A;
		buffer[msgLen++] = received_char;
		buffer[msgLen++] = received_char;
		uart_transmitData(); // send back string
		//msgLen = 2;
	}
	else {
		buffer[msgLen++] = received_char;
	}
}

void uart_transmitData(void) {

	LL_USART_DisableIT_RXNE(USART1);
	RS485_TxEnable();
	uint8_t pos = 0;
	while (pos < msgLen) {
		const uint8_t tmp = (uint8_t)buffer[pos++];
	    LL_USART_TransmitData8(USART1, tmp);
	    while(!LL_USART_IsActiveFlag_TXE(USART1)); // Wait until the data in TDR has been loaded to the TX shift-register.
	}
	// To ensure that the transmit of the last bit has been finalized i.e., that the transmit from
	// tx shift-register to tx pin is complete, await the Transmit Complete to be set.
	while(!LL_USART_IsActiveFlag_TC(USART1));
	RS485_TxDisable();
	LL_USART_EnableIT_RXNE(USART1);
	msgLen = 2;

}

void configureUartDma(const uint32_t dmaDir) {

}
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

