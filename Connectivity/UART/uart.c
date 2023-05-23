/*
 * uart.c
 *
 *  Created on: 16 May 2023
 *      Author: dan84
 */
#include "main.h"
#include "priorities.h"
#include "uart.h"
#include "dmaConfig.h"
#include "stm32l4xx_ll_usart.h"		// Already included in uart.h, but for simplicity...
#include "stm32l4xx_ll_dma.h"		// Already included in dmaConfig.h, but for simplicity...
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"		// Already included in both uart.h and dmaConfig.h, but for simplicity...
#include "stm32l4xx_ll_rcc.h"		// Already included in uart.h, but for simplicity...
#include "stdint.h"

extern uint8_t buffer[75];

static void set_uart_dma_addresses() {

	LL_USART_DisableDMAReq_RX(USART1);
	LL_DMA_SetPeriphAddress(DMA1, UART_RX_DMA_CHANNEL, (uint32_t) &(USART1->RDR));
	LL_DMA_SetMemoryAddress(DMA1, UART_RX_DMA_CHANNEL, buffer);
	uint8_t msgSize = sizeof(buffer)/sizeof(uint8_t);
	LL_DMA_SetDataLength(DMA1, UART_RX_DMA_CHANNEL, msgSize);
	LL_USART_EnableDMAReq_RX(USART1);
	LL_DMA_EnableChannel(DMA1, UART_RX_DMA_CHANNEL);
}

static void setup_uart_dma(uint32_t dmaChannel, IRQn_Type dmaChannelIRQ, uint8_t  priority) {

	// Enable Transfer Complete and Transfer Error interrupts
	LL_DMA_EnableIT_TC(UART_DMA, dmaChannel);
	LL_DMA_EnableIT_TE(UART_DMA, dmaChannel);

	NVIC_SetPriority(dmaChannelIRQ, priority);
	NVIC_EnableIRQ(dmaChannelIRQ);

	LL_DMA_ConfigTransfer(UART_DMA, dmaChannel,
				LL_DMA_DIRECTION_MEMORY_TO_PERIPH 	|
				LL_DMA_PRIORITY_HIGH 				|
				LL_DMA_MODE_NORMAL 					|
				LL_DMA_PERIPH_NOINCREMENT 			|
				LL_DMA_MEMORY_INCREMENT 			|
				LL_DMA_PDATAALIGN_BYTE 				|
				LL_DMA_MDATAALIGN_BYTE);

	LL_DMA_SetPeriphRequest(UART_DMA, dmaChannel, UART_DMA_REQUEST);

	/*
	// Setup for TX
	LL_DMA_EnableIT_TC(UART_DMA, UART_TX_DMA_CHANNEL);
	LL_DMA_EnableIT_TE(UART_DMA, UART_TX_DMA_CHANNEL);

	NVIC_SetPriority(UART_TX_DMA_IRQ, PRIORITY_INTERRUPT_UART_TX);
	NVIC_EnableIRQ(UART_TX_DMA_IRQ);

	LL_DMA_ConfigTransfer(UART_DMA, UART_TX_DMA_CHANNEL,
				LL_DMA_DIRECTION_MEMORY_TO_PERIPH 	|
				LL_DMA_PRIORITY_HIGH 				|
				LL_DMA_MODE_NORMAL 					|
				LL_DMA_PERIPH_NOINCREMENT 			|
				LL_DMA_MEMORY_INCREMENT 			|
				LL_DMA_PDATAALIGN_BYTE 				|
				LL_DMA_MDATAALIGN_BYTE);


	LL_DMA_SetPeriphRequest(UART_DMA, UART_TX_DMA_CHANNEL, UART_DMA_REQUEST);


	// Setup for RX
	LL_DMA_EnableIT_TC(UART_DMA, UART_RX_DMA_CHANNEL);
	LL_DMA_EnableIT_TE(UART_DMA, UART_RX_DMA_CHANNEL);

	NVIC_SetPriority(UART_RX_DMA_IRQ, PRIORITY_INTERRUPT_UART_RX);
	NVIC_EnableIRQ(UART_RX_DMA_IRQ);

	LL_DMA_ConfigTransfer(UART_DMA, UART_RX_DMA_CHANNEL,
				LL_DMA_DIRECTION_MEMORY_TO_PERIPH 	|
				LL_DMA_PRIORITY_HIGH 				|
				LL_DMA_MODE_NORMAL 					|
				LL_DMA_PERIPH_NOINCREMENT 			|
				LL_DMA_MEMORY_INCREMENT 			|
				LL_DMA_PDATAALIGN_BYTE 				|
				LL_DMA_MDATAALIGN_BYTE);


	LL_DMA_SetPeriphRequest(UART_DMA, UART_RX_DMA_CHANNEL, UART_DMA_REQUEST);//*/
}


void uartInit(void) {
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
	NVIC_SetPriority(USART1_IRQn, PRIORITY_INTERRUPT_UART_RX);
	NVIC_EnableIRQ(USART1_IRQn);

	LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);

	// TX/RX direction
	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

	// 8 data bit, 1 start bit, 1 stop bit, no parity
	// Set the PS, PCE, M1, and M2 bits in USART_CR1 register and the STOP bit in USART_CR2 register
	LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B,
			LL_USART_PARITY_NONE, LL_USART_STOPBITS_1); // Not needed since default values

	// Configure transfer bit order : MSB first
	LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_LSBFIRST); // Not needed

	// Configure Clock signal format as: Phase 2 edges, Polarity Low, Last Bit Clock output enabled
	// Sets the CPHA [9], CPOL [10], and LBCL [8] bits in USART_CR2 register.
	LL_USART_ConfigClock(USART1, LL_USART_PHASE_2EDGE, LL_USART_POLARITY_LOW,
			LL_USART_LASTCLKPULSE_OUTPUT);

	// Configure USART BRR register for achieving expected Baud Rate value.
	const uint32_t oversampling_mode = LL_USART_OVERSAMPLING_16; // Define
	uint32_t periph_clk = LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE);
	const uint32_t baudrate = 115200;
	// Compute and set USARTDIV value in BRR Register (full BRR content) according to used Peripheral Clock, Oversampling
	// mode, and expected Baud Rate values
	LL_USART_SetBaudRate(USART1, periph_clk, oversampling_mode, baudrate);


	// DRIVER ENABLE MODE start ***************************************************************************************

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

	// DRIVER ENABLE MODE end ***************************************************************************************

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


	LL_USART_ClearFlag_IDLE(USART1);
	LL_USART_EnableIT_IDLE(USART1);
	LL_USART_EnableIT_RXNE(USART1);


	// #SETUP UART DMA
	//----------------------------------------------------------------------------

	// Check if the DMA peripheral clock is enabled. If not, enable the clock.
	if (!LL_AHB1_GRP1_IsEnabledClock(UART_DMA_PCLOCK)) {
		LL_AHB1_GRP1_EnableClock(UART_DMA_PCLOCK);
	}

	// Setup DMA for UART tx
	setup_uart_dma(UART_TX_DMA_CHANNEL, UART_TX_DMA_IRQ, PRIORITY_INTERRUPT_UART_TX);
	// Setup DMA for UART rx
	setup_uart_dma(UART_RX_DMA_CHANNEL, UART_RX_DMA_IRQ, PRIORITY_INTERRUPT_UART_RX);

	set_uart_dma_addresses();

	// #CLEAR UART



}
