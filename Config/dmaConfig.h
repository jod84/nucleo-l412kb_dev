/*
 * dmaConfig.h
 *
 *  Created on: May 17, 2023
 *      Author: dan84
 */

#ifndef DMACONFIG_H_
#define DMACONFIG_H_

#include "stm32l412xx.h"        // Included for references to DMAx and IRQn:s
#include "stm32l4xx_ll_bus.h"   // Included for reference to peripheral clock
#include "stm32l4xx_ll_dma.h"   // Included for references to DMA channels

// DMA configuration for UART
#define UART_DMA				DMA1
#define UART_DMA_REQUEST		LL_DMA_REQUEST_2
#define UART_DMA_PCLOCK			LL_AHB1_GRP1_PERIPH_DMA1	// RCC_AHB1ENR_DMAxEN, x=1

#define UART_TX_DMA_CHANNEL  	LL_DMA_CHANNEL_4
#define UART_TX_DMA_IRQ			DMA1_Channel4_IRQn

#define UART_RX_DMA_CHANNEL  	LL_DMA_CHANNEL_5
#define UART_RX_DMA_IRQ			DMA1_Channel5_IRQn



#endif /* DMACONFIG_H_ */
