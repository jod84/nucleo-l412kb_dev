/*
 * uart.h
 *
 *  Created on: 16 May 2023
 *      Author: dan84
 */

#ifndef UART_H_
#define UART_H_

#include "stm32l412xx.h"        // Included for references to USARTx
#include "stm32l4xx_ll_bus.h"   // Included for reference to peripheral clock
#include "stm32l4xx_ll_rcc.h"   // Included for references to clock source

#define UART_RS485				USART1
#define UART_RS485_IRQn			USART1_IRQn
#define UART_RS485_PCLOCK		LL_APB2_GRP1_PERIPH_USART1
#define UART_RS485_CLK_SRC		LL_RCC_USART1_CLKSOURCE_SYSCLK



void uartInit(void);

#endif /* UART_H_ */
