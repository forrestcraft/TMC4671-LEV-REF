/*
 * UART.h
 *
 *  Created on: 04.06.2019
 *      Author: ED
 */

#ifndef UART_H
#define UART_H

	#include "../Hal_Definitions.h"

#if defined(USE_UART_INTERFACE)

	void uart_init(uint32_t baudRate);
	void uart_write(char ch);
	char uart_read(char *ch);
	uint32_t uart_checkTimeout();

#endif

#endif
