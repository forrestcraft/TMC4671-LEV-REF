/*
 * IO.h
 *
 *  Created on: 01.04.2019
 *      Author: ED
 */

#ifndef IO_H
#define IO_H

	#include "../Hal_Definitions.h"

	void io_resetCPU(uint8_t resetPeripherals);
	void io_enableInterrupts();
	void io_disableInterrupts();

#endif
