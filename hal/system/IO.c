/*
 * IO.c
 *
 *  Created on: 01.04.2019
 *      Author: OK / ED
 */

#include "IO.h"

/* reset cpu with or without peripherals */
void io_resetCPU(uint8_t resetPeripherals)
{
	if(resetPeripherals)
		NVIC_GenerateSystemReset();
	else
		NVIC_GenerateCoreReset();
}

/* enable interrupts globally */
void io_enableInterrupts(void)
{
	__asm__ volatile("CPSIE I\n");
}

/* disable interrupts globally */
void io_disableInterrupts(void)
{
	__asm__ volatile("CPSID I\n");
}
