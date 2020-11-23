/*
 * SysTick.c
 *
 *  Created on: 01.04.2019
 *      Author: OK / ED
 */

#include "SysTick.h"

static volatile uint32_t sysTickTimer = 0;
static volatile uint8_t sysTickDivFlag = 0;

#ifdef USE_UART_INTERFACE
	#include "../comm/UART.h"
	extern volatile uint8_t UARTTimeoutFlag;
	extern volatile uint32_t UARTTimeoutTimer;
#endif

/* handler for SysTick interrupt */
void SysTickHandler(void)
{
	if(sysTickDivFlag)
	{
		// increment 1ms timer for systick_getTimer()
		sysTickTimer++;

#ifdef USE_UART_INTERFACE

		// decrease UART receive timeout
		if(UARTTimeoutTimer > 0)
		{
			UARTTimeoutTimer--;
			if(UARTTimeoutTimer == 0)
				UARTTimeoutFlag = true;
		}
#endif

		sysTickDivFlag = 0;
	}
	else
	{
		sysTickDivFlag = 1;
	}
}

/* initialize SysTick timer */
void systick_init()
{
	/* Select AHB clock(HCLK) as SysTick clock source */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	/* SysTick interrupt each 500usec with Core clock equal to 72MHz */
	SysTick_SetReload(36000);
	/* Enable SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Enable);
	NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 3, 0);
	/* Enable SysTick interrupt */
	SysTick_ITConfig(ENABLE);
}

/* get system timer in [ms] */
uint32_t systick_getTimer()
{
	return sysTickTimer;
}

void wait(uint16_t time)
{
	uint32_t startTime = systick_getTimer();
	while ((systick_getTimer()-startTime) < time){;}
}
