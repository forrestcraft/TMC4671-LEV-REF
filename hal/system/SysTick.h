/*
 * SysTick.h
 *
 *  Created on: 01.04.2019
 *      Author: OK / ED
 */

#ifndef SYS_TICK_H
#define SYS_TICK_H

	#include "../Hal_Definitions.h"

	void systick_init();
	uint32_t systick_getTimer();
	void wait(uint16_t time);

#endif
