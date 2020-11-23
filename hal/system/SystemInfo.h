/*
 * SystemInfo.h
 *
 *  Created on: 04.04.2019
 *      Author: ED
 */

#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

	#include "../Hal_Definitions.h"

	void systemInfo_update(uint32_t actualSystick);

	void systemInfo_incMainLoopCounter();
	uint32_t systemInfo_getMainLoopsPerSecond();

	void systemInfo_incCommunicationLoopCounter();
	uint32_t systemInfo_getCommunicationsPerSecond();

	void systemInfo_incCurrentLoopCounter();
	uint32_t systemInfo_getCurrentLoopsPerSecond();

	void systemInfo_incVelocityLoopCounter();
	uint32_t systemInfo_getVelocityLoopsPerSecond();

#endif /* SYSTEM_INFO_H */
