/*
 * Button.h
 *
 *  Created on: 26.03.2019
 *      Author: MO
 */
#ifndef BUTTON_H
#define BUTTON_H

	#define BIKE_DISABLED 				1
	#define BIKE_ENABLED				0
	#define DEACTIVATE_TIME_IN_SECONDS 	180 //30

	#include "Definitions.h"

	void button_updateBatteryStatusFactor();
	void button_periodicJob();
	void button_autoDeactivation();
	s32 button_batterySaving(s32 m_targetTorque);
	void button_resetSavingTimer();

#endif /* BUTTON_H */
