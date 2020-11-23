/*
 * LedRear.h
 *
 *  Created on: 26.03.2019
 *      Author: MO
 */
#ifndef LED_REAR_H
#define LED_REAR_H

	#define REAR_LED_MODE_OFF			0
	#define REAR_LED_MODE_ON			1
	#define REAR_LED_MODE_BLINK			2
	#define REAR_LED_MODE_BRIGHTNESS	3

	#include "Definitions.h"

	void rear_led_init();
	void rear_led_setMode(u8 LedModeRear);
	void rear_led_setBlinkTime(u16 time);
	void rear_led_brightness(u8 led_Brightness);
	void rear_led_periodicJob();

#endif /* LED_REAR_H */
