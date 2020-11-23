/*
 * Led.h
 *
 *  Created on: 22.03.2019
 *      Author: ED
 */
#ifndef LED_H
#define LED_H

	#define LED_MODE_OFF	0
	#define LED_MODE_ON		1
	#define LED_MODE_BLINK	2
	#define LED_MODE_FADE	3
	#define LED_MODE_SWIPE	4

	#define LED_COLOR_GREEN 0

	#define HALF_PWM_COUNT	(u8)5
	#define MAX_PWM_COUNT	(u8)(HALF_PWM_COUNT<<1)
	#define MAX_COLOR		(u8)(MAX_PWM_COUNT-1)

	#include "Definitions.h"

	void led_init();
	void led_setMode(u8 mode);
	void led_setColor(u8 color);
	void led_setBlinkTime(u16 time);
	void led_periodicJob();

#endif /* LED_H */
