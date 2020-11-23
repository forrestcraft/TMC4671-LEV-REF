/*
 * Led.c
 *
 *  Created on: 22.03.2019
 *      Author: ED
 */

#include "Led.h"
#include "hal/system/SysTick.h"

u16 pwmCounter = 0;
u8 ledMode = 0;
u8 ledColor = 0;
u16 msCounter = 0;
u16 swipeColor = 0;
bool swipeDirection = 0;
u16 blinkTime = 1000;

void led_init()
{
	tmcm_ledGreenOff();
	tmcm_ledRedOff();
}

void led_resetCounter()
{
	pwmCounter = 0;
	msCounter = 0;
	swipeColor = LED_COLOR_GREEN;
}

void led_setMode(u8 mode)
{
	u8 actualMode = ledMode;

	switch(mode)
	{
		case LED_MODE_OFF:
		case LED_MODE_ON:
		case LED_MODE_BLINK:
		case LED_MODE_FADE:
		case LED_MODE_SWIPE:
			ledMode = mode;
			break;
		default:
			ledMode = LED_MODE_OFF;
	}

	// reset timer on mode switch, to get clean switch over
	if (actualMode != mode)
		led_resetCounter();
}

void led_setColor(u8 color)
{
	if (color <= MAX_COLOR)
		ledColor = color;
	else
		ledColor = LED_COLOR_GREEN;
}

void led_updateColor(u8 duty, u8 color)
{
	if (duty < color)
	{
		tmcm_ledGreenOn();
		tmcm_ledRedOff();
	}
	else
	{
		tmcm_ledGreenOff();
		tmcm_ledRedOn();
	}
}

void led_setBlinkTime(u16 time)
{
	blinkTime = time;
}

void led_off()
{
	tmcm_ledGreenOff();
	tmcm_ledRedOff();
}

void led_periodicJob()
{
	switch(ledMode)
	{
		case LED_MODE_OFF:
			led_off();
			break;
		case LED_MODE_ON:
			led_updateColor(pwmCounter, ledColor);
			break;
		case LED_MODE_BLINK:
			(msCounter < (blinkTime>>1)) ? led_updateColor(pwmCounter, ledColor) : led_off();
			break;
		case LED_MODE_FADE:
			// change color every blinking time
			if (msCounter == 0)
			{
				swipeColor++;
				if (swipeColor >= MAX_PWM_COUNT)
					swipeColor = 0;
			}
			led_updateColor(pwmCounter, swipeColor);
			break;
		case LED_MODE_SWIPE:
			// change color every blinking time
			if (msCounter == 0)
			{
				swipeColor++;
				if (swipeColor >= MAX_PWM_COUNT)
				{
					swipeColor = 0;
					swipeDirection = !swipeDirection;
				}
			}
			// set actual color
			(swipeDirection) ? led_updateColor(pwmCounter, swipeColor) : led_updateColor(pwmCounter, MAX_COLOR-swipeColor);
			break;
	}

	// counter for pwm duty cycle (frequncy = 1kHz / MAX_PWM_COUNT)
	pwmCounter++;
	if (pwmCounter >= MAX_PWM_COUNT)
		pwmCounter = 0;

	// millisecond counter for blinking
	msCounter++;
	if (msCounter >= blinkTime)
		msCounter=0;
}
