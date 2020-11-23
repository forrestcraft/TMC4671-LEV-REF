/*
 * LedRear.c
 *
 *  Created on: 26.03.2019
 *      Author: MO
 */

#include "LedRear.h"

#define HALF_PWM_COUNT	(u8)5
#define MAX_PWM_COUNT	(u8)(HALF_PWM_COUNT<<1)

u16 pwmCounterRearLed = 0;
u8 rearLedMode = 0;
u16 msCounterRearLed = 0;
u8 m_rearLedBrightness = 0;
u16 blinkTimeRearLed = 0;

void rear_led_init()
{
	tmcm_rearLedOff();
}

void rear_led_resetCounter()
{
	pwmCounterRearLed = 0;
	msCounterRearLed = 0;
}

void rear_led_setMode(u8 LedModeRear)
{
	switch(rearLedMode)
	{
		case REAR_LED_MODE_OFF:
		case REAR_LED_MODE_ON:
		case REAR_LED_MODE_BLINK:
		case REAR_LED_MODE_BRIGHTNESS:
			rearLedMode = LedModeRear;
			break;
		default:
			rearLedMode = REAR_LED_MODE_OFF;
	}

	// reset timer on mode switch, to get clean switch over
	if (LedModeRear != rearLedMode)
		rear_led_resetCounter();
}

void rear_led_update(u8 duty, u8 status)
{
	if (duty < status)
	{
		tmcm_rearLedOff();
	}
	else
	{
		tmcm_rearLedOn();
	}
}

void rear_led_setBlinkTime(u16 time)
{
	blinkTimeRearLed = time;
}

void rear_led_brightness(u8 led_Brightness)
{
	m_rearLedBrightness = led_Brightness;
}

void rear_led_off()
{
	tmcm_rearLedOff();
}

void rear_led_periodicJob()
{
	switch(rearLedMode)
	{
		case REAR_LED_MODE_OFF:
			rear_led_off();
			break;
		case REAR_LED_MODE_ON:
			rear_led_update(pwmCounterRearLed, 0);
			break;
		case REAR_LED_MODE_BLINK:
			(msCounterRearLed < (blinkTimeRearLed>>1)) ? rear_led_update(pwmCounterRearLed, 0) : rear_led_off();
			break;
		case REAR_LED_MODE_BRIGHTNESS:
			rear_led_update(pwmCounterRearLed, m_rearLedBrightness);
			break;
	}

	// counter for pwm duty cycle (frequency = 1kHz / MAX_PWM_COUNT)
	pwmCounterRearLed++;
	if (pwmCounterRearLed >= MAX_PWM_COUNT)
		pwmCounterRearLed = 0;

	// millisecond counter for blinking
	msCounterRearLed++;
	if (msCounterRearLed >= blinkTimeRearLed)
		msCounterRearLed = 0;
}
