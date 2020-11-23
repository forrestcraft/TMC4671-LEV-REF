/*
 * Button.c
 *
 *  Created on: 26.03.2019
 *      Author: MO
 */

#include "Button.h"
#include "Led.h"
#include "LedRear.h"
#include "Sensor.h"
#include "hal/Debug.h"
#include "hal/Flags.h"
#include "BLDC.h"

	s16 pressTime = 0;
	s8 m_buttonCounter = 0;
	s8 m_buttonStateLast = 0;
	s8 m_buttonState = 0;
	s8 m_autoState = 0;
	s8 m_bikeState = 0;

	s32 m_deactiveTimer = 0;
	s8 m_autoDeactive = 0;

	u8 m_batteryStatusFactor = 0;
	s32 m_savingTimer = 0;
	s8 bikePWMState = 0;
	s32 m_pedalCounterLast = 0;

	// forward declarations
	void updateButtonCounter();
	void enableBike();
	void disableBike();
	void batteryCheck();

void button_periodicJob()
{
	static u8 lastBikeState = 2;

	updateButtonCounter();

	switch(m_bikeState)
	{
		case BIKE_DISABLED:
			disableBike();
			if (lastBikeState != m_bikeState)
			{
				rear_led_setMode(REAR_LED_MODE_OFF);
				tmcm_frontLedOff();
				tmcm_disablePower();
			}
			break;
		case BIKE_ENABLED:
			if (tmcm_getPowerState() != 1)
			{
				tmcm_enablePower();
			}
			if (lastBikeState != m_bikeState)
			{
				//front/rear Light enabled
				tmcm_frontLedOn();
				tmcm_enableDriver();
				tmcm_updateConfig();
				enableBike();
			}
			batteryCheck();
			break;
	}

	lastBikeState = m_bikeState;
}

void updateButtonCounter()
{
	u8 m_buttonPositionActual = tmcm_getButtonValue();
	if (m_buttonPositionActual == 0)
	{
		pressTime++;
		if(pressTime == 100)
		{
			m_buttonCounter++;
		}
		// enable Power
//		if(pressTime == 500)
//		{
//			tmcm_enablePower();
//		}
	}
	else
	{
		pressTime = 0;
	}

	if(m_buttonCounter == 100)
	{
		m_buttonCounter = 0;
	}
	m_buttonState = m_buttonCounter % 2;

	// auto Bike enable
	if((m_bikeState == BIKE_DISABLED) && (m_buttonState == BIKE_ENABLED))
	{
		if(tmcm_getInterruptValue() == 1)
		{
			m_autoState = BIKE_ENABLED;
		}
	}

	// auto Bike disable
	if(m_bikeState == BIKE_ENABLED)
	{
		if(m_autoDeactive == 1)
		{
			m_autoState = BIKE_DISABLED;
		}
	}

	if((m_autoState == BIKE_DISABLED)&&(m_buttonState != m_buttonStateLast))
	{
		m_buttonCounter = BIKE_ENABLED;
		m_buttonState = BIKE_ENABLED;
		m_autoState = BIKE_ENABLED;
	}

	if(m_buttonState == BIKE_ENABLED)				// auto Bike handling
	{
		m_bikeState = m_autoState;
	}
	else if(m_buttonStateLast != m_buttonState)		// button handling
	{
		m_bikeState = m_buttonState;
	}

	m_buttonStateLast = m_buttonState;

	debug_setTestVar0(m_bikeState);
}

void enableBike()
{
	if(COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST != motorConfig.commutationMode)
	{
		motorConfig.commutationMode = COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST;
	}
	m_deactiveTimer = 0;
}

void disableBike()
{
	motorConfig.commutationMode = COMM_MODE_FOC_DISABLED;

	// hold module in stop mode to suppress regulation
	bldc_switchToMotionMode(STOP_MODE);
	tmcm_disableDriver();
	led_setMode(LED_MODE_OFF);
}

void button_autoDeactivation()
{
	if(	motorConfig.commutationMode != COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST)
	{
		m_deactiveTimer = 0;
	}

	// tracking time before Bike get deactivated
	if((sensor_wheelVelocity() == 0) && (m_bikeState == BIKE_ENABLED))
	{
		m_deactiveTimer ++;
	}
	else
	{
		m_deactiveTimer = 0;
	}

	// trigger to deactivate the bike
	if((m_deactiveTimer >= DEACTIVATE_TIME_IN_SECONDS) && (m_bikeState == BIKE_ENABLED))
	{
		m_autoDeactive = 1;
	}
	else
	{
		m_autoDeactive = 0;
	}
}

void button_updateBatteryStatusFactor()
{
	m_batteryStatusFactor = (motorConfig.maxBatteryVoltage - motorConfig.cutOffVoltage)/MAX_COLOR;
}

void batteryCheck()
{
	s32 actualRampTargetTorque = bldc_rampTargetTorque();
	if(actualRampTargetTorque <= motorConfig.minimumMotorCurrent)
	{
		s32 actualSupplyVoltage = bldc_getSupplyVoltage();

		if(actualSupplyVoltage <= motorConfig.cutOffVoltage )
		{
			motorConfig.commutationMode = COMM_MODE_FOC_DISABLED;
			led_setMode(LED_MODE_BLINK);
			led_setColor(0);
			led_setBlinkTime(1000);
		}
		else if(actualSupplyVoltage-motorConfig.minBatteryVoltage >= 0)
		{
			s8 batteryStatus = actualSupplyVoltage-motorConfig.cutOffVoltage;
			batteryStatus /= m_batteryStatusFactor;
			if(batteryStatus <= 0)
			{
				batteryStatus = 0;
			}
			if(actualSupplyVoltage >= motorConfig.maxBatteryVoltage)
			{
				batteryStatus = 9;
			}
			led_setMode(LED_MODE_ON);
			led_setColor(batteryStatus);
		}
	}
}

s32 button_batterySaving(s32 m_targetTorque)
{
	s32 m_pedalCounter = sensor_pedalCounter();

	// tracking time before bike get deactivated
	if((m_bikeState == BIKE_ENABLED) && (m_targetTorque <= 0))
	{
		m_savingTimer ++;
	}
	else
	{
		m_savingTimer = 0;
	}

	// trigger to deactivate the bike
	if(m_savingTimer == (s32)motorConfig.batterySavingTimer*1000)
	{
		bldc_setTargetMotorCurrent(0);
		tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_SV_CHOP, 0x00000000);
		tmc4671_setTorqueFluxPI(DEFAULT_MC, motorConfig.pidTorque_P_param, 0);
		bikePWMState = 0;
		m_targetTorque = 0;
	}

	if(bikePWMState == 0)
	{
		if ((m_pedalCounter > m_pedalCounterLast))
		{
			bldc_setTargetMotorCurrent(0);
			tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_SV_CHOP, 0x00000007);
			tmc4671_setTorqueFluxPI(DEFAULT_MC, motorConfig.pidTorque_P_param, motorConfig.pidTorque_I_param);
			bikePWMState = 1;
			m_savingTimer = 0;
		}
	}

	m_pedalCounterLast = m_pedalCounter;

	return m_targetTorque;
}

void button_resetSavingTimer()
{
	m_savingTimer = 0;
}
