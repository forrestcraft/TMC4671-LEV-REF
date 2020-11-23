/*
 * Sensor.c
 *
 *  Created on: 21.02.2019
 *      Author: MO
 */

#include "Button.h"
#include "Sensor.h"
#include "LedRear.h"
#include "BLDC.h"
#include "hal/Debug.h"
#include "hal/ic/LIS2DH12.h"

	u32 m_cutOffTime = 0;

	// filter for torque values
	int64_t akkuActualPedalTorque = 0;
	s32 filteredActualPedalTorque = 0;
	s32 fltActualTorque = 4;

	int64_t akkuPedalTorqueAverage = 0;
	s32 m_filteredPedalTorqueAverage = 0;

	s32 m_filteredActualPedalVelocity = 0;

	int64_t akkuActualPedalVelocitySwift = 0;
	s32 m_filteredActualPedalVelocitySwift = 0;

	int64_t akkuActualMotorVelocityFast = 0;
	int64_t akkuActualWheelMotorVelocityFast = 0;

	// min / max pedal torque
	float m_minPedalTorque = 0;
	float m_maxPedalTorque = 0;
	float dT = 0.06;

	// wheel velocity measurement
	u8 m_wheelDirectionCounter = 0;
	u8 m_wheelCounterPrev = 0;
	s32 m_wheelVelocityTimer = 0;
	s32 m_wheelVelocity = 0;

	// wheelMotor velocity measurement
	s32 m_wheelMotorVelocity = 0;
	s32 m_actualWheelMotorVelocity = 0;

	// pedal Counter
	s32 m_pedalCounter = 0;
	s8 m_pedalDirectionCounter = 0;
	s32 m_pedalPosition = 0;
	s32 pedalCounterPrev = 0;
	s32 pedalCounterLast = 0;
	s32 m_pedalCountsPer30MSecond = 0;

	// min / max average
	u16 maxPedalData[MAX_PEDAL_POSITIONS];
	u16 minPedalData[MAX_PEDAL_POSITIONS];
	u32 maxPedalAverage = 0;
	u32 minPedalAverage = 0;

	// torque at position
	u16 m_pedalTorqueAverage[MAX_PEDAL_POSITIONS];
	u16 m_leftPedalTorqueRaw[MAX_PEDAL_POSITIONS];
	u16 m_rightPedalTorqueRaw[MAX_PEDAL_POSITIONS];
	u16 m_minEstimatedPedalTorque[MAX_PEDAL_POSITIONS];
	u16 m_leftPedalTorque = 0;
	u16 m_rightPedalTorque = 0;
	s32 targetPedalTorque = 0;

	// torque/speed map
	s32 m_mapSpeedTorque = 0;
	u8 cutOffTimer = 0;

	s32 m_actualTorque = 0;
	int64_t akkuActualMotorTorque = 0;

	// config parameter
	s8 m_pedalDirection = 0;
	s8 m_pedalMotorEnable = 0;
	s32 m_averageTorque = 0;
	s32 m_pedalVelocity = 0;
	s32 m_motorVelocity = 0;
	s16 m_actualGain = 0;
	bool m_actualTorqueLimit = 0;

	// brake detection
	s32 out_Y = 0;
	s32 minGDetection = 0;
	s32 minGDetectionLast = 0;
	s32 m_wheelVelocityLast = 0;
	s32 m_brakeStartPoint = 0;

	// forward declarations
	void cutOffTime();
	int wheelSensorDiff(u8 previousState, u8 actualState);
	int bottomBracketDiff(u8 previousState, u8 actualState);

void sensor_init()
{
	sensor_updateCutOffTime();
	button_updateBatteryStatusFactor();

	s32 ADCTorqueValue = tmcm_getADCTorqueValue();
	for (int i = 0; i < motorConfig.pedalPulsesPerRotation; i++)
	{
		maxPedalData[i] = ADCTorqueValue;
		minPedalData[i] = ADCTorqueValue;

		m_leftPedalTorqueRaw[i] = 0;
		m_rightPedalTorqueRaw[i] = 0;
		m_minEstimatedPedalTorque[i] = 0;
		m_pedalTorqueAverage[i] = 0;
	}
}

// ===== analog signal processing =====

void sensor_updatePedalTorque()
{
	filteredActualPedalTorque = tmc_filterPT1(&akkuActualPedalTorque, tmcm_getADCTorqueValue(), filteredActualPedalTorque, fltActualTorque, 16);
}

s32 sensor_actualPedalTorque()
{
	return filteredActualPedalTorque;
}

int sensor_getFltActualTorque()
{
	return fltActualTorque;
}

void sensor_setFltActualTorque(int flt)
{
	fltActualTorque = flt;
}

void sensor_computePedalTorqMinMax()
{
	int actualTorque = filteredActualPedalTorque;

	if (actualTorque > m_maxPedalTorque)
		m_maxPedalTorque = actualTorque;
	else
		m_maxPedalTorque -= dT;

	if (actualTorque < m_minPedalTorque)
		m_minPedalTorque = actualTorque;
	else
		m_minPedalTorque += dT;

	if (abs(m_pedalCountsPer30MSecond) < 2)
	{
		m_maxPedalTorque = actualTorque;
		m_minPedalTorque = actualTorque;
	}
}

// ===== digital signal processing =====

s32 m_pedalPositionLast = 0;
s8 m_pedalPositionStart = 0;
s8 m_pedalPositionStartReset = 0;

void sensor_updatePedalCounter()
{
	// update pedal counter
	u8 pedalCounterActual = tmcm_getSinCosValue();
	m_pedalDirectionCounter = bottomBracketDiff(pedalCounterPrev, pedalCounterActual);

	m_pedalCounter += m_pedalDirectionCounter;
	if (m_pedalCounter < 0)
	{
		m_pedalCounter = 0;
	}

	pedalCounterPrev = pedalCounterActual;

	// update pedal position
	m_pedalPosition = (int)m_pedalCounter % (int)motorConfig.pedalPulsesPerRotation;
	if ((m_pedalPosition < 0) && (m_pedalPosition >= motorConfig.pedalPulsesPerRotation))
	{
		m_pedalPosition = 0;
	}

	if ((m_pedalPosition >= 0) && (m_pedalPosition < motorConfig.pedalPulsesPerRotation))
	{
		// estimate torque for actual position + 180°
		s32 index = (m_pedalPosition + ((int)motorConfig.pedalPulsesPerRotation/2));
		if (index >= motorConfig.pedalPulsesPerRotation)
		{
			index = index - motorConfig.pedalPulsesPerRotation;
		}

		s32 pedalTorque = motorConfig.torqueSensorOffset-filteredActualPedalTorque;
		if (pedalTorque < 0)
			pedalTorque = 0;

		// average measured torque
		m_pedalTorqueAverage[m_pedalPosition] = pedalTorque;

		// clipping PedalTorque
		if((pedalTorque*motorConfig.torqueSensorGain >= (s32)motorConfig.maximumCurrent) && (m_wheelMotorVelocity <= 40) )
		{
			pedalTorque = motorConfig.maximumCurrent/motorConfig.torqueSensorGain;
		}

		// starting position detection
		if(m_pedalPosition == m_pedalPositionLast-1)
		{
			m_pedalPositionStart = m_pedalPosition;

			//setting end of the starting point
			m_pedalPositionStart = ((int)m_pedalPositionStart+((int)motorConfig.pedalPulsesPerRotation/4*3))%(int)motorConfig.pedalPulsesPerRotation;
		}

		// reset start position
		if((m_wheelMotorVelocity == 0) &&(m_wheelVelocity <= (s32)motorConfig.initialRightTorqueSpeed))
		{
			m_pedalPositionStartReset = 0;
		}

		// save raw torque at actual position
		m_leftPedalTorqueRaw[m_pedalPosition] = (pedalTorque*motorConfig.leftRightRatio)/100;

		if((m_pedalPosition != m_pedalPositionStart)&&(m_pedalPositionStartReset == 0))
		{
			m_rightPedalTorqueRaw[m_pedalPosition] = (motorConfig.initialRightTorque*(100-motorConfig.leftRightRatio))/100;
		}
		else
		{
			m_rightPedalTorqueRaw[index] = (pedalTorque*(100-motorConfig.leftRightRatio))/100;
		}

		if (m_filteredActualPedalVelocitySwift <= motorConfig.pedalSenseDelay)
		{
			for (int i = 0; i < motorConfig.pedalPulsesPerRotation; i++)
			{
				s32 index_2 = (i + ((int)motorConfig.pedalPulsesPerRotation/2));
				if (index_2 >= motorConfig.pedalPulsesPerRotation)
				{
					index_2 = index_2 - motorConfig.pedalPulsesPerRotation;
				}

				m_leftPedalTorqueRaw[i] = pedalTorque;
				m_rightPedalTorqueRaw[index_2] = pedalTorque;

				//AB Parameter Average measured Torque
				m_pedalTorqueAverage[i] = pedalTorque;
			}
		}

		// calculate average torque
		u32 m_leftPedalTorqueAverage = 0;
		u32 m_rightPedalTorqueAverage = 0;
		u32 measuredPedalTorque = 0;
		for (int i = 0; i < motorConfig.pedalPulsesPerRotation; i++)
		{
			m_leftPedalTorqueAverage += m_leftPedalTorqueRaw[i];
			m_rightPedalTorqueAverage += m_rightPedalTorqueRaw[i];

			// average measured torque
			measuredPedalTorque += m_pedalTorqueAverage[i];
		}
		m_leftPedalTorqueAverage /= motorConfig.pedalPulsesPerRotation;
		m_rightPedalTorqueAverage /= motorConfig.pedalPulsesPerRotation;

		// adjusting average/sport mode
		m_leftPedalTorque = ((m_leftPedalTorqueAverage*(100-motorConfig.averageSportMode)) + (m_leftPedalTorqueRaw[m_pedalPosition]*motorConfig.averageSportMode))/100;			//PedalTorque = ((PedalTorqueAverage*(100-AverageSportMode)/50) + (PedalTorqueRaw*AverageSportMode)/50)/2;
		m_rightPedalTorque = ((m_rightPedalTorqueAverage*(100-motorConfig.averageSportMode)) + (m_rightPedalTorqueRaw[m_pedalPosition]*motorConfig.averageSportMode))/100;

		//
		if (m_leftPedalTorqueRaw[m_pedalPosition] > m_rightPedalTorqueRaw[m_pedalPosition])
		{
			m_minEstimatedPedalTorque[m_pedalPosition] = m_leftPedalTorque;
		}
		if (m_leftPedalTorqueRaw[m_pedalPosition] < m_rightPedalTorqueRaw[m_pedalPosition])
		{
			m_minEstimatedPedalTorque[m_pedalPosition] = m_rightPedalTorque;
		}
		else if(m_leftPedalTorqueRaw[m_pedalPosition] == m_rightPedalTorqueRaw[m_pedalPosition])
		{
			m_minEstimatedPedalTorque[m_pedalPosition] = MAX(m_leftPedalTorque, m_rightPedalTorque);
		}

		if(motorConfig.averageSportMode <= 50)
		{
			m_filteredPedalTorqueAverage = tmc_filterPT1(&akkuPedalTorqueAverage, m_minEstimatedPedalTorque[m_pedalPosition],m_filteredPedalTorqueAverage, 6, 12);
		}
		else
		{
			m_filteredPedalTorqueAverage = m_minEstimatedPedalTorque[m_pedalPosition];
		}

		m_pedalPositionLast = m_pedalPosition;

		//save start Position
		if(m_pedalPosition == m_pedalPositionStart)
		{
			m_pedalPositionStartReset = 1;
		}

		// average measured torque
		measuredPedalTorque /= motorConfig.pedalPulsesPerRotation;
		m_averageTorque = (measuredPedalTorque)*11355/100000;

//		debug_setTestVar6(m_rightPedalTorqueRaw[m_pedalPosition]);
//		debug_setTestVar5(m_leftPedalTorqueRaw[m_pedalPosition]);
//		debug_setTestVar4(m_rightPedalTorqueAverage);
//		debug_setTestVar3(m_leftPedalTorqueAverage);
	}
}

s32 sensor_pedalCounter()
{
	return m_pedalCounter;
}

s32 sensor_pedalPosition()
{
	return m_pedalPosition;
}

void sensor_updatePedalVelocity()
{
	m_pedalCountsPer30MSecond = m_pedalCounter-pedalCounterLast;
	pedalCounterLast = m_pedalCounter;

	m_pedalVelocity = (m_pedalCountsPer30MSecond*666)/(motorConfig.pedalPulsesPerRotation);	// RPM
	m_filteredActualPedalVelocitySwift = tmc_filterPT1(&akkuActualPedalVelocitySwift, m_pedalVelocity, m_filteredActualPedalVelocitySwift, 1, 1);

	if (m_pedalCountsPer30MSecond > 0)
	{
		m_pedalDirection = 1;
	}
	if (m_pedalCountsPer30MSecond == 0 )
	{
		m_pedalDirection = 0;
	}
	if (m_pedalCountsPer30MSecond < 0)
	{
		m_pedalDirection = -1;
	}
//	debug_setTestVar0(m_filteredActualPedalVelocitySwift);
}

s32 sensor_pedalCounterPer500MSeconds()
{
	return m_pedalCountsPer30MSecond;
}

void sensor_updateMotorVelocity()
{
	s32 actualMotorVelocity = tmc4671_readInt(0, TMC4671_PID_VELOCITY_ACTUAL);
	if(actualMotorVelocity <= 0)
	{
		actualMotorVelocity = 0;
	}
	m_motorVelocity = tmc_filterPT1(&akkuActualMotorVelocityFast, actualMotorVelocity, m_motorVelocity, 2, 4);

	m_actualWheelMotorVelocity = (actualMotorVelocity*motorConfig.wheelDiameter*47)/(motorConfig.gearRatio*25000);

	m_wheelMotorVelocity = tmc_filterPT1(&akkuActualWheelMotorVelocityFast, m_actualWheelMotorVelocity, m_wheelMotorVelocity, 9, 16);
}

void sensor_updateWheelCounter()
{
	u8 m_wheelCounterActual = tmcm_getSpeedValue();
	m_wheelDirectionCounter = wheelSensorDiff(m_wheelCounterPrev, m_wheelCounterActual);
	m_wheelCounterPrev = m_wheelCounterActual;
//	debug_setTestVar7(m_wheelDirectionCounter);
}

void sensor_updateWheelVelocity()
{
	if (m_wheelVelocityTimer < 2500)
	{
		m_wheelVelocityTimer++;
	}
	if (m_wheelVelocityTimer >= 2500)
	{
		m_wheelVelocity = 0;
	}
	if(m_wheelDirectionCounter == 1)
	{
		// Wheel Velocity in 0,1km/h
		m_wheelVelocity = ((s32)113*(s32)motorConfig.wheelDiameter)/(m_wheelVelocityTimer*(s32)motorConfig.wheelPulsesPerRotation);
		m_wheelVelocityTimer = 0;
		m_wheelDirectionCounter = 0;
	}
	debug_setTestVar2(m_wheelVelocity);
}

void sensor_mapSpeedTorque()
{
	s32 m_torqueDiff = 0;
	s32 m_speedDiff = 0;
	s32 m_speedWheelDiff = 0;

	if(motorConfig.speed_0 >= m_wheelMotorVelocity)
	{
		m_mapSpeedTorque = motorConfig.maximumCurrent*motorConfig.torque_0/100;
	}

	if((motorConfig.speed_0 < m_wheelMotorVelocity) && (motorConfig.speed_1 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_0;
		m_torqueDiff = (s16)motorConfig.torque_1 - (s16)motorConfig.torque_0;
		m_speedDiff = (s32)motorConfig.speed_1-(s32)motorConfig.speed_0;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_0 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_1 < m_wheelMotorVelocity) && (motorConfig.speed_2 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_1;
		m_torqueDiff = (s16)motorConfig.torque_2 - (s16)motorConfig.torque_1;
		m_speedDiff = (s32)motorConfig.speed_2-(s32)motorConfig.speed_1;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_1 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_2 < m_wheelMotorVelocity) && (motorConfig.speed_3 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_2;
		m_torqueDiff = (s16)motorConfig.torque_3 - (s16)motorConfig.torque_2;
		m_speedDiff = (s32)motorConfig.speed_3-(s32)motorConfig.speed_2;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_2 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_3 < m_wheelMotorVelocity) && (motorConfig.speed_4 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_3;
		m_torqueDiff = (s16)motorConfig.torque_4 - (s16)motorConfig.torque_3;
		m_speedDiff = (s32)motorConfig.speed_4-(s32)motorConfig.speed_3;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_3 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_4 < m_wheelMotorVelocity) && (motorConfig.speed_5 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_4;
		m_torqueDiff = (s16)motorConfig.torque_5 - (s16)motorConfig.torque_4;
		m_speedDiff = (s32)motorConfig.speed_5-(s32)motorConfig.speed_4;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_4 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_5 < m_wheelMotorVelocity) && (motorConfig.speed_6 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_5;
		m_torqueDiff = (s16)motorConfig.torque_6 - (s16)motorConfig.torque_5;
		m_speedDiff = (s32)motorConfig.speed_6-(s32)motorConfig.speed_5;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_5 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_6 < m_wheelMotorVelocity) && (motorConfig.speed_7 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_6;
		m_torqueDiff = (s16)motorConfig.torque_7 - (s16)motorConfig.torque_6;
		m_speedDiff = (s32)motorConfig.speed_7-(s32)motorConfig.speed_6;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_6 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if((motorConfig.speed_7 < m_wheelMotorVelocity) && (motorConfig.speed_8 >= m_wheelMotorVelocity))
	{
		m_speedWheelDiff = m_wheelMotorVelocity-motorConfig.speed_7;
		m_torqueDiff = (s16)motorConfig.torque_8 - (s16)motorConfig.torque_7;
		m_speedDiff = (s32)motorConfig.speed_8-(s32)motorConfig.speed_7;

		m_mapSpeedTorque = (s32)motorConfig.maximumCurrent*( (s32)motorConfig.torque_7 + (m_torqueDiff*m_speedWheelDiff)/m_speedDiff)/100;
	}
	if(motorConfig.speed_8 < m_wheelMotorVelocity)
	{
		m_mapSpeedTorque = motorConfig.maximumCurrent*motorConfig.torque_8/100;
	}
}

s32 sensor_motorAssistance()
{
	targetPedalTorque = m_filteredPedalTorqueAverage * motorConfig.torqueSensorGain;

	// use only if above a minimal positive value
	if ((targetPedalTorque <= (s16)motorConfig.torqueDeadband) || (/*m_filteredActualPedalVelocitySwift*/m_pedalVelocity <=(s16)motorConfig.pedalSenseDelay))
	{
		targetPedalTorque = 0;
	}

	// limit to max positive value
	if (targetPedalTorque > (s32)motorConfig.maximumCurrent)
	{
		targetPedalTorque = motorConfig.maximumCurrent;
	}
	// limit to Speed/Torque Map
	if (targetPedalTorque > (s32)m_mapSpeedTorque)
	{
		targetPedalTorque = m_mapSpeedTorque;
	}

	if (m_wheelMotorVelocity > motorConfig.maximumSpeed*VelocityScaling)
	{
		targetPedalTorque = 0;
	}

	if (targetPedalTorque > 0)
	{
		m_pedalMotorEnable = 1;
	}
	if (targetPedalTorque <= 0)
	{
		m_pedalMotorEnable = 0;
	}

	return targetPedalTorque;
}

void sensor_updateCutOffTime()
{
	m_cutOffTime = (motorConfig.assistCutOutDistance*3600)/motorConfig.maximumSpeed; // 3600km/h = 1m/ms		m_cutOffTime = ms
	if(m_cutOffTime > 100) //delay compensation of the pedal velocity filter
	{
		m_cutOffTime -= 100;
	}
}

s32 sensor_cutOffDistance(s32 m_rampTargetTorque)
{
	if(targetPedalTorque <= 0)
	{
		cutOffTimer++;
		if(cutOffTimer >= m_cutOffTime )
		{
			m_rampTargetTorque = 0;
		}
	}
	else
	{
		cutOffTimer = 0;
	}
	return m_rampTargetTorque;
}

void sensor_brakeDetection()
{
	if (m_wheelVelocity <= m_wheelVelocityLast)
	{
		m_brakeStartPoint = m_wheelVelocity;
	}

	out_Y = (s8)LIS2DH12_spi_readInt(LIS2DH12_OUT_Y_H);
	out_Y *= 16;		// LIS2DH12 data sheet Table 4. Mechanical characteristics	-	Low-power mode & measurement range 2g

	if ((m_brakeStartPoint >= MIN_BRAKE_SPEED) && (m_wheelVelocityLast*9/10 >= m_wheelVelocity))
	{
		minGDetection += (out_Y >= MIN_G_FORCES) ? 1 : 0;
		if(minGDetection > minGDetectionLast)
		{
			rear_led_setMode(REAR_LED_MODE_BLINK);
			rear_led_setBlinkTime(300);
		}
		minGDetectionLast = minGDetection;
	}
	else if ((m_wheelVelocity <= 0) || (m_wheelVelocityLast < m_wheelVelocity))
	{
		rear_led_setMode(REAR_LED_MODE_ON);
		minGDetection = 0;
		minGDetectionLast = 0;
	}

	m_wheelVelocityLast = m_wheelVelocity;

//	debug_setTestVar9(out_Y);
//	debug_setTestVar0(minGDetection);
//	debug_setTestVar1(m_brakeStartPoint);
}

void sensor_updateActualTorqueLimit_Gain()
{
	s32 m_rampTargetTorque = bldc_rampTargetTorque();
	s32 m_actualMotorTorque = tmc4671_getActualTorqueFluxSum_mA(0, motorConfig.dualShuntFactor);

	m_actualTorque = tmc_filterPT1(&akkuActualMotorTorque, m_actualMotorTorque, m_actualTorque, 6, 8);

	if((m_actualMotorTorque >= m_rampTargetTorque)|| (m_actualMotorTorque >= (s32)motorConfig.maximumCurrent))
	{
		m_actualTorqueLimit = 1;
	}
	else
	{
		m_actualTorqueLimit = 0;
	}
}

int32 sensor_actualMapSpeedTorque()
{
	return m_mapSpeedTorque;
}

int32 sensor_actualGain()
{
	return m_actualGain;
}

int32 sensor_actualTorqueLimit()
{
	return m_actualTorqueLimit;
}

int32 sensor_leftPedalTorque()
{
	return m_leftPedalTorque;
}

int32 sensor_rightPedalTorque()
{
	return m_rightPedalTorque;
}

int32 sensor_targetpedalTorque()
{
	return sensor_motorAssistance();
}

int32 sensor_pedalVelocity()
{
	return m_pedalVelocity;
}

int32 sensor_filteredPedalVelocity()
{
	return m_filteredActualPedalVelocity;
}

int32 sensor_filteredPedalVelocityFast()
{
	return m_filteredActualPedalVelocitySwift;
}

int8 sensor_pedalDirection()
{
	return m_pedalDirection;
}

int8 sensor_pedalMotorEnable()
{
	return m_pedalMotorEnable;
}

int32 sensor_averageTorque()
{
	return m_averageTorque;
}

int32 sensor_motorVelocity()
{
	return m_motorVelocity;
}

int32 sensor_wheelMotorVelocity()
{
	return m_wheelMotorVelocity;
}

int32 sensor_wheelVelocity()
{
	return m_wheelVelocity;
}

int wheelSensorDiff(u8 previousState, u8 actualState)
{
	switch (previousState)
	{
		case 1:
			return (actualState == 0) ? 1 : 0;
			break;
	}
	return 0;
}

int bottomBracketDiff(u8 previousState, u8 actualState)
{
	switch(previousState)
	{
		case 0:
			if(actualState==1)
			{
				return 1;
			}
			else if(actualState==2)
			{
				return -1;
			}
			break;
		case 1:
			if(actualState==3)
			{
				return 1;
			}
			else if(actualState==0)
			{
				return -1;
			}
			break;
		case 3:
			if(actualState==2)
			{
				return 1;
			}
			else if(actualState==1)
			{
				return -1;
			}
			break;
		case 2:
			if(actualState==0)
			{
				return 1;
			}
			else if(actualState==3)
			{
				return -1;
			}
			break;
	}
	return 0;
}
