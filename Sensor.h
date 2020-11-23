/*
 * Sensor.h
 *
 *  Created on: 21.02.2019
 *      Author: MO
 */
#ifndef SENSOR_H
#define SENSOR_H

	#define MAX_PEDAL_POSITIONS 	32
	#define VelocityScaling			10
	#define BATTERYSCALING			10
	#define MIN_BRAKE_SPEED 		15
	#define MIN_G_FORCES 			400

	#include "Definitions.h"

	void sensor_init();
	void sensor_updatePedalTorque();
	s32 sensor_actualPedalTorque();
	void sensor_computePedalTorqMinMax();
	void sensor_updatePedalCounter();
	s32 sensor_pedalCounter();
	s32 sensor_pedalPosition();
	void sensor_updatePedalVelocity();
	s32 sensor_pedalCounterPer500MSeconds();
	int sensor_getFltActualTorque();
	void sensor_setFltActualTorque(int flt);
	void sensor_updateMotorVelocity();
	void sensor_updateWheelCounter();
	void sensor_updateWheelVelocity();
	void sensor_mapSpeedTorque();
	s32 sensor_motorAssistance();
	void sensor_updateCutOffTime();
	s32 sensor_cutOffDistance(s32 m_rampTargetTorque);
	void sensor_brakeDetection();
	void sensor_updateActualTorqueLimit_Gain();
	int32 sensor_actualMapSpeedTorque();
	int32 sensor_actualGain();
	int32 sensor_actualTorqueLimit();
	int32 sensor_leftPedalTorque();
	int32 sensor_rightPedalTorque();
	int32 sensor_targetpedalTorque();
	int32 sensor_pedalVelocity();
	int32 sensor_filteredPedalVelocity();
	int32 sensor_filteredPedalVelocityFast();
	int8 sensor_pedalDirection();
	int8 sensor_pedalMotorEnable();
	int32 sensor_averageTorque();
	int32 sensor_motorVelocity();
	int32 sensor_wheelMotorVelocity();
	int32 sensor_wheelVelocity();

#endif /* SENSOR_H */
