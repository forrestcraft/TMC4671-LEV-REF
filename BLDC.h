/*
 * BLDC.h
 *
 *  Created on: 31.03.2019
 *      Author: ED
 */

#ifndef BLDC_H
#define BLDC_H

	#include "Definitions.h"
	#include "hal/modules/SelectModule.h"

	void bldc_init();
	void bldc_processBLDC();

	// general info
	u32 bldc_getSupplyVoltage();
	int bldc_getMotorTemperature();
	u32 bldc_getInput12V();
	u32 bldc_getInput6V();
	u32 bldc_getInput5V();
	u8 bldc_getMotorPolePairs();

	// commutation
	void bldc_switchToMotionMode(uint32_t mode);
	bool bldc_setCommutationMode(u8 mode);
	u8 bldc_getCommutationMode();
	s16 bldc_getOpenLoopAngle();
	s16 bldc_getEncoderAngle();
	s16 bldc_getDigitalHallAngle();

	// torque mode
	int bldc_getActualMotorCurrent();
	int bldc_getTargetMotorCurrent();
	void bldc_setTargetMotorCurrent(int targetTorque);
	s32 bldc_rampTargetTorque();

	// velocity mode
	int bldc_getTargetVelocity();
	void bldc_setTargetVelocity(int velocity);
	void bldc_stopMotor();
	int bldc_getRampGeneratorVelocity();
	int bldc_getActualVelocity();
	int bldc_getMaxVelocity();
	bool bldc_setMaxVelocity(int maxVelocity);
	bool bldc_setAcceleration(int acceleration);
	bool bldc_setRampEnabled(int enableRamp);

	// ADC configuration
	bool bldc_setDualShuntFactor(u16 factor);
	u16 bldc_getDualShuntFactor();
	int bldc_getAdcI0Offset();
	bool bldc_setAdcI0Offset(int offset);
	int bldc_getAdcI1Offset();
	bool bldc_setAdcI1Offset(int offset);

#endif
