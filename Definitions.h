/*
 * Definitions.h
 *
 *  Created on: 31.03.2019
 *      Author: ED / OK
 */

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

	#include "TMC-API/tmc/helpers/API_Header.h"

	#define DEFAULT_MC  0
	#define DEFAULT_DRV 1

	// commutation modes
	#define COMM_MODE_FOC_DISABLED						0
	#define COMM_MODE_FOC_OPEN_LOOP  					1
	#define COMM_MODE_FOC_DIGITAL_HALL					2
	#define COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST		3

	extern u8 ADC_SUPPLY_VOLTAGE;
  	extern u8 ADC_VOLTAGE_12V;
  	extern u8 ADC_VOLTAGE_6V;
  	extern u8 ADC_VOLTAGE_5V;
	extern u8 ADC_MOT_TEMP;

	typedef struct
	{
		uint8_t baudrate;
		uint8_t serialModuleAddress;
		uint8_t serialHostAddress;
		uint8_t CANBitrate;
		uint32_t CANReceiveID;
		uint32_t CANSendID;
		uint32_t CANSecondaryID;
	} TModuleConfig;

	typedef struct
	{
		u16 pedalPulsesPerRotation;
		u16 pedalSenseDelay;
		s16 torqueSensorGain;
		u16 torqueSensorOffset;
		u16 torqueDeadband;
		u16 assistCutOutDistance;
		u32 initialRightTorque;
		u32 initialRightTorqueSpeed;
		u8 leftRightRatio;
		u8 averageSportMode;

		u16 positiveMotoringRampTime;
		u16 negativeMotoringRampTime;
		u16 speed_0;
		u16 speed_1;
		u16 speed_2;
		u16 speed_3;
		u16 speed_4;
		u16 speed_5;
		u16 speed_6;
		u16 speed_7;
		u16 speed_8;
		u8 torque_0;
		u8 torque_1;
		u8 torque_2;
		u8 torque_3;
		u8 torque_4;
		u8 torque_5;
		u8 torque_6;
		u8 torque_7;
		u8 torque_8;
		u16 maximumSpeed;
		u32 maximumCurrent;
		u8 polePairs;
		u8 gearRatio;
		u16 wheelDiameter;
		u8 wheelPulsesPerRotation;

		s16 hallOffset;
		bool hallPolarity;
		bool hallInterpolation;
		bool hallDirection;

		u16 currentRegulatorBandwidth;
		u16 minimumMotorCurrent;

		bool swapMotorAAndCPhase;
		bool motorTestModes;

		u16 minBatteryVoltage;
		u16 maxBatteryVoltage;
		u16 cutOffVoltage;
		u16 batterySavingTimer;

		s32 maxPositioningSpeed;
		s32 acceleration;
		u32 openLoopCurrent;
		u16 adc_I0_offset;
		u16 adc_I1_offset;
		u16 dualShuntFactor;		// u8.u8
		u8 commutationMode;
		u8 encoder_m_direction;
		u8 encoderInitMode;
		u16 encoderInitDelay;
		s16 encoderInitVelocity;
		u32 encoder_m_steps;
		u16 encoder_m_offset;
		u8 encoderInitState;
		u8 useVelocityRamp;
		u8 positioningFlags;
		s32 positionReachedVelocity;
		s32 positionReachedDistance;
		u16 pidTorque_P_param;
		u16 pidTorque_I_param;
		u16 pidVelocity_P_param;
		u16 pidVelocity_I_param;
	} TMotorConfig;

	extern TMotorConfig motorConfig;
	extern TModuleConfig moduleConfig;

	// SPI chip selects
	extern void tmcm_enableCsWeasel();
	extern void tmcm_disableCsWeasel();

	extern void tmcm_enableCsDragon();
	extern void tmcm_disableCsDragon();

	extern void tmcm_enableCsLIS2DH12();
	extern void tmcm_disableCsLIS2DH12();

	extern void tmcm_enableCsMem();
	extern void tmcm_disableCsMem();

	// driver/power control
	extern void tmcm_enableDriver();
	extern void tmcm_disableDriver();
	extern uint8_t tmcm_getDriverState();

	extern void tmcm_disablePower();
	extern void tmcm_enablePower();
	extern u8 tmcm_getPowerState();

	// LED control
	extern void tmcm_ledGreenOn();
	extern void tmcm_ledGreenOff();
	extern void tmcm_ledGreenToggle();

	extern void tmcm_ledRedOn();
	extern void tmcm_ledRedOff();

	extern void tmcm_frontLedOn();
	extern void tmcm_frontLedOff();

	extern void tmcm_rearLedOn();
	extern void tmcm_rearLedOff();

	extern void tmcm_setUartToSendMode();
	extern void tmcm_setUartToReceiveMode();
	extern uint8_t tmcm_isUartSending();

	// init functions
	extern void tmcm_initModuleConfig();
	extern void tmcm_initMotorConfig();
	extern void tmcm_initModuleSpecificIO();
	extern void tmcm_initModuleSpecificADC();
	extern void tmcm_updateConfig();

	// IO functions
	extern u32 tmcm_getModuleSpecificADCValue(u8 pin);
	extern void tmcm_clearModuleSpecificIOPin(u8 pin);
	extern void tmcm_setModuleSpecificIOPin(u8 pin);
	extern u8 tmcm_getModuleSpecificIOPin(u8 pin);
	extern u8 tmcm_getModuleSpecificIOPinStatus(u8 pin);

	extern u8 tmcm_getInterruptValue();
	extern u8 tmcm_getSinCosValue();
	extern int tmcm_getADCTorqueValue();
	extern u8 tmcm_getSpeedValue();
	extern u8 tmcm_getButtonValue();

	// constants for the EEPROM
	#define TMCM_ADDR_MODULE_CONFIG 	0
	#define	TMCM_ADDR_MOTOR_CONFIG 		64
	#define	TMCM_ADDR_EEPROM_MAGIC 		2047

#endif /* DEFINITIONS_H */
