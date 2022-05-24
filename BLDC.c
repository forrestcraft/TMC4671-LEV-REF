/*
 * BLDC.c
 *
 *  Created on: 31.03.2019
 *      Author: ED
 */

#include "BLDC.h"
#include "hal/system/SystemInfo.h"
#include "hal/system/SysTick.h"
#include "hal/Flags.h"
#include "hal/Debug.h"
#include "Led.h"
#include "LedRear.h"
#include "Button.h"
#include "Sensor.h"
#include "math.h"

	// === private variables ===

	// general information
	s32 gActualMotorTemperature = 0;		// actual motor temperature
	s32	gActualSupplyVoltage = 0;			// actual supply voltage
	s32 gActualInput12V = 0;				// actual 12V
	s32 gActualInput6V = 0;					// actual 6V
	s32 gActualInput5V = 0;					// actual 5V

	// velocity regulation
	int	gDesiredVelocity = 0;				// requested target velocity
	int	gDesiredPosition = 0;				// requested target velocity

	// torque ramp
	int32_t gRampTargetTorque = 0;
	int32_t gTargetTorque = 0;

	// commutation mode
	u8	gLastSetCommutationMode = 0xFF;		// actual commutation mode

	// motion mode
	uint32_t gMotionMode = STOP_MODE;

void bldc_switchToMotionMode(uint32_t mode)
{
	flags_clearStatusFlag(STOP_MODE | TORQUE_MODE | VELOCITY_MODE | POSITION_MODE | POSITION_END);

	switch(mode)
	{
		case STOP_MODE:
			flags_setStatusFlag(STOP_MODE);
			gMotionMode = STOP_MODE;
			tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_STOPPED);
			break;
		case TORQUE_MODE:
			flags_setStatusFlag(TORQUE_MODE);
			gMotionMode = TORQUE_MODE;
			tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_TORQUE);
			break;
		case VELOCITY_MODE:
			flags_setStatusFlag(VELOCITY_MODE);
			gMotionMode = VELOCITY_MODE;
			tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_VELOCITY);
			break;
        case POSITION_MODE:
			flags_setStatusFlag(POSITION_MODE);
			gMotionMode = POSITION_MODE;
			tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_POSITION);
			break;
	}
}

void bldc_init()
{
	// init commutation mode
	gLastSetCommutationMode = 0xFF;
	bldc_switchToMotionMode(STOP_MODE);
}

/* actual motor current in mA */
int bldc_getActualMotorCurrent()
{
	return tmc4671_getActualTorqueFluxSum_mA(DEFAULT_MC, motorConfig.dualShuntFactor);
}

int bldc_getTargetMotorCurrent()
{
	return tmc4671_getTargetTorqueFluxSum_mA(DEFAULT_MC, motorConfig.dualShuntFactor);
}

void bldc_setTargetMotorCurrent(int targetCurrent)
{
	if ((motorConfig.commutationMode == COMM_MODE_FOC_DISABLED)
	  ||(motorConfig.commutationMode == COMM_MODE_FOC_OPEN_LOOP))
		return;

	if((targetCurrent >= -MAX_TORQUE) && (targetCurrent <= MAX_TORQUE))
	{
		gTargetTorque = targetCurrent;

		// switch to torque mode
		bldc_switchToMotionMode(TORQUE_MODE);
	}
}

int bldc_getTargetVelocity()
{
	return (motorConfig.commutationMode == COMM_MODE_FOC_OPEN_LOOP) ? tmc4671_readInt(DEFAULT_MC, TMC4671_OPENLOOP_VELOCITY_TARGET) : gDesiredVelocity;
}
int bldc_getTargetPosition()
{
	return gDesiredPosition;
}

/* set target velocity [rpm] (x{>0:CW | 0:Stop | <0: CCW} */
void bldc_setTargetVelocity(int velocity)
{
	if (motorConfig.commutationMode == COMM_MODE_FOC_DISABLED)
		return;

	if ((velocity >= -MAX_VELOCITY) && (velocity <= MAX_VELOCITY))
	{
		gDesiredVelocity = velocity;

		// switch to velocity motion mode
		bldc_switchToMotionMode(VELOCITY_MODE);
	}
}
void bldc_setTargetPosition(int position)
{
	if (motorConfig.commutationMode == COMM_MODE_FOC_DISABLED)
		return;

    gDesiredPosition = position;

    // switch to velocity motion mode
    bldc_switchToMotionMode(POSITION_MODE);

}

/* actual ramp generator velocity */
int bldc_getRampGeneratorVelocity()
{
	if (motorConfig.commutationMode == COMM_MODE_FOC_OPEN_LOOP)
		return tmc4671_readInt(DEFAULT_MC, TMC4671_OPENLOOP_VELOCITY_ACTUAL);
	else
		return rampGenerator.rampVelocity;
}

/* actual velocity in rpm */
int bldc_getActualVelocity()
{
	return tmc4671_readInt(DEFAULT_MC, (motorConfig.commutationMode == COMM_MODE_FOC_OPEN_LOOP) ? TMC4671_OPENLOOP_VELOCITY_ACTUAL : TMC4671_PID_VELOCITY_ACTUAL);
}

/* actual Position? in rpm */
int bldc_getActualPosition()
{
    return tmc4671_readInt(DEFAULT_MC, TMC4671_PID_POSITION_ACTUAL);
}

void bldc_setActualPosition(int position)
{
    tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_POSITION_ACTUAL, position);
}

int bldc_getMaxVelocity()
{
	return motorConfig.maxPositioningSpeed;
}

bool bldc_setMaxVelocity(int maxVelocity)
{
	if((maxVelocity >= 0) && (maxVelocity <= MAX_VELOCITY))
	{
		motorConfig.maxPositioningSpeed = maxVelocity;
		rampGenerator.maxVelocity = maxVelocity;
		tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_VELOCITY_LIMIT, maxVelocity);
		return true;
	}
	return false;
}

bool bldc_setAcceleration(int acceleration)
{
	if((acceleration >= 0) && (acceleration <= MAX_ACCELERATION))
	{
		motorConfig.acceleration = acceleration;
		rampGenerator.acceleration = acceleration;
		tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_ACCELERATION, acceleration);
		return true;
	}
	return false;
}

bool bldc_setRampEnabled(int enableRamp)
{
	if((enableRamp == 0) || (enableRamp == 1))
	{
		motorConfig.useVelocityRamp = enableRamp;
		rampGenerator.rampEnabled = enableRamp;
		return true;
	}
	return false;
}

/* observe over-/under-voltage and disable driver if necessary */
void bldc_checkSupplyVoltage()
{
	// update actual voltage [100mV]
	gActualSupplyVoltage = (VOLTAGE_FACTOR_SUPPLY*tmcm_getModuleSpecificADCValue(ADC_SUPPLY_VOLTAGE))/4095;

	if (gActualSupplyVoltage >= MAX_SUPPLY_VOLTAGE)
	{
		flags_setStatusFlag(OVERVOLTAGE);
	}
	else if (gActualSupplyVoltage <= MIN_SUPPLY_VOLTAGE)
	{
		flags_setStatusFlag(UNDERVOLTAGE);
		bldc_switchToMotionMode(STOP_MODE);
		tmcm_disableDriver();
		flags_clearStatusFlag(OVERVOLTAGE);
	}
	else if (gActualSupplyVoltage > ON__SUPPLY_VOLTAGE)
	{
		flags_clearStatusFlag(OVERVOLTAGE);
		flags_clearStatusFlag(UNDERVOLTAGE);
	}
}

u32 bldc_getSupplyVoltage()
{
	return gActualSupplyVoltage;
}

void bldc_checkInput12V()
{
	// update actual voltage [100mV]
	gActualInput12V = (VOLTAGE_FACTOR_12V*tmcm_getModuleSpecificADCValue(ADC_VOLTAGE_12V))/4095;
}

u32 bldc_getInput12V()
{
	return gActualInput12V;
}

void bldc_checkInput6V()
{
	// update actual voltage [100mV]
	gActualInput6V = (VOLTAGE_FACTOR_6V*tmcm_getModuleSpecificADCValue(ADC_VOLTAGE_6V))/4095;
}

u32 bldc_getInput6V()
{
	return gActualInput6V;
}

void bldc_checkInput5V()
{
	// update actual voltage [100mV]
	gActualInput5V = (VOLTAGE_FACTOR_5V*tmcm_getModuleSpecificADCValue(ADC_VOLTAGE_5V))/4095;
}

u32 bldc_getInput5V()
{
	return gActualInput5V;
}

/* monitor the motor temperature in centigrade and turn off the motor on over temperature */
void bldc_checkMotorTemperature()
{
	float vTherm = tmcm_getModuleSpecificADCValue(ADC_MOT_TEMP)*3.3 / 4095.0;
	float rNTC = (vTherm)/((3.3-vTherm)/10.0);
	float b = 3455.0;
	float temp = (b * 298.16)/(b + (log(rNTC/10.0)*298.16))-273.16;
	gActualMotorTemperature = temp;

	if(gActualMotorTemperature >= MAX_CRITICAL_TEMP)
	{
		flags_setStatusFlag(OVERTEMPERATURE);

		// hold module in stop mode to suppress regulation
		bldc_switchToMotionMode(STOP_MODE);
		tmcm_disableDriver();
	}
	else if(gActualMotorTemperature <= MIN_CRITICAL_TEMP)
	{
		flags_clearStatusFlag(OVERTEMPERATURE);
		if(!tmcm_getDriverState() && !flags_getErrorFlags() && !flags_isStatusFlagSet(FREERUNNING) && !flags_isStatusFlagSet(STOP_MODE))
			tmcm_enableDriver();
	}
}

int bldc_getMotorTemperature()
{
	return gActualMotorTemperature;
}

/* check new requested commutation mode*/
void bldc_checkCommutationMode()
{
	if(gLastSetCommutationMode != motorConfig.commutationMode)
	{
		// switch to new mode
		switch(motorConfig.commutationMode)
		{
			case COMM_MODE_FOC_DISABLED:
				flags_clearStatusFlag(MODULE_INITIALIZED);
				tmc4671_writeInt(DEFAULT_MC, TMC4671_UQ_UD_EXT, 0);
				tmc4671_writeInt(DEFAULT_MC, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_EXTERNAL);
				tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_UQ_UD_EXT);
				break;
			case COMM_MODE_FOC_OPEN_LOOP:
				flags_clearStatusFlag(MODULE_INITIALIZED);
				tmc4671_writeInt(DEFAULT_MC, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_OPEN_LOOP);
				tmc4671_switchToMotionMode(DEFAULT_MC, TMC4671_MOTION_MODE_TORQUE);
				break;
			case COMM_MODE_FOC_DIGITAL_HALL:
                tmc4671_writeInt(DEFAULT_MC, TMC4671_ABN_2_DECODER_PPR, 4096);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_ABN_2_DECODER_MODE, 0x1000);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_VELOCITY_SELECTION, TMC4671_VELOCITY_PHI_M_ABN_2);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_POSITION_SELECTION, TMC4671_VELOCITY_PHI_M_ABN_2);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
                break;
            case COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST:
                tmc4671_writeInt(DEFAULT_MC, TMC4671_ABN_2_DECODER_PPR, 4096);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_ABN_2_DECODER_MODE, 0x1000);
				tmc4671_writeInt(DEFAULT_MC, TMC4671_VELOCITY_SELECTION, TMC4671_VELOCITY_PHI_M_ABN_2);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_POSITION_SELECTION, TMC4671_VELOCITY_PHI_M_ABN_2);
                tmc4671_writeInt(DEFAULT_MC, TMC4671_PHI_E_SELECTION, TMC4671_PHI_E_HALL);
				break;
		}
		gLastSetCommutationMode = motorConfig.commutationMode;
	}
}

void bldc_stopMotor()
{
	if (motorConfig.commutationMode == COMM_MODE_FOC_DIGITAL_HALL || motorConfig.commutationMode == COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST )
		bldc_setTargetMotorCurrent(0);
	else
		bldc_setTargetVelocity(0);
}

int updateRampTargetTorque(int actualTargetTorque, int desiredTorque)
{
	if((motorConfig.positiveMotoringRampTime == 0) || (motorConfig.negativeMotoringRampTime == 0))
	{
		actualTargetTorque = desiredTorque;
	}
	else if(actualTargetTorque < desiredTorque)		// increase torque
	{
		actualTargetTorque += motorConfig.positiveMotoringRampTime/(1000);				// divide with pre-factor
		actualTargetTorque = MIN(actualTargetTorque, (int)motorConfig.maximumCurrent); 	// limit target torque to max allowed motor torque
	}
	else if(actualTargetTorque > desiredTorque)  	// decrease torque
	{
		actualTargetTorque -= motorConfig.negativeMotoringRampTime/(1000);				// divide with pre-factor
		actualTargetTorque = (sensor_wheelVelocity()>= motorConfig.maximumSpeed*10/2) ? MAX(actualTargetTorque, 0) :	MAX(actualTargetTorque, (int)motorConfig.minimumMotorCurrent);	// limit target torque
	}
	return actualTargetTorque;
}

/* main regulation function */
void bldc_processBLDC()
{
	// for timing debugging purposes only
	static u32 lastSystick = 0;
	static u32 lastSecCheckTime = 0;
	static u32 last50MSecCheckTime = 0;
	static u32 last90MSecCheckTime = 0;

	u32 newSystick = systick_getTimer();

	// 1ms "timer"
	if (lastSystick != newSystick)
	{
		bldc_checkCommutationMode();
		systemInfo_incCurrentLoopCounter();
		systemInfo_incVelocityLoopCounter();

		led_periodicJob();
		rear_led_periodicJob();

		// todo: use the next line to start the LEV by Power-On-Button:
		//button_periodicJob();

		sensor_brakeDetection();

		//  ===== analog information handling =====
		sensor_updatePedalTorque();
		sensor_computePedalTorqMinMax();

		// ===== digital information handling =====
		sensor_updatePedalCounter();
		sensor_updateWheelCounter();
		sensor_updateWheelVelocity();
		sensor_updateActualTorqueLimit_Gain();

		if (motorConfig.commutationMode == COMM_MODE_FOC_OPEN_LOOP)
		{
			// update target velocity
			rampGenerator.targetVelocity = gDesiredVelocity;
			tmc_linearRamp_computeRampVelocity(&rampGenerator);

			// use flux instead of torque
			tmc4671_setTargetTorque_mA(DEFAULT_MC, motorConfig.dualShuntFactor, 0);
			tmc4671_setTargetFlux_mA(DEFAULT_MC, motorConfig.dualShuntFactor, (rampGenerator.rampVelocity == 0) ? 0 : motorConfig.openLoopCurrent);

			// set open loop velocity
			tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_VELOCITY_TARGET, rampGenerator.rampVelocity);
		}
		else if (motorConfig.commutationMode == COMM_MODE_FOC_DIGITAL_HALL)
		{
			// reset flux value
			tmc4671_setTargetFlux_mA(DEFAULT_MC, motorConfig.dualShuntFactor, 0);

			if (gMotionMode == TORQUE_MODE)
			{
				// set new target torque
				tmc4671_setTargetTorque_mA(DEFAULT_MC, motorConfig.dualShuntFactor, gTargetTorque);
			}
			else if (gMotionMode == VELOCITY_MODE)
			{
				// update target velocity
				rampGenerator.targetVelocity = gDesiredVelocity;
				tmc_linearRamp_computeRampVelocity(&rampGenerator);

				// set new target velocity
				tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_VELOCITY_TARGET, rampGenerator.rampVelocity);
			}
            else if (gMotionMode == POSITION_MODE)
			{
				// set new target velocity
				tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_POSITION_TARGET, gDesiredPosition);
			}
		}
		else if (motorConfig.commutationMode == COMM_MODE_FOC_DIGITAL_HALL_PEDAL_ASSIST)
		{
			s32 pedalTargetTorque = sensor_motorAssistance();
			debug_setTestVar1(pedalTargetTorque);
			gRampTargetTorque = updateRampTargetTorque(gRampTargetTorque, pedalTargetTorque);

			// reset flux value
			tmc4671_setTargetFlux_mA(DEFAULT_MC, motorConfig.dualShuntFactor, 0);

			gRampTargetTorque = sensor_cutOffDistance(gRampTargetTorque);
			gRampTargetTorque = button_batterySaving(gRampTargetTorque);

			bldc_setTargetMotorCurrent(gRampTargetTorque);

			// debug_setTestVar6(cutOffDistance);
			// debug_setTestVar7(cutOffTime);
		}

		sensor_updateMotorVelocity();

		// check supply voltage
		bldc_checkSupplyVoltage();
		bldc_checkInput12V();
		bldc_checkInput6V();
		bldc_checkInput5V();

		// update check time
		lastSystick = newSystick;
	}

	// 50ms "timer"
	if (abs(newSystick-last50MSecCheckTime) >= 50)
	{
		sensor_mapSpeedTorque();

		// update check time
		last50MSecCheckTime = newSystick;
	}

	//90ms "timer"
	if (abs(newSystick-last90MSecCheckTime) >= 90)
	{
		// update velocities
		sensor_updatePedalVelocity();

		// update check time
		last90MSecCheckTime = newSystick;
	}

	// 1s "timer"
	if (abs(newSystick-lastSecCheckTime) >= 1000)
	{
		// check motor temperature every second
		bldc_checkMotorTemperature();

		button_autoDeactivation();

		// update check time
		lastSecCheckTime = newSystick;
	}
}

s32 bldc_rampTargetTorque()
{
	return gRampTargetTorque;
}

u8 bldc_getMotorPolePairs()
{
	motorConfig.polePairs = tmc4671_getPolePairs(DEFAULT_MC);
	return motorConfig.polePairs;
}

bool bldc_setCommutationMode(u8 mode)
{
	if (mode <= 3)
	{
		motorConfig.commutationMode = mode;
		return true;
	}
	return false;
}

u8 bldc_getCommutationMode()
{
	return motorConfig.commutationMode;
}

s16 bldc_getOpenLoopAngle()
{
	return FIELD_GET(tmc4671_readInt(DEFAULT_MC, TMC4671_OPENLOOP_PHI), TMC4671_OPENLOOP_PHI_MASK, TMC4671_OPENLOOP_PHI_SHIFT);
}

s16 bldc_getEncoderAngle()
{
	return FIELD_GET(tmc4671_readInt(DEFAULT_MC, TMC4671_ABN_DECODER_PHI_E_PHI_M), TMC4671_ABN_DECODER_PHI_E_MASK, TMC4671_ABN_DECODER_PHI_E_SHIFT);
}

s16 bldc_getDigitalHallAngle()
{
	return FIELD_GET(tmc4671_readInt(DEFAULT_MC, TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E), TMC4671_HALL_PHI_E_MASK, TMC4671_HALL_PHI_E_SHIFT);
}

// ADC configuration
bool bldc_setDualShuntFactor(u16 factor)
{
	motorConfig.dualShuntFactor = factor;
	return true;
}

u16 bldc_getDualShuntFactor()
{
	return motorConfig.dualShuntFactor;
}

int bldc_getAdcI0Offset()
{
	motorConfig.adc_I0_offset = tmc4671_getAdcI0Offset(DEFAULT_MC);
	return motorConfig.adc_I0_offset;
}

bool bldc_setAdcI0Offset(int offset)
{
	if(offset >= 0 && offset < 65536)
	{
		motorConfig.adc_I0_offset = offset;
		tmc4671_setAdcI0Offset(DEFAULT_MC, motorConfig.adc_I0_offset);
		return true;
	}
	return false;
}

int bldc_getAdcI1Offset()
{
	motorConfig.adc_I1_offset = tmc4671_getAdcI1Offset(DEFAULT_MC);
	return motorConfig.adc_I1_offset;
}

bool bldc_setAdcI1Offset(int offset)
{
	if(offset >= 0 && offset < 65536)
	{
		motorConfig.adc_I1_offset = offset;
		tmc4671_setAdcI1Offset(DEFAULT_MC, motorConfig.adc_I1_offset);
		return true;
	}
	return false;
}

/* commutation interrupt, take over the new desired pwm */
void TIM1_TRG_COM_IRQHandler()
{
	// clear interrupt flag
	TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
}

void TIM1_UP_IRQHandler()
{
	// clear update flag
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}

/* Interrupt handler for timer 2 */
void TIM2_IRQHandler(void)
{
	// clear the interrupt pending flag
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

/* Interrupt handler for timer 3 */
void TIM3_IRQHandler(void)
{
	// clear the interrupt pending flag
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
}

void TIM4_IRQHandler(void)
{
	// clear the interrupt pending flag
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}

/* interrupt handler for encoder-Null-channel */
void EXTI9_5_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		// clear interrupt bit
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

/* interrupt handler for uart interface */
void USART2_IRQHandler(){}

/* interrupt handler for uart interface */
void USART3_IRQHandler(){}

