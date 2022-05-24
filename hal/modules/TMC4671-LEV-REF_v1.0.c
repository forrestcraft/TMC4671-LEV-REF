/*
 * TMC4671-LEV-REF_v1.0.c
 *
 *  Created on: 12.11.2019
 *      Author: ED
 */
#include "TMC4671-LEV-REF_v1.0.h"
#include "hal/system/SysTick.h"
#include "hal/Flags.h"
#include "BLDC.h"

#if DEVICE==TMC4671_LEV_REF_V10

// general module settings
const char *VersionString="1634V100";
//const char *VersionString = "0022V100";


// ADC configuration
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC1_CHANNELS		6
static volatile uint16_t ADC1Value[ADC1_CHANNELS];	// array for analog values (filled by DMA)

TMotorConfig motorConfig;
TModuleConfig moduleConfig;
TMC_LinearRamp rampGenerator;


// ADC inputs
uint8_t ADC_VOLTAGE_5V 		= 6;
uint8_t ADC_VOLTAGE_6V 		= 5;
uint8_t ADC_VOLTAGE_12V 	= 4;
uint8_t ADC_SUPPLY_VOLTAGE	= 3;
uint8_t ADC_MOT_TEMP		= 2;

void tmcm_initModuleConfig()
{
	moduleConfig.baudrate = 7;				// UART 115200bps
	moduleConfig.serialModuleAddress = 1;	// UART
	moduleConfig.serialHostAddress = 2;		// UART
	moduleConfig.CANBitrate = 8;			// CAN-Bitrate (1000kBit/s)
	moduleConfig.CANReceiveID = 1;			// CAN
	moduleConfig.CANSendID = 2;				// CAN
	moduleConfig.CANSecondaryID = 0;		// CAN
}

void tmcm_initMotorConfig()
{
	motorConfig.pedalPulsesPerRotation 		= 32;
	motorConfig.assistCutOutDistance		= 1;
	motorConfig.leftRightRatio				= 50;
	motorConfig.averageSportMode			= 50;
	motorConfig.batterySavingTimer			= 5;
	motorConfig.positiveMotoringRampTime	= 20000;
	motorConfig.negativeMotoringRampTime	= 30000;

#if defined(STD_CUSTOMER)
	motorConfig.adc_I0_offset				= 33650;
	motorConfig.adc_I1_offset				= 33650;
	motorConfig.maximumCurrent				= 23500;
	motorConfig.polePairs					= 8;

	motorConfig.hallOffset					= -5461;
	motorConfig.hallPolarity				= 0;
	motorConfig.hallInterpolation			= 1;
	motorConfig.hallDirection				= 0;

	motorConfig.minBatteryVoltage			= 400;
	motorConfig.maxBatteryVoltage			= 540;
	motorConfig.cutOffVoltage				= 420;

	motorConfig.gearRatio					= 14;
	motorConfig.wheelDiameter				= 712;
	motorConfig.wheelPulsesPerRotation		= 1;

	motorConfig.torqueSensorOffset			= 2050; // torque sensor offset + 50
	motorConfig.pedalSenseDelay				= 1;
	motorConfig.torqueSensorGain			= 600;
	motorConfig.torqueDeadband				= 500;
	motorConfig.initialRightTorque			= 200;
	motorConfig.initialRightTorqueSpeed		= 50;

	motorConfig.speed_0						= 0;
	motorConfig.speed_1						= 50;
	motorConfig.speed_2						= 80;
	motorConfig.speed_3						= 100;
	motorConfig.speed_4						= 150;
	motorConfig.speed_5						= 180;
	motorConfig.speed_6						= 200;
	motorConfig.speed_7						= 230;
	motorConfig.speed_8						= 270;
	motorConfig.torque_0					= 100;
	motorConfig.torque_1					= 95;
	motorConfig.torque_2					= 80;
	motorConfig.torque_3					= 63;
	motorConfig.torque_4					= 49;
	motorConfig.torque_5					= 43;
	motorConfig.torque_6					= 36;
	motorConfig.torque_7					= 25;
	motorConfig.torque_8					= 0;

	motorConfig.maximumSpeed				= 27; // km/h

#elif defined(INTERNAL_SHORTY)
	motorConfig.adc_I0_offset				= 33500;
	motorConfig.adc_I1_offset				= 33500;
	motorConfig.maximumCurrent				= 23500;
	motorConfig.polePairs					= 6;


	motorConfig.hallOffset					= 0;
	motorConfig.hallPolarity				= 0;
	motorConfig.hallInterpolation			= 1;
	motorConfig.hallDirection				= 0;

	motorConfig.minBatteryVoltage			= 350;
	motorConfig.maxBatteryVoltage			= 420;
	motorConfig.cutOffVoltage				= 360;

	motorConfig.gearRatio					= 7;
	motorConfig.wheelDiameter				= 670;
	motorConfig.wheelPulsesPerRotation		= 0;

	motorConfig.torqueSensorOffset			= 2150;// torque sensor offset + 50 //2140; PT1_Estonia
	motorConfig.pedalSenseDelay				= 1;
	motorConfig.torqueSensorGain			= 600;
	motorConfig.torqueDeadband				= 500;
	motorConfig.initialRightTorque			= 200;
	motorConfig.initialRightTorqueSpeed		= 50;

	motorConfig.speed_0						= 0;
	motorConfig.speed_1						= 50;
	motorConfig.speed_2						= 80;
	motorConfig.speed_3						= 100;
	motorConfig.speed_4						= 150;
	motorConfig.speed_5						= 180;
	motorConfig.speed_6						= 200;
	motorConfig.speed_7						= 230;
	motorConfig.speed_8						= 270;
	motorConfig.torque_0					= 100;
	motorConfig.torque_1					= 95;
	motorConfig.torque_2					= 80;
	motorConfig.torque_3					= 63;
	motorConfig.torque_4					= 49;
	motorConfig.torque_5					= 43;
	motorConfig.torque_6					= 36;
	motorConfig.torque_7					= 25;
	motorConfig.torque_8					= 0;

	motorConfig.maximumSpeed				= 27; // km/h
#endif

	motorConfig.currentRegulatorBandwidth	= 25;
	motorConfig.minimumMotorCurrent			= 100;

	motorConfig.swapMotorAAndCPhase			= 0;
	motorConfig.motorTestModes				= 0;

	// motor control configuration
	motorConfig.maxPositioningSpeed 		= 4000;
	motorConfig.acceleration				= 2000;
	motorConfig.useVelocityRamp				= true;
	motorConfig.openLoopCurrent				= 2000;
	motorConfig.commutationMode				= COMM_MODE_FOC_DISABLED;

	motorConfig.encoder_m_direction			= 0;
	motorConfig.encoder_m_steps				= 4000;

	motorConfig.encoder_m_offset			= 0;
	motorConfig.encoderInitMode				= 0;
	motorConfig.encoderInitState			= 0;
	motorConfig.encoderInitDelay			= 1000;
	motorConfig.encoderInitVelocity			= 200;
	motorConfig.dualShuntFactor				= 650;
	motorConfig.positioningFlags			= 0;
	motorConfig.positionReachedDistance		= 50;
	motorConfig.positionReachedVelocity		= 500;

	motorConfig.pidTorque_P_param			= 800;
	motorConfig.pidTorque_I_param			= 1000;
	motorConfig.pidVelocity_P_param			= 100;
	motorConfig.pidVelocity_I_param			= 100;
	motorConfig.pidPosition_P_param			= 0;
	motorConfig.pidPosition_I_param			= 0;

	// init ramp generator
	tmc_linearRamp_init(&rampGenerator);
}

void tmcm_initModuleSpecificIO()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// port A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// outputs port A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BSRR = BIT12;	// CS_6200
	GPIOA->BSRR = BIT15;	// EN_Power
	GPIOA->BRR = BIT11;		// EN_FRONT_LIGHT off
	GPIOA->BRR = BIT10;		// EN_BACK_LIGHT off
	GPIOA->BRR = BIT9;		// EN_LED_GREEN off
	GPIOA->BRR = BIT8;		// EN_LED_RED off

	// inputs port A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// analog inputs port A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// port B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// outputs port B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_10|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRR = GPIO_Pin_3;	// CS_4671
	GPIOB->BSRR = GPIO_Pin_4;	// EN_4671
	GPIOB->BSRR = GPIO_Pin_10;	// CS_LIS2DH12
	GPIOB->BSRR = GPIO_Pin_12;	// CS_EEPROM

	// inputs port B
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// analog inputs port B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// port C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// inputs port C without pull up
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// port D
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_PD01, DISABLE);  // PD0 und PD1 bleiben Oszillatoranschl�sse
}

void tmcm_interruptConfig()
{
	//clear Standby flag
	if(PWR_GetFlagStatus(PWR_FLAG_SB))
	{
		PWR_ClearFlag(PWR_FLAG_SB);
	}

	//clear WKUP flag
//	PWR_WakeUpPinCmd(DISABLE);
	PWR_ClearFlag(PWR_FLAG_WU);
//	PWR_WakeUpPinCmd(ENABLE);
}

void tmcm_enableStandbyMode()
{
//	PWR_BackupAccessCmd(DISABLE);
//	PWR_WakeUpPinCmd(ENABLE);
//	PWR_EnterSTANDBYMode();
}

void tmcm_updateConfig()
{
	// === configure linear ramp generator
	rampGenerator.maxVelocity  = motorConfig.maxPositioningSpeed;
	rampGenerator.acceleration = motorConfig.acceleration;
	rampGenerator.rampEnabled  = motorConfig.useVelocityRamp;

	// let weasel and dragon some time to start-up
	wait(200);

	// === configure TMC6200 ===
	tmc6200_writeInt(DEFAULT_DRV, TMC6200_GCONF, 0);	// normal pwm control
	tmc6200_writeInt(DEFAULT_DRV, TMC6200_DRV_CONF, 0);	// BBM_OFF and DRVSTRENGTH to weak
	TMC6200_FIELD_UPDATE(DEFAULT_DRV, TMC6200_SHORT_CONF, TMC6200_DISABLE_S2G_MASK, TMC6200_DISABLE_S2G_SHIFT, 1);
	TMC6200_FIELD_UPDATE(DEFAULT_DRV, TMC6200_SHORT_CONF, TMC6200_DISABLE_S2VS_MASK, TMC6200_DISABLE_S2VS_SHIFT, 1);

	// === configure TMC4671 ===

	// dummy readout
	tmc4671_readInt(DEFAULT_MC, TMC4671_MOTOR_TYPE_N_POLE_PAIRS);

	// motor type &  PWM configuration
	tmc4671_writeInt(DEFAULT_MC, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, 0x00030000 | motorConfig.polePairs);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_POLARITIES, 0x00000000);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_MAXCNT, 0x00000F9F);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_BBM_H_BBM_L, 0x00001919);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_SV_CHOP, 0x00000007);

	// ADC configuration
	tmc4671_writeInt(0, TMC4671_ADC_I_SELECT, 0x12000100);
	tmc4671_writeInt(0, TMC4671_dsADC_MCFG_B_MCFG_A, 0x00100010);
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_A, 0x10000000);
	tmc4671_writeInt(0, TMC4671_dsADC_MCLK_B, 0x00000000);
	tmc4671_writeInt(0, TMC4671_dsADC_MDEC_B_MDEC_A, 0x03FF03FF); //0x014E014E);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_I0_SCALE_OFFSET, 0x00FF0000 | motorConfig.adc_I0_offset);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_I1_SCALE_OFFSET, 0x00FF0000 | motorConfig.adc_I1_offset);

	// digital hall settings
	TMC4671_FIELD_UPDATE(0, TMC4671_HALL_MODE, TMC4671_HALL_DIRECTION_MASK, TMC4671_HALL_DIRECTION_SHIFT, motorConfig.hallDirection);
	TMC4671_FIELD_UPDATE(0, TMC4671_HALL_MODE, TMC4671_HALL_INTERPOLATION_MASK, TMC4671_HALL_INTERPOLATION_SHIFT, motorConfig.hallInterpolation);
	TMC4671_FIELD_UPDATE(0, TMC4671_HALL_MODE, TMC4671_HALL_POLARITY_MASK, TMC4671_HALL_POLARITY_SHIFT, motorConfig.hallPolarity);
	TMC4671_FIELD_UPDATE(0, TMC4671_HALL_PHI_E_PHI_M_OFFSET, TMC4671_HALL_PHI_E_OFFSET_MASK, TMC4671_HALL_PHI_E_OFFSET_SHIFT, motorConfig.hallOffset);

	// PI configuration
	tmc4671_setTorqueFluxPI(DEFAULT_MC, motorConfig.pidTorque_P_param, motorConfig.pidTorque_I_param);
	tmc4671_setVelocityPI(DEFAULT_MC, motorConfig.pidVelocity_P_param, motorConfig.pidVelocity_I_param);
	tmc4671_setPositionPI(DEFAULT_MC, 0, 0);

	// limit configuration
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_VELOCITY_LIMIT, motorConfig.maxPositioningSpeed);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_ACCELERATION, motorConfig.acceleration);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDOUT_UQ_UD_LIMITS, 0x7FFF);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, 0x1);
	tmc4671_setTorqueFluxLimit_mA(DEFAULT_MC, motorConfig.dualShuntFactor, motorConfig.maximumCurrent);

	// reset target values
	tmc4671_writeInt(DEFAULT_MC, TMC4671_UQ_UD_EXT, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_VELOCITY_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_OPENLOOP_VELOCITY_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_POSITION_TARGET, 0);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PIDIN_TORQUE_TARGET_FLUX_TARGET, 0);

	bldc_switchToMotionMode(STOP_MODE);
	tmc4671_writeInt(DEFAULT_MC, TMC4671_PWM_SV_CHOP, 0x00000007);
}

void tmcm_initModuleSpecificADC()
{
	// enable clock for ADC and DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// DMA configuration
	DMA_InitTypeDef DMAInit;
	DMA_DeInit(DMA1_Channel1);
	DMAInit.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMAInit.DMA_MemoryBaseAddr = (u32)ADC1Value;
	DMAInit.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMAInit.DMA_BufferSize = ADC1_CHANNELS;
	DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMAInit.DMA_Mode = DMA_Mode_Circular;
	DMAInit.DMA_Priority = DMA_Priority_High;
	DMAInit.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMAInit);

	// enable DMA
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// ADC configuration
	ADC_InitTypeDef ADCInit;
	ADCInit.ADC_Mode = ADC_Mode_Independent;
	ADCInit.ADC_ScanConvMode = ENABLE;
	ADCInit.ADC_ContinuousConvMode = ENABLE;
	ADCInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADCInit.ADC_DataAlign = ADC_DataAlign_Right;
	ADCInit.ADC_NbrOfChannel = ADC1_CHANNELS;
	ADC_Init(ADC1, &ADCInit);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_28Cycles5);

	ADC_ITConfig(ADC1, ADC_IT_EOC|ADC_IT_AWD|ADC_IT_JEOC, DISABLE);

	// enable ADC-DMA
	ADC_DMACmd(ADC1, ENABLE);

	// enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// calibrate ADC1 automatically
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	// start current measurement
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// UART configuration
void tmcm_setUartToSendMode() {}
void tmcm_setUartToReceiveMode() {}
u8 tmcm_isUartSending()
{
	return false;
}

void tmcm_enableCsMem()
{
	GPIOB->BRR = BIT12;
}

void tmcm_disableCsMem()
{
	GPIOB->BSRR = BIT12;
}

void tmcm_clearModuleSpecificIOPin(u8 pin)
{
	switch(pin)
	{
		case 0:
			tmcm_disableDriver(); 	// EN_WEASEL
			break;
		case 1:
			GPIOA->BRR = BIT11; 	// clear EN_FRONT_LIGHT
			break;
		case 2:
			GPIOA->BRR = BIT10; 	// clear EN_BACK_LIGHT
			break;
		case 3:
			GPIOA->BRR = BIT9; 		// clear EN_LED_GREEN
			break;
		case 4:
			GPIOA->BRR = BIT8; 		// clear EN_LED_RED
			break;
	}
}

void tmcm_setModuleSpecificIOPin(u8 pin)
{
	switch(pin)
	{
		case 0:
			tmcm_enableDriver(); 	// ENABLE_WEASEL
			break;
		case 1:
			GPIOA->BSRR = BIT11; 	// set EN_FRONT_LIGHT
			break;
		case 2:
			GPIOA->BSRR = BIT10; 	// set EN_BACK_LIGHT
			break;
		case 3:
			GPIOA->BSRR = BIT9; 	// set EN_LED_GREEN
			break;
		case 4:
			GPIOA->BSRR = BIT8; 	// set EN_LED_RED
			break;
	}
}

u8 tmcm_getModuleSpecificIOPin(u8 pin)
{
	switch(pin)
	{
		case 0:			//BUTTON_INPUT
			return (GPIOB->IDR & BIT2) ? 1:0;
			break;
		case 1:			//INTERRUPT_INPUT
			return (GPIOA->IDR & BIT0) ? 1:0;
			break;
		case 2:			//COSINUS_INPUT
			return (GPIOC->IDR & BIT15) ? 1:0;
			break;
		case 3:			//SINUS_INPUT
			return (GPIOC->IDR & BIT14) ? 1:0;
			break;
		case 4:			//SPEED_INPUT
			return (GPIOC->IDR & BIT13) ? 1:0;
			break;
		default:
			return 0;
	}
}
u8 tmcm_getInterruptValue()	//INTERRUPT_INPUT
{
	return (GPIOA->IDR & BIT0) ? 1:0;
}
u8 tmcm_getSinCosValue()	//SINUS&COSINUS_INPUT
{
	return ((GPIOC->IDR >> 14) & 0x0003);
}

u8 tmcm_getSpeedValue()		// SPEED_INPUT
{
	return (GPIOC->IDR & BIT13) ? 1:0;
}

u8 tmcm_getButtonValue() 	// BUTTON_INPUT
{
	return (GPIOB->IDR & BIT2) ? 1:0;
}

u8 tmcm_getModuleSpecificIOPinStatus(u8 pin)
{
	switch (pin)
	{
		case 0:
			return (GPIOB->IDR & BIT4) ? 1:0; 	// WEASEL_ENABLED ?
			break;
		case 1:
			return (GPIOA->IDR & BIT11) ? 1:0; 	// EN_FRONT_LIGHT
			break;
		case 2:
			return (GPIOA->IDR & BIT10) ? 1:0; 	// EN_BACK_LIGHT
			break;
		case 3:
			return (GPIOA->IDR & BIT9) ? 1:0; 	// EN_LED_GREEN
			break;
		case 4:
			return (GPIOA->IDR & BIT8) ? 1:0; 	// EN_LED_RED
			break;
	}
	return 0;
}

u32 tmcm_getModuleSpecificADCValue(u8 pin)
{
	switch(pin)
	{
		case 0:
			return tmc4671_readFieldWithDependency(DEFAULT_MC, TMC4671_ADC_RAW_DATA, TMC4671_ADC_RAW_ADDR, 0x00, TMC4671_ADC_I0_RAW_MASK, TMC4671_ADC_I0_RAW_SHIFT);
			break;
		case 1:
			return tmc4671_readFieldWithDependency(DEFAULT_MC, TMC4671_ADC_RAW_DATA, TMC4671_ADC_RAW_ADDR, 0x00, TMC4671_ADC_I1_RAW_MASK, TMC4671_ADC_I1_RAW_SHIFT);
			break;
		case 2:
			return ADC1Value[1];  // ADC_MOT_TEMP;
			break;
		case 3:
			return ADC1Value[0];  // ADC_VOLTAGE
			break;
		case 4:
			return ADC1Value[2];  // ADC_12V
			break;
		case 5:
			return ADC1Value[3];  // ADC_6V
			break;
		case 6:
			return ADC1Value[4];  // ADC_5V
			break;
		case 7:
			return ADC1Value[5];  // ADC_torque
			break;
	}
	return 0;
}

int tmcm_getADCTorqueValue()		// ADC_TORQUE_INPUT
{
	return ADC1Value[5];
}

void tmcm_enableCsWeasel()
{
	GPIOB->BRR = GPIO_Pin_3;
}

void tmcm_disableCsWeasel()
{
	GPIOB->BSRR = GPIO_Pin_3;
}

void tmcm_enableCsDragon()
{
	GPIOA->BRR = GPIO_Pin_12;
}

void tmcm_disableCsDragon()
{
	GPIOA->BSRR = GPIO_Pin_12;
}

void tmcm_enableCsLIS2DH12()
{
	GPIOB->BRR = GPIO_Pin_10;
}

void tmcm_disableCsLIS2DH12()
{
	GPIOB->BSRR = GPIO_Pin_10;
}

void tmcm_enableDriver()
{
	if (!tmcm_getDriverState())
	{
		GPIOB->BSRR = BIT4; 	// enable WEASEL

		// wait to have Weasel ready after reset
		wait(150);
	}
}
void tmcm_disableDriver()
{
	bldc_switchToMotionMode(STOP_MODE);
	GPIOB->BRR = BIT4; 		// disable WEASEL
}

uint8_t tmcm_getDriverState()
{
	return ((GPIOB->IDR & BIT4) ? 1:0); // WEASEL enabled?
}

void tmcm_enablePower()
{
	GPIOA->BSRR = BIT15;	// enable Power
}

void tmcm_disablePower()
{
	GPIOA->BRR = BIT15;		// disable Power
}
u8 tmcm_getPowerState()
{
	return ((GPIOA->IDR & BIT15) ? 1:0);
}

void tmcm_frontLedOn()
{
	GPIOA->BSRR = BIT11;
}

void tmcm_frontLedOff()
{
	GPIOA->BRR = BIT11;
}

void tmcm_rearLedOn()
{
	GPIOA->BSRR = BIT10;
}

void tmcm_rearLedOff()
{
	GPIOA->BRR = BIT10;
}

void tmcm_ledGreenOn()
{
	GPIOA->BSRR = BIT9;
}

void tmcm_ledGreenOff()
{
	GPIOA->BRR = BIT9;
}

void tmcm_ledGreenToggle()
{
	GPIOA->ODR ^= BIT9;
}

void tmcm_ledRedOn()
{
	GPIOA->BSRR = BIT8;
}

void tmcm_ledRedOff()
{
	GPIOA->BRR = BIT8;
}
#endif
