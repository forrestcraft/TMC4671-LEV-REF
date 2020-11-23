/*
 * TMCL.c
 *
 *  Created on: 31.03.2019
 *      Author: ED / MO
 */

#include "TMCL.h"
#include "BLDC.h"
#include "hal/tmcl/TMCL-Defines.h"
#include "hal/system/IO.h"
#include "hal/system/SysTick.h"
#include "hal/system/SystemInfo.h"
#include "hal/comm/SPI.h"
#include "hal/comm/Eeprom.h"
#include "hal/ic/LIS2DH12.h"
#include "hal/Flags.h"
#include "hal/Debug.h"
#include "Sensor.h"
#include "Button.h"

#if defined(USE_CAN_INTERFACE)
	#include "hal/comm/CAN.h"
#endif

#if defined(USE_UART_INTERFACE)
	#include "hal/comm/UART.h"
#endif

	uint8_t ResetRequested = false;
	uint8_t TMCLReplyFormat;
	uint8_t SpecialReply[9];
	TTMCLCommand ActualCommand;
	TTMCLReply ActualReply;

	extern const char *VersionString;

	// local used functions
	void tmcl_setOutput();
	void tmcl_getInput();
	void tmcl_getVersion();
	void tmcl_boot();
	void tmcl_handleAxisParameter(uint8_t command);
	void tmcl_handleGlobalParameter(uint8_t command);
	void tmcl_softwareReset();
	void tmcl_factoryDefault();

// => SPI wrapper
u8 tmc4671_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
	return (motor == DEFAULT_MC) ? weasel_spi_readWriteByte(data, lastTransfer) : 0;
}

u8 tmc6200_readwriteByte(u8 motor, u8 data, u8 lastTransfer)
{
	return (motor == DEFAULT_DRV) ? dragon_spi_readWriteByte(data, lastTransfer) : 0;
}
// <= SPI wrapper

/* TMCL command ROL */
void tmcl_rotateLeft()
{
	bldc_setTargetVelocity(-ActualCommand.Value.Int32);
}

/* TMCL command ROR */
void tmcl_rotateRight()
{
	bldc_setTargetVelocity(ActualCommand.Value.Int32);
}

/* TMCL command MST */
void tmcl_motorStop()
{
	bldc_stopMotor();
}

/* execute the TMCL-Command stored in "ActualCommand" */
void tmcl_executeActualCommand()
{
	// prepare reply command
	ActualReply.Opcode = ActualCommand.Opcode;
	ActualReply.Status = REPLY_OK;
	ActualReply.Value.Int32 = ActualCommand.Value.Int32;

	// get command
	switch(ActualCommand.Opcode)
	{
		case TMCL_ROR:
			tmcl_rotateRight();
			break;
		case TMCL_ROL:
			tmcl_rotateLeft();
			break;
		case TMCL_MST:
			tmcl_motorStop();
			break;
    	case TMCL_SAP:
    	case TMCL_GAP:
    	case TMCL_STAP:
    	case TMCL_RSAP:
    		tmcl_handleAxisParameter(ActualCommand.Opcode);
    		break;
    	case TMCL_SGP:
    	case TMCL_GGP:
    	case TMCL_STGP:
    	case TMCL_RSGP:
    		tmcl_handleGlobalParameter(ActualCommand.Opcode);
    		break;
    	case TMCL_GetVersion:
    		tmcl_getVersion();
    		break;
    	case TMCL_SIO:
    		tmcl_setOutput();
    		break;
    	case TMCL_GIO:
    	    tmcl_getInput();
    		break;
    	case TMCL_FactoryDefault:
    		tmcl_factoryDefault();
    		break;
    	case TMCL_Boot:
    		tmcl_boot();
    		break;
    	case TMCL_SoftwareReset:
    		tmcl_softwareReset();
    		break;
		case TMCL_readRegisterChannel_1:
			if (ActualCommand.Motor == 0)
        		ActualReply.Value.Int32 = tmc4671_readInt(ActualCommand.Motor, ActualCommand.Type);
			else if (ActualCommand.Motor == 1)
				ActualReply.Value.Int32 = tmc6200_readInt(DEFAULT_DRV, ActualCommand.Type);
			else if (ActualCommand.Motor == 2)
				ActualReply.Value.Int32 = LIS2DH12_spi_readInt(ActualCommand.Type);
			break;
		case TMCL_writeRegisterChannel_1:
			if (ActualCommand.Motor == 0)
				tmc4671_writeInt(ActualCommand.Motor, ActualCommand.Type, ActualCommand.Value.Int32);
			else if (ActualCommand.Motor == 1)
    			tmc6200_writeInt(ActualCommand.Motor, ActualCommand.Type, ActualCommand.Value.Int32);
			else if (ActualCommand.Motor == 2)
				LIS2DH12_spi_writeInt(ActualCommand.Type, ActualCommand.Value.Int32);
			break;
    	default:
    		ActualReply.Status = REPLY_INVALID_CMD;
    		break;
	}
}

/* initialize tmcl */
void tmcl_init() {}

void tmcl_processCommand()
{
	static uint8_t TMCLCommandState;
    uint32_t i;

#ifdef USE_UART_INTERFACE
	uint8_t Byte;
	static uint8_t UARTCmd[9];
	static uint8_t UARTCount;
#endif

#ifdef USE_CAN_INTERFACE
	TCanFrame CanFrame;
	static uint8_t ExtendedCANFrame;
#endif

    /* send reply for last TMCL request */

#ifdef USE_CAN_INTERFACE

	if(TMCLCommandState == TCS_CAN7 || TMCLCommandState == TCS_CAN8)  // CAN reply
    {
    	CanFrame.Id = moduleConfig.CANSendID;
    	CanFrame.Dlc = (TMCLCommandState == TCS_CAN7 ? 7:8);
    	CanFrame.Ext = ExtendedCANFrame;
    	CanFrame.Rtr = false;

    	if(TMCLReplyFormat == RF_STANDARD)
    	{
    		CanFrame.Data[0] = moduleConfig.CANReceiveID & 0xff;
    		CanFrame.Data[1] = ActualReply.Status;
    		CanFrame.Data[2] = ActualReply.Opcode;
    		CanFrame.Data[3] = ActualReply.Value.Byte[3];
    		CanFrame.Data[4] = ActualReply.Value.Byte[2];
    		CanFrame.Data[5] = ActualReply.Value.Byte[1];
    		CanFrame.Data[6] = ActualReply.Value.Byte[0];
    		CanFrame.Data[7] = 0;
    	}
    	else if(TMCLReplyFormat == RF_SPECIAL)
    	{
    		for(i=0; i<8; i++)
    		{
    			CanFrame.Data[i] = SpecialReply[i+1];
    		}
    	}

    	if(TMCLReplyFormat != RF_NO_REPLY)
    	{
    		if(!can_sendMessage(&CanFrame)) return;
    	}
    }

#endif

#if defined(USE_UART_INTERFACE)

    if(TMCLCommandState==TCS_UART)  // UART reply
    {
    	if(TMCLReplyFormat==RF_STANDARD)
    	{
    		uint8_t checksum = moduleConfig.serialHostAddress+moduleConfig.serialModuleAddress+
    				ActualReply.Status+ActualReply.Opcode+
					ActualReply.Value.Byte[3]+ActualReply.Value.Byte[2]+
					ActualReply.Value.Byte[1]+ActualReply.Value.Byte[0];

    		uart_write(moduleConfig.serialHostAddress);
    		uart_write(moduleConfig.serialModuleAddress);
    		uart_write(ActualReply.Status);
    		uart_write(ActualReply.Opcode);
    		uart_write(ActualReply.Value.Byte[3]);
    		uart_write(ActualReply.Value.Byte[2]);
    		uart_write(ActualReply.Value.Byte[1]);
    		uart_write(ActualReply.Value.Byte[0]);
    		uart_write(checksum);
    	}
    	else if(TMCLReplyFormat==RF_SPECIAL)
    	{
    		for(i=0; i<9; i++)
    			uart_write(SpecialReply[i]);
    	}
    }
    else if(TMCLCommandState==TCS_UART_ERROR)  // last command had a wrong checksum
    {
    	ActualReply.Opcode = 0;
    	ActualReply.Status = REPLY_CHKERR;
    	ActualReply.Value.Int32 = 0;

    	uint8_t checksum = moduleConfig.serialHostAddress + moduleConfig.serialModuleAddress +
    			ActualReply.Status+ActualReply.Opcode +
				ActualReply.Value.Byte[3] + ActualReply.Value.Byte[2]+
				ActualReply.Value.Byte[1] + ActualReply.Value.Byte[0];

    	uart_write(moduleConfig.serialHostAddress);
    	uart_write(moduleConfig.serialModuleAddress);
    	uart_write(ActualReply.Status);
    	uart_write(ActualReply.Opcode);
    	uart_write(ActualReply.Value.Byte[3]);
    	uart_write(ActualReply.Value.Byte[2]);
    	uart_write(ActualReply.Value.Byte[1]);
    	uart_write(ActualReply.Value.Byte[0]);
    	uart_write(checksum);
    }
#endif

    // reset TMCL state (reply has been send)
  	TMCLCommandState = TCS_IDLE;
  	TMCLReplyFormat = RF_STANDARD;

  	// last command was a reset?
  	if(ResetRequested)
  	{
  		/* delay the reset, that the reply is send completely */
  		wait(100);
  		io_resetCPU(true);
  	}

  	/* read next request */

#ifdef USE_CAN_INTERFACE

  	if(can_getMessage(&CanFrame))  // new CAN request
  	{
  		ActualCommand.Opcode = CanFrame.Data[0];
  		ActualCommand.Type = CanFrame.Data[1];
  		ActualCommand.Motor = CanFrame.Data[2];
  		ActualCommand.Value.Byte[3] = CanFrame.Data[3];
  		ActualCommand.Value.Byte[2] = CanFrame.Data[4];
  		ActualCommand.Value.Byte[1] = CanFrame.Data[5];
  		ActualCommand.Value.Byte[0] = CanFrame.Data[6];
  		ExtendedCANFrame = CanFrame.Ext;

  		if(CanFrame.Dlc == 7)
  			TMCLCommandState = TCS_CAN7;
  		else
  			TMCLCommandState = TCS_CAN8;
  	}
#endif

#ifdef USE_UART_INTERFACE

  	if(uart_read((char *)&Byte))  // new UART request available?
  	{
  		if(uart_checkTimeout())
  			UARTCount = 0;  // discard everything when there has been a command timeout

  		UARTCmd[UARTCount++] = Byte;

  		if(UARTCount==9)  // Nine bytes have been received without timeout
  		{
  			UARTCount=0;
  			if(UARTCmd[0] == moduleConfig.serialModuleAddress)  // is this our address?
  			{
  				uint8_t checksum = 0;
  				for(i=0; i<8; i++)
  					checksum += UARTCmd[i];

  				if(checksum==UARTCmd[8])  // checksum correct?
  				{
  					ActualCommand.Opcode=UARTCmd[1];
  					ActualCommand.Type=UARTCmd[2];
  					ActualCommand.Motor=UARTCmd[3];
  					ActualCommand.Value.Byte[3]=UARTCmd[4];
  					ActualCommand.Value.Byte[2]=UARTCmd[5];
  					ActualCommand.Value.Byte[1]=UARTCmd[6];
  					ActualCommand.Value.Byte[0]=UARTCmd[7];
 					TMCLCommandState = TCS_UART;
  				}
  				else TMCLCommandState=TCS_UART_ERROR;  // checksum is wrong
  			}
  		}
  	}
#endif

   	// handle request after successful reading
  	if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR)
  		tmcl_executeActualCommand();
}

void tmcl_handleAxisParameter(uint8_t command)
{
	if(ActualCommand.Motor == 0)
	{
		switch(ActualCommand.Type)
		{
			// ===== ADC settings =====

			case 3: // adc_I0_raw
				if (command == TMCL_GAP)
				{
					tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_RAW_ADDR, ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW);
					ActualReply.Value.Int32 = TMC4671_FIELD_READ(DEFAULT_MC, TMC4671_ADC_RAW_DATA, TMC4671_ADC_I0_RAW_MASK, TMC4671_ADC_I0_RAW_SHIFT);
				}
				break;
			case 4: // adc_I1_raw
				if (command == TMCL_GAP)
				{
					tmc4671_writeInt(DEFAULT_MC, TMC4671_ADC_RAW_ADDR, ADC_RAW_ADDR_ADC_I1_RAW_ADC_I0_RAW);
					ActualReply.Value.Int32 = TMC4671_FIELD_READ(DEFAULT_MC, TMC4671_ADC_RAW_DATA, TMC4671_ADC_I1_RAW_MASK, TMC4671_ADC_I1_RAW_SHIFT);
				}
				break;
			case 5: // dual-shunt phase_A offset
				if (command == TMCL_SAP) {
					if (!bldc_setAdcI0Offset(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getAdcI0Offset();
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.adc_I0_offset-(u32)&motorConfig,
							(u8 *)&motorConfig.adc_I0_offset, sizeof(motorConfig.adc_I0_offset));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.adc_I0_offset-(u32)&motorConfig,
							(u8 *)&motorConfig.adc_I0_offset, sizeof(motorConfig.adc_I0_offset));
				}
				break;
			case 6: // dual-shunt phase_B offset
				if (command == TMCL_SAP) {
					if (!bldc_setAdcI1Offset(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getAdcI1Offset();
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.adc_I1_offset-(u32)&motorConfig,
							(u8 *)&motorConfig.adc_I1_offset, sizeof(motorConfig.adc_I1_offset));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.adc_I1_offset-(u32)&motorConfig,
							(u8 *)&motorConfig.adc_I1_offset, sizeof(motorConfig.adc_I1_offset));
				}
				break;
			case 7: // current_phase_U
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = (int16_t)TMC4671_FIELD_READ(DEFAULT_MC, TMC4671_ADC_IV, TMC4671_ADC_IV_MASK, TMC4671_ADC_IV_SHIFT);
				break;
			case 8: // current_phase_V
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = (int16_t)TMC4671_FIELD_READ(DEFAULT_MC, TMC4671_ADC_IWY_IUX, TMC4671_ADC_IWY_MASK, TMC4671_ADC_IWY_SHIFT);
				break;
			case 9: // current_phase_W
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = (int16_t)TMC4671_FIELD_READ(DEFAULT_MC, TMC4671_ADC_IWY_IUX, TMC4671_ADC_IUX_MASK, TMC4671_ADC_IUX_SHIFT);
				break;
			case 10: // dual shunt factor
				if (command == TMCL_SAP) {
					if (!bldc_setDualShuntFactor(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getDualShuntFactor();
				}
				else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.dualShuntFactor-(u32)&motorConfig,
							(u8 *)&motorConfig.dualShuntFactor, sizeof(motorConfig.dualShuntFactor));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.dualShuntFactor-(u32)&motorConfig,
							(u8 *)&motorConfig.dualShuntFactor, sizeof(motorConfig.dualShuntFactor));
				}
				break;

			// ===== motor settings =====

			case 12: // open loop current
				if (command == TMCL_SAP) {
					if((ActualCommand.Value.Int32 >= 0) && (ActualCommand.Value.Int32 <= MAX_TORQUE))
						motorConfig.openLoopCurrent = ActualCommand.Value.Int32;
					else
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.openLoopCurrent;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.openLoopCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.openLoopCurrent, sizeof(motorConfig.openLoopCurrent));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.openLoopCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.openLoopCurrent, sizeof(motorConfig.openLoopCurrent));
				}
				break;
			case 14: // motor type
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = TMC4671_THREE_PHASE_BLDC; // firmware supports only BLDC motor
				}
				break;
			case 15: // commutation mode
				if (command == TMCL_SAP) {
					if (!bldc_setCommutationMode(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getCommutationMode();
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.commutationMode-(u32)&motorConfig,
						(u8 *)&motorConfig.commutationMode, sizeof(motorConfig.commutationMode));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.commutationMode-(u32)&motorConfig,
						(u8 *)&motorConfig.commutationMode, sizeof(motorConfig.commutationMode));
				}
				break;

			case 16: // actual open loop commutation angle
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getOpenLoopAngle();
				}
				break;
			case 17: // actual encoder commutation angle
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getEncoderAngle();
				}
				break;
			case 18: // actual digital hall angle
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getDigitalHallAngle();
				}
				break;

			case 20: // torque P
				if (command == TMCL_SAP)
				{
					motorConfig.pidTorque_P_param = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_P_MASK, TMC4671_PID_FLUX_P_SHIFT, motorConfig.pidTorque_P_param);
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_P_MASK, TMC4671_PID_TORQUE_P_SHIFT, motorConfig.pidTorque_P_param);
				}
				else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pidTorque_P_param;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidTorque_P_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidTorque_P_param, sizeof(motorConfig.pidTorque_P_param));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidTorque_P_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidTorque_P_param, sizeof(motorConfig.pidTorque_P_param));
				}
				break;
			case 21: // torque I
				if (command == TMCL_SAP)
				{
					motorConfig.pidTorque_I_param = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_FLUX_P_FLUX_I, TMC4671_PID_FLUX_I_MASK, TMC4671_PID_FLUX_I_SHIFT, motorConfig.pidTorque_I_param);
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_TORQUE_P_TORQUE_I, TMC4671_PID_TORQUE_I_MASK, TMC4671_PID_TORQUE_I_SHIFT, motorConfig.pidTorque_I_param);
				}
				else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pidTorque_I_param;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidTorque_I_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidTorque_I_param, sizeof(motorConfig.pidTorque_I_param));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidTorque_I_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidTorque_I_param, sizeof(motorConfig.pidTorque_I_param));
				}
				break;
			case 22: // velocity P
				if (command == TMCL_SAP)
				{
					motorConfig.pidVelocity_P_param = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_P_MASK, TMC4671_PID_VELOCITY_P_SHIFT, motorConfig.pidVelocity_P_param);
				}
				else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pidVelocity_P_param;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidVelocity_P_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidVelocity_P_param, sizeof(motorConfig.pidVelocity_P_param));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidVelocity_P_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidVelocity_P_param, sizeof(motorConfig.pidVelocity_P_param));
				}
				break;
			case 23: // velocity I
				if (command == TMCL_SAP)
				{
					motorConfig.pidVelocity_I_param = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_PID_VELOCITY_P_VELOCITY_I, TMC4671_PID_VELOCITY_I_MASK, TMC4671_PID_VELOCITY_I_SHIFT, motorConfig.pidVelocity_I_param);
				}
				else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pidVelocity_I_param;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidVelocity_I_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidVelocity_I_param, sizeof(motorConfig.pidVelocity_I_param));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pidVelocity_I_param-(u32)&motorConfig,
						(u8 *)&motorConfig.pidVelocity_I_param, sizeof(motorConfig.pidVelocity_I_param));
				}
				break;

			// ===== torque mode settings =====

			case 30: // target torque
				if (command == TMCL_SAP) {
					if((ActualCommand.Value.Int32 >= -MAX_TORQUE) && (ActualCommand.Value.Int32 <= MAX_TORQUE))
						bldc_setTargetMotorCurrent(ActualCommand.Value.Int32);
					else
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getTargetMotorCurrent();
				}
				break;
			case 31: // actual torque
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = bldc_getActualMotorCurrent();
					//ActualReply.Value.Int32 = actualTorque();
				break;

			// ===== velocity mode settings =====

			case 40: // target velocity
				if (command == TMCL_SAP)
				{
					if((ActualCommand.Value.Int32 >= -MAX_VELOCITY) && (ActualCommand.Value.Int32 <= MAX_VELOCITY))
						bldc_setTargetVelocity(ActualCommand.Value.Int32);
					else
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getTargetVelocity();
				}
				break;
			case 41: // ramp velocity
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getRampGeneratorVelocity();
				}
				break;
			case 42: // actual velocity
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getActualVelocity();
				}
				break;
			case 43: // max velocity
				if (command == TMCL_SAP) {
					if (!bldc_setMaxVelocity(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.maxPositioningSpeed;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maxPositioningSpeed-(u32)&motorConfig,
							(u8 *)&motorConfig.maxPositioningSpeed, sizeof(motorConfig.maxPositioningSpeed));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maxPositioningSpeed-(u32)&motorConfig,
							(u8 *)&motorConfig.maxPositioningSpeed, sizeof(motorConfig.maxPositioningSpeed));
				}
				break;
			case 44: // acceleration
				if (command == TMCL_SAP) {
					if (!bldc_setAcceleration(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.acceleration;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.acceleration-(u32)&motorConfig,
						(u8 *)&motorConfig.acceleration, sizeof(motorConfig.acceleration));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.acceleration-(u32)&motorConfig,
						(u8 *)&motorConfig.acceleration, sizeof(motorConfig.acceleration));
				}
				break;
			case 45: // enable velocity ramp
				if (command == TMCL_SAP) {
					if (!bldc_setRampEnabled(ActualCommand.Value.Int32))
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.useVelocityRamp ? 1:0;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.useVelocityRamp-(u32)&motorConfig,
						(u8 *)&motorConfig.useVelocityRamp, sizeof(motorConfig.useVelocityRamp));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.useVelocityRamp-(u32)&motorConfig,
						(u8 *)&motorConfig.useVelocityRamp, sizeof(motorConfig.useVelocityRamp));
				}
				break;

				// ===== pedal sensor settings =====

			case 50: // pedal pulses per rotation
				if (command == TMCL_SAP)
				{
					if ((ActualCommand.Value.Int32 >= 0) && (ActualCommand.Value.Int32 <= MAX_PEDAL_POSITIONS))
						motorConfig.pedalPulsesPerRotation = ActualCommand.Value.Int32;
					else
						ActualReply.Status = REPLY_INVALID_VALUE;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pedalPulsesPerRotation;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pedalPulsesPerRotation-(u32)&motorConfig,
						(u8 *)&motorConfig.pedalPulsesPerRotation, sizeof(motorConfig.pedalPulsesPerRotation));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pedalPulsesPerRotation-(u32)&motorConfig,
						(u8 *)&motorConfig.pedalPulsesPerRotation, sizeof(motorConfig.pedalPulsesPerRotation));
				}
				break;
			case 52: // pedal sense delay
				if (command == TMCL_SAP)
				{
					motorConfig.pedalSenseDelay = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.pedalSenseDelay;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pedalSenseDelay-(u32)&motorConfig,
						(u8 *)&motorConfig.pedalSenseDelay, sizeof(motorConfig.pedalSenseDelay));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.pedalSenseDelay-(u32)&motorConfig,
						(u8 *)&motorConfig.pedalSenseDelay, sizeof(motorConfig.pedalSenseDelay));
				}
				break;
			case 53: // torque sensor gain
				if (command == TMCL_SAP)
				{
					motorConfig.torqueSensorGain = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torqueSensorGain;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueSensorGain-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueSensorGain, sizeof(motorConfig.torqueSensorGain));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueSensorGain-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueSensorGain, sizeof(motorConfig.torqueSensorGain));
				}
				break;
			case 54: // torque sensor offset
				if (command == TMCL_SAP)
				{
					motorConfig.torqueSensorOffset = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torqueSensorOffset;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueSensorOffset-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueSensorOffset, sizeof(motorConfig.torqueSensorOffset));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueSensorOffset-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueSensorOffset, sizeof(motorConfig.torqueSensorOffset));
				}
				break;
			case 55: // torque dead band
				if (command == TMCL_SAP)
				{
					motorConfig.torqueDeadband = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torqueDeadband;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueDeadband-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueDeadband, sizeof(motorConfig.torqueDeadband));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torqueDeadband-(u32)&motorConfig,
						(u8 *)&motorConfig.torqueDeadband, sizeof(motorConfig.torqueDeadband));
				}
				break;
			case 56: // assist cut out distance
				if (command == TMCL_SAP)
				{
					motorConfig.assistCutOutDistance = ActualCommand.Value.Int32;
					sensor_updateCutOffTime();
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.assistCutOutDistance;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.assistCutOutDistance-(u32)&motorConfig,
						(u8 *)&motorConfig.assistCutOutDistance, sizeof(motorConfig.assistCutOutDistance));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.assistCutOutDistance-(u32)&motorConfig,
						(u8 *)&motorConfig.assistCutOutDistance, sizeof(motorConfig.assistCutOutDistance));
				}
				break;
			case 57: // initial right torque
				if (command == TMCL_SAP)
				{
					motorConfig.initialRightTorque = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.initialRightTorque;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.initialRightTorque-(u32)&motorConfig,
						(u8 *)&motorConfig.initialRightTorque, sizeof(motorConfig.initialRightTorque));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.initialRightTorque-(u32)&motorConfig,
						(u8 *)&motorConfig.initialRightTorque, sizeof(motorConfig.initialRightTorque));
				}
				break;
			case 58: // initial tight torque speed
				if (command == TMCL_SAP)
				{
					motorConfig.initialRightTorqueSpeed = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.initialRightTorqueSpeed / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.initialRightTorqueSpeed-(u32)&motorConfig,
						(u8 *)&motorConfig.initialRightTorqueSpeed, sizeof(motorConfig.initialRightTorqueSpeed));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.initialRightTorqueSpeed-(u32)&motorConfig,
						(u8 *)&motorConfig.initialRightTorqueSpeed, sizeof(motorConfig.initialRightTorqueSpeed));
				}
				break;
			case 60: // left to right ratio
				if (command == TMCL_SAP)
				{
					motorConfig.leftRightRatio = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.leftRightRatio;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.leftRightRatio-(u32)&motorConfig,
						(u8 *)&motorConfig.leftRightRatio, sizeof(motorConfig.leftRightRatio));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.leftRightRatio-(u32)&motorConfig,
						(u8 *)&motorConfig.leftRightRatio, sizeof(motorConfig.leftRightRatio));
				}
				break;
			case 61: // average to sport mode
				if (command == TMCL_SAP)
				{
					motorConfig.averageSportMode = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.averageSportMode;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.averageSportMode-(u32)&motorConfig,
						(u8 *)&motorConfig.averageSportMode, sizeof(motorConfig.averageSportMode));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.averageSportMode-(u32)&motorConfig,
						(u8 *)&motorConfig.averageSportMode, sizeof(motorConfig.averageSportMode));
				}
				break;

			case 65: // pedal direction
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_pedalDirection();
				}
				break;
			case 66: // pedal motor enable
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_pedalMotorEnable();
				}
				break;
			case 67: // average torque
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_averageTorque();
				}
				break;

			// ===== motor power settings =====

			case 70: // positive motoring ramp time
				if (command == TMCL_SAP)
				{
					motorConfig.positiveMotoringRampTime = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.positiveMotoringRampTime;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.positiveMotoringRampTime-(u32)&motorConfig,
						(u8 *)&motorConfig.positiveMotoringRampTime, sizeof(motorConfig.positiveMotoringRampTime));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.positiveMotoringRampTime-(u32)&motorConfig,
						(u8 *)&motorConfig.positiveMotoringRampTime, sizeof(motorConfig.positiveMotoringRampTime));
				}
				break;
			case 71: // negative motoring ramp time
				if (command == TMCL_SAP)
				{
					motorConfig.negativeMotoringRampTime = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.negativeMotoringRampTime;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.negativeMotoringRampTime-(u32)&motorConfig,
						(u8 *)&motorConfig.negativeMotoringRampTime, sizeof(motorConfig.negativeMotoringRampTime));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.negativeMotoringRampTime-(u32)&motorConfig,
						(u8 *)&motorConfig.negativeMotoringRampTime, sizeof(motorConfig.negativeMotoringRampTime));
				}
				break;

			case 73: // speed 0
				if (command == TMCL_SAP)
				{
					motorConfig.speed_0 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_0 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_0-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_0, sizeof(motorConfig.speed_0));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_0-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_0, sizeof(motorConfig.speed_0));
				}
				break;
			case 74: // speed 1
				if (command == TMCL_SAP)
				{
					motorConfig.speed_1 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_1 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_1-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_1, sizeof(motorConfig.speed_1));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_1-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_1, sizeof(motorConfig.speed_1));
				}
				break;
			case 75: // speed 2
				if (command == TMCL_SAP)
				{
					motorConfig.speed_2 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_2 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_2-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_2, sizeof(motorConfig.speed_2));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_2-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_2, sizeof(motorConfig.speed_2));
				}
				break;
			case 76: // speed 3
				if (command == TMCL_SAP)
				{
					motorConfig.speed_3 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_3 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_3-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_3, sizeof(motorConfig.speed_3));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_3-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_3, sizeof(motorConfig.speed_3));
				}
				break;
			case 77: // speed 4
				if (command == TMCL_SAP)
				{
					motorConfig.speed_4 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_4 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_4-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_4, sizeof(motorConfig.speed_4));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_4-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_4, sizeof(motorConfig.speed_4));
				}
				break;
			case 78: // speed 5
				if (command == TMCL_SAP)
				{
					motorConfig.speed_5 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_5 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_5-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_5, sizeof(motorConfig.speed_5));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_5-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_5, sizeof(motorConfig.speed_5));
				}
				break;
			case 79: // speed 6
				if (command == TMCL_SAP)
				{
					motorConfig.speed_6 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_6 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_6-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_6, sizeof(motorConfig.speed_6));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_6-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_6, sizeof(motorConfig.speed_6));
				}
				break;
			case 80: // speed 7
				if (command == TMCL_SAP)
				{
					motorConfig.speed_7 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_7 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_7-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_7, sizeof(motorConfig.speed_7));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_7-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_7, sizeof(motorConfig.speed_7));
				}
				break;
			case 81: // speed 8
				if (command == TMCL_SAP)
				{
					motorConfig.speed_8 = ActualCommand.Value.Int32 * VelocityScaling;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.speed_8 / VelocityScaling;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_8-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_8, sizeof(motorConfig.speed_8));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.speed_8-(u32)&motorConfig,
						(u8 *)&motorConfig.speed_8, sizeof(motorConfig.speed_8));
				}
				break;
			case 82: // torque 0
				if (command == TMCL_SAP)
				{
					motorConfig.torque_0 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_0;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_0-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_0, sizeof(motorConfig.torque_0));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_0-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_0, sizeof(motorConfig.torque_0));
				}
				break;
			case 83: // torque 1
				if (command == TMCL_SAP)
				{
					motorConfig.torque_1 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_1;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_1-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_1, sizeof(motorConfig.torque_1));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_1-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_1, sizeof(motorConfig.torque_1));
				}
				break;
			case 84: // torque 2
				if (command == TMCL_SAP)
				{
					motorConfig.torque_2 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_2;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_2-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_2, sizeof(motorConfig.torque_2));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_2-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_2, sizeof(motorConfig.torque_2));
				}
				break;
			case 85: // torque 3
				if (command == TMCL_SAP)
				{
					motorConfig.torque_3 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_3;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_3-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_3, sizeof(motorConfig.torque_3));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_3-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_3, sizeof(motorConfig.torque_3));
				}
				break;
			case 86: // torque 4
				if (command == TMCL_SAP)
				{
					motorConfig.torque_4 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_4;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_4-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_4, sizeof(motorConfig.torque_4));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_4-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_4, sizeof(motorConfig.torque_4));
				}
				break;
			case 87: // torque 5
				if (command == TMCL_SAP)
				{
					motorConfig.torque_5 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_5;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_5-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_5, sizeof(motorConfig.torque_5));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_5-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_5, sizeof(motorConfig.torque_5));
				}
				break;
			case 88: // torque 6
				if (command == TMCL_SAP)
				{
					motorConfig.torque_6 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_6;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_6-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_6, sizeof(motorConfig.torque_6));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_6-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_6, sizeof(motorConfig.torque_6));
				}
				break;
			case 89: // torque 7
				if (command == TMCL_SAP)
				{
					motorConfig.torque_7 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_7;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_7-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_7, sizeof(motorConfig.torque_7));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_7-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_7, sizeof(motorConfig.torque_7));
				}
				break;
			case 90: // torque 8
				if (command == TMCL_SAP)
				{
					motorConfig.torque_8 = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.torque_8;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_8-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_8, sizeof(motorConfig.torque_8));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.torque_8-(u32)&motorConfig,
						(u8 *)&motorConfig.torque_8, sizeof(motorConfig.torque_8));
				}
				break;

			case 91: // maximum speed
				if (command == TMCL_SAP)
				{
					motorConfig.maximumSpeed = ActualCommand.Value.Int32;
					sensor_updateCutOffTime();
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.maximumSpeed;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maximumSpeed-(u32)&motorConfig,
						(u8 *)&motorConfig.maximumSpeed, sizeof(motorConfig.maximumSpeed));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maximumSpeed-(u32)&motorConfig,
						(u8 *)&motorConfig.maximumSpeed, sizeof(motorConfig.maximumSpeed));
				}
				break;
			case 92: // actual torque/speed map
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_actualMapSpeedTorque();
				}
				break;
			case 93: // actual gain
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_actualGain();
				}
				break;
			case 94: // actual torque limit
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_actualTorqueLimit();
				}
				break;

			// ===== motor settings =====

			case 100: // maximum current
				if (command == TMCL_SAP)
				{
					motorConfig.maximumCurrent = ActualCommand.Value.Int32;
					tmc4671_setTorqueFluxLimit_mA(DEFAULT_MC, motorConfig.dualShuntFactor, motorConfig.maximumCurrent);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.maximumCurrent;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maximumCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.maximumCurrent, sizeof(motorConfig.maximumCurrent));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maximumCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.maximumCurrent, sizeof(motorConfig.maximumCurrent));
				}
				break;
			case 101: // pole pairs
				if (command == TMCL_SAP)
				{
					motorConfig.polePairs = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(DEFAULT_MC, TMC4671_MOTOR_TYPE_N_POLE_PAIRS, TMC4671_N_POLE_PAIRS_MASK, TMC4671_N_POLE_PAIRS_SHIFT, motorConfig.polePairs);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getMotorPolePairs();
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.polePairs-(u32)&motorConfig,
						(u8 *)&motorConfig.polePairs, sizeof(motorConfig.polePairs));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.polePairs-(u32)&motorConfig,
						(u8 *)&motorConfig.polePairs, sizeof(motorConfig.polePairs));
				}
				break;
			case 102: // gear ratio
				if (command == TMCL_SAP)
				{
					motorConfig.gearRatio = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.gearRatio;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.gearRatio-(u32)&motorConfig,
						(u8 *)&motorConfig.gearRatio, sizeof(motorConfig.gearRatio));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.gearRatio-(u32)&motorConfig,
						(u8 *)&motorConfig.gearRatio, sizeof(motorConfig.gearRatio));
				}
				break;
			case 103: // wheel diameter
				if (command == TMCL_SAP)
				{
					motorConfig.wheelDiameter = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.wheelDiameter;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.wheelDiameter-(u32)&motorConfig,
						(u8 *)&motorConfig.wheelDiameter, sizeof(motorConfig.wheelDiameter));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.wheelDiameter-(u32)&motorConfig,
						(u8 *)&motorConfig.wheelDiameter, sizeof(motorConfig.wheelDiameter));
				}
				break;
			case 104: // wheel pulses per rotation
				if (command == TMCL_SAP)
				{
					motorConfig.wheelPulsesPerRotation = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.wheelPulsesPerRotation;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.wheelPulsesPerRotation-(u32)&motorConfig,
						(u8 *)&motorConfig.wheelPulsesPerRotation, sizeof(motorConfig.wheelPulsesPerRotation));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.wheelPulsesPerRotation-(u32)&motorConfig,
						(u8 *)&motorConfig.wheelPulsesPerRotation, sizeof(motorConfig.wheelPulsesPerRotation));
				}
				break;
			case 105: // hall sensor offset
				if (command == TMCL_SAP)
				{
					motorConfig.hallOffset = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(DEFAULT_MC, TMC4671_HALL_PHI_E_PHI_M_OFFSET, TMC4671_HALL_PHI_E_OFFSET_MASK, TMC4671_HALL_PHI_E_OFFSET_SHIFT, motorConfig.hallOffset);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.hallOffset;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallOffset-(u32)&motorConfig,
						(u8 *)&motorConfig.hallOffset, sizeof(motorConfig.hallOffset));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallOffset-(u32)&motorConfig,
						(u8 *)&motorConfig.hallOffset, sizeof(motorConfig.hallOffset));
				}
				break;
			case 106: // hall sensor polarity
				if (command == TMCL_SAP)
				{
					motorConfig.hallPolarity = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(DEFAULT_MC, TMC4671_HALL_MODE, TMC4671_HALL_POLARITY_MASK, TMC4671_HALL_POLARITY_SHIFT, motorConfig.hallPolarity);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.hallPolarity;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallPolarity-(u32)&motorConfig,
						(u8 *)&motorConfig.hallPolarity, sizeof(motorConfig.hallPolarity));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallPolarity-(u32)&motorConfig,
						(u8 *)&motorConfig.hallPolarity, sizeof(motorConfig.hallPolarity));
				}
				break;
			case 107: // hall sensor interpolation
				if (command == TMCL_SAP)
				{
					motorConfig.hallInterpolation = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_HALL_MODE, TMC4671_HALL_INTERPOLATION_MASK, TMC4671_HALL_INTERPOLATION_SHIFT, motorConfig.hallInterpolation);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.hallInterpolation;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallInterpolation-(u32)&motorConfig,
						(u8 *)&motorConfig.hallInterpolation, sizeof(motorConfig.hallInterpolation));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallInterpolation-(u32)&motorConfig,
						(u8 *)&motorConfig.hallInterpolation, sizeof(motorConfig.hallInterpolation));
				}
				break;
			case 108: // hall sensor direction
				if (command == TMCL_SAP)
				{
					motorConfig.hallDirection = ActualCommand.Value.Int32;
					TMC4671_FIELD_UPDATE(0, TMC4671_HALL_MODE, TMC4671_HALL_DIRECTION_MASK, TMC4671_HALL_DIRECTION_SHIFT, motorConfig.hallDirection);
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.hallDirection;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallDirection-(u32)&motorConfig,
						(u8 *)&motorConfig.hallDirection, sizeof(motorConfig.hallDirection));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.hallDirection-(u32)&motorConfig,
						(u8 *)&motorConfig.hallDirection, sizeof(motorConfig.hallDirection));
				}
				break;
			case 110: // current regulator bandwidth
				if (command == TMCL_SAP)
				{
					motorConfig.currentRegulatorBandwidth = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = motorConfig.currentRegulatorBandwidth;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.currentRegulatorBandwidth-(u32)&motorConfig,
						(u8 *)&motorConfig.currentRegulatorBandwidth, sizeof(motorConfig.currentRegulatorBandwidth));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.currentRegulatorBandwidth-(u32)&motorConfig,
						(u8 *)&motorConfig.currentRegulatorBandwidth, sizeof(motorConfig.currentRegulatorBandwidth));
				}
				break;
			case 111: // minimum motor current
				if (command == TMCL_SAP)
				{
					motorConfig.minimumMotorCurrent = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.minimumMotorCurrent;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.minimumMotorCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.minimumMotorCurrent, sizeof(motorConfig.minimumMotorCurrent));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.minimumMotorCurrent-(u32)&motorConfig,
						(u8 *)&motorConfig.minimumMotorCurrent, sizeof(motorConfig.minimumMotorCurrent));
				}
				break;
			case 114: // swap motor A and C phase
				if (command == TMCL_SAP)
				{
					motorConfig.swapMotorAAndCPhase = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.swapMotorAAndCPhase;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.swapMotorAAndCPhase-(u32)&motorConfig,
						(u8 *)&motorConfig.swapMotorAAndCPhase, sizeof(motorConfig.swapMotorAAndCPhase));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.swapMotorAAndCPhase-(u32)&motorConfig,
						(u8 *)&motorConfig.swapMotorAAndCPhase, sizeof(motorConfig.swapMotorAAndCPhase));
				}
				break;
			case 115: // motor test modes
				if (command == TMCL_SAP)
				{
					motorConfig.motorTestModes = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.motorTestModes;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.motorTestModes-(u32)&motorConfig,
						(u8 *)&motorConfig.motorTestModes, sizeof(motorConfig.motorTestModes));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.motorTestModes-(u32)&motorConfig,
						(u8 *)&motorConfig.motorTestModes, sizeof(motorConfig.motorTestModes));
				}
				break;
			case 116: // actual speed [rpm]
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_motorVelocity();
				}
				break;
			case 117: // actual speed [m/s]
				if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = sensor_wheelVelocity()*10/36/VelocityScaling;
				}
				break;
			case 118: // actual speed [km/h]
				if (command == TMCL_GAP)
				{
#if defined(INTERNAL_SHORTY)
					ActualReply.Value.Int32 = sensor_wheelMotorVelocity()/VelocityScaling;
#else
					ActualReply.Value.Int32 = sensor_wheelVelocity()/VelocityScaling;
#endif
				}
				break;
			case 130: // min battery voltage
				if (command == TMCL_SAP)
				{
					motorConfig.minBatteryVoltage = ActualCommand.Value.Int32*BATTERYSCALING;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.minBatteryVoltage/BATTERYSCALING;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.minBatteryVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.minBatteryVoltage, sizeof(motorConfig.minBatteryVoltage));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.minBatteryVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.minBatteryVoltage, sizeof(motorConfig.minBatteryVoltage));
				}
				break;
			case 131: // max battery voltage
				if (command == TMCL_SAP)
				{
					motorConfig.maxBatteryVoltage = ActualCommand.Value.Int32*BATTERYSCALING;
					button_updateBatteryStatusFactor();
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.maxBatteryVoltage/BATTERYSCALING;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maxBatteryVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.maxBatteryVoltage, sizeof(motorConfig.maxBatteryVoltage));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.maxBatteryVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.maxBatteryVoltage, sizeof(motorConfig.maxBatteryVoltage));
				}
				break;
			case 132: // cut off voltage
				if (command == TMCL_SAP)
				{
					motorConfig.cutOffVoltage = ActualCommand.Value.Int32*BATTERYSCALING;
					button_updateBatteryStatusFactor();
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.cutOffVoltage/BATTERYSCALING;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.cutOffVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.cutOffVoltage, sizeof(motorConfig.cutOffVoltage));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.cutOffVoltage-(u32)&motorConfig,
						(u8 *)&motorConfig.cutOffVoltage, sizeof(motorConfig.cutOffVoltage));
				}
				break;
			case 133: // battery-saving timer
				if (command == TMCL_SAP)
				{
					button_resetSavingTimer();
					motorConfig.batterySavingTimer = ActualCommand.Value.Int32;
				} else if (command == TMCL_GAP)
				{
					ActualReply.Value.Int32 = motorConfig.batterySavingTimer;
				} else if (command == TMCL_STAP) {
					eeprom_writeConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.batterySavingTimer-(u32)&motorConfig,
						(u8 *)&motorConfig.batterySavingTimer, sizeof(motorConfig.batterySavingTimer));
				} else if (command == TMCL_RSAP) {
					eeprom_readConfigBlock(TMCM_ADDR_MOTOR_CONFIG+(u32)&motorConfig.batterySavingTimer-(u32)&motorConfig,
						(u8 *)&motorConfig.batterySavingTimer, sizeof(motorConfig.batterySavingTimer));
				}
				break;

			// ===== general info =====

			case 220: // supply voltage
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getSupplyVoltage();
				}
				break;
			case 221: // driver temperature
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getMotorTemperature();
				}
				break;
			case 222: // status flags
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = flags_getAllStatusFlags();
				}
				break;
			case 223: // 12V
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getInput12V();
				}
				break;
			case 224: // 6V
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getInput6V();
				}
				break;
			case 225: // 5V
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = bldc_getInput5V();
				}
				break;
			case 226: // actual pedal torque
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = sensor_actualPedalTorque();
				}
				break;
			case 227: // left pedal torque
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = sensor_leftPedalTorque();
				}
				break;
			case 228: //right pedal torque
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = sensor_rightPedalTorque();
				}
				break;
			case 229: // target pedal torque
				if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = sensor_targetpedalTorque();
				}
				break;

			// ===== system info =====

			case 230:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = systemInfo_getMainLoopsPerSecond();
				break;
			case 231:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = systemInfo_getCurrentLoopsPerSecond();
				break;
			case 232:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = systemInfo_getVelocityLoopsPerSecond();
				break;

			// ===== pedal sensor info =====

			case 233:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_pedalCounter();
				break;
			case 234:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_pedalPosition();
				break;
			case 235:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_pedalCounterPer500MSeconds();
				break;
			case 236:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_pedalVelocity();
				break;
			case 237:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_filteredPedalVelocity();
				break;
			case 238:
				if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_filteredPedalVelocityFast();
				break;

			// ===== debugging =====

			case 240: // debug value 0
				if (command == TMCL_SAP)
					debug_setTestVar0(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar0();
				break;
			case 241: // debug value 1
				if (command == TMCL_SAP)
					debug_setTestVar1(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar1();
				break;
			case 242: // debug value 2
				if (command == TMCL_SAP)
					debug_setTestVar2(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar2();
				break;
			case 243: // debug value 3
				if (command == TMCL_SAP)
					debug_setTestVar3(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar3();
				break;
			case 244: // debug value 4
				if (command == TMCL_SAP)
					debug_setTestVar4(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar4();
				break;
			case 245: // debug value 5
				if (command == TMCL_SAP)
					debug_setTestVar5(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar5();
				break;
			case 246: // debug value 6
				if (command == TMCL_SAP)
					debug_setTestVar6(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar6();
				break;
			case 247: // debug value 7
				if (command == TMCL_SAP)
					debug_setTestVar7(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar7();
				break;
			case 248: // debug value 8
				if (command == TMCL_SAP)
					debug_setTestVar8(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar8();
				break;
			case 249:
				// debug value 9
				if (command == TMCL_SAP)
					debug_setTestVar9(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = debug_getTestVar9();
				break;

			case 250: // filter for torque_actual
				if (command == TMCL_SAP)
					sensor_setFltActualTorque(ActualCommand.Value.Int32);
				else if (command == TMCL_GAP)
					ActualReply.Value.Int32 = sensor_getFltActualTorque();
				break;

			case 255: // enable/disable mc & driver
				if (command == TMCL_SAP)
				{
					(ActualCommand.Value.Int32 == 0) ? tmcm_disableDriver() : tmcm_enableDriver();
				} else if (command == TMCL_GAP) {
					ActualReply.Value.Int32 = tmcm_getDriverState();
				}
				break;
			default:
				ActualReply.Status = REPLY_WRONG_TYPE;
				break;
		}
	} else {
		ActualReply.Status = REPLY_INVALID_VALUE;
	}
}

void tmcl_handleGlobalParameter(uint8_t command)
{
	if (ActualCommand.Motor == 0)	// module config values
	{
		// global_parameters bank 0
		switch(ActualCommand.Type)
		{
			case 64:
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = eeprom_readConfigByte(TMCM_ADDR_EEPROM_MAGIC);
				} else if (command == TMCL_SGP) {
					eeprom_writeConfigByte(TMCM_ADDR_EEPROM_MAGIC, ActualCommand.Value.Byte[0]);
				}
				break;
			case 65: // serial baud rate
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.baudrate;
				} else if (command == TMCL_SGP) {
					moduleConfig.baudrate = ActualCommand.Value.Byte[0];
					eeprom_writeConfigByte(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.baudrate - (u32)&moduleConfig, moduleConfig.baudrate);
				}
				break;
			case 66: // serial address
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.serialModuleAddress;
				} else if (command == TMCL_SGP) {
					moduleConfig.serialModuleAddress = ActualCommand.Value.Byte[0];
					eeprom_writeConfigByte(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.serialModuleAddress - (u32)&moduleConfig, moduleConfig.serialModuleAddress);
				}
				break;
			case 69: // CAN bit rate
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.CANBitrate;
				} else if (command == TMCL_SGP) {
					moduleConfig.CANBitrate = ActualCommand.Value.Byte[0];
					eeprom_writeConfigByte(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.CANBitrate - (u32)&moduleConfig, moduleConfig.CANBitrate);
				}
				break;
			case 70:
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.CANSendID;
				} else if (command == TMCL_SGP) {
					moduleConfig.CANSendID = ActualCommand.Value.Int32;
					eeprom_writeConfigBlock(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.CANSendID - (u32)&moduleConfig,
							(u8 *) &moduleConfig.CANSendID, sizeof(moduleConfig.CANSendID));
				}
				break;
			case 71:
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.CANReceiveID;
				} else if (command == TMCL_SGP) {
					moduleConfig.CANReceiveID = ActualCommand.Value.Int32;
					eeprom_writeConfigBlock(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.CANReceiveID - (u32)&moduleConfig,
							(u8 *) &moduleConfig.CANReceiveID, sizeof(moduleConfig.CANReceiveID));
					can_init(moduleConfig.CANBitrate, moduleConfig.CANReceiveID, moduleConfig.CANSecondaryID);
				}
				break;
			case 76: // serial host address
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.serialHostAddress;
				} else if (command == TMCL_SGP) {
					moduleConfig.serialHostAddress = ActualCommand.Value.Byte[0];
					eeprom_writeConfigByte(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.serialHostAddress - (u32)&moduleConfig, moduleConfig.serialHostAddress);
				}
				break;
			case 83:
				if (command == TMCL_GGP) {
					ActualReply.Value.Int32 = moduleConfig.CANSecondaryID;
				} else if (command == TMCL_SGP) {
					moduleConfig.CANSecondaryID = ActualCommand.Value.Int32;
					eeprom_writeConfigBlock(TMCM_ADDR_MODULE_CONFIG+(u32)&moduleConfig.CANSecondaryID - (u32)&moduleConfig,
							(u8 *)&moduleConfig.CANSecondaryID, sizeof(moduleConfig.CANSecondaryID));
				}
				break;
			default:
				ActualReply.Status = REPLY_WRONG_TYPE;
				break;
		}
	}
}

/* TMCL command SIO */
void tmcl_setOutput()
{
	switch(ActualCommand.Motor)
	{
    	case 2: // digital_outputs
    		if(ActualCommand.Value.Int32 == 0)
    			tmcm_clearModuleSpecificIOPin(ActualCommand.Type);
    		else
    			tmcm_setModuleSpecificIOPin(ActualCommand.Type);
    		break;
    	default:
    		ActualReply.Status = REPLY_INVALID_VALUE;
    		break;
	}
}

/* TMCL command GIO */
void tmcl_getInput()
{
	switch(ActualCommand.Motor)
	{
		case 0: // digital_inputs
			ActualReply.Value.Int32 = tmcm_getModuleSpecificIOPin(ActualCommand.Type);
    		break;
    	case 1: // analog_inputs
    		ActualReply.Value.Int32 = tmcm_getModuleSpecificADCValue(ActualCommand.Type);
    		break;
    	case 2: // digital outputs
    		ActualReply.Value.Int32 = tmcm_getModuleSpecificIOPinStatus(ActualCommand.Type);
    		break;
    	default:
    		ActualReply.Status = REPLY_INVALID_VALUE;
    		break;
	}
}

/* TMCL command 136 (Get Version) */
void tmcl_getVersion()
{
	uint32_t i;

	switch(ActualCommand.Type)
	{
		case 0:
			TMCLReplyFormat = RF_SPECIAL;
			SpecialReply[0] = moduleConfig.serialHostAddress;
			for(i=0; i<8; i++)
				SpecialReply[i+1]=VersionString[i];
			break;
		case 1:
			ActualReply.Value.Byte[3] = SW_TYPE_HIGH;
			ActualReply.Value.Byte[2] = SW_TYPE_LOW;
			ActualReply.Value.Byte[1] = SW_VERSION_HIGH;
			ActualReply.Value.Byte[0] = SW_VERSION_LOW;
			break;
	    default:
	      ActualReply.Status = REPLY_WRONG_TYPE;
	      break;
	}
}

/* special command to switch to bootloader mode */
void tmcl_boot()
{
	if(ActualCommand.Type==0x81 && ActualCommand.Motor==0x92 &&
			ActualCommand.Value.Byte[3]==0xa3 && ActualCommand.Value.Byte[2]==0xb4 &&
			ActualCommand.Value.Byte[1]==0xc5 && ActualCommand.Value.Byte[0]==0xd6)
	{
		io_disableInterrupts();
		NVIC_DeInit();
		SysTick_ITConfig(DISABLE);
		DMA_Cmd(DMA1_Channel1, DISABLE);
		DMA_DeInit(DMA1_Channel1);
		ADC_DeInit(ADC1);
		io_resetCPU(false);
	}
}

/* TMCL command 255 (Reset) */
void tmcl_softwareReset()
{
	if(ActualCommand.Value.Int32==1234)
		ResetRequested = true;
}

/* reset to factory defaults */
void tmcl_factoryDefault()
{
	if((ActualCommand.Type==0) && (ActualCommand.Motor==0) && (ActualCommand.Value.Int32==1234))
	{
		eeprom_writeConfigByte(TMCM_ADDR_EEPROM_MAGIC, 0);
		io_resetCPU(true);
	}
}
