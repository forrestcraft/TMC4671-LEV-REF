/*
 * TMC4671-LEV-REF_v1.0.h
 *
 *  Created on: 12.11.2019
 *      Author: ED
 */

#ifndef TMC4671_LEV_REF_V10_H
#define TMC4671_LEV_REF_V10_H

	#include "SelectModule.h"

#if DEVICE==TMC4671_LEV_REF_V10

	#include "cpu/STM32F103/stm32f10x_lib.h"
	#include "cpu/STM32F103/stm32f10x_gpio.h"
	#include "cpu/STM32F103/stm32f10x_rcc.h"
	#include "cpu/STM32F103/stm32f10x_can.h"

	#define USE_16_MHZ_CLOCK
	#define USE_ALIVE_LED

	#define USE_CAN_INTERFACE
	#define USE_CAN_PB8_PB9			// used CAN-Pins

	#define USE_UART_INTERFACE
	#define USE_UART1_PB6_PB7		// used UART-Pins

	#define MAX_VELOCITY 			(s32)200000
	#define MAX_ACCELERATION		(s32)100000
	#define MAX_TORQUE 				(int)30000

	#define STD_CUSTOMER
	//#define INTERNAL_SHORTY

	// module number in HEX (1634)
	#define SW_TYPE_HIGH			0x06
	#define SW_TYPE_LOW				0x52

	#define SW_VERSION_HIGH 		1
	#define SW_VERSION_LOW  		0

	#define TMCM_EEPROM_MAGIC 		(u8)0x64 // 100

	#include "../Hal_Definitions.h"
	#include "TMC-API/tmc/ic/TMC4671/TMC4671.h"
	#include "TMC-API/tmc/ic/TMC4671/TMC4671_Variants.h"
	#include "TMC-API/tmc/ic/TMC6200/TMC6200.h"
	#include "TMC-API/tmc/ramp/LinearRamp.h"

	// for temperature monitoring
	#define MIN_CRITICAL_TEMP    	100		// 100�C
	#define MAX_CRITICAL_TEMP     	110		// 110�C

	// for supply monitoring
	#define MAX_SUPPLY_VOLTAGE		560  	// 56.0V
	#define ON__SUPPLY_VOLTAGE		140 	// 14.0V
	#define MIN_SUPPLY_VOLTAGE		120		// 12.0V

	#define VOLTAGE_FACTOR_SUPPLY	740
	#define VOLTAGE_FACTOR_12V		138
	#define VOLTAGE_FACTOR_6V		62		// IC702 output 5V at the moment
	#define VOLTAGE_FACTOR_5V		62

	extern TMC_LinearRamp rampGenerator;

#endif /* DEVICE==TMC4671_LEV_REF_V10 */

#endif /* TMC4671_LEV_REF_V10_H */
