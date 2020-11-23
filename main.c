/*
 * main.c
 *
 *  Created on: 14.02.2019
 *      Author: ED
 */

#include "hal/system/Cpu.h"
#include "hal/system/IO.h"
#include "hal/system/SysTick.h"
#include "hal/system/SystemInfo.h"
#include "hal/comm/SPI.h"
#include "hal/comm/Eeprom.h"
#include "hal/ic/LIS2DH12.h"
#include "BLDC.h"
#include "TMCL.h"
#include "Led.h"
#include "LedRear.h"
#include "Sensor.h"

#if defined(USE_CAN_INTERFACE)
	#include "hal/comm/CAN.h"
#endif

#if defined(USE_UART_INTERFACE)
	#include "hal/comm/UART.h"
#endif

PIrqFunction CanTxUsbHpIrqVector;      // interrupt vector for CAN-TX or USB
PIrqFunction CanRx0UsbLpIrqVector;     // interrupt vector for CAN-RX0 or USB
PIrqFunction CanRx1IrqVector;          // interrupt vector for CAN-RX1
PIrqFunction CanSceIrqVector;          // interrupt vector for CAN-Error

void setCanUsbInterruptVectors()
{
	CanTxUsbHpIrqVector  = can_TxIrq;
	CanRx0UsbLpIrqVector = can_Rx0Irq;
	CanRx1IrqVector      = can_Rx1Irq;
}

int main()
{
	cpu_init();

	// load default values of the module
	tmcm_initModuleConfig();
	tmcm_initMotorConfig();

	// initialize periphery
	tmcm_initModuleSpecificIO();
	tmcm_initModuleSpecificADC();

	// init CAN/USB vectors
	setCanUsbInterruptVectors();

	// initialize hal functionality
	systick_init();
	spi_init();
	LIS2DH12_spi_init();
	eeprom_initConfig(&moduleConfig, &motorConfig);

	// initialize communication interfaces
#if defined(USE_CAN_INTERFACE)
	can_init(moduleConfig.CANBitrate, moduleConfig.CANReceiveID, moduleConfig.CANSecondaryID);
#endif
#ifdef USE_UART_INTERFACE
	uart_init(moduleConfig.baudrate);
#endif

    // use saved configuration values from EEPROM
    tmcm_updateConfig();

    // init main software functions
	tmcl_init();
    bldc_init();
    LI2DH12_init();
    sensor_init();
    led_init();
    rear_led_init();

	for(;;)
	{
		systemInfo_incMainLoopCounter();

		// process incoming tmcl commands
		tmcl_processCommand();
		bldc_processBLDC();

#ifdef USE_ALIVE_LED
		// I am alive LED
		static uint32_t ledCounterCheckTime = 0;
		if ((systick_getTimer()-ledCounterCheckTime) > 1000)
		{
			tmcm_ledGreenToggle();
			ledCounterCheckTime = systick_getTimer();
		}
#endif

		systemInfo_update(systick_getTimer());
	}
}
