/*
 * Eeprom.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#ifndef EEPROM_H
#define EEPROM_H

	#include "../Hal_Definitions.h"

	void eeprom_initConfig(TModuleConfig *moduleConfig, TMotorConfig *motorConfig);
	void eeprom_writeConfigByte(uint32_t address, uint8_t value);
	uint8_t eeprom_readConfigByte(uint32_t address);
	void eeprom_writeConfigBlock(uint32_t address, uint8_t *block, uint32_t size);
	void eeprom_readConfigBlock(uint32_t address, uint8_t *block, uint32_t size);

#endif
