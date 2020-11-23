/*
 * SPI.h
 *
 *  Created on: 13.11.2019
 *      Author: ED
 */

#ifndef SPI_H
#define SPI_H

	#include "../Hal_Definitions.h"

	void spi_init();

	uint8_t eeprom_spi_readWriteByte(uint8_t data, uint8_t lastTransfer);
	uint8_t weasel_spi_readWriteByte(uint8_t data, uint8_t lastTransfer);
	uint8_t dragon_spi_readWriteByte(uint8_t data, uint8_t lastTransfer);

#endif /* SPI_H */
