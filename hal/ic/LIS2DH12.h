/*
 * LIS2DH12.h
 *
 *  Created on: 07.02.2019
 *      Author: MO
 */
#ifndef LIS2DH12_H
#define LIS2DH12_H

	#include "../Hal_Definitions.h"

	// ===== LIS2DH12 register set =====

	#define LIS2DH12_STATUS_REG_AUX				0x07
	#define LIS2DH12_OUT_TEMP_L					0x0C
	#define LIS2DH12_OUT_TEMP_H					0x0D
	#define LIS2DH12_WHO_AM_I					0x0F
	#define LIS2DH12_CTRL_REG0					0x1E
	#define LIS2DH12_TEMP_CFG_REG				0x1F
	#define LIS2DH12_CTRL_REG1					0x20
	#define LIS2DH12_CTRL_REG2					0x21
	#define LIS2DH12_CTRL_REG3					0x22
	#define LIS2DH12_CTRL_REG4					0x23
	#define LIS2DH12_CTRL_REG5					0x24
	#define LIS2DH12_CTRL_REG6					0x25
	#define LIS2DH12_REFERENCE					0x26
	#define LIS2DH12_STATUS_REG					0x27
	#define LIS2DH12_OUT_X_L					0x28
	#define LIS2DH12_OUT_X_H    		        0x29
	#define LIS2DH12_OUT_Y_L            		0x2A
	#define LIS2DH12_OUT_Y_H            		0x2B
	#define LIS2DH12_OUT_Z_L           			0x2C
	#define LIS2DH12_OUT_Z_H            		0x2D
	#define LIS2DH12_FIFO_CTRL_REG				0x2E
	#define LIS2DH12_FIFO_SRC_REG				0x2F
	#define LIS2DH12_INT1_CFG					0x30
	#define LIS2DH12_INT1_SRC					0x31
	#define LIS2DH12_INT1_THS					0x32
	#define LIS2DH12_INT1_DURATION				0x33
	#define LIS2DH12_ACT_THS					0x3E
	#define LIS2DH12_INACT_DUR					0x3F

	#define WHO_AM_I	0x33

	void LI2DH12_init();
	void LIS2DH12_spi_init();
	int LIS2DH12_spi_readInt(uint8_t address);
	void LIS2DH12_spi_writeInt(uint8_t address, int32_t value);

#endif /* LIS2DH12_H */
