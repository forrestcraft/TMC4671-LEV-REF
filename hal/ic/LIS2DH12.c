/*
 * LIS2DH12.c
 *
 *  Created on: 07.02.2019
 *      Author: MO
 */
#include "LIS2DH12.h"

	// forward declaration
	void LIS2DH12_interrupt();

// SPI configuration
void LIS2DH12_spi_init()
{
	// enable clock for SPI?
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// enable clock for GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// use pins A5..B7 for SPI1
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// initialize SPI1
	SPI_InitTypeDef SPIInit;
	SPIInit.SPI_Direction			=	SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode				=	SPI_Mode_Master;
	SPIInit.SPI_DataSize			=	SPI_DataSize_8b;
	SPIInit.SPI_CPOL				=	SPI_CPOL_Low;
	SPIInit.SPI_CPHA				=	SPI_CPHA_2Edge;
	SPIInit.SPI_NSS					=	SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler	=	SPI_BaudRatePrescaler_8;
	SPIInit.SPI_FirstBit			=	SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial		=	0;
	SPI_Init(SPI1, &SPIInit);

	SPI_Cmd(SPI1, ENABLE);
}

u8 LIS2DH12_spi_readWriteByte(u8 data, u8 lastTransfer)
{
	SPI_TypeDef *spiChannel = SPI1;

	// set CS signal low
	tmcm_enableCsLIS2DH12();

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	u8 dataST = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsLIS2DH12();

	return dataST;
}

void LIS2DH12_spi_writeInt(uint8_t address, int32_t value)
{
	LIS2DH12_spi_readWriteByte(address & 0x3F, false);
	LIS2DH12_spi_readWriteByte(0xFF & (value>>0), true);
}

int LIS2DH12_spi_readInt(uint8_t address)
{
	LIS2DH12_spi_readWriteByte(address | 0x80, false);
	int value = LIS2DH12_spi_readWriteByte(0, true);
	return value & 0xFF;
}

void LI2DH12_init()
{
	LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG0, 0x90);			// pull down on SDO

	if (LIS2DH12_spi_readInt(LIS2DH12_WHO_AM_I) == WHO_AM_I)
	{
		LIS2DH12_spi_writeInt(LIS2DH12_TEMP_CFG_REG, 0xC0);		// temperature sensor enabled
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG1, 0x2F);		// 10Hz; low power mode; XYZ enabled
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG2, 0x0);			// filter disabled
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG3, 0x0);			// interrupts disabled
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG4, 0x0);			// +- 2.0g enabled; 4-wire SPI enabled
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG5, 0x0);
		LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG6, 0x0);
	}
	LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG0, 0x90);			// pull down on SDO

	LIS2DH12_interrupt();
}

void LIS2DH12_interrupt()
{
	LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG1, 0x2F);			// 10Hz; low power mode; XYZ enabled
	LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG2, 0x19);			// filter for INT1 & data
	LIS2DH12_spi_writeInt(LIS2DH12_CTRL_REG3, 0x40);			// interrupt enabled
	LIS2DH12_spi_writeInt(LIS2DH12_INT1_CFG, 0x02);				// interrupt on X high
	LIS2DH12_spi_writeInt(LIS2DH12_INT1_THS, 0x05);				// threshold
	LIS2DH12_spi_writeInt(LIS2DH12_INT1_DURATION, 0x01);		// duration
}
