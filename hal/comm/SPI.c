/*
 * SPI.c
 *
 *  Created on: 13.11.2019
 *      Author: ED
 */
#include "SPI.h"

void spi_init()
{
	// enable clock for SPI2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// enable clock for GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// use pins B13..B15 for SPI2
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// initialize SPI2
	SPI_InitTypeDef SPIInit;
	SPIInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPIInit.SPI_Mode = SPI_Mode_Master;
	SPIInit.SPI_DataSize = SPI_DataSize_8b;
	SPIInit.SPI_CPOL = SPI_CPOL_High;
	SPIInit.SPI_CPHA = SPI_CPHA_2Edge;
	SPIInit.SPI_NSS = SPI_NSS_Soft;
	SPIInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPIInit.SPI_FirstBit = SPI_FirstBit_MSB;
	SPIInit.SPI_CRCPolynomial = 0;
	SPI_Init(SPI2, &SPIInit);

	SPI_Cmd(SPI2, ENABLE);
}

uint8_t eeprom_spi_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
	SPI_TypeDef *spiChannel = SPI2;

	// set CS signal low
	tmcm_enableCsMem();

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	u8 data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsMem();

	return data2;
}

uint8_t weasel_spi_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
	SPI_TypeDef *spiChannel = SPI2;

	// set CS signal low
	tmcm_enableCsWeasel();

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	u8 data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsWeasel();

	return data2;
}

uint8_t dragon_spi_readWriteByte(uint8_t data, uint8_t lastTransfer)
{
	SPI_TypeDef *spiChannel = SPI2;

	// set CS signal low
	tmcm_enableCsDragon();

	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(spiChannel, data);
	while(SPI_I2S_GetFlagStatus(spiChannel, SPI_I2S_FLAG_RXNE)==RESET);
	u8 data2 = SPI_I2S_ReceiveData(spiChannel);

	// set CS signal high after last transfer
	if(lastTransfer)
		tmcm_disableCsDragon();

	return data2;
}
