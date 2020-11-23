/*
 * CAN.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#include "CAN.h"

#if defined(USE_CAN_INTERFACE)

#define CAN_INTR_PRI 7
#define CAN_TX_BUF_SIZE 8
#define CAN_RX_BUF_SIZE 8

volatile int CanRxWritePtr, CanRxReadPtr, CanTxWritePtr, CanTxReadPtr;
volatile TCanFrame CanTxBuffer[CAN_TX_BUF_SIZE];
volatile TCanFrame CanRxBuffer[CAN_RX_BUF_SIZE];

/* CAN-Receive-Interrupt Handler for Receive FIFO 0 */
void can_Rx0Irq()
{
	CanRxMsg CanMessage;
	int i, j;

	i = CanRxWritePtr+1;
	if(i==CAN_RX_BUF_SIZE)
		i=0;

	if(i != CanRxReadPtr)  // read data if some space in buffer available
	{
		CAN_Receive(CAN_FIFO0, &CanMessage);

		if(CanMessage.IDE==CAN_ID_EXT)
		{
			CanRxBuffer[CanRxWritePtr].Ext = true;
			CanRxBuffer[CanRxWritePtr].Id = CanMessage.ExtId;
		}
		else
		{
			CanRxBuffer[CanRxWritePtr].Ext = false;
			CanRxBuffer[CanRxWritePtr].Id = CanMessage.StdId;
		}

		if(CanMessage.RTR==CAN_RTR_REMOTE)
			CanRxBuffer[CanRxWritePtr].Rtr = true;
		else
			CanRxBuffer[CanRxWritePtr].Rtr = false;

		CanRxBuffer[CanRxWritePtr].Dlc = CanMessage.DLC;
		for(j=0; j<8; j++)
			CanRxBuffer[CanRxWritePtr].Data[j] = CanMessage.Data[j];

		CanRxWritePtr = i;
	}
	else
	{
		// release mailbox if no space left
		CAN_FIFORelease(CAN_FIFO0);
	}
}


/* CAN-Receive-Interrupt Handler for Receive FIFO 1 */
void can_Rx1Irq()
{
	CanRxMsg CanMessage;
	int i, j;

	i = CanRxWritePtr+1;
	if(i==CAN_RX_BUF_SIZE)
		i=0;

	if(i != CanRxReadPtr)  // read data if some space in buffer available
	{
		CAN_Receive(CAN_FIFO1, &CanMessage);

		if(CanMessage.IDE==CAN_ID_EXT)
		{
			CanRxBuffer[CanRxWritePtr].Ext = true;
			CanRxBuffer[CanRxWritePtr].Id=CanMessage.ExtId;
		}
		else
		{
			CanRxBuffer[CanRxWritePtr].Ext = false;
			CanRxBuffer[CanRxWritePtr].Id=CanMessage.StdId;
		}

		if(CanMessage.RTR == CAN_RTR_REMOTE)
			CanRxBuffer[CanRxWritePtr].Rtr = true;
		else
			CanRxBuffer[CanRxWritePtr].Rtr = false;

		CanRxBuffer[CanRxWritePtr].Dlc = CanMessage.DLC;
		for(j=0; j<8; j++)
			CanRxBuffer[CanRxWritePtr].Data[j]=CanMessage.Data[j];

		CanRxWritePtr = i;
	}
	else
	{
		// release mailbox if no space left
		CAN_FIFORelease(CAN_FIFO1);
	}
}


/* CAN-Transmit-Interrupt Handler */
void can_TxIrq()
{
	CanTxMsg CanMessage;
	int i;

	if(CanTxReadPtr==CanTxWritePtr)  // buffer empty?
	{
		CAN_ITConfig(CAN_IT_TME, DISABLE);
	}
	else
	{
		if(CanTxBuffer[CanTxReadPtr].Ext)
		{
			CanMessage.IDE = CAN_ID_EXT;
			CanMessage.StdId = 0;
			CanMessage.ExtId = CanTxBuffer[CanTxReadPtr].Id;
		}
		else
		{
			CanMessage.IDE = CAN_ID_STD;
			CanMessage.StdId = CanTxBuffer[CanTxReadPtr].Id;
			CanMessage.ExtId = 0;
		}

		if(CanTxBuffer[CanTxReadPtr].Rtr)
			CanMessage.RTR = CAN_RTR_REMOTE;
		else
			CanMessage.RTR = CAN_RTR_DATA;

		CanMessage.DLC = CanTxBuffer[CanTxReadPtr].Dlc;
		for(i=0; i<8; i++)
			CanMessage.Data[i]=CanTxBuffer[CanTxReadPtr].Data[i];

		CanTxReadPtr++;
		if(CanTxReadPtr == CAN_TX_BUF_SIZE)
			CanTxReadPtr=0;

		CAN_Transmit(&CanMessage);
	}
}

/* initialize can interface 									*/
/* baudrate: CAN-Bitrate  1  2  3  4   5   6   7   8			*/
/*                        10 20 50 100 125 250 500 1000kBit/s 	*/
void can_init(int Bitrate, int ReceiveID, int SecondaryID)
{
	CAN_InitTypeDef CANInit;
	CAN_FilterInitTypeDef CANFilterInit;
	GPIO_InitTypeDef GPIOInit;
	NVIC_InitTypeDef NVICInit;

	switch(Bitrate)
	{
		case 8:     // 1000kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 2;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 7:     // 500kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 4;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 6:     // 250kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 8;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 5:     // 125kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 16;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 4:     // 100kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 20;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 3:     // 50kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 40;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 2:     // 20kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 100;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		case 1:     // 10kBit/s: 18 Time Quanta, Sampling Point 72.27%
			CANInit.CAN_Prescaler = 200;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
		default:    // 250kBit/s: 18 Time Quanta, Sampling Point 72.2%
			CANInit.CAN_Prescaler = 8;
			CANInit.CAN_BS1 = CAN_BS1_13tq;
			CANInit.CAN_BS2 = CAN_BS2_4tq;
			CANInit.CAN_SJW = CAN_SJW_1tq;
			break;
	}

	// enable CAN and AFIO clock
	CAN_DeInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

#if defined(USE_CAN_PB8_PB9)
	// connect CAN-Pins (PB8 and PB9)
	// activate GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);
	GPIO_PinRemapConfig(GPIO_Remap1_CAN, ENABLE);

	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInit.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInit.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIOInit);

	GPIOInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIOInit.GPIO_Mode = GPIO_Mode_IPU;
	GPIOInit.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIOInit);

#else
	#error "CAN pins not defined!"
#endif

	// initialize CAN
	CANInit.CAN_Mode = CAN_Mode_Normal;
	CANInit.CAN_TTCM = DISABLE;
	CANInit.CAN_ABOM = ENABLE;
	CANInit.CAN_AWUM = DISABLE;
	CANInit.CAN_NART = DISABLE;
	CANInit.CAN_RFLM = ENABLE;
	CANInit.CAN_TXFP = ENABLE;
	CAN_Init(&CANInit);

	// configure CAN-Filter

	// Filter 0 for Standard Frames with FIFO 0
	CANFilterInit.CAN_FilterNumber = 0;
	CANFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
	CANFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
	CANFilterInit.CAN_FilterIdHigh = ReceiveID << 5;
	CANFilterInit.CAN_FilterIdLow = 0;
	CANFilterInit.CAN_FilterMaskIdHigh = 0xffff;
	CANFilterInit.CAN_FilterMaskIdLow = 0xffff;
	CANFilterInit.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
	CANFilterInit.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CANFilterInit);

	// Filter 1 for Standard Frames with FIFO 1
	if(SecondaryID != 0)
	{
		CANFilterInit.CAN_FilterNumber = 1;
		CANFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
		CANFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
		CANFilterInit.CAN_FilterIdHigh = SecondaryID << 5;
		CANFilterInit.CAN_FilterIdLow = 0;
		CANFilterInit.CAN_FilterMaskIdHigh = 0xffff;
		CANFilterInit.CAN_FilterMaskIdLow = 0xffff;
		CANFilterInit.CAN_FilterFIFOAssignment = CAN_FilterFIFO1;
		CANFilterInit.CAN_FilterActivation = ENABLE;
		CAN_FilterInit(&CANFilterInit);
	}

	// Filter 2 for Extended Frames with FIFO 0
	ReceiveID <<= 3;
	CANFilterInit.CAN_FilterNumber = 2;
	CANFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
	CANFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
	CANFilterInit.CAN_FilterIdHigh = ReceiveID >> 16;
	CANFilterInit.CAN_FilterIdLow = ReceiveID | CAN_ID_EXT;
	CANFilterInit.CAN_FilterMaskIdHigh = 0xffff;
	CANFilterInit.CAN_FilterMaskIdLow = 0xffff;
	CANFilterInit.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
	CANFilterInit.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CANFilterInit);

	// Filter 3 for Extended Frames with FIFO 1
	if(SecondaryID > 0)
	{
		SecondaryID <<= 3;
		CANFilterInit.CAN_FilterNumber = 3;
		CANFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
		CANFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
		CANFilterInit.CAN_FilterIdHigh = SecondaryID >> 16;
		CANFilterInit.CAN_FilterIdLow = SecondaryID | CAN_ID_EXT;
		CANFilterInit.CAN_FilterMaskIdHigh = 0xffff;
		CANFilterInit.CAN_FilterMaskIdLow = 0xffff;
		CANFilterInit.CAN_FilterFIFOAssignment = CAN_FilterFIFO1;
		CANFilterInit.CAN_FilterActivation = ENABLE;
		CAN_FilterInit(&CANFilterInit);
	}

	// activate interrupt for CAN RX FIFO 0
	NVICInit.NVIC_IRQChannel = USB_LP_CAN_RX0_IRQChannel;
	NVICInit.NVIC_IRQChannelPreemptionPriority = CAN_INTR_PRI;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	// activate interrupt for CAN RX FIFO 1
	NVICInit.NVIC_IRQChannel = CAN_RX1_IRQChannel;
	NVICInit.NVIC_IRQChannelPreemptionPriority = CAN_INTR_PRI;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	// activate interrupt for CAN TX Mailbox
	NVICInit.NVIC_IRQChannel = USB_HP_CAN_TX_IRQChannel;
	NVICInit.NVIC_IRQChannelPreemptionPriority = CAN_INTR_PRI;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	// configure CAN interrupts
	// (enable only receive interrupts here, send interrupts later)
	CAN_ITConfig(CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN_IT_FMP1, ENABLE);
}


/* send can message */
int can_sendMessage(TCanFrame *Msg)
{
	CanTxMsg CanMessage;
	int i;

	// try to send direct
	if(Msg->Ext)
	{
		CanMessage.IDE = CAN_ID_EXT;
		CanMessage.ExtId = Msg->Id;
		CanMessage.StdId = 0;
	}
	else
	{
		CanMessage.IDE = CAN_ID_STD;
		CanMessage.StdId = Msg->Id;
		CanMessage.ExtId = 0;
	}

	if(Msg->Rtr)
		CanMessage.RTR = CAN_RTR_REMOTE;
	else
		CanMessage.RTR = CAN_RTR_DATA;

	CanMessage.DLC = Msg->Dlc;
	for(i=0; i<8; i++)
		CanMessage.Data[i] = Msg->Data[i];

	if(CAN_Transmit(&CanMessage) != CAN_NO_MB)
		return true;

	// if transmit mailboxes are full: use software buffer, activate Tx-interrupt
	i = CanTxWritePtr+1;
	if(i==CAN_TX_BUF_SIZE)
		i=0;

	// write into buffer, if not full
	if(i != CanTxReadPtr)
	{
		CanTxBuffer[CanTxWritePtr] = *Msg;
		CanTxWritePtr = i;

		// activate transmit interrupt
		CAN_ITConfig(CAN_IT_TME, ENABLE);

		return true;
	}
	else
		return false;
}

/* get CAN message */
int can_getMessage(TCanFrame *Msg)
{
	// is buffer empty?
	if(CanRxReadPtr == CanRxWritePtr)
		return false;

	*Msg = CanRxBuffer[CanRxReadPtr++];
	if(CanRxReadPtr == CAN_RX_BUF_SIZE)
		CanRxReadPtr=0;  // Ringbuffer

	return true;
}

#else
	void can_Rx0Irq() {}
	void CanRx1Irq() {}
	void CanTxIrq() {}
#endif
