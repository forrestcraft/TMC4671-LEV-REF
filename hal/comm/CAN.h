/*
 * CAN.h
 *
 *  Created on: 01.04.2020
 *      Author: OK / ED
 */

#ifndef CAN_H
#define CAN_H

	#include "../Hal_Definitions.h"

	typedef struct {
		unsigned char Dlc, Ext, Rtr;
		unsigned long Id;
		unsigned char Data[8];
	} TCanFrame;

	void can_init(int Bitrate, int ReceiveID, int SecondaryID);
	int can_sendMessage(TCanFrame *Msg);
	int can_getMessage(TCanFrame *Msg);
	void can_TxIrq();
	void can_Rx0Irq();
	void can_Rx1Irq();
#endif
