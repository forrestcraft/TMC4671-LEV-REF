/*
 * Flags.h
 *
 *  Created on: 31.03.2019
 *      Author: ED
 */

#ifndef FLAGS_H
#define FLAGS_H

	#include "Hal_Definitions.h"

	// error- and status flags
	#define OVERCURRENT             0x00000001	// 0
	#define UNDERVOLTAGE            0x00000002	// 1
	#define OVERVOLTAGE             0x00000004	// 2
	#define OVERTEMPERATURE         0x00000008  // 3

	#define MOTORHALTED             0x00000010	// 4
	#define HALLERROR               0x00000020	// 5
	#define DRIVER_ERROR			0x00000040	// 6
	#define INIT_ERROR				0x00000080	// 7

	#define STOP_MODE			 	0x00000100	// 8
	#define VELOCITY_MODE			0x00000200	// 9
	#define POSITION_MODE           0x00000400	// 10
	#define TORQUE_MODE				0x00000800	// 11

	#define EMERGENCYSTOP           0x00001000	// 12
	#define FREERUNNING             0x00002000	// 13
	#define POSITION_END            0x00004000	// 14
	#define MODULE_INITIALIZED		0x00008000	// 15

  //#define UNUSED					0x00010000	// 16
	#define IIT_EXCEEDED			0x00020000	// 17
	#define BRAKE_ACTIVE			0x00040000	// 18
  //#define UNUSED					0x00080000	// 19

	#define HOMED					0x00100000	// 20
	#define HOMING					0x00200000	// 21
	#define MIN_POS_LIMIT			0x00400000	// 22
	#define MAX_POS_LIMIT			0x00800000	// 23

	void flags_setStatusFlag(u32 flag);
	void flags_clearStatusFlag(u32 flag);
	void flags_setStatusFlagEnabled(u32 flag, bool enabled);
	u8 flags_isStatusFlagSet(u32 flag);
	void flags_resetErrorFlags();
	u32 flags_getAllStatusFlags();
	u32 flags_getErrorFlags();

#endif // FLAGS_H
