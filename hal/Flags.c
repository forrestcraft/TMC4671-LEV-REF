/*
 * Flags.c
 *
 *  Created on: 31.03.2019
 *      Author: ED
 */

#include "Flags.h"

volatile u32 statusFlags = 0;	// error and status flags (Overvoltage, Overcurrent,...)

/* set status flags */
void flags_setStatusFlag(u32 flag)
{
	statusFlags |= flag;
}

/* clear status flag */
void flags_clearStatusFlag(u32 flag)
{
	statusFlags &= ~flag;
}

/* clear/set status flag */
void flags_setStatusFlagEnabled(u32 flag, bool enabled)
{
	if (enabled)
		statusFlags |= flag;
	else
		statusFlags &= ~flag;
}

/* check if status flag is set */
u8 flags_isStatusFlagSet(u32 flag)
{
	if(statusFlags & flag)
		return true;
	else
		return false;
}

/* reset error flags */
void flags_resetErrorFlags()
{
	flags_clearStatusFlag(OVERCURRENT|HALLERROR|DRIVER_ERROR);
}

/* get status flags and afterwards reset error flags */
u32 flags_getAllStatusFlags()
{
	u32 flags = statusFlags;
	// reset error flags
	flags_resetErrorFlags();
	return flags;
}

/* get and afterwards reset error flags */
u32 flags_getErrorFlags()
{
	u32 flags = statusFlags;
	u32 mask = OVERCURRENT|UNDERVOLTAGE|OVERTEMPERATURE|HALLERROR|DRIVER_ERROR;

	// return only error flags
	flags &= mask;

	// reset error flags
	flags_resetErrorFlags();
    return flags;
}
