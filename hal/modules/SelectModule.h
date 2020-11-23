/*
 * SelectModule.h
 *
 *  Created on: 12.11.2019
 *      Author: ED
 */

#ifndef SELECT_MODULE_H
#define SELECT_MODULE_H

	// define module type
	#define TMC4671_LEV_REF_V10		1

	// select the actual module
	#define DEVICE TMC4671_LEV_REF_V10

#if DEVICE==TMC4671_LEV_REF_V10

	/* device configuration for TMC4671-LEV-REF hardware version v1.0 */
	#include "TMC4671-LEV-REF_v1.0.h"

#else
	/* device not found */
	#error "Device not defined!"
#endif

#endif /* SELECT_MODULE_H */
