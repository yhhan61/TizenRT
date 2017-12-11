/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
//
// arm_cacheops.c
//
#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <sys/types.h>

#include <arch/armv6/cacheops.h>

#ifdef CONFIG_ARCH_ARM1176

//
// Cache maintenance operations for ARMv6
//
// NOTE: The following functions should hold all variables in CPU registers. Currently this will be
//   ensured using maximum optimation (see cacheops.h).
//
//   The following numbers can be determined (dynamically) using CTR.
//   As long we use the ARM1176JZF-S implementation in the BCM2835 these static values will work:
//

#define DATA_CACHE_LINE_LENGTH		32

void CleanAndInvalidateDataCacheRange(u32 nAddress, u32 nLength)
{
	nLength += DATA_CACHE_LINE_LENGTH;

	while (1) {
		asm volatile("mcr p15, 0, %0, c7, c14,  1"::"r"(nAddress):"memory");

		if (nLength < DATA_CACHE_LINE_LENGTH) {
			break;
		}

		nAddress += DATA_CACHE_LINE_LENGTH;
		nLength -= DATA_CACHE_LINE_LENGTH;
	}
}

#else

//
// Cache maintenance operations for ARMv6
//
// See: ARMv7-A Architecture Reference Manual, Section B4.2.1
//
// NOTE: The following functions should hold all variables in CPU registers. Currently this will be
//   ensured using the register keyword and maximum optimation (see cacheops.h).
//
//   The following numbers can be determined (dynamically) using CTR, CSSELR, CCSIDR and CLIDR.
//   As long we use the Cortex-A7 implementation in the BCM2836 or the Cortex-A53 implementation
//   in the BCM2837 these static values will work:
//

#define L1_DATA_CACHE_LINE_LENGTH	64
#define L2_CACHE_LINE_LENGTH		64
#define DATA_CACHE_LINE_LENGTH_MIN	64	// min(L1_DATA_CACHE_LINE_LENGTH, L2_CACHE_LINE_LENGTH)

void CleanAndInvalidateDataCacheRange(u32 nAddress, u32 nLength)
{
	nLength += DATA_CACHE_LINE_LENGTH_MIN;

	while (1) {
		__asm volatile("mcr p15, 0, %0, c7, c14,  1"::"r"(nAddress):"memory");	// DCCIMVAC

		if (nLength < DATA_CACHE_LINE_LENGTH_MIN) {
			break;
		}

		nAddress += DATA_CACHE_LINE_LENGTH_MIN;
		nLength -= DATA_CACHE_LINE_LENGTH_MIN;
	}
}

#endif
