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
/****************************************************************************
 * arch/arm/src/bcm2835/bcm2835_watchdog.c
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The Samsung sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 *   Copyright (c) 2016 Samsung Electronics, Inc.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <stdio.h>
#include <stddef.h>
#include <sys/types.h>
#include <string.h>

#include "up_arch.h"

#include "bcm2835_watchdog.h"
#include <arch/irq.h>
#include <chip.h>
#include <tinyara/clock.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2835_watchdog_disable
 *
 * Description:
 *   Disable the watchdog timer.
 *
 ****************************************************************************/
void bcm2835_watchdog_disable(void)
{
	PM_RSTC = PM_PASSWORD | PM_RSTC_RESET;
}

#ifdef CONFIG_BCM2835_WATCHDOG
/****************************************************************************
 * Name: bcm2835_watchdog_start
 *
 * Description:
 *   Start watchdog operation .
 *   timeout - tick counts which occurs reset after it [1 tick == 1 ms]
 *
 ****************************************************************************/
void bcm2835_watchdog_start(uint32_t timeout)
{
	/* Setup watchdog for reset */
	uint32_t pm_rstc = PM_RSTC;

	//* watchdog timer = timer clock / 16; need password (31:16) + value (11:0) */
	uint32_t pm_wdog = PM_PASSWORD | (timeout & PM_WDOG_TIME_SET);
	pm_rstc = PM_PASSWORD | (pm_rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET;
	PM_WDOG = pm_wdog;
	PM_RSTC = pm_rstc;
}

/****************************************************************************
 * Name: bcm2835_watchdog_get_remaining
 *
 * Description:
 *   Return remaining watchdog tick. [1 tick == 1 ms]
 *
 ****************************************************************************/
uint32_t bcm2835_watchdog_get_remaining(void)
{
	return PM_WDOG & PM_WDOG_TIME_SET;
}
#endif
