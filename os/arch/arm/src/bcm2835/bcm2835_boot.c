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
 * arch/arm/src/bcm2835/bcm2835_boot.c
 *
 *   Copyright (C) 2009-2010, 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/init.h>

#include "up_arch.h"
#include "up_internal.h"

#include <chip.h>
#include "bcm2835_watchdog.h"
#include "arm.h"
#include "bcm2835_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef USE_GPIO24_AS_LED_BLINK
static void bcm2835_led_on(void)
{
	bcm2835_configgpio(GPIO_OUTPUT | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN4);
	bcm2835_gpiowrite(24, 1);
}

void bcm2835_onboard_led_blink(void)
{
	static int i = 0;
	if (i == 0) {
		bcm2835_configgpio(GPIO_OUTPUT | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN4);
	} else if (i == 500000) {
		bcm2835_gpiowrite(24, 1);
	} else if (i == 2000000) {
		bcm2835_gpiowrite(24, 0);
		i = 0;
	}
	i++;
}
#endif

void arm_boot(void)
{
#ifdef USE_GPIO24_AS_LED_BLINK
	bcm2835_led_on();
#endif

	/* Disable the watchdog timer */
	bcm2835_watchdog_disable();

#ifdef USE_EARLYSERIALINIT
	up_earlyserialinit();
	up_lowputc('#');
#endif

	/*
	 * Perform board-specific initialization. This must include:
	 *
	 * - Initialization of board-specific memory resources (e.g., SDRAM)
	 * - Configuration of board specific resources (GPIOs, LEDs, etc).
	 *
	 * NOTE: we must use caution prior to this point to make sure that
	 * the logic does not access any global variables that might lie
	 * in SDRAM.
	 */
	bcm2835_board_initialize();

	os_start();
}

