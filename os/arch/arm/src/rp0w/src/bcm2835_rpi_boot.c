/*****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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
/*****************************************************************************
 * arch/arm/src/rp0w/src/bcm2835_boot.c
 *
 *   Copyright (C) 2009, 2011, 2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
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

/*****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <debug.h>
#include <assert.h>

#include <tinyara/gpio.h>

#include "up_arch.h"
#include "bcm2835_gpio.h"

#define	SDIO_GPIO_SWITCHING

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gpio_initialize
 *
 * Description:
 *  Expose board dependent GPIOs
 ****************************************************************************/
static void board_gpio_initialize(void)
{
#ifdef CONFIG_GPIO
	int i;
	struct gpio_lowerhalf_s *lower;

	struct {
		uint8_t minor;
		uint16_t pincfg;
	} pins[] = {
		{
			0, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN0
		},	/* BCM2835_GPIO_O0 */
		{
			1, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1
		},	/* BCM2835_GPIO_O1 */
		{
			2, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN2
		},	/* BCM2835_GPIO_O2 */
		{
			3, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN3
		},	/* BCM2835_GPIO_O3 */
		{
			4, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN4
		},	/* BCM2835_GPIO_O4 */
		{
			5, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5
		},	/* BCM2835_GPIO_O5 */
		{
			6, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN6
		},	/* BCM2835_GPIO_O6 */
		{
			7, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN7
		},	/* BCM2835_GPIO_O7 */
		{
			8, GPIO_INPUT | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8
		},	/* BCM2835_GPIO_O8 */
		{
			9, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT0 | GPIO_PIN9
		},	/* BCM2835_GPIO_O9 */
		{
			10, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN0
		},	/* BCM2835_GPIO_10 */
		{
			11, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN1
		},	/* BCM2835_GPIO_11 */
		{
			12, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN2
		},	/* BCM2835_GPIO_12 */
		{
			13, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN3
		},	/* BCM2835_GPIO_13 */
#if 0
		{
			14, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN4
		},	/* BCM2835_GPIO_14 */
		{
			15, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN5
		},	/* BCM2835_GPIO_15 */
#endif
		{
			16, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN6
		},	/* BCM2835_GPIO_16 */
		{
			17, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN7
		},	/* BCM2835_GPIO_17 */
		{
			18, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN8
		},	/* BCM2835_GPIO_18 */
		{
			19, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT1 | GPIO_PIN9
		},	/* BCM2835_GPIO_19 */
		{
			20, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN0
		},	/* BCM2835_GPIO_20 */
		{
			21, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN1
		},	/* BCM2835_GPIO_21 */
		{
			22, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN2
		},	/* BCM2835_GPIO_22 */
		{
			23, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN3
		},	/* BCM2835_GPIO_23 */
		{
			24, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN4
		},	/* BCM2835_GPIO_24 */
		{
			25, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN5
		},	/* BCM2835_GPIO_25 */
		{
			26, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN6
		},	/* BCM2835_GPIO_26 */
		{
			27, GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORT2 | GPIO_PIN7
		},	/* BCM2835_GPIO_27 */
#ifdef	SDIO_GPIO_SWITCHING
		{
			34, GPIO_ALT3 | GPIO_FLOAT | GPIO_PORT3 | GPIO_PIN4
		},	/* BCM2835_GPIO_34 */
		{
			35, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN5
		},	/* BCM2835_GPIO_35 */
		{
			36, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN6
		},	/* BCM2835_GPIO_36 */
		{
			37, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN7
		},	/* BCM2835_GPIO_37 */
		{
			38, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN8
		},	/* BCM2835_GPIO_38 */
		{
			39, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT3 | GPIO_PIN9
		},	/* BCM2835_GPIO_39 */
		{
			48, GPIO_ALT0 | GPIO_FLOAT | GPIO_PORT4 | GPIO_PIN8
		},	/* BCM2835_GPIO_48 */
		{
			49, GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN9
		},	/* BCM2835_GPIO_49 */
		{
			50, GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN0
		},	/* BCM2835_GPIO_50 */
		{
			51, GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN1
		},	/* BCM2835_GPIO_51 */
		{
			52, GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN2
		},	/* BCM2835_GPIO_52 */
		{
			53, GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN3
		},	/* BCM2835_GPIO_53 */
#else

		{
			48, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN8
		},
		{
			49, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT4 | GPIO_PIN9
		},
		{
			50, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN0
		},
		{
			51, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN1
		},
		{
			52, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN2
		},
		{
			53, GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT5 | GPIO_PIN3
		},

#endif
	};

	for (i = 0; i < sizeof(pins) / sizeof(*pins); i++) {
		lower = bcm2835_gpio_lowerhalf(pins[i].pincfg);
		gpio_register(pins[i].minor, lower);

	#ifdef	SDIO_GPIO_SWITCHING
		if((pins[i].minor >= 34 && pins[i].minor < 40)
			|| (pins[i].minor >= 48 && pins[i].minor < 54)) {
			bcm2835_configgpio(pins[i].pincfg);
		}
	#else
		if(pins[i].minor >= 48 && pins[i].minor < 54) {
			bcm2835_configgpio(pins[i].pincfg);
		}
	#endif
	}
#endif							/* CONFIG_GPIO */
}

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *  Expose board dependent I2Cs
 ****************************************************************************/
static void board_i2c_initialize(void)
{
#ifdef CONFIG_I2C
	bcm2835_i2c_register(0);
	bcm2835_i2c_register(1);
#endif
}

/*****************************************************************************
 * Public Functions
 ****************************************************************************/
static void rp0w_clear_bootcount(void)
{
#ifdef CONFIG_RP0W_BOOT_COUNTS_ADDR
	/*
	 * As BL2 sets up a watchdog before it jumps to secondary OS,
	 * we should disable the watchdog to prevent it from barking.
	 *
	 * FIXME: use watchdog driver rather than accessing SFR directly
	 */
	putreg32(0, 0x80030000);

	/* then, clear the boot count */
	putreg32(0, CONFIG_RP0W_BOOT_COUNTS_ADDR);
#endif
}

/****************************************************************************
 * Name: bcm2835_board_initialize
 *
 * Description:
 *   All BCM2835 architectures must provide the following entry point. This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 *   This function must perform low level initialization including:
 *
 *   - Initialization of board-specific memory resources (e.g., SDRAM)
 *   - Configuration of board-specific resources (GPIOs, LEDs, etc).
 *   - Setup of the console UART. This UART done early so that the serial
 *     console is available for debugging very early in the boot sequence.
 *
 *   Special precautions must be taken if .data/.bss lie in SRAM. In that
 *   case, the boot logic cannot initialize .data or .bss. The function
 *   must then:
 *
 *   - Take precautions to assume that logic does not access any global
 *     data that might lie in SDRAM.
 *   - Call the function arm_data_initialize() as soon as SDRAM has been
 *     properly configured for use.
 *
 ****************************************************************************/
void bcm2835_board_initialize(void)
{
#ifdef CONFIG_ARCH_USE_MMU
	bcm2835_mmu_init();
#endif

#ifdef CONFIG_BCM2835_SFLASH
	/* BCM2835 use SPI Serial flash, do not need to init any device */
#endif

#ifdef HAVE_SDIO
	/* Initialize the SDIO block driver */
	if (bcm2835_sdio_initialize() != OK) {
		lldbg("Failed to initialize MMC/SD driver\n");
		return ret;
	}
#endif

#ifdef HAVE_USBHOST
	/* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
	 * will monitor for USB connection and disconnection events.
	 */
	if (bcm2835_usbhost_initialize() != OK) {
		lldbg("Failed to initialize USB host\n");
		return ret;
	}
#endif

#ifdef HAVE_USBMONITOR
	/* Start the USB Monitor */
	if (usbmonitor_start(0, NULL) != OK) {
		udbg("Start USB monitor\n");
	}
#endif
}

#ifdef CONFIG_BOARD_INITIALIZE
/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/
//extern void gpio_print_reg(void);
void board_initialize(void)
{
	rp0w_clear_bootcount();

	/* Perform app-specific initialization here instaed of from the TASH. */
	board_app_initialize();

#ifdef CONFIG_BCM2835_PWM
	board_pwm_setup();
#endif

	board_gpio_initialize();

#ifdef CONFIG_BCM2835_I2C
	board_i2c_initialize();
#endif

#ifdef CONFIG_BRCM_WLAN
	slsi_driver_initialize();
#endif
}
#endif							/* CONFIG_BOARD_INITIALIZE */
