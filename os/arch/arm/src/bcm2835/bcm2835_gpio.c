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
 * arch/arm/src/bcm2835/bcm2835_gpio.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "bcm2835_gpio.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
#define DEFAULT_INPUT (GPIO_INPUT | GPIO_PULLUP)

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Base addresses for each GPIO block */
static const uint32_t g_gpiobase[BCM2835_GPIO_NPORTS] = {
	GPFSEL0_ADDR,
	GPFSEL1_ADDR,
	GPFSEL2_ADDR,
	GPFSEL3_ADDR,
	GPFSEL4_ADDR,
	GPFSEL5_ADDR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void bcm2835_gpio_clear_pending(uint32_t pincfg);

static void setbit_reg32(void *addr, unsigned mask, unsigned set)
{
	unsigned ctrl;
	ctrl = getreg32(addr);
	if (set) {
		ctrl |= mask;
	} else {
		ctrl &= ~mask;
	}
	putreg32(ctrl, addr);
}

static void bcm2835_delay(uint32_t n)
{
	volatile uint32_t i = 0;
	for (i = 0; i < n; i++) ;
}

/****************************************************************************
 * Name: bcm2835_pullup
 *
 * Description:
 *   Set pull-up/down on a GPIO pin.
 *
 ****************************************************************************/
static void bcm2835_pullup(uint32_t cfgset, unsigned port, unsigned int pin)
{
	unsigned pinNum = port * 10 + pin;
	unsigned mask = (1 << (pinNum % 32));
	unsigned offset = pinNum / 32;
	uint32_t cfg;
	uint32_t base = g_gpiobase[port];
	unsigned int clk_addr;
	unsigned int value;

	cfg = (cfgset & GPIO_PUPD_MASK) >> GPIO_PUPD_SHIFT;

	putreg32(cfg, base + GPPUD_ADDR);
	bcm2835_delay(150);
	clk_addr = offset ? GPPUDCLK1_ADDR : GPPUDCLK0_ADDR;

	switch (cfgset & GPIO_PUPD_MASK) {
	case GPIO_PULLDOWN:
	case GPIO_PULLUP:
	case GPIO_FLOAT:
		value = getreg32(base + clk_addr);
		value |= mask;
		putreg32(value, base + clk_addr);
		break;
	default:
		return;
	}

	bcm2835_delay(150);
	putreg32(0, base + clk_addr);
}

/****************************************************************************
 * Name: bcm2835_setintedge
 *
 * Description:
 *
 ****************************************************************************/
static inline void bcm2835_setintedge(unsigned int port, unsigned int pin, int edge)
{
	unsigned gpio = port * 10 + pin;
	unsigned pinNum = gpio & 0xffff;
	unsigned offset = pinNum / 32;
	unsigned mask = (1 << (pinNum % 32));
	unsigned type;
	type = (edge & GPIO_EINT_MASK) == GPIO_EINT_LOW ? 1 : 0;
	setbit_reg32(offset ? (void *)GPLEN1_ADDR : (void *)GPLEN0_ADDR, mask, type);	/* LOW */
	type = (edge & GPIO_EINT_MASK) == GPIO_EINT_HIGH ? 1 : 0;
	setbit_reg32(offset ? (void *)GPHEN1_ADDR : (void *)GPHEN0_ADDR, mask, type);	/* HIGH */
	if ((edge & GPIO_EINT_MASK) == GPIO_EINT_BOTH_EDGE) {
		type = 1;
	} else {
		type = (edge & GPIO_EINT_MASK) == GPIO_EINT_FALLING_EDGE ? 1 : 0;
	}
	setbit_reg32(offset ? (void *)GPFEN1_ADDR : (void *)GPFEN0_ADDR, mask, type);	/* FALLING */
	if ((edge & GPIO_EINT_MASK) == GPIO_EINT_BOTH_EDGE) {
		type = 1;
	} else {
		type = (edge & GPIO_EINT_MASK) == GPIO_EINT_RISING_EDGE ? 1 : 0;
	}
	setbit_reg32(offset ? (void *)GPREN1_ADDR : (void *)GPREN0_ADDR, mask, type);	/* RISING */
#if 0
	setbit_reg32(offset ? (void *)GPAREN1_ADDR : (void *)GPAREN0_ADDR, mask, 0);	/* RISING_ASYNC */
	setbit_reg32(offset ? (void *)GPAFEN1_ADDR : (void *)GPAFEN0_ADDR, mask, 0);	/* FALLING_ASYNC */
#endif
	bcm2835_gpio_clear_pending(gpio);
}

/****************************************************************************
 * Name: bcm2835_intmask
 *
 * Description:
 *   Mask interrupts from GPIO pins.
 *
 ****************************************************************************/
static inline void bcm2835_intmask(unsigned int port, unsigned int pin)
{
	unsigned pinNum = (port * 10 + pin) & 0xffff;
	unsigned offset = pinNum / 32;
	unsigned mask = (1 << (pinNum % 32));
	setbit_reg32(offset ? (void *)GPLEN1_ADDR : (void *)GPLEN0_ADDR, mask, 0);	/* LOW */
	setbit_reg32(offset ? (void *)GPHEN1_ADDR : (void *)GPHEN0_ADDR, mask, 0);	/* HIGH */
	setbit_reg32(offset ? (void *)GPFEN1_ADDR : (void *)GPFEN0_ADDR, mask, 0);	/* FALLING */
	setbit_reg32(offset ? (void *)GPREN1_ADDR : (void *)GPREN0_ADDR, mask, 0);	/* RISING */
	setbit_reg32(offset ? (void *)GPAREN1_ADDR : (void *)GPAREN0_ADDR, mask, 0);	/* RISING_ASYNC */
	setbit_reg32(offset ? (void *)GPAFEN1_ADDR : (void *)GPAFEN0_ADDR, mask, 0);	/* FALLING_ASYNC */
}

/****************************************************************************
 * Name: bcm2835_configinput
 *
 * Description:
 *   COnfigure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/
static inline int bcm2835_configinput(uint32_t cfgset, unsigned int port, unsigned int pin)
{
	uint32_t base;
	unsigned long value, offset;
	base = g_gpiobase[port];
	offset = pin * 3;

	/* Set as input */
	value = getreg32(base);
	value &= ~(0x07 << offset);
	value |= ((GPIO_INPUT & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT) << offset;
	putreg32(value, base);

	/* Disable all interrupt configurations that this pin might have. */
	bcm2835_intmask(port, pin);

	/* Set pull-up/down */
	bcm2835_pullup(cfgset, port, pin);

	return OK;
}

/****************************************************************************
 * Name: bcm2835_configalt
 *
 * Description:
 *   Configure a GPIO alternate function pin based on bit-encoded
 *   description of the pin.
 *
 ****************************************************************************/
static int bcm2835_configalt(uint32_t cfgset, unsigned int port, unsigned int pin, uint32_t alt)
{
	uint32_t base = g_gpiobase[port];
	unsigned long value, offset;
	offset = pin * 3;

	/*
	 * First, configure the port as a generic input so that we have
	 * a known starting point and consistent behavior during the
	 * re-configuration.
	 */
	bcm2835_configinput(DEFAULT_INPUT, port, pin);

	value = getreg32(base);
	value &= ~(0x07 << offset);
	value |= alt << offset;
	putreg32(value, base);

	/* Set pull-up mode */
	bcm2835_pullup(cfgset, port, pin);

//  if(port==5 && pin < 4)
//      bcm2835_setintedge(port, pin, GPIO_EINT_BOTH_EDGE);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/
static inline int bcm2835_configoutput(uint32_t cfgset, unsigned int port, unsigned int pin)
{
	uint32_t base = g_gpiobase[port];
	unsigned long value, offset;
	offset = pin * 3;

	/*
	 * First, configure the port as a generic input so that we have
	 * a known starting point and conssitent behavior during the
	 * re-configuration.
	 */
	bcm2835_configinput(DEFAULT_INPUT, port, pin);

	/* Set the initial value of the output */
	bcm2835_gpiowrite(cfgset, (cfgset & GPIO_VALUE_MASK) ^ GPIO_VALUE_ZERO);

	/* Now, reconfigure the pin as an output */
	value = getreg32(base);
	value &= ~(0x07 << offset);
	value |= ((GPIO_OUTPUT & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT) << offset;
	putreg32(value, base);

	return OK;
}

/****************************************************************************
 * Name: bcm2835_configinterrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description
 *   of the pin
 *
 ****************************************************************************/
static inline int bcm2835_configinterrupt(uint32_t cfgset, unsigned int port, unsigned int pin)
{
	/*
	 * First, configure the port as a generic input so that we have
	 * a known starting point and consistent behavior during the
	 * re-configuration.
	 */
	bcm2835_configinput(cfgset, port, pin);

	/* Set as interrupt */
	bcm2835_setintedge(port, pin, cfgset & GPIO_EINT_MASK);

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
uint8_t bcm2835_gpio_irqvector(uint32_t pincfg)
{
	return BCM2835_IRQ_ID_GPIO_3;
}

void bcm2835_gpio_clear_pending(uint32_t pincfg)
{
	uint8_t pin = (pincfg & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	uint8_t port = (pincfg & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	uint32_t gpio = port * 10 + pin;

	unsigned pinNum = gpio & 0xffff;
	unsigned mask = (1 << (pinNum % 32));
	unsigned offset = pinNum / 32;
	unsigned pend;

	pend = getreg32(offset ? (void *)GPEDS1_ADDR : (void *)GPEDS0_ADDR);
	if (pend & mask) {
		return;
	}

	putreg32(mask, offset ? (void *)GPEDS1_ADDR : (void *)GPEDS0_ADDR);
}

/****************************************************************************
 * Name: bcm2835_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as alternative (GPIO_ALT1-5) function, it must be
 *   unconfigured with bcm2835_unconfiggpio() with the same cfgset first before
 *   it can be set to non-alternative function.
 *
 * Returns:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 ****************************************************************************/
int bcm2835_configgpio(uint32_t cfgset)
{
	int ret = -EINVAL;
	unsigned int pin;
	unsigned int port;

	/* Verify that this hardware supports the selected GPIO port */
	port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	if (port >= BCM2835_GPIO_NPORTS) {
		return ret;
	}

	/* Get the pin number */
	pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

	switch (cfgset & GPIO_FUNC_MASK) {
	case GPIO_INPUT:
		ret = bcm2835_configinput(cfgset, port, pin);
		break;

	case GPIO_EINT:
		ret = bcm2835_configinterrupt(cfgset, port, pin);
		break;

	case GPIO_OUTPUT:
		ret = bcm2835_configoutput(cfgset, port, pin);
		break;

	case GPIO_ALT0:
	case GPIO_ALT1:
//  case GPIO_ALT2:
	case GPIO_ALT3:
	case GPIO_ALT4:
	case GPIO_ALT5:
		ret = bcm2835_configalt(cfgset, port, pin, (cfgset & GPIO_FUNC_MASK) >> GPIO_FUNC_SHIFT);
		/*
		   {
		   int val = getreg32((void *)g_gpiobase[port]);
		   lldbg("GPIO[%d,%d]: %08X\n", port, pin, val);
		   }
		 */
		break;
	}

	return ret;
}

/****************************************************************************
 * Name: bcm2835_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default state.
 *
 * Returns:
 *   OK on success
 *   A negated errno value on invalid port
 *
 ****************************************************************************/
int bcm2835_unconfiggpio(uint32_t cfgset)
{
	cfgset &= (GPIO_PORT_MASK | GPIO_PIN_MASK);

	return bcm2835_configgpio(cfgset);
}

/****************************************************************************
 * Name: bcm2835_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/
void bcm2835_gpiowrite(uint32_t pinset, bool value)
{
	unsigned int pin;
	unsigned int port;
	unsigned int gpio;

	port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	if (port >= BCM2835_GPIO_NPORTS) {
		return;
	}

	pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	gpio = port * 10 + pin;

	if (gpio < BCM2835_GPIO_MAX) {
		void *gpio_base;
		unsigned int regval, offset, mask;
		offset = gpio / 32;
		mask = (1 << (gpio % 32));
		if (value) {
			gpio_base = offset ? (void *)GPSET1_ADDR : (void *)GPSET0_ADDR;
		} else {
			gpio_base = offset ? (void *)GPCLR1_ADDR : (void *)GPCLR0_ADDR;
		}

		regval = getreg32(gpio_base);
		regval |= mask;
		putreg32(regval, gpio_base);
	}
}

/****************************************************************************
 * Name: bcm2835_gpioread
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/
bool bcm2835_gpioread(uint32_t pinset)
{
	unsigned int pin;
	unsigned int port;
	unsigned int gpio;

	port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
	if (port >= BCM2835_GPIO_NPORTS) {
		return false;
	}

	pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
	gpio = port * 10 + pin;
	if (gpio < BCM2835_GPIO_MAX) {
		unsigned int value, high, low;
		void *gpio_base;

		high = gpio / 32;
		low = (gpio % 32);
		gpio_base = high ? (void *)GPLEV1_ADDR : (void *)GPLEV0_ADDR;

		value = getreg32(gpio_base);

		return (value >> low) & 1;
	}
	return false;
}

void gpio_irqinitialize(void)
{
}

#ifdef CONFIG_BCM2835_MULTIPLE_GPIO_USE_ONE_IRQ
static int gpio_irq_port_pin = 0;

int check_gpio_irq_edge(int irq)
{
	static unsigned pos = 0;
	unsigned mask;
	int i;
	int find = 0;

	mask = getreg32((void *)GPEDS0_ADDR);
	putreg32(mask, (void *)GPEDS0_ADDR);

	for (i = 0; i < 32; i++) {
		if (mask & (1 << pos)) {
			find = 1;
			gpio_irq_port_pin = pos;
		}
		pos = (pos + 1) & 0x1F;
		if (find) {
			break;
		}
	}
	return 1;
}

int get_gpio_irq_port_pin(void)
{
	return gpio_irq_port_pin;
}
#endif							/* CONFIG_BCM2835_MULTIPLE_GPIO_USE_ONE_IRQ */
/*
void gpio_print_reg(void)
{
	int j;
	unsigned val;
	for(j=0; j<6; j++)
	{
		val = getreg32((void *)g_gpiobase[j]);
	    lldbg("GPIO[%d]: %08X\n", j, val);
	}

    lldbg("GPEDS0_ADDR: %08X\n", val);
	val = getreg32((void *)GPEDS1_ADDR);
    lldbg("GPEDS1_ADDR: %08X\n", val);
	val = getreg32((void *)GPREN0_ADDR);
    lldbg("GPREN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPREN1_ADDR);
    lldbg("GPREN1_ADDR: %08X\n", val);
	val = getreg32((void *)GPFEN0_ADDR);
    lldbg("GPFEN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPFEN1_ADDR);
    lldbg("GPFEN1_ADDR: %08X\n", val);
	val = getreg32((void *)GPHEN0_ADDR);
    lldbg("GPHEN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPHEN1_ADDR);
    lldbg("GPHEN1_ADDR: %08X\n", val);
	val = getreg32((void *)GPLEN0_ADDR);
    lldbg("GPLEN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPLEN1_ADDR);
    lldbg("GPLEN1_ADDR: %08X\n", val);
	val = getreg32((void *)GPAREN0_ADDR);
    lldbg("GPAREN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPAREN1_ADDR);
    lldbg("GPAREN1_ADDR: %08X\n", val);
	val = getreg32((void *)GPAFEN0_ADDR);
    lldbg("GPAFEN0_ADDR: %08X\n", val);
	val = getreg32((void *)GPAFEN1_ADDR);
    lldbg("GPAFEN1_ADDR: %08X\n", val);
}
*/
