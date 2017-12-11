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
 * arch/arm/src/bcm2835/bcm2835_pwrcal.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stddef.h>
#include <sys/types.h>
#include <string.h>

#include <tinyara/kmalloc.h>
#include <arch/chip/irq.h>
#include <chip.h>

#include "bcm2835_vclk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/
unsigned int cal_clk_is_enabled(unsigned int id)
{
	return 0;
}

/*
 *  This definition of SPI_CLK should be replaced by function
 *  reruning actual CLK frequency
 */
#define SPI_CLK 40000000


/*
 *  This definition of BSC_CLOCK_FREQ should be replaced by function
 *  reruning actual CLK frequency (I2C)
 */
#define BSC_CLOCK_FREQ 150000000

/* This definitions should be moved into the right place */

int cal_clk_setrate(unsigned int id, unsigned long rate)
{
	unsigned long parents;
	unsigned int div;

	switch (id) {
	case d1_spi0:
		parents = SPI_CLK;
		div = parents / rate;
		if (div == 0) {
			div = 1;
		}
		/* not support now, for future use */
		break;
	case d1_spi1:
		/* CLK_CON_DIV_DIV_CLK_SPI1 */
		parents = SPI_CLK;
		div = parents / rate;
		if (div == 0) {
			div = 1;
		}
		/* not support now, for future use */
		break;
	case gate_i2c0:
	case gate_i2c1:
		break;
	default:
		break;
	}

	return -1;
}

unsigned long cal_clk_getrate(unsigned int id)
{
	unsigned long rate = 0;

	switch (id) {
	case d1_spi0:
	case d1_spi1:
		/* not used now */
		break;
	case clk_uart:
		rate = BCM2835_CLOCK_FREQ;
		break;
	case gate_i2c0:
	case gate_i2c1:
		rate = BSC_CLOCK_FREQ;
		break;
	default:
		break;
	}

	return rate;
}

int cal_clk_enable(unsigned int id)
{
	return 0;
}

int cal_clk_disable(unsigned int id)
{
	return 0;
}

int cal_init(void)
{
	return 0;
}
