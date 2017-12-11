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
 * arch/arm/src/bcm2835/bcm2835_irq.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
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
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"
#include "bcm2835_timer.h"
#include "bcm2835_serial.h"

#include "chip.h"
#include "arm.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/
typedef struct {
	unsigned long IRQBasic;
	unsigned long Pending1;
	unsigned long Pending2;
	unsigned long FIQCtrl;
	unsigned long Enable1;
	unsigned long Enable2;
	unsigned long EnableBasic;
	unsigned long Disable1;
	unsigned long Disable2;
	unsigned long DisableBasic;
} BCM2835_INTC_REGS;

static volatile BCM2835_INTC_REGS *const pRegs = (BCM2835_INTC_REGS *)(BCM2835_BASE_INTC);
/* Remember which interrupts have been enabled */
static unsigned long enabled[3];

volatile uint32_t *current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void up_irqinitialize(void)
{
	gpio_irqinitialize();

	/* Clear pending bit */
	pRegs->IRQBasic = 0x0;
	pRegs->Pending1 = 0x0;
	pRegs->Pending2 = 0x0;
	up_udelay(10);

	/* Disable all interrupt */
	pRegs->EnableBasic = 0;
	pRegs->Enable1 = 0;
	pRegs->Enable2 = 0;
	up_udelay(10);

	/* disable unknown interrupt */
	ARM_TIMER_CLI = 0;
	AUX_MU_IER_REG = 0; /* disable UART interrupt */

	(void)irqenable();
}

void up_enable_irq(int irq)
{
	unsigned long mask = 1UL << (irq % 32);

	if (irq <= 31) {
		enabled[0] |= mask;
		pRegs->Enable1 = mask;
	} else if (irq <= 63) {
		enabled[1] |= mask;
		pRegs->Enable2 = mask;
	} else if (irq < NR_IRQS) {
		enabled[2] |= mask;
		pRegs->EnableBasic = mask;
	}
}

void up_disable_irq(int irq)
{
	unsigned long mask = 1UL << (irq % 32);

	if (irq <= 31) {
		pRegs->Disable1 = mask;
		enabled[0] &= ~mask;
	} else if (irq <= 63) {
		pRegs->Disable2 = mask;
		enabled[1] &= ~mask;
	} else if (irq < NR_IRQS) {
		pRegs->DisableBasic = mask;
		enabled[2] &= ~mask;
	}
}

static int up_check_irq(int irq)
{
	unsigned long mask = 1UL << (irq % 32);

	if (irq <= 31) {
		pRegs->Enable1 = mask;
		return (enabled[0] & mask);
	} else if (irq <= 63) {
		pRegs->Enable2 = mask;
		return (enabled[1] & mask);
	} else if (irq < NR_IRQS) {
		pRegs->EnableBasic = mask;
		return (enabled[2] & mask);
	}
	return 0;
}

int irq_count[NR_IRQS] = { 0, };

uint32_t *arm_decodeirq(uint32_t *regs)
{
	register unsigned long ulMaskedStatus = pRegs->IRQBasic;
	unsigned int base;
	unsigned long pending;

	pending = 0;

	/* Bits 7 through 0 in IRQBasic represent interrupts 64-71 */
	if (ulMaskedStatus & 0xFF) {
		pending = ulMaskedStatus & 0xFF & enabled[2];
		base = 64;
	} else {
		/* Bit 8 in IRQBasic indicates interrupts in Pending1 (interrupts 31-0) */
		pending = pRegs->Pending1 & enabled[0];
		base = 0;
		if (pending == 0) {
			/* Bit 9 in IRQBasic indicates interrupts in Pending2 (interrupts 63-32) */
			pending = pRegs->Pending2 & enabled[1];
			base = 32;
		}
	}

	while (pending) {
		/* Get index of first set bit */
		unsigned int bit = 31 - __builtin_clz(pending);

		/* Map to IRQ number */
		unsigned int irq = base + bit;

		/* Call interrupt handler, if enabled */
		if ((unsigned)irq < NR_IRQS && up_check_irq(irq) != 0) {
			++irq_count[irq];
#ifdef CONFIG_BCM2835_MULTIPLE_GPIO_USE_ONE_IRQ
			if (irq >= 49 && irq <= 52) {
				if (check_gpio_irq_edge(irq)) {
					arm_doirq(irq, (void *)regs);
				}
			} else
#endif
				arm_doirq(irq, (void *)regs);
		}
		/* Clear bit in bitfield */
		pending &= ~(1UL << bit);
	}
	return regs;
}
