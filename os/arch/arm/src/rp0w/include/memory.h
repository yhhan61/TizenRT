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
 * arch/arm/src/rp0w/include/memory.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_RP0W_MEMORY_H
#define __ARCH_ARM_SRC_RP0W_MEMORY_H

// DRAM addresses and sizes
#define MEGABYTE		0x100000
#define BCM2835_DRAM_BASE	0x0
#define BCM2835_DRAM_SIZE	(512 * MEGABYTE)	// default size
#define SDRAM_SIZE_MBYTE	512
#define GPU_MEM_SIZE		(64 * MEGABYTE)	// set in config.txt
#define ARM_MEM_SIZE		(BCM2835_DRAM_SIZE - GPU_MEM_SIZE)	// normally overwritten

#define PAGE_SIZE		4096	// page size used by us
#define BCM2835_PAGE_TABLE1	0x1000000	// must be 16K aligned
#define PAGE_TABLE1_SIZE	0x4000

#define BCM2835_PAGE_TABLE2	(BCM2835_PAGE_TABLE1+PAGE_TABLE1_SIZE)	// must be 16K aligned
#define PAGE_TABLE2_SIZE	0x4000

#define ARM_IO_BASE		0x20000000
#define GPU_IO_BASE		0x7E000000

#define GPU_CACHED_BASE		0x40000000
#define GPU_UNCACHED_BASE	0xC0000000

/* BCM2835 Physical Memory Map */
#define BCM2835_FLASH_PADDR	0x04000000	/* 0x04000000-0x04FFFFFF NOR flash */

/* BCM2835 Internal Peripherals at 0x80000000 */
#define GPIO_CON_BASE		0x20000000

#define BCM2835_UART0_BASE	0x20201000
#define BCM2835_UART1_BASE	0x20215000	/* = AUX_IRQ */
#define SFI_BASE		0x80310000

#define BCM2835_CMU_BASE	0x80080000

// system options
#ifdef CONFIG_ARCH_ARM1176		// valid on Raspberry Pi 1 only
#define ARM_STRICT_ALIGNMENT
#define GPU_L2_CACHE_ENABLED
#endif

#ifdef CONFIG_ARCH_CHIP_RP0W
#ifdef GPU_L2_CACHE_ENABLED
#define GPU_MEM_BASE	GPU_CACHED_BASE
#else
#define GPU_MEM_BASE	GPU_UNCACHED_BASE
#endif
#endif

// Convert physical ARM address into bus address
// (does even work, if a bus address is provided already)
#define BUS_ADDRESS(phys)	(((phys) & ~0xC0000000) | GPU_MEM_BASE)

//
// GPIOs
//
#define BCM2835_GPIO_NPORTS		6
#define BCM2835_GPIO_MAX		54

// Registers
#define GPFSEL0_ADDR 		0x20200000
#define GPFSEL1_ADDR 		0x20200004
#define GPFSEL2_ADDR 		0x20200008
#define GPFSEL3_ADDR 		0x2020000C
#define GPFSEL4_ADDR 		0x20200010
#define GPFSEL5_ADDR 		0x20200014

#define GPSET0_ADDR  		0x2020001C
#define GPSET1_ADDR  		0x20200020
#define GPCLR0_ADDR  		0x20200028
#define GPCLR1_ADDR  		0x2020002C
#define GPLEV0_ADDR		0x20200034
#define GPLEV1_ADDR		0x20200038
#define GPEDS0_ADDR		0x20200040
#define GPEDS1_ADDR		0x20200044
#define GPREN0_ADDR		0x2020004C
#define GPREN1_ADDR		0x20200050
#define GPFEN0_ADDR		0x20200058
#define GPFEN1_ADDR		0x2020005C
#define GPHEN0_ADDR		0x20200064
#define GPHEN1_ADDR		0x20200068
#define GPLEN0_ADDR		0x20200070
#define GPLEN1_ADDR		0x20200074
#define GPAREN0_ADDR		0x2020007C
#define GPAREN1_ADDR		0x20200080
#define GPAFEN0_ADDR		0x20200088
#define GPAFEN1_ADDR		0x2020008C

#define GPPUD_ADDR		0x20200094
#define GPPUDCLK0_ADDR		0x20200098
#define GPPUDCLK1_ADDR		0x2020009C

//
// Mailbox
//
#define MAILBOX_BASE		(ARM_IO_BASE + 0xB880)

#define MAILBOX0_READ  		(MAILBOX_BASE + 0x00)
#define MAILBOX0_STATUS 	(MAILBOX_BASE + 0x18)
#define MAILBOX_STATUS_EMPTY	0x40000000
#define MAILBOX1_WRITE		(MAILBOX_BASE + 0x20)
#define MAILBOX1_STATUS 	(MAILBOX_BASE + 0x38)
#define MAILBOX_STATUS_FULL	0x80000000

#define MAILBOX_CHANNEL_PM	0	// power management
#define MAILBOX_CHANNEL_FB 	1	// frame buffer
#define BCM_MAILBOX_PROP_OUT	8	// property tags (ARM to VC)

//
// PWM
//
#ifndef REG
#define REG(x) (*(volatile uint32_t *)(x))
#endif
#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#define	BCM2835_PWM_BASE	0x2020C000

#define PWM_CTL			REG(0x2020C000)
#define PWM_STATUS		REG(0x2020C004)

#define PWM0_RANGE		REG(0x2020C010)
#define PWM0_DATA		REG(0x2020C014)

#define PWM1_RANGE		REG(0x2020C020)
#define PWM1_DATA		REG(0x2020C024)

#define PWM0_ENABLE		BIT(0)
#define PWM0_MODE_MS		BIT(7)

#define PWM1_ENABLE		BIT(8)
#define PWM1_MODE_MS		BIT(15)

#define PWM_MODE_MS		0xFF
#define GPIO_CLK_PWD		0x5a000000

#define GPIO0_CLK_CTL		REG(0x201010A0)
#define GPIO0_CLK_DIV		REG(0x201010A4)

//
// System Timers
//
#define ARM_SYSTIMER_BASE       (ARM_IO_BASE + 0x3000)

#define ARM_SYSTIMER_CS         (ARM_SYSTIMER_BASE + 0x00)
#define ARM_SYSTIMER_CLO        (ARM_SYSTIMER_BASE + 0x04)
#define ARM_SYSTIMER_CHI        (ARM_SYSTIMER_BASE + 0x08)
#define ARM_SYSTIMER_C0         (ARM_SYSTIMER_BASE + 0x0C)
#define ARM_SYSTIMER_C1         (ARM_SYSTIMER_BASE + 0x10)
#define ARM_SYSTIMER_C2         (ARM_SYSTIMER_BASE + 0x14)
#define ARM_SYSTIMER_C3         (ARM_SYSTIMER_BASE + 0x18)

//
// USB Host Controller
//
#define BCM2835_USB_BASE	(ARM_IO_BASE + 0x980000)

#define BCM2835_USB_CORE_BASE	BCM2835_USB_BASE
#define BCM2835_USB_HOST_BASE	(BCM2835_USB_BASE + 0x400)
#define BCM2835_USB_POWER	(BCM2835_USB_BASE + 0xE00)

#endif							/* __ARCH_ARM_SRC_RP0W_MEMORY_H */
