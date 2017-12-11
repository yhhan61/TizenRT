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
 * arch/arm/src/bcm2835/bcm2835_serial.h
 *
 *   Copyright (C) 2009-2010, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_BCM2835_SERIAL_H
#define __ARCH_ARM_SRC_BCM2835_SERIAL_H

#ifndef REG
#define REG(x) (*(volatile uint32_t *)(x))
#endif
#ifndef BIT
#define BIT(n) (1 << (n))
#endif

/* Register Map */
/*
#define UART_LCON       0x00
#define UART_CON        0x04
#define UART_FCON       0x08
#define UART_MCON       0x0C
#define UART_TRSTAT     0x10
#define UART_ERSTAT     0x14
#define UART_FSTAT      0x18
#define UART_MSTAT      0x1C
#define UART_TXH        0x20
#define UART_RXH        0x24
#define UART_BRDIV      0x28
#define UART_FRACVAL    0x2C
#define UART_INTP       0x30
#define UART_INTSP      0x34
*/
#define AUX_ENABLES		REG(0x20215004)
#define AUX_MU_IO_REG		REG(0x20215040)
#define AUX_MU_IER_REG		REG(0x20215044)
#define AUX_MU_IIR_REG		REG(0x20215048)
#define AUX_MU_LCR_REG		REG(0x2021504C)
#define AUX_MU_MCR_REG		REG(0x20215050)
#define AUX_MU_LSR_REG		REG(0x20215054)
#define AUX_MU_MSR_REG		REG(0x20215058)
#define AUX_MU_SCRATCH		REG(0x2021505C)
#define AUX_MU_CNTL_REG		REG(0x20215060)
#define AUX_MU_STAT_REG		REG(0x20215064)
#define AUX_MU_BAUD_REG		REG(0x20215068)

#define AUX_MU_IER_TX_IRQEN	BIT(1)
#define AUX_MU_IER_RX_IRQEN	BIT(0)

#define AUX_MU_IIR_RX_IRQ	((AUX_MU_IIR_REG & 0x07) == 0x04)
#define AUX_MU_IIR_TX_IRQ	((AUX_MU_IIR_REG & 0x07) == 0x02)

#define AUX_MU_LSR_RX_RDY	(AUX_MU_LSR_REG & BIT(0))
#define AUX_MU_LSR_TX_RDY	(AUX_MU_LSR_REG & BIT(5))

#define AUX_ENABLES_ADDR	0x04
#define AUX_MU_IO_REG_ADDR	0x40
#define AUX_MU_IER_REG_ADDR	0x44
#define AUX_MU_IIR_REG_ADDR	0x48
#define AUX_MU_LCR_REG_ADDR	0x4C
#define AUX_MU_MCR_REG_ADDR	0x50
#define AUX_MU_LSR_REG_ADDR	0x54
#define AUX_MU_CNTL_REG_ADDR	0x60
#define AUX_MU_BAUD_REG_ADDR	0x68

#define UART_IIR_INT_MASK	0x07
#define UART_LSR_RX_MASK	0x1
#define UART_LSR_TX_IDLE_MASK	0x40
#define UART_LSR_TX_EMPTY_MASK	0x20
#define UART_RX_MASK		0xFF

#define AUX_ENABLES_UART	BIT(0)

/* Enumultion & Structure */

typedef enum {
	SB_1 = 0,		/**< One stop bit per frame */
	SB_2,			/**< Two stop bits per frame */
} UART_STOP_BIT;

typedef enum {
	PM_NO = 0,		/**< No Parity */
	PM_ODD = 4,		/**< Odd Parity */
	PM_EVEN,		/**< Even Parity */
	PM_1,			/**< Parity forced, checked as 1 */
	PM_0,			/**< Parity forced, checked as 0 */
} UART_PARITY_MODE;

typedef enum {
	DISABLE_MODE = 0,		/**<  Disabled */
	POLL_MODE,			/**<  Polling mode */
	INT_MODE,			/**<  Interrupt mode */
	DMA_MODE,			/**<  DMA mode */
} UART_MODE;

typedef enum {
	NONE_INT = 0,
	RX_INT = (1 << 0),		/**<  Receive Interrupt     */
	TX_INT = (1 << 1),		/**<  Transmit Interrupt     */
	ALL_INT = 0x3,			/**<  All Interrupts: Receive, Transmit     */
} UART_INTERRUPT;

#define RX_INT_IIR 4
#define TX_INT_IIR 2

typedef enum {
	UART0,
	UART1,
	UART_MAX_CHANNEL,
} UART_CHANNEL;

typedef enum uart_baudrate {
	BAUD_9600 = 9600,
	BAUD_14400 = 14400,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200,
	BAUD_230400 = 230400,
} UART_BAUDRATE;

#endif							/* __ARCH_ARM_SRC_BCM2835_SERIAL_H */
