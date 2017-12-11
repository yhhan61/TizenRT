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
 * arch/arm/src/bcm2835/bcm2835_serial.c
 *
 *   Copyright (C) 2009-2010, 2012-2014 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/serial/serial.h>
#include <tinyara/fs/ioctl.h>

#ifdef CONFIG_SERIAL_TERMIOS
#include <termios.h>
#endif

#include "bcm2835_gpio.h"
#include "up_arch.h"
#include "up_internal.h"
#include "bcm2835_serial.h"
#include "bcm2835_vclk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/*
 * If we are not using the serial driver for the console, then we
 * still must provide some minimal implementation of up_putc.
 */

#undef TTYS0_DEV
#undef TTYS1_DEV

#undef UART0_ASSIGNED
#undef UART1_ASSIGNED

/* Which UART with be ttyS0/console and which ttyS1? ttyS2? ... ttyS4 */

/* First pick the console and ttys0. This could be any of UART0-4 */
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#define CONSOLE_DEV		g_uart0port	/* UART0 is console */
#define TTYS0_DEV		g_uart0port	/* UART0 is ttyS0 */
#define UART0_ASSIGNED	1
#define HAVE_SERIAL_CONSOLE
#else
#define CONSOLE_DEV		g_uart1port	/* UART1 is console */
#define TTYS0_DEV		g_uart1port	/* UART1 is ttyS0 */
#define UART1_ASSIGNED	1
#define HAVE_SERIAL_CONSOLE
#endif

/* Pick ttyS1. This could be any of UART0-4 excluding the console UART. */
#if defined(CONFIG_BCM2835_UART0) && !defined(UART0_ASSIGNED)
#define TTYS1_DEV		g_uart0port	/* UART0 is ttyS1 */
#define UART0_ASSIGNED	1
#endif
#if defined(CONFIG_BCM2835_UART1) && !defined(UART1_ASSIGNED)
#define TTYS1_DEV		g_uart1port	/* UART1 is ttyS1 */
#define UART1_ASSIGNED	1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct up_dev_s {
	uint32_t uartbase;			/* Base address of UART registers */
	uint32_t baud;				/* Configured baud */
	uint16_t irq;				/* IRQ associated with this UART */
	uint8_t parity;				/* 0=none, 4=odd, 5=even */
	uint8_t bits;				/* Number of bits (5, 6, 7 or 8) */
	bool stopbits2;				/* true: Configure with 2 stop bits instead of 1 */
	UART_CHANNEL eCh;			/*  Number of Uart Channel  */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int up_interrupt(int irq, void *context, void *arg);
static int up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);
static void uart_init(UART_CHANNEL uart);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART operations structure */
static const struct uart_ops_s g_uart_ops = {
	.setup = up_setup,
	.shutdown = up_shutdown,
	.attach = up_attach,
	.detach = up_detach,
	.ioctl = up_ioctl,
	.receive = up_receive,
	.rxint = up_rxint,
	.rxavailable = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
	.rxflowcontrol = NULL,
#endif
	.send = up_send,
	.txint = up_txint,
	.txready = up_txready,
	.txempty = up_txempty,
};

/* I/O buffers */
#ifdef CONFIG_BCM2835_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_BCM2835_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif

#ifdef CONFIG_BCM2835_UART0
static struct up_dev_s g_uart0priv = {
	.uartbase = BCM2835_UART0_BASE,
	.baud = CONFIG_UART0_BAUD,
	.irq = BCM2835_IRQ_UART0,
	.parity = CONFIG_UART0_PARITY,
	.bits = CONFIG_UART0_BITS,
	.stopbits2 = CONFIG_UART0_2STOP,
	.eCh = UART0,
};

static uart_dev_t g_uart0port = {
	.recv = {
		.size = CONFIG_UART0_RXBUFSIZE,
		.buffer = g_uart0rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART0_TXBUFSIZE,
		.buffer = g_uart0txbuffer,
	},
	.ops = &g_uart_ops,
	.priv = &g_uart0priv,
};
#endif

#ifdef CONFIG_BCM2835_UART1
static struct up_dev_s g_uart1priv = {
	.uartbase = BCM2835_UART1_BASE,
	.baud = CONFIG_UART1_BAUD,
	.irq = BCM2835_IRQ_UART1,
	.parity = CONFIG_UART1_PARITY,
	.bits = CONFIG_UART1_BITS,
	.stopbits2 = CONFIG_UART1_2STOP,
	.eCh = UART1,
};

static uart_dev_t g_uart1port = {
	.recv = {
		.size = CONFIG_UART1_RXBUFSIZE,
		.buffer = g_uart1rxbuffer,
	},
	.xmit = {
		.size = CONFIG_UART1_TXBUFSIZE,
		.buffer = g_uart1txbuffer,
	},
	.ops = &g_uart_ops,
	.priv = &g_uart1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BCM2835_UART1
static void up_uart_chip_init_begin(uint32_t uBase)
{
	if (uBase == BCM2835_UART1_BASE) {
		REG_IRQ_DISABLE1 = BIT(29);	/* Disable AUX irq */
		putreg32(0x01, uBase + AUX_ENABLES_ADDR);	/* AUX enable UART */
		putreg32(0x00, uBase + AUX_MU_IER_REG_ADDR);
		putreg32(0x03, uBase + AUX_MU_LCR_REG_ADDR);	/* bit 1 must be 1 */
		putreg32(0x00, uBase + AUX_MU_MCR_REG_ADDR);
		putreg32(0xC6, uBase + AUX_MU_IIR_REG_ADDR);
	}
}

static void up_uart_chip_init_end(uint32_t uBase)
{
	if (uBase == BCM2835_UART1_BASE) {
		putreg32(0xC6, uBase + AUX_MU_IIR_REG_ADDR);
		putreg32(0x03, uBase + AUX_MU_CNTL_REG_ADDR);
		REG_IRQ_ENABLE1 = BIT(29);	/* Enable AUX irq */
	}
}
#endif

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static int up_interrupt(int irq, void *context, void *arg)
{
	struct uart_dev_s *dev = NULL;
	struct up_dev_s *priv;
	UART_INTERRUPT mis;
	int passes;
	bool handled;
#if 1
	if (irq == BCM2835_IRQ_UART1) {
		dev = &g_uart1port;
	}
#ifdef CONFIG_BCM2835_UART0
	else {
		dev = &g_uart0port;
	}
#endif
#else
	dev = (uart_dev_t *) arg;
#endif

	priv = (struct up_dev_s *)dev->priv;

	/* Loop until there are no characters to be transferred or,
	 * until we have been looping for a long time.
	 */

	handled = true;
	for (passes = 0; passes < 256 && handled; passes++) {
		handled = false;

		/* Get the masked UART status and clear the pending interrupts. */

		mis = getreg32(priv->uartbase + AUX_MU_IIR_REG_ADDR) & UART_IIR_INT_MASK;
		putreg32(mis, priv->uartbase + AUX_MU_IIR_REG_ADDR);

		/* Handle incoming, receive bytes (with or without timeout) */

		if ((mis & RX_INT_IIR) != 0) {
			/* Rx buffer not empty ... process incoming bytes */

			uart_recvchars(dev);
		}

		/* Handle outgoing, transmit bytes */

		if ((mis & TX_INT_IIR) != 0) {
			/* Tx FIFO not full ... process outgoing bytes */

			uart_xmitchars(dev);
		}
	}

	return OK;
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/
static int up_setup(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	/* Initialize UART */
	int clk;
	int baud_rate = priv->baud;

	clk = BCM2835_CLOCK_FREQ;	//cal_clk_getrate(m1_clkcmu_uart);

	UART_CHANNEL eCh = priv->eCh;
	uint32_t uBase = priv->uartbase;
	u32 div;

#ifdef CONFIG_BCM2835_UART1
	up_uart_chip_init_begin(uBase);
#endif

	if (eCh == UART1) {
		bcm2835_configgpio(GPIO_ALT5 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN4);	/* GPIO 14 */
		bcm2835_configgpio(GPIO_ALT5 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN5);	/* GPIO 15 */
	} else if (eCh == UART0) {
		bcm2835_configgpio(GPIO_ALT0 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN4);	/* GPIO 14 */
		bcm2835_configgpio(GPIO_ALT0 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN5);	/* GPIO 15 */
	}

	/* disable ALL INT */
	modifyreg32(uBase + AUX_MU_IER_REG_ADDR, ALL_INT, 0);

	div = (clk / (baud_rate * 8)) - 1;
	putreg32(div, uBase + AUX_MU_BAUD_REG_ADDR);

#ifdef CONFIG_BCM2835_UART1
	up_uart_chip_init_end(uBase);
#endif

	getreg32(uBase + AUX_MU_IO_REG_ADDR);	/* read unknown data and clear FIFO */

	return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/
static void up_shutdown(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	/* Disable all interrupts */
	putreg32(~ALL_INT, priv->uartbase + AUX_MU_IER_REG_ADDR);

	if (priv->eCh == UART1) {
		bcm2835_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN4);	/* GPIO 14 */
		bcm2835_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN5);	/* GPIO 15 */
	}
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	int ret;

	/* Attach and enable the IRQ */
	ret = irq_attach(priv->irq, up_interrupt, NULL);
	if (ret == OK) {
		/* Enable the interrupt (RX and TX interrupts are still disabled
		 * in the UART
		 */
		up_enable_irq(priv->irq);
	}

	return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally before the shutdown method is called.The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	up_disable_irq(priv->irq);
	irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/
static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
	struct inode *inode = filep->f_inode;
	struct uart_dev_s *dev = inode->i_private;
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	int ret = OK;
	struct termios *termiosp = (struct termios *)arg;

	switch (cmd) {
	case TCGETS:
		if (!termiosp) {
			return -EINVAL;
		}

		cfsetispeed(termiosp, priv->baud);

		termiosp->c_cflag = 0;
#if 0							/* not supported parity&stopbit2 in MINI UART */
		if (priv->parity) {
			termiosp->c_cflag |= PARENB;
			if (priv->parity == 1) {
				termiosp->c_cflag |= PARODD;
			}
		}

		if (priv->stopbits2) {
			termiosp->c_cflag |= CSTOPB;
		}
#endif
		termiosp->c_cflag |= CS5 + (priv->bits - 5);
		break;

	case TCSETS:
		if (!termiosp) {
			return -EINVAL;
		}

		priv->bits = 5 + (termiosp->c_cflag & CSIZE);

		priv->parity = 0;
#if 0							/* not supported parity&stopbit2 in MINI UART */
		if (termiosp->c_cflag & PARENB) {
			if (termiosp->c_cflag & PARODD) {
				priv->parity = 1;
			} else {
				priv->parity = 2;
			}
		}
		priv->stopbits2 = ! !(termiosp->c_cflag & CSTOPB);
#else
		priv->parity = PM_EVEN;
		priv->stopbits2 = SB_1;
#endif
		priv->baud = cfgetispeed(termiosp);

		dev->ops->setup(dev);
		break;

	default:
		ret = -ENOTTY;
		break;
	}
#else
	int ret = -ENOTTY;
#endif

	return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART. Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/
static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	/* Return the error information in the saved status */
	*status = 0;
	int empty;
	do {
		empty = !(getreg32(priv->uartbase + AUX_MU_LSR_REG_ADDR) & UART_LSR_RX_MASK);
	} while (empty);

	return getreg32(priv->uartbase + AUX_MU_IO_REG_ADDR) & UART_RX_MASK;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static void up_rxint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint32_t uintm = getreg32(priv->uartbase + AUX_MU_IER_REG_ADDR);
	if (enable) {
		uintm |= (RX_INT);
	} else {
		uintm &= ~(RX_INT);
	}

	putreg32(uintm, priv->uartbase + AUX_MU_IER_REG_ADDR);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/
static bool up_rxavailable(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((getreg32(priv->uartbase + AUX_MU_LSR_REG_ADDR) & UART_LSR_RX_MASK) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/
static void up_send(struct uart_dev_s *dev, int ch)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	putreg32(ch, priv->uartbase + AUX_MU_IO_REG_ADDR);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static void up_txint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint32_t uintm = getreg32(priv->uartbase + AUX_MU_IER_REG_ADDR);
	if (enable) {
		uintm |= (TX_INT);

	} else {
		/* Disable the TX interrupt */
		uintm &= ~(TX_INT);
	}
	putreg32(uintm, priv->uartbase + AUX_MU_IER_REG_ADDR);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Check if the tranmsit fifo is not full
 *
 * Input Parameters:
 *   dev - pointer to uart dev structure
 *
 * Returned Value:
 *   Return true if the tx fifo is not full
 *
 ****************************************************************************/
static bool up_txready(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((getreg32(priv->uartbase + AUX_MU_LSR_REG_ADDR) & UART_LSR_TX_IDLE_MASK) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty.
 *
 ****************************************************************************/
static bool up_txempty(struct uart_dev_s *dev)
{
	struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return ((getreg32(priv->uartbase + AUX_MU_LSR_REG_ADDR) & UART_LSR_TX_EMPTY_MASK) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/
#if defined(USE_EARLYSERIALINIT)
void up_earlyserialinit(void)
{
#if defined(HAVE_SERIAL_CONSOLE)
	CONSOLE_DEV.isconsole = true;
	up_setup(&CONSOLE_DEV);
#endif
}
#endif							/* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/
#if defined(USE_SERIALDRIVER)
void up_serialinit(void)
{
	/* Register the console */
#if defined(HAVE_SERIAL_CONSOLE)
#if !defined(USE_EARLYSERIALINIT)
	CONSOLE_DEV.isconsole = true;
	up_setup(&CONSOLE_DEV);
#endif
	uart_register("/dev/console", &CONSOLE_DEV);
#endif

	/* Register all UARTs */
#ifdef TTYS0_DEV
	uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
	uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}
#endif							/* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one byte on the serial console
 *
 * Input Parameters:
 *   ch - chatacter to output
 *
 * Returned Value:
 *  sent character
 *
 ****************************************************************************/
int up_putc(int ch)
{
#if defined(HAVE_SERIAL_CONSOLE)
	/* Check for LF */
	if (ch == '\n') {
		/* Add CR */
		up_lowputc('\r');
	}

	up_lowputc(ch);
#endif

	return ch;
}

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 * Input Parameters:
 *   ch - chatacter to output
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void up_lowputc(char ch)
{
	struct up_dev_s *priv;
	priv = CONSOLE_DEV.priv;
	while (!up_txempty(&CONSOLE_DEV)) ;
	putreg32(ch, priv->uartbase + AUX_MU_IO_REG_ADDR);
}
