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
 * arch/arm/src/bcm2835/bcm2835_i2c.c
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/irq.h>
#include <tinyara/i2c.h>
#include <tinyara/clock.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "bcm2835_gpio.h"
#include "bcm2835_i2c.h"
#include "bcm2835_vclk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define BCM2835_DEFAULT_I2CXFER_CLOCK	100*1000	/* 100Khz */
#define BCM2835_DEFAULT_I2CSLAVE_ADDR	0x22
#define BCM2835_DEFAULT_I2C_TIMEOUT	10000

#define HSI2C_INT_XFER_DONE (HSI2C_INT_XFER_DONE_NOACK_MANUAL | HSI2C_INT_XFER_DONE_MANUAL)

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bcm2835_i2c_priv_s *g_bcm2835_i2c_priv[4] = { NULL };

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C Interface */
static const struct i2c_ops_s bcm2835_i2c_ops = {
	.setfrequency = bcm2835_i2c_setclock,
	.setaddress = bcm2835_i2c_setownaddress,
	.write = bcm2835_i2c_write,
	.read = bcm2835_i2c_read,
#ifdef CONFIG_I2C_TRANSFER
	.transfer = bcm2835_i2c_transfer,
#endif
#ifdef CONFIG_I2C_SLAVE
	.setownaddress = NULL,
	.registercallback = NULL,
#endif
};

static const struct bcm2835_i2c_config_s bcm2835_i2c0_config = {
	.base = BSC0_ADDR,
	.sda_pin = (GPIO_PIN0 | GPIO_ALT0),
	.scl_pin = (GPIO_PIN1 | GPIO_ALT0),
	.isr = bcm2835_i2c0_interrupt,
	.irq = BCM2835_I2C_IRQ,
	.devno = 0,
};

static struct bcm2835_i2c_priv_s bcm2835_i2c0_priv = {
	.xfer_speed = BCM2835_DEFAULT_I2CXFER_CLOCK,
	.master = I2C_MASTER,
	.mode = I2C_POLLING,
	.slave_addr = BCM2835_DEFAULT_I2CSLAVE_ADDR,
	.addrlen = 7,
	.timeout = BCM2835_DEFAULT_I2C_TIMEOUT,
	.name = "bcm2835_i2c0",
	.initialized = 0,
	.retries = 3,
};

static const struct bcm2835_i2c_config_s bcm2835_i2c1_config = {
	.base = BSC1_ADDR,
	.sda_pin = (GPIO_PIN2 | GPIO_ALT0),
	.scl_pin = (GPIO_PIN3 | GPIO_ALT0),
	.isr = bcm2835_i2c1_interrupt,
	.irq = BCM2835_I2C_IRQ,
	.devno = 1,
};

static struct bcm2835_i2c_priv_s bcm2835_i2c1_priv = {
	.xfer_speed = BCM2835_DEFAULT_I2CXFER_CLOCK,
	.master = I2C_MASTER,
	.mode = I2C_POLLING,
	.slave_addr = BCM2835_DEFAULT_I2CSLAVE_ADDR,
	.addrlen = 7,
	.timeout = BCM2835_DEFAULT_I2C_TIMEOUT,
	.name = "bcm2835_i2c1",
	.initialized = 0,
	.retries = 3,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void i2c_calculate_timing(unsigned int base, unsigned int nPclk, unsigned int nOpClk)
{
	unsigned int speed;
	/* setup BSC/I2C clock divider */

	assert(nOpClk != 0);
	speed = nPclk / nOpClk;
	putreg32(speed, base + BSC_CLOCK_DIVIDER);
}

static void i2c_enable_int(unsigned int base, unsigned int bit)
{
	unsigned int val;

	val = getreg32(base + BSC_CONTROL);
	val |= bit;
	putreg32(val, base + BSC_CONTROL);
}

static void i2c_disable_int(unsigned int base, unsigned int bit)
{
	unsigned int val;

	val = getreg32(base + BSC_CONTROL);
	val &= ~bit;
	putreg32(val, base + BSC_CONTROL);
}

static void i2c_clear_int(unsigned int base, unsigned int bit)
{
	putreg32(bit, base + BSC_STATUS);
}

static unsigned int i2c_read_int_status(unsigned int base)
{
	return getreg32(base + BSC_STATUS);
}

static void i2c_set_slave_addr(unsigned int base, u16 addr, unsigned int is_master)
{
	unsigned int val;

	addr &= 0x3FF;

	val = getreg32(base + BSC_SLAVE_ADDRESS);

	if (is_master == 0) {
		val &= ~0x3ff;
		val |= addr;
	} else {
		val &= ~(0x3FF << 10);
		val |= (addr << 10);
	}
	putreg32(val, base + BSC_SLAVE_ADDRESS);
}

static int i2c_manual_fast_init(struct bcm2835_i2c_priv_s *priv)
{
	unsigned int val, speed;
	unsigned int base = priv->config->base;

	/* setup BSC/I2C clock divider */
	assert(priv->xfer_speed != 0);
	speed = priv->clock / priv->xfer_speed;
	putreg32(speed, base + BSC_CLOCK_DIVIDER);

	/* Enable BSC/I2C */
	val = getreg32(base + BSC_CONTROL);
	val |= BSC_I2CEN;
	putreg32(val, base + BSC_CONTROL);

	priv->initialized = 1;

	i2c_enable_int(base, BSC_INTR | BSC_INTT | BSC_INTD);

	return 0;
}

static int i2c_wait_xfer_done(struct bcm2835_i2c_priv_s *priv)
{
	int timeout = priv->timeout;
	unsigned int status;
	unsigned int base = priv->config->base;

	while (timeout-- > 0) {
		status = getreg32(base + BSC_STATUS);
		if (status & (BSC_CLKT | BSC_ERR)) {
			putreg32(BSC_CLEAR, base + BSC_CONTROL);
			putreg32(BSC_CLKT | BSC_ERR, base + BSC_STATUS);
		} else if (status & BSC_DONE) {
			putreg32(0, base + BSC_CONTROL);
			putreg32(BSC_CLKT | BSC_ERR | BSC_DONE, priv->config->base + BSC_STATUS);
			return ((status & BSC_DONE) == BSC_DONE);
		} else if (status & BSC_TXW) {
			/* TODO implement, no case yet */
		}
	}

	return -ETIMEDOUT;
}

static int i2c_wait_rcv_done(struct bcm2835_i2c_priv_s *priv)
{
	int timeout = priv->timeout;
	unsigned int status;
	while (timeout-- > 0) {
		status = getreg32(priv->config->base + BSC_STATUS);
		if (status & (BSC_CLKT | BSC_ERR)) {
			putreg32(BSC_CLEAR, priv->config->base + BSC_CONTROL);
			putreg32(BSC_CLKT | BSC_ERR, priv->config->base + BSC_STATUS);
		} else if (status & BSC_DONE) {
			putreg32(0, priv->config->base + BSC_CONTROL);
			putreg32(BSC_CLKT | BSC_ERR | BSC_DONE, priv->config->base + BSC_STATUS);
			return ((status & BSC_DONE) == BSC_DONE);
		} else if (status & BSC_RXR) {
			/* TODO implement, no case yet */
		}
	}
	return -ETIMEDOUT;
}

static void i2c_start(struct bcm2835_i2c_priv_s *priv)
{
	unsigned int val;
	struct i2c_msg_s *pmsg;
	pmsg = priv->msgv;

	putreg32(pmsg->addr, priv->config->base + BSC_SLAVE_ADDRESS);
	putreg32(pmsg->length, priv->config->base + BSC_DATA_LENGTH);
	putreg32(CLEAR_STATUS, priv->config->base + BSC_STATUS);

	/* Enable Interrupts and start transfer. */
	val = getreg32(priv->config->base + BSC_CONTROL);
	val |= (BSC_INTT | BSC_INTD | START_WRITE);
	putreg32(val, priv->config->base + BSC_CONTROL);
}

static void i2c_stop(struct bcm2835_i2c_priv_s *priv)
{
	/* Disable Interrupts and stop transfer. */
	putreg32(0, priv->config->base + BSC_CONTROL);
}

static void i2c_repstart(struct bcm2835_i2c_priv_s *priv)
{
	unsigned int val;
	struct i2c_msg_s *pmsg;
	pmsg = priv->msgv;

	if (pmsg->flags & I2C_M_READ) {
		/* Enable Interrupts and start transfer. */
		val = getreg32(priv->config->base + BSC_CONTROL);
		val |= (BSC_INTT | BSC_INTD | START_READ);
		putreg32(val, priv->config->base + BSC_CONTROL);
	}
}

static int i2c_outb(struct bcm2835_i2c_priv_s *priv, u8 data)
{
	int ret = 1;

	int timeout = priv->timeout;
	unsigned int status;

	while (timeout-- > 0) {
		status = getreg32(priv->config->base + BSC_STATUS);
		if (status & (BSC_TXD)) {
			break;
		}
	}

	putreg32(data, priv->config->base + BSC_DATA_FIFO);

	return ret;
}

static int i2c_inb(struct bcm2835_i2c_priv_s *priv, bool is_ack)
{
	u8 data;
	int timeout = priv->timeout;
	unsigned int status;

	while (timeout-- > 0) {
		status = getreg32(priv->config->base + BSC_STATUS);
		if (status & (BSC_RXD)) {
			data = getreg32(priv->config->base + BSC_DATA_FIFO);
			return data;
		}
	}

	return -ETIMEDOUT;
}

static int sendbytes(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg)
{
	const u8 *p = msg->buffer;
	int count = msg->length;
	int nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int wrcount = 0, ret;

	while (count > 0) {
		ret = i2c_outb(priv, *p);
		if ((ret == 1) || ((ret == 0) && nak_ok)) {
			count--;
			p++;
			wrcount++;
		} else if (ret == 0) {
			/* NAK from the slave */
			return -EIO;
		} else {
			/* Timeout */
			return ret;
		}
	}

	i2c_wait_xfer_done(priv);

	return wrcount;
}

static int readbytes(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg)
{
	int val;
	int rdcount = 0;
	u8 *p = msg->buffer;
	int count = msg->length;

	/* Enable Interrupts and start transfer. */
	putreg32(count, priv->config->base + BSC_DATA_LENGTH);
	val = getreg32(priv->config->base + BSC_CONTROL);
	val |= (BSC_INTT | BSC_INTD | START_READ);
	putreg32(val, priv->config->base + BSC_CONTROL);

	while (count > 0) {
		val = i2c_inb(priv, (count > 1));
		if (val < 0) {
			break;
		}

		*p++ = val;
		rdcount++;
		count--;
	}

	i2c_wait_rcv_done(priv);
	return rdcount;
}

static int try_address(struct bcm2835_i2c_priv_s *priv, u8 addr, int retries)
{
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(priv, addr);
		i2c_wait_rcv_done(priv);
		if (ret == 1 || i == retries) {
			break;
		}
		i2c_stop(priv);
		up_udelay(priv->timeout / 2);
		i2c_start(priv);
	}

	return ret;
}

static int do_address(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	u8 addr;
	int ret;
	int retries;

	retries = nak_ok ? 0 : priv->retries;

	if (flags & I2C_M_TEN) {
		/* a 10-bit address in manual mode */
		addr = 0xf0 | ((msg->addr >> 7) & 0x06);

		ret = try_address(priv, addr, retries);
		if ((ret != 1) && !nak_ok) {
			return -ENXIO;
		}

		/* the remaining 8 bit address */
		ret = i2c_outb(priv, msg->addr & 0xff);
		if ((ret != 1) && !nak_ok) {
			return -ENXIO;
		}

		if (flags & I2C_M_READ) {
			i2c_repstart(priv);
			addr |= 0x1;
			ret = try_address(priv, addr, retries);
			if ((ret != 1) && !nak_ok) {
				return -EIO;
			}
		}
	} else {
		/* 7-bit address */
		addr = msg->addr << 1;
		if (flags & I2C_M_READ) {
			addr |= 0x1;
		}

		ret = try_address(priv, addr, retries);
		if ((ret != 1) && !nak_ok) {
			return -ENXIO;
		}
	}

	return 0;
}

static int i2c_master_handler(void *args)
{
	struct bcm2835_i2c_priv_s *priv = args;
	unsigned int int_status;
	int off = priv->master_test_data->cur_msg;
	struct i2c_msg_s *msg = &priv->master_test_data->msg[off];
	unsigned int base = priv->config->base;

	/* TODO : do not test interrupt mode */

	int_status = i2c_read_int_status(base);
	if (int_status & (BSC_CLKT | BSC_ERR)) {
		/* TODO set error flags */
		putreg32(BSC_CLEAR, base + BSC_CONTROL);
		putreg32(BSC_CLKT | BSC_ERR, base + BSC_STATUS);
	} else if (int_status & BSC_DONE) {
		putreg32(0, base + BSC_CONTROL);
		putreg32(BSC_CLKT | BSC_ERR | BSC_DONE, priv->config->base + BSC_STATUS);
	}

	if (msg->flags & I2C_M_READ) {
		if (int_status & BSC_RXD) {
			readbytes(priv, msg);
		}
	} else {
		if (int_status & BSC_TXW) {
			sendbytes(priv, msg);
		}
	}

	i2c_clear_int(base, int_status);

	return 0;
}

static void i2c_master_setup(struct bcm2835_i2c_priv_s *priv, unsigned int mode, unsigned int speed, unsigned int slave_addr)
{
	if (priv->mode == I2C_POLLING) {
		priv->xfer_speed = speed;
		i2c_calculate_timing(priv->config->base, priv->clock, speed);
	}
#ifdef CONFIG_BCM2835_I2C_INTERRUPT_MODE
	else if (priv->mode == I2C_INTERRUPT) {
		priv->master_test_data = (struct master_data *)malloc(sizeof(struct master_data));
		/* complete_init(&priv->master_test_data->done); */
		i2c_calculate_timing(priv->config->base, priv->clock, speed);
		i2c_set_slave_addr(priv->config->base, slave_addr, 1);
	}
#endif
}

static void i2c_slave_setup(struct bcm2835_i2c_priv_s *priv, unsigned int mode, unsigned int speed, unsigned int slave_addr)
{
	lldbg("* No slave operation mode support\n");
}

static void i2c_master_cleanup(struct bcm2835_i2c_priv_s *priv)
{
#ifdef CONFIG_BCM2835_I2C_INTERRUPT_MODE
	if (priv->mode == I2C_INTERRUPT) {
		free(priv->master_test_data);
		priv->master_test_data = NULL;
		irq_detach(priv->config->irq);
	}
#endif
}

static void i2c_slave_cleanup(struct bcm2835_i2c_priv_s *priv)
{
	lldbg("* No slave operation mode support\n");
	free(priv->slave_test_data);
	priv->slave_test_data = NULL;
#ifdef CONFIG_BCM2835_I2C_INTERRUPT_MODE
	if ((priv->master == I2C_SLAVE_MODE) || (priv->mode = I2C_INTERRUPT)) {
		irq_detach(priv->config->irq);
	}
#endif
}

static int i2c_setup(struct bcm2835_i2c_priv_s *priv, unsigned int master, unsigned int mode, unsigned int speed, unsigned int slave_addr)
{
	priv->master = master;
	priv->mode = mode;
	priv->xfer_speed = speed;
	priv->slave_addr = slave_addr;

	i2c_manual_fast_init(priv);

	if (master == I2C_MASTER) {
		i2c_master_setup(priv, mode, speed, slave_addr);
	} else if (master == I2C_SLAVE_MODE) {
		i2c_slave_setup(priv, mode, speed, slave_addr);
	}

	return 0;
}

static int i2c_cleanup(struct bcm2835_i2c_priv_s *priv)
{
	if (priv->master == I2C_MASTER) {
		i2c_master_cleanup(priv);
	} else if (priv->master == I2C_SLAVE_MODE) {
		i2c_slave_cleanup(priv);
	}

	priv->master = I2C_MASTER;
	priv->mode = I2C_POLLING;
	priv->xfer_speed = I2C_SPEED_400KHZ;

	/* Disable Interrupts and stop transfer. */
	putreg32(0, priv->config->base + BSC_CONTROL);

	return 0;
}

static inline void bcm2835_i2c_sem_wait(struct bcm2835_i2c_priv_s *priv)
{
	while (sem_wait(&priv->exclsem) != 0) {
		ASSERT(errno == EINTR);
	}
}

static inline void bcm2835_i2c_sem_post(struct bcm2835_i2c_priv_s *priv)
{
	sem_post(&priv->exclsem);
}

static inline void bcm2835_i2c_sem_init(struct bcm2835_i2c_priv_s *priv)
{
	sem_init(&priv->exclsem, 0, 1);
#ifndef CONFIG_I2C_POLLED
	sem_init(&priv->waitsem, 0, 0);
#endif
}

static inline void bcm2835_i2c_sem_destroy(struct bcm2835_i2c_priv_s *priv)
{
	sem_destroy(&priv->exclsem);
#ifndef CONFIG_I2C_POLLED
	sem_destroy(&priv->waitsem);
#endif
}

static int bcm2835_i2c0_interrupt(int irq, void *context, void *arg)
{
	struct bcm2835_i2c_priv_s *priv;

	/* Read the masked interrupt status */
	priv = &bcm2835_i2c0_priv;

	/* Let the common interrupt handler do the rest of the work */
	return i2c_master_handler(priv);
}

static int bcm2835_i2c1_interrupt(int irq, void *context, void *arg)
{
	struct bcm2835_i2c_priv_s *priv;

	/* Read the masked interrupt status */
	priv = &bcm2835_i2c1_priv;

	/* Let the common interrupt handler do the rest of the work */
	return i2c_master_handler(priv);
}

static int bcm2835_i2c_initialize(struct bcm2835_i2c_priv_s *priv, unsigned int frequency)
{
	const struct bcm2835_i2c_config_s *config = priv->config;
	int ret;

	ret = bcm2835_configgpio(config->scl_pin);
	if (ret < 0) {
		return ret;
	}

	ret = bcm2835_configgpio(config->sda_pin);
	if (ret < 0) {
		return ret;
	}

	/* Enable the I2C master block */
	/* Configure the the initial I2C clock frequency. */
	priv->xfer_speed = frequency;
	i2c_setup(priv, priv->master, priv->mode, priv->xfer_speed, priv->slave_addr);

	/*
	 * Attach interrupt handlers and enable interrupts at the NVIC (still
	 * disabled at the source).
	 */
#ifdef CONFIG_BCM2835_I2C_INTERRUPT_MODE
	if ((priv->master == I2C_SLAVE_MODE) || (priv->mode == I2C_INTERRUPT)) {
		irq_attach(config->irq, config->isr, NULL);
		up_enable_irq(config->irq);
	}
#endif

	return OK;
}

static int bcm2835_i2c_uninitialize(struct bcm2835_i2c_priv_s *priv)
{
	/* Disable I2C */
	i2c_cleanup(priv);

	/* Unconfigure GPIO pins */
	bcm2835_unconfiggpio(priv->config->scl_pin);
	bcm2835_unconfiggpio(priv->config->sda_pin);

	/* Disable and detach interrupts */
#ifdef CONFIG_BCM2835_I2C_INTERRUPT_MODE
	if ((priv->master == I2C_SLAVE_MODE) || (priv->mode == I2C_INTERRUPT)) {
		up_disable_irq(priv->config->irq);
		irq_detach(priv->config->irq);
	}
#endif

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/**
 * @brief    i2c clock setting
 * @param    struct i2c_dev_s *dev : pointer to i2c_dev_s
 * @param    unsigned int frequency : ex ) 100000 == 100khz
 * @return   void
 * @note
 */
unsigned int bcm2835_i2c_setclock(FAR struct i2c_dev_s *dev, unsigned int frequency)
{
	unsigned int speed;
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;
	unsigned int base = priv->config->base;

	/* Save the new I2C frequency */
	priv->xfer_speed = frequency;

	/* setup BSC/I2C clock divider */
	assert(priv->xfer_speed != 0);
	speed = priv->clock / priv->xfer_speed;
	putreg32(speed, base + BSC_CLOCK_DIVIDER);

	return 0;
}

/**
 * @brief    Setting Slave Address for slave mode
 * @param    struct i2c_dev_s *dev : pointer to i2c_dev_s
 * @param    int addr : Slave address
 * @param     int nbits : 0==7bit
 *          1==10bit
 * @return    = 0
 * @note
 */
int bcm2835_i2c_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits)
{
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;

	priv->slave_addr = addr;
	if (nbits == 1) {
		priv->msgv->flags |= I2C_M_TEN;
	}
	i2c_set_slave_addr(priv->config->base, addr, 0);
	return 0;
}

/**
 * @brief    Generic I2C transfer function
 * @param    struct i2c_master_s *dev : structure visible to the I2C client
 * @param    struct i2c_msg_s *msgv : I2C transaction segment
 * @param    int msgc :  length
 * @return   int : ==0 :OK
 * @note
 */
int bcm2835_i2c_transfer(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc)
{
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;
	struct i2c_msg_s *pmsg;
	int ret = OK;
	int i;
	int nak_ok;
	int start = 1;
	int stop = 1;

	/* Ensure that address or flags don't change meanwhile */
	bcm2835_i2c_sem_wait(priv);

	priv->mcnt = 0;
	priv->mptr = NULL;
	priv->msgv = msgv;
	priv->msgc = msgc;

	i2c_start(priv);

	for (i = 0; i < msgc; i++) {
		pmsg = &msgv[i];
		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
		if (!(pmsg->flags & I2C_M_NOSTART)) {
			if ((i > 0) || (start == 0)) {
				i2c_repstart(priv);
			}
			ret = do_address(priv, pmsg);
			if ((ret != 0) && !nak_ok) {
				goto fail;
			}
		}
		if (pmsg->flags & I2C_M_READ) {
			/* read bytes into buffer */
			ret = readbytes(priv, pmsg);
			if (ret < pmsg->length) {
				if (ret >= 0) {
					return -EIO;
				}
				goto fail;
			}
		} else {
			/* write bytes from buffer */
			ret = sendbytes(priv, pmsg);
			if (ret < pmsg->length) {
				if (ret >= 0) {
					ret = -EIO;
				}
				goto fail;
			}
		}
	}
	ret = i;

fail:
	priv->mcnt = 0;
	priv->mptr = NULL;
	if (stop) {
		i2c_stop(priv);
	}

	/* Ensure that address or flags don't change meanwhile */
	bcm2835_i2c_sem_post(priv);

	return ret;
}

int bcm2835_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int buflen)
{
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;
	struct i2c_msg_s msg;
	unsigned int flags;

	/* 7- or 10-bit? */
	flags = (priv->addrlen == 10) ? I2C_M_TEN : 0;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr, msg.flags = (flags | I2C_M_READ);
	msg.buffer = (FAR uint8_t *) buffer;
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */

	return bcm2835_i2c_transfer(dev, &msg, 1);
}

int bcm2835_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer, int buflen)
{
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;
	struct i2c_msg_s msg;

	/* Setup for the transfer */
	msg.addr = priv->slave_addr;
	msg.flags = (priv->addrlen == 10) ? I2C_M_TEN : 0;
	msg.buffer = (FAR uint8_t *) buffer;	/* Override const */
	msg.length = buflen;

	/*
	 * Then perform the transfer
	 *
	 * REVISIT:  The following two operations must become atomic in order to
	 * assure thread safety.
	 */
	return bcm2835_i2c_transfer(dev, &msg, 1);
}

/**
 * @brief   Initialize one I2C bus
 * @param   int port :
 * @return  struct i2c_master_s : device structure
 * @note
 */
struct i2c_dev_s *up_i2cinitialize(int port)
{
	struct bcm2835_i2c_priv_s *priv = NULL;
	const struct bcm2835_i2c_config_s *config;
	int flags;

	/* Get I2C private structure */
	if (g_bcm2835_i2c_priv[port] != NULL) {
		return (FAR struct i2c_dev_s *)g_bcm2835_i2c_priv[port];
	}

	switch (port) {
	case 0:
		priv = &bcm2835_i2c0_priv;
		config = &bcm2835_i2c0_config;
		cal_clk_enable(gate_i2c0);
		priv->clock = cal_clk_getrate(gate_i2c0);
		break;

	case 1:
		priv = &bcm2835_i2c1_priv;
		config = &bcm2835_i2c1_config;
		cal_clk_enable(gate_i2c1);
		priv->clock = cal_clk_getrate(gate_i2c1);
		break;

	default:
		return NULL;
	}

	/* Initialize private device structure */
	priv->ops = &bcm2835_i2c_ops;

	/*
	 * Initialize private data for the first time, increment reference count,
	 * power-up hardware and configure GPIOs.
	 */
	flags = irqsave();;

	priv->refs++;
	if (priv->refs == 1) {
		/* Initialize the device structure */
		priv->config = config;
		bcm2835_i2c_sem_init(priv);

		/* Initialize the I2C hardware */
		bcm2835_i2c_initialize(priv, priv->xfer_speed);
	}

	g_bcm2835_i2c_priv[port] = priv;
	irqrestore(flags);

	return (FAR struct i2c_dev_s *)priv;
}

/**
 * @brief    Unitialize one I2C bus
 * @param    struct i2c_master_s *dev :
 * @return   ==0 : OK
 * @note
 */

int bcm2835_i2cbus_uninitialize(struct i2c_dev_s *dev)
{
	struct bcm2835_i2c_priv_s *priv = (struct bcm2835_i2c_priv_s *)dev;
	int flags;

	DEBUGASSERT(priv && priv->config && priv->refs > 0);

	/* Decrement reference count and check for underflow */
	flags = irqsave();

	/* Check if the reference count will decrement to zero */
	if (priv->refs < 2) {
		/* Yes.. Disable power and other HW resource (GPIO's) */
		bcm2835_i2c_uninitialize(priv);
		priv->refs = 0;

		/* Release unused resources */
		bcm2835_i2c_sem_destroy(priv);
	} else {
		/* No.. just decrement the number of references to the device */
		priv->refs--;
	}

	irqrestore(flags);
	return OK;
}

/**
 * @brief   Unitialize one I2C bus for the I2C character driver
 * @param   struct i2c_master_s *dev :
 * @return  ==0 : OK
 * @note
 */
void bcm2835_i2c_register(int bus)
{
	FAR struct i2c_dev_s *i2c;
	char path[16];
	int ret;

	i2c = up_i2cinitialize(bus);
	if (i2c != NULL) {
		snprintf(path, 16, "/dev/i2c-%d", bus);
		ret = i2c_uioregister(path, i2c);
		if (ret < 0) {
			bcm2835_i2cbus_uninitialize(i2c);
		}
	}
}
