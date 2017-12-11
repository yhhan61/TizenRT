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
 * arch/arm/src/bcm2835/bcm2835_spi.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/*****************************************************************************
 * Included Files
 *****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/spi/spi.h>

#include "up_arch.h"
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <tinyara/irq.h>
#include <arch/irq.h>
#include <arch/board/board.h>
#include <tinyara/kmalloc.h>
#include <poll.h>
#include <tinyara/fs/fs.h>
#include <stddef.h>
#include <chip.h>
#include <bcm2835_spi.h>

#include "bcm2835_vclk.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct spi_ops_s g_spiops = {
#ifndef CONFIG_SPI_OWNBUS
	.lock = spi_lock,
#endif
	.select = spi_select,
	.setfrequency = spi_setfrequency,
	.setmode = (void *)spi_setmode,
	.setbits = (void *)spi_setbits,
	.status = 0,
#ifdef CONFIG_SPI_CMDDATA
	.cmddata = 0,
#endif
	.send = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
#ifdef CONFIG_SPI_POLLED
	.exchange = spi_polled_exchange,
#else
	.exchange = spi_exchange,
#endif
#else
	.sndblock = spi_sndblock,
	.recvblock = spi_recvblock,
#endif
	.registercallback = 0,
};

// BCM2835 support only one SPI
// BCM2835 have simple two more SPI, but not used now
static struct bcm2835_spidev_s g_spi0dev = {
	.spidev = {.ops = &g_spiops},
	.base = BCM2835_SPI0_BASE,
	.port = SPI_PORT0,
	.freqid = d1_spi0,
	.clock_divider = 25000000,	// 25MHz ??(APB clock)
	.gpio_clk = (GPIO_SPI0_CLK | GPIO_ALT0),
	.gpio_nss = (GPIO_SPI0_CS | GPIO_ALT0),
	.gpio_miso = (GPIO_SPI0_MISO | GPIO_ALT0),
	.gpio_mosi = (GPIO_SPI0_MOSI | GPIO_ALT0),
};

static inline uint32_t bcm2835_rd(struct bcm2835_spidev_s *spi, unsigned reg)
{
	return getreg32(spi->base + reg);
}

static inline void bcm2835_wr(struct bcm2835_spidev_s *spi, unsigned reg, uint32_t val)
{
	putreg32(val, spi->base + reg);
}

static inline void bcm2835_rd_fifo(struct bcm2835_spidev_s *spi)
{
	uint8_t *rxbuf = (uint8_t *) spi->rxbuf;
	uint8_t byte;

	while ((spi->rx_cnt) && (bcm2835_rd(spi, BCM2835_SPI_CS) & SPI_CS_RXD)) {
		byte = bcm2835_rd(spi, BCM2835_SPI_FIFO);
		if (spi->rxbuf) {
			*rxbuf++ = byte;
		}
		spi->rx_cnt--;
	}
}

static inline void bcm2835_wr_fifo(struct bcm2835_spidev_s *spi)
{
	uint8_t *txbuf = (uint8_t *) spi->txbuf;
	uint8_t byte;

	while ((spi->tx_cnt) && (bcm2835_rd(spi, BCM2835_SPI_CS) & SPI_CS_TXD)) {
		byte = spi->txbuf ? *txbuf++ : 0;
		bcm2835_wr(spi, BCM2835_SPI_FIFO, byte);
		spi->tx_cnt--;
	}
}

static void bcm2835_spi_reset_hw(struct bcm2835_spidev_s *spi)
{
	uint32_t cs_reg = bcm2835_rd(spi, BCM2835_SPI_CS);

	/* Disable SPI interrupts and transfer */
	cs_reg &= ~(SPI_CS_INTR | SPI_CS_INTD | SPI_CS_DMAEN | SPI_CS_TA);

	/* and reset RX/TX FIFOS */
	cs_reg |= SPI_CS_CLEAR_TX | SPI_CS_CLEAR_RX;

	/* and reset the SPI_HW */
	bcm2835_wr(spi, BCM2835_SPI_CS, cs_reg);

	/* as well as DLEN */
	bcm2835_wr(spi, BCM2835_SPI_DLEN, 0);
}

static void bcm2835_spi_serve_interrupt(int irq, void *context, void *arg)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)arg;

	/* Read as many bytes as possible from FIFO */
	bcm2835_rd_fifo(priv);

	/* Write as many bytes as possible to FIFO */
	bcm2835_wr_fifo(priv);

	/* based on flags decide if we can finish the transfer */
	if (bcm2835_rd(priv, BCM2835_SPI_CS) & SPI_CS_DONE) {
		/* Transfer complete - reset SPI HW */
		bcm2835_spi_reset_hw(priv);

		/* wake up the framework */
		priv->state = SPI_COMPLETE;
	}

	if (bcm2835_rd(priv, BCM2835_SPI_CS) & (SPI_CS_RXR | SPI_CS_RXF)) {

		/* Read as many bytes as possible from FIFO */
		bcm2835_rd_fifo(priv);
	}

}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 ****************************************************************************/
static int spi_lock(struct spi_dev_s *dev, bool lock)
{
#ifndef CONFIG_SPI_OWNBUS
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;

	if (lock) {
		while (sem_wait(&priv->exclsem) != 0) {
			DEBUGASSERT(errno == EINTR);
		}
	} else {
		(void)sem_post(&priv->exclsem);
	}
#endif

	return OK;
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * defaults to 0, which means a divider of 65536.
 * The divisor must be a power of 2. Odd numbers
 * rounded down. The maximum SPI clock rate is
 * of the APB clock
 *
 ****************************************************************************/
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;
	unsigned cdiv;

	cdiv = ((priv->clock_divider / frequency) / 2) * 2;
	if (cdiv >= 65536) {
		cdiv = 0;
	}

	bcm2835_wr(priv, BCM2835_SPI_CLK, cdiv);

	return OK;
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enable/disable the SPI slave select.   The implementation of this method
 *   must include handshaking:  If a device is selected, it must hold off
 *   all other attempts to select the device until the device is deselecte.
 *
 ****************************************************************************/
static void spi_select(struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;

	unsigned int cs_reg;
	if (selected == TRUE) {
		cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
		cs_reg &= ~SPI_CS_CS;
		bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);
	} else {
		cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
		cs_reg |= SPI_CS_CS;
		bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);
	}
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 ****************************************************************************/
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;

	unsigned int cs_reg;
	cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
	cs_reg &= ~(SPI_CS_CSPOL | SPI_CS_CPOL);

	switch (mode) {
	case SPIDEV_MODE0:			/* CPOL=0 CPHA=0 */
		break;
	case SPIDEV_MODE1:			/* CPOL=0 CPHA=1 */
		cs_reg |= SPI_CS_CPHA;
		break;
	case SPIDEV_MODE2:			/* CPOL=1 CPHA=0 */
		cs_reg |= SPI_CS_CPOL;
		break;
	case SPIDEV_MODE3:			/* CPOL=1 CPHA=1 */
		cs_reg |= SPI_CS_CSPOL | SPI_CS_CPHA;
		break;
	default:
		break;
	}

	bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 ****************************************************************************/
static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;
	bcm2835_wr(priv, BCM2835_SPI_DLEN, nbits);	// Only for DMA mode
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI. Currently support only byte transfers.
 *
 ****************************************************************************/
static uint16_t spi_send(struct spi_dev_s *dev, uint16_t wd)
{
	uint8_t txbyte;
	uint8_t rxbyte;

	txbyte = (uint8_t) wd;
	rxbyte = (uint8_t) 0;
	spi_exchange(dev, &txbyte, &rxbyte, 1);

	return (uint16_t) rxbyte;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block data with the SPI device. Support only byte transfers.
 *
 ****************************************************************************/
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;

	/* Clear TX and RX fifos. */
	unsigned int cs_reg;
	cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
	cs_reg |= SPI_CS_CLEAR;
	bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);

	/* set Tx/Rx buffer & transfer count */
	if (txbuffer == NULL) 
		priv->txbuf = (void *)rxbuffer;
	else 
		priv->txbuf = (void *)txbuffer;

	if (rxbuffer == NULL)
		priv->rxbuf = (void *)txbuffer;
	else 
		priv->rxbuf = (void *)rxbuffer;

	priv->tx_cnt = nwords ;
	priv->rx_cnt = nwords ;

	/* set state machine to ready */
	priv->state = SPI_READY;

	/* Enable SPI interrupts and activate transfer. */
	cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
	cs_reg |= SPI_CS_INTD | SPI_CS_INTR | SPI_CS_TA;
	bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);

	/* need some sleep time : IMPORTANT */
	up_udelay(200);

	/* state change to active */
	priv->state = SPI_ACTIVE;

	/* fill in the fifo */
	bcm2835_wr_fifo(priv);

	while ((priv->state != SPI_COMPLETE) && priv->rx_cnt) {
		// do nothing
	}
}

static void spi_polled_exchange(struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;

	/* set Tx/Rx buffer & transfer count */
	if (txbuffer == NULL)
		priv->txbuf = (void *)rxbuffer;
	else
		priv->txbuf = (void *)txbuffer;

	if (rxbuffer == NULL)
		priv->rxbuf = (void *)txbuffer;
	else
		priv->rxbuf = (void *)rxbuffer;

	priv->tx_cnt = nwords;
	priv->rx_cnt = nwords;

	/* set state machine to ready */
	priv->state = SPI_READY;

	/* enable HW block without interrupts */
	uint32_t cs_reg = bcm2835_rd(priv, BCM2835_SPI_CS);
	cs_reg |= SPI_CS_CLEAR_TX | SPI_CS_CLEAR_RX;
	bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg | SPI_CS_TA);

	/* loop until finished the transfer */
	while (priv->rx_cnt) {
		/* set state machine to active */
		priv->state = SPI_ACTIVE;

		/* fill in tx fifo with remaining data */
		bcm2835_wr_fifo(priv);

		/* need some sleep time : IMPORTANT */
		up_udelay(100);

		/* read from fifo as much as possible */
		bcm2835_rd_fifo(priv);
	}

	/* based on flags decide if we can finish the transfer */
	if (bcm2835_rd(priv, BCM2835_SPI_CS) & SPI_CS_DONE) {
		/* Transfer complete - reset SPI HW */
		bcm2835_spi_reset_hw(priv);

		/* wake up the framework */
		priv->state = SPI_COMPLETE;
	}
}

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI. Support only byte transfers
 *
 ****************************************************************************/
static void spi_sndblock(struct spi_dev_s *dev, const void *txbuffer, size_t nwords)
{
	spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI. Support only byte transfers.
 *
 ****************************************************************************/
static void spi_recvblock(struct spi_dev_s *dev, void *rxbuffer, size_t nwords)
{
	spi_exchange(dev, NULL, rxbuffer, nwords);
}

/**
 * Initialzie SPI interface
 */
static void spi_irqinitialize(struct spi_dev_s *dev)
{
	irq_attach(BCM2835_IRQ_SPI, (xcpt_t) bcm2835_spi_serve_interrupt, dev);

	/* Enable the timer interrupt */
	up_enable_irq(BCM2835_IRQ_SPI);
}

static void spi_dev_init(struct spi_dev_s *dev)
{
	FAR struct bcm2835_spidev_s *priv = (FAR struct bcm2835_spidev_s *)dev;
	uint32_t cs_reg = 0;

	/* create tx/rx buffer */
	priv->tx_cnt = 0;
	priv->rx_cnt = 0;

	cs_reg &= ~SPI_CS_REN;
	cs_reg |= SPI_CS_CLEAR_TX | SPI_CS_CLEAR_RX;
	bcm2835_wr(priv, BCM2835_SPI_CS, cs_reg);

	bcm2835_spi_reset_hw(priv);
}

struct spi_dev_s *bcm2835_spibus_initialize(int port)
{
	FAR struct bcm2835_spidev_s *priv = NULL;

	if (port >= SPI_PORT_MAX) {
		return NULL;
	}

	switch (port) {
	case 0:
		priv = &g_spi0dev;
		break;
	}

	lldbg("SPI %d for Master operation\n", priv->port);
	return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 ****************************************************************************/
struct spi_dev_s *up_spiinitialize(int port)
{
	FAR struct bcm2835_spidev_s *priv = NULL;

	if (port >= SPI_PORT_MAX) {
		return NULL;
	}

	switch (port) {
	case 0:
		priv = &g_spi0dev;
		break;
	}
	lldbg("Prepare SPI%d for Master operation\n", priv->port);

	/* SET GPIO for the port */
	bcm2835_configgpio(priv->gpio_clk);
	bcm2835_configgpio(priv->gpio_nss);
	bcm2835_configgpio(priv->gpio_miso);
	bcm2835_configgpio(priv->gpio_mosi);

#ifndef CONFIG_SPI_POLLWAIT
	sem_init(&priv->xfrsem, 0, 0);
#endif

#ifndef CONFIG_SPI_OWNBUS
	sem_init(&priv->exclsem, 0, 1);
#endif

	spi_dev_init((struct spi_dev_s *)priv);

	spi_irqinitialize((struct spi_dev_s *)priv);

	return (struct spi_dev_s *)priv;
}
