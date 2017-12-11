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
 * arch/arm/src/bcm2835/bcm2835_spi.h
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_BCM2835_SPIDRV_H
#define __ARCH_ARM_SRC_BCM2835_SPIDRV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arch/chip/irq.h>
#include <chip.h>
#include <chip/bcm2835_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(SPI_USE_MUTUAL_EXCLUSION)
#define SPI_USE_MUTUAL_EXCLUSION 1
#define CH_USE_SEMAPHORES 1
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define GPIO_SPI0_CS		(GPIO_PORT0 | GPIO_PIN8)	// SPI0_CE0_N (8)
#define GPIO_SPI0_MISO		(GPIO_PORT0 | GPIO_PIN9)	// SPI0_MISO (9)
#define GPIO_SPI0_MOSI		(GPIO_PORT1 | GPIO_PIN0)	// SPI0_MOSI (10)
#define GPIO_SPI0_CLK		(GPIO_PORT1 | GPIO_PIN1)	// SPIO_SCLK (11)

/// See 10.5 SPI Register Map
#define BCM2835_SPI0_BASE	0x20204000	/* @brief SPI Master Control and Status. */
#define BCM2835_SPI_CS		0x00	/* SPI Master Control and Status. */
#define BCM2835_SPI_FIFO	0x04	/* SPI Master TX and RX FIFOs. */
#define BCM2835_SPI_CLK		0x08	/* SPI Master Clock Divider. */
#define BCM2835_SPI_DLEN	0x0C	/* SPI Master Data Length. */
#define BCM2835_SPI_STOH	0x10	/* SPI LOSSI mode TOH. */
#define BCM2835_SPI_DC		0x14	/* SPI DMA DREQ Controls. */

// Register masks for SPI0_CS
#define SPI_CS_LEN_LONG             0x02000000	/* @brief Enable Long data word in Lossi mode if DMA_LEN is set. */
#define SPI_CS_DMA_LEN              0x01000000	/* @brief Enable DMA mode in Lossi mode. */
#define SPI_CS_CSPOL2               0x00800000	/* @brief Chip Select 2 Polarity. */
#define SPI_CS_CSPOL1               0x00400000	/* @brief Chip Select 1 Polarity. */
#define SPI_CS_CSPOL0               0x00200000	/* @brief Chip Select 0 Polarity. */
#define SPI_CS_RXF                  0x00100000	/* @brief RXF - RX FIFO Full. */
#define SPI_CS_RXR                  0x00080000	/* @brief RXR RX FIFO needs Reading ( full). */
#define SPI_CS_TXD                  0x00040000	/* @brief TXD TX FIFO can accept Data. */
#define SPI_CS_RXD                  0x00020000	/* @brief RXD RX FIFO contains Data. */
#define SPI_CS_DONE                 0x00010000	/* @brief Done transfer Done. */
#define SPI_CS_TE_EN                0x00008000	/* @brief Unused. */
#define SPI_CS_LMONO                0x00004000	/* @brief Unused. */
#define SPI_CS_LEN                  0x00002000	/* @brief LEN LoSSI enable. */
#define SPI_CS_REN                  0x00001000	/* @brief REN Read Enable. */
#define SPI_CS_ADCS                 0x00000800	/* @brief ADCS Automatically Deassert Chip Select. */
#define SPI_CS_INTR                 0x00000400	/* @brief INTR Interrupt on RXR. */
#define SPI_CS_INTD                 0x00000200	/* @brief INTD Interrupt on Done. */
#define SPI_CS_DMAEN                0x00000100	/* @brief DMAEN DMA Enable. */
#define SPI_CS_TA                   0x00000080	/* @brief Transfer Active. */
#define SPI_CS_CSPOL                0x00000040	/* @brief Chip Select Polarity. */
#define SPI_CS_CLEAR                0x00000030	/* @brief Clear FIFO Clear RX and TX. */
#define SPI_CS_CLEAR_RX             0x00000020	/* @brief Clear FIFO Clear RX . */
#define SPI_CS_CLEAR_TX             0x00000010	/* @brief Clear FIFO Clear TX . */
#define SPI_CS_CPOL                 0x00000008	/* @brief Clock Polarity. */
#define SPI_CS_CPHA                 0x00000004	/* @brief Clock Phase. */
#define SPI_CS_CS                   0x00000003	/* @brief Chip Select. */

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
	SPI_UNINIT = 0,
	/**< Not initialized.                   */
	SPI_STOP = 1,
	/**< Stopped.                           */
	SPI_READY = 2,
	/**< Ready.                             */
	SPI_ACTIVE = 3,
	/**< Exchanging data.                   */
	SPI_COMPLETE = 4
				   /**< Asynchronous operation complete.   */
}
spistate_t;

struct bcm2835_spidev_s {
	struct spi_dev_s spidev;
	uint32_t base;
	uint8_t port;
	unsigned int freqid;
	uint32_t clock_divider;
	s32 gpio_clk;
	s32 gpio_nss;
	s32 gpio_miso;
	s32 gpio_mosi;

#ifndef CONFIG_SPI_POLLWAIT
	sem_t xfrsem;			/* Wait for transfer to complete */
#endif

#ifndef CONFIG_SPI_OWNBUS
	sem_t exclsem;
#endif

	void *rxbuf;
	const void *txbuf;
	size_t count;
	size_t tx_cnt;
	size_t rx_cnt;

	/* LoSSI enable
	 * 0 = The serial interface will behave as an SPI master.
	 * 1 = The serial interface will behave as a LoSSI master.
	 */
	uint8_t lossiEnabled;

	spistate_t state;
};

typedef enum {
	SPI_PORT0 = 0,
#if 0
	SPI_PORT1,
#endif
	SPI_PORT_MAX,
} SPI_PORT;

/* SPI driver local function */
static inline uint32_t bcm2835_rd(struct bcm2835_spidev_s *spi, unsigned reg);
static inline void bcm2835_wr(struct bcm2835_spidev_s *spi, unsigned reg, uint32_t val);
static inline void bcm2835_rd_fifo(struct bcm2835_spidev_s *spi);
static inline void bcm2835_wr_fifo(struct bcm2835_spidev_s *spi);
static void bcm2835_spi_reset_hw(struct bcm2835_spidev_s *spi);
static void bcm2835_spi_serve_interrupt(int irq, void *context, void *arg);

/* SPI Driver Methods */
static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd);
static void spi_exchange(FAR struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords);
static void spi_polled_exchange(FAR struct spi_dev_s *dev, const void *txbuffer, void *rxbuffer, size_t nwords);

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, const void *txbuffer, size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, void *rxbuffer, size_t nwords);
#endif

static void spi_dev_init(struct spi_dev_s *dev);
static void spi_irqinitialize(struct spi_dev_s *dev);
extern void bcm2835_delay(uint32_t n);

#ifdef __cplusplus
}
#endif
#endif							/* __ARCH_ARM_SRC_BCM2835_SPIDRV_H */
