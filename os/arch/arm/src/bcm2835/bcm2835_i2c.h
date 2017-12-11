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
 * arch/arm/src/bcm2835/bcm2835_i2c.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_BCM2835_I2C_H
#define __ARCH_ARM_SRC_BCM2835_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <tinyara/i2c.h>

#include "chip.h"

// *****************************************************************************
//                 Broadcom Serial Controllers (BSC/I2C)
// *****************************************************************************

/* Only BSC0 is accessible from the RPi pi header.*/
#define BSC0_ADDR 0x20205000
#define BSC1_ADDR 0x20804000
#define BSC2_ADDR 0x20805000

#define BSC_CLOCK_FREQ 150000000

/* I2C register map offset */
#define BSC_CONTROL		0x00
#define BSC_STATUS		0x04
#define BSC_DATA_LENGTH		0x08
#define BSC_SLAVE_ADDRESS	0x0C
#define BSC_DATA_FIFO		0x10
#define BSC_CLOCK_DIVIDER	0x14
#define BSC_DATA_DELAY		0x18
#define BSC_CLOCK_STRETCH_TIMEOUT	0x1C

/* I2C control flags */
#define BSC_I2CEN BIT(15)
#define BSC_INTR  BIT(10)
#define BSC_INTT  BIT(9)
#define BSC_INTD  BIT(8)
#define BSC_ST    BIT(7)
#define BSC_CLEAR BIT(4)
#define BSC_READ  BIT(0)

/* I2C status flags */
#define BSC_TA   BIT(0)	/** @brief Transfer active.*/
#define BSC_DONE BIT(1)	/** @brief Transfer done.*/
#define BSC_TXW  BIT(2)	/** @brief FIFO needs writing.*/
#define BSC_RXR  BIT(3)	/** @brief FIFO needs reading.*/
#define BSC_TXD  BIT(4)	/** @brief FIFO can accept data.*/
#define BSC_RXD  BIT(5)	/** @brief FIFO contains data.*/
#define BSC_TXE  BIT(6)	/** @brief FIFO empty.*/
#define BSC_RXF  BIT(7)	/** @brief FIFO full.*/
#define BSC_ERR  BIT(8)	/** @brief ACK error.*/
#define BSC_CLKT BIT(9)	/** @brief Clock stretch timeout.*/

/* Rising/Falling Edge Delay Defaults.*/
#define BSC_DEFAULT_FEDL       0x30
#define BSC_DEFAULT_REDL       0x30

/* Clock Stretch Timeout Defaults.*/
#define BSC_DEFAULT_CLKT       0x40

#define CLEAR_STATUS  BSC_CLKT|BSC_ERR|BSC_DONE

#define START_READ    BSC_I2CEN|BSC_ST|BSC_CLEAR|BSC_READ
#define START_WRITE   BSC_I2CEN|BSC_ST

#define I2C_SPEED_400KHZ  400000
#define I2C_MASTER      0
#define I2C_SLAVE_MODE  1
#define I2C_POLLING     0
/* #define I2C_INTERRUPT   1 */

enum slave_status {
	SLAVE_IDLE,
	SLAVE_GET_REG,
	SLAVE_GET_DATA,
	SLAVE_SET_DATA,
};

struct slave_data {
	enum slave_status status;
	u32 current_reg;
	u8 data[20];
};

struct master_data {
	struct i2c_msg_s *msg;
	int num;
	int cur_msg;
	int buf_count;
	/*  struct completion done; */
};

/* I2C Device hardware configuration */
struct bcm2835_i2c_config_s {
	uintptr_t base;				/* I2C base address */
	unsigned int scl_pin;		/* GPIO configuration for SCL as SCL */
	unsigned int sda_pin;		/* GPIO configuration for SDA as SDA */
	int (*isr)(int, void *, void *);	/* Interrupt handler */
	unsigned int irq;			/* IRQ number */
	uint8_t devno;				/* I2Cn where n = devno */
};

/* I2C Device Private Data */
struct bcm2835_i2c_priv_s {
	const struct i2c_ops_s *ops;	/* Standard I2C operations */
	const struct bcm2835_i2c_config_s *config;	/* Port configuration */
	sem_t exclsem;				/* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
	sem_t waitsem;				/* Interrupt wait semaphore */
#endif
	uint8_t refs;				/* Reference count */
	volatile uint8_t intstate;	/* Interrupt handshake (see enum s5j_intstate_e) */

	int state;
	int clock;
	int xfer_speed;
	unsigned int master;
	unsigned int mode;
	unsigned int slave_addr;
	unsigned int addrlen;
	unsigned int timeout;
	char name[16];

	unsigned int initialized;
	unsigned int retries;
	/*  master data  */
	uint8_t msgc;				/* Message count */
	struct i2c_msg_s *msgv;		/* Message list */
	uint8_t *mptr;				/* Current message buffer */
	int mcnt;					/* Current message length */
	uint16_t mflags;			/* Current message flags */

	struct i2c_msg_s *msg;

	/* interrupt */
	struct slave_data *slave_test_data;
	struct master_data *master_test_data;

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void i2c_calculate_timing(unsigned int base, unsigned int nPclk, unsigned int nOpClk);
static void i2c_enable_int(unsigned int base, unsigned int bit);
static void i2c_disable_int(unsigned int base, unsigned int bit);
static void i2c_clear_int(unsigned int base, unsigned int bit);
static unsigned int i2c_read_int_status(unsigned int base);
static void i2c_set_slave_addr(unsigned int base, u16 addr, unsigned int is_master);
static int i2c_manual_fast_init(struct bcm2835_i2c_priv_s *priv);
static int i2c_wait_xfer_done(struct bcm2835_i2c_priv_s *priv);
static int i2c_wait_rcv_done(struct bcm2835_i2c_priv_s *priv);
static void i2c_start(struct bcm2835_i2c_priv_s *priv);
static void i2c_stop(struct bcm2835_i2c_priv_s *priv);
static void i2c_repstart(struct bcm2835_i2c_priv_s *priv);
static int i2c_outb(struct bcm2835_i2c_priv_s *priv, u8 data);
static int i2c_inb(struct bcm2835_i2c_priv_s *priv, bool is_ack);
static int sendbytes(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg);
static int readbytes(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg);
static int try_address(struct bcm2835_i2c_priv_s *priv, u8 addr, int retries);
static int do_address(struct bcm2835_i2c_priv_s *priv, struct i2c_msg_s *msg);
static int i2c_master_handler(void *args);
static void i2c_master_setup(struct bcm2835_i2c_priv_s *priv, unsigned int mode, unsigned int speed, unsigned int slave_addr);
static void i2c_slave_setup(struct bcm2835_i2c_priv_s *priv, unsigned int mode, unsigned int speed, unsigned int slave_addr);
static void i2c_master_cleanup(struct bcm2835_i2c_priv_s *priv);
static void i2c_slave_cleanup(struct bcm2835_i2c_priv_s *priv);
static int i2c_setup(struct bcm2835_i2c_priv_s *priv, unsigned int master, unsigned int mode, unsigned int speed, unsigned int slave_addr);
static int i2c_cleanup(struct bcm2835_i2c_priv_s *priv);
static inline void bcm2835_i2c_sem_wait(struct bcm2835_i2c_priv_s *priv);
static inline void bcm2835_i2c_sem_post(struct bcm2835_i2c_priv_s *priv);
static inline void bcm2835_i2c_sem_init(struct bcm2835_i2c_priv_s *priv);
static inline void bcm2835_i2c_sem_destroy(struct bcm2835_i2c_priv_s *priv);
static int bcm2835_i2c_interrupt(struct bcm2835_i2c_priv_s *priv, unsigned int status);
static int bcm2835_i2c0_interrupt(int irq, void *context, void *arg);
static int bcm2835_i2c1_interrupt(int irq, void *context, void *arg);
static int bcm2835_i2c_initialize(struct bcm2835_i2c_priv_s *priv, unsigned int frequency);
static int bcm2835_i2c_uninitialize(struct bcm2835_i2c_priv_s *priv);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
unsigned int bcm2835_i2c_setclock(struct i2c_dev_s *dev, unsigned int frequency);
int bcm2835_i2c_setownaddress(FAR struct i2c_dev_s *dev, int addr, int nbits);
int bcm2835_i2c_transfer(struct i2c_dev_s *dev, struct i2c_msg_s *msgv, int msgc);
int bcm2835_i2c_read(FAR struct i2c_dev_s *dev, FAR uint8_t *buffer, int buflen);
int bcm2835_i2c_write(FAR struct i2c_dev_s *dev, FAR const uint8_t *buffer, int buflen);
struct i2c_dev_s *up_i2cinitialize(int port);
int bcm2835_i2cbus_uninitialize(FAR struct i2c_dev_s *dev);
void bcm2835_i2c_register(int bus);

#endif							/* __ARCH_ARM_SRC_BCM2835_I2C_H */
