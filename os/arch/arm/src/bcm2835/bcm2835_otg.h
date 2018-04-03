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
/************************************************************************************
 * arch/arm/src/bcm2835/bcm2835_otg.h
 *
 *   Copyright (C) 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           dev@ziggurat29.com
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_BCM2835_BCM2835_OTG_H
#define __ARCH_ARM_SRC_BCM2835_BCM2835_OTG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>
#include <arch/board/memory.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* General definitions */

#define OTG_EPTYPE_CTRL		(0) /* Control */
#define OTG_EPTYPE_ISOC		(1) /* Isochronous */
#define OTG_EPTYPE_BULK		(2) /* Bulk */
#define OTG_EPTYPE_INTR		(3) /* Interrupt */

#define OTG_PID_DATA0		(0)
#define OTG_PID_DATA2		(1)
#define OTG_PID_DATA1		(2)
#define OTG_PID_MDATA		(3) /* Non-control */
#define OTG_PID_SETUP		(3) /* Control */

/* Configuration ********************************************************************/
#define DWHCI_MAX_CHANNELS		16

#define DWHCI_DATA_FIFO_SIZE 		0x1000

//
// Core Registers
//
#define DWHCI_CORE_OTG_CTRL		(BCM2835_USB_CORE_BASE + 0x000)
	#define DWHCI_CORE_OTG_CTRL_HST_SET_HNP_EN	(1 << 10)
#define DWHCI_CORE_OTG_INT		(BCM2835_USB_CORE_BASE + 0x004)
#define DWHCI_CORE_AHB_CFG		(BCM2835_USB_CORE_BASE + 0x008)
	#define DWHCI_CORE_AHB_CFG_GLOBALINT_MASK	(1 << 0)
	#define DWHCI_CORE_AHB_CFG_MAX_AXI_BURST__SHIFT	1		// BCM2835 only
	#define DWHCI_CORE_AHB_CFG_MAX_AXI_BURST__MASK	(3 << 1)	// BCM2835 only
	#define DWHCI_CORE_AHB_CFG_WAIT_AXI_WRITES	(1 << 4)	// BCM2835 only
	#define DWHCI_CORE_AHB_CFG_DMAENABLE 		(1 << 5)
	#define DWHCI_CORE_AHB_CFG_AHB_SINGLE 		(1 << 23)
#define DWHCI_CORE_USB_CFG		(BCM2835_USB_CORE_BASE + 0x00C)
	#define DWHCI_CORE_USB_CFG_PHYIF		(1 << 3)
	#define DWHCI_CORE_USB_CFG_ULPI_UTMI_SEL	(1 << 4)
	#define DWHCI_CORE_USB_CFG_SRP_CAPABLE 		(1 << 8)
	#define DWHCI_CORE_USB_CFG_HNP_CAPABLE 		(1 << 9)
	#define DWHCI_CORE_USB_CFG_ULPI_FSLS		(1 << 17)
	#define DWHCI_CORE_USB_CFG_ULPI_CLK_SUS_M	(1 << 19)
	#define DWHCI_CORE_USB_CFG_ULPI_EXT_VBUS_DRV	(1 << 20)
	#define DWHCI_CORE_USB_CFG_TERM_SEL_DL_PULSE	(1 << 22)
#define DWHCI_CORE_RESET		(BCM2835_USB_CORE_BASE + 0x010)
	#define DWHCI_CORE_RESET_SOFT_RESET		(1 << 0)
	#define DWHCI_CORE_RESET_RX_FIFO_FLUSH		(1 << 4)
	#define DWHCI_CORE_RESET_TX_FIFO_FLUSH		(1 << 5)
	#define DWHCI_CORE_RESET_TX_FIFO_NUM__SHIFT	6
	#define DWHCI_CORE_RESET_TX_FIFO_NUM__MASK	(0x1F << 6)
	#define DWHCI_CORE_RESET_AHB_IDLE		(1 << 31)
#define DWHCI_CORE_INT_STAT		(BCM2835_USB_CORE_BASE + 0x014)
	#define DWHCI_CORE_INT_STAT_SOF_INTR		(1 << 3)
	#define DWHCI_CORE_INT_STAT_PORT_INTR		(1 << 24)
	#define DWHCI_CORE_INT_STAT_HC_INTR		(1 << 25)
#define DWHCI_CORE_INT_MASK		(BCM2835_USB_CORE_BASE + 0x018)
	#define DWHCI_CORE_INT_MASK_MODE_MISMATCH	(1 << 1)
	#define DWHCI_CORE_INT_MASK_SOF_INTR		(1 << 3)
	#define DWHCI_CORE_INT_MASK_RX_STS_Q_LVL	(1 << 4)
	#define DWHCI_CORE_INT_MASK_USB_SUSPEND		(1 << 11)
	#define DWHCI_CORE_INT_MASK_PORT_INTR		(1 << 24)
	#define DWHCI_CORE_INT_MASK_HC_INTR		(1 << 25)
	#define DWHCI_CORE_INT_MASK_CON_ID_STS_CHNG	(1 << 28)
	#define DWHCI_CORE_INT_MASK_DISCONNECT		(1 << 29)
	#define DWHCI_CORE_INT_MASK_SESS_REQ_INTR	(1 << 30)
	#define DWHCI_CORE_INT_MASK_WKUP_INTR		(1 << 31)
#define DWHCI_CORE_RX_STAT_RD		(BCM2835_USB_CORE_BASE + 0x01C)	// RO, slave mode only
#define DWHCI_CORE_RX_STAT_POP		(BCM2835_USB_CORE_BASE + 0x020)	// RO, slave mode only
	// for read and pop register in host mode
	#define DWHCI_CORE_RX_STAT_CHAN_NUMBER__MASK	0xF
	#define DWHCI_CORE_RX_STAT_BYTE_COUNT__SHIFT	4
	#define DWHCI_CORE_RX_STAT_BYTE_COUNT__MASK	(0x7FF << 4)
	#define DWHCI_CORE_RX_STAT_PACKET_STATUS__SHIFT	17
	#define DWHCI_CORE_RX_STAT_PACKET_STATUS__MASK	(0xF << 17)
		#define DWHCI_CORE_RX_STAT_PACKET_STATUS_IN			2
		#define DWHCI_CORE_RX_STAT_PACKET_STATUS_IN_XFER_COMP		3
		#define DWHCI_CORE_RX_STAT_PACKET_STATUS_DATA_TOGGLE_ERR	5
		#define DWHCI_CORE_RX_STAT_PACKET_STATUS_CHAN_HALTED		7
#define DWHCI_CORE_RX_FIFO_SIZ		(BCM2835_USB_CORE_BASE + 0x024)
#define DWHCI_CORE_NPER_TX_FIFO_SIZ	(BCM2835_USB_CORE_BASE + 0x028)
#define DWHCI_CORE_NPER_TX_STAT		(BCM2835_USB_CORE_BASE + 0x02C)	// RO
	#define DWHCI_CORE_NPER_TX_STAT_QUEUE_SPACE_AVL(reg)	(((reg) >> 16) & 0xFF)
#define DWHCI_CORE_I2C_CTRL		(BCM2835_USB_CORE_BASE + 0x030)
#define DWHCI_CORE_PHY_VENDOR_CTRL	(BCM2835_USB_CORE_BASE + 0x034)
#define DWHCI_CORE_GPIO			(BCM2835_USB_CORE_BASE + 0x038)
#define DWHCI_CORE_USER_ID		(BCM2835_USB_CORE_BASE + 0x03C)
#define DWHCI_CORE_VENDOR_ID		(BCM2835_USB_CORE_BASE + 0x040)
#define DWHCI_CORE_HW_CFG1		(BCM2835_USB_CORE_BASE + 0x044)	// RO
#define DWHCI_CORE_HW_CFG2		(BCM2835_USB_CORE_BASE + 0x048)	// RO
	#define DWHCI_CORE_HW_CFG2_OP_MODE(reg)			(((reg) >> 0) & 7)
	#define DWHCI_CORE_HW_CFG2_ARCHITECTURE(reg)		(((reg) >> 3) & 3)
	#define DWHCI_CORE_HW_CFG2_HS_PHY_TYPE(reg)		(((reg) >> 6) & 3)
		#define DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_NOT_SUPPORTED		0
		#define DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_UTMI			1
		#define DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_ULPI			2
		#define DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_UTMI_ULPI		3
	#define DWHCI_CORE_HW_CFG2_FS_PHY_TYPE(reg)		(((reg) >> 8) & 3)
		#define DWHCI_CORE_HW_CFG2_FS_PHY_TYPE_DEDICATED		1
	#define DWHCI_CORE_HW_CFG2_NUM_HOST_CHANNELS(reg)	((((reg) >> 14) & 0xF) + 1)
#define DWHCI_CORE_HW_CFG3		(BCM2835_USB_CORE_BASE + 0x04C)	// RO
	#define DWHCI_CORE_HW_CFG3_DFIFO_DEPTH(reg)	(((reg) >> 16) & 0xFFFF)
#define DWHCI_CORE_HW_CFG4		(BCM2835_USB_CORE_BASE + 0x050)	// RO
	#define DWHCI_CORE_HW_CFG4_DED_FIFO_EN			(1 << 25)
	#define DWHCI_CORE_HW_CFG4_NUM_IN_EPS(reg)		(((reg) >> 26) & 0xF)
#define DWHCI_CORE_LPM_CFG		(BCM2835_USB_CORE_BASE + 0x054)
#define DWHCI_CORE_POWER_DOWN		(BCM2835_USB_CORE_BASE + 0x058)
#define DWHCI_CORE_DFIFO_CFG		(BCM2835_USB_CORE_BASE + 0x05C)
	#define DWHCI_CORE_DFIFO_CFG_EPINFO_BASE__SHIFT		16
	#define DWHCI_CORE_DFIFO_CFG_EPINFO_BASE__MASK		(0xFFFF << 16)
#define DWHCI_CORE_ADP_CTRL		(BCM2835_USB_CORE_BASE + 0x060)
// gap
#define DWHCI_VENDOR_MDIO_CTRL		(BCM2835_USB_CORE_BASE + 0x080)		// BCM2835 only
#define DWHCI_VENDOR_MDIO_DATA		(BCM2835_USB_CORE_BASE + 0x084)		// BCM2835 only
#define DWHCI_VENDOR_VBUS_DRV		(BCM2835_USB_CORE_BASE + 0x088)		// BCM2835 only
// gap
#define DWHCI_CORE_HOST_PER_TX_FIFO_SIZ (BCM2835_USB_CORE_BASE + 0x100)
// fifo := 0..14 :
#define DWHCI_CORE_DEV_PER_TX_FIFO(fifo) (BCM2835_USB_CORE_BASE + 0x104 + (fifo)*4)	// dedicated FIFOs on
#define DWHCI_CORE_DEV_TX_FIFO(fifo)	(BCM2835_USB_CORE_BASE + 0x104 + (fifo)*4)	// dedicated FIFOs off

//
// Host Registers
//
#define DWHCI_HOST_CFG			(BCM2835_USB_HOST_BASE + 0x000)
	#define DWHCI_HOST_CFG_FSLS_PCLK_SEL__SHIFT	0
	#define DWHCI_HOST_CFG_FSLS_PCLK_SEL__MASK	(3 << 0)
		#define DWHCI_HOST_CFG_FSLS_PCLK_SEL_30_60_MHZ	0
		#define DWHCI_HOST_CFG_FSLS_PCLK_SEL_48_MHZ	1
		#define DWHCI_HOST_CFG_FSLS_PCLK_SEL_6_MHZ	2
#define DWHCI_HOST_FRM_INTERVAL		(BCM2835_USB_HOST_BASE + 0x004)
#define DWHCI_HOST_FRM_NUM		(BCM2835_USB_HOST_BASE + 0x008)
	#define DWHCI_HOST_FRM_NUM_NUMBER(reg)			((reg) & 0xFFFF)
		#define DWHCI_MAX_FRAME_NUMBER			0x3FFF
	#define DWHCI_HOST_FRM_NUM_REMAINING(reg)		(((reg) >> 16) & 0xFFFF)
// gap
#define DWHCI_HOST_PER_TX_FIFO_STAT	(BCM2835_USB_HOST_BASE + 0x010)
#define DWHCI_HOST_ALLCHAN_INT		(BCM2835_USB_HOST_BASE + 0x014)
#define DWHCI_HOST_ALLCHAN_INT_MASK	(BCM2835_USB_HOST_BASE + 0x018)
#define DWHCI_HOST_FRMLST_BASE		(BCM2835_USB_HOST_BASE + 0x01C)
// gap
#define DWHCI_HOST_PORT 		(BCM2835_USB_HOST_BASE + 0x040)
	#define DWHCI_HOST_PORT_CONNECT				(1 << 0)
	#define DWHCI_HOST_PORT_CONNECT_CHANGED			(1 << 1)
	#define DWHCI_HOST_PORT_ENABLE				(1 << 2)
	#define DWHCI_HOST_PORT_ENABLE_CHANGED			(1 << 3)
	#define DWHCI_HOST_PORT_OVERCURRENT			(1 << 4)
	#define DWHCI_HOST_PORT_OVERCURRENT_CHANGED		(1 << 5)
	#define DWHCI_HOST_PORT_RESET				(1 << 8)
	#define DWHCI_HOST_PORT_POWER				(1 << 12)
	#define DWHCI_HOST_PORT_SPEED(reg)			(((reg) >> 17) & 3)
		#define DWHCI_HOST_PORT_SPEED_HIGH		0
		#define DWHCI_HOST_PORT_SPEED_FULL		1
		#define DWHCI_HOST_PORT_SPEED_LOW		2
	#define DWHCI_HOST_PORT_DEFAULT_MASK			(  DWHCI_HOST_PORT_CONNECT_CHANGED \
								 | DWHCI_HOST_PORT_ENABLE	   \
								 | DWHCI_HOST_PORT_ENABLE_CHANGED  \
								 | DWHCI_HOST_PORT_OVERCURRENT_CHANGED)
// gap
// chan := 0..15 :
#define DWHCI_HOST_CHAN_CHARACTER(chan)	(BCM2835_USB_HOST_BASE + 0x100 + (chan)*0x20)
	#define DWHCI_HOST_CHAN_CHARACTER_MAX_PKT_SIZ__MASK	0x7FF
	#define DWHCI_HOST_CHAN_CHARACTER_EP_NUMBER__SHIFT	11
	#define DWHCI_HOST_CHAN_CHARACTER_EP_NUMBER__MASK	(0xF << 11)
	#define DWHCI_HOST_CHAN_CHARACTER_EP_DIRECTION_IN	(1 << 15)
	#define DWHCI_HOST_CHAN_CHARACTER_LOW_SPEED_DEVICE	(1 << 17)
	#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE__SHIFT	18
	#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE__MASK		(3 << 18)
		#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE_CONTROL	0
		#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE_ISO		1
		#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE_BULK		2
		#define DWHCI_HOST_CHAN_CHARACTER_EP_TYPE_INTERRUPT	3
	#define DWHCI_HOST_CHAN_CHARACTER_MULTI_CNT__SHIFT	20
	#define DWHCI_HOST_CHAN_CHARACTER_MULTI_CNT__MASK	(3 << 20)
	#define DWHCI_HOST_CHAN_CHARACTER_DEVICE_ADDRESS__SHIFT	22
	#define DWHCI_HOST_CHAN_CHARACTER_DEVICE_ADDRESS__MASK	(0x7F << 22)
	#define DWHCI_HOST_CHAN_CHARACTER_PER_ODD_FRAME		(1 << 29)
	#define DWHCI_HOST_CHAN_CHARACTER_DISABLE		(1 << 30)
	#define DWHCI_HOST_CHAN_CHARACTER_ENABLE		(1 << 31)
#define DWHCI_HOST_CHAN_SPLIT_CTRL(chan) (BCM2835_USB_HOST_BASE + 0x104 + (chan)*0x20)
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_PORT_ADDRESS__MASK	0x7F
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_HUB_ADDRESS__SHIFT	7
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_HUB_ADDRESS__MASK	(0x7F << 7)
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_XACT_POS__SHIFT	14
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_XACT_POS__MASK	(3 << 14)
		#define DWHCI_HOST_CHAN_SPLIT_CTRL_ALL		3
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_COMPLETE_SPLIT	(1 << 16)
	#define DWHCI_HOST_CHAN_SPLIT_CTRL_SPLIT_ENABLE		(1 << 31)
#define DWHCI_HOST_CHAN_INT(chan)	(BCM2835_USB_HOST_BASE + 0x108 + (chan)*0x20)
	#define DWHCI_HOST_CHAN_INT_XFER_COMPLETE		(1 << 0)
	#define DWHCI_HOST_CHAN_INT_HALTED			(1 << 1)
	#define DWHCI_HOST_CHAN_INT_AHB_ERROR			(1 << 2)
	#define DWHCI_HOST_CHAN_INT_STALL			(1 << 3)
	#define DWHCI_HOST_CHAN_INT_NAK				(1 << 4)
	#define DWHCI_HOST_CHAN_INT_ACK				(1 << 5)
	#define DWHCI_HOST_CHAN_INT_NYET			(1 << 6)
	#define DWHCI_HOST_CHAN_INT_XACT_ERROR			(1 << 7)
	#define DWHCI_HOST_CHAN_INT_BABBLE_ERROR		(1 << 8)
	#define DWHCI_HOST_CHAN_INT_FRAME_OVERRUN		(1 << 9)
	#define DWHCI_HOST_CHAN_INT_DATA_TOGGLE_ERROR		(1 << 10)
	#define DWHCI_HOST_CHAN_INT_ERROR_MASK			(  DWHCI_HOST_CHAN_INT_AHB_ERROR     \
								 | DWHCI_HOST_CHAN_INT_STALL	     \
								 | DWHCI_HOST_CHAN_INT_XACT_ERROR    \
								 | DWHCI_HOST_CHAN_INT_BABBLE_ERROR  \
								 | DWHCI_HOST_CHAN_INT_FRAME_OVERRUN \
								 | DWHCI_HOST_CHAN_INT_DATA_TOGGLE_ERROR)
#define DWHCI_HOST_CHAN_INT_MASK(chan)	(BCM2835_USB_HOST_BASE + 0x10C + (chan)*0x20)
#define DWHCI_HOST_CHAN_XFER_SIZ(chan)	(BCM2835_USB_HOST_BASE + 0x110 + (chan)*0x20)
	#define DWHCI_HOST_CHAN_XFER_SIZ_BYTES__MASK		0x7FFFF
	#define DWHCI_HOST_CHAN_XFER_SIZ_PACKETS__SHIFT		19
	#define DWHCI_HOST_CHAN_XFER_SIZ_PACKETS__MASK		(0x3FF << 19)
	#define DWHCI_HOST_CHAN_XFER_SIZ_PACKETS(reg)		(((reg) >> 19) & 0x3FF)
	#define DWHCI_HOST_CHAN_XFER_SIZ_PID__SHIFT		29
	#define DWHCI_HOST_CHAN_XFER_SIZ_PID__MASK		(3 << 29)
	#define DWHCI_HOST_CHAN_XFER_SIZ_PID(reg)		(((reg) >> 29) & 3)
		#define DWHCI_HOST_CHAN_XFER_SIZ_PID_DATA0 	0
		#define DWHCI_HOST_CHAN_XFER_SIZ_PID_DATA1 	2
		#define DWHCI_HOST_CHAN_XFER_SIZ_PID_DATA2 	1
		#define DWHCI_HOST_CHAN_XFER_SIZ_PID_MDATA 	3	// non-control transfer
		#define DWHCI_HOST_CHAN_XFER_SIZ_PID_SETUP 	3
#define DWHCI_HOST_CHAN_DMA_ADDR(chan)	(BCM2835_USB_HOST_BASE + 0x114 + (chan)*0x20)
// gap
#define DWHCI_HOST_CHAN_DMA_BUF(chan)	(BCM2835_USB_HOST_BASE + 0x11C + (chan)*0x20)	// DDMA only

//
// Data FIFOs (non-DMA mode only)
//
#define DWHCI_DATA_FIFO(chan)		(BCM2835_USB_HOST_BASE + 0x1000 + (chan)*DWHCI_DATA_FIFO_SIZE)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: bcm2835_otghost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
struct usbhost_connection_s;
FAR struct usbhost_connection_s *bcm2835_otghost_initialize(int controller);
#endif

/************************************************************************************
 * Name:  bcm2835_usbsuspend
 *
 * Description:
 *   Board logic must provide the bcm2835_usbsuspend logic if the OTG FS device driver
 *   is used.  This function is called whenever the USB enters or leaves suspend
 *   mode. This is an opportunity for the board logic to shutdown clocks, power,
 *   etc. while the USB is suspended.
 *
 ************************************************************************************/

void bcm2835_usbsuspend(FAR struct usbdev_s *dev, bool resume);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_BCM2835_BCM2835_OTG_H */
