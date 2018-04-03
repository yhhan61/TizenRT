/****************************************************************************
 * arch/arm/src/bcm2835/bcm2835_otghost.c
 *
 *   Copyright (C) 2012-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            dev@ziggurat29.com
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>
#include <tinyara/clock.h>
#include <tinyara/semaphore.h>
#include <tinyara/usb/usb.h>
#include <tinyara/usb/usbhost.h>
#include <tinyara/usb/usbhost_devaddr.h>
#include <tinyara/usb/usbhost_trace.h>

#include <tinyara/irq.h>

#include <arch/armv6/cacheops.h>

#include "chip.h"		/* Includes default GPIO settings */
#include <arch/board/board.h>	/* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "bcm2835_propertytags.h"
#include "bcm2835_usbhost.h"

//#define hbahn_dbg	lldbg
#define hbahn_dbg(x...)

#define hbahn_dbg1	lldbg

#if defined(CONFIG_USBHOST) && defined(CONFIG_BCM2835_OTG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ***************************************************************/
/* BCM2835 USB OTG Host Driver Support
 *
 * Pre-requisites
 *
 *  CONFIG_USBHOST      - Enable general USB host support
 *  CONFIG_BCM2835_OTG  - Enable the BCM2835 USB OTG block
 *  CONFIG_BCM2835_SYSCFG - Needed
 *
 * Options:
 *
 *  CONFIG_BCM2835_OTG_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
 *    Default 128 (512 bytes)
 *  CONFIG_BCM2835_OTG_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
 *    in 32-bit words.  Default 96 (384 bytes)
 *  CONFIG_BCM2835_OTG_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
 *    words.  Default 96 (384 bytes)
 *  CONFIG_BCM2835_OTG_DESCSIZE - Maximum size of a descriptor.  Default: 128
 *  CONFIG_BCM2835_OTG_SOFINTR - Enable SOF interrupts.  Why would you ever
 *    want to do that?
 *  CONFIG_BCM2835_USBHOST_REGDEBUG - Enable very low-level register access
 *    debug.  Depends on CONFIG_DEBUG.
 *  CONFIG_BCM2835_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
 *    packets. Depends on CONFIG_DEBUG.
 */

/* Pre-requisites (partial) */

#define CONFIG_BCM2835_SYSCFG 1
#ifndef CONFIG_BCM2835_SYSCFG
#error "CONFIG_BCM2835_SYSCFG is required"
#endif

/* for set_power_state_on */
#define DEVICE_ID_USB_HCD	3

/*
 * Configuration
 */
#define DWC_CFG_DYNAMIC_FIFO				/* re-program FIFOs with these sizes: */
	#define DWC_CFG_HOST_RX_FIFO_SIZE	1024	/* number of 32 bit words */
	#define DWC_CFG_HOST_NPER_TX_FIFO_SIZE	1024	/* number of 32 bit words */
	#define DWC_CFG_HOST_PER_TX_FIFO_SIZE	1024	/* number of 32 bit words */

/* Default RxFIFO size */

#ifndef CONFIG_BCM2835_OTG_RXFIFO_SIZE
#define CONFIG_BCM2835_OTG_RXFIFO_SIZE 128
#endif

/* Default host non-periodic Tx FIFO size */

#ifndef CONFIG_BCM2835_OTG_NPTXFIFO_SIZE
#define CONFIG_BCM2835_OTG_NPTXFIFO_SIZE 96
#endif

/* Default host periodic Tx fifo size register */

#ifndef CONFIG_BCM2835_OTG_PTXFIFO_SIZE
#define CONFIG_BCM2835_OTG_PTXFIFO_SIZE 96
#endif

/* Maximum size of a descriptor */

#ifndef CONFIG_BCM2835_OTG_DESCSIZE
#define CONFIG_BCM2835_OTG_DESCSIZE 128
#endif

/* Register/packet debug depends on CONFIG_DEBUG */

#ifndef CONFIG_DEBUG
#undef CONFIG_BCM2835_USBHOST_REGDEBUG
#undef CONFIG_BCM2835_USBHOST_PKTDUMP
#else
#define CONFIG_BCM2835_USBHOST_PKTDUMP 1
#endif

/* HCD Setup *******************************************************************/
/* Hardware capabilities */
#define DWHCI_MAX_CHANNELS		16
#define DWHCI_DATA_FIFO_SIZE		0x1000

//XXX I think this needs to be 12 for the 'L4
#define BCM2835_NHOST_CHANNELS		8	/* Number of host channels */
#define BCM2835_MAX_PACKET_SIZE		64	/* Full speed max packet size */
#define BCM2835_EP0_DEF_PACKET_SIZE	8	/* EP0 default packet size */
#define BCM2835_EP0_MAX_PACKET_SIZE	64	/* EP0 max packet size */
#define BCM2835_MAX_TX_FIFOS		15	/* Max number of TX FIFOs */
#define BCM2835_MAX_PKTCOUNT		256	/* Max packet count */
#define BCM2835_RETRY_COUNT		3	/* Number of ctrl transfer retries */
#define BCM2835_FRAME_UNSET		(DWHCI_MAX_FRAME_NUMBER+1)

/* Delays **********************************************************************/

#define BCM2835_READY_DELAY		200000	/* In loop counts */
#define BCM2835_FLUSH_DELAY		200000	/* In loop counts */
#define BCM2835_SETUP_DELAY		SEC2TICK(5)	/* 5 seconds in system ticks */
//#define BCM2835_DATANAK_DELAY		SEC2TICK(5)	/* 5 seconds in system ticks */
#define BCM2835_DATANAK_DELAY		MSEC2TICK(100)	/* 100 mili seconds in system ticks */

/* Ever-present MIN/MAX macros */

#ifndef MIN
#define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The following enumeration represents the various states of the USB host
 * state machine (for debug purposes only)
 */

enum bcm2835_smstate_e {
	SMSTATE_DETACHED = 0,			/* Not attached to a device */
	SMSTATE_ATTACHED,			/* Attached to a device */
	SMSTATE_ENUM,				/* Attached, enumerating */
	SMSTATE_CLASS_BOUND,			/* Enumeration complete, class bound */
};

/* This enumeration provides the reason for the channel halt. */

enum bcm2835_chreason_e {
	CHREASON_IDLE = 0,			/* Inactive (initial state) */
	CHREASON_FREED,				/* Channel is no longer in use */
	CHREASON_XFRC,				/* Transfer complete */
	CHREASON_NAK,				/* NAK received */
	CHREASON_NYET,				/* NotYet received */
	CHREASON_STALL,				/* Endpoint stalled */
	CHREASON_TXERR,				/* Transfer error received */
	CHREASON_DTERR,				/* Data toggle error received */
	CHREASON_FRMOR,				/* Frame overrun */
	CHREASON_CANCELLED			/* Transfer cancelled */
};

/* BCM2835 channel stage substate, This is used for check channel operation stage status 
 * This substate vale used with member status of structure bcm2835_chan_s
 */
enum stage_substate_i {
	STAGE_SUBSTATE_WAIT_FOR_CHANNEL_DISABLE = 0,
	STAGE_SUBSTATE_WAIT_FOR_TRANSACTION_COMPLETE,
	STAGE_SUBSTATE_UNKNOWN
};

/* BCM2835 channel stage mstate, This is used for check channel operation stage status 
 * This mstate vale used with member status of structure bcm2835_chan_s
 */
enum stage_mstate_i {
	STAGE_STATE_NO_SPLIT_TRANSFER = 0,
	STAGE_STATE_START_SPLIT,
	STAGE_STATE_COMPLETE_SPLIT,
	STAGE_STATE_PERIODIC_DELAY,
	STAGE_STATE_UNKNOWN
};


/* This structure retains the state of one host channel.  NOTE: Since there
 * is only one channel operation active at a time, some of the fields in
 * in the structure could be moved in struct bcm2835_ubhost_s to achieve
 * some memory savings.
 */

struct bcm2835_chan_s {
	sem_t waitsem;				/* Channel wait semaphore */
	volatile uint8_t result;		/* The result of the transfer */
	volatile uint8_t chreason;		/* Channel halt reason. See enum bcm2835_chreason_e */
	uint8_t chidx;				/* Channel index */
	uint8_t epno;				/* Device endpoint number (0-127) */
	uint8_t eptype;				/* See OTG_EPTYPE_* definitions */
	uint8_t funcaddr;			/* Device function address */
	uint8_t speed;				/* Device speed */
	uint8_t interval;			/* Interrupt/isochronous EP polling interval */
	uint8_t pid;				/* Data PID */
	uint8_t npackets;			/* Number of packets (for data toggle) */
	uint8_t substate;			/* Channel stage sub state for BCM2835 */
	uint8_t mstate;				/* Channel stage master state for BCM2835 */
	bool inuse;				/* True: This channel is "in use" */
	volatile bool indata1;			/* IN data toggle. True: DATA01 (Bulk and INTR only) */
	volatile bool outdata1;			/* OUT data toggle.  True: DATA01 */
	bool in;				/* True: IN endpoint */
	volatile bool waiter;			/* True: Thread is waiting for a channel event */
	uint16_t maxpacket;			/* Max packet size */
	uint16_t buflen;			/* Buffer length (at start of transfer) */
	volatile uint16_t xfrd;			/* Bytes transferred (at end of transfer) */
	volatile uint16_t inflight;		/* Number of Tx bytes "in-flight" */
	FAR uint8_t *buffer;			/* Transfer buffer pointer */
#ifdef CONFIG_USBHOST_ASYNCH
	usbhost_asynch_t callback;		/* Transfer complete callback */
	FAR void *arg;				/* Argument that accompanies the callback */
#endif
};

/* A channel represents on uni-directional endpoint.  So, in the case of the
 * bi-directional, control endpoint, there must be two channels to represent
 * the endpoint.
 */

struct bcm2835_ctrlinfo_s {
	uint8_t inndx;				/* EP0 IN control channel index */
	uint8_t outndx;				/* EP0 OUT control channel index */
};

/* This structure retains the state of the USB host controller */

struct bcm2835_usbhost_s {
	/* Common device fields.  This must be the first thing defined in the
	 * structure so that it is possible to simply cast from struct usbhost_s
	 * to structbcm2835_usbhost_s.
	 */
	struct usbhost_driver_s drvr;

	/* This is the hub port description understood by class drivers */
	struct usbhost_roothubport_s rhport;

	/* Overall driver status */
	volatile uint8_t smstate;		/* The state of the USB host state machine */
	uint8_t chidx;				/* ID of channel waiting for space in Tx FIFO */
	volatile bool connected;		/* Connected to device */
	volatile bool change;			/* Connection change */
	volatile bool pscwait;			/* True: Thread is waiting for a port event */
	sem_t exclsem;				/* Support mutually exclusive access */
	sem_t pscsem;				/* Semaphore to wait for a port event */
	struct bcm2835_ctrlinfo_s ep0;		/* Root hub port EP0 description */

#ifdef CONFIG_USBHOST_HUB
	/* Used to pass external hub port events */
	volatile struct usbhost_hubport_s *hport;
#endif

	/* Channel control information */
	uint32_t channels;			/* maximum number of channels */
        volatile uint32_t channel_allocated;	/* one bit per channel, set if allocated */

	uint32_t nextframe;			/* next frame number */

	/* The state of each host channel */
	struct bcm2835_chan_s chan[BCM2835_MAX_TX_FIFOS];

	FAR uint8_t buffer[BCM2835_MAX_PACKET_SIZE]; /* user buffer with Full speed max packet size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_BCM2835_USBHOST_REGDEBUG
static void bcm2835_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void bcm2835_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t bcm2835_getreg(uint32_t addr);
static void bcm2835_putreg(uint32_t addr, uint32_t value);
#else
#define bcm2835_getreg(addr)     getreg32(addr)
#define bcm2835_putreg(addr,val) putreg32(val,addr)
#endif

static inline void bcm2835_modifyreg(uint32_t addr, uint32_t clrbits, uint32_t setbits);

#ifdef CONFIG_BCM2835_USBHOST_PKTDUMP
#define bcm2835_pktdump(m,b,n) lib_dumpbuffer(m,b,n)
#else
#define bcm2835_pktdump(m,b,n)
#endif

/* Semaphores ******************************************************************/

static void bcm2835_takesem(FAR sem_t *sem);
#define bcm2835_givesem(s) sem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t bcm2835_getle16(FAR const uint8_t *val);

/* Channel management **********************************************************/

static int bcm2835_chan_alloc(FAR struct bcm2835_usbhost_s *priv);
static inline void bcm2835_chan_free(FAR struct bcm2835_usbhost_s *priv, int chidx);
static inline void bcm2835_chan_freeall(FAR struct bcm2835_usbhost_s *priv);
static void bcm2835_chan_configure(FAR struct bcm2835_usbhost_s *priv, int chidx);
static void bcm2835_chan_halt(FAR struct bcm2835_usbhost_s *priv, int chidx, enum bcm2835_chreason_e chreason);
static int bcm2835_chan_waitsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_chan_asynchsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan, usbhost_asynch_t callback, FAR void *arg);
#endif
static int bcm2835_chan_wait(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
static void bcm2835_chan_wakeup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
static int bcm2835_ctrlchan_alloc(FAR struct bcm2835_usbhost_s *priv, uint8_t epno, uint8_t funcaddr, uint8_t speed, FAR struct bcm2835_ctrlinfo_s *ctrlep);
static int bcm2835_ctrlep_alloc(FAR struct bcm2835_usbhost_s *priv, FAR const struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep);
static int bcm2835_xfrep_alloc(FAR struct bcm2835_usbhost_s *priv, FAR const struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep);

/* Control/data transfer logic *************************************************/

static void bcm2835_wait_for_frame(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
static void bcm2835_start_channel(FAR struct bcm2835_usbhost_s *priv, int chidx);
static void bcm2835_start_transaction(FAR struct bcm2835_usbhost_s *priv, int chidx);
static void bcm2835_transfer_start(FAR struct bcm2835_usbhost_s *priv, int chidx);
#if 0							/* Not used */
static inline uint16_t bcm2835_getframe(void);
#endif
static int bcm2835_ctrl_sendsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR const struct usb_ctrlreq_s *req);
static int bcm2835_ctrl_senddata(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR uint8_t *buffer, unsigned int buflen);
static int bcm2835_ctrl_recvdata(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR uint8_t *buffer, unsigned int buflen);
static int bcm2835_in_setup(FAR struct bcm2835_usbhost_s *priv, int chidx);
static ssize_t bcm2835_in_transfer(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void bcm2835_in_next(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
static int bcm2835_in_asynch(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg);
#endif
static int bcm2835_out_setup(FAR struct bcm2835_usbhost_s *priv, int chidx);
static ssize_t bcm2835_out_transfer(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void bcm2835_out_next(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan);
static int bcm2835_out_asynch(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg);
#endif

/* Interrupt handling **********************************************************/
/* Lower level interrupt handlers */

static void bcm2835_gint_wrpacket(FAR struct bcm2835_usbhost_s *priv, FAR uint8_t *buffer, int chidx, int buflen);
static void bcm2835_usb_connected(FAR struct bcm2835_usbhost_s *priv);
static void bcm2835_usb_disconnected(FAR struct bcm2835_usbhost_s *priv);

/* Second level interrupt handlers */

#ifdef CONFIG_BCM2835_OTG_SOFINTR
static inline void bcm2835_gint_sofisr(FAR struct bcm2835_usbhost_s *priv);
#endif

/* First level, global interrupt handler */

static int bcm2835_transfer_complete(FAR struct bcm2835_usbhost_s *priv, int chidx);
static inline void bcm2835_channel_int_isr(FAR struct bcm2835_usbhost_s *priv, int chidx);
static int bcm2835_gint_isr(int irq, FAR void *context, FAR void *arg);

/* Interrupt controls */

void bcm2835_channel_int_enable(FAR struct bcm2835_usbhost_s *priv, int chidx);
void bcm2835_channel_int_disable(FAR struct bcm2835_usbhost_s *priv, int chidx);
static void bcm2835_gint_enable(void);
static void bcm2835_gint_disable(void);

/* USB host controller operations **********************************************/

static int bcm2835_wait(FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s **hport);
static int bcm2835_rh_enumerate(FAR struct bcm2835_usbhost_s *priv, FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s *hport);
static int bcm2835_enumerate(FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s *hport);

static int bcm2835_ep0configure(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed, uint16_t maxpacketsize);
static int bcm2835_epalloc(FAR struct usbhost_driver_s *drvr, FAR const FAR struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep);
static int bcm2835_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int bcm2835_otg_alloc(FAR struct usbhost_driver_s *drvr, FAR uint8_t **buffer, FAR size_t *maxlen);
static int bcm2835_otg_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int bcm2835_otg_ioalloc(FAR struct usbhost_driver_s *drvr, FAR uint8_t **buffer, size_t buflen);
static int bcm2835_otg_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int bcm2835_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req, FAR uint8_t *buffer);
static int bcm2835_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req, FAR const uint8_t *buffer);
static ssize_t bcm2835_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg);
#endif
static int bcm2835_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int bcm2835_connect(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_hubport_s *hport, bool connected);
#endif
static void bcm2835_disconnect(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_hubport_s *hport);

/* Initialization **************************************************************/

static bool bcm2835_wait_for_bit_check(FAR struct bcm2835_usbhost_s *priv, uint32_t p_register, u32 mask, bool wait_set, unsigned ms_tout);
static uint8_t bcm2835_get_port_speed(FAR struct bcm2835_usbhost_s *priv);
static bool bcm2835_detect_over_current(FAR struct bcm2835_usbhost_s *priv);
static bool bcm2835_otg_reset(FAR struct bcm2835_usbhost_s *priv);
static bool bcm2835_enable_root_port(FAR struct bcm2835_usbhost_s *priv);
static void bcm2835_disable_root_port(FAR struct bcm2835_usbhost_s *priv);
static bool bcm2835_rootport_initialize(FAR struct bcm2835_usbhost_s *priv);

static void bcm2835_portreset(FAR struct bcm2835_usbhost_s *priv);
static void bcm2835_flush_txfifos(FAR struct bcm2835_usbhost_s *priv, uint32_t txfnum);
static void bcm2835_flush_rxfifo(FAR struct bcm2835_usbhost_s *priv);
static bool bcm2835_host_initialize(FAR struct bcm2835_usbhost_s *priv);

static inline void bcm2835_sw_initialize(FAR struct bcm2835_usbhost_s *priv);
static inline int bcm2835_hw_initialize(FAR struct bcm2835_usbhost_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct bcm2835_usbhost_s g_usbhost;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn = {
	.wait = bcm2835_wait,
	.enumerate = bcm2835_enumerate,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2835_printreg
 *
 * Description:
 *   Print the contents of an BCM2835 register operation
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2835_USBHOST_REGDEBUG
static void bcm2835_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
	lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: bcm2835_checkreg
 *
 * Description:
 *   Get the contents of an BCM2835 register
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2835_USBHOST_REGDEBUG
static void bcm2835_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
	static uint32_t prevaddr = 0;
	static uint32_t preval = 0;
	static uint32_t count = 0;
	static bool prevwrite = false;

	/* Is this the same value that we read from/wrote to the same register last time?
	 * Are we polling the register?  If so, suppress the output.
	 */

	if (addr == prevaddr && val == preval && prevwrite == iswrite) {
		/* Yes.. Just increment the count */

		count++;
	} else {
		/* No this is a new address or value or operation. Were there any
		 * duplicate accesses before this one?
		 */

		if (count > 0) {
			/* Yes.. Just one? */

			if (count == 1) {
				/* Yes.. Just one */

				bcm2835_printreg(prevaddr, preval, prevwrite);
			} else {
				/* No.. More than one. */

				lldbg("[repeats %d more times]\n", count);
			}
		}

		/* Save the new address, value, count, and operation for next time */

		prevaddr = addr;
		preval = val;
		count = 0;
		prevwrite = iswrite;

		/* Show the new regisgter access */

		bcm2835_printreg(addr, val, iswrite);
	}
}
#endif

/****************************************************************************
 * Name: bcm2835_getreg
 *
 * Description:
 *   Get the contents of an BCM2835 register
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2835_USBHOST_REGDEBUG
static uint32_t bcm2835_getreg(uint32_t addr)
{
	/* Read the value from the register */

	uint32_t val = getreg32(addr);

	/* Check if we need to print this value */

	bcm2835_checkreg(addr, val, false);
	return val;
}
#endif

/****************************************************************************
 * Name: bcm2835_putreg
 *
 * Description:
 *   Set the contents of an BCM2835 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2835_USBHOST_REGDEBUG
static void bcm2835_putreg(uint32_t addr, uint32_t val)
{
	/* Check if we need to print this value */

	bcm2835_checkreg(addr, val, true);

	/* Write the value */

	putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: bcm2835_modifyreg
 *
 * Description:
 *   Modify selected bits of an BCM2835 register.
 *
 ****************************************************************************/

static inline void bcm2835_modifyreg(uint32_t addr, uint32_t clrbits, uint32_t setbits)
{
	bcm2835_putreg(addr, (((bcm2835_getreg(addr)) & ~clrbits) | setbits));
}

/****************************************************************************
 * Name: bcm2835_wait_for_bit_check
 *
 * Description:
 *   waiting for specil but setting or clearing
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *   p_register -- register pointer
 *   mask -- mask value pointer
 *   wait_set -- the request wait check bit
 *   ms_tout -- the waittime as mili-second
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_wait_for_bit_check(FAR struct bcm2835_usbhost_s *priv, 
			uint32_t p_register, u32 mask, 
			bool wait_set, 
			unsigned ms_tout)
{
	assert (p_register != 0);
	assert (mask != 0);
	assert (ms_tout > 0);

	while ((getreg32(p_register) & mask) ? !wait_set : wait_set)
	{
		up_mdelay(10);

		if (--ms_tout == 0)
		{
			lldbg("Timeout\n");
			return FALSE;
		}
	}
	
	return TRUE;
}

/****************************************************************************
 * Name: bcm2835_get_port_speed
 *
 * Description:
 *   Get port speed information
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   port speed
 *	USB_SPEED_UNKNOWN	: Transfer rate not yet set
 *	USB_SPEED_LOW		: USB 1.1
 *	USB_SPEED_FULL		: USB 1.1
 *	USB_SPEED_HIGH		: USB 2.0
 *	USB_SPEED_VARIABLE	: Wireless USB 2.5
 *
 ****************************************************************************/

static uint8_t bcm2835_get_port_speed(FAR struct bcm2835_usbhost_s *priv)
{
	uint8_t speed = USB_SPEED_UNKNOWN;

	switch (DWHCI_HOST_PORT_SPEED(getreg32(DWHCI_HOST_PORT)))
	{
	case DWHCI_HOST_PORT_SPEED_HIGH:
		speed = USB_SPEED_HIGH;
		break;

	case DWHCI_HOST_PORT_SPEED_FULL:
		speed = USB_SPEED_FULL;
		break;

	case DWHCI_HOST_PORT_SPEED_LOW:
		speed = USB_SPEED_LOW;
		break;

	default:
		break;
	}

	return speed;
}

/****************************************************************************
 * Name: bcm2835_detect_over_current
 *
 * Description:
 *   Reset the OTG device
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on over current, false on no over current 
 *
 ****************************************************************************/
static bool bcm2835_detect_over_current(FAR struct bcm2835_usbhost_s *priv)
{
	if (getreg32(DWHCI_HOST_PORT) & DWHCI_HOST_PORT_OVERCURRENT)
	{
		return TRUE;
	}

	return FALSE;
}

/****************************************************************************
 * Name: set_power_state_on
 * Description:
 *   setUSB power state to on state
 *
 * Input Parameters:
 *   device_id -- device ID
 *
 * Returned Value:
 *   error : -1, success : OK
 *
 ****************************************************************************/

int set_power_state_on(unsigned device_id)
{
	bcm_property_tags_s tags;
	propertytag_power_state  powerstate;

	select_property_tags(&tags);
        powerstate.device_id = device_id;
        powerstate.state = POWER_STATE_ON | POWER_STATE_WAIT;
        if (!bcm_property_get_tags(&tags, PROPTAG_SET_POWER_STATE, &powerstate, sizeof powerstate, 8)
            ||  (powerstate.state & POWER_STATE_NO_DEVICE)
            || !(powerstate.state & POWER_STATE_ON))
        {
		unselect_property_tags(&tags);
		return -1;
        }

	unselect_property_tags(&tags);

	return OK;
}

/****************************************************************************
 * Name: bcm2835_otg_reset
 *
 * Description:
 *   Reset the OTG device
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_otg_reset(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t reg = 0;

	// wait for AHB master IDLE state
	if (!bcm2835_wait_for_bit_check(priv, DWHCI_CORE_RESET, 
			DWHCI_CORE_RESET_AHB_IDLE, 
			TRUE, 100))
	{
		return FALSE;
	}
	
	// core soft reset
	reg |= DWHCI_CORE_RESET_SOFT_RESET;
	putreg32(reg,DWHCI_CORE_RESET);

	if (!bcm2835_wait_for_bit_check(priv, DWHCI_CORE_RESET, 
			DWHCI_CORE_RESET_SOFT_RESET, 
			FALSE, 10))
	{
		return FALSE;
	}
	
	up_mdelay (1000);

	return TRUE;
}

/****************************************************************************
 * Name: bcm2835_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void bcm2835_takesem(FAR sem_t *sem)
{
	int ret;

	do {
		/* Take the semaphore (perhaps waiting) */

		ret = sem_wait(sem);

		/* The only case that an error should occur here is if the wait was
		 * awakened by a signal.
		 */

		DEBUGASSERT(ret == OK || ret == -EINTR);
	} while (ret == -EINTR);
}

/****************************************************************************
 * Name: bcm2835_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t bcm2835_getle16(FAR const uint8_t *val)
{
	return (uint16_t) val[1] << 8 | (uint16_t) val[0];
}

/****************************************************************************
 * Name: bcm2835_chan_alloc
 *
 * Description:
 *   Allocate a channel.
 *
 ****************************************************************************/

static int bcm2835_chan_alloc(FAR struct bcm2835_usbhost_s *priv)
{
	int chidx;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Search the table of channels */

	for (chidx = 0; chidx < BCM2835_NHOST_CHANNELS; chidx++) {
		/* Is this channel available? */

		if (!priv->chan[chidx].inuse) {
			/* Yes... make it "in use" and return the index */

			priv->chan[chidx].inuse = true;
	hbahn_dbg("*HBAHN[%d] : chidx(%d)\n",__LINE__, chidx);
			return chidx;
		}
	}

	/* All of the channels are "in-use" */

	return -EBUSY;
}

/****************************************************************************
 * Name: bcm2835_chan_free
 *
 * Description:
 *   Free a previoiusly allocated channel.
 *
 ****************************************************************************/

static void bcm2835_chan_free(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	DEBUGASSERT((unsigned)chidx < BCM2835_NHOST_CHANNELS);

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Halt the channel */

	bcm2835_chan_halt(priv, chidx, CHREASON_FREED);

	/* Mark the channel available */

	priv->chan[chidx].inuse = false;
}

/****************************************************************************
 * Name: bcm2835_chan_freeall
 *
 * Description:
 *   Free all channels.
 *
 ****************************************************************************/

static inline void bcm2835_chan_freeall(FAR struct bcm2835_usbhost_s *priv)
{
	uint8_t chidx;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Free all host channels */

	for (chidx = 2; chidx < BCM2835_NHOST_CHANNELS; chidx++) {
		bcm2835_chan_free(priv, chidx);
	}
}

/****************************************************************************
 * Name: bcm2835_chan_configure
 *
 * Description:
 *   Configure or re-configure a host channel.  Host channels are configured
 *   when endpoint is allocated and EP0 (only) is re-configured with the
 *   max packet size or device address changes.
 *
 ****************************************************************************/

static void bcm2835_chan_configure(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	uint32_t intr;

	hbahn_dbg("*HBAHN[%d] : chidx(%d)\n",__LINE__, chidx);

	/* reset all pending channel interrupts */
	intr = (uint32_t) -1;
	putreg32(intr, DWHCI_HOST_CHAN_INT(chidx));

	/* Nothing else to do here,
	 * channel configuration done on bcm2835_start_channel()
	 */

	 up_mdelay (50);
}

/****************************************************************************
 * Name: bcm2835_chan_halt
 *
 * Description:
 *   Halt the channel associated with 'chidx' by setting the CHannel DISable
 *   (CHDIS) bit in in the HCCHAR register.
 *
 ****************************************************************************/

static void bcm2835_chan_halt(FAR struct bcm2835_usbhost_s *priv, int chidx, enum bcm2835_chreason_e chreason)
{
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
#if 0 //HBAHN
	uint32_t hcchar;
	uint32_t intmsk;
	uint32_t eptype;
	unsigned int avail;

	lldbg("*HBAHN[%d] : \n",__LINE__);
	/* Save the reason for the halt.  We need this in the channel halt interrupt
	 * handling logic to know what to do next.
	 */

	usbhost_vtrace2(OTG_VTRACE2_CHANHALT, chidx, chreason);

	priv->chan[chidx].chreason = (uint8_t) chreason;

	/* "The application can disable any channel by programming the OTG_HCCHARx
	 *  register with the CHDIS and CHENA bits set to 1. This enables the OTG
	 *  host to flush the posted requests (if any) and generates a channel halted
	 *  interrupt. The application must wait for the CHH interrupt in OTG_HCINTx
	 *  before reallocating the channel for other transactions.  The OTG host
	 *  does not interrupt the transaction that has already been started on the
	 *  USB."
	 */

	hcchar = bcm2835_getreg(BCM2835_OTG_HCCHAR(chidx));
	hcchar |= (OTG_HCCHAR_CHDIS | OTG_HCCHAR_CHENA);

	/* Get the endpoint type from the HCCHAR register */

	eptype = hcchar & OTG_HCCHAR_EPTYP_MASK;

	/* Check for space in the Tx FIFO to issue the halt.
	 *
	 * "Before disabling a channel, the application must ensure that there is at
	 *  least one free space available in the non-periodic request queue (when
	 *  disabling a non-periodic channel) or the periodic request queue (when
	 *  disabling a periodic channel). The application can simply flush the
	 *  posted requests when the Request queue is full (before disabling the
	 *  channel), by programming the OTG__HCCHARx register with the CHDIS bit
	 *  set to 1, and the CHENA bit cleared to 0.
	 */

	if (eptype == OTG_HCCHAR_EPTYP_CTRL || eptype == OTG_HCCHAR_EPTYP_BULK) {
		/* Get the number of words available in the non-periodic Tx FIFO. */

		avail = bcm2835_getreg(BCM2835_OTG_HNPTXSTS) & OTG_HNPTXSTS_NPTXAV_MASK;
	} else {					/* if (eptype == OTG_HCCHAR_EPTYP_ISOC || eptype == OTG_HCCHAR_EPTYP_INTR) */

		/* Get the number of words available in the non-periodic Tx FIFO. */

		avail = bcm2835_getreg(BCM2835_OTG_HPTXSTS) & OTG_HPTXSTS_PTXAVL_MASK;
	}

	/* Check if there is any space available in the Tx FIFO. */

	if (avail == 0) {
		/* The Tx FIFO is full... disable the channel to flush the requests */

		hcchar &= ~OTG_HCCHAR_CHENA;
	}

	/* Unmask the CHannel Halted (CHH) interrupt */

	intmsk = bcm2835_getreg(BCM2835_OTG_HCINTMSK(chidx));
	intmsk |= OTG_HCINT_CHH;
	bcm2835_putreg(BCM2835_OTG_HCINTMSK(chidx), intmsk);

	/* Halt the channel by setting CHDIS (and maybe CHENA) in the HCCHAR */

	bcm2835_putreg(BCM2835_OTG_HCCHAR(chidx), hcchar);
#endif //HBAHN
}

/****************************************************************************
 * Name: bcm2835_chan_waitsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Called from a normal thread context BEFORE the transfer has been started.
 *
 ****************************************************************************/

static int bcm2835_chan_waitsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	irqstate_t flags = irqsave();
	int ret = -ENODEV;

	/* Is the device still connected? */

	if (priv->connected) {
		/* Yes.. then set waiter to indicate that we expect to be informed when
		 * either (1) the device is disconnected, or (2) the transfer completed.
		 */

		chan->waiter = true;
#ifdef CONFIG_USBHOST_ASYNCH
		chan->callback = NULL;
		chan->arg = NULL;
#endif
		ret = OK;
	}

	irqrestore(flags);
	return ret;
}

/****************************************************************************
 * Name: bcm2835_chan_asynchsetup
 *
 * Description:
 *   Set the request for the transfer complete event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumptions:
 *   Might be called from the level of an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_chan_asynchsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan, usbhost_asynch_t callback, FAR void *arg)
{
	irqstate_t flags = irqsave();
	int ret = -ENODEV;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Is the device still connected? */

	if (priv->connected) {
		/* Yes.. then set waiter to indicate that we expect to be informed when
		 * either (1) the device is disconnected, or (2) the transfer completed.
		 */

		chan->waiter = false;
		chan->callback = callback;
		chan->arg = arg;
		ret = OK;
	}

	irqrestore(flags);
	return ret;
}
#endif

/****************************************************************************
 * Name: bcm2835_chan_wait
 *
 * Description:
 *   Wait for a transfer on a channel to complete.
 *
 * Assumptions:
 *   Called from a normal thread context
 *
 ****************************************************************************/

static int bcm2835_chan_wait(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	irqstate_t flags;
	int ret;

	/* Disable interrupts so that the following operations will be atomic.  On
	 * the OTG global interrupt needs to be disabled.  However, here we disable
	 * all interrupts to exploit that fact that interrupts will be re-enabled
	 * while we wait.
	 */

	flags = irqsave();

	/* Loop, testing for an end of transfer condition.  The channel 'result'
	 * was set to EBUSY and 'waiter' was set to true before the transfer; 'waiter'
	 * will be set to false and 'result' will be set appropriately when the
	 * transfer is completed.
	 */

	do {
		/* Wait for the transfer to complete.  NOTE the transfer may already
		 * completed before we get here or the transfer may complete while we
		 * wait here.
		 */

		ret = sem_wait(&chan->waitsem);

		/* sem_wait should succeed.  But it is possible that we could be
		 * awakened by a signal too.
		 */

		DEBUGASSERT(ret == OK || ret == -EINTR);
	} while (chan->waiter);

	/* The transfer is complete re-enable interrupts and return the result */

	ret = -(int)chan->result;
	irqrestore(flags);
	return ret;
}

/****************************************************************************
 * Name: bcm2835_chan_wakeup
 *
 * Description:
 *   A channel transfer has completed... wakeup any threads waiting for the
 *   transfer to complete.
 *
 * Assumptions:
 *   This function is called from the transfer complete interrupt handler for
 *   the channel.  Interrupts are disabled.
 *
 ****************************************************************************/

static void bcm2835_chan_wakeup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	/* Is the transfer complete? */

	if (chan->result != EBUSY) {
		/* Is there a thread waiting for this transfer to complete? */

		if (chan->waiter) {
#ifdef CONFIG_USBHOST_ASYNCH
			/* Yes.. there should not also be a callback scheduled */

			DEBUGASSERT(chan->callback == NULL);
#endif
			/* Wake'em up! */

			usbhost_vtrace2(chan->in ? OTG_VTRACE2_CHANWAKEUP_IN : OTG_VTRACE2_CHANWAKEUP_OUT, chan->epno, chan->result);

			bcm2835_givesem(&chan->waitsem);
			chan->waiter = false;
		}
#ifdef CONFIG_USBHOST_ASYNCH
		/* No.. is an asynchronous callback expected when the transfer
		 * completes?
		 */

		else if (chan->callback) {
			/* Handle continuation of IN/OUT pipes */

			if (chan->in) {
				bcm2835_in_next(priv, chan);
			} else {
				bcm2835_out_next(priv, chan);
			}
		}
#endif
	}
}

/****************************************************************************
 * Name: bcm2835_ctrlchan_alloc
 *
 * Description:
 *   Allocate and configured channels for a control pipe.
 *
 ****************************************************************************/

static int bcm2835_ctrlchan_alloc(FAR struct bcm2835_usbhost_s *priv, uint8_t epno, uint8_t funcaddr, uint8_t speed, FAR struct bcm2835_ctrlinfo_s *ctrlep)
{
	FAR struct bcm2835_chan_s *chan;
	int inndx;
	int outndx;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	outndx = bcm2835_chan_alloc(priv);
	if (outndx < 0) {
		return -ENOMEM;
	}

	ctrlep->outndx = outndx;
	chan = &priv->chan[outndx];
	chan->epno = epno;
	chan->in = false;
	chan->eptype = OTG_EPTYPE_CTRL;
	chan->funcaddr = funcaddr;
	chan->speed = speed;
	chan->interval = 0;
	chan->maxpacket = BCM2835_EP0_DEF_PACKET_SIZE;
	chan->indata1 = false;
	chan->outdata1 = false;

	/* Configure control OUT channels */
	bcm2835_chan_configure(priv, outndx);

	/* Allocate and initialize the control IN channel */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	inndx = bcm2835_chan_alloc(priv);
	if (inndx < 0) {
		bcm2835_chan_free(priv, outndx);
		return -ENOMEM;
	}

	ctrlep->inndx = inndx;
	chan = &priv->chan[inndx];
	chan->epno = epno;
	chan->in = true;
	chan->eptype = OTG_EPTYPE_CTRL;
	chan->funcaddr = funcaddr;
	chan->speed = speed;
	chan->interval = 0;
	chan->maxpacket = BCM2835_EP0_DEF_PACKET_SIZE;
	chan->indata1 = false;
	chan->outdata1 = false;

	/* Configure control IN channels */
	bcm2835_chan_configure(priv, inndx);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_ctrlep_alloc
 *
 * Description:
 *   Allocate a container and channels for control pipe.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int bcm2835_ctrlep_alloc(FAR struct bcm2835_usbhost_s *priv, FAR const struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep)
{
	FAR struct usbhost_hubport_s *hport;
	FAR struct bcm2835_ctrlinfo_s *ctrlep;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Sanity check.  NOTE that this method should only be called if a device is
	 * connected (because we need a valid low speed indication).
	 */

	DEBUGASSERT(epdesc->hport != NULL);
	hport = epdesc->hport;

	/* Allocate a container for the control endpoint */

	ctrlep = (FAR struct bcm2835_ctrlinfo_s *)kmm_malloc(sizeof(struct bcm2835_ctrlinfo_s));
	if (ctrlep == NULL) {
		lldbg("ERROR: Failed to allocate control endpoint container\n");
		return -ENOMEM;
	}

	/* Then allocate and configure the IN/OUT channnels  */

	ret = bcm2835_ctrlchan_alloc(priv, epdesc->addr & USB_EPNO_MASK, hport->funcaddr, hport->speed, ctrlep);
	if (ret < 0) {
		lldbg("ERROR: bcm2835_ctrlchan_alloc failed: %d\n", ret);
		kmm_free(ctrlep);
		return ret;
	}

	/* Return a pointer to the control pipe container as the pipe "handle" */

	*ep = (usbhost_ep_t) ctrlep;
	return OK;
}

/************************************************************************************
 * Name: bcm2835_xfrep_alloc
 *
 * Description:
 *   Allocate and configure one unidirectional endpoint.
 *
 * Input Parameters:
 *   priv - The private USB host driver state.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int bcm2835_xfrep_alloc(FAR struct bcm2835_usbhost_s *priv, FAR const struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep)
{
	struct usbhost_hubport_s *hport;
	FAR struct bcm2835_chan_s *chan;
	int chidx;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Sanity check.  NOTE that this method should only be called if a device is
	 * connected (because we need a valid low speed indication).
	 */

	DEBUGASSERT(epdesc->hport != NULL);
	hport = epdesc->hport;

	/* Allocate a host channel for the endpoint */
	chidx = bcm2835_chan_alloc(priv);
	if (chidx < 0) {
		lldbg("ERROR: Failed to allocate a host channel\n");
		return -ENOMEM;
	}

	/* Decode the endpoint descriptor to initialize the channel data structures.
	 * Note:  Here we depend on the fact that the endpoint point type is
	 * encoded in the same way in the endpoint descriptor as it is in the OTG
	 * HS hardware.
	 */

	chan = &priv->chan[chidx];
	chan->epno = epdesc->addr & USB_EPNO_MASK;
	chan->in = epdesc->in;
	chan->eptype = epdesc->xfrtype;
	chan->funcaddr = hport->funcaddr;
	chan->speed = hport->speed;
	chan->interval = epdesc->interval;
	chan->maxpacket = epdesc->mxpacketsize;
	chan->indata1 = false;
	chan->outdata1 = false;

	/* Then configure the endpoint */
	bcm2835_chan_configure(priv, chidx);

	/* Return the index to the allocated channel as the endpoint "handle" */

	*ep = (usbhost_ep_t) chidx;
	return OK;
}

/****************************************************************************
 * Name: bcm2835_wait_for_frame
 *
 * Description:
 *   Wait for frame done
 *
 ****************************************************************************/
static void bcm2835_wait_for_frame(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	uint32_t periodic;

 	periodic = (chan->eptype == OTG_EPTYPE_INTR) || (chan->eptype == OTG_EPTYPE_ISOC);

        if (!periodic)
        {
		priv->nextframe = (DWHCI_HOST_FRM_NUM_NUMBER(getreg32(DWHCI_HOST_FRM_NUM))+1) & DWHCI_MAX_FRAME_NUMBER;
                while ((DWHCI_HOST_FRM_NUM_NUMBER(getreg32(DWHCI_HOST_FRM_NUM)) & DWHCI_MAX_FRAME_NUMBER) != priv->nextframe)
                {
                        // do nothing
                }
        }
	else 
	{
		if (priv->nextframe == BCM2835_FRAME_UNSET)
		{
			priv->nextframe = (DWHCI_HOST_FRM_NUM_NUMBER(getreg32(DWHCI_HOST_FRM_NUM)) + 1) & 7;
			if (priv->nextframe == 6)
			{
				priv->nextframe++;
			}
		}
	
		while ((DWHCI_HOST_FRM_NUM_NUMBER(getreg32(DWHCI_HOST_FRM_NUM)) & 7) != priv->nextframe)
		{
			// do nothing
		}
	}
}

/****************************************************************************
 * Name: bcm2835_start_channel
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 ****************************************************************************/
static void bcm2835_start_channel(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;
	uint32_t mask, intr, split, periodic; 
	uint32_t character, splitcontrol, dma_address, xfersize;
	bool oddframe;

	chan = &priv->chan[chidx];
	hbahn_dbg1("*HBAHN[%d] : chan->in(%d), buffer(%x), buflen(%d), packets(%d), pid(%x)\n",__LINE__, chan->in, chan->buffer, chan->buflen, chan->npackets, chan->pid);

	/* copy data for out transfer */
	if (!chan->in && chan->buflen > 0) {
		memcpy(priv->buffer, chan->buffer, chan->buflen);
#if 0
	hbahn_dbg("*HBAHN[%d] : Out Transfer =>\n",__LINE__);
		bcm2835_pktdump("+++ Sending", chan->buffer, chan->buflen);
#endif
	}
	/* set stage substate to wait for transaction complete, 
	 * this checked on channel interrupt service routine
	 */
	chan->substate = STAGE_SUBSTATE_WAIT_FOR_TRANSACTION_COMPLETE;

	// reset all pending channel interrupts
	intr = (uint32_t) -1;
	putreg32(intr, DWHCI_HOST_CHAN_INT(chidx));

	/* set transfer size, packet count and pid */
	xfersize = 0;
	xfersize |= (uint32_t)chan->buflen & DWHCI_HOST_CHAN_XFER_SIZ_BYTES__MASK;
	xfersize |= ((uint32_t)chan->npackets << DWHCI_HOST_CHAN_XFER_SIZ_PACKETS__SHIFT)
			& DWHCI_HOST_CHAN_XFER_SIZ_PACKETS__MASK ;
	xfersize |= (uint32_t)chan->pid << DWHCI_HOST_CHAN_XFER_SIZ_PID__SHIFT;
	putreg32(xfersize, DWHCI_HOST_CHAN_XFER_SIZ (chidx));
	hbahn_dbg("*HBAHN[%d] : xfersize(%x)\n",__LINE__,xfersize);

	/* set DMA address */
	dma_address = BUS_ADDRESS((uint32_t)priv->buffer);
	putreg32(dma_address, DWHCI_HOST_CHAN_DMA_ADDR(chidx));

	/* flush transfer data buffer */
	CleanAndInvalidateDataCacheRange(priv->buffer, chan->buflen);
	DataMemBarrier();

	/* is this split transfer or periodic transfer ? */
	split = (priv->rhport.hport.port !=0) && (chan->speed != USB_SPEED_HIGH);
	periodic = (chan->eptype == OTG_EPTYPE_INTR) || (chan->eptype == OTG_EPTYPE_ISOC);
	hbahn_dbg("*HBAHN[%d] : split(%d), periodic(%d)\n",__LINE__,split, periodic);

	/* set split control */
	splitcontrol = 0;

	/* TODO: need to fix it, I do not use split control now */
	if (split) {
	hbahn_dbg("*HBAHN[%d] : split(%d)\n",__LINE__,split);
		splitcontrol |= priv->rhport.hport.port;
		splitcontrol |= priv->hport->port << DWHCI_HOST_CHAN_SPLIT_CTRL_HUB_ADDRESS__SHIFT;
#if 0 	
		splitcontrol |= DWHCITransferStageDataGetSplitPosition (pStageData)
					<< DWHCI_HOST_CHAN_SPLIT_CTRL_XACT_POS__SHIFT;
		if (DWHCITransferStageDataIsSplitComplete (pStageData))
			splitcontrol |= DWHCI_HOST_CHAN_SPLIT_CTRL_COMPLETE_SPLIT;
#endif
		splitcontrol |= DWHCI_HOST_CHAN_SPLIT_CTRL_SPLIT_ENABLE;
	}

	putreg32(splitcontrol, DWHCI_HOST_CHAN_SPLIT_CTRL(chidx));

	/* Setup the channel parameters : Frame oddness and host channel enable */
	character = getreg32(DWHCI_HOST_CHAN_CHARACTER(chidx));
	character &= ~DWHCI_HOST_CHAN_CHARACTER_MAX_PKT_SIZ__MASK;
	character |= chan->maxpacket & DWHCI_HOST_CHAN_CHARACTER_MAX_PKT_SIZ__MASK;

	character &= ~DWHCI_HOST_CHAN_CHARACTER_MULTI_CNT__MASK;
	character |= 1 << DWHCI_HOST_CHAN_CHARACTER_MULTI_CNT__SHIFT; // TODO: optimize

	if (chan->in)
		character |= DWHCI_HOST_CHAN_CHARACTER_EP_DIRECTION_IN;
	else
		character &= ~DWHCI_HOST_CHAN_CHARACTER_EP_DIRECTION_IN;

	if (chan->speed == USB_SPEED_LOW)
		character |= DWHCI_HOST_CHAN_CHARACTER_LOW_SPEED_DEVICE;
	else
		character &= ~DWHCI_HOST_CHAN_CHARACTER_LOW_SPEED_DEVICE;

	character &= ~DWHCI_HOST_CHAN_CHARACTER_DEVICE_ADDRESS__MASK;
	character |= chan->funcaddr << DWHCI_HOST_CHAN_CHARACTER_DEVICE_ADDRESS__SHIFT;

	character &= ~DWHCI_HOST_CHAN_CHARACTER_EP_TYPE__MASK;
	character |= chan->eptype << DWHCI_HOST_CHAN_CHARACTER_EP_TYPE__SHIFT;

	character &= ~DWHCI_HOST_CHAN_CHARACTER_EP_NUMBER__MASK;
	character |= chan->epno << DWHCI_HOST_CHAN_CHARACTER_EP_NUMBER__SHIFT;

	bcm2835_wait_for_frame(priv, chan);

	/* Set/clear the Odd Frame bit.  Check for an even frame; if so set Odd
	 * Frame. This field is applicable for only periodic (isochronous and
	 * interrupt) channels.
	 */
	oddframe = priv->nextframe & 1 ? true : false;

	/* If split transfer or periodic transfer, we need to do more things */
	if (split || periodic) {
		if (oddframe)
			character |= DWHCI_HOST_CHAN_CHARACTER_PER_ODD_FRAME;
		else
			character &= ~DWHCI_HOST_CHAN_CHARACTER_PER_ODD_FRAME;
	}

	mask = DWHCI_HOST_CHAN_INT_XFER_COMPLETE | DWHCI_HOST_CHAN_INT_HALTED | DWHCI_HOST_CHAN_INT_ERROR_MASK;
	mask |=   DWHCI_HOST_CHAN_INT_ACK | DWHCI_HOST_CHAN_INT_NAK | DWHCI_HOST_CHAN_INT_NYET;
//	mask |=   DWHCI_HOST_CHAN_INT_ACK | DWHCI_HOST_CHAN_INT_NAK ;

	putreg32(mask, DWHCI_HOST_CHAN_INT_MASK(chidx));
	
	character |= DWHCI_HOST_CHAN_CHARACTER_ENABLE;
	character &= ~DWHCI_HOST_CHAN_CHARACTER_DISABLE;
	putreg32(character, DWHCI_HOST_CHAN_CHARACTER(chidx));
}

/****************************************************************************
 * Name: bcm2835_start_transaction
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 ****************************************************************************/

static void bcm2835_start_transaction(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;
	chan = &priv->chan[chidx];

	hbahn_dbg1("*HBAHN[%d] : chan->in(%d), chidx(%d)\n",__LINE__, chan->in, chidx);
	/* set stage mstate to no split transfer
	 * this checked on channel interrupt service routine
	 */
	chan->mstate = STAGE_STATE_NO_SPLIT_TRANSFER;

	/* Enable channel interrupt */
	bcm2835_channel_int_enable(priv, chidx);

	/* channel must be disabled, if not already done but controller */
        if (getreg32(DWHCI_HOST_CHAN_CHARACTER(chidx)) & DWHCI_HOST_CHAN_CHARACTER_ENABLE)
	{
	hbahn_dbg1("*HBAHN[%d] : chidx(%d), still enabled\n",__LINE__, chidx);
		/* waiting for disabled */
		bcm2835_wait_for_bit_check(priv, DWHCI_HOST_CHAN_CHARACTER(chidx), 
				DWHCI_HOST_CHAN_CHARACTER_ENABLE, 
				FALSE, 100);
	}

	/* start channel transaction */
	bcm2835_start_channel(priv, chidx);
}

/****************************************************************************
 * Name: bcm2835_transfer_start
 *
 * Description:
 *   Start at transfer on the select IN or OUT channel.
 *
 ****************************************************************************/

static void bcm2835_transfer_start(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;
	unsigned int npackets;
	unsigned int maxpacket;

	/* Set up the initial state of the transfer */

	chan = &priv->chan[chidx];
	hbahn_dbg1("*HBAHN[%d] : chan->in(%d)\n",__LINE__, chan->in);

	usbhost_vtrace2(OTG_VTRACE2_STARTTRANSFER, chidx, chan->buflen);

	chan->result = EBUSY;
	chan->inflight = 0;
	chan->xfrd = 0;
	priv->chidx = chidx;
	priv->nextframe = BCM2835_FRAME_UNSET;

	/* Compute the expected number of packets associated to the transfer.
	 * If the transfer length is zero (or less than the size of one maximum
	 * size packet), then one packet is expected.
	 */

	/* If the transfer size is greater than one packet, then calculate the
	 * number of packets that will be received/sent, including any partial
	 * final packet.
	 */

	maxpacket = chan->maxpacket;

	if (chan->buflen > maxpacket) {
		npackets = (chan->buflen + maxpacket - 1) / maxpacket;

		/* Clip if the buffer length if it exceeds the maximum number of
		 * packets that can be transferred (this should not happen).
		 */

		if (npackets > BCM2835_MAX_PKTCOUNT) {
			npackets = BCM2835_MAX_PKTCOUNT;
			chan->buflen = BCM2835_MAX_PKTCOUNT * maxpacket;
			usbhost_trace2(OTG_TRACE2_CLIP, chidx, chan->buflen);
		}
	} else {
		/* One packet will be sent/received (might be a zero length packet) */
		npackets = 1;
	}

	/* If it is an IN transfer, then adjust the size of the buffer UP to
	 * a full number of packets.  Hmmm... couldn't this cause an overrun
	 * into unallocated memory?
	 */

#if 0							/* Think about this */
	if (chan->in) {
		/* Force the buffer length to an even multiple of maxpacket */

		chan->buflen = npackets * maxpacket;
	}
#endif

	/* Save the number of packets in the transfer.  We will need this in
	 * order to set the next data toggle correctly when the transfer
	 * completes.
	 */
	chan->npackets = (uint8_t) npackets;

	/* start transaction */
	bcm2835_start_transaction(priv, chidx);
}

/****************************************************************************
 * Name: bcm2835_ctrl_sendsetup
 *
 * Description:
 *   Send an IN/OUT SETUP packet.
 *
 ****************************************************************************/

static int bcm2835_ctrl_sendsetup(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR const struct usb_ctrlreq_s *req)
{
	FAR struct bcm2835_chan_s *chan;
	systime_t start;
	systime_t elapsed;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Loop while the device reports NAK (and a timeout is not exceeded */

	chan = &priv->chan[ep0->outndx];
	start = clock_systimer();

	do {
		/* Send the  SETUP packet */

		chan->pid = OTG_PID_SETUP;
		chan->buffer = (FAR uint8_t *) req;
		chan->buflen = USB_SIZEOF_CTRLREQ;
		chan->xfrd = 0;

		/* Set up for the wait BEFORE starting the transfer */

		ret = bcm2835_chan_waitsetup(priv, chan);
		if (ret < 0) {
			usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
			return ret;
		}

		/* Start the transfer */

	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
		bcm2835_transfer_start(priv, ep0->outndx);

		/* Wait for the transfer to complete */

		ret = bcm2835_chan_wait(priv, chan);

		/* Return on success and for all failures other than EAGAIN.  EAGAIN
		 * means that the device NAKed the SETUP command and that we should
		 * try a few more times.
		 */

		if (ret != -EAGAIN) {
			/* Output some debug information if the transfer failed */

			if (ret < 0) {
				usbhost_trace1(OTG_TRACE1_TRNSFRFAILED, ret);
			}

			/* Return the result in any event */

			return ret;
		}

		/* Get the elapsed time (in frames) */

		elapsed = clock_systimer() - start;
	} while (elapsed < BCM2835_SETUP_DELAY);

	return -ETIMEDOUT;
}

/****************************************************************************
 * Name: bcm2835_ctrl_senddata
 *
 * Description:
 *   Send data in the data phase of an OUT control transfer.  Or send status
 *   in the status phase of an IN control transfer
 *
 ****************************************************************************/

static int bcm2835_ctrl_senddata(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR uint8_t *buffer, unsigned int buflen)
{
	FAR struct bcm2835_chan_s *chan = &priv->chan[ep0->outndx];
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Save buffer information */

	chan->buffer = buffer;
	chan->buflen = buflen;
	chan->xfrd = 0;

	/* Set the DATA PID */

	if (buflen == 0) {
		/* For status OUT stage with buflen == 0, set PID DATA1 */

		chan->outdata1 = true;
	}

	/* Set the Data PID as per the outdata1 bool */

	chan->pid = chan->outdata1 ? OTG_PID_DATA1 : OTG_PID_DATA0;

	/* Set up for the wait BEFORE starting the transfer */

	ret = bcm2835_chan_waitsetup(priv, chan);
	if (ret < 0) {
		usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
		return ret;
	}

	/* Start the transfer */

	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
	bcm2835_transfer_start(priv, ep0->outndx);

	/* Wait for the transfer to complete and return the result */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return bcm2835_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: bcm2835_ctrl_recvdata
 *
 * Description:
 *   Receive data in the data phase of an IN control transfer.  Or receive status
 *   in the status phase of an OUT control transfer
 *
 ****************************************************************************/

static int bcm2835_ctrl_recvdata(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_ctrlinfo_s *ep0, FAR uint8_t *buffer, unsigned int buflen)
{
	FAR struct bcm2835_chan_s *chan = &priv->chan[ep0->inndx];
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Save buffer information */

	chan->pid = OTG_PID_DATA1;
	chan->buffer = buffer;
	chan->buflen = buflen;
	chan->xfrd = 0;

	/* Set up for the wait BEFORE starting the transfer */

	ret = bcm2835_chan_waitsetup(priv, chan);
	if (ret < 0) {
		usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
		return ret;
	}

	/* Start the transfer */

	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
	bcm2835_transfer_start(priv, ep0->inndx);

	/* Wait for the transfer to complete and return the result */

	return bcm2835_chan_wait(priv, chan);
}

/****************************************************************************
 * Name: bcm2835_in_setup
 *
 * Description:
 *   Initiate an IN transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int bcm2835_in_setup(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
	/* Set up for the transfer based on the direction and the endpoint type */

	chan = &priv->chan[chidx];
	switch (chan->eptype) {
	default:
	case OTG_EPTYPE_CTRL: {	/* Control */
		/* This kind of transfer on control endpoints other than EP0 are not
		 * currently supported
		 */
	hbahn_dbg("*HBAHN[%d] : OTG_EPTYPE_CTRL\n",__LINE__);

		return -ENOSYS;
	}

	case OTG_EPTYPE_ISOC: {	/* Isochronous */
		/* Set up the IN data PID */
	hbahn_dbg("*HBAHN[%d] : OTG_EPTYPE_ISOC\n",__LINE__);

		usbhost_vtrace2(OTG_VTRACE2_ISOCIN, chidx, chan->buflen);
		chan->pid = OTG_PID_DATA0;
	}
	break;

	case OTG_EPTYPE_BULK: {	/* Bulk */
		/* Setup the IN data PID */
	hbahn_dbg("*HBAHN[%d] : OTG_EPTYPE_BULK, chan->indata1(%d)\n",__LINE__, chan->indata1);

		usbhost_vtrace2(OTG_VTRACE2_BULKIN, chidx, chan->buflen);
		chan->pid = chan->indata1 ? OTG_PID_DATA1 : OTG_PID_DATA0;
	}
	break;

	case OTG_EPTYPE_INTR: {	/* Interrupt */
		/* Setup the IN data PID */
	hbahn_dbg("*HBAHN[%d] : OTG_EPTYPE_INTR, chan->indata1(%d)\n",__LINE__, chan->indata1);

		usbhost_vtrace2(OTG_VTRACE2_INTRIN, chidx, chan->buflen);
		chan->pid = chan->indata1 ? OTG_PID_DATA1 : OTG_PID_DATA0;
	}
	break;
	}
	hbahn_dbg("*HBAHN[%d] : chan->pid(%d)\n",__LINE__, chan->pid);

	/* Start the transfer */
	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
	bcm2835_transfer_start(priv, chidx);
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_in_transfer
 *
 * Description:
 *   Transfer 'buflen' bytes into 'buffer' from an IN channel.
 *
 ****************************************************************************/

static ssize_t bcm2835_in_transfer(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen)
{
	FAR struct bcm2835_chan_s *chan;
	systime_t start;
	ssize_t xfrd;
	int ret;

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
	/* Loop until the transfer completes (i.e., buflen is decremented to zero)
	 * or a fatal error occurs any error other than a simple NAK.  NAK would
	 * simply indicate the end of the transfer (short-transfer).
	 */

	chan = &priv->chan[chidx];
	chan->buffer = buffer;
	chan->buflen = buflen;
	chan->xfrd = 0;
	xfrd = 0;

	start = clock_systimer();
	while (chan->xfrd < chan->buflen) {
		/* Set up for the wait BEFORE starting the transfer */

	hbahn_dbg1("*HBAHN[%d] : IN LOOP START\n",__LINE__);
		ret = bcm2835_chan_waitsetup(priv, chan);
		if (ret < 0) {
			usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
			return (ssize_t) ret;
		}

		/* Set up for the transfer based on the direction and the endpoint type */

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
		ret = bcm2835_in_setup(priv, chidx);
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
		if (ret < 0) {
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
			lldbg("ERROR: bcm2835_in_setup failed: %d\n", ret);
			return (ssize_t) ret;
		}

		/* Wait for the transfer to complete and get the result */

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
		ret = bcm2835_chan_wait(priv, chan);

		/* EAGAIN indicates that the device NAKed the transfer. */

	hbahn_dbg1("*HBAHN[%d] : ret(%d)\n",__LINE__, ret);
		if (ret < 0) {
			/* The transfer failed.  If we received a NAK, return all data
			 * buffered so far (if any).
			 */

			if (ret == -EAGAIN) {
				/* Was data buffered prior to the NAK? */

	hbahn_dbg1("*HBAHN[%d] : xfrd(%d)\n",__LINE__, xfrd);
				if (xfrd > 0) {
					/* Yes, return the amount of data received.
					 *
					 * REVISIT: This behavior is clearly correct for CDC/ACM
					 * bulk transfers and HID interrupt transfers.  But I am
					 * not so certain for MSC bulk transfers which, I think,
					 * could have NAKed packets in the middle of a transfer.
					 */

					return xfrd;
				} else {
					useconds_t delay;

					/* Get the elapsed time.  Has the timeout elapsed?
					 * if not then try again.
					 */

					systime_t elapsed = clock_systimer() - start;
	hbahn_dbg1("*HBAHN[%d] : elapsed(%d)\n",__LINE__,elapsed);
	hbahn_dbg1("*HBAHN[%d] : DELAY(%d)\n",__LINE__,BCM2835_DATANAK_DELAY);
					if (elapsed >= BCM2835_DATANAK_DELAY) {
						/* Timeout out... break out returning the NAK as
						 * as a failure.
						 */

						return (ssize_t) ret;
					}

					/* Wait a bit before retrying after a NAK. */

					if (chan->eptype == OTG_EPTYPE_INTR) {
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
						/* For interrupt (and isochronous) endpoints, the
						 * polling rate is determined by the bInterval field
						 * of the endpoint descriptor (in units of frames
						 * which we treat as milliseconds here).
						 */

						if (chan->interval > 0) {
							/* Convert the delay to units of microseconds */

							delay = (useconds_t) chan->interval * 1000;
						} else {
							/* Out of range! For interrupt endpoints, the valid
							 * range is 1-255 frames.  Assume one frame.
							 */

							delay = 1000;
						}
					} else {
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
						/* For Isochronous endpoints, bInterval must be 1.  Bulk
						 * endpoints do not have a polling interval.  Rather,
						 * the should wait until data is received.
						 *
						 * REVISIT:  For bulk endpoints this 1 msec delay is only
						 * intended to give the CPU a break from the bulk EP tight
						 * polling loop.  But are there performance issues?
						 */

						delay = 1000;
					}

					/* Wait for the next polling interval.  For interrupt and
					 * isochronous endpoints, this is necessaryto assure the
					 * polling interval.  It is used in other cases only to
					 * prevent the polling from consuming too much CPU bandwith.
					 *
					 * Small delays could require more resolution than is provided
					 * by the system timer.  For example, if the system timer
					 * resolution is 10MS, then sig_usleep(1000) will actually request
					 * a delay 20MS (due to both quantization and rounding).
					 *
					 * REVISIT: So which is better?  To ignore tiny delays and
					 * hog the system bandwidth?  Or to wait for an excessive
					 * amount and destroy system throughput?
					 */
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);

					if (delay > CONFIG_USEC_PER_TICK) {
						sig_usleep(delay - CONFIG_USEC_PER_TICK);
					}
	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
				}
			} else {
				/* Some unexpected, fatal error occurred. */

				usbhost_trace1(OTG_TRACE1_TRNSFRFAILED, ret);

				/* Break out and return the error */

				lldbg("ERROR: bcm2835_chan_wait failed: %d\n", ret);
				return (ssize_t) ret;
			}
		} else {
			/* Successfully received another chunk of data... add that to the
			 * runing total.  Then continue reading until we read 'buflen'
			 * bytes of data or until the devices NAKs (implying a short
			 * packet).
			 */

			xfrd += chan->xfrd;
	hbahn_dbg1("*HBAHN[%d] : xfrd(%d), chan->xfrd(%d)\n",__LINE__, xfrd, chan->xfrd);
		}
	}
	hbahn_dbg1("*HBAHN[%d] : xfrd(%d)\n",__LINE__, xfrd);

	return xfrd;
}

/****************************************************************************
 * Name: bcm2835_in_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void bcm2835_in_next(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	usbhost_asynch_t callback;
	FAR void *arg;
	ssize_t nbytes;
	int result;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Is the full transfer complete? Did the last chunk transfer complete OK? */

	result = -(int)chan->result;
	if (chan->xfrd < chan->buflen && result == OK) {
		/* Yes.. Set up for the next transfer based on the direction and the
		 * endpoint type
		 */

		ret = bcm2835_in_setup(priv, chan->chidx);
		if (ret >= 0) {
			return;
		}

		lldbg("ERROR: bcm2835_in_setup failed: %d\n", ret);
		result = ret;
	}

	/* The transfer is complete, with or without an error */

	uvdbg("Transfer complete:  %d\n", result);

	/* Extract the callback information */

	callback = chan->callback;
	arg = chan->arg;
	nbytes = chan->xfrd;

	chan->callback = NULL;
	chan->arg = NULL;
	chan->xfrd = 0;

	/* Then perform the callback */

	if (result < 0) {
		nbytes = (ssize_t) result;
	}

	callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: bcm2835_in_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_in_asynch(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg)
{
	FAR struct bcm2835_chan_s *chan;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Set up for the transfer data and callback BEFORE starting the first transfer */

	chan = &priv->chan[chidx];
	chan->buffer = buffer;
	chan->buflen = buflen;
	chan->xfrd = 0;

	ret = bcm2835_chan_asynchsetup(priv, chan, callback, arg);
	if (ret < 0) {
		lldbg("ERROR: bcm2835_chan_asynchsetup failed: %d\n", ret);
		return ret;
	}

	/* Set up for the transfer based on the direction and the endpoint type */

	ret = bcm2835_in_setup(priv, chidx);
	if (ret < 0) {
		lldbg("ERROR: bcm2835_in_setup failed: %d\n", ret);
	}

	/* And return with the transfer pending */

	return ret;
}
#endif

/****************************************************************************
 * Name: bcm2835_out_setup
 *
 * Description:
 *   Initiate an OUT transfer on an bulk, interrupt, or isochronous pipe.
 *
 ****************************************************************************/

static int bcm2835_out_setup(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
	/* Set up for the transfer based on the direction and the endpoint type */

	chan = &priv->chan[chidx];
	switch (chan->eptype) {
	default:
	case OTG_EPTYPE_CTRL: {	/* Control */
		/* This kind of transfer on control endpoints other than EP0 are not
		 * currently supported
		 */

		return -ENOSYS;
	}

	case OTG_EPTYPE_ISOC: {	/* Isochronous */
		/* Set up the OUT data PID */

		usbhost_vtrace2(OTG_VTRACE2_ISOCOUT, chidx, chan->buflen);
		chan->pid = OTG_PID_DATA0;
	}
	break;

	case OTG_EPTYPE_BULK: {	/* Bulk */
		/* Setup the OUT data PID */

		usbhost_vtrace2(OTG_VTRACE2_BULKOUT, chidx, chan->buflen);
		chan->pid = chan->outdata1 ? OTG_PID_DATA1 : OTG_PID_DATA0;
	}
	break;

	case OTG_EPTYPE_INTR: {	/* Interrupt */
		/* Setup the OUT data PID */

		usbhost_vtrace2(OTG_VTRACE2_INTROUT, chidx, chan->buflen);
		chan->pid = chan->outdata1 ? OTG_PID_DATA1 : OTG_PID_DATA0;

		/* Toggle the OUT data PID for the next transfer */

		chan->outdata1 ^= true;
	}
	break;
	}

	/* Start the transfer */

	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
	bcm2835_transfer_start(priv, chidx);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_out_transfer
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' through an OUT channel.
 *
 ****************************************************************************/

static ssize_t bcm2835_out_transfer(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen)
{
	FAR struct bcm2835_chan_s *chan;
	systime_t start;
	systime_t elapsed;
	size_t xfrlen;
	ssize_t xfrd;
	int ret;

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
	/* Loop until the transfer completes (i.e., buflen is decremented to zero)
	 * or a fatal error occurs (any error other than a simple NAK)
	 */

	chan = &priv->chan[chidx];
	start = clock_systimer();
	xfrd = 0;

	while (buflen > 0) {
		/* Transfer one packet at a time.  The hardware is capable of queueing
		 * multiple OUT packets, but I just haven't figured out how to handle
		 * the case where a single OUT packet in the group is NAKed.
		 */

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
		xfrlen = MIN(chan->maxpacket, buflen);
		chan->buffer = buffer;
		chan->buflen = xfrlen;
		chan->xfrd = 0;

		/* Set up for the wait BEFORE starting the transfer */

		ret = bcm2835_chan_waitsetup(priv, chan);
		if (ret < 0) {
			usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
			return (ssize_t) ret;
		}

		/* Set up for the transfer based on the direction and the endpoint type */

	hbahn_dbg1("*HBAHN[%d] : \n",__LINE__);
		ret = bcm2835_out_setup(priv, chidx);
		if (ret < 0) {
			lldbg("ERROR: bcm2835_out_setup failed: %d\n", ret);
			return (ssize_t) ret;
		}

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
		/* Wait for the transfer to complete and get the result */

		ret = bcm2835_chan_wait(priv, chan);

		/* Handle transfer failures */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
		if (ret < 0) {
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
			usbhost_trace1(OTG_TRACE1_TRNSFRFAILED, ret);

			/* Check for a special case:  If (1) the transfer was NAKed and (2)
			 * no Tx FIFO empty or Rx FIFO not-empty event occurred, then we
			 * should be able to just flush the Rx and Tx FIFOs and try again.
			 * We can detect this latter case because then the transfer buffer
			 * pointer and buffer size will be unaltered.
			 */

			elapsed = clock_systimer() - start;
			if (ret != -EAGAIN ||	/* Not a NAK condition OR */
				elapsed >= BCM2835_DATANAK_DELAY ||	/* Timeout has elapsed OR */
				chan->xfrd > 0) {	/* Data has been partially transferred */
				/* Break out and return the error */

				lldbg("ERROR: bcm2835_chan_wait failed: %d\n", ret);
				return (ssize_t) ret;
			}

	hbahn_dbg("*HBAHN[%d} : TODO flush tx FIFO \n",__LINE__);
#if 0 //HBAHN
			/* Is this flush really necessary? What does the hardware do with the
			 * data in the FIFO when the NAK occurs?  Does it discard it?
			 */

			bcm2835_flush_txfifos(OTG_GRSTCTL_TXFNUM_HALL);

			/* Get the device a little time to catch up.  Then retry the transfer
			 * using the same buffer pointer and length.
			 */
#endif

			sig_usleep(20 * 1000);
		} else {
			/* Successfully transferred.  Update the buffer pointer and length */
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

			buffer += xfrlen;
			buflen -= xfrlen;
			xfrd += chan->xfrd;
		}
	}
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return xfrd;
}

/****************************************************************************
 * Name: bcm2835_out_next
 *
 * Description:
 *   Initiate the next of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is always called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void bcm2835_out_next(FAR struct bcm2835_usbhost_s *priv, FAR struct bcm2835_chan_s *chan)
{
	usbhost_asynch_t callback;
	FAR void *arg;
	ssize_t nbytes;
	int result;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Is the full transfer complete? Did the last chunk transfer complete OK? */

	result = -(int)chan->result;
	if (chan->xfrd < chan->buflen && result == OK) {
		/* Yes.. Set up for the next transfer based on the direction and the
		 * endpoint type
		 */

		ret = bcm2835_out_setup(priv, chan->chidx);
		if (ret >= 0) {
			return;
		}

		lldbg("ERROR: bcm2835_out_setup failed: %d\n", ret);
		result = ret;
	}

	/* The transfer is complete, with or without an error */

	uvdbg("Transfer complete:  %d\n", result);

	/* Extract the callback information */

	callback = chan->callback;
	arg = chan->arg;
	nbytes = chan->xfrd;

	chan->callback = NULL;
	chan->arg = NULL;
	chan->xfrd = 0;

	/* Then perform the callback */

	if (result < 0) {
		nbytes = (ssize_t) result;
	}

	callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: bcm2835_out_asynch
 *
 * Description:
 *   Initiate the first of a sequence of asynchronous transfers.
 *
 * Assumptions:
 *   This function is never called from an interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_out_asynch(FAR struct bcm2835_usbhost_s *priv, int chidx, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg)
{
	FAR struct bcm2835_chan_s *chan;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Set up for the transfer data and callback BEFORE starting the first transfer */

	chan = &priv->chan[chidx];
	chan->buffer = buffer;
	chan->buflen = buflen;
	chan->xfrd = 0;

	ret = bcm2835_chan_asynchsetup(priv, chan, callback, arg);
	if (ret < 0) {
		lldbg("ERROR: bcm2835_chan_asynchsetup failed: %d\n", ret);
		return ret;
	}

	/* Set up for the transfer based on the direction and the endpoint type */

	ret = bcm2835_out_setup(priv, chidx);
	if (ret < 0) {
		lldbg("ERROR: bcm2835_out_setup failed: %d\n", ret);
	}

	/* And return with the transfer pending */

	return ret;
}
#endif

/****************************************************************************
 * Name: bcm2835_gint_wrpacket
 *
 * Description:
 *   Transfer the 'buflen' bytes in 'buffer' to the Tx FIFO associated with
 *   'chidx' (non-DMA).
 *
 ****************************************************************************/

static void bcm2835_gint_wrpacket(FAR struct bcm2835_usbhost_s *priv, FAR uint8_t *buffer, int chidx, int buflen)
{
	FAR uint32_t *src;
	uint32_t fifo;
	int buflen32;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	bcm2835_pktdump("Sending", buffer, buflen);

	/* Get the number of 32-byte words associated with this byte size */
	buflen32 = (buflen + 3) >> 2;

	/* Get the address of the Tx FIFO associated with this channel */
	fifo = DWHCI_DATA_FIFO(chidx);

	/* Transfer all of the data into the Tx FIFO */
	src = (FAR uint32_t *) buffer;
	for (; buflen32 > 0; buflen32--) {
		uint32_t data = *src++;
		bcm2835_putreg(fifo, data);
	}

	/* Increment the count of bytes "in-flight" in the Tx FIFO */
	priv->chan[chidx].inflight += buflen;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
}

/****************************************************************************
 * Name: bcm2835_usb_connected
 *
 * Description:
 *   Handle a connection event.
 *
 ****************************************************************************/

static void bcm2835_usb_connected(FAR struct bcm2835_usbhost_s *priv)
{
	/* We previously disconnected? */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	if (!priv->connected) {
		/* Yes.. then now we are connected */

		usbhost_vtrace1(OTG_VTRACE1_CONNECTED, 0);
		priv->connected = true;
		priv->change = true;
		DEBUGASSERT(priv->smstate == SMSTATE_DETACHED);

		/* Notify any waiters */

		priv->smstate = SMSTATE_ATTACHED;
		if (priv->pscwait) {
			bcm2835_givesem(&priv->pscsem);
			priv->pscwait = false;
		}
	}
}

/****************************************************************************
 * Name: bcm2835_usb_disconnected
 *
 * Description:
 *   Handle a disconnection event.
 *
 ****************************************************************************/

static void bcm2835_usb_disconnected(FAR struct bcm2835_usbhost_s *priv)
{
	/* Were previously connected? */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	if (priv->connected) {
		/* Yes.. then we no longer connected */

		usbhost_vtrace1(OTG_VTRACE1_DISCONNECTED, 0);

		/* Are we bound to a class driver? */

		if (priv->rhport.hport.devclass) {
			/* Yes.. Disconnect the class driver */

			CLASS_DISCONNECTED(priv->rhport.hport.devclass);
			priv->rhport.hport.devclass = NULL;
		}

		/* Re-Initialize Host for new Enumeration */

		priv->smstate = SMSTATE_DETACHED;
		priv->connected = false;
		priv->change = true;
		bcm2835_chan_freeall(priv);

		priv->rhport.hport.speed = USB_SPEED_FULL;
		priv->rhport.hport.funcaddr = 0;

		/* Notify any waiters that there is a change in the connection state */

		if (priv->pscwait) {
			bcm2835_givesem(&priv->pscsem);
			priv->pscwait = false;
		}
	}
}

/****************************************************************************
 * Name: bcm2835_gint_sofisr
 *
 * Description:
 *   USB OTG  start-of-frame interrupt handler
 *
 ****************************************************************************/

#ifdef CONFIG_BCM2835_OTG_SOFINTR
static inline void bcm2835_gint_sofisr(FAR struct bcm2835_usbhost_s *priv)
{
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Handle SOF interrupt */
#warning "Do what?"

	/* Clear pending SOF interrupt */

	bcm2835_putreg(BCM2835_OTG_GINTSTS, OTG_GINT_SOF);
}
#endif

/****************************************************************************
 * Name: bcm2835_transfer_complete
 *
 * Description:
 *   USB OTG Channel transfer complete
 *
 * Input Parameters:
 *   priv - Driver state structure reference
 *   chidx - The channel that requires the Tx FIFO empty interrupt
 *
 * Returned Value:
 *   Err or Ok
 *
 ****************************************************************************/
static int bcm2835_transfer_complete(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	FAR struct bcm2835_chan_s *chan;
	uint32_t xfersize, packetleft, byteleft;
	uint32_t packet_transfered, byte_transfered;

	chan = &priv->chan[chidx];

	/* flush transfer data buffer */
	CleanAndInvalidateDataCacheRange(priv->buffer, chan->buflen);
	DataMemBarrier();

	/* 
	 * something strange, If I remove debug message, the operation fault
	 * when operation fault, the both of packetleft and byteleft are not zero
	 * so, I add wait routine to check clean one of packetleft or byteleft to zero
	 * TODO: this routine must fixed
	 */
	hbahn_dbg("*HBAHN[%d] : channel(%d), chan-in(%d)\n",__LINE__,chidx, chan->in);
	do {
		xfersize = getreg32(DWHCI_HOST_CHAN_XFER_SIZ(chidx));
		packetleft = DWHCI_HOST_CHAN_XFER_SIZ_PACKETS(xfersize);
		byteleft = xfersize & DWHCI_HOST_CHAN_XFER_SIZ_BYTES__MASK;
	hbahn_dbg1("*HBAHN[%d] : xfersize(%x), pid(%d), packetleft(%d), byteleft(%d), chan->in(%d) \n",__LINE__,xfersize, DWHCI_HOST_CHAN_XFER_SIZ_PID(xfersize), packetleft,byteleft, chan->in);
	} while ((packetleft != 0 && byteleft != 0));

	if ((packetleft != 0 && byteleft != 0)) {
//		return -1;
	}

	assert (DWHCI_HOST_CHAN_XFER_SIZ_PID(xfersize) != DWHCI_HOST_CHAN_XFER_SIZ_PID_MDATA);

	packet_transfered = chan->npackets - packetleft;
	byte_transfered = chan->buflen - byteleft;

	hbahn_dbg("*HBAHN[%d] : packets transfered(%d), npackets(%d), packleft(%d)\n",__LINE__, packet_transfered, chan->npackets, packetleft);
	hbahn_dbg("*HBAHN[%d] : bytes transfered(%d), buflen(%d), byteleft(%d)\n",__LINE__, byte_transfered, chan->buflen, byteleft);

	chan->inflight = byte_transfered;
	chan->npackets -= packet_transfered;

	hbahn_dbg("*HBAHN[%d] : chan->in(%d), buffer(%x), buflen(%d)\n",__LINE__, chan->in, chan->buffer, chan->buflen);
	if (chan->in && byte_transfered > 0) {
		/* copy input data to channel buffer */
		memcpy(chan->buffer, priv->buffer, chan->inflight);
#if 0
	hbahn_dbg("*HBAHN[%d] : In Transfer =>\n",__LINE__);
		bcm2835_pktdump("--- Receiving", chan->buffer, chan->buflen);
#endif
	}

	if (chan->in && packet_transfered > 0) {
		if ((chan->eptype == OTG_EPTYPE_INTR) || (chan->eptype == OTG_EPTYPE_BULK))
		{
	hbahn_dbg("*HBAHN[%d] : Toggle the IN data %d\n",__LINE__,priv->chan[chidx].indata1);
			/* Toggle the IN data pid (Used by Bulk and INTR only) */
			priv->chan[chidx].indata1 ^= true;

#if 0
			switch (priv->chan[chidx].pid ) {
			case OTG_PID_DATA0:
				priv->chan[chidx].indata1 = true;
				break;
			case OTG_PID_DATA1:
				priv->chan[chidx].indata1 = false;
				break;
			default:
				priv->chan[chidx].indata1 = false;
				break;
			}
#endif
		}
	}

	chan->buffer += chan->inflight;
	chan->xfrd += chan->inflight;

//	priv->nextframe = (priv->nextframe + 2) & 7;

	return 0;
}

/****************************************************************************
 * Name: bcm2835_channel_int_isr
 *
 * Description:
 *   USB OTG Channel interrupt service routine
 *
 * Input Parameters:
 *   priv - Driver state structure reference
 *   chidx - The channel that requires the Tx FIFO empty interrupt
 *
 * Returned Value:
 *   None
 *
 * result of channel
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 ****************************************************************************/

static inline void bcm2835_channel_int_isr(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
	uint32_t intr;
	uint32_t periodic;
	uint32_t ret=0;
	FAR struct bcm2835_chan_s *chan;
	chan = &priv->chan[chidx];

	DataMemBarrier ();

	periodic = (chan->eptype == OTG_EPTYPE_INTR) || (chan->eptype == OTG_EPTYPE_ISOC);

	intr = getreg32(DWHCI_HOST_CHAN_INT(chidx));
	hbahn_dbg("*HBAHN[%d] : intr(%x), channel(%d), chan-in(%d)\n",__LINE__,intr, chidx, chan->in);

	switch (chan->substate) {
	case STAGE_SUBSTATE_WAIT_FOR_CHANNEL_DISABLE:
		hbahn_dbg("STAGE_SUBSTATE_WAIT_FOR_CHANNEL_DISABLE condition\n",__LINE__);
		/* start channel transaction */
		bcm2835_start_channel(priv, chidx);
		return;

	case STAGE_SUBSTATE_WAIT_FOR_TRANSACTION_COMPLETE:

		/* restart halted transaction */
		if (intr == DWHCI_HOST_CHAN_INT_HALTED)
		{
	hbahn_dbg1("*HBAHN[%d] : call bcm2835_transfer_start\n",__LINE__);
			bcm2835_transfer_start(priv, chidx);
			return;
		}

		if (intr & DWHCI_HOST_CHAN_INT_ACK)
			ret = bcm2835_transfer_complete(priv, chidx);
	
		break;

	default:
		lldbg("No condition\n",__LINE__);
		break;
	}


	switch (chan->mstate) {
	case STAGE_STATE_NO_SPLIT_TRANSFER:
		hbahn_dbg("STAGE_STATE_NO_SPLIT_TRANSFER condition\n",__LINE__);
		if (intr & DWHCI_HOST_CHAN_INT_ERROR_MASK)
		{
			hbahn_dbg("Transaction failed : (intr 0x%X)\n", intr);
		 	/* restart halted transaction */
//			bcm2835_enable_root_port(priv);

			/* channel result is stall condition */
			chan->result = EPERM;	
		}
		else if ( (intr & (DWHCI_HOST_CHAN_INT_NAK | DWHCI_HOST_CHAN_INT_NYET)) && periodic)
		{
			hbahn_dbg("Transaction failed (intr 0x%X)\n", intr);
			/* channel result is again condition */
			chan->result = EAGAIN;	
#if 0
			unsigned nInterval = USBEndpointGetInterval (USBRequestGetEndpoint (pURB));
			StartUSPiTimer (MSEC2HZ (nInterval), DWHCIDeviceTimerHandler, pStageData, pThis);
#endif
			break;
		}
		else if ( intr & DWHCI_HOST_CHAN_INT_NAK )	//HBAHN for in_transfer
		{
			uint32_t character = getreg32(DWHCI_HOST_CHAN_CHARACTER(chidx));

			hbahn_dbg("Transaction NAK (intr 0x%X)\n", intr);
			/* channel result is again condition */
			chan->result = EAGAIN;	
#if 0
			unsigned nInterval = USBEndpointGetInterval (USBRequestGetEndpoint (pURB));
			StartUSPiTimer (MSEC2HZ (nInterval), DWHCIDeviceTimerHandler, pStageData, pThis);
#endif
			/* disable transfer */
			character &= ~DWHCI_HOST_CHAN_CHARACTER_ENABLE;
			character |= DWHCI_HOST_CHAN_CHARACTER_DISABLE;
			putreg32(character, DWHCI_HOST_CHAN_CHARACTER(chidx));

		}
		else
		{
			hbahn_dbg("Transaction success(intr 0x%X)\n", intr);
#if 0
			if (!DWHCITransferStageDataIsStatusStage (pStageData))
				USBRequestSetResultLen (pURB, DWHCITransferStageDataGetResultLen (pStageData));
			USBRequestSetStatus (pURB, 1);
#endif

			/* channel result is complete */
			if (ret == 0) 
				chan->result = OK;
			else {
				uint32_t character = getreg32(DWHCI_HOST_CHAN_CHARACTER(chidx));

				chan->result = EAGAIN;

				/* disable transfer */
				character &= ~DWHCI_HOST_CHAN_CHARACTER_ENABLE;
				character |= DWHCI_HOST_CHAN_CHARACTER_DISABLE;
				putreg32(character, DWHCI_HOST_CHAN_CHARACTER(chidx));
			}
		}

#if 0
		DWHCIDeviceFreeChannel (pThis, nChannel);
		USBRequestCallCompletionRoutine (pURB);
#endif
		chan->inflight = 0;

		/* disable channel interrupt */
		bcm2835_channel_int_disable(priv, chidx);

		/* Check for a transfer complete event */
		bcm2835_chan_wakeup(priv, chan);

		hbahn_dbg("Transaction Complete\n",__LINE__);
		break;

	case STAGE_STATE_START_SPLIT:
		hbahn_dbg("STAGE_STATE_START_SPLIT condition\n",__LINE__);
		break;

	case STAGE_STATE_COMPLETE_SPLIT:
		hbahn_dbg("STAGE_STATE_COMPLETE_SPLIT condition\n",__LINE__);
		break;

	case STAGE_STATE_PERIODIC_DELAY:
		hbahn_dbg("STAGE_STATE_PERIODIC_DELAY condition\n",__LINE__);
		break;


	default:
		hbahn_dbg("No condition\n",__LINE__);
		break;
	}

	DataMemBarrier ();
}

/****************************************************************************
 * Name: bcm2835_gint_isr
 *
 * Description:
 *   USB OTG  global interrupt handler
 *
 ****************************************************************************/

static int bcm2835_gint_isr(int irq, FAR void *context, FAR void *arg)
{
	/* At present, there is only support for a single OTG  host. Hence it is
	 * pre-allocated as g_usbhost.  However, in most code, the private data
	 * structure will be referenced using the 'priv' pointer (rather than the
	 * global data) in order to simplify any future support for multiple devices.
	 */

	FAR struct bcm2835_usbhost_s *priv = &g_usbhost;
	uint32_t status, intr;

	status = getreg32(DWHCI_CORE_INT_STAT);
	hbahn_dbg("*HBAHN[%d] : status(%x)\n",__LINE__,status);

	if (status & DWHCI_CORE_INT_STAT_HC_INTR)
        {
		uint32_t chidx, mask = 1;
		intr = getreg32(DWHCI_HOST_ALLCHAN_INT);
		putreg32(intr, DWHCI_HOST_ALLCHAN_INT);

		/* Handle channel transfer */
		for (chidx = 0; chidx < priv->channels; chidx++)
		{
			if (intr & mask)
			{
	hbahn_dbg("*HBAHN[%d] : channel(%x), status(%x), intr(%x) \n",__LINE__,chidx, status, intr);
				putreg32(0, DWHCI_HOST_CHAN_INT_MASK(chidx));
				bcm2835_channel_int_isr(priv, chidx);
			}
			mask <<= 1;
		}

	}

	if (status & DWHCI_CORE_INT_STAT_SOF_INTR)
        {
		/* nothing to do now */
	}

	if (status & DWHCI_CORE_INT_STAT_PORT_INTR)
        {
		/* nothing to do now */
	}

	putreg32(status, DWHCI_CORE_INT_STAT);

	/* We won't get here */

	return OK;
}

/****************************************************************************
 * Name: bcm2835_common_int_enable and bcm2835_common_int_disable
 *
 * Description:
 *   Respectively enable or disable the Common OTG interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bcm2835_common_int_enable(void)
{
	uint32_t reg;

	// Clear any pending interrupts
	reg = getreg32(DWHCI_CORE_INT_STAT);
	reg = (uint32_t) -1;
	putreg32(reg, DWHCI_CORE_INT_STAT);
}

/****************************************************************************
 * Name: bcm2835_channel_int_enable  & disable
 *
 * Description:
 *   Respectively enable or disable the Channel interrupt.
 *
 * Input Parameters:
 *   priv - Driver state structure reference
 *   chidx - The channel that requires the Tx FIFO empty interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bcm2835_channel_int_enable(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
        uint32_t mask;

        irqstate_t flags = irqsave();

        mask = getreg32(DWHCI_HOST_ALLCHAN_INT_MASK);
        mask |= 1 << chidx;
        putreg32(mask, DWHCI_HOST_ALLCHAN_INT_MASK);

        irqrestore(flags);
}

void bcm2835_channel_int_disable(FAR struct bcm2835_usbhost_s *priv, int chidx)
{
        uint32_t mask;

        irqstate_t flags = irqsave();

        mask = getreg32(DWHCI_HOST_ALLCHAN_INT_MASK);
        mask &= ~(1 << chidx);
        putreg32(mask, DWHCI_HOST_ALLCHAN_INT_MASK);

        irqrestore(flags);
}


/****************************************************************************
 * Name: bcm2835_host_int_enable 
 *
 * Description:
 *   Respectively enable or disable the OTG Host interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bcm2835_host_int_enable(void)
{
	uint32_t mask;

	putreg32(0, DWHCI_CORE_INT_MASK);

	bcm2835_common_int_enable();

	mask = getreg32(DWHCI_CORE_INT_MASK);
	mask |= DWHCI_CORE_INT_MASK_HC_INTR
//		| DWHCI_CORE_INT_MASK_PORT_INTR
		| DWHCI_CORE_INT_MASK_DISCONNECT
		;

	putreg32(mask, DWHCI_CORE_INT_MASK);
}

/****************************************************************************
 * Name: bcm2835_gint_enable and bcm2835_gint_disable
 *
 * Description:
 *   Respectively enable or disable the global OTG interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2835_gint_enable(void)
{
	uint32_t reg;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	// Enable all interrupts
	reg = getreg32(DWHCI_CORE_AHB_CFG);
	reg |= DWHCI_CORE_AHB_CFG_GLOBALINT_MASK;
	putreg32(reg, DWHCI_CORE_AHB_CFG);
}

static void bcm2835_gint_disable(void)
{
	uint32_t reg;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	// Disable all interrupts
	reg = getreg32(DWHCI_CORE_AHB_CFG);
	reg &= ~DWHCI_CORE_AHB_CFG_GLOBALINT_MASK;
	putreg32(reg, DWHCI_CORE_AHB_CFG);
}

/****************************************************************************
 * USB Host Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2835_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
 *
 * Returned Values:
 *   Zero (OK) is returned on success when a device in connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int bcm2835_wait(FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s **hport)
{
	FAR struct bcm2835_usbhost_s *priv = &g_usbhost;
	struct usbhost_hubport_s *connport;
	irqstate_t flags;

	/* Loop until a change in connection state is detected */

	flags = irqsave();
	for (;;) {
		/* Is there a change in the connection state of the single root hub
		 * port?
		 */

		if (priv->change) {
			connport = &priv->rhport.hport;

			/* Yes. Remember the new state */

			connport->connected = priv->connected;
			priv->change = false;

			/* And return the root hub port */

			*hport = connport;
			irqrestore(flags);

			uvdbg("RHport Connected: %s\n", connport->connected ? "YES" : "NO");
			return OK;
		}
#ifdef CONFIG_USBHOST_HUB
		/* Is a device connected to an external hub? */

		if (priv->hport) {
			/* Yes.. return the external hub port */

			connport = (struct usbhost_hubport_s *)priv->hport;
			priv->hport = NULL;

			*hport = connport;
			irqrestore(flags);

			uvdbg("Hub port Connected: %s\n", connport->connected ? "YES" : "NO");
			return OK;
		}
#endif

		/* Wait for the next connection event */

		priv->pscwait = true;
		bcm2835_takesem(&priv->pscsem);
	}
}

/****************************************************************************
 * Name: bcm2835_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int bcm2835_rh_enumerate(FAR struct bcm2835_usbhost_s *priv, FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s *hport)
{
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

	/* Are we connected to a device?  The caller should have called the wait()
	 * method first to be assured that a device is connected.
	 */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	while (!priv->connected) {
		/* No, return an error */

		usbhost_trace1(OTG_TRACE1_DEVDISCONN, 0);
		return -ENODEV;
	}
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	DEBUGASSERT(priv->smstate == SMSTATE_ATTACHED);

	/* USB 2.0 spec says at least 50ms delay before port reset.  We wait
	 * 100ms.
	 */
	sig_usleep(100 * 1000);

	/* Reset the host port */
	bcm2835_portreset(priv);

	/* Get the current device speed */
	priv->rhport.hport.speed = bcm2835_get_port_speed(priv);

	/* Allocate and initialize the root hub port EP0 channels */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	ret = bcm2835_ctrlchan_alloc(priv, 0, 0, priv->rhport.hport.speed, &priv->ep0);
	if (ret < 0) {
		lldbg("ERROR: Failed to allocate a control endpoint: %d\n", ret);
	}
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return ret;
}

static int bcm2835_enumerate(FAR struct usbhost_connection_s *conn, FAR struct usbhost_hubport_s *hport)
{
	FAR struct bcm2835_usbhost_s *priv = &g_usbhost;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	DEBUGASSERT(hport);

	/* If this is a connection on the root hub, then we need to go to
	 * little more effort to get the device speed.  If it is a connection
	 * on an external hub, then we already have that information.
	 */

#ifdef CONFIG_USBHOST_HUB
	if (ROOTHUB(hport))
#endif
	{
		ret = bcm2835_rh_enumerate(priv, conn, hport);
		if (ret < 0) {
			return ret;
		}
	}

	/* Then let the common usbhost_enumerate do the real enumeration. */

	uvdbg("Enumerate the device\n");
	priv->smstate = SMSTATE_ENUM;
	ret = usbhost_enumerate(hport, &hport->devclass);

	/* The enumeration may fail either because of some HCD interfaces failure
	 * or because the device class is not supported.  In either case, we just
	 * need to perform the disconnection operation and make ready for a new
	 * enumeration.
	 */

	if (ret < 0) {
		/* Return to the disconnected state */

		lldbg("ERROR: Enumeration failed: %d\n", ret);
		bcm2835_usb_disconnected(priv);
	}
	else
		lowsyslog(LOG_INFO, "USB host enumerated\n");

	return ret;
}

/************************************************************************************
 * Name: bcm2835_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support an
 *   external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int bcm2835_ep0configure(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed, uint16_t maxpacketsize)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	FAR struct bcm2835_ctrlinfo_s *ep0info = (FAR struct bcm2835_ctrlinfo_s *)ep0;
	FAR struct bcm2835_chan_s *chan;

	hbahn_dbg("*HBAHN[%d] : drvr(%x), ep0info(%x), funcaddr(%d), maxpacketsize(%d)\n",__LINE__,drvr,ep0info,funcaddr,maxpacketsize);
	DEBUGASSERT(drvr != NULL && ep0info != NULL && funcaddr < 128 && maxpacketsize <= 64);

	/* We must have exclusive access to the USB host hardware and state structures */
	bcm2835_takesem(&priv->exclsem);

	/* Configure the EP0 OUT channel */
	chan = &priv->chan[ep0info->outndx];
	chan->funcaddr = funcaddr;
	chan->speed = speed;
	chan->maxpacket = maxpacketsize;

	up_mdelay(100);

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	bcm2835_chan_configure(priv, ep0info->outndx);

	/* Configure the EP0 IN channel */
	chan = &priv->chan[ep0info->inndx];
	chan->funcaddr = funcaddr;
	chan->speed = speed;
	chan->maxpacket = maxpacketsize;

	up_mdelay(100);

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	bcm2835_chan_configure(priv, ep0info->inndx);

	bcm2835_givesem(&priv->exclsem);
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return OK;
}

/************************************************************************************
 * Name: bcm2835_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int bcm2835_epalloc(FAR struct usbhost_driver_s *drvr, FAR const struct usbhost_epdesc_s *epdesc, FAR usbhost_ep_t *ep)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Sanity check.  NOTE that this method should only be called if a device is
	 * connected (because we need a valid low speed indication).
	 */

	DEBUGASSERT(drvr != 0 && epdesc != NULL && ep != NULL);

	/* We must have exclusive access to the USB host hardware and state structures */

	bcm2835_takesem(&priv->exclsem);

	/* Handler control pipes differently from other endpoint types.  This is
	 * because the normal, "transfer" endpoints are unidirectional an require
	 * only a single channel.  Control endpoints, however, are bi-diretional
	 * and require two channels, one for the IN and one for the OUT direction.
	 */

	if (epdesc->xfrtype == OTG_EPTYPE_CTRL) {
		ret = bcm2835_ctrlep_alloc(priv, epdesc, ep);
	} else {
		ret = bcm2835_xfrep_alloc(priv, epdesc, ep);
	}

	bcm2835_givesem(&priv->exclsem);
	return ret;
}

/************************************************************************************
 * Name: bcm2835_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int bcm2835_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	DEBUGASSERT(priv);

	/* We must have exclusive access to the USB host hardware and state structures */

	bcm2835_takesem(&priv->exclsem);

	/* A single channel is represent by an index in the range of 0 to BCM2835_MAX_TX_FIFOS.
	 * Otherwise, the ep must be a pointer to an allocated control endpoint structure.
	 */

	if ((uintptr_t) ep < BCM2835_MAX_TX_FIFOS) {
		/* Halt the channel and mark the channel available */

		bcm2835_chan_free(priv, (int)ep);
	} else {
		/* Halt both control channel and mark the channels available */

		FAR struct bcm2835_ctrlinfo_s *ctrlep = (FAR struct bcm2835_ctrlinfo_s *)ep;
		bcm2835_chan_free(priv, ctrlep->inndx);
		bcm2835_chan_free(priv, ctrlep->outndx);

		/* And free the control endpoint container */

		kmm_free(ctrlep);
	}

	bcm2835_givesem(&priv->exclsem);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_otg_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/
static int bcm2835_otg_alloc(FAR struct usbhost_driver_s *drvr, FAR uint8_t **buffer, FAR size_t *maxlen)
{
	FAR uint8_t *alloc;

	DEBUGASSERT(drvr && buffer && maxlen);

	/* There is no special memory requirement for the BCM2835. */

	alloc = (FAR uint8_t *)kmm_malloc(CONFIG_BCM2835_OTG_DESCSIZE);
	if (!alloc) {
		return -ENOMEM;
	}

	/* Return the allocated address and size of the descriptor buffer */

	*buffer = alloc;
	*maxlen = CONFIG_BCM2835_OTG_DESCSIZE;
	return OK;
}

/****************************************************************************
 * Name: bcm2835_otg_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/
static int bcm2835_otg_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
	/* There is no special memory requirement */

	DEBUGASSERT(drvr && buffer);
	kmm_free(buffer);
	return OK;
}

/************************************************************************************
 * Name: bcm2835_otg_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/
static int bcm2835_otg_ioalloc(FAR struct usbhost_driver_s *drvr, FAR uint8_t **buffer, size_t buflen)
{
	FAR uint8_t *alloc;

	DEBUGASSERT(drvr && buffer && buflen > 0);

	/* There is no special memory requirement */

	alloc = (FAR uint8_t *)kmm_malloc(buflen);
	if (!alloc) {
		return -ENOMEM;
	}

	/* Return the allocated buffer */

	*buffer = alloc;
	return OK;
}

/************************************************************************************
 * Name: bcm2835_otg_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/
static int bcm2835_otg_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
	/* There is no special memory requirement */

	DEBUGASSERT(drvr && buffer);
	kmm_free(buffer);
	return OK;
}

/****************************************************************************
 * Name: bcm2835_ctrlin and bcm2835_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int bcm2835_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req, FAR uint8_t *buffer)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	FAR struct bcm2835_ctrlinfo_s *ep0info = (FAR struct bcm2835_ctrlinfo_s *)ep0;
	uint16_t buflen;
	systime_t start;
	systime_t elapsed;
	int retries;
	int ret;

	hbahn_dbg("*HBAHN[%d] : ######################\n",__LINE__);
	DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
	usbhost_vtrace2(OTG_VTRACE2_CTRLIN, req->type, req->req);
	uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n", req->type, req->req, req->value[1], req->value[0], req->index[1], req->index[0], req->len[1], req->len[0]);
	hbahn_dbg("*HBAHN[%d] type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n", __LINE__, req->type, req->req, req->value[1], req->value[0], req->index[1], req->index[0], req->len[1], req->len[0]);

	/* Extract values from the request */

	buflen = bcm2835_getle16(req->len);

	/* We must have exclusive access to the USB host hardware and state structures */

	bcm2835_takesem(&priv->exclsem);

	/* Loop, retrying until the retry time expires */

	for (retries = 0; retries < BCM2835_RETRY_COUNT; retries++) {
		/* Send the SETUP request */

		ret = bcm2835_ctrl_sendsetup(priv, ep0info, req);
		if (ret < 0) {
			usbhost_trace1(OTG_TRACE1_SENDSETUP, -ret);
			continue;
		}

		/* Get the start time.  Loop again until the timeout expires */

		start = clock_systimer();
		do {
			/* Handle the IN data phase (if any) */

			if (buflen > 0) {
				ret = bcm2835_ctrl_recvdata(priv, ep0info, buffer, buflen);
				if (ret < 0) {
					usbhost_trace1(OTG_TRACE1_RECVDATA, -ret);
				}
			}

			/* Handle the status OUT phase */

			if (ret == OK) {
				priv->chan[ep0info->outndx].outdata1 ^= true;
				ret = bcm2835_ctrl_senddata(priv, ep0info, NULL, 0);
				if (ret == OK) {
					/* All success transactions exit here */

					bcm2835_givesem(&priv->exclsem);
					return OK;
				}

				usbhost_trace1(OTG_TRACE1_SENDDATA, ret < 0 ? -ret : ret);
			}

			/* Get the elapsed time (in frames) */

			elapsed = clock_systimer() - start;
		} while (elapsed < BCM2835_DATANAK_DELAY);
	}

	/* All failures exit here after all retries and timeouts have been exhausted */

	bcm2835_givesem(&priv->exclsem);
	return -ETIMEDOUT;
}

static int bcm2835_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0, FAR const struct usb_ctrlreq_s *req, FAR const uint8_t *buffer)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	FAR struct bcm2835_ctrlinfo_s *ep0info = (FAR struct bcm2835_ctrlinfo_s *)ep0;
	uint16_t buflen;
	systime_t start;
	systime_t elapsed;
	int retries;
	int ret;

	hbahn_dbg("*HBAHN[%d] : $$$$$$$$$$$$$$$$$$$\n",__LINE__);
	DEBUGASSERT(priv != NULL && ep0info != NULL && req != NULL);
	usbhost_vtrace2(OTG_VTRACE2_CTRLOUT, req->type, req->req);
	uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n", req->type, req->req, req->value[1], req->value[0], req->index[1], req->index[0], req->len[1], req->len[0]);
	hbahn_dbg("*HBAHN[%d] type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n", __LINE__, req->type, req->req, req->value[1], req->value[0], req->index[1], req->index[0], req->len[1], req->len[0]);

	/* Extract values from the request */

	buflen = bcm2835_getle16(req->len);

	/* We must have exclusive access to the USB host hardware and state structures */

	bcm2835_takesem(&priv->exclsem);

	/* Loop, retrying until the retry time expires */

	for (retries = 0; retries < BCM2835_RETRY_COUNT; retries++) {
		/* Send the SETUP request */

		ret = bcm2835_ctrl_sendsetup(priv, ep0info, req);
		if (ret < 0) {
			usbhost_trace1(OTG_TRACE1_SENDSETUP, -ret);
			continue;
		}

		/* Get the start time.  Loop again until the timeout expires */

		start = clock_systimer();
		do {
			/* Handle the data OUT phase (if any) */

			if (buflen > 0) {
				/* Start DATA out transfer (only one DATA packet) */

				priv->chan[ep0info->outndx].outdata1 = true;
				ret = bcm2835_ctrl_senddata(priv, ep0info, NULL, 0);
				if (ret < 0) {
					usbhost_trace1(OTG_TRACE1_SENDDATA, -ret);
				}
			}

			/* Handle the status IN phase */

			if (ret == OK) {
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
				ret = bcm2835_ctrl_recvdata(priv, ep0info, NULL, 0);
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
				if (ret == OK) {
					/* All success transactins exit here */

					bcm2835_givesem(&priv->exclsem);
					return OK;
				}

				usbhost_trace1(OTG_TRACE1_RECVDATA, ret < 0 ? -ret : ret);
			}

			/* Get the elapsed time (in frames) */

			elapsed = clock_systimer() - start;
		} while (elapsed < BCM2835_DATANAK_DELAY);
	}

	/* All failures exit here after all retries and timeouts have been exhausted */

	bcm2835_givesem(&priv->exclsem);
	return -ETIMEDOUT;
}

/****************************************************************************
 * Name: bcm2835_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes. Only
 *   one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value is
 *   returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t bcm2835_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	unsigned int chidx = (unsigned int)ep;
	ssize_t nbytes;

	hbahn_dbg1("*HBAHN[%d] : chidx(%d)\n",__LINE__, chidx);
	uvdbg("chidx: %d buflen: %d\n", (unsigned int)ep, buflen);

	DEBUGASSERT(priv && buffer && chidx < BCM2835_MAX_TX_FIFOS && buflen > 0);

	/* We must have exclusive access to the USB host hardware and state structures */

	hbahn_dbg1("*HBAHN[%d] : chidx(%d)\n",__LINE__, chidx);
	bcm2835_takesem(&priv->exclsem);

	/* Handle IN and OUT transfer slightly differently */

	hbahn_dbg1("*HBAHN[%d] : chidx(%d)\n",__LINE__, chidx);
	if (priv->chan[chidx].in) {
	hbahn_dbg1("*HBAHN[%d] : IN transfer, chidx(%d)\n",__LINE__, chidx);
		nbytes = bcm2835_in_transfer(priv, chidx, buffer, buflen);
	} else {
	hbahn_dbg1("*HBAHN[%d] : OUT transfer, chidx(%d)\n",__LINE__, chidx);
		nbytes = bcm2835_out_transfer(priv, chidx, buffer, buflen);
	}
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	bcm2835_givesem(&priv->exclsem);
	return nbytes;
}

/****************************************************************************
 * Name: bcm2835_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback function
 *     when the transfer completes.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int bcm2835_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep, FAR uint8_t *buffer, size_t buflen, usbhost_asynch_t callback, FAR void *arg)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	unsigned int chidx = (unsigned int)ep;
	int ret;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	uvdbg("chidx: %d buflen: %d\n", (unsigned int)ep, buflen);

	DEBUGASSERT(priv && buffer && chidx < BCM2835_MAX_TX_FIFOS && buflen > 0);

	/* We must have exclusive access to the USB host hardware and state structures */

	bcm2835_takesem(&priv->exclsem);

	/* Handle IN and OUT transfer slightly differently */

	if (priv->chan[chidx].in) {
		ret = bcm2835_in_asynch(priv, chidx, buffer, buflen, callback, arg);
	} else {
		ret = bcm2835_out_asynch(priv, chidx, buffer, buflen, callback, arg);
	}

	bcm2835_givesem(&priv->exclsem);
	return ret;
}
#endif							/* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: bcm2835_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which an
 *      asynchronous transfer should be transferred.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

static int bcm2835_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	FAR struct bcm2835_chan_s *chan;
	unsigned int chidx = (unsigned int)ep;
	irqstate_t flags;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	uvdbg("chidx: %u: %d\n", chidx);

	DEBUGASSERT(priv && chidx < BCM2835_MAX_TX_FIFOS);
	chan = &priv->chan[chidx];

	/* We need to disable interrupts to avoid race conditions with the asynchronous
	 * completion of the transfer being cancelled.
	 */

	flags = irqsave();

	/* Halt the channel */

	bcm2835_chan_halt(priv, chidx, CHREASON_CANCELLED);
	chan->result = -ESHUTDOWN;

	/* Is there a thread waiting for this transfer to complete? */

	if (chan->waiter) {
#ifdef CONFIG_USBHOST_ASYNCH
		/* Yes.. there should not also be a callback scheduled */

		DEBUGASSERT(chan->callback == NULL);
#endif

		/* Wake'em up! */

		bcm2835_givesem(&chan->waitsem);
		chan->waiter = false;
	}
#ifdef CONFIG_USBHOST_ASYNCH
	/* No.. is an asynchronous callback expected when the transfer
	 * completes?
	 */

	else if (chan->callback) {
		usbhost_asynch_t callback;
		FAR void *arg;

		/* Extract the callback information */

		callback = chan->callback;
		arg = chan->arg;

		chan->callback = NULL;
		chan->arg = NULL;
		chan->xfrd = 0;

		/* Then perform the callback */

		callback(arg, -ESHUTDOWN);
	}
#endif

	irqrestore(flags);
	return OK;
}

/************************************************************************************
 * Name: bcm2835_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int bcm2835_connect(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_hubport_s *hport, bool connected)
{
	FAR struct bcm2835_usbhost_s *priv = (FAR struct bcm2835_usbhost_s *)drvr;
	irqstate_t flags;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	DEBUGASSERT(priv != NULL && hport != NULL);

	/* Set the connected/disconnected flag */

	hport->connected = connected;
	uinfo("Hub port %d connected: %s\n", hport->port, connected ? "YES" : "NO");

	/* Report the connection event */

	flags = irqsave();
	priv->hport = hport;
	if (priv->pscwait) {
		priv->pscwait = false;
		bcm2835_givesem(&priv->pscsem);
	}

	irqrestore(flags);
	return OK;
}
#endif

/****************************************************************************
 * Name: bcm2835_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be a port
 *      on a hub.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void bcm2835_disconnect(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_hubport_s *hport)
{
	DEBUGASSERT(hport != NULL);
	hport->devclass = NULL;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/
/****************************************************************************
 * Name: bcm2835_portreset
 *
 * Description:
 *   Reset the USB host port.
 *
 *   NOTE: "Before starting to drive a USB reset, the application waits for the
 *   OTG interrupt triggered by the debounce done bit (DBCDNE bit in
 *   OTG_GOTGINT), which indicates that the bus is stable again after the
 *   electrical debounce caused by the attachment of a pull-up resistor on DP
 *   or DM (LS).
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bcm2835_portreset(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t hport;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	hport = getreg32(DWHCI_HOST_PORT);
	hport &= ~DWHCI_HOST_PORT_DEFAULT_MASK;
	hport |= DWHCI_HOST_PORT_RESET;
	putreg32(hport, DWHCI_HOST_PORT);

	up_mdelay(100);		/* see USB 2.0 spec (tDRSTR) */

	hport = getreg32(DWHCI_HOST_PORT);
	hport &= ~DWHCI_HOST_PORT_DEFAULT_MASK;
	hport &= ~DWHCI_HOST_PORT_RESET;
	putreg32(hport, DWHCI_HOST_PORT);

	/* normally 10ms, seems to be too short for some devices */
	up_mdelay (100);	/* see USB 2.0 spec (tRSTRCY) */
}

/****************************************************************************
 * Name: bcm2835_flush_txfifos
 *
 * Description:
 *   Flush the selected Tx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *   txfnum -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void bcm2835_flush_txfifos(FAR struct bcm2835_usbhost_s *priv, uint32_t txfnum)
{
	uint32_t reset = 0;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	reset |= DWHCI_CORE_RESET_TX_FIFO_FLUSH;
	reset &= ~DWHCI_CORE_RESET_TX_FIFO_NUM__MASK;
	reset |= txfnum << DWHCI_CORE_RESET_TX_FIFO_NUM__SHIFT;
	putreg32(reset, DWHCI_CORE_RESET);

	if (bcm2835_wait_for_bit_check(priv, DWHCI_CORE_RESET, 
				DWHCI_CORE_RESET_TX_FIFO_FLUSH, 
				FALSE, 10))
	{
		up_udelay(30);		/* Wait for 3 PHY clocks */
	}
}

/****************************************************************************
 * Name: bcm2835_flush_rxfifo
 *
 * Description:
 *   Flush the Rx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void bcm2835_flush_rxfifo(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t reset = 0;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	reset |= DWHCI_CORE_RESET_RX_FIFO_FLUSH;
	putreg32(reset, DWHCI_CORE_RESET);

	if (bcm2835_wait_for_bit_check(priv, DWHCI_CORE_RESET, 
				DWHCI_CORE_RESET_RX_FIFO_FLUSH, 
				FALSE, 10))
	{
		up_udelay(30);		/* Wait for 3 PHY clocks */
	}
}

/****************************************************************************
 * Name: bcm2835_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline void bcm2835_sw_initialize(FAR struct bcm2835_usbhost_s *priv)
{
	FAR struct usbhost_driver_s *drvr;
	FAR struct usbhost_hubport_s *hport;
	int i;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Initialize the device operations */

	drvr = &priv->drvr;
	drvr->ep0configure = bcm2835_ep0configure;
	drvr->epalloc = bcm2835_epalloc;
	drvr->epfree = bcm2835_epfree;
	drvr->alloc = bcm2835_otg_alloc;
	drvr->free = bcm2835_otg_free;
	drvr->ioalloc = bcm2835_otg_ioalloc;
	drvr->iofree = bcm2835_otg_iofree;
	drvr->ctrlin = bcm2835_ctrlin;
	drvr->ctrlout = bcm2835_ctrlout;
	drvr->transfer = bcm2835_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
	drvr->asynch = bcm2835_asynch;
#endif
	drvr->cancel = bcm2835_cancel;
#ifdef CONFIG_USBHOST_HUB
	drvr->connect = bcm2835_connect;
#endif
	drvr->disconnect = bcm2835_disconnect;

	/* Initialize the public port representation */

	hport = &priv->rhport.hport;
	hport->drvr = drvr;
#ifdef CONFIG_USBHOST_HUB
	hport->parent = NULL;
#endif
	hport->ep0 = (usbhost_ep_t) & priv->ep0;
	hport->speed = USB_SPEED_FULL;

	/* Initialize function address generation logic */

	usbhost_devaddr_initialize(&priv->rhport);

	/* Initialize semaphores */

	sem_init(&priv->pscsem, 0, 0);
	sem_init(&priv->exclsem, 0, 1);

	/* The pscsem semaphore is used for signaling and, hence, should not have
	 * priority inheritance enabled.
	 */

	sem_setprotocol(&priv->pscsem, SEM_PRIO_NONE);

	/* Initialize the driver state data */

	priv->smstate = SMSTATE_DETACHED;
	priv->connected = false;
	priv->change = false;

	/* Put all of the channels back in their initial, allocated state */

	memset(priv->chan, 0, BCM2835_MAX_TX_FIFOS * sizeof(struct bcm2835_chan_s));

	/* Initialize each channel */

	for (i = 0; i < BCM2835_MAX_TX_FIFOS; i++) {
		FAR struct bcm2835_chan_s *chan = &priv->chan[i];

		chan->chidx = i;

		/* The waitsem semaphore is used for signaling and, hence, should not
		 * have priority inheritance enabled.
		 */

		sem_init(&chan->waitsem, 0, 0);
		sem_setprotocol(&chan->waitsem, SEM_PRIO_NONE);
	}
}


/****************************************************************************
 * Name: bcm2835_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from bcm2835_hw_initialize().  But if OTG mode
 *   were supported, this function would also be called to swtich between
 *   host and device modes on a connector ID change interrupt.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_host_initialize(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t creg, cfg2, hport;
	uint32_t nonperiodic_txfifosize, hostperiodic_txfifosize;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	/* Restart the PHY clock */

	putreg32(0, BCM2835_USB_POWER);
	
	creg = getreg32(DWHCI_HOST_CFG);
	creg &= ~DWHCI_HOST_CFG_FSLS_PCLK_SEL__MASK;

	cfg2 = getreg32(DWHCI_CORE_HW_CFG2);
	if (   DWHCI_CORE_HW_CFG2_HS_PHY_TYPE(cfg2) == DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_ULPI
	    && DWHCI_CORE_HW_CFG2_FS_PHY_TYPE(cfg2) == DWHCI_CORE_HW_CFG2_FS_PHY_TYPE_DEDICATED
	    && (getreg32(DWHCI_CORE_USB_CFG) & DWHCI_CORE_USB_CFG_ULPI_FSLS))
	{
		creg |= DWHCI_HOST_CFG_FSLS_PCLK_SEL_48_MHZ;
	}
	else
	{
		creg |= DWHCI_HOST_CFG_FSLS_PCLK_SEL_30_60_MHZ;
	}

	putreg32(creg, DWHCI_HOST_CFG);

#ifdef DWC_CFG_DYNAMIC_FIFO
	putreg32(DWC_CFG_HOST_RX_FIFO_SIZE, DWHCI_CORE_RX_FIFO_SIZ);
	
	nonperiodic_txfifosize = 0;
	nonperiodic_txfifosize |= DWC_CFG_HOST_RX_FIFO_SIZE;
	nonperiodic_txfifosize |= DWC_CFG_HOST_NPER_TX_FIFO_SIZE << 16;
	putreg32(nonperiodic_txfifosize, DWHCI_CORE_NPER_TX_FIFO_SIZ);
	
	hostperiodic_txfifosize = 0;
	hostperiodic_txfifosize |= DWC_CFG_HOST_RX_FIFO_SIZE + DWC_CFG_HOST_NPER_TX_FIFO_SIZE;
	hostperiodic_txfifosize |= DWC_CFG_HOST_PER_TX_FIFO_SIZE << 16;
	putreg32(hostperiodic_txfifosize, DWHCI_CORE_NPER_TX_FIFO_SIZ);
#endif

	/* Flush all FIFOs */
	bcm2835_flush_txfifos(priv, 0x10);	 	/* Flush all TX FIFOs */
	bcm2835_flush_rxfifo(priv);			/* Flush all RX FIFOs */

	hport = getreg32(DWHCI_HOST_PORT);
	hport &= ~DWHCI_HOST_PORT_DEFAULT_MASK;
	if (!(hport & DWHCI_HOST_PORT_POWER))
	{
		hport |= DWHCI_HOST_PORT_POWER;
		putreg32(hport, DWHCI_HOST_PORT);
	}

	bcm2835_host_int_enable();
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return TRUE;
}

/****************************************************************************
 * Name: bcm2835_enable_root_port / disable_root_port
 *
 * Description:
 *   Initialize/re-enable root port
 *   this function is called only from bcm2835_hw_initialize(). 
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_enable_root_port(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t hport;

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* the different device need more big timeout value */
	if (!bcm2835_wait_for_bit_check(priv, DWHCI_HOST_PORT, 
				DWHCI_HOST_PORT_CONNECT, 
				TRUE, 20000))
	{
		lldbg("failed to wait for bit check\n");
		return FALSE;
	}

	up_mdelay(100); 	/* see USB 2.0 spec */

	hport = getreg32(DWHCI_HOST_PORT);
	hport &= ~DWHCI_HOST_PORT_DEFAULT_MASK;
	hport |= DWHCI_HOST_PORT_RESET;
	putreg32(hport, DWHCI_HOST_PORT);

	up_mdelay(100);		/* see USB 2.0 spec (tDRSTR) */

	hport = getreg32(DWHCI_HOST_PORT);
	hport &= ~DWHCI_HOST_PORT_DEFAULT_MASK;
	hport &= ~DWHCI_HOST_PORT_RESET;
	putreg32(hport, DWHCI_HOST_PORT);

	/* normally 10ms, seems to be too short for some devices */
	up_mdelay (100);		/* see USB 2.0 spec (tRSTRCY) */
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return TRUE;
}

static void bcm2835_disable_root_port(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t hp;

	hp = getreg32(DWHCI_HOST_PORT);
	hp &= ~DWHCI_HOST_PORT_POWER;
	putreg32(hp,DWHCI_HOST_PORT);
}

/****************************************************************************
 * Name: bcm2835_rootport_initialize
 *
 * Description:
 *   Initialize/re-initialize root port
 *   this function is called only from bcm2835_hw_initialize(). 
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_rootport_initialize(FAR struct bcm2835_usbhost_s *priv)
{
	uint8_t speed;
	speed = bcm2835_get_port_speed(priv);
	if (speed == USB_SPEED_UNKNOWN)
	{
		lldbg("Cannot detect port speed\n");
		return FALSE;
	}
	
	switch (speed) {
 	case USB_SPEED_LOW :
		lowsyslog(LOG_INFO, "Low Speed USB device detected\n");
		break;
 	case USB_SPEED_FULL :
		lowsyslog(LOG_INFO, "Full Speed USB device detected\n");
		break;
 	case USB_SPEED_HIGH :
		lowsyslog(LOG_INFO, "High Speed USB device detected\n");
		break;
 	case USB_SPEED_VARIABLE :
		lowsyslog(LOG_INFO, "Variable Speed USB device detected\n");
		break;
	default:
		lowsyslog(LOG_INFO, "Unknown Speed USB device detected\n");
		break;
	}

	// check for over-current
	if (bcm2835_detect_over_current (priv))
	{
		lldbg("Over-current condition\n");
		bcm2835_disable_root_port(priv);
		return FALSE;
	}

	/* TODO : only check on boot time, must implement by plug-in/out envent later */
	bcm2835_usb_connected(priv);


#if 0
	assert (pThis->m_pHost != 0);
	TUSBSpeed Speed = DWHCIDeviceGetPortSpeed (pThis->m_pHost);
	if (Speed == USBSpeedUnknown)
	{
		lldbg("Cannot detect port speed\n");

		return FALSE;
	}
	
	// first create default device
	assert (pThis->m_pDevice == 0);
	pThis->m_pDevice = (TUSBDevice *) malloc (sizeof (TUSBDevice));
	assert (pThis->m_pDevice != 0);
	USBDevice (pThis->m_pDevice, pThis->m_pHost, Speed, 0, 1);

	if (!USBDeviceInitialize (pThis->m_pDevice))
	{
		_USBDevice (pThis->m_pDevice);
		free (pThis->m_pDevice);
		pThis->m_pDevice = 0;

		return FALSE;
	}

	TString *pNames = USBStandardHubGetDeviceNames (pThis->m_pDevice);
	assert (pNames != 0);

	lldbg("Device %s found\n", StringGet (pNames));

	_String (pNames);
	free (pNames);

	// now create specific device from default device
	TUSBDevice *pChild = USBDeviceFactoryGetDevice (pThis->m_pDevice);
	if (pChild != 0)
	{
		_USBDevice (pThis->m_pDevice);		// delete default device
		free (pThis->m_pDevice);
		pThis->m_pDevice = pChild;		// assign specific device

		if (!(*pThis->m_pDevice->Configure) (pThis->m_pDevice))
		{
			lldbg("Cannot configure device\n");

			_USBDevice (pThis->m_pDevice);
			free (pThis->m_pDevice);
			pThis->m_pDevice = 0;

			return FALSE;
		}
		
		lldbg("Device configured\n");
	}
	else
	{
		lldbg("Device is not supported\n");
		
		_USBDevice (pThis->m_pDevice);
		free (pThis->m_pDevice);
		pThis->m_pDevice = 0;

		return FALSE;
	}
#endif

	return TRUE;
}

/****************************************************************************
 * Name: bcm2835_otg_init_core
 *
 * Description:
 *   One-time setup of the otg host controller harware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   TRUE on success; a negated errno value on failure.
 *
 ****************************************************************************/

static bool bcm2835_otg_init_core(FAR struct bcm2835_usbhost_s *priv)
{
	uint32_t areg, creg, cfg2;
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	creg = getreg32(DWHCI_CORE_USB_CFG);
	creg &= ~DWHCI_CORE_USB_CFG_ULPI_EXT_VBUS_DRV;
	creg &= ~DWHCI_CORE_USB_CFG_TERM_SEL_DL_PULSE;
	putreg32(creg, DWHCI_CORE_USB_CFG);

	if (!bcm2835_otg_reset(priv))
	{
		lldbg("Reset failed\n");
		return FALSE;
	}

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	creg = getreg32(DWHCI_CORE_USB_CFG);
	creg &= ~DWHCI_CORE_USB_CFG_ULPI_UTMI_SEL;	/* select UTMI+ */
	creg &= ~DWHCI_CORE_USB_CFG_PHYIF;		/* UTMI width is 8 */
	putreg32(creg, DWHCI_CORE_USB_CFG);

	/* Internal DMA mode only */
	cfg2 = getreg32(DWHCI_CORE_HW_CFG2);
	assert (DWHCI_CORE_HW_CFG2_ARCHITECTURE(cfg2) == 2);
	
	creg = getreg32(DWHCI_CORE_USB_CFG);
	if (   DWHCI_CORE_HW_CFG2_HS_PHY_TYPE(cfg2) == DWHCI_CORE_HW_CFG2_HS_PHY_TYPE_ULPI
	    && DWHCI_CORE_HW_CFG2_FS_PHY_TYPE(cfg2) == DWHCI_CORE_HW_CFG2_FS_PHY_TYPE_DEDICATED)
	{
		creg |= DWHCI_CORE_USB_CFG_ULPI_FSLS;
		creg |= DWHCI_CORE_USB_CFG_ULPI_CLK_SUS_M;
	}
	else
	{
		creg &= ~DWHCI_CORE_USB_CFG_ULPI_FSLS;
		creg &= ~DWHCI_CORE_USB_CFG_ULPI_CLK_SUS_M;
	}
	putreg32(creg, DWHCI_CORE_USB_CFG);

	assert (priv->channels == 0);
	priv->channels = DWHCI_CORE_HW_CFG2_NUM_HOST_CHANNELS(cfg2);
	assert (4 <= priv->channels && priv->channels <= DWHCI_MAX_CHANNELS);
	hbahn_dbg("*HBAHN[%d] : channels(%d)\n",__LINE__, priv->channels);

	areg = getreg32(DWHCI_CORE_AHB_CFG);
	areg |= DWHCI_CORE_AHB_CFG_DMAENABLE;
	areg |= DWHCI_CORE_AHB_CFG_WAIT_AXI_WRITES;
	areg &= ~DWHCI_CORE_AHB_CFG_MAX_AXI_BURST__MASK;
	putreg32(areg, DWHCI_CORE_AHB_CFG);

	// HNP and SRP are not used
	creg = getreg32(DWHCI_CORE_USB_CFG);
	creg &= ~DWHCI_CORE_USB_CFG_HNP_CAPABLE;
	creg &= ~DWHCI_CORE_USB_CFG_SRP_CAPABLE;
	putreg32(creg, DWHCI_CORE_USB_CFG);

	bcm2835_common_int_enable();
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	return TRUE;
}

/****************************************************************************
 * Name: bcm2835_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller harware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline int bcm2835_hw_initialize(FAR struct bcm2835_usbhost_s *priv)
{
	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);

	/* Initialzie USB OTG core */
	if (!bcm2835_otg_init_core(priv))
	{
		lldbg("Fail to initialize core\n");
		return FALSE;
	}

	/* Enable USB OTG global interrupts */
	bcm2835_gint_enable();

	
	/* Initialize host mode and return success */
	if (!bcm2835_host_initialize(priv))
	{
		lldbg("Fail to initialize host\n");
		return FALSE;
	}

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* 
	 * The following calls will fail if there is no device or no supported device connected
	 * to root port. This is not an error because the system may run without an USB device. 
	 */
	if (!bcm2835_enable_root_port(priv))
	{
		lldbg("No device connected to root port\n");
		return TRUE;
	}
	lldbg("bcm2835 root port enable done\n");

	if (!bcm2835_rootport_initialize(priv))
	{
		lldbg("Cannot initialize root port\n");
		return TRUE;
	}
	lldbg("root_port_initiialize done\n");

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

FAR struct usbhost_connection_s *bcm2835_otghost_initialize(int controller)
{
	uint32_t reg;

	/* At present, there is only support for a single OTG  host. Hence it is
	 * pre-allocated as g_usbhost.  However, in most code, the private data
	 * structure will be referenced using the 'priv' pointer (rather than the
	 * global data) in order to simplify any future support for multiple devices.
	 */

	FAR struct bcm2835_usbhost_s *priv = &g_usbhost;

	/* Sanity checks */
	DEBUGASSERT(controller == 0);

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Disable interrupts at the interrupt controller */
	up_disable_irq(BCM2835_IRQ_USB);

	/* check vendor ID */
	reg = getreg32(DWHCI_CORE_VENDOR_ID);
	if (reg != 0x4F54280A)
	{
		lowsyslog(LOG_INFO, "Unknown vendor 0x%0X", reg);
		return FALSE;
	}
	

	if (set_power_state_on(DEVICE_ID_USB_HCD) < 0)
	{
		lldbg("Cannot power on\n");
		return FALSE;
	}
	lldbg("set_power_state_on done\n");
	/* Make sure that interrupts from the OTG core are disabled */

	bcm2835_gint_disable();

	/* Reset the state of the host driver */

	bcm2835_sw_initialize(priv);

	/* Initialize the USB OTG  core */

	bcm2835_hw_initialize(priv);

	/* Attach USB host controller interrupt handler */

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	if (irq_attach(BCM2835_IRQ_USB, bcm2835_gint_isr, NULL) != 0) {
		usbhost_trace1(OTG_TRACE1_IRQATTACH, 0);
		return NULL;
	}

	hbahn_dbg("*HBAHN[%d] : \n",__LINE__);
	/* Enable interrupts at the interrupt controller */
	up_enable_irq(BCM2835_IRQ_USB);

	return &g_usbconn;
}

#endif							/* CONFIG_USBHOST && CONFIG_BCM2835_OTG */
