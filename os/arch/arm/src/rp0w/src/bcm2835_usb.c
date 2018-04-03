/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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
 * Included Files
 ************************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/usb/usbdev.h>
#include <tinyara/usb/usbhost.h>
#include <tinyara/usb/usbdev_trace.h>

#include "up_arch.h"
#include "rp0w.h"

#ifdef CONFIG_BCM2835_OTG
#include "bcm2835_otg.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_BCM2835_OTG is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_RP0W_USBHOST_PRIO
#  define CONFIG_RP0W_USBHOST_PRIO 100
#endif

#ifndef CONFIG_RP0W_USBHOST_STACKSIZE
#  define CONFIG_RP0W_USBHOST_STACKSIZE 1024
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;

  uvdbg("Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uvdbg("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: bcm2835_usbinitialize
 *
 * Description:
 *   Called from bcm2835_usbinitialize very early in initialization to setup USB-related
 *   GPIO pins for the BCM2835 Raspberry Pi0 W board.
 *
 ************************************************************************************/

void bcm2835_usbinitialize(void)
{
#ifdef CONFIG_BCM2835_OTG
	/* Nothing to do */
#endif
}

/***********************************************************************************
 * Name: bcm2835_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
int bcm2835_usbhost_initialize(void)
{
	int pid;
#if defined(CONFIG_USBHOST_HUB)    || defined(CONFIG_USBHOST_MSC) || \
    defined(CONFIG_USBHOST_HIDKBD) || defined(CONFIG_USBHOST_HIDMOUSE)
	int ret;
#endif

	/* First, register all of the class drivers needed to support the drivers
	 * that we care about:
	 */

	uvdbg("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
	/* Initialize USB hub class support */

	ret = usbhost_hub_initialize();
	if (ret < 0)
	{
		udbg("ERROR: usbhost_hub_initialize failed: %d\n", ret);
	}
#endif

#if defined(CONFIG_USBHOST_MSC) && !defined(CONFIG_USBHOST_BULK_DISABLE) && \
        !defined(CONFIG_DISABLE_MOUNTPOINT) && CONFIG_NFILE_DESCRIPTORS > 0
	/* Register the USB mass storage class class */

	ret = usbhost_msc_initialize();
	if (ret != OK)
	{
		udbg("ERROR: Failed to register the mass storage class: %d\n", ret);
	}
#endif

#ifdef CONFIG_USBHOST_CDCACM
	/* Register the CDC/ACM serial class */

	ret = usbhost_cdcacm_initialize();
	if (ret != OK)
	{
		udbg("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
	}
#endif

#ifdef CONFIG_USBHOST_HIDKBD
	/* Initialize the HID keyboard class */

	ret = usbhost_kbdinit();
	if (ret != OK)
	{
		udbg("Failed to register the HID keyboard class\n");
	}
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
	/* Initialize the HID mouse class */

	ret = usbhost_mouse_init();
	if (ret != OK)
	{
		udbg("Failed to register the HID mouse class\n");
	}
#endif

	/* Then get an instance of the USB host interface */

	uvdbg("Initialize USB host\n");
	g_usbconn = bcm2835_otghost_initialize(0);
	if (g_usbconn)
	{
		/* Start a thread to handle device connection. */

		uvdbg("Start usbhost_waiter\n");

		pid = task_create("usbhost", CONFIG_RP0W_USBHOST_PRIO,
				CONFIG_RP0W_USBHOST_STACKSIZE,
				(main_t)usbhost_waiter, (FAR char * const *)NULL);
		return pid < 0 ? -ENOEXEC : OK;
	}

	return -ENODEV;
}
#endif

/************************************************************************************
 * Name:  bcm2835_usbsuspend
 *
 * Description:
 *   Board logic must provide the bcm2835_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_USBDEV
void bcm2835_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_BCM2835_OTG */
