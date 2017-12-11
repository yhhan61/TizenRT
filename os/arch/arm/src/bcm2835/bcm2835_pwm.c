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
 * arch/arm/src/bcm2835/bcm2835_pwm.c
 *
 *   Copyright (C) 2011-2012, 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <tinyara/pwm.h>

#include "up_arch.h"
#include "bcm2835_pwm.h"
#include "bcm2835_gpio.h"
#include "bcm2835_vclk.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct bcm2835_pwmtimer_s {
	FAR const struct pwm_ops_s *ops;

	unsigned int base;
	uint8_t id;
	uint16_t pincfg;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static unsigned int bcm2835_get_oscclk(void)
{
	return BCM2835_CLOCK_FREQ;
}

/****************************************************************************
 * Name: bcm2835_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/
static int bcm2835_pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct bcm2835_pwmtimer_s *priv = (FAR struct bcm2835_pwmtimer_s *)dev;

	return bcm2835_configgpio(priv->pincfg);
}

/****************************************************************************
 * Name: bcm2835_pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int bcm2835_pwm_start(FAR struct pwm_lowerhalf_s *dev, FAR const struct pwm_info_s *info)
{
	uint32_t tcntb;
	uint32_t tcmpb;
	FAR struct bcm2835_pwmtimer_s *priv = (FAR struct bcm2835_pwmtimer_s *)dev;

	tcntb = bcm2835_get_oscclk() / info->frequency - 1;
	tcmpb = (((tcntb + 1) * info->duty) / 65536) - 1;

	if (priv->id == 0) {
		PWM0_DATA = tcmpb;
		PWM0_RANGE = tcntb;
		PWM_CTL |= PWM0_ENABLE;
	} else {
		PWM1_DATA = tcmpb;
		PWM1_RANGE = tcntb;
		PWM_CTL |= PWM1_ENABLE;
	}

	return OK;
}

/****************************************************************************
 * Name: bcm2835_pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/
static int bcm2835_pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct bcm2835_pwmtimer_s *priv = (FAR struct bcm2835_pwmtimer_s *)dev;

	PWM_CTL &= ~(priv->id == 0 ? PWM0_ENABLE : PWM1_ENABLE);

	return OK;
}

/****************************************************************************
 * Name: bcm2835_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int bcm2835_pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
	FAR struct bcm2835_pwmtimer_s *priv = (FAR struct bcm2835_pwmtimer_s *)dev;

	/* Make sure that the output has been stopped */
	bcm2835_pwm_stop(dev);

	/* Then put the GPIO pins back to the default state */
	return bcm2835_unconfiggpio(priv->pincfg);
}

/****************************************************************************
 * Name: bcm2835_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/
static int bcm2835_pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
	return -ENOTTY;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct pwm_ops_s g_pwm_ops = {
	.setup = bcm2835_pwm_setup,
	.shutdown = bcm2835_pwm_shutdown,
	.start = bcm2835_pwm_start,
	.stop = bcm2835_pwm_stop,
	.ioctl = bcm2835_pwm_ioctl,
};

#ifdef CONFIG_BCM2835_PWM0
static struct bcm2835_pwmtimer_s g_pwm0_0 = {
	.ops = &g_pwm_ops,
	.id = 0,
	.pincfg = GPIO_PWM_TOUT0,
	.base = BCM2835_PWM_BASE,
};
#endif

#ifdef CONFIG_BCM2835_PWM1
static struct bcm2835_pwmtimer_s g_pwm0_1 = {
	.ops = &g_pwm_ops,
	.id = 1,
	.pincfg = GPIO_PWM_TOUT1,
	.base = BCM2835_PWM_BASE,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2835_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use. The number of valid timer
 *     IDs varies with the BCM2835 family but is somewhere in the range of
 *     {0,1}.
 *
 * Returned Value:
 *   On success, a pointer to the lower-half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/
FAR struct pwm_lowerhalf_s *bcm2835_pwminitialize(int timer)
{
	struct pwm_lowerhalf_s *lower = NULL;

#ifdef CONFIG_BCM2835_PWM0
	if (timer == 0) {
		lower = (struct pwm_lowerhalf_s *)&g_pwm0_0;
	} else
#endif
#ifdef CONFIG_BCM2835_PWM1
		if (timer == 1) {
			lower = (struct pwm_lowerhalf_s *)&g_pwm0_1;
		} else
#endif
		{
			lldbg("ERROR: invalid PWM is requested\n");
		}

	return lower;
}
