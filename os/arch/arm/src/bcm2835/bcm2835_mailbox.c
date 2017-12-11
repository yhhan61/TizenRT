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
//
// bcm2835_mailbox.c
//
// Copyright (C) 2014-2016  R. Stange <rsta2@o2online.de>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <sys/types.h>

#include <arch/board/memory.h>
#include <arch/armv6/cacheops.h>

#include "up_arch.h"
#include "bcm2835_mmailbox.h"

void select_mailbox(mailbox_s *priv, uint32_t channel)
{
	assert(priv != 0);
	priv->channel = channel;
}

void unselect_mailbox(mailbox_s *priv)
{
	assert(priv != 0);
}

uint32_t mailbox_exchange(mailbox_s *priv, uint32_t data)
{
	uint32_t result = 0;
	assert(priv != 0);

	DataMemBarrier();

	mailbox_flush(priv);

	mailbox_write(priv, data);

	result = mailbox_read(priv);

	DataMemBarrier();

	return result;
}

void mailbox_flush(mailbox_s *priv)
{
	assert(priv != 0);

	while (!(getreg32(MAILBOX0_STATUS) & MAILBOX_STATUS_EMPTY)) {
		getreg32(MAILBOX0_READ);
		up_mdelay(20);
	}
}

uint32_t mailbox_read(mailbox_s *priv)
{
	uint32_t result;
	assert(priv != 0);

	do {
		while (getreg32(MAILBOX0_STATUS) & MAILBOX_STATUS_EMPTY) {
			// do nothing
		}

		result = getreg32(MAILBOX0_READ);
	} while ((result & 0xF) != priv->channel);	// channel number is in the lower 4 bits

	return result & ~0xF;
}

void mailbox_write(mailbox_s *priv, uint32_t data)
{
	assert(priv != 0);

	while (getreg32(MAILBOX1_STATUS) & MAILBOX_STATUS_FULL) {
		// do nothing
	}

	assert((data & 0xF) == 0);
	putreg32(priv->channel | data, MAILBOX1_WRITE);	// channel number is in the lower 4 bits
}
