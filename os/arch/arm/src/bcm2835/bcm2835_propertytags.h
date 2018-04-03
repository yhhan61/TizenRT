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
// bcm2835_propertytag.h
//
// Copyright (C) 2014  R. Stange <rsta2@o2online.de>
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
#ifndef _bcm2835_propertytag_h
#define _bcm2835_propertytag_h

#include "bcm2835_mailbox.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PROPTAG_END			0x00000000

#define PROPTAG_GET_FIRMWARE_REVISION	0x00000001
#define PROPTAG_GET_BOARD_MODEL		0x00010001
#define PROPTAG_GET_BOARD_REVISION	0x00010002
#define PROPTAG_GET_MAC_ADDRESS		0x00010003
#define PROPTAG_GET_BOARD_SERIAL	0x00010004
#define PROPTAG_GET_ARM_MEMORY		0x00010005
#define PROPTAG_GET_VC_MEMORY		0x00010006
#define PROPTAG_SET_POWER_STATE		0x00028001
#define PROPTAG_GET_CLOCK_RATE		0x00030002
#define PROPTAG_GET_TEMPERATURE		0x00030006
#define PROPTAG_GET_EDID_BLOCK		0x00030020
#define PROPTAG_GET_DISPLAY_DIMENSIONS	0x00040003
#define PROPTAG_GET_COMMAND_LINE	0x00050001

typedef struct property_tags_s {
	uint32_t tag_id;
	uint32_t value_buffer_size;	// bytes, multiple of 4
	uint32_t value_length;		// bytes
#define VALUE_LENGTH_RESPONSE	(1 << 31)
} property_tags_s;

typedef struct simple_tag_s {
	property_tags_s Tag;
	uint32_t value;
} simple_tag_s;

typedef struct propertytag_power_state {
	property_tags_s Tag;
	uint32_t device_id;
#define DEVICE_ID_SD_CARD	0
#define DEVICE_ID_USB_HCD	3
	uint32_t state;
#define POWER_STATE_OFF		(0 << 0)
#define POWER_STATE_ON		(1 << 0)
#define POWER_STATE_WAIT	(1 << 1)
#define POWER_STATE_NO_DEVICE	(1 << 1)	// in response
} propertytag_power_state;

typedef struct bcm_property_tags_s {
	mailbox_s mailbox;
} bcm_property_tags_s;

void select_property_tags(bcm_property_tags_s *priv);
void unselect_property_tags(bcm_property_tags_s *priv);

int bcm_property_get_tags(bcm_property_tags_s *priv, uint32_t tag_id,		// tag identifier
						  void *p_tag,			// pointer to tag struct
						  unsigned tag_size,		// size of tag struct
						  unsigned req_param_size /* = 0 */);	// number of parameter bytes

#ifdef __cplusplus
}
#endif
#endif
