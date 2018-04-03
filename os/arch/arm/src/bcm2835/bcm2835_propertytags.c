//
// bcm2835_propertytags.c
//
// Copyright (C) 2014-2017  R. Stange <rsta2@o2online.de>
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

#include <arch/armv6/cacheops.h>
#include <arch/board/memory.h>

#include "up_arch.h"
#include "bcm2835_mailbox.h"
#include "bcm2835_propertytags.h"

typedef struct property_buffer_s {
	uint32_t buffer_size;		// bytes
	uint32_t code;
#define CODE_REQUEST		0x00000000
#define CODE_RESPONSE_SUCCESS	0x80000000
#define CODE_RESPONSE_FAILURE	0x80000001
	uint8_t Tags[0];
} property_buffer_s;

void select_property_tags(bcm_property_tags_s *priv)
{
	assert(priv != 0);

	select_mailbox(&priv->mailbox, BCM_MAILBOX_PROP_OUT);
}

void unselect_property_tags(bcm_property_tags_s *priv)
{
	assert(priv != 0);

	unselect_mailbox(&priv->mailbox);
}

int bcm_property_get_tags(bcm_property_tags_s *priv, uint32_t tag_id, void *p_tag, unsigned tag_size, unsigned req_param_size)
{
	assert(priv != 0);

	assert(p_tag != 0);
	assert(tag_size >= sizeof(simple_tag_s));
	unsigned buffer_size = sizeof(property_buffer_s) + tag_size + sizeof(uint32_t);
	assert((buffer_size & 3) == 0);

	// cannot use malloc() here because this is used before mem_init() is called
	uint8_t Buffer[buffer_size + 15];
	property_buffer_s *pBuffer = (property_buffer_s *)(((uint32_t) Buffer + 15) & ~15);

	pBuffer->buffer_size = buffer_size;
	pBuffer->code = CODE_REQUEST;
	memcpy(pBuffer->Tags, p_tag, tag_size);

	property_tags_s *p_header = (property_tags_s *) pBuffer->Tags;
	p_header->tag_id = tag_id;
	p_header->value_buffer_size = tag_size - sizeof(property_tags_s);
	p_header->value_length = req_param_size & ~VALUE_LENGTH_RESPONSE;

	uint32_t *pEndTag = (uint32_t *)(pBuffer->Tags + tag_size);
	*pEndTag = PROPTAG_END;

	CleanDataCache();
	DataSyncBarrier();

	uint32_t buffer_address = BUS_ADDRESS((uint32_t) pBuffer);
	if (mailbox_exchange(&priv->mailbox, buffer_address) != buffer_address) {
		return FALSE;
	}

	InvalidateDataCache();
	DataSyncBarrier();

	if (pBuffer->code != CODE_RESPONSE_SUCCESS) {
		return FALSE;
	}

	if (!(p_header->value_length & VALUE_LENGTH_RESPONSE)) {
		return FALSE;
	}

	p_header->value_length &= ~VALUE_LENGTH_RESPONSE;
	if (p_header->value_length == 0) {
		return FALSE;
	}

	memcpy(p_tag, pBuffer->Tags, tag_size);

	return TRUE;
}
