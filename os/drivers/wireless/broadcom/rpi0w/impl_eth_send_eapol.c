/****************************************************************************
 * Copyright 2018 DIGNSYS All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
****************************************************************************/

#include <errno.h>
#include <stdint.h>
#include <sys/types.h>
#include <stdbool.h>
#include "wwd_buffer_interface.h"
#include "wwd_network_interface.h"
#include "wwd_network_constants.h"
#include "wwd_structures.h"
#include <lwip/pbuf.h>

typedef struct __attribute__((packed, aligned(1))) {
#define _MAC_HWADDR_LEN 			6
	uint8_t destination[_MAC_HWADDR_LEN];
	uint8_t source[_MAC_HWADDR_LEN];
	uint16_t ether_type;
} _ethernet_header_t;

int eth_send_eapol(const u8 *src_mac_addr, const u8 *dst_mac_addr,	const u8 *buf, u16 len, u16 proto)
{
#define SDPCM_DATA_HEADER_SIZE	(22)

	wiced_buffer_t pbuf;
	_ethernet_header_t *ethhdr;
	void* payload;
	uint16_t packet_size;

	packet_size = SDPCM_DATA_HEADER_SIZE + sizeof(_ethernet_header_t) + len;
	internal_host_buffer_get(&pbuf, WWD_NETWORK_TX, packet_size, 0);
	if (pbuf == NULL)
		return -1;

	if (host_buffer_add_remove_at_front(&pbuf, SDPCM_DATA_HEADER_SIZE) != WWD_SUCCESS) {
		host_buffer_release(pbuf, WWD_NETWORK_TX);

		return -1;
	}

	ethhdr = (_ethernet_header_t*)host_buffer_get_current_piece_data_pointer(pbuf);
	payload = (void*)((_ethernet_header_t*)(ethhdr + 1));

	memcpy(ethhdr->source, src_mac_addr, _MAC_HWADDR_LEN);
	memcpy(ethhdr->destination, dst_mac_addr, _MAC_HWADDR_LEN);
	ethhdr->ether_type = htons(proto);
	memcpy(payload, buf, len);

	wwd_network_send_ethernet_data(pbuf, WWD_STA_INTERFACE);

	return 0;
}
