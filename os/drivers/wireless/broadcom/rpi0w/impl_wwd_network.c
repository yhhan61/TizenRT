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

#include <stdlib.h>
#include <string.h>
#include <tinyara/kmalloc.h>
#include "lwip/tcpip.h"
#include "lwip/debug.h"
#include "lwip/netif.h"
#include "lwip/ipv4/igmp.h"
#include "lwip/netif/etharp.h"
#include "wwd_network.h"
#include "wwd_wifi.h"
#include "wwd_eapol.h"
#include "network/wwd_network_constants.h"
#include "network/wwd_network_interface.h"
#include "network/wwd_buffer_interface.h"
#include "wwd_assert.h"
#include "wiced_constants.h"


#ifdef ADD_LWIP_EAPOL_SUPPORT
#define ETHTYPE_EAPOL    0x888E
#endif

#if LWIP_NETIF_HOSTNAME
#define LWIP_DEFAULT_CLIENT_OBJECT_NAME     "lwip"
#endif /* LWIP_NETIF_HOSTNAME */

/*****************************************************************************
 * The following is based on the skeleton Ethernet driver in LwIP            *
 *****************************************************************************/


/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include <lwip/netif/etharp.h>
#include "wwd_bus_protocol.h"

#ifdef LWIP_IGMP
#undef LWIP_IGMP
#define LWIP_IGMP 0
#endif

/* Define those to better describe your network interface. */
#define IFNAME0 'w'
#define IFNAME1 'l'

#define MULTICAST_IP_TO_MAC(ip)       { (uint8_t) 0x01,             \
                                        (uint8_t) 0x00,             \
                                        (uint8_t) 0x5e,             \
                                        (uint8_t) ((ip)[1] & 0x7F), \
                                        (uint8_t) (ip)[2],          \
                                        (uint8_t) (ip)[3]           \
                                      }


/* Forward declarations. */
#if LWIP_IGMP
static err_t lwip_igmp_mac_filter( struct netif *netif, ip_addr_t *group, u8_t action );
#endif

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void wwd_low_level_init( /*@partial@*/ struct netif *netif )
{
    /* Set MAC hardware address length */
    netif->hwaddr_len = (u8_t) ETHARP_HWADDR_LEN;

    /* Setup the physical address of this IP instance. */
    if ( wwd_wifi_get_mac_address( (wiced_mac_t*) ( netif->hwaddr ), (wwd_interface_t) netif->state ) != WWD_SUCCESS )
    {
        WPRINT_NETWORK_DEBUG(("Couldn't get MAC address\n"));
        return;
    }

    memcpy(&netif->d_mac, &netif->hwaddr, netif->hwaddr_len);

    /* Set Maximum Transfer Unit */
    netif->mtu = (u16_t) WICED_PAYLOAD_MTU;

    /* Set device capabilities. Don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
//    netif->flags = (u8_t) ( NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP );
	netif->flags = NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_BROADCAST | NETIF_FLAG_IGMP;

    /* Do whatever else is needed to initialize interface. */
#if LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
    netif_set_igmp_mac_filter(netif, lwip_igmp_mac_filter);
#endif
}

static void wwd_dump_bytes(void *data, uint32_t bytes)
{
	uint32_t i;
	uint8_t *buf;

	buf = (uint8_t *)data;
	printf("---------------------------\n");
	for (i = 0; i < bytes; i++) {
		if ((i % 16) == 8) printf(" ");
		else if(( i% 16) == 0) printf("\n");
		printf("%02X ", buf[i]);
	}
	printf("\n---------------------------\n");
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, /*@only@*/ struct pbuf *p)
{
#define SDPCM_DATA_HEADER_SIZE	22

	uint8_t *bptr;
	uint32_t tot_len;
	wiced_buffer_t new_pb;
	struct pbuf *tmp_p;
	uint32_t cur_size;

	if (((wiced_interface_t) netif->state == WICED_ETHERNET_INTERFACE)
			|| (wwd_wifi_is_ready_to_transceive((wwd_interface_t) netif->state) == WWD_SUCCESS)) {
		tot_len = p->tot_len + SDPCM_DATA_HEADER_SIZE;
		internal_host_buffer_get(&new_pb, WWD_NETWORK_TX, tot_len, 0);
		if (new_pb == NULL) {
			return (err_t) ERR_MEM;
		}

		new_pb->lwip_buf->flags = p->flags;
		new_pb->lwip_buf->ref = 0;

		bptr = host_buffer_get_current_piece_data_pointer(new_pb) + SDPCM_DATA_HEADER_SIZE;

		tmp_p = p;
		do {
			cur_size = tmp_p->len;
			memcpy(bptr, tmp_p->payload, cur_size);
			bptr += cur_size;
		} while ((tmp_p = tmp_p->next) != NULL);

		host_buffer_add_remove_at_front(&new_pb, SDPCM_DATA_HEADER_SIZE);

		pbuf_ref(new_pb->lwip_buf);

#if 0
		wwd_dump_bytes(new_pb->lwip_buf->payload, new_pb->lwip_buf->tot_len);
#endif

		wwd_network_send_ethernet_data(new_pb, (wwd_interface_t)netif->state);

		LINK_STATS_INC(link.xmit);

		return (err_t) ERR_OK;
	} else
		return (err_t) ERR_INPROGRESS;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param p : the incoming ethernet packet
 */
void host_network_process_ethernet_data( /*@only@*/ wiced_buffer_t buffer, wwd_interface_t interface )
{
	struct eth_hdr* ethernet_header;
	struct netif* tmp_netif;
	u8_t result;
	uint16_t ethertype;

	if (buffer == NULL)
		return;

	ethernet_header = (struct eth_hdr *) buffer->lwip_buf->payload;

#if 0
	wwd_dump_bytes(buffer->lwip_buf->payload, buffer->lwip_buf->tot_len);
#endif

	ethertype = htons(ethernet_header->type);

	if (ethertype == WICED_ETHERTYPE_8021Q) {
		/* Need to remove the 4 octet VLAN Tag, by moving src and dest addresses 4 octets to the right,
		 * and then read the actual ethertype. The VLAN ID and priority fields are currently ignored. */
		uint8_t temp_buffer[12];
		memcpy(temp_buffer, buffer->lwip_buf->payload, 12);
		memcpy(((uint8_t*) buffer->lwip_buf->payload) + 4, temp_buffer, 12);

		buffer->lwip_buf->payload = ((uint8_t*) buffer->lwip_buf->payload) + 4;
		buffer->lwip_buf->len = (u16_t) (buffer->lwip_buf->len - 4);

		ethernet_header = (struct eth_hdr *) buffer->lwip_buf->payload;
		ethertype = htons(ethernet_header->type);
	}

	switch (ethertype) {
	case WICED_ETHERTYPE_IPv4:
	case WICED_ETHERTYPE_ARP:
#if PPPOE_SUPPORT
		/* PPPoE packet? */
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
		/* Find the netif object matching the provided interface */
		for (tmp_netif = netif_list;
				(tmp_netif != NULL) && (tmp_netif->state != (void*) interface);
				tmp_netif = tmp_netif->next) {
		}

		if (tmp_netif == NULL) {
			/* Received a packet for a network interface is not initialised Cannot do anything with packet - just drop it. */
			result = pbuf_free(buffer->lwip_buf);
			LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
			buffer = NULL;
			printf("\t %s[%d]\n", __FUNCTION__, __LINE__);
			return;
		}

		/* Send to packet to tcpip_thread to process */
		if (tcpip_input(buffer->lwip_buf, tmp_netif) != ERR_OK) {
			LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));

			/* Stop lint warning - packet has not been released in this case *//*@-usereleased@*/
			result = pbuf_free(buffer->lwip_buf);
			LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
			buffer = NULL;
		}
		break;

#ifdef ADD_LWIP_EAPOL_SUPPORT
		case WICED_ETHERTYPE_EAPOL:
		wwd_eapol_receive_eapol_packet( buffer, interface );
		break;
#endif
	default:
		result = pbuf_free(buffer->lwip_buf);
		LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
		buffer = NULL;
		break;
	}

    mem_free(buffer);
    buffer = NULL;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t wwd_ethernetif_init( /*@partial@*/ struct netif *netif )
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    if ( lwip_hostname_ptr != NULL )
    {
        netif_set_hostname(netif, lwip_hostname_ptr);
    }
    else
    {
        netif_set_hostname(netif, (char *)LWIP_DEFAULT_CLIENT_OBJECT_NAME);
    }
#endif /* LWIP_NETIF_HOSTNAME */

    /* Verify the netif is a valid interface */
    if ( (wwd_interface_t) netif->state > WWD_ETHERNET_INTERFACE )
    {
        return ERR_ARG;
    }

#if 0
    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);
#endif


    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
	snprintf(netif->d_ifname, 6, "wl%d", netif->num);

    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...)
     */
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

    /* Initialize the hardware */
    wwd_low_level_init( netif );

    return ERR_OK;
}

#if LWIP_IGMP
/**
 * Interface between LwIP IGMP MAC filter and WICED MAC filter
 */
static err_t lwip_igmp_mac_filter( struct netif *netif, ip_addr_t *group, u8_t action )
{
    wiced_mac_t mac = { MULTICAST_IP_TO_MAC((uint8_t*)group) };
    /*@-noeffect@*/
    UNUSED_PARAMETER(netif);
    /*@+noeffect@*/

    switch ( action )
    {
        case IGMP_ADD_MAC_FILTER:
            if ( wwd_wifi_register_multicast_address( &mac ) != WWD_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        case IGMP_DEL_MAC_FILTER:
            if ( wwd_wifi_unregister_multicast_address( &mac ) != WWD_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        default:
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

/********************************** End of file ******************************************/
