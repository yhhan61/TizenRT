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

#include <stdio.h>
#include <fcntl.h>
#include <stdbool.h>
#include <net/if.h>
#include <net/lwip/opt.h>
#include <net/lwip/netif.h>
#include <net/lwip/tcpip.h>
#include <tinyara/kmalloc.h>
#include "wwd_management.h"
#include "wwd_network.h"
#include "wwd_constants.h"
#include "wwd_structures.h"
#include "wwd_internal.h"
#include "wwd_thread_internal.h"
#include "wwd_wifi.h"
#include "RTOS/wwd_rtos_interface.h"

static struct netif *cyw43438_wlan_netif;

struct netif* get_cyw43438_wlan_netif(void)
{
	return cyw43438_wlan_netif;
}

static int __up_wlan_init(void)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;

	cyw43438_wlan_netif = (struct netif *)kmm_zalloc(sizeof(struct netif));
	if (cyw43438_wlan_netif == NULL)
		return -1;

	ipaddr.addr = inet_addr("192.168.99.155");
	netmask.addr = inet_addr("255.255.255.0");
	gw.addr = inet_addr("192.168.99.1");

	netif_add(cyw43438_wlan_netif, &ipaddr, &netmask, &gw, WWD_STA_INTERFACE, wwd_ethernetif_init, tcpip_input);
	netif_set_down(cyw43438_wlan_netif);
	netif_set_up(cyw43438_wlan_netif);
	netif_set_default(cyw43438_wlan_netif);

#if 0 /* Test code */
	printf("ifname = %c%c, %s, %02X:%02X:%02X:%02X:%02X:%02X\n",
    		cyw43438_wlan_netif->name[0],
			cyw43438_wlan_netif->name[1],
			cyw43438_wlan_netif->d_ifname,
			cyw43438_wlan_netif->d_mac.ether_addr_octet[0],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[1],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[2],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[3],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[4],
			cyw43438_wlan_netif->d_mac.ether_addr_octet[5]);
#endif
    cyw43438_wlan_netif->d_flags |= IFF_UP;

    return 0;
}

void rpi0w_brcm_wlan_driver_initialize(void)
{
extern int bring_up_wpa_supplicant(void);

	if (wwd_buffer_init() != WWD_SUCCESS)
		return;

	if (wwd_management_wifi_on(WICED_COUNTRY_KOREA_REPUBLIC_OF) == WWD_SUCCESS)
		printf("Success Wifi-ON CYW43438\n");
	else {
		printf("Failed Wifi-ON CYW43438\n");
		return;
	}

	WWD_WLAN_KEEP_AWAKE();
	if (__up_wlan_init())
		return;

	sleep(1);
	bring_up_wpa_supplicant();
}
