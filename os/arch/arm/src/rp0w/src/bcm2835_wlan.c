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
#include <tinyara/config.h>

#include <stdio.h>
#include <pthread.h>
#include <fcntl.h>
#include <stdbool.h>
#include <debug.h>

#include <net/if.h>
#include <net/lwip/opt.h>
#include <net/lwip/netif.h>
#include <net/lwip/tcpip.h>

#include <tinyara/configdata.h>

#include "up_arch.h"
#include "rp0w.h"

int up_wlan_get_country(char *alpha2)
{
	return cyw43438_get_country(NULL, alpha2);
}

int up_wlan_set_country(char *alpha2)
{
	return cyw43438_set_country(NULL, alpha2);
}

int up_wlan_get_txpower(void)
{
	return (int)cyw43438_get_tx_power(NULL);
}

int up_wlan_set_txpower(uint8_t *dbm)
{
	return cyw43438_set_tx_power(NULL, *dbm);
}

int bring_up_wpa_supplicant(void)
{
extern	int wpa_supplicant_main(int argc, char *argv[]);
	pid_t task;
	static char *argv[5];

	argv[0] = "-B";
	argv[1] = "-t";
	argv[2] = "-i wl1";
	argv[3] = "-Cudp";
	argv[4] = NULL;

	task = task_create("WPA Supplicant",
						100,
						4096,
						wpa_supplicant_main,
						argv);

	sleep(1);

	if (task < 0) {
		printf("ERROR: Failed wpa_supplicant_task\n");
		return -1;;
	}

	return 0;
}
