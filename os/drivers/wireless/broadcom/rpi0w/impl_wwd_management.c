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

#include "wwd_structures.h"

 /**
  * The host MCU can provide a MAC address to the Wi-Fi driver (rather than
  * the driver using the MAC in NVRAM or OTP). To use this functionality,
  * add a global define (MAC_ADDRESS_SET_BY_HOST) in the application makefile.
  * Further information is available in the generated_mac_address.txt file
  * that is created during the application build process.
  * @param mac : A wiced_mac_t pointer to the Wi-Fi MAC address
  */
wwd_result_t host_platform_get_mac_address(wiced_mac_t* mac)
{
	mac->octet[0] = 0xB8;
	mac->octet[1] = 0x27;
	mac->octet[2] = 0xEB;
	mac->octet[3] = 0xA1;
	mac->octet[4] = 0xB2;
	mac->octet[5] = 0xC3;

	return WWD_SUCCESS;
}
