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

#include <stdint.h>
#include "wwd_constants.h"
#include "wwd_resource_interface.h"
#include "brcmfmac43430-sdio.txt.h"
#include "brcmfmac43430-sdio.bin.h"

/*
 * This function is used when WWD_DIRECT_RESOURCES is defined
 */
wwd_result_t host_platform_resource_size(wwd_resource_t resource, uint32_t* size_out)
{
	if (resource == WWD_RESOURCE_WLAN_FIRMWARE)
		*size_out = (uint32_t)brcmfmac43430_sdio_bin.length;
	else if (resource == WWD_RESOURCE_WLAN_NVRAM)
		*size_out = (uint32_t)brcmfmac43430_sdio_txt.length;

	return WWD_SUCCESS;
}

/*
 * This function is used when WWD_DIRECT_RESOURCES is defined
 */
wwd_result_t host_platform_resource_read_direct(wwd_resource_t resource, const void** ptr_out)
{
	if (resource == WWD_RESOURCE_WLAN_FIRMWARE)
		*ptr_out = (const void*)brcmfmac43430_sdio_bin.data;
	else if (resource == WWD_RESOURCE_WLAN_NVRAM)
		*ptr_out = (const void*)brcmfmac43430_sdio_txt.data;

	return WWD_SUCCESS;
}
