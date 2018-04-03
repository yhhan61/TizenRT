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

#include <time.h>
#include <errno.h>
#include "wwd_rtos_interface.h"

/* Time management functions */

/**
 * Gets time in milliseconds since RTOS start
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to retrieve the current time.
 *
 * @note: Since this is only 32 bits, it will roll over every 49 days, 17 hours, 2 mins, 47.296 seconds
 *
 * @returns Time in milliseconds since the RTOS started.
 */
wwd_time_t host_rtos_get_time( void )
{
	struct timespec tp;

	clock_gettime(CONFIG_CLOCK_MONOTONIC, &tp);


	return ((tp.tv_sec * 1000) + (tp.tv_nsec / 1000000));
}

/**
 * Delay for a number of milliseconds
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to delay processing
 * Processing of this function depends on the minimum sleep
 * time resolution of the RTOS.
 * The current thread should sleep for the longest period possible which
 * is less than the delay required, then makes up the difference
 * with a tight loop
 *
 * @return wwd_result_t : WWD_SUCCESS if delay was successful
 *                        : Error code if an error occurred
 *
 */
wwd_result_t host_rtos_delay_milliseconds(uint32_t num_ms)
{
	usleep(num_ms * 1000);

	return WWD_SUCCESS;
}
