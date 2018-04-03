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

#include <semaphore.h>
#include <errno.h>
#include <time.h>
#include <tinyara/irq.h>
#include "wwd_rtos_interface.h"

/**
 * Create a Semaphore
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to create a semaphore
 *
 * @param semaphore         : Pointer to the semaphore handle to be initialized
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_rtos_init_semaphore(host_semaphore_type_t* semaphore)
{
	int err;

	err = sem_init(semaphore, 0, 0);
	if (err != OK)
		return WWD_SEMAPHORE_ERROR;

	return WWD_SUCCESS;
}

/**
 * Get a semaphore
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to get a semaphore.
 * - If the semaphore value is greater than zero, the sempahore value is decremented and the function returns immediately.
 * - If the semaphore value is zero, the current thread is put on a queue of threads waiting on the
 *   semaphore, and sleeps until another thread sets the semaphore and causes it to wake. Upon waking, the
 *   semaphore is decremented and the function returns.
 *
 * @note : This function must not be called from an interrupt context as it may block.
 *
 * @param semaphore   : Pointer to the semaphore handle
 * @param timeout_ms  : Maximum number of milliseconds to wait while attempting to get
 *                      the semaphore. Use the NEVER_TIMEOUT constant to wait forever.
 * @param will_set_in_isr : True if the semaphore will be set in an ISR. Currently only used for NoOS/NoNS
 *
 * @return wwd_result_t : WWD_SUCCESS if semaphore was successfully acquired
 *                     : WICED_TIMEOUT if semaphore was not acquired before timeout_ms period
 */
wwd_result_t host_rtos_get_semaphore(
		host_semaphore_type_t* semaphore,
		uint32_t timeout_ms,
		wiced_bool_t will_set_in_isr)
{
	struct timespec abstime;
	time_t sec;
	int err;

	UNUSED_PARAMETER(will_set_in_isr);

	if (timeout_ms == NEVER_TIMEOUT)
		return (sem_wait(semaphore) == OK) ? WWD_SUCCESS : WWD_SEMAPHORE_ERROR;

	if (timeout_ms == 0) {
		err = sem_trywait(semaphore);

		if (err != ERROR)
			return WWD_SUCCESS;
		else if (errno == EAGAIN)
			return WWD_TIMEOUT;
		else
			return WWD_SEMAPHORE_ERROR;
    }

	err = clock_gettime(CLOCK_REALTIME, &abstime);
	if (err != ERROR) {
		sec = timeout_ms / 1000;
		abstime.tv_sec += sec;
		abstime.tv_nsec += (long)(timeout_ms - (sec * 1000)) * 1000000;
		if (abstime.tv_nsec > 1000 * 1000 * 1000) {
			abstime.tv_sec++;
			abstime.tv_nsec -= 1000 * 1000 * 1000;
		}

		err = sem_timedwait( semaphore, &abstime );
	}

	if (err != ERROR)
		return WWD_SUCCESS;
	else if ((errno == ETIMEDOUT) || (errno == EINTR))
		return WWD_TIMEOUT;

	return WWD_SEMAPHORE_ERROR;
}

/**
 * Set a semaphore
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to set a semaphore.
 * The value of the semaphore is incremented, and if there are any threads waiting on the semaphore,
 * then the first waiting thread is woken.
 *
 * Some RTOS implementations require different processing when setting a semaphore from within an ISR.
 * A parameter is provided to allow this.
 *
 * @param semaphore       : Pointer to the semaphore handle
 * @param called_from_ISR : Value of WICED_TRUE indicates calling from interrupt context
 *                          Value of WICED_FALSE indicates calling from normal thread context
 *
 * @return wwd_result_t : WWD_SUCCESS if semaphore was successfully set
 *                        : Error code if an error occurred
 *
 */
wwd_result_t host_rtos_set_semaphore(
		host_semaphore_type_t* semaphore,
		wiced_bool_t called_from_ISR )
{
	int err;
	int sem_value;

	UNUSED_PARAMETER(called_from_ISR);

	err = sem_getvalue(semaphore, &sem_value);
	if (err != OK)
		return WWD_SEMAPHORE_ERROR;

	if (sem_value == 1)
		return WWD_SUCCESS;

	err = sem_post(semaphore);
	if (err != OK)
		return WWD_SEMAPHORE_ERROR;

	return WWD_SUCCESS;
}

/**
 * Deletes a semaphore
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to delete a semaphore.
 *
 * @param semaphore         : Pointer to the semaphore handle
 *
 * @return wwd_result_t : WWD_SUCCESS if semaphore was successfully deleted
 *                        : Error code if an error occurred
 */
wwd_result_t host_rtos_deinit_semaphore(host_semaphore_type_t* semaphore)
{
	int err;

	err = sem_destroy(semaphore);
	if (err != OK)
		return WWD_SEMAPHORE_ERROR;

	return WWD_SUCCESS;
}
