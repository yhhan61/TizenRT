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

#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include "wwd_rtos_interface.h"

static pthread_addr_t host_rtos_thread_wrapper(pthread_addr_t arg)
{
	host_thread_type_t *thread;
	uint32_t  param1;

	thread = (host_thread_type_t*)arg;
	param1 = thread->arg;

	thread->entry_function(param1);

	return NULL;
}

/******************************************************
 *             Function declarations
 ******************************************************/

/* Thread management functions */

/**
 * Create a thread
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to create a new thread and start it running.
 *
 * @param thread         : pointer to a variable which will receive the new thread handle
 * @param entry_function : function pointer which points to the main function for the new thread
 * @param name           : a string thread name used for a debugger
 * @param stack_size     : the size of the thread stack in bytes
 * @param priority       : the priority of the thread
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_rtos_create_thread(
		/*@out@*/ host_thread_type_t* thread,
		void(*entry_function)( wwd_thread_arg_t arg ),
		const char* name,
		/*@null@*/ void* stack,
		uint32_t stack_size,
		uint32_t priority ) /*@modifies *thread@*/
{
	return host_rtos_create_thread_with_arg(thread, entry_function, name, stack,
			stack_size, priority, 0);
}

/**
 * Create a thread with specific thread argument
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 *
 * @param thread         : pointer to a variable which will receive the new thread handle
 * @param entry_function : function pointer which points to the main function for the new thread
 * @param name           : a string thread name used for a debugger
 * @param stack_size     : the size of the thread stack in bytes
 * @param priority       : the priority of the thread
 * @param arg            : the argument to pass to the new thread
 *
 * @return WWD result code
 */
wwd_result_t host_rtos_create_thread_with_arg(
		/*@out@*/ host_thread_type_t* thread,
		void(*entry_function)(wwd_thread_arg_t arg),
		const char* name,
		/*@null@*/ void* stack,
		uint32_t stack_size,
		uint32_t priority,
		wwd_thread_arg_t arg )
{
	wwd_result_t result;
	struct sched_param sched_param;
	int err;
	pthread_attr_t attr;

	UNUSED_PARAMETER(stack);
	result = WWD_THREAD_CREATE_FAILED;
	sched_param.sched_priority = priority;

	err = pthread_attr_init(&attr);
	if (err != 0)
		return result;

	err = pthread_attr_setstacksize(&attr, (long) stack_size);
	if (err != 0)
		goto error_exit;

	err = pthread_attr_setschedparam(&attr, &sched_param);
	if (err != 0)
		goto error_exit;

	err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (err != 0)
		goto error_exit;

	err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (err != 0)
		goto error_exit;

	thread->entry_function = entry_function;
	thread->arg = arg;
	err = pthread_create(&thread->handle, &attr, host_rtos_thread_wrapper, thread);
	if (err != 0)
		goto error_exit;

	pthread_setname_np(thread->handle, name);

	result = WWD_SUCCESS;

error_exit:
	pthread_attr_destroy(&attr);

	return result;
}

/**
 * Note: different RTOS have different parameters for creating threads.
 * Use this function carefully if portability is important.
 * Create a thread with RTOS specific thread argument (E.g. specify time-slicing behavior)
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 *
 * @param thread         : pointer to a variable which will receive the new thread handle
 * @param entry_function : function pointer which points to the main function for the new thread
 * @param name           : a string thread name used for a debugger
 * @param stack_size     : the size of the thread stack in bytes
 * @param priority       : the priority of the thread
 * @param arg            : the argument to pass to the new thread
 * @param config        : os specific thread configuration
 * @return WWD result code
 */
wwd_result_t host_rtos_create_configed_thread(
		/*@out@*/ host_thread_type_t* thread,
		void(*entry_function)(uint32_t),
		const char* name,
		/*@null@*/ void* stack,
		uint32_t stack_size,
		uint32_t priority,
		host_rtos_thread_config_type_t *config)
{
    UNUSED_PARAMETER(config);

    return host_rtos_create_thread( thread, entry_function, name, stack, stack_size, priority );
}

/**
 * Exit a thread
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * WICED uses this function to exit the current thread just before its main
 * function would otherwise return. Some RTOSs allow threads to exit by simply returning
 * from their main function. If this is the case, then the implementation of
 * this function should be empty.
 *
 * @param thread         : Pointer to the current thread handle
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_rtos_finish_thread( host_thread_type_t* thread ) /*@modifies *thread@*/
{
	int err;

	if ((thread == NULL) || (pthread_equal(thread->handle, pthread_self()) != 0 ))
		pthread_exit( NULL );

	err = pthread_cancel( thread->handle ); /* Newly created thread has PTHREAD_CANCEL_ENABLE state */
	if ( err != 0 )
		return WWD_THREAD_FINISH_FAIL;

	return WWD_SUCCESS;
}

/**
 * Deletes a terminated thread
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 * Some RTOS implementations require that another thread deletes any terminated thread
 * If RTOS does not require this, leave empty
 *
 * @param thread         : handle of the terminated thread to delete
 *
 * @returns WWD_SUCCESS on success, Error code otherwise
 */
wwd_result_t host_rtos_delete_terminated_thread( host_thread_type_t* thread )
{
    int err;

    if (thread == NULL)
    	return WWD_THREAD_FINISH_FAIL;

    if (pthread_equal( thread->handle, pthread_self() ) != 0)
    	return WWD_THREAD_FINISH_FAIL;

    /* Thread is not detached, so free join structure otherwise memory leak. */
	err = pthread_join( thread->handle, NULL );
	if ((err != 0) && (err != ESRCH))
		return WWD_THREAD_FINISH_FAIL;

	return WWD_SUCCESS;
}

/**
 * Waits for a thread to complete
 *
 * Implemented in the WICED RTOS interface which is specific to the
 * RTOS in use.
 *
 * @param thread         : handle of the thread to wait for
 *
 * @returns WWD_SUCCESS on success, Error code otherwise
 */
wwd_result_t host_rtos_join_thread(host_thread_type_t* thread)
{
	int err;

	err = pthread_join(thread->handle, NULL);
	if ( err != 0 )
		return WWD_THREAD_FINISH_FAIL;

	return WWD_SUCCESS;
}



