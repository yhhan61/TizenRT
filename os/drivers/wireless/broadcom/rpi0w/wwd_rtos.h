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

#ifndef DRIVERS_WIRELESS_BROADCOM_RPI0W_WWD_RTOS_H_
#define DRIVERS_WIRELESS_BROADCOM_RPI0W_WWD_RTOS_H_

#include <semaphore.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/
#define configMAX_PRIORITIES 				(3) // HSLEE
#define RTOS_HIGHER_PRIORTIY_THAN(x)     (x < RTOS_HIGHEST_PRIORITY ? x+1 : RTOS_HIGHEST_PRIORITY)
#define RTOS_LOWER_PRIORTIY_THAN(x)      (x > RTOS_LOWEST_PRIORITY ? x-1 : RTOS_LOWEST_PRIORITY)
#define RTOS_LOWEST_PRIORITY             (0)
#define RTOS_HIGHEST_PRIORITY            (configMAX_PRIORITIES-1)
#define RTOS_DEFAULT_THREAD_PRIORITY     (1)

#define RTOS_USE_DYNAMIC_THREAD_STACK

#define WWD_THREAD_STACK_SIZE        (544 + 4096 + 1400)

/******************************************************
 *             Structures
 ******************************************************/

typedef sem_t    host_semaphore_type_t;

typedef struct
{
    uint32_t              message_num;
    uint32_t              message_size;
    uint8_t*              buffer;
    uint32_t              push_pos;
    uint32_t              pop_pos;
    host_semaphore_type_t push_sem;
    host_semaphore_type_t pop_sem;
    uint32_t              occupancy;
} host_queue_type_t;

typedef struct
{
    pthread_t handle;
    uint32_t  arg;
    void(*entry_function)(uint32_t);
} host_thread_type_t;

typedef struct
{
    uint8_t info;    /* not supported yet */
} host_rtos_thread_config_type_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* DRIVERS_WIRELESS_BROADCOM_RPI0W_WWD_RTOS_H_ */
