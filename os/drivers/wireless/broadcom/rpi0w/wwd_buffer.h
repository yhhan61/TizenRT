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

#ifndef INCLUDED_WWD_BUFFER_H_
#define INCLUDED_WWD_BUFFER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define WICED_BUFFER_lwIP_NEED_FREE	(1)
#define WICED_BUFFER_EMPTY_FLAGFS		(0)
typedef struct wiced_buffer {
	struct pbuf *lwip_buf;

	void* org_paylaod;
	uint32_t flags;
	void* next;
} *wiced_buffer_t;

typedef void wiced_buffer_fifo_t;


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ifndef INCLUDED_WWD_BUFFER_H_ */
