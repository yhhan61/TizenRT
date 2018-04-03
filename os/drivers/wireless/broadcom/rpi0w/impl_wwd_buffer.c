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

#include "network/wwd_buffer_interface.h"
#include "network/wwd_network_constants.h"
#include "platform/wwd_bus_interface.h"
#include "lwip/netbuf.h"
#include "lwip/memp.h"
#include <string.h>
#include "wwd_assert.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wiced_utilities.h"
#include <tinyara/kmalloc.h>

/**
 * Initialize the packet buffer interface
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Some implementations of the packet buffer interface may need additional
 * information for initialization, especially the location of packet buffer
 * pool(s). These can be passed via the 'native_arg' parameter.
 *
 * @param native_arg  An implementation specific argument
 *
 * @return WWD_SUCCESS = Success, Error code = Failure
 */
wwd_result_t wwd_buffer_init( /*@null@*/ void* native_arg )
{
	return WWD_SUCCESS;
}

/**
 * Deinitialize the packet buffer interface
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 *
 * @return WWD_SUCCESS = Success, Error code = Failure
 */

wwd_result_t wwd_buffer_deinit(void)
{
	return WWD_SUCCESS;
}

wwd_result_t internal_host_buffer_get(wiced_buffer_t *buffer, wwd_buffer_dir_t direction, unsigned short size, unsigned long timeout_ms )
{
	UNUSED_PARAMETER(direction);
	UNUSED_PARAMETER(timeout_ms);

	*buffer = NULL;

	if (size > ((uint16_t)WICED_LINK_MTU) || (size == 0))
		return WWD_MALLOC_FAILURE;


    *buffer = (wiced_buffer_t) mem_malloc(sizeof (struct wiced_buffer));
    if (*buffer == NULL)
        return WWD_MALLOC_FAILURE;

    (*buffer)->flags = WICED_BUFFER_lwIP_NEED_FREE;
    if (direction == WWD_NETWORK_TX)
        (*buffer)->lwip_buf = pbuf_alloc(PBUF_RAW, size, PBUF_RAM);
    else
        (*buffer)->lwip_buf = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);

    (*buffer)->org_paylaod = (*buffer)->lwip_buf->payload;

    return WWD_SUCCESS;
}

/**
 * @brief Allocates a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Attempts to allocate a packet buffer of the size requested. It can do this
 * by allocating a pre-existing packet from a pool, using a static buffer,
 * or by dynamically allocating memory. The method of allocation does not
 * concern WICED, however it must match the way the network stack expects packet
 * buffers to be allocated.
 *
 * @param buffer     A pointer which receives the allocated packet buffer handle
 * @param direction : Indicates transmit/receive direction that the packet buffer is
 *                    used for. This may be needed if tx/rx pools are separate.
 * @param size      : The number of bytes to allocate.
 * @param wait      : Whether to wait for a packet buffer to be available
 *
 * @return WWD_SUCCESS = Success, Error code = Failure
 *
 */
wwd_result_t host_buffer_get( /*@special@*/ /*@out@*/ wiced_buffer_t* buffer, wwd_buffer_dir_t direction, unsigned short size, wiced_bool_t wait ) /*@allocates *buffer@*/  /*@defines **buffer@*/
{
	unsigned long wait_option;

	wait_option = (wait == WICED_TRUE) ? (unsigned long) WICED_NEVER_TIMEOUT : 0;

	return internal_host_buffer_get(buffer, direction, size, wait_option);
}

/**
 * Releases a packet buffer
 *
 * Implemented in the Wiced buffer interface, which will be specific to the
 * buffering scheme in use.
 * This function is used by WICED to indicate that it no longer requires
 * a packet buffer. The buffer can then be released back into a pool for
 * reuse, or the dynamically allocated memory can be freed, according to
 * how the packet was allocated.
 * Returns void since WICED cannot do anything about failures
 *
 * @param buffer    : the handle of the packet buffer to be released
 * @param direction : indicates transmit/receive direction that the packet buffer has
 *                    been used for. This might be needed if tx/rx pools are separate.
 *
 */
void host_buffer_release( /*@only@*/wiced_buffer_t buffer,	wwd_buffer_dir_t direction)
{
	UNUSED_PARAMETER(direction);

	if (buffer) {
		if (buffer->lwip_buf) {
			buffer->lwip_buf->payload = buffer->org_paylaod;
			pbuf_free(buffer->lwip_buf);
		}

		mem_free(buffer);
		buffer = NULL;
	}
}

/**
 * Retrieves the current pointer of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, this function allows WICED to get
 * the current 'front' location pointer.
 *
 * @param buffer : The handle of the packet buffer whose pointer is to be retrieved
 *
 * @return The packet buffer's current pointer.
 */
/*@exposed@*/ uint8_t* host_buffer_get_current_piece_data_pointer( /*@temp@*/ wiced_buffer_t buffer )
{
	wiced_assert("Error: Invalid buffer\n", buffer != NULL);
	return (uint8_t*) buffer->lwip_buf->payload;
}

/**
 * Retrieves the size of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, the memory block used to contain a packet buffer
 * will often be larger than the current size of the packet buffer data.
 * This function allows WICED to retrieve the current size of a packet buffer's data.
 *
 * @param buffer : The handle of the packet buffer whose size is to be retrieved
 *
 * @return The size of the packet buffer.
 */
uint16_t host_buffer_get_current_piece_size( /*@temp@*/ wiced_buffer_t buffer )
{
	wiced_assert("Error: Invalid buffer\n", buffer != NULL);
	return (uint16_t) buffer->lwip_buf->len;
}

/**
 * Retrieves the next piece of a set of daisy chained packet buffers
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Some buffering schemes allow buffers to be daisy chained into linked lists.
 * This allows more flexibility with packet buffers and avoids memory copies.
 * It does however require scatter-gather DMA for the hardware bus.
 * This function retrieves the next buffer in a daisy chain of packet buffers.
 *
 * @param buffer : The handle of the packet buffer whose next buffer is to be retrieved
 *
 * @return The handle of the next buffer, or NULL if there is none.
 */
/*@exposed@*/ /*@dependent@*/ /*@null@*/ wiced_buffer_t host_buffer_get_next_piece( /*@dependent@*/ wiced_buffer_t buffer )
{
	wiced_assert("Error: Invalid buffer\n", buffer != NULL);
	return buffer->next;
}

/**
 * Moves the current pointer of a packet buffer
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * Since packet buffers usually need to be created with space at the
 * front for additional headers, this function allows WICED to move
 * the current 'front' location pointer so that it has space to add headers
 * to transmit packets, and so that the network stack does not see the
 * internal WICED headers on received packets.
 *
 * @param buffer    : A pointer to the handle of the current packet buffer
 *                    for which the current pointer will be moved. On return
 *                    this may contain a pointer to a newly allocated packet
 *                    buffer which has been daisy chained to the front of the
 *                    given one. This would be the case if the given packet buffer
 *                    didn't have enough space at the front.
 * @param add_remove_amount : This is the number of bytes to move the current pointer
 *                            of the packet buffer - a negative value increases the space
 *                            for headers at the front of the packet, a positive value
 *                            decreases the space.
 * @return WWD_SUCCESS = Success, Error code = Failure
 */
wwd_result_t host_buffer_add_remove_at_front( wiced_buffer_t * buffer, int32_t add_remove_amount )
{
	wiced_assert("Error: Invalid buffer\n", buffer != NULL);
	if ((u8_t) 0 != pbuf_header((*buffer)->lwip_buf, (s16_t) (-add_remove_amount))) {
		WPRINT_NETWORK_DEBUG(
				("Failed to move pointer - usually because not enough space at front of buffer\n"));
		return WWD_BUFFER_POINTER_MOVE_ERROR;
	}

	return WWD_SUCCESS;
}

/**
 * Sets the current size of a Wiced packet
 *
 * Implemented in the WICED buffer interface which is specific to the
 * buffering scheme in use.
 * This function sets the current length of a WICED packet buffer
 *
 * @param buffer : The packet to be modified
 * @param size   : The new size of the packet buffer
 *
 * @return WWD_SUCCESS = Success, Error code = Failure
 */
wwd_result_t host_buffer_set_size( /*@temp@*/ wiced_buffer_t buffer, unsigned short size )
{
	if (size > (unsigned short) WICED_LINK_MTU) {
		WPRINT_NETWORK_ERROR(
				("Attempt to set a length larger than the MTU of the link\n"));
		return WWD_BUFFER_SIZE_SET_ERROR;
	}

	buffer->lwip_buf->tot_len = size;
	buffer->lwip_buf->len = size;

	return WWD_SUCCESS;
}
