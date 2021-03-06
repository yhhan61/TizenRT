/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
/****************************************************************************
 * arch/arm/src/armv6/arm_saveusercontext.S
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/irq.h>
#include "up_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	up_mpucontextsave
	.globl	up_saveusercontext

	.syntax	unified

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: up_saveusercontext
 ****************************************************************************/

	.globl	up_saveusercontext
	.type	up_saveusercontext, function

up_saveusercontext:

	/* On entry, a1 (r0) holds address of struct xcptcontext */

	/* Make sure that the return value will be non-zero (the value of the
	 * other volatile registers don't matter -- r1-r3, ip).  This function
	 * is called through the normal C calling conventions and the values of
	 * these registers cannot be assumed at the point of setjmp return.
	 */
	mov		ip, #1
	str		ip, [r0, #(4*REG_R0)]

	/* Save the volatile registers (plus r12 which really
	 * doesn't need to be saved)
	 */

	add		r1, r0, #(4*REG_R4)
	stmia	r1, {r4-r14}

	/* Save the current cpsr */

	mrs		r2, cpsr		/* R3 = CPSR value */
	add		r1, r0, #(4*REG_CPSR)
	str		r2, [r1]

	/* Save the return address as the PC so that we return to the exit from
	 * this function.
	 */

	add		r1, r0, #(4*REG_PC)
	str		lr, [r1]

	/* Return 0 now indicating that this return is not a context switch */
	mov		r0, #0			/* Return value == 0 */
	mov		pc, lr			/* Return */
	.size	up_saveusercontext, . - up_saveusercontext
	.end
