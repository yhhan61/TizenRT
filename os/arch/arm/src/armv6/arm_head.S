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
 * arch/arm/src/arm6/arm_head.S
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

#include <tinyara/config.h>

#include "arm.h"
#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include <arch/board/board.h>

/**********************************************************************************
 * Configuration
 **********************************************************************************/

#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE


/****************************************************************************
 * Global Symbols
 ****************************************************************************/

/* Imported symbols */

	.global	arm_boot			/* Branch to continue initialization in C */

	.global	_sbss				/* Start of .bss in RAM */
	.global	_ebss				/* End+1 of .bss in RAM */

/* Exported symbols */

	.global	__start				/* Power-up/Reset entry point */
	.global	arm_data_initialize	/* Perform C data initialization */
	.global	g_idle_topstack		/* Top of the initial/IDLE stack */

	.syntax	unified

/***************************************************************************
 * .text
 ***************************************************************************/

	.text
	.section .text.startup, "ax"

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */

	.type	__start, #function
__start:

	/* Make sure that we are in SVC mode with IRQs and FIQs disabled */

	mov		r0, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, r0

	/* Set up the stack pointer and clear the frame pointer. */
	adr		r0, .Linitparms
	ldmia		r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */
	mov		fp, #0

.Lbssinit:	
	cmp		r0, r1					/* Clear up to _bss_end_ */
	strcc		fp, [r0],#4
	bcc		.Lbssinit
	
	/* Perform early C-level, platform-specific initialization.  Logic
	 * within arm_boot() must configure SDRAM and call arm_data_initialize()
	 * if CONFIG_ARMV6_MEMINIT=y.
	 *
	 * This function does not return.  It must give control to os_start()
	 * at the completion of its initialization.
	 *
	 * Why not just call arm_boot() and branch to os_start() when it returns?
	 * If the stack pointer initialized above lies in SDRAM, then that may
	 * not be possible.  Also, in the special case of the TMS570, it may
	 * perform a destructive test, losing the pushed content of the stack.
	 */
	bl		__early_init

	b		arm_boot

	/* copy exception vector table to address 0x0 */
	.align		4
.global __early_init
__early_init:
        mov r0,#0x8000
        mov r1,#0x0000
        ldmia r0!,{r2,r3,r4,r5,r6,r7,r8,r9}
        stmia r1!,{r2,r3,r4,r5,r6,r7,r8,r9}
        ldmia r0!,{r2,r3,r4,r5,r6,r7,r8,r9}
        stmia r1!,{r2,r3,r4,r5,r6,r7,r8,r9}
        bx      lr
	
/*
 * Data memory barrier
 * No memory access after the DMB can run until all memory accesses before it
 * have completed
 */
.global dmb
dmb:
    mov     r0, #0
    mcr     p15, #0, r0, c7, c10, #5
    mov     pc, lr

	/* .text Data:
	 *
	 *   _sbss is the start of the BSS region (see linker script)
	 *   _ebss is the end of the BSS regsion (see linker script)
	 *
	 * Typical Configuration:
	 *   The idle task stack starts at the end of BSS and is of size
	 *   CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there
	 * until the end of memory.  See g_idle_topstack below.
	 */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4

/***************************************************************************
 * .rodata
 ***************************************************************************/

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Linitparms
	 * above.
	 */

	.data
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_idle_topstack, .-g_idle_topstack
	.end
