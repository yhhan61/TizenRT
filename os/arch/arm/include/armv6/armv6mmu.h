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
 */

/*
 * armv6mmu.h
 *
 */
#ifndef _armv6mmu_h
#define _armv6mmu_h

#include <arch/armv6/macros.h>

// NOTE: VMSAv6 and VMSAv6 are very similar for our purpose.
//   The definitions in this file are used for both.

// Sizes
#define SMALL_PAGE_SIZE		0x1000
#define LARGE_PAGE_SIZE		0x10000
#define SECTION_SIZE		0x100000
#define SUPER_SECTION_SIZE	0x1000000

// Access permissions (AP[1:0])
#define AP_NO_ACCESS		0
#define AP_SYSTEM_ACCESS	1
#define AP_USER_RO_ACCESS	2
#define AP_ALL_ACCESS		3

// Access permissions extended (APX)
#define APX_RW_ACCESS		0
#define APX_RO_ACCESS		1

// Domains
#define DOMAIN_NO_ACCESS	0
#define DOMAIN_CLIENT		1
#define DOMAIN_MANAGER		3

typedef struct MMU_L1_SECTION_DESC {	// subpages disabled
	u32 Value10:2,				// set to 2
	 BBit:1,					// bufferable bit
	 CBit:1,					// cacheable bit
	 XNBit:1,					// execute never bit
	 Domain:4,					// 0..15
	 IMPBit:1,					// implementation defined bit
	 AP:2,						// access permissions
	 TEX:3,						// memory type extension field
	 APXBit:1,					// access permissions extended bit
	 SBit:1,					// shareable bit
	 NGBit:1,					// non-global bit
	 Value0:1,					// set to 0
	 SBZ:1,						// should be 0
	 Base:12;					// base address [31:20]
} PACKED MMU_L1_SECTION_DESC;

#define MMU_L1_SECTIONBASE(addr)	(((addr) >> 20) & 0xFFF)
#define MMU_L1_SECTIONPTR(base)		((void *) ((base) << 20))

typedef struct MMU_L1_COARSE_PAGE_DESC {	// subpages disabled
	u32 Value01:2,				// set to 1
	 SBZ:3,						// should be 0
	 Domain:4,					// 0..15
	 IMPBit:1,					// implementation defined bit
	 Base:22;					// coarse page table base address [31..10]
} PACKED MMU_L1_COARSE_PAGE_DESC;

#define MMU_L1_COARSEBASE(addr)		(((addr) >> 10) & 0x3FFFFF)
#define MMU_L1_COARSEPTR(base)		((void *) ((base) << 10))

#define MMU__LEVEL2_COARSE_PAGE_TABLE_SIZE	0x400

typedef struct MMU_L2_EXT_SMALL_PAGE_DESC {	// subpages disabled
	u32 XNBit:1,				// execute never bit
	 Value1:1,					// set to 1
	 BBit:1,					// bufferable bit
	 CBit:1,					// cacheable bit
	 AP:2,						// access permissions
	 TEX:3,						// memory type extension field
	 APXBit:1,					// access permissions extended bit
	 SBit:1,					// shareable bit
	 NGBit:1,					// non-global bit
	 Base:20;					// extended small page base address
} PACKED MMU_L2_EXT_SMALL_PAGE_DESC;

#define MMU_L2_SMALLPAGEBASE(addr)	(((addr) >> 12) & 0xFFFFF)
#define MMU_L2_SMALLPAGEPTR(base)	((void *) ((base) << 12))

// TLB type register
#define ARM_TLB_TYPE_SEPARATE_TLBS	(1 << 0)

// (System) Control register
#define ARM_CONTROL_MMU			(1 << 0)
#define ARM_CONTROL_STRICT_ALIGNMENT	(1 << 1)
#define ARM_CONTROL_L1_CACHE		(1 << 2)
#define ARM_CONTROL_BRANCH_PREDICTION	(1 << 11)
#define ARM_CONTROL_L1_INSTRUCTION_CACHE (1 << 12)
#ifdef CONFIG_ARCH_ARM1176
#define ARM_CONTROL_UNALIGNED_PERMITTED	(1 << 22)
#define ARM_CONTROL_EXTENDED_PAGE_TABLE	(1 << 23)
#endif

// Translation table base registers
#ifdef CONFIG_ARCH_ARM1176
#define ARM_TTBR_INNER_CACHEABLE	(1 << 0)
#else
#define ARM_TTBR_INNER_NON_CACHEABLE	((0 << 6) | (0 << 0))
#define ARM_TTBR_INNER_WRITE_ALLOCATE	((1 << 6) | (0 << 0))
#define ARM_TTBR_INNER_WRITE_THROUGH	((0 << 6) | (1 << 0))
#define ARM_TTBR_INNER_WRITE_BACK	((1 << 6) | (1 << 0))
#endif

#define ARM_TTBR_USE_SHAREABLE_MEM	(1 << 1)

#define ARM_TTBR_OUTER_NON_CACHEABLE	(0 << 3)
#define ARM_TTBR_OUTER_WRITE_ALLOCATE	(1 << 3)
#define ARM_TTBR_OUTER_WRITE_THROUGH	(2 << 3)
#define ARM_TTBR_OUTER_WRITE_BACK	(3 << 3)

#ifndef CONFIG_ARCH_ARM1176
#define ARM_TTBR_NOT_OUTER_SHAREABLE	(1 << 5)
#endif

//  Auxiliary Control register
#ifdef CONFIG_ARCH_ARM1176
#define ARM_AUX_CONTROL_CACHE_SIZE	(1 << 6)	// restrict cache size to 16K (no page coloring)
#else
#define ARM_AUX_CONTROL_SMP		(1 << 6)
#endif

#endif
