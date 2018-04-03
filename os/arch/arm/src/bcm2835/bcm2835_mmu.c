//
// bcm2835_mmu.c
//
// Copyright (C) 2014-2015  R. Stange <rsta2@o2online.de>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <tinyara/config.h>
#include <tinyara/arch.h>
#include <sys/types.h>

#include <arch/armv6/armv6mmu.h>
#include <arch/armv6/cacheops.h>
#include <arch/board/memory.h>

#include "up_arch.h"

#ifdef CONFIG_ARCH_ARM1176
#define MMU_MODE	(  ARM_CONTROL_MMU			\
			 | ARM_CONTROL_L1_CACHE			\
			 | ARM_CONTROL_L1_INSTRUCTION_CACHE	\
			 | ARM_CONTROL_BRANCH_PREDICTION	\
			 | ARM_CONTROL_EXTENDED_PAGE_TABLE)

#define TTBCR_SPLIT	3
#else
#define MMU_MODE	(  ARM_CONTROL_MMU			\
			 | ARM_CONTROL_L1_CACHE			\
			 | ARM_CONTROL_L1_INSTRUCTION_CACHE	\
			 | ARM_CONTROL_BRANCH_PREDICTION)

#define TTBCR_SPLIT	2
#endif

#ifdef CONFIG_ARCH_CHIP_RP0W

#define TTBR_MODE	(  ARM_TTBR_INNER_CACHEABLE		\
			 | ARM_TTBR_OUTER_NON_CACHEABLE)
#else

#define TTBR_MODE	(  ARM_TTBR_INNER_WRITE_BACK		\
			 | ARM_TTBR_OUTER_WRITE_BACK)
#endif

typedef struct pagetable_s {
	int table_alloc;
	MMU_L1_SECTION_DESC *table_desc;
} pagetable_s;

typedef struct mmu_table_s {
	int m_enableMMU;
	uint32_t memory_size;		/* define programming memory size, not total DRAM size */

	pagetable_s *level1_default_table;
	pagetable_s *level1_table;
} mmu_table_s;

static void enable_MMU(mmu_table_s *priv);
static void mmu_initialize(mmu_table_s *priv, int enableMMU);
static uint32_t get_system_memory_size(mmu_table_s *priv);

static void setup_pagetable(pagetable_s *priv);
static void setup_pagetable_default(pagetable_s *priv, uint32_t memory_size);
static uint32_t get_pagetable_base_address(pagetable_s *priv);	// with mode bits to be loaded into TTBRn

static mmu_table_s mmu_table_g;
static pagetable_s ttbr0_table;	/* translation table for TTBR 0, as default */
static pagetable_s ttbr1_table;	/* translation table for TTBR 1 */

int bcm2835_mmu_init(void)
{
	mmu_initialize(&mmu_table_g, TRUE);
	return 0;
}

static void mmu_initialize(mmu_table_s *priv, int enableMMU)
{
	assert(priv != 0);

	lowsyslog(LOG_INFO, "\nSetup MMU & Enable Cache\n");
	lowsyslog(LOG_INFO, " - Page Table 1\t : 0x%08X\n", BCM2835_PAGE_TABLE1);
	lowsyslog(LOG_INFO, " - Page Table 2\t : 0x%08X\n", BCM2835_PAGE_TABLE2);
	priv->m_enableMMU = enableMMU;

	priv->level1_default_table = (pagetable_s *)&ttbr0_table;
	priv->level1_table = (pagetable_s *)&ttbr1_table;
	priv->memory_size = ARM_MEM_SIZE;	/* set system memory size */

	if (priv->m_enableMMU) {
		assert(priv->level1_default_table != 0);
		setup_pagetable_default(priv->level1_default_table, priv->memory_size);

		assert(priv->level1_table != 0);
		setup_pagetable(priv->level1_table);

		enable_MMU(priv);
	}
}

static uint32_t get_system_memory_size(mmu_table_s *priv)
{
	assert(priv != 0);

	return priv->memory_size;
}

static void enable_MMU(mmu_table_s *priv)
{
	assert(priv != 0);
	assert(priv->m_enableMMU);
	uint32_t nAuxControl;

	asm volatile("mrc p15, 0, %0, c1, c0,  1":"=r"(nAuxControl));
#ifdef CONFIG_ARCH_ARM1176
	nAuxControl |= ARM_AUX_CONTROL_CACHE_SIZE;	// restrict cache size (no page coloring)
#else
	nAuxControl |= ARM_AUX_CONTROL_SMP;
#endif
	asm volatile("mcr p15, 0, %0, c1, c0,  1"::"r"(nAuxControl));

	uint32_t nTLBType;
	asm volatile("mrc p15, 0, %0, c0, c0,  3":"=r"(nTLBType));
	assert(!(nTLBType & ARM_TLB_TYPE_SEPARATE_TLBS));

	// set TTB control
	asm volatile("mcr p15, 0, %0, c2, c0,  2"::"r"(TTBCR_SPLIT));

	// set TTBR0
	assert(priv->level1_default_table != 0);
	asm volatile("mcr p15, 0, %0, c2, c0,  0"::"r"(get_pagetable_base_address(priv->level1_default_table)));

	// set TTBR1
	assert(priv->level1_table != 0);
	asm volatile("mcr p15, 0, %0, c2, c0,  1"::"r"(get_pagetable_base_address(priv->level1_table)));

	// set Domain Access Control register (Domain 0 and 1 to client)
	asm volatile("mcr p15, 0, %0, c3, c0,  0"::"r"(DOMAIN_CLIENT << 0 | DOMAIN_CLIENT << 2));

	InvalidateDataCache();
	FlushPrefetchBuffer();

	// enable MMU
	uint32_t nControl;
	asm volatile("mrc p15, 0, %0, c1, c0,  0":"=r"(nControl));
#ifdef CONFIG_ARCH_ARM1176
#ifdef ARM_STRICT_ALIGNMENT
	nControl &= ~ARM_CONTROL_UNALIGNED_PERMITTED;
	nControl |= ARM_CONTROL_STRICT_ALIGNMENT;
#else
	nControl &= ~ARM_CONTROL_STRICT_ALIGNMENT;
	nControl |= ARM_CONTROL_UNALIGNED_PERMITTED;
#endif
#endif
	nControl |= MMU_MODE;
	asm volatile("mcr p15, 0, %0, c1, c0,  0"::"r"(nControl):"memory");
}

static void setup_pagetable(pagetable_s *priv)
{
	unsigned int nEntry;
	assert(priv != 0);

	priv->table_alloc = FALSE;
	priv->table_desc = (MMU_L1_SECTION_DESC *) BCM2835_PAGE_TABLE2;

	assert(((uint32_t) priv->table_desc & 0x3FFF) == 0);	/* Check 4K alignment */

	for (nEntry = 0; nEntry < 4096; nEntry++) {
		uint32_t base = MEGABYTE * nEntry;

		MMU_L1_SECTION_DESC *pEntry = &priv->table_desc[nEntry];

		// shared device
		pEntry->Value10 = 2;	/* Section Descriptor */
		pEntry->BBit = 1;		/* Write Buffer enable */
		pEntry->CBit = 0;		/* Cache Disable */
		pEntry->XNBit = 0;
		pEntry->Domain = 0;
		pEntry->IMPBit = 0;
		pEntry->AP = AP_SYSTEM_ACCESS;
		pEntry->TEX = 0;
		pEntry->APXBit = APX_RW_ACCESS;
		pEntry->SBit = 1;
		pEntry->NGBit = 0;
		pEntry->Value0 = 0;
		pEntry->SBZ = 0;
		pEntry->Base = MMU_L1_SECTIONBASE(base);

		if (nEntry >= SDRAM_SIZE_MBYTE) {
			pEntry->XNBit = 1;
		}
	}

	CleanDataCache();
	DataSyncBarrier();
}

static void setup_pagetable_default(pagetable_s *priv, uint32_t memory_size)
{
	unsigned int nEntry;
	assert(priv != 0);

	priv->table_alloc = TRUE;
	priv->table_desc = (MMU_L1_SECTION_DESC *) BCM2835_PAGE_TABLE1;

	assert(priv->table_desc != 0);
	assert(((uint32_t) priv->table_desc & 0xFFF) == 0);

	for (nEntry = 0; nEntry < SDRAM_SIZE_MBYTE; nEntry++) {
		uint32_t base = MEGABYTE * nEntry;

		MMU_L1_SECTION_DESC *pEntry = &priv->table_desc[nEntry];

		// outer and inner write back, no write allocate
		pEntry->Value10 = 2;
		pEntry->BBit = 1;
		pEntry->CBit = 1;		/* Cache Enable */
		pEntry->XNBit = 0;
		pEntry->Domain = 0;
		pEntry->IMPBit = 0;
		pEntry->AP = AP_SYSTEM_ACCESS;
		pEntry->TEX = 0;
		pEntry->APXBit = APX_RW_ACCESS;
		pEntry->SBit = 0;
		pEntry->NGBit = 0;
		pEntry->Value0 = 0;
		pEntry->SBZ = 0;
		pEntry->Base = MMU_L1_SECTIONBASE(base);

		extern u8 _etext;
		if (base >= (uint32_t) & _etext) {
			pEntry->XNBit = 1;

			if (base >= memory_size) {
				// shared device
				pEntry->BBit = 1;
				pEntry->CBit = 0;
				pEntry->TEX = 0;
				pEntry->SBit = 1;
			}
		}
	}

	CleanDataCache();
	DataSyncBarrier();
}

static uint32_t get_pagetable_base_address(pagetable_s *priv)
{
	assert(priv != 0);
	return (uint32_t) priv->table_desc | TTBR_MODE;
}
