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

/** @file
 * Defines WWD SDIO functions for Raspberry Pi Zero W
 */

#ifdef USE_LED_BLINK
#include <fcntl.h>
#include <tinyara/irq.h>
#include <tinyara/gpio.h>
#endif
#include "wwd_bus_protocol.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_sdio_interface.h"
#include "platform/wwd_bus_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_constants.h"
#include "network/wwd_network_constants.h"

#pragma pack(1)
typedef struct
{
    unsigned char stuff_bits;
    unsigned int  ocr :24;
} sdio_cmd5_argument_t;

typedef struct
{
    unsigned int  _unique2         : 9; /* 0-8   */
    unsigned int  register_address :17; /* 9-25  */
    unsigned int  _unique          : 2; /* 26-27 */
    unsigned int  function_number  : 3; /* 28-30 */
    unsigned int  rw_flag          : 1; /* 31    */
} sdio_cmd5x_argument_t;

typedef struct
{
    uint8_t       write_data;           /* 0 - 7 */
    unsigned int  _stuff2          : 1; /* 8     */
    unsigned int  register_address :17; /* 9-25  */
    unsigned int  _stuff           : 1; /* 26    */
    unsigned int  raw_flag         : 1; /* 27    */
    unsigned int  function_number  : 3; /* 28-30 */
    unsigned int  rw_flag          : 1; /* 31    */
} wwd_bus_sdio_cmd52_argument_t;

typedef struct
{
    unsigned int  count            : 9; /* 0-8   */
    unsigned int  register_address :17; /* 9-25  */
    unsigned int  op_code          : 1; /* 26    */
    unsigned int  block_mode       : 1; /* 27    */
    unsigned int  function_number  : 3; /* 28-30 */
    unsigned int  rw_flag          : 1; /* 31    */
} wwd_bus_sdio_cmd53_argument_t;

typedef union
{
	uint32_t value;
	sdio_cmd5_argument_t cmd5;
	wwd_bus_sdio_cmd52_argument_t cmd52;
	wwd_bus_sdio_cmd53_argument_t cmd53;
} sdio_cmd_argument_t;

#pragma pack()

#define __IO	volatile
#define __O	volatile
#define __I	volatile const

typedef struct {
	__IO uint32_t CS;					/* 0x00   System Timer Control/Status */
	__IO uint32_t CLO;          	/* 0x04   System Timer Counter Lower 32 bits */
	__IO uint32_t CHI;          	/* 0x08   System Timer Counter Higher 32 bits */
	__I uint32_t C0;            	/* 0x0C   System Timer Compare 0.  DO NOT USE; is used by GPU. */
	__IO uint32_t C1;           	/* 0x10   System Timer Compare 1 */
	__I uint32_t C2;            	/* 0x14   System Timer Compare 2.  DO NOT USE; is used by GPU. */
	__IO uint32_t C3;           	/* 0x18   System Timer Compare 3 */
} BCM2835_ST_TypeDef;

typedef struct {
    __IO uint32_t ARG2;             /* 0x00 */
    __IO uint32_t BLKSIZECNT;       /* 0x04 */
    __IO uint32_t ARG1;             /* 0x08 */
    __IO uint32_t CMDTM;            /* 0x0C */
    __O uint32_t RESP0;             /* 0x10 */
    __O uint32_t RESP1;             /* 0x14 */
    __O uint32_t RESP2;             /* 0x18 */
    __O uint32_t RESP3;             /* 0x1C */
    __IO uint32_t DATA;             /* 0x20 */
    __O uint32_t STATUS;            /* 0x24 */
    __IO uint32_t CONTROL0;         /* 0x28 */
    __IO uint32_t CONTROL1;         /* 0x2C */
    __IO uint32_t INTERRUPT;        /* 0x30 */
    __IO uint32_t IRPT_MASK;        /* 0x34 */
    __IO uint32_t IRPT_EN;          /* 0x38 */
    __IO uint32_t CONTROL2;         /* 0x3C */
    __IO uint32_t CAPABILITIES_0;   /* 0x40 */
    __IO uint32_t CAPABILITIES_1;   /* 0x44 */
    __IO uint32_t NOTINUSE1[2];
    __IO uint32_t FORCE_IRPT;       /* 0x50 */
    __IO uint32_t NOTINUSE2[7];
    __IO uint32_t BOOT_TIMEOUT;     /* 0x70 */
    __IO uint32_t DBG_SEL;          /* 0x74 */
    __IO uint32_t NOTINUSE3[2];
    __IO uint32_t EXRDFIFO_CFG;     /* 0x80 */
    __IO uint32_t EXRDFIFO_EN;      /* 0x84 */
    __IO uint32_t TUNE_STEP;        /* 0x88 */
    __IO uint32_t TUNE_STEPS_STD;   /* 0x8C */
    __IO uint32_t TUNE_STEPS_DDR;   /* 0x90 */
    __IO uint32_t NOTINUSE4[23];
    __IO uint32_t SPI_INT_SPT;      /* 0xF0 */
    __IO uint32_t NOTINUSE5[2];
    __IO uint32_t SLOTISR_VER;      /* 0xFC */
} BCM2835_EMMC1_TypeDef;

#define ARASAN_EMMC_ISR
//#define ARASAN_EMMC_POLLING
#define ARASAN_EMMC1_BASE_CLOCK					((uint32_t)250000000)
#define MIN_FREQ 									(400000)
#define BCM2835_EMMC1_WRITE_DELAY				(((2 * 1000000) / MIN_FREQ) + 1)
#define BCM2835_PERI_BASE						(0x20000000)
#define BCM2835_ST_BASE							(BCM2835_PERI_BASE + 0x003000)
#define ARM_IRQ2_BASE 							(32)
#define INTERRUPT_VC_ARASANSDIO					(ARM_IRQ2_BASE + 30)
#define ARASAN_EMMC1_BASE						(0x20300000)
#define ARASAN_EMMC1								((volatile BCM2835_EMMC1_TypeDef*) ARASAN_EMMC1_BASE)
#define BCM2835_ST								((volatile BCM2835_ST_TypeDef*)   BCM2835_ST_BASE)
#define BCM2835_TIMEOUT_WAIT(stop_if_true, usec)					\
	{																	\
		do {															\
			uint32_t start;											\
			uint32_t wait_usec;										\
																		\
			start = BCM2835_ST->CLO;									\
			wait_usec = (uint32_t)usec;								\
			do {														\
				if(stop_if_true)										\
					break;												\
			} while(abs(BCM2835_ST->CLO - start) < wait_usec);	\
		} while(0);													\
	};
#define BCM2835_TIMEOUT_WAIT2(stop_if_true, usec, ret)			\
	{																	\
		ret = 1;														\
		do {															\
			uint32_t start;											\
			uint32_t wait_usec;										\
																		\
			start = BCM2835_ST->CLO;									\
			wait_usec = (uint32_t)usec;								\
			do {														\
				if(stop_if_true){										\
					ret = 0;											\
					break;												\
				}														\
			} while(abs(BCM2835_ST->CLO - start) < wait_usec);	\
		} while(0);													\
	};
#define BCM2835_TIME_WAIT(usec)										\
	{																	\
		do {															\
			uint32_t start;											\
			uint32_t wait_usec;										\
																		\
			start = BCM2835_ST->CLO;									\
			wait_usec = (uint32_t)usec;								\
			do {														\
			} while(abs(BCM2835_ST->CLO - start) < wait_usec);	\
		} while(0);													\
	};

#define ARASAN_EMMC_CARD_INTERRUPT       		(1 << 8)
#define ARASAN_EMMC_IRPT_READ_RDY       		(1 << 5)
#define ARASAN_EMMC_IRPT_WRITE_RDY       		(1 << 4)
#define ARASAN_EMMC_IRPT_DATA_DONE       		(1 << 1)
#define ARASAN_EMMC_IRPT_CMD_DONE       		(1 << 0)
#define ARASAN_EMMC_INTERRUPT					(0x30)
#define ARASAN_EMMC_IRPT_MASK					(0x34)
#define ARASAN_EMMC_IRPT_EN						(0x38)
#define ARASAN_EMMC_IRPT_ACMD_ERR				(1 << 24)
#define ARASAN_EMMC_IRPT_DEND_ERR				(1 << 22)
#define ARASAN_EMMC_IRPT_DCRC_ERR				(1 << 21)
#define ARASAN_EMMC_IRPT_DTO_ERR				(1 << 20)
#define ARASAN_EMMC_IRPT_CBAD_ERR				(1 << 19)
#define ARASAN_EMMC_IRPT_CEND_ERR				(1 << 18)
#define ARASAN_EMMC_IRPT_CCRC_ERR				(1 << 17)
#define ARASAN_EMMC_IRPT_CTO_ERR				(1 << 16)
#define ARASAN_EMMC_IRPT_ERR_MASK \
		( ARASAN_EMMC_IRPT_ACMD_ERR	\
		| ARASAN_EMMC_IRPT_DEND_ERR \
		| ARASAN_EMMC_IRPT_DCRC_ERR \
		| ARASAN_EMMC_IRPT_DTO_ERR	\
		| ARASAN_EMMC_IRPT_CBAD_ERR	\
		| ARASAN_EMMC_IRPT_CEND_ERR	\
		| ARASAN_EMMC_IRPT_CCRC_ERR)
#define ARASAN_EMMC_SRST_DATA					(1 << 26)
#define ARASAN_EMMC_SRST_CMD					(1 << 25)
#define ARASAN_EMMC_IRPT_READ_RDY       		(1 << 5)
#define ARASAN_EMMC_IRPT_WRITE_RDY       		(1 << 4)
#define ARASAN_EMMC_IRPT_DATA_DONE       		(1 << 1)
#define ARASAN_EMMC_IRPT_CMD_DONE       		(1 << 0)
#define ARASAN_EMMC_CONTROL0_USE_4BITBUS		((uint32_t)(1 << 1))
#define ARASAN_EMMC_CONTROL0_HCTL_HS_EN		((uint32_t)(1 << 2))
#define ARASAN_EMMC_CONTROL0_USE_8BITBUS		((uint32_t)(1 << 5))
#define ARASAN_EMMC_DMA_INTERRUPT        		(1 << 3)
#define ARASAN_EMMC_INT_ADMA_ERROR				(1 << 25)
#define ARASAN_EMMC_ALL_INTTERUPT_DISABLE		(0xffffffff)
#define _NOP(N) {uint32_t cnt = N; while(cnt--) asm ("nop"); }

static host_semaphore_type_t __sdio_irq_semaphore;
static volatile uint32_t __current_irtp;
static pthread_mutex_t __sdio_bus_mutex;

static int __arasan_emmc_irq_isr(int irq, void *context, void *arg);
static __inline__ void __arasan_emmc_card_irq_disable(void);
void arasan_emmc_card_irq_enable(void);
static void __arasan_emmc_reset_data_circuit(void);
static void __arasan_emmc_reset_cmd_circuit(void);
static void __bcm2835_udelay(const uint32_t d);
static int32_t __arasan_emmc1_is_dmable(uint32_t block_size);
static void __arasan_emmc1_unset_transfer_irqs_dma(void);
static void __arasan_emmc1_set_transfer_irqs_pio(void);
static void __arasan_emmc1_4bit_mode_change_bit(void);
static uint32_t __arasan_emmc1_get_clock_divider(const uint32_t target_clock);
static uint32_t __arasan_emmc1_reset(void);
static void __sdio_enable_bus_irq_normal(void);
static void __sdio_disable_bus_irq_normal(void);
static uint32_t __arasan_emmc_proc_data_pio_read(void* data, uint32_t data_size);
static uint32_t __arasan_emmc_proc_data_pio_write(void* data, uint32_t data_size);
static uint32_t __arasan_emmc_proc_data_pio(wwd_bus_transfer_direction_t dir, uint32_t* data, uint32_t data_size);
static uint32_t __arasan_emmc_proc_data_dma_read(void* data, uint32_t data_size);
static uint32_t __arasan_emmc_proc_data_dma_write(void* data, uint32_t data_size);
static uint32_t __arasan_emmc_proc_data_dma(wwd_bus_transfer_direction_t dir, uint32_t* data, uint32_t data_size);
static uint32_t __get_block_size(sdio_block_size_t block_size);
static void __sdio_enable_bus_irq_data_write(wwd_bus_transfer_direction_t dir);

#ifdef USE_LED_BLINK

#define BLINK_RED_GPIO_FILE	"/dev/gpio47"
static int __gpio_led_fd = 0;

static void __gpio_led_open(void)
{
	__gpio_led_fd = open(BLINK_RED_GPIO_FILE, O_WRONLY);
	if (__gpio_led_fd > 0)
		ioctl(__gpio_led_fd, GPIOIOC_SET_DIRECTION, GPIO_DIRECTION_OUT);
}
static void __gpio_led_close(void)
{
	if (__gpio_led_fd > 0) {
		close(__gpio_led_fd);
		__gpio_led_fd = 0;
	}
}
static void __led_blink_on(void)
{
	if (__gpio_led_fd > 0)
		write(__gpio_led_fd, "0""\0x0", 2);
}
static void __led_blink_off(void)
{
	if (__gpio_led_fd > 0)
		write(__gpio_led_fd, "1""\0x0", 2);
}

#define RPI0W_BLINK_LED_OPEN	__gpio_led_open();
#define RPI0W_BLINK_LED_CLOSE	__gpio_led_close();
#define RPI0W_BLINK_LED_ON		__led_blink_on();
#define RPI0W_BLINK_LED_OFF		__led_blink_off();
#else
#define RPI0W_BLINK_LED_OPEN
#define RPI0W_BLINK_LED_CLOSE
#define RPI0W_BLINK_LED_ON
#define RPI0W_BLINK_LED_OFF
#endif /* USE_LED_BLINK */


static void __arasan_emmc_reset_data_circuit(void)
{
    uint32_t control1;

    control1 = ARASAN_EMMC1->CONTROL1;
    ARASAN_EMMC1->CONTROL1 = control1;

#if 0
    BCM2835_TIMEOUT_WAIT((ARASAN_EMMC1->CONTROL1 & ARASAN_EMMC_SRST_DATA) == 0, 1000000);
#else
    usleep(1000);
#endif

#if 0 /* Test code */
	if((ARASAN_EMMC1->CONTROL1 & ARASAN_EMMC_SRST_DATA) != 0)
		printf("DATA line did not reset properly\n");
#endif
}

static void __arasan_emmc_reset_cmd_circuit(void)
{
    uint32_t control1;

    control1 = ARASAN_EMMC1->CONTROL1;
    control1 |= ARASAN_EMMC_SRST_CMD;
    ARASAN_EMMC1->CONTROL1 = control1;

#if 0
    BCM2835_TIMEOUT_WAIT((ARASAN_EMMC1->CONTROL1 & ARASAN_EMMC_SRST_CMD) == 0, 1000000);
#else
    usleep(1000);
#endif

#if 0 /* Test code */
	if((ARASAN_EMMC1->CONTROL1 & ARASAN_EMMC_SRST_CMD) != 0)
		printf("CMD line did not reset properly\n");
#endif
}

static void __bcm2835_udelay(const uint32_t d)
{
    const uint32_t clo = BCM2835_ST->CLO;

    while ((BCM2835_ST->CLO - clo) < d) {
    };
}

static int32_t __arasan_emmc1_is_dmable(uint32_t block_size)
{
//	if (64 < block_size)
//		return 1;

	return 0;
}

static void __arasan_emmc1_unset_transfer_irqs_dma(void)
{
	uint32_t signal;

	signal = ARASAN_EMMC1->IRPT_EN;
	signal &= ~(ARASAN_EMMC_DMA_INTERRUPT | ARASAN_EMMC_INT_ADMA_ERROR);

	ARASAN_EMMC1->IRPT_EN = signal;
}

static void __arasan_emmc1_set_transfer_irqs_dma(void)
{
#define SD_DMA_INTERRUPT			((uint32_t)(1 << 3))
#define EMMC_INT_ADMA_ERROR		((uint32_t)(1 << 25))

	ARASAN_EMMC1->IRPT_MASK |=  SD_DMA_INTERRUPT | EMMC_INT_ADMA_ERROR;
	ARASAN_EMMC1->IRPT_EN |= SD_DMA_INTERRUPT | EMMC_INT_ADMA_ERROR;
}

static void __arasan_emmc1_set_transfer_irqs_pio(void)
{
#define SD_BUFFER_WRITE_READY	((uint32_t)(1 << 4))
#define SD_BUFFER_READ_READY	((uint32_t)(1 << 5))

	ARASAN_EMMC1->IRPT_MASK |=  SD_BUFFER_WRITE_READY | SD_BUFFER_READ_READY;
	ARASAN_EMMC1->IRPT_EN |= SD_BUFFER_WRITE_READY | SD_BUFFER_READ_READY;
}

static void __arasan_emmc1_4bit_mode_change_bit(void)
{
	uint32_t control0;

	control0 = ARASAN_EMMC1->CONTROL0;
	control0 &= ~ARASAN_EMMC_CONTROL0_USE_8BITBUS;
	control0 |= ARASAN_EMMC_CONTROL0_USE_4BITBUS;

	ARASAN_EMMC1->CONTROL0 = control0;

	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);
}

static uint32_t __arasan_emmc1_get_clock_divider(const uint32_t target_clock)
{
	uint32_t targetted_divisor;
	uint32_t ret;
	int32_t divisor;
	int32_t first_bit;
	uint32_t bit_test;
	uint32_t mod;

	targetted_divisor = 0;

	if (target_clock > ARASAN_EMMC1_BASE_CLOCK)
		targetted_divisor = 1;
	else {
		targetted_divisor = ARASAN_EMMC1_BASE_CLOCK / target_clock;
		mod = ARASAN_EMMC1_BASE_CLOCK % target_clock;
		if (mod)
			targetted_divisor++;
	}

	divisor = -1;
	for (first_bit = 31; first_bit >= 0; first_bit--) {
		bit_test = (1 << first_bit);
		if (targetted_divisor & bit_test) {
			divisor = first_bit;
			targetted_divisor &= ~bit_test;
			if (targetted_divisor)
				divisor++; /* The divisor is not a power-of-two, increase it */
			break;
		}
	}

	if(divisor == -1) divisor = 31;
	if(divisor >= 32) divisor = 31;
	if(divisor != 0) divisor = (1 << (divisor - 1));
	if(divisor >= 0x400) divisor = 0x3ff;

#define SDHCI_DIVIDER_SHIFT		((uint32_t)8)
#define SDHCI_DIVIDER_HI_SHIFT	((uint32_t)6)
#define SDHCI_DIV_MASK			((uint32_t)0xFF)
#define SDHCI_DIV_MASK_LEN		((uint32_t)8)
#define SDHCI_DIV_HI_MASK		((uint32_t)0x300)

	ret = (divisor & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	ret |= ((divisor & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN) << SDHCI_DIVIDER_HI_SHIFT;

	return ret;
}

static int32_t __arasan_emmc1_set_clock(const uint32_t target_clock) {
#define SD_GET_CLOCK_DIVIDER_FAIL			((uint32_t)0xffffffff)
#define BCM2835_EMMC_STATUS_CMD_INHIBIT	((uint32_t)0x00000001)
#define BCM2835_EMMC_STATUS_DATA_INHIBIT	((uint32_t)0x00000002)
#define BCM2835_EMMC_CONTROL1_CLK_STABLE	((uint32_t)(1 << 1))
#define BCM2835_EMMC_CONTROL1_CLK_EN		((uint32_t)(1 << 2))

	const uint32_t divider = __arasan_emmc1_get_clock_divider(target_clock);
	uint32_t control1;

    if (divider == SD_GET_CLOCK_DIVIDER_FAIL) {
    	printf("Couldn't get a valid divider for target clock %d Hz", target_clock);
		return -1;
	}

	/* Wait for the command inhibit (CMD and DAT) bits to clear */
#if 0
    BCM2835_TIMEOUT_WAIT(!(ARASAN_EMMC1->STATUS & (BCM2835_EMMC_STATUS_CMD_INHIBIT || BCM2835_EMMC_STATUS_DATA_INHIBIT)), 1000000);
#else
    usleep(1000);
#endif

	if ((ARASAN_EMMC1->STATUS & (BCM2835_EMMC_STATUS_CMD_INHIBIT || BCM2835_EMMC_STATUS_DATA_INHIBIT)) != 0) {
		printf("Timeout waiting for inhibit flags. Status %08x", ARASAN_EMMC1->STATUS);
		return -1;
	}

	/* Set the SD clock off */
	control1 = ARASAN_EMMC1->CONTROL1;
	control1 &= ~BCM2835_EMMC_CONTROL1_CLK_EN;
	ARASAN_EMMC1->CONTROL1 = control1;
	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

	/* Write the new divider */
	control1 &= ~0xffe0; /* Clear old setting + clock generator select */
	control1 |= divider;
	ARASAN_EMMC1->CONTROL1 = control1;
	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

#if 0
	BCM2835_TIMEOUT_WAIT(ARASAN_EMMC1->CONTROL1 & BCM2835_EMMC_CONTROL1_CLK_STABLE, 1000000);
#else
	usleep(1000);
#endif

	if ((ARASAN_EMMC1->CONTROL1 & BCM2835_EMMC_CONTROL1_CLK_STABLE) == 0) {
		printf("Controller's clock did not stabilize within 1 second");
		return -1;
	}

	/* Enable the SD clock */
	control1 |= BCM2835_EMMC_CONTROL1_CLK_EN;
	ARASAN_EMMC1->CONTROL1 = control1;
	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

	return 0;
}

static uint32_t __arasan_emmc1_reset(void)
{
#define BCM2835_EMMC_CONTROL1_SRST_HC				((uint32_t)(1 << 24))
#define BCM2835_EMMC_CONTROL1_DATA_TOUNIT			((uint32_t)(1 << 0))
#define BCM2835_EMMC_CONTROL1_CLK_INTLEN			((uint32_t)(1 << 0))
#define BCM2835_EMMC_CONTROL1_DATA_TOUNIT_MAX		((uint32_t)(0x000e0000))
#define SD_CLOCK_ID									4000000

	uint32_t control1;

	ARASAN_EMMC1->CONTROL0 = (uint32_t)0;
	ARASAN_EMMC1->CONTROL2 = (uint32_t)0;

	/* Send reset host controller and wait for complete. */
	control1 = ARASAN_EMMC1->CONTROL1;
	control1 |= BCM2835_EMMC_CONTROL1_SRST_HC;
	ARASAN_EMMC1->CONTROL1 = control1;
	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

#if 0
	BCM2835_TIMEOUT_WAIT((ARASAN_EMMC1->CONTROL1 & BCM2835_EMMC_CONTROL1_SRST_HC) == 0, 1000000);
#else
	usleep(100);
#endif

	if ((ARASAN_EMMC1->CONTROL1 & BCM2835_EMMC_CONTROL1_SRST_HC) != 0) {
		printf("Controller failed to reset");
		return -1;
	}

	/* Enable internal clock and set data timeout. */
	control1 = ARASAN_EMMC1->CONTROL1;
	control1 |= BCM2835_EMMC_CONTROL1_CLK_INTLEN;
	control1 |= BCM2835_EMMC_CONTROL1_DATA_TOUNIT_MAX;
	control1 |= 7 << 16; // 0xE is maximum timeout value.
	ARASAN_EMMC1->CONTROL1 = control1;
	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

	if (__arasan_emmc1_set_clock(SD_CLOCK_ID) < 0)
		return -1;

	/* Mask off sending interrupts to the ARM.
	 * Reset interrupts.
	 * Have all interrupts sent to the INTERRUPT register. */
	ARASAN_EMMC1->IRPT_EN = 0;
	ARASAN_EMMC1->INTERRUPT = 0xffffffff;
	ARASAN_EMMC1->IRPT_MASK = 0xffffffff;// & (~ARASAN_SD_CARD_INTERRUPT);

	__bcm2835_udelay(BCM2835_EMMC1_WRITE_DELAY);

	return 0;
}

#ifdef ARASAN_EMMC_POLLING
static void __sdio_enable_bus_irq_normal(void)
{
	uint32_t irpts;

	irpts = ARASAN_EMMC_IRPT_ERR_MASK | ARASAN_EMMC_IRPT_CMD_DONE;
	irpts &= ~(ARASAN_EMMC_IRPT_DATA_DONE | ARASAN_EMMC_IRPT_READ_RDY | ARASAN_EMMC_IRPT_WRITE_RDY);
	irpts |= ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT;

	ARASAN_EMMC1->IRPT_MASK = irpts;
/*	ARASAN_EMMC1->IRPT_EN = ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT; */
}
#endif /* #ifdef ARASAN_EMMC_POLLING */

#ifdef ARASAN_EMMC_ISR
static void __sdio_enable_bus_irq_normal(void)
{
	uint32_t irpts;

	irpts = ARASAN_EMMC_IRPT_ERR_MASK | ARASAN_EMMC_IRPT_CMD_DONE;
	irpts &= ~(ARASAN_EMMC_IRPT_DATA_DONE | ARASAN_EMMC_IRPT_READ_RDY | ARASAN_EMMC_IRPT_WRITE_RDY);
	irpts |= ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT;

	ARASAN_EMMC1->IRPT_MASK = irpts;
	ARASAN_EMMC1->IRPT_EN = irpts;
}
#endif /* #ifdef ARASAN_EMMC_ISR */

static void __sdio_disable_bus_irq_normal(void)
{
}

#ifdef ARASAN_EMMC_POLLING
static void __sdio_enable_bus_irq_data_write(wwd_bus_transfer_direction_t dir)
{
	uint32_t irpts;

	irpts = ARASAN_EMMC_IRPT_ERR_MASK | ARASAN_EMMC_IRPT_DATA_DONE;

	if (dir == BUS_READ) {
		irpts |= ARASAN_EMMC_IRPT_READ_RDY;
		irpts &= ~ARASAN_EMMC_IRPT_WRITE_RDY;
	}
	else {
		irpts |= ARASAN_EMMC_IRPT_WRITE_RDY;
		irpts &= ~ARASAN_EMMC_IRPT_READ_RDY;
	}

	irpts &= ~ARASAN_EMMC_IRPT_CMD_DONE;
	irpts |= ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT;

#if 0
	ARASAN_EMMC1->IRPT_MASK = irpts;
	ARASAN_EMMC1->IRPT_EN = irpts;
#else
	ARASAN_EMMC1->IRPT_MASK = irpts;
/*	ARASAN_EMMC1->IRPT_EN = ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT; */
#endif
}
#endif /* #ifdef ARASAN_EMMC_POLLING */

#ifdef ARASAN_EMMC_ISR
static void __sdio_enable_bus_irq_data_write(wwd_bus_transfer_direction_t dir)
{
	/* Interrupt */
	uint32_t irpts;

	irpts = ARASAN_EMMC_IRPT_ERR_MASK | ARASAN_EMMC_IRPT_DATA_DONE;

	if (dir == BUS_READ) {
		irpts |= ARASAN_EMMC_IRPT_READ_RDY;
		irpts &= ~ARASAN_EMMC_IRPT_WRITE_RDY;
	}
	else {
		irpts |= ARASAN_EMMC_IRPT_WRITE_RDY;
		irpts &= ~ARASAN_EMMC_IRPT_READ_RDY;
	}

	irpts &= ~ARASAN_EMMC_IRPT_CMD_DONE;
	irpts |= ARASAN_EMMC1->IRPT_EN & ARASAN_EMMC_CARD_INTERRUPT;

	ARASAN_EMMC1->IRPT_MASK = irpts;
	ARASAN_EMMC1->IRPT_EN = irpts;
}
#endif /* #ifdef ARASAN_EMMC_ISR */

#ifndef WICED_DISABLE_MCU_POWERSAVE
wwd_result_t host_enable_oob_interrupt(void)
{
    return WWD_SUCCESS;
}

uint8_t host_platform_get_oob_interrupt_pin(void)
{
    return 0;
}
#endif /* ifndef  WICED_DISABLE_MCU_POWERSAVE */

/**
 * Initializes the ARASAN EMMC Bus
 *
 * Implemented in the platform interface which is specific to the
 * platform in use.
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_platform_bus_init(void)
{
	wwd_result_t ret;

	if (pthread_mutex_init(&__sdio_bus_mutex, NULL))
		return WWD_SDIO_BUS_UP_FAIL;

	ret = host_rtos_init_semaphore(&__sdio_irq_semaphore);
	if (ret == WWD_SEMAPHORE_ERROR) {
		printf("Oops~ host_rtos_init_semaphore() is WWD_SEMAPHORE_ERROR\n");
		return WWD_SDIO_BUS_UP_FAIL;
	}

	if (__arasan_emmc1_reset() != 0)
		return WWD_SDIO_BUS_UP_FAIL;

	ARASAN_EMMC1->IRPT_EN = 0;
	ARASAN_EMMC1->IRPT_MASK = 0;
	ARASAN_EMMC1->INTERRUPT = ARASAN_EMMC_ALL_INTTERUPT_DISABLE;

	irq_attach(INTERRUPT_VC_ARASANSDIO, __arasan_emmc_irq_isr, NULL);
	up_enable_irq(INTERRUPT_VC_ARASANSDIO);

	RPI0W_BLINK_LED_OPEN

    return WWD_SUCCESS;
}

/**
 * De-Initializes the ARASAN EMMC
 *
 * Implemented in the platform interface which is specific to the
 * platform in use.
 * This function does the reverse of @ref host_platform_bus_init
 *
 * @return WWD_SUCCESS or Error code
 */
wwd_result_t host_platform_bus_deinit(void)
{
	pthread_mutex_destroy(&__sdio_bus_mutex);
	host_rtos_deinit_semaphore(&__sdio_irq_semaphore);
	pthread_mutex_destroy(&__sdio_bus_mutex);

	up_disable_irq(INTERRUPT_VC_ARASANSDIO);
	irq_detach(INTERRUPT_VC_ARASANSDIO);

	RPI0W_BLINK_LED_CLOSE

	return WWD_SUCCESS;
}

/**
 * Performs SDIO enumeration
 *
 * This needs to be called if the WLAN chip is reset
 *
 */
wwd_result_t host_platform_sdio_enumerate( void )
{
#define SDIO_ENUMERATION_TIMEOUT_MS    ((uint32_t)500)
	wwd_result_t result;
	uint32_t loop_count;
	uint32_t data = 0;

	loop_count = 0;
	do {
			/* Send CMD0 to set it to idle state */
			host_platform_sdio_transfer(BUS_WRITE, SDIO_CMD_0, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL);

			host_platform_sdio_transfer(BUS_READ, SDIO_CMD_5, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, NO_RESPONSE, NULL);

			/* Send CMD3 to get RCA. */
			result = host_platform_sdio_transfer(BUS_READ, SDIO_CMD_3, SDIO_BYTE_MODE, SDIO_1B_BLOCK, 0, 0, 0, RESPONSE_NEEDED, &data);
			loop_count++;
			if (loop_count >= SDIO_ENUMERATION_TIMEOUT_MS)
				return WWD_TIMEOUT;
    } while ((result != WWD_SUCCESS ) && (host_rtos_delay_milliseconds((uint32_t) 1 ), ( 1 == 1 )));
    /* If you're stuck here, check the platform matches your hardware */

    /* Send CMD7 with the returned RCA to select the card */
    host_platform_sdio_transfer( BUS_WRITE, SDIO_CMD_7, SDIO_BYTE_MODE, SDIO_1B_BLOCK, data, 0, 0, RESPONSE_NEEDED, NULL);

    return WWD_SUCCESS;
}

static uint32_t __arasan_emmc_proc_data_pio_read(void* data, uint32_t data_size)
{
	uint32_t chunk;
	uint32_t scratch;
	uint8_t *buf;

	chunk = 0;
	scratch = 0;
	buf = (uint8_t *)data;

	while (data_size--) {
		if (chunk == 0) {
			scratch = ARASAN_EMMC1->DATA;
			chunk = 4;
			_NOP(16);
		}

		*buf++ = scratch & 0xff;
		scratch >>= 8;
		chunk--;
	}

	return 0;
}

static uint32_t __arasan_emmc_proc_data_pio_write(void* data, uint32_t data_size)
{
	uint32_t chunk;
	uint32_t scratch;
	uint8_t *buf;
	uint32_t remainder;
	uint32_t quotient;

	chunk = 0;
	scratch = 0;

	quotient = data_size / 4;
	remainder = data_size % 4;
	uint32_t *buf2;
	buf2 = (uint32_t *)data;

	while (quotient) {
		ARASAN_EMMC1->DATA = *buf2++;
		quotient--;
		_NOP(16);
	}

	buf = (uint8_t *)buf2;
	scratch = 0;
	while (remainder) {
		scratch |= *buf << (chunk << 3);

		buf++;
		chunk++;
		remainder--;

		if ((chunk == 4)	|| (remainder == 0)) {
			ARASAN_EMMC1->DATA = scratch;
			chunk = 0;
			scratch = 0;
		}
	}

	return 0;
}

static uint32_t __arasan_emmc_proc_data_pio(wwd_bus_transfer_direction_t dir, uint32_t* data, uint32_t data_size)
{
	if (dir == BUS_READ)
		__arasan_emmc_proc_data_pio_read(data, data_size);
	else if (dir == BUS_WRITE)
		__arasan_emmc_proc_data_pio_write(data, data_size);

	return 0;
}

static uint32_t __arasan_emmc_proc_data_dma_read(void* data, uint32_t data_size)
{
	return 0;
}

static uint32_t __arasan_emmc_proc_data_dma_write(void* data, uint32_t data_size)
{
	return bcm2835_arasan_emmc_dma_w((ARASAN_EMMC1_BASE + 0x10), data, data_size);
}

static uint32_t __arasan_emmc_proc_data_dma(wwd_bus_transfer_direction_t dir, uint32_t* data, uint32_t data_size)
{
	if (dir == BUS_READ)
		__arasan_emmc_proc_data_dma_read(data, data_size);
	else if (dir == BUS_WRITE)
		__arasan_emmc_proc_data_dma_write(data, data_size);

	return 0;
}

static uint32_t __get_block_size(sdio_block_size_t block_size)
{
	switch (block_size) {
	case SDIO_1B_BLOCK:
		return 1;
	case SDIO_2B_BLOCK:
		return 2;
	case SDIO_4B_BLOCK:
		return 4;
	case SDIO_8B_BLOCK:
		return 8;
	case SDIO_16B_BLOCK:
		return 16;
	case SDIO_32B_BLOCK:
		return 32;
	case SDIO_64B_BLOCK:
		return 64;
	case SDIO_128B_BLOCK:
		return 128;
	case SDIO_256B_BLOCK:
		return 256;
	default:
		return 0;
	}

	return 0;
}

static void __print_cmd52_arg(wwd_bus_sdio_cmd52_argument_t* arg)
{
	printf("CMD52:");
	printf("write_data:0x%02x,", arg->write_data);
	printf("address:0x%x,", arg->register_address);
	printf("raw_flag:0x%x,", arg->raw_flag);
	printf("function:%d,", arg->function_number);
	printf("rw_flag:%c\n", arg->rw_flag ? 'w' : 'r');
}

static void __print_cmd53_arg(wwd_bus_sdio_cmd53_argument_t* arg)
{
	printf("CMD53:");
	printf("blksize|blkcnt:%d,", arg->count);
	printf("address:0x%x,", arg->register_address);
	printf("op_code:%d,", arg->op_code);
	printf("block_mode:%d,", arg->block_mode);
	printf("function:%d,", arg->function_number);
	printf("rw_flag:%c\n", arg->rw_flag ? 'w' : 'r');
}

wwd_result_t host_platform_unmask_sdio_interrupt(void)
{
	arasan_emmc_card_irq_enable();

	return WWD_SUCCESS;
}


/**
 * Transfers SDIO data
 *
 * Implemented in the Platform interface, which is specific to the
 * platform in use.
 * Uses this function as a generic way to transfer data
 * across an SDIO bus.
 * Please refer to the SDIO specification.
 *
 * @param direction         : Direction of transfer - Write = to Wi-Fi device,
 *                                                    Read  = from Wi-Fi device
 * @param command           : The SDIO command number
 * @param mode              : Indicates whether transfer will be byte mode or block mode
 * @param block_size        : The block size to use (if using block mode transfer)
 * @param argument          : The argument of the particular SDIO command
 * @param data              : A pointer to the data buffer used to transmit or receive
 * @param data_size         : The length of the data buffer
 * @param response_expected : Indicates if a response is expected - RESPONSE_NEEDED = Yes
 *                                                                  NO_RESPONSE     = No
 * @param response  : A pointer to a variable which will receive the SDIO response.
 *                    Can be null if the caller does not care about the response value.
 *
 * @return WWD_SUCCESS if successful, otherwise an error code
 */
static
wwd_result_t __host_platform_sdio_transfer(
		wwd_bus_transfer_direction_t direction,
		sdio_command_t command,
		sdio_transfer_mode_t mode,
		sdio_block_size_t block_size_t,
		uint32_t argument,
		/*@null@*/ uint32_t* data,
		uint16_t data_size,
		sdio_response_needed_t response_expected,
		/*@out@*/ /*@null@*/ uint32_t* response)
{
#define SD_CMD_ID(a)					((uint32_t)((a) << 24))
#define SD_CMD_RSPNS_TYPE_136		(1 << 16)
#define SD_CMD_RSPNS_TYPE_48		(2 << 16)
#define SD_CMD_RSPNS_TYPE_48B		(3 << 16)
#define SD_CMD_CRCCHK_EN				(1 << 19)
#define SD_CMD_ISCMD					(1 << 20)
#define SD_CMD_ISDATA				(1 << 21)
#define SD_RESP_R1b         		(SD_CMD_RSPNS_TYPE_48B | SD_CMD_CRCCHK_EN)
#define SD_RESP_R4					(SD_CMD_RSPNS_TYPE_136)
#define SD_RESP_R5         			(SD_CMD_RSPNS_TYPE_48 | SD_CMD_CRCCHK_EN | SD_CMD_ISCMD)
#define SD_RESP_R6					(SD_CMD_RSPNS_TYPE_48 | SD_CMD_CRCCHK_EN)
#define SDHCI_TRNS_DMA     			(1 << 0)
#define SDHCI_TRNS_BLK_CNT_EN  		(1 << 1)
#define SDHCI_TRNS_READ				(1 << 4)
#define SDHCI_TRNS_MULTI_BLOCK		(1 << 5)

	uint32_t block_size;
	uint32_t cmd_reg;
	int32_t dmable;
	uint32_t blkcnt;
	uint32_t blksize;
	sdio_cmd_argument_t sdio_cmd;
	uint32_t wait_irtp;
#ifdef ARASAN_EMMC_ISR
	wwd_result_t result;
	uint32_t irtp;
#endif /* ARASAN_EMMC_ISR */
#ifdef ARASAN_EMMC_POLLING
	uint32_t time_out;
#endif /* ARASAN_EMMC_POLLING */

    while(ARASAN_EMMC1->STATUS & BCM2835_EMMC_STATUS_CMD_INHIBIT);
    while(ARASAN_EMMC1->STATUS & BCM2835_EMMC_STATUS_DATA_INHIBIT);

	switch (command) {
	case SDIO_CMD_0:
		cmd_reg = SD_CMD_ID(0);
		break;
	case SDIO_CMD_3:
		cmd_reg = SD_CMD_ID(3) | SD_RESP_R6;
			break;
	case SDIO_CMD_5:
		cmd_reg = SD_CMD_ID(5) | SD_RESP_R4;
		sdio_cmd.value = argument;
			break;
	case SDIO_CMD_7:
		cmd_reg = SD_CMD_ID(7) | SD_RESP_R1b;
			break;
	case SDIO_CMD_52:
		cmd_reg = SD_CMD_ID(52) | SD_RESP_R5;
		sdio_cmd.value = argument;
//		__print_cmd52_arg(&sdio_cmd.cmd52);
			break;
	case SDIO_CMD_53:
		cmd_reg = SD_CMD_ID(53) | SD_CMD_ISDATA;
		sdio_cmd.value = argument;
//		__print_cmd53_arg(&sdio_cmd.cmd53);
			break;
	default:
		return WWD_BADARG;
	}

	dmable = 0;
#if 0
	BCM2835_TIME_WAIT(5);
#else
	_NOP(500);
#endif

	if ((command == SDIO_CMD_53)) {
		if (!data) {
			return WWD_TIMEOUT;
		}

		block_size = __get_block_size(block_size_t);
		if (block_size == 0) {
			return WWD_TIMEOUT;
		}

		if (data_size < block_size) {
			blkcnt = 1;
			blksize = data_size;
		} else {
			blkcnt = data_size / block_size;
			blksize = block_size;
		}

		if (sdio_cmd.cmd53.block_mode) {
			blkcnt = sdio_cmd.cmd53.count;
			blksize = block_size;
			data_size = blksize * blkcnt;
		}

		if (sdio_cmd.cmd53.block_mode) {
			cmd_reg |= SDHCI_TRNS_BLK_CNT_EN;
			if (blkcnt > 1)
				cmd_reg |= SDHCI_TRNS_MULTI_BLOCK;
		}

		dmable = __arasan_emmc1_is_dmable(data_size);

		if (dmable) {
			ARASAN_EMMC1->BLKSIZECNT = (((7 & 0x7) << 12) | (blksize & 0xFFF));
//			__arasan_emmc1_set_transfer_irqs_dma();
//			cmd_reg |= SDHCI_TRNS_DMA;
		} else
			ARASAN_EMMC1->BLKSIZECNT =  blksize & 0x1FF;
		ARASAN_EMMC1->BLKSIZECNT |= (blkcnt << 16);

		if (direction == BUS_READ)
			wait_irtp = ARASAN_EMMC_IRPT_READ_RDY;
		else
			wait_irtp = ARASAN_EMMC_IRPT_WRITE_RDY;;

		__sdio_enable_bus_irq_data_write(direction);
	} else {
		ARASAN_EMMC1->BLKSIZECNT = 0;
		wait_irtp = ARASAN_EMMC_IRPT_CMD_DONE;
		__sdio_enable_bus_irq_normal();
	}

	if (direction == BUS_READ)
		cmd_reg |= SDHCI_TRNS_READ;

	RPI0W_BLINK_LED_ON

	ARASAN_EMMC1->INTERRUPT = ARASAN_EMMC_ALL_INTTERUPT_DISABLE;
	ARASAN_EMMC1->ARG1 = argument;
	ARASAN_EMMC1->CMDTM = cmd_reg;

#ifdef ARASAN_EMMC_POLLING
	BCM2835_TIMEOUT_WAIT2(ARASAN_EMMC1->INTERRUPT & wait_irtp, 5000, time_out);
	if ((ARASAN_EMMC1->INTERRUPT & ARASAN_EMMC_IRPT_ERR_MASK) || time_out)
		goto cmd_err;
#endif /* #ifdef ARASAN_EMMC_POLLING */
#ifdef ARASAN_EMMC_ISR
	result = host_rtos_get_semaphore(&__sdio_irq_semaphore, 20, WICED_TRUE);
	irtp = __current_irtp;
	if ((irtp & ARASAN_EMMC_IRPT_ERR_MASK) || (result != WWD_SUCCESS))
		goto cmd_err;
#endif /* #ifdef ARASAN_EMMC_POLLING */
	ARASAN_EMMC1->IRPT_MASK &= ~wait_irtp;
	ARASAN_EMMC1->INTERRUPT |= wait_irtp;
	if (dmable) {
		/* TODO */
		/* The dma transmission is performed here. */
		if (__arasan_emmc_proc_data_dma(direction, (void*)data, data_size))
			goto data_err;
	} else {
		if (data) {
			if (__arasan_emmc_proc_data_pio(direction, (void*)data, data_size))
				goto data_err;

#ifdef ARASAN_EMMC_POLLING
			/* Polling */
			BCM2835_TIMEOUT_WAIT2(ARASAN_EMMC1->INTERRUPT & ARASAN_EMMC_IRPT_DATA_DONE, 5000, time_out);
			if ((ARASAN_EMMC1->INTERRUPT & ARASAN_EMMC_IRPT_ERR_MASK) || time_out)
				goto data_err;
#endif /* #ifdef ARASAN_EMMC_POLLING */
#ifdef ARASAN_EMMC_ISR
			result = host_rtos_get_semaphore(&__sdio_irq_semaphore, 10, WICED_FALSE);
			irtp = __current_irtp;
			if (result != WWD_SUCCESS)
				goto data_err;
			if ((irtp & ARASAN_EMMC_IRPT_ERR_MASK) || !(irtp & ARASAN_EMMC_IRPT_DATA_DONE))
				goto data_err;
#endif /* #ifdef ARASAN_EMMC_ISR */
			ARASAN_EMMC1->IRPT_MASK &= ~ARASAN_EMMC_IRPT_DATA_DONE;
			ARASAN_EMMC1->INTERRUPT |= ARASAN_EMMC_IRPT_DATA_DONE;
		} else if (response_expected == RESPONSE_NEEDED && response != NULL) {
			*response = ARASAN_EMMC1->RESP0;
		}
	}

	RPI0W_BLINK_LED_OFF

   return WWD_SUCCESS;

cmd_err:
#ifdef ARASAN_EMMC_POLLING
#endif /* #ifdef ARASAN_EMMC_POLLING */
#ifdef ARASAN_EMMC_ISR
#endif /* #ifdef ARASAN_EMMC_ISR */
	__arasan_emmc_reset_cmd_circuit();
	RPI0W_BLINK_LED_OFF
	_NOP(10000);

	return WWD_TIMEOUT;

data_err:
#ifdef ARASAN_EMMC_POLLING
#endif /* #ifdef ARASAN_EMMC_POLLING */
#ifdef ARASAN_EMMC_ISR
#endif /* #ifdef ARASAN_EMMC_ISR */
	__arasan_emmc_reset_data_circuit();
	RPI0W_BLINK_LED_OFF
	_NOP(10000);

	return WWD_TIMEOUT;
}

wwd_result_t host_platform_sdio_transfer(
		wwd_bus_transfer_direction_t direction,
		sdio_command_t command,
		sdio_transfer_mode_t mode,
		sdio_block_size_t block_size_t,
		uint32_t argument,
		/*@null@*/ uint32_t* data,
		uint16_t data_size,
		sdio_response_needed_t response_expected,
		/*@out@*/ /*@null@*/ uint32_t* response)
{
	wwd_result_t ret;

	pthread_mutex_lock(&__sdio_bus_mutex);
	ret = __host_platform_sdio_transfer(
			direction,
			command,
			mode,
			block_size_t,
			argument,
			data,
			data_size,
			response_expected,
			response
			);
	pthread_mutex_unlock(&__sdio_bus_mutex);

	return ret;
}
/**
 * Switch SDIO bus to high speed mode
 *
 * When SDIO starts, it must be in a low speed mode
 * This function switches it to high speed mode (up to 50MHz)
 *
 */
void host_platform_enable_high_speed_sdio( void )
{
#define SD_CLOCK_HIGH				(50000000) /* Hz */

	__arasan_emmc1_set_clock(SD_CLOCK_HIGH);
	__arasan_emmc1_4bit_mode_change_bit();
}

/**
 * Enables the bus interrupt
 *
 * This function is called by the platform during init, once
 * the system is ready to receive interrupts
 */
wwd_result_t host_platform_bus_enable_interrupt(void)
{
	arasan_emmc_card_irq_enable();

    return  WWD_SUCCESS;
}

/**
 * Disables the bus interrupt
 *
 * This function is called by the platform during de-init, to stop
 * the system supplying any more interrupts in preparation
 * for shutdown
 */
wwd_result_t host_platform_bus_disable_interrupt(void)
{
	__arasan_emmc_card_irq_disable();

    return  WWD_SUCCESS;
}

/**
 * Informs the platform bus implementation that a buffer has been freed
 *
 * This function is called by the platform during buffer release to allow
 * the platform to reuse the released buffer - especially where
 * a DMA chain needs to be refilled.
 */
void host_platform_bus_buffer_freed(wwd_buffer_dir_t direction)
{
    UNUSED_PARAMETER(direction);
}

static __inline__ void __arasan_emmc_card_irq_disable(void)
{
	ARASAN_EMMC1->IRPT_EN = ARASAN_EMMC1->IRPT_EN & (~ARASAN_EMMC_CARD_INTERRUPT);
	ARASAN_EMMC1->IRPT_MASK = ARASAN_EMMC1->IRPT_MASK & (~ARASAN_EMMC_CARD_INTERRUPT);
	ARASAN_EMMC1->INTERRUPT = ARASAN_EMMC1->INTERRUPT | ARASAN_EMMC_CARD_INTERRUPT;
}

void arasan_emmc_card_irq_enable(void)
{
	ARASAN_EMMC1->IRPT_EN = ARASAN_EMMC1->IRPT_EN | ARASAN_EMMC_CARD_INTERRUPT;
	ARASAN_EMMC1->IRPT_MASK = ARASAN_EMMC1->IRPT_MASK | ARASAN_EMMC_CARD_INTERRUPT;
}

#ifdef ARASAN_EMMC_POLLING
static int __arasan_emmc_irq_isr(int irq, void *context, void *arg)
{
	enum ds_irqreturn {
		DS_IRQ_NONE = (0 << 0),
		DS_IRQ_HANDLED = (1 << 0),
		DS_IRQ_WAKE_THREAD = (1 << 1),
	};

	UNUSED_PARAMETER(irq);
	UNUSED_PARAMETER(context);
	UNUSED_PARAMETER(arg);


	if (ARASAN_EMMC1->INTERRUPT & ARASAN_EMMC_CARD_INTERRUPT) {
		__arasan_emmc_card_irq_disable();
		wwd_thread_notify_irq();

		return DS_IRQ_HANDLED;
	}

	return DS_IRQ_HANDLED;
}
#endif /* #ifdef ARASAN_EMMC_POLLING */

#ifdef ARASAN_EMMC_ISR
static int __arasan_emmc_irq_isr(int irq, void *context, void *arg)
{
	enum ds_irqreturn {
		DS_IRQ_NONE = (0 << 0),
		DS_IRQ_HANDLED = (1 << 0),
		DS_IRQ_WAKE_THREAD = (1 << 1),
	};

	UNUSED_PARAMETER(irq);
	UNUSED_PARAMETER(context);
	UNUSED_PARAMETER(arg);

	if (ARASAN_EMMC1->INTERRUPT & ARASAN_EMMC_CARD_INTERRUPT) {
		__arasan_emmc_card_irq_disable();
		wwd_thread_notify_irq();

		return DS_IRQ_HANDLED;
	}

	__current_irtp = ARASAN_EMMC1->INTERRUPT;
	ARASAN_EMMC1->INTERRUPT = __current_irtp;

	host_rtos_set_semaphore(&__sdio_irq_semaphore, WICED_TRUE);

	return DS_IRQ_HANDLED;
}
#endif /* #ifdef ARASAN_EMMC_ISR */
