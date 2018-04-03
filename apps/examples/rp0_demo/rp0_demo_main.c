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

/**
 * @file artik_demo.c
 */
#include <stdio.h>
#include <apps/shell/tash.h>

static void show_usage(FAR const char *program)
{
	printf("USAGE:\n");
	printf(" %s gpio       : gpio led on/off & button test\n", program);
	printf(" %s gy30       : i2c light sensor(gy30) test\n", program);
	printf(" %s spi        : spi simple test\n", program);
	printf(" %s sflash     : sflash read test\n", program);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int rp0_demo_main(int argc, FAR char *argv[])
#endif
{
	int ret = 0;

	switch (argc) {
	case 2:
		if(0) {
#ifdef CONFIG_IOTBUS_GPIO
		} else if (strcmp(argv[1], "gpio") == 0) {
			gpio_test_main(argc, argv);
#endif			
#ifdef CONFIG_IOTBUS_I2C
		} else if (strcmp(argv[1], "gy30") == 0) {
			gy30_main(argc, argv);
#endif
#ifdef CONFIG_IOTBUS_SPI
		} else if (strcmp(argv[1], "spi") == 0) {
			spi_test_main(argc, argv);
		} else if (strcmp(argv[1], "sflash") == 0) {
			argv[2] = 0;
			sflash_read_main(argc, argv);
#endif
		} else {
			show_usage(argv[0]);
		}
		break;
	case 3:
		if (strcmp(argv[1], "sflash") == 0) {
			sflash_read_main(argc, argv);
		}
		break;

	default:
		show_usage(argv[0]);
		break;
	}

	return ret;
}
