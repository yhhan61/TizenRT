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
 * examples/examples/rp0_demo/examples/spi_test.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <tinyara/spi/spi.h>

/* define SPI device structure */
static struct spi_dev_s *spi_dev;

/* SFLASH Clock Frequency */
#define SFLAH_CLOCK 	1000000
#define SFLAH_MODE 	0			/* CPOL & CPHA */

#define SFLASH_CHIP_ERASE

/* command for EN25F80 serial flash */
static const unsigned short _SERIAL_FLASH_CMD_RDID = 0x9F;	// 25P80
static const unsigned short _SERIAL_FLASH_CMD_READ = 0x03;
static const unsigned short _SERIAL_FLASH_CMD_FREAD = 0x03;
static const unsigned short _SERIAL_FLASH_CMD_WRITE = 0x02;
static const unsigned short _SERIAL_FLASH_CMD_WREN = 0x06;
static const unsigned short _SERIAL_FLASH_CMD_WRDIS = 0x04;
static const unsigned short _SERIAL_FLASH_CMD_RDSR = 0x05;
static const unsigned short _SERIAL_FLASH_CMD_ERASE = 0xC7;	// 25P80
static const unsigned short _SERIAL_FLASH_CMD_EWSR = 0x06;	// 25P80
static const unsigned short _SERIAL_FLASH_CMD_WRSR = 0x01;
static const unsigned short _SERIAL_FLASH_CMD_SER = 0xD8;	//25P80

/* simple read/write test */
static char spi_read(int port, int addr, int frequency, int bits, int conf);
static void spi_write(int port, int addr, int frequency, int bits, int conf, char value);
static void spi_read_write_test(void);

/* serial flash handling utility */
static unsigned char sflash_is_write_busy(void);
static int sflash_write_array(unsigned long addr, unsigned char *pData, int ncount);
static void sflash_read_array(unsigned long addr, unsigned char *pData, unsigned int ncount);
static unsigned char sflash_read_byte(unsigned long addr);
static void sflash_write_byte(unsigned long addr, unsigned char data);
static void sflash_sector_erase(unsigned long addr);
static void sflash_chip_erase(void);
static void sflash_reset_write_protection(void);
static void sflash_write_enable(bool enable);
static unsigned char sflash_read_status(void);
static unsigned char sflash_read_id(void);

static char spi_read(int port, int addr, int frequency, int bits, int conf)
{
	unsigned char buf[2];
	buf[0] = addr | 0x80;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, frequency);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);

	SPI_SELECT(spi_dev, port, true);
	SPI_RECVBLOCK(spi_dev, buf, 2);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	return buf[1];
}

static void spi_write(int port, int addr, int frequency, int bits, int conf, char value)
{
	unsigned char buf[2];
	buf[0] = addr;
	buf[1] = value;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, frequency);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);

	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, buf, 2);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);
}

static void spi_read_write_test(void)
{
	int port = 0;
	int freq = 1000000;
	int bits = 8;
	int conf = 0;
	char ch;
	int i;

	printf("\t TASH SPI WRITE TEST \n");
	printf("\t --------------------\n");

	for (i = 0; i < 1000; i++) {
		printf("write data 0x55 to 0x80\n");
		spi_write(port, 0x80, freq, bits, conf, 0x55);

		printf("write data 0xAA to 0x80\n");
		spi_write(port, 0x80, freq, bits, conf, 0xAA);

		ch = spi_read(port, 0x80, freq, bits, conf);
		printf("read data from 0x80 : %X\n", ch);

		ch = spi_read(port, 0x81, freq, bits, conf);
		printf("read data from 0x81 : %X\n", ch);
	}

	printf("\n");
}

static unsigned char sflash_is_write_busy(void)
{
	unsigned char status;
	status = sflash_read_status();
	return (status & 0x01);
}

static void sflash_read_array(unsigned long addr, unsigned char *pData, unsigned int ncount)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[ncount + 4];

	/* set read command */
	buf[0] = _SERIAL_FLASH_CMD_READ;

	/* set write address */
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = (addr >> 0) & 0xFF;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_RECVBLOCK(spi_dev, &buf, ncount + 4);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	memcpy(pData, &buf[4], ncount);
}

static int sflash_write_array(unsigned long addr, unsigned char *pData, int ncount)
{
	unsigned long address;
	unsigned char *pD;
	int counter;
	int ret = 0;

	printf("Serial Flash Write Array\n");

	address = addr;
	pD = pData;

	// WRITE
	for (counter = 0; counter < ncount; counter++) {
		sflash_write_byte(address++, *pD++);
	}

	// VERIFY
	for (counter = 0; counter < ncount; counter++) {
		unsigned char data = sflash_read_byte(addr);
		if (*pData != data) {
			printf("read data is wrong at 0x%08X write data(%02X), read data(%02X)\n", addr, *pData, data);
			ret = -1;
		}
		pData++;
		addr++;
	}

	return ret;
}

static unsigned char sflash_read_byte(unsigned long addr)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char data;
	unsigned char buf[10];

	/* set read command */
	buf[0] = _SERIAL_FLASH_CMD_READ;

	/* set read address */
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = (addr >> 0) & 0xFF;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_RECVBLOCK(spi_dev, &buf, 5);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	data = buf[4];
//  printf(" [%02X]\n", buf[4]);

	return data;
}

static void sflash_write_byte(unsigned long addr, unsigned char data)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[10];

	/* write enable */
	sflash_write_enable(TRUE);

	/* set write command */
	buf[0] = _SERIAL_FLASH_CMD_WRITE;

	/* set write address */
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = (addr >> 0) & 0xFF;

	/* set write data */
	buf[4] = data;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 5);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	// Wait for write end
	while (sflash_is_write_busy()) ;

	/* write disable */
	sflash_write_enable(FALSE);
}

static void sflash_sector_erase(unsigned long addr)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[10];

	sflash_write_enable(TRUE);

	/* set sector erase command */
	buf[0] = _SERIAL_FLASH_CMD_SER;

	/* set erase address */
	buf[1] = (addr >> 16) & 0xFF;
	buf[2] = (addr >> 8) & 0xFF;
	buf[3] = (addr >> 0) & 0xFF;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 4);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	// Wait for write end
	while (sflash_is_write_busy()) ;
}

static void sflash_chip_erase(void)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[10];

	sflash_write_enable(TRUE);

	buf[0] = _SERIAL_FLASH_CMD_ERASE;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 1);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	// Wait for write end
	while (sflash_is_write_busy()) ;
}

static void sflash_reset_write_protection(void)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[10];

	buf[0] = _SERIAL_FLASH_CMD_EWSR;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 1);
	SPI_SELECT(spi_dev, port, false);

	buf[0] = _SERIAL_FLASH_CMD_EWSR;
	buf[1] = 0;
	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 2);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);
}

static void sflash_write_enable(bool enable)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char buf[10];

	if (enable == TRUE) {
		buf[0] = _SERIAL_FLASH_CMD_WREN;
	} else {
		buf[0] = _SERIAL_FLASH_CMD_WRDIS;
	}

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_SNDBLOCK(spi_dev, &buf, 1);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);
}

static unsigned char sflash_read_status(void)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char status = 0;
	unsigned char buf[10];
	memset(&buf, 0xFF, 10);

	buf[0] = _SERIAL_FLASH_CMD_RDSR;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_RECVBLOCK(spi_dev, &buf, 2);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	status = buf[1];

	return status;
}

static unsigned char sflash_read_id(void)
{
	int port = 0;
	int freq = SFLAH_CLOCK;
	int bits = 8;
	int conf = SFLAH_MODE;
	unsigned char manufacturer = 0;
	unsigned short device_id = 0;
	unsigned char buf[10];
	memset(&buf, 0xff, 10);

	buf[0] = _SERIAL_FLASH_CMD_RDID;

	SPI_LOCK(spi_dev, true);

	SPI_SETFREQUENCY(spi_dev, freq);
	SPI_SETBITS(spi_dev, bits);
	SPI_SETMODE(spi_dev, conf);
	SPI_SELECT(spi_dev, port, true);
	SPI_RECVBLOCK(spi_dev, &buf, 4);
	SPI_SELECT(spi_dev, port, false);

	SPI_LOCK(spi_dev, false);

	manufacturer = buf[1];
	device_id = buf[2] << 8 | buf[3];

	printf("Serial Flash Manufacture ID : 0x%02X\n", manufacturer);
	printf("Serial Flash Device ID : 0x%04X\n", device_id);

	return manufacturer;
}

/* serial flash array read/write test data */
#define _DATA_ARRAY_SIZE 16
unsigned char write_array[_DATA_ARRAY_SIZE] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
unsigned char read_array[_DATA_ARRAY_SIZE] = { 0 };

void spi_test_main(int argc, char *argv[])
{
	int port = 0;
	int id;
	unsigned char data;
	unsigned long addr;
	unsigned long status;
	int i, k;

	spi_dev = up_spiinitialize(port);

	id = sflash_read_id();
#ifdef SFLASH_CHIP_ERASE
	printf("Serial Flash Chip Erase ...\n");
	sflash_chip_erase();
	up_mdelay(500);
#endif
	printf("Serial Flash Write test ...\n");
	addr = 0x123456;
	printf("\t - write data [0x%02X] to [0x%08X]\n", 0x55, addr);
	sflash_write_byte(addr + 0, 0x55);
	printf("\t - write data [0x%02X] to [0x%08X]\n", 0xAA, addr + 1);
	sflash_write_byte(addr + 1, 0xAA);

	printf("Serial Flash Read test ...\n");
	data = sflash_read_byte(addr);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr, data);

	data = sflash_read_byte(addr + 1);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 1, data);

	data = sflash_read_byte(addr + 2);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 2, data);
	printf("\n");

	addr = 0x1000;
	printf("Serial Flash multiple read data from [0x%08X] :\n", addr);
	for (k = 0; k < _DATA_ARRAY_SIZE; k++) {
		data = sflash_read_byte(addr + k);
		printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + k, data);
	}
	printf("\n");

	printf("Serial Flash Write/Read array test ...\n");
	status = sflash_write_array(addr, (unsigned char *)&write_array, _DATA_ARRAY_SIZE);
	if (status < 0) {
		printf("\t - Write/Read array test Fail !!!\n");
	} else {
		printf("\t - Write/Read array test Success !!!\n");
	}
	printf("\n");

	printf("Serial Flash multiple read data from [0x%08X] :\n", addr);
	for (k = 0; k < _DATA_ARRAY_SIZE; k++) {
		data = sflash_read_byte(addr + k);
		printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + k, data);
	}

	printf("Serial Flash Read data by array ...\n");
	sflash_read_array(addr, (unsigned char *)&read_array, _DATA_ARRAY_SIZE);
	printf("Serial Flash read array data at [0x%08X] : \n\t", addr);
	for (i = 0; i < _DATA_ARRAY_SIZE; i++) {
		printf(" %02X", read_array[i]);
	}
	printf("\n");

	printf("\n");
	printf("Serial Flash Test Finished !!!\n");
	printf("\n");
}

void sflash_read_main(int argc, char *argv[])
{
	int port = 0;
	unsigned char data;
	unsigned long addr;
	int i;

	spi_dev = up_spiinitialize(port);

	printf("Serial Flash Read (/dev/smart0p8) ...\n");
	addr = 0x4D000;

	data = sflash_read_byte(addr);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr, data);

	data = sflash_read_byte(addr + 1);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 1, data);

	data = sflash_read_byte(addr + 2);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 2, data);

	data = sflash_read_byte(addr + 3);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 3, data);

	data = sflash_read_byte(addr + 4);
	printf("\t - read data from [0x%08X] is [0x%02X]\n", addr + 4, data);

	printf("Serial Flash Read data by array ...\n");
	addr = 0x4D000;
	sflash_read_array(addr, (unsigned char *)&read_array, _DATA_ARRAY_SIZE);
	printf("Serial Flash read array data at [0x%08X] : \n\t", addr);
	for (i = 0; i < _DATA_ARRAY_SIZE; i++) {
		printf(" %02X", read_array[i]);
	}
	printf("\n");

	printf("\n");

}
