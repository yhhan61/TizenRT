/****************************************************************************
 * examples/usbserial/host.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
#ifdef CONFIG_USBHOST_CDCACM
#define DEFAULT_TTYDEV "/dev/ttyACM0"
#else
#define DEFAULT_TTYDEV "/dev/ttyUSB0"
#endif

#define BUFFER_SIZE    1024
#define COUNTER_NEEDED 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_ttydev = DEFAULT_TTYDEV;
static const char g_shortmsg[] = "Sure... You betcha!!\n";
static char g_iobuffer[BUFFER_SIZE];

/****************************************************************************
 * show_usage
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
	fprintf(stderr, "USAGE: %s [<ttydev>]\n", progname);
	exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int usbserial_main(int argc, char *argv[])
#endif
{
	struct termios tty;
	ssize_t nbytes;
#ifdef COUNTER_NEEDED
	int count = 0;
#endif
	int fd;
	int ret;

	/* Handle input parameters */

	if (argc > 1) {
		if (argc > 2) {
			fprintf(stderr, "Too many arguments on command line\n");
			show_usage(argv[0], 1);
		}
		g_ttydev = argv[1];
	}

	/* Open the USB serial device for blocking read/write */

	do {
		printf("main: Opening USB serial driver\n");
		fd = open(g_ttydev, O_RDWR);
		if (fd < 0) {
			printf("main: ERROR: Failed to open %s: %s\n", g_ttydev, strerror(errno));
			printf("main:        Assume not connected. Wait and try again.\n");
			printf("main:        (Control-C to terminate).\n");
			sleep(5);
		}
	} while (fd < 0);
	printf("main: Successfully opened the serial driver\n");

	/* Configure the serial port in raw mode (at least turn off echo) */

	ret = tcgetattr(fd, &tty);
	if (ret < 0) {
		printf("main: ERROR: Failed to get termios for %s: %s\n", g_ttydev, strerror(errno));
		close(fd);
		return 1;
	}

	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_oflag &= ~OPOST;
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_cflag &= ~(CSIZE | PARENB);
	tty.c_cflag |= CS8;

	ret = tcsetattr(fd, TCSANOW, &tty);
	if (ret < 0) {
		printf("main: ERROR: Failed to set termios for %s: %s\n", g_ttydev, strerror(errno));
		close(fd);
		return 1;
	}
	printf("main: Success to set termios for %s\n", g_ttydev);

	/* Wait for and/or send messages -- forever */

	for (;;) {
		count++;

		/* Test OUT messages (host-to-device) */
		printf("main: Sending %d bytes..\n", sizeof(g_shortmsg));
		nbytes = write(fd, g_shortmsg, sizeof(g_shortmsg));

		/* Test if write was successful */
		if (nbytes < 0) {
			printf("main: ERROR: Failed to write to %s: %s\n", g_ttydev, strerror(errno));
			close(fd);
			return 2;
		}
		printf("main: %ld bytes sent\n", (long)nbytes);
		sleep(1);

		/* Test IN messages (device-to-host) */
		printf("main: Reading from the serial driver\n");
		nbytes = read(fd, g_iobuffer, BUFFER_SIZE - 1);
		if (nbytes < 0) {
			printf("main: ERROR: Failed to read from %s: %s\n", g_ttydev, strerror(errno));
			close(fd);
			return 2;
		} else if (nbytes == 0) {
			printf("main: End-of-file encountered\n");
			break;
		}

		g_iobuffer[nbytes] = '\0';
		printf("main: Received %ld bytes:\n", (long)nbytes);
		printf("      \"%s\"\n", g_iobuffer);

		sleep(1);
	}

	close(fd);
	return 0;
}
