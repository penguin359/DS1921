#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
//#include <sys/select.h>
#include <sys/stat.h>

#include "serial.h"


int readSerial(serial_t *serial)
{
	unsigned char c;
	int count;
	int savedErrno;

	if((count = read(serial->fd, &c, 1)) < 0) {
#ifndef __AVR__
		savedErrno = errno;
		if(errno != EAGAIN)
			perror("read()");
		errno = savedErrno;
#endif
		return -1;
	}

	if(count == 0) {
		fprintf(stderr, "End of file\n");
		exit(0);
	}

	return c;
}

int writeSerial(serial_t *serial, unsigned char c)
{
	return write(serial->fd, &c, sizeof(c));
}

int closeSerial(serial_t *serial)
{
	if(serial->type == CHAR_SERIAL_TYPE)
		tcsetattr(serial->fd, TCSANOW, &serial->savedTermios);
	close(serial->fd);
	free(serial);

	return 0;
}

serial_t *openSerial(char *file)
{
	serial_t *serial;
	struct termios termios;
	struct stat stat;
	int fd;
	int savedErrno;

	if((serial = malloc(sizeof(*serial))) == NULL) {
		savedErrno = errno;
		perror("malloc()");
		errno = savedErrno;
		return NULL;
	}
	memset(serial, 0UL, sizeof(*serial));

	printf("Opening...\n");
	if((fd = open(file, O_RDWR | O_NONBLOCK)) < 0) {
		savedErrno = errno;
		perror("open()");
		free(serial);
		errno = savedErrno;
		return NULL;
	}

	if(fstat(fd, &stat) < 0) {
		savedErrno = errno;
		perror("stat()");
		close(fd);
		free(serial);
		errno = savedErrno;
		return NULL;
	}

	serial->fd = fd;
	serial->type = FILE_SERIAL_TYPE;

	if((stat.st_mode & S_IFMT) != S_IFCHR)
		return serial;

	serial->type = CHAR_SERIAL_TYPE;

	printf("Setting...\n");
	if(tcgetattr(fd, &termios) < 0) {
		savedErrno = errno;
		perror("tcgetattr()");
		close(fd);
		free(serial);
		errno = savedErrno;
		return NULL;
	}

	memcpy(&serial->savedTermios, &termios, sizeof(struct termios));
	cfsetspeed(&termios, B115200);
	termios.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	termios.c_cflag |= CS8 | CREAD | CLOCAL;
	termios.c_lflag &= ~(ECHO | ICANON | IEXTEN);
	termios.c_cc[VMIN] = 1;
	termios.c_cc[VTIME] = 0;

	if(tcsetattr(fd, TCSANOW, &termios) < 0) {
		savedErrno = errno;
		perror("tcsetattr()");
		tcsetattr(fd, TCSANOW, &serial->savedTermios);
		close(fd);
		free(serial);
		errno = savedErrno;
		return NULL;
	}

	return serial;
}
