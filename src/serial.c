#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <time.h>
//#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
//#include <sys/select.h>
#include <sys/stat.h>

#include "serial.h"


int openSerial(char *file)
{
	struct termios termios, savedTermios;
	struct stat stat;
	int fd;

	printf("Opening...\n");
	if((fd = open(file, O_RDWR | O_NONBLOCK)) < 0) {
		perror("open()");
		exit(1);
	}

	if(fstat(fd, &stat) < 0) {
		perror("stat()");
		close(fd);
		exit(1);
	}

	if((stat.st_mode & S_IFMT) != S_IFCHR)
		return fd;

	printf("Setting...\n");
	if(tcgetattr(fd, &termios) < 0) {
		perror("tcgetattr()");
		close(fd);
		exit(1);
	}

	memcpy(&savedTermios, &termios, sizeof(struct termios));
	cfsetspeed(&termios, B9600);
	termios.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	termios.c_cflag |= CS8 | CREAD | CLOCAL;
	termios.c_lflag &= ~(ECHO | ICANON | IEXTEN);
	termios.c_cc[VMIN] = 1;
	termios.c_cc[VTIME] = 0;

	if(tcsetattr(fd, TCSANOW, &termios) < 0) {
		perror("tcsetattr()");
		close(fd);
		exit(1);
	}

	return fd;
}
