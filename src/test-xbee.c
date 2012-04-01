//#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
//#include <sys/stat.h>

#include "xbee.h"


int main(int argc, char **argv)
{
	xbee_t xbeeDevice;
	xbee_t *xbee = &xbeeDevice;
	macAddr_t addr = { { 1, 2, 3, 4, 5, 6, 7, 8 } };
	char *str = "Hello, World!";

	if(argc < 2) {
		fprintf(stderr, "Usage: %s file\n", argv[0]);
		exit(1);
	}

	if((xbee->fd = open(argv[1], O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)) < 0) {
		perror("open");
		exit(1);
	}
	xbee->frameId = 1;
	xbee->bufLen = 0;
	xbee->bufMaxLen = sizeof(xbee->buf);

	if(sendApi(xbee, "hello", 5) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendApi(xbee, "world", 5) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendApi(xbee, "escape\x13me", 9) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendAt(xbee, "NJ", TRUE) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendAt(xbee, "BD", FALSE) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendTx(xbee, addr, str, strlen(str)) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}
	if(sendRemoteAt(xbee, addr, "AO", TRUE) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(xbee->fd);
		exit(1);
	}

	lseek(xbee->fd, 0L, SEEK_SET);
	while(1)
		recvApi(xbee);

	close(xbee->fd);

	exit(0);
}
