#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
//#include <termios.h>
#include <sys/select.h>

#include "serial.h"
#include "xbee.h"
#include "sensor.h"


xbee_t xbeeDevice;
xbee_t *xbee = &xbeeDevice;


int addNewNodeCallback(nodeIdentification_t *node)
{
	sendTime(xbee, &node->addr64);
	return 0;
}

int main(int argc, char **argv)
{
	char *file;
	int fd;
	int count;
	char buf[128];
	char line[128];
	int linePos = 0;
	struct timeval timeout;
	node_t *node;
	int type = 0;

	if(argc < 2) {
		fprintf(stderr, "Usage: %s tty\n", argv[0]);
		exit(1);
	}

	file = argv[1];

	initNodes();

	fd = openSerial(file);
	//if((fd = open("testfile", O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)) < 0) {
	//	perror("open");
	//	exit(1);
	//}

	xbee->fd = fd;
	xbee->frameId = 1;
	xbee->bufLen = 0;
	xbee->bufMaxLen = sizeof(xbee->buf);

	time_t lastTime = 0, currentTime;
	while(1) {
		time(&currentTime);
		if(currentTime >= lastTime + 10) {
			startNodeSearch();
			while((node = findNextNode()) != NULL) {
				switch(type) {
				case 0:
					sendTime(xbee, &node->addr64);
					break;

				case 1:
					queryTime(xbee, &node->addr64);
					break;

				case 2:
				case 3:
				case 4:
				case 5:
					querySensor(xbee, &node->addr64, type-1);
					break;

				default:
					type = 0;
					break;
				}
				
				type = (type+1) % 6;
			}
			if(sendAt(xbee, FREE_CHILD_NODES_AT_CMD, 0) < 0) {
				perror("sendAt()");
				close(fd);
				exit(1);
			}
			if(sendAt(xbee, "NJ", TRUE) < 0) {
				perror("sendAt()");
				close(fd);
				exit(1);
			}
#if 0
			if(write(fd, "D\n", 2) < 2) {
				perror("write()");
				//if(tcsetattr(fd, TCSANOW, &savedTermios) < 0) {
				//	perror("tcsetattr()");
				//	close(fd);
				//	exit(1);
				//}
				close(fd);
				exit(1);
			}
#endif
			lastTime = currentTime;
		}
		fd_set rfds, wfds;
		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		FD_SET(fd, &rfds);
		timeout.tv_sec = 10;
		timeout.tv_usec = 0;
		if((count = select(fd+1, &rfds, &wfds, NULL, &timeout)) < 0) {
			perror("select()");
			close(fd);
			exit(1);
		//} else if(count == 0) {
		//	printf("Nothing.\n");
		}
		if(!FD_ISSET(fd, &rfds)) {
			fprintf(stderr, "Odd, fd not set.\n");
			continue;
		}
		//printf("I see data!\n");
		while(1)
			if(recvApi(xbee) < 0)
				break;
		//printf("No more!\n");
		if(errno == EAGAIN)
			continue;
		break;
		if((count = read(fd, buf, sizeof(buf))) > 0) {
			int i = 0;
			while(linePos < sizeof(line)-1 && i < count) {
				line[linePos++] = buf[i++];
				if(line[linePos-1] == '\n') {
				}
			}
			write(1, buf, count);
		}
	}

	close(fd);

	exit(0);
}
