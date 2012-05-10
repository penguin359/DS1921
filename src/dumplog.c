#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
//#include <fcntl.h>
#include <unistd.h>
//#include <termios.h>
#include <sys/select.h>

#include "serial.h"


int main(int argc, char **argv)
{
	char *file;
	int fd;
	int count;
	char buf[128];
	char line[128];
	int linePos = 0;

	if(argc < 2) {
		fprintf(stderr, "Usage: %s tty\n", argv[0]);
		exit(1);
	}

	file = argv[1];

	fd = openSerial(file);

	time_t lastTime = 0, currentTime;
	while(1) {
		time(&currentTime);
		if(currentTime >= lastTime + 60) {
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
			lastTime = currentTime;
		}
		fd_set rfds, wfds;
		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		FD_SET(fd, &rfds);
		if((count = select(fd+1, &rfds, NULL, NULL, NULL)) < 0) {
			perror("select()");
			close(fd);
			exit(1);
		//} else if(count == 0) {
		//	printf("Nothing.\n");
		}
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
