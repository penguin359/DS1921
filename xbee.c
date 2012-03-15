#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
//#include <sys/stat.h>


typedef int bool;
#define TRUE	1
#define FALSE	0


#define API_START	0x7E
#define API_ESCAPE	0x7D
#define API_XOR		0x20
#define XON		0x11
#define XOFF		0x13

int writeChar(int fd, unsigned char c, int len)
{
	return write(fd, &c, sizeof(c));
}

int writeByte(int fd, unsigned char byte, bool escape)
{
	if(escape &&
	   (byte == API_START ||
	    byte == API_ESCAPE ||
	    byte == XON ||
	    byte == XOFF)) {
		if(writeChar(fd, API_ESCAPE, 1) < 0)
			return -1;
		if(writeChar(fd, byte ^ API_XOR, 1) < 0)
			return -1;
	} else {
		if(writeChar(fd, byte, 1) < 0)
			return -1;
	}

	return 0;
}

int sendApi(int fd, char *data, int len)
{
	unsigned char checksum = 0;

	if(writeByte(fd, API_START, FALSE) < 0)
		return -1;
	if(writeByte(fd, (len >> 8) & 0xff, TRUE) < 0)
		return -1;
	if(writeByte(fd, (len >> 0) & 0xff, TRUE) < 0)
		return -1;
	while(len-- > 0) {
		checksum += *data;
		if(writeByte(fd, *data++, TRUE) < 0)
			return -1;
	}
	if(writeByte(fd, 0xff - checksum, TRUE) < 0)
		return -1;

	return 0;
}

enum {
	WAIT_API_STATE,
	//START_API_STATE,
	LEN_MSB_API_STATE,
	LEN_LSB_API_STATE,
	DATA_API_STATE,
	CKSUM_API_STATE,
};

int recvApiState = WAIT_API_STATE;

unsigned char buf[128];
unsigned char *bufPtr;
unsigned int len = 0;
unsigned int bytesLeft = 0;
unsigned char cksum = 0;
bool escapeNextByte = FALSE;

int processApi(unsigned char *buf, int len)
{
	printf("P: '");
	while(len-- > 0) {
		if(isprint(*buf)) {
			printf("%c", *buf++);
		} else {
			printf("<%02X>", (unsigned)*buf++);
		}
	}
	printf("'\n");

	return 0;
}

int recvApi(int fd)
{
	int count;
	unsigned char c;

	if((count = read(fd, &c, 1)) < 0) {
		perror("read()");
		return -1;
	}

	if(count == 0) {
		fprintf(stderr, "End of file\n");
		exit(1);
	}

	if(c == API_START) {
		escapeNextByte = FALSE;
		recvApiState = LEN_MSB_API_STATE;
		return 0;
	} else if(c == API_ESCAPE) {
		escapeNextByte = TRUE;
		return 0;
	} else if(escapeNextByte) {
		escapeNextByte = FALSE;
		c ^= API_XOR;
	}

	switch(recvApiState) {
	case WAIT_API_STATE:
		/* happily discard bytes until I see a start of frame delimiter */
		break;

	case LEN_MSB_API_STATE:
		len = (c & 0xff) << 8;
		recvApiState = LEN_LSB_API_STATE;
		break;

	case LEN_LSB_API_STATE:
		len |= c & 0xff;
		if(len > sizeof(buf)) {
			fprintf(stderr, "API Packet too large: %u\n", len);
			recvApiState = WAIT_API_STATE;
			return -1;
		}
		cksum = 0;
		bufPtr = buf;
		bytesLeft = len;
		if(bytesLeft > 0)
			recvApiState = DATA_API_STATE;
		else
			recvApiState = CKSUM_API_STATE;
		break;

	case DATA_API_STATE:
		*bufPtr++ = c;
		cksum += c;
		bytesLeft--;
		if(bytesLeft <= 0)
			recvApiState = CKSUM_API_STATE;
		break;

	case CKSUM_API_STATE:
		cksum += c;
		recvApiState = WAIT_API_STATE;
		if(cksum != 0xff) {
			fprintf(stderr, "API Packet bad checksum: %u\n", cksum);
			return -1;
		}

		processApi(buf, len);
		return 1;
		break;
	}

	return 0;
}

int main(int argc, char **argv)
{
	int fd;

	if(argc < 2) {
		fprintf(stderr, "Usage: %s file\n", argv[0]);
		exit(1);
	}

	if((fd = open(argv[1], O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)) < 0) {
		perror("open");
		exit(1);
	}
	
	if(sendApi(fd, "hello", 5) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(fd);
		exit(1);
	}
	if(sendApi(fd, "world", 5) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(fd);
		exit(1);
	}
	if(sendApi(fd, "escape\x13me", 9) < 0) {
		fprintf(stderr, "Error sending API packet\n");
		close(fd);
		exit(1);
	}

	lseek(fd, 0L, SEEK_SET);
	while(1)
		recvApi(fd);

	close(fd);

	exit(0);
}
