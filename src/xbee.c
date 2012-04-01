#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
//#include <fcntl.h>
//#include <sys/stat.h>

#include "xbee.h"


int writeChar(xbee_t *xbee, unsigned char c, int len)
{
	return write(xbee->fd, &c, sizeof(c));
}

int writeByte(xbee_t *xbee, unsigned char byte, bool escape)
{
	if(escape &&
	   (byte == API_START ||
	    byte == API_ESCAPE ||
	    byte == XON ||
	    byte == XOFF)) {
		if(writeChar(xbee, API_ESCAPE, 1) < 0)
			return -1;
		if(writeChar(xbee, byte ^ API_XOR, 1) < 0)
			return -1;
	} else {
		if(writeChar(xbee, byte, 1) < 0)
			return -1;
	}

	return 0;
}

int sendApi(xbee_t *xbee, char *data, int len)
{
	unsigned char checksum = 0;

	if(writeByte(xbee, API_START, FALSE) < 0)
		return -1;
	if(writeByte(xbee, (len >> 8) & 0xff, TRUE) < 0)
		return -1;
	if(writeByte(xbee, (len >> 0) & 0xff, TRUE) < 0)
		return -1;
	while(len-- > 0) {
		checksum += *data;
		if(writeByte(xbee, *data++, TRUE) < 0)
			return -1;
	}
	if(writeByte(xbee, 0xff - checksum, TRUE) < 0)
		return -1;

	return 0;
}

int sendApiCmd(xbee_t *xbee, int type, bool ack)
{
	if(xbee->bufMaxLen - xbee->bufLen < 2)
		return -1;

	xbee->buf[xbee->bufLen++] = type;
	if(ack) {
		xbee->buf[xbee->bufLen++] = xbee->frameId;
		xbee->frameId++;
		if(xbee->frameId <= 0 || xbee->frameId > 255)
			xbee->frameId = 1;
	} else {
		xbee->buf[xbee->bufLen++] = 0;
	}

	return 0;
}

int sendApiAddr(xbee_t *xbee, macAddr_t *addr)
{
	if(xbee->bufMaxLen - xbee->bufLen < sizeof(macAddr_t)+2)
		return -1;

	memcpy(&xbee->buf[xbee->bufLen], addr, sizeof(macAddr_t));
	xbee->bufLen += sizeof(macAddr_t);
	xbee->buf[xbee->bufLen++] = 0xff; /* 16-bit destination address */
	xbee->buf[xbee->bufLen++] = 0xfe; /* FFFE means unknown */

	return 0;
}

int sendAt(xbee_t *xbee, char *cmd, bool queue)
{
	char buf[128], *bufPtr = buf;
	int bufLen = 0;

	*bufPtr++ = AT_API_CMD; bufLen++;
	*bufPtr++ = xbee->frameId; bufLen++;
	*bufPtr++ = cmd[0]; bufLen++;
	*bufPtr++ = cmd[1]; bufLen++;
	if(queue)
		buf[0] = AT_QUEUE_API_CMD;
	xbee->frameId++;
	if(xbee->frameId <= 0 || xbee->frameId > 255)
		xbee->frameId = 1;

	return sendApi(xbee, buf, bufLen);
}

int sendRemoteAt(xbee_t *xbee, macAddr_t addr, char *cmd, bool queue)
{
	int options = 0;

	if(queue)
		options = 0x02;
	sendApiCmd(xbee, REMOTE_AT_API_CMD, TRUE);
	sendApiAddr(xbee, &addr);
	xbee->buf[xbee->bufLen++] = options;
	xbee->buf[xbee->bufLen++] = cmd[0];
	xbee->buf[xbee->bufLen++] = cmd[1];

	return sendApi(xbee, xbee->buf, xbee->bufLen);
}

int sendTx(xbee_t *xbee, macAddr_t addr, void *data, int len)
{
	char buf[128], *bufPtr = buf;
	int bufLen = 0;

	*bufPtr++ = ZB_TX_API_CMD; bufLen++;
	*bufPtr++ = xbee->frameId; bufLen++;
	xbee->frameId++;
	if(xbee->frameId <= 0 || xbee->frameId > 255)
		xbee->frameId = 1;
	memcpy(bufPtr, &addr, sizeof(addr));
	bufPtr += sizeof(addr); bufLen += sizeof(addr);
	*bufPtr++ = 0xff; bufLen++; /* 16-bit destination address */
	*bufPtr++ = 0xfe; bufLen++;
	*bufPtr++ = 0x00; bufLen++; /* broadcast radius */
	*bufPtr++ = 0x00; bufLen++; /* options */
	if(len > sizeof(buf)-bufLen)
		return -1;
	memcpy(&buf[14], data, sizeof(buf)-14);
	memcpy(bufPtr, data, len);
	bufPtr += len; bufLen += len;

	return sendApi(xbee, buf, bufLen);
}

int sendTxExplicit(xbee_t *xbee, macAddr_t addr, void *data, int len, int srcEndpoint, int dstEndpoint, int clusterId, int profileId)
{
	char buf[128], *bufPtr = buf;
	int bufLen = 0;

	*bufPtr++ = ZB_TX_EXPLICIT_API_CMD; bufLen++;
	*bufPtr++ = xbee->frameId; bufLen++;
	xbee->frameId++;
	if(xbee->frameId <= 0 || xbee->frameId > 255)
		xbee->frameId = 1;
	memcpy(bufPtr, &addr, sizeof(addr));
	bufPtr += sizeof(addr); bufLen += sizeof(addr);
	*bufPtr++ = 0xff; bufLen++; /* 16-bit destination address */
	*bufPtr++ = 0xfe; bufLen++;
	*bufPtr++ = srcEndpoint; bufLen++;
	*bufPtr++ = dstEndpoint; bufLen++;
	*bufPtr++ = (clusterId >> 8) & 0xff; bufLen++;
	*bufPtr++ = (clusterId >> 0) & 0xff; bufLen++;
	*bufPtr++ = (profileId >> 8) & 0xff; bufLen++;
	*bufPtr++ = (profileId >> 0) & 0xff; bufLen++;
	*bufPtr++ = 0x00; bufLen++; /* broadcast radius */
	*bufPtr++ = 0x00; bufLen++; /* options */
	if(len > sizeof(buf)-bufLen)
		return -1;
	memcpy(&buf[14], data, sizeof(buf)-14);
	memcpy(bufPtr, data, len);
	bufPtr += len; bufLen += len;

	return sendApi(xbee, buf, bufLen);
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
	switch(buf[0]) {
	case ZB_TX_API_CMD:
		printf("I spy a Zigbee packet transmission -> ");
		break;

	case ZB_RX_API_CMD:
		printf("I spy a Zigbee packet reception -> ");
		break;

	case AT_API_CMD:
		printf("I spy an AT command -> ");
		break;

	case AT_RESP_API_CMD:
		printf("I spy an AT command response -> ");
		break;

	default:
		/* Ignore unknown commands */
		break;
	}
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

int recvApi(xbee_t *xbee)
{
	int count;
	unsigned char c;

	if((count = read(xbee->fd, &c, 1)) < 0) {
		perror("read()");
		return -1;
	}

	if(count == 0) {
		fprintf(stderr, "End of file\n");
		exit(0);
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

const macAddr_t broadcastAddr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff }};
const macAddr_t coordinatorAddr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};
