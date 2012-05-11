#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
//#include <fcntl.h>
//#include <sys/stat.h>

#include "serial.h"
#include "xbee.h"
#include "sensor.h"


#ifdef __AVR__
typedef uint32_t time_t;
#endif


#define MAX_NODES		10
node_t nodes[MAX_NODES];

int nodeSearchIdx = 0;

void startNodeSearch(void)
{
	nodeSearchIdx = 0;
}

node_t *findNextNode(void)
{
	while(nodeSearchIdx < MAX_NODES) {
		if(nodes[nodeSearchIdx].status != UNUSED_NODE_STATUS)
			return &nodes[nodeSearchIdx++];
		nodeSearchIdx++;
	}

	return NULL;
}

int addNewNode(nodeIdentification_t *node)
{
	int freeNode = -1;
	int i;

	for(i = 0; i < MAX_NODES; i++) {
		if(nodes[i].status == UNUSED_NODE_STATUS) {
			if(freeNode < 0)
				freeNode = i;
			continue;
		}
		if(memcmp(&node->addr64, &nodes[i].addr64, sizeof(macAddr64_t)) == 0) {
			freeNode = i;
			break;
		}
	}

	if(freeNode < 0) {
		fprintf(stderr, "No free slots for XBee nodes\n");
		return -1;
	}

	nodes[freeNode].status = SENSOR_NODE_STATUS;
	memcpy(&nodes[freeNode].addr64, &node->addr64, sizeof(macAddr64_t));
	memcpy(&nodes[freeNode].addr16, &node->addr16, sizeof(macAddr16_t));
	strncpy(nodes[freeNode].identifier, node->identifier, MAX_IDENTIFIER_LEN);

	printf("New node: %s\n", node->identifier);

	return 0;
}

node_t *findNodeByAddr64(macAddr64_t *addr64)
{
	int i;

	for(i = 0; i < MAX_NODES; i++)
		if(memcmp(addr64, &nodes[i].addr64, sizeof(macAddr64_t)) == 0)
			break;
	if(i >= MAX_NODES)
		return NULL;

	return &nodes[i];
}

void initNodes(void)
{
	int i;

	memset(nodes, 0, sizeof(nodes));
	for(i = 0; i < MAX_NODES; i++)
		nodes[i].status = UNUSED_NODE_STATUS;
}

char *strMacAddr64(macAddr64_t *addr64)
{
	static char str[24];
	unsigned char *bytes = (unsigned char *)addr64;
	int i;

	for(i = 0; i < sizeof(addr64); i++)
		snprintf(str + i*3, sizeof(str) - i*3, "%02X:", bytes[i]);
	str[sizeof(str)-1] = '\0';

	return str;
}

int writeChar(xbee_t *xbee, unsigned char c, int len)
{
	//return write(xbee->fd, &c, sizeof(c));
	return writeSerial(xbee->serial, c);
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
	xbee->bufLen = 0;
	if(xbee->bufMaxLen - xbee->bufLen < 2)
		return -1;

	xbee->buf[xbee->bufLen++] = type;
	if(ack) {
		xbee->buf[xbee->bufLen++] = xbee->frameId;
		xbee->frameId++;
		//if(xbee->frameId <= 0 || xbee->frameId > 255)
		if(xbee->frameId == 0)
			xbee->frameId = 1;
	} else {
		xbee->buf[xbee->bufLen++] = 0;
	}

	return 0;
}

int sendApiAddr(xbee_t *xbee, macAddr64_t *addr64)
{
	macAddr16_t addr16 = UNKNOWN_ADDR16;
	//node_t *node;

	if(xbee->bufMaxLen - xbee->bufLen < sizeof(macAddr64_t)+2)
		return -1;

#if 0
	if((node = findNodeByAddr64(addr64)) != NULL)
		addr16 = node->addr16;
#endif

	memcpy(&xbee->buf[xbee->bufLen], addr64, sizeof(macAddr64_t));
	xbee->bufLen += sizeof(macAddr64_t);
	xbee->buf[xbee->bufLen++] = (addr16 >> 8) & 0xff;
	xbee->buf[xbee->bufLen++] = (addr16 >> 0) & 0xff;

	return 0;
}

int sendApiBuf(xbee_t *xbee, char *buf, int len)
{
	if(xbee->bufMaxLen - xbee->bufLen < len)
		return -1;

	memcpy(&xbee->buf[xbee->bufLen], buf, len);
	xbee->bufLen += len;

	return 0;
}

int sendApiBytes(xbee_t *xbee, int bytesAvail)
{
	if(xbee->bufMaxLen - xbee->bufLen < bytesAvail)
		return -1;

	return 0;
}

int sendAt(xbee_t *xbee, char *cmd, bool queue)
{
	sendApiCmd(xbee, queue ? AT_QUEUE_API_CMD : AT_API_CMD, TRUE);
	sendApiBytes(xbee, 2);
	xbee->buf[xbee->bufLen++] = cmd[0];
	xbee->buf[xbee->bufLen++] = cmd[1];

	return sendApi(xbee, xbee->buf, xbee->bufLen);
}

int sendRemoteAt(xbee_t *xbee, macAddr64_t *addr64, char *cmd, bool queue)
{
	int options = 0;

	if(queue)
		options = 0x02;
	sendApiCmd(xbee, REMOTE_AT_API_CMD, TRUE);
	sendApiAddr(xbee, addr64);
	sendApiBytes(xbee, 3);
	xbee->buf[xbee->bufLen++] = options;
	xbee->buf[xbee->bufLen++] = cmd[0];
	xbee->buf[xbee->bufLen++] = cmd[1];

	return sendApi(xbee, xbee->buf, xbee->bufLen);
}

int sendTx(xbee_t *xbee, macAddr64_t *addr64, void *data, int len)
{
	sendApiCmd(xbee, ZB_TX_API_CMD, TRUE);
	sendApiAddr(xbee, addr64);
	sendApiBytes(xbee, 2 + len);
	xbee->buf[xbee->bufLen++] = 0x00; /* broadcast radius */
	xbee->buf[xbee->bufLen++] = 0x00; /* options */
	sendApiBuf(xbee, data, len);

	return sendApi(xbee, xbee->buf, xbee->bufLen);
}

int sendTxExplicit(xbee_t *xbee, macAddr64_t *addr64, void *data, int len, int srcEndpoint, int dstEndpoint, int clusterId, int profileId)
{
	char buf[128], *bufPtr = buf;
	int bufLen = 0;

	*bufPtr++ = ZB_TX_EXPLICIT_API_CMD; bufLen++;
	*bufPtr++ = xbee->frameId; bufLen++;
	xbee->frameId++;
	//if(xbee->frameId <= 0 || xbee->frameId > 255)
	if(xbee->frameId == 0)
		xbee->frameId = 1;
	memcpy(bufPtr, addr64, sizeof(addr64));
	bufPtr += sizeof(addr64); bufLen += sizeof(addr64);
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

int addNewNodeCallback(nodeIdentification_t *node);

int processApi(unsigned char *buf, int len)
{
	int i;
	zbTxStatusResponse_t *txStatus;
	node_t		     *node;
	nodeIdentification_t *nodeId;

	switch(buf[0]) {
	case ZB_TX_API_CMD:
		printf("I spy a Zigbee packet transmission -> ");
		break;

	case ZB_RX_API_CMD:
		if(len < 12)
			break;
		switch(buf[12]) {
		case QUERY_TIME_SENSOR_CMD | 0x80:
			printf("Retrieved time: %u\n", *(unsigned int *)&buf[13]);
			return 0;
			break;

		case QUERY_SENSOR_SENSOR_CMD | 0x80:
			{
				struct querySensorResponse *sensor =
				    (struct querySensorResponse *)&buf[12];
				int num = sensor->sensor;
				int type = sensor->type;
				time_t time = sensor->time;
				int val = sensor->data16[0];
				int val2 = sensor->data16[1];

				node = findNodeByAddr64((macAddr64_t *)&buf[1]);
				if(node != NULL) {
					if(type == 3)
						printf("Sensor(%d) on %s is %.2f°F and %.2f%% humidity @ %lu\n", num, node->identifier, ((double)val / (double)(1 << 14) * 165. - 40.) * 9./5. + 32., (double)val2 / (double)(1 << 14) * 100., time);
					else if(type == 6)
						printf("Sensor(%d) on %s is at %.2f%% of light @ %lu\n", num, node->identifier, (double)val / (double)(1 << 15) * 100., time);
					else
						printf("Sensor(%d) on %s is %.2f°F @ %lu\n", num, node->identifier, (double)(val - (273 << 4)) * 0.0625 * 9./5. + 32., time);
				}
				//temp = (double)val / (double)(1 << 14) * 165. - 40.;
			}
			return 0;
			break;

		case DEBUG_SENSOR_CMD:
			printf("Debug: '");
			fflush(stdout);
			buf += 13;
			len -= 13;
			while(len-- > 0) {
				if(isprint(*buf)) {
					printf("%c", *buf++);
				} else {
					printf("<%02X>", (unsigned)*buf++);
				}
			}
			printf("'\n");
			return 0;
			break;

		default:
			printf("Unknown command: 0x%x\n", buf[12]);
		}
		if(len == 16) {
			printf("Retrieved time: %u\n", *(unsigned int *)&buf[12]);
			return 0;
		}
		printf("I spy a Zigbee packet reception -> ");
		for(i = 12; i < len; i++)
			fputc(buf[i], stdout);
		fputc('\n', stdout);
		return 0;
		break;

	case AT_API_CMD:
		printf("I spy an AT command -> ");
		break;

	case AT_RESP_API_CMD:
		printf("I spy an AT command response -> ");
		break;

	case ZB_TX_STATUS_API_CMD:
		txStatus = (zbTxStatusResponse_t *)buf;
		printf("TX Status report: ");
		if(txStatus->deliveryStatus != ZB_DELIVERY_STATUS_SUCCESS) {
			printf("Failed to transmit packet: 0x%x\n", txStatus->deliveryStatus);
		} else {
			printf("Success with %d retries.\n", txStatus->retries);
		}
		return 0;
		break;

	case NODE_IDENTIFICATION_API_CMD:
		nodeId = (nodeIdentification_t *)buf;
		//printf("I spy a node identification -> %s\n", nodeId->identifier); //(char *)&buf[22]);
		addNewNode(nodeId);
		addNewNodeCallback(nodeId);
		return 0;
		break;

	default:
		/* Ignore unknown commands */
		printf("Unknown API Packet: 0x%x -> ", buf[0]);
		break;
	}
	buf++;
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
	int c;

	if((c = readSerial(xbee->serial)) < 0)
		return -1;
	//printf("C:'%c' (0x%02x)\n", c, c);

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

const macAddr64_t broadcastAddr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff }};
const macAddr64_t coordinatorAddr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};
