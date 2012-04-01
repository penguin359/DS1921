#ifndef _XBEE_H_
#define _XBEE_H_

typedef int bool;
#define TRUE	1
#define FALSE	0


#define API_START	0x7E
#define API_ESCAPE	0x7D
#define API_XOR		0x20
#define XON		0x11
#define XOFF		0x13

#define AT_API_CMD		0x08
#define AT_QUEUE_API_CMD	0x09
#define ZB_TX_API_CMD		0x10
#define ZB_TX_EXPLICIT_API_CMD	0x11
#define REMOTE_AT_API_CMD	0x17
#define CREATE_SRC_ROUTE_CMD	0x21
#define AT_RESP_API_CMD		0x88
#define MODEM_STATUS_API_CMD	0x8a
#define ZB_TX_STATUS_API_CMD	0x8b
#define ZB_RX_API_CMD		0x90
#define ZB_RX_EXPLICIT_API_CMD	0x91
#define ZB_IO_DATA_SAMPLE_API_CMD	0x92
#define XBEE_SENSOR_READ_API_CMD	0x94
#define NODE_IDENTIFICATION_API_CMD	0x95
#define REMOTE_AT_RESP_API_CMD	0x97
#define OTA_UPDATE_API_CMD	0xa0
#define ROUTE_RECORD_API_CMD	0xa1
#define MANY_TO_ONE_API_CMD	0xa3

#define DEST_HIGH_AT_CMD	"DH"
#define DEST_LOW_AT_CMD		"DL"
#define MY_NET_ADDR_AT_CMD	"MY"
#define PARENT_NET_ADDR_AT_CMD	"MP"
#define FREE_CHILD_NODES_AT_CMD	"NC"
#define SERIAL_HIGH_AT_CMD	"SH"
#define SERIAL_LOW_AT_CMD	"SL"

typedef struct {
	unsigned char addr[8];
} macAddr_t;

typedef struct {
	int fd;
	int frameId;
	char buf[128];
	size_t bufLen;
	size_t bufMaxLen;
} xbee_t;

extern const macAddr_t broadcastAddr;
extern const macAddr_t coordinatorAddr;

int sendApi(xbee_t *xbee, char *data, int len);
int sendAt(xbee_t *xbee, char *cmd, bool queue);
int sendRemoteAt(xbee_t *xbee, macAddr_t addr, char *cmd, bool queue);
int sendTx(xbee_t *xbee, macAddr_t addr, void *data, int len);
int recvApi(xbee_t *xbee);

#endif /* _XBEE_H_ */
