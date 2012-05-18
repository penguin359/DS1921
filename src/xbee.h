#ifndef _XBEE_H_
#define _XBEE_H_

#include <stdint.h>

#include "serial.h"


#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif


#ifndef __cplusplus
typedef int bool;
#endif
#define TRUE				1
#define FALSE				0


#define API_START			0x7E
#define API_ESCAPE			0x7D
#define API_XOR				0x20
#define XON				0x11
#define XOFF				0x13

#define AT_API_CMD			0x08
#define AT_QUEUE_API_CMD		0x09
#define ZB_TX_API_CMD			0x10
#define ZB_TX_EXPLICIT_API_CMD		0x11
#define REMOTE_AT_API_CMD		0x17
#define CREATE_SRC_ROUTE_CMD		0x21
#define AT_RESP_API_CMD			0x88
#define MODEM_STATUS_API_CMD		0x8a
#define ZB_TX_STATUS_API_CMD		0x8b
#define ZB_RX_API_CMD			0x90
#define ZB_RX_EXPLICIT_API_CMD		0x91
#define ZB_IO_DATA_SAMPLE_API_CMD	0x92
#define XBEE_SENSOR_READ_API_CMD	0x94
#define NODE_IDENTIFICATION_API_CMD	0x95
#define REMOTE_AT_RESP_API_CMD		0x97
#define OTA_UPDATE_API_CMD		0xa0
#define ROUTE_RECORD_API_CMD		0xa1
#define MANY_TO_ONE_API_CMD		0xa3

#define DEST_HIGH_AT_CMD		"DH"
#define DEST_LOW_AT_CMD			"DL"
#define MY_NET_ADDR_AT_CMD		"MY"
#define PARENT_NET_ADDR_AT_CMD		"MP"
#define FREE_CHILD_NODES_AT_CMD		"NC"
#define SERIAL_HIGH_AT_CMD		"SH"
#define SERIAL_LOW_AT_CMD		"SL"


#define UNKNOWN_ADDR16			0xfffe

typedef struct {
	uint8_t addr[8];
} macAddr64_t;

typedef uint16_t macAddr16_t;

typedef enum {
	UNUSED_NODE_STATUS,
	SENSOR_NODE_STATUS,
} nodeStatus_t;

#define MAX_IDENTIFIER_LEN		20

typedef struct {
	nodeStatus_t	status;
	macAddr64_t	addr64;
	macAddr16_t	addr16;
	char		identifier[MAX_IDENTIFIER_LEN];
} node_t;

typedef struct {
	serial_t *serial;
	uint8_t frameId;
	char buf[128];
	size_t bufLen;
	size_t bufMaxLen;
} xbee_t;

typedef struct {
	uint8_t		type;
	uint8_t		frameId;
	macAddr16_t	addr16;
	uint8_t		retries;
	uint8_t		deliveryStatus;
	uint8_t		discoveryStatus;
} __attribute((packed)) zbTxStatusResponse_t;

#define ZB_DELIVERY_STATUS_SUCCESS	0x00

typedef struct {
	uint8_t		type;
	macAddr64_t	addr64;
	macAddr16_t	addr16;
	uint8_t		options;
	macAddr16_t	net16;
	macAddr64_t	net64;
	char		identifier[0];
} __attribute((packed)) nodeIdentification_t;

extern const macAddr64_t broadcastAddr;
extern const macAddr64_t coordinatorAddr;

void initNodes(void);

void startNodeSearch(void);
node_t *findNextNode(void);

char *strMacAddr64(macAddr64_t *addr64);

int sendApi(xbee_t *xbee, char *data, int len);
int sendAt(xbee_t *xbee, char *cmd, bool queue);
int sendRemoteAt(xbee_t *xbee, macAddr64_t *addr64, char *cmd, bool queue);
int sendTx(xbee_t *xbee, macAddr64_t *addr64, void *data, int len);
int recvApi(xbee_t *xbee);


int addNewNodeCallback(nodeIdentification_t *node);


#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif
#endif /* _XBEE_H_ */
