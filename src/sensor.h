#ifndef _SENSOR_H_
#define _SENSOR_H_

#define QUERY_TIME_SENSOR_CMD	1
#define SEND_TIME_SENSOR_CMD	2
#define QUERY_SENSOR_SENSOR_CMD	3

int queryTime(xbee_t *xbee, macAddr64_t *addr64);
int sendTime(xbee_t *xbee, macAddr64_t *addr64);
int querySensor(xbee_t *xbee, macAddr64_t *addr64, int sensor);

#endif /* _SENSOR_H_ */
