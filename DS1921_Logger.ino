#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <Wire.h>

#include "DS1921_Logger.h"

/* OneWire DS18S20, DS18B20, DS1822 Temperature Example
 *
 * http://www.pjrc.com/teensy/td_libs_OneWire.html
 *
 * The DallasTemperature library can do all this work for you!
 * http://milesburton.com/Dallas_Temperature_Control_Library
 */


//#define ARDUINO_UNO

//#define DEBUG_XBEE
#define DEBUG_SENSOR
//#define TIMING_DEBUG

#ifdef ARDUINO_UNO
#include <SoftwareSerial.h>
#endif

#ifndef DEBUG_XBEE
#include <XBee.h>
#endif

#define SELF_POWERED			1

#define ONE_WIRE_SENSORS

#ifdef ONE_WIRE_SENSORS
#include <OneWire.h>
#endif

#define LED_NOTIFICATION
#define PIEZO_NOTIFICATION


#define DS18B20_CONVERT_TEMP		0x44
#define DS18B20_COPY_SCRATCHPAD		0x48
#define DS18B20_READ_SCRATCHPAD		0xBE
#define DS18B20_WRITE_SCRATCHPAD	0x4E
#define DS18B20_RECALL_EE		0xB8
#define DS18B20_READ_POWER_SUPPLY	0xB4

#define DS1921_CONVERT_TEMP		0x44
#define DS1921_COPY_SCRATCHPAD		0x55
#define DS1921_READ_SCRATCHPAD		0xAA
#define DS1921_WRITE_SCRATCHPAD		0x0F
#define DS1921_CLEAR_MEMORY		0x3C
#define DS1921_READ_MEMORY		0xF0

#define DS1921_RTC_REGISTER		0x0200
#define DS1921_RTC_ALARM_REGISTER	0x0207
#define DS1921_CONTROL_REGISTER		0x020E
#define DS1921_STATUS_REGISTER		0x0214
#define DS1921_SAMPLE_REGISTER		0x020D
#define DS1921_DEVICE_SAMPLES_COUNTER	0x021D
#define DS1921_MISSION_SAMPLES_COUNTER	0x021A
#define DS1921_MISSION_TIMESTAMP	0x0215
#define DS1921_MISSION_START_DELAY	0x0212
#define DS1921_TEMPERATURE		0x0211
#define DS1921_TEMPERATURE_LOW_ALARM	0x020B
#define DS1921_TEMPERATURE_HIGH_ALARM	0x020C
#define DS1921_TEMP_HISTOGRAM		0x0800
#define DS1921_DATA_LOG			0x1000

/*
* Device Samples Counter: 24-bit unsigned little-endian
* Mission Samples Counter: 24-bit unsigned little-endian
* Mission Timestamp:
*   BCD minutes
*   BCD hours (24-hour)
*   BCD day of month
*   BCD year without century
*/


#define DS1921_CONTROL_nEOSC		0x80
#define DS1921_CONTROL_EMCLR		0x40
#define DS1921_CONTROL_nEM		0x10
#define DS1921_CONTROL_RO		0x08
#define DS1921_CONTROL_TLS		0x04
#define DS1921_CONTROL_THS		0x02
#define DS1921_CONTROL_TAS		0x01

#define DS1921_STATUS_nTCB		0x80
#define DS1921_STATUS_MEMCLR		0x40
#define DS1921_STATUS_MIP		0x20
#define DS1921_STATUS_SIP		0x10
#define DS1921_STATUS_TLF		0x04
#define DS1921_STATUS_THF		0x02
#define DS1921_STATUS_TAF		0x01


#ifdef LED_NOTIFICATION
//#ifdef CORE_TEENSY
//#define LED_PIN				11
//#else
#define LED_PIN				LED_BUILTIN
//#endif
#define ledOn()				digitalWrite(LED_PIN, HIGH)
#define ledOff()			digitalWrite(LED_PIN, LOW)
#endif

#ifdef PIEZO_NOTIFICATION
#define PIEZO_PIN			12
#define piezoOn()			digitalWrite(PIEZO_PIN, LOW)
#define piezoOff()			digitalWrite(PIEZO_PIN, HIGH)
#endif


#ifdef CORE_TEENSY
#define AREF_MV				5000.
#else
#ifdef ARDUINO_UNO
#define AREF_MV				5000.
#else
#define AREF_MV				3300.
#endif
#endif


#ifdef ARDUINO_UNO
#define XBEE_RX_PIN			2
#define XBEE_TX_PIN			3
SoftwareSerial xbeeSerial2 = SoftwareSerial(XBEE_RX_PIN, XBEE_TX_PIN);
#endif


#ifdef ONE_WIRE_SENSORS
OneWire ds(0);
#endif

class Dummy : public Stream {
    public:
	virtual int available(void) { return 0; }
	virtual int read(void) { return 0; }
	virtual int peek(void) { return 0; }
	virtual void flush(void) {}
	virtual size_t write(uint8_t val) { return 0; }
};

#ifdef DEBUG_XBEE
HardwareSerial uart = HardwareSerial();
#endif
//#define debug Uart
#if defined(CORE_TEENSY) || defined(ARDUINO_UNO)
#define debug Serial
#else
Dummy Dummy;
#define debug Dummy
#endif

#ifndef DEBUG_XBEE
XBee xbee = XBee();
//XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
XBeeAddress64 coordinator = XBeeAddress64(0x0, 0x0);
uint8_t payload[] = { 0x04, 'H', 'i' };
uint8_t timePayload[] = { 0x81, 0, 0, 0, 0 };
//uint8_t sensorPayload[] = { 'S', 'e', 'n', 's', 'o', 'r', '-' };
uint8_t sensorPayload[] = { 0x83, 0, 0x01, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

uint32_t clock;
//elapsedMillis clockTick;
unsigned long clockTick;
unsigned long sensorTick;

//#define D0 5
//#define D1 6

#if 0
void debugPrint(char *str)
{
	ZBTxRequest zbTx = ZBTxRequest(coordinator, str, strlen(str));
	//zbTx.setPayload(payload);
	//zbTx.setPayloadLength(sizeof(payload));
	xbee.send(zbTx);
}
#endif

class SensorDebug : public Stream {
    private:
	uint8_t debugPayload[110];
	unsigned int offset;
#ifndef DEBUG_XBEE
	ZBTxRequest zbTx;
#endif

    public:
	SensorDebug() : Stream() {
		debugPayload[0] = 0x04;
		offset = 1U;
#ifndef DEBUG_XBEE
		zbTx = ZBTxRequest(coordinator, debugPayload, offset);
#endif
	}

	virtual int available(void) { return 0; }
	virtual int read(void) { return 0; }
	virtual int peek(void) { return 0; }
	virtual void flush(void) {
#ifndef DEBUG_XBEE
		zbTx.setPayloadLength(offset);
		xbee.send(zbTx);
#endif
		offset = 1U;
	}

	virtual size_t write(uint8_t val) {
		return write(&val, sizeof(val));
	}

	virtual size_t write(uint8_t *val, size_t len) {
		unsigned int i;

		for(i = 0U; offset < sizeof(debugPayload) && i < len; offset++, i++) {
			if(val[i] == '\r') {
				offset--;
				continue;
			} else if(val[i] == '\n') {
				i++;
				flush();
				return len - i;
			}
			debugPayload[offset] = val[i];
		}
		if(offset > 60U)
			flush();

		return len - i;
	}
};

#ifdef CORE_TEENSY
#define testDebug Serial
#else
SensorDebug testDebug = SensorDebug();
#endif

#ifdef ONE_WIRE_SENSORS
void writeDS18B20(byte *addr, byte high, byte low, byte config)
{
	ds.reset();
	ds.select(addr);
	ds.write(DS18B20_WRITE_SCRATCHPAD);
	ds.write(high);
	ds.write(low);
	ds.write(config);

	ds.reset();
	ds.select(addr);
	ds.write(DS18B20_READ_SCRATCHPAD);
	ds.read();
	ds.read();
	if(ds.read() != high ||
	   ds.read() != low ||
	   ds.read() != config) {
		/* TODO: record failure to configure */
		return;
	}

	ds.reset();
	ds.select(addr);
	ds.write(DS18B20_COPY_SCRATCHPAD);
#ifdef SELF_POWERED
	/* Wait for finished writing */
	while(!ds.read_bit())
		;
#else
	/* TODO: enable strong pull-up */
	delay(10);
	/* TODO: disable strong pull-up */
#endif
	/* TODO: replace delay with better solution */
}

void writeDS1921(byte *addr, int target, byte *data, int len)
{
	byte status;

	ds.reset();
	ds.select(addr);
	ds.write(DS1921_WRITE_SCRATCHPAD);
	ds.write(target & 0x00FF);
	ds.write(target >> 8 & 0x00FF);
	while(len-- > 0)
		ds.write(*data++);
	/* TODO: write til end of page for CRC-16 */

	ds.reset();
	ds.select(addr);
	ds.write(DS1921_READ_SCRATCHPAD);
	target  = (int)ds.read();
	target |= (int)ds.read() << 8;
	status = ds.read();
	/* TODO: check AA and PF flags */
	/* TODO: read back and verify data */
	/* TODO: read til end of page for CRC-16 */

	ds.reset();
	ds.select(addr);
	ds.write(DS1921_COPY_SCRATCHPAD);
	ds.write(target & 0x00FF);
	ds.write(target >> 8 & 0x00FF);
	ds.write(status);
	delay(1);
	/* TODO: replace delay with better solution */
}

void readDS1921(byte *addr, int target, byte *data, int len)
{
	ds.reset();
	ds.select(addr);
	ds.write(DS1921_READ_MEMORY);
	ds.write(target & 0x00FF);
	ds.write(target >> 8 & 0x00FF);
	while(len-- > 0)
		*data++ = ds.read();
}

void clearDS1921(byte *addr)
{
	byte control;

	debug.println("  Clearing memory...");
	readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
	if(control & DS1921_STATUS_MEMCLR)
		debug.println("  Memory previously cleared.");

	readDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

	debug.print("  Control = ");
	debug.println(control, HEX);
	control |= DS1921_CONTROL_EMCLR;
	control |= DS1921_CONTROL_RO;

	writeDS1921(addr, DS1921_CONTROL_REGISTER, &control, sizeof(control));

	ds.reset();
	ds.select(addr);
	ds.write(DS1921_CLEAR_MEMORY);
	delayMicroseconds(550);

	readDS1921(addr, DS1921_STATUS_REGISTER, &control, sizeof(control));
	debug.print("  Status = ");
	debug.println(control, HEX);
	if(control & DS1921_STATUS_MEMCLR)
		debug.println("  Memory cleared successfully.");
	else
		debug.println("  Failed to clear memory!");
}

void stopMission(byte *addr)
{
	byte zero = 0;
	writeDS1921(addr, DS1921_TEMPERATURE, &zero, sizeof(zero));
}
#endif

enum {
	HOME_STATE,
	D_STATE,
	C_STATE,
	R_STATE,
	RT_STATE,
	RTC_STATE,
};

byte addr[8];
byte buf[11];
size_t bufCount = 0U;
long val;

#ifdef ONE_WIRE_SENSORS
void parseSerial(char c)
{
	static int state = HOME_STATE;
	byte rtcBuf[7];

	int i;

	debug.print("S=");
	debug.print(state, DEC);
	debug.print(": ");
	debug.println(c);
	switch(state) {
	case HOME_STATE:
		if(c == 'D') {
			state = D_STATE;
			break;
		} else if(c == 'C') {
			state = C_STATE;
			bufCount = 0U;
			val = 0;
			break;
		} else if(c == 'R') {
			state = R_STATE;
			break;
		}

		state = HOME_STATE;
		break;

	case D_STATE:
		if(c == '\n') {
			debug.println("STARTLOG");
			readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtcBuf, 5);
			debug.print("Time=20");
			debug.print(rtcBuf[4], HEX);
			debug.print("-");
			debug.print(rtcBuf[3], HEX);
			debug.print("-");
			debug.print(rtcBuf[2], HEX);
			debug.print("T");
			debug.print(rtcBuf[1], HEX);
			debug.print(":");
			debug.print(rtcBuf[0], HEX);
			debug.println(":00Z");

			long count = 0;
			byte countBytes[3];
			readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
			count = (long)countBytes[0] << 0 | (long)countBytes[1] << 8 | (long)countBytes[2] << 16;
			debug.print("Count=");
			debug.println(count, DEC);

			readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
			debug.println("LOG");
			if(count > 2048)
				count = 2048;
			while(count-- > 0) {
				debug.print("    ");
				debug.println(ds.read(), DEC);
			}
			debug.println("ENDLOG");
			state = HOME_STATE;
			break;
		}

		state = HOME_STATE;
		break;

	case C_STATE:
		if(isdigit(c) && bufCount < sizeof(buf)-1) {
			val = val*10 + c - '0';
			bufCount++;
			//buf[bufCount] = c;
			break;
		}

		clock = val;
		debug.print("Time set to ");
		debug.println(clock, DEC);
		state = HOME_STATE;
		break;

	case R_STATE:
		if(c == 'T') {
			state = RT_STATE;
			break;
		}

		state = HOME_STATE;
		break;

	case RT_STATE:
		if(c == 'C') {
			state = RTC_STATE;
			break;
		}

		state = HOME_STATE;
		break;

	case RTC_STATE:
		if(c == '\n') {
			debug.println("rtc time is ...");
			readDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
			debug.print("20");
			debug.print((unsigned char)rtcBuf[6], HEX);
			debug.print("-");
			debug.print((unsigned char)rtcBuf[5] & ~0x80, HEX);
			debug.print("-");
			debug.print((unsigned char)rtcBuf[4], HEX);
			debug.print("T");
			debug.print((unsigned char)rtcBuf[2], HEX);
			debug.print(":");
			debug.print((unsigned char)rtcBuf[1], HEX);
			debug.print(":");
			debug.print((unsigned char)rtcBuf[0], HEX);
			debug.println("Z");
			state = HOME_STATE;
			break;
		} else {
			memset(rtcBuf, 0, sizeof(rtcBuf));
			/* discard 2 */
			while(!debug.available())
				;
			c = debug.read();
			/* discard 0 */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[6] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[6] |= (c & ~0x30);
			while(!debug.available())
				;
			c = debug.read();
			/* discard - */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[5] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[5] |= (c & ~0x30);
			rtcBuf[5] |= 0x80;
			while(!debug.available())
				;
			c = debug.read();
			/* discard - */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[4] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[4] |= (c & ~0x30);
			while(!debug.available())
				;
			c = debug.read();
			/* discard T */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[2] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[2] |= (c & ~0x30);
			while(!debug.available())
				;
			c = debug.read();
			/* discard : */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[1] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[1] |= (c & ~0x30);
			while(!debug.available())
				;
			c = debug.read();
			/* discard : */
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[0] = (c & ~0x30) << 4;
			while(!debug.available())
				;
			c = debug.read();
			rtcBuf[0] |= (c & ~0x30);
			while(!debug.available())
				;
			c = debug.read();
			/* discard '\n' */
			rtcBuf[3] = 1; /* day of week (1-7) */
			debug.print("  Set RTC = 20");
			for(i = 6; i >= 0; i--) {
				debug.print(rtcBuf[i], HEX);
				debug.print(" ");
			}
			debug.println("");
			debug.println("  Updating RTC...");
			writeDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
		}

		state = HOME_STATE;
		break;

	default:
		state = HOME_STATE;
		break;
	}
}

/* 2011-12-25T05:22:40 Sun */
byte rtc[] = {
	0x40,	  /* BCD seconds */
	0x22,	  /* BCD minutes */
	0x05,	  /* BCD hours (24-hour) */
	0x01,	  /* day of week (1-7) */
	0x25,	  /* BCD day of month */
	0x92,	  /* century bit + BCD month */
	0x11,	  /* BCD year without century */
};

int writeRtc = 0;
#endif

void printTemp(temp_t celsius)
{
	static int alarm = 0;
	float fahrenheit;

	if(celsius < 8. || celsius > 58.) {
		testDebug.println("ALARM!");
		alarm = 1;
#ifdef LED_NOTIFICATION
		ledOn();
	} else {
		ledOff();
#endif
	}
	fahrenheit = celsius * 1.8 + 32.0;
	//testDebug.print("  ATemperature = ");
	testDebug.print("  T=");
	testDebug.print(celsius);
	//testDebug.print(" Celsius, ");
	testDebug.print("C, ");
	testDebug.print(fahrenheit);
	//testDebug.println(" Fahrenheit");
	testDebug.println("F");
}



sensor_t sensors[32];


/* Analog sensor support */

#define ANALOG_BASE_IDX		0

sensorState_t readAnalogSensor(sensor_t *sensor)
{
	if(sensor->type != ANALOG_SENSOR_TYPE &&
	   sensor->type != LIGHT_SENSOR_TYPE)
		return ERROR_SENSOR_STATE;

	long val = analogRead(sensor->addr);
	if(sensor->type == LIGHT_SENSOR_TYPE)
		sensor->data16[0] = val << 5 | (val >> 5);
	else
		sensor->data16[0] = val * AREF_MV * 16 / 1023 / 10 + (CELSIUS_TO_KELVIN << 4) - (50 << 4);
#ifdef DEBUG_SENSOR
	temp_t temp = ((float)val / 1023. * AREF_MV - 500.)/10.;
	testDebug.print("ADC Val: ");
	testDebug.println(val, DEC);
	if(sensor->type == LIGHT_SENSOR_TYPE) {
		testDebug.print("  L=");
		testDebug.print((double)val * 100. / 1023.);
		testDebug.println("%");
	} else {
		printTemp(temp);
	}
#endif

	return COMPLETED_SENSOR_STATE;
}

void analogSensorInit(void)
{
	const int sensorPin[] = {
		A0,
		A1,
		A2,
		A3,
#ifdef CORE_TEENSY
		A4,
		A5,
#else
		A6,
		A7,
#endif
	};

	for(int8_t i = sizeof(sensorPin)/sizeof(sensorPin[0])-1; i >= 0; i--) {
		uint8_t addr = sensorPin[i];
		pinMode(addr, INPUT);
		digitalWrite(addr, LOW);
		uint16_t val = analogRead(addr);
		if(val < 5)
			continue;
		sensor_t *sensor = &sensors[i+ANALOG_BASE_IDX];
		sensor->type = ANALOG_SENSOR_TYPE;
		if(addr == A2)
			sensor->type = LIGHT_SENSOR_TYPE;
		sensor->addr = addr;
	}
}


/* LM75 Sensor support */

#define LM75_BASE_IDX			8
#define LM75_NUM_SENSORS		8
#define LM75_BASE_ADDR			0x48

#define LM75_TEMP_REGISTER		0
#define LM75_CONFIG_REGISTER		1
#define LM75_THYST_REGISTER		2
#define LM75_TSET_REGISTER		3

#define LM75_CONFIG_ONE_SHOT		0x80

#define LM75_CONFIG_9BIT_RESOLUTION	0x00
#define LM75_CONFIG_10BIT_RESOLUTION	0x20
#define LM75_CONFIG_11BIT_RESOLUTION	0x40
#define LM75_CONFIG_12BIT_RESOLUTION	0x60
#define LM75_CONFIG_RESOLUTION_MASK	0x60

#define LM75_CONFIG_1_FAULT_QUEUE	0x00
#define LM75_CONFIG_2_FAULT_QUEUE	0x08
#define LM75_CONFIG_4_FAULT_QUEUE	0x10
#define LM75_CONFIG_6_FAULT_QUEUE	0x18
#define LM75_CONFIG_FAULT_QUEUE_MASK	0x18

#define LM75_CONFIG_ALERT_ACTIVE_HIGH	0x04
#define LM75_CONFIG_ALERT_INTERRUPT	0x02
#define LM75_CONFIG_SHUTDOWN		0x01

sensorState_t readLM75Sensor(sensor_t *sensor)
{
	uint8_t addr = sensor->addr;
	uint8_t config = 0;
	uint8_t val;
#ifdef DEBUG_SENSOR
	temp_t temp;
#endif

	if(sensor->type != LM75_SENSOR_TYPE)
		return ERROR_SENSOR_STATE;

	switch(sensor->state) {
	case START_SENSOR_STATE:
		Wire.beginTransmission(addr);
		Wire.write(LM75_CONFIG_REGISTER);
		Wire.endTransmission();
		Wire.requestFrom(addr, (uint8_t)1);
		config = Wire.read();
#ifdef DEBUG_SENSOR
		testDebug.print("LM75 Start Conf: 0x");
		testDebug.println(config, HEX);
#endif
		config |= LM75_CONFIG_ONE_SHOT;
		Wire.beginTransmission(addr);
		Wire.write(LM75_CONFIG_REGISTER);
		Wire.write(config);
		Wire.endTransmission();
		sensor->waitTime = millis() + 200UL;
		return WAIT_SENSOR_STATE;
		break;

	case READ_SENSOR_STATE:
		Wire.requestFrom(addr, (uint8_t)1);
		config = Wire.read();
#ifdef DEBUG_SENSOR
		testDebug.print("LM75 Read Conf: 0x");
		testDebug.println(config, HEX);
#endif
		if(config & LM75_CONFIG_ONE_SHOT)
			return READ_SENSOR_STATE;
		Wire.beginTransmission(addr);
		Wire.write(LM75_TEMP_REGISTER);
		Wire.endTransmission();
		Wire.requestFrom(addr, (uint8_t)2);
		val = Wire.read() << 8;
		val |= Wire.read();
		val >>= 4;
		sensor->data16[0] = val + (CELSIUS_TO_KELVIN << 4);
#ifdef DEBUG_SENSOR
		temp = (temp_t)val * 0.0625f;
		testDebug.print("I2C Val: ");
		testDebug.println(val, DEC);
		printTemp(temp);
		break;
#endif

	default:
		break;
	}

	return COMPLETED_SENSOR_STATE;
}

void lm75SensorInit(void)
{
	for(int8_t i = LM75_NUM_SENSORS-1; i >= 0; i--) {
		uint8_t addr = LM75_BASE_ADDR + i;
		Wire.beginTransmission(addr);
		Wire.write(LM75_CONFIG_REGISTER);
		Wire.write(LM75_CONFIG_12BIT_RESOLUTION | LM75_CONFIG_SHUTDOWN);
		if(Wire.endTransmission() != 0)
			continue;
		sensor_t *sensor = &sensors[i+LM75_BASE_IDX];
		sensor->type = LM75_SENSOR_TYPE;
		sensor->addr = addr;
	}
}


#define HIH6130_BASE_IDX	16
#define HIH6130_NUM_SENSORS	1
#define HIH6130_BASE_ADDR	0x27
//#define HIH6130_MAX_ADDR	HIH6130_BASE_ADDR + 1

#define	HIH6130_NORMAL_STATUS		0x0000L
#define	HIH6130_STALE_STATUS		0x4000L
#define	HIH6130_COMMAND_MODE_STATUS	0x8000L
#define	HIH6130_DIAGNOSTIC_STATUS	0xc000L
#define HIH6130_STATUS_MASK		0xc000L

sensorState_t readHIH6130Sensor(sensor_t *sensor)
{
	long humidityVal, tempVal;

	if(sensor->type != HIH6130_SENSOR_TYPE)
		return ERROR_SENSOR_STATE;

	switch(sensor->state) {
	case START_SENSOR_STATE:
		Wire.beginTransmission(sensor->addr);
		Wire.endTransmission();
		return READ_SENSOR_STATE;
		break;

	case READ_SENSOR_STATE:
		Wire.requestFrom(sensor->addr, (uint8_t)4);
		humidityVal = Wire.read() << 8;
		humidityVal |= Wire.read();
		tempVal = Wire.read() << 8;
		tempVal |= Wire.read();
		//testDebug.print("H:");
		//testDebug.print(humidityVal, HEX);
		//testDebug.print(",");
		//testDebug.print(humidityVal & HIH6130_STATUS_MASK, HEX);
		//testDebug.print(",");
		//testDebug.println(HIH6130_STALE_STATUS, HEX);
		if((humidityVal & HIH6130_STATUS_MASK) == HIH6130_STALE_STATUS)
			return READ_SENSOR_STATE;
		break;

	default:
		return ERROR_SENSOR_STATE;
		break;
	}
	long status = humidityVal & HIH6130_STATUS_MASK;
	//testDebug.print("HIH Humidity: ");
	//testDebug.print(humidityVal, DEC);
	//testDebug.print(", HIH Humidity: ");
	//testDebug.print(humidityVal, DEC);
	//testDebug.print(", HIH loops: ");
	//testDebug.println(i, DEC);
	//float humidity = (float)(humidityVal & ~HIH6130_STATUS_MASK) / (float)(2^14 - 1);
	float humidity = (float)(humidityVal & ~HIH6130_STATUS_MASK) / 16383.f * 100.f;
#ifdef DEBUG_SENSOR
	testDebug.print("HIH Humidity: ");
	testDebug.print(humidityVal, DEC);
	testDebug.print(", Temp: ");
	testDebug.println(tempVal, DEC);
	testDebug.print("  Humidity: ");
	testDebug.print(humidity);
	testDebug.print("%, Temperature: ");
#endif
	tempVal >>= 2;
	sensor->data16[0] = tempVal;
	sensor->data16[1] = humidityVal & ~HIH6130_STATUS_MASK;
#ifdef DEBUG_SENSOR
	//temp_t temp = (temp_t)tempVal / (float)(2^14 - 1) * (125.f - -40.f);
	//temp_t temp = (temp_t)(tempVal & 16383L) / 16383.f * (125.f - -40.f) + -40.f;
	temp_t temp = (float)tempVal / 16383.f * 165.f - 40.f;
	testDebug.print(temp * 9.f/5.f + 32.f);
	testDebug.println("°F");
	printTemp(temp);
#endif

	switch(status) {
	case HIH6130_NORMAL_STATUS:
		break;

	case HIH6130_COMMAND_MODE_STATUS:
		testDebug.println("Command Mode Error");
		return ERROR_SENSOR_STATE;
		break;

	case HIH6130_DIAGNOSTIC_STATUS:
		testDebug.println("Diagnostic Error");
		return ERROR_SENSOR_STATE;
		break;

	default:
		testDebug.println("Unknown Error");
		return ERROR_SENSOR_STATE;
		break;
	}

	return COMPLETED_SENSOR_STATE;
}

void hih6130SensorInit(void)
{
	for(int8_t i = HIH6130_NUM_SENSORS-1; i >= 0; i--) {
		uint8_t addr = HIH6130_BASE_ADDR + i;
		Wire.beginTransmission(addr);
		if(Wire.endTransmission() != 0)
			continue;
		sensor_t *sensor = &sensors[i+HIH6130_BASE_IDX];
		sensor->type = HIH6130_SENSOR_TYPE;
		sensor->addr = addr;
	}
}


#ifdef ONE_WIRE_SENSORS
#define ONE_WIRE_BASE_IDX	19
#define MAX_ONE_WIRE_SENSORS	5
typedef struct {
	byte addr[8];
} oneWireSensor_t;

oneWireSensor_t oneWireSensors[MAX_ONE_WIRE_SENSORS];

sensorState_t readOneWireSensor(sensor_t *sensor)
{
	int i;
	byte present = 0;
	static byte type_s = 0, type_19 = 0;
	byte data[12];
	static byte *addr = NULL;
	temp_t temp = 0;

	if(sensor->type != ONE_WIRE_SENSOR_TYPE)
		return ERROR_SENSOR_STATE;

	switch(sensor->state) {
	case START_SENSOR_STATE:
		addr = oneWireSensors[sensor->addr].addr;
#if 1
		debug.print("ROM =");
		for( i = 0; i < 8; i++) {
			debug.write(' ');
			debug.print(addr[i], HEX);
		}
#endif

		if (OneWire::crc8(addr, 7) != addr[7]) {
			debug.println("CRC is not valid!");
			return ERROR_SENSOR_STATE;
		}
		debug.println();

		type_19 = 0;
		// the first ROM byte indicates which chip
		switch (addr[0]) {
		case 0x10:
			debug.println("  Chip = DS18S20");  // or old DS1820
			type_s = 1;
			break;
		case 0x28:
			debug.println("  Chip = DS18B20");
			type_s = 0;
			break;
		case 0x22:
			debug.println("  Chip = DS1822");
			type_s = 0;
			break;
		case 0x21:
			debug.println("  Chip = DS1921G");
			type_19 = 1;
			break;
		default:
#if 0
			debug.println("Device is not a DS18x20 family device.");
#endif
			return ERROR_SENSOR_STATE;
		}

		ds.reset();
		ds.select(addr);
		ds.write(DS1921_CONVERT_TEMP, 1);         // start conversion, with parasite power on at the end
		sensor->waitTime = millis() + 750UL;
		return WAIT_SENSOR_STATE;
		break;

	case READ_SENSOR_STATE:

#if 0
		//delay(1000);     // maybe 750ms is enough, maybe not
		// we might do a ds.depower() here, but the reset will take care of it.
		if(millis() < sensor->waitTime)
			return WAIT_SENSOR_STATE;
#endif

		present = ds.reset();
		ds.select(addr);
		if(type_19) {
			ds.write(DS1921_READ_MEMORY);
#if 0
			ds.write(0x00);
			ds.write(0x02);
			debug.print("  Clock =");
			for(i = 0x00; i <= 0x0A; i++) {
				debug.print(" ");
				debug.print(ds.read(), HEX);
			}
			debug.println("");
			for( ; i < 0x11; i++)
				ds.read();
#else
			ds.write(0x11);
			ds.write(0x02);
#endif
			data[0] = ds.read();
#if 0
			debug.print("  Data = ");
			debug.print(present,HEX);
			debug.print(" ");
			debug.println(data[0], HEX);
#endif
			sensor->data16[0] = (data[0] - (40 << 1) + (CELSIUS_TO_KELVIN << 1)) << 3;
			temp = (temp_t)data[0] / 2.0f - 40.0f;

#if 0
			if(!writeRtc) {
				byte sampleRate;
				readDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
				debug.print("  sample rate = ");
				debug.println(sampleRate, DEC);
				readDS1921(addr, DS1921_STATUS_REGISTER, &sampleRate, sizeof(sampleRate));
				if(sampleRate & DS1921_STATUS_MIP) {
					debug.println("  Mission in progress.");
					writeRtc = 1;
					return;
				}
				clearDS1921(addr);
				debug.println("  Starting a mission.");
				sampleRate = 1;
				writeDS1921(addr, DS1921_SAMPLE_REGISTER, &sampleRate, sizeof(sampleRate));
				//debug.println("  Updating RTC...");
				//writeDS1921(addr, DS1921_RTC_REGISTER, rtc, sizeof(rtc));
				writeRtc = 1;
			}

			long count = 0;
			byte countBytes[3];

			readDS1921(addr, DS1921_DEVICE_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
			count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
			debug.print("  Device samples = ");
			debug.println(count, DEC);

			readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
			count = countBytes[0] << 0 | countBytes[1] << 8 | countBytes[2] << 16;
			debug.print("  Mission samples = ");
			debug.println(count, DEC);
			readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtc, 5);
			debug.print("  Mission timestamp = 20");
			//for(i = 4; i >= 0; i--) {
			//	debug.print(rtc[i], HEX);
			//	debug.print(" ");
			//}
			for(i = 0; i < 5; i++) {
				debug.print(rtc[4-i], HEX);
				debug.print(" ");
			}
			debug.println("");
			readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
			debug.println("  Data log:");
			if(count > 2048)
				count = 2048;
			count = 4;
			while(count-- > 0) {
				debug.print("    ");
				debug.println(ds.read(), DEC);
			}
			debug.println("");
#endif
		} else {
			ds.write(DS18B20_READ_SCRATCHPAD);

#if 0
			debug.print("  Data = ");
			debug.print(present,HEX);
			debug.print(" ");
#endif
			for ( i = 0; i < 9; i++) {           // we need 9 bytes
				data[i] = ds.read();
#if 0
				debug.print(data[i], HEX);
				debug.print(" ");
#endif
			}
#if 0
			debug.print(" CRC=");
			debug.print(OneWire::crc8(data, 8), HEX);
			debug.println();
#endif

			// convert the data to actual temperature

			unsigned int raw = (data[1] << 8) | data[0];
			if (type_s) {
				raw = raw << 3; // 9 bit resolution default
				if (data[7] == 0x10) {
					// count remain gives full 12 bit resolution
					raw = (raw & 0xFFF0) + 12 - data[6];
				}
			} else {
				byte cfg = (data[4] & 0x60);
				if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
				else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
				else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
				// default is 12 bit resolution, 750 ms conversion time
			}
			sensor->data16[0] = raw + (CELSIUS_TO_KELVIN << 4);
			temp = (temp_t)raw / 16.0;
		}
		break;

	default:
		return ERROR_SENSOR_STATE;
		break;
	}

#ifdef DEBUG_SENSOR
	printTemp(temp);
#endif
	return COMPLETED_SENSOR_STATE;
}

void oneWireSensorInit(void)
{
	int i, j;

	for(i = 0; i < MAX_ONE_WIRE_SENSORS; i++)
		oneWireSensors[i].addr[0] = 0x00;

	ds.reset_search();
	debug.println("Scanning 1-Wire...");
	for(i = 0; i < MAX_ONE_WIRE_SENSORS &&
	    ds.search(oneWireSensors[i].addr); i++) {
		sensor_t *sensor = &sensors[i+ONE_WIRE_BASE_IDX];
		sensor->type = ONE_WIRE_SENSOR_TYPE;
		sensor->addr = i;
		debug.print("Found ROM =");
		for(j = 0; j < 8; j++) {
			debug.write(' ');
			debug.print(oneWireSensors[i].addr[j], HEX);
		}
		debug.println("");
	}
}
#endif

sensorState_t readSensor(sensor_t *sensor)
{
	if(sensor->type == ANALOG_SENSOR_TYPE ||
	   sensor->type == LIGHT_SENSOR_TYPE)
		return readAnalogSensor(sensor);
	else if(sensor->type == LM75_SENSOR_TYPE)
		return readLM75Sensor(sensor);
	else if(sensor->type == HIH6130_SENSOR_TYPE)
		return readHIH6130Sensor(sensor);
#ifdef ONE_WIRE_SENSORS
	else if(sensor->type == ONE_WIRE_SENSOR_TYPE)
		return readOneWireSensor(sensor);
#endif

	return ERROR_SENSOR_STATE;
}

void processSensor(sensor_t *sensor)
{
	ZBTxRequest zbTx = ZBTxRequest(coordinator, payload, sizeof(payload));
	sensorState_t state = sensor->state;

	if(state < COMPLETED_SENSOR_STATE) {
		if(state == WAIT_SENSOR_STATE) {
			if(millis() < sensor->waitTime)
				return;
			sensor->state = READ_SENSOR_STATE;
		}
		state = readSensor(sensor);
		if(state == COMPLETED_SENSOR_STATE)
			sensor->readingTime = clock;
		sensor->state = state;
		return;
	}

	switch(state) {
	case COMPLETED_SENSOR_STATE:
		sensorPayload[1] = sensor->id;
		sensorPayload[2] = sensor->type;
		*(uint32_t *)&sensorPayload[3] = sensor->readingTime;
		*(uint32_t *)&sensorPayload[7] = sensor->data32[0];
		zbTx.setPayload(sensorPayload);
		zbTx.setPayloadLength(sizeof(sensorPayload));
		xbee.send(zbTx);
		sensor->state = XBEE_SEND_SENSOR_STATE;
		break;

	default:
		sensor->state = START_SENSOR_STATE;
		break;
	}
}

void printSensor(sensor_t *sensor)
{
	int idx;

	idx = sensor - sensors;
	switch(sensor->type) {
	case ANALOG_SENSOR_TYPE:
		testDebug.print("Analog(");
		idx -= ANALOG_BASE_IDX;
		break;
	case LM75_SENSOR_TYPE:
		testDebug.print("LM75(");
		idx -= LM75_BASE_IDX;
		break;
	case HIH6130_SENSOR_TYPE:
		testDebug.print("HIH6130(");
		idx -= HIH6130_BASE_IDX;
		break;
	case ONE_WIRE_SENSOR_TYPE:
		testDebug.print("1-Wire(");
		idx -= ONE_WIRE_BASE_IDX;
		break;
	case LIGHT_SENSOR_TYPE:
		testDebug.print("Light(");
		idx -= ANALOG_BASE_IDX;
		break;
	default:
		testDebug.print("Unknown(");
		idx -= 0;
		break;
	}
	testDebug.print(idx, DEC);
	testDebug.print("): ");
}

void sensorInit(void)
{
	for(int8_t i = sizeof(sensors)/sizeof(sensors[0])-1; i >= 0; i--) {
		sensors[i].id = i;
		sensors[i].type = NONE_SENSOR_TYPE;
		sensors[i].state = START_SENSOR_STATE;
		sensors[i].flags = 0;
	}

	analogSensorInit();
	lm75SensorInit();
	hih6130SensorInit();
#ifdef ONE_WIRE_SENSORS
	oneWireSensorInit();
#endif
}



#ifdef LED_NOTIFICATION
typedef enum {
	OFF_LED_MODE,
	HEARTBEAT_LED_MODE,
	ALARM_LED_MODE,
} ledMode_t;

//#define DEFAULT_LED_MODE	OFF_LED_MODE
#define DEFAULT_LED_MODE	HEARTBEAT_LED_MODE

ledMode_t ledMode = DEFAULT_LED_MODE;

#define HEARTBEAT_LED_PERIOD	4700UL
#define HEARTBEAT_LED_ON_TIME	500UL
#define ALARM_LED_PERIOD	2000UL
#define ALARM_LED_ON_TIME	1000UL

void ledHandler(void)
{
	switch(ledMode) {
	case OFF_LED_MODE:
		ledOff();
		break;

	case HEARTBEAT_LED_MODE:
		if(millis() % HEARTBEAT_LED_PERIOD < HEARTBEAT_LED_ON_TIME)
			ledOn();
		else
			ledOff();
		break;

	case ALARM_LED_MODE:
		if(millis() % ALARM_LED_PERIOD < ALARM_LED_ON_TIME)
			ledOn();
		else
			ledOff();
		break;

	default:
		ledMode = OFF_LED_MODE;
		ledOff();
		break;
	}
}

void ledInit(void)
{
	pinMode(LED_PIN, OUTPUT);
	ledOff();
}
#endif

#ifdef PIEZO_NOTIFICATION
typedef enum {
	OFF_PIEZO_MODE,
	HEARTBEAT_PIEZO_MODE,
	ALARM_PIEZO_MODE,
} piezoMode_t;

//#define DEFAULT_PIEZO_MODE	OFF_PIEZO_MODE
#define DEFAULT_PIEZO_MODE	HEARTBEAT_PIEZO_MODE
//#define DEFAULT_PIEZO_MODE	ALARM_PIEZO_MODE

piezoMode_t piezoMode = DEFAULT_PIEZO_MODE;

#define HEARTBEAT_PIEZO_PERIOD	1300UL
#define HEARTBEAT_PIEZO_ON_TIME	100UL
#define ALARM_PIEZO_PERIOD	2000UL
#define ALARM_PIEZO_ON_TIME	1000UL

void piezoHandler(void)
{
	switch(piezoMode) {
	case OFF_PIEZO_MODE:
		piezoOff();
		break;

	case HEARTBEAT_PIEZO_MODE:
		if(millis() % HEARTBEAT_PIEZO_PERIOD < HEARTBEAT_PIEZO_ON_TIME)
			piezoOn();
		else
			piezoOff();
		break;

	case ALARM_PIEZO_MODE:
		if(millis() % ALARM_PIEZO_PERIOD < ALARM_PIEZO_ON_TIME)
			piezoOn();
		else
			piezoOff();
		break;

	default:
		piezoMode = OFF_PIEZO_MODE;
		piezoOff();
		break;
	}
}

void piezoInit(void)
{
	pinMode(PIEZO_PIN, OUTPUT);
	piezoOff();
}
#endif


void xbeeHandler(void)
{
#ifndef DEBUG_XBEE
	ZBTxRequest zbTx = ZBTxRequest(coordinator, payload, sizeof(payload));
#endif
	byte *dataPtr;
	uint8_t sensor;

#ifdef DEBUG_XBEE
	if(uart.available()) {
		debug.print("XB: [");
		while(uart.available())
			debug.print(uart.read(), HEX);
		debug.println("]");
	}
#else
	xbee.readPacket();
	if(xbee.getResponse().isAvailable()) {
		uint32_t temp;
		switch(xbee.getResponse().getApiId()) {
		case ZB_RX_RESPONSE:
			xbee.getResponse().getZBRxResponse(rx);
			dataPtr = rx.getData();
			debug.print("ZB RZ [");
			for(int i = 0; i < rx.getDataLength(); i++) {
				if(i > 0)
					debug.print(" ");
				debug.print(rx.getData()[i], HEX);
			}
			debug.print("]");
			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
				debug.print(" (ACK)");
			}
			debug.println("");
			if(rx.getDataLength() < 1)
				break;
			switch(dataPtr[0]) {
			case 1: /* Query time */
				*(uint32_t *)&timePayload[1] = clock;
				zbTx.setPayload(timePayload);
				zbTx.setPayloadLength(sizeof(timePayload));
				xbee.send(zbTx);
				break;

			case 2: /* Set time */
				if(rx.getDataLength() < 5)
					break;
				clock = *(uint32_t *)&dataPtr[1];
				break;

			case 3: /* Query sensor */
				if(rx.getDataLength() < 2)
					break;
				sensor = (uint8_t)dataPtr[1];
				temp = -1;
				if(sensor < sizeof(sensors)/sizeof(sensors[0])) {
					temp = sensors[sensor].data32[0];
					sensorPayload[2] = sensors[sensor].type;
				}
				sensorPayload[1] = dataPtr[1];
				*(uint32_t *)&sensorPayload[3] = clock;
				*(uint32_t *)&sensorPayload[7] = temp;
				zbTx.setPayload(sensorPayload);
				zbTx.setPayloadLength(sizeof(sensorPayload));
				xbee.send(zbTx);
				break;

			default:
				debug.println("Unknown Data packet.");
				break;
			}
			break;

		case ZB_TX_STATUS_RESPONSE:
			debug.print("ZB TX Status: ");
			xbee.getResponse().getZBTxStatusResponse(txStatus);
			if(txStatus.getDeliveryStatus() == 0) {
				debug.print("Success with ");
				debug.print(txStatus.getTxRetryCount());
				debug.println(" retries");
			} else {
				debug.print("Failed with status ");
				debug.println(txStatus.getDeliveryStatus(), HEX);
			}
			break;

		case MODEM_STATUS_RESPONSE:
			debug.println("MODEM");
			xbee.getResponse().getModemStatusResponse(msr);
			if(msr.getStatus() == ASSOCIATED)
				debug.println("ASSOCIATED");
			break;

		default:
			debug.println("Unknown");
			break;
		}
		return;
	} else if(xbee.getResponse().isError()) {
		debug.print("XBee Error: ");
		debug.println(xbee.getResponse().getErrorCode(), DEC);
	}
#endif

}

void xbeeInit(void)
{
#ifdef DEBUG_XBEE
	uart.begin(9600);
#else
#ifdef ARDUINO_UNO
	debug.begin(9600);
	pinMode(XBEE_RX_PIN, INPUT);
	pinMode(XBEE_TX_PIN, OUTPUT);
	xbeeSerial2.begin(9600);
	xbee.setSerial(xbeeSerial2);
#else
	xbee.begin(9600);
#endif
#endif
}



void setup(void)
{
	xbeeInit();
	delay(5000);
	debug.println("Hello, World!");

#ifdef LED_NOTIFICATION
	ledInit();
#endif
#ifdef PIEZO_NOTIFICATION
	piezoInit();
#endif

	/* Initialize Analog sensors */
	analogReference(DEFAULT);
	pinMode(A1, INPUT);
	digitalWrite(A1, LOW);

	/* Initialize Digital sensors */
//#ifdef CORE_TEENSY
//	pinMode(D0, INPUT);
//	pinMode(D1, INPUT);
//	digitalWrite(D0, HIGH); /* Turn on pull-ups */
//	digitalWrite(D1, HIGH);
//#else
	pinMode(SCL, INPUT);
	pinMode(SDA, INPUT);
	digitalWrite(SCL, HIGH); /* Turn on pull-ups */
	digitalWrite(SDA, HIGH);
//#endif
	Wire.begin();

	sensorInit();
}

void loop(void)
{
#ifdef TIMING_DEBUG
	unsigned long mainLoopStartTime = millis();
#endif
	static int sensorNum = 0;
	sensor_t *sensor;

#if 0
	//set_sleep_mode(SLEEP_MODE_IDLE);
	//set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	cli();
	sleep_mode();
#endif
#ifdef LED_NOTIFICATION
	ledHandler();
#endif
#ifdef PIEZO_NOTIFICATION
	piezoHandler();
#endif
	//return;

	unsigned long currentMillis = millis();
	if(currentMillis - clockTick >= 1000UL) {
		clockTick += 1000UL;
		clock++;
		debug.print("UTime=");
		debug.println(clock, DEC);
	}

#ifdef ONE_WIRE_SENSORS
	if(debug.available()) {
		parseSerial(debug.read());
	}
#endif

	xbeeHandler();

	if(currentMillis - sensorTick >= 5000UL) {
#ifdef TIMING_DEBUG
		unsigned long startTime;
#endif

		sensor = &sensors[sensorNum];
		switch(sensorNum) {
		case 0 + ANALOG_BASE_IDX:
		case 1 + ANALOG_BASE_IDX:
		case 2 + ANALOG_BASE_IDX:
		case 7 + LM75_BASE_IDX:
		case 0 + HIH6130_BASE_IDX:
#ifdef ONE_WIRE_SENSORS
		case 0 + ONE_WIRE_BASE_IDX:
		case 1 + ONE_WIRE_BASE_IDX:
		case 2 + ONE_WIRE_BASE_IDX:
#endif
			testDebug.print("Sensor: ");
			testDebug.println(sensorNum);
#ifdef TIMING_DEBUG
			startTime = millis();
#endif
			processSensor(sensor);
			printSensor(sensor);
			if(sensor->state >= COMPLETED_SENSOR_STATE) {
				processSensor(sensor);
				sensorNum++;
				sensor = &sensors[sensorNum];
				sensor->state = START_SENSOR_STATE;
			} else {
				testDebug.println("Waiting...");
			}
#ifdef TIMING_DEBUG
			testDebug.print("Took ");
			testDebug.print(millis() - startTime, DEC);
			testDebug.println(" ms\n");
#endif
			break;

		default:
			if(sensorNum >= sizeof(sensors)/sizeof(sensors[0])) {
				sensorTick += 5000UL;
				sensorNum = 0;
			} else {
				sensorNum++;
			}
			sensor = &sensors[sensorNum];
			sensor->state = START_SENSOR_STATE;

			break;
		}
	}

#ifdef TIMING_DEBUG
	testDebug.print("Main loop took ");
	testDebug.print(millis() - mainLoopStartTime, DEC);
	testDebug.println(" ms");
#endif
}
