#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <SPI.h>
#include <Wire.h>

/* OneWire DS18S20, DS18B20, DS1822 Temperature Example
 *
 * http://www.pjrc.com/teensy/td_libs_OneWire.html
 *
 * The DallasTemperature library can do all this work for you!
 * http://milesburton.com/Dallas_Temperature_Control_Library
 */


#define ARDUINO_UNO

//#define DEBUG_SENSOR
//#define DEBUG_TIMING
//#define DEBUG_WIFLY
//#define USE_ZIGBEE_DEBUG

#define USE_ARDUINO_XBEE

#include "DS1921_Logger.h"
#include "serial.h"
#include "xbee.h"
#include "sensor.h"

#ifdef ARDUINO_UNO
#include <SoftwareSerial.h>
#endif

#ifdef USE_ARDUINO_XBEE
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


class Dummy : public Stream {
    public:
	virtual int available(void) { return 0; }
	virtual int read(void) { return 0; }
	virtual int peek(void) { return 0; }
	virtual void flush(void) {}
	virtual size_t write(uint8_t val) { return 0; }
};

#ifdef USE_ARDUINO_XBEE
XBee xbee = XBee();
//XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ModemStatusResponse msr = ModemStatusResponse();
XBeeAddress64 coordinator = XBeeAddress64(0x0, 0x0);
#else
xbee_t xbeeDevice;
xbee_t *xbee = &xbeeDevice;
#endif
uint8_t timePayload[] = { 0x81, 0, 0, 0, 0 };
struct querySensorResponse sensorPayload;

uint32_t clock;
unsigned long clockTick;
unsigned long sensorTick;
unsigned long uploadTick = 10000UL;

class SensorDebug : public Stream {
    private:
	uint8_t debugPayload[110];
	unsigned int offset;
#ifdef USE_ARDUINO_XBEE
	ZBTxRequest zbTx;
#endif

    public:
	SensorDebug() : Stream() {
		debugPayload[0] = 0x04;
		offset = 1U;
#ifdef USE_ARDUINO_XBEE
		zbTx = ZBTxRequest(coordinator, debugPayload, offset);
#endif
	}

	virtual int available(void) { return 0; }
	virtual int read(void) { return 0; }
	virtual int peek(void) { return 0; }
	virtual void flush(void) {
#ifdef USE_ARDUINO_XBEE
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

#ifndef USE_ARDUINO_XBEE
HardwareSerial uart = HardwareSerial();
#endif
//#define debug Uart
#ifdef USE_ZIGBEE_DEBUG
SensorDebug debug = SensorDebug();
#else
#if defined(CORE_TEENSY) || defined(ARDUINO_UNO)
#define debug Serial
#else
#define debug dummy
#define NEED_DUMMY_STREAM
#endif
#endif

#if defined(CORE_TEENSY) || defined(ARDUINO_UNO)
#define localDebug Serial
#else
#define localDebug dummy
#define NEED_DUMMY_STREAM
#endif

#ifdef NEED_DUMMY_STREAM
Dummy dummy = Dummy();
#undef NEED_DUMMY_STREAM
#endif

//#define serial Serial
#define serial localDebug



uint8_t frameId = -1;

uint8_t getFrameId(void)
{
	frameId++;
	if(frameId == 0)
		frameId = 1;

	return frameId;
}

#ifdef ONE_WIRE_SENSORS
OneWire ds(0);

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

	serial.print("S=");
	serial.print(state, DEC);
	serial.print(": ");
	serial.println(c);
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
			serial.println("STARTLOG");
			readDS1921(addr, DS1921_MISSION_TIMESTAMP, rtcBuf, 5);
			serial.print("Time=20");
			serial.print(rtcBuf[4], HEX);
			serial.print("-");
			serial.print(rtcBuf[3], HEX);
			serial.print("-");
			serial.print(rtcBuf[2], HEX);
			serial.print("T");
			serial.print(rtcBuf[1], HEX);
			serial.print(":");
			serial.print(rtcBuf[0], HEX);
			serial.println(":00Z");

			long count = 0;
			byte countBytes[3];
			readDS1921(addr, DS1921_MISSION_SAMPLES_COUNTER, countBytes, sizeof(countBytes));
			count = (long)countBytes[0] << 0 | (long)countBytes[1] << 8 | (long)countBytes[2] << 16;
			serial.print("Count=");
			serial.println(count, DEC);

			readDS1921(addr, DS1921_DATA_LOG, NULL, 0);
			serial.println("LOG");
			if(count > 2048)
				count = 2048;
			while(count-- > 0) {
				serial.print("    ");
				serial.println(ds.read(), DEC);
			}
			serial.println("ENDLOG");
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
		serial.print("Time set to ");
		serial.println(clock, DEC);
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
			serial.println("rtc time is ...");
			readDS1921(addr, DS1921_RTC_REGISTER, rtcBuf, sizeof(rtcBuf));
			serial.print("20");
			serial.print((unsigned char)rtcBuf[6], HEX);
			serial.print("-");
			serial.print((unsigned char)rtcBuf[5] & ~0x80, HEX);
			serial.print("-");
			serial.print((unsigned char)rtcBuf[4], HEX);
			serial.print("T");
			serial.print((unsigned char)rtcBuf[2], HEX);
			serial.print(":");
			serial.print((unsigned char)rtcBuf[1], HEX);
			serial.print(":");
			serial.print((unsigned char)rtcBuf[0], HEX);
			serial.println("Z");
			state = HOME_STATE;
			break;
		} else {
			memset(rtcBuf, 0, sizeof(rtcBuf));
			/* discard 2 */
			while(!serial.available())
				;
			c = serial.read();
			/* discard 0 */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[6] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[6] |= (c & ~0x30);
			while(!serial.available())
				;
			c = serial.read();
			/* discard - */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[5] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[5] |= (c & ~0x30);
			rtcBuf[5] |= 0x80;
			while(!serial.available())
				;
			c = serial.read();
			/* discard - */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[4] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[4] |= (c & ~0x30);
			while(!serial.available())
				;
			c = serial.read();
			/* discard T */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[2] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[2] |= (c & ~0x30);
			while(!serial.available())
				;
			c = serial.read();
			/* discard : */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[1] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[1] |= (c & ~0x30);
			while(!serial.available())
				;
			c = serial.read();
			/* discard : */
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[0] = (c & ~0x30) << 4;
			while(!serial.available())
				;
			c = serial.read();
			rtcBuf[0] |= (c & ~0x30);
			while(!serial.available())
				;
			c = serial.read();
			/* discard '\n' */
			rtcBuf[3] = 1; /* day of week (1-7) */
			serial.print("  Set RTC = 20");
			for(i = 6; i >= 0; i--) {
				serial.print(rtcBuf[i], HEX);
				serial.print(" ");
			}
			serial.println("");
			serial.println("  Updating RTC...");
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
		debug.println("ALARM!");
		alarm = 1;
#ifdef LED_NOTIFICATION
		ledOn();
	} else {
		ledOff();
#endif
	}
	fahrenheit = celsius * 1.8 + 32.0;
	//debug.print("  ATemperature = ");
	debug.print("  T=");
	debug.print(celsius);
	//debug.print(" Celsius, ");
	debug.print("C, ");
	debug.print(fahrenheit);
	//debug.println(" Fahrenheit");
	debug.println("F");
}



sensor_t sensors[32];

void printSensor(sensor_t *sensor);


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
	printSensor(sensor);
	temp_t temp = ((float)val / 1023. * AREF_MV - 500.)/10.;
	debug.print("ADC Val: ");
	debug.println(val, DEC);
	if(sensor->type == LIGHT_SENSOR_TYPE) {
		debug.print("  L=");
		debug.print((double)val * 100. / 1023.);
		debug.println("%");
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

#define LM75_TEMP_REGISTER		(uint8_t)0
#define LM75_CONFIG_REGISTER		(uint8_t)1
#define LM75_THYST_REGISTER		(uint8_t)2
#define LM75_TSET_REGISTER		(uint8_t)3

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
		debug.print("LM75 Start Conf: 0x");
		debug.println(config, HEX);
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
		debug.print("LM75 Read Conf: 0x");
		debug.println(config, HEX);
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
		printSensor(sensor);
		temp = (temp_t)val * 0.0625f;
		debug.print("I2C Val: ");
		debug.println(val, DEC);
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


/* HIH6130 sensor support */

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
		//debug.print("H:");
		//debug.print(humidityVal, HEX);
		//debug.print(",");
		//debug.print(humidityVal & HIH6130_STATUS_MASK, HEX);
		//debug.print(",");
		//debug.println(HIH6130_STALE_STATUS, HEX);
		if((humidityVal & HIH6130_STATUS_MASK) == HIH6130_STALE_STATUS)
			return READ_SENSOR_STATE;
		break;

	default:
		return ERROR_SENSOR_STATE;
		break;
	}
	long status = humidityVal & HIH6130_STATUS_MASK;
	//debug.print("HIH Humidity: ");
	//debug.print(humidityVal, DEC);
	//debug.print(", HIH Humidity: ");
	//debug.print(humidityVal, DEC);
	//debug.print(", HIH loops: ");
	//debug.println(i, DEC);
	//float humidity = (float)(humidityVal & ~HIH6130_STATUS_MASK) / (float)(2^14 - 1);
#ifdef DEBUG_SENSOR
	float humidity = (float)(humidityVal & ~HIH6130_STATUS_MASK) / 16383.f * 100.f;
	printSensor(sensor);
	debug.print("HIH Humidity: ");
	debug.print(humidityVal, DEC);
	debug.print(", Temp: ");
	debug.println(tempVal, DEC);
	debug.print("  Humidity: ");
	debug.print(humidity);
	debug.print("%, Temperature: ");
#endif
	tempVal >>= 2;
	sensor->data16[0] = tempVal;
	sensor->data16[1] = humidityVal & ~HIH6130_STATUS_MASK;
#ifdef DEBUG_SENSOR
	//temp_t temp = (temp_t)tempVal / (float)(2^14 - 1) * (125.f - -40.f);
	//temp_t temp = (temp_t)(tempVal & 16383L) / 16383.f * (125.f - -40.f) + -40.f;
	temp_t temp = (float)tempVal / 16383.f * 165.f - 40.f;
	debug.print(temp * 9.f/5.f + 32.f);
	debug.println("°F");
	printTemp(temp);
#endif

	switch(status) {
	case HIH6130_NORMAL_STATUS:
		break;

	case HIH6130_COMMAND_MODE_STATUS:
		debug.println("Command Mode Error");
		return ERROR_SENSOR_STATE;
		break;

	case HIH6130_DIAGNOSTIC_STATUS:
		debug.println("Diagnostic Error");
		return ERROR_SENSOR_STATE;
		break;

	default:
		debug.println("Unknown Error");
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


/* TC Sensor support */

#define TC_BASE_IDX			17
#define TC_NUM_SENSORS			1
//#define TC_BASE_ADDR			9

//#define tcOn()				digitalWrite(TC_BASE_ADDR, LOW)
//#define tcOff()				digitalWrite(TC_BASE_ADDR, HIGH)

#define TC_SIGN_FLAG				0x8000
#define TC_OPEN_FLAG				0x0004
#define TC_DEVICE_ID_FLAG			0x0002
#define TC_TRISTATE_FLAG			0x0001

#define TC_VALUE_MASK				0x7ff8

sensorState_t readTCSensor(sensor_t *sensor)
{
#ifdef DEBUG_SENSOR
	temp_t temp;
#endif

	if(sensor->type != TC_SENSOR_TYPE)
		return ERROR_SENSOR_STATE;

	switch(sensor->state) {
		uint8_t val;
	case START_SENSOR_STATE:
		val = SPI.transfer(0xff) << 8;
		val |= SPI.transfer(0xff);
#ifdef DEBUG_SENSOR
		printSensor(sensor);
		temp = (float)(val & ~0x05) * 0.25f;
		debug.print("SPI Val: ");
		debug.println(val, DEC);
		printTemp(temp);
#endif
		if(val & (TC_SIGN_FLAG | TC_OPEN_FLAG | TC_DEVICE_ID_FLAG)) {
			sensor->data16[0] = -1;
			return ERROR_SENSOR_STATE;
		}
		sensor->data16[0] = ((val & TC_VALUE_MASK) << 2) + (CELSIUS_TO_KELVIN << 4);
		break;

	default:
		return ERROR_SENSOR_STATE;
		break;
	}

	return COMPLETED_SENSOR_STATE;
}

void tcSensorInit(void)
{
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE3); /* Idle high, falling-edge */
	SPI.setClockDivider(SPI_CLOCK_DIV4); /* 4.3MHz max */

	for(int8_t i = TC_NUM_SENSORS-1; i >= 0; i--) {
		//uint8_t addr = TC_BASE_ADDR + i;
		if(1 != 0)
			continue;
		sensor_t *sensor = &sensors[i+TC_BASE_IDX];
		sensor->type = TC_SENSOR_TYPE;
		//sensor->addr = addr;
	}
}


/* 1-Wire sensor support */

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
			debug.print("  CRC is not valid!");
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
	printSensor(sensor);
	printTemp(temp);
#endif
	return COMPLETED_SENSOR_STATE;
}

void oneWireSensorInit(void)
{
	int i;

	for(i = 0; i < MAX_ONE_WIRE_SENSORS; i++)
		oneWireSensors[i].addr[0] = 0x00;

	ds.reset_search();
#ifdef DEBUG_SENSOR
	debug.println("Scanning 1-Wire...");
#endif
	for(i = 0; i < MAX_ONE_WIRE_SENSORS &&
	    ds.search(oneWireSensors[i].addr); i++) {
		sensor_t *sensor = &sensors[i+ONE_WIRE_BASE_IDX];
		sensor->type = ONE_WIRE_SENSOR_TYPE;
		sensor->addr = i;
#ifdef DEBUG_SENSOR
		debug.print("Found ROM =");
		for(int j = 0; j < 8; j++) {
			debug.write(' ');
			debug.print(oneWireSensors[i].addr[j], HEX);
		}
		debug.println("");
#endif
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
	else if(sensor->type == TC_SENSOR_TYPE)
		return readTCSensor(sensor);
#ifdef ONE_WIRE_SENSORS
	else if(sensor->type == ONE_WIRE_SENSOR_TYPE)
		return readOneWireSensor(sensor);
#endif

	return ERROR_SENSOR_STATE;
}

void processSensor(sensor_t *sensor)
{
#ifdef USE_ARDUINO_XBEE
	ZBTxRequest zbTx = ZBTxRequest(coordinator, NULL, 0);
#endif
	sensorState_t state = sensor->state;

#ifdef DEBUG_TIMING
	if(state == START_SENSOR_STATE)
		sensor->startTime = millis();
#endif
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
		frameId = getFrameId();
		sensor->frameId = frameId;
		sensorPayload.cmd = QUERY_SENSOR_SENSOR_CMD | 0x80;
		sensorPayload.sensor = sensor->id;
		sensorPayload.type = sensor->type;
		sensorPayload.time = sensor->readingTime;
		sensorPayload.data32[0] = sensor->data32[0];
#ifdef USE_ARDUINO_XBEE
		zbTx.setPayload((uint8_t *)&sensorPayload);
		zbTx.setPayloadLength(sizeof(sensorPayload));
		zbTx.setFrameId(frameId);
		xbee.send(zbTx);
#else
		sendTx(xbee, (macAddr64_t *)&coordinatorAddr, &sensorPayload, sizeof(sensorPayload));
#endif
		sensor->waitTime = millis() + 5000UL;
		sensor->state = XBEE_SEND_SENSOR_STATE;
		break;

	case XBEE_SEND_SENSOR_STATE:
		if(millis() < sensor->waitTime)
			return;
		/* XXX Failed to send sensor data */
		sensor->state = XBEE_ACK_SENSOR_STATE;
		break;

	case ERROR_SENSOR_STATE:
	case XBEE_ACK_SENSOR_STATE:
	case STOP_SENSOR_STATE:
#ifdef DEBUG_TIMING
		debug.print("Took ");
		debug.print(millis() - sensor->startTime, DEC);
		debug.println(" ms\n");
#endif
		/* nothing more to do once sensor data has been acknowledged */
		sensor->state = STOP_SENSOR_STATE;
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
		debug.print("Analog(");
		idx -= ANALOG_BASE_IDX;
		break;
	case LM75_SENSOR_TYPE:
		debug.print("LM75(");
		idx -= LM75_BASE_IDX;
		break;
	case HIH6130_SENSOR_TYPE:
		debug.print("HIH6130(");
		idx -= HIH6130_BASE_IDX;
		break;
	case TC_SENSOR_TYPE:
		debug.print("TC(");
		idx -= TC_BASE_IDX;
		break;
	case ONE_WIRE_SENSOR_TYPE:
		debug.print("1-Wire(");
		idx -= ONE_WIRE_BASE_IDX;
		break;
	case LIGHT_SENSOR_TYPE:
		debug.print("Light(");
		idx -= ANALOG_BASE_IDX;
		break;
	default:
		debug.print("Unknown(");
		idx -= 0;
		break;
	}
	debug.print(idx, DEC);
	debug.print("): ");
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
	tcSensorInit();
#ifdef ONE_WIRE_SENSORS
	oneWireSensorInit();
#endif

#ifdef DEBUG_SENSOR
	debug.println("Found sensors:");
	for(int8_t i = sizeof(sensors)/sizeof(sensors[0])-1; i >= 0; i--) {
		sensor_t *sensor = &sensors[32-i-1];
		if(sensor->type == NONE_SENSOR_TYPE)
			continue;
		debug.print("  ");
		printSensor(sensor);
		debug.println(sensor->addr);
	}
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
#ifdef USE_ARDUINO_XBEE
	ZBTxRequest zbTx = ZBTxRequest(coordinator, NULL, 0);
#endif
	byte *dataPtr;
	uint8_t sensor;

#ifndef USE_ARDUINO_XBEE
	if(uart.available()) {
		localDebug.print("XB: [");
		while(uart.available())
			localDebug.print(uart.read(), HEX);
		localDebug.println("]");
	}
	recvApi(xbee);
#else
	xbee.readPacket();
	if(xbee.getResponse().isAvailable()) {
		uint32_t temp;
		switch(xbee.getResponse().getApiId()) {
		case ZB_RX_RESPONSE:
			xbee.getResponse().getZBRxResponse(rx);
			dataPtr = rx.getData();
			localDebug.print("ZB RZ [");
			for(int i = 0; i < rx.getDataLength(); i++) {
				if(i > 0)
					localDebug.print(" ");
				localDebug.print(rx.getData()[i], HEX);
			}
			localDebug.print("]");
			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
				localDebug.print(" (ACK)");
			}
			localDebug.println("");
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
					sensorPayload.type = sensors[sensor].type;
				}
				sensorPayload.cmd = QUERY_SENSOR_SENSOR_CMD | 0x80;
				sensorPayload.sensor = dataPtr[1];
				sensorPayload.time = clock;
				sensorPayload.data32[0] = temp;
				zbTx.setPayload((uint8_t *)&sensorPayload);
				zbTx.setPayloadLength(sizeof(sensorPayload));
				xbee.send(zbTx);
				break;

			default:
				localDebug.println("Unknown Data packet.");
				break;
			}
			break;

		case ZB_TX_STATUS_RESPONSE:
			localDebug.print("ZB TX Status: ");
			xbee.getResponse().getZBTxStatusResponse(txStatus);
			if(txStatus.getDeliveryStatus() == 0) {
				localDebug.print("Success with ");
				localDebug.print(txStatus.getTxRetryCount());
				localDebug.println(" retries");
			} else {
				localDebug.print("Failed with status ");
				localDebug.println(txStatus.getDeliveryStatus(), HEX);
			}
			for(int8_t i = sizeof(sensors)/sizeof(sensors[0])-1; i >= 0; i--) {
				sensor_t *sensor = &sensors[i];
				if(sensor->type != NONE_SENSOR_TYPE && sensor->state == XBEE_SEND_SENSOR_STATE && sensor->frameId == txStatus.getFrameId()) {
					sensor->state = XBEE_ACK_SENSOR_STATE;
#ifdef DEBUG_SENSOR
					printSensor(sensor);
					localDebug.println("Sensor ACK");
#endif
				}
			}
			break;

		case MODEM_STATUS_RESPONSE:
			localDebug.println("MODEM");
			xbee.getResponse().getModemStatusResponse(msr);
			if(msr.getStatus() == ASSOCIATED)
				localDebug.println("ASSOCIATED");
			break;

		default:
			localDebug.println("Unknown");
			break;
		}
		return;
	} else if(xbee.getResponse().isError()) {
		localDebug.print("XBee Error: ");
		localDebug.println(xbee.getResponse().getErrorCode(), DEC);
	}
#endif

}

void xbeeInit(void)
{
#ifndef USE_ARDUINO_XBEE
	uart.begin(9600);
#else
#ifdef ARDUINO_UNO
	localDebug.begin(9600);
	pinMode(XBEE_RX_PIN, INPUT);
	pinMode(XBEE_TX_PIN, OUTPUT);
	//xbeeSerial2.begin(9600);
	//xbee.setSerial(xbeeSerial2);
#else
	xbee.begin(9600);
#endif
#endif
}



/* WiFly support */

typedef struct {
	const char *identifier;
	struct querySensorResponse sensor;
} sensorData_t;

sensorData_t sensorData = {
	"COOLER",
	{
		3,
		15,
		0x01,
		1324178532,
	},
};

#define HTTP_HOST				"www.example.org"
#define HTTP_PORT				"80"
#define HTTP_PATH				"/cgi/nagios.pl"

#define WIFLY_RX_PIN				2
#define WIFLY_TX_PIN				3
SoftwareSerial wiflyPort = SoftwareSerial(WIFLY_RX_PIN, WIFLY_TX_PIN);

#define WIFLY_READ_LINE_TIME			5000
#define WIFLY_GUARD_TIME			350
#define WIFLY_ESCAPE_STRING			"$$$"

typedef enum {
	IDLE_WIFLY_STATE,
	START_WIFLY_STATE,
	WAIT1_WIFLY_STATE,
	WAIT2_WIFLY_STATE,
	CMD_WIFLY_STATE,
	CONNECTED_WIFLY_STATE,
	HTTP_STATUS_WIFLY_STATE,
	FINISHED_WIFLY_STATE,
	READ_LINE_WIFLY_STATE,
	RESET_WIFLY_STATE,
} wiflyState_t;

wiflyState_t wiflyState;
wiflyState_t wiflyNextState;

unsigned long wiflyWait;
bool wiflyReadStar;

#define wiflyReadLine(state, readStar)	\
			do { \
				lineLen = 0; \
				wiflyReadStar = (readStar); \
				wiflyWait = millis() + WIFLY_READ_LINE_TIME; \
				wiflyNextState = (state); \
				wiflyState = READ_LINE_WIFLY_STATE; \
			} while(0);
#define wiflyReadNextLine(readStar) 	\
			do { \
				lineLen = 0; \
				wiflyReadStar = (readStar); \
				wiflyNextState = wiflyState; \
				wiflyState = READ_LINE_WIFLY_STATE; \
			} while(0);
#define wiflyWait(state, wait)		\
			do { \
				wiflyWait = millis() + (wait); \
				wiflyNextState = (state); \
				wiflyState = WAIT_WIFLY_STATE; \
			} while(0);

#define MAX_CHAR	16

void wiflyHandler(void)
{
	static char line[32];
	static size_t lineLen;
	int i;
	char c;
	char *status, *msg;
	int statusCode;

#ifdef DEBUG_WIFLY
	//debug.print("S:");
	//debug.print(wiflyState);
	//debug.println("");
#endif
	switch(wiflyState) {
	case IDLE_WIFLY_STATE:
		/* Waiting for instructions */
		break;

	case START_WIFLY_STATE:
#ifdef DEBUG_WIFLY
		debug.println("Starting WiFly...");
#endif
		wiflyWait = millis() + WIFLY_GUARD_TIME;
		wiflyState = WAIT1_WIFLY_STATE;
		break;

	case WAIT1_WIFLY_STATE:
		/* flush out any characters in the input buffer */
#ifdef DEBUG_WIFLY
		for(i = 0; i < MAX_CHAR && wiflyPort.available(); i++)
			debug.write(wiflyPort.read());
#endif
		if((long)(millis() - wiflyWait) < 0L)
			return;
		//debug.println(WIFLY_ESCAPE_STRING);
		wiflyPort.print(WIFLY_ESCAPE_STRING);
		wiflyWait = millis() + WIFLY_GUARD_TIME;
		wiflyState = WAIT2_WIFLY_STATE;
		break;

	case WAIT2_WIFLY_STATE:
		if((long)(millis() - wiflyWait) < 0L)
			return;
#ifdef DEBUG_WIFLY
		debug.println("Entering command mode...");
#endif
		wiflyWait = millis() + WIFLY_GUARD_TIME * 8;
		//wiflyState = CMD_WIFLY_STATE;
		wiflyReadLine(CMD_WIFLY_STATE, FALSE);
		break;

	case CMD_WIFLY_STATE:
		//debug.print("Comparing ");
		//debug.print(line);
		//debug.println(" to CMD");
		if(strcmp(line, "CMD\n") == 0) {
			if((long)(millis() - wiflyWait) < 0L) {
				return;
			}
#ifdef DEBUG_WIFLY
			debug.println("Connected.");
#endif
		} else {
			//debug.println("Something funny happening.");
			if((long)(millis() - wiflyWait) >= 0L) {
#ifdef DEBUG_WIFLY
				debug.println("Timed out.");
#endif
				//wiflyPort.print("exit\r");
				wiflyState = RESET_WIFLY_STATE;
				break;
			}
			wiflyReadNextLine(FALSE);
			break;
		}
		wiflyPort.print("open " HTTP_HOST " " HTTP_PORT "\r");
		wiflyReadLine(CONNECTED_WIFLY_STATE, TRUE);
		break;
		//wiflyWait = millis() + WIFLY_GUARD_TIME;
		//wiflyState = CONNECTED_WIFLY_STATE;
		//wiflyState = START_WIFLY_STATE;
		/* fall through */

	case CONNECTED_WIFLY_STATE:
		if(strcmp(line, "*OPEN*") != 0) {
#ifdef DEBUG_WIFLY
			debug.println("Failed to connect.");
#endif
			//debug.print("Have:'");
			//debug.print(line);
			//debug.println("'");
			wiflyState = RESET_WIFLY_STATE;
			break;
		}
#ifdef DEBUG_WIFLY
		debug.println("Requesting page...");
#endif
		wiflyPort.print("GET " HTTP_PATH "?node=");
		wiflyPort.print(sensorData.identifier);
		wiflyPort.print("&sensor=");
		wiflyPort.print(sensorData.sensor.sensor);
		wiflyPort.print("&type=");
		wiflyPort.print(sensorData.sensor.type);
		wiflyPort.print("&time=");
		wiflyPort.print(sensorData.sensor.time);
		wiflyPort.print("&value=");
		wiflyPort.print(sensorData.sensor.data16[0]);
		wiflyPort.print("&status=");
		wiflyPort.print("ok");
		wiflyPort.print("&message=");
		wiflyPort.print("Hello");
		wiflyPort.print(" HTTP/1.1\r\n");
		wiflyPort.print("Host: " HTTP_HOST "\r\n");
		wiflyPort.print("Connection: close\r\n");
		wiflyPort.print("\r\n");
		wiflyReadLine(HTTP_STATUS_WIFLY_STATE, FALSE);
		break;

	case HTTP_STATUS_WIFLY_STATE:
		if(strncmp(line, "HTTP/1.", 7) != 0 ||
		   (status = strchr(line, ' ')) == NULL) {
			debug.println("Failed to HTTP respond.");
			//debug.print("Have:'");
			//debug.print(line);
			//debug.println("'");
			//wiflyState = RESET_WIFLY_STATE;
			wiflyReadLine(FINISHED_WIFLY_STATE, TRUE);
			break;
		}
		*status++ = '\0';
		if((msg = strchr(status, ' ')) == NULL) {
			debug.println("Failed to HTTP2 respond.");
			//debug.print("Have:'");
			//debug.print(line);
			//debug.println("'");
			//wiflyState = RESET_WIFLY_STATE;
			wiflyReadLine(FINISHED_WIFLY_STATE, TRUE);
			break;
		}
		*msg++ = '\0';
		statusCode = atoi(status);
		if(statusCode != 200) {
			debug.println("Bad HTTP status.");
		}
		wiflyReadLine(FINISHED_WIFLY_STATE, TRUE);
		break;

	case FINISHED_WIFLY_STATE:
#if 0
		for(i = 0; i < MAX_CHAR && wiflyPort.available(); i++)
			debug.write(wiflyPort.read());
		if((long)(millis() - wiflyWait) < 0L)
			return;
#endif
		if(strcmp(line, "*CLOS*") != 0) {
			debug.println("Failed to receive document.");
			wiflyState = RESET_WIFLY_STATE;
			break;
		}
		debug.println("Done.");
		wiflyState = IDLE_WIFLY_STATE;
		break;

	case READ_LINE_WIFLY_STATE:
		/* Default behavior is to read a full line including the
		 * trailing '\n', but wiflyReadStar changes to reading a
		 * word surrounded by '*'s without a newline. */
		for(i = 0; i < MAX_CHAR && wiflyPort.available(); i++) {
			c = wiflyPort.read();
#ifdef DEBUG_WIFLY
			//debug.write(".");
			debug.write(c);
#endif
			if(c == '\r')
				continue;
			if(wiflyReadStar && lineLen == 0 && c != '*')
				continue;
			if(lineLen < sizeof(line)-1)
				line[lineLen++] = c;
			if(wiflyReadStar && lineLen <= 1)
				break;
			if((wiflyReadStar && c == '*') ||
			  (!wiflyReadStar && c == '\n')) {
				line[lineLen] = '\0';
				wiflyState = wiflyNextState;
				break;
			}
		}
		if((long)(millis() - wiflyWait) >= 0L) {
			line[lineLen] = '\0';
			wiflyState = wiflyNextState;
		}
		break;

	case RESET_WIFLY_STATE:
#ifdef DEBUG_WIFLY
		debug.println("Reseting WiFly...");
#endif
		wiflyPort.print("\rexit\r");
		/* flush out any characters in the input buffer */
#ifdef DEBUG_WIFLY
		for(i = 0; i < MAX_CHAR && wiflyPort.available(); i++)
			debug.write(wiflyPort.read());
#endif
		wiflyState = START_WIFLY_STATE;
		break;

	default:
		debug.println("Funny WiFly state, FIXME!");
		wiflyState = RESET_WIFLY_STATE;
		break;
	}
}

void wiflyInit(void)
{
	debug.println("Initializing WiFly...");
	pinMode(WIFLY_RX_PIN, INPUT);
	pinMode(WIFLY_TX_PIN, OUTPUT);
	wiflyPort.begin(9600);
}



void setup(void)
{
	//xbeeInit();
	debug.begin(9600);
	delay(5000);
	debug.println("Hello, World!");

#ifdef LED_NOTIFICATION
	ledInit();
#endif
#ifdef PIEZO_NOTIFICATION
	piezoInit();
#endif

	/* Initialize Analog sensors */
	//analogReference(DEFAULT);

	/* Initialize Digital sensors */
	//pinMode(SCL, INPUT);
	//pinMode(SDA, INPUT);
	//digitalWrite(SCL, HIGH); /* Turn on pull-ups */
	//digitalWrite(SDA, HIGH);
	//Wire.begin();

	//sensorInit();

	wiflyInit();
}

//extern "C" int addNewNodeCallback(nodeIdentification_t *node)
//{
//	return 0;
//}

void loop(void)
{
#ifdef DEBUG_TIMING
	static unsigned long lastMainLoopTime = 0;
	static unsigned long maxMainLoopTime = 0;
	unsigned long mainLoopStartTime = millis();
#endif
	static size_t sensorNum = 0;
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

	if((long)(millis() - uploadTick) >= 0L) {
		if(wiflyState == IDLE_WIFLY_STATE) {
			sensorData.sensor.data16[0] = 0x421f;
			debug.println("New sensor reading.");
			wiflyState = START_WIFLY_STATE;
			uploadTick = millis() + 30000UL;
		}
	}
	wiflyHandler();

	unsigned long currentMillis = millis();
	if(currentMillis - clockTick >= 1000UL) {
		clockTick += 1000UL;
		clock++;
		debug.print("UTime=");
		debug.println(clock, DEC);
	}
	return;

#ifdef ONE_WIRE_SENSORS
	if(serial.available()) {
		parseSerial(serial.read());
	}
#endif

	xbeeHandler();

	if(currentMillis - sensorTick >= 5000UL) {
		sensor = &sensors[sensorNum];
		switch(sensorNum) {
		case 0 + ANALOG_BASE_IDX:
		case 1 + ANALOG_BASE_IDX:
		case 2 + ANALOG_BASE_IDX:
		case 7 + LM75_BASE_IDX:
		case 0 + HIH6130_BASE_IDX:
		case 0 + TC_BASE_IDX:
#ifdef ONE_WIRE_SENSORS
		case 0 + ONE_WIRE_BASE_IDX:
		case 1 + ONE_WIRE_BASE_IDX:
		case 2 + ONE_WIRE_BASE_IDX:
#endif
			//debug.print("Sensor: ");
			//debug.println(sensorNum);
			processSensor(sensor);
			//printSensor(sensor);
			if(sensor->state >= STOP_SENSOR_STATE) {
				processSensor(sensor);
				sensorNum++;
				sensor = &sensors[sensorNum];
				sensor->state = START_SENSOR_STATE;
			} else {
				//debug.println("Waiting...");
			}
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

#ifdef DEBUG_TIMING
	currentMillis = millis();
	if(maxMainLoopTime < currentMillis - mainLoopStartTime)
		maxMainLoopTime = currentMillis - mainLoopStartTime;
	if(currentMillis - lastMainLoopTime >= 10000UL) {
		debug.print("Main loop took ");
		debug.print(maxMainLoopTime, DEC);
		debug.println(" ms max");
		maxMainLoopTime = 0;
		lastMainLoopTime = millis();
	}
#endif
}
