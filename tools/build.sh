#!/bin/sh

PATH=/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin:$PATH
COREPATH=/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/cores/arduino
VARIANTPATH=/Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/variants/standard
LIBPATH=/Applications/Arduino.app/Contents/Resources/Java/libraries
BUILDPATH=/var/folders/Lh/Lh3XwfGLG6S-C8UoPDEOW++++TI/-Tmp-/build2814928687313585811.tmp
CFLAGS="-g -Os -Wall -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=100"
CPPFLAGS="-g -Os -Wall -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -MMD -DARDUINO=100"
HEADERS="-I$COREPATH -I$VARIANTPATH -I${LIBPATH}/SPI -I${LIBPATH}/Wire -I${LIBPATH}/SoftwareSerial -I${LIBPATH}/XBee -I${LIBPATH}/OneWire"

set -e

avr-gcc -c $CFLAGS   $HEADERS $BUILDPATH/sensor.c -o $BUILDPATH/sensor.c.o 
avr-gcc -c $CFLAGS   $HEADERS $BUILDPATH/xbee.c -o $BUILDPATH/xbee.c.o 
avr-g++ -c $CPPFLAGS $HEADERS $BUILDPATH/DS1921_Logger.cpp -o $BUILDPATH/DS1921_Logger.cpp.o 
avr-g++ -c $CPPFLAGS $HEADERS $BUILDPATH/serial.cpp -o $BUILDPATH/serial.cpp.o 
avr-g++ -c $CPPFLAGS $HEADERS -I${LIBPATH}/SPI/utility ${LIBPATH}/SPI/SPI.cpp -o $BUILDPATH/SPI/SPI.cpp.o 
avr-g++ -c $CPPFLAGS $HEADERS -I${LIBPATH}/Wire/utility ${LIBPATH}/Wire/Wire.cpp -o $BUILDPATH/Wire/Wire.cpp.o 
avr-gcc -c $CFLAGS   $HEADERS -I${LIBPATH}/Wire/utility ${LIBPATH}/Wire/utility/twi.c -o $BUILDPATH/Wire/utility/twi.c.o 
avr-g++ -c $CPPFLAGS $HEADERS -I${LIBPATH}/SoftwareSerial/utility ${LIBPATH}/SoftwareSerial/SoftwareSerial.cpp -o $BUILDPATH/SoftwareSerial/SoftwareSerial.cpp.o 
avr-g++ -c $CPPFLAGS $HEADERS -I${LIBPATH}/XBee/utility ${LIBPATH}/XBee/XBee.cpp -o $BUILDPATH/XBee/XBee.cpp.o 
avr-g++ -c $CPPFLAGS $HEADERS -I${LIBPATH}/OneWire/utility ${LIBPATH}/OneWire/OneWire.cpp -o $BUILDPATH/OneWire/OneWire.cpp.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/WInterrupts.c -o $BUILDPATH/WInterrupts.c.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/wiring.c -o $BUILDPATH/wiring.c.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/wiring_analog.c -o $BUILDPATH/wiring_analog.c.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/wiring_digital.c -o $BUILDPATH/wiring_digital.c.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/wiring_pulse.c -o $BUILDPATH/wiring_pulse.c.o 
avr-gcc -c $CFLAGS   -I$COREPATH -I$VARIANTPATH $COREPATH/wiring_shift.c -o $BUILDPATH/wiring_shift.c.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/CDC.cpp -o $BUILDPATH/CDC.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/HardwareSerial.cpp -o $BUILDPATH/HardwareSerial.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/HID.cpp -o $BUILDPATH/HID.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/IPAddress.cpp -o $BUILDPATH/IPAddress.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/main.cpp -o $BUILDPATH/main.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/new.cpp -o $BUILDPATH/new.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/Print.cpp -o $BUILDPATH/Print.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/Stream.cpp -o $BUILDPATH/Stream.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/Tone.cpp -o $BUILDPATH/Tone.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/USBCore.cpp -o $BUILDPATH/USBCore.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/WMath.cpp -o $BUILDPATH/WMath.cpp.o 
avr-g++ -c $CPPFLAGS -I$COREPATH -I$VARIANTPATH $COREPATH/WString.cpp -o $BUILDPATH/WString.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/WInterrupts.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/wiring.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/wiring_analog.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/wiring_digital.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/wiring_pulse.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/wiring_shift.c.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/CDC.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/HardwareSerial.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/HID.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/IPAddress.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/main.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/new.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/Print.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/Stream.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/Tone.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/USBCore.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/WMath.cpp.o 
avr-ar rcs $BUILDPATH/core.a $BUILDPATH/WString.cpp.o 
avr-gcc -Os -Wl,--gc-sections -mmcu=atmega328p -o $BUILDPATH/DS1921_Logger.cpp.elf $BUILDPATH/sensor.c.o $BUILDPATH/xbee.c.o $BUILDPATH/DS1921_Logger.cpp.o $BUILDPATH/serial.cpp.o $BUILDPATH/SPI/SPI.cpp.o $BUILDPATH/Wire/Wire.cpp.o $BUILDPATH/Wire/utility/twi.c.o $BUILDPATH/SoftwareSerial/SoftwareSerial.cpp.o $BUILDPATH/XBee/XBee.cpp.o $BUILDPATH/OneWire/OneWire.cpp.o $BUILDPATH/core.a -L$BUILDPATH -lm 
avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $BUILDPATH/DS1921_Logger.cpp.elf $BUILDPATH/DS1921_Logger.cpp.eep 
avr-objcopy -O ihex -R .eeprom $BUILDPATH/DS1921_Logger.cpp.elf $BUILDPATH/DS1921_Logger.cpp.hex
