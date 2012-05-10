#!/bin/sh

echo "DS1921G"
		#-u 100 -r -l 10 \
rrdtool graph "DS1921G"-hour.png --start -1800 \
		-u 100 -r -l 25 \
            DEF:temp="DS1921G".rrd:temp:AVERAGE \
            LINE1:temp#00FF00:"Temperature"
            #DEF:inoctets="DS1921G".rrd:input:AVERAGE \
            #AREA:inoctets#00FF00:"In traffic" \

echo "DS18B20"
rrdtool graph "DS18B20"-hour.png --start -300 \
		-u 150 -r -l 25 \
            DEF:temp="DS18B20".rrd:temp:AVERAGE \
            LINE1:temp#00FF00:"Temperature"
            #DEF:inoctets="DS1921G".rrd:input:AVERAGE \
            #AREA:inoctets#00FF00:"In traffic" \
