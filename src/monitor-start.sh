#!/bin/sh

rrdtool create DS1921G.rrd		\
    --step 1			\
    DS:temp:GAUGE:60:U:U \
    RRA:AVERAGE:0.5:1:2048	\
    RRA:AVERAGE:0.5:6:2048
rrdtool create DS18B20.rrd		\
    --step 1			\
    DS:temp:GAUGE:60:U:U \
    RRA:AVERAGE:0.5:1:2048	\
    RRA:AVERAGE:0.5:6:2048
