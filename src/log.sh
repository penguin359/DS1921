#!/bin/sh

rrdtool create test.rrd		\
    --start 920804400		\
    --step 60			\
    DS:temp:GAUGE:120:U:U \
    RRA:AVERAGE:0.5:1:2048	\
    RRA:AVERAGE:0.5:6:2048
