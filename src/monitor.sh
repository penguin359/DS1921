#!/bin/sh

ALARM=/Developer/Platforms/iPhoneSimulator.platform/Developer/SDKs/iPhoneSimulator3.2.sdk/System/Library/PrivateFrameworks/AOSNotification.framework/Resources/findme_alarm_1.aiff

./dumplog /dev/cu.usbserial-A8004Z0z|perl -e '$|=1;while(<>){chomp;s/\r//;print"$_\n"}'|awk 'BEGIN{C="Unknown";A=0}/Chip = ([A-Z0-9]+)/{C=$3; A=0}/ALARM/{A=1}/T=/{AS="";if(A){AS=" (Alarm!)";system("afplay \"'"$ALARM"'\"")}print C": "$2AS}'
